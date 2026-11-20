#pragma once

#include "example/app_interface.h"
#include "example/arm_visual_tracking.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace chassis
{
// All entry points run on the hardware owner's serial executor.
class AppArmControl
{
public:
  using Clock = std::chrono::steady_clock;

  AppArmControl(
    rclcpp::Node & node, ArmController & arm, ArmVisualTracking & tracker,
    std::function<bool(Clock::time_point)> fresh_command)
  : node_(node), arm_(arm), tracker_(tracker), fresh_command_(std::move(fresh_command))
  {
    const double rpm = node.has_parameter("yiyou_arm_speed_rpm") ?
      node.get_parameter("yiyou_arm_speed_rpm").as_double() : 0.6;
    if (!std::isfinite(rpm) || rpm <= 0.0) {
      throw std::invalid_argument("App arm requires a positive finite profile speed");
    }
    joint_lead_ = rpm * 2.0 * arm::kPi / 60.0 * 0.2;
  }

  // Only accepted arm-mode frames call this: chassis heartbeats cannot renew it.
  void velocity(double x, double z, double pitch, Clock::time_point now = Clock::now())
  {
    expire_input(now);
    std::array<double, 3> input{x, z, pitch};
    for (auto & component : input) {
      if (!std::isfinite(component) || std::abs(component) > 1.0) {stop(now); return;}
      if (std::abs(component) <= 1e-6) {component = 0.0;}
    }
    if (input == std::array<double, 3>{}) {
      if (continuous_) {stop(now);}
      neutral_required_ = false;
      // A tap has a bounded endpoint. Neutral release frames let it finish.
      if (discrete_ && fresh_command_(now)) {arm_deadline_ = now + kInputLease;}
      return;
    }
    if (neutral_required_ || !fresh_command_(now) ||
      node_.get_parameter(kArmTrackingEnabledParameter).as_bool()) {return;}

    bool changed_direction = false;
    for (std::size_t i = 0; i < input.size(); ++i) {
      changed_direction = changed_direction ||
        (input_[i] != 0.0 && (input[i] == 0.0 || input[i] * input_[i] < 0.0));
    }
    if (!continuous_ || changed_direction) {
      // Discard the old endpoint before waiting for measured standstill. This
      // fresh frame may request the next direction, but cannot release the halt.
      stop(now);
      neutral_required_ = false;
      continuous_ = true;
    }
    input_ = input;
    last_input_ = now;
    arm_deadline_ = now + kInputLease;
  }

  void command(int value, Clock::time_point now = Clock::now())
  {
    if (value == 109) {stop(now); return;}
    expire_input(now);
    if (!fresh_command_(now)) {
      RCLCPP_WARN(node_.get_logger(), "App arm command ignored: no fresh gateway control heartbeat");
      return;
    }
    if (value == 107 || value == 108) {
      // One tap grants a bounded jog. Further taps cannot renew or reverse it.
      if (!arm_.has_fourth_axis() || fourth_deadline_) {return;}
      arm_.set_fourth_axis_jog(value == 107 ? 1 : -1);
      fourth_deadline_ = now + std::chrono::milliseconds(100);
      stop_latched_ = false;
      return;
    }
    const auto target = app_arm_target(arm_.actual_pose(), value);
    if (!target || continuous_ || neutral_required_) {return;}
    // Reuse IK/limits/visual-ownership validation and the existing staged PP path.
    // Rejected/busy steps are discarded, never queued for later.
    const auto result = node_.set_parameter(rclcpp::Parameter(
      kYiyouPoseParameter, std::vector<double>{target->x, target->z, target->pitch}));
    if (result.successful) {
      stop_latched_ = false;
      discrete_ = true;
      arm_deadline_ = now + kInputLease;
    } else {
      RCLCPP_WARN(node_.get_logger(), "App arm step rejected: %s", result.reason.c_str());
    }
  }

  void stop(Clock::time_point now = Clock::now())
  {
    // Clear intent even when the hardware is already held. Otherwise a timeout
    // or owner change during settling could leave a cached command to replay.
    input_ = {};
    reference_.reset();
    arm_deadline_.reset();
    continuous_ = false;
    discrete_ = false;
    neutral_required_ = true;
    fourth_deadline_.reset();
    if (stop_latched_ && !node_.get_parameter(kArmTrackingEnabledParameter).as_bool() &&
      !arm_.position_command_pending() && arm_.fourth_axis_jog() == 0)
    {
      // Pending PP only describes the handshake. An external parameter target
      // can be unapplied or still moving after that handshake has finished.
      const auto actual = arm_.actual_pose();
      if (arm_.planar_halt_pending() ||
        node_.get_parameter(kYiyouPoseParameter).as_double_array() ==
        std::vector<double>{actual.x, actual.z, actual.pitch}) {return;}
    }
    stop_latched_ = false;
    // Consume a committed target as well as cancelling an in-flight PP group,
    // so the normal parameter path cannot resume a stopped App action.
    const auto result = node_.set_parameter(rclcpp::Parameter(kArmTrackingEnabledParameter, false));
    if (!result.successful) {throw std::runtime_error(result.reason);}
    tracker_.stop_tracking(now);
    if (arm_.has_fourth_axis()) {arm_.set_fourth_axis_jog(0);}
    stop_latched_ = true;
  }

  void before_update(Clock::time_point now = Clock::now())
  {
    // Expiry must precede PP advancement, including the trigger phase.
    expire_input(now);
    if (fourth_deadline_ && now >= *fourth_deadline_) {
      arm_.set_fourth_axis_jog(0);
      fourth_deadline_.reset();
    }
  }

  // Called after arm feedback and tracker.after_update() have synchronized a halt.
  void after_update(Clock::time_point now = Clock::now())
  {
    expire_input(now);
    if (!continuous_) {return;}
    const auto previous = integration_time_;
    integration_time_ = now;  // Busy/held cycles never accrue time to catch up later.
    if (arm_.planar_halt_pending()) {return;}
    const auto actual = arm_.actual_pose();
    if (!reference_) {
      // The synchronization barrier is authoritative; encoder jitter need not
      // exactly reproduce the previously synchronized floating-point pose.
      if (!tracker_.complete_source_handover()) {return;}
      reference_ = actual;
      settled_at_ = now;
      return;
    }
    // Require a frame received after the halt and baseline synchronization.
    if (last_input_ <= settled_at_) {return;}
    const double dt = std::clamp(std::chrono::duration<double>(now - previous).count(), 0.0, 0.05);
    auto next = *reference_;
    next.x += input_[0] * 0.005 * dt;
    next.z += input_[1] * 0.005 * dt;
    next.pitch += input_[2] * arm::kPi / 180.0 * dt;
    const auto within_lead = [&](const arm::Pose & pose) {
        return std::hypot(pose.x - actual.x, pose.z - actual.z) <= 0.001 + 1e-12 &&
               std::abs(std::remainder(pose.pitch - actual.pitch, 2.0 * arm::kPi)) <=
               0.2 * arm::kPi / 180.0 + 1e-12;
      };
    if (!within_lead(*reference_)) {
      stop(now);
      return;
    }
    // Keep only one reference bounded against feedback, even during a handshake.
    // Reject the whole increment at its bound; never clip individual joints.
    if (within_lead(next)) {reference_ = next;}
    if (arm_.position_command_pending() ||
      node_.get_parameter(kYiyouPoseParameter).as_double_array() == values(*reference_)) {return;}
    try {
      const auto target_joints = arm_.validate_target_pose(*reference_);
      const auto measured_joints = arm_.actual_joints();
      for (std::size_t i = 0; i < target_joints.size(); ++i) {
        if (std::abs(target_joints[i] - measured_joints[i]) > joint_lead_ + 1e-12) {
          throw std::invalid_argument("App arm target exceeds profile joint lead");
        }
      }
      const auto result = node_.set_parameter(rclcpp::Parameter(
        kYiyouPoseParameter, values(*reference_)));
      if (!result.successful) {throw std::invalid_argument(result.reason);}
      stop_latched_ = false;
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN(node_.get_logger(), "App arm velocity rejected: %s", error.what());
      stop(now);
    }
  }

private:
  static constexpr auto kInputLease = std::chrono::milliseconds(250);

  static std::vector<double> values(const arm::Pose & pose)
  {
    return {pose.x, pose.z, pose.pitch};
  }

  void expire_input(Clock::time_point now)
  {
    if (arm_deadline_ && now >= *arm_deadline_) {stop(now);}
  }

  rclcpp::Node & node_;
  ArmController & arm_;
  ArmVisualTracking & tracker_;
  std::function<bool(Clock::time_point)> fresh_command_;
  std::optional<Clock::time_point> fourth_deadline_;
  std::optional<Clock::time_point> arm_deadline_;
  std::array<double, 3> input_{};
  std::optional<arm::Pose> reference_;
  Clock::time_point last_input_{};
  Clock::time_point integration_time_{};
  Clock::time_point settled_at_{};
  double joint_lead_{};
  bool continuous_{false};
  bool discrete_{false};
  bool neutral_required_{false};
  bool stop_latched_{false};
};
}  // namespace chassis
