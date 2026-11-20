#pragma once

#include "example/arm_parameters.h"
#include "example/imu_heading_guard.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace chassis
{
inline constexpr auto kArmTrackingEnabledParameter = "arm_tracking_enabled";
inline constexpr auto kArmTrackingXEnabledParameter = "arm_tracking_x_enabled";
inline constexpr auto kArmTrackingGoalEnabledParameter = "arm_tracking_goal_enabled";

// Declare before opening hardware. Starting a process must never start visual motion.
inline void declare_arm_tracking_parameter(rclcpp::Node & node)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description =
    "Runtime-only visual arm and chassis lateral tracking; must be false at startup";
  if (node.declare_parameter<bool>(kArmTrackingEnabledParameter, false, descriptor)) {
    throw std::invalid_argument("arm_tracking_enabled must be false at startup");
  }
  descriptor.description =
    "Runtime-only depth-based arm X tracking; must be false at startup";
  if (node.declare_parameter<bool>(kArmTrackingXEnabledParameter, false, descriptor)) {
    throw std::invalid_argument("arm_tracking_x_enabled must be false at startup");
  }
  descriptor.description =
    "Runtime-only fixed arm goal with command heartbeat; must be false at startup";
  if (node.declare_parameter<bool>(kArmTrackingGoalEnabledParameter, false, descriptor)) {
    throw std::invalid_argument("arm_tracking_goal_enabled must be false at startup");
  }
}

// All callbacks and update calls belong to the hardware owner's serial executor.
class ArmVisualTracking
{
public:
  using Clock = std::chrono::steady_clock;

  ArmVisualTracking(
    rclcpp::Node & node, ArmController & arm,
    std::function<void(const std::exception &)> on_fault = {},
    std::function<bool()> external_allowed = {},
    std::function<std::uint64_t()> current_generation = {})
  : node_(node), arm_(arm), on_fault_(std::move(on_fault)),
    external_allowed_(std::move(external_allowed)), current_generation_(std::move(current_generation))
  {
    if (external_allowed_ && !current_generation_) {
      throw std::invalid_argument("shared arm tracking requires a control generation callback");
    }
    imu_subscription_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      "/hardware/imu_data", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        guarded([&] {observe_imu(*message);});
      });
    parameter_callback_ = node_.add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return validate_parameters(parameters);
      });
    const auto qos = rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.25));
    if (external_allowed_) {
      stroke_state_publisher_ = node_.create_publisher<std_msgs::msg::String>(
        "/car2/arm_tracking_stroke_state", qos);
      correction_subscription_ = node_.create_subscription<std_msgs::msg::String>(
        "/car2/arm_tracking_correction", qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          guarded([&] {observe_correction(message->data);});
        });
      shared_target_subscription_ = node_.create_subscription<std_msgs::msg::String>(
        "/car2/arm_tracking_target", qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          guarded([&] {
              if (external_allowed_()) {observe_shared_command(message->data, false);}
            });
        });
      shared_goal_subscription_ = node_.create_subscription<std_msgs::msg::String>(
        "/car2/arm_tracking_goal_target", qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          guarded([&] {
              if (external_allowed_()) {observe_shared_command(message->data, true);}
            });
        });
    } else {
      target_subscription_ = node_.create_subscription<geometry_msgs::msg::PointStamped>(
        "/arm_tracking_target", qos,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
          guarded([&] {observe_target(*message);});
        });
      goal_subscription_ = node_.create_subscription<geometry_msgs::msg::PointStamped>(
        "/arm_tracking_goal_target", qos,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
          guarded([&] {observe_goal(*message);});
        });
      // Shared cleanup uses generation-checked parameters; legacy Empty has no owner.
      stop_subscription_ = node_.create_subscription<std_msgs::msg::Empty>(
        "/arm_tracking_stop", rclcpp::QoS(1),
        [this](std_msgs::msg::Empty::ConstSharedPtr) {guarded([&] {stop_tracking();});});
    }
  }

  void observe_imu(
    const sensor_msgs::msg::Imu & message, Clock::time_point now = Clock::now())
  {
    imu_heading_.observe(message, node_.now().nanoseconds(), now);
  }

  // Run before ArmController::update: expired input cannot advance a staged target.
  void before_update(Clock::time_point now = Clock::now())
  {
    const bool requested = node_.get_parameter(kArmTrackingEnabledParameter).as_bool();
    if (requested != enabled_) {
      enabled_ = requested;
      if (enabled_) {
        tracking_latched_ = false;
        x_enabled_ = node_.get_parameter(kArmTrackingXEnabledParameter).as_bool();
        goal_enabled_ = node_.get_parameter(kArmTrackingGoalEnabledParameter).as_bool();
      }
      goal_.reset();
      chassis_neutral_pending_ = true;
      baseline_.reset();
      manual_sync_pending_ = !enabled_;
      // Consume any old manual request before changing ownership.
      last_manual_request_ = node_.get_parameter(kYiyouPoseParameter).as_double_array();
      imu_heading_.end_session();
      halt_and_clear(enabled_ ? "enabled; waiting for standstill" : "disabled", true);
      stroke_.reset();
      if (enabled_ && goal_enabled_) {
        if (!imu_heading_.start_session(now)) {
          cancel_for_imu();
        } else {
          RCLCPP_INFO(node_.get_logger(),
            "ARM_TRACKING IMU reference_yaw_deg=%.3f max_error_deg=8 timeout_ms=200 "
            "policy=cancel_on_fault",
            imu_heading_.reference_yaw() * 180.0 / arm::kPi);
        }
      }
      if (enabled_) {
        RCLCPP_INFO(node_.get_logger(),
          "ARM_TRACKING target_policy=direct fixed_goal_profile=proportional "
          "motor_profile_controls_speed "
          "lateral_speed_mm_s=%.1f lateral_motion_time_limit=none x_tracking=%s fixed_goal=%s",
          kMaxLateralSpeed * 1000.0, x_enabled_ ? "enabled" : "disabled",
          goal_enabled_ ? "enabled" : "disabled");
      }
    }
    if (enabled_ && goal_enabled_ && !tracking_latched_ && !imu_heading_.check(now)) {
      cancel_for_imu();
    }
    if (stroke_ && !tracking_latched_) {
      if (!external_allowed_() || current_generation_() != stroke_->generation) {
        halt_and_clear("stroke lost control ownership", true);
      } else if (stroke_->stage == StrokeStage::settled && now >= stroke_->verify_deadline) {
        halt_and_clear("stroke verification timed out", true);
      }
    }
    if (target_ && (now >= deadline_ || !stamp_fresh(target_->stamp_ns))) {
      halt_and_clear(goal_ ? "fixed goal command expired; disable and re-enable to recover" :
        "target expired");
    }
  }

  // Run after fresh motor feedback. This also owns the legacy manual parameter path.
  void after_update(Clock::time_point now = Clock::now())
  {
    if (arm_.planar_halt_pending()) {return;}
    if (!enabled_) {
      if (manual_sync_pending_) {synchronize_manual_pose();}
      try {
        apply_arm_pose_parameter(node_, arm_, last_manual_request_);
      } catch (const std::invalid_argument & error) {
        RCLCPP_WARN(node_.get_logger(), "Arm target rejected before motion: %s", error.what());
      }
      return;
    }
    if (tracking_latched_) {return;}
    const auto actual = arm_.actual_pose();
    if (stroke_) {
      // ArmController clears planar_halt_pending only after fresh standstill feedback.
      if (stroke_->stage == StrokeStage::first_halt) {
        stroke_->settled_pose = actual;
        stroke_->settled_ns = node_.now().nanoseconds();
        stroke_->verify_deadline = now + std::chrono::milliseconds(2500);
        stroke_->stage = StrokeStage::settled;
        publish_stroke_state();
      } else if (stroke_->stage == StrokeStage::final_halt) {
        if (!at_pose(actual, stroke_->corrected_pose)) {
          halt_and_clear("stroke correction settled outside the target tolerance", true);
          return;
        }
        stroke_->stage = StrokeStage::complete;
        tracking_latched_ = true;
        target_.reset();
        publish_stroke_state();
        return;
      } else if (stroke_->stage == StrokeStage::correcting && !submitted_target_ &&
        !at_pose(actual, stroke_->settled_pose))
      {
        halt_and_clear("arm moved after the stroke verification reference", true);
        return;
      }
      if (stroke_->stage == StrokeStage::settled) {return;}
    }
    if (!baseline_) {baseline_ = actual;}
    if (!target_ || (goal_enabled_ && !goal_)) {
      // Live vision may align the chassis before goal submission, but cannot move the arm.
      return;
    }
    const arm::Pose next = stroke_ && stroke_->stage == StrokeStage::correcting ?
      stroke_->corrected_pose :
      arm::Pose{x_enabled_ ? target_->x : baseline_->x, target_->z, baseline_->pitch};
    if (std::abs(next.z - baseline_->z) > kHeightRange) {
      halt_and_clear("height outside activation range");
      return;
    }
    // Completion still requires measured pitch: proportional PP is not exact synchronization.
    if (at_pose(actual, next)) {
      if (stroke_) {
        stroke_->stage = stroke_->stage == StrokeStage::correcting ?
          StrokeStage::final_halt : StrokeStage::first_halt;
        submitted_target_.reset();
        arm_.halt_planar_position();
      } else if (goal_) {
        halt_and_clear("fixed goal completed; disable and re-enable for a new goal", true);
      } else if (submitted_target_) {
        halt_and_clear(x_enabled_ ? "planar target in deadband" : "height in deadband");
      }
      return;
    }
    if (arm_.position_command_pending()) {return;}
    // Fresh command stamps renew the heartbeat without retriggering an unchanged endpoint.
    if (submitted_target_ && next.x == submitted_target_->x &&
      next.z == submitted_target_->z && next.pitch == submitted_target_->pitch)
    {
      return;
    }
    try {
      if (goal_enabled_) {
        if (!arm_.set_synchronized_target_pose(next)) {return;}
      } else {arm_.set_target_pose(next);}
      submitted_target_ = next;
      direction_x_ = (next.x > actual.x) - (next.x < actual.x);
      direction_z_ = (next.z > actual.z) - (next.z < actual.z);
    } catch (const std::invalid_argument & error) {
      // Unreachable vision requests are input errors, not hardware faults.
      halt_and_clear(error.what());
    }
  }

  void observe_target(
    const geometry_msgs::msg::PointStamped & message, Clock::time_point now = Clock::now())
  {
    before_update(now);
    if (!enabled_ || tracking_latched_) {return;}
    // Once committed, only goal command heartbeats can renew the fixed destination.
    // Queued live captures must neither change that destination nor restart lateral motion.
    if (goal_) {return;}
    const auto & stamp = message.header.stamp;
    const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const auto now_ns = node_.now().nanoseconds();
    const char * rejection = nullptr;
    if (message.header.frame_id != "arm_shoulder") {
      rejection = "unexpected frame; expected arm_shoulder";
    } else if (!std::isfinite(message.point.x) || !std::isfinite(message.point.y) ||
      !std::isfinite(message.point.z))
    {
      rejection = "non-finite target coordinates";
    } else if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || stamp_ns <= 0) {
      rejection = "missing or malformed capture timestamp";
    } else if (stamp_ns > now_ns) {
      rejection = "capture timestamp is in the future";
    } else if (now_ns - stamp_ns > 250000000LL) {
      rejection = "capture age exceeds 250 ms";
    } else if (stamp_ns <= last_stamp_ns_) {
      rejection = "duplicate or out-of-order capture";
    } else if (stamp_ns <= minimum_stamp_ns_) {
      rejection = "capture is not newer than the last hold/enable event";
    }
    if (rejection) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "ARM_TRACKING target rejected: %s; frame=%s age_ms=%.1f "
        "after_hold_ms=%.1f after_last_ms=%.1f",
        rejection, message.header.frame_id.c_str(), (now_ns - stamp_ns) / 1.0e6,
        (stamp_ns - minimum_stamp_ns_) / 1.0e6, (stamp_ns - last_stamp_ns_) / 1.0e6);
      halt_and_clear(rejection);
      return;
    }
    last_stamp_ns_ = stamp_ns;
    if (!goal_enabled_ && baseline_ && std::abs(message.point.z - baseline_->z) > kHeightRange) {
      halt_and_clear("height outside activation range");
      return;
    }
    if (submitted_target_) {
      const auto actual = arm_.actual_pose();
      if ((message.point.z - actual.z) * direction_z_ < -kDeadband ||
        (x_enabled_ && (message.point.x - actual.x) * direction_x_ < -kDeadband))
      {
        // Cancel either reversed axis before re-basing from stopped feedback. A subsequent
        // capture (strictly after this hold) may resume once the drives settle.
        halt_and_clear("target reversed; waiting for standstill and a new capture");
        return;
      }
    }
    target_ = Target{message.point.x, message.point.y, message.point.z, stamp_ns};
    deadline_ = now + std::chrono::milliseconds(300);
  }

  // Goal stamps describe current command generation, not the original camera capture.
  // The first accepted point is immutable until an explicit disable/enable transition.
  void observe_goal(
    const geometry_msgs::msg::PointStamped & message, Clock::time_point now = Clock::now(),
    std::uint64_t stroke_id = 0)
  {
    before_update(now);
    if (!enabled_ || !goal_enabled_) {return;}
    if (goal_ && stroke_id != (stroke_ ? stroke_->id : 0)) {return;}
    if (!goal_ && stroke_id != 0) {
      if (!external_allowed_ || !external_allowed_()) {return;}
      const auto generation = current_generation_();
      if (stroke_history_generation_ != generation) {
        used_strokes_.clear();
        stroke_history_generation_ = generation;
      }
      if (used_strokes_.count(stroke_id)) {return;}
    }
    if (tracking_latched_) {
      if (stroke_) {publish_stroke_state();}
      return;
    }
    if (!baseline_ || (arm_.planar_halt_pending() && !stroke_)) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "ARM_TRACKING goal deferred: waiting for settled activation feedback");
      return;
    }
    const auto reject = [&](const char * reason) {
        RCLCPP_WARN(node_.get_logger(), "ARM_TRACKING goal rejected: %s", reason);
        cancel_tracking(reason, now);
      };
    const auto & stamp = message.header.stamp;
    const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const auto now_ns = node_.now().nanoseconds();
    const char * rejection = nullptr;
    if (message.header.frame_id != "arm_shoulder") {
      rejection = "unexpected goal frame; expected arm_shoulder";
    } else if (!std::isfinite(message.point.x) || !std::isfinite(message.point.y) ||
      !std::isfinite(message.point.z))
    {
      rejection = "non-finite goal coordinates";
    } else if (message.point.y != 0.0) {
      rejection = "fixed goal requires zero lateral command";
    } else if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || stamp_ns <= 0) {
      rejection = "missing or malformed goal command timestamp";
    } else if (stamp_ns > now_ns || now_ns - stamp_ns > 250000000LL) {
      rejection = "goal command timestamp is future or older than 250 ms";
    } else if (stamp_ns <= last_stamp_ns_ || stamp_ns <= minimum_stamp_ns_) {
      rejection = "goal command is not newer than the last input or hold event";
    } else if (std::abs(message.point.z - baseline_->z) > kHeightRange) {
      rejection = "goal height outside activation range";
    } else if (goal_ && (message.point.x != goal_->x || message.point.z != goal_->z)) {
      rejection = "fixed goal changed; disable and re-enable for a new goal";
    }
    if (rejection) {
      reject(rejection);
      return;
    }
    if (!goal_) {
      const arm::Pose endpoint{message.point.x, message.point.z, baseline_->pitch};
      try {
        (void)arm_.validate_target_pose(endpoint);
      } catch (const std::invalid_argument & error) {
        reject(error.what());
        return;
      }
      goal_ = endpoint;
      if (stroke_id != 0) {
        used_strokes_.insert(stroke_id);
        stroke_.emplace();
        stroke_->id = stroke_id;
        stroke_->generation = current_generation_();
      }
      chassis_neutral_pending_ = true;
      RCLCPP_INFO(node_.get_logger(), "ARM_TRACKING goal accepted: x=%.6f z=%.6f pitch=%.6f",
        endpoint.x, endpoint.z, endpoint.pitch);
    }
    last_stamp_ns_ = stamp_ns;
    target_ = Target{goal_->x, 0.0, goal_->z, stamp_ns};
    deadline_ = now + std::chrono::milliseconds(300);
    if (stroke_) {publish_stroke_state();}
  }

  // Chassis ownership includes the transition and the neutral command required on exit.
  bool owns_chassis() const
  {
    return enabled_ || manual_sync_pending_ || chassis_neutral_pending_ ||
           node_.get_parameter(kArmTrackingEnabledParameter).as_bool();
  }

  // The shared owner already required neutral before accepting the new input.
  // Only clear the visual handover interlock after measured halt and baseline sync;
  // this does not enable tracking or restore any previous target.
  bool complete_source_handover()
  {
    if (enabled_ || manual_sync_pending_ || arm_.planar_halt_pending() ||
      node_.get_parameter(kArmTrackingEnabledParameter).as_bool()) {return false;}
    chassis_neutral_pending_ = false;
    return true;
  }

  bool accept_manual_chassis_command(
    double vx, double vy, double yaw, Clock::time_point now = Clock::now())
  {
    before_update(now);
    const bool neutral = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(yaw) &&
      std::abs(vx) <= 1e-6 && std::abs(vy) <= 1e-6 && std::abs(yaw) <= 1e-6;
    if (enabled_ || manual_sync_pending_) {
      // Neutral joystick refreshes must not overwrite a visual lateral command.
      // A deliberate manual command stops both visual axes before handover.
      if (!neutral) {cancel_tracking("manual chassis input interrupted tracking", now);}
      return false;
    }
    if (chassis_neutral_pending_) {
      if (!neutral) {return false;}
      chassis_neutral_pending_ = false;
    }
    return true;
  }

  void cancel_tracking(const char * reason, Clock::time_point now = Clock::now())
  {
    before_update(now);
    chassis_neutral_pending_ = true;
    if (tracking_latched_) {return;}
    tracking_latched_ = true;
    halt_and_clear(reason, true);
  }

  // Only the hardware owner applies this result, with vx=yaw=0.
  double lateral_velocity(Clock::time_point now = Clock::now())
  {
    before_update(now);
    double velocity = 0.0;
    if (enabled_ && !tracking_latched_ && !goal_ && baseline_ && target_ &&
      !arm_.planar_halt_pending() && std::abs(target_->y) > kDeadband)
    {
      velocity = kLateralGain * std::clamp(
        target_->y, -kMaxLateralSpeed / kLateralGain, kMaxLateralSpeed / kLateralGain);
    }
    return velocity;
  }

  // Explicit stop cancels planar motion even if manual ownership is being restored.
  // Live tracking may resume from a new capture; a committed fixed goal is cancelled for the session.
  void stop_tracking(Clock::time_point now = Clock::now())
  {
    before_update(now);
    chassis_neutral_pending_ = true;
    if (!enabled_) {manual_sync_pending_ = true;}
    halt_and_clear("stop requested", true);
  }

private:
  enum class StrokeStage { nominal, first_halt, settled, correcting, final_halt, complete, cancelled };

  struct Stroke
  {
    std::uint64_t generation{}, id{};
    StrokeStage stage{StrokeStage::nominal};
    std::int64_t settled_ns{};
    Clock::time_point verify_deadline{};
    arm::Pose settled_pose{}, corrected_pose{};
  };

  void publish_stroke_state()
  {
    const char * stage;
    switch (stroke_->stage) {
      case StrokeStage::settled: stage = "settled"; break;
      case StrokeStage::correcting:
      case StrokeStage::final_halt: stage = "correcting"; break;
      case StrokeStage::complete: stage = "complete"; break;
      case StrokeStage::cancelled: stage = "cancelled"; break;
      default: return;  // No settled acquisition barrier exists during the first move.
    }
    std_msgs::msg::String message;
    message.data = "{\"generation\":" + std::to_string(stroke_->generation) +
      ",\"stroke_id\":" + std::to_string(stroke_->id) + ",\"stage\":\"" + stage +
      "\",\"settled_ns\":" + std::to_string(stroke_->settled_ns) + "}";
    stroke_state_publisher_->publish(message);
  }

  void cancel_stroke()
  {
    if (!stroke_ || stroke_->stage == StrokeStage::complete ||
      stroke_->stage == StrokeStage::cancelled) {return;}
    stroke_->stage = StrokeStage::cancelled;
    publish_stroke_state();
  }

  static bool unsigned_integer(const YAML::Node & value)
  {
    if (!value.IsScalar() || value.Tag() == "!") {return false;}
    const auto scalar = value.Scalar();
    return !scalar.empty() && std::all_of(scalar.begin(), scalar.end(),
      [](char digit) {return digit >= '0' && digit <= '9';});
  }

  void observe_correction(const std::string & payload)
  {
    before_update();
    if (!stroke_ || stroke_->stage != StrokeStage::settled || tracking_latched_ ||
      !external_allowed_() || payload.size() > 512) {return;}
    double dx, dz;
    try {
      const auto data = YAML::Load(payload);
      if (!data.IsMap() || data.size() != 6 || !unsigned_integer(data["generation"]) ||
        data["generation"].as<std::uint64_t>() != stroke_->generation ||
        !unsigned_integer(data["stroke_id"]) || data["stroke_id"].as<std::uint64_t>() != stroke_->id ||
        !unsigned_integer(data["settled_ns"]) || data["settled_ns"].as<std::int64_t>() != stroke_->settled_ns ||
        !unsigned_integer(data["stamp_ns"])) {return;}
      const auto stamp = data["stamp_ns"].as<std::int64_t>();
      if (stamp <= stroke_->settled_ns || !stamp_fresh(stamp)) {return;}
      if (!data["dx"].IsScalar() || !data["dz"].IsScalar() ||
        data["dx"].Tag() == "!" || data["dz"].Tag() == "!") {return;}
      dx = data["dx"].as<double>();
      dz = data["dz"].as<double>();
    } catch (const YAML::Exception &) {return;}
    if (!std::isfinite(dx) || !std::isfinite(dz) || std::hypot(dx, dz) > 0.025) {
      halt_and_clear("stroke correction exceeds 25 mm or is non-finite", true);
      return;
    }
    if (dx == 0.0 && dz == 0.0) {
      // Keep the first physical halt. A no-adjustment request must never
      // drive back to the old reference if the arm moved during verification.
      stroke_->corrected_pose = stroke_->settled_pose;
      stroke_->stage = StrokeStage::final_halt;
      return;
    }
    const arm::Pose corrected{stroke_->settled_pose.x + dx, stroke_->settled_pose.z + dz,
      baseline_->pitch};
    if (std::abs(corrected.z - baseline_->z) > kHeightRange) {
      halt_and_clear("stroke correction outside activation height range", true);
      return;
    }
    try {
      (void)arm_.validate_target_pose(corrected);
    } catch (const std::invalid_argument & error) {
      halt_and_clear(error.what(), true);
      return;
    }
    stroke_->corrected_pose = corrected;
    stroke_->stage = StrokeStage::correcting;
    publish_stroke_state();
    // Only the regular feedback/control loop may submit the PP command. The
    // correction never refreshes the original fixed-goal heartbeat deadline.
  }

  void cancel_for_imu()
  {
    tracking_latched_ = true;
    chassis_neutral_pending_ = true;
    const std::string reason = "docking canceled: " + imu_heading_.error() +
      "; disable and re-enable to recover";
    halt_and_clear(reason.c_str(), true);
  }

  void observe_shared_command(const std::string & payload, bool fixed_goal)
  {
    if (payload.size() > 512) {return;}
    YAML::Node data;
    try {
      data = YAML::Load(payload);
      if (!data.IsMap() || !unsigned_integer(data["generation"]) ||
        data["generation"].as<std::uint64_t>() != current_generation_()) {return;}
    } catch (const YAML::Exception &) {return;}
    geometry_msgs::msg::PointStamped point;
    bool malformed = false;
    std::uint64_t stroke_id = 0;
    try {
      constexpr auto max_stamp = static_cast<std::int64_t>(
        std::numeric_limits<std::int32_t>::max()) * 1000000000LL + 999999999LL;
      const bool has_stroke = fixed_goal && data["stroke_id"].IsDefined();
      if (has_stroke && unsigned_integer(data["stroke_id"])) {
        stroke_id = data["stroke_id"].as<std::uint64_t>();
      }
      if (data.size() != (has_stroke ? 7U : 6U) || (has_stroke && stroke_id == 0) ||
        !unsigned_integer(data["stamp_ns"])) {
        malformed = true;
      } else {
        const auto stamp = data["stamp_ns"].as<std::int64_t>();
        if (stamp <= 0 || stamp > max_stamp) {
          malformed = true;
        } else {
          point.header.stamp.sec = static_cast<std::int32_t>(stamp / 1000000000LL);
          point.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % 1000000000LL);
          point.header.frame_id = data["frame_id"].as<std::string>();
          point.point.x = data["x"].as<double>();
          point.point.y = data["y"].as<double>();
          point.point.z = data["z"].as<double>();
        }
      }
    } catch (const YAML::Exception &) {malformed = true;}
    if (malformed) {
      before_update();
      if (!enabled_ || tracking_latched_ || (fixed_goal ? !goal_enabled_ : bool(goal_))) {return;}
      if (fixed_goal) {cancel_tracking("malformed shared goal command");}
      else {halt_and_clear("malformed shared target command");}
      return;
    }
    if (shared_message_predates_session(point)) {return;}
    if (fixed_goal) {observe_goal(point, Clock::now(), stroke_id);}
    else {observe_target(point);}
  }

  bool shared_message_predates_session(const geometry_msgs::msg::PointStamped & message)
  {
    if (!external_allowed_) {return false;}
    // Apply a committed enable/disable before comparing its time barrier. Old
    // traffic is ignored here rather than reaching rejection code that halts motion.
    before_update();
    const auto & stamp = message.header.stamp;
    const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    return stamp_ns <= minimum_stamp_ns_ || stamp_ns <= last_stamp_ns_;
  }

  struct Target
  {
    double x;
    double y;
    double z;
    std::int64_t stamp_ns;
  };
  static constexpr double kHeightRange = 0.15;
  static constexpr double kDeadband = 0.002;
  static constexpr double kPitchTolerance = 0.01;
  static constexpr double kMaxLateralSpeed = 0.020;
  static constexpr double kLateralGain = 2.0;

  static bool at_pose(const arm::Pose & actual, const arm::Pose & target)
  {
    return std::hypot(target.x - actual.x, target.z - actual.z) <= kDeadband &&
           std::abs(std::remainder(target.pitch - actual.pitch, 2.0 * arm::kPi)) <= kPitchTolerance;
  }

  bool stamp_fresh(std::int64_t stamp_ns) const
  {
    const auto now_ns = node_.now().nanoseconds();
    return stamp_ns > 0 && stamp_ns <= now_ns && now_ns - stamp_ns <= 250000000LL;
  }

  void halt_and_clear(const char * reason, bool explicit_event = false)
  {
    // A committed goal must never restart after stop, expiry, completion or a planning failure.
    // Keep the frozen endpoint until a mode transition so new command stamps cannot revive it.
    if (goal_) {tracking_latched_ = true;}
    // Once held, queued/rejected captures must not keep moving the resume barrier.
    // With normal camera latency that would prevent every subsequent frame from recovering.
    // Deliberate mode/stop events still require a capture strictly after that event.
    if (!explicit_event && !target_ && !submitted_target_) {return;}
    target_.reset();
    submitted_target_.reset();
    minimum_stamp_ns_ = node_.now().nanoseconds();
    arm_.halt_planar_position();
    cancel_stroke();
    RCLCPP_INFO(node_.get_logger(), "ARM_TRACKING hold: %s", reason);
  }

  rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool has_mode = false;
    bool has_x_mode = false;
    bool has_goal_mode = false;
    bool has_pose = false;
    const bool requested = node_.get_parameter(kArmTrackingEnabledParameter).as_bool();
    bool next_requested = requested;
    bool next_x_enabled = node_.get_parameter(kArmTrackingXEnabledParameter).as_bool();
    bool next_goal_enabled = node_.get_parameter(kArmTrackingGoalEnabledParameter).as_bool();
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == kArmTrackingEnabledParameter) {
        has_mode = true;
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "arm_tracking_enabled requires a boolean";
          return result;
        }
        next_requested = parameter.as_bool();
      }
      if (parameter.get_name() == kArmTrackingXEnabledParameter) {
        has_x_mode = true;
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "arm_tracking_x_enabled requires a boolean";
          return result;
        }
        next_x_enabled = parameter.as_bool();
      }
      if (parameter.get_name() == kArmTrackingGoalEnabledParameter) {
        has_goal_mode = true;
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "arm_tracking_goal_enabled requires a boolean";
          return result;
        }
        next_goal_enabled = parameter.as_bool();
      }
      if (parameter.get_name() == kYiyouPoseParameter) {has_pose = true;}
    }
    if (next_requested && next_goal_enabled && !next_x_enabled) {
      result.successful = false;
      result.reason = "fixed goal tracking requires arm_tracking_x_enabled=true";
      return result;
    }
    if (has_x_mode &&
      next_x_enabled != node_.get_parameter(kArmTrackingXEnabledParameter).as_bool() &&
      (enabled_ || requested) && !(has_mode && !next_requested))
    {
      result.successful = false;
      result.reason = "arm_tracking_x_enabled can change only while tracking is disabled "
        "or in the same atomic request that disables tracking";
      return result;
    }
    if (has_goal_mode &&
      next_goal_enabled != node_.get_parameter(kArmTrackingGoalEnabledParameter).as_bool() &&
      (enabled_ || requested) && !(has_mode && !next_requested))
    {
      result.successful = false;
      result.reason = "arm_tracking_goal_enabled can change only while tracking is disabled "
        "or in the same atomic request that disables tracking";
      return result;
    }
    if (!has_pose || synchronizing_manual_) {return result;}
    if (has_mode || has_x_mode || has_goal_mode || enabled_ || manual_sync_pending_ ||
      arm_.planar_halt_pending() ||
      node_.get_parameter(kArmTrackingEnabledParameter).as_bool())
    {
      result.successful = false;
      result.reason = "manual arm target unavailable during visual tracking or ownership change";
      return result;
    }
    return validate_arm_pose_parameters(arm_, parameters);
  }

  void synchronize_manual_pose()
  {
    const auto actual = arm_.actual_pose();
    const std::vector<double> values{actual.x, actual.z, actual.pitch};
    synchronizing_manual_ = true;
    rcl_interfaces::msg::SetParametersResult result;
    try {
      result = node_.set_parameter(rclcpp::Parameter(kYiyouPoseParameter, values));
    } catch (...) {
      synchronizing_manual_ = false;
      throw;
    }
    synchronizing_manual_ = false;
    if (!result.successful) {
      throw std::runtime_error("cannot restore manual arm baseline: " + result.reason);
    }
    last_manual_request_ = values;
    manual_sync_pending_ = false;
  }

  void guarded(const std::function<void()> & callback)
  {
    try {
      callback();
    } catch (const std::exception & error) {
      if (!on_fault_) {throw;}
      on_fault_(error);
    }
  }

  rclcpp::Node & node_;
  ArmController & arm_;
  std::function<void(const std::exception &)> on_fault_;
  std::function<bool()> external_allowed_;
  std::function<std::uint64_t()> current_generation_;
  ImuHeadingGuard imu_heading_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shared_target_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shared_goal_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stroke_state_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr correction_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  bool enabled_{false};
  bool x_enabled_{false};
  bool goal_enabled_{false};
  bool manual_sync_pending_{false};
  bool synchronizing_manual_{false};
  bool tracking_latched_{false};
  bool chassis_neutral_pending_{false};
  std::optional<arm::Pose> baseline_;
  std::optional<arm::Pose> goal_;
  std::optional<Stroke> stroke_;
  std::uint64_t stroke_history_generation_{0};
  std::set<std::uint64_t> used_strokes_;
  std::optional<arm::Pose> submitted_target_;
  std::optional<Target> target_;
  std::int64_t last_stamp_ns_{0};
  std::int64_t minimum_stamp_ns_{0};
  Clock::time_point deadline_{};
  int direction_x_{0};
  int direction_z_{0};
  std::vector<double> last_manual_request_;
};
}  // namespace chassis
