#pragma once

#include "example/control_session.h"
#include "example/app_control_frame.h"

#include <cmath>
#include <cstdint>

namespace chassis
{
// Only the active source refreshes the hardware watchdog. App messages carry
// generation time so queued traffic from before a handover cannot resume motion.
class SharedCommandGate
{
public:
  explicit SharedCommandGate(RemoteControlSession & session) : session_(session) {}

  void reset(std::int64_t now_ns)
  {
    handover_ns_ = now_ns;
    last_velocity_ns_ = last_action_ns_ = 0;
    app_neutral_ = false;
    app_mode_.reset();
    previous_action_ = 0;
  }

  bool remote(std::uint64_t generation) const
  {
    return session_.stage == RemoteControlSession::Stage::running &&
           session_.owner == RemoteControlSession::Source::remote &&
           generation == session_.generation;
  }

  bool app_velocity(std::int64_t stamp_ns, std::int64_t now_ns, double x, double y, double yaw)
  {
    if (unified_input_seen_ || !app_owner() || !fresh(stamp_ns, now_ns, 300000000) ||
      stamp_ns <= last_velocity_ns_ ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {return false;}
    last_velocity_ns_ = stamp_ns;
    if (!app_neutral_) {
      app_neutral_ = std::abs(x) <= 1e-6 && std::abs(y) <= 1e-6 && std::abs(yaw) <= 1e-6;
    }
    return app_neutral_;
  }

  bool app_action(std::int64_t stamp_ns, std::int64_t now_ns, int command)
  {
    if ((unified_input_seen_ && command != 110) || command == 105 || command == 106) {return false;}
    if (session_.stage != RemoteControlSession::Stage::running ||
      !fresh(stamp_ns, now_ns, 100000000) || stamp_ns <= last_action_ns_) {return false;}
    if (command != 110 && (!app_owner() || (command != 109 && !app_neutral_))) {return false;}
    last_action_ns_ = stamp_ns;
    return command >= 101 && command <= 110;
  }

  struct AppDecision
  {
    bool accepted{false};
    bool mode_changed{false};
    bool motion_allowed{false};
    int action{0};
  };

  AppDecision app_control(const AppControlFrame & frame, std::int64_t now_ns)
  {
    if (!frame.valid() || !app_owner() || !fresh(frame.stamp_ns, now_ns, 100000000) ||
      frame.stamp_ns <= last_velocity_ns_) {return {};}
    last_velocity_ns_ = frame.stamp_ns;
    unified_input_seen_ = true;
    const bool changed = !app_mode_ || *app_mode_ != frame.mode;
    if (changed) {
      app_mode_ = frame.mode;
      app_neutral_ = false;
      previous_action_ = 0;
    }
    if (frame.neutral()) {app_neutral_ = true;}
    int action = 0;
    if (frame.action == 109 || (app_neutral_ && frame.action != previous_action_)) {
      action = frame.action;
    }
    previous_action_ = frame.action;
    if (frame.action != 0) {last_action_ns_ = frame.stamp_ns;}
    return {true, changed, app_neutral_, action};
  }

  bool app_arm_mode() const noexcept
  {
    return app_owner() && app_mode_ == AppControlMode::arm;
  }

private:
  bool app_owner() const
  {
    return session_.stage == RemoteControlSession::Stage::running &&
           session_.owner == RemoteControlSession::Source::app;
  }
  bool fresh(std::int64_t stamp, std::int64_t now, std::int64_t lifetime) const
  {
    return stamp > 0 && stamp > handover_ns_ && stamp <= now && now - stamp <= lifetime;
  }
  RemoteControlSession & session_;
  std::int64_t handover_ns_{0}, last_velocity_ns_{0}, last_action_ns_{0};
  bool app_neutral_{false};
  bool unified_input_seen_{false};
  std::optional<AppControlMode> app_mode_;
  int previous_action_{0};
};
}  // namespace chassis
