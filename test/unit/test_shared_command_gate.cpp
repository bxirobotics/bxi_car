#include "example/shared_command_gate.h"
#include "example/hardware_owner.h"

#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace
{
void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}
void ownership_and_stale_commands()
{
  using Session = chassis::RemoteControlSession;
  Session session;
  chassis::SharedCommandGate gate(session);
  constexpr std::int64_t t = 1000000000;
  gate.reset(t);
  require(!gate.app_action(t + 1, t + 1, 110), "waiting cannot accept takeover");
  session.request_start(Session::Source::app);
  session.mark_running();
  require(!gate.app_velocity(t, t + 1, 0, 0, 0), "pre-handover neutral is stale");
  require(!gate.app_velocity(t + 1, t + 1, 1, 0, 0), "App must neutral before motion");
  require(!gate.app_action(t + 2, t + 2, 101), "arm motion also needs neutral");
  require(gate.app_velocity(t + 3, t + 3, 0, 0, 0), "fresh neutral arms App");
  require(gate.app_velocity(t + 4, t + 4, 1, 0, 0), "new App motion accepted");
  require(!gate.app_velocity(t + 4, t + 5, 1, 0, 0), "duplicate rejected");
  require(!gate.app_velocity(t + 7, t + 6, 0, 0, 0), "future command rejected");
  require(!gate.app_velocity(t + 8, t + 300000009, 0, 0, 0), "expired velocity rejected");
  require(!gate.app_velocity(t + 9, t + 9, std::numeric_limits<double>::quiet_NaN(), 0, 0),
    "nonfinite velocity rejected");
  require(gate.app_action(t + 10, t + 10, 101), "fresh arm action accepted");
  require(!gate.app_action(t + 11, t + 100000012, 101), "expired arm action rejected");
  session.claim(Session::Source::remote);
  gate.reset(t + 20);
  const auto remote_generation = session.generation;
  require(gate.remote(remote_generation), "current remote generation accepted");
  require(!gate.remote(remote_generation - 1), "old remote generation rejected");
  require(!gate.app_velocity(t + 21, t + 21, 0, 0, 0), "unowned App zero cannot interrupt remote");
  require(!gate.app_action(t + 21, t + 21, 109), "unowned App arm stop is not global stop");
  require(!gate.app_action(t + 19, t + 21, 110), "queued takeover cannot reclaim");
  require(gate.app_action(t + 22, t + 22, 110), "fresh explicit App takeover accepted");
  session.claim(Session::Source::app);
  gate.reset(t + 23);
  require(!gate.remote(remote_generation), "handover invalidates prior remote callbacks");
  require(!gate.app_velocity(t + 24, t + 24, 1, 0, 0), "App must neutral again after reclaim");
  require(gate.app_action(t + 25, t + 25, 109), "owned arm stop does not require neutral");
  session.release(Session::Source::app);
  gate.reset(t + 26);
  require(!gate.app_velocity(t + 27, t + 27, 0, 0, 0), "timeout never auto restores App");
  session.request_stop();
  require(!gate.app_action(t + 28, t + 28, 110), "stop blocks takeover");
}
void exclusive_hardware_owner()
{
  const auto path = std::filesystem::temp_directory_path() /
    ("car2-hardware-owner-test-" + std::to_string(::getpid())) / "hardware.lock";
  {
    chassis::HardwareOwner owner(path.string());
    bool rejected = false;
    try {chassis::HardwareOwner duplicate(path.string());}
    catch (const std::runtime_error &) {rejected = true;}
    require(rejected, "another hardware owner must be rejected before initialization");
  }
  require(std::filesystem::exists(path), "keep same lock inode between starts");
  {chassis::HardwareOwner restarted(path.string());}
  std::filesystem::remove_all(path.parent_path());
}

void unified_arm_frames_do_not_become_chassis_commands()
{
  using Session = chassis::RemoteControlSession;
  using Mode = chassis::AppControlMode;
  Session session;
  session.request_start(Session::Source::app);
  session.mark_running();
  chassis::SharedCommandGate gate(session);
  constexpr std::int64_t t = 1000000000;
  gate.reset(t);
  auto frame = chassis::AppControlFrame{t + 1, Mode::chassis, 0, 0, 0, 0};
  auto result = gate.app_control(frame, frame.stamp_ns);
  require(result.accepted && result.mode_changed && result.motion_allowed,
    "neutral chassis input must establish the new ordered stream");
  frame = {t + 2, Mode::arm, 1, 0, 0, 0};
  result = gate.app_control(frame, frame.stamp_ns);
  require(result.accepted && result.mode_changed && !result.motion_allowed && gate.app_arm_mode(),
    "entering arm with a held stick must stop the old mode and await neutral");
  frame = {t + 3, Mode::arm, 0, 0, 0, 101};
  result = gate.app_control(frame, frame.stamp_ns);
  require(!result.motion_allowed && result.action == 0, "a step is not a neutral arming frame");
  frame = {t + 4, Mode::arm, 0, 0, 0, 0};
  require(gate.app_control(frame, frame.stamp_ns).motion_allowed, "fresh arm neutral arms motion");
  frame = {t + 5, Mode::arm, .5, -.25, .1, 0};
  result = gate.app_control(frame, frame.stamp_ns);
  require(result.motion_allowed && !result.mode_changed, "new normalized arm input is allowed");
  require(!gate.app_control(frame, frame.stamp_ns + 1).accepted, "duplicate frame cannot renew a lease");
  require(!gate.app_velocity(t + 6, t + 6, 1, 0, 0), "legacy velocity must not override arm mode");
  require(!gate.app_action(t + 6, t + 6, 101), "separate legacy actions must not cross the ordered stream");
  frame = {t + 7, Mode::arm, 0, 0, 0, 101};
  require(gate.app_control(frame, frame.stamp_ns).action == 101, "point action must fire once");
  frame.stamp_ns++;
  require(gate.app_control(frame, frame.stamp_ns).action == 0, "held step must not repeat");
  frame = {t + 9, Mode::arm, 0, 0, 0, 0};
  gate.app_control(frame, frame.stamp_ns);
  frame = {t + 10, Mode::arm, 0, 0, 0, 101};
  require(gate.app_control(frame, frame.stamp_ns).action == 101, "new tap after release is allowed");
  frame = {t + 11, Mode::chassis, 1, 0, 0, 0};
  result = gate.app_control(frame, frame.stamp_ns);
  require(result.mode_changed && !result.motion_allowed && !gate.app_arm_mode(),
    "exit must clear old arm intent and require chassis neutral");
  frame = {t + 12, Mode::chassis, 0, 0, 0, 109};
  require(gate.app_control(frame, frame.stamp_ns).action == 109,
    "owned stop must not require neutral");
  frame = {t + 13, Mode::arm, 0, 0, 0, 0};
  require(!gate.app_control(frame, frame.stamp_ns + 100000001).accepted, "old arm input expires");
  require(!gate.app_control(frame, frame.stamp_ns - 1).accepted, "future input is rejected");
  session.claim(Session::Source::remote);
  gate.reset(t + 14);
  frame = {t + 15, Mode::arm, 0, 0, 0, 0};
  require(!gate.app_control(frame, frame.stamp_ns).accepted, "unowned arm zero cannot interrupt remote");
  session.claim(Session::Source::app);
  gate.reset(t + 16);
  require(!gate.app_control(frame, t + 17).accepted, "pre-handover arm frame cannot resume");
  require(!gate.app_velocity(t + 17, t + 17, 0, 0, 0),
    "handover must not re-enable the separate legacy topic");
}

void strict_app_frame_protocol()
{
  const auto valid = chassis::parse_app_control_frame(
    R"({"stamp_ns":42,"mode":"arm","vx":0.5,"vy":-1,"wz":0.2,"action":0})");
  require(valid && valid->mode == chassis::AppControlMode::arm && valid->x == .5 && valid->y == -1,
    "arm frame axes must retain their explicit mode");
  for (const auto * invalid : {
      R"({"stamp_ns":42,"mode":"arm","vx":1.1,"vy":0,"wz":0,"action":0})",
      R"({"stamp_ns":42,"mode":"arm","vx":0,"vy":0,"wz":0,"action":105})",
      R"({"stamp_ns":42,"mode":"arm","vx":0,"vy":0,"wz":0,"action":106})",
      R"({"stamp_ns":42,"mode":"arm","vx":1,"vy":0,"wz":0,"action":101})",
      R"({"stamp_ns":42,"mode":"chassis","vx":0,"vy":0,"wz":0,"action":101})",
      R"({"stamp_ns":42,"mode":"manual","vx":1,"vy":0,"wz":0,"action":0})",
      R"({"stamp_ns":42,"mode":"arm","vx":".nan","vy":0,"wz":0,"action":0})",
      R"({"stamp_ns":42,"mode":"arm","vx":0,"vy":0,"wz":0,"action":0,"extra":1})",
      R"({"stamp_ns":42,"mode":"arm","vx":0,"vy":0,"wz":0})",
      R"({"stamp_ns":-1,"mode":"arm","vx":0,"vy":0,"wz":0,"action":0})"})
  {
    require(!chassis::parse_app_control_frame(invalid), "malformed arm input must not fall back to chassis");
  }
  require(!chassis::parse_app_control_frame(std::string(513, 'x')), "oversized frame is rejected");
}
}
int main()
{
  try {
    ownership_and_stale_commands(); exclusive_hardware_owner();
    unified_arm_frames_do_not_become_chassis_commands(); strict_app_frame_protocol();
  }
  catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
  return 0;
}
