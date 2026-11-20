#include "example/imu_heading_guard.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using Guard = chassis::ImuHeadingGuard;
using Clock = Guard::Clock;
using namespace std::chrono_literals;
constexpr auto kStart = Clock::time_point(10s);
constexpr std::int64_t kWallStart = 20000000000LL;
constexpr double kDegree = Guard::kPi / 180.0;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

sensor_msgs::msg::Imu sample(
  std::int64_t stamp_ns, double yaw = 0.0, double pitch = 0.0, double roll = 0.0)
{
  sensor_msgs::msg::Imu message;
  message.header.frame_id = "imu_link";
  message.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  message.orientation.w = cr * cp * cy + sr * sp * sy;
  message.orientation.x = sr * cp * cy - cr * sp * sy;
  message.orientation.y = cr * sp * cy + sr * cp * sy;
  message.orientation.z = cr * cp * sy - sr * sp * cy;
  return message;
}

struct Fixture
{
  Guard guard;
  Clock::time_point now{kStart};
  std::int64_t wall{kWallStart};

  bool feed(double yaw = 0.0)
  {
    now += 2ms;
    wall += 2000000LL;
    return guard.observe(sample(wall, yaw), wall, now);
  }

  void ready(double yaw = 0.0)
  {
    require(feed(yaw) && feed(yaw) && feed(yaw), "three valid frames must be accepted");
  }

  void start(double yaw = 0.0)
  {
    ready(yaw);
    require(guard.start_session(now), "fresh frames must allow a session");
  }
};

void acquisition_and_inactive_checks()
{
  Fixture f;
  require(!f.guard.check(f.now), "inactive guard must not authorize motion");
  require(!f.guard.start_session(f.now), "no IMU must block session entry");
  require(!f.guard.active(), "failed entry must not activate a session");
  require(f.feed() && f.feed(), "first two good frames should be accepted");
  require(!f.guard.start_session(f.now), "two frames cannot authorize motion");
  require(f.feed(0.3), "third frame must be accepted");
  require(f.guard.start_session(f.now), "third valid frame must authorize entry");
  require(std::abs(f.guard.reference_yaw() - 0.3) < 1e-12, "entry locks the current yaw");
  require(f.feed(0.31), "small heading change must be allowed");
  require(f.guard.start_session(f.now), "repeated start only checks the current session");
  require(std::abs(f.guard.reference_yaw() - 0.3) < 1e-12,
    "repeated start must never reset the heading reference");
}

void expiration_includes_original_message_age()
{
  Fixture f;
  for (int i = 0; i < 3; ++i) {
    f.now += 2ms;
    f.wall += 2000000LL;
    require(f.guard.observe(sample(f.wall - 150000000LL), f.wall, f.now),
      "a 150 ms old message is inside the accepted capture-age window");
  }
  require(f.guard.start_session(f.now), "fresh capture can start a session");
  require(f.guard.check(f.now + 50ms), "capture age of exactly 200 ms is accepted");
  require(!f.guard.check(f.now + 50ms + 1ns), "capture age beyond 200 ms must stop motion");
  require(!f.feed(), "fresh data cannot revive a faulted session");
  require(!f.guard.start_session(f.now), "repeated start cannot clear the timeout latch");
}

void fresh_gap_cannot_hide_a_timeout()
{
  Fixture f;
  f.start();
  f.now += 201ms;
  f.wall += 201000000LL;
  require(!f.feed(), "a new frame after a long gap must latch the active session's timeout");
  require(f.guard.error() == "IMU data expired", "gap must preserve its timeout reason");
  f.guard.end_session();
  require(f.feed() && f.feed(), "fresh post-fault frames can acquire outside the session");
  require(!f.guard.start_session(f.now), "post-fault acquisition still needs three new frames");
  require(f.feed(0.5) && f.guard.start_session(f.now), "new session can capture a new yaw");
  require(std::abs(f.guard.reference_yaw() - 0.5) < 1e-12, "new entry must rebaseline");
}

void wrap_and_heading_fault_latch()
{
  Fixture f;
  f.start(179 * kDegree);
  require(f.feed(-179 * kDegree), "crossing pi should be a 2 degree change");
  require(std::abs(f.guard.heading_error() - 2 * kDegree) < 1e-12,
    "heading error must wrap across pi");
  require(f.guard.check(f.now), "wrapped small error remains healthy");
  require(!f.feed(-171 * kDegree), "10 degree deviation must cancel motion");
  const std::string reason = f.guard.error();
  require(!f.feed(179 * kDegree), "returning to the reference must not restart motion");
  require(f.guard.error() == reason, "later frames cannot replace the latched fault reason");
  require(!f.guard.check(f.now), "heading fault must remain latched");
  f.guard.end_session();
  f.ready(-170 * kDegree);
  require(f.guard.start_session(f.now), "operator stop and new entry can reacquire heading");
}

void bad_frames_fail_before_and_during_motion()
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const std::function<void(sensor_msgs::msg::Imu &)> changes[] = {
    [](auto & m) {m.header.frame_id = "camera_imu";},
    [](auto & m) {m.header.stamp.sec = -1;},
    [](auto & m) {m.header.stamp.nanosec = 1000000000U;},
    [](auto & m) {m.orientation_covariance[0] = -1;},
    [nan](auto & m) {m.orientation.w = nan;},
    [](auto & m) {m.orientation.w = 0.0;},
    [](auto & m) {m.orientation.w = 1.2;},
    [nan](auto & m) {m.angular_velocity.x = nan;},
    [nan](auto & m) {m.angular_velocity.y = nan;},
    [nan](auto & m) {m.angular_velocity.z = nan;},
    [](auto & m) {m.angular_velocity.z = 2.01;},
    [](auto & m) {m.angular_velocity.z = -2.01;},
  };
  for (const auto & change : changes) {
    for (bool active : {false, true}) {
      Fixture f;
      if (active) {f.start();} else {require(f.feed() && f.feed(), "acquire before bad frame");}
      f.now += 2ms;
      f.wall += 2000000LL;
      auto message = sample(f.wall);
      change(message);
      require(!f.guard.observe(message, f.wall, f.now), "invalid frame must be rejected");
      require(!f.guard.start_session(f.now), "bad frame cannot permit motion");
      if (active) {
        require(!f.feed(), "valid data cannot clear an active fault");
        f.guard.end_session();
      }
      require(f.feed() && f.feed(), "good frames should resume acquisition outside a session");
      require(!f.guard.start_session(f.now), "bad frame resets the three-frame count");
      require(f.feed() && f.guard.start_session(f.now), "three new frames restore entry");
    }
  }
}

void tilted_sensor_cancels_motion()
{
  for (bool pitch : {false, true}) {
    for (double angle : {-20.1, 20.1}) {
      Fixture f;
      f.start();
      f.now += 2ms;
      f.wall += 2000000LL;
      const auto message = sample(f.wall, 0.0, pitch ? angle * kDegree : 0.0,
        pitch ? 0.0 : angle * kDegree);
      require(!f.guard.observe(message, f.wall, f.now), "excessive roll or pitch must cancel");
      require(!f.guard.check(f.now), "tilt failure remains latched");
    }
  }
}

void timestamps_cannot_repeat_or_move_backwards()
{
  for (std::int64_t delta : {0LL, -1000000LL, -201000000LL, 2000001LL}) {
    Fixture f;
    f.start();
    require(!f.guard.observe(sample(f.wall + delta), f.wall, f.now),
      "repeated, reordered, old or future samples must be rejected");
    require(!f.guard.check(f.now), "timestamp rejection must latch during motion");
  }
  Fixture f;
  require(f.feed(), "first frame accepted");
  f.now += 2ms;
  f.wall += 2000000LL;
  auto invalid = sample(f.wall);
  invalid.orientation.w = 0;
  require(!f.guard.observe(invalid, f.wall, f.now), "reject bad quaternion");
  require(!f.guard.observe(sample(f.wall), f.wall, f.now),
    "a corrected body cannot replay the rejected body's timestamp");
  require(f.guard.error() == "repeated or reordered IMU timestamp",
    "body rejection must retain the timestamp high-water mark");
  f.guard.end_session();
  require(!f.guard.observe(sample(f.wall), f.wall, f.now),
    "operator stop must not discard the anti-replay high-water mark");
}

void stale_acquisition_and_clock_reversal_block_entry()
{
  Fixture f;
  f.ready();
  require(!f.guard.start_session(f.now + 201ms), "stale pre-session data must block entry");
  require(!f.guard.check(f.now + 201ms), "failed entry must remain inactive");
  Fixture g;
  g.start();
  require(!g.guard.check(g.now - 1ns), "a check before the last receipt must cancel");
  Fixture h;
  h.start();
  require(!h.guard.observe(sample(h.wall + 1), h.wall + 1, h.now - 1ns),
    "reversed monotonic observation time must cancel");
}

void continuing_fixed_goal_still_requires_fresh_imu()
{
  Fixture f;
  f.start();
  const double reference = f.guard.reference_yaw();
  for (int i = 0; i < 300; ++i) {
    require(f.feed(0.01), "motion phase change must not need a new reference");
    require(f.guard.check(f.now), "fresh IMU keeps a fixed goal healthy");
  }
  require(f.guard.reference_yaw() == reference, "reference must span the full goal lifetime");
  require(!f.guard.check(f.now + 201ms), "fixed goal execution cannot outlive its IMU stream");
}

void invalid_configuration_is_rejected()
{
  for (double limit : {0.0, -1.0, Guard::kPi,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    bool rejected = false;
    try {Guard guard(limit);} catch (const std::invalid_argument &) {rejected = true;}
    require(rejected, "invalid heading limit must fail before motion");
  }
}
}  // namespace

int main()
{
  try {
    acquisition_and_inactive_checks();
    expiration_includes_original_message_age();
    fresh_gap_cannot_hide_a_timeout();
    wrap_and_heading_fault_latch();
    bad_frames_fail_before_and_during_motion();
    tilted_sensor_cancels_motion();
    timestamps_cannot_repeat_or_move_backwards();
    stale_acquisition_and_clock_reversal_block_entry();
    continuing_fixed_goal_still_requires_fresh_imu();
    invalid_configuration_is_rejected();
    std::cout << "IMU heading guard tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "IMU heading guard: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
