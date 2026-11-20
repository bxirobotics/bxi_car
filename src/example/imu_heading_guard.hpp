#pragma once

#include "sensor_msgs/msg/imu.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace chassis
{

// State only: the hardware owner's executor supplies both clocks and observations.
class ImuHeadingGuard
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr auto kMaxSampleAge = std::chrono::milliseconds(200);
  static constexpr auto kMaxFuture = std::chrono::milliseconds(2);
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kDefaultHeadingLimit = 8.0 * kPi / 180.0;

  explicit ImuHeadingGuard(double heading_limit = kDefaultHeadingLimit)
  : heading_limit_(heading_limit)
  {
    if (!std::isfinite(heading_limit) || heading_limit <= 0.0 || heading_limit >= kPi) {
      throw std::invalid_argument("IMU heading limit must be finite and within (0, pi)");
    }
  }

  bool observe(
    const sensor_msgs::msg::Imu & message, std::int64_t ros_now_ns,
    Clock::time_point now)
  {
    if (active_ && !fault_.empty()) {return false;}
    if (last_receipt_ && now < *last_receipt_) {return fail("IMU monotonic clock reversed");}
    if (sample_time_ && now - *sample_time_ > kMaxSampleAge) {
      if (active_) {return fail("IMU data expired");}
      consecutive_ = 0;
    }
    last_receipt_ = now;
    const auto & stamp = message.header.stamp;
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || ros_now_ns <= 0) {
      return fail("invalid IMU timestamp");
    }
    const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const auto max_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kMaxSampleAge).count();
    const auto max_future_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      kMaxFuture).count();
    if (stamp_ns <= 0 || (stamp_ns > ros_now_ns && stamp_ns - ros_now_ns > max_future_ns) ||
      (stamp_ns <= ros_now_ns && ros_now_ns - stamp_ns > max_age_ns))
    {
      return fail("stale or future IMU timestamp");
    }
    if (last_stamp_ns_ && stamp_ns <= *last_stamp_ns_) {
      return fail("repeated or reordered IMU timestamp");
    }
    // Even a rejected body cannot later be replayed with this timestamp.
    last_stamp_ns_ = stamp_ns;
    if (message.header.frame_id != "imu_link") {return fail("unexpected IMU frame");}
    if (message.orientation_covariance[0] == -1.0) {
      return fail("IMU orientation unavailable");
    }
    const auto & q = message.orientation;
    if (!finite(q.w, q.x, q.y) || !std::isfinite(q.z)) {
      return fail("non-finite IMU quaternion");
    }
    const double norm = std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
    if (norm < 0.9 || norm > 1.1) {return fail("invalid IMU quaternion norm");}
    const double w = q.w / norm, x = q.x / norm, y = q.y / norm, z = q.z / norm;
    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    if (std::max(std::abs(roll), std::abs(pitch)) > 20.0 * kPi / 180.0) {
      return fail("IMU tilt exceeds 20 degrees");
    }
    const auto & gyro = message.angular_velocity;
    if (!finite(gyro.x, gyro.y, gyro.z)) {return fail("non-finite IMU angular velocity");}
    if (std::abs(gyro.z) > 2.0) {return fail("IMU yaw rate exceeds 2 rad/s");}
    yaw_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    sample_time_ = now - std::chrono::nanoseconds(std::max<std::int64_t>(0, ros_now_ns - stamp_ns));
    consecutive_ = std::min(consecutive_ + 1, 3U);
    if (active_ && std::abs(heading_error()) > heading_limit_) {
      return fail("IMU heading deviation exceeds limit");
    }
    error_.clear();
    return true;
  }

  bool start_session(Clock::time_point now)
  {
    // Repeated start checks must not replace an active session's reference.
    if (active_) {return check(now);}
    if (consecutive_ < 3 || !sample_time_) {
      error_ = "IMU needs three consecutive fresh samples";
      return false;
    }
    if (!fresh(now)) {return fail("IMU data expired");}
    active_ = true;
    reference_yaw_ = yaw_;
    fault_.clear();
    error_.clear();
    return true;
  }

  bool check(Clock::time_point now)
  {
    if (!active_) {
      error_ = "IMU session is inactive";
      return false;
    }
    if (!fault_.empty()) {return false;}
    if (!fresh(now)) {return fail("IMU data expired");}
    if (std::abs(heading_error()) > heading_limit_) {
      return fail("IMU heading deviation exceeds limit");
    }
    return true;
  }

  void end_session()
  {
    active_ = false;
    reference_yaw_ = 0.0;
    fault_.clear();
    error_.clear();
  }

  bool active() const {return active_;}
  const std::string & error() const {return error_;}
  double reference_yaw() const {return reference_yaw_;}
  // Positive is measured yaw to the left of the session's starting yaw.
  double heading_error() const {return std::remainder(yaw_ - reference_yaw_, 2.0 * kPi);}

private:
  static bool finite(double x, double y, double z)
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

  bool fresh(Clock::time_point now) const
  {
    return sample_time_ && last_receipt_ && now >= *last_receipt_ &&
           now >= *sample_time_ && now - *sample_time_ <= kMaxSampleAge;
  }

  bool fail(const char * reason)
  {
    consecutive_ = 0;
    sample_time_.reset();
    error_ = reason;
    if (active_) {fault_ = error_;}
    return false;
  }

  double heading_limit_;
  double yaw_{0.0};
  double reference_yaw_{0.0};
  unsigned consecutive_{0};
  bool active_{false};
  std::optional<std::int64_t> last_stamp_ns_;
  std::optional<Clock::time_point> sample_time_;
  std::optional<Clock::time_point> last_receipt_;
  std::string error_{"IMU needs three consecutive fresh samples"};
  std::string fault_;
};

}  // namespace chassis
