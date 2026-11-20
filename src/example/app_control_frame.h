#pragma once

#include "yaml-cpp/yaml.h"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace chassis
{
enum class AppControlMode {chassis, arm};

// One ordered envelope keeps mode, velocity and one-shot actions together.
// Chassis axes use m/s, m/s, rad/s; arm axes are normalized X, Z and pitch.
struct AppControlFrame
{
  std::int64_t stamp_ns{};
  AppControlMode mode{AppControlMode::chassis};
  double x{}, y{}, yaw{};
  int action{};

  bool neutral() const noexcept
  {
    return std::abs(x) <= 1e-6 && std::abs(y) <= 1e-6 &&
           std::abs(yaw) <= 1e-6 && action == 0;
  }

  bool valid() const noexcept
  {
    if (stamp_ns <= 0 || (mode != AppControlMode::chassis && mode != AppControlMode::arm) ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
      (action != 0 && (action < 101 || action > 110 || action == 105 || action == 106)))
    {return false;}
    if (mode == AppControlMode::arm &&
      (std::abs(x) > 1.0 || std::abs(y) > 1.0 || std::abs(yaw) > 1.0)) {return false;}
    return action < 101 || action > 104 ||
           (mode == AppControlMode::arm && std::abs(x) <= 1e-6 &&
           std::abs(y) <= 1e-6 && std::abs(yaw) <= 1e-6);
  }
};

inline std::optional<AppControlFrame> parse_app_control_frame(const std::string & text)
{
  if (text.empty() || text.size() > 512) {return std::nullopt;}
  try {
    const auto data = YAML::Load(text);
    if (!data.IsMap() || data.size() != 6) {return std::nullopt;}
    AppControlFrame frame;
    frame.stamp_ns = data["stamp_ns"].as<std::int64_t>();
    const auto mode = data["mode"].as<std::string>();
    if (mode != "chassis" && mode != "arm") {return std::nullopt;}
    frame.mode = mode == "arm" ? AppControlMode::arm : AppControlMode::chassis;
    frame.x = data["vx"].as<double>();
    frame.y = data["vy"].as<double>();
    frame.yaw = data["wz"].as<double>();
    frame.action = data["action"].as<int>();
    if (!frame.valid()) {return std::nullopt;}
    return frame;
  } catch (const YAML::Exception &) {return std::nullopt;}
}
}  // namespace chassis
