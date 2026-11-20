#include "example/app_interface.h"
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using namespace std::chrono_literals;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void targets_and_manifest()
{
  const arm::Pose pose{0.2, 0.3, 0.4};
  for (int code = 101; code <= 104; ++code) {
    const auto target = chassis::app_arm_target(pose, code);
    require(bool(target), "each Cartesian command must produce a target");
    const double dx = target->x - pose.x;
    const double dz = target->z - pose.z;
    const double dp = target->pitch - pose.pitch;
    const int sign = code % 2 ? 1 : -1;
    require(std::abs(dx - (code <= 102 ? sign * 0.001 : 0.0)) < 1e-12,
      "X taps must move exactly one millimetre");
    require(std::abs(dz - (code >= 103 ? sign * 0.001 : 0.0)) < 1e-12,
      "Z taps must move exactly one millimetre");
    require(std::abs(dp) < 1e-12, "Cartesian taps must preserve pitch");
  }
  require(!chassis::app_arm_target(pose, 105) && !chassis::app_arm_target(pose, 106),
    "removed App pitch steps must not produce a target");
  require(!chassis::app_arm_target(pose, 1), "legacy ELF3 values must never move the arm");
  require(!chassis::app_arm_target(pose, 109), "stop must not become a position target");
  for (const bool enabled : {false, true}) {
    for (const bool fourth : {false, true}) {
      const auto snapshot = YAML::Load(chassis::app_state_snapshot(enabled, fourth));
      const auto graph = snapshot["graph"];
      const auto expected = enabled ? (fourth ? 7U : 5U) : 0U;
      require(snapshot["control_owner"].as<std::string>() == "none" && !graph["capabilities"],
        "legacy snapshots must not advertise shared arm control or an owner");
      require(graph["remote_events"].IsMap(), "gateway requires object remote_events");
      require(graph["actions"].size() == expected, "only available hardware gets actions");
      require(graph["remote_events"].size() == expected, "all actions need a wire binding");
      for (const auto & action : graph["actions"]) {
        const auto event = action["event"].as<std::string>();
        require(graph["remote_events"][event]["slot"].as<std::string>() == "btn_10",
          "manifest actions must use the dedicated slot");
        require(action["from"].as<std::string>() == snapshot["current"]["name"].as<std::string>(),
          "App must be able to show actions for the current state");
        const int command = graph["remote_events"][event]["value"].as<int>();
        require(command != 105 && command != 106, "pitch tap actions must be absent");
        if (command <= 104) {
          require(action["label"].as<std::string>().find("1 mm") != std::string::npos,
            "Cartesian action labels must agree with one-millimetre targets");
        }
      }
    }
  }
}

void ros_contract()
{
  auto node = std::make_shared<rclcpp::Node>(
    "car2_app_interface_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  std::vector<int> commands;
  chassis::AppInterface interface(*node, true, true,
    [&](int command) {commands.push_back(command);});
  unsigned ready = 0;
  unsigned states = 0;
  auto ready_sub = node->create_subscription<std_msgs::msg::Header>(
    "/car2/ready", rclcpp::QoS(1).best_effort(),
    [&](std_msgs::msg::Header::ConstSharedPtr message) {
      require(message->frame_id == "car2", "readiness must identify car2");
      ++ready;
    });
  auto state_sub = node->create_subscription<std_msgs::msg::String>(
    "/car2/state_machine_info", rclcpp::QoS(1).best_effort(),
    [&](std_msgs::msg::String::ConstSharedPtr message) {
      require(YAML::Load(message->data)["graph"]["actions"].size() == 7,
        "full-arm state snapshot must traverse ROS unchanged");
      ++states;
    });
  auto publisher = node->create_publisher<std_msgs::msg::Int8>(
    "/car2/arm_command", rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin = [&] {executor.spin_some(); executor.spin_some();};
  spin();
  require(ready == 0 && states == 0, "constructing the interface must not announce hardware ready");
  for (int value : {0, 1, 100, 101, 105, 106, 107, 109, 110, -1}) {
    std_msgs::msg::Int8 message;
    message.data = static_cast<int8_t>(value);
    publisher->publish(message);
    spin();
  }
  require(commands == std::vector<int>({101, 107, 109}), "reject legacy and invalid arm commands");
  const auto now = chassis::AppInterface::Clock::now();
  interface.publish_ready(now);
  spin();
  require(ready == 1 && states == 1, "a successful control cycle publishes ready and state");
  interface.publish_ready(now + 20ms);
  spin();
  require(ready == 2 && states == 1, "ready follows control cycles; state is throttled");
  interface.publish_ready(now + 100ms);
  spin();
  require(ready == 3 && states == 2, "state refresh must keep the App manifest alive");
  spin();
  require(ready == 3, "no independent timer may advertise stalled hardware as alive");
  executor.remove_node(node);
}

void absent_axes_reject_commands()
{
  auto node = std::make_shared<rclcpp::Node>(
    "car2_app_three_axes_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  std::vector<int> commands;
  chassis::AppInterface interface(*node, true, false,
    [&](int command) {commands.push_back(command);});
  auto publisher = node->create_publisher<std_msgs::msg::Int8>("/car2/arm_command", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  for (int value : {107, 108, 109}) {
    std_msgs::msg::Int8 message;
    message.data = static_cast<int8_t>(value);
    publisher->publish(message);
    executor.spin_some();
    executor.spin_some();
  }
  require(commands == std::vector<int>({109}), "three-axis setups must retain stop without fourth-axis jog");
  executor.remove_node(node);
}

void shared_contract()
{
  for (const bool arm : {false, true}) {
    const auto snapshot = YAML::Load(chassis::app_state_snapshot(arm, false, true, "remote"));
    require(snapshot["graph"]["actions"].size() == (arm ? 6U : 1U),
      "shared mode exposes takeover even without arm");
    require(snapshot["graph"]["remote_events"]["app_take_control_event"]["value"].as<int>() == 110,
      "App takeover uses reserved command 110");
    require(snapshot["control_owner"].as<std::string>() == "remote",
      "shared ownership must have an explicit machine-readable field");
    const auto capabilities = snapshot["graph"]["capabilities"];
    require(bool(capabilities) == arm, "continuous arm capability requires enabled shared arm");
    if (arm) {
      require(capabilities["car2_arm_control"].as<int>() == 1 &&
        capabilities["arm_step_m"].as<double>() == 0.001,
        "continuous arm protocol must advertise its version and one-millimetre tap");
    }
  }
  require(YAML::Load(chassis::app_state_snapshot(true, true, true, "app"))[
      "control_owner"].as<std::string>() == "app", "App ownership must be explicit");
  auto node = std::make_shared<rclcpp::Node>(
    "car2_shared_app_interface_test", rclcpp::NodeOptions().use_intra_process_comms(true));
  std::vector<int> commands;
  chassis::AppInterface interface(*node, true, false,
    [&](int) {throw std::runtime_error("shared mode must not dispatch legacy arm messages");},
    [&](int command, std::int64_t stamp) {
      require(stamp == 123456789, "shared action preserves generation timestamp");
      commands.push_back(command);
    });
  auto shared = node->create_publisher<std_msgs::msg::String>("/car2/app_arm_command", 1);
  auto legacy = node->create_publisher<std_msgs::msg::Int8>("/car2/arm_command", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  for (const auto & data : {"{", "[]", "{command: 101}",
      "{command: 101, stamp_ns: 123456789, extra: 1}",
      "{command: 107, stamp_ns: 123456789}",
      "{command: 105, stamp_ns: 123456789}",
      "{command: 106, stamp_ns: 123456789}",
      "{command: 101, stamp_ns: 123456789}",
      "{command: 109, stamp_ns: 123456789}",
      "{command: 110, stamp_ns: 123456789}"})
  {
    std_msgs::msg::String message;
    message.data = data;
    shared->publish(message);
    executor.spin_some(); executor.spin_some();
  }
  std_msgs::msg::Int8 old;
  old.data = 101;
  legacy->publish(old);
  executor.spin_some(); executor.spin_some();
  require(commands == std::vector<int>({101, 109, 110}),
    "shared interface rejects malformed, legacy and unavailable axis messages");
  executor.remove_node(node);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    targets_and_manifest();
    ros_contract();
    absent_axes_reject_commands();
    shared_contract();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
