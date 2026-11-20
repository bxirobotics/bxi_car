#include "example/arm_visual_tracking.h"
#include "example/app_arm_control.h"
#include "rcl/time.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace
{
using namespace std::chrono_literals;
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoResponse;
using Clock = chassis::ArmVisualTracking::Clock;
constexpr double kPulses = 3600000.0;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class CoeNode final : public chassis::ethercat::Node
{
public:
  explicit CoeNode(std::uint16_t id) : id_(id)
  {
    put<std::uint16_t>(0x6041, 0x40);
    put<std::uint8_t>(0x2100, 1);
    put<std::int8_t>(0x6060, 0);
    put<std::int8_t>(0x6061, 0);
    put<std::int8_t>(0x2707, 1);
    put<std::int32_t>(0x6064, 0);
    put<std::int32_t>(0x606C, 0);
    put<std::int32_t>(0x60FF, 0);
    put<std::int16_t>(0x6071, 0);
    put<std::int16_t>(0x6078, 0);
    put<std::int16_t>(0x60B2, 0);
    put<std::int32_t>(0x2025, static_cast<std::int32_t>(kPulses));
    put<std::uint16_t>(0x26A2, 1);
    put<std::uint16_t>(0x26A3, 1);
  }

  std::uint16_t id() const noexcept override {return id_;}

  template<typename T>
  void put(std::uint16_t index, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values_[{index, 0}] = {bytes.begin(), bytes.end()};
  }

  template<typename T>
  T get(std::uint16_t index)
  {
    const auto result = read(iswv::canopen::Object<T>{{index, 0}, "fake object"});
    require(bool(result), "fake object must exist");
    return result.value();
  }

  iswv::Result<SdoResponse> upload_blocking(
    ObjectAddress object, iswv::canopen::SdoOptions) override
  {
    ++reads;
    if (object.index == 0x2707 && fail_standstill_read) {
      return iswv::Result<SdoResponse>::failure(
        iswv::make_sdo_abort(0x08000000, "injected standstill read failure"));
    }
    const auto found = values_.find(object);
    if (found == values_.end()) {
      return iswv::Result<SdoResponse>::failure(
        iswv::make_sdo_abort(0x06020000, "unknown test object"));
    }
    SdoResponse response;
    response.object = object;
    response.size = static_cast<std::uint8_t>(found->second.size());
    std::copy(found->second.begin(), found->second.end(), response.data.begin());
    return iswv::Result<SdoResponse>::success(response);
  }

  iswv::Result<void> download_blocking(
    ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions) override
  {
    ++writes;
    std::uint32_t raw = 0;
    for (std::size_t i = 0; i < size; ++i) {
      raw |= static_cast<std::uint32_t>(data[i]) << (8U * i);
    }
    if (object.index == 0x6040 && (raw & 0x0100U) && fail_halt) {
      return iswv::Result<void>::failure(
        iswv::make_sdo_abort(0x08000000, "injected halt failure"));
    }
    if (object.index == 0x6083 && fail_profile) {
      return iswv::Result<void>::failure(
        iswv::make_sdo_abort(0x08000000, "injected profile failure"));
    }
    values_[object] = {data, data + size};
    if (object.index == 0x607A) {++position_writes;}
    if (object.index == 0x6081 || object.index == 0x6083 || object.index == 0x6084) {
      ++profile_writes;
    }
    if (object.index == 0x6060) {put<std::int8_t>(0x6061, static_cast<std::int8_t>(raw));}
    if (object.index == 0x6040) {
      std::uint16_t status = 0x40;
      switch (raw & 0x0f) {
        case 0x0b: status = 0x07; break;
        case 0x06: status = 0x21; break;
        case 0x07: status = 0x23; break;
        case 0x0f: status = (raw & 0x10U) ? 0x1027 : 0x27; break;
        default: break;
      }
      if (raw & 0x10U) {++triggers;}
      put<std::uint16_t>(0x6041, status);
    }
    return iswv::Result<void>::success();
  }

  unsigned triggers{0};
  unsigned reads{0};
  unsigned writes{0};
  unsigned position_writes{0};
  unsigned profile_writes{0};
  bool fail_halt{false};
  bool fail_profile{false};
  bool fail_standstill_read{false};

private:
  std::uint16_t id_;
  std::map<ObjectAddress, std::vector<std::uint8_t>> values_;
};

struct Fixture
{
  explicit Fixture(
    bool controlled_ros_time = false, bool fourth_axis = true,
    std::function<bool()> external_allowed = {},
    std::function<std::uint64_t()> current_generation = {})
  {
    // These scenarios use a folded zero pose for their reachability and pitch-wrap boundaries.
    motion.kinematics.zero_offsets = {5.0 * arm::kPi / 6.0, -5.0 * arm::kPi / 6.0, 0.0};
    motion.kinematics.limits[1] = {-arm::kPi / 6.0, 11.0 * arm::kPi / 6.0};
    for (std::size_t i = 0; i < motors.size(); ++i) {
      motors[i] = std::make_shared<CoeNode>(static_cast<std::uint16_t>(i + 1));
    }
    controller = std::make_unique<chassis::ArmController>(
      std::array<std::shared_ptr<chassis::ethercat::Node>, 3>{motors[0], motors[1], motors[2]},
      [this] {power_off = true;}, fourth_axis ? motors[3] : nullptr, motion);
    controller->initialize();
    node = std::make_shared<rclcpp::Node>(
      "arm_visual_tracking_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    if (controlled_ros_time) {
      require(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()) == RCL_RET_OK,
        "test ROS clock override must enable");
      ros_time_ns = 10000000000LL;
      require(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), *ros_time_ns) ==
        RCL_RET_OK, "test ROS time must initialize");
    }
    chassis::declare_arm_pose_parameter(*node);
    chassis::declare_arm_tracking_parameter(*node);
    tracker = std::make_unique<chassis::ArmVisualTracking>(
      *node, *controller, std::function<void(const std::exception &)>{}, std::move(external_allowed),
      std::move(current_generation));
    now = Clock::now() + 20ms;
    initial = controller->actual_pose();
    for (int i = 2; i >= 0; --i) {imu(0.0, now - std::chrono::milliseconds(i));}
  }

  void imu(double yaw, Clock::time_point received_at)
  {
    sensor_msgs::msg::Imu message;
    message.header.frame_id = "imu_link";
    const auto stamp = std::max(last_imu_stamp + 1, node->now().nanoseconds());
    message.header.stamp = rclcpp::Time(stamp);
    last_imu_stamp = stamp;
    message.orientation.w = std::cos(yaw / 2.0);
    message.orientation.z = std::sin(yaw / 2.0);
    tracker->observe_imu(message, received_at);
  }

  void tick(std::chrono::milliseconds elapsed = 20ms, bool plan = true)
  {
    // IMU samples continue arriving between controller updates in a long test tick.
    while (auto_imu && elapsed > 50ms) {
      advance_time(50ms);
      imu(imu_yaw, now);
      elapsed -= 50ms;
    }
    advance_time(elapsed);
    if (auto_imu) {imu(imu_yaw, now);}
    tracker->before_update(now);
    controller->update(now);
    if (plan) {tracker->after_update(now);}
  }

  void advance_time(std::chrono::milliseconds elapsed)
  {
    // Executor callbacks read the real steady clock; do not feed them future samples.
    if (pace_ticks) {std::this_thread::sleep_until(now + elapsed);}
    now += elapsed;
    if (ros_time_ns) {
      *ros_time_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
      require(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), *ros_time_ns) ==
        RCL_RET_OK, "test ROS time must advance");
    }
  }

  void enable()
  {
    require(node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", true)).successful,
      "enable must commit");
    tick();
  }

  void enable_x()
  {
    require(node->set_parameters_atomically({
        rclcpp::Parameter("arm_tracking_x_enabled", true),
        rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
      "atomic X tracking enable must commit");
    tick();
  }

  void enable_goal()
  {
    require(node->set_parameters_atomically({
        rclcpp::Parameter("arm_tracking_x_enabled", true),
        rclcpp::Parameter("arm_tracking_goal_enabled", true),
        rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
      "atomic fixed goal enable must commit");
    tick();
  }

  geometry_msgs::msg::PointStamped message(double z, double y = 0.5)
  {
    geometry_msgs::msg::PointStamped result;
    result.header.frame_id = "arm_shoulder";
    result.header.stamp = node->now();
    result.point.x = 1.0;  // Legacy Z-only sessions must ignore this X coordinate.
    result.point.y = y;
    result.point.z = z;
    return result;
  }

  void target(double z, double y = 0.5) {tracker->observe_target(message(z, y), now);}

  void target_xz(double x, double z, double y = 0.5)
  {
    auto point = message(z, y);
    point.point.x = x;
    tracker->observe_target(point, now);
  }

  geometry_msgs::msg::PointStamped goal_message(double x, double z)
  {
    auto point = message(z, 0.0);
    point.point.x = x;
    return point;
  }

  void goal(double x, double z) {tracker->observe_goal(goal_message(x, z), now);}

  void start_target()
  {
    enable();
    target(initial.z + 0.05);
    tick(100ms);
    require(controller->position_command_pending(), "fresh target must queue the complete endpoint");
  }

  void finish_handshake()
  {
    for (unsigned i = 0; i < 8 && controller->position_command_pending(); ++i) {
      tick(20ms, false);
    }
    require(!controller->position_command_pending(), "fake PP handshake must finish");
  }

  bool halted()
  {
    return std::all_of(motors.begin(), motors.begin() + 3, [](const auto & motor) {
      return (motor->template get<std::uint16_t>(0x6040) & 0x0100U) != 0;
    });
  }

  arm::JointAngles commanded_joints()
  {
    arm::JointAngles angles{};
    for (std::size_t i = 0; i < angles.size(); ++i) {
      angles[i] = motors[i]->get<std::int32_t>(0x607A) * 2.0 * arm::kPi / kPulses;
    }
    return angles;
  }

  arm::Pose commanded_pose()
  {
    return arm::Kinematics(motion.kinematics).forward(commanded_joints());
  }

  void measure_joints(const arm::JointAngles & joints)
  {
    for (std::size_t i = 0; i < joints.size(); ++i) {
      motors[i]->put<std::int32_t>(0x6064, static_cast<std::int32_t>(std::llround(
          joints[i] * kPulses / (2.0 * arm::kPi))));
    }
  }

  void measure(const arm::Pose & pose)
  {
    const auto result = arm::Kinematics(motion.kinematics).inverse(pose, arm::JointAngles{0, 0, 0});
    require(result.status == arm::InverseStatus::Success, "test measured pose must be feasible");
    measure_joints(result.solutions.front().joints);
  }

  chassis::ArmMotionConfig motion;
  bool power_off{false};
  std::array<std::shared_ptr<CoeNode>, 4> motors;
  std::unique_ptr<chassis::ArmController> controller;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<chassis::ArmVisualTracking> tracker;
  Clock::time_point now;
  bool pace_ticks{false};
  bool auto_imu{true};
  double imu_yaw{0.0};
  std::int64_t last_imu_stamp{0};
  std::optional<std::int64_t> ros_time_ns;
  arm::Pose initial;
};

using Profile = std::array<std::uint32_t, 3>;

Profile read_profile(const std::shared_ptr<CoeNode> & motor)
{
  return {motor->get<std::uint32_t>(0x6081), motor->get<std::uint32_t>(0x6083),
    motor->get<std::uint32_t>(0x6084)};
}

// Controller-only rig: unequal encoder scales, reversed motor 2, and nonzero encoder zeros.
// It verifies commands/guards, not the physical trajectory produced by drive firmware.
struct ProfileFixture
{
  explicit ProfileFixture(double speed_rpm = 0.6)
  {
    motion.speed_rpm = speed_rpm;
    motion.kinematics.zero_offsets = {5.0 * arm::kPi / 6.0, -5.0 * arm::kPi / 6.0, 0.0};
    motion.kinematics.limits[1] = {-arm::kPi / 6.0, 11.0 * arm::kPi / 6.0};
    motion.motor_directions = {1, -1, 1};
    motion.zero_counts = {12000, -30000, 45000};
    motion.home_joints = {0.02, 0.04, -0.01};
    for (unsigned i = 0; i < 3; ++i) {
      motors[i] = std::make_shared<CoeNode>(static_cast<std::uint16_t>(i + 1));
      motors[i]->put<std::int32_t>(0x2025, scales[i]);
      motors[i]->put<std::int32_t>(0x6064, motion.zero_counts[i]);
    }
    controller = std::make_unique<chassis::ArmController>(
      std::array<std::shared_ptr<chassis::ethercat::Node>, 3>{motors[0], motors[1], motors[2]},
      [this] {power_off = true;}, nullptr, motion);
    controller->initialize();
    now = Clock::now() + 20ms;
    for (unsigned i = 0; i < 3; ++i) {defaults[i] = read_profile(motors[i]);}
  }

  arm::Pose pose(const std::array<std::int32_t, 3> & displacement) const
  {
    arm::JointAngles joints{};
    for (unsigned i = 0; i < 3; ++i) {
      joints[i] = motion.motor_directions[i] * double(displacement[i]) *
        2.0 * arm::kPi / scales[i];
    }
    return arm::Kinematics(motion.kinematics).forward(joints);
  }

  void tick()
  {
    now += 20ms;
    controller->update(now);
  }

  void finish()
  {
    for (unsigned i = 0; i < 12 && controller->position_command_pending(); ++i) {tick();}
    require(!controller->position_command_pending() && !power_off &&
      controller->fault_reason().empty(), "profile test handshake must finish without fault");
  }

  std::array<unsigned, 3> triggers() const
  {
    return {motors[0]->triggers, motors[1]->triggers, motors[2]->triggers};
  }

  std::int32_t stationary_velocity_limit(unsigned axis) const
  {
    return static_cast<std::int32_t>(std::floor(
        std::min(0.02, motion.speed_rpm * 0.05) * scales[axis] / 60.0));
  }

  void expect_defaults() const
  {
    for (unsigned i = 0; i < 3; ++i) {
      require(read_profile(motors[i]) == defaults[i], "ordinary commands must restore all original PP limits");
    }
  }

  void expect_fault_without_trigger(const std::array<unsigned, 3> & before)
  {
    bool failed = false;
    try {
      for (unsigned i = 0; i < 10 && controller->position_command_pending(); ++i) {tick();}
    } catch (const std::exception &) {failed = true;}
    require(failed && power_off && !controller->fault_reason().empty(),
      "unsafe profile setup must latch a fault and request power off");
    require(triggers() == before, "unsafe profile setup must not trigger any axis");
  }

  chassis::ArmMotionConfig motion;
  const std::array<std::int32_t, 3> scales{3600000, 7200000, 1800000};
  std::array<std::shared_ptr<CoeNode>, 3> motors;
  std::array<Profile, 3> defaults{};
  bool power_off{false};
  std::unique_ptr<chassis::ArmController> controller;
  Clock::time_point now;
};

void synchronized_profiles_scale_signed_pulse_travel_before_any_trigger()
{
  for (const auto & travel : {std::array<std::int32_t, 3>{12000, -48000, 9000},
      std::array<std::int32_t, 3>{-6000, 24000, -4500}})
  {
    ProfileFixture f;
    const auto triggers = f.triggers();
    const std::array<unsigned, 3> writes{f.motors[0]->profile_writes,
      f.motors[1]->profile_writes, f.motors[2]->profile_writes};
    const std::array<unsigned, 3> targets{f.motors[0]->position_writes,
      f.motors[1]->position_writes, f.motors[2]->position_writes};
    f.controller->set_synchronized_target_pose(f.pose(travel));
    for (unsigned step = 0; step < 12 && f.controller->position_command_pending(); ++step) {
      f.tick();
      if (f.triggers() == triggers) {continue;}
      for (unsigned i = 0; i < 3; ++i) {
        require(f.motors[i]->profile_writes == writes[i] + 3 &&
          f.motors[i]->position_writes == targets[i] + 1,
          "all three profiles and targets must be staged before the first trigger");
      }
    }
    require(!f.controller->position_command_pending(), "signed travel handshake must finish");
    const std::array<Profile, 3> expected{
      Profile{18000, 30000, 30000}, Profile{72000, 120000, 120000},
      Profile{13500, 22500, 22500}};
    for (unsigned i = 0; i < 3; ++i) {
      require(read_profile(f.motors[i]) == expected[i],
        "PP limits must follow absolute pulse travel, including encoder scale and motor sign");
      require(f.motors[i]->get<std::int32_t>(0x607A) == f.motion.zero_counts[i] + travel[i],
        "profile scaling must preserve the complete signed endpoint");
      for (unsigned field = 0; field < 3; ++field) {
        require(expected[i][field] > 0 && expected[i][field] <= f.defaults[i][field],
          "synchronized profiles must remain positive and within every original limit");
      }
      require(f.motors[i]->triggers == triggers[i] + 1, "each synchronized axis triggers once");
    }
  }
}

void synchronized_profiles_handle_zero_and_one_pulse_axes()
{
  const std::array<std::array<std::int32_t, 3>, 3> travels{
    std::array<std::int32_t, 3>{0, -48000, 9000}, {0, 0, 0}, {1, -720000, 0}};
  for (unsigned scenario = 0; scenario < travels.size(); ++scenario) {
    ProfileFixture f;
    const auto triggers = f.triggers();
    const auto writes = std::array<unsigned, 3>{f.motors[0]->writes,
      f.motors[1]->writes, f.motors[2]->writes};
    require(f.controller->set_synchronized_target_pose(f.pose(travels[scenario])),
      "valid zero/tiny synchronized targets must be accepted from standstill");
    f.finish();
    for (unsigned i = 0; i < 3; ++i) {
      require(f.motors[i]->get<std::int32_t>(0x607A) ==
        f.motion.zero_counts[i] + travels[scenario][i], "zero/tiny axes must keep exact pulse endpoints");
      const auto profile = read_profile(f.motors[i]);
      for (unsigned field = 0; field < 3; ++field) {
        require(profile[field] > 0 && profile[field] <= f.defaults[i][field],
          "zero/tiny axes must not create zero or over-limit PP fields");
      }
      if (travels[scenario][i] == 0) {
        require(profile == f.defaults[i], "stationary axes must retain usable original PP limits");
      }
      if (scenario == 1) {
        require(f.motors[i]->writes == writes[i] && f.motors[i]->triggers == triggers[i],
          "an all-zero synchronized move must perform no writes or trigger");
      }
    }
    if (scenario == 2) {
      require(read_profile(f.motors[0]) == Profile{1, 1, 1},
        "a one-pulse axis below integer resolution must clamp PP limits to one");
    }
  }
}

void synchronized_profiles_defer_initial_motion_and_reject_configuration_drift()
{
  for (const double speed_rpm : {0.6, 0.001, 0.00002}) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      for (const int direction : {-1, 0, 1}) {
        ProfileFixture f(speed_rpm);
        f.motors[axis]->put<std::int32_t>(0x606C,
          direction * (f.stationary_velocity_limit(axis) + 1));
        f.motors[axis]->put<std::int8_t>(0x2707, direction == 0 ? 0 : 1);
        const auto before = f.triggers();
        const unsigned writes = f.motors[0]->writes + f.motors[1]->writes + f.motors[2]->writes;
        require(!f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000})) &&
          !f.controller->position_command_pending() && !f.power_off &&
          f.controller->fault_reason().empty() && f.triggers() == before,
          "speed above the scale-dependent tolerance or missing stop flag must defer without fault");
        require(f.motors[0]->writes + f.motors[1]->writes + f.motors[2]->writes == writes,
          "initial-motion deferral must not partially configure the group");
        f.motors[axis]->put<std::int32_t>(0x606C, 0);
        f.motors[axis]->put<std::int8_t>(0x2707, 1);
        require(f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000})),
          "the same target must be accepted once fresh standstill feedback arrives");
        f.finish();
      }
    }
  }
  {
    ProfileFixture f;
    const auto before = f.triggers();
    f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000}));
    f.motors[2]->put<std::int32_t>(0x606C, f.stationary_velocity_limit(2) + 1);
    f.expect_fault_without_trigger(before);
  }
  for (unsigned configured = 0; configured <= 3; ++configured) {
    for (const bool moving : {false, true}) {
      ProfileFixture f;
      const auto before = f.triggers();
      f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000}));
      f.tick();  // Freeze the common starting positions.
      for (unsigned i = 0; i < configured; ++i) {f.tick();}
      if (moving) {
        f.motors[0]->put<std::int32_t>(0x606C, f.stationary_velocity_limit(0) + 1);
      } else {
        f.motors[0]->put<std::int32_t>(0x6064, f.motion.zero_counts[0] + f.scales[0] / 18000);
      }
      f.expect_fault_without_trigger(before);
    }
  }
}

void synchronized_profiles_allow_bounded_stopped_velocity_jitter()
{
  for (const double speed_rpm : {0.6, 0.001}) {
    for (const bool boundary : {false, true}) {
      ProfileFixture f(speed_rpm);
      const auto before = f.triggers();
      for (unsigned i = 0; i < 3; ++i) {
        // 0.0114 RPM reproduces observed parked feedback; the slow case exercises one LSB.
        const auto jitter = boundary ? f.stationary_velocity_limit(i) :
          (speed_rpm == 0.6 ? static_cast<std::int32_t>(std::llround(0.0114 * f.scales[i] / 60)) : 1);
        f.motors[i]->put<std::int32_t>(0x606C, i == 1 ? -jitter : jitter);
      }
      require(f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000})),
        "stopped-flag feedback inside the supplied velocity tolerance must queue normally");
      f.finish();
      for (unsigned i = 0; i < 3; ++i) {
        require(f.motors[i]->triggers == before[i] + 1 &&
          f.motors[i]->get<std::int32_t>(0x6064) == f.motion.zero_counts[i],
          "bounded speed noise without position drift must survive profile setup and final guards");
      }
    }
  }
}

void synchronized_profile_failure_and_cancellation_never_trigger_partial_groups()
{
  for (unsigned axis = 0; axis < 3; ++axis) {
    ProfileFixture f;
    const auto before = f.triggers();
    f.motors[axis]->fail_profile = true;
    f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000}));
    f.expect_fault_without_trigger(before);
  }
  for (unsigned ticks = 0; ticks <= 4; ++ticks) {
    ProfileFixture f;
    const auto before = f.triggers();
    f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000}));
    for (unsigned i = 0; i < ticks; ++i) {f.tick();}
    require(f.triggers() == before, "cancellation setup must remain before group trigger");
    f.controller->halt_planar_position();
    for (unsigned i = 0; i < 8; ++i) {f.tick();}
    require(f.triggers() == before && !f.controller->position_command_pending() &&
      !f.controller->planar_halt_pending() && !f.power_off,
      "cancelling any configuration phase must not replay staged targets");
    f.controller->set_target_pose(f.pose({6000, -24000, 3000}));
    f.finish();
    f.expect_defaults();
  }
}

void profile_standstill_read_failure_faults_both_target_setters()
{
  for (const bool scaled : {false, true}) {
    ProfileFixture f;
    if (scaled) {
      f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000}));
      f.finish();
    }
    const auto before = f.triggers();
    f.motors[1]->fail_standstill_read = true;
    bool failed = false;
    try {
      if (scaled) {f.controller->set_target_pose(f.pose({6000, -24000, 3000}));}
      else {f.controller->set_synchronized_target_pose(f.pose({6000, -24000, 3000}));}
    } catch (const std::runtime_error &) {failed = true;}
    require(failed && f.power_off && !f.controller->fault_reason().empty() &&
      f.triggers() == before, "standstill read failure in either setter must fault and power off without trigger");
  }
}

void synchronized_profile_handshake_deadline_prevents_late_trigger()
{
  for (unsigned configured = 0; configured <= 3; ++configured) {
    ProfileFixture f;
    const auto before = f.triggers();
    f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000}));
    f.tick();
    for (unsigned i = 0; i < configured; ++i) {f.tick();}
    f.now += 1001ms;
    f.expect_fault_without_trigger(before);
    require(f.controller->fault_reason().find("timed out") != std::string::npos,
      "the one-second group deadline must include PP profile configuration");
  }
}

void synchronized_move_halt_restores_profiles_for_ordinary_and_home_targets()
{
  for (const bool home : {false, true}) {
    ProfileFixture f;
    f.controller->set_synchronized_target_pose(f.pose({12000, -48000, 9000}));
    f.finish();
    for (unsigned i = 0; i < 3; ++i) {
      const auto target = f.motors[i]->get<std::int32_t>(0x607A);
      f.motors[i]->put<std::int32_t>(0x6064, (target + f.motion.zero_counts[i]) / 2);
    }
    f.motors[0]->put<std::int32_t>(0x606C, 120);
    f.motors[0]->put<std::int8_t>(0x2707, 0);
    f.tick();
    bool rejected = false;
    try {f.controller->set_target_pose(f.pose({6000, -24000, 3000}));}
    catch (const std::invalid_argument &) {rejected = true;}
    require(rejected && !f.controller->position_command_pending(),
      "an ordinary target must not replace scaled PP limits while the old move is active");
    f.controller->halt_planar_position();
    f.motors[0]->put<std::int32_t>(0x606C, 0);
    f.motors[0]->put<std::int8_t>(0x2707, 1);
    f.tick();
    const auto before = f.triggers();
    if (home) {
      f.controller->return_to_home([&] {
          for (unsigned i = 0; i < 3; ++i) {
            if (f.motors[i]->triggers == before[i]) {continue;}
            f.expect_defaults();
            f.motors[i]->put<std::int32_t>(0x6064, f.motors[i]->get<std::int32_t>(0x607A));
            f.motors[i]->put<std::uint16_t>(0x6041,
              f.motors[i]->get<std::uint16_t>(0x6041) | 0x0400U);
          }
          return true;
        }, 2s);
      const auto actual = f.controller->actual_joints();
      for (unsigned i = 0; i < 3; ++i) {
        require(std::abs(actual[i] - f.motion.home_joints[i]) <= arm::kPi / f.scales[i],
          "return home must retain its exact configured joints within half a pulse");
      }
    } else {
      f.controller->set_target_pose(f.pose({18000, -24000, 3000}));
      f.finish();
    }
    f.expect_defaults();
    for (unsigned i = 0; i < 3; ++i) {
      require(f.motors[i]->triggers == before[i] + 1,
        "the replacement/home endpoint must trigger once after profile restoration");
    }
  }
}

void fixed_goal_scales_profiles_while_live_tracking_keeps_original_limits()
{
  for (const bool fixed : {false, true}) {
    Fixture f(true);
    if (fixed) {f.enable_goal();} else {f.enable_x();}
    f.tick();
    std::array<Profile, 4> defaults{};
    std::array<unsigned, 4> writes{};
    for (unsigned i = 0; i < 4; ++i) {
      defaults[i] = read_profile(f.motors[i]);
      writes[i] = f.motors[i]->profile_writes;
    }
    if (fixed) {
      f.motors[0]->put<std::int32_t>(0x606C, 1201);
      f.goal(f.initial.x + 0.03, f.initial.z + 0.04);
    }
    else {f.target_xz(f.initial.x + 0.03, f.initial.z + 0.04);}
    f.tick(100ms);
    if (fixed) {
      require(!f.controller->position_command_pending() && !f.power_off &&
        f.controller->fault_reason().empty() && f.motors[0]->profile_writes == writes[0],
        "a fixed goal must wait when raw velocity exceeds the standstill tolerance");
      f.motors[0]->put<std::int32_t>(0x606C, 0);
      f.goal(f.initial.x + 0.03, f.initial.z + 0.04);
      f.tick();
    }
    require(f.controller->position_command_pending(), "visual target must queue a complete endpoint");
    f.finish_handshake();
    bool reduced = false;
    for (unsigned i = 0; i < 3; ++i) {
      require(f.motors[i]->profile_writes == writes[i] + (fixed ? 3U : 0U),
        "only fixed visual goals may configure synchronized PP profiles");
      const auto profile = read_profile(f.motors[i]);
      if (fixed) {
        reduced = reduced || profile != defaults[i];
        for (unsigned field = 0; field < 3; ++field) {
          require(profile[field] > 0 && profile[field] <= defaults[i][field],
            "visual synchronized profiles must respect original PP limits");
        }
      } else {
        require(profile == defaults[i], "live tracking must retain its existing PP behavior");
      }
    }
    require(!fixed || reduced, "unequal fixed-goal travel must reduce at least one axis profile");
    require(f.motors[3]->profile_writes == writes[3] && read_profile(f.motors[3]) == defaults[3],
      "planar synchronization must not reconfigure the fourth axis");
  }
}

void fixed_goal_expiry_cancels_profile_setup_before_controller_update()
{
  for (unsigned ticks = 0; ticks <= 4; ++ticks) {
    Fixture f(true);
    f.enable_goal();
    f.tick();
    const auto before = std::array<unsigned, 3>{f.motors[0]->triggers,
      f.motors[1]->triggers, f.motors[2]->triggers};
    f.goal(f.initial.x + 0.03, f.initial.z + 0.04);
    f.tick();
    require(f.controller->position_command_pending(), "fixed goal must queue before expiry scenario");
    // queued, common start frozen, then one/two/all three profiles staged.
    for (unsigned i = 0; i < ticks; ++i) {f.tick(20ms, false);}
    f.tick(350ms, false);  // IMU stays fresh; only the fixed-goal heartbeat expires.
    require(f.halted() && !f.controller->position_command_pending() &&
      !f.power_off && f.controller->fault_reason().empty(),
      "goal expiry must halt profile setup before the controller can trigger staged targets");
    for (unsigned i = 0; i < 3; ++i) {
      require(f.motors[i]->triggers == before[i],
        "a fixed goal that expires in any profile phase must never trigger an axis");
    }
  }
}

void lateral_direction_limits_and_independent_deadband()
{
  Fixture f(true);
  require(!f.tracker->owns_chassis(), "disabled tracking must not own the chassis");
  require(f.tracker->accept_manual_chassis_command(0.1, 0.0, 0.0, f.now),
    "ordinary manual chassis commands must be accepted before tracking");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", true)).successful,
    "enable request must commit");
  require(f.tracker->owns_chassis(), "an enable awaiting the control tick must already own chassis");
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "activation without settled feedback or a target must stay stopped");
  f.tick();
  require(f.tracker->lateral_velocity(f.now) == 0.0, "settled feedback alone cannot move chassis");

  const std::array<std::pair<double, double>, 9> cases{{
    {0.004, 0.008}, {-0.004, -0.008}, {0.5, 0.020}, {-0.5, -0.020},
    {0.002, 0.0}, {-0.002, 0.0}, {0.0, 0.0}, {1e300, 0.020}, {-1e300, -0.020}}};
  for (const auto & [error, expected] : cases) {
    f.tick();
    f.target(f.initial.z, error);
    f.tracker->after_update();
    require(std::abs(f.tracker->lateral_velocity(f.now) - expected) < 1e-12,
      "left/right errors must map to bounded signed vy, with a separate lateral deadband");
    require(!f.controller->position_command_pending(),
      "lateral correction must work with height centered without moving planar joints");
    require(!f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
      "manual neutral heartbeat must not take visual chassis ownership");
    require(std::abs(f.tracker->lateral_velocity(f.now) - expected) < 1e-12,
      "manual neutral heartbeat must preserve the visual lateral command");
  }
}

void lateral_waits_for_halt_and_exit_requires_neutral()
{
  Fixture f(true);
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  f.enable();
  f.tick();
  f.target(f.initial.z, 0.01);
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "chassis must wait for planar standstill during activation");
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  f.tick();
  require(f.tracker->lateral_velocity(f.now) == 0.020,
    "valid horizontal input may move after standstill and baseline acquisition");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
    "visual disable must commit");
  require(f.tracker->owns_chassis(), "pending disable cannot expose the last visual velocity");
  require(f.tracker->lateral_velocity(f.now) == 0.0, "disable must zero lateral motion immediately");
  require(!f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
    "neutral command cannot release ownership before arm handover settles");
  f.tick();
  require(f.tracker->owns_chassis(), "handover still requires a fresh manual neutral");
  require(!f.tracker->accept_manual_chassis_command(0.0, 0.2, 0.0, f.now),
    "a held manual command must not replay after visual exit");
  require(f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
    "post-handover neutral must release manual chassis interlock");
  require(!f.tracker->owns_chassis(), "manual neutral must release chassis ownership without deadlock");
  require(f.tracker->accept_manual_chassis_command(0.0, -0.2, 0.0, f.now),
    "fresh manual motion may resume after neutral");
}

void manual_motion_and_watchdog_latch_both_visual_axes()
{
  const std::array<std::array<double, 3>, 5> commands{{
    {0.1, 0.0, 0.0}, {0.0, -0.1, 0.0}, {0.0, 0.0, 0.1},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    {0.0, std::numeric_limits<double>::infinity(), 0.0}}};
  for (std::size_t kind = 0; kind <= commands.size(); ++kind) {
    Fixture f(true);
    f.enable();
    f.tick();
    f.target(f.initial.z + 0.05, 0.01);
    f.tick(100ms);
    require(f.controller->position_command_pending() && f.tracker->lateral_velocity(f.now) > 0,
      "fixture must have both a staged height move and a lateral command");
    if (kind < commands.size()) {
      const auto & command = commands[kind];
      require(!f.tracker->accept_manual_chassis_command(
          command[0], command[1], command[2], f.now),
        "manual motion or invalid input must interrupt tracking before taking control");
    } else {
      f.tracker->cancel_tracking("manual command stream timed out", f.now);
    }
    require(f.tracker->lateral_velocity(f.now) == 0.0 && f.halted() &&
      !f.controller->position_command_pending(), "interruption must stop both visual axes");
    f.tick();
    f.target(f.initial.z + 0.05, -0.01);
    f.tick();
    require(f.tracker->lateral_velocity(f.now) == 0.0 &&
      !f.controller->position_command_pending(), "fresh targets cannot bypass an interruption latch");
    require(!f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
      "neutral cannot release a still-enabled tracking latch");
    require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
      "operator disable must commit");
    f.tick();
    require(!f.tracker->accept_manual_chassis_command(0.1, 0.0, 0.0, f.now),
      "manual input must still return to neutral after interrupted tracking exits");
    require(f.tracker->accept_manual_chassis_command(0.0, 0.0, 0.0, f.now),
      "neutral must restore manual ownership after interruption");
    f.enable();
    f.tick();
    f.target(f.initial.z, -0.01);
    require(f.tracker->lateral_velocity(f.now) == -0.020,
      "explicit disable and re-enable must permit a newly acquired safe session");
  }
}

void explicit_stop_clears_lateral_and_requires_a_new_capture()
{
  Fixture f(true);
  f.enable();
  f.tick();
  const auto old = f.message(f.initial.z, 0.01);
  f.tracker->observe_target(old, f.now);
  require(f.tracker->lateral_velocity(f.now) > 0, "fixture must command lateral motion");
  f.tracker->stop_tracking(f.now);
  require(f.tracker->lateral_velocity(f.now) == 0.0, "explicit stop must clear lateral velocity");
  f.tracker->observe_target(old, f.now);
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "a capture predating stop cannot restart lateral movement");
  f.tick();
  f.target(f.initial.z, -0.01);
  require(f.tracker->lateral_velocity(f.now) < 0,
    "after standstill an explicitly newer capture may resume an ordinary stop");
}

void lateral_motion_continues_until_disabled_and_recovers_after_loss()
{
  Fixture f(true);
  f.enable();
  f.tick();
  f.target(f.initial.z, 0.01);
  require(f.tracker->lateral_velocity(f.now) > 0, "fixture must begin lateral motion");
  for (unsigned i = 0; i < 600; ++i) {
    f.tick(100ms);
    f.target(f.initial.z, 0.01);
    require(f.tracker->lateral_velocity(f.now) > 0,
      "fresh targets must sustain lateral motion throughout a sixty-second session");
  }

  f.tick(20000ms);
  require(f.tracker->lateral_velocity(f.now) == 0.0 && f.halted(),
    "missing targets must still halt both visual axes without a session time limit");
  f.tick();
  f.target(f.initial.z, -0.01);
  require(f.tracker->lateral_velocity(f.now) < 0,
    "a fresh capture after a long target loss must resume without a mode toggle");
  for (unsigned i = 0; i < 150; ++i) {
    f.tick(100ms);
    f.target(f.initial.z, -0.01);
    require(f.tracker->lateral_velocity(f.now) < 0,
      "resumed tracking must also continue beyond ten seconds");
  }

  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
    "operator disable must commit after a long tracking session");
  f.tick();
  require(f.tracker->lateral_velocity(f.now) == 0.0 && f.halted(),
    "disabling tracking must halt both visual axes immediately");
  f.tick();
  f.target(f.initial.z + 0.05, 0.01);
  f.tick(100ms);
  require(f.tracker->lateral_velocity(f.now) == 0.0 &&
    !f.controller->position_command_pending(), "new images cannot restart disabled tracking");
  f.enable();
  f.tick();
  f.target(f.initial.z, 0.01);
  require(f.tracker->lateral_velocity(f.now) > 0,
    "a new activation and fresh target must resume tracking");
}

void defaults_and_startup_guard()
{
  Fixture f;
  require(!f.node->get_parameter("arm_tracking_enabled").as_bool(), "tracking defaults off");
  require(!f.node->get_parameter("arm_tracking_x_enabled").as_bool(), "X tracking defaults off");
  require(!f.node->get_parameter("arm_tracking_goal_enabled").as_bool(), "fixed goal defaults off");
  f.target(f.initial.z + 0.05);
  f.tick();
  require(!f.controller->position_command_pending(), "disabled vision must not move");
  for (const char * parameter : {"arm_tracking_enabled", "arm_tracking_x_enabled",
      "arm_tracking_goal_enabled"})
  {
    auto startup = std::make_shared<rclcpp::Node>("tracking_bad_startup",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter(parameter, true)}));
    bool rejected = false;
    try {chassis::declare_arm_tracking_parameter(*startup);} catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "startup override must not enable any tracking mode");
  }
}

void imu_loss_and_heading_fault_latch_fixed_goal_and_lateral_motion()
{
  for (unsigned stage = 0; stage < 3; ++stage) {
    Fixture f(true);
    f.enable_goal();
    f.tick();
    if (stage != 0) {
      f.goal(f.initial.x + 0.04, f.initial.z);
      f.tick();
      require(f.controller->position_command_pending(), "fresh IMU allows a fixed endpoint");
      if (stage == 2) {
        f.finish_handshake();
        require(!f.halted(), "arm must have accepted the endpoint before the IMU fault");
      }
    } else {
      f.target(f.initial.z, 0.01);
      require(f.tracker->lateral_velocity(f.now) > 0.0, "fresh IMU allows visual alignment");
    }
    if (stage == 2) {
      f.auto_imu = false;
      f.advance_time(20ms);
      sensor_msgs::msg::Imu invalid;
      invalid.header.frame_id = "imu_link";
      invalid.header.stamp = f.node->now();
      invalid.orientation.w = std::numeric_limits<double>::quiet_NaN();
      f.tracker->observe_imu(invalid, f.now);
      f.tick(0ms);
      f.auto_imu = true;
    } else {
      f.imu_yaw = 9.0 * arm::kPi / 180.0;
      f.tick();
    }
    require(f.halted() && !f.controller->position_command_pending() &&
      f.tracker->lateral_velocity(f.now) == 0.0, "IMU fault halts arm and lateral motion");
    f.imu_yaw = 0.0;
    for (unsigned i = 0; i < 4; ++i) {f.tick();}
    if (stage != 0) {f.goal(f.initial.x + 0.04, f.initial.z);}
    else {f.target(f.initial.z, 0.01);}
    f.tick();
    require(f.tracker->lateral_velocity(f.now) == 0.0 &&
      !f.controller->position_command_pending(), "fresh IMU and target cannot revive faulted docking");
  }
  Fixture f(true);
  f.auto_imu = false;
  f.tick(201ms);
  f.enable_goal();
  f.tick();
  f.goal(f.initial.x + 0.04, f.initial.z);
  f.tick();
  require(!f.controller->position_command_pending(), "stale IMU prevents docking activation");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
    "operator can end the faulted session");
  f.tick();
  f.auto_imu = true;
  for (unsigned i = 0; i < 3; ++i) {f.tick();}
  f.enable_goal();
  f.tick();
  f.target(f.initial.z, 0.01);
  require(f.tracker->lateral_velocity(f.now) > 0.0,
    "new operator session after three fresh IMU samples can align again");
  f.auto_imu = false;
  f.tick(201ms);
  require(f.halted() && f.tracker->lateral_velocity(f.now) == 0.0,
    "IMU timeout stops lateral motion independently of image freshness");
}

void x_mode_changes_require_disabled_tracking_or_atomic_disable()
{
  Fixture f(true);
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "X tracking can be configured while disabled");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", false)).successful,
    "disabled X tracking configuration can be cleared");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", true),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "X tracking and visual ownership can be enabled atomically");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", false)).successful,
    "pending activation must already protect the selected X mode");
  f.tick();
  require(!f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "an enabled session cannot change X mode by repeating enable");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "writing the unchanged X mode is harmless");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("arm_tracking_enabled", false)}).successful,
    "an atomic disable may also clear X tracking");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "X reconfiguration must wait for a pending disable to reach the hardware owner");
  f.tick();
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", true)).successful,
    "X tracking may be configured again after disable is applied");
  const std::vector<double> manual{f.initial.x, f.initial.z + 0.01, f.initial.pitch};
  require(!f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("yiyou_arm_target", manual)}).successful,
    "X mode changes cannot smuggle a simultaneous manual target through arbitration");
}

void depth_x_moves_both_directions_and_keeps_z_and_pitch()
{
  for (int direction : {-1, 1}) {
    Fixture f(true);
    f.measure({0.35, 0.35, 0.0});
    f.enable_x();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    // The absolute X target is not restricted to a 15 cm activation window.
    const double x = baseline.x + direction * 0.20;
    f.target_xz(x, baseline.z);
    f.tick(100ms);
    require(f.controller->position_command_pending(), "pure X error must queue the full target");
    f.target_xz(x, baseline.z);
    f.finish_handshake();
    const auto next = f.commanded_pose();
    require(std::abs(direction * (next.x - baseline.x) - 0.20) < 0.000002,
      "pure X tracking must submit the complete target in either direction");
    require(std::abs(next.z - baseline.z) < 0.000002 &&
      std::abs(next.pitch - baseline.pitch) < 0.000002,
      "pure X tracking must preserve height and activation pitch");
  }
}

void depth_diagonal_targets_are_complete_at_any_update_interval()
{
  for (const auto elapsed : {20ms, 100ms, 240ms}) {
    Fixture f(true);
    f.measure({0.35, 0.35, 0.0});
    f.enable_x();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    f.target_xz(baseline.x + 0.03, baseline.z + 0.04);
    f.tick(elapsed);
    require(f.controller->position_command_pending(), "a diagonal target must queue in full");
    f.target_xz(baseline.x + 0.03, baseline.z + 0.04);
    f.finish_handshake();
    const auto next = f.commanded_pose();
    require(std::abs(next.x - baseline.x - 0.03) < 0.000002 &&
      std::abs(next.z - baseline.z - 0.04) < 0.000002 &&
      std::abs(next.pitch - baseline.pitch) < 0.000002,
      "elapsed time must not scale a valid X/Z endpoint or change activation pitch");
  }
  Fixture close(true);
  close.measure({0.35, 0.35, 0.0});
  close.enable_x();
  close.tick();
  const auto baseline = close.controller->actual_pose();
  close.target_xz(baseline.x + 0.0015, baseline.z + 0.002);
  close.tick();
  close.finish_handshake();
  const auto next = close.commanded_pose();
  require(std::abs(next.x - baseline.x - 0.0015) < 0.000002 &&
    std::abs(next.z - baseline.z - 0.002) < 0.000002,
    "a close diagonal target must be submitted exactly without overshoot");
}

void latest_depth_target_waits_only_for_the_previous_handshake()
{
  Fixture f(true);
  f.measure({0.35, 0.35, 0.0});
  f.enable_x();
  f.tick();
  const auto baseline = f.controller->actual_pose();
  f.target_xz(baseline.x + 0.05, baseline.z);
  f.tick();
  const auto triggers = f.motors[0]->triggers;
  f.target_xz(baseline.x + 0.06, baseline.z);
  f.tick();
  f.target_xz(baseline.x + 0.08, baseline.z);
  require(f.controller->position_command_pending() && f.motors[0]->triggers == triggers,
    "new captures must not overwrite an in-flight PP handshake");
  f.finish_handshake();
  require(std::abs(f.commanded_pose().x - baseline.x - 0.05) < 0.000002,
    "the initial complete endpoint must finish its handshake first");
  f.target_xz(baseline.x + 0.08, baseline.z);
  f.tracker->after_update();
  require(f.controller->position_command_pending(),
    "the latest target must queue without waiting for the previous physical arrival");
  f.finish_handshake();
  require(std::abs(f.commanded_pose().x - baseline.x - 0.08) < 0.000002 &&
    std::abs(f.controller->actual_pose().x - baseline.x) < 0.000002,
    "frozen feedback must not prevent replacing the endpoint with the newest capture");
  require(f.motors[0]->triggers == triggers + 2,
    "only the initial and latest endpoints may trigger; intermediate captures must be superseded");
}

void depth_x_reversal_halts_before_accepting_a_new_capture()
{
  for (bool accepted : {false, true}) {
    Fixture f(true);
    f.enable_x();
    f.tick();
    f.target_xz(f.initial.x + 0.05, f.initial.z);
    f.tick(100ms);
    if (accepted) {
      f.target_xz(f.initial.x + 0.05, f.initial.z);
      f.finish_handshake();
    }
    f.target_xz(f.initial.x - 0.05, f.initial.z);
    require(f.halted() && !f.controller->position_command_pending() &&
      f.tracker->lateral_velocity(f.now) == 0.0,
      "an X reversal must cancel an unaccepted or accepted target and lateral motion");
    f.tick();
    f.target_xz(f.initial.x - 0.05, f.initial.z);
    f.tick(100ms);
    f.target_xz(f.initial.x - 0.05, f.initial.z);
    f.finish_handshake();
    require(std::abs(f.commanded_pose().x - f.initial.x + 0.05) < 0.000002,
      "a fresh post-stop capture may submit the complete reverse X target");
  }
}

void unreachable_depth_target_is_rejected_without_partial_motion()
{
  for (double x : {0.6858, std::sqrt(0.7 * 0.7 - 0.14 * 0.14)}) {
    Fixture f(true);
    f.measure({x, 0.14, 0.0});
    f.enable_x();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    const auto triggers = f.motors[0]->triggers;
    f.target_xz(baseline.x + 0.05, baseline.z + 0.05);
    f.tick();
    require(f.halted() && !f.controller->position_command_pending() &&
      f.motors[0]->triggers == triggers && f.tracker->lateral_velocity(f.now) == 0.0,
      "an unreachable complete endpoint must halt without substituting a smaller move");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "unreachable vision input must remain an input rejection, not a hardware fault");
  }
}

void depth_tracking_stop_expiry_and_handover_halt_all_visual_motion()
{
  for (unsigned interruption = 0; interruption < 4; ++interruption) {
    Fixture f(true);
    f.enable_x();
    f.tick();
    f.target_xz(f.initial.x + 0.05, f.initial.z + 0.05);
    f.tick(100ms);
    require(f.controller->position_command_pending() && f.tracker->lateral_velocity(f.now) > 0,
      "fixture must command depth-based planar motion and chassis lateral motion");
    if (interruption == 0) {
      f.tracker->stop_tracking(f.now);
    } else if (interruption == 1) {
      f.tick(400ms);
    } else if (interruption == 2) {
      require(!f.tracker->accept_manual_chassis_command(0.1, 0.0, 0.0, f.now),
        "manual takeover must stop depth tracking before accepting manual velocity");
    } else {
      require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
        "operator mode exit must commit");
      f.tick();
    }
    require(f.halted() && !f.controller->position_command_pending() &&
      f.tracker->lateral_velocity(f.now) == 0.0,
      "stop, stale target, manual takeover and mode exit must halt X/Z and chassis together");
  }
}

void goal_mode_requires_x_and_atomic_mode_transitions()
{
  Fixture f(true);
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_goal_enabled", true)).successful,
    "fixed goal can be configured while tracking is disabled");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", true)).successful,
    "fixed goal activation requires X tracking in the final parameter state");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_x_enabled", true),
      rclcpp::Parameter("arm_tracking_goal_enabled", true),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "fixed goal and X tracking may activate together");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_goal_enabled", false)).successful,
    "pending activation must protect the fixed goal mode");
  f.tick();
  require(!f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_goal_enabled", false),
      rclcpp::Parameter("arm_tracking_enabled", true)}).successful,
    "repeating enable cannot change the active fixed goal mode");
  require(!f.node->set_parameter(rclcpp::Parameter("arm_tracking_x_enabled", false)).successful,
    "an active fixed goal cannot drop its X tracking prerequisite");
  require(f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_enabled", false),
      rclcpp::Parameter("arm_tracking_x_enabled", false),
      rclcpp::Parameter("arm_tracking_goal_enabled", false)}).successful,
    "an atomic disable may clear both tracking modes");
  f.tick();
  f.enable();
  f.tick();
  f.goal(f.initial.x + 0.05, f.initial.z);
  f.tick(100ms);
  require(!f.controller->position_command_pending(),
    "the dedicated goal topic must not affect a legacy tracking session");
}

void goal_submission_waits_for_settled_activation_feedback()
{
  Fixture f(true);
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  f.enable_goal();
  f.goal(f.initial.x + 0.05, f.initial.z);
  require(f.controller->planar_halt_pending() && !f.controller->position_command_pending(),
    "a goal arriving before standstill and baseline acquisition must not queue motion");
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  f.tick();
  f.goal(f.initial.x + 0.05, f.initial.z);
  f.tick(100ms);
  require(f.controller->position_command_pending(),
    "a fresh goal retry may proceed after settled activation feedback exists");
}

void goal_alignment_holds_arm_and_committed_goal_ignores_live_packets()
{
  Fixture f(true);
  f.enable_goal();
  f.tick();
  auto queued_live = f.message(f.initial.z + 0.05, 0.02);
  queued_live.point.x = f.initial.x + 0.05;
  f.tracker->observe_target(queued_live, f.now);
  f.tick(100ms);
  require(!f.controller->position_command_pending() && f.halted() &&
    f.tracker->lateral_velocity(f.now) > 0.0,
    "before goal commitment, live vision may align Y but must hold arm X/Z");
  const double goal_x = f.initial.x + 0.08, goal_z = f.initial.z + 0.03;
  f.goal(goal_x, goal_z);
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "committing a fixed goal must immediately clear chassis lateral motion");
  f.tick();
  require(f.controller->position_command_pending(), "the committed goal must queue its complete endpoint");
  f.tracker->observe_target(queued_live, f.now);
  f.target_xz(f.initial.x - 0.1, f.initial.z - 0.1, -0.02);
  f.goal(goal_x, goal_z);
  f.finish_handshake();
  const auto next = f.commanded_pose();
  require(next.x > f.initial.x && next.z > f.initial.z &&
    f.tracker->lateral_velocity(f.now) == 0.0,
    "queued and fresh live packets must not cancel, redirect or restore lateral motion after a goal");
}

void fixed_goal_heartbeats_keep_one_command_beyond_ten_seconds()
{
  Fixture f(true);
  f.enable_goal();
  f.tick();
  const double goal_x = f.initial.x + 0.30, goal_z = f.initial.z;
  const auto triggers = f.motors[0]->triggers;
  f.goal(goal_x, goal_z);
  for (unsigned i = 0; i < 600; ++i) {
    f.tick(20ms, false);
    // The camera may be occluded; only the heartbeat changes, never the destination.
    f.goal(goal_x, goal_z);
    f.tracker->after_update();
    require(f.tracker->lateral_velocity(f.now) == 0.0,
      "fixed goal execution must never command chassis lateral motion");
  }
  require(f.motors[0]->triggers == triggers + 1 && !f.halted() &&
    !f.controller->position_command_pending(),
    "fresh goal heartbeats must preserve one full PP command without a motion duration limit");
  require(std::abs(f.commanded_pose().x - goal_x) < 0.000002 &&
    std::abs(f.controller->actual_pose().x - f.initial.x) < 0.000002,
    "unchanged feedback must neither shrink the endpoint nor cause repeated triggers");
  require(f.controller->fault_reason().empty() && !f.power_off,
    "long-running fresh goal commands must not synthesize a hardware fault");
}

void fixed_goals_submit_the_full_endpoint_once()
{
  for (int z_direction : {-1, 0, 1}) {
    for (const auto elapsed : {20ms, 100ms, 240ms}) {
      Fixture f(true);
      f.measure({0.35, 0.35, 0.0});
      f.enable_goal();
      f.tick();
      const auto baseline = f.controller->actual_pose();
      const double goal_x = baseline.x + 0.03;
      const double goal_z = baseline.z + z_direction * 0.04;
      f.goal(goal_x, goal_z);
      f.tick(elapsed);
      require(f.controller->position_command_pending(), "fixed goal must queue the full endpoint");
      f.goal(goal_x, goal_z);
      f.finish_handshake();
      const auto target = f.commanded_pose();
      require(std::abs(target.x - goal_x) < 0.000002 &&
        std::abs(target.z - goal_z) < 0.000002 &&
        std::abs(target.pitch - baseline.pitch) < 0.000002,
        "fixed-goal submission must preserve the full endpoint and activation pitch");
      const auto triggers = f.motors[0]->triggers;
      for (unsigned i = 0; i < 3; ++i) {
        f.tick(100ms, false);
        f.goal(goal_x, goal_z);
        f.tracker->after_update();
        require(!f.controller->position_command_pending() && f.motors[0]->triggers == triggers,
          "same-goal heartbeats must not retrigger an accepted PP endpoint");
      }
    }
  }
}

void invalid_full_goal_is_rejected_before_motion_and_latches()
{
  for (unsigned invalid = 0; invalid < 4; ++invalid) {
    Fixture f(true);
    f.enable_goal();
    f.tick();
    auto point = f.goal_message(f.initial.x + 0.05, f.initial.z);
    if (invalid == 0) {point.point.x = 10.0;}
    if (invalid == 1) {point.point.z = f.initial.z + 0.16;}
    if (invalid == 2) {point.point.y = 0.01;}
    if (invalid == 3) {point.header.frame_id = "camera";}
    const auto triggers = f.motors[0]->triggers;
    f.tracker->observe_goal(point, f.now);
    f.tick();
    f.goal(f.initial.x + 0.05, f.initial.z);
    f.tick(100ms);
    require(f.halted() && !f.controller->position_command_pending() &&
      f.motors[0]->triggers == triggers && f.tracker->lateral_velocity(f.now) == 0.0,
      "an invalid full goal must latch rejection before any reduced step or later command can move");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "goal input rejection must not be reported as a hardware fault");
  }
}

void endpoint_completion_waits_for_locked_x_and_pitch()
{
  for (bool fixed_goal : {false, true}) {
    for (bool pitch_lags : {false, true}) {
      Fixture f(true);
      f.measure({0.35, 0.35, 0.0});
      if (fixed_goal) {f.enable_goal();} else {f.enable();}
      f.tick();
      const auto baseline = f.controller->actual_pose();
      const double x = baseline.x + (fixed_goal ? 0.03 : 0.0);
      const double z = baseline.z + 0.04;
      if (fixed_goal) {f.goal(x, z);} else {f.target(z);}
      f.tick();
      f.finish_handshake();
      const auto endpoint_joints = f.commanded_joints();
      const auto endpoint = f.commanded_pose();
      const auto triggers = f.motors[0]->triggers;
      if (pitch_lags) {
        auto lagging = endpoint_joints;
        lagging[2] -= 0.1;
        f.measure_joints(lagging);
      } else {
        f.measure({endpoint.x + 0.01, endpoint.z, endpoint.pitch});
      }
      f.tick(20ms, false);
      if (fixed_goal) {f.goal(x, z);} else {f.target(z);}
      f.tracker->after_update();
      require(!f.halted() && !f.controller->planar_halt_pending() &&
        !f.controller->position_command_pending() && f.motors[0]->triggers == triggers,
        "height or X/Z arrival must not halt a full target before locked X and pitch arrive");
      f.measure_joints(endpoint_joints);
      f.tick();
      require(f.halted() && !f.controller->position_command_pending(),
        "the complete endpoint may halt after X, Z and wrapped pitch are within tolerance");
    }
  }
}

void goal_expiry_stop_change_completion_and_replays_require_a_new_session()
{
  for (unsigned terminal = 0; terminal < 6; ++terminal) {
    Fixture f(true);
    f.enable_goal();
    f.tick();
    const double goal_x = f.initial.x + 0.02;
    const auto original = f.goal_message(goal_x, f.initial.z);
    f.tracker->observe_goal(original, f.now);
    f.tick();
    require(f.controller->position_command_pending(), "a valid goal must initially queue motion");
    if (terminal == 0) {
      f.tick(260ms);
    } else if (terminal == 1) {
      f.tracker->stop_tracking(f.now);
    } else if (terminal == 2) {
      f.goal(goal_x + 0.01, f.initial.z);
    } else if (terminal == 3) {
      f.measure({goal_x, f.initial.z, f.initial.pitch});
      f.tick();
    } else if (terminal == 4) {
      f.tracker->observe_goal(original, f.now);
    } else {
      auto stale = f.goal_message(goal_x, f.initial.z);
      stale.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.3);
      f.tracker->observe_goal(stale, f.now);
    }
    const auto triggers = f.motors[0]->triggers;
    for (unsigned i = 0; i < 3; ++i) {
      f.tick();
      f.goal(goal_x, f.initial.z);
      f.target_xz(goal_x, f.initial.z, 0.02);
    }
    require(f.halted() && !f.controller->position_command_pending() &&
      f.motors[0]->triggers == triggers && f.tracker->lateral_velocity(f.now) == 0.0,
      "terminal fixed-goal states must reject fresh-stamped cached goals and live images");
    require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
      "operator disable must end the completed or cancelled goal session");
    f.tick();
    f.enable_goal();
    f.tick();
    const auto baseline = f.controller->actual_pose();
    f.goal(baseline.x + 0.01, baseline.z);
    f.tick(100ms);
    require(f.controller->position_command_pending(),
      "a new enable session may accept a different fixed goal");
  }
}

void live_targets_keep_activation_x_and_pitch_without_arrival_waits()
{
  for (int direction : {-1, 1}) {
    for (bool long_idle : {false, true}) {
      Fixture f(true);
      f.enable();
      f.tick(long_idle ? 1000ms : 20ms);
      f.target(f.initial.z + direction * 0.05);
      f.tick();
      require(f.controller->position_command_pending(), "fresh target must queue in full");
      f.finish_handshake();
      const auto first = f.commanded_pose();
      require(std::abs(first.x - f.initial.x) < 0.000002 &&
        std::abs(first.pitch - f.initial.pitch) < 0.000002 &&
        std::abs(first.z - f.initial.z - direction * 0.05) < 0.000002,
        "Z-only tracking must submit full height and retain activation X/pitch after any idle");
      f.target(f.initial.z + direction * 0.1);
      f.tick();
      require(f.controller->position_command_pending(),
        "a different same-direction target must not wait for measured arrival");
      f.finish_handshake();
      require(std::abs(f.commanded_pose().z - f.initial.z - direction * 0.1) < 0.000002 &&
        std::abs(f.controller->actual_pose().z - f.initial.z) < 0.000002,
        "the full replacement height must reach the drive despite unchanged actual feedback");
      require(f.motors[0]->get<std::uint32_t>(0x6081) == 36000 &&
        f.motors[0]->get<std::uint32_t>(0x6083) == 60000 &&
        f.motors[0]->get<std::uint32_t>(0x6084) == 60000,
        "visual targets must retain the configured motor PP speed and acceleration");
    }
  }
}

void deadband_and_activation_standstill_still_guard_targets()
{
  for (bool waiting_for_halt : {false, true}) {
    Fixture f(true);
    if (waiting_for_halt) {
      f.motors[0]->put<std::int8_t>(0x2707, 0);
      f.motors[0]->put<std::int32_t>(0x606C, 6000);
    }
    f.enable();
    for (unsigned i = 0; i < 10; ++i) {
      f.tick(100ms);
      f.target(f.initial.z + (waiting_for_halt ? 0.05 : 0.001));
      f.tracker->after_update();
      require(!f.controller->position_command_pending(),
        "deadband and pending halt must not queue a target");
    }
    if (waiting_for_halt) {
      f.motors[0]->put<std::int8_t>(0x2707, 1);
      f.motors[0]->put<std::int32_t>(0x606C, 0);
      f.tick(20ms, false);
      require(!f.controller->planar_halt_pending(), "feedback must confirm standstill");
    }
    f.tick(20ms, false);
    f.target(f.initial.z + 0.05);
    f.tracker->after_update();
    require(f.controller->position_command_pending(), "fresh input must permit the full target");
    f.finish_handshake();
    require(std::abs(f.commanded_pose().z - f.initial.z - 0.05) < 0.000002,
      "a complete target must be submitted after the activation and deadband guards clear");
  }
}

void valid_large_joint_changes_are_not_reduced()
{
  Fixture f(true);
  f.measure_joints({0.5 - 5.0 * arm::kPi / 6.0, -0.02 + 5.0 * arm::kPi / 6.0, -0.48});
  f.enable();
  f.tick();
  const auto actual_joints = f.controller->actual_joints();
  const auto baseline = f.controller->actual_pose();
  auto endpoint = baseline;
  endpoint.z -= 0.005;
  const auto full_joints = f.controller->validate_target_pose(endpoint);
  require(std::abs(full_joints[1] - actual_joints[1]) > 3.0 * arm::kPi / 180.0,
    "fixture must require a legal joint change larger than the removed one-degree bound");
  f.target(endpoint.z);
  f.tick();
  require(f.controller->position_command_pending(), "a legal full IK endpoint must proceed");
  f.finish_handshake();
  const auto commanded = f.commanded_joints();
  for (std::size_t i = 0; i < commanded.size(); ++i) {
    require(std::abs(commanded[i] - full_joints[i]) < 0.000002,
      "the drive must receive the complete validated IK solution without halving");
  }
  const auto target = f.commanded_pose();
  require(std::abs(target.z - endpoint.z) < 0.000002 &&
    std::abs(target.x - baseline.x) < 0.000002 &&
    std::abs(target.pitch - baseline.pitch) < 0.000002,
    "large joint changes must still preserve the requested height and locked X/pitch");
}

void valid_activation_pose_corrections_are_not_step_limited()
{
  for (bool pitch_drift : {false, true}) {
    Fixture f(true);
    f.enable();
    auto drifted = f.initial;
    if (pitch_drift) {drifted.pitch += 0.05;} else {drifted.x += 0.02;}
    f.measure(drifted);
    f.tick(20ms, false);
    f.target(f.initial.z + 0.05);
    f.tick();
    require(f.controller->position_command_pending(),
      "a legal correction of locked X or pitch must not hit an artificial joint-step limit");
    f.finish_handshake();
    const auto target = f.commanded_pose();
    require(std::abs(target.x - f.initial.x) < 0.000002 &&
      std::abs(target.z - f.initial.z - 0.05) < 0.000002 &&
      std::abs(target.pitch - f.initial.pitch) < 0.000002,
      "the full target must restore activation X/pitch while moving to the requested height");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "a valid complete correction must not latch tracking or hardware faults");
  }
}

void expiry_precedes_handshake_and_new_capture_is_required()
{
  Fixture f;
  f.start_target();
  require(f.tracker->lateral_velocity(f.now) > 0, "fixture must also command lateral motion");
  f.tick(20ms, false);
  const auto triggers = f.motors[0]->triggers;
  const auto old = f.message(f.initial.z + 0.05);
  f.tick(400ms);
  require(f.halted() && !f.controller->position_command_pending(), "timeout must cancel staging");
  require(f.tracker->lateral_velocity(f.now) == 0.0, "target timeout must stop lateral motion");
  require(f.motors[0]->triggers == triggers, "expired target must never trigger");
  f.tracker->observe_target(old, f.now);
  f.tick();
  require(!f.controller->position_command_pending(), "pre-stop capture must not restart motion");
  f.target(f.initial.z + 0.05);
  f.tick(100ms);
  require(f.controller->position_command_pending(), "new capture may resume after standstill");
}

void malformed_stale_and_replayed_frames_halt()
{
  for (unsigned kind = 0; kind < 9; ++kind) {
    Fixture f;
    f.start_target();
    require(f.tracker->lateral_velocity(f.now) > 0, "fixture must also command lateral motion");
    auto message = f.message(f.initial.z + 0.05);
    switch (kind) {
      case 0: message.header.frame_id = "camera"; break;
      case 1: message.point.z = std::numeric_limits<double>::quiet_NaN(); break;
      case 2: message.point.x = std::numeric_limits<double>::infinity(); break;
      case 3: message.header.stamp = builtin_interfaces::msg::Time{}; break;
      case 4: message.header.stamp = f.node->now() - rclcpp::Duration(1, 0); break;
      case 5: message.header.stamp = f.node->now() + rclcpp::Duration(1, 0); break;
      case 6:
        f.tracker->observe_target(message, f.now);
        break;  // Deliver exactly the same stamp again below.
      case 7: message.point.z = f.initial.z + 0.151; break;
      case 8: message.point.y = std::numeric_limits<double>::quiet_NaN(); break;
    }
    f.tracker->observe_target(message, f.now);
    require(f.halted() && !f.controller->position_command_pending(),
      "invalid target must cancel pending motion immediately");
    require(f.tracker->lateral_velocity(f.now) == 0.0,
      "invalid target must immediately clear its lateral command as well");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "invalid input must not latch a hardware fault");
  }
}

void delayed_captures_cross_the_activation_barrier()
{
  Fixture f(true);
  f.enable();
  f.target(f.initial.z + 0.05);  // Exactly at enable time is still too early.
  require(!f.controller->position_command_pending(), "enable-time capture cannot start motion");
  for (unsigned frame = 1; frame <= 3; ++frame) {
    f.tick(30ms, false);
    auto message = f.message(f.initial.z + 0.05);
    message.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.08);
    f.tracker->observe_target(message, f.now);
    f.tracker->after_update();
    require(f.controller->position_command_pending() == (frame == 3),
      "80 ms capture latency at 30 ms cadence must recover on the first post-enable capture");
  }
  f.finish_handshake();
  const auto target = f.commanded_pose();
  require(std::abs(target.z - f.initial.z - 0.05) < 0.000002 &&
    std::abs(target.x - f.initial.x) < 0.000002 &&
    std::abs(target.pitch - f.initial.pitch) < 0.000002,
    "delayed capture recovery must submit full height and preserve locked X/pitch");
}

void delayed_captures_recover_after_a_hold_without_replaying_old_input()
{
  // Exercise explicit stop, direction reversal, and an invalid frame during motion.
  for (unsigned reason = 0; reason < 3; ++reason) {
    Fixture f(true);
    f.enable();
    f.tick(1ms, false);
    const auto accepted = f.message(f.initial.z + 0.05);
    f.tracker->observe_target(accepted, f.now);
    f.tick(100ms);
    require(f.controller->position_command_pending(), "fixture must have a staged motion");
    const double next_z = f.initial.z + (reason == 1 ? -0.05 : 0.05);
    auto queued_before_hold = f.message(next_z);
    if (reason == 0) {
      f.tracker->stop_tracking(f.now);
    } else {
      auto trigger = queued_before_hold;
      if (reason == 2) {trigger.point.z = std::numeric_limits<double>::quiet_NaN();}
      f.tracker->observe_target(trigger, f.now);
    }
    require(f.halted() && !f.controller->position_command_pending(),
      "each hold must immediately cancel the staged motion");

    for (unsigned frame = 1; frame <= 3; ++frame) {
      f.tick(30ms, false);
      auto stale = f.message(next_z);
      stale.header.stamp = f.node->now() - rclcpp::Duration(1, 0);
      auto future = f.message(next_z);
      future.header.stamp = f.node->now() + rclcpp::Duration(1, 0);
      for (const auto & rejected : {queued_before_hold, accepted, stale, future}) {
        f.tracker->observe_target(rejected, f.now);
        f.tracker->after_update();
        require(f.halted() && !f.controller->position_command_pending(),
          "queued, duplicate, stale and future captures must never release a hold");
      }
      auto delayed = f.message(next_z);
      delayed.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.08);
      f.tracker->observe_target(delayed, f.now);
      f.tracker->after_update();
      require(f.controller->position_command_pending() == (frame == 3),
        "the first post-hold capture may queue immediately once the standstill guard clears");
    }
    f.tick(20ms);
    require(f.controller->position_command_pending(),
      "the first post-hold capture must recover without moving the capture barrier");
    require(f.controller->fault_reason().empty() && !f.power_off,
      "invalid input and safe recovery must not cause a hardware fault or power cut");
  }
}

void explicit_stop_while_already_held_requires_a_new_capture()
{
  Fixture f(true);
  f.enable();
  f.tick(60ms, false);
  auto before_stop = f.message(f.initial.z + 0.05);
  before_stop.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.02);
  f.tracker->stop_tracking(f.now);
  f.tracker->observe_target(before_stop, f.now);
  f.tick(20ms);
  require(f.halted() && !f.controller->position_command_pending(),
    "an explicit stop while already held must invalidate captures newer than enable but older than stop");
  f.tick(60ms, false);
  auto after_stop = f.message(f.initial.z + 0.05);
  after_stop.header.stamp = f.node->now() - rclcpp::Duration::from_seconds(0.04);
  f.tracker->observe_target(after_stop, f.now);
  f.tracker->after_update();
  require(f.controller->position_command_pending(),
    "a valid delayed capture taken after the explicit stop may resume motion");
}

void fresh_live_targets_keep_one_command_beyond_two_seconds()
{
  Fixture f(true);
  f.enable();
  f.tick();
  f.target(f.initial.z + 0.05);
  f.tick();
  f.finish_handshake();
  const auto triggers = f.motors[0]->triggers;
  f.target(f.initial.z + 0.05);
  for (unsigned i = 0; i < 30; ++i) {
    f.tick(100ms, false);
    f.target(f.initial.z + 0.05);
    f.tracker->after_update();
  }
  require(!f.halted() && !f.controller->position_command_pending() &&
    f.motors[0]->triggers == triggers && f.controller->fault_reason().empty() && !f.power_off,
    "fresh same-target captures must preserve one PP command beyond the removed two-second limit");
  require(std::abs(f.commanded_pose().z - f.initial.z - 0.05) < 0.000002,
    "unchanged actual feedback must not replace the full endpoint with partial moves");
  f.tick(260ms);
  require(f.halted() && !f.controller->position_command_pending(),
    "removing motion-duration limits must preserve stale-input shutdown");
}

void reversal_cancels_the_old_direction_before_restarting()
{
  for (bool accepted : {false, true}) {
    Fixture f;
    f.start_target();
    if (accepted) {f.finish_handshake();}
    f.target(f.initial.z - 0.05);
    require(f.halted() && !f.controller->position_command_pending(),
      "opposite target must cancel the old endpoint immediately, even after acceptance");
    f.tick();
    f.target(f.initial.z - 0.05);
    f.tick(100ms);
    f.finish_handshake();
    const auto next = f.commanded_pose();
    require(std::abs(next.z - f.initial.z + 0.05) < 0.000002,
      "the full reverse height target may proceed only after stopping");
  }
}

void activation_waits_for_actual_stop_and_manual_restore_is_fresh()
{
  Fixture f;
  const auto stale_manual = std::vector<double>{f.initial.x, f.initial.z + 0.02, f.initial.pitch};
  require(f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", stale_manual)).successful,
    "manual target accepted before activation");
  f.tick();
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  f.enable();
  f.target(f.initial.z + 0.04);
  f.tick(100ms);
  require(f.controller->planar_halt_pending() && !f.controller->position_command_pending(),
    "activation cannot move before all axes settle");
  require(!f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", stale_manual)).successful,
    "manual writes must be rejected during visual ownership");
  const arm::Pose settled{f.initial.x + 0.01, f.initial.z + 0.01, 0.03};
  f.measure(settled);
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  f.tick();
  const auto baseline = f.controller->actual_pose();
  f.target(baseline.z + 0.04);
  f.tick(100ms);
  f.finish_handshake();
  const auto commanded = f.commanded_pose();
  require(std::abs(commanded.x - baseline.x) < 0.000002 &&
    std::abs(commanded.pitch - baseline.pitch) < 0.000002,
    "activation must lock post-stop feedback, not pre-stop coordinates");
  const auto mixed = f.node->set_parameters_atomically({
      rclcpp::Parameter("arm_tracking_enabled", false),
      rclcpp::Parameter("yiyou_arm_target", stale_manual)});
  require(!mixed.successful, "mode changes cannot smuggle manual targets through arbitration");
  require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
    "disable must commit");
  require(!f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", stale_manual)).successful,
    "manual writes must wait for ownership restoration");
  f.tick();
  const auto restored = f.node->get_parameter("yiyou_arm_target").as_double_array();
  const auto actual = f.controller->actual_pose();
  require(restored == std::vector<double>({actual.x, actual.z, actual.pitch}),
    "manual parameter baseline must synchronize to measured stopped pose");
  require(!f.controller->position_command_pending() && f.halted(),
    "restoring manual state must not replay an old target");
  auto next = restored;
  next[1] += 0.005;
  require(f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target", next)).successful,
    "manual input must work again after restoration");
  f.tick();
  require(f.controller->position_command_pending(), "fresh manual target resumes normal control");
}

void planar_halt_cancels_each_phase_without_touching_fourth_axis()
{
  for (unsigned ticks = 0; ticks <= 7; ++ticks) {
    Fixture f;
    auto pose = f.initial;
    pose.z += 0.005;
    f.controller->set_target_pose(pose);
    for (unsigned i = 0; i < ticks; ++i) {
      f.now += 20ms;
      f.controller->update(f.now);
    }
    f.controller->set_fourth_axis_jog(1);
    const auto fourth_control = f.motors[3]->get<std::uint16_t>(0x6040);
    const auto fourth_triggers = f.motors[3]->triggers;
    const auto planar_triggers = f.motors[0]->triggers;
    f.controller->halt_planar_position();
    require(f.halted() && !f.controller->position_command_pending(),
      "Halt must cancel queued, staged, triggered and accepted targets");
    require(f.controller->fourth_axis_jog() == 1 &&
      f.motors[3]->get<std::uint16_t>(0x6040) == fourth_control &&
      f.motors[3]->triggers == fourth_triggers,
      "planar Halt must leave fourth-axis state and commands untouched");
    f.now += 20ms;
    f.controller->update(f.now);
    require(!f.controller->planar_halt_pending() &&
      f.motors[0]->triggers == planar_triggers && f.halted(),
      "post-halt feedback must settle without replaying a target");
  }
}

void planar_halt_refreshes_the_ik_seed()
{
  Fixture f;
  auto old_target = f.initial;
  old_target.pitch = -3.0;
  f.controller->set_target_pose(old_target);
  for (unsigned i = 0; i < 7; ++i) {
    f.now += 20ms;
    f.controller->update(f.now);
  }
  require(!f.controller->position_command_pending(), "old target must be accepted");
  auto measured = f.initial;
  measured.pitch = 3.0;
  f.measure(measured);
  f.now += 20ms;
  f.controller->update(f.now);
  bool rejected_old_seed = false;
  try {(void)f.controller->validate_target_pose(measured);} catch (const std::invalid_argument &) {
    rejected_old_seed = true;
  }
  require(rejected_old_seed, "test must distinguish a stale accepted seed from actual feedback");
  f.controller->halt_planar_position();
  f.now += 20ms;
  f.controller->update(f.now);
  (void)f.controller->validate_target_pose(measured);
}

void stop_topic_and_hardware_failure()
{
  Fixture f;
  f.start_target();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(f.node);
  auto publisher = f.node->create_publisher<std_msgs::msg::Empty>("/arm_tracking_stop", 1);
  publisher->publish(std_msgs::msg::Empty{});
  executor.spin_some();
  executor.spin_some();
  require(f.halted() && !f.controller->position_command_pending(), "stop topic must halt now");
  executor.remove_node(f.node);

  Fixture broken;
  broken.motors[0]->fail_halt = true;
  bool threw = false;
  try {broken.controller->halt_planar_position();} catch (const std::exception &) {threw = true;}
  require(threw && broken.power_off && !broken.controller->fault_reason().empty(),
    "Halt I/O failure must retain shared power-off and fault latch");
  broken.motors[0]->fail_halt = false;
}

void shared_tracking_ignores_old_commands_without_stopping_the_new_session()
{
  for (const bool fixed_goal : {false, true}) {
    bool external_allowed = true;
    std::uint64_t generation = 41;
    Fixture f(true, true, [&] {return external_allowed;}, [&] {return generation;});
    f.pace_ticks = fixed_goal;
    chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
      [](Clock::time_point) {return true;});
    const auto enable = [&] {
        if (fixed_goal) {f.enable_goal();} else {f.enable();}
        f.tick();  // Capture/command must postdate the enable barrier.
      };
    const auto message = [&](double z) {
        return fixed_goal ? f.goal_message(f.initial.x, z) : f.message(z, 0.01);
      };
    const auto envelope = [](const geometry_msgs::msg::PointStamped & point, std::uint64_t source) {
        std::ostringstream json;
        json.precision(17);
        json << "{\"generation\":" << source << ",\"stamp_ns\":" <<
          static_cast<std::int64_t>(point.header.stamp.sec) * 1000000000LL +
          point.header.stamp.nanosec << ",\"frame_id\":\"" << point.header.frame_id <<
          "\",\"x\":" << point.point.x << ",\"y\":" << point.point.y <<
          ",\"z\":" << point.point.z << '}';
        std_msgs::msg::String result;
        result.data = json.str();
        return result;
      };
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(f.node);
    auto commands = f.node->create_publisher<std_msgs::msg::String>(
      fixed_goal ? "/car2/arm_tracking_goal_target" : "/car2/arm_tracking_target", 1);
    auto legacy_commands = f.node->create_publisher<geometry_msgs::msg::PointStamped>(
      fixed_goal ? "/arm_tracking_goal_target" : "/arm_tracking_target", 1);
    auto stop = f.node->create_publisher<std_msgs::msg::Empty>("/arm_tracking_stop", 1);
    const auto deliver = [&] {executor.spin_some(); executor.spin_some();};
    enable();
    const auto previous_command = message(f.initial.z + 0.01);
    commands->publish(envelope(previous_command, generation));
    deliver();
    f.tick();
    require(f.controller->position_command_pending(), "previous vision command must be in flight");
    app.stop(f.now);
    ++generation;
    f.tick();
    require(f.tracker->complete_source_handover(), "old vision session must stop and synchronize");
    enable();
    const auto current_command = message(f.initial.z + 0.015);
    commands->publish(envelope(current_command, generation));
    deliver();
    f.tick();
    require(f.controller->position_command_pending(),
      "a fresh envelope from the current generation must start a new target");
    commands->publish(envelope(previous_command, generation));
    stop->publish(std_msgs::msg::Empty{});
    deliver();
    require(f.controller->position_command_pending(),
      "old vision packets and unversioned stop must not cancel a new shared session");
    commands->publish(envelope(current_command, generation));
    deliver();
    require(f.controller->position_command_pending(),
      "duplicate shared packets must be ignored without halting the current target");

    auto refreshed_old = message(f.initial.z - 0.02);
    commands->publish(envelope(refreshed_old, generation - 1));
    refreshed_old.header.frame_id = "invalid_legacy_frame";
    legacy_commands->publish(refreshed_old);
    deliver();
    require(f.controller->position_command_pending(),
      "old-generation envelopes with new stamps and legacy topics must not halt current motion");

    external_allowed = false;
    f.tick();
    auto rejected_source = message(f.initial.z + 0.02);
    rejected_source.header.frame_id = "invalid_disallowed_frame";
    commands->publish(envelope(rejected_source, generation));
    deliver();
    require(f.controller->position_command_pending(),
      "a disabled external source must not change or cancel the current target");
    external_allowed = true;
    f.tick();
    auto malformed = message(f.initial.z + 0.02);
    malformed.header.frame_id = "invalid_current_frame";
    commands->publish(envelope(malformed, generation));
    deliver();
    require(!f.controller->position_command_pending() && f.halted(),
      "a new malformed command from the current shared session must still halt");
    require(!f.power_off && f.controller->fault_reason().empty(),
      "input rejection must not become a hardware fault");

    app.stop(f.now);
    ++generation;
    f.tick();
    require(f.tracker->complete_source_handover(), "invalid input must allow an explicit new session");
    enable();
    auto expiring_command = message(f.initial.z + 0.015);
    commands->publish(envelope(expiring_command, generation));
    deliver();
    f.tick();
    require(f.controller->position_command_pending(), "expiry fixture must first have active motion");
    ++expiring_command.header.stamp.nanosec;
    *f.ros_time_ns += 300000000LL;
    require(rcl_set_ros_time_override(f.node->get_clock()->get_clock_handle(), *f.ros_time_ns) ==
      RCL_RET_OK, "expiry fixture must advance capture age");
    commands->publish(envelope(expiring_command, generation));
    deliver();
    require(f.halted() && !f.controller->position_command_pending(),
      "an expired current-generation command must retain the original freshness stop");
    executor.remove_node(f.node);
  }
}

struct StrokeFixture
{
  bool owner{true};
  std::uint64_t generation{41};
  Fixture f{true, true, [this] {return owner;}, [this] {return generation;}};
  arm::Pose nominal{f.initial.x, f.initial.z + 0.01, f.initial.pitch};
  const std::uint64_t id{831};
  std::string stage;
  std::int64_t settled_ns{};
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goals, corrections;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state;

  StrokeFixture()
  {
    f.pace_ticks = true;
    executor.add_node(f.node);
    goals = f.node->create_publisher<std_msgs::msg::String>("/car2/arm_tracking_goal_target", 1);
    corrections = f.node->create_publisher<std_msgs::msg::String>("/car2/arm_tracking_correction", 1);
    state = f.node->create_subscription<std_msgs::msg::String>(
      "/car2/arm_tracking_stroke_state", 1, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const auto data = YAML::Load(msg->data);
        require(data.size() == 4 && data["generation"].as<unsigned>() == 41 &&
          data["stroke_id"].as<unsigned>() == id, "stroke state must retain its original identity");
        stage = data["stage"].as<std::string>();
        const auto stamp = data["settled_ns"].as<std::int64_t>();
        require(!settled_ns || settled_ns == stamp, "first settled timestamp must remain immutable");
        settled_ns = stamp;
      });
    f.enable_goal();
    f.tick();
  }

  void receive() {executor.spin_some(); executor.spin_some();}

  void send(rclcpp::Publisher<std_msgs::msg::String> & publisher, const std::string & json)
  {
    std_msgs::msg::String msg;
    msg.data = json;
    publisher.publish(msg);
    receive();
  }

  void heartbeat()
  {
    std::ostringstream json;
    json.precision(17);
    json << "{\"generation\":" << generation << ",\"stroke_id\":" << id
         << ",\"stamp_ns\":" << f.node->now().nanoseconds() << ",\"frame_id\":\"arm_shoulder\","
         << "\"x\":" << nominal.x << ",\"y\":0,\"z\":" << nominal.z << "}";
    send(*goals, json.str());
  }

  void correction(double dx, double dz, std::int64_t stamp, std::int64_t barrier,
    std::uint64_t stroke = 831, std::uint64_t source = 41)
  {
    std::ostringstream json;
    json.precision(17);
    json << "{\"generation\":" << source << ",\"stroke_id\":" << stroke
         << ",\"stamp_ns\":" << stamp << ",\"settled_ns\":" << barrier
         << ",\"dx\":" << dx << ",\"dz\":" << dz << "}";
    send(*corrections, json.str());
  }

  arm::Pose settle()
  {
    heartbeat();
    f.tick();
    require(f.controller->position_command_pending(), "nominal stroke must submit its fixed goal");
    f.finish_handshake();
    heartbeat();
    // The measured settled X differs from the requested endpoint within its tolerance.
    f.measure({nominal.x + 0.001, nominal.z, nominal.pitch});
    f.motors[0]->put<std::int8_t>(0x2707, 0);
    f.motors[0]->put<std::int32_t>(0x606C, 6000);
    f.tick();
    require(f.halted(), "nominal stroke must request a physical halt");
    f.tick();
    receive();
    require(stage.empty(), "coasting feedback cannot open the verification window");
    f.motors[0]->put<std::int8_t>(0x2707, 1);
    f.motors[0]->put<std::int32_t>(0x606C, 0);
    f.tick();
    receive();
    const auto measured = f.controller->actual_pose();
    require(stage == "settled" && settled_ns == f.node->now().nanoseconds(),
      "acknowledge the first stationary feedback with its ROS timestamp");
    f.tick();
    heartbeat();
    return measured;
  }
};

void shared_stroke_corrects_once_from_settled_feedback()
{
  StrokeFixture s;
  const auto measured = s.settle();
  auto & f = s.f;
  const auto before = f.motors[0]->triggers;
  const auto stamp = f.node->now().nanoseconds();
  s.correction(.01, 0., stamp, s.settled_ns, s.id + 1);
  s.correction(.01, 0., stamp, s.settled_ns, s.id, s.generation + 1);
  s.correction(.01, 0., stamp, s.settled_ns - 1);
  s.correction(.01, 0., s.settled_ns, s.settled_ns);
  s.correction(.01, 0., stamp + 1, s.settled_ns);
  for (int i = 0; i < 3; ++i) {
    f.tick(100ms);
    s.heartbeat();
  }
  s.correction(.01, 0., stamp, s.settled_ns);
  require(s.stage == "settled" && !f.controller->position_command_pending(),
    "foreign, premature or future observations must not initiate correction");
  s.correction(.01, -.003, f.node->now().nanoseconds(), s.settled_ns);
  require(s.stage == "correcting", "the fresh current observation must be accepted");
  s.correction(.02, 0., stamp, s.settled_ns);
  f.tick();
  s.heartbeat();  // The original coordinates renew the heartbeat while the correction moves.
  f.finish_handshake();
  const auto endpoint = f.commanded_pose();
  require(std::abs(endpoint.x - measured.x - .01) < 1e-5 &&
    std::abs(endpoint.z - measured.z + .003) < 1e-5 &&
    std::abs(endpoint.pitch - s.nominal.pitch) < 1e-5 &&
    f.motors[0]->triggers == before + 1 && f.tracker->lateral_velocity(f.now) == 0.0,
    "exactly one correction must use settled FK plus the residual and preserve locked pitch");
  f.measure(endpoint);
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  f.tick();
  f.tick();
  s.heartbeat();
  require(s.stage == "correcting", "the second halt must settle before reporting completion");
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  f.tick();
  s.receive();
  require(s.stage == "complete" && f.halted(), "completion acknowledges the second settled halt");
  s.correction(.02, 0., f.node->now().nanoseconds(), s.settled_ns);
  s.heartbeat();
  f.tick();
  require(s.stage == "complete" && f.motors[0]->triggers == before + 1 &&
    !f.controller->position_command_pending(), "no fresh-stamped message can rearm this stroke");
}

void shared_stroke_holds_on_cancel_timeout_or_missing_heartbeat()
{
  for (const std::string reason : {"limit", "timeout", "heartbeat", "generation", "owner",
    "accepted-then-handover", "changed-goal", "disable", "imu", "zero", "zero-drift", "drift"})
  {
    StrokeFixture s;
    auto & f = s.f;
    s.settle();
    const auto before = f.motors[0]->triggers;
    if (reason == "limit") {
      s.correction(.02, .02, f.node->now().nanoseconds(), s.settled_ns);
    } else if (reason == "timeout") {
      f.pace_ticks = false;
      // Both heartbeats stay fresh, while the independent verification deadline expires.
      for (int i = 0; i < 26; ++i) {
        f.tick(100ms);
        f.tracker->observe_goal(f.goal_message(s.nominal.x, s.nominal.z), f.now, s.id);
      }
    } else if (reason == "heartbeat") {
      f.pace_ticks = false;
      f.tick(301ms);
    } else if (reason == "disable") {
      require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", false)).successful,
        "operator disable must be accepted");
      f.tick();
    } else if (reason == "generation") {
      ++s.generation;
      f.tick();
    } else if (reason == "owner" || reason == "accepted-then-handover") {
      if (reason == "accepted-then-handover") {
        s.correction(.01, 0., f.node->now().nanoseconds(), s.settled_ns);
        require(s.stage == "correcting", "correction must have been accepted before handover");
      }
      s.owner = false;
      f.tick();
    } else if (reason == "changed-goal") {
      s.nominal.x += 0.01;
      f.tick();
      s.heartbeat();
    } else if (reason == "imu") {
      f.imu_yaw = .25;
      f.tick();
    } else {
      s.correction(reason == "drift" ? .01 : 0., 0., f.node->now().nanoseconds(), s.settled_ns);
      if (reason != "zero") {
        auto moved = f.controller->actual_pose();
        moved.z -= .008;
        f.measure(moved);
      }
      f.tick();
      f.tick();
    }
    s.receive();
    require(s.stage == (reason == "zero" ? "complete" : "cancelled"),
      "end of the verification window must be explicit");
    s.correction(.01, 0., f.node->now().nanoseconds(), s.settled_ns);
    require(f.halted() && f.motors[0]->triggers == before && !f.controller->position_command_pending(),
      "no terminal or no-adjustment case may send another PP command");
    if (reason == "disable") {
      f.enable_goal();
      f.tick();
      s.heartbeat();
      f.tick();
      f.tick();
      s.receive();
      require(s.stage == "cancelled" && !f.controller->position_command_pending(),
        "retired stroke id cannot enter a later button session");
    }
  }
}

void shared_tracking_requires_a_generation_callback()
{
  Fixture f;
  bool rejected = false;
  try {
    chassis::ArmVisualTracking incomplete(*f.node, *f.controller,
      std::function<void(const std::exception &)>{}, [] {return true;});
  } catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "shared tracking must not start without generation isolation");
}

void unreachable_target_is_input_rejection()
{
  Fixture f;
  f.measure({0.14, std::sqrt(0.7 * 0.7 - 0.14 * 0.14), 0.0});
  f.enable();
  const auto baseline = f.controller->actual_pose();
  f.target(baseline.z + 0.05);
  f.tick(100ms);
  require(f.halted() && !f.controller->position_command_pending() &&
    f.controller->fault_reason().empty() && !f.power_off,
    "unreachable complete target must hold without hardware fault");
  require(f.tracker->lateral_velocity(f.now) == 0.0,
    "unreachable height must not leave chassis lateral motion active");
}

void unreachable_height_is_not_replaced_with_a_partial_move()
{
  Fixture f(true);
  f.measure({0.14, 0.6858, 0.0});
  f.enable();
  f.tick();
  const auto baseline = f.controller->actual_pose();
  const auto triggers = f.motors[0]->triggers;
  f.target(baseline.z + 0.05);
  f.tick();
  require(f.halted() && !f.controller->position_command_pending() &&
    f.motors[0]->triggers == triggers && f.controller->fault_reason().empty(),
    "an unreachable full height must be rejected even when a shorter move would be reachable");
}

void app_stop_discards_unapplied_and_in_flight_targets()
{
  for (const bool fourth_axis : {false, true}) {
    // Zero ticks leaves the parameter committed but unapplied. Later ticks
    // cover queued, staged, triggered and acknowledged PP commands.
    for (unsigned ticks = 0; ticks <= 8; ++ticks) {
      Fixture f(false, fourth_axis);
      chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
        [](Clock::time_point) {return true;});
      app.command(103, f.now);
      const auto requested = f.node->get_parameter("yiyou_arm_target").as_double_array();
      require(requested == std::vector<double>({f.initial.x, f.initial.z + 0.001, f.initial.pitch}),
        "App step must commit the same metre/radian target used by manual control");
      for (unsigned i = 0; i < ticks; ++i) {f.tick();}
      const std::array<unsigned, 3> triggers{
        f.motors[0]->triggers, f.motors[1]->triggers, f.motors[2]->triggers};
      f.motors[0]->put<std::int8_t>(0x2707, 0);
      f.motors[0]->put<std::int32_t>(0x606C, 6000);
      app.command(109, f.now);
      require(f.halted() && !f.controller->position_command_pending(),
        "App stop must cancel a committed or in-flight target on three or four axes");
      f.tick();
      require(f.controller->planar_halt_pending(), "App stop must wait for measured standstill");
      app.command(103, f.now);
      require(f.node->get_parameter("yiyou_arm_target").as_double_array() == requested,
        "App steps during halt must not replace the committed request");
      auto settled = f.initial;
      settled.z += 0.002;
      f.measure(settled);
      f.motors[0]->put<std::int8_t>(0x2707, 1);
      f.motors[0]->put<std::int32_t>(0x606C, 0);
      for (unsigned i = 0; i < 12; ++i) {f.tick();}
      const auto actual = f.controller->actual_pose();
      require(f.node->get_parameter("yiyou_arm_target").as_double_array() ==
        std::vector<double>({actual.x, actual.z, actual.pitch}),
        "App stop must consume the old request by synchronizing the stopped pose");
      require(f.halted() && !f.controller->planar_halt_pending() &&
        !f.controller->position_command_pending(), "App stop must remain held after standstill");
      for (std::size_t i = 0; i < triggers.size(); ++i) {
        require(f.motors[i]->triggers == triggers[i], "App stop must never replay the old target");
      }
      require(!f.power_off && f.controller->fault_reason().empty(),
        "ordinary App stop must hold safely without a hardware fault");
    }
  }
}

void shared_source_handover_waits_for_feedback_and_consumes_old_targets()
{
  for (const bool visual_target : {false, true}) {
    Fixture f(true);
    chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
      [](Clock::time_point) {return true;});
    if (visual_target) {
      require(f.node->set_parameter(rclcpp::Parameter("arm_tracking_enabled", true)).successful,
        "visual enable must commit");
      require(!f.tracker->complete_source_handover(),
        "a pending visual enable cannot be bypassed by source handover");
      f.tick();
      require(!f.tracker->complete_source_handover(),
        "an enabled visual session cannot be cleared by source handover");
      f.tick();
      f.target(f.initial.z + 0.01);
    } else {
      app.command(103, f.now);
    }
    for (unsigned i = 0; i < 4; ++i) {f.tick();}
    const auto old_visual = f.message(f.initial.z + 0.01);
    const std::array<unsigned, 3> triggers{
      f.motors[0]->triggers, f.motors[1]->triggers, f.motors[2]->triggers};
    f.motors[0]->put<std::int8_t>(0x2707, 0);
    f.motors[0]->put<std::int32_t>(0x606C, 6000);
    app.stop(f.now);
    require(!f.tracker->complete_source_handover(),
      "source handover must not skip the halt command acknowledgement");
    f.tick();
    require(f.controller->planar_halt_pending() && !f.tracker->complete_source_handover(),
      "source handover must wait while motor feedback still reports motion");
    require(!f.tracker->accept_manual_chassis_command(0.2, 0.0, 0.0, f.now),
      "new source motion must remain stopped during the physical halt");

    f.motors[0]->put<std::int8_t>(0x2707, 1);
    f.motors[0]->put<std::int32_t>(0x606C, 0);
    f.tick(20ms, false);
    require(!f.controller->planar_halt_pending(), "standstill feedback must complete the halt");
    require(!f.tracker->complete_source_handover(),
      "halt completion alone cannot bypass pending manual baseline synchronization");
    f.tracker->after_update();
    require(f.tracker->complete_source_handover(),
      "shared source handover must complete after feedback and baseline synchronization");
    require(f.tracker->accept_manual_chassis_command(0.2, 0.0, 0.0, f.now),
      "new source motion must not require another neutral gesture after a safe handover");
    f.tracker->observe_target(old_visual, f.now);
    for (unsigned i = 0; i < 8; ++i) {f.tick();}
    require(f.halted() && !f.controller->position_command_pending() &&
      !f.node->get_parameter("arm_tracking_enabled").as_bool(),
      "completing source handover must not enable tracking or restore an old manual target");
    for (std::size_t i = 0; i < triggers.size(); ++i) {
      require(f.motors[i]->triggers == triggers[i], "old targets must never retrigger after handover");
    }
  }
}

void app_fourth_pulses_expire_without_renewal_or_replay()
{
  for (const int command : {107, 108}) {
    for (unsigned ticks = 0; ticks <= 4; ++ticks) {
      Fixture f;
      f.motors[3]->put<std::int32_t>(0x6064, static_cast<std::int32_t>(kPulses));
      chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
        [](Clock::time_point) {return true;});
      const auto start = f.now;
      const auto initial_triggers = f.motors[3]->triggers;
      app.command(command, start);
      const int direction = command == 107 ? 1 : -1;
      require(f.controller->fourth_axis_jog() == direction, "App fourth-axis pulse must start");
      for (unsigned i = 0; i < ticks; ++i) {
        app.before_update(f.now + 20ms);
        f.tick();
      }
      if (ticks == 2) {
        require(f.motors[3]->triggers == initial_triggers,
          "expiry fixture must cover a staged target before its trigger");
      } else if (ticks == 3) {
        require(f.motors[3]->triggers > initial_triggers,
          "expiry fixture must also cover an already triggered target");
      }
      app.command(command, start + 90ms);
      app.command(command == 107 ? 108 : 107, start + 99ms);
      app.before_update(start + 99ms);
      require(f.controller->fourth_axis_jog() == direction,
        "repeated or opposite clicks must not replace the active pulse");
      const auto triggers = f.motors[3]->triggers;
      app.before_update(start + 100ms);
      require(f.controller->fourth_axis_jog() == 0 &&
        (f.motors[3]->get<std::uint16_t>(0x6040) & 0x0100U),
        "repeated clicks must not extend the original 100 ms deadline");
      f.tick(std::chrono::duration_cast<std::chrono::milliseconds>(start + 100ms - f.now));
      for (unsigned i = 0; i < 8; ++i) {
        app.before_update(f.now + 20ms);
        f.tick();
      }
      require(f.motors[3]->triggers == triggers,
        "expired fourth-axis work must never trigger later");
      app.command(command, f.now);
      require(f.controller->fourth_axis_jog() == direction,
        "a genuinely new tap after expiry must be accepted");
      app.stop(f.now);
      require(f.controller->fourth_axis_jog() == 0, "explicit stop must also cancel the new pulse");
    }
  }
}

void app_heartbeat_and_visual_ownership_gate_moves_but_not_stop()
{
  Fixture f;
  bool fresh = false;
  chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
    [&](Clock::time_point) {return fresh;});
  for (int command = 101; command <= 108; ++command) {app.command(command, f.now);}
  require(f.node->get_parameter("yiyou_arm_target").as_double_array().empty() &&
    f.controller->fourth_axis_jog() == 0 && !f.controller->position_command_pending(),
    "without a fresh gateway heartbeat no App movement may be accepted");
  fresh = true;
  app.command(103, f.now);
  f.tick();
  app.command(107, f.now);
  require(f.controller->position_command_pending() && f.controller->fourth_axis_jog() == 1,
    "fresh heartbeat must permit planar and fourth-axis commands");
  fresh = false;
  app.command(109, f.now);
  require(f.halted() && !f.controller->position_command_pending() &&
    f.controller->fourth_axis_jog() == 0, "109 must stop every axis even after heartbeat loss");
  f.tick();
  fresh = true;
  f.enable();
  const auto manual = f.node->get_parameter("yiyou_arm_target").as_double_array();
  app.command(103, f.now);
  require(f.node->get_parameter("yiyou_arm_target").as_double_array() == manual,
    "App planar commands must respect visual ownership");
  app.stop(f.now);
  require(!f.node->get_parameter("arm_tracking_enabled").as_bool() && f.halted(),
    "App stop must revoke visual ownership as well as halt the arm");
}

void app_repeated_stop_is_idle_but_new_targets_are_stopped()
{
  for (const bool fourth_axis : {false, true}) {
    Fixture f(false, fourth_axis);
    chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
      [](Clock::time_point) {return true;});
    const auto io_counts = [&] {
        std::array<std::array<unsigned, 3>, 4> counts{};
        for (std::size_t i = 0; i < f.motors.size(); ++i) {
          counts[i] = {f.motors[i]->reads, f.motors[i]->writes, f.motors[i]->triggers};
        }
        return counts;
      };
    app.command(103, f.now);
    f.tick();
    if (fourth_axis) {app.command(107, f.now);}
    f.motors[0]->put<std::int8_t>(0x2707, 0);
    f.motors[0]->put<std::int32_t>(0x606C, 6000);
    app.command(109, f.now);
    const auto halting_io = io_counts();
    for (unsigned i = 0; i < 20; ++i) {app.command(109, f.now);}
    require(io_counts() == halting_io,
      "repeated App stop while settling must not read, write or trigger hardware");
    f.motors[0]->put<std::int8_t>(0x2707, 1);
    f.motors[0]->put<std::int32_t>(0x606C, 0);
    f.tick();
    const auto stopped_io = io_counts();
    for (unsigned i = 0; i < 20; ++i) {app.command(109, f.now);}
    require(io_counts() == stopped_io,
      "repeated App stop at standstill must not read, write or trigger hardware");

    // A new App target is committed before the periodic parameter application.
    app.velocity(0.0, 0.0, 0.0, f.now);
    app.command(103, f.now);
    app.command(109, f.now);
    for (unsigned i = 0; i < 10; ++i) {f.tick();}
    require(f.halted() && !f.controller->position_command_pending(),
      "a stop latch must not hide a later App target");

    // External manual requests must also invalidate the idle shortcut, both
    // before application and after the PP acknowledgement has completed.
    for (const bool finish_handshake : {false, true}) {
      const auto actual = f.controller->actual_pose();
      require(f.node->set_parameter(rclcpp::Parameter("yiyou_arm_target",
          std::vector<double>{actual.x, actual.z + 0.01, actual.pitch})).successful,
        "external manual target must be accepted after App stop");
      if (finish_handshake) {
        f.tick();
        f.finish_handshake();
        require(!f.halted(), "external target must have released planar hold");
      }
      const auto before_stop = io_counts();
      app.command(109, f.now);
      for (unsigned i = 0; i < 10; ++i) {f.tick();}
      require(f.halted() && !f.controller->position_command_pending(),
        "repeated App stop must cancel new external manual work");
      const auto stopped = f.controller->actual_pose();
      require(f.node->get_parameter("yiyou_arm_target").as_double_array() ==
        std::vector<double>{stopped.x, stopped.z, stopped.pitch},
        "external target must be consumed rather than replayed after stop");
      for (std::size_t i = 0; i < f.motors.size(); ++i) {
        require(f.motors[i]->triggers == before_stop[i][2],
          "repeated stop must never trigger stale App or external work");
      }
    }
  }
}

void app_tick(Fixture & f, chassis::AppArmControl & app, std::chrono::milliseconds elapsed = 20ms)
{
  app.before_update(f.now + elapsed);
  f.tick(elapsed);
  app.after_update(f.now);
}

void app_frame(
  Fixture & f, chassis::AppArmControl & app, const std::array<double, 3> & input,
  std::chrono::milliseconds elapsed = 20ms)
{
  app.velocity(input[0], input[1], input[2], f.now + elapsed);
  app_tick(f, app, elapsed);
}

void app_taps_finish_on_release_but_need_their_own_lease()
{
  for (const int command : {101, 102, 103, 104}) {
    Fixture f;
    chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
      [](Clock::time_point) {return true;});  // Chassis heartbeats stay fresh throughout.
    for (const int removed : {105, 106}) {app.command(removed, f.now);}
    require(f.node->get_parameter("yiyou_arm_target").as_double_array().empty(),
      "removed App pitch actions must be rejected at the motion entry point");
    app.velocity(1e-6, -1e-6, 1e-6, f.now);
    app.command(command, f.now);
    const auto expected = *chassis::app_arm_target(f.initial, command);
    const auto target = f.node->get_parameter("yiyou_arm_target").as_double_array();
    require(target == std::vector<double>({expected.x, expected.z, expected.pitch}),
      "all four App taps must commit a one-millimetre Cartesian target");
    const unsigned triggers = f.motors[0]->triggers;
    for (unsigned i = 0; i < 20; ++i) {app_frame(f, app, {0, 0, 0});}
    require(f.motors[0]->triggers == triggers + 1 && !f.halted() &&
      f.node->get_parameter("yiyou_arm_target").as_double_array() == target,
      "neutral arm frames must renew a tap without truncating or repeating its endpoint");
    const auto last_frame = f.now;
    app.before_update(last_frame + 249ms);
    require(!f.halted(), "a tap remains valid strictly before its own lease deadline");
    app.before_update(last_frame + 250ms);
    require(f.halted() && !f.controller->position_command_pending(),
      "fresh chassis heartbeats cannot keep an arm tap alive without arm frames");
    app_tick(f, app, 250ms);
    const auto held = f.node->get_parameter("yiyou_arm_target").as_double_array();
    app.command(command, f.now);
    require(f.node->get_parameter("yiyou_arm_target").as_double_array() == held,
      "a legacy tap cannot bypass the neutral gate after the arm lease expires");
    app.velocity(0, 0, 0, f.now);
    app.command(command, f.now);
    require(f.node->get_parameter("yiyou_arm_target").as_double_array() != held,
      "a fresh neutral arm frame can rearm a new explicit tap after expiry");
  }
}

void app_continuous_input_is_proportional_and_time_bounded()
{
  for (unsigned axis = 0; axis < 3; ++axis) {
    for (const double scale : {0.5, 1.0}) {
      Fixture f;
      chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
        [](Clock::time_point) {return true;});
      std::array<double, 3> input{};
      input[axis] = scale;
      app.velocity(input[0], input[1], input[2], f.now);
      app_tick(f, app);
      require(f.halted() && !f.controller->position_command_pending(),
        "first held input must establish a stopped feedback baseline");
      app_frame(f, app, input);
      const auto target = f.node->get_parameter("yiyou_arm_target").as_double_array();
      const std::array<double, 3> baseline{f.initial.x, f.initial.z, f.initial.pitch};
      for (unsigned i = 0; i < 3; ++i) {
        const double rate = i == 2 ? arm::kPi / 180.0 : 0.005;
        require(std::abs(target[i] - baseline[i] - (i == axis ? scale * rate * 0.02 : 0)) < 1e-12,
          "continuous input must be proportional and preserve all unoperated coordinates");
      }
    }
  }
  Fixture f;
  chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
    [](Clock::time_point) {return true;});
  app.velocity(1, 0, 0, f.now);
  app_tick(f, app);
  app_frame(f, app, {1, 0, 0}, 200ms);
  require(std::abs(f.node->get_parameter("yiyou_arm_target").as_double_array()[0] -
      f.initial.x - 0.005 * 0.05) < 1e-12,
    "a delayed tick must integrate no more than 50 ms and discard the remaining time");
}

void app_continuous_frozen_feedback_has_no_windup_or_replay()
{
  Fixture f;
  chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
    [](Clock::time_point) {return true;});
  app.velocity(1, 0, 1, f.now);
  app_tick(f, app);
  unsigned plateau_triggers = 0;
  for (unsigned i = 0; i < 1000; ++i) {
    app_frame(f, app, {1, 0, 1});
    const auto target = f.node->get_parameter("yiyou_arm_target").as_double_array();
    require(target[0] - f.initial.x <= 0.001 + 1e-12 &&
      target[2] - f.initial.pitch <= 0.2 * arm::kPi / 180.0 + 1e-12 &&
      target[1] == f.initial.z,
      "frozen feedback must bound the sole reference to 1 mm / 0.2 degree without Z drift");
    const auto joints = f.commanded_joints();
    for (const auto angle : joints) {
      require(std::abs(angle) <= 0.72 * arm::kPi / 180.0 + 2.0 * arm::kPi / kPulses,
        "submitted joints must remain inside the configured profile's 200 ms lead");
    }
    if (i == 100) {plateau_triggers = f.motors[0]->triggers;}
  }
  require(f.motors[0]->triggers == plateau_triggers && plateau_triggers > 0,
    "repeated frozen-feedback frames must not queue or keep retriggering an unchanged endpoint");
  const auto previous = f.node->get_parameter("yiyou_arm_target").as_double_array();
  f.measure({previous[0], previous[1] + 0.00005, previous[2]});
  app_frame(f, app, {1, 0, 1});
  const auto resumed = f.node->get_parameter("yiyou_arm_target").as_double_array();
  require(resumed[0] - previous[0] <= 0.0001 + 1e-12 &&
    resumed[2] - previous[2] <= 0.02 * arm::kPi / 180.0 + 1e-12 &&
    resumed[1] == f.initial.z,
    "restored feedback must not release accumulated targets or resample the inactive Z baseline");
}

void app_continuous_release_and_reversal_cancel_every_pp_phase()
{
  for (const bool reverse : {false, true}) {
    for (unsigned ticks = 0; ticks <= 8; ++ticks) {
      Fixture f;
      chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
        [](Clock::time_point) {return true;});
      app.velocity(1, 0, 0, f.now);
      app_tick(f, app);
      app_frame(f, app, {1, 0, 0});
      for (unsigned i = 0; i < ticks; ++i) {app_frame(f, app, {1, 0, 0});}
      const unsigned triggers = f.motors[0]->triggers;
      if (ticks != 0) {
        // Before any PP application the arm is still physically held from the
        // initial stop. Later phases also cover a motor still decelerating.
        f.motors[0]->put<std::int8_t>(0x2707, 0);
        f.motors[0]->put<std::int32_t>(0x606C, 6000);
      }
      app.velocity(reverse ? -1 : 0, 0, 0, f.now);
      require(f.halted() && !f.controller->position_command_pending(),
        "release or reversal must cancel unapplied, staged and triggered continuous endpoints");
      if (ticks != 0) {
        for (unsigned i = 0; i < 3; ++i) {app_frame(f, app, {reverse ? -1.0 : 0.0, 0, 0});}
        require(f.controller->planar_halt_pending() && f.motors[0]->triggers == triggers,
          "held reverse input must never mix a new target into moving halt feedback");
      }
      auto coasted = f.initial;
      coasted.x += 0.0002;
      f.measure(coasted);
      f.motors[0]->put<std::int8_t>(0x2707, 1);
      f.motors[0]->put<std::int32_t>(0x606C, 0);
      app_tick(f, app);
      const auto actual = f.controller->actual_pose();
      app_tick(f, app);
      require(f.motors[0]->triggers == triggers && !f.controller->position_command_pending() &&
        f.node->get_parameter("yiyou_arm_target").as_double_array() ==
        std::vector<double>({actual.x, actual.z, actual.pitch}),
        "standstill must consume the old endpoint and must not replay a pre-stop input");
      if (reverse) {
        app_frame(f, app, {-1, 0, 0});
        const auto target = f.node->get_parameter("yiyou_arm_target").as_double_array();
        require(target[0] < actual.x && target[1] == actual.z && target[2] == actual.pitch,
          "fresh post-standstill reverse input must start from measured position");
      }
    }
  }
}

void app_continuous_expiry_and_idle_stop_clear_cached_input()
{
  Fixture f;
  chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
    [](Clock::time_point) {return true;});
  app.velocity(1, 0, 0, f.now);
  app_tick(f, app);
  app_frame(f, app, {1, 0, 0});
  for (unsigned i = 0; i < 3; ++i) {app_frame(f, app, {1, 0, 0});}
  const unsigned triggers = f.motors[0]->triggers;
  // A fresh callback arriving after a stall must expire the old lease before it
  // can renew it, even if before_update has not run during the stall.
  app_frame(f, app, {1, 0, 0}, 300ms);
  for (unsigned i = 0; i < 10; ++i) {app_frame(f, app, {1, 0, 0});}
  require(f.halted() && f.motors[0]->triggers == triggers &&
    !f.controller->position_command_pending(), "held input after lease loss must require neutral");
  app_frame(f, app, {0, 0, 0});
  app_frame(f, app, {1, 0, 0});
  // The arm is already held and a fresh command is only waiting for its next
  // frame; stop's I/O shortcut must still clear that cached continuous intent.
  app.stop(f.now);
  for (unsigned i = 0; i < 10; ++i) {app_tick(f, app);}
  require(f.halted() && f.motors[0]->triggers == triggers,
    "a repeated idle stop must clear any pending continuous intent without replay");
  app_frame(f, app, {0, 0, 0});
  app_frame(f, app, {1, 0, 0});
  app_frame(f, app, {1, 0, 0});
  require(f.node->get_parameter("yiyou_arm_target").as_double_array()[0] >
    f.controller->actual_pose().x, "neutral and fresh post-stop frames must allow deliberate recovery");
}

void app_continuous_stopped_feedback_jitter_does_not_block_start()
{
  Fixture f;
  chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
    [](Clock::time_point) {return true;});
  app.stop(f.now);
  app_tick(f, app);
  app.velocity(0, 0, 0, f.now);
  // The stop shortcut sees the previously synchronized cached pose. The next
  // feedback differs by one encoder count while still reporting standstill.
  f.measure_joints({2.0 * arm::kPi / kPulses, 0, 0});
  app.velocity(1, 0, 0, f.now);
  app_tick(f, app);
  const auto baseline = f.controller->actual_pose();
  app_frame(f, app, {1, 0, 0});
  const auto target = f.node->get_parameter("yiyou_arm_target").as_double_array();
  require(std::abs(target[0] - baseline.x - 0.0001) < 1e-12 &&
    target[1] == baseline.z && target[2] == baseline.pitch,
    "one-count stopped feedback jitter must not block a fresh continuous command");
}

void app_handover_neutral_during_halt_arms_only_fresh_motion()
{
  Fixture f;
  chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
    [](Clock::time_point) {return true;});
  app.command(103, f.now);
  app_tick(f, app);
  f.motors[0]->put<std::int8_t>(0x2707, 0);
  f.motors[0]->put<std::int32_t>(0x606C, 6000);
  app.stop(f.now);  // Main's mode/owner handover, before forwarding its neutral.
  const unsigned triggers = f.motors[0]->triggers;
  app_frame(f, app, {0, 0, 0});
  require(f.controller->planar_halt_pending() && f.halted() &&
    f.motors[0]->triggers == triggers,
    "neutral during handover must arm input without releasing the physical halt");
  f.motors[0]->put<std::int8_t>(0x2707, 1);
  f.motors[0]->put<std::int32_t>(0x606C, 0);
  app_tick(f, app);
  require(!f.controller->position_command_pending() && f.halted(),
    "handover standstill alone must not replay the old tap");
  app_frame(f, app, {1, 0, 0});
  app_frame(f, app, {1, 0, 0});
  const auto actual = f.controller->actual_pose();
  const auto target = f.node->get_parameter("yiyou_arm_target").as_double_array();
  require(std::abs(target[0] - actual.x - 0.0001) < 1e-12 &&
    target[1] == actual.z && target[2] == actual.pitch,
    "post-handover held input must work without a second neutral gesture");
}

void app_continuous_rejects_invalid_or_infeasible_groups_before_writes()
{
  for (const bool joint_lead : {false, true}) {
    Fixture f;
    if (joint_lead) {
      f.node->declare_parameter("yiyou_arm_speed_rpm", 0.0001);
    } else {
      f.measure({0.14, std::sqrt(0.7 * 0.7 - 0.14 * 0.14), 0.0});
      f.tick();
    }
    chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
      [](Clock::time_point) {return true;});
    app.velocity(0, 1, 0, f.now);
    app_tick(f, app);
    const std::array<unsigned, 3> writes{f.motors[0]->position_writes,
      f.motors[1]->position_writes, f.motors[2]->position_writes};
    app_frame(f, app, {0, 1, 0});
    for (unsigned i = 0; i < 10; ++i) {app_frame(f, app, {0, 1, 0});}
    require(f.halted() && !f.controller->position_command_pending() && !f.power_off &&
      f.controller->fault_reason().empty(), "infeasible IK or excessive joint lead must hold safely");
    for (unsigned i = 0; i < 3; ++i) {
      require(f.motors[i]->position_writes == writes[i],
        "rejected continuous targets must never partially write a motor target");
    }
  }
  for (const double invalid : {1.01, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    Fixture f;
    chassis::AppArmControl app(*f.node, *f.controller, *f.tracker,
      [](Clock::time_point) {return true;});
    app.velocity(1, 0, 0, f.now);
    app_tick(f, app);
    app_frame(f, app, {1, 0, 0});
    const unsigned triggers = f.motors[0]->triggers;
    app.velocity(invalid, 0, 0, f.now);
    for (unsigned i = 0; i < 8; ++i) {app_frame(f, app, {1, 0, 0});}
    require(f.halted() && !f.controller->position_command_pending() &&
      f.motors[0]->triggers == triggers,
      "malformed input must stop and require neutral instead of leaving an old endpoint active");
  }
}

void startup_home_parameter_contract()
{
  auto node = std::make_shared<rclcpp::Node>("arm_home_parameter_test",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter(
      "yiyou_arm_home_joint_degrees", std::vector<double>{0.0, -60.0, -15.0})}));
  const auto config = chassis::declare_arm_motion_parameters(*node);
  require(config.home_on_start && std::abs(config.home_joints[1] + arm::kPi / 3) < 1e-12 &&
    std::abs(config.home_joints[2] + arm::kPi / 12) < 1e-12,
    "home parameter must convert calibrated degrees to radians without geometric offsets");
  require(!node->set_parameter(rclcpp::Parameter("yiyou_arm_home_joint_degrees",
      std::vector<double>{0.0, 0.0, 0.0})).successful,
    "startup home target cannot be changed at runtime");
  for (const auto & value : {std::vector<double>{0.0, 0.0},
      std::vector<double>{0.0, -330.01, 0.0},
      std::vector<double>{0.0, 0.01, 0.0},
      std::vector<double>{0.0, std::numeric_limits<double>::infinity(), 0.0}})
  {
    auto invalid = std::make_shared<rclcpp::Node>("arm_invalid_home_test",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("yiyou_arm_home_joint_degrees", value)}));
    bool rejected = false;
    try {(void)chassis::declare_arm_motion_parameters(*invalid);}
    catch (const std::invalid_argument &) {rejected = true;}
    require(rejected, "malformed, out-of-limit or nonfinite startup home must be rejected");
  }
}
void startup_horizontal_home_from_yaml()
{
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  const auto config_directory = root / "src/config";
  const auto file = (config_directory / "yiyou_ethercat.yaml").string();
  auto node = std::make_shared<rclcpp::Node>("chassis",
    rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", file}));
  const auto config = chassis::declare_arm_motion_parameters(*node, config_directory);
  require(node->get_parameter("yiyou_arm_kinematics_file").as_string() ==
    "package://chassis/config/arm_kinematics.json",
    "deployed YAML must explicitly load this checkout's calibrated JSON model");
  require(config.home_on_start && config.motor_directions == std::array<int, 3>{1, -1, 1} &&
    config.zero_counts == std::array<std::int32_t, 3>{0, 0, 0},
    "deployed YAML must enable startup home with the calibrated motor mapping");
  require(std::abs(config.home_joints[0] + 10.0 * arm::kPi / 180.0) < 1e-12 &&
    std::abs(config.home_joints[1] + 115.31 * arm::kPi / 180.0) < 1e-12 &&
    std::abs(config.home_joints[2] - 117.56840515136719 * arm::kPi / 180.0) < 1e-12 &&
    std::abs(config.kinematics.limits[1].min + 330.0 * arm::kPi / 180.0) < 1e-12 &&
    std::abs(config.kinematics.limits[1].max) < 1e-12,
    "deployed YAML must load horizontal home inside the authorized elbow range");
  require(std::abs(config.kinematics.zero_offsets[2] + 0.4811594394151231) < 1e-12,
    "deployed YAML must use the IMU-calibrated wrist offset instead of the built-in baseline");
  const auto pose = arm::Kinematics(config.kinematics).forward(config.home_joints);
  require(std::abs(pose.x - 0.1401923788646684) < 1e-12 &&
    std::abs(pose.z - 0.15) < 1e-12 && std::abs(pose.pitch - arm::kPi / 2) < 1e-12,
    "calibrated home must preserve position and the model's vertical tool direction");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    startup_home_parameter_contract();
    startup_horizontal_home_from_yaml();
    synchronized_profiles_scale_signed_pulse_travel_before_any_trigger();
    synchronized_profiles_handle_zero_and_one_pulse_axes();
    synchronized_profiles_defer_initial_motion_and_reject_configuration_drift();
    synchronized_profiles_allow_bounded_stopped_velocity_jitter();
    synchronized_profile_failure_and_cancellation_never_trigger_partial_groups();
    profile_standstill_read_failure_faults_both_target_setters();
    synchronized_profile_handshake_deadline_prevents_late_trigger();
    synchronized_move_halt_restores_profiles_for_ordinary_and_home_targets();
    fixed_goal_scales_profiles_while_live_tracking_keeps_original_limits();
    fixed_goal_expiry_cancels_profile_setup_before_controller_update();
    defaults_and_startup_guard();
    imu_loss_and_heading_fault_latch_fixed_goal_and_lateral_motion();
    x_mode_changes_require_disabled_tracking_or_atomic_disable();
    depth_x_moves_both_directions_and_keeps_z_and_pitch();
    depth_diagonal_targets_are_complete_at_any_update_interval();
    latest_depth_target_waits_only_for_the_previous_handshake();
    depth_x_reversal_halts_before_accepting_a_new_capture();
    unreachable_depth_target_is_rejected_without_partial_motion();
    depth_tracking_stop_expiry_and_handover_halt_all_visual_motion();
    goal_mode_requires_x_and_atomic_mode_transitions();
    goal_submission_waits_for_settled_activation_feedback();
    goal_alignment_holds_arm_and_committed_goal_ignores_live_packets();
    fixed_goal_heartbeats_keep_one_command_beyond_ten_seconds();
    fixed_goals_submit_the_full_endpoint_once();
    invalid_full_goal_is_rejected_before_motion_and_latches();
    endpoint_completion_waits_for_locked_x_and_pitch();
    goal_expiry_stop_change_completion_and_replays_require_a_new_session();
    lateral_direction_limits_and_independent_deadband();
    lateral_waits_for_halt_and_exit_requires_neutral();
    manual_motion_and_watchdog_latch_both_visual_axes();
    explicit_stop_clears_lateral_and_requires_a_new_capture();
    lateral_motion_continues_until_disabled_and_recovers_after_loss();
    live_targets_keep_activation_x_and_pitch_without_arrival_waits();
    deadband_and_activation_standstill_still_guard_targets();
    valid_large_joint_changes_are_not_reduced();
    valid_activation_pose_corrections_are_not_step_limited();
    expiry_precedes_handshake_and_new_capture_is_required();
    malformed_stale_and_replayed_frames_halt();
    delayed_captures_cross_the_activation_barrier();
    delayed_captures_recover_after_a_hold_without_replaying_old_input();
    explicit_stop_while_already_held_requires_a_new_capture();
    fresh_live_targets_keep_one_command_beyond_two_seconds();
    reversal_cancels_the_old_direction_before_restarting();
    activation_waits_for_actual_stop_and_manual_restore_is_fresh();
    planar_halt_cancels_each_phase_without_touching_fourth_axis();
    planar_halt_refreshes_the_ik_seed();
    stop_topic_and_hardware_failure();
    shared_tracking_ignores_old_commands_without_stopping_the_new_session();
    shared_stroke_corrects_once_from_settled_feedback();
    shared_stroke_holds_on_cancel_timeout_or_missing_heartbeat();
    shared_tracking_requires_a_generation_callback();
    unreachable_target_is_input_rejection();
    unreachable_height_is_not_replaced_with_a_partial_move();
    app_stop_discards_unapplied_and_in_flight_targets();
    shared_source_handover_waits_for_feedback_and_consumes_old_targets();
    app_fourth_pulses_expire_without_renewal_or_replay();
    app_heartbeat_and_visual_ownership_gate_moves_but_not_stop();
    app_repeated_stop_is_idle_but_new_targets_are_stopped();
    app_taps_finish_on_release_but_need_their_own_lease();
    app_continuous_input_is_proportional_and_time_bounded();
    app_continuous_frozen_feedback_has_no_windup_or_replay();
    app_continuous_release_and_reversal_cancel_every_pp_phase();
    app_continuous_expiry_and_idle_stop_clear_cached_input();
    app_continuous_stopped_feedback_jitter_does_not_block_start();
    app_handover_neutral_during_halt_arms_only_fresh_motion();
    app_continuous_rejects_invalid_or_infeasible_groups_before_writes();
    std::cout << "arm visual tracking tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "arm visual tracking: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
