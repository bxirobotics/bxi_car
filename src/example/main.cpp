#include "chassis/control.h"
#include "arm/control.h"
#include "arm/torque_log.h"
#include "example/arm_parameters.h"
#include "example/arm_position_remote.h"
#include "example/arm_visual_tracking.h"
#include "example/app_interface.h"
#include "example/app_arm_control.h"
#include "example/velocity_command_watchdog.hpp"
#include "bxi/board.h"
#include "bxi/can_timing.h"
#include "relay/control.h"
#include "motor/iswv/diagnostic_log.h"
#include "example/remote_control.h"
#include "example/shared_control_server.h"
#include "example/shared_command_gate.h"
#include "example/hardware_owner.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <set>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <cerrno>
#include <cstring>
#include <csignal>
#include <spawn.h>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>

extern char ** environ;

#ifndef CHASSIS_HARDWARE_LOCK_FILE
#define CHASSIS_HARDWARE_LOCK_FILE "/run/car2/hardware.lock"
#endif

// ROS 2 底盘节点：负责参数、速度消息和定时调度，硬件控制由 ChassisController 完成。
namespace
{
std::filesystem::path default_config_directory()
{
  if (const char * configured = std::getenv("CHASSIS_CONFIG_DIR"); configured && *configured) {
    const std::filesystem::path directory(configured);
    if (!directory.is_absolute() || !std::filesystem::is_directory(directory)) {
      throw std::invalid_argument("CHASSIS_CONFIG_DIR must be an existing absolute directory");
    }
    return directory;
  }
  // 开发时直接读取源码配置；独立部署时使用可执行程序所在安装目录的配置。
  const std::filesystem::path source(CHASSIS_SOURCE_CONFIG_DIR);
  if (std::filesystem::is_directory(source)) {return source;}
  const auto executable = std::filesystem::canonical("/proc/self/exe");
  return executable.parent_path().parent_path().parent_path() / "share/chassis/config";
}

// Runs only before opening hardware or after every hardware owner was destroyed.
// A separate process keeps plotting dependencies outside the control process.
void generate_torque_reports(const std::string & directory) noexcept
{
  if (directory.empty()) {return;}
  try {
    if (!std::filesystem::is_directory(directory)) {return;}
    std::filesystem::path script(CHASSIS_TORQUE_REPORT_SCRIPT);
    if (!std::filesystem::is_regular_file(script)) {
      script = std::filesystem::canonical("/proc/self/exe").parent_path() /
        "arm_torque_report.py";
    }
    if (!std::filesystem::is_regular_file(script)) {
      std::cerr << "Arm torque report script unavailable; CSV retained in " << directory << '\n';
      return;
    }
    std::array<std::string, 8> arguments{
      "/usr/bin/python3", "-I", "-S", script.string(), "--directory", directory, "--keep", "3"};
    std::array<char *, 9> argv{};
    for (std::size_t i = 0; i < arguments.size(); ++i) {argv[i] = arguments[i].data();}
    pid_t child{};
    const int error = ::posix_spawn(&child, argv[0], nullptr, nullptr, argv.data(), environ);
    if (error != 0) {
      std::cerr << "Arm torque report launch failed: " << std::strerror(error) << '\n';
      return;
    }
    int status{};
    pid_t waited;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (true) {
      waited = ::waitpid(child, &status, WNOHANG);
      if (waited == child || (waited < 0 && errno != EINTR)) {break;}
      if (std::chrono::steady_clock::now() >= deadline) {
        // Reporting must not prevent the supervised controller returning to waiting.
        (void)::kill(child, SIGKILL);
        do {waited = ::waitpid(child, &status, 0);} while (waited < 0 && errno == EINTR);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      std::cerr << "Arm torque report generation incomplete; CSV retained in " << directory << '\n';
    }
  } catch (const std::exception & error) {
    std::cerr << "Arm torque report unavailable: " << error.what() << '\n';
  }
}

class ChassisNode final : public rclcpp::Node
{
public:
  explicit ChassisNode(
    bool enable_arm, const rclcpp::NodeOptions & options,
    std::function<void()> poll_remote = {},
    std::shared_ptr<chassis::ArmTorqueLog> torque_log = {},
    std::shared_ptr<chassis::RemoteControlSession> session = {})
  : Node("chassis", options), torque_log_(std::move(torque_log)), session_(std::move(session))
  {
    const auto startup = read_startup_config(enable_arm);
    // Vendor threads survive device teardown. Keep other controllers excluded
    // until this process has actually exited, including torque report cleanup.
    static const auto process_hardware_owner =
      std::make_shared<chassis::HardwareOwner>(CHASSIS_HARDWARE_LOCK_FILE);
    hardware_owner_ = process_hardware_owner;
    initialize_hardware(startup, enable_arm, poll_remote);
    setup_ros_interfaces();
  }

  bool failed() const noexcept {return failed_;}

  void handover()
  {
    if (!session_ || failed_) {return;}
    try {
      InternalWrite write(internal_parameter_write_);
      command_gate_->reset(now().nanoseconds());
      command_watchdog_ = std::make_unique<chassis::VelocityCommandWatchdog>(
        std::chrono::duration<double>(command_timeout_));
      pending_remote_step_.reset();
      source_handover_pending_ = bool(arm_);
      controller_->stop();
      remote_tracking_requested_ = false;
      publish_tracking_request(false);
      if (app_arm_control_) {app_arm_control_->stop();}
      if (arm_position_remote_) {arm_position_remote_->command(0);}
      set_parameter(rclcpp::Parameter("shared_control_generation",
        static_cast<std::int64_t>(session_->generation)));
      RCLCPP_INFO(get_logger(), "CONTROL_OWNER source=%s generation=%llu; motion cleared",
        chassis::source_name(session_->owner),
        static_cast<unsigned long long>(session_->generation));
    } catch (const std::exception & error) {on_fault(error);}
  }

  void remote_velocity(std::uint64_t generation, double x, double y, double yaw)
  {
    if (command_gate_ && command_gate_->remote(generation)) {accept_velocity(x, y, yaw);}
  }

  bool remote_arm_step(std::uint64_t generation, unsigned axis, double delta)
  {
    if (!command_gate_ || !command_gate_->remote(generation) || !arm_ || axis > 2 ||
      !std::isfinite(delta)) {return false;}
    if (source_handover_pending_) {
      if (pending_remote_step_) {return false;}
      pending_remote_step_ = PendingRemoteStep{generation, axis, delta,
        std::chrono::steady_clock::now() + std::chrono::milliseconds(300)};
      return true;
    }
    if (arm_->position_command_pending()) {return false;}
    try {
      InternalWrite write(internal_parameter_write_);
      auto values = get_parameter(chassis::kYiyouPoseParameter).as_double_array();
      if (values.empty()) {
        const auto pose = arm_->actual_pose();
        values = {pose.x, pose.z, pose.pitch};
      }
      values.at(axis) += delta;
      const auto result = set_parameter(rclcpp::Parameter(chassis::kYiyouPoseParameter, values));
      if (!result.successful) {
        RCLCPP_WARN(get_logger(), "ARM_REMOTE rejected: %s", result.reason.c_str());
      }
      return result.successful;
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN(get_logger(), "ARM_REMOTE rejected: %s", error.what());
      return false;
    } catch (const std::exception & error) {on_fault(error); return false;}
  }

  void remote_arm_jog(std::uint64_t generation, int direction)
  {
    if (!command_gate_ || !command_gate_->remote(generation) || !arm_position_remote_) {return;}
    try {arm_position_remote_->command(direction);}
    catch (const std::exception & error) {on_fault(error);}
  }

  void remote_tracking(std::uint64_t generation, bool requested)
  {
    if (!command_gate_ || !command_gate_->remote(generation)) {return;}
    try {
      InternalWrite write(internal_parameter_write_);
      if (remote_tracking_requested_ && !requested && app_arm_control_) {app_arm_control_->stop();}
      remote_tracking_requested_ = requested;
      if (!source_handover_pending_ || !requested) {publish_tracking_request(requested);}
    } catch (const std::exception & error) {on_fault(error);}
  }

  void remote_relay_toggle(std::uint64_t generation)
  {
    if (failed_ || !relay_) {return;}
    if (session_ && (!command_gate_ || !command_gate_->remote(generation))) {return;}
    if (!relay_->toggle()) {
      RCLCPP_WARN(get_logger(), "CAN_RELAY toggle ignored: initialization pending or state UNKNOWN");
    }
  }

private:
  void update_relay()
  {
    if (!relay_) {return;}
    relay_->update();
    const auto report = relay_->take_report();
    if (!report.empty()) {
      if (relay_->failed()) {RCLCPP_ERROR(get_logger(), "%s", report.c_str());}
      else {RCLCPP_INFO(get_logger(), "%s", report.c_str());}
    }
  }

  struct InternalWrite
  {
    bool & flag;
    bool previous;
    explicit InternalWrite(bool & value) : flag(value), previous(value) {flag = true;}
    ~InternalWrite() {flag = previous;}
  };

  bool startup_cancelled() const
  {
    using Stage = chassis::RemoteControlSession::Stage;
    return session_ && (session_->stage == Stage::stopping || session_->stage == Stage::fault);
  }

  void publish_tracking_request(bool requested)
  {
    if (!tracking_request_publisher_) {return;}
    std_msgs::msg::Bool message;
    message.data = requested;
    tracking_request_publisher_->publish(message);
  }

  void accept_velocity(double x, double y, double yaw)
  {
    try {
      InternalWrite write(internal_parameter_write_);
      command_watchdog_->record_command(chassis::VelocityCommandWatchdog::Clock::now());
      if (source_handover_pending_) {controller_->stop(); return;}
      if (!arm_visual_tracking_ || arm_visual_tracking_->accept_manual_chassis_command(x, y, yaw)) {
        controller_->set_velocity(x, y, yaw);
        visual_owned_chassis_ = false;
      } else {
        controller_->set_velocity(0.0, arm_visual_tracking_->lateral_velocity(), 0.0);
      }
    } catch (const std::exception & error) {on_fault(error);}
  }

  void accept_app_control(const chassis::AppControlFrame & frame)
  {
    if (!session_ || !command_gate_ || failed_) {return;}
    const auto stamp = now().nanoseconds();
    if (frame.action == 110) {
      if (command_gate_->app_action(frame.stamp_ns, stamp, 110)) {
        session_->claim(chassis::RemoteControlSession::Source::app);
      }
      return;  // A takeover never carries motion across the ownership boundary.
    }
    const auto decision = command_gate_->app_control(frame, stamp);
    if (!decision.accepted) {return;}
    try {
      InternalWrite write(internal_parameter_write_);
      command_watchdog_->record_command(chassis::VelocityCommandWatchdog::Clock::now());
      if (decision.mode_changed) {
        controller_->stop();
        pending_remote_step_.reset();
        remote_tracking_requested_ = false;
        publish_tracking_request(false);
        if (app_arm_control_) {app_arm_control_->stop();}
        if (arm_position_remote_) {arm_position_remote_->command(0);}
        source_handover_pending_ = bool(arm_);
      }
      if (decision.action == 109 && app_arm_control_) {app_arm_control_->stop();}
      if (!decision.motion_allowed || source_handover_pending_) {
        controller_->stop();
        if (app_arm_control_ && frame.mode == chassis::AppControlMode::arm && frame.neutral()) {
          // Preserve an observed neutral while the measured halt completes.
          // Motion still waits for the handover and a fresh post-halt frame.
          app_arm_control_->velocity(0.0, 0.0, 0.0);
        }
        return;
      }
      if (frame.mode == chassis::AppControlMode::arm) {
        // Arm values are normalized X/Z/pitch, never chassis velocities.
        controller_->set_velocity(0.0, 0.0, 0.0);
        if (app_arm_control_) {
          if (decision.action != 109) {
            app_arm_control_->velocity(frame.x, frame.y, frame.yaw);
          }
          if (decision.action != 0) {app_arm_control_->command(decision.action);}
        }
      } else {
        accept_velocity(frame.x, frame.y, frame.yaw);
        if (decision.action != 0 && app_arm_control_) {
          app_arm_control_->command(decision.action);
        }
      }
    } catch (const std::exception & error) {on_fault(error);}
  }
  // 只在启动阶段传递，先完成全部参数校验，再创建硬件对象。
  struct StartupConfig
  {
    chassis::SteeringConfigFile chassis_config;
    double forward_rpm;
    chassis::ethercat::Options ethercat_options;
    chassis::ArmMotionConfig arm_motion;
    std::array<std::uint16_t, 3> ethercat_positions;
    std::optional<std::uint16_t> fourth_position;
  };

  StartupConfig read_startup_config(bool enable_arm)
  {
    rcl_interfaces::msg::ParameterDescriptor app_descriptor;
    app_descriptor.read_only = true;
    app_interface_enabled_ = declare_parameter<bool>("app_interface_enabled", bool(session_), app_descriptor);
    relay_enabled_ = declare_parameter<bool>("relay_enabled", false, app_descriptor);
    if (relay_enabled_) {
      const int address = declare_parameter<int>("relay_address", 1, app_descriptor);
      const int channel = declare_parameter<int>("relay_output_channel", 2, app_descriptor);
      if (address < 1 || address > 255 || channel < 1 || channel > 64) {
        throw std::invalid_argument("relay_address must be 1..255 and relay_output_channel 1..64");
      }
      relay_config_.address = static_cast<unsigned>(address);
      relay_config_.channel = static_cast<unsigned>(channel);
      relay_config_.active_high = declare_parameter<bool>("relay_active_high", true, app_descriptor);
      if (chassis::BxiPciTransport::runtime_used()) {
        throw std::runtime_error("CAN6 rate must be selected before BXI initialization; restart controller");
      }
      chassis::bxi_timing::configure_can6_bitrate(1000000);
    }
    if (session_ && !app_interface_enabled_) {
      throw std::invalid_argument("shared control requires app_interface_enabled=true");
    }
    rcl_interfaces::msg::ParameterDescriptor timeout_descriptor;
    timeout_descriptor.read_only = true;
    timeout_descriptor.description = "Stop chassis after this many seconds without cmd_vel_car";
    command_timeout_ = declare_parameter<double>("command_timeout_sec", 0.3, timeout_descriptor);
    command_watchdog_ = std::make_unique<chassis::VelocityCommandWatchdog>(
      std::chrono::duration<double>(command_timeout_));
    // 默认配置不依赖工作目录，仍可通过 ROS 参数指定其他文件。
    const auto config_directory = default_config_directory();
    const auto path = declare_parameter<std::string>(
      "steering_config_file", (config_directory / "steering.yaml").string());
    if (path.empty()) {
      throw std::invalid_argument("set steering_config_file to a chassis YAML configuration file");
    }
    // 校准后的自动前进速度，单位为行进轮输出轴 RPM；0 表示停车等待外部指令。
    const double forward_rpm = declare_parameter<double>("post_calibration_rpm", 0.0);
    // 读取机械参数、CAN 映射和控制策略；启动调参时可能向此 YAML 回填电机参数。
    auto config = chassis::load_steering_config(path);
    // 拒绝 NaN、无穷值、负值及超过配置上限的启动速度。
    if (!std::isfinite(forward_rpm) || forward_rpm < 0.0 ||
      forward_rpm > config.drive_max_output_rpm)
    {
      throw std::invalid_argument("post_calibration_rpm must be in [0, drive_max_output_rpm]");
    }
    chassis::ethercat::Options ethercat_options;
    chassis::ArmMotionConfig arm_motion;
    std::array<std::uint16_t, 3> ethercat_positions{};
    std::optional<std::uint16_t> fourth_position;
    if (enable_arm) {
      rcl_interfaces::msg::ParameterDescriptor topology_descriptor;
      topology_descriptor.read_only = true;
      ethercat_options.interface = declare_parameter<std::string>(
        "yiyou_ethercat_interface", "", topology_descriptor);
      const auto positions = declare_parameter<std::vector<std::int64_t>>(
        "yiyou_ethercat_slaves", {1, 2, 3, 4}, topology_descriptor);
      if ((positions.size() != 3 && positions.size() != 4) ||
        std::set<std::int64_t>(positions.begin(), positions.end()).size() != positions.size())
      {
        throw std::invalid_argument(
                "set three or four unique yiyou_ethercat_slaves; fourth entry uses PP jog");
      }
      for (std::size_t i = 0; i < positions.size(); ++i) {
        if (positions[i] < 1 || positions[i] > 199) {
          throw std::invalid_argument("yiyou_ethercat_slaves must be in 1..199");
        }
        ethercat_options.slaves.push_back(static_cast<std::uint16_t>(positions[i]));
        if (i < 3) {ethercat_positions[i] = static_cast<std::uint16_t>(positions[i]);}
      }
      if (positions.size() == 4) {fourth_position = ethercat_options.slaves[3];}
      chassis::ethercat::validate_selection(ethercat_options);
      chassis::declare_legacy_arm_current_parameter(*this);
      arm_motion = chassis::declare_arm_motion_parameters(*this, config_directory);
      arm_motion.ethercat_interface = ethercat_options.interface;
      ethercat_options.monitor_torque = bool(torque_log_);
      arm_motion.monitor_torque = bool(torque_log_);
      chassis::declare_arm_pose_parameter(*this);
      chassis::declare_arm_tracking_parameter(*this);
      rcl_interfaces::msg::ParameterDescriptor limit_descriptor;
      limit_descriptor.read_only = true;
      limit_descriptor.description =
        "Absolute EtherCAT arm speed that cuts shared motor power (RPM)";
      declare_parameter<double>("can3_speed_limit_rpm", 60.0, limit_descriptor, true);
      rcl_interfaces::msg::ParameterDescriptor hold_descriptor;
      hold_descriptor.read_only = true;
      hold_descriptor.description = "Legacy joint jog disabled; use yiyou_arm_target [x,z,pitch]";
      for (unsigned id = 1; id <= 3; ++id) {
        const auto name = "can3_motor_" + std::to_string(id) + "_angle_deg";
        if (declare_parameter<double>(name, 0.0, hold_descriptor) != 0.0) {
          throw std::invalid_argument("CAN3 startup angle must be zero: " + name);
        }
      }
    }
    return {
      std::move(config), forward_rpm, std::move(ethercat_options),
      std::move(arm_motion), ethercat_positions, fourth_position};
  }

  // 启动顺序：底盘上电 -> 机械臂初始化/回零 -> 底盘校准。
  void initialize_hardware(
    const StartupConfig & startup, bool enable_arm,
    const std::function<void()> & poll_remote)
  {
    // 节点独占控制器；节点销毁时自动析构控制器，执行停机、失能和断电清理。
    controller_ = std::make_unique<chassis::ChassisController>(startup.chassis_config);
    // 初始化和校准中的耗时循环通过此回调检查 ROS 是否仍在运行，支持取消启动。
    const auto keep_running = [this, poll_remote] {
        // 统一入口在启动等待和校准期间继续处理遥控器，包括急停按钮。
        if (poll_remote) {poll_remote();}
        if (!rclcpp::ok() || startup_cancelled()) {return false;}
        // 回零与校准会同步阻塞，取消检查同时推进机械臂反馈和指令握手。
        if (arm_) {
          arm_->update();
          log_arm_feedback();
        }
        return true;
      };
    // 打开 CAN0/1/2、建立电机对象并上电；校准完成前禁止行进轮运动。
    controller_->initialize(keep_running);
    if (poll_remote) {poll_remote();}
    if (!rclcpp::ok() || startup_cancelled()) {throw std::runtime_error("startup cancelled");}
    if (enable_arm) {
      // 四轴均使用 PP，第四轴独立位置点动；共享电源已由底盘开启。
      auto master = std::make_shared<chassis::ethercat::Master>(startup.ethercat_options);
      if (poll_remote) {poll_remote();}
      if (!rclcpp::ok() || startup_cancelled()) {throw std::runtime_error("startup cancelled");}
      // BXI remains the shared power switch; no Yiyou CAN frames are sent.
      auto power = std::make_shared<chassis::BxiPciTransport>(3);
      arm_ = std::make_unique<chassis::ArmController>(
        master, startup.ethercat_positions, [power] {
          const auto result = power->set_motor_power(false);
          if (!result) {throw std::runtime_error(result.error().message);}
        }, startup.fourth_position, startup.arm_motion);
      if (torque_log_) {
        arm_->set_torque_observer([log = torque_log_](const chassis::ArmTorqueSample & sample) {
          log->record(sample);
        });
      }
      arm_->initialize();
      if (!rclcpp::ok() || startup_cancelled()) {throw std::runtime_error("startup cancelled");}
      arm_->update();
      RCLCPP_INFO(
        get_logger(),
        "EtherCAT: Yiyou axes 1-3 initially hold startup positions in PP mode; "
        "no zero saving or EEPROM writes");
      if (arm_->has_fourth_axis()) {
        RCLCPP_INFO(
          get_logger(), "EtherCAT: Yiyou axis 4 uses PP at slave %u; speed=%.3f RPM; "
          "software limits [0,1200] degrees; release holds; timeout 300 ms",
          static_cast<unsigned>(*startup.fourth_position), startup.arm_motion.fourth_speed_rpm);
      }
      RCLCPP_INFO(
        get_logger(), "EtherCAT protection: abs(speed) >= 60 RPM cuts shared motor power; "
        "fault stays latched until restart; EtherCAT feedback failures also stop control");
      log_arm_feedback();
      if (startup.arm_motion.home_on_start) {
        RCLCPP_INFO(
          get_logger(), "ARM_HOME_START target_joint_degrees=[%.3f,%.3f,%.3f] speed_rpm=%.3f; "
          "waiting for configured home before chassis calibration (timeout 300 s)",
          startup.arm_motion.home_joints[0] * 180.0 / arm::kPi,
          startup.arm_motion.home_joints[1] * 180.0 / arm::kPi,
          startup.arm_motion.home_joints[2] * 180.0 / arm::kPi,
          startup.arm_motion.speed_rpm);
        arm_->return_to_home(keep_running);
        RCLCPP_INFO(get_logger(), "ARM_HOME_DONE: all three axes reached configured home");
      } else {
        RCLCPP_INFO(get_logger(), "ARM_HOME_DISABLED: holding startup positions");
      }
    } else {
      RCLCPP_INFO(get_logger(), "Chassis only: CAN0/1/2 enabled; CAN3 arm control disabled");
    }
    RCLCPP_INFO(
      get_logger(),
      "CAN0: calibrating all four steering axes concurrently; travel axes inhibited");
    // 四个转向轴并行搜索机械限位并独立回中；确认全部到位后才使能行进轮。
    // 此调用同步执行，完成前尚未创建速度订阅和控制定时器。
    controller_->calibrate(keep_running);
    if (!rclcpp::ok() || startup_cancelled()) {throw std::runtime_error("startup cancelled");}
    // Begin relay exchanges after blocking homing/calibration has finished.
    if (relay_enabled_) {
      auto transport = std::make_shared<chassis::BxiPciTransport>(6);
      relay_ = std::make_unique<chassis::CanRelay>(transport, relay_config_);
      RCLCPP_INFO(get_logger(), "CAN_RELAY protocol=pinquanwei bus=6 bitrate=1000000 address=%u channel=Y%u button=7/RB",
        relay_config_.address, relay_config_.channel);
      relay_->start();
    }
    // 正值进入持续前进状态，后续速度消息可替换该指令；默认 0 不自动前进。
    if (startup.forward_rpm > 0.0) {controller_->forward(startup.forward_rpm);}
    if (startup.forward_rpm > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; CAN1/2 ISWV travel forward %.3f output RPM; waiting for /cmd_vel_car",
        startup.forward_rpm);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "calibration complete; travel wheels stopped; waiting for /cmd_vel_car");
    }
    // 输出运动模式和转向未对齐时的行进策略，便于核对当前配置。
    RCLCPP_INFO(
      get_logger(),
      "steering control: swerve; alignment policy=%s",
      startup.chassis_config.drive_alignment_policy.c_str());
    // 解析逐轮参数；未配置 modules 时由全局机械参数生成。
    // 轮序 FL/RL/RR/FR 对应前左、后左、后右、前右。
    const auto modules = chassis::resolved_modules(startup.chassis_config);
    const auto wheel_speed_limit = [](const chassis::SwerveModuleConfig & module) {
        return module.drive_max_motor_rpm / module.drive_gear_ratio *
               (2.0 * std::acos(-1.0) * module.wheel_radius_m) / 60.0;
      };
    RCLCPP_INFO(
      get_logger(), "travel velocity mode=%d; wheel speed limits [FL,RL,RR,FR]: "
      "[%.4f, %.4f, %.4f, %.4f] m/s; normal stop ramp=%s",
      startup.chassis_config.drive_velocity_mode,
      wheel_speed_limit(modules[0]), wheel_speed_limit(modules[1]),
      wheel_speed_limit(modules[2]), wheel_speed_limit(modules[3]),
      startup.chassis_config.drive_velocity_mode == -3 ? "software" : "drive");
    RCLCPP_INFO(
      get_logger(), "steering target rate limits [FL,RL,RR,FR]: [%.3f, %.3f, %.3f, %.3f] rad/s",
      modules[0].steering_rate_limit_rad_s, modules[1].steering_rate_limit_rad_s,
      modules[2].steering_rate_limit_rad_s, modules[3].steering_rate_limit_rad_s);
  }

  // 校准完成后才接收控制命令，并启动 50 Hz 调度。
  void setup_ros_interfaces()
  {
    if (session_) {
      command_gate_ = std::make_unique<chassis::SharedCommandGate>(*session_);
      command_gate_->reset(now().nanoseconds());
      declare_parameter<std::int64_t>("shared_control_generation", session_->generation);
      declare_parameter<std::int64_t>("shared_control_request_generation", 0);
      tracking_request_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "/arm_tracking_requested", rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.25)));
    }
    // 末端目标在校准完成后开放；旧单轴调角参数仍为只读。
    if (arm_) {
      arm_position_remote_ = std::make_unique<chassis::ArmPositionRemote>(
        *this, *arm_, !session_);
      std::function<bool()> external_tracking_allowed;
      if (session_) {
        external_tracking_allowed = [this] {
            return session_->owner == chassis::RemoteControlSession::Source::remote &&
              session_->stage == chassis::RemoteControlSession::Stage::running &&
              !source_handover_pending_ && remote_tracking_requested_;
          };
      }
      arm_visual_tracking_ = std::make_unique<chassis::ArmVisualTracking>(
        *this, *arm_, [this](const std::exception & error) {on_fault(error);},
        std::move(external_tracking_allowed), session_ ?
        std::function<std::uint64_t()>([this] {return session_->generation;}) :
        std::function<std::uint64_t()>());
      arm_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/arm_pose", 10);
      RCLCPP_INFO(
        get_logger(),
        "EtherCAT arm IK ready: set yiyou_arm_target=[x_m,z_m,pitch_rad]; feedback on /arm_pose");
    }
    RCLCPP_INFO(get_logger(), "Chassis command watchdog: %.3f s", command_timeout_);
    // 接收车体速度指令，只保留最新一条，避免断连后旧速度排队重放。
    // linear.x/y 为前后/横向速度（m/s），angular.z 为偏航角速度（rad/s）。
    // 控制器保持最后一条指令；停止发布超过 command_timeout_sec 后强制停车。
    if (session_) {
      app_control_subscription_ = create_subscription<std_msgs::msg::String>(
        "/car2/app_control", rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.1)),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          const auto frame = chassis::parse_app_control_frame(message->data);
          if (frame) {accept_app_control(*frame);}
        });
      // Legacy chassis control remains usable until the first unified input.
      // Arm movement requires the unified mode and its own neutral/lease stream.
      app_velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/car2/app_cmd_vel", rclcpp::QoS(1).lifespan(rclcpp::Duration::from_seconds(0.3)),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
          const auto & stamp = message->header.stamp;
          if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {return;}
          const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
          const auto & v = message->twist;
          if (command_gate_->app_velocity(stamp_ns, now().nanoseconds(), v.linear.x, v.linear.y, v.angular.z)) {
            accept_velocity(v.linear.x, v.linear.y, v.angular.z);
          }
        });
    } else {
      subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_car", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          accept_velocity(message->linear.x, message->linear.y, message->angular.z);
        });
    }
    if (app_interface_enabled_) {
      if (arm_) {
        app_arm_control_ = std::make_unique<chassis::AppArmControl>(
          *this, *arm_, *arm_visual_tracking_,
          [this](chassis::AppArmControl::Clock::time_point now) {
            return command_watchdog_->has_fresh_command(now);
          });
      }
      std::function<void(int, std::int64_t)> shared_command;
      if (session_) {
        shared_command = [this](int command, std::int64_t stamp) {
            if (!command_gate_->app_action(stamp, now().nanoseconds(), command)) {return;}
            if (command == 110) {
              session_->claim(chassis::RemoteControlSession::Source::app);
              return;
            }
            try {
              InternalWrite write(internal_parameter_write_);
              if (app_arm_control_) {app_arm_control_->command(command);}
            } catch (const std::exception & error) {on_fault(error);}
          };
      }
      app_interface_ = std::make_unique<chassis::AppInterface>(
        *this, bool(arm_), arm_ && arm_->has_fourth_axis(),
        [this](int command) {
          try {
            if (app_arm_control_) {app_arm_control_->command(command);}
          } catch (const std::exception & error) {on_fault(error);}
        }, std::move(shared_command), [this] {
          return session_ ? chassis::source_name(session_->owner) : "none";
        });
    }
    if (session_) {
      shared_parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          if (internal_parameter_write_) {return result;}
          bool protected_parameter = false;
          bool tracking_only = true;
          std::int64_t generation = -1;
          for (const auto & p : parameters) {
            const auto & name = p.get_name();
            const bool tracking = name == chassis::kArmTrackingEnabledParameter ||
              name == chassis::kArmTrackingXEnabledParameter || name == chassis::kArmTrackingGoalEnabledParameter;
            if (name == "shared_control_request_generation") {
              if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {generation = p.as_int();}
            } else if (!tracking) {tracking_only = false;}
            protected_parameter = protected_parameter || tracking || name == chassis::kYiyouPoseParameter ||
              name == "shared_control_generation" || name == "shared_control_request_generation";
          }
          if (protected_parameter && !(tracking_only && !source_handover_pending_ && remote_tracking_requested_ &&
              session_->owner == chassis::RemoteControlSession::Source::remote && generation >= 0 &&
              static_cast<std::uint64_t>(generation) == session_->generation))
          {
            result.successful = false;
            result.reason = "shared control ownership changed; use the active source command interface";
          }
          return result;
        });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {update_control();});
  }

  // 每周期先检查速度指令是否断流，再继续机械臂、底盘和反馈处理。
  void update_control()
  {
    if (session_ && session_->stage != chassis::RemoteControlSession::Stage::running) {return;}
    try {
      InternalWrite write(internal_parameter_write_);
      update_relay();
      if (command_watchdog_->consume_timeout(chassis::VelocityCommandWatchdog::Clock::now())) {
        RCLCPP_WARN(
          get_logger(),
          "/cmd_vel_car silent for %.3f s; forcing zero chassis velocity",
          command_timeout_);
        if (session_) {session_->release(session_->owner);}
        if (failed_) {return;}
        if (arm_visual_tracking_ && arm_visual_tracking_->owns_chassis()) {
          arm_visual_tracking_->cancel_tracking("chassis command heartbeat expired");
        }
        controller_->stop();
        if (app_arm_control_) {app_arm_control_->stop();}
      }
      if (arm_) {
        if (app_arm_control_) {app_arm_control_->before_update();}
        arm_position_remote_->update();
        arm_visual_tracking_->before_update();
        // Timeout logging is one-shot; the motion gate must remain closed until
        // a new heartbeat arrives, including after an explicit tracking re-enable.
        if (arm_visual_tracking_->owns_chassis() &&
          !command_watchdog_->has_fresh_command(chassis::VelocityCommandWatchdog::Clock::now()))
        {
          arm_visual_tracking_->cancel_tracking("no fresh chassis command heartbeat");
        }
        arm_->update();
        arm_visual_tracking_->after_update();
        if (source_handover_pending_ && arm_visual_tracking_->complete_source_handover()) {
          source_handover_pending_ = false;
          if (remote_tracking_requested_) {publish_tracking_request(true);}
          if (pending_remote_step_) {
            const auto step = *pending_remote_step_;
            pending_remote_step_.reset();
            const auto now = std::chrono::steady_clock::now();
            if (now < step.deadline && command_watchdog_->has_fresh_command(now)) {
              remote_arm_step(step.generation, step.axis, step.delta);
              if (failed_) {return;}
            }
          }
        }
        if (app_arm_control_) {app_arm_control_->after_update();}
        const bool visual_owns_chassis = arm_visual_tracking_->owns_chassis();
        if (command_gate_ && command_gate_->app_arm_mode()) {
          controller_->set_velocity(0.0, 0.0, 0.0);
        } else if (visual_owns_chassis) {
          controller_->set_velocity(0.0, arm_visual_tracking_->lateral_velocity(), 0.0);
        } else if (visual_owned_chassis_) {
          // Do not retain the last visual velocity across the manual handover.
          controller_->stop();
        }
        visual_owned_chassis_ = visual_owns_chassis;
        const auto pose = arm_->actual_pose();
        geometry_msgs::msg::PoseStamped feedback;
        feedback.header.stamp = get_clock()->now();
        feedback.header.frame_id = "arm_shoulder";
        feedback.pose.position.x = pose.x;
        feedback.pose.position.z = pose.z;
        feedback.pose.orientation.y = -std::sin(pose.pitch / 2.0);
        feedback.pose.orientation.w = std::cos(pose.pitch / 2.0);
        arm_pose_publisher_->publish(feedback);
      }
      controller_->update();
      log_arm_feedback();
      // 消费最近生成的一秒零速反馈报告；无新报告时为空，不额外读取 CAN。
      const auto report = controller_->take_stop_diagnostic();
      if (!report.empty()) {RCLCPP_INFO(get_logger(), "%s", report.c_str());}
      if (app_interface_) {app_interface_->publish_ready();}
    } catch (const std::exception & error) {on_fault(error);}
  }

  void log_arm_feedback()
  {
    if (!arm_) {return;}
    const auto report = arm_->take_feedback_diagnostic();
    if (!report.empty()) {RCLCPP_INFO(get_logger(), "%s", report.c_str());}
  }

  void on_fault(const std::exception & error)
  {
    if (failed_) {return;}
    // 回调异常统一在这里记录并结束 ROS 运行；硬件清理由控制器故障路径和析构负责。
    RCLCPP_ERROR(get_logger(), "chassis stopped: %s", error.what());
    failed_ = true;
    if (arm_ && !arm_->fault_reason().empty()) {
      RCLCPP_ERROR(get_logger(), "%s", arm_->fault_reason().c_str());
    }
    if (timer_) {timer_->cancel();}
    try {
      if (arm_) {arm_->stop();}
    } catch (const std::exception & stop_error) {
      RCLCPP_ERROR(get_logger(), "EtherCAT stop failed: %s", stop_error.what());
    }
    if (session_) {session_->latch_fault(error.what());}
    else {rclcpp::shutdown();}
  }

  // 保存对象所有权，使控制器、订阅和定时器在节点运行期间持续有效。
  std::shared_ptr<chassis::ArmTorqueLog> torque_log_;
  std::shared_ptr<chassis::RemoteControlSession> session_;
  std::shared_ptr<chassis::HardwareOwner> hardware_owner_;
  std::unique_ptr<chassis::ChassisController> controller_;
  // Relay detaches from CAN6 before the chassis releases the shared board.
  // Its destructor deliberately sends no output changes, matching car3.
  std::unique_ptr<chassis::CanRelay> relay_;
  chassis::CanRelay::Config relay_config_{};
  bool relay_enabled_{false};
  // 声明在 controller_ 后，正常退出时先停止意优电机，再由底盘断电。
  std::unique_ptr<chassis::ArmController> arm_;
  std::unique_ptr<chassis::ArmPositionRemote> arm_position_remote_;
  std::unique_ptr<chassis::ArmVisualTracking> arm_visual_tracking_;
  std::unique_ptr<chassis::AppArmControl> app_arm_control_;
  std::unique_ptr<chassis::AppInterface> app_interface_;
  bool app_interface_enabled_{false};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr app_velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr app_control_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_request_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr shared_parameter_callback_;
  std::unique_ptr<chassis::SharedCommandGate> command_gate_;
  struct PendingRemoteStep
  {
    std::uint64_t generation;
    unsigned axis;
    double delta;
    std::chrono::steady_clock::time_point deadline;
  };
  std::optional<PendingRemoteStep> pending_remote_step_;
  bool source_handover_pending_{false};
  bool remote_tracking_requested_{false};
  bool internal_parameter_write_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<chassis::VelocityCommandWatchdog> command_watchdog_;
  double command_timeout_{0.3};
  bool visual_owned_chassis_{false};
  bool failed_{false};
};

void print_help(const char * program_name, const std::string & mode, bool enable_arm)
{
  std::cout << "Usage: " << program_name << " --ros-args -p name:=value ...\n"
            << "Mode: " << mode << '\n'
            << "key_control/robot_control wait for button14 before chassis startup.\n"
            << "Defaults: source src/config, or installed share/chassis/config.\n"
            << "steering.yaml loads automatically; override with steering_config_file:=/path.\n"
            << "Use unqualified steering_config_file for the same chassis and remote file.\n"
            <<
    "Remote speed/acceleration defaults use its remote_ctrl section; ROS parameters override.\n"
            << "Remote parameters: start_button:=14, emergency_stop_button:=11.\n"
            << "Set start_chassis:=false for remote commands only (external chassis node).\n"
            << "chassis/chassis_only start hardware directly without a remote button.\n"
            << "chassis_only_remote waits for button14 before chassis startup.\n";
  std::cout << "Shared App/remote daemon: shared_control:=true; waits for either App start or button14.\n"
    "Shared lifecycle socket: shared_control_socket:=/run/car2/control.sock; no power while waiting.\n"
    "CAN6 Pinquanwei relay: relay_enabled:=true, 1M, address=1, Y2; button7/RB toggles ON/OFF.\n"
    "Relay startup requests OFF; disconnect/exit sends no OFF; confirmed states are logged.\n";
  if (enable_arm) {
    std::cout <<
      "yiyou_ethercat.yaml loads automatically; --params-file and -p override its values.\n"
      "Set yiyou_ethercat_interface:=enpXsY; yiyou_ethercat_slaves defaults to [1,2,3,4].\n"
      "First three entries hold startup positions in PP; fourth entry uses PP jog.\n"
      "Three entries retain PP-only operation; entries are physical slave positions.\n"
      "Fourth PP speed: yiyou_motor_4_speed_rpm (12 RPM); limits [0,1200] degrees.\n"
      "Runtime arm target: yiyou_arm_target=[x_m,z_m,pitch_rad]; feedback: /arm_pose.\n"
      "Remote buttons 1/3 select x/z/pitch; axis7 steps by 1 cm or 1 degree.\n"
      "Holding axis7 repeats every 0.1 s; neutral stops repeats; busy steps are discarded.\n"
      "Hold button0/4 for fourth-axis positive/negative position jog; release/both holds.\n"
      "Release both jog buttons before use; /arm_motor4_jog expires after 300 ms.\n"
      "Default joint zero: links rise [20,54.69,90] degrees above horizontal -X; "
      "absolute +X angles [160,125.31,90] degrees.\n"
      "yiyou_arm_home_on_start defaults true: return to configured home before chassis calibration.\n"
      "yiyou_arm_home_joint_degrees defaults [0,0,0]: calibrated joint targets, not link angles.\n"
      "Set yiyou_arm_home_on_start:=false to hold startup positions instead.\n"
      "Axes 1-3 PP speed defaults 0.6 RPM; acceleration/deceleration 1.0 RPM/s.\n"
      "Axis 4 PP acceleration/deceleration defaults 24 RPM/s; old current commands disabled.\n";
  } else {
    std::cout << "Chassis only: CAN0/1/2; no CAN3 initialization or arm motor commands.\n";
  }
  std::cout << "App gateway: app_interface_enabled:=true publishes /car2/ready and "
    "/car2/state_machine_info; CHASSIS_CONFIG_DIR selects persistent configuration.\n";
  std::cout << "Help does not open CAN devices or command motors.\n";
}

// 加载默认 YAML，同时保留用户本地参数优先于全局默认值的行为。
rclcpp::NodeOptions initialize_ros(int argc, char ** argv, bool enable_arm)
{
  const std::vector<std::string> user_arguments(argv, argv + argc);
  std::vector<std::string> arguments{argv[0]};
  if (enable_arm) {
    const auto config = default_config_directory() / "yiyou_ethercat.yaml";
    arguments.insert(arguments.end(), {"--ros-args", "--params-file", config.string(), "--"});
  }
  arguments.insert(arguments.end(), user_arguments.begin() + 1, user_arguments.end());
  std::vector<const char *> argument_pointers;
  for (const auto & argument : arguments) {
    argument_pointers.push_back(argument.c_str());
  }
  rclcpp::init(static_cast<int>(argument_pointers.size()), argument_pointers.data());
  // 用户参数作为节点本地参数再次应用，优先于全局默认配置。
  // 这样默认 YAML 的节点分组顺序不会改变用户 -p / --params-file 的覆盖行为。
  return rclcpp::NodeOptions().arguments(user_arguments);
}

// 统一管理遥控等待、节点启动和执行器，返回前释放节点及硬件。
int run_control(
  bool enable_arm, bool enable_remote, const rclcpp::NodeOptions & options,
  const std::shared_ptr<chassis::ArmTorqueLog> & torque_log)
{
  std::shared_ptr<ChassisNode> chassis_node;
  std::shared_ptr<rclcpp::Node> remote_node;
  auto session = std::make_shared<chassis::RemoteControlSession>();
  rclcpp::executors::SingleThreadedExecutor executor;
  auto lifecycle_node = std::make_shared<rclcpp::Node>("car2_control_session", options);
  session->shared_control = lifecycle_node->declare_parameter<bool>("shared_control", false);
  if (session->shared_control && !enable_remote) {
    throw std::invalid_argument("shared_control requires robot_control or chassis_only_remote");
  }
  if (enable_remote) {
    remote_node = chassis::make_remote_control(
      options, session, enable_arm, (default_config_directory() / "steering.yaml").string());
    executor.add_node(remote_node);
  }
  session->remote_relay_toggle = [&chassis_node](std::uint64_t generation) {
      if (chassis_node) {chassis_node->remote_relay_toggle(generation);}
    };
  if (session->shared_control) {
    using Stage = chassis::RemoteControlSession::Stage;
    if (!session->start_chassis) {
      throw std::invalid_argument("shared_control requires start_chassis:=true");
    }
    const auto socket_path = lifecycle_node->declare_parameter<std::string>(
      "shared_control_socket", "/run/car2/control.sock");
    chassis::SharedControlServer server(session, socket_path, enable_arm);
    const auto timer = lifecycle_node->create_wall_timer(
      std::chrono::milliseconds(20), [&server] {server.poll();});
    executor.add_node(lifecycle_node);
    session->on_owner_changed = [&chassis_node] {
        if (chassis_node) {chassis_node->handover();}
      };
    session->remote_velocity = [&chassis_node](std::uint64_t generation, double x, double y, double yaw) {
        if (chassis_node) {chassis_node->remote_velocity(generation, x, y, yaw);}
      };
    session->remote_arm_step = [&chassis_node](std::uint64_t generation, unsigned axis, double delta) {
        return chassis_node && chassis_node->remote_arm_step(generation, axis, delta);
      };
    session->remote_arm_jog = [&chassis_node](std::uint64_t generation, int direction) {
        if (chassis_node) {chassis_node->remote_arm_jog(generation, direction);}
      };
    session->remote_tracking = [&chassis_node](std::uint64_t generation, bool requested) {
        if (chassis_node) {chassis_node->remote_tracking(generation, requested);}
      };
    RCLCPP_INFO(lifecycle_node->get_logger(),
      "SHARED_CONTROL waiting: App start or remote button14; motors off; socket=%s", socket_path.c_str());
    std::optional<std::chrono::steady_clock::time_point> restart_at;
    bool shutdown_failure_reported = false;
    while (rclcpp::ok()) {
      executor.spin_once(std::chrono::milliseconds(20));
      if (session->stage == Stage::requested) {
        session->stage = Stage::starting;
        session->hardware_active = true;
        RCLCPP_INFO(lifecycle_node->get_logger(), "SHARED_CONTROL starting: source=%s",
          chassis::source_name(session->startup_source));
        try {
          // Poll lifecycle/remote during homing and steering calibration so Stop can cancel startup.
          chassis_node = std::make_shared<ChassisNode>(
            enable_arm, options, [&executor] {executor.spin_some();}, torque_log, session);
          executor.add_node(chassis_node);
          if (rclcpp::ok() && session->stage == Stage::starting) {session->mark_running();}
        } catch (const std::exception & error) {
          if (session->stage != Stage::stopping && rclcpp::ok()) {
            session->latch_fault(error.what());
            RCLCPP_ERROR(lifecycle_node->get_logger(), "SHARED_CONTROL fault: %s", error.what());
          }
        }
      }
      if (session->stage == Stage::stopping || session->stage == Stage::fault) {
        const bool was_active = session->hardware_active;
        if (chassis_node) {
          executor.remove_node(chassis_node);
          chassis_node.reset();  // Stop arm and release chassis power; process retains hardware lock.
        }
        const bool shutdown_failed = chassis::BxiPciTransport::runtime_cleanup_failed();
        const bool restart = !shutdown_failed && session->stage == Stage::stopping &&
          chassis::BxiPciTransport::runtime_used();
        if (shutdown_failed) {
          if (!shutdown_failure_reported) {
            const std::string message =
              "BXI power-off or CAN shutdown failed; motor shutdown not confirmed";
            if (session->stage == Stage::fault) {session->last_error += "; " + message;}
            else {session->latch_fault(message);}
            RCLCPP_ERROR(lifecycle_node->get_logger(),
              "SHARED_CONTROL shutdown unconfirmed: power-off or CAN shutdown failed; "
              "automatic restart blocked");
            shutdown_failure_reported = true;
          }
        } else {
          session->finish_stop(restart);
        }
        if (was_active && !shutdown_failed) {
          RCLCPP_INFO(lifecycle_node->get_logger(), "SHARED_CONTROL stopped: stage=%s; motors off",
            chassis::stage_name(session->stage));
        }
        if (restart) {
          // Let stop clients confirm hardware shutdown before systemd restarts
          // this process. Never recycle a fault into an automatically retryable state.
          restart_at = std::chrono::steady_clock::now() + std::chrono::seconds(1);
          RCLCPP_INFO(lifecycle_node->get_logger(),
            "SHARED_CONTROL restarting after clean stop: fresh CAN runtime required; "
            "service will return to waiting without powering motors");
        }
      }
      if (session->stage == Stage::fault) {restart_at.reset();}
      if (restart_at && std::chrono::steady_clock::now() >= *restart_at &&
        !server.stop_confirmation_pending()) {break;}
    }
    if (chassis_node) {
      executor.remove_node(chassis_node);
      chassis_node.reset();
    }
    session->finish_stop();
    session->on_owner_changed = {};
    session->remote_velocity = {};
    session->remote_arm_step = {};
    session->remote_arm_jog = {};
    session->remote_tracking = {};
    session->remote_relay_toggle = {};
    executor.remove_node(lifecycle_node);
    return session->stage == Stage::fault ? 1 : 0;
  }
  if (!enable_remote || session->start_chassis) {
    if (enable_remote) {
      while (rclcpp::ok() && session->stage == chassis::RemoteControlSession::Stage::waiting) {
        executor.spin_once(std::chrono::milliseconds(100));
      }
    }
    if (rclcpp::ok()) {
      // 启动请求由遥控回调提出；在回调外初始化，校准时才能继续 spin 遥控器。
      session->stage = chassis::RemoteControlSession::Stage::starting;
      chassis_node = std::make_shared<ChassisNode>(
        enable_arm, options, [&executor] {executor.spin_some();}, torque_log);
      executor.add_node(chassis_node);
      session->stage = chassis::RemoteControlSession::Stage::running;
    }
  }
  // 串行调度硬件控制与遥控器；参数请求使用异步回调，不阻塞执行器。
  if (rclcpp::ok()) {executor.spin();}
  return chassis_node && chassis_node->failed() ? 1 : 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string mode = CHASSIS_CONTROL_MODE;
  const bool enable_arm = mode != "chassis_only" && mode != "chassis_only_remote";
  const bool enable_remote = mode != "chassis" && mode != "chassis_only";
  if (argc == 2 && std::string(argv[1]) == "--help") {
    print_help(argv[0], mode, enable_arm);
    return 0;
  }

  int result = 0;
  std::string torque_directory;
  std::shared_ptr<chassis::ArmTorqueLog> torque_log;
  if (enable_arm) {
    const auto configured = std::getenv("CHASSIS_ARM_TORQUE_LOG_DIR");
    torque_directory = configured ? configured : "log/arm_torque";
    generate_torque_reports(torque_directory);
    if (!torque_directory.empty()) {
      try {
        torque_log = std::make_shared<chassis::ArmTorqueLog>(torque_directory);
      } catch (const std::exception & error) {
        std::cerr << "Arm torque logging unavailable: " << error.what() << '\n';
      }
    }
  }
  // Keep retention running while the operator is waiting to press start, too.
  std::unique_ptr<iswv::canopen::DiagnosticLog> can_log_retention;
  try {
    const auto directory = std::getenv("CHASSIS_CAN_LOG_DIR");
    can_log_retention = std::make_unique<iswv::canopen::DiagnosticLog>(
      directory ? directory : "log/can", "retention");
  } catch (const std::exception & error) {
    std::cerr << "CAN log retention unavailable: " << error.what() << '\n';
  }
  try {
    const auto options = initialize_ros(argc, argv, enable_arm);
    result = run_control(enable_arm, enable_remote, options, torque_log);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("control"), "%s", error.what());
    result = 1;
  }
  // run_control 返回或抛出时已析构节点：先停机械臂，再由底盘关闭共享电源。
  rclcpp::shutdown();
  if (torque_log) {torque_log->finish();}
  generate_torque_reports(torque_directory);
  return result;
}
