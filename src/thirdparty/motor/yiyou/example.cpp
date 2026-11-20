// 可复制到自己的 .cpp 中；不提供 main，不加入默认编译。链接 chassis_yiyou_motor。
// 调用方创建 chassis::YiyouMotor motor(master, slave_position)，站位从 1 开始。
// 同一链路共用一个 EtherCAT Master，并让它比 motor 活得更久；主站内部运行周期通信。
// 调用方串行访问 motor，周期调用 master.check_health()，负责共享电源与异常清理。
// PV/PP/CST 初始化择一调用，切换模式或重新初始化前先 yiyou_stop；失败均向上传递异常。

#include "motor/yiyou/motor.h"

#include <chrono>
#include <thread>

struct YiyouFeedback
{
  std::int32_t position_pulses;       // 绝对编码器位置，pulse。
  std::int32_t velocity_pulses_s;     // 编码器速度，pulse/s。
  std::int16_t current_permille;     // 额定电流千分比，1000 表示额定电流。
};

void yiyou_enable_velocity(
  chassis::YiyouMotor & motor, double acceleration_rpm_s, double deceleration_rpm_s)
// 输入：尚未使能的 motor；输出轴加减速度（RPM/s，有限且大于零）。
// 输出：无返回值，读取编码器/减速比后使能 PV 模式，初始目标速度为零。
//       参数、单位换算、通信或使能失败抛异常；仅在启动/重新配置时调用。
{
  motor.configure_rpm_units();
  motor.enable_rpm(acceleration_rpm_s, deceleration_rpm_s);
}

void yiyou_set_velocity(chassis::YiyouMotor & motor, double output_rpm)
// 输入：已通过 yiyou_enable_velocity 初始化的 motor；输出轴目标速度（RPM）。
// 输出：无返回值，写入目标但不等待达到目标；无效值、换算或通信失败抛异常。
{
  motor.set_velocity_rpm(output_rpm);
}

void yiyou_enable_position(
  chassis::YiyouMotor & motor, std::uint32_t velocity_pulses_s,
  std::uint32_t acceleration_pulses_s2, std::uint32_t deceleration_pulses_s2)
// 输入：尚未使能的 motor；位置模式速度（pulse/s）、加减速度（pulse/s²），均大于零。
// 输出：无返回值，使能 PP 模式并保持当前位置；参数、通信或使能失败抛异常。
{
  motor.enable_position(velocity_pulses_s, acceleration_pulses_s2, deceleration_pulses_s2);
}

void yiyou_set_position(chassis::YiyouMotor & motor, std::int32_t position_pulses)
// 输入：已使能 PP 模式且没有未完成命令握手的 motor；绝对目标位置（pulse）。
// 输出：无返回值；同步等待目标确认握手，不等待到位，不能作为无阻塞周期函数。
//       状态、通信或握手超时抛异常；不自动检查应用的机械限位。
{
  motor.move_to_position(position_pulses);
}

bool yiyou_position_reached(chassis::YiyouMotor & motor, std::uint32_t tolerance_pulses)
// 输入：已使能 PP 模式的 motor；允许的位置误差（pulse）。
// 输出：最近一次握手成功的目标已到达且误差在容差内返回 true，否则 false。
//       无成功目标也返回 false；状态或通信失败抛异常。
{
  return motor.position_reached(tolerance_pulses);
}

void yiyou_enable_current(chassis::YiyouMotor & motor)
// 输入：尚未使能的 motor；创建主站时必须已在 Options::cst_slave 中选择该站位。
// 输出：无返回值，使能 CST 模式，初始电流为零；通信、状态或使能失败抛异常。
//       零电流不保持位置，也不保证机械臂静止。
{
  motor.enable_cst();
}

void yiyou_set_current(chassis::YiyouMotor & motor, std::int16_t current_permille)
// 输入：已使能 CST 模式的 motor；目标电流为额定电流千分比，范围 [-1000,1000]。
// 输出：无返回值，检查 CST 状态并写入电流目标；越界、状态或通信失败抛异常。
{
  motor.set_current_permille(current_permille);
}

YiyouFeedback yiyou_feedback(chassis::YiyouMotor & motor)
// 输入：已使能 PV、PP 或 CST 模式的 motor。
// 输出：绝对位置（pulse）、速度（pulse/s）、电流（额定电流千分比）。
//       三个值依次读取，非同一时刻快照；通信失败或读取速度时未使能会抛异常。
{
  return {motor.actual_position(), motor.actual_velocity(), motor.actual_current_permille()};
}

void yiyou_stop(chassis::YiyouMotor & motor)
// 输入：需要停止的 motor。
// 输出：无返回值，按模式停止/清零并失能，不关闭共享电源；失败抛异常。
//       PV/PP 内部等待停稳，有超时；CST 清零失能，不等待机械停稳。
{
  motor.stop();
}

// 速度调用片段：yiyou_enable_velocity(motor, acceleration_rpm_s, deceleration_rpm_s);
// 运行中：yiyou_set_velocity(motor, target_rpm); const auto state = yiyou_feedback(motor);
// 位置/电流模式改用各自的 enable 和 set 函数；退出或异常清理时显式 yiyou_stop(motor)。

void example_yiyou_control(chassis::YiyouMotor & motor)
// 输入：已连接且尚未使能的 motor；通信参数由调用方预先配置。
// 输出：无返回值，以输出轴 1 RPM 为目标运行约 1 秒后停止并失能。
//       失败抛异常；异常时尽力停止并保留原始异常，调用方负责后续清理。
{
  try {
    yiyou_enable_velocity(motor, 10.0, 10.0);  // 加减速度：输出轴 RPM/s。
    yiyou_set_velocity(motor, 1.0);           // 目标速度：输出轴 RPM。
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } catch (...) {
    try {
      yiyou_stop(motor);
    } catch (...) {
      // 保留原始异常；停止失败时由上层处理断电。
    }
    throw;
  }
  yiyou_stop(motor);
}

// 调用：example_yiyou_control(motor);
