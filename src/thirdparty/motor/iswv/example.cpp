// 可复制到自己的 .cpp 中；不提供 main，不加入默认编译。链接 chassis_iswv_motor。
// 调用方创建 chassis::IswvDrive motor(master, config)，config 含 Node-ID、编码器和减速比。
// 同一 CAN 总线共用一个 CanopenMaster；transport、master 比 motor 活得更久。
// 共享电源和调用周期由应用管理；下列函数异常向上传递，调用方负责停车与清理。

#include "motor/iswv/motor.h"

#include <chrono>
#include <thread>

void iswv_enable_velocity(
  chassis::IswvDrive & motor, double acceleration_rpm_s, double deceleration_rpm_s)
// 输入：motor；输出轴加减速度（RPM/s，有限且非负）。启动或重新配置时调用一次。
// 输出：无返回值，完成 PV 模式配置与使能，初始目标为零；不保证电机已停稳。
//       参数无效或通信/状态失败抛异常；改变配置前先调用 iswv_stop。
{
  motor.enable_rpm(acceleration_rpm_s, deceleration_rpm_s);
}

void iswv_set_velocity(chassis::IswvDrive & motor, double output_rpm)
// 输入：已使能 PV 模式的 motor；有限的减速器输出轴目标速度（RPM）。
// 输出：无返回值，通过 SDO 写入目标速度，不等待实际达到目标；失败抛异常。
//       运行中可以反复调用，不重复初始化电机。
{
  motor.set_velocity_rpm(output_rpm);
}

chassis::IswvDrive::Feedback iswv_feedback(chassis::IswvDrive & motor)
// 输入：已使能 PV 模式并开始接收 TPDO 的 motor。
// 输出：output_rpm 为输出轴 RPM；position_inc 为编码器计数；current_raw 为原始电流值。
//       读取最近 PDO 缓存；反馈缺失、超过 250 ms 或驱动状态异常时抛异常。
{
  return motor.actual_feedback_pdo();
}

void iswv_stop(chassis::IswvDrive & motor)
// 输入：需要停止的 motor。
// 输出：无返回值，尝试写零速度、快速停止并失能；不关闭共享电源。
//       失败时仍尝试后续停止步骤，最后抛异常；返回不表示已确认机械停稳。
{
  motor.stop();
}

// 调用片段：iswv_enable_velocity(motor, acceleration_rpm_s, deceleration_rpm_s);
// 运行中：iswv_set_velocity(motor, target_rpm); const auto state = iswv_feedback(motor);
// 退出或异常清理时：iswv_stop(motor)，再释放电机/主站并处理共享电源。

void example_iswv_control(chassis::IswvDrive & motor)
// 输入：已连接且尚未使能的 motor；通信参数由调用方预先配置。
// 输出：无返回值，以输出轴 1 RPM 为目标运行约 1 秒后停止并失能。
//       失败抛异常；异常时尽力停止并保留原始异常，调用方负责后续清理。
{
  try {
    iswv_enable_velocity(motor, 10.0, 10.0);  // 加减速度：输出轴 RPM/s。
    iswv_set_velocity(motor, 1.0);           // 目标速度：输出轴 RPM。
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } catch (...) {
    try {
      iswv_stop(motor);
    } catch (...) {
      // 保留原始异常；停止失败时由上层处理断电。
    }
    throw;
  }
  iswv_stop(motor);
}

// 调用：example_iswv_control(motor);
