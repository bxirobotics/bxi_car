// 可复制到自己的 .cpp 中；不提供 main，不加入默认编译。链接 chassis_bxi_motor。
// 调用方创建并长期持有 transport 和 bxi::Motor motor(transport, command_id, options)；
// transport 比 motor 活得更久。共享电源、收帧路由、命令发送周期由调用方管理。
// 以下 Result 先判断成功，再读取 value() 或 error()；发送成功不等于设备确认。

#include "motor/bxi/motor.h"

#include <chrono>
#include <thread>

iswv::Result<void> bxi_enable(bxi::Motor & motor)
// 输入：已连接 transport 的电机对象；电源与 CAN 参数由调用方配置。
// 输出：进入 MIT 模式帧的发送结果；失败信息在 error()，不等待设备确认。
{
  return motor.enter_motor_mode();
}

iswv::Result<void> bxi_set_target(bxi::Motor & motor, const bxi::Command & target)
// 输入：已进入 MIT 模式的 motor；target 含 position(rad)、velocity(rad/s)、
//       torque(Nm)、kp、kd，以及与设备一致的 ranges 编码范围。
// 输出：校验和发送结果；无效/越界参数、传输失败由 error() 返回，不自动使能。
{
  return motor.command(target);
}

iswv::Result<bxi::Feedback> bxi_feedback(
  const iswv::CanFrame & frame, const bxi::EncodingRanges & ranges)
// 输入：调用方按设备 master_id 筛选出的接收帧、与发送端相同的编码范围。
// 输出：成功时 value() 含 position(rad)、velocity(rad/s) 和原始 prefix。
//       无效帧/范围返回失败；这里只解码已有帧，不主动查询电机或等待反馈。
{
  return bxi::decode_feedback(frame, ranges);
}

iswv::Result<void> bxi_disable(bxi::Motor & motor)
// 输入：待退出 MIT 模式的电机对象。
// 输出：退出帧的发送结果；失败信息在 error()，不确认停稳，也不关闭共享电源。
{
  return motor.exit_motor_mode();
}

// 调用片段：const auto enabled = bxi_enable(motor);
// enabled 成功后：const auto sent = bxi_set_target(motor, target);
// 收到对应反馈帧后：const auto state = bxi_feedback(frame, target.ranges);
// 退出时显式调用 bxi_disable(motor)，检查结果并由应用处理断电。

iswv::Result<void> example_bxi_control(
  bxi::Motor & motor, const bxi::EncodingRanges & ranges)
// 输入：已连接的 motor、与实际电机型号一致的编码范围 ranges。
// 输出：以 0.5 rad/s 为目标，每 10 ms 发一次命令，约 1 秒后退出 MIT 模式。
//       返回发送结果，不保证实际速度或停稳；出错仍尝试退出，优先返回原始错误。
{
  bxi::Command target;
  target.ranges = ranges;
  target.velocity = 0.5F;  // rad/s，可按需要修改。
  target.kd = 0.2F;        // 速度反馈增益；kp、前馈力矩保持为零。

  auto result = bxi_enable(motor);
  for (int i = 0; result && i < 100; ++i) {
    result = bxi_set_target(motor, target);
    if (result) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  const auto stopped = bxi_disable(motor);
  return result ? stopped : result;
}

// 调用：const auto result = example_bxi_control(
//   motor, bxi::encoding_ranges(bxi::Model::BXI5014_19));  // 型号按实际设备选择。
