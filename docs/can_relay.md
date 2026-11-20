# CAN6 品全微继电器、气泵与 button7 / RB

当前模块使用上海品全微《数字量输出系列使用手册（CAN版）V3.0》的自定义协议，
替换旧 car3/Zhongsheng 继电器协议。气泵连接第 2 路（Y2/OUT2），地址 1，
COM/NO 接线、线圈得电表示开启。模块出厂为 **250K**，完成下述一次性配置后，
主程序按 **1M** 通信。

## 一次性将新模块从 250K 改为 1M

停止主控制服务，确认运动已停止、共享电源已关闭，再独占使用配置工具。
此工具只查询信息和输出；指定 `--set-1m` 时仅写波特率寄存器，不写任何线圈。
`--power-on` 会开启板卡共享供电，等待 4 秒；退出时关闭共享供电，不驱动电机。
外部独立供电的继电器需要单独断电。

```bash
sudo systemctl stop ros_elf_launch.service
sudo build/chassis/can_relay_setup --bitrate 250000 --address 1 --channel 2 --power-on --set-1m
```

工具先读地址、波特率、通道模式、应答和通信检测配置，然后发送：

```text
CAN6 250K，标准帧 ID=0x601，DLC=4，DATA=44 01 00 0D
```

这是将寄存器 `0x44` 写为 `13`（1M）；地址寄存器 `0x43` 保持不变。
必须检查写回显和寄存器读回。正常控制程序不会写 EEPROM。

手册第 27 页要求模块断电、等电源指示灯熄灭后重新上电，新速率才生效。
仅重启主控进程不能代替模块断电。不要发送 `0x85`，那是恢复出厂参数。
供电重启后，用新进程在 1M 验证：

```bash
sudo build/chassis/can_relay_setup --bitrate 1000000 --address 1 --channel 2 --power-on
sudo systemctl start ros_elf_launch.service
```

只有在 1M 下收到匹配回复，才能证明新速率实际生效。写入后在 250K 读回 `13`
仅证明保存值。写命令超时表示状态未知，应先查询核实，不要盲目重复写入。
若工具报告 `shutdown unconfirmed`，保持主控服务停止并检查供电。

## 气泵操作

整机配置 `src/config/yiyou_ethercat.yaml`：

```yaml
relay_enabled: true
relay_address: 1
relay_output_channel: 2
relay_active_high: true
```

同文件 `remote_ctrl.ros__parameters.relay_enabled: true` 开启 RB。统一禁用可传
`--ros-args -p relay_enabled:=false`；参数重启程序后生效。支持通道 1..64。

完成 button14 启动和校准后，button7（RB）按一次开气泵，松开再按一次关闭；
长按只切换一次，去抖 200 ms。启动、重连、控制权切换后先松开 RB。
启动期间的按键不排队。

启动先确认地址、速率码 `13`、选中通道为普通模式（模式寄存器 `channel-1=0`）、
全应答（`0x40=0`）及通信检测禁用（`0x41=0`），再确认气泵 OFF。
配置不符合时保持 `UNKNOWN` 并禁用本次继电器切换，不自动改写模块设置。
只写所选线圈，其余 63 路不会被批量覆盖。每次写入检查完整回显并独立读回。

地址 1、Y2 的报文（标准帧，经典 CAN）：

| 操作 | CAN ID | DLC | 数据 |
| --- | --- | --- | --- |
| 读地址和波特率 | `0x701` | 2 | `43 02` |
| 地址和 1M 配置回复 | `0x701` | 6 | `43 02 00 01 00 0D` |
| 开气泵 | `0x201` | 3 | `01 FF 00` |
| 关气泵 | `0x201` | 3 | `01 00 00` |
| 读气泵输出 | `0x301` | 1 | `01` |
| 输出回复 | `0x301` | 3 | `01 FF 00` 开，`01 00 00` 关；非零均视为开 |

单线圈写与读见手册第 19、21 页；寄存器定义和读写见第 23–24、27、33 页。
只有日志 `CAN_RELAY ... state=ON/OFF ... confirmed` 表示输出读回已确认；
读回不能证明气压或吸附力。

断连、Ctrl-C、急停和进程退出不额外发送继电器 OFF 命令；关闭输出需再次按 RB。
共享电源关闭、模块断电是否使气泵停止取决于硬件接线。通信超时、发送失败或
读回不一致时显示 `state=UNKNOWN`，不自动重试或回滚。新硬件会话再次确认默认 OFF。

CAN6 复用主进程的 BXI 初始化和收发线程。配置工具只调整 CAN6 的 BRP：
250K 为 3，1M 为 0，SJW/TSEG2/TSEG1 保留厂商 9/9/28。CAN0..CAN5 保持原值。
初始化完成后读回全部七路时序，失败在电机上电前拒绝启动。
日志 `CAN6_TIMING verified bitrate=1000000` 表示正常控制的板卡时序核验通过。

离线验证：`can_timing`、`can_relay`、`bxi_runtime`、`remote_disconnect`、
共享会话生命周期测试和 `can_relay_setup_help`。模拟测试不访问真实板卡。
