# 底盘与机械臂控制

基于 ROS 2 Humble 和 C++17 的舵轮底盘与机械臂控制工程，ROS 包名为 `chassis`。支持手柄和 App 控制，使用 BXI 板卡连接 CAN 设备，通过 EtherCAT 控制意优机械臂电机。

现有手机 App 的接入接口、启停和机械臂按钮见 [App 接入说明](docs/app_integration.md)；独立升级包、配置迁移和制包命令见 [OTA 说明](docs/car2_ota.md)。

## 目录

```text
CMakeLists.txt            # 根目录统一构建入口
start.sh                  # 日常启动菜单：意优设零、仅底盘、整机
start_yiyou_jog.sh         # 意优点动 / 设置零位快捷入口
start_chassis_only.sh      # 底盘手柄启动快捷入口
start_chassis_arm.sh       # 整机手柄启动快捷入口
start_qr_midpoint.sh       # Gemini 335 双码中点识别（独立运行）
src/
  thirdparty/
    bxi/                  # board.h/.cpp：BXI 板卡、CAN 收发、共享电机电源
      vendor/             # 厂家头文件和静态库
    motor/
      bxi/                # communication.h/.cpp + motor.h/.cpp：MIT 电机
      yiyou/              # communication.h/.cpp + motor.h/.cpp：EtherCAT 电机
        detail/           # CiA402 辅助逻辑
        vendor/soem/      # 原有 SOEM 源码
      iswv/               # communication.h/.cpp + motor.h/.cpp：CANopen 电机
        protocol/         # PDO、Axis、对象字典等协议内部实现
    arm/                  # kinematics.h/.cpp：正逆解；control.h/.cpp：机械臂控制
    chassis/              # control.h/.cpp：底盘控制；转向校准、限位和运动学
    vision/               # Gemini 335 取帧、标记识别及二维/三维定位
  example/
    main.cpp              # ROS 主入口，调用各模块
    remote_control.h/.cpp # 手柄与 ROS 接口
    arm_parameters.h      # 机械臂 ROS 参数适配
  config/                 # 原有机械尺寸、电机方向和站位配置
  cmake/                  # 安装包导出模板
test/
  debug.py               # 唯一面向使用者的调试菜单
  tools/                 # 菜单调用的 C++ 后端和 Python 辅助模块
  unit/                  # 离线回归与模拟通信测试
docs/                    # 协议与机械臂说明
```

`src/thirdparty` 按本项目约定包含自有模块和外部依赖。`bxi/board` 与 `motor/bxi` 分别控制板卡和电机。通信层负责收发、编解码或总线会话，调用层提供电机操作；底盘调参在确认电机失能后由底盘层回调执行。

## 构建与测试

在项目根目录直接执行 `colcon build`，产物使用标准的 `build/`、`install/`、`log/`。
启动脚本、调试工具和整车服务都使用这里生成的程序。

随附的 BXI PCI 静态库适用于 Linux x86-64。构建其他架构时，需要对应架构的厂家库。第三方代码、库和手册的来源见[依赖说明](docs/dependencies.md)。

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

部署模板使用参考工控机的账户 `bxi`、工程目录 `/home/bxi/tang/car2` 和网卡名。换机时请按实机修改 `deploy/` 中的服务和设备规则。底盘与机械臂的参考参数在 `src/config/`，零位记录和第四轴位置记忆要在对应机器人上建立或迁移。

参考工控机编译成功后，用
`sudo systemctl restart ros_elf_launch.service` 加载新程序。只改 YAML 时无需编译。
需要自定义构建目录时，仍可设置 `CHASSIS_BUILD_DIR` 和 `CHASSIS_INSTALL_DIR`。
`src/cmake/` 中的 `.cmake.in` 是安装包导出模板，属于构建源码，需要保留。

## 日常启动脚本

参考 car2 的操作方式，在源码根目录执行：

```bash
sudo -E bash start.sh
```

菜单选择 `1` 意优点动/设置零位、`2` 底盘单独启动、`3` 整机启动、`0` 退出。
也可以直接使用与 car2 同名的快捷脚本：

```bash
# 意优点动、设置零位：先在终端选网口，再输入要控制的从站，例如 1,3。
sudo -E bash start_yiyou_jog.sh
# 也可以直接指定网口和站位，跳过选择。
sudo -E bash start_yiyou_jog.sh --interface enp86s0 --slaves 1,2,3
# 仅底盘：等待手柄启动键，不初始化机械臂。
sudo -E bash start_chassis_only.sh
# 整机：等待手柄启动键，按配置机械臂回零后校准底盘。
sudo -E bash start_chassis_arm.sh
```

意优校准入口默认使用**松闸手动调零**：电机全程未使能，左右键选轴，`b` 松开当前轴抱闸。
先托住机构，再手动摆正；停稳后按空格恢复全部抱闸自动控制，再按 `z` 保存当前轴零位。
一次只允许松开一个轴；切换选择不会松开另一轴，↑/↓不发送运动命令。
`q` 正常退出时先发送闭闸命令并等待配置延时，再断共享电源；故障时优先断电。
松闸仍需要驱动/抱闸供电；短型号RHU40H/RHU32H不能确认带闸，B/N选配须核实实物。
指令读回不等于物理抱闸反馈；松闸后难以推动时不要硬扳。详见[手动调零说明](docs/yiyou_manual_zero.md)。
只有按 `z` 才设零并保存该轴全部可持久化参数；记录写入项目根目录的
`yiyou_ethercat_zero_calibration.json`。这与启动时返回已保存零位是两个操作。
选择界面列出本机实体有线网口，输入编号选择；只有一个网口时可直接回车。
从站按 EtherCAT 链路物理站位输入，支持 `1,3` 这样只控制部分电机，回车默认 `1,2,3`。
`YIYOU_INTERFACE`、`YIYOU_SLAVES` 可修改选择界面的默认值；默认网口只有在列表中存在时才生效。
`--interface`、`--slaves` 分别跳过对应选择，输入 `q`、Ctrl-C 或遇到 EOF 时取消，选择期间不上电。
`--dry-run` 仍只预览默认值或显式参数对应的命令，不弹出网口/从站选择。

点动/零位校准默认启用 **100 N·m** 转矩断电阈值，可用
`--torque-limit-nm 100` 显式指定正数。比较的是每个所选电机的
`abs(0x6077 × 0x6076 / 1000000)`，即驱动报告的电机侧转矩，不乘减速比。
任一轴严格超过阈值，或必要反馈缺失、通信故障时，优先切断共享电机电源并锁定退出，
不会自动恢复使能。任一轴超限会影响同一共享电源上的全部电机。
保护在 EtherCAT 周期线程中运行，覆盖点动、保持和写零期间，不依赖终端刷新。
启动时缺少有效 `0x6077` 或非零 `0x6076` 将拒绝校准；界面同时显示实际转矩和阈值。
正常设零仍等待 EEPROM 保存完成；保护触发时断电优先，正在保存的零位可能无效，需重新核验。
这是 Linux 软件监测与断电命令，周期约 2 ms，并非保证最坏响应时间的硬件安全功能。
未使能手动调节时，6077不是外力/输出端负载测量，100 N·m阈值不能保证手推力或重力负载安全。
`--control-mode torque` 可显式选择转矩调节，每键增减电机轴侧名义目标0.05 N·m。
该模式电机可能持续转动，零转矩不提供位置保持；左右选轴不会清除之前轴的出力。
`--speed-rpm` 在此模式下是输出轴速度的**软件断电阈值**，不是恒速指令；默认1 RPM。
写零前需要其他轴全部失能；当前轴清零失能并静止后才保存，保存后不重新使能。
界面分开显示请求值、整数指令量化后的名义值和驱动反馈。量化按每台电机实读额定转矩计算；
原始指令限制为 ±1000‰，且请求值与量化值均不能超过配置的转矩保护阈值。
详细单位依据与限制见 [转矩调节说明](docs/yiyou_torque_control.md)。

`--control-mode position` 可切回每次1°点动；只有该模式接受 `--startup-retreat-slave N`，
显式请求指定轴启动后向下1°一次。未指定则等待按键，其他轴首次点动时才使能。
位置模式下已调过的轴继续保持，已保持轴设零后恢复保持；未使能轴设零后仍未使能。
直接运行 Python 前端默认仍是位置模式；校准启动脚本显式选用手动模式。
保护触发或驱动故障时，所有模式都不会自动重试、清故障或恢复使能。
`--dry-run` 不上电、不使能、不写零位。

底盘和整机默认松开后按 **button14** 开始，**button11** 急停，`Ctrl-C` 退出。
整机的 **button7 / RB** 控制 CAN6 继电器：按一次开启、松开再按一次关闭；
品全微新模块默认 250K，先用配置工具改为 1M 并断电重启；运行时为 1M、地址1、Y2气泵，
操作与配置步骤见 [继电器说明](docs/can_relay.md)。
整机调用 `robot_control`，底盘调用 `chassis_only_remote`；其余控制参数沿用当前程序和
`src/config` 的配置。可用 `CHASSIS_CONFIG`、`YIYOU_CONFIG` 覆盖配置文件，
`JOYSTICK_DEVICE` 覆盖手柄设备，末尾追加 ROS 参数作临时覆盖：

```bash
sudo -E bash start_chassis_arm.sh --ros-args -p yiyou_arm_home_on_start:=false
# 以下只预览命令，不加载 ROS 环境或启动设备，也不需要 sudo。
bash start.sh --dry-run
bash start_yiyou_jog.sh --dry-run
bash start_chassis_only.sh --dry-run
bash start_chassis_arm.sh --dry-run
```

脚本可从任意工作目录调用，按项目根目录定位 `build/` 和 `install/`，支持
`CHASSIS_BUILD_DIR` / `CHASSIS_INSTALL_DIR` 覆盖目录。
底盘/整机脚本自动加载 ROS 环境；脚本不自动提权或启用网卡。
运行前需有设备访问权限、EtherCAT 网口已启用，并避免多个程序同时控制同一设备。
`--help` 查看脚本参数。日常启动脚本保留在源码根目录，调试菜单仍使用下面的入口。

## CAN 故障诊断日志

底盘/整机启动脚本默认使用项目根目录 `log/can/`；可通过环境变量
`CHASSIS_CAN_LOG_DIR` 更换目录，设为空字符串可关闭。直接运行二进制时默认使用
当前工作目录的 `log/can/`。启动后即维护保留期限，包括等待手柄启动键期间。

CAN0/1/2 各自在内存保留最近 4096 条诊断事件。SDO 超时、SDO 错误回复、发送失败
或收到 CAN 错误帧时，后台保存故障前记录和随后 2 秒内的事件到
`can-diagnostic-*.log`；正常退出会提前保存已经收集的记录。正常通信不持续写报文文件。
记录包括 CAN 请求/接收报文、SDO 节点与对象地址、请求序号、成功/失败、耗时和协议队列丢帧统计。
每次捕获及待处理捕获数有上限，日志中会标明因日志容量限制而丢弃的数量。

`CAN_TX_ATTEMPT` 表示调用发送；`CAN_TX_QUEUED` 只表示传输层接受了请求，
不证明电机已收到。`CAN_RX` 表示收到板卡回调；`SDO_RX_DISPATCH` 表示协议线程
处理该报文，`rx_age_us` 可用于判断接收后的排队延迟；`SDO_UNMATCHED` 表示回复
没有匹配当前事务。仅凭未出现 `CAN_RX` 仍不能区分电机未回复与底层丢帧，必要时需外部抓包。

日志后台线程在启动时及运行中每小时删除修改时间超过 **168 小时（7 天）** 的本组件
诊断文件，不递归、不跟随符号链接、不删除构建日志或系统 journal；程序停用期间的过期
文件会在下次启动时清理。诊断写盘失败只报警，不改变原有 250 ms 超时和故障停机行为。

## 机械臂四轴力矩记录与图表

含机械臂的主程序自动记录四个意优电机的实际力矩，默认目录为项目下
`log/arm_torque/`，可用 `CHASSIS_ARM_TORQUE_LOG_DIR` 指定其他目录，空字符串关闭。
从第一次机械臂反馈开始到程序结束视为一次记录，覆盖回零、底盘校准和后续运行；
仅等待启动键且尚未启动机械臂不会产生空记录。

按机械臂反馈周期约 20 ms 采样（50 Hz，非硬实时），异步保存 CSV。
驱动器 `0x6077` 是有符号额定力矩千分比，`0x6076` 为额定力矩 mNm：
`torque_Nm = torque_permille * rated_torque_mNm / 1000000`。
这是驱动器报告的电机力矩，不等于额外传感器测得的机械臂末端力或减速器输出扭矩。
不支持的反馈记为空值；额定力矩未知时保留千分比，不伪造 N·m。
记录队列满时明确留下丢样数量和空值缺口，不用零代替缺失数据。

正常结束、急停或故障退出时，先完成原有停机和断电流程，再保存 CSV 并生成四轴 PNG
图表和 JSON 摘要。仅保留最近 **3 次**完成记录的 CSV、PNG 和摘要，旧记录自动删除。
图表显示**输出端力矩估算值**：前三轴暂用 160∶1，第四轴暂用 101∶1，
这些速比由历史校准标尺与厂家标准配置反推，尚未经本次寄存器实读确认。
估算为 `电机侧力矩 × 推断减速比`，未计传动损耗（效率暂按 100%），不能当作输出轴实测值。
图中明确标注估算、速比来源及忽略损耗；更换电机或确认速比后应更新出图脚本中的速比。
CSV 与 JSON 的 `motors` 字段仍保存电机侧数据；JSON 的 `output_estimate` 保存估算峰值和假设。
额定力矩未知的轴只画电机侧原始千分比，不伪造输出端 N·m。旧版图表会在下次出图时自动重绘。
强制杀进程或掉电可能留下 `.csv.part`；下次启动时通过文件锁确认没有活跃写入者后恢复
已有数据并补图，未写出的末尾样本无法恢复。图表生成失败会保留最近记录的 CSV。

图表工具使用现有系统 Matplotlib，以隔离 Python 路径避免其他环境的 NumPy 版本干扰：

```bash
/usr/bin/python3 -I -S tools/arm_torque_report.py --directory log/arm_torque --keep 3
```

整机力矩记录只用于观察；上述转矩断电保护由点动/零位校准入口启用。
CAN 故障诊断日志仍按前述 7 天规则保留。

## 底盘日常调参

修改 `src/config/steering.yaml` 后重启程序生效，底盘和遥控器共同读取它。
下表对应仓库当前配置；运行时以实际加载的 YAML 和适用的 ROS 参数覆盖值为准。

| 调节内容 | 参数 | 当前值 |
| --- | --- | --- |
| 四轮输出轴最高转速 | `drive_max_output_rpm` | 300 RPM |
| 四轮轮缘加速度 / 减速度 | `drive_acceleration_m_s2` / `drive_deceleration_m_s2` | 2 / 2 m/s² |
| 舵轮转向速度 / 整车 yaw 变化率 | `steering_rate_limit_rad_s` / `command_yaw_acceleration_rad_s2` | 4 rad/s / 4 rad/s² |
| 整车 x / y / yaw 速度上限 | `max_velocity_x_m_s` / `max_velocity_y_m_s` / `max_velocity_yaw_rad_s` | 0 / 0 / 0 |
| 遥控满杆 x / y / yaw 速度 | `scale_linear` / `scale_lateral` / `scale_angular` | 0.6 m/s / 0.6 m/s / 1 rad/s |
| 遥控 x/y 正向 / 负向指令变化率 | `max_accel` / `max_decel` | 2 / 2 m/s² |

整车上限为 `0` 时保留原有轮速限制；设为正数后，超限指令的 x/y/yaw 同比例缩小。
这些上限适用于 `/cmd_vel_car`、遥控器和控制器直接调用，自动前进也受 x 上限约束。
遥控变化率沿用原算法，按指令差值正负选择，不按车速绝对值的加速/减速分类。

`hardware_config_file` 指向同目录的 `steering_hardware.yaml`，保存 CAN、机械尺寸、
方向、校准和驱动器参数。驱动器 `null` 参数的自动回填留在所属文件，日常文件不会
因此展开。日常文件同名参数优先；旧单文件 YAML 和显式逐轮 `modules` 仍可加载，
旧配置的逐轮字段仍优先于全局字段。复制配置时同时复制这两个文件。
通过无节点限定的 `-p steering_config_file:=/path/steering.yaml` 同时选择底盘和遥控配置；
遥控的显式 ROS 参数（例如 `-p scale_linear:=0.3`）优先于 YAML 中的默认值。

## 统一调试入口

```bash
python3 test/debug.py
# 安装后也可以：
ros2 run chassis debug.py
```

- `1` 电机调试，再选择 `1 BXI / 2 意优 / 3 ISWV`。
- `2` 机械臂离线正逆解，使用 `src/config/arm_kinematics.json`，不连接电机。
- `3` 底盘调试，选择直接启动或遥控启动。
- `0` 退出。

预览菜单将运行的命令：

```bash
python3 test/debug.py --dry-run
python3 test/debug.py --mode yiyou --interface enp86s0 --slaves 1,2,3 --dry-run
python3 test/debug.py --mode iswv --bus 1 --node 1 --dry-run
python3 test/debug.py --mode arm
```

电机、底盘模式会实际打开设备；`--dry-run` 和 `--help` 不打开设备。意优网口通过 `--interface` 指定，站位通过 `--slaves` 指定。调试工具默认机械参数需与实机一致，使用 `--help` 查看接口单位与可选项。不要同时启动多个控制同一设备的程序。

## Gemini 335 双码中点识别与对接

IMU 静态读取与第三轴安装角说明见 [Gemini 335 IMU](docs/gemini_imu.md)。

独立预览识别两个 QR 码／ArUco 的中点。深度有效时同时输出相机三维坐标，
不发送运动指令。

```bash
python3 -m venv .venv-vision
.venv-vision/bin/python -m pip install --no-deps -r requirements-vision.txt
bash start_qr_midpoint.sh
# 现场 ArUco 4×4 标记 ID 0 和 ID 1：
bash start_qr_midpoint.sh --marker-type aruco --marker-ids 0 1
# 离线图片验证，不打开相机：
bash start_qr_midpoint.sh --image /path/to/two_codes.png --headless
```

默认恰好两个码才输出目标；QR 模式可用 `--qr-texts 'left-marker' 'right-marker'` 指定一对，
ArUco 模式可用 `--marker-ids 0 1` 指定 ID。标记完整黑边和四角应无遮挡。
启动参数、深度坐标定义及后续机械臂标定边界见 [识别模块说明](docs/qr_midpoint.md)。

视觉对准入口 `bash start_arm_tracking.sh` 默认仅预览；使用 `--joystick` 后由 button6
开启／关闭，或用 `--execute` 显式开启。跟踪默认使用两个 `DICT_4X4_50` ArUco 的中点，
将两码中心连线的中点对准实物对接基准点；只看到一个码时不产生对接目标。
画面有其他码时可用 `--marker-ids 0 1` 明确选择现场的两个码。像素模式让机械臂上下、底盘左右配合，
将目标对准画面中心；底盘横移最多 20 mm/s，前后和旋转保持零指令。
机器人通过 `car2-arm-tracking.service` 开机无窗口待机，无需登录桌面，仍由 button6 开启／关闭。
原桌面相机自启动已关闭，避免重复占用相机。服务使用 `--tracking-mode approach --marker-ids 0 1`。底盘先横向对齐，停止横移并稳定
0.3 秒后记录终点，机械臂 X 停在两码三维中点前 4 cm，Z 到该中点下方 4.25 cm。
模型工具 pitch 为 90° 时镜头水平，前伸量为 `max(0, 深度 − 4 cm)`。中点与镜头同高时，末端目标比记录时高 2 cm。
目标俯仰固定，完整 X/Z 终点直接交给电机 PP 控制，速度和加减速由意优电机配置决定。
记录后标记出画仍按固定坐标前伸，到位保持；
button6 退出再开启才记录新目标。
`approach` 使用 `car2-imu.service` 发布的外接 IMU 数据检查起始航向；偏航超过 8°、
IMU 超过 200 ms 未更新或姿态异常会取消本轮，重新开关对接才可恢复。
执行断流、反馈失效或故障也会取消本轮。寻找目标没有 10 秒超时。
`pixel-depth` 保留像素对中加深度定距，`depth` 保留固定 X 的三维高度跟踪。
手柄打断、失效停车和启动要求见 [记录三维目标并控制机械臂前伸](docs/arm_height_tracking.md)。

## 主控制程序

只有一份 `src/example/main.cpp`，保留原来的可执行名称供现有 ROS 调用继续使用：

`main()` 负责模式选择、ROS 初始化和程序退出；`run_control()` 管理遥控等待和节点调度。
节点内部按 `read_startup_config()`、`initialize_hardware()`、`setup_ros_interfaces()` 顺序启动，
运行期间由 `update_control()` 统一处理机械臂、底盘和反馈。

| 程序 | 启动方式 |
| --- | --- |
| `chassis` | 底盘与机械臂直接启动 |
| `key_control` / `robot_control` | 等待手柄启动键，再启动底盘与机械臂 |
| `chassis_only` | 只启动底盘 |
| `chassis_only_remote` | 手柄启动，只控制底盘 |

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run chassis robot_control
```

程序默认读取本项目 `src/config/steering.yaml`；含机械臂的模式还会读取
`src/config/yiyou_ethercat.yaml`。查找不依赖终端当前目录，在本项目中修改配置后重启程序即可生效。
源码配置目录不存在时，使用可执行程序所在安装前缀的 `share/chassis/config/`。
手柄模式仍等待 14 号启动键，再按配置进行机械臂回零和底盘校准。

临时使用其他配置或参数时，仍可覆盖默认值：

```bash
ros2 run chassis robot_control --ros-args \
  -p steering_config_file:=/absolute/path/steering.yaml \
  --params-file /absolute/path/yiyou_ethercat.yaml \
  -p yiyou_arm_home_on_start:=false
```

底盘接收 `/cmd_vel_car`；机械臂使用 `yiyou_arm_target=[x,z,pitch]` 和 `/arm_pose`。
底盘默认在 300 ms 没有新速度指令后下发零速，每 20 ms 控制周期检查一次。
可在启动参数中用 `-p chassis:command_timeout_sec:=0.3` 设置正数超时值；不要用单次非零消息保持行驶。

Battle Dragon 蓝牙手柄（20d6:400d）的链路保护每 100 ms 主动读取一次 HID 输入报告，
500 ms 没有成功应答即禁止运动，在下一个 50 ms 遥控周期持续发布零速度。
查询在独立线程执行，蓝牙读操作卡住不会阻塞遥控停车；摇杆保持不动不要求产生新事件。
断连、设备拔除、读取失败或权限不足均停止输出；恢复后重新读取轴状态，必须先回中再操作。
目前主动探测只验证过该型号的蓝牙连接：其他设备或 USB 接收器连接会保持零速并记录原因，
需先验证对应接收器的无线连接检测协议才能开放控制。服务以 root 运行可读取 hidraw；
手动运行时也需有对应 hidraw 设备的读取权限。

以上时间是零速度指令的检测/下发时限，实际停车还需要驱动器减速时间。
两层保护依赖控制进程正常调度，不能替代硬件急停。机械臂方向配置保持原值。

## 模块调用示例

各模块的 `example.cpp` 都是可复制到其他 C++ 文件的函数片段：
参数传入数据或已有对象，通过返回值、回调或异常表达结果。
每个函数签名后注明输入、输出、单位和调用前提；不提供 main()，不加入默认编译。

| 模块 | 示例 | 展示内容 |
| --- | --- | --- |
| BXI 板卡 | [bxi/example.cpp](src/thirdparty/bxi/example.cpp) | 打开通道、共享电源、发帧、接收回调 |
| BXI 电机 | [motor/bxi/example.cpp](src/thirdparty/motor/bxi/example.cpp) | MIT 使能、目标输入、反馈解码、退出模式 |
| ISWV 电机 | [motor/iswv/example.cpp](src/thirdparty/motor/iswv/example.cpp) | 速度模式初始化、速度目标、反馈、停止 |
| 意优电机 | [motor/yiyou/example.cpp](src/thirdparty/motor/yiyou/example.cpp) | 速度、位置、电流模式及反馈、停止 |
| 机械臂运动学 | [arm/example.cpp](src/thirdparty/arm/example.cpp) | 正解、逆解、结果状态与单位 |
| 底盘 | [chassis/example.cpp](src/thirdparty/chassis/example.cpp) | 舵轮解算、配置、启动、速度目标、周期更新、停车 |

电机和通信对象由调用方长期持有，初始化与周期目标更新分开。
各文件顶部注明所需链接库；错误处理保留原接口的 Result 或异常语义。
机械臂示例只依赖 chassis_arm_kinematics。

| 头文件 | 调用方式与单位 |
| --- | --- |
| `bxi/board.h` | `chassis::BxiPciTransport(bus)`；`set_motor_power(true/false)` |
| `motor/bxi/motor.h` | `bxi::Motor`：`enter_motor_mode()`、`exit_motor_mode()`、`command()`；MIT 参数为 rad、rad/s、Nm 和 Kp/Kd |
| `motor/yiyou/motor.h` | `chassis::YiyouMotor`：`enable_rpm()`、`enable_position()`、`enable_cst()`、目标输入、`stop()`；分别为输出轴 RPM、编码器脉冲、额定电流千分比 |
| `motor/iswv/motor.h` | `chassis::IswvDrive`：`enable_rpm()`、`set_velocity_rpm()`、`stop()`；转向位置和其他支持的模式复用 `iswv::Axis` / `axis.drive()` |

BXI 的接口提供 MIT 模式，不虚构 CANopen 式模式切换；ISWV 与意优保留各自协议支持的模式。`stop()` 在意优和 ISWV 中包含停车及失能。构造板卡/主站、上电、校准和退出顺序由应用管理，正逆解可以独立使用 `arm::Kinematics`。

安装后 CMake 使用 `find_package(chassis REQUIRED)`，链接例如 `chassis::chassis_bxi_motor`、`chassis::chassis_arm_kinematics`；CANopen 仍导出 `iswv::canopen`。供应商来源与许可范围见 [docs/dependencies.md](docs/dependencies.md)。

## 作为示例工程移植

移植板卡、电机、机械臂或底盘模块时，可以从上方的[模块调用示例](#模块调用示例)找到对应入口，再从根目录的 `CMakeLists.txt` 核对所需头文件和链接目标。

完整 ROS 控制可以复用应用层；纯 C++ 移植需要给选定模块单独接入 CMake，
因为参考工程根 CMake 仍依赖 ROS/ament。新设备的网卡、ID、方向、零偏、限位和启动回零须另行配置；移植验证默认不操作硬件。
