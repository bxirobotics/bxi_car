# car2 的 App 启动与 OTA 包

保持现有 `chassis` 控制模块和硬件参数。独立模式下，网关选择 `BXI_CONTROL_BACKEND=car2` 后，Wi-Fi/BLE 启动调用 `/opt/bxi/car2/scripts/run_app_control.sh`，继承网关的 ROS domain 和 DDS 环境。启用 `BXI_CAR2_SHARED_CONTROL=1` 后改为调用 `scripts/shared_control.py`，向常驻主控请求启停；共享服务的部署与控制权规则见 [app_integration.md](app_integration.md#app-与手柄共用主控)。App、共享主控与导航必须使用同一 domain。

## 目录与配置

| 内容 | 默认路径 |
| --- | --- |
| 可替换程序与包内默认配置 | `/opt/bxi/car2` |
| 实机配置、转向标定、机械臂网卡及零偏 | `/var/lib/bxi/car2/config` |
| 第四轴多圈位置记忆 | `/var/lib/bxi/car2/state/yiyou_motor4_position.yaml` |
| CAN/机械臂日志 | `/var/log/bxi/car2` |
| 控制运行与 OTA 互斥锁 | `/var/lib/bxi/car2/control.lock` |

`scripts/postinstall.sh` 检查 ROS Humble 和已安装二进制的动态库依赖，然后只复制不存在的配置文件。已有配置和状态保留原值；更新包中的默认参数不会自动覆盖实机参数。新增参数请按发布说明合入持久配置。包外目录可用 `deploy/car2-app.env` 中的环境变量修改，启动器与 OTA 管理器必须使用同一个锁文件。四轴模式需要现有可信位置记忆；文件缺失时启动器会在访问硬件前拒绝启动，安装钩子不会伪造零位置。三轴模式不要求第四轴文件。

## 首次部署

这些步骤供部署时执行，本轮离线验证不会自动停止服务或启动电机。

1. 构建非 `--symlink-install` 安装前缀，部署到 `/opt/bxi/car2`。
2. 选择一种主控部署方式：独立 App 模式需停止并禁用旧手柄主控服务；共享模式则更新 `ros_elf_launch.service`，启用 `shared_control:=true` 并与网关统一 ROS domain。不能同时运行两套硬件主控。共享服务启动后仅等待请求，不会立即上电。
3. **首次初始化时**从这台机器正在使用的源码目录迁移配置和位置记忆。例如原部署确为 `/home/bxi/tang/car2` 时：

   ```bash
   sudo bash /opt/bxi/car2/scripts/postinstall.sh --migrate-from /home/bxi/tang/car2
   ```

   路径应使用实机实际目录。若之前已初始化持久配置，迁移也不会覆盖它；应自行检查现有文件再合入差异。不要用另一台机器的位置记忆或标定覆盖本机。

4. 首次部署时将 `deploy/car2-app.env` 安装到 `/etc/bxi/control_backend.env`。网关会在包内 `env.conf` 后读取这个持久配置，网关 OTA 不会覆盖它。若该文件已经存在，应合并条目并保留原 DDS 配置；设备密钥仍留在原管理位置。`BXI_CAR2_ENABLE_ARM=1` 启动底盘及机械臂；`0` 仅启动底盘。启动器不改网卡状态，请沿用已经配置好的 EtherCAT 网口。测试可用 `BXI_CONTROL_BACKEND_CONFIG` 替换配置文件路径。
5. 先用下列命令检查将调用的路径和参数，再在实机条件具备时从 App 启动：

   ```bash
   # 共享模式，只查询常驻主控：
   sudo python3 /opt/bxi/car2/scripts/shared_control.py preflight
   # 独立模式，只打印命令：
   bash /opt/bxi/car2/scripts/run_app_control.sh --dry-run
   ```

`--dry-run` 不创建目录、不加锁、不加载 ROS、不访问硬件。实际启动按所选 `yiyou_ethercat.yaml` 的 `yiyou_arm_home_on_start` 决定是否回位，目标由 `yiyou_arm_home_joint_degrees` 指定（未配置时默认 `[0,0,0]`）；这不是重新设置零点。elf3-59 的 App 和手柄在共享模式下都使用 Gemini 335 调平后的 `[-10,-115.31,117.56840515136719]°` 和配套模型。独立四轴模式仍读取自己的持久配置。

遥控源码配置现使用与 App 持久配置相同的 `drive_velocity_mode: -3`。
该模式的普通加速、减速和零速停车由底盘控制器的软件斜坡执行；显式保护停止立即下发零速。
同步模式时，配置解析、电机模式回读和速度 PDO 下发必须支持同一模式。
遥控满杆 x/y 速度和整车 x/y 速度上限均为 1.0 m/s；直径 0.13 m 的车轮输出轴上限为 146.91 RPM（约 1.0 m/s），斜向行驶及平移叠加转动仍受轮速限制。
独立模式的 App 与遥控各自保存配置和硬件调参，升级不会自动互相覆盖。共享模式两方使用常驻主控选定的同一份实机配置；当前 elf3-59 使用 `/home/bxi/tang/car2/src/config`，不要用包内默认值覆盖它。

EtherCAT 网口检查要求接口存在并处于 UP 状态，允许网口配置 IPv4/IPv6 地址；程序不自动配置 IP。
通信异常、反馈超时和转矩保护仍由原有逻辑处理。共用网口后的通信稳定性需结合实际雷达流量验证。

## 构建与制包

下面示例只构建、安装到临时目录并制包，不部署或启动控制器：

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B /tmp/car2-release-build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX=/tmp/car2-release-prefix
cmake --build /tmp/car2-release-build --parallel 4
ctest --test-dir /tmp/car2-release-build --output-on-failure
cmake --install /tmp/car2-release-build
python3 scripts/package_ota.py \
  --install-prefix /tmp/car2-release-prefix \
  --output-dir /tmp/car2-releases --version v261030
```

输出 `car2_v261030.tar.gz` 和 `car2_v261030.tar.gz.sha256`，包内根目录为 `car2/`、`car2/.version` 为 `v261030`。版本格式为 `vYYMMDD` 或 `vYYMMDD-N`。制包器拒绝 symlink 安装、不完整安装和同名已存在发布产物，不携带运行日志或位置记忆；保留根许可证、SOEM 来源和本地补丁记录。生产包应在同架构、同 ROS Humble ABI 的环境构建。

使用现有 OTA 发布渠道时，把两个文件放在设备身份对应的 `ELF3/car2` 目录（arm64 为 `ELF3/arm64/car2`）。网关仍使用已有设备身份和鉴权，`car2` 后端包列表用 `car2` 替换旧 RL Controller 项。上传、渠道发布、真实设备安装需要按既有发布流程单独执行。

## 升级运行约束

App 控制进程持有互斥锁直到退出；OTA 部署持同一把锁，因此先停止 car2 再执行升级。下载阶段可以正常下载，目录替换阶段拒绝运行中的 car2，包括未使用新启动器的旧直接启动进程。升级不会自动上电或启动程序。配置、标定和位置记忆不在包交换范围内；若安装钩子失败，网关使用已有机制回退程序目录。该回退不等于电机或重启健康检查已通过，仍需实机验证启动、停止和机械臂动作。

共享模式正常停止后断电，随后由 systemd 重建主控进程回到等待状态，以清理 BXI 厂商库不会自行结束的收发线程。因此升级主控程序前还需在硬件停止后停掉服务，防止它自动拉起等待进程。故障状态不自动重建，也不会自动上电。当前 elf3-59 服务运行源码构建 `/home/bxi/tang/car2/build/chassis/robot_control`；更新 `/opt/bxi/car2` 的 OTA 包不会更新该源码构建，须同步并构建服务实际使用的版本，再恢复等待服务。
