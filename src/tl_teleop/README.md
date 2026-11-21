<div align="center">

# 天链机器人tl_teleop使用说明书

</div>

## 目录
* 1.[tl_teleop功能包说明](#tl_teleop功能包说明)
* 2.[tl_teleop功能包使用](#tl_teleop功能包使用)
* 3.[tl_teleop功能包架构说明](#tl_teleop功能包架构说明)
* 4.[tl_teleop功能包话题与服务说明](#tl_teleop功能包话题与服务说明)

## tl_teleop功能包说明
tl_teleop功能包实现了机械臂的遥操作（Teleoperation）功能，通过 PXREA Robot SDK 与 VR 遥操作手柄设备通信，将手柄的位姿映射为机械臂末端的运动指令，实现对机械臂的实时跟随控制。

* 1.功能包使用。
* 2.功能包架构说明。
* 3.功能包话题与服务说明。

通过这三部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用

### 功能特性
- **VR 手柄遥操作**：通过 PXREA Robot SDK 读取 VR 手柄位姿（位置 + 四元数姿态），实时驱动机械臂末端运动
- **6/7 轴适配**：通过 `arm_axis_mode` 参数切换 6 轴模式（TCB605 系列）或 7 轴模式（TCB710 系列），自动匹配关节限位和末端关节倍率设置
- **握紧触发机制**：启动或持续断帧后先松开握把与 A 键，再握紧使能。握力超过 0.9 开始跟随，降到 0.8 或更低停止跟随，每次握紧从实测关节角和末端位姿重新起步
- **位置死区滤波**：手柄各轴微小位移（< 5mm）被单独过滤，避免手部抖动导致机械臂振荡
- **单步位置增量限幅**：相对握紧基准的各轴位移限制为 300mm，求解前再按线速度和角速度分步逼近目标
- **末端关节位移倍率**：握紧时末端两个关节超过配置角度则使用较小位移倍率，本次握持期间倍率固定，避免倍率切换造成目标跳变
- **关节跳变检测**：检查相邻有效逆解是否跳变。遇到不连续解时缩小笛卡尔步长重新求解，实际输出由 Ruckig 约束速度、加速度和 jerk
- **A 键复位**：按住右手 A 键，关节按速度和加速度限制向零位靠近，松开结束回零
- **可选乐白夹爪**：复用 `vr_tcb610` 的 USB/RS485 SDK。按住 Grip 跟随时，右 Trigger 按下闭合、松开张开；串口读写由独立节点处理
- **250Hz 控制循环**：独立控制线程以 250Hz 频率运行，通过 tl_driver 的 ServoJ 模式实现高速位置跟随
- **VR 断帧处理**：200–500 毫秒内平滑减速并保留握持基准，恢复数据后继续跟随。超过 500 毫秒结束本次跟随，必须先松开再握紧。
- **最短旋转路径**：四元数自动取最短路径，避免姿态反向绕远路

### 系统依赖关系

tl_teleop 运行时依赖 **tl_driver** 功能包提供以下服务：

| 依赖项 | 类型 | 说明 |
|--------|------|------|
| `/tl_driver/set_current_mode` | 服务 | 切换到运行模式（模式 2） |
| `/tl_driver/set_speed` | 服务 | 设置运动速度 |
| `/tl_driver/open_servoj` | 服务 | 开启 ServoJ 模式 |
| `/tl_driver/close_servoj` | 服务 | 关闭 ServoJ 模式 |
| `/tl_driver/coord_transform` | 服务 | 逆运动学求解（笛卡尔 → 关节） |
| `/tcp_pose` | 话题 | 获取当前末端位姿 |
| `/joint_states` | 话题 | 获取当前关节角，作为每次握紧和回零的起点 |
| `/tl_driver/set_servoj_pos` | 话题 | 发送目标关节位置 |

因此使用 tl_teleop 前应先启动 tl_driver。

### 外部环境依赖

tl_teleop 的 PXREA Robot SDK 依赖 **XRobotToolKit-PC_Service** 作为 PC 端后台服务，用于与 VR 遥操作设备建立连接并传输手柄数据。使用 tl_teleop 前必须安装并启动该服务。

**下载地址**：[XRoboToolkit-PC-Service v1.0.0](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)

根据平台架构选择对应的安装包：

| 架构 | 安装包 |
|------|--------|
| ARM64 | `XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb` |
| x86_64 | `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` |

安装并启动 XRobotToolKit-PC_Service 后，再启动 tl_teleop 节点，PXREA SDK 才能正常连接 VR 设备并接收手柄位姿数据。

## tl_teleop功能包使用
### tl_teleop功能包编译

```bash
cd ~/tl_robot_ros2_cpp
colcon build --packages-select tl_teleop
source install/setup.bash
```

> 若首次编译或接口包有更新，请使用 `colcon build`（不带 `--packages-select`）以确保 tl_ros2_interface 先被编译。

### 启动方式

tl_teleop 依赖 XRobotToolKit-PC_Service 和 tl_driver，启动前需**按顺序**依次启动以下服务与节点：

**第一步：启动 XRobotToolKit-PC_Service**

```bash
source /opt/apps/roboticsservice/runService.sh
```

该命令启动 PC 端后台服务，PXREA SDK 通过该服务与 VR 遥操作设备通信。

> 注意：不同平台的 XRobotToolKit-PC_Service 启动命令可能不同，具体查阅 XRobotToolKit-PC_Service 的相关说明。

**TCB610 同时启动驱动和遥操**

PC 端后台服务启动后，TCB610 可用下面的命令一次启动两个节点。已有驱动或遥操节点时，先结束它们再运行。驱动启动会连接机械臂并上电。

```bash
ros2 launch tl_teleop tl_tcb610_teleop.launch.py arm_ip:=192.168.1.13
```

此入口使用 TCB610 驱动配置和 6 轴遥操配置。TCB610 驱动配置已将 `joint_monitor_rate` 固定为 `0.0`，单独运行 `ros2 launch tl_driver tl_tcb610_driver.launch.py` 时也会关闭电流、温度、电压的周期查询。这些查询与逆解共用控制器连接，实测会使逆解等待约 200–335 ms。因此 `/tl_driver/joint_monitor` 停止更新，关节位置和末端位姿继续通过状态流发布，速度和力矩的 `feedback_rate` 为 100 Hz。

**第二步：启动 tl_driver（以 TCB710 为例）**

```bash
ros2 launch tl_driver tl_tcb710_driver.launch.py
```

> 其他臂型的启动命令参见 [tl_driver 说明文档](../tl_driver/README.md)。

**第三步：启动 tl_teleop**

```bash
# 6 轴
ros2 launch tl_teleop tl_teleop_6axis.launch.py
# 7 轴
ros2 launch tl_teleop tl_teleop_7axis.launch.py
```

启动成功后，节点将依次执行：
1. 检查是否已有 `tl_teleop` 进程，避免多个节点争用同一 VR SDK；重复启动会给出占用者 PID 并退出
2. 等待驱动服务可用（最长 30 秒，可用 Ctrl+C 立即结束等待），然后初始化 PXREA Robot SDK
3. 调用 tl_driver 服务完成模式切换（模式 2）、速度设置、开启 ServoJ（tl_driver 自动处理上电逻辑）
4. 进入遥操作控制循环（250Hz，每周期4ms）。Ruckig 和笛卡尔目标步长使用同一4ms周期，关节速度、加速度和 jerk 仍按配置约束。VR 数据间隔突增时，目标以限定步长进入逆解，避免直接追赶积压帧。VR 输入帧率和驱动反馈频率独立于 ServoJ 下发频率。

PC Service 连接成功不代表已收到头显数据。没有有效 VR 帧、VR 数据过期、夹爪未就绪会分别输出提示；保持手柄 Grip 和 A 松开，收到有效输入并出现 `Teleoperation armed` 后再开始控制。

若出现 `PXREA: server connected`，但一直提示 `No valid VR controller frames received yet`，先在头显 XRoboToolkit 中对 PC 的 Wi-Fi 地址执行断开/重新连接，保持应用前台，再松开 Grip/A 重新使能。PC 的 Wi-Fi 地址可用 `ip -4 addr show wlo1` 查看；不要填写机械臂控制器的 `192.168.1.13`。

已实测遇到 PC Service 将设备标为离线后，旧 TCP 连接仍收到数据，但没有向 SDK 转发设备帧的情况。仅重启遥操节点不能保证恢复；必要时重启 PC Service，然后在头显重新连接，确认出现 `PXREA: device found`、`PXREA state frames` 和 `Teleoperation armed`。这类故障不需要重新编译遥操包。正常结束时先退出遥操、再退出驱动，使 ServoJ 能正常关闭。

### 乐白夹爪与 Trigger

夹爪适配来自本机 `vr_tcb610` 项目的 `LebaiClaw` SDK 和 `CheckedClaw` 回复校验。
SDK 已复制到本包的 `tl_teleop_gripper/vendor/`，来源见其中 `UPSTREAM.md`，运行时不依赖旧项目目录或 Conda 环境。
夹爪由 PC 的独立 USB/RS485 串口控制，使用 115200-8N1、Modbus RTU 地址 1。
Python 节点需要 `rclpy` 和 `python3-serial`，依赖已写入 `package.xml`。

每个启动终端都要独立加载工作区环境，尤其是迁移夹爪前已经打开的旧终端。若夹爪进程启动即退出并提示 `ModuleNotFoundError: No module named 'tl_teleop_gripper'`，遥操会停在 `Waiting for fresh calibrated gripper feedback`。退出该遥操 launch，在准备重新启动它的终端执行以下命令；驱动和 PC Service 可以保持运行：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/tl_robot_ros2-V3.1.0/install/setup.bash
/usr/bin/python3 -c 'import tl_teleop_gripper, serial; print("gripper imports OK")'
ros2 launch tl_teleop tl_teleop_6axis.launch.py gripper_enabled:=true
```

确认模块已安装但终端缺少模块路径时，重新加载环境即可，无须重新编译。看到 `Gripper ready` 和 `Teleoperation armed` 后再操作。

首次加入这些代码后重新编译本包。已有 `tl_driver` 运行时，停止旧 `tl_teleop` 后使用：

```bash
ros2 launch tl_teleop tl_teleop_6axis.launch.py gripper_enabled:=true
```

从零启动驱动、遥操和夹爪时使用：

```bash
ros2 launch tl_teleop tl_tcb610_teleop.launch.py gripper_enabled:=true
```

两个入口的 `gripper_enabled` 默认为 `false`，未接夹爪时仍可单独遥操机械臂。
默认串口为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，可用
`gripper_port:=/dev/ttyUSB0` 覆盖。确保串口可读写且没有其他夹爪程序占用；不要同时运行旧项目的遥操入口。

操作规则：

- 启动仅查询状态，要求夹爪已经校准，不自动找行程或张开。
- 松开 Grip 和 A 完成使能，再按住 Grip 跟随。Trigger 达到 `0.5` 时闭合到 0%，低于 `0.5` 时张开到 100%；保持同一状态不会重复刷写。
- 刚握住 Grip 时，如果 Trigger 原本松开，不自动张开；先按下再松开 Trigger 才会执行开合。握住 Grip 时 Trigger 已按下则允许闭合。
- 松开 Grip、按 A 回零、VR 数据超时、遥操退出时取消尚未下发的夹爪目标，不自动松开夹持物。已经被夹爪接受的动作仍由夹爪完成。
- 串口异常、CRC/回复校验失败、未校准会锁存故障。启用夹爪时，夹爪未就绪或就绪心跳丢失也会暂停机械臂目标下发。处理故障后重启遥操及夹爪节点，再松开 Grip/A 重新使能。

`config/tl_gripper_config.yaml` 设置串口、力度、速度和 Trigger 阈值。
默认力度 `10`、速度 `20`，沿用旧项目；它们是 SDK 的 0–100 档位，不是牛顿或毫米/秒。
适配器目前允许力度 1–20、速度 1–30，仅提供全开/全闭，不提供按扳机深度连续开度控制。
参数在启动时读取；修改安装目录配置后重启节点生效。

夹爪串口查询包含同步等待，全部在独立 Python 节点的工作线程中执行，不占用机械臂的控制循环或 IK 连接。
VR 仍只由现有 `tl_teleop` 读取，不会启动第二个 PXREA 客户端。

| 话题 | 类型 | 内容 |
|------|------|------|
| `/tl_teleop/gripper_input` | `sensor_msgs/Joy` | `axes[0]` 为 Trigger；`buttons[0]` 为夹爪操作使能；时间戳对应原 VR 样本接收时间 |
| `/tl_teleop/gripper_ready` | `std_msgs/Bool` | 10 Hz 就绪心跳，需已校准且状态新鲜；遥操侧 1 秒无有效心跳即停止下发 |
| `/tl_teleop/gripper_state` | `std_msgs/String` | JSON 状态：实际开度、反馈力矩、动作完成、校准状态、状态年龄、最近指令、错误 |

夹爪故障联锁是停止后续目标下发，不是控制器急停接口。

### 配置参数说明

配置文件位于 `config/` 目录，按轴数分为两份：

- `config/tl_teleop_6axis_config.yaml` — 6 轴机械臂（TCB605 系列）
- `config/tl_teleop_7axis_config.yaml` — 7 轴机械臂（TCB710 系列）

节点名称为 `tl_teleop_node`，参数在该命名空间下配置。示例（7 轴配置）：

```yaml
tl_teleop_node:
  ros__parameters:
    # ========== 轴数配置 ==========
    arm_axis_mode: 7                 # 机械臂关节数（6 / 7）

    # ========== 位置控制参数 ==========
    pos_scale: 0.5                   # 位置缩放系数（VR 位移 1m → 末端移动 0.5m）
    pos_deadzone: 0.005              # 位置死区（m），各轴独立判定
    max_pos_delta_m: 0.3             # 单步最大位置增量（m），防止异常跳变

    # ========== 奇异点防护参数 ==========
    joint_jump_threshold: 30.0       # 关节跳变阈值（度），超过此值拒绝执行
    singular_angle: 160.0            # 奇异点判定角度（度）
    singular_scale: 0.2              # 握紧时末端关节超过阈值的位移倍率
    # 关节硬限位—扁平数组，每两个值一组 [min1, max1, min2, max2, ...]（单位：度）
    joint_limits: [-180.0, 180.0, -180.0, 180.0, -180.0, 180.0,
                   -180.0, 180.0, -180.0, 180.0, -170.0, 170.0, -170.0, 170.0]

    # ========== ServoJ 初始化参数（标量值，自动广播到所有关节）==========
    servo_speed: 25.0                # ServoJ 运动速度
    servo_vmax: 80.0                 # ServoJ 最大速度，配置用 °/s（内部转 rad/s）
    servo_amax: 3000.0               # ServoJ 最大加速度，配置用 °/s²（内部转 rad/s²）
    servo_jmax: 50000.0              # ServoJ 最大加加速度，配置用 °/s³（内部转 rad/s³）
```

**参数说明**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `arm_axis_mode` | int | — | 机械臂关节数，必须为 6 或 7 |
| `max_linear_speed` | double | 0.25 | 逆解目标的最大平移速度，m/s |
| `max_angular_speed` | double | 1.5 | 逆解目标的最大旋转速度，rad/s |
| `pos_scale` | double | 0.5 | 位置缩放系数。手柄位移 1m → 末端移动 `pos_scale` m |
| `pos_deadzone` | double | 0.005 | 位置死区（m），各轴独立判定，小于此值的位移被忽略 |
| `max_pos_delta_m` | double | 0.3 | 单步最大位置增量（m），防止异常跳变 |
| `singular_angle` | double | 160.0 | 握紧时选择位移倍率的角度（度）。6 轴检测 J5/J6，7 轴检测 J6/J7 |
| `singular_scale` | double | 0.2 | 握紧时末端关节超过阈值的位移倍率 |
| `joint_jump_threshold` | double | 30.0 | 相邻有效逆解的跳变阈值（度），超出时缩小笛卡尔步长重试 |
| `joint_limits` | double[] | — | 关节硬限位，扁平数组 `[min1, max1, min2, max2, ...]`（度） |
| `servo_speed` | double | 25.0 | ServoJ 运动速度 |
| `servo_vmax` | double | 80.0 | ServoJ 各关节最大速度（度/秒），标量值广播到所有关节 |
| `servo_amax` | double | 3000.0 | ServoJ 各关节最大加速度（度/秒²） |
| `servo_jmax` | double | 50000.0 | ServoJ 各关节最大加加速度（度/秒³） |

> 注意：`GRIP_THRESHOLD`（握紧阈值 0.9）、`CONTROL_RATE_HZ`（250Hz）和由它计算的 `CONTROL_PERIOD_S`（0.004s）为代码常量，不在 YAML 中配置。修改频率后需要重新编译并重启遥操节点。启动日志会打印设置的 ServoJ 频率；实际频率可在操作时用 `ros2 topic hz /tl_driver/set_servoj_pos` 查看，空闲模式不下发目标。

## tl_teleop功能包架构说明
### 功能包文件总览
```
├── CMakeLists.txt                     # 编译规则文件（支持 ARM/x86 架构自动检测）
├── config                             # 配置文件
│   ├── tl_teleop_6axis_config.yaml    # 6 轴遥操作参数
│   └── tl_teleop_7axis_config.yaml    # 7 轴遥操作参数
├── include                            # 头文件
│   └── tl_teleop
│       └── tl_teleop.h                # TL_Teleop 类定义
├── launch                             # 启动文件
│   ├── tl_teleop_6axis.launch.py      # 6 轴启动
│   └── tl_teleop_7axis.launch.py      # 7 轴启动
├── lib                                # PXREA Robot SDK 库（第三方预编译）
│   ├── arm
│   │   └── libPXREARobotSDK.so        # ARM64 架构预编译库
│   ├── include
│   │   └── PXREARobotSDK.h            # PXREA SDK C API 头文件
│   └── x86
│       └── libPXREARobotSDK.so        # x86_64 架构预编译库
├── package.xml                        # 依赖说明文件
├── README.md                          # 说明文档
└── src                                # 遥操作源文件
    └── tl_teleop.cpp                  # 遥操作节点实现
```

### 代码架构说明

tl_teleop 节点基于 `rclcpp::Node` 实现，采用 **双线程架构**：

- **主线程**：ROS2 事件循环（`SingleThreadedExecutor::spin()`），负责处理服务调用异步响应、话题订阅回调
- **控制线程**：250Hz 固定频率控制循环（`control_loop()`），负责读取 VR 手柄位姿、计算笛卡尔偏差、调用逆运动学、发布关节目标

**核心类 `TL_Teleop` 主要成员：**

| 类别 | 成员 | 说明 |
|------|------|------|
| 服务客户端 | `set_speed_client_` 等 6 个 | 调用 tl_driver 各项服务（不含 power_on/power_off） |
| 话题发布 | `servoj_pos_pub_` | 发布 Float64MultiArray 到 `/tl_driver/set_servoj_pos` |
| 话题订阅 | `tcp_pose_sub_` | 订阅 `/tcp_pose` 获取当前末端位姿 |
| PXREA 回调 | `on_pxrea_client_cb()` | PXREA SDK 事件回调（设备连接、断开、状态更新） |
| VR 状态 | `VRState` | 线程安全的手柄姿态缓存结构体（含互斥锁） |
| 运动解算 | `get_inverse_kinematics()` | 通过 tl_driver 服务进行笛卡尔 → 关节逆解 |
| 安全保护 | `clamp_joints()` / `joints_safe()` | 关节限位裁剪 / 跳变检测 |
| 四元数运算 | `quat_multiply()` / `quat_inverse()` / `quat2rpy()` | 姿态差值运算 |
| 位姿缓存 | `latest_tcp_pose_` | 互斥锁保护的 `/tcp_pose` 最新值缓存 |

**控制流程**：
```
VR手柄位姿 → 握紧触发 → 记录基准位姿 → 计算偏差(位置+姿态) → 坐标映射(VR→机械臂)
→ 奇异点减速 → 逆运动学求解 → 关节跳变检测 → 关节限位裁剪
→ 发布到 /tl_driver/set_servoj_pos → tl_driver 执行
```

## tl_teleop功能包话题与服务说明

tl_teleop 作为客户端节点，主要使用（消费）tl_driver 提供的话题与服务，并将其转化为机械臂运动指令。

### 发布的话题
```
  Publishers:
    /tl_driver/set_servoj_pos: std_msgs/msg/Float64MultiArray
```
| 话题 | 类型 | 说明 |
|------|------|------|
| `/tl_driver/set_servoj_pos` | `std_msgs/Float64MultiArray` | 目标关节角度数组（单位：rad），跟随和回零时以250Hz目标频率发布 |

### 订阅的话题
```
  Subscribers:
    /tcp_pose: tl_ros2_interface/msg/CartesianPose
```
| 话题 | 类型 | 说明 |
|------|------|------|
| `/tcp_pose` | `tl_ros2_interface/msg/CartesianPose` | 获取机械臂当前末端位姿（position 单位 m，rpy 单位 rad），用于基准点标定 |

### 调用的服务（客户端）
```
  Service Clients:
    /tl_driver/set_speed: tl_ros2_interface/srv/SetSpeed
    /tl_driver/set_current_mode: tl_ros2_interface/srv/SetCurrentMode
    /tl_driver/open_servoj: tl_ros2_interface/srv/OpenServoJ
    /tl_driver/close_servoj: std_srvs/srv/Trigger
    /tl_driver/coord_transform: tl_ros2_interface/srv/CoordTransform
```

以上 **5 个服务**均为调用 tl_driver 提供的接口，tl_teleop 本身不提供服务端。

### 控制逻辑说明

1. **启动初始化** → 调用 `set_current_mode`（模式 2）、`set_speed`、`open_servoj`。各步骤均有 5 秒超时保护，失败则终止（tl_driver 自动处理上电）
2. **遥操作循环**（250Hz）：
   - 读取 VR 位姿 → 按住 A 键时逐步回零
   - 握下扳机时以当前末端位姿为基准（记录位置 + RPY → 四元数）
   - 手柄相对于基准的偏移映射为笛卡尔增量
   - VR 坐标系 → 机械臂坐标系映射（VR: X右 Y上 Z前 → 机械臂: X前 Y左 Z上）
   - 姿态使用四元数差值计算最短旋转路径
   - 奇异点检测 → 获取异步逆解（一次等待一个结果）→ 关节跳变及限位检查 → 按速度、加速度约束逐周期发布目标
3. **安全退出** → 停止控制循环 → 异步关闭 ServoJ → PXREA SDK 反初始化
