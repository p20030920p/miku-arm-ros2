<div align="center">

# miku-arm-ros2

**达妙电机 6 轴机械臂 + 夹爪，从 ROS 1 Noetic 移植到 ROS 2 Jazzy —— 并在没有实机的情况下完成验证**

<sub>catkin → ament · <b>roscpp</b> → rclcpp · 逐字节复现二进制串口协议 · 仿真中闭环</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#快速开始)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![Packages](https://img.shields.io/badge/colcon%20packages-7%20passing-success)](#快速开始)
[![Protocol](https://img.shields.io/badge/串口协议-7%2F7%20通过-success)](#验证)
[![Simulation](https://img.shields.io/badge/仿真端到端-6%2F6%20通过-success)](#验证)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)

[快速开始](#快速开始) &nbsp;•&nbsp; [为什么移植](#为什么移植) &nbsp;•&nbsp; [架构](#架构) &nbsp;•&nbsp; [验证](#验证) &nbsp;•&nbsp; [无实机仿真](#无实机仿真) &nbsp;•&nbsp; [文档](#文档)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

![系统架构、真实示教轨迹回放、串口下发与回读对比、重力下垂与力矩前馈补偿](docs/figures/hero.png)

*四个面板全部来自真实运行数据。**A** 控制链路，话题名与代码逐字一致。**B** 真实录制的示教轨迹
在仿真臂上回放时的关节角 —— 2 201 帧 @100 Hz；`joint_6` 扫过 1.36 rad，t = 7.5 s 处的小台阶
来自轨迹本身的跳变，不是滤波。**C** ×1000 定点串口编码往返误差 0.0000 rad。**D** 0.45 N·m
重力负载使 `joint_2` 在位置模式下下垂 0.056 rad，MIT + 力矩前馈下为 0.000 rad。*

这是把一套原本能在 ROS 1 上跑通的机械臂控制程序重建到 ROS 2。机械臂是 6 个达妙电机加一个夹爪，
挂在同一块 MCU 驱动板上，通过 `/dev/ttyACM0` 用「下行 50 字节 / 上行 46 字节」的二进制协议通信。
原始工程原样保留在 `reference/ros1-original/`，`src/` 下是 Jazzy 移植版。

| | |
|---|---|
| **移植范围** | 5 个 catkin 包 → 7 个 ament 包 · **21 个可执行文件** · 8 082 行 C++ |
| **串口协议** | 对逐字节等价的虚拟驱动板 **7/7** 项通过 —— 无需硬件 |
| **闭环控制** | 仿真端到端 **6/6** 项通过，`/joint_states` **100 Hz** |
| **示教轨迹** | **38 610 行**真实录制轨迹可在仿真臂上回放 |
| **行为保持** | 所有阈值、增益、帧偏移未改；另修正 3 处上游缺陷（见下） |
| **未验证** | 实机物理特性（真实刚度、摩擦、真实重力负载）· RealSense 实物相机 |

---

## 为什么移植

原工程是 ROS 1 Noetic 的 catkin 工作空间。Noetic 不支持 Ubuntu 24.04，在本机无法安装，
因此把代码迁到已有的 ROS 2 Jazzy，而不是想办法搭一个 ROS 1 环境。

这是**翻译，不是重新设计**。凡 ROS 2 确实没有对应物的地方，源码中都标注了
`// ROS 2 差异：`，并在 [`docs/PORTING.md`](docs/PORTING.md) 中说明。

顺带修正了两处**上游缺陷**——它们都让某个功能静默失效：

| 位置 | 原来 | 现在 |
|---|---|---|
| `hardware.cpp` | `catch (SerialException&)` 写在 `catch (IOException&)` 之前，后者**永远不可达** | 调换顺序，设备消失时能如实报成 I/O 异常 |
| `aruco` | `topic_marker_remove` 被读进了 *register* 变量，导致「移除标记」从未生效 | 每个参数读入各自变量 |

第三处是真正的 API 变化而非缺陷：orocos_kdl 1.5 的 `KDL::Tree::getChain` 返回 `bool`
（原来是 `int`），所以旧的 `< 0` 失败判断恒为假。

## 架构

![实机链路与仿真链路共用全部算法节点，只有电机接口不同](docs/figures/architecture.png)

原工程**不经过 MoveIt，也不经过 `ros2_control`**。它自己用 KDL 做正逆运动学、自己做直线插值，
并以 MIT 模式直接下发电机指令 —— 位置刚度 + 重力前馈力矩。这对可测试性很关键：整个控制器就是
架在两个话题上的普通 ROS 节点，所以替换电机板就等于替换了实机与仿真之间的**全部**差异。

| 话题 | 类型 | |
|---|---|---|
| `Arm_tx` | `arm_control/msg/ArmMsg` | 下行设定值：7 个电机 × 位置/速度/力矩，外加 `kp`、`kd`、模式 |
| `Arm_rx` | `arm_control/msg/ArmMsg` | 上行实测状态，速率由驱动板决定 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 100 Hz，来自机械臂回读 |

设计由两个模式承载。`mode=2` 是限速位置控制，用于归零与轨迹复现；`mode=1` 是 MIT ——
当 `kp = 0` 时机械臂可被反驱，仅靠重力补偿就能悬停，示教就是这么做的：你用手拖动机械臂，
1 kHz 录制把路径记下来。

## 快速开始

```bash
source /opt/ros/jazzy/setup.bash          # 必须先执行：提供 ros2 与 colcon
git clone https://github.com/p20030920p/miku-arm-ros2.git
cd miku-arm-ros2
colcon build --symlink-install            # 在仓库根目录构建
source install/setup.bash                 # 每个新终端都要重新 source
```

**不需要硬件**即可启动仿真机械臂与显示：

```bash
ros2 launch miku_sim sim.launch.py                     # 仿真电机 + RViz2
ros2 launch miku_sim sim.launch.py use_rviz:=false      # 无界面

# 另开终端：把真实录制的示教轨迹在仿真臂上回放
ros2 run hardware trajectory_track                      # 会提示输入 teach_path 下的文件名
```

接实机时，同一批节点，只是把串口设备接上：

```bash
ros2 launch miku_dummy display.launch.py      # robot_state_publisher + RViz2
ros2 run hardware hardware                    # 串口节点；-p serial_port:=/dev/ttyACM0
ros2 run arm_control teach_one_node           # 仅重力补偿下手动示教
```

或使用一键脚本（退出时会自动安全归零）：

```bash
./launch_hardware.sh                # 显示 + 串口节点
./launch_arm_teach_one_node.sh      # 再加示教节点
./launch_claw_test.sh               # 再加夹爪循环测试
```

## 验证

实机不可用，所以把**两件否则无法检查的事**各自复现了一遍：驱动板，和机械臂本身。

### 串口协议 —— 对虚拟驱动板

`virtual_motor_board.py` 实现了导线的另一端：帧头 `0x86C1` / `0x86C2`、50 与 46 字节帧、
字段偏移、×1000 定点编码。通过 `socat` 造的 PTY 配对后，**真正的 `hardware` 二进制**
可以不加修改地跑起来。

```bash
ros2 run miku_sim run_serial_hil_test.sh
```

| 检查项 | 结果 |
|---|---|
| 串口打开并初始化 | ✅ |
| 双向帧流，`/Arm_rx` 回读 207 条 | ✅ |
| 位置模式 6 关节保持设定值 —— **最大误差 < 0.002 rad** | ✅ |
| MIT 模式力矩前馈，符号与幅值（+0.5 / −0.5 / +0.25 N·m） | ✅ |
| 位置模式重力下垂（−0.056 rad）被前馈消除（0.000 rad） | ✅ |
| 夹爪接触物体后停在 −1.2 rad、力矩升到 0.6 N·m | ✅ |
| 运行中拔掉驱动板不会导致节点退出 | ✅ |

最后一项正是原代码围绕的场景：MCU 复位、串口抛 `IOException`，节点必须关闭并重连，
而不是直接死掉。

### 控制链路 —— 在仿真中

```bash
ros2 run miku_sim run_sim_e2e_test.sh
```

| 检查项 | 结果 |
|---|---|
| 仿真链路启动，`/joint_states` **100 Hz** | ✅ |
| TF 树完整，`base_link → link_1 … → link_6` | ✅ |
| `arm_control_node` IK 闭环：目标位姿 → 逆解 → **机械臂真的动了**（0.056 rad） | ✅ |
| `teach_one_node` 负载下保持姿态 —— 12 秒漂移 **0.0000 rad** | ✅ |
| 真实示教文件回放：**加载 7 325 点，下发 2 229 点** | ✅ |
| 夹爪状态机在接触后进入「已夹到」 | ✅ |

### 明确**未**验证的部分

这些很重要，所以单独列出：

- **实机物理特性。** 协议已验证到字节级、控制律已在仿真中验证，但真实电机刚度、摩擦与
  真实重力负载**没有**验证。`arm_control_params.yaml` 里的增益是在实机上手工调出来的、
  本次未改动，因此"应该仍然合适"—— 这是判断，不是测量。
- **RealSense D435。** 没有相机。`deep_camera` 与 `aruco` 编译通过，并用合成图像验证过
  （正确读出 750 mm 深度、识别出标记 `ID=341`、解出完整位姿），但没有接触过真实传感器数据。
- **关节限位。** `KDL::ChainIkSolverPos_LMA` 不考虑限位，上游如此、此处亦然。不可达目标会打印
  `IK 失败，本步跳过` 并被跳过。这是**继承来的行为，有意不去"改进"**。
- **图形窗口交互。** 本机 Qt5 highgui 起不了 X 窗口（连一个 15 行的非 ROS `imshow` 程序也一样），
  所以 `deep_camera` 里的鼠标取点与 ESC 退出路径从未被点击过。代码路径 1:1 保留。

## 无实机仿真

`miku_sim` 里放了两个替代品。它们回答不同的问题，因此**有意分开**：

| | 回答什么问题 | 机制 |
|---|---|---|
| `virtual_motor_board.py` | *串口协议对不对？* | 坐在 PTY 的另一端，说真实的帧格式 —— 被测对象是**真正的 `hardware` 二进制** |
| `sim_motor_board` | *控制律对不对？* | 在 ROS 侧替换驱动板，发布 `/joint_states`，让 RViz2 与 TF 显示机械臂运动 |

两者都建模了 `joint_2`、`joint_3` 上的重力负载，也都在夹爪接触物体时卡住。
重力在**位置模式下也被建模**（表现为刚度受限的稳态下垂）—— 否则重力补偿无从体现，
测试也就证明不了任何事。这是本仿真器第一版的真实缺陷，也是面板 D 存在的原因。

`gz sim` 8.11 已安装且能运行，但 `gz_ros2_control` 的 `GazeboSimSystem` 只在 `gz-sim`
进程内部注册，独立 `controller_manager` 加载会报 `plugin does not exist`。既然原工程本就
不用 `ros2_control`，去掉它没有任何损失，反而让仿真链路与实机链路保持一致。

## 目录结构

```
src/
  arm_control/                KDL 正逆解 · 直线规划 · 重力补偿 · 夹爪状态机
  hardware/                   串口节点 + 轨迹复现 + 示教录制 + 各测试节点
  miku_sim/                   虚拟驱动板、仿真电机、两套自动化测试
  miku_dummy/                 URDF、meshes、RViz2 配置
  miku_dummy_moveit_config/   MoveIt 2 配置（SRDF + 规划器参数；MoveIt 1 的 launch 已替换）
  aruco/                      ArUco 检测器、ROS 2 节点、标记制作资料
  deep_camera/                RealSense RGB-D 采集、位姿估计、视觉引导抓取
docs/                         PORTING.md、TESTING.md、figures/
reference/ros1-original/      原始 ROS 1 工作空间，未改动，供对照
tools/                        配图生成脚本、关节状态采集
```

`reference/` 下放了一个 `COLCON_IGNORE`，避免其中的 ROS 1 包与移植版包名冲突。

## 文档

| | |
|---|---|
| [`docs/PORTING.md`](docs/PORTING.md) | 全工程使用的 ROS 1 → ROS 2 映射规则，以及每一处不适用之处 |
| [`docs/TESTING.md`](docs/TESTING.md) | 两套测试怎么跑、每项断言什么、配图如何重新生成 |

## 许可

MIT。随包附带的 `aruco` 检测器为 MIT，© 2017 Tentone —— 见 [`src/aruco/LICENSE`](src/aruco/LICENSE)。
