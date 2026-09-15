<div align="center">

# miku-arm-ros2

**达妙电机六轴机械臂（含夹爪）的 ROS 2 Jazzy 驱动与控制库**

<sub>KDL 正逆运动学 · 重力补偿 · 轨迹复现与示教 · ArUco / RealSense 位姿估计</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#构建)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](#构建)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)

[构建](#构建) &nbsp;•&nbsp; [使用硬件](#使用硬件) &nbsp;•&nbsp; [无硬件仿真](#无硬件仿真) &nbsp;•&nbsp; [验证](#验证) &nbsp;•&nbsp; [文档](#文档)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

<p align="center">
  <img src="docs/figures/demo.gif" width="760"
       alt="录制的示教轨迹在仿真机械臂上回放"/>
</p>

*录制的示教轨迹在仿真中回放：网格取自 URDF，位姿取自回放过程的 `/joint_states`。
50 秒、100 Hz。*

<p align="center">
  <img src="docs/figures/demo_rviz.gif" width="760"
       alt="在 RViz2 的 MotionPlanning 面板里规划并执行"/>
</p>

*在 RViz2 的 MotionPlanning 面板里规划并执行：MoveIt 为 `manipulator` 规划轨迹，
`trajectory_bridge` 转成 `ArmMsg(mode=2)` 下发，仿真机械臂跟随。录制时回放放慢到 0.18×，
实际到位误差 0.01 rad 以内。*

机械臂为六个达妙电机加一个夹爪，挂在同一块 MCU 驱动板上，通过 `/dev/ttyACM0` 以「下行
50 字节 / 上行 46 字节」的二进制协议通信。

| | |
|---|---|
| **软件包** | `arm_control` · `hardware` · `deep_camera` · `aruco` · `miku_dummy` · `miku_dummy_moveit_config` · `miku_sim` · `miku_moveit_demo` |
| **控制** | KDL 正逆解、直线插补、六通道重力补偿、四态夹爪状态机 |
| **模式** | `mode=1` MIT（刚度、阻尼、力矩前馈） · `mode=2` 限速位置控制 |
| **仿真** | 无需硬件：仿真电机 + 协议级虚拟驱动板 |
| **来源** | 由 ROS 1 Noetic 工程移植，原始工作空间见 [`reference/`](reference/) |

## 构建

需要 Ubuntu 24.04 与 ROS 2 Jazzy。

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/p20030920p/miku-arm-ros2.git
cd miku-arm-ros2
colcon build --symlink-install
source install/setup.bash
```

## 使用硬件

先启动模型显示与串口节点，再选择控制器：

```bash
ros2 launch miku_dummy display.launch.py     # robot_state_publisher + RViz2
ros2 run hardware hardware                   # 串口节点
```

```bash
# 键盘输入笛卡尔目标位姿
ros2 run arm_control arm_control_node

# 手动示教：仅重力补偿，kp = 0
ros2 run arm_control teach_one_node

# 复现 hardware/teach_path 下录制的轨迹
ros2 run hardware trajectory_track
```

串口默认 `/dev/ttyACM0`、115200，可通过参数修改：

```bash
ros2 run hardware hardware --ros-args -p serial_port:=/dev/ttyACM1
```

`./launch_hardware.sh`、`./launch_arm_teach_one_node.sh`、`./launch_claw_test.sh`
封装了上述流程，并在退出时自动归零。

## 无硬件仿真

启动仿真机械臂，用同一批控制器节点驱动：

```bash
ros2 launch miku_sim sim.launch.py           # 仿真电机 + RViz2
ros2 run hardware trajectory_track           # 复现录制的轨迹
```

`miku_sim` 提供两个替代品，覆盖不同的层：

- **`sim_motor_board`** 在 ROS 侧替换驱动板并发布 `/joint_states`，使控制回路在仿真中闭环。
- **`virtual_motor_board.py`** 实现驱动板一侧的协议 —— `0x86C1` / `0x86C2` 帧头、字段偏移、
  ×1000 定点。经 `socat` 的 PTY 配对，用它测试串口协议，被测对象是真实的 `hardware` 二进制。

![两条路径运行同一批 arm_control 二进制，仅电机接口不同](docs/figures/architecture.png)

控制器不经过 MoveIt 或 `ros2_control`，而是自己跑 KDL 运动学、以 MIT 模式下发电机指令。

## 验证

```bash
ros2 run miku_sim run_serial_hil_test.sh     # 串口协议，7 项
ros2 run miku_sim run_sim_e2e_test.sh        # 控制链路，6 项
```

| 串口协议 | 结果 |
|---|---|
| 驱动板初始化、双向帧流 | 通过 |
| 六关节定位精度 | < 0.002 rad |
| MIT 力矩前馈符号与幅值 | 通过 |
| 重力下垂被前馈消除 | 通过 |
| 夹爪接触后卡住 | 通过 |
| 运行中拔掉驱动板 | 节点不退出 |

| 控制链路 | 结果 |
|---|---|
| `/joint_states` 频率 | 100 Hz |
| TF 树 `base_link → link_6` | 完整 |
| IK 闭环使机械臂运动 | 通过 |
| 重力补偿悬停漂移 | 0.0000 rad |
| 复现录制的示教文件 | 7 325 点 |
| 夹爪状态机到达「已夹到」 | 通过 |

各项断言内容与未覆盖范围见 [`docs/TESTING.md`](docs/TESTING.md)。

## 软件包

| 包 | 内容 |
|---|---|
| `arm_control` | 运动学、重力补偿、直线规划、夹爪状态机、控制节点 |
| `hardware` | 串口节点、轨迹复现、示教录制、测试节点 |
| `miku_sim` | 仿真电机、虚拟驱动板、两套测试 |
| `deep_camera` | RealSense RGB-D 采集与 ArUco 位姿估计 |
| `aruco` | ArUco 检测器、ROS 2 节点、标记制作资料 |
| `miku_dummy` | URDF、meshes、RViz2 配置 |
| `miku_dummy_moveit_config` | MoveIt 2 配置（SRDF、规划器参数） |
| `miku_moveit_demo` | MoveIt 2 + RViz2 规划演示，轨迹桥接到 `ArmMsg` |

## 文档

| | |
|---|---|
| [`docs/OVERVIEW.md`](docs/OVERVIEW.md) | 控制链路、话题、仿真设计 |
| [`docs/PORTING.md`](docs/PORTING.md) | ROS 1 → ROS 2 映射规则 |
| [`docs/TESTING.md`](docs/TESTING.md) | 测试套件、覆盖范围与缺口 |
| [`docs/RECORDING.md`](docs/RECORDING.md) | 录制演示 |
| [`CHANGELOG.md`](CHANGELOG.md) | 版本记录 |

## 许可

MIT。随包附带的 ArUco 检测器为 MIT，© 2017 Tentone —— 见 [`src/aruco/LICENSE`](src/aruco/LICENSE)。
