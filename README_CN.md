<div align="center">

# miku-arm-ros2

**达妙电机六轴机械臂（含夹爪）的 ROS 2 Jazzy 驱动与控制库**

<sub>KDL 正逆运动学 · 重力补偿 · 轨迹复现与示教 · ArUco / RealSense 位姿估计</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#构建)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](#构建)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)

[构建](#构建) &nbsp;•&nbsp; [使用硬件](#使用硬件) &nbsp;•&nbsp; [无硬件仿真](#无硬件仿真) &nbsp;•&nbsp; [验证](#验证) &nbsp;•&nbsp; [来源](#来源)

*[English](README.md) &nbsp;|&nbsp; 中文*

</div>

<p align="center">
  <img src="docs/figures/demo.gif" width="760"
       alt="录制的示教轨迹在仿真机械臂上回放"/>
</p>

*录制的示教轨迹在仿真中回放：网格取自 URDF，位姿取自回放过程的 `/joint_states`。
50 秒、100 Hz。*

<p align="center">
  <img src="docs/figures/demo_hardware.gif" width="400"
       alt="实验台上的自制机械臂跟随 RViz2 运动规划面板里拖动的目标"/>
</p>

*实机演示。实验台上的自制机械臂，跟随 RViz2 的 MotionPlanning 面板里拖动的目标，走的是与上面仿真同一套
`trajectory_bridge` 链路。*

机械臂为六个达妙电机加一个夹爪，挂在同一块 MCU 驱动板上，通过 `/dev/ttyACM0` 以「下行
50 字节 / 上行 46 字节」的二进制协议通信。

| | |
|---|---|
| **软件包** | `arm_control` · `hardware` · `deep_camera` · `aruco` · `miku_dummy` · `miku_dummy_moveit_config` · `miku_sim` · `miku_moveit_demo` |
| **控制** | KDL 正逆解、直线插补、六通道重力补偿、四态夹爪状态机 |
| **模式** | `mode=1` MIT（刚度、阻尼、力矩前馈） · `mode=2` 限速位置控制 |
| **验证** | 13 项检查，无需接硬件——串口协议 7 项、控制链路 6 项 |
| **来源** | 由 ROS 1 Noetic 工程移植，原始工作空间见 [`reference/`](reference/) |

## 为什么要做这两套仿真

改动运动学、重力补偿或夹爪状态机之后，不必等机械臂在场就能验证。`sim_motor_board` 在 ROS 侧替换驱动板；
`virtual_motor_board.py` 实现驱动板一侧的协议（`0x86C1` / `0x86C2` 帧头、字段偏移、×1000 定点），
真实的 `hardware` 二进制经 `socat` 的 PTY 配对跑在它上面。

![两条路径运行同一批 arm_control 二进制，仅电机接口不同](docs/figures/architecture.png)

控制器不经过 MoveIt 或 `ros2_control`，而是自己跑 KDL 运动学、以 MIT 模式下发电机指令。

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

### 用 MoveIt 2 规划

```bash
ros2 launch miku_moveit_demo moveit_demo.launch.py
```

该启动会拉起仿真电机、`move_group`、`trajectory_bridge` 与 RViz2。在三维视图里拖动交互标记设定目标，
再点 MotionPlanning 面板的 **Plan** 与 **Execute**。`trajectory_bridge` 以 50 Hz 把轨迹作为
`ArmMsg(mode=2)` 发布到 `/Arm_tx`；`speed_scale:=0.2` 放慢回放便于演示，`record:=true`
把 RViz 切到录制用的单面板布局。

## 验证

```bash
ros2 run miku_sim run_serial_hil_test.sh     # 串口协议，7 项
ros2 run miku_sim run_sim_e2e_test.sh        # 控制链路，6 项
```

两套测试都只跑 CPU，在一台轻薄本上完成（华为 MateBook 14，Intel Core i5-1240P）；整条链路不需要 GPU。

| 套件 | 验证内容 |
|---|---|
| `run_serial_hil_test.sh` | 驱动板初始化与双向帧流；六关节定位精度 < 0.002 rad；MIT 力矩前馈符号与幅值；重力下垂被前馈消除；夹爪接触后卡住；运行中拔掉驱动板节点不退出 |
| `run_sim_e2e_test.sh` | `/joint_states` 频率 100 Hz；TF 树 `base_link → link_6` 完整；IK 闭环使机械臂运动；重力补偿悬停漂移 0.0000 rad；复现录制的示教文件 7 325 点；夹爪状态机到达「已夹到」 |

各项断言内容与未覆盖范围见 [`docs/TESTING.md`](docs/TESTING.md)。

## 来源

原始工程是 ROS 1 Noetic 的 catkin 工作空间，未作修改地保留在 [`reference/`](reference/) 下并加
`COLCON_IGNORE`。本仓库是它的 ROS 2 Jazzy 移植，并补了 MoveIt 2 配置与演示、两套仿真替代品及其测试，以及
[`CHANGELOG.md`](CHANGELOG.md) 中列出的修复——其中包括 MoveIt 关节限位文件里 `max_acceleration: 0`
导致轨迹"规划成功却从不执行"。

## 文档

各包内容与参考文档：

- [`arm_control`](src/arm_control) —— 运动学、重力补偿、直线规划、夹爪状态机、控制节点
- [`hardware`](src/hardware) —— 串口节点、轨迹复现、示教录制、测试节点
- [`miku_sim`](src/miku_sim) —— 仿真电机、虚拟驱动板、两套测试
- [`deep_camera`](src/deep_camera) · [`aruco`](src/aruco) —— RealSense RGB-D 采集与 ArUco 位姿估计
- [`miku_dummy`](src/miku_dummy) · [`miku_dummy_moveit_config`](src/miku_dummy_moveit_config) · [`miku_moveit_demo`](src/miku_moveit_demo) —— URDF、meshes、MoveIt 2 配置与规划演示

[`docs/OVERVIEW.md`](docs/OVERVIEW.md) 控制链路、话题、仿真设计 ·
[`docs/PORTING.md`](docs/PORTING.md) ROS 1 → ROS 2 映射规则 ·
[`docs/TESTING.md`](docs/TESTING.md) 测试套件、覆盖范围与缺口 ·
[`docs/RECORDING.md`](docs/RECORDING.md) 录制演示 ·
[`CHANGELOG.md`](CHANGELOG.md) 版本记录。

## 许可

MIT。随包附带的 ArUco 检测器为 MIT，© 2017 Tentone —— 见 [`src/aruco/LICENSE`](src/aruco/LICENSE)。
