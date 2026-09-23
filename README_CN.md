<div align="center">

# miku-arm-ros2

**达妙电机六轴机械臂（含夹爪）的 ROS 2 Jazzy 驱动与控制库**

<sub>KDL 正逆运动学 · 重力补偿 · 轨迹复现与示教 · ArUco / RealSense 位姿估计</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#构建)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](#构建)

[构建](#构建) &nbsp;•&nbsp; [使用硬件](#使用硬件) &nbsp;•&nbsp; [无硬件仿真](#无硬件仿真)

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
50 字节 / 上行 46 字节」的二进制协议通信。控制器不经过 MoveIt 或 `ros2_control`，而是自己跑
KDL 运动学、以 MIT 模式下发电机指令。

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
