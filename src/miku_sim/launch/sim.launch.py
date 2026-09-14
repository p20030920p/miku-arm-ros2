#!/usr/bin/env python3
# ============================================================================
# sim.launch.py —— 无实机仿真启动（ROS 2 Jazzy）
#
# 干什么：把整个控制链路搬进仿真，不需要任何硬件、也不需要串口。
#
#   trajectory_track / teach_one_node / arm_control_node   (真实节点，未改动)
#                │  Arm_tx
#                ▼
#   sim_motor_board ── 模拟达妙电机（mode=1 MIT / mode=2 速度位置
#                │      + 重力负载 + 夹爪夹到物体）
#                ├── /Arm_rx  ──> 上层算法（形成闭环，与实机完全一致）
#                └── /joint_states ──> robot_state_publisher ──> TF ──> RViz2
#
# 为什么不用 ros2_control / Gazebo：
#   原工程（ROS 1）的控制回路本来就不经过 ros2_control —— 它自己用 KDL 做
#   运动学 + 直线插值，直接以 MIT 模式下发 ArmMsg。所以仿真只需要一个忠实的
#   "虚拟驱动板"（sim_motor_board），链路才与实机一致。
#   （本机也没有可用于独立 controller_manager 的 Gazebo 硬件插件：
#     gz_ros2_control/GazeboSimSystem 只在 gz-sim 进程内部注册，独立
#     controller_manager 加载它会报 plugin does not exist。RViz2 已能完整
#     显示机械臂与 TF，因此不依赖该插件。）
#
# 用法：
#   ros2 launch miku_sim sim.launch.py                     # 仿真 + RViz2
#   ros2 launch miku_sim sim.launch.py use_rviz:=false      # 纯命令行
#
# 然后另开终端跑真实算法节点：
#   ros2 run hardware trajectory_track       # 轨迹复现（读 teach_path/*.txt）
#   ros2 run arm_control teach_one_node      # 示教 / 重力补偿悬停
#   ros2 run arm_control arm_control_node    # 键盘输入目标位姿
# ============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('miku_sim')
    miku_share = get_package_share_directory('miku_dummy')

    urdf_path = os.path.join(sim_share, 'urdf', 'miku_dummy_sim.urdf')
    if not os.path.exists(urdf_path):
        urdf_path = os.path.join(
            get_package_share_directory('arm_control'),
            'urdf', 'miku_dummy_sim.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration('use_rviz')
    gravity_load = LaunchConfiguration('gravity_load')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 虚拟驱动板：模拟电机 + 回读，链路与实机一致（只少了串口那一段）
    motor_board = Node(
        package='miku_sim',
        executable='sim_motor_board',
        name='sim_motor_board',
        output='screen',
        parameters=[{
            'gravity_load': gravity_load,
            'grip_object': True,
            'rate': 100.0,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(miku_share, 'urdf_ros2.rviz')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('gravity_load', default_value='true',
                              description='仿真电机是否施加类重力负载'),
        rsp,
        motor_board,
        rviz,
    ])
