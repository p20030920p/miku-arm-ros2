#!/usr/bin/env python3
# ============================================================================
# move_group.launch.py —— MoveIt 2 版的 move_group 启动（替代 MoveIt 1 的一组
# planning_context / move_group / trajectory_execution / planning_pipeline ... launch）
#
# ROS 1 → ROS 2 差异：
#   MoveIt 1 需要 planning_context.launch + move_group.launch +
#   ompl_planning_pipeline.launch.xml 等多个文件层层 include；
#   MoveIt 2 用 moveit_configs_utils 一次性装载 URDF/SRDF/各 yaml，代码量大幅减少。
#
# 用法：
#   ros2 launch miku_dummy_moveit_config move_group.launch.py
#   ros2 launch miku_dummy_moveit_config move_group.launch.py use_rviz:=true
#
# 注意：本工程的主控制回路【不经过 MoveIt】（见 README_ROS2.md），
#       这个包是给需要 MoveIt 规划/可视化时使用的可选组件。
# ============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('miku_dummy_moveit_config')

    # MoveIt 2 的标准做法：一次性装载 robot_description / SRDF / 各规划器配置
    # URDF 用 arm_control 包里的 miku_arm.urdf（与 SRDF 里的 miku_arm 对应）
    moveit_config = (
        MoveItConfigsBuilder('miku_arm', package_name='miku_dummy_moveit_config')
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('arm_control'),
                'urdf', 'miku_arm.urdf'))
        .robot_description_semantic(
            file_path=os.path.join(pkg_share, 'config', 'miku_arm.srdf'))
        .robot_description_kinematics(
            file_path=os.path.join(pkg_share, 'config', 'kinematics.yaml'))
        .joint_limits(
            file_path=os.path.join(pkg_share, 'config', 'joint_limits.yaml'))
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = os.path.join(pkg_share, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否同时启动 RViz2'),
        move_group_node,
        rviz_node,
    ])
