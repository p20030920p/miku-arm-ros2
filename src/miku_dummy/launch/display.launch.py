#!/usr/bin/env python3
# ============================================================================
# display.launch.py —— ROS 2 Jazzy 移植版（原 display.launch）
#
# 作用：加载 miku_dummy 的 URDF，启动 robot_state_publisher 发布 TF，
#       并打开 RViz2 显示模型。
#
# ROS 1 → ROS 2 差异：
#   - XML launch → Python launch（ROS 2 的 launch 语法重写）
#   - <param name="robot_description" textfile="...">  →
#     robot_state_publisher 的 robot_description 参数（由 launch 读文件后传入）
#   - pkg="rviz" type="rviz" → pkg="rviz2" executable="rviz2"
#   - 原文件里注释掉的 joint_state_publisher_gui 保留为可选开关
#     use_joint_state_publisher_gui（默认 false，与原行为一致）
#
# 用法：
#   ros2 launch miku_dummy display.launch.py
#   ros2 launch miku_dummy display.launch.py use_joint_state_publisher_gui:=true
# ============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('miku_dummy')

    urdf_path = os.path.join(pkg_share, 'urdf', 'miku_dummy.urdf')
    rviz_config = os.path.join(pkg_share, 'urdf_ros2.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    # 与 ROS 1 的 <arg name="model" /> 对应（原文件声明了但未实际使用，
    # 这里保留同名参数以兼容既有调用习惯）
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=urdf_path,
        description='URDF 模型文件路径（原 ROS 1 launch 的 model 参数）')

    use_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui',
        default_value='false',
        description='是否启动 joint_state_publisher_gui（原文件中被注释掉，默认关闭）')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 原 ROS 1 launch 里这段是被注释掉的，这里做成可选开关，默认不启动
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher_gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        model_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
