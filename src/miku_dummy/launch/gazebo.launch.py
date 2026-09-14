#!/usr/bin/env python3
# ============================================================================
# gazebo.launch.py —— ROS 2 Jazzy 移植版（原 gazebo.launch）
#
# 作用：在 Gazebo 中生成 miku_dummy 模型，并发布 base_link → base_footprint
#       的静态 TF。
#
# ROS 1 → ROS 2 差异：
#   - <include file="$(find gazebo_ros)/launch/empty_world.launch" />
#     → 直接启动 gzserver / gzclient（ROS 2 的 gazebo_ros 不再提供该 launch）
#   - pkg="gazebo_ros" type="spawn_model" args="-file ... -urdf -model ..."
#     → Node(package='gazebo_ros', executable='spawn_entity.py',
#            arguments=['-file', ..., '-entity', ...])
#   - pkg="tf" type="static_transform_publisher" → tf2_ros 的
#     static_transform_publisher，参数形式由 "x y z yaw pitch roll parent child period"
#     改为 ROS 2 的 --x --y --z --frame-id --child-frame-id
#   - 原文件中 fake_joint_calibration（rostopic pub /calibrated）是
#     ROS 1 的 gazebo 校准机制，ROS 2 无对应话题，已移除。
#
# 注意：需要安装 gazebo_ros 与 gazebo（例如 ros-jazzy-gazebo-ros-pkgs）。
#       未安装时本 launch 会在启动 gazebo 时失败，属预期行为。
#
# 用法：
#   ros2 launch miku_dummy gazebo.launch.py
# ============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('miku_dummy')
    urdf_path = os.path.join(pkg_share, 'urdf', 'miku_dummy.urdf')

    # gazebo_ros 提供了 gazebo.launch.py（ROS 2 版本的空世界启动）
    # 未安装 gazebo_ros 时明确报错，而不是让用户面对一个含义不明的异常
    try:
        gazebo_share = get_package_share_directory('gazebo_ros')
    except Exception as exc:
        raise RuntimeError(
            "找不到 gazebo_ros 包，无法启动仿真。请先安装，例如：\n"
            "  sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros\n"
            f"（原始错误：{exc}）")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')))

    # 生成模型（对应 ROS 1 的 spawn_model -file ... -urdf -model miku_dummy）
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        output='screen',
        arguments=['-file', urdf_path, '-urdf', '-entity', 'miku_dummy'],
    )

    # base_link → base_footprint 静态 TF
    # ROS 1: args="0 0 0 0 0 0 base_link base_footprint 40"
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='仿真时间'),
        gazebo_launch,
        spawn_model_node,
        static_tf_node,
    ])
