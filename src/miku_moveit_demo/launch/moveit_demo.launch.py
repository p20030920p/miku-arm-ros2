#!/usr/bin/env python3
# ============================================================================
# moveit_demo.launch.py —— MoveIt 2 规划演示（连仿真电机，无需硬件）
#
# 启动内容：
#   sim_motor_board      仿真电机（发布 /joint_states、回读 /Arm_rx）
#   move_group           MoveIt 2，使用 fake controller
#   trajectory_bridge    把 MoveIt 的轨迹转成 ArmMsg 发给仿真电机
#   robot_state_publisher / RViz2（MotionPlanning 面板）
#
# 于是可以在 RViz 里拖动交互标记调位姿 → Plan → Execute，
# 仿真机械臂会真的运动。
#
# 用法：
#   ros2 launch miku_moveit_demo moveit_demo.launch.py              # 带 RViz
#   ros2 launch miku_moveit_demo moveit_demo.launch.py rviz:=false
# ============================================================================

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def load_yaml(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def to_double(node):
    """把 YAML 里的整数提升为 double。

    MoveIt 2 声明这些参数为 double，而 `max_velocity: 2` 会被 PyYAML 解析成 int，
    直接传进去会抛 InvalidParameterTypeException。递归处理整个配置树。
    """
    if isinstance(node, dict):
        return {k: to_double(v) for k, v in node.items()}
    if isinstance(node, list):
        return [to_double(v) for v in node]
    if isinstance(node, bool):
        return node
    if isinstance(node, int):
        return float(node)
    return node


def generate_launch_description():
    demo_share = get_package_share_directory("miku_moveit_demo")
    cfg_share = get_package_share_directory("miku_dummy_moveit_config")
    arm_share = get_package_share_directory("arm_control")

    cfg = os.path.join(cfg_share, "config")
    urdf = open(os.path.join(arm_share, "urdf", "miku_arm.urdf"), encoding="utf-8").read()
    srdf = open(os.path.join(cfg, "miku_arm.srdf"), encoding="utf-8").read()

    user_params = load_yaml(os.path.join(demo_share, "config", "moveit_params.yaml"))
    mg = user_params["move_group"]["ros__parameters"]

    # MoveIt 1 的 ompl_planning.yaml 缺少 MoveIt 2 需要的插件与适配器声明
    ompl = to_double(load_yaml(os.path.join(cfg, "ompl_planning.yaml")))
    ompl["planning_plugins"] = ["ompl_interface/OMPLPlanner"]
    ompl["request_adapters"] = [
        "default_planning_request_adapters/ResolveConstraintFrames",
        "default_planning_request_adapters/ValidateWorkspaceBounds",
        "default_planning_request_adapters/CheckStartStateBounds",
        "default_planning_request_adapters/CheckStartStateCollision",
    ]
    ompl["response_adapters"] = [
        "default_planning_response_adapters/AddTimeOptimalParameterization",
        "default_planning_response_adapters/ValidateSolution",
        "default_planning_response_adapters/DisplayMotionPath",
    ]

    move_group_params = {
        "robot_description": urdf,
        "robot_description_semantic": srdf,
        "robot_description_kinematics": load_yaml(os.path.join(cfg, "kinematics.yaml")),
        "robot_description_planning": to_double(
            load_yaml(os.path.join(cfg, "joint_limits.yaml"))),
        "ompl": ompl,
    }
    move_group_params.update(mg)
    # MoveIt 不直接驱动电机：轨迹交给 trajectory_bridge，由它转成
    # ArmMsg(mode=2) 下发到 /Arm_tx。因此这里用 simple controller manager，
    # 把 MoveIt 的 action 客户端指向 bridge 的 action server。
    # 句柄拼出的 action 名是 "<控制器名>/<action_ns>"，所以控制器就叫
    # miku_arm_controller，与 trajectory_bridge 的默认名一致。
    move_group_params["moveit_controller_manager"] = \
        "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    move_group_params["trajectory_execution"] = {
        "allowed_execution_duration_scaling": 1.5,
        "allowed_goal_duration_margin": 0.5,
        "allowed_start_tolerance": 0.05,
        "execution_duration_monitoring": False,
    }
    move_group_params["moveit_simple_controller_manager"] = {
        "controller_names": ["miku_arm_controller"],
        "miku_arm_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": ["joint_1", "joint_2", "joint_3",
                       "joint_4", "joint_5", "joint_6"],
        },
    }

    # 录制用的配置只保留 MotionPlanning 一个面板，窗口更窄、3D 视口更宽。
    default_rviz = os.path.join(demo_share, "rviz", "moveit_demo.rviz")
    record_rviz = os.path.join(demo_share, "rviz", "moveit_demo_record.rviz")
    rviz_cfg = PythonExpression([
        "'", record_rviz, "' if '", LaunchConfiguration("record"), "' == 'true' else '",
        default_rviz, "'"])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="是否启动 RViz2"),
        DeclareLaunchArgument("record", default_value="false",
                              description="用录制布局（单面板、窗口 1400x900）"),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="轨迹速度缩放（演示可调慢）"),

        # 仿真电机：与 miku_sim/sim.launch.py 用的是同一个节点，
        # 这样 /joint_states 由它发布，MoveIt 与 RViz 都读这一路。
        Node(package="miku_sim", executable="sim_motor_board", name="sim_motor_board",
             output="screen",
             parameters=[{"gravity_load": True, "grip_object": True, "rate": 100.0}]),

        # robot_state_publisher 提供 /robot_description 与 TF。
        # 缺了它 RViz 里的机械臂不会出现（MotionPlanning 显示要靠 TF 摆放连杆）。
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf}]),

        Node(package="moveit_ros_move_group", executable="move_group", name="move_group",
             output="screen", parameters=[move_group_params]),

        Node(package="miku_moveit_demo", executable="trajectory_bridge",
             name="trajectory_bridge", output="screen",
             parameters=[{"speed_scale": LaunchConfiguration("speed_scale")}]),

        # RViz 必须拿到 robot_description 与 robot_description_semantic，
        # 否则 MotionPlanning 面板的 Planning Group 会是空的、无法规划。
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_cfg],
             parameters=[{
                 "robot_description": urdf,
                 "robot_description_semantic": srdf,
                 "robot_description_kinematics": move_group_params["robot_description_kinematics"],
             }],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
