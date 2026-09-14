cmake_minimum_required(VERSION 3.8)
project(miku_dummy_moveit_config)

# ============================================================
# miku_dummy_moveit_config —— 由 ROS 1 配置移植到 MoveIt 2 (ROS 2 Jazzy)
#
# ROS 1 → ROS 2 差异：
#   - MoveIt 1 的 *.launch / *.launch.xml（planning_context、move_group、
#     trajectory_execution、warehouse、setup_assistant ... 共 20 多个）
#     在 MoveIt 2 中已由 moveit_configs_utils 统一生成，不再需要这些文件，
#     因此没有逐个移植，而是用一个 move_group.launch.py 覆盖同样的功能。
#   - ros_controllers.yaml（ros_control）在 ROS 2 中由 ros2_control 取代，
#     仿真用 gz_ros2_control 的控制器配置，见 miku_dummy 包的 sim 部分。
#   - SRDF / joint_limits / kinematics / ompl / chomp / stomp / cartesian_limits
#     这些是跨版本通用的，原样保留。
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
