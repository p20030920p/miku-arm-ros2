cmake_minimum_required(VERSION 3.8)
project(miku_moveit_demo)

# ============================================================
# miku_moveit_demo —— MoveIt 2 规划演示（连到仿真电机，无需硬件）
#
# trajectory_bridge 的作用：MoveIt 的 fake controller 只更新 /joint_states，
# 不会驱动我们的机械臂。这个节点把 MoveIt 下发的轨迹转成 ArmMsg（mode=2）
# 发给 sim_motor_board，于是 RViz 里按 Execute 时仿真机械臂真的会动。
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arm_control REQUIRED)

add_executable(trajectory_bridge src/trajectory_bridge.cpp)
ament_target_dependencies(trajectory_bridge
  rclcpp rclcpp_action sensor_msgs control_msgs trajectory_msgs arm_control)

install(TARGETS trajectory_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp arm_control)
ament_package()
