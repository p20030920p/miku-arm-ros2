cmake_minimum_required(VERSION 3.8)
project(miku_sim)

# ============================================================
# miku_sim —— 无实机仿真与测试包（新增，原 ROS 1 工程没有）
#
# 目的：实机不可用时，仍能验证整条链路
#   virtual_motor_board : 串口协议级仿真（配合 socat 虚拟串口，测 hardware 节点）
#   sim_motor_board     : ArmMsg 闭环 + Gazebo 可视化（测 arm_control 上层逻辑）
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arm_control REQUIRED)

add_executable(sim_motor_board src/sim_motor_board.cpp)
ament_target_dependencies(sim_motor_board
  rclcpp sensor_msgs std_msgs trajectory_msgs arm_control)

install(TARGETS sim_motor_board
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 串口协议级仿真器（纯 Python，无需编译）
install(PROGRAMS
  scripts/virtual_motor_board.py
  scripts/run_serial_hil_test.sh
  scripts/run_sim_e2e_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp arm_control)
ament_package()
