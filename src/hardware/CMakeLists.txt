cmake_minimum_required(VERSION 3.8)
project(hardware)

# ============================================================
# hardware —— ROS 2 Jazzy 移植版
#
# 说明：ROS 1 版本依赖 catkin 的 serial / joy / control_msgs /
#       moveit_core / moveit_ros_planning_interface。
#       实际检查源码后确认：
#         - 没有任何文件真正调用 MoveIt API（只在注释和日志字符串里出现
#           "moveit"），因此不再依赖 MoveIt；
#         - joy 只用到 sensor_msgs/Joy 消息，属于 sensor_msgs 包；
#         - serial 在 ROS 2 Jazzy 没有对应 apt 包，本包自带
#           include/serial/serial.h（POSIX termios 实现，API 兼容）。
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arm_control REQUIRED)

set(HARDWARE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---------- 可执行文件 ----------
set(HARDWARE_NODES
  hardware          # 串口通信主节点（Arm_tx ↔ 下位机，Arm_rx 回读）
  pub_test
  stable_test
  joy_test
  read_angle
  trajectory_track
  my_moveit_test
  moveit2hardware
  claw_test
)

foreach(node ${HARDWARE_NODES})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE ${HARDWARE_INCLUDE_DIRS})
  ament_target_dependencies(${node}
    rclcpp
    ament_index_cpp
    std_msgs
    sensor_msgs
    trajectory_msgs
    arm_control
  )
  install(TARGETS ${node}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# 串口实现是 header-only，随包安装以便其它包复用
install(DIRECTORY include/
  DESTINATION include
)

# 示教轨迹目录（原 .gitignore 忽略内容，这里只保证目录存在）
install(DIRECTORY teach_path
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp ament_index_cpp arm_control)
ament_package()
