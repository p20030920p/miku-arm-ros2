cmake_minimum_required(VERSION 3.8)
project(arm_control)

# ============================================================
# arm_control —— ROS 2 Jazzy 移植版
#   arm_core 库：运动学 / 重力补偿 / 直线规划 / 夹爪状态机
#   可执行：arm_control_node, go_home, teach_one_node, claw_arm_test
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
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------- 自定义消息 ----------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmMsg.msg"
  DEPENDENCIES std_msgs
)

# ---------- 算法库 ----------
add_library(arm_core SHARED
  src/kinematics_solver.cpp    # 正逆运动学封装
  src/gravity_compensator.cpp  # 重力补偿封装
  src/linear_planner.cpp       # 直线插值规划封装
  src/claw_controller.cpp      # 夹爪状态机封装
)
target_include_directories(arm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_core
  rclcpp
  rcutils
  kdl_parser
  orocos_kdl
  urdf
)

# ---------- 可执行文件 ----------
add_executable(arm_control_node src/arm_control_node.cpp)
target_link_libraries(arm_control_node arm_core)
ament_target_dependencies(arm_control_node
  rclcpp ament_index_cpp std_msgs sensor_msgs kdl_parser orocos_kdl urdf
)

add_executable(go_home src/go_home.cpp)
ament_target_dependencies(go_home rclcpp std_msgs)

add_executable(teach_one_node src/teach_one_node.cpp)
target_link_libraries(teach_one_node arm_core)
ament_target_dependencies(teach_one_node
  rclcpp ament_index_cpp std_msgs sensor_msgs kdl_parser orocos_kdl urdf
)

add_executable(claw_arm_test src/claw_arm_test.cpp)
target_link_libraries(claw_arm_test arm_core)
ament_target_dependencies(claw_arm_test
  rclcpp ament_index_cpp std_msgs sensor_msgs kdl_parser orocos_kdl urdf
)

# 消息生成必须先于使用它的可执行文件
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(arm_control_node "${cpp_typesupport_target}")
target_link_libraries(teach_one_node   "${cpp_typesupport_target}")
target_link_libraries(claw_arm_test    "${cpp_typesupport_target}")
target_link_libraries(go_home          "${cpp_typesupport_target}")

# ---------- 安装 ----------
install(TARGETS
    arm_core
    arm_control_node
    go_home
    teach_one_node
    claw_arm_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# URDF / meshes / 配置（供节点运行时按 share 目录查找）
# 说明：miku_dummy.urdf 里的 mesh 引用原本是 package://miku_dummy/meshes/，
#       但该 URDF 被 arm_control 的节点加载，因此这里把 meshes 一并随包安装，
#       并把引用改写为 package://arm_control/meshes/，使本包自包含。
install(DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(arm_core)
ament_export_dependencies(rclcpp ament_index_cpp kdl_parser orocos_kdl urdf)
ament_package()
