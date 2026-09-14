# ==========================================================
# 项目与依赖
# ==========================================================
cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  std_msgs
  urdf
  message_generation
)

# ==========================================================
# 消息生成（ArmMsg.msg）
# ==========================================================
add_message_files(
  FILES
  ArmMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

# ==========================================================
# 包配置（声明给下游包的依赖）
# ==========================================================
catkin_package(
  CATKIN_DEPENDS kdl_parser roscpp std_msgs urdf message_runtime
)

# ==========================================================
# 头文件目录
# ==========================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ==========================================================
# 核心库 arm_core（三个封装模块，供所有节点共用）
# ==========================================================
add_library(arm_core
  src/kinematics_solver.cpp    # 正逆运动学封装
  src/gravity_compensator.cpp  # 重力补偿封装
  src/linear_planner.cpp       # 直线插值规划封装
  src/claw_controller.cpp      # 夹爪状态机封装
)
target_link_libraries(arm_core ${catkin_LIBRARIES})

# ==========================================================
# 可执行节点（每个节点三行：编译 + 依赖 + 链接）
# ==========================================================

## ---- 主控制节点：直线运动 + 逆解 + 重力补偿 ----
add_executable(arm_control_node src/arm_control_node.cpp)
add_dependencies(arm_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_control_node arm_core ${catkin_LIBRARIES})

## ---- 安全归零程序（Ctrl+C 时由 bash 调用）----
add_executable(go_home src/go_home.cpp)
add_dependencies(go_home ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_home ${catkin_LIBRARIES})

## ---- 示教节点：重力补偿悬停，可手拖 ----
add_executable(teach_one_node src/teach_one_node.cpp)
add_dependencies(teach_one_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teach_one_node arm_core ${catkin_LIBRARIES})

## ---- 夹爪 + 机械臂同步控制测试 ----
add_executable(claw_arm_test src/claw_arm_test.cpp)
add_dependencies(claw_arm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(claw_arm_test arm_core ${catkin_LIBRARIES})


# 注：src/camera_arm_test.cpp 未加入编译（如需启用，补一行 add_executable 即可）

# 注：teach_node / track_node / teach_path 已归档到 misc/，不再编译

# ==========================================================
# 安装（头文件）
# ==========================================================
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)