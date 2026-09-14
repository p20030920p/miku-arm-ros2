cmake_minimum_required(VERSION 3.8)
project(deep_camera)

# ============================================================================
#  deep_camera —— ROS 1 (catkin/Noetic) → ROS 2 (ament/colcon, Jazzy) 移植
# ============================================================================
#
#  【最重要：aruco 的 C++ 源码级依赖在两个版本下的差异与处理方式】
#
#  --- ROS 1 (catkin) 原来是怎么做的 ---
#  miku_dummy-master/src/deep_camera/CMakeLists.txt 中：
#      find_package(catkin REQUIRED COMPONENTS ... aruco)
#      include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
#                          ${CMAKE_SOURCE_DIR}/aruco/src)
#      target_link_libraries(camera_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
#  并且源码里直接写 `#include "ArucoDetector.cpp"`（注意是 .cpp，不是 .h）。
#
#  之所以能这么写：aruco 的这份“库”其实是**头文件式实现**——
#      ArucoDetector.cpp / ArucoMarker.cpp / ArucoMarkerInfo.cpp /
#      SquareFinder.cpp / CornerRefinement.cpp / math/*.cpp
#  每个文件都以 `#pragma once` 开头，类的成员函数全部写在类体内（隐式 inline）。
#  于是“谁 include 谁就得到完整实现”，既不需要链接任何库，也不会留下
#  需要外部解析的符号。
#
#  --- ROS 2 下的问题 ---
#  aruco 包由另一路任务并行移植，**当前它没有导出任何可链接的库**
#  （目录下没有 CMakeLists.txt / package.xml，install/ 下也没有 aruco）。
#  因此：
#    · 若这里写 find_package(aruco REQUIRED) → deep_camera 无法独立构建；
#    · 若 aruco 之后把 ArucoDetector.cpp 一并编进它自己的库，两边同时
#      提供同名符号也存在重复符号（duplicate symbol）风险。
#
#  --- 本文件采用的方案（任务约定的回退方案：源码头文件式编入本包）---
#   1. 用**缓存变量** ARUCO_SRC_DIR 定位 aruco 源码目录，默认
#      ${CMAKE_CURRENT_SOURCE_DIR}/../aruco/src（即同一工作空间里的 aruco 包），
#      **不硬编码任何绝对路径**；找不到 ArucoDetector.cpp 直接 FATAL_ERROR
#      并给出修复提示。
#   2. 把该目录加入需要 ArUco 的目标（camera_test / aruco_pos_test）的
#      PRIVATE include path。
#   3. 源码里的 `#include "ArucoDetector.cpp"` 改为
#      `#include "deep_camera/aruco_detector_compat.hpp"`：该头文件先补
#      OpenCV 4 兼容宏（CV_RGB2GRAY / CV_THRESH_BINARY / CV_THRESH_OTSU
#      在 OpenCV 4 中已被删除），再 include aruco 的 ArucoDetector.cpp。
#      兼容宏与原常量数值一一对应，检测逻辑零改动。
#   4. **不链接** aruco 库 —— 实现已经通过 include 进入本目标的编译单元，
#      既不依赖 aruco 是否构建成功，也彻底杜绝重复符号问题。
#
#  因为使用的是真实的 `#include "…/ArucoDetector.cpp"`，编译器生成的依赖
#  文件会自动跟踪这些 .cpp，改动 aruco 源码即可触发 deep_camera 重编，
#  无需额外 add_dependencies。
#
#  --- 首选方案的开关（默认关闭）---
#  如果将来 aruco 包导出了可链接的库（ament_export_include_directories +
#  ament_export_libraries / ament_export_targets），可以打开
#      -DDEEP_CAMERA_LINK_ARUCO_PACKAGE=ON
#  改为 find_package(aruco REQUIRED) + 链接该库，即任务里的“首选方案”。
#  默认 OFF，保证 deep_camera 可以独立构建。
# ============================================================================

# ---------------------------------------------------------------------------
# 依赖查找
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# 查找 OpenCV（系统依赖，libopencv-dev 4.6）
find_package(OpenCV REQUIRED)

# ---------------------------------------------------------------------------
# aruco 依赖的两种接入方式（详见文件顶部说明）
# ---------------------------------------------------------------------------
option(DEEP_CAMERA_LINK_ARUCO_PACKAGE
  "Link the aruco package's exported library instead of compiling aruco sources into deep_camera"
  OFF)

set(ARUCO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../aruco/src"
    CACHE PATH "aruco 包的 src/ 目录（必须包含 ArucoDetector.cpp）")
# 规范化路径（默认值里的 .. 会被展开），便于在报错信息里直接看出实际位置
get_filename_component(ARUCO_SRC_DIR "${ARUCO_SRC_DIR}" ABSOLUTE)

set(ARUCO_LINK_LIBRARIES "")
set(ARUCO_EXTRA_INCLUDE_DIRS "")

if(DEEP_CAMERA_LINK_ARUCO_PACKAGE)
  # 首选方案：链接 aruco 包导出的检测库。
  # 注意：具体的导入目标名取决于 aruco 包自己的导出方式——
  #   · ament_export_libraries() → 变量 aruco_LIBRARIES
  #   · ament_export_targets()   → 形如 aruco::<target> 的导入目标
  # 这里同时兼容两者，若 aruco 用了别的名字，请在这里补一行。
  find_package(aruco REQUIRED)
  set(ARUCO_LINK_LIBRARIES ${aruco_LIBRARIES})
  if(TARGET aruco::aruco)
    list(APPEND ARUCO_LINK_LIBRARIES aruco::aruco)
  endif()
  message(STATUS "deep_camera: 链接 aruco 包导出的库 (${ARUCO_LINK_LIBRARIES})")
else()
  # 回退方案（默认）：把 aruco 库源码直接编入 deep_camera 的目标
  if(NOT EXISTS "${ARUCO_SRC_DIR}/ArucoDetector.cpp")
    message(FATAL_ERROR
      "找不到 aruco 的库源码：${ARUCO_SRC_DIR}/ArucoDetector.cpp\n"
      "deep_camera 需要 aruco 包提供的 C++ 检测库（头文件式实现）。\n"
      "  修复方式 1：确认 aruco 包与 deep_camera 位于同一工作空间，即\n"
      "              <ws>/src/aruco/src/ArucoDetector.cpp 存在；\n"
      "  修复方式 2：用 -DARUCO_SRC_DIR=/绝对路径/aruco/src 显式指定；\n"
      "  修复方式 3：若 aruco 包已导出可链接的库，改用\n"
      "              -DDEEP_CAMERA_LINK_ARUCO_PACKAGE=ON")
  endif()
  set(ARUCO_EXTRA_INCLUDE_DIRS "${ARUCO_SRC_DIR}")
  message(STATUS "deep_camera: 直接编入 aruco 源码，ARUCO_SRC_DIR=${ARUCO_SRC_DIR}")
endif()

# ---------------------------------------------------------------------------
# 编译选项（C++17）
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 三个可执行文件（话题名、消息类型、算法逻辑与 ROS 1 完全一致）
# ---------------------------------------------------------------------------
add_executable(visual_grasp_node src/visual_grasp_node.cpp)
add_executable(camera_test       src/camera_test.cpp)
add_executable(aruco_pos_test    src/aruco_pos_test.cpp)

# 公共依赖：rclcpp / std_msgs / sensor_msgs / cv_bridge / message_filters / OpenCV
set(DEEP_CAMERA_COMMON_LIBRARIES
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  message_filters::message_filters
  ${OpenCV_LIBS}
)

foreach(_target visual_grasp_node camera_test aruco_pos_test)
  target_include_directories(${_target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${_target} ${DEEP_CAMERA_COMMON_LIBRARIES})
endforeach()

# 只有用到 ArUco 的两个目标需要 aruco 的源码目录 / 导入库
foreach(_target camera_test aruco_pos_test)
  if(ARUCO_EXTRA_INCLUDE_DIRS)
    target_include_directories(${_target} PRIVATE ${ARUCO_EXTRA_INCLUDE_DIRS})
  endif()
  if(ARUCO_LINK_LIBRARIES)
    target_link_libraries(${_target} ${ARUCO_LINK_LIBRARIES})
  endif()
endforeach()

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(TARGETS visual_grasp_node camera_test aruco_pos_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
