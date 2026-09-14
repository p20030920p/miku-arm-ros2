cmake_minimum_required(VERSION 3.8)
project(aruco)

# ROS 2 差异：ROS 1 的 catkin 构建强制 -std=c++11；ROS 2 Jazzy 统一使用 C++17（C++11 的超集）
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Packages
# ROS 2 差异：catkin -> ament_cmake，message_generation -> rosidl_default_generators
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#Messages
# ROS 2 差异：add_message_files/generate_messages -> rosidl_generate_interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/Marker.msg"
	DEPENDENCIES std_msgs
)

#Aruco ROS node
# ROS 2 差异：与原 CMakeLists 结构一致——只编译 src/ros/ArucoNode.cpp。
# aruco 库的实现文件（ArucoMarker.cpp / ArucoMarkerInfo.cpp / ArucoDetector.cpp 以及它们
# 再包含的 SquareFinder.cpp / CornerRefinement.cpp / math/*.cpp）全部由 ArucoNode.cpp
# 顶部的 #include "../ArucoMarker.cpp" 等直接包含进同一个翻译单元，
# 因此不需要单独编译成库；原构建也是靠 include_directories(src) 提供包含路径。
#
# ROS 2 差异：rosidl_generate_interfaces(${PROJECT_NAME} ...) 已经占用了名为 "aruco" 的
# CMake 目标（消息生成用的 custom target），因此可执行目标改名为 aruco_node，再用
# OUTPUT_NAME 把产物命名回 aruco。安装后的可执行文件仍是 lib/aruco/aruco，
# `ros2 run aruco aruco`、launch 里的 executable='aruco' 与 ROS 1 的 type="aruco" 保持一致。
add_executable(aruco_node src/ros/ArucoNode.cpp)
set_target_properties(aruco_node PROPERTIES OUTPUT_NAME aruco)

target_include_directories(aruco_node PRIVATE
	src
	src/math
	${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(aruco_node
	rclcpp
	std_msgs
	sensor_msgs
	geometry_msgs
	nav_msgs
	cv_bridge
)

# ROS 2 差异：ament 用 typesupport target 取代 catkin 的
# add_dependencies(aruco aruco_generate_messages_cpp ...)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aruco_node
	${cpp_typesupport_target}
	${OpenCV_LIBS}
)

#Install
install(TARGETS aruco_node
	ARCHIVE DESTINATION lib/${PROJECT_NAME}
	LIBRARY DESTINATION lib/${PROJECT_NAME}
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
