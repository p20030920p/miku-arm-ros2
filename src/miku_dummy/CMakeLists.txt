cmake_minimum_required(VERSION 3.8)
project(miku_dummy)

# ============================================================
# miku_dummy —— ROS 2 Jazzy 移植版（纯资源包：URDF + meshes + launch + rviz）
#
# ROS 1 → ROS 2 差异：
#   - 原来的 *.launch (XML) 改为 *.launch.py (Python)，
#     因为 ROS 2 的 launch 系统重写，XML 语法不兼容。
#   - rviz 配置由 urdf.rviz 移植为 urdf_ros2.rviz
#     （插件类名从 rviz/ 改为 rviz_default_plugins/）。
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# 模型资源：URDF / meshes / config
install(DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

# 启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# RViz 配置（ROS 2 版本）
install(FILES
  urdf.rviz
  urdf_ros2.rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
