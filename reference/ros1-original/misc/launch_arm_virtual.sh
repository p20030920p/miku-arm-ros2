#!/bin/bash

# ==================================================
# 机械臂系统一键启动脚本（完全关闭残留进程）- 虚拟仿真版
# 版本：5.0
# 作者：ROS开发者
# 路径：/workspace/ws_moveit/launch_arm_virtual.sh
# ==================================================

# 设置工作空间路径（脚本所在目录）
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_PATH="$SCRIPT_DIR"

# 设置唯一窗口标题前缀
TITLE_PREFIX="ArmSystem_"

# 检查工作空间目录是否存在
if [ ! -d "$WORKSPACE_PATH" ]; then
  echo -e "\033[31m错误：工作空间目录不存在 - $WORKSPACE_PATH\033[0m"
  exit 1
fi

# 检查setup.bash文件是否存在
SETUP_FILE="$WORKSPACE_PATH/devel/setup.bash"
if [ ! -f "$SETUP_FILE" ]; then
  echo -e "\033[31m错误：setup.bash文件未找到 - $SETUP_FILE\033[0m"
  echo "请确保已经编译工作空间 (catkin_make)"
  exit 1
fi

# ==================================================
# 彻底关闭所有残留进程
# ==================================================
echo -e "\033[33m正在清理所有残留进程...\033[0m"

# 1. 关闭所有相关终端窗口
TERMINAL_WINDOWS=$(wmctrl -l | grep "$TITLE_PREFIX" | awk '{print $1}')
if [ -n "$TERMINAL_WINDOWS" ]; then
  echo "关闭残留终端窗口:"
  wmctrl -l | grep "$TITLE_PREFIX"
  for window in $TERMINAL_WINDOWS; do
    wmctrl -ic $window
  done
fi

# 2. 关闭所有Gazebo和RVIZ窗口
GAZEBO_WINDOWS=$(wmctrl -l | grep -iE "gazebo|rviz" | awk '{print $1}')
if [ -n "$GAZEBO_WINDOWS" ]; then
  echo "关闭Gazebo/RViz窗口:"
  wmctrl -l | grep -iE "gazebo|rviz"
  for window in $GAZEBO_WINDOWS; do
    wmctrl -ic $window
  done
fi

# 3. 关闭所有MoveIt相关进程
MOVEIT_PROCESSES=$(pgrep -f "roslaunch.*(firstarm_moveit_config|my_moveit)")
if [ -n "$MOVEIT_PROCESSES" ]; then
  echo "关闭MoveIt进程: $MOVEIT_PROCESSES"
  kill -9 $MOVEIT_PROCESSES
fi

# 4. 关闭所有Gazebo相关进程
GAZEBO_PROCESSES=$(pgrep -f "gazebo")
if [ -n "$GAZEBO_PROCESSES" ]; then
  echo "关闭Gazebo进程: $GAZEBO_PROCESSES"
  kill -9 $GAZEBO_PROCESSES
fi

# 5. 关闭残留ROS节点
ROSCORE_PID=$(pgrep roscore)
if [ -n "$ROSCORE_PID" ]; then
  echo "关闭roscore进程: $ROSCORE_PID"
  kill -9 $ROSCORE_PID
fi

ROSCORE_MASTER_PID=$(pgrep rosmaster)
if [ -n "$ROSCORE_MASTER_PID" ]; then
  echo "关闭rosmaster进程: $ROSCORE_MASTER_PID"
  kill -9 $ROSCORE_MASTER_PID
fi

# 6. 等待所有进程关闭
sleep 5
echo -e "\033[32m所有残留进程已关闭\033[0m"

# ==================================================
# 启动独立的终端窗口
# ==================================================
echo -e "\033[32m================================================"
echo " 启动机械臂虚拟仿真系统 "
echo " 工作空间: $WORKSPACE_PATH"
echo "================================================"

# 1. 启动Gazebo仿真环境
gnome-terminal \
  --window --title="${TITLE_PREFIX}Gazebo_Simulation" \
  --geometry=80x24+50+100 \
  --working-directory="$WORKSPACE_PATH" \
  -- bash -c \
    "echo -e '\033[34m=== 启动Gazebo仿真环境 ===\033[0m'; \
     source devel/setup.bash; \
     roslaunch miku_dummy_moveit_config gazebo.launch; \
     echo -e '\033[31m=== Gazebo仿真已停止 ===\033[0m'; \
     exec bash"
     
echo -e "\033[32m启动Gazebo仿真环境... "
# 等待8秒确保Gazebo完全启动
sleep 8

# 2. 启动主节点
gnome-terminal \
  --window --title="${TITLE_PREFIX}Main_Node" \
  --geometry=80x24+600+100 \
  --working-directory="$WORKSPACE_PATH" \
  -- bash -c \
    "echo -e '\033[34m=== 启动主控制节点 ===\033[0m'; \
     source devel/setup.bash; \
     rosrun arm_control arm_control_node; \
     echo -e '\033[31m=== 主控制节点已停止 ===\033[0m'; \
     exec bash"
     
echo -e "\033[32m启动主控制节点... "
echo "================================================"

echo -e "\033[32m系统启动完成"
echo -e "( ゜- ゜)つロ 干杯~ "
