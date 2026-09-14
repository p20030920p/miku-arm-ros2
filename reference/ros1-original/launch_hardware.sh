#!/bin/bash
# ==================================================
# 机械臂硬件系统一键启动（bash 版）
# 功能：
#   1. 自动启动 roscore（无需手动开）
#   2. 加载 yaml 参数
#   3. 独立窗口启动显示（RViz + robot_state_publisher）
#   4. 独立窗口启动硬件（HUD 实时刷新）
#   5. Ctrl+C 退出时自动清理所有窗口和进程
# ==================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS="$SCRIPT_DIR"
TITLE_PREFIX="ArmSystem_"

# ---------- ROS 环境 ----------
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

# ---------- 清理函数：退出时杀掉所有相关进程和窗口 ----------
cleanup() {
    if [ "$CLEANUP_DONE" = "1" ]; then
        return          # 已经清理过，直接跳过，避免二次归零
    fi
    CLEANUP_DONE=1
    
    echo -e "\033[31m\n退出，安全归零中...\033[0m"

    # ① 先杀掉所有可能抢 Arm_tx 的算法节点（逐个点名，绝不误杀 go_home）
    pkill -f "rosrun arm_control" 2>/dev/null
    sleep 1          # 等算法停止发布

    # ② 发布回零任务（hardware 还活着，负责把指令发到串口）
    echo -e "\033[33m[归零] 机械臂低速回零中，请稍候...\033[0m"
    timeout 25 rosrun arm_control go_home
    echo -e "\033[32m[归零] 完成。\033[0m"

    # ③ 归零完成，再关闭硬件和显示窗口/进程
    wmctrl -c "${TITLE_PREFIX}hardware" 2>/dev/null
    wmctrl -c "${TITLE_PREFIX}display"  2>/dev/null
    pkill -f "rosrun hardware hardware"  2>/dev/null
    pkill -f "roslaunch miku_dummy display" 2>/dev/null
    pkill -f "rviz"  2>/dev/null
    pkill -f "robot_state_publisher"  2>/dev/null

    # ④ 最后关 master
    pkill -f "rosmaster" 2>/dev/null
    pkill -f "roscore"   2>/dev/null

    sleep 1
    echo -e "\033[32m清理完成 ( ゜- ゜)つロ 干杯~ \033[0m"
    exit 0
}
trap cleanup EXIT INT TERM

# ---------- 0. 预清理（防止上次残留的 roscore/窗口） ----------
pkill -f "rosmaster" 2>/dev/null
pkill -f "roscore"   2>/dev/null
sleep 1

# ---------- 1. 自动启动 roscore（后台） ----------
echo -e "\033[34m[1/4] 启动 roscore...\033[0m"
setsid roscore &
sleep 3      # 等 master 起来

# ---------- 2. 加载参数 ----------
echo -e "\033[34m[2/4] 加载参数...\033[0m"
rosparam load "$WS/src/arm_control/config/arm_control.yaml"
rosparam load "$WS/src/arm_control/config/claw_controller.yaml"

# ---------- 3. 显示：robot_state_publisher + RViz（独立窗口） ----------
echo -e "\033[34m[3/4] 启动显示...\033[0m"
gnome-terminal --title="${TITLE_PREFIX}display" -- \
  bash -c "source $WS/devel/setup.bash; roslaunch miku_dummy display.launch; exec bash" &
sleep 3

# ---------- 4. 硬件（独立窗口，HUD） ----------
echo -e "\033[34m[4/4] 启动硬件...\033[0m"
gnome-terminal --title="${TITLE_PREFIX}hardware" -- \
  bash -c "source $WS/devel/setup.bash; rosrun hardware hardware; exec bash" &

# ---------- 5. 保持前台，等 Ctrl+C ----------
echo -e "\033[32m系统启动完成 ( ゜- ゜)つロ 干杯~ \033[0m"
echo -e "\033[32m----------------------------\033[0m"
echo -e "\033[32m按 Ctrl+C 退出并自动清理。\033[0m"
while true; do
    sleep 5
done