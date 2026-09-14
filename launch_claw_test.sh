#!/bin/bash
# ==================================================
# 机械臂夹爪测试一键启动（ROS 2 Jazzy 版）
# 基于 launch_hardware.sh 模板
#
# 功能：
#   1. 自动建立 ROS 2 环境（无需手动 source，也无需 roscore）
#   2. 通过 --params-file 加载 arm_control 参数
#   3. 独立窗口启动显示（RViz2 + robot_state_publisher）
#   4. 独立窗口启动硬件（HUD 实时刷新）
#   5. 独立窗口启动夹爪测试（claw_arm_test）
#   6. Ctrl+C 退出时：先安全归零，再自动清理所有窗口和进程
#
# 与 ROS 1 版本的差异见 arm_common.sh 顶部说明。
# ==================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ARM_WS="$SCRIPT_DIR"
source "$ARM_WS/arm_common.sh"

trap arm_cleanup EXIT INT TERM

# ---------- 0. 预清理（防止上次残留的窗口/节点） ----------
arm_precleanup

# ---------- 1. ROS 2 环境 ----------
echo -e "\033[34m[1/4] ROS 2 环境就绪（Jazzy，无需 roscore）\033[0m"

# ---------- 2. 显示：robot_state_publisher + RViz2（独立窗口） ----------
echo -e "\033[34m[2/4] 启动显示...\033[0m"
arm_start_display

# ---------- 3. 硬件（独立窗口，HUD） ----------
echo -e "\033[34m[3/4] 启动硬件...\033[0m"
arm_start_hardware

# ---------- 4. 夹爪测试（独立窗口） ----------
echo -e "\033[34m[4/4] 启动夹爪测试...\033[0m"
arm_start_control "claw_arm_test" "claw"

# ---------- 5. 保持前台，等 Ctrl+C ----------
echo -e "\033[32m系统启动完成 ( ゜- ゜)つロ 干杯~ \033[0m"
echo -e "\033[32m----------------------------\033[0m"
echo -e "\033[32m按 Ctrl+C 退出并自动清理。\033[0m"
while true; do
    sleep 5
done
