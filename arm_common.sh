#!/bin/bash
# ============================================================================
# arm_common.sh —— 三个一键启动脚本共用的环境/清理逻辑（ROS 2 Jazzy 版）
#
# 被 launch_hardware.sh / launch_arm_teach_one_node.sh / launch_claw_test.sh
# 通过 `source` 引入，不单独执行。
#
# ROS 1 → ROS 2 差异小结：
#   - source /opt/ros/noetic/setup.bash + devel/setup.bash
#       → source /opt/ros/jazzy/setup.bash + install/setup.bash
#   - 不再需要手动 `roscore`：ROS 2 的 DDS 没有 master 进程，
#     `ros2 run` 会自己起发现服务（rmw 守护进程按需启动）
#   - `rosparam load xxx.yaml` → 通过 `--params-file` 传给节点
#   - `roslaunch miku_dummy display.launch`
#       → `ros2 launch miku_dummy display.launch.py`
#   - `rosrun <pkg> <node>` → `ros2 run <pkg> <node>`
#   - 清理时不再 pkill rosmaster/roscore，改为 pkill ros2 launch/run 的进程
# ============================================================================

# ---------------------------------------------------------------------------
# 工作空间与环境
# ---------------------------------------------------------------------------
# ARM_WS 默认取本文件所在目录的父目录（脚本都放在工作空间根目录下）
ARM_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
: "${ARM_WS:=$ARM_COMMON_DIR}"
export ARM_WS

TITLE_PREFIX="ArmSystem_"

# ROS 2 环境
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "\033[31m找不到 /opt/ros/jazzy/setup.bash，请先安装 ROS 2 Jazzy。\033[0m"
    exit 1
fi

if [ ! -f "$ARM_WS/install/setup.bash" ]; then
    echo -e "\033[31m$ARM_WS/install/setup.bash 不存在，请先构建工作空间：\033[0m"
    echo -e "\033[33m  cd $ARM_WS && colcon build --symlink-install\033[0m"
    exit 1
fi
source "$ARM_WS/install/setup.bash"

# 参数文件（原 arm_control.yaml + claw_controller.yaml 合并为 ROS 2 参数文件）
ARM_PARAMS_FILE="$ARM_WS/install/arm_control/share/arm_control/config/arm_control_params.yaml"

# ---------------------------------------------------------------------------
# 进程名（用于清理）
# ---------------------------------------------------------------------------
# ROS 2 里 node 名与可执行名一致，直接按可执行名匹配即可
ARM_PKILL_PATTERNS=(
    "ros2 launch miku_dummy display.launch.py"
    "ros2 run hardware hardware"
    "ros2 run arm_control"
    "rviz2"
    "robot_state_publisher"
)

# ---------------------------------------------------------------------------
# 预清理：防止上次残留的窗口 / 节点
# ---------------------------------------------------------------------------
arm_precleanup() {
    for pat in "${ARM_PKILL_PATTERNS[@]}"; do
        pkill -f "$pat" 2>/dev/null
    done
    sleep 1
}

# ---------------------------------------------------------------------------
# 清理函数：退出时先安全归零，再关窗口/进程
# 用法：在脚本里 `trap arm_cleanup EXIT INT TERM`
# ---------------------------------------------------------------------------
arm_cleanup() {
    if [ "$CLEANUP_DONE" = "1" ]; then
        return          # 已经清理过，直接跳过，避免二次归零
    fi
    CLEANUP_DONE=1

    echo -e "\033[31m\n退出，安全归零中...\033[0m"

    # ① 先杀掉所有可能抢 Arm_tx 的算法节点（逐个点名，绝不误杀 go_home）
    pkill -f "ros2 run arm_control" 2>/dev/null
    sleep 1          # 等算法停止发布

    # ② 发布回零任务（hardware 还活着，负责把指令发到串口）
    echo -e "\033[33m[归零] 机械臂低速回零中，请稍候...\033[0m"
    timeout 25 ros2 run arm_control go_home
    echo -e "\033[32m[归零] 完成。\033[0m"

    # ③ 归零完成，再关闭硬件/显示/算法窗口和进程
    wmctrl -c "${TITLE_PREFIX}claw"     2>/dev/null
    wmctrl -c "${TITLE_PREFIX}control"  2>/dev/null
    wmctrl -c "${TITLE_PREFIX}hardware" 2>/dev/null
    wmctrl -c "${TITLE_PREFIX}display"  2>/dev/null

    pkill -f "ros2 launch miku_dummy display.launch.py" 2>/dev/null
    pkill -f "ros2 run hardware hardware"               2>/dev/null
    pkill -f "ros2 run arm_control"                     2>/dev/null
    pkill -f "rviz2"                                    2>/dev/null
    pkill -f "robot_state_publisher"                    2>/dev/null

    sleep 1
    echo -e "\033[32m清理完成 ( ゜- ゜)つロ 干杯~ \033[0m"
    exit 0
}

# ---------------------------------------------------------------------------
# 启动辅助
# ---------------------------------------------------------------------------
# 在独立终端窗口里跑一条命令（窗口标题带 ArmSystem_ 前缀，便于 wmctrl 关闭）
# 用法：arm_spawn_terminal <标题后缀> <工作目录> <命令>
arm_spawn_terminal() {
    local title="$1"; shift
    local workdir="$1"; shift
    local cmd="$*"

    # 环境变量需要在子 shell 里重新 source，这里把环境导出后继承
    gnome-terminal --title="${TITLE_PREFIX}${title}" --working-directory="$workdir" -- \
        bash -c "source /opt/ros/jazzy/setup.bash; source '$ARM_WS/install/setup.bash'; $cmd; exec bash" &
}

# 参数文件存在则拼出 --ros-args --params-file 片段，否则返回空串
arm_params_args() {
    if [ -f "$ARM_PARAMS_FILE" ]; then
        echo "--ros-args --params-file $ARM_PARAMS_FILE"
    fi
}

# 启动显示（robot_state_publisher + RViz2）
arm_start_display() {
    echo -e "\033[34m启动显示 (robot_state_publisher + RViz2)...\033[0m"
    arm_spawn_terminal "display" "$ARM_WS" "ros2 launch miku_dummy display.launch.py"
    sleep 3
}

# 启动硬件节点（串口通信 + HUD）
arm_start_hardware() {
    echo -e "\033[34m启动硬件 (hardware)...\033[0m"
    arm_spawn_terminal "hardware" "$ARM_WS" "ros2 run hardware hardware"
    sleep 3
}

# 启动一个 arm_control 算法节点（独立窗口）
# 用法：arm_start_control <节点名> <窗口标题>
arm_start_control() {
    local node="$1"
    local title="$2"
    echo -e "\033[34m启动算法节点 (${node})...\033[0m"
    arm_spawn_terminal "$title" "$ARM_WS" \
        "ros2 run arm_control ${node} $(arm_params_args)"
}
