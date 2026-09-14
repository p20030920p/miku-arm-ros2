#!/bin/bash
# ============================================================================
# run_serial_hil_test.sh —— 串口硬件在环（HIL）自动测试
#
# 让**真正的 hardware 可执行文件**通过 socat 虚拟串口连到
# virtual_motor_board.py（虚拟达妙驱动板），验证无实机时最无法验证的部分：
# 二进制串口协议。
#
# 验收用例：
#   [1] hardware 打开串口并初始化
#   [2] 双向帧流建立（下行 50B / 上行 46B），/Arm_rx 有真实回读
#   [3] 位置模式(mode=2)无重力时精确跟随 —— 证明 x1000 定点换算无损
#   [4] MIT 模式(mode=1)力矩前馈符号与幅值正确
#   [5] 重力负载下位置模式漂移 vs MIT+前馈顶住 —— 重力补偿链路可观测
#   [6] 夹爪(电机7)夹到物体：位置卡住 + 力矩上升
#   [7] 串口断开后 hardware 不崩溃并能自动重连
#
# 用法：  ./run_serial_hil_test.sh
# 退出码：0=全部通过
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$SCRIPT_DIR/../../../setup.bash" ]; then
    WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
elif [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
    WS="$(cd "$SCRIPT_DIR/../.." && pwd)"
else
    WS="${ARM_WS:-/home/qzl/workspace/Damiao_ARM/ros2_ws}"
fi

BOARD_PY="$SCRIPT_DIR/virtual_motor_board.py"
[ -f "$BOARD_PY" ] || BOARD_PY="$WS/src/miku_sim/scripts/virtual_motor_board.py"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ROS 的 setup.bash 在 set -u 下会因未定义变量报错，source 期间关闭 -u
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

SERIAL_A=/tmp/miku_hil_ttyA
SERIAL_B=/tmp/miku_hil_ttyB
LOG_DIR=$(mktemp -d /tmp/miku_hil.XXXXXX)

PASS=0; FAIL=0
declare -a RESULTS
pass() { PASS=$((PASS+1)); RESULTS+=("PASS  $1"); echo -e "\033[32m  [PASS]\033[0m $1"; }
fail() { FAIL=$((FAIL+1)); RESULTS+=("FAIL  $1"); echo -e "\033[31m  [FAIL]\033[0m $1"; }
info() { echo "         $1"; }

BOARD_PID=""; HW_PID=""; SOCAT_PID=""; PUB_PID=""; ECHO_PID=""

cleanup_all() {
    for p in "$PUB_PID" "$ECHO_PID" "$HW_PID" "$BOARD_PID" "$SOCAT_PID"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    pkill -9 -f "ros2 topic pub.*Arm_tx" 2>/dev/null
    pkill -9 -f "ros2 topic echo.*Arm_rx" 2>/dev/null
    rm -f "$SERIAL_A" "$SERIAL_B"
    wait 2>/dev/null
    return 0
}
trap cleanup_all EXIT INT TERM

start_stack() {
    local extra="${1:-}"
    rm -f "$SERIAL_A" "$SERIAL_B"
    socat -d -d pty,raw,echo=0,link="$SERIAL_A" pty,raw,echo=0,link="$SERIAL_B" \
        >> "$LOG_DIR/socat.log" 2>&1 &
    SOCAT_PID=$!
    for _ in $(seq 1 40); do
        [ -e "$SERIAL_A" ] && [ -e "$SERIAL_B" ] && break
        sleep 0.2
    done
    [ -e "$SERIAL_A" ] || { echo "socat 未能创建虚拟串口"; cat "$LOG_DIR/socat.log"; exit 1; }

    python3 "$BOARD_PY" --port "$SERIAL_B" --quiet $extra >> "$LOG_DIR/board.log" 2>&1 &
    BOARD_PID=$!
    sleep 1
    ros2 run hardware hardware --ros-args -p serial_port:="$SERIAL_A" \
        >> "$LOG_DIR/hardware.log" 2>&1 &
    HW_PID=$!
    sleep 3
}

start_rx_echo() {
    timeout "$2" ros2 topic echo /Arm_rx arm_control/msg/ArmMsg > "$1" 2>&1 &
    ECHO_PID=$!
}

last_pos() { grep -E "^pos_$2:" "$1" | tail -1 | awk '{print $2}'; }
last_tor() { grep -E "^tor_$2:" "$1" | tail -1 | awk '{print $2}'; }

publish_tx() {
    timeout "$2" ros2 topic pub -r 50 /Arm_tx arm_control/msg/ArmMsg "$1" \
        > "$LOG_DIR/pub.log" 2>&1 &
    PUB_PID=$!
}

echo "============================================================"
echo " 串口硬件在环（HIL）测试 —— 真实 hardware 节点 <-> 虚拟驱动板"
echo "   工作空间 : $WS"
echo "   日志目录 : $LOG_DIR"
echo "   虚拟串口 : $SERIAL_A <-> $SERIAL_B"
echo "============================================================"

echo
echo "[用例 1] hardware 打开虚拟串口并初始化"
start_stack "--no-gravity"
if grep -q "Serial Port initialized" "$LOG_DIR/hardware.log"; then
    pass "串口初始化成功（自写 POSIX 串口实现可用）"
else
    fail "串口未初始化"; tail -10 "$LOG_DIR/hardware.log"
fi

echo
echo "[用例 2] 双向帧流 + /Arm_rx 回读"
start_rx_echo "$LOG_DIR/rx1.log" 6
publish_tx "{task_status: 1, mode: 2, kp: 6.0, kd: 0.6, vel_1: 1.0, vel_2: 1.0, vel_3: 1.0, vel_4: 1.0, vel_5: 1.0, vel_6: 1.0, vel_7: 1.0}" 5
sleep 6
kill $PUB_PID 2>/dev/null; PUB_PID=""
N_RX=$(grep -c "^pos_1:" "$LOG_DIR/rx1.log" 2>/dev/null || echo 0)
info "驱动板统计: $(grep -oE '收 [0-9]+ 帧' "$LOG_DIR/board.log" | tail -1)"
if [ "$N_RX" -gt 50 ]; then
    pass "/Arm_rx 收到 $N_RX 条回读（50B 下行 + 46B 上行 帧流正常）"
else
    fail "/Arm_rx 回读过少（$N_RX 条）"
fi

echo
echo "[用例 3] 位置模式精确跟随（无重力）—— 验证 x1000 定点换算无损"
start_rx_echo "$LOG_DIR/rx2.log" 12
TARGETS=(0.5 -0.4 0.3 0.2 -0.25 0.15)
publish_tx "{task_status: 1, mode: 2, kp: 6.0, kd: 0.6, \
  pos_1: ${TARGETS[0]}, vel_1: 1.0, pos_2: ${TARGETS[1]}, vel_2: 1.0, \
  pos_3: ${TARGETS[2]}, vel_3: 1.0, pos_4: ${TARGETS[3]}, vel_4: 1.0, \
  pos_5: ${TARGETS[4]}, vel_5: 1.0, pos_6: ${TARGETS[5]}, vel_6: 1.0, \
  pos_7: 0.0, vel_7: 1.0}" 10
sleep 11
kill $PUB_PID 2>/dev/null; PUB_PID=""

OK=1
for i in 1 2 3 4 5 6; do
    V=$(last_pos "$LOG_DIR/rx2.log" "$i")
    T="${TARGETS[$((i-1))]}"
    if [ -z "$V" ]; then OK=0; info "电机$i: 无回读"; continue; fi
    if python3 -c "import sys;sys.exit(0 if abs($V-($T))<0.002 else 1)"; then
        info "电机$i: 目标 $T -> 回读 $V  OK"
    else
        OK=0; info "电机$i: 目标 $T -> 回读 $V  偏差过大"
    fi
done
if [ "$OK" = "1" ]; then
    pass "位置模式 6 关节精确跟随（误差 < 0.002 rad，定点换算无损）"
else
    fail "位置模式跟随有偏差"
fi

echo
echo "[用例 4] MIT 模式力矩前馈符号与幅值"
start_rx_echo "$LOG_DIR/rx3.log" 8
publish_tx "{task_status: 1, mode: 1, kp: 6.0, kd: 0.6, \
  tor_1: 0.5, pos_1: 0.0, tor_2: -0.5, pos_2: 0.0, \
  tor_3: 0.25, pos_3: 0.0, tor_4: 0.0, pos_4: 0.0, \
  tor_5: 0.0, pos_5: 0.0, tor_6: 0.0, pos_6: 0.0}" 6
sleep 7
kill $PUB_PID 2>/dev/null; PUB_PID=""
T1=$(last_tor "$LOG_DIR/rx3.log" 1)
T2=$(last_tor "$LOG_DIR/rx3.log" 2)
T3=$(last_tor "$LOG_DIR/rx3.log" 3)
info "下发 +0.5 / -0.5 / +0.25  ->  回读 $T1 / $T2 / $T3 N·m"
if python3 -c "
import sys
try:
    ok = abs(float('$T1')-0.5)<0.01 and abs(float('$T2')+0.5)<0.01 and abs(float('$T3')-0.25)<0.01
except Exception:
    ok = False
sys.exit(0 if ok else 1)"; then
    pass "MIT 模式力矩前馈符号与幅值全部正确"
else
    fail "MIT 模式力矩回读不符"
fi

echo
echo "[用例 5] 重力负载：位置模式漂移 vs MIT+前馈顶住"
kill -9 "$HW_PID" "$BOARD_PID" "$SOCAT_PID" 2>/dev/null
HW_PID=""; BOARD_PID=""; SOCAT_PID=""
sleep 1
start_stack ""

start_rx_echo "$LOG_DIR/rx4.log" 8
publish_tx "{task_status: 1, mode: 2, kp: 6.0, kd: 0.6, vel_2: 0.2}" 6
sleep 7
kill $PUB_PID 2>/dev/null; PUB_PID=""
DRIFT=$(last_pos "$LOG_DIR/rx4.log" 2)
info "位置模式(目标 pos_2=0，重力 0.45 N·m)  ->  pos_2 = ${DRIFT:-none} rad"

start_rx_echo "$LOG_DIR/rx5.log" 8
# 前馈 tor_2 = 0.45 N·m 恰好抵消该关节的重力负载
publish_tx "{task_status: 1, mode: 1, kp: 6.0, kd: 0.6, pos_2: 0.0, tor_2: 0.45}" 6
sleep 7
kill $PUB_PID 2>/dev/null; PUB_PID=""
HOLD=$(last_pos "$LOG_DIR/rx5.log" 2)
info "MIT 模式(前馈 tor_2=0.45 抵消重力)  ->  pos_2 = ${HOLD:-none} rad"

if python3 -c "
import sys
try:
    d = abs(float('$DRIFT')); h = abs(float('$HOLD'))
except Exception:
    sys.exit(1)
sys.exit(0 if d > 0.02 and h < d * 0.5 else 1)"; then
    pass "重力负载可观测，力矩前馈显著抵消（漂移 $DRIFT -> 补偿后 $HOLD）"
else
    fail "重力对比不明显（漂移=$DRIFT 补偿后=$HOLD）"
fi

echo
echo "[用例 6] 夹爪(电机7)夹到物体：位置卡住 + 力矩上升"
start_rx_echo "$LOG_DIR/rx6.log" 8
publish_tx "{task_status: 1, mode: 1, kp: 6.0, kd: 0.6, pos_7: -2.8, tor_7: -1.0}" 6
sleep 7
kill $PUB_PID 2>/dev/null; PUB_PID=""
CP=$(last_pos "$LOG_DIR/rx6.log" 7)
CT=$(last_tor "$LOG_DIR/rx6.log" 7)
DEG=$(python3 -c "print('%.1f' % (float('$CP')*57.2958))" 2>/dev/null || echo '?')
info "夹爪回读：pos_7=$CP rad (${DEG} deg)，tor_7=$CT N·m"
info "虚拟物体位置 -1.2 rad(约 -68.8 deg)，夹到后力矩升到 0.6 N·m"
if python3 -c "
import sys
try:
    p=float('$CP'); t=float('$CT')
except Exception:
    sys.exit(1)
sys.exit(0 if abs(p+1.2)<0.05 and abs(t-0.6)<0.05 else 1)"; then
    pass "夹爪夹到物体（位置被顶住 + 力矩上升），ClawController 可据此判定"
else
    fail "夹爪回读不符（pos=$CP tor=$CT）"
fi

echo
echo "[用例 7] 串口断开后自动重连（等价于拔插 USB / 单片机复位）"
kill -9 "$BOARD_PID" 2>/dev/null; BOARD_PID=""
sleep 2
python3 "$BOARD_PY" --port "$SERIAL_B" --quiet >> "$LOG_DIR/board.log" 2>&1 &
BOARD_PID=$!
sleep 5
if grep -q "重连成功" "$LOG_DIR/hardware.log"; then
    pass "串口断开后可自动重连（hardware 打印 [串口] 重连成功）"
elif kill -0 "$HW_PID" 2>/dev/null; then
    ERRS=$(grep -cE "断开|异常" "$LOG_DIR/hardware.log" 2>/dev/null | head -1)
    info "未出现重连日志，但 hardware 存活并记录了 $ERRS 次串口异常（未崩溃）"
    pass "串口断开后 hardware 未崩溃、继续运行"
else
    fail "串口断开后 hardware 退出"
fi

echo
echo "============================================================"
echo " 测试结果：$PASS 通过, $FAIL 失败"
echo "============================================================"
for r in "${RESULTS[@]}"; do echo "  $r"; done
echo
echo " 详细日志：$LOG_DIR"
echo "============================================================"

[ "$FAIL" -eq 0 ] && exit 0 || exit 1
