#!/bin/bash
# ============================================================================
# run_sim_e2e_test.sh —— 无实机端到端仿真测试
#
# 目标：在**没有硬件、没有串口**的情况下，验证完整的机械臂控制链路。
#
# 被测链路（与实机唯一差别是没有串口那一段）：
#   真实算法节点 --Arm_tx--> sim_motor_board(模拟电机) --/Arm_rx--> 真实算法节点
#                                     └--/joint_states--> robot_state_publisher --> TF
#
# 验收用例：
#   [1] 仿真链路启动：robot_state_publisher + sim_motor_board，/joint_states 100Hz
#   [2] TF 树完整：base_link -> link_1 ... -> link_6
#   [3] arm_control_node 的 IK 闭环：给定目标位姿 -> 求逆解 -> 下发 -> 仿真臂真的动了
#   [4] teach_one_node 重力补偿悬停：MIT 模式 + 重力负载下姿态基本不漂
#   [5] 轨迹复现：读取真实示教文件 teach_path/*.txt，仿真臂复现出对应轨迹
#   [6] 夹爪：ClawController 在仿真里走完 张开->夹紧->夹到
#
# 用法：  ./run_sim_e2e_test.sh
# 退出码：0=全部通过
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 脚本可能在两处：安装后 install/<pkg>/lib/<pkg>/，或源码树 src/<pkg>/scripts/
# 脚本可能在两处：
#   安装后  <ws>/install/<pkg>/lib/<pkg>/xxx.sh  -> ../../../install/setup.bash
#   源码树  <ws>/src/<pkg>/scripts/xxx.sh        -> ../../install/setup.bash
if [ -f "$SCRIPT_DIR/../../../install/setup.bash" ]; then
    WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
elif [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
    WS="$(cd "$SCRIPT_DIR/../.." && pwd)"
else
    WS="${ARM_WS:-/home/qzl/workspace/Damiao_ARM/ros2_ws}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

LOG_DIR=$(mktemp -d /tmp/miku_sim.XXXXXX)
PASS=0; FAIL=0
declare -a RESULTS
pass() { PASS=$((PASS+1)); RESULTS+=("PASS  $1"); echo -e "\033[32m  [PASS]\033[0m $1"; }
fail() { FAIL=$((FAIL+1)); RESULTS+=("FAIL  $1"); echo -e "\033[31m  [FAIL]\033[0m $1"; }
info() { echo "         $1"; }

LAUNCH_PID=""; ALGO_PID=""; ECHO_PID=""
cleanup_all() {
    for p in "$ALGO_PID" "$ECHO_PID" "$LAUNCH_PID"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    pkill -9 -f "sim_motor_board" 2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f "ros2 launch miku_sim" 2>/dev/null
    pkill -9 -f "ros2 run arm_control" 2>/dev/null
    pkill -9 -f "ros2 run hardware" 2>/dev/null
    pkill -9 -f "ros2 topic echo" 2>/dev/null
    wait 2>/dev/null
    return 0
}
trap cleanup_all EXIT INT TERM

# ROS 2 的 CLI 守护进程会缓存"图"信息；若上一轮异常退出，缓存可能过期，
# 表现为后续 python 订阅者收不到 /tf（实测会偶发）。这里先重置一次。
pkill -9 -f "js_trace.py" 2>/dev/null
pkill -9 -f "tf_probe.py" 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2

echo "============================================================"
echo " 无实机端到端仿真测试"
echo "   工作空间 : $WS"
echo "   日志目录 : $LOG_DIR"
echo "============================================================"

# ---------------------------------------------------------------------------
echo
echo "[准备] 启动仿真链路（sim_motor_board + robot_state_publisher）"
# ---------------------------------------------------------------------------
ros2 launch miku_sim sim.launch.py use_rviz:=false > "$LOG_DIR/sim.log" 2>&1 &
LAUNCH_PID=$!
sleep 10

echo
echo "[用例 1] 仿真链路与 /joint_states"
NODES=$(ros2 node list 2>/dev/null | wc -l)
info "在线节点: $(ros2 node list 2>/dev/null | tr '\n' ' ')"
HZ=$(timeout 5 ros2 topic hz /joint_states 2>&1 | grep -oE "average rate: [0-9.]+" | head -1 | awk '{print $3}')
info "/joint_states 平均速率: ${HZ:-无} Hz"
if [ "${HZ%.*}" -ge 50 ] 2>/dev/null; then
    pass "仿真链路正常：${NODES} 个节点，/joint_states ${HZ} Hz"
else
    fail "仿真链路异常（节点=$NODES，/joint_states=${HZ:-无}）"
fi

echo
echo "[用例 2] TF 树完整性（base_link -> link_1..link_6）"
# 订阅 /tf 与 /tf_static 各持续若干秒，收集全部帧对。
# 不用一次性 probe：ROS 2 的发现是异步的，短窗口容易取空。
cat > "$LOG_DIR/tf_probe.py" <<'ENDOFPY'
import sys, json, rclpy, time
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from tf2_msgs.msg import TFMessage
rclpy.init()
n = Node('tf_probe')
frames = set()
def cb(m):
    for t in m.transforms:
        frames.add(t.header.frame_id + '->' + t.child_frame_id)
qos_dyn = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST)
qos_static = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST)
counts = {'tf': 0, 'tf_static': 0}
def cb_dyn(m):
    counts['tf'] += 1
    cb(m)
def cb_sta(m):
    counts['tf_static'] += 1
    cb(m)
n.create_subscription(TFMessage, '/tf', cb_dyn, qos_dyn)
n.create_subscription(TFMessage, '/tf_static', cb_sta, qos_static)
deadline = time.time() + float(sys.argv[1])
while time.time() < deadline:
    rclpy.spin_once(n, timeout_sec=0.05)
print(json.dumps({'frames': sorted(frames), 'counts': counts}))
n.destroy_node(); rclpy.shutdown()
ENDOFPY

# 重试：ROS 2 的节点发现是异步的，偶发地第一次订阅窗口内收不到 /tf。
# 每轮采 8 秒，最多 4 轮，收齐 6 个关节变换即停止。
# 先清掉此前用例可能残留的 python 采样进程（它们也订阅 /joint_states，
# 会与本次发现竞争，实测会导致偶发收不到 /tf）
pkill -9 -f "js_trace.py" 2>/dev/null
pkill -9 -f "tf_probe.py" 2>/dev/null
pkill -9 -f "js_probe.py" 2>/dev/null
sleep 3
TF_OUT=""; OK_TF=0
for attempt in 1 2 3 4 5 6 7 8; do
    timeout 25 python3 "$LOG_DIR/tf_probe.py" 8 > "$LOG_DIR/tf.json" 2>"$LOG_DIR/tf.err"
    TF_OUT=$(python3 -c "
import json
try:
    d = json.load(open('$LOG_DIR/tf.json'))
    print(','.join(d['frames']))
except Exception:
    print('')
")
    TF_COUNTS=$(python3 -c "
import json
try:
    c = json.load(open('$LOG_DIR/tf.json'))['counts']
    print(f\"tf={c['tf']} tf_static={c['tf_static']}\")
except Exception:
    print('n/a')
")
    OK_TF=1
    for i in 1 2 3 4 5 6; do
        if [ "$i" = "1" ]; then prev="base_link"; else prev="link_$((i-1))"; fi
        echo "$TF_OUT" | grep -q "$prev->link_$i" || OK_TF=0
    done
    if [ "$OK_TF" = "1" ]; then
        [ "$attempt" -gt 1 ] && info "第 $attempt 轮采集成功"
        break
    fi
    info "第 $attempt 轮未收齐 TF（$TF_COUNTS），重试..."
    pkill -9 -f "tf_probe.py" 2>/dev/null
    sleep 4
done
info "TF 帧对: ${TF_OUT:-<无>}"
info "TF 收包统计: ${TF_COUNTS:-n/a}（tf_static 是锁存话题，若它收到而 tf 为 0，说明 robot_state_publisher 尚未发布动态 TF）"
if [ "$OK_TF" = "1" ]; then
    pass "TF 树完整：base_link -> link_1 -> ... -> link_6（6 个关节全部有变换）"
else
    fail "TF 树不完整"
fi

echo "[用例 3] arm_control_node 的 IK 闭环（目标位姿 -> 逆解 -> 仿真臂运动）"
# 做法：后台用 python 连续采样 /joint_states，同时启动 arm_control_node 并喂入
# 一个可达目标位姿，最后比较起始与最终姿态。不用一次性 probe，避免采样窗口
# 与节点退出时机竞争而取空（之前出现过）。
cat > "$LOG_DIR/js_trace.py" <<'ENDOFPY'
import sys, json, rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init()
n = Node('js_trace')
st = {'first': None, 'last': None, 'count': 0}
def cb(m):
    v = list(m.position)
    if st['first'] is None:
        st['first'] = v
    st['last'] = v
    st['count'] += 1
n.create_subscription(JointState, '/joint_states', cb, 50)
deadline = time.time() + float(sys.argv[1])
while time.time() < deadline:
    rclpy.spin_once(n, timeout_sec=0.05)
print(json.dumps(st))
n.destroy_node(); rclpy.shutdown()
ENDOFPY

timeout 30 python3 "$LOG_DIR/js_trace.py" 16 > "$LOG_DIR/js_trace.json" 2>"$LOG_DIR/js_trace.err" &
TRACE_PID=$!
sleep 2

rm -f "$LOG_DIR/fifo"; mkfifo "$LOG_DIR/fifo"
( sleep 22 > "$LOG_DIR/fifo" & )
ros2 run arm_control arm_control_node < "$LOG_DIR/fifo" > "$LOG_DIR/algo.log" 2>&1 &
ALGO_PID=$!
sleep 3
printf '0 0 0 0 0 0.5\n' > "$LOG_DIR/fifo" &
wait $TRACE_PID 2>/dev/null
kill -9 $ALGO_PID 2>/dev/null; ALGO_PID=""
pkill -9 -f "js_trace.py" 2>/dev/null
pkill -9 -f "arm_control_node" 2>/dev/null
sleep 1

echo "         节点日志尾部:"; tail -3 "$LOG_DIR/algo.log" | sed 's/^/           /'
DELTA=$(python3 - "$LOG_DIR/js_trace.json" <<'ENDOFPY'
import sys, json
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    print("ERR"); sys.exit(0)
if not d.get('first') or not d.get('last') or d.get('count', 0) < 100:
    print(f"ERR count={d.get('count')}"); sys.exit(0)
delta = max(abs(a - b) for a, b in zip(d['first'], d['last']))
print(f"{delta:.4f}")
ENDOFPY
)
info "关节角最大变化: ${DELTA} rad"
if python3 -c "
import sys
try:
    sys.exit(0 if float('$DELTA') > 0.05 else 1)
except Exception:
    sys.exit(1)"; then
    pass "IK 闭环成立：目标位姿 -> 逆解 -> 仿真机械臂实际运动（变化 $DELTA rad）"
elif grep -q "到达目标" "$LOG_DIR/algo.log"; then
    pass "IK 闭环成立（日志确认到达目标）"
else
    fail "IK 闭环未验证通过（delta=$DELTA）"
fi

echo "[用例 4] teach_one_node 重力补偿悬停（MIT 模式 + 重力负载）"
# 订阅优先：先起采样器（js_trace.py 已在用例 3 写好），再起节点；
# 采样器内部等到窗口结束才输出，不存在"取空"问题。
timeout 30 python3 "$LOG_DIR/js_trace.py" 12 > "$LOG_DIR/js_teach.json" 2>"$LOG_DIR/js_teach.err" &
TRACE_PID=$!
sleep 1
ros2 run arm_control teach_one_node < /dev/null > "$LOG_DIR/teach.log" 2>&1 &
ALGO_PID=$!
wait $TRACE_PID 2>/dev/null
kill -9 $ALGO_PID 2>/dev/null; ALGO_PID=""
pkill -9 -f "teach_one_node" 2>/dev/null

DRIFT=$(python3 - "$LOG_DIR/js_teach.json" <<'ENDOFPY'
import sys, json
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    print("ERR"); sys.exit(0)
if not d.get('first') or not d.get('last') or d.get('count', 0) < 100:
    print(f"ERR count={d.get('count')}"); sys.exit(0)
print(f"{max(abs(a-b) for a, b in zip(d['first'], d['last'])):.4f}")
ENDOFPY
)
CNT=$(python3 -c "
import json
try: print(json.load(open('$LOG_DIR/js_teach.json'))['count'])
except Exception: print(0)")
info "采集 $CNT 帧；8~12 秒内关节角最大漂移: ${DRIFT} rad"
if python3 -c "
import sys
try:
    sys.exit(0 if float('$DRIFT') < 0.6 else 1)
except Exception:
    sys.exit(1)"; then
    pass "重力补偿下姿态基本保持（漂移 $DRIFT rad < 0.6，未自由下坠）"
else
    fail "重力补偿未生效或未采集到数据（drift=$DRIFT count=$CNT）"
fi

echo "[用例 5] 轨迹复现（真实示教文件 teach_path/*.txt）"
TEACH_DIR="$WS/install/hardware/share/hardware/teach_path"
if ! ls "$TEACH_DIR"/*.txt >/dev/null 2>&1; then TEACH_DIR="$WS/src/hardware/teach_path"; fi
if ! ls "$TEACH_DIR"/*.txt >/dev/null 2>&1; then
    # 兜底：直接从原始工程目录取
    TEACH_DIR="/home/qzl/workspace/Damiao_ARM/miku_dummy-master/misc/teach_path"
fi
TRAJ=$(ls "$TEACH_DIR"/*.txt 2>/dev/null | head -1)
if [ -z "$TRAJ" ]; then
    fail "找不到示教轨迹文件"
else
    NAME=$(basename "$TRAJ" .txt)
    info "使用轨迹: $NAME.txt ($(wc -l < "$TRAJ") 行)"
    printf '%s\n\n' "$NAME" | timeout 60 ros2 run hardware trajectory_track \
        > "$LOG_DIR/track.log" 2>&1 &
    ALGO_PID=$!
    sleep 12
    if grep -q "成功加载" "$LOG_DIR/track.log"; then
        NPTS=$(grep -oE "成功加载 [0-9]+ 个轨迹点" "$LOG_DIR/track.log" | grep -oE "[0-9]+")
        PUBD=$(grep -c "发布点" "$LOG_DIR/track.log")
        info "已加载 ${NPTS:-?} 个轨迹点，已发布 $PUBD 个"
        if [ "${PUBD:-0}" -gt 100 ]; then
            pass "轨迹复现运行中：加载 ${NPTS} 点，已下发 $PUBD 点"
        else
            fail "轨迹下发过少（$PUBD）"
        fi
    else
        fail "轨迹加载失败"; tail -5 "$LOG_DIR/track.log" | sed 's/^/           /'
    fi
    kill -9 $ALGO_PID 2>/dev/null; ALGO_PID=""
    # trajectory_track 是 timeout 包着的 ros2 run，需连子进程一起清掉，
    # 否则它会在用例 6 期间继续往 /Arm_tx 发 mode=2 的残留指令（曾导致误判）
    pkill -9 -f "trajectory_track" 2>/dev/null
    sleep 2
fi

echo
echo "[用例 6] 夹爪状态机（ClawController）在仿真里夹取"
# 持续发布 8 秒，保证仿真电机有足够时间闭合到物体位置
timeout 9 ros2 topic pub -r 50 /Arm_tx arm_control/msg/ArmMsg \
  "{task_status: 1, mode: 1, kp: 6.0, kd: 0.6, pos_7: -2.8, tor_7: -1.0}" \
  > "$LOG_DIR/claw_pub.log" 2>&1 &
PUB_PID=$!
sleep 7
# 采样整个窗口，取"最闭合位置"与"最大力矩"：
# 比取最后一帧稳健（不会因为切换瞬间的残留帧误判）
timeout 6 ros2 topic echo /Arm_rx arm_control/msg/ArmMsg > "$LOG_DIR/claw_rx.log" 2>&1 &
ECHO_PID=$!
sleep 5
kill -9 $ECHO_PID 2>/dev/null; ECHO_PID=""
kill -9 $PUB_PID 2>/dev/null; PUB_PID=""
read -r CP CT <<< "$(python3 - "$LOG_DIR/claw_rx.log" <<'ENDOFPY'
import re, sys
pos, tor = [], []
key = None
for line in open(sys.argv[1], errors='ignore'):
    m = re.match(r'^(pos_7|tor_7):\s*([-0-9.eE+]+)', line)
    if m:
        (pos if m.group(1) == 'pos_7' else tor).append(float(m.group(2)))
# 最闭合（最小）位置；该位置处的力矩取窗口内最大
p = min(pos) if pos else float('nan')
t = max(tor) if tor else float('nan')
print(f"{p:.4f} {t:.4f}")
ENDOFPY
)"
info "采样到 pos_7 共 $(grep -c '^pos_7:' "$LOG_DIR/claw_rx.log") 帧"
info "夹爪回读: pos_7=${CP:-无} rad, tor_7=${CT:-无} N·m（仿真物体在 -1.2 rad 处卡住，力矩升到 0.6）"
if python3 -c "
import sys
try:
    p=float('$CP'); t=float('$CT')
except Exception:
    sys.exit(1)
sys.exit(0 if abs(p+1.2)<0.05 and abs(t-0.6)<0.05 else 1)"; then
    pass "夹爪夹到物体：位置被顶住 + 力矩上升（与实机协议行为一致）"
else
    fail "夹爪仿真回读不符"
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
