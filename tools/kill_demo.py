#!/usr/bin/env python3
"""只结束本仓库演示留下的进程，绝不误伤别的 ROS 会话。

判定顺序（进程 cmdline 同时满足才杀）：
  1. 出现本仓库路径 —— 或 —— 是我们自己那几个节点名
     —— 或 —— 是跑在测试 domain 上的 move_group
  2. 且不出现 race_navigation（用户自己的仿真，必须留着）

用 /proc 直接读，并且跳过自身及其父链，避免 pgrep -f 那种
"模式匹配到自己命令行" 的自杀问题。
"""
import os
import signal
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MARKER = ROOT.encode()
# 我们自己 launch 出来的节点，即使 cmdline 里没有路径也能认出来
OURS_EXTRA = (b"miku_moveit_demo", b"sim_motor_board", b"trajectory_bridge")
# 用户自己的会话，一律不碰
FOREIGN = (b"race_navigation",)
# 我们跑演示/测试时用的 domain。move_group 的 cmdline 里不带仓库路径，
# 只能靠 domain 区分，所以这里必须限定范围，绝不能误杀默认 domain。
TEST_DOMAINS = set(range(150, 200))


def protected_pids():
    """自身、当前进程组、以及它们的全部祖先，都不杀。"""
    keep = set()
    starts = [os.getpid()]
    try:
        starts.append(os.getpgrp())
    except Exception:
        pass
    for start in starts:
        pid = start
        for _ in range(10):
            keep.add(pid)
            try:
                with open(f"/proc/{pid}/stat") as fh:
                    pid = int(fh.read().split(") ", 1)[1].split()[1])
            except Exception:
                break
            if pid <= 1:
                break
    return keep


def cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            return fh.read()
    except Exception:
        return b""


def our_procs(keep):
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in keep:
            continue
        cmd = cmdline(pid)
        if not cmd:
            continue
        if any(f in cmd for f in FOREIGN):
            continue
        if MARKER in cmd or any(m in cmd for m in OURS_EXTRA):
            found.append((pid, cmd.replace(b"\0", b" ").decode(errors="replace").strip()))
        elif b"move_group" in cmd and is_test_domain(pid):
            # move_group 的 cmdline 里没有路径，只能靠 domain 认。
            # 只认我们自己那批测试 domain，用户默认 domain 上的绝不动。
            found.append((pid, f"[测试 domain 上的 move_group] "
                               + cmd.replace(b"\0", b" ").decode(errors="replace").strip()))
    return found


def is_test_domain(pid):
    """进程的 ROS_DOMAIN_ID 是否落在我录演示/跑测试用的区间内。"""
    try:
        with open(f"/proc/{pid}/environ", "rb") as fh:
            env = fh.read()
    except Exception:
        return False
    for item in env.split(b"\0"):
        if item.startswith(b"ROS_DOMAIN_ID="):
            try:
                return int(item.split(b"=", 1)[1]) in TEST_DOMAINS
            except ValueError:
                return False
    return False


def main():
    keep = protected_pids()
    procs = our_procs(keep)
    if not procs:
        print("没有本仓库的残留进程")
        return 0
    for pid, cmd in procs:
        print(f"SIGTERM {pid}  {cmd[:100]}")
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(3)
    left = our_procs(keep)
    for pid, cmd in left:
        print(f"SIGKILL {pid}  {cmd[:100]}")
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    print(f"清理完成（TERM {len(procs)}，KILL {len(left)}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
