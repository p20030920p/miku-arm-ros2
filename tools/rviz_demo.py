#!/usr/bin/env python3
"""录制 RViz 里的 MoveIt 规划/执行演示，合成为 GIF。

为什么要脚本驱动而不是模拟鼠标：
  本机是 Wayland 会话，RViz 跑在 XWayland 上。XTEST 合成的移动事件能被
  Qt 收到（悬停高亮正常），但按下/抬起事件不会触发控件动作 —— 实测点
  File 菜单、Joints 标签、Grid 勾选框、Plan 按钮都没有任何反应。
  所以这里不去点 GUI，而是直接调 MoveIt 的规划服务，RViz 的
  MotionPlanning 面板会照常显示目标位姿、轨迹和机械臂运动，
  画面内容与手工操作时一致。

    python3 tools/rviz_demo.py --out docs/figures/demo_rviz.gif
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "win_capture"))

import numpy as np  # noqa: E402
import xgrab  # noqa: E402
from PIL import Image  # noqa: E402

PNGRAB = os.path.join(HERE, "win_capture", "pngrab")

import rclpy  # noqa: E402
from moveit_msgs.action import MoveGroup  # noqa: E402
from moveit_msgs.msg import (Constraints, JointConstraint,  # noqa: E402
                             MotionPlanRequest, WorkspaceParameters)
from rclpy.action import ActionClient  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

JOINTS = [f"joint_{i}" for i in range(1, 7)]
GROUP = "manipulator"
# 目标位形：第一轴转、第二三轴抬起、手腕跟随，动作幅度看得清又不夸张
GOAL = [1.05, -0.62, 0.72, 0.35, 0.70, -0.45]


class Driver(rclpy.node.Node):
    def __init__(self):
        super().__init__("rviz_demo_driver")
        self.latest = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.move = ActionClient(self, MoveGroup, "/move_action")

    def _on_js(self, msg):
        if set(JOINTS).issubset(set(msg.name)):
            self.latest = [msg.position[msg.name.index(j)] for j in JOINTS]

    def wait_state(self, timeout=40.0):
        """等第一帧 /joint_states。

        DDS 发现在这台机器上偶尔要十几秒（尤其刚 daemon stop 之后），
        所以给足时间并周期性打印等待时长，不要静默失败。
        """
        t0 = time.time()
        last = 0.0
        while self.latest is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            waited = time.time() - t0
            if waited - last >= 5.0:
                last = waited
                print(f"  等待 /joint_states… {waited:.0f}s")
        return self.latest


def build_goal(vel_scale, acc_scale):
    """构造一个关节空间的 MoveGroup goal。"""
    req = MotionPlanRequest()
    req.group_name = GROUP
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = vel_scale
    req.max_acceleration_scaling_factor = acc_scale
    req.workspace_parameters = WorkspaceParameters()
    req.workspace_parameters.header.frame_id = "base_link"
    req.workspace_parameters.min_corner.x = -1.0
    req.workspace_parameters.min_corner.y = -1.0
    req.workspace_parameters.min_corner.z = -1.0
    req.workspace_parameters.max_corner.x = 1.0
    req.workspace_parameters.max_corner.y = 1.0
    req.workspace_parameters.max_corner.z = 1.0

    c = Constraints()
    c.name = "goal"
    for name, val in zip(JOINTS, GOAL):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(val)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints = [c]

    g = MoveGroup.Goal()
    g.request = req
    g.planning_options.plan_only = False
    g.planning_options.replan = False
    g.planning_options.look_around = False
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "docs/figures/demo_rviz.gif"))
    ap.add_argument("--width", type=int, default=880, help="GIF 输出宽度")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="GIF 相对真实时间的播放倍速；2.0 表示快一倍")
    ap.add_argument("--frames-dir", default="/tmp/rviz_frames",
                    help="中间帧的落盘目录，每次运行会清空")
    ap.add_argument("--vel-scale", type=float, default=0.5,
                    help="MoveIt 速度缩放；只影响轨迹的时间参数化，"
                         "真正拖慢机械臂的是 launch 的 speed_scale")
    ap.add_argument("--acc-scale", type=float, default=0.5,
                    help="MoveIt 加速度缩放，同上")
    ap.add_argument("--hold-fps", type=float, default=25.0,
                    help="抓帧频率")
    ap.add_argument("--crop", default="0,60,1400,920",
                    help="窗口内裁剪区域 x0,y0,x1,y1；去掉底部 Displays 面板")
    ap.add_argument("--capture", default="0,60,1900,1250",
                    help="XShm 抓取的窗口区域 x0,y0,x1,y1")
    args = ap.parse_args()

    os.makedirs(args.frames_dir, exist_ok=True)
    for f in os.listdir(args.frames_dir):
        os.remove(os.path.join(args.frames_dir, f))

    # 找 RViz 客户窗口（跳过 mutter 框架窗口：它的类名不是 rviz2）
    wins = [w for w in xgrab.find_windows(600) if w[3] == "rviz2" and w[1] > 800]
    if not wins:
        print("找不到 RViz 窗口，先把 demo launch 起起来", file=sys.stderr)
        return 1
    wid, ww, wh = wins[0][0], wins[0][1], wins[0][2]
    print(f"RViz 窗口 0x{wid:x} {ww}x{wh}")

    # 上一次录制留下的 ros2 daemon 可能缓存旧图，导致订阅收不到
    # /joint_states（实测踩过）。先停掉，让节点重新发现。
    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)

    rclpy.init()
    drv = Driver()
    start = drv.wait_state()
    if start is None:
        print("没收到 /joint_states", file=sys.stderr)
        return 1
    print("起始位形:", [round(v, 3) for v in start])

    # 窗口坐标下的目标区域 (x0,y0,x1,y1)
    box = tuple(int(v) for v in args.crop.split(",")) if args.crop else None
    if box:
        bx0, by0, bx1, by1 = box
        cbox = tuple(int(v) for v in args.capture.split(","))
        col_box = (max(0, bx0 - cbox[0]), max(0, by0 - cbox[1]),
                   bx1 - cbox[0], by1 - cbox[1])
    else:
        col_box = None

    frames = []
    frame_t = []          # 每帧的实际抓取时刻，用来还原真实时长
    idx = 0

    cap = tuple(int(v) for v in args.capture.split(","))

    def shoot():
        """抓一帧。

        用 pngrab 走 XShmGetImage + stdout 管道：1900x1190 单帧约 43 ms
        （ctypes 的 XGetImage 路径要 340 ms，帧率差一个数量级）。
        抓到的原始像素先在本地裁列，再交给 PIL 缩放。
        """
        nonlocal idx
        n = (cap[2] - cap[0]) * (cap[3] - cap[1]) * 3
        out = subprocess.run([PNGRAB, hex(wid), "-", *map(str, cap)],
                             capture_output=True, check=True).stdout
        if len(out) < n:
            raise RuntimeError(f"pngrab 只返回 {len(out)} 字节，期望 {n}")
        im = Image.frombytes("RGB", (cap[2] - cap[0], cap[3] - cap[1]), out[:n])
        if col_box:
            im = im.crop(col_box)
        w = args.width
        h = int(im.height * w / im.width)
        im = im.resize((w, h), Image.LANCZOS)
        path = os.path.join(args.frames_dir, f"{idx:05d}.png")
        im.save(path)
        frames.append(path)
        frame_t.append(time.time())
        idx += 1
    every = 1.0 / args.hold_fps

    def hold(seconds, every=every):
        t0 = time.time()
        while time.time() - t0 < seconds:
            shoot()
            time.sleep(max(0.0, every - (time.time() - t0) % every))

    def wait_goal(handle, timeout):
        t0 = time.time()
        fut = handle.get_result_async()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(drv, timeout_sec=0.0)
        return fut.result() if fut.done() else None

    print("静止起手帧…")
    hold(1.2)

    # 规划阶段（此时画面里出现目标位姿与轨迹）：一边 spin 一边抓帧
    print("发起规划与执行（MoveIt /move_action）…")
    if not drv.move.wait_for_server(timeout_sec=15.0):
        print("MoveIt /move_action 不在，确认 move_group 起来了", file=sys.stderr)
        return 1
    t_send = time.time()
    fut = drv.move.send_goal_async(build_goal(args.vel_scale, args.acc_scale))
    handle = None
    while handle is None and time.time() - t_send < 20.0:
        rclpy.spin_once(drv, timeout_sec=0.0)
        if fut.done():
            handle = fut.result()
        shoot()
    if handle is None or not handle.accepted:
        print("goal 未被接受", file=sys.stderr)
        return 1
    print("目标已接受，继续抓帧到执行结束…")

    # 执行阶段：一直抓到机械臂到位并稳定
    target = np.array(GOAL)
    t0 = time.time()
    reached_at = None
    result = None
    while time.time() - t0 < 30.0:
        shoot()
        rclpy.spin_once(drv, timeout_sec=0.0)
        if drv.latest is not None:
            err = float(np.abs(np.array(drv.latest) - target).max())
            if err < 0.03 and reached_at is None:
                reached_at = time.time() - t0
        if reached_at is not None and time.time() - t0 > reached_at + 1.0:
            break
    result = wait_goal(handle, timeout=15.0)
    ok = result is not None and result.result.error_code.val == 1
    npts = len(result.result.planned_trajectory.joint_trajectory.points) if result else 0
    print(f"执行阶段录制 {time.time()-t0:.1f}s，到位用时 {reached_at}，"
          f"成功={ok} 轨迹点={npts}")

    final = drv.wait_state()
    err = float(np.abs(np.array(final) - target).max())
    print("结束位形:", [round(v, 3) for v in final], f"最大误差 {err:.3f} rad")

    drv.destroy_node()
    rclpy.shutdown()

    if not frames:
        print("没有抓到帧", file=sys.stderr)
        return 1

    # 合成 GIF：每帧沿用抓取时的真实间隔，这样播放速度由抓帧节奏决定，
    # 不再被"抽帧到目标帧率"那套换算悄悄改掉（之前就是这么把 4 s 压成 1 s 的）。
    imgs = [Image.open(p).convert("RGB") for p in frames]
    gaps = [(frame_t[i] - frame_t[i - 1]) * 1000.0 / args.speed
            for i in range(1, len(frame_t))]
    if not gaps:
        gaps = [1000.0 / args.hold_fps]
    delays = [max(20, min(1000, int(round(g)))) for g in gaps] + [int(gaps[-1])]
    pal = imgs[0].quantize(colors=128, method=Image.MEDIANCUT)
    quant = [im.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for im in imgs]
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    quant[0].save(args.out, save_all=True, append_images=quant[1:],
                  duration=delays, loop=0, optimize=True)
    total = sum(delays) / 1000.0
    size = os.path.getsize(args.out)
    print(f"已写出 {os.path.relpath(args.out, ROOT)}  {len(quant)} 帧  "
          f"播放 {total:.1f}s（真实 {sum(delays)*args.speed/1000:.1f}s）  {size/1024:.0f} KB")
    return 0 if err < 0.05 else 1


if __name__ == "__main__":
    sys.exit(main())
