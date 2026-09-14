#!/usr/bin/env python3
"""
make_architecture.py —— 生成 docs/figures/architecture.png

画的是**实际实现**的消息总线：话题名、消息类型、频率、方向全部对照代码，
不画示意图。左侧为实机链路，右侧为仿真链路，两者共用同一批算法节点。
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

BG = "#ffffff"
INK = "#1b1f24"
DIM = "#6b7280"
ACCENT = "#22314E"
SIM = "#1f77b4"
RED = "#c62828"
GREY = "#9ca3af"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 9, "text.color": INK,
    "figure.facecolor": BG, "axes.facecolor": BG,
})

fig, ax = plt.subplots(figsize=(11.0, 6.6), dpi=160)
ax.set_xlim(-3, 103)
ax.set_ylim(-4, 74)
ax.axis("off")

LCOL, RCOL = 2.0, 53.0
BW = 45.0
LX, RX = LCOL + BW / 2, RCOL + BW / 2


def box(x, y, w, h, title, lines, fc, ec, tc=None, tsize=8.5, sub_size=6.8):
    tc = tc or INK
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.6,rounding_size=1.2",
                                fc=fc, ec=ec, lw=1.25))
    ax.text(x + w / 2, y + h - 2.7, title, ha="center", va="center",
            fontsize=tsize, fontweight="bold", color=tc)
    for i, ln in enumerate(lines):
        ax.text(x + w / 2, y + h - 5.6 - i * 2.5, ln, ha="center",
                va="center", fontsize=sub_size, color=tc, alpha=0.88)


def arrow(x1, y1, x2, y2, color=ACCENT, lw=1.3, ls="-"):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                 mutation_scale=11, lw=lw, color=color,
                                 linestyle=ls, shrinkA=2, shrinkB=2))


def label(x, y, text, color=DIM, size=6.8, ha="center", weight="normal"):
    ax.text(x, y, text, ha=ha, va="center", fontsize=size, color=color,
            fontweight=weight)


# ------------------------------------------------------------------ 图例（最上方）
ax.add_patch(FancyBboxPatch((LCOL, 66.5), 40, 5.2,
                            boxstyle="round,pad=0.4,rounding_size=1.0",
                            fc="#fdf2f2", ec=RED, lw=1.05))
label(LCOL + 20, 69.1, "simulation replaces ONLY the highlighted box",
      RED, 7.1, weight="bold")

# ------------------------------------------------------------------ 视觉前端
Y_CAM, H_CAM = 54.5, 9.5
box(LCOL, Y_CAM, 20, H_CAM, "deep_camera + aruco",
    ["ArucoDetector", "solvePnP → /aruco/pose"],
    "#f7f8fa", GREY, tsize=7.4, sub_size=6.3)
label(LCOL + 10, Y_CAM - 2.2, "vision front-end (grasping path only)", GREY, 6.2)

# ------------------------------------------------------------------ 算法层
Y_ALGO, H_ALGO = 54.5, 9.5
box(LCOL + 23, Y_ALGO, 44.5, H_ALGO, "arm_control",
    ["KinematicsSolver · KDL FK/IK    ·    LinearPlanner",
     "GravityCompensator · tau = tau_g x gains    ·    ClawController"],
    "#eef2f8", ACCENT, tc=ACCENT, tsize=8.4, sub_size=6.1)

arrow(LCOL + 20, Y_CAM + H_CAM / 2, LCOL + 22.5, Y_ALGO + H_ALGO / 2, color=GREY)

# ------------------------------------------------------------------ 两条链路
Y_LINK, H_LINK = 30.0, 14.0
box(LCOL, Y_LINK, BW, H_LINK, "hardware   ·   real robot",
    ["serial::Serial  (POSIX termios, in-tree)",
     "50 B down @ 0x86C1   /   46 B up @ 0x86C2",
     "×1000 fixed point  ·  auto-reconnect"],
    "#f3f4f6", GREY)

box(RCOL, Y_LINK, BW, H_LINK, "miku_sim / sim_motor_board   ·   no hardware",
    ["subscribes /Arm_tx,   publishes /Arm_rx",
     "position & MIT modes + gravity load",
     "gripper stalls on contact (0.6 N·m)"],
    "#eef6ff", SIM, tc=SIM)

arrow(LX, Y_ALGO, LX, Y_LINK + H_LINK)
label(LX + 1.8, Y_ALGO - 3.0, "Arm_tx", ACCENT, ha="left")
arrow(LX, Y_LINK, LX, Y_LINK - 5.5, color=DIM)
label(LX + 1.8, Y_LINK - 3.0, "Arm_rx", DIM, ha="left")

arrow(RX, Y_ALGO, RX, Y_LINK + H_LINK, color=SIM)
label(RX + 1.8, Y_ALGO - 3.0, "Arm_tx", SIM, ha="left")
arrow(RX, Y_LINK, RX, Y_LINK - 5.5, color=SIM)
label(RX + 1.8, Y_LINK - 3.0, "/Arm_rx", SIM, ha="left")

# ------------------------------------------------------------------ 执行层
Y_EXE, H_EXE = 11.0, 9.5
box(LCOL, Y_EXE, BW, H_EXE, "Damiao motors ×6 + gripper",
    ["MCU driver board on /dev/ttyACM0"], "#f3f4f6", GREY, tsize=8.2)

box(RCOL, Y_EXE, BW, H_EXE, "virtual MCU board (socat PTY)",
    ["byte-faithful protocol peer"], "#eef6ff", SIM, tc=SIM, tsize=8.2)

# 串口段：实机链路用虚线标注
ax.plot([LCOL + 4, LCOL + 4], [Y_LINK, Y_EXE + H_EXE], color=GREY,
        lw=1.0, ls=":")
label(LCOL + 2.6, (Y_LINK + Y_EXE + H_EXE) / 2, "serial", GREY, 6.4, ha="right")

# ------------------------------------------------------------------ 可视化
Y_VIS, H_VIS = -3.0, 9.5
box(26.0, Y_VIS, 48.0, H_VIS, "robot_state_publisher   →   TF   →   RViz2",
    ["/joint_states   sensor_msgs/JointState   @ 100 Hz"],
    "#f7f8fa", GREY, tsize=8.4)

arrow(LX + 12, Y_EXE, 40.0, Y_VIS + H_VIS - 0.3, color=DIM)
arrow(RX - 12, Y_EXE, 60.0, Y_VIS + H_VIS - 0.3, color=SIM)

# ------------------------------------------------------------------ 取景
fig.suptitle("miku-arm-ros2  ·  message bus as implemented",
             fontsize=12.5, fontweight="bold", x=0.035, ha="left", y=0.985)
fig.text(0.035, 0.928,
         "Every algorithm node is the same binary in both paths. "
         "Topic names, types and rates are verbatim from the code.",
         fontsize=8.2, color=DIM, ha="left")

fig.subplots_adjust(left=0.005, right=0.995, top=0.905, bottom=0.005)
fig.savefig("docs/figures/architecture.png", facecolor=BG)
print("wrote docs/figures/architecture.png")
