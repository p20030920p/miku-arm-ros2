#!/usr/bin/env python3
"""
make_architecture.py —— 生成 docs/figures/architecture.png

与 demo.gif 同一套视觉语言：深色底、克制的配色、只有一个高亮色。
画的是**实际实现**：话题名、类型全部对照 src/ 下的代码，不是示意图。

左右两条链路共用同一批算法节点；唯一差别是高亮的那一个方框。
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

BG = "#0e1014"
PANEL = "#171b21"
PANEL_EDGE = "#2a313b"
INK = "#e4e9f0"
DIM = "#8b95a5"
FAINT = "#5a6472"
BLUE = "#2f7fc1"
ORANGE = "#e08a2e"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "text.color": INK,
    "figure.facecolor": BG, "axes.facecolor": BG,
})

fig, ax = plt.subplots(figsize=(10.4, 4.5), dpi=170)
ax.set_facecolor(BG)
ax.set_xlim(0, 104)
ax.set_ylim(0, 46)
ax.axis("off")

LCOL, RCOL, BW, BH = 2.0, 54.0, 44.0, 11.0
LX, RX = LCOL + BW / 2, RCOL + BW / 2


def panel(x, y, w, h, title, sub, edge=PANEL_EDGE, tcol=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.5,rounding_size=1.0",
                                fc=PANEL, ec=edge, lw=1.3))
    ax.text(x + w / 2, y + h - 3.1, title, ha="center", va="center",
            fontsize=9.2, fontweight="bold", color=tcol or INK)
    ax.text(x + w / 2, y + h - 6.4, sub, ha="center", va="center",
            fontsize=7.0, color=DIM)


def arrow(x1, y1, x2, y2, color=BLUE, lw=1.5):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                 mutation_scale=12, lw=lw, color=color,
                                 shrinkA=2, shrinkB=2))


def tag(x, y, s, color=FAINT, size=7.0, ha="center"):
    ax.text(x, y, s, ha=ha, va="center", fontsize=size, color=color)


panel(24, 32, 76, 12, "arm_control",
      "KDL FK/IK   ·   linear planner   ·   gravity compensator   ·   claw FSM")
tag(11, 36.0, "same binary,", FAINT, 6.6)
tag(11, 33.9, "both paths", FAINT, 6.6)

panel(LCOL, 15, BW, BH, "hardware", "serial::Serial · /dev/ttyACM0", edge="#39424e")
panel(RCOL, 15, BW, BH, "miku_sim / sim_motor_board",
      "simulated motors · no serial", edge=BLUE, tcol=BLUE)

arrow(46, 32, 46, 26, BLUE)
tag(45.0, 29, "Arm_tx", BLUE, 7.0, ha="right")
arrow(70, 32, 70, 26, BLUE)
tag(68.4, 29, "Arm_tx", BLUE, 7.0, ha="right")

arrow(24, 20.5, 14, 20.5, FAINT, 1.2)
tag(19, 22.2, "Arm_rx", FAINT, 6.6)
arrow(98, 20.5, 88, 20.5, BLUE, 1.2)
tag(93, 22.2, "/Arm_rx", DIM, 6.6)

panel(LCOL, 2, BW, 8, "Damiao motors ×6 + gripper",
      "50 B down @ 0x86C1  /  46 B up @ 0x86C2", edge="#39424e", tcol=ORANGE)
panel(RCOL, 2, BW, 8, "simulated arm",
      "/joint_states → TF → RViz2", edge=BLUE, tcol=BLUE)

arrow(LX, 15, LX, 10, FAINT, 1.2)
arrow(RX, 15, RX, 10, BLUE, 1.2)

ax.add_patch(FancyBboxPatch((RCOL, 15), BW, BH,
                            boxstyle="round,pad=0.5,rounding_size=1.0",
                            fc="none", ec=BLUE, lw=2.0, ls=(0, (5, 3))))
tag(RCOL + BW / 2, 28.4, "the only difference", BLUE, 7.4)

fig.text(0.012, 0.955, "miku-arm-ros2", fontsize=12.5, fontweight="bold",
         color=INK, ha="left")
fig.text(0.012, 0.885, "message bus as implemented — topic names verbatim from the code",
         fontsize=7.8, color=DIM, ha="left")

fig.subplots_adjust(left=0.0, right=1.0, top=0.86, bottom=0.0)
fig.savefig("docs/figures/architecture.png", facecolor=BG)
print("wrote docs/figures/architecture.png")
