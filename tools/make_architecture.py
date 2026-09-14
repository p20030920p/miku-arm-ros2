#!/usr/bin/env python3
"""
make_architecture.py —— 生成 docs/figures/architecture.png

按 IEEE / RA-L 论文中系统框图的惯例绘制：

  · 白底、无填充、细黑实线框；层次由线型与字重表达，不用装饰色
  · Times（Nimbus Roman）正文 + Helvetica（Nimbus Sans）标签 —— IEEE 字体搭配
  · 自上而下的块流程，两种实验条件为并列支路
  · **先测量、后布局**：每个框的宽高由实测文字决定，纵向位置由累计高度加固定
    间距推出，因此框与框不可能重叠、文字不可能溢出
  · 图题单独占用底部的固定带，"Fig. 1." 加粗起头

信号名与 src/ 下代码逐字一致。
"""

import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

SERIF, SANS = "Nimbus Roman", "Nimbus Sans"
plt.rcParams.update({
    "font.family": SERIF, "font.size": 8, "figure.facecolor": "white",
    "axes.facecolor": "white", "text.color": "black", "savefig.facecolor": "white",
})
LW_BOX, LW_ARR = 0.9, 0.7
GREY = "0.32"


class Canvas:
    def __init__(self, w_in, h_in):
        self.fig = plt.figure(figsize=(w_in, h_in), dpi=300)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, 100); self.ax.set_ylim(0, 100); self.ax.axis("off")
        self._r = self.fig.canvas.get_renderer()
        self._ux = 100.0 / (self.fig.get_dpi() * w_in)
        self._uy = 100.0 / (self.fig.get_dpi() * h_in)

    # ---------- 测量 ----------
    def tw(self, s, size, family=SANS, style="normal", weight="normal"):
        w = 0.0
        for ln in str(s).split("\n"):
            t = self.ax.text(0, -500, ln, fontsize=size, fontfamily=family,
                             fontstyle=style, fontweight=weight)
            w = max(w, t.get_window_extent(renderer=self._r).width * self._ux)
            t.remove()
        return w

    def th(self, s, size, family=SANS, style="normal", weight="normal"):
        t = self.ax.text(0, -500, s, fontsize=size, fontfamily=family,
                         fontstyle=style, fontweight=weight)
        h = t.get_window_extent(renderer=self._r).height * self._uy
        t.remove()
        return h

    def wrap(self, text, size, width_units):
        lines, cur = [], ""
        for word in str(text).split():
            trial = (cur + " " + word).strip()
            if self.tw(trial, size, SERIF, style="italic") <= width_units or not cur:
                cur = trial
            else:
                lines.append(cur); cur = word
        if cur:
            lines.append(cur)
        return lines

    # ---------- 规格 ----------
    def spec(self, name, sub, ns=8.2, ss=6.8, wrap_at=21.0):
        """返回 (宽, 高, 折行后的副标题行, 行高)"""
        lines = self.wrap(sub, ss, wrap_at) if sub else []
        w = self.tw(name, ns)
        for ln in lines:
            w = max(w, self.tw(ln, ss, SERIF, style="italic"))
        w += 4.2
        hn = self.th(name, ns, SANS)
        if lines:
            lh = self.th("Agq", ss, SERIF, style="italic") * 1.45
            h = hn + lh * len(lines) + 4.6
        else:
            lh = 0.0
            h = hn + 6.4
        return w, h, lines, lh

    def draw(self, cx, cy, name, sub, ns=8.2, ss=6.8, wrap_at=21.0, min_w=0.0):
        w, h, lines, lh = self.spec(name, sub, ns, ss, wrap_at)
        w = max(w, min_w)
        self.ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fill=False,
                                    ec="black", lw=LW_BOX, zorder=3))
        y = cy + h / 2 - 2.3
        self.ax.text(cx, y, name, ha="center", va="top", fontsize=ns,
                     fontfamily=SANS, zorder=4)
        for i, ln in enumerate(lines):
            self.ax.text(cx, y - self.th(name, ns, SANS) - lh * (i + 0.45), ln,
                         ha="center", va="top", fontsize=ss, fontfamily=SERIF,
                         fontstyle="italic", color=GREY, zorder=4)
        return dict(cx=cx, cy=cy, w=w, h=h, top=cy + h / 2, bot=cy - h / 2,
                    left=cx - w / 2, right=cx + w / 2)

    # ---------- 图元 ----------
    def arr(self, x1, y1, x2, y2, color="black", lw=LW_ARR, ms=5):
        self.ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                          mutation_scale=ms, lw=lw, color=color,
                                          shrinkA=0, shrinkB=0, zorder=2))

    def line(self, xs, ys, color=GREY, lw=0.6):
        self.ax.plot(xs, ys, color=color, lw=lw, zorder=2)

    def lab(self, x, y, s, size=6.5, color=GREY, ha="center", va="center"):
        self.ax.text(x, y, s, fontsize=size, color=color, ha=ha, va=va,
                     fontfamily=SERIF, fontstyle="italic", zorder=5)

    def head(self, x, y, s, size=8.0):
        self.ax.text(x, y, s, fontsize=size, fontfamily=SANS, ha="center",
                     va="center", zorder=5)


# ==================================================================== 内容
CAM = ("deep_camera + aruco", "ArUco detection; pose from solvePnP")
ALG = ("arm_control", "KinematicsSolver, LinearPlanner, GravityCompensator, "
                      "ClawController")
REAL = ("hardware", "serial::Serial over /dev/ttyACM0; 50 B down @ 0x86C1, "
                    "46 B up @ 0x86C2; x1000 fixed point; auto-reconnect")
SIM = ("sim_motor_board", "simulated motors: position and MIT modes, gravity load, "
                          "gripper contact; emits /Arm_rx and /joint_states")
VIS = ("robot_state_publisher \u2192 TF \u2192 RViz2", "/joint_states @ 100 Hz")

WRAP = 20.0
CAP_BAND = 9.0            # 底部留给图题的高度
GAP = 6.0                 # 行间距

d = Canvas(6.6, 5.6)

# ---------------- 先测量 ----------------
specs = {k: d.spec(n, s, wrap_at=WRAP) for k, (n, s) in
         dict(CAM=CAM, ALG=ALG, REAL=REAL, SIM=SIM, VIS=VIS).items()}
top_w = max(specs["CAM"][0], specs["ALG"][0])
real_w, sim_w = specs["REAL"][0], specs["SIM"][0]
span = real_w + GAP + sim_w
vis_w = specs["VIS"][0]
head_h = d.th("Agq", 8.0, SANS) * 1.4

# ---------------- 再布局：自上而下累计 ----------------
y = 96.0
y -= specs["CAM"][1]; cam_cy = y + specs["CAM"][1] / 2
y -= GAP
y -= specs["ALG"][1]; alg_cy = y + specs["ALG"][1] / 2
y -= GAP * 1.3
head_y = y - head_h / 2
y -= head_h + 1.6
y -= max(specs["REAL"][1], specs["SIM"][1])
br_cy = y + max(specs["REAL"][1], specs["SIM"][1]) / 2
y -= GAP * 2.0
y -= specs["VIS"][1]; vis_cy = y + specs["VIS"][1] / 2
bottom = y - GAP

# 水平位置
real_cx = 50.0 - span / 2 + real_w / 2
sim_cx = 50.0 + span / 2 - sim_w / 2
assert span <= 94.0, f"两支路合计 {span:.1f} 超宽"
assert bottom >= CAP_BAND, f"纵向空间不足：内容底部 {bottom:.1f} < 图题带 {CAP_BAND}"

# ---------------- 绘制 ----------------
cam = d.draw(50.0, cam_cy, *CAM, wrap_at=WRAP, min_w=min(top_w, 48.0))
alg = d.draw(50.0, alg_cy, *ALG, wrap_at=WRAP, ns=9.0, min_w=min(top_w, 48.0))
real = d.draw(real_cx, br_cy, *REAL, wrap_at=WRAP)
sim = d.draw(sim_cx, br_cy, *SIM, wrap_at=WRAP)
vis = d.draw(50.0, vis_cy, *VIS, wrap_at=WRAP, min_w=max(vis_w, 36.0))

d.head(real_cx, head_y, "(a)  Real hardware")
d.head(sim_cx, head_y, "(b)  Simulation \u2014 no hardware")

# 感知 -> 规划
d.arr(cam["cx"], cam["bot"], alg["cx"], alg["top"])

# 规划 -> 两支路：斜线避开分支名，落点在框顶两侧
d.arr(alg["cx"] - 3.0, alg["bot"], real_cx + real_w * 0.18, real["top"])
d.arr(alg["cx"] + 3.0, alg["bot"], sim_cx - sim_w * 0.18, sim["top"])
d.lab((alg["cx"] + real_cx) / 2 - 1.0, alg["bot"] - 4.6, "Arm_tx", ha="right")
d.lab((alg["cx"] + sim_cx) / 2 + 1.0, alg["bot"] - 4.6, "Arm_tx", ha="left")

# 回读：两条支路自外侧绕回，汇入算法框下沿同一条水平线
merge_y = alg["bot"] - 2.2
for cx, w, side in ((real_cx, real_w, -1), (sim_cx, sim_w, +1)):
    xr = cx + side * (w / 2 + 4.5)
    d.line([cx, xr], [br_cy, br_cy])                      # 从框中心高度引出
    d.line([xr, xr], [br_cy, merge_y])
    d.arr(xr, merge_y, alg["cx"] + side * 16.0, merge_y)
    d.lab(xr + side * 1.3, (br_cy + merge_y) / 2, "/Arm_rx",
          ha="left" if side > 0 else "right")

# 仿真支路 -> 状态发布
d.arr(sim["cx"], sim["bot"], vis["right"] - 5.0, vis["top"])
d.lab(sim["cx"] + 1.6, (sim["bot"] + vis["top"]) / 2, "/joint_states", ha="left")

# ==================================================================== 图题
lead = "Fig. 1."
body = ("Implemented message bus. Both conditions execute the same arm_control binaries; only the "
        "motor interface differs, which is what makes the simulated path a valid test of the real one.")
cy = 2.0
d.ax.text(0.8, cy, lead, ha="left", va="bottom", fontsize=7.4, fontfamily=SERIF,
          fontweight="bold")
x_off = 0.8 + d.tw(lead + "  ", 7.4, SERIF, weight="bold")
d.ax.text(x_off, cy, textwrap.fill(body, 100), ha="left", va="bottom",
          fontsize=7.4, fontfamily=SERIF, linespacing=1.6)

d.fig.savefig("docs/figures/architecture.png", dpi=300)
print(f"wrote docs/figures/architecture.png  支路合计 {span:.1f}，内容底部 {bottom:.1f}")
