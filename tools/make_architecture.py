#!/usr/bin/env python3
"""
make_architecture.py —— 生成 docs/figures/architecture.png

布局思路（这是这张图反复改过的地方）：
  旧版把 8 个同尺寸方框排在一条线上，没有任何主次，读者不知道从哪儿看起。
  现在改成"一条主线 + 两个互斥分支"：

      输入 ──▶ 控制器 ──┬──▶ (a) 实机
                        └──▶ (b) 仿真
                                 └──▶ 输出

  · 控制器框画得略重（浅底、粗边）——它是唯一贯穿两条路径的环节，也是视觉焦点
  · 两个分支框用同一支灰色，且只画一次"互斥"的连接，不做双向箭头堆叠
  · 不在图内写解释性文字；能被框图表达的就不写成句子

绘图规范：
  · 白底无填充、细黑实线；Times（Nimbus Roman）+ Helvetica（Nimbus Sans）
  · 框宽高由**实测文字**决定，位置由显式分列/分行推出，带越界断言
  · 画布刻意做成 8.6"×3.2"（约 2.7:1），README 缩到 800 px 宽时 11 pt 主名
    约 14 px，可读（旧版近正方形，缩完只剩 9 px）
"""

import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

SERIF, SANS = "Nimbus Roman", "Nimbus Sans"
plt.rcParams.update({
    "font.family": SERIF, "font.size": 8, "figure.facecolor": "white",
    "axes.facecolor": "white", "text.color": "black", "savefig.facecolor": "white",
})
FS_NAME, FS_SUB, FS_LAB, FS_CAP = 11.0, 9.0, 8.6, 9.0

BLACK = "#000000"
GREY = "#6b6b6b"
FOCUS_FC = "#eef1f5"      # 控制器框的浅底，用于建立视觉焦点
OPT_EC = "#444444"        # 分支框边色


class Canvas:
    def __init__(self, w_in, h_in):
        self.fig = plt.figure(figsize=(w_in, h_in), dpi=300)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, 100); self.ax.set_ylim(0, 100); self.ax.axis("off")
        self._r = self.fig.canvas.get_renderer()
        self._ux = 100.0 / (self.fig.get_dpi() * w_in)
        self._uy = 100.0 / (self.fig.get_dpi() * h_in)

    def tw(self, s, size, family=SANS, style="normal", weight="normal"):
        w = 0.0
        for ln in str(s).split("\n"):
            t = self.ax.text(0, -600, ln, fontsize=size, fontfamily=family,
                             fontstyle=style, fontweight=weight)
            w = max(w, t.get_window_extent(renderer=self._r).width * self._ux)
            t.remove()
        return w

    def th(self, s, size, family=SANS, style="normal", weight="normal"):
        t = self.ax.text(0, -600, s, fontsize=size, fontfamily=family,
                         fontstyle=style, fontweight=weight)
        h = t.get_window_extent(renderer=self._r).height * self._uy
        t.remove()
        return h

    def wrap(self, text, size, width_units):
        """按给定宽度折行，避免副标题把框撑宽"""
        lines, cur = [], ""
        for wd in str(text).split():
            trial = (cur + " " + wd).strip()
            if self.tw(trial, size, SERIF, style="italic") <= width_units or not cur:
                cur = trial
            else:
                lines.append(cur); cur = wd
        if cur:
            lines.append(cur)
        return lines

    def spec(self, name, sub=None, ns=FS_NAME, ss=FS_SUB, tag=None, wrap_at=24.0):
        lines = self.wrap(sub, ss, wrap_at) if sub else []
        w = self.tw(name, ns)
        if tag:
            w = max(w, self.tw(tag, FS_LAB, SANS))
        for ln in lines:
            w = max(w, self.tw(ln, ss, SERIF, style="italic"))
        w += 5.2
        hs = self.th("Agq", ss, SERIF, style="italic")
        h = self.th(name, ns, SANS) + len(lines) * hs * 1.45 + 5.0
        if tag:
            h += self.th(tag, FS_LAB, SANS) * 1.6
        return w, h

    def box(self, cx, cy, name, sub=None, fc="none", ec=BLACK, lw=1.1,
            min_w=0.0, ns=FS_NAME, ss=FS_SUB, radius=1.4, tag=None,
            wrap_at=24.0):
        w, h = self.spec(name, sub, ns, ss, tag, wrap_at)
        w = max(w, min_w)
        self.ax.add_patch(FancyBboxPatch(
            (cx - w / 2, cy - h / 2), w, h,
            boxstyle=f"round,pad=0,rounding_size={radius}",
            fc=fc, ec=ec, lw=lw, zorder=3))
        # 自下而上排布：先放最后一行副标题，再往上叠。
        # 之前用 va="top" 自上而下叠，文字会溢出框的下沿（检查器抓到过）。
        hn = self.th(name, ns, SANS)
        hs = self.th("Agq", ss, SERIF, style="italic")
        lines = self.wrap(sub, ss, wrap_at) if sub else []
        y = cy - h / 2 + 2.1                       # 底边内留白
        for ln in reversed(lines):
            self.ax.text(cx, y, ln, ha="center", va="bottom", fontsize=ss,
                         fontfamily=SERIF, fontstyle="italic", color=GREY,
                         zorder=4)
            y += hs * 1.42
        self.ax.text(cx, y, name, ha="center", va="bottom", fontsize=ns,
                     fontfamily=SANS, zorder=4)
        y += hn
        if tag:
            y += self.th(tag, FS_LAB, SANS) * 0.5
            self.ax.text(cx, y, tag, ha="center", va="bottom", fontsize=FS_LAB,
                         fontfamily=SANS, zorder=4, color="#333333")
        return dict(cx=cx, cy=cy, w=w, h=h, top=cy + h / 2, bot=cy - h / 2,
                    left=cx - w / 2, right=cx + w / 2)

    def arrow(self, x1, y1, x2, y2, color=BLACK, lw=1.1, ms=7):
        self.ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                          mutation_scale=ms, lw=lw, color=color,
                                          shrinkA=0, shrinkB=0, zorder=2))

    def elbow(self, pts, color=BLACK, lw=1.1, arrow=True):
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        self.ax.plot(xs[:-1], ys[:-1], color=color, lw=lw, zorder=2,
                     solid_capstyle="round")
        if arrow:
            self.arrow(xs[-2], ys[-2], xs[-1], ys[-1], color=color, lw=lw)

    def lab(self, x, y, s, size=FS_LAB, color=GREY, ha="center", va="center"):
        self.ax.text(x, y, s, fontsize=size, color=color, ha=ha, va=va,
                     fontfamily=SERIF, fontstyle="italic", zorder=5)

    def chip(self, x, y, s, size=FS_LAB):
        """分支名标签：小号无衬线 + 浅灰底，与框内文字区分"""
        self.ax.text(x, y, s, fontsize=size, fontfamily=SANS, ha="left",
                     va="center", zorder=5, color="#333333")


# ==================================================================== 版面
d = Canvas(8.6, 2.05)
ax = d.ax

CAM = ("deep_camera + aruco", "marker pose")
ALG = ("arm_control", "KDL FK/IK · planner · gravity · claw")
REAL = ("hardware", "serial::Serial · /dev/ttyACM0")
SIM = ("sim_motor_board", "simulated motors")
OUT = ("robot_state_publisher\n→ TF → RViz2", "/joint_states @ 100 Hz")

MARGIN_L, MARGIN_R = 4.0, 4.0
COL_GAP = 8.0
ROW_GAP = 6.5

cam_s = d.spec(*CAM)
alg_s = d.spec(*ALG)
col1_w = max(cam_s[0], alg_s[0])
col2_w = max(d.spec(*REAL, tag="(a)  real hardware")[0],
             d.spec(*SIM, tag="(b)  simulation")[0])
col3_w = d.spec(*OUT)[0]

span = col1_w + COL_GAP + col2_w + COL_GAP + col3_w
print(f"横向：col1={col1_w:.1f} col2={col2_w:.1f} col3={col3_w:.1f} 合计={span:.1f}/92")
assert span <= 100 - MARGIN_L - MARGIN_R, f"横向超宽 {span:.1f}"

x1 = MARGIN_L + col1_w / 2
x2 = MARGIN_L + col1_w + COL_GAP + col2_w / 2
x3 = MARGIN_L + col1_w + COL_GAP + col2_w + COL_GAP + col3_w / 2

CAP_BAND = 13.0
TOP = 96.0
row_h = max(cam_s[1], alg_s[1],
            d.spec(*REAL, tag="(a)  real hardware")[1],
            d.spec(*SIM, tag="(b)  simulation")[1])
y_top = TOP - row_h / 2
y_bot = y_top - row_h - ROW_GAP
assert y_bot - row_h / 2 >= CAP_BAND, f"纵向不足：底部 {y_bot-row_h/2:.1f} < {CAP_BAND}"

# ==================================================================== 绘制
cam = d.box(x1, y_top, *CAM)
alg = d.box(x1, y_bot, *ALG, fc=FOCUS_FC, ec=BLACK, lw=1.9, min_w=col1_w)
real = d.box(x2, y_top, *REAL, ec=OPT_EC, tag="(a)  real hardware")
sim = d.box(x2, y_bot, *SIM, ec=OPT_EC, tag="(b)  simulation")
out = d.box(x3, (y_top + y_bot) / 2, *OUT, ec=OPT_EC, min_w=col3_w)

# 分支名（贴在各自框左上，避免与连线争位置）
# ---------------- 连线 ----------------
d.arrow(cam["cx"] + 5.0, cam["bot"], alg["cx"] + 5.0, alg["top"])

# 控制器 -> 两个分支：共用一段竖线再分叉
fork_x = (alg["right"] + real["left"]) / 2
d.elbow([(alg["right"], alg["cy"]), (fork_x, alg["cy"]),
         (fork_x, real["cy"]), (real["left"], real["cy"])])
d.elbow([(fork_x, alg["cy"]), (fork_x, sim["cy"]), (sim["left"], sim["cy"])])
# 仿真分支 -> 输出
d.elbow([(sim["right"], sim["cy"]), (x3, sim["cy"]), (x3, out["bot"])])
d.lab((sim["right"] + x3) / 2, sim["cy"] - 3.2, "/joint_states", size=7.6)

# ==================================================================== 图题
lead = "Fig. 1."
body = ("Both branches run the same arm_control binaries; only the motor interface differs. The "
        "simulated branch reproduces the board's 50 B / 46 B framing over a socat PTY, so the "
        "hardware binary runs against it unmodified.")
cy = 2.6
x_off = 1.0 + d.tw(lead + "  ", FS_CAP, SERIF, weight="bold")
avail = 99.0 - x_off
words, lines, cur = body.split(), [], ""
for wd in words:
    trial = (cur + " " + wd).strip()
    if d.tw(trial, FS_CAP, SERIF) <= avail or not cur:
        cur = trial
    else:
        lines.append(cur); cur = wd
if cur:
    lines.append(cur)
lh = d.th("Agq", FS_CAP, SERIF) * 1.5
n = len(lines)
for i, ln in enumerate(lines):
    d.ax.text(x_off, cy + (n - 1 - i) * lh, ln, ha="left", va="bottom",
              fontsize=FS_CAP, fontfamily=SERIF)
d.ax.text(1.0, cy + (n - 1) * lh, lead, ha="left", va="bottom", fontsize=FS_CAP,
          fontfamily=SERIF, fontweight="bold")

d.fig.savefig("docs/figures/architecture.png", dpi=300)
print("wrote docs/figures/architecture.png")
