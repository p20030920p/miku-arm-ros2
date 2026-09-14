#!/usr/bin/env python3
"""
make_architecture.py —— 生成 docs/figures/architecture.png

按 IEEE / RA-L 论文中系统框图的惯例绘制：

  · 白底、无填充、细黑实线框；层次由线型与字重表达
  · Times（Nimbus Roman）正文 + Helvetica（Nimbus Sans）标签 —— IEEE 字体搭配
  · **扁平版面**：框内每行最多 2 行文字，细节搬进图题
  · 先测量后布局：框宽高由实测文字决定，带越界断言

版面尺寸的理由（这是踩过坑的地方）：
  README 正文栏宽约 800 px，图会被等比缩到该宽度显示。若图又高又窄
  （如 6.6"×5.6"），8 pt 的字缩完后只剩约 9 px，必然看不清。
  因此这里刻意做成 8.0"×3.3" 的扁幅（约 2.4:1）：
      · 显示宽 800 px 时，8.2 pt 主名约 30 px 高 —— 清晰
      · 框内一律单行副标题，框高统一，版面紧凑
      · 协议常量（0x86C1、50 B/46 B、×1000）等细节移到图题，不挤在图里

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
LW_BOX, LW_ARR, LW_FB = 1.0, 0.8, 0.65
GREY = "0.30"
FS_NAME, FS_SUB, FS_LAB, FS_HEAD, FS_CAP = 11.0, 9.2, 9.0, 10.5, 9.2


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

    # ---------------- 框 ----------------
    def spec(self, name, sub=None, ns=FS_NAME, ss=FS_SUB):
        w = self.tw(name, ns)
        if sub:
            w = max(w, self.tw(sub, ss, SERIF, style="italic"))
        w += 5.4
        hn, hs = self.th(name, ns, SANS), self.th("Agq", ss, SERIF, style="italic")
        h = hn + (hs * 1.45 + 1.6 if sub else 0.0) + 5.0
        return w, h

    def box(self, cx, cy, name, sub=None, min_w=0.0, ns=FS_NAME, ss=FS_SUB):
        w, h = self.spec(name, sub, ns, ss)
        w = max(w, min_w)
        self.ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fill=False,
                                    ec="black", lw=LW_BOX, zorder=3))
        if sub:
            y = cy + h / 2 - 2.4
            self.ax.text(cx, y, name, ha="center", va="top", fontsize=ns,
                         fontfamily=SANS, zorder=4)
            self.ax.text(cx, y - self.th(name, ns, SANS) - 1.8, sub, ha="center",
                         va="top", fontsize=ss, fontfamily=SERIF,
                         fontstyle="italic", color=GREY, zorder=4)
        else:
            self.ax.text(cx, cy, name, ha="center", va="center", fontsize=ns,
                         fontfamily=SANS, zorder=4)
        return dict(cx=cx, cy=cy, w=w, h=h, top=cy + h / 2, bot=cy - h / 2,
                    left=cx - w / 2, right=cx + w / 2)

    # ---------------- 图元 ----------------
    def arr(self, x1, y1, x2, y2, color="black", lw=LW_ARR, ms=6):
        self.ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>",
                                          mutation_scale=ms, lw=lw, color=color,
                                          shrinkA=0, shrinkB=0, zorder=2))

    def line(self, xs, ys, color=GREY, lw=LW_FB):
        self.ax.plot(xs, ys, color=color, lw=lw, zorder=2)

    def lab(self, x, y, s, size=FS_LAB, color=GREY, ha="center", va="center"):
        self.ax.text(x, y, s, fontsize=size, color=color, ha=ha, va=va,
                     fontfamily=SERIF, fontstyle="italic", zorder=5)

    def head(self, x, y, s, size=FS_HEAD):
        self.ax.text(x, y, s, fontsize=size, fontfamily=SANS, ha="center",
                     va="center", zorder=5)


# ==================================================================== 版面
d = Canvas(8.6, 2.9)

CAM = ("deep_camera + aruco", "ArUco pose (solvePnP)")
ALG = ("arm_control", "KDL FK/IK · planner · gravity · claw")
REAL = ("hardware", "serial::Serial  ·  /dev/ttyACM0")
SIM = ("sim_motor_board", "simulated motors  ·  no serial")
VIS = ("robot_state_publisher \u2192 TF \u2192 RViz2", "/joint_states @ 100 Hz")

PAD_L, PAD_R, GPX, GPY = 12.5, 2.5, 5.0, 7.5   # 左边距须容纳 '/Arm_rx' 整串
specs = {k: d.spec(n, s) for k, (n, s) in
         dict(CAM=CAM, ALG=ALG, REAL=REAL, SIM=SIM, VIS=VIS).items()}

# 横向：三个位置 —— 左（感知+规划）、中（两支路）、右（发布）
left_w = max(specs["CAM"][0], specs["ALG"][0])
mid_w = max(specs["REAL"][0], specs["SIM"][0])
right_w = min(specs["VIS"][0], 23.0)   # 限宽，保证总和留有余量

C1 = PAD_L + left_w / 2
mid_l = PAD_L + left_w + GPX
C2 = mid_l + mid_w / 2
right_l = mid_l + mid_w + GPX
C3 = right_l + right_w / 2
span_r = C3 + right_w / 2
assert span_r <= 100 - PAD_R, f"横向超宽：{span_r:.1f} > {100-PAD_R}"

# 纵向：三行
row_h = max(specs[k][1] for k in specs)
CAP_BAND = 11.0
top = 96.0
y1 = top - row_h / 2                     # 第 1 行：感知 / 规划 / 发布
y2 = y1 - row_h / 2 - GPY - row_h / 2    # 第 2 行：真实硬件
y3 = y2 - row_h / 2 - GPY - row_h / 2    # 第 3 行：仿真
assert y3 - row_h / 2 >= CAP_BAND, f"纵向不足：底部 {y3-row_h/2:.1f}"

# ==================================================================== 绘制
cam = d.box(C1, y1, *CAM)
alg = d.box(C1, y2, *ALG, min_w=left_w)
vis = d.box(C3, y1, *VIS, min_w=right_w)
real = d.box(C2, y2, *REAL, min_w=mid_w)
sim = d.box(C2, y3, *SIM, min_w=mid_w)

d.head(C2, (y2 + y3) / 2 + row_h / 2 + GPY * 0.45, "")   # 占位，保持对称
d.ax.text(real["cx"], real["top"] + GPY * 0.42, "(a)  Real hardware",
          fontsize=FS_HEAD, fontfamily=SANS, ha="center", va="center", zorder=5)
d.ax.text(sim["cx"], sim["bot"] - GPY * 0.42, "(b)  Simulation \u2014 no hardware",
          fontsize=FS_HEAD, fontfamily=SANS, ha="center", va="center", zorder=5)

# 感知 -> 规划
d.arr(cam["cx"], cam["bot"], alg["cx"], alg["top"])

# 规划 -> 两支路
# 两条 Arm_tx 分别标注在各自箭头的上/下侧，避免两个标签落在同一点
d.arr(alg["right"], alg["cy"], real["left"] - 0.6, real["cy"])
d.arr(alg["right"], alg["cy"] - 3.4, sim["left"] - 0.6, sim["cy"])
d.lab((alg["right"] + real["left"]) / 2, real["cy"] + 3.0, "Arm_tx")
d.lab((alg["right"] + sim["left"]) / 2, sim["cy"] - 3.4, "Arm_tx")

# 回读：两条支路自右侧绕回规划（左列外侧）
fb_x = alg["left"] - GPX * 0.55
for b in (real, sim):
    d.line([b["left"], fb_x], [b["cy"], b["cy"]])
    d.lab(fb_x - 0.8, b["cy"] + 2.6, "/Arm_rx", ha="right")
d.line([fb_x, fb_x], [real["cy"], sim["cy"]])
d.arr(fb_x, alg["cy"], alg["left"] - 0.4, alg["cy"])

# 仿真支路 -> 状态发布
sim_r = sim["right"]
d.line([sim_r, sim_r + (right_l - sim_r) * 0.45], [sim["cy"], sim["cy"]])
d.arr(sim_r + (right_l - sim_r) * 0.45, sim["cy"], vis["left"] - 0.5, vis["cy"] - 2.0)
d.lab(sim_r + 3.2, sim["cy"] + 4.2, "/joint_states", ha="left")

# ==================================================================== 图题
lead = "Fig. 1."
body = ("Implemented message bus. Both conditions run the same arm_control binaries; only the "
        "motor interface differs. The simulated branch reproduces the real board's 50 B / 46 B "
        "framing byte for byte over a socat PTY, so the hardware binary runs against it unmodified.")
cy = 2.2
x_off = 0.8 + d.tw(lead + "  ", FS_CAP, SERIF, weight="bold")
# 逐行绘制：首行从左边线起（"Fig. 1." 占位），续行缩进到正文起点 —— IEEE 的悬挂缩进
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
lh = d.th("Agq", FS_CAP, SERIF) * 1.55
n = len(lines)
# lines[0] 是首行，应位于**最上方**：y 随行号递减。
# 首行从左边线开始（"Fig. 1." 占位），续行缩进到正文起点。
for i, ln in enumerate(lines):
    y = cy + (n - 1 - i) * lh
    d.ax.text(x_off, y, ln, ha="left", va="bottom",
              fontsize=FS_CAP, fontfamily=SERIF)
d.ax.text(0.8, cy + (n - 1) * lh, lead, ha="left", va="bottom",
          fontsize=FS_CAP, fontfamily=SERIF, fontweight="bold")

d.fig.savefig("docs/figures/architecture.png", dpi=300)
print(f"wrote docs/figures/architecture.png  "
      f"预算：横 {span_r:.1f}/98u，底 {y3-row_h/2:.1f}/{CAP_BAND}u")
