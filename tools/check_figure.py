#!/usr/bin/env python3
"""
check_figure.py —— 架构图的几何自检（防止图版回归）

绘图过程中反复出现过"框压框、文字溢出、标注压在线上"这类问题，在缩略图里
很难用肉眼发现，因此把检查固化成脚本。

它在**子进程**里运行 tools/make_architecture.py（本仓库的图由它生成），
让脚本把每个文本元素与每个内容框的包围盒以 JSON 写到 stdout，再逐项核对：

  1. 文本不得越出画布
  2. 文本不得跨越框线（允许 1 单位字体度量容差）
  3. 任意两个文本框不得重叠

之所以走子进程：matplotlib 在同一进程内二次构建图形时，文字度量会出现漂移，
导致复检结果与真实出图不一致（本脚本早期版本正是因此误报）。

用法：
    python3 tools/check_figure.py           # 退出码 0 = 通过
"""

import json
import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
GEN = REPO / "tools" / "make_architecture.py"
TOL = 1.0

PROBE = r'''
import json, sys, runpy
# 用 runpy 而不是 importlib：后者的导入机制会改变 matplotlib 的字体解析，
# 实测文字度量与直接运行不一致（96.3 vs 112.9），会得出错误结论。
mod = runpy.run_path(%r, run_name="__main__")
plt = mod["plt"]
fig = plt.gcf()
ax = fig.axes[0]
r = fig.canvas.get_renderer()
inv = ax.transData.inverted()

def d(x, y):
    return inv.transform((x, y))

texts = []
for t in ax.texts:
    bb = t.get_window_extent(renderer=r)
    (x0, y0), (x1, y1) = d(bb.x0, bb.y0), d(bb.x1, bb.y1)
    texts.append({"s": t.get_text(),
                  "b": [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]})

from matplotlib.patches import Rectangle
rects = []
for p in ax.patches:
    if isinstance(p, Rectangle) and p.get_width() < 90 and p.get_height() < 90:
        x, y = p.get_xy()
        rects.append([x, y, x + p.get_width(), y + p.get_height()])

json.dump({"texts": texts, "rects": rects}, sys.stdout)
''' % str(GEN)


def main():
    out = subprocess.run([sys.executable, "-c", PROBE], capture_output=True,
                         text=True, cwd=str(REPO))
    if out.returncode != 0:
        print("绘图脚本执行失败：")
        print(out.stderr[-1500:])
        return 1
    # 生成图的同时也会打印自己的预算信息，只取最后一行 JSON
    line = [l for l in out.stdout.splitlines() if l.startswith("{")][-1]
    data = json.loads(line)
    texts, rects = data["texts"], data["rects"]

    problems = []
    for t in texts:
        s = (t["s"][:36] or "<empty>")
        x0, y0, x1, y1 = t["b"]
        if x0 < -TOL or x1 > 100 + TOL or y0 < -TOL or y1 > 100 + TOL:
            problems.append(f"越出画布: {s!r}  x[{x0:.1f},{x1:.1f}] y[{y0:.1f},{y1:.1f}]")
            continue
        for rx0, ry0, rx1, ry1 in rects:
            if x1 <= rx0 + TOL or x0 >= rx1 - TOL:
                continue
            if y1 <= ry0 + TOL or y0 >= ry1 - TOL:
                continue
            inside = (x0 >= rx0 - TOL and x1 <= rx1 + TOL and
                      y0 >= ry0 - TOL and y1 <= ry1 + TOL)
            if not inside:
                problems.append(f"跨越框线: {s!r}  框=({rx0:.1f},{ry0:.1f},"
                                f"{rx1:.1f},{ry1:.1f})")
    for i in range(len(texts)):
        for j in range(i + 1, len(texts)):
            a, b = texts[i]["b"], texts[j]["b"]
            if (min(a[2], b[2]) - max(a[0], b[0]) > TOL and
                    min(a[3], b[3]) - max(a[1], b[1]) > TOL):
                problems.append(f"文本重叠: {texts[i]['s'][:22]!r} × "
                                f"{texts[j]['s'][:22]!r}")

    print(f"检查 {len(texts)} 个文本元素 / {len(rects)} 个内容框")
    if problems:
        print(f"\n发现 {len(problems)} 处问题：")
        for p in problems:
            print("  ✗", p)
        return 1
    print("✓ 无越界、无压线、无重叠")
    return 0


if __name__ == "__main__":
    sys.exit(main())
