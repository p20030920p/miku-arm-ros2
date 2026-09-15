#!/usr/bin/env python3
"""列出窗口树里所有大于给定尺寸的窗口（含被 mutter 重父化的客户窗口）。

xwininfo -root -children 只看根的直接子窗口，mutter 会把客户窗口塞进
frame 里，所以那里找不到 RViz。直接解析 xwininfo -tree -root 的输出。
"""
import re
import subprocess
import sys

MIN = int(sys.argv[1]) if len(sys.argv) > 1 else 400

out = subprocess.run(["xwininfo", "-tree", "-root"],
                     capture_output=True, text=True).stdout

pat = re.compile(
    r'^\s*(0x[0-9a-f]+)\s+"([^"]*)":\s+\("([^"]*)"\s+"([^"]*)"\)\s+'
    r"(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)",
    re.M)

rows = []
for m in pat.finditer(out):
    wid, name, cls, inst, w, h, x, y, ax, ay = m.groups()
    w, h, ax, ay = int(w), int(h), int(ax), int(ay)
    if w >= MIN and h >= MIN:
        rows.append((wid, w, h, ax, ay, cls, name))

for wid, w, h, ax, ay, cls, name in rows:
    print(f"{wid}  {w:>5}x{h:<5} at {ax:>5},{ay:<5}  [{cls}] {name[:70]}")
if not rows:
    print(f"没有 >= {MIN}px 的窗口")
