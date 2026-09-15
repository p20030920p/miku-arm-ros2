#!/usr/bin/env python3
"""从演示用 RViz 配置派生出录制用配置。

区别只有一个：只保留 MotionPlanning 一个面板。RViz 的窗口最小尺寸由各面板的
sizeHint 决定，去掉 Displays 面板后窗口可以缩到 1400x900，3D 视口随之变宽，
录出来的 GIF 里机械臂才看得清。显示项本身不改动。

    python3 tools/make_record_config.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "src/miku_moveit_demo/rviz/moveit_demo.rviz")
DST = os.path.join(ROOT, "src/miku_moveit_demo/rviz/moveit_demo_record.rviz")

# 只留 Displays 面板：RViz 在没有 Displays 面板时会自己补一个空的出来，
# 反而更宽，所以它必须留着。
DISPLAYS_PANEL = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /MotionPlanning1
      Splitter Ratio: 0.5
    Tree Height: 120
"""

WINDOW = """Window Geometry:
  Height: 1250
  Width: 1900
"""

RECORD_WIDTH = 1900
RECORD_HEIGHT = 1250


def main():
    text = open(SRC, encoding="utf-8").read()

    # 面板段：从 "Panels:" 到 "Visualization Manager:" 之前
    new_text, n_panels = re.subn(
        r"^Panels:.*?(?=^Visualization Manager:)",
        DISPLAYS_PANEL,
        text,
        flags=re.S | re.M,
    )
    if n_panels != 1:
        sys.exit("没能替换 Panels 段，源配置结构变了")
    text = new_text

    # 窗口尺寸：让整窗落在 2520x1680 屏幕内
    text, n_win = re.subn(
        r"^Window Geometry:\n(?:  .*\n)*",
        WINDOW,
        text,
        flags=re.M,
    )
    if n_win != 1:
        sys.exit("没能替换 Window Geometry 段，源配置结构变了")

    # 断言：面板只剩一个，尺寸写对了
    kept = re.findall(r"^  - Class: rviz_common/(\w+)", text, flags=re.M)
    assert kept == ["Displays"], f"面板列表不对：{kept}"
    assert f"Width: {RECORD_WIDTH}" in text and f"Height: {RECORD_HEIGHT}" in text
    assert "Name: MotionPlanning\n" in text, "MotionPlanning 显示项丢了"

    with open(DST, "w", encoding="utf-8") as fh:
        fh.write(text)
    print(f"已写出 {os.path.relpath(DST, ROOT)}")
    print(f"  面板：{kept}")
    print(f"  窗口：{RECORD_WIDTH}x{RECORD_HEIGHT}")


if __name__ == "__main__":
    main()
