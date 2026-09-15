#!/usr/bin/env python3
"""从演示用 RViz 配置派生出录制用配置。

改动只有面板和窗口尺寸，显示项一个不动：

  · 去掉 Views 面板（录制时不需要调相机，省一列控件）
  · Displays 面板的树高从 220 压到 120，它只用来证明模型加载成功，不占地方
  · 窗口写死 1900x1250

Displays 面板不能删：RViz 在没有它的时候会自己补一个空的出来，反而更占宽度。

    python3 tools/make_record_config.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "src/miku_moveit_demo/rviz/moveit_demo.rviz")
DST = os.path.join(ROOT, "src/miku_moveit_demo/rviz/moveit_demo_record.rviz")

# 保留 Displays 面板（RViz 缺了它会自己补一个空的，更宽），但压扁
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
