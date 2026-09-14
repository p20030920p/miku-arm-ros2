#!/usr/bin/env python3
"""
make_placeholder.py —— 生成"待补视频"占位图。

README 里给实物视频和 RViz2 录屏各留了一个位置。在你把真实素材放进去之前，
那里需要一个**不破图**、又不喧宾夺主的占位：安静的深色底 + 一行小字，
尺寸与 demo.gif 一致（760×480），保证版式不跳动。

放好真实素材后，这个文件就不再被引用，可以直接删掉。

用法：python3 tools/make_placeholder.py
"""

import pathlib

from PIL import Image, ImageDraw, ImageFont

REPO = pathlib.Path(__file__).resolve().parent.parent
OUT = REPO / "docs/figures"

BG = (14, 16, 20)
DIM = (110, 120, 133)
FAINT = (58, 64, 72)
SIZE = (760, 480)


# 用 TrueType 字体：PIL 的默认位图字体不含破折号等字形，会渲染成方框
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def placeholder(label, hint, path):
    im = Image.new("RGB", SIZE, BG)
    d = ImageDraw.Draw(im)
    f_label = ImageFont.truetype(FONT_BOLD, 17)
    f_hint = ImageFont.truetype(FONT_REG, 13)

    # 一圈细虚线框，暗示"这里将放入素材"
    x0, y0, x1, y1 = 16, 16, SIZE[0] - 16, SIZE[1] - 16
    dash, gap = 10, 7
    for x in range(x0, x1, dash + gap):
        d.line([(x, y0), (min(x + dash, x1), y0)], fill=FAINT, width=1)
        d.line([(x, y1), (min(x + dash, x1), y1)], fill=FAINT, width=1)
    for y in range(y0, y1, dash + gap):
        d.line([(x0, y), (x0, min(y + dash, y1))], fill=FAINT, width=1)
        d.line([(x1, y), (x1, min(y + dash, y1))], fill=FAINT, width=1)

    # 居中的播放符号（纯几何，不依赖字体图标）
    cx, cy = SIZE[0] // 2, SIZE[1] // 2 - 26
    r = 30
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=FAINT, width=2)
    d.polygon([(cx - 10, cy - 15), (cx - 10, cy + 15), (cx + 16, cy)],
              fill=FAINT)

    def center(text, y, fill, font):
        w = d.textlength(text, font=font)
        d.text((SIZE[0] / 2 - w / 2, y), text, fill=fill, font=font)

    center(label, cy + 46, DIM, f_label)
    center(hint, cy + 72, FAINT, f_hint)

    im.save(path)
    print(f"wrote {path}  ({path.stat().st_size/1024:.0f} KB)")


def main():
    placeholder("hardware demo — video pending",
                "drop docs/figures/hardware.gif; see docs/MEDIA.md",
                OUT / "hardware.gif")
    placeholder("RViz2 screen recording — pending",
                "drop docs/figures/rviz2.gif; see docs/MEDIA.md",
                OUT / "rviz2.gif")


if __name__ == "__main__":
    main()
