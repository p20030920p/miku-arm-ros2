#!/usr/bin/env python3
"""Resize downloaded ArUco SVGs to the sizes specified in aruco标定码分布.md
and add a white margin around each marker.

The nominal size (1cm / 3cm) is the TOTAL sticker size INCLUDING the white
margin: a 1cm sticker is 10mm total (2mm white margin per side), a 3cm
sticker is 30mm total (6mm white margin per side).

Input : aruco/*.svg  (7x7 cell grid, currently 100mm x 100mm)
Output: 发给商家/<编号>_<尺寸>cm.svg
"""

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
INPUT_DIR = ROOT / "aruco"
OUTPUT_DIR = ROOT / "发给商家"

# Specs from aruco标定码分布.md: white margin per marker size.
MARGIN_MM = {10: 2.0, 30: 6.0}


def size_mm_for_code(code: int) -> int | None:
    """Return the marker size in mm derived from the code, or None if unknown."""
    if 1 <= code <= 4:  # server cabinets -> 1cm
        return 10
    if 100 <= code <= 999:
        last_digit = code % 10
        if last_digit in (1, 2):  # HDD wide face -> 3cm
            return 30
        if last_digit == 3:  # HDD narrow face -> 1cm
            return 10
    return None


def fmt_number(value: float) -> str:
    """Format a float without trailing zeros (e.g. 9.8, 1.4)."""
    return f"{value:.10f}".rstrip("0").rstrip(".")


def process_svg(src: Path) -> str | None:
    """Return the resized SVG text for one source file, or None if skipped."""
    match = re.search(r"aruco-(\d+)\.svg$", src.name)
    if not match:
        print(f"[跳过] 文件名不匹配: {src.name}")
        return None
    code = int(match.group(1))

    size_mm = size_mm_for_code(code)
    if size_mm is None:
        print(f"[跳过] 无法从编号 {code} 推导尺寸: {src.name}")
        return None
    margin_mm = MARGIN_MM[size_mm]

    content = src.read_text(encoding="utf-8")
    tag_match = re.search(r"<svg[^>]*>", content)
    if not tag_match:
        print(f"[跳过] 未找到 <svg> 标签: {src.name}")
        return None
    svg_tag = tag_match.group(0)
    if 'viewBox="0 0 7 7"' not in svg_tag:
        print(f"[跳过] 非预期的 viewBox: {src.name}")
        return None

    # Nominal size includes the white margin: pattern = total - 2 * margin.
    pattern_mm = size_mm - 2.0 * margin_mm            # 6mm / 18mm
    # Convert the margin from mm to grid units: 7 cells == pattern size.
    margin_cells = margin_mm * 7.0 / pattern_mm       # 7/3 for both specs
    total_cells = 7.0 + 2.0 * margin_cells            # 35/3
    total_mm = size_mm                                # 10mm / 30mm

    new_tag = (
        f'<svg viewBox="0 0 {fmt_number(total_cells)} {fmt_number(total_cells)}" '
        f'xmlns="http://www.w3.org/2000/svg" shape-rendering="crispEdges" '
        f'width="{fmt_number(total_mm)}mm" height="{fmt_number(total_mm)}mm">'
    )

    body = (
        f'<rect width="{fmt_number(total_cells)}" height="{fmt_number(total_cells)}" '
        f'fill="white"></rect>'
        f'<g transform="translate({fmt_number(margin_cells)},{fmt_number(margin_cells)})">'
    )

    new_content = content.replace(svg_tag, new_tag + body, 1)
    if not new_content.rstrip().endswith("</svg>"):
        print(f"[跳过] 未找到 </svg> 结束标签: {src.name}")
        return None
    new_content = new_content[: new_content.rfind("</svg>")] + "</g></svg>"
    return new_content


def main() -> int:
    OUTPUT_DIR.mkdir(exist_ok=True)
    sources = sorted(INPUT_DIR.glob("aruco-*.svg"))
    if not sources:
        print(f"[错误] {INPUT_DIR} 中没有找到 aruco-*.svg 文件")
        return 1

    generated = 0
    for src in sources:
        match = re.search(r"aruco-(\d+)\.svg$", src.name)
        code = int(match.group(1))
        new_content = process_svg(src)
        if new_content is None:
            continue

        size_mm = size_mm_for_code(code)
        out_name = f"{code:03d}_{size_mm // 10}cm.svg"
        out_path = OUTPUT_DIR / out_name
        out_path.write_text(new_content, encoding="utf-8")
        generated += 1
        print(f"[生成] {src.name} -> {out_path.name}")

    print(f"完成：共生成 {generated} 个文件到 {OUTPUT_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
