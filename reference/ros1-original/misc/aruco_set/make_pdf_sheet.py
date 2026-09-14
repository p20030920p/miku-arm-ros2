#!/usr/bin/env python3
"""Lay out the generated ArUco stickers onto A4 pages and export a PDF.

Input : 发给商家/<编号>_<尺寸>cm.svg  (already resized, with white margins)
Output: 发给商家/aruco贴纸排版_A4.pdf
        2 pages: page 1 = 24x 3cm stickers, page 2 = 8x 3cm + 32x 1cm stickers.
        Every sticker has its own dashed cut border; code labels are printed
        outside the borders (in margins/gaps), never on the sticker itself.

Pure stdlib, no third-party dependencies.
"""

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
OUTPUT_DIR = ROOT / "发给商家"
PDF_NAME = "aruco贴纸排版_A4.pdf"

MM2PT = 72.0 / 25.4
PAGE_W_MM = 210.0
PAGE_H_MM = 297.0


class Rect:
    __slots__ = ("x", "y", "w", "h", "fill")

    def __init__(self, x, y, w, h, fill):
        self.x, self.y, self.w, self.h, self.fill = x, y, w, h, fill


def parse_svg(path: Path):
    """Return (total_mm, [Rect, ...]) for one sticker SVG."""
    text = path.read_text(encoding="utf-8")
    m = re.search(r'width="([\d.]+)mm"', text)
    if not m:
        raise ValueError(f"no mm width in {path.name}")
    total_mm = float(m.group(1))

    group = re.search(
        r'<g transform="translate\(([\d.]+),([\d.]+)\)">(.*?)</g>',
        text,
        re.S,
    )
    if not group:
        raise ValueError(f"no marker group in {path.name}")
    tx, ty = float(group.group(1)), float(group.group(2))
    group_body = group.group(3)

    def attrs(tag):
        def val(name):
            mv = re.search(name + r'="([\d.]+)"', tag)
            return float(mv.group(1)) if mv else 0.0

        mf = re.search(r'fill="(white|black)"', tag)
        return val("x"), val("y"), val("width"), val("height"), mf.group(1)

    rects = []
    bg = re.search(r"<rect[^>]*></rect>", text)
    if bg:
        x, y, w, h, fill = attrs(bg.group(0))
        rects.append(Rect(x, y, w, h, fill))
    for tag in re.findall(r"<rect[^>]*></rect>", group_body):
        x, y, w, h, fill = attrs(tag)
        rects.append(Rect(x + tx, y + ty, w, h, fill))
    return total_mm, rects


def mm_to_pt(mx, my, w_mm=0.0, h_mm=0.0):
    px = mx * MM2PT
    py = (PAGE_H_MM - my - h_mm) * MM2PT
    return px, py, w_mm * MM2PT, h_mm * MM2PT


def draw_sticker(ops, rects, left, top, size_mm):
    """Draw one sticker: dashed cut border + white margin + pattern cells."""
    px, py, pw, ph = mm_to_pt(left, top, size_mm, size_mm)
    ops.append("0.55 0.55 0.55 RG 0.5 w [2.5 1.8] 0 d")
    ops.append(f"{px:.2f} {py:.2f} {pw:.2f} {ph:.2f} re S")
    ops.append("[] 0 d")
    unit = size_mm / 9.8  # one sticker canvas is 9.8 SVG units
    for r in rects:
        x_mm = left + r.x * unit
        y_mm = top + r.y * unit
        w_mm = r.w * unit
        h_mm = r.h * unit
        qx, qy, qw, qh = mm_to_pt(x_mm, y_mm, w_mm, h_mm)
        color = "0 0 0" if r.fill == "black" else "1 1 1"
        ops.append(f"{color} rg")
        ops.append(f"{qx:.2f} {qy:.2f} {qw:.2f} {qh:.2f} re f")


def add_label(ops, code, cx_mm, baseline_mm, font_pt):
    """Print a 3-digit code label centered at (cx_mm, baseline_mm), top-down."""
    text = f"{code:03d}"
    text_w_pt = 0.6 * font_pt * len(text)  # Helvetica-Bold digits ~0.6em
    lx = cx_mm * MM2PT - text_w_pt / 2.0
    ly = (PAGE_H_MM - baseline_mm) * MM2PT
    ops.append(f"BT /F1 {font_pt:.1f} Tf 0 g")
    ops.append(f"{lx:.2f} {ly:.2f} Td ({text}) Tj ET")


def layout_3cm(rect_sets, codes, cols, rows, gx, gy, label_font=12.0):
    """3cm stickers in a grid; each row is one code, labeled in the left margin."""
    ops = []
    cell, gap = 42.0, 1.0
    for i in range(len(codes) * 4):
        code = codes[i // 4]
        col, row = i % cols, i // cols
        x = gx + col * (cell + gap)
        y = gy + row * (cell + gap)
        draw_sticker(ops, rect_sets[code], x, y, cell)
        if col == 0:  # one row label per code, outside the stickers
            add_label(ops, code, gx / 2.0, y + cell / 2.0 + 1.4, label_font)
    return ops


def layout_1cm(rect_sets, codes, gx, gy):
    """1cm stickers in an 8x4 grid; each sticker labeled below its border."""
    ops = []
    sticker, hgap, strip, vgap = 14.0, 2.0, 3.5, 2.0
    col_pitch = sticker + hgap
    row_pitch = sticker + strip + vgap
    for i in range(len(codes) * 4):
        code = codes[i // 4]
        col, row = i % 8, i // 8
        x = gx + col * col_pitch
        y = gy + row * row_pitch
        draw_sticker(ops, rect_sets[code], x, y, sticker)
        add_label(ops, code, x + sticker / 2.0, y + sticker + 2.8, 9.0)
    return ops


def build_page(ops, page_index):
    """Return (page_obj, content_obj) as PDF object strings."""
    stream = "\n".join(ops) + "\n"
    content = f"<< /Length {len(stream.encode('latin-1'))} >>\nstream\n{stream}endstream"
    page = (
        "<< /Type /Page /Parent 2 0 R "
        "/MediaBox [0 0 595.28 841.89] "
        f"/Resources << /Font << /F1 9 0 R >> >> /Contents {page_index * 2 + 2} 0 R >>"
    )
    return page, content


def collect_stickers():
    """Return (stickers_by_size, rect_sets_by_size)."""
    stickers = {}
    rect_sets = {}
    for path in sorted(OUTPUT_DIR.glob("*_*cm.svg")):
        m = re.match(r"^(\d+)_(\d+)cm\.svg$", path.name)
        if not m:
            print(f"[跳过] 文件名不匹配: {path.name}")
            continue
        code = int(m.group(1))
        size_mm = int(m.group(2)) * 10
        try:
            total_mm, rects = parse_svg(path)
        except ValueError as exc:
            print(f"[跳过] {exc}")
            continue
        if total_mm != size_mm + 4 * (size_mm // 10):
            print(f"[跳过] 尺寸异常 {path.name}: total={total_mm}mm")
            continue
        stickers.setdefault(size_mm, []).append(code)
        rect_sets[code] = rects
    return stickers, rect_sets


def build_pdf(stickers, rect_sets):
    page_objs, content_objs = [], []

    def emit(ops, idx):
        page, content = build_page(ops, idx)
        page_objs.append(page)
        content_objs.append(content)

    codes_3cm = sorted(stickers.get(30, []))
    if codes_3cm:
        first = codes_3cm[:6]  # 121, 122, 131, 132, 221, 222
        gx = (PAGE_W_MM - (4 * 42.0 + 3 * 1.0)) / 2.0  # 19.5mm
        gy = (PAGE_H_MM - (6 * 42.0 + 5 * 1.0)) / 2.0  # 20.0mm
        ops = layout_3cm(rect_sets, first, 4, 6, gx, gy)
        emit(ops, 1)
        print(f"[页面1] 3cm: {len(first)} 个编号, 共 {len(first) * 4} 张贴纸 (4x6 网格)")

        rest = codes_3cm[6:]  # 231, 232
        ops = layout_3cm(rect_sets, rest, 4, 2, gx, 64.0)
        codes_1cm = sorted(stickers.get(10, []))
        if codes_1cm:
            gx1 = (PAGE_W_MM - (8 * 14.0 + 7 * 2.0)) / 2.0  # 42.0mm
            ops += layout_1cm(rect_sets, codes_1cm, gx1, 157.0)
            print(f"[页面2] 3cm: {len(rest)} 个编号, 共 {len(rest) * 4} 张贴纸; "
                  f"1cm: {len(codes_1cm)} 个编号, 共 {len(codes_1cm) * 4} 张贴纸 (8x4 网格)")
        else:
            print(f"[页面2] 3cm: {len(rest)} 个编号, 共 {len(rest) * 4} 张贴纸")
        emit(ops, 2)

    return page_objs, content_objs


def assemble_pdf(page_objs, content_objs):
    objects = []
    objects.append("<< /Type /Catalog /Pages 2 0 R >>")
    kids = " ".join(f"{3 + i * 2} 0 R" for i in range(len(page_objs)))
    objects.append(f"<< /Type /Pages /Kids [{kids}] /Count {len(page_objs)} >>")
    for i, (page, content) in enumerate(zip(page_objs, content_objs)):
        objects.append(page)  # page object
        objects.append(content)  # content stream object
    objects.append("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>")

    buf = bytearray(b"%PDF-1.4\n")
    offsets = [0]
    for i, body in enumerate(objects, start=1):
        offsets.append(len(buf))
        buf += f"{i} 0 obj\n{body}\nendobj\n".encode("latin-1")

    xref_pos = len(buf)
    buf += f"xref\n0 {len(objects) + 1}\n".encode("latin-1")
    buf += b"0000000000 65535 f \n"
    for off in offsets[1:]:
        buf += f"{off:010d} 00000 n \n".encode("latin-1")
    buf += (
        f"trailer\n<< /Size {len(objects) + 1} /Root 1 0 R >>\n"
        f"startxref\n{xref_pos}\n%%EOF\n"
    ).encode("latin-1")
    return bytes(buf)


def main() -> int:
    stickers, rect_sets = collect_stickers()
    if not stickers:
        print(f"[错误] {OUTPUT_DIR} 中没有可用的贴纸 SVG")
        return 1
    page_objs, content_objs = build_pdf(stickers, rect_sets)
    if not page_objs:
        print("[错误] 没有生成任何页面")
        return 1
    pdf = assemble_pdf(page_objs, content_objs)
    out_path = OUTPUT_DIR / PDF_NAME
    out_path.write_bytes(pdf)
    print(f"[完成] {out_path.name} 已生成 ({len(page_objs)} 页, {len(pdf)} 字节)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
