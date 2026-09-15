#!/usr/bin/env python3
"""按窗口 ID 抓 X11 客户窗口，返回 PIL 图。

只处理本机 XWayland：根窗口 XGetImage 会被拒（BadMatch），客户窗口可以。
按标题找窗口不可靠——mutter 的框架窗口标题里也含同一串路径，
会抓到带标题栏的整块，裁剪坐标就全错了。所以这里直接吃窗口 ID。

    from xgrab import grab_window
    im = grab_window(0x2800106)
"""
import ctypes
import os
import ctypes.util
import sys

import numpy as np
from PIL import Image

_x11 = ctypes.CDLL(ctypes.util.find_library("X11"))
_x11.XOpenDisplay.restype = ctypes.c_void_p
_x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
_x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
_x11.XGetImage.restype = ctypes.c_void_p
_x11.XGetImage.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                           ctypes.c_int, ctypes.c_uint, ctypes.c_uint,
                           ctypes.c_ulong, ctypes.c_int]
_x11.XDestroyImage.argtypes = [ctypes.c_void_p]
_x11.XFree.argtypes = [ctypes.c_void_p]
_x11.XGetWindowAttributes.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                      ctypes.c_void_p]
_x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
_x11.XSetErrorHandler.argtypes = [ctypes.c_void_p]

ZPIXMAP = 2
ALL_PLANES = ctypes.c_ulong(-1).value


class _XWindowAttributes(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_int), ("y", ctypes.c_int),
        ("width", ctypes.c_int), ("height", ctypes.c_int),
        ("border_width", ctypes.c_int), ("depth", ctypes.c_int),
        ("visual", ctypes.c_void_p), ("root", ctypes.c_ulong),
        ("class_", ctypes.c_int), ("bit_gravity", ctypes.c_int),
        ("win_gravity", ctypes.c_int), ("backing_store", ctypes.c_int),
        ("backing_planes", ctypes.c_ulong), ("backing_pixel", ctypes.c_ulong),
        ("save_under", ctypes.c_int), ("colormap", ctypes.c_ulong),
        ("map_installed", ctypes.c_int), ("map_state", ctypes.c_int),
        ("all_event_masks", ctypes.c_long), ("your_event_mask", ctypes.c_long),
        ("do_not_propagate_mask", ctypes.c_long), ("override_redirect", ctypes.c_int),
        ("screen", ctypes.c_void_p),
    ]


class _XImage(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int), ("height", ctypes.c_int),
        ("xoffset", ctypes.c_int), ("format", ctypes.c_int),
        ("data", ctypes.c_void_p),
        ("byte_order", ctypes.c_int), ("bitmap_unit", ctypes.c_int),
        ("bitmap_bit_order", ctypes.c_int), ("bitmap_pad", ctypes.c_int),
        ("depth", ctypes.c_int), ("bytes_per_line", ctypes.c_int),
        ("bits_per_pixel", ctypes.c_int), ("red_mask", ctypes.c_ulong),
        ("green_mask", ctypes.c_ulong), ("blue_mask", ctypes.c_ulong),
    ]


_BAD = []


def _handler(_dpy, _ev):
    _BAD.append(1)
    return 0


_HANDLER = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)(_handler)


def grab_window(wid, display_name=b":0", box=None):
    """抓取窗口 pid 的客户区，返回 PIL.Image（RGB）。失败抛 RuntimeError。

    box=(x0, y0, x1, y1) 时只向 X 要这一块 —— 全窗抓一次约 380 ms，
    裁一块通常降到几十毫秒，录演示需要的帧率才够。
    """
    dpy = _x11.XOpenDisplay(display_name)
    if not dpy:
        raise RuntimeError(f"打不开 display {display_name}")
    _x11.XSetErrorHandler(ctypes.cast(_HANDLER, ctypes.c_void_p))
    try:
        attrs = _XWindowAttributes()
        if not _x11.XGetWindowAttributes(dpy, wid, ctypes.byref(attrs)):
            raise RuntimeError(f"窗口 0x{wid:x} 不存在")
        W, H = attrs.width, attrs.height
        if box:
            x0, y0, x1, y1 = (int(v) for v in box)
            x0, y0 = max(0, x0), max(0, y0)
            x1, y1 = min(W, x1), min(H, y1)
            if x1 <= x0 or y1 <= y0:
                raise RuntimeError(f"裁剪区域 {box} 超出窗口 {W}x{H}")
        else:
            x0 = y0 = 0
            x1, y1 = W, H
        px, py, w, h = x0, y0, x1 - x0, y1 - y0
        _BAD.clear()
        img_p = _x11.XGetImage(dpy, wid, px, py, w, h, ALL_PLANES, ZPIXMAP)
        if not img_p or _BAD:
            raise RuntimeError(f"XGetImage 失败（{w}x{h} 起点 {px},{py}）")
        img = ctypes.cast(img_p, ctypes.POINTER(_XImage)).contents
        bpl, bpp = img.bytes_per_line, img.bits_per_pixel // 8
        raw = ctypes.string_at(img.data, bpl * h)
        _x11.XDestroyImage(img_p)
    finally:
        _x11.XSync(dpy, 0)
        _x11.XCloseDisplay(dpy)

    # XWayland 上是 32 位 BGRA（小端）。逐像素写 Python 循环要 400 ms，
    # numpy 切片只要 8 ms，录演示帧率全靠这个。
    arr = np.frombuffer(raw, dtype=np.uint8)[: bpl * h].reshape(h, bpl)
    bgra = arr[:, : w * bpp].reshape(h, w, bpp)
    return Image.fromarray(bgra[:, :, [2, 1, 0]], "RGB")


def find_windows(min_size=300, display_name=":0"):
    """返回 [(wid, w, h, class_hint, title)]，含被重父化的客户窗口。"""
    import re
    import subprocess
    env = dict(os.environ, DISPLAY=display_name)
    out = subprocess.run(["xwininfo", "-tree", "-root"], capture_output=True,
                         text=True, env=env).stdout
    pat = re.compile(
        r'^\s*(0x[0-9a-f]+)\s+"([^"]*)":\s+\("([^"]*)"\s+"([^"]*)"\)\s+'
        r"(\d+)x(\d+)\+(-?\d+)\+(-?\d+)", re.M)
    rows = []
    for m in pat.finditer(out):
        wid, name, cls, _inst, w, h, _x, _y = m.groups()
        if int(w) >= min_size and int(h) >= min_size:
            rows.append((int(wid, 16), int(w), int(h), cls, name))
    return rows


if __name__ == "__main__":
    for wid, w, h, cls, name in find_windows(int(sys.argv[1]) if len(sys.argv) > 1 else 300):
        print(f"0x{wid:x}  {w:>5}x{h:<5}  [{cls}] {name[:70]}")
