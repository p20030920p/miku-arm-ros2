#!/usr/bin/env python3
"""
make_figure_from_video.py —— 把**实物录像**转成仓库用的动图/静帧。

用途：README 里给实物演示留了位置（docs/figures/hardware.gif）。你手机拍完、
或桌面录屏拿到 mp4/mov 之后，用这个脚本一步转成合适的尺寸与体积，不必依赖
ffmpeg（本机没有 ffmpeg，走 OpenCV + Pillow）。

用法
----
# 1) 转成动图，自动缩到 760 px 宽、12 fps，起始 6 秒
python3 tools/make_figure_from_video.py 实拍.mp4 docs/figures/hardware.gif

# 2) 指定时间区间与参数（秒）
python3 tools/make_figure_from_video.py 实拍.mp4 docs/figures/hardware.gif \
        --start 12 --duration 8 --width 760 --fps 12 --colors 96

# 3) 顺便导出一张静帧当海报图（README 折叠时用）
python3 tools/make_figure_from_video.py 实拍.mp4 docs/figures/hardware_poster.png \
        --poster --at 14

# 4) 只做压缩，不改内容（转成 h264 mp4，便于塞进仓库）
python3 tools/make_figure_from_video.py 实拍.mov docs/media/hardware.mp4 --mp4

支持的输入：mp4 / mov / avi / mkv / m4v，以及手机常见的 HEVC（取决于本机 OpenCV
的解码器，若失败先用系统工具转成 h264 mp4）。
"""

import argparse
import pathlib
import sys

import cv2
import numpy as np
from PIL import Image


def open_video(path):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        sys.exit(f"打不开视频：{path}\n"
                 f"（若为 HEVC/H.265，请先转成 h264 mp4 再试）")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cap, fps, total, w, h


def grab_frames(cap, fps, start, duration, out_fps, width):
    """按时间区间取帧，并按 out_fps 抽帧、按 width 缩放"""
    cap.set(cv2.CAP_PROP_POS_MSEC, start * 1000.0)
    step = max(1, int(round(fps / max(out_fps, 0.1))))
    frames, idx, kept = [], 0, 0
    end_frame = None if duration is None else int(round((start + duration) * fps))
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if end_frame is not None and idx >= end_frame:
            break
        if idx % step == 0:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(rgb)
            if width and im.width != width:
                im = im.resize((width, max(1, round(im.height * width / im.width))),
                               Image.LANCZOS)
            frames.append(im)
            kept += 1
        idx += 1
    return frames


def write_gif(frames, out, fps, colors):
    if not frames:
        sys.exit("没有取到任何帧，检查 --start/--duration 是否超出视频长度")
    quant = [f.convert("P", palette=Image.ADAPTIVE, colors=colors) for f in frames]
    quant[0].save(out, save_all=True, append_images=quant[1:],
                  duration=int(1000 / max(fps, 0.1)), loop=0, optimize=True)


def write_mp4(frames, out, fps):
    if not frames:
        sys.exit("没有取到任何帧")
    w, h = frames[0].size
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for im in frames:
        vw.write(cv2.cvtColor(np.asarray(im.convert("RGB")), cv2.COLOR_RGB2BGR))
    vw.release()


def main():
    ap = argparse.ArgumentParser(description="实物录像 → 仓库用动图/静帧")
    ap.add_argument("input", help="输入视频")
    ap.add_argument("output", help="输出 .gif / .png / .mp4")
    ap.add_argument("--start", type=float, default=0.0, help="起始时间（秒）")
    ap.add_argument("--duration", type=float, default=None, help="时长（秒），默认到结尾")
    ap.add_argument("--width", type=int, default=760, help="输出宽度（px）")
    ap.add_argument("--fps", type=float, default=12.0, help="输出帧率")
    ap.add_argument("--colors", type=int, default=96, help="GIF 调色板颜色数（越小体积越小）")
    ap.add_argument("--poster", action="store_true", help="只导出一张静帧 PNG")
    ap.add_argument("--at", type=float, default=None, help="静帧取第几秒（默认取中点）")
    ap.add_argument("--mp4", action="store_true", help="输出 mp4 而不是 gif")
    args = ap.parse_args()

    inp = pathlib.Path(args.input)
    if not inp.exists():
        sys.exit(f"找不到输入文件：{inp}")
    out = pathlib.Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    cap, fps, total, w, h = open_video(inp)
    dur_total = total / fps if fps else 0
    print(f"输入 {inp.name}：{w}×{h}，{fps:.1f} fps，{total} 帧（约 {dur_total:.1f} s）")

    if args.poster:
        at = args.at if args.at is not None else dur_total / 2
        cap.set(cv2.CAP_PROP_POS_MSEC, at * 1000.0)
        ok, frame = cap.read()
        cap.release()
        if not ok:
            sys.exit(f"取不到第 {at:.1f} s 的帧")
        im = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if args.width and im.width != args.width:
            im = im.resize((args.width, round(im.height * args.width / im.width)),
                           Image.LANCZOS)
        im.save(out)
        print(f"已写出静帧 {out}（{out.stat().st_size/1024:.0f} KB，取自 {at:.1f} s）")
        return

    frames = grab_frames(cap, fps, args.start, args.duration, args.fps, args.width)
    cap.release()
    if args.mp4 or out.suffix.lower() == ".mp4":
        write_mp4(frames, out, args.fps)
    else:
        write_gif(frames, out, args.fps, args.colors)
    print(f"已写出 {out}：{len(frames)} 帧，{out.stat().st_size/1024:.0f} KB")
    if not args.mp4 and out.stat().st_size > 4 * 1024 * 1024:
        print("提示：超过 4 MB，建议调小 --width / --fps / --colors 或缩短 --duration")


if __name__ == "__main__":
    main()
