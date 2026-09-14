# Media

Two places in the README are reserved for recordings that only exist on real hardware or on a
machine with a working display. Until then they hold a neutral placeholder, so the page never shows
a broken image and the layout never shifts.

| Slot | File | What belongs there |
|---|---|---|
| Hardware demo | `figures/hardware.gif` | the real arm moving — a phone clip, or a screen capture of the run |
| RViz2 recording | `figures/rviz2.gif` | RViz2 with the arm live, TF updating |

Replace a slot by overwriting the file. Nothing else changes: both are referenced at a fixed
`width="760"`, so the layout stays put and the placeholder stops being used.

---

## Converting a recording

`ffmpeg` is not available on this machine, so the converter uses OpenCV + Pillow, both already
installed. It takes any format OpenCV can decode (mp4, mov, avi, mkv, m4v).

```bash
# keep 8 seconds starting at 0:12, scale to 760 px, 12 fps
python3 tools/make_figure_from_video.py 实拍.mp4 docs/figures/hardware.gif \
        --start 12 --duration 8 --width 760 --fps 12

# a still frame, for a poster or a docs figure
python3 tools/make_figure_from_video.py 实拍.mp4 docs/figures/hardware_poster.png \
        --poster --at 14

# just recompress to h264 mp4 for the repo
python3 tools/make_figure_from_video.py 实拍.mov docs/media/hardware.mp4 --mp4
```

Useful flags: `--colors` (GIF palette size; lower is smaller), `--duration` (defaults to the end of
the file). The script warns when the result exceeds 4 MB.

### Before you commit

Keep the GIF under about 3 MB. GitHub serves it as-is, and a 10 MB autoplay GIF is unpleasant on a
laptop and unusable on a phone. If it is too big, in this order:

1. `--duration` down to 6–8 s. A short loop that shows the motion beats a long one.
2. `--width 640` — still readable in the README's content column.
3. `--fps 10`, then `--colors 64`.

The existing `demo.gif` lands at 1.2 MB with 85 frames at 760×480, which is a reasonable target.

### If OpenCV cannot open the file

Phone footage is often HEVC/H.265. Convert it first with whatever is on hand:

```bash
# if ffmpeg is available elsewhere
ffmpeg -i 实拍.mov -c:v libx264 -crf 20 -pix_fmt yuv420p 实拍_h264.mp4
```

Then run the converter on the h264 file.

---

## A drop-in shell for the README

The README references both slots with a fixed width, so overwriting the file is the only step.
Paths in the README are relative to the repository root:

```html
<img src="docs/figures/hardware.gif" width="372" alt="..."/>
```

If you would rather have nothing at all in a slot — no placeholder — delete its block from
`README.md` and stop referencing the file.

## Regenerating the placeholders

Only needed if you want a different size or wording; real media replaces them anyway.

```bash
python3 tools/make_placeholder.py
```
