# Recording demos

The README shows `figures/demo.gif`, which is rendered by `tools/record_demo.py` (see
[`TESTING.md`](TESTING.md)). Footage of real hardware can be added alongside it.

To add one:

1. Convert the clip to a repository-sized GIF:

   ```bash
   python3 tools/make_figure_from_video.py <clip>.mp4 docs/figures/hardware.gif \
           --start 10 --duration 8 --width 760 --fps 12
   ```

2. Add a centred image block to `README.md` under the existing demo figure:

   ```html
   <p align="center">
     <img src="docs/figures/hardware.gif" width="760" alt="..."/>
   </p>
   ```

Keep the file under about 3 MB; GitHub serves it as-is. If it is larger, in this order: shorten
`--duration` to 6–8 s, reduce `--width` to 640, then lower `--fps` to 10 and `--colors` to 64.

`docs/figures/demo.gif` is 1.2 MB with 85 frames at 760 × 480, which is a reasonable target.

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

The placeholder files are committed at their final 760 × 480 size, so they only need replacing if
the slot dimensions change.
