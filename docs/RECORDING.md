# Recording demos

Two figures in the README are generated from data, not hand-drawn:

| Figure | Generator | Source |
|---|---|---|
| `docs/figures/demo.gif` | `tools/record_demo.py` | a recorded teach path replayed through the simulator |
| `docs/figures/demo_rviz.gif` | `tools/rviz_demo.py` | a live MoveIt plan-and-execute run captured from the RViz2 window |

## demo.gif — teach path replay

`tools/capture_joint_states.py` records `/joint_states` while `hardware trajectory_track` replays a
teach file; `tools/record_demo.py` renders the URDF and its STL meshes through VTK at each sampled
pose. No hardware and no display are involved, so it is reproducible on any machine.

## demo_rviz.gif — MoveIt plan and execute

`tools/rviz_demo.py` starts a MoveIt goal for the `manipulator` group, captures the RViz2 window
while the arm moves, and writes the GIF. It is scripted rather than mouse-driven for a specific
reason:

> This machine is a Wayland session and RViz runs under XWayland. XTEST-synthesised pointer
> *motion* reaches Qt (hover highlighting works), but button press/release events do not activate
> widgets — clicking the File menu, the Joints tab, the Grid checkbox and the Plan button were all
> verified to have no effect. So the recorder drives MoveIt's `/move_action` directly instead of
> clicking the panel. RViz still shows the goal, the planned trajectory and the arm moving, because
> that is the same code path the panel's **Plan** and **Execute** buttons invoke.

Run it against a demo launch that is already up:

```bash
# terminal 1 — slow playback so the motion is legible when recorded
ros2 launch miku_moveit_demo moveit_demo.launch.py record:=true speed_scale:=0.18

# terminal 2
python3 tools/rviz_demo.py --out docs/figures/demo_rviz.gif --width 900 \
        --crop 20,60,1620,980 --capture 20,60,1620,980 \
        --vel-scale 0.5 --acc-scale 0.5 --hold-fps 20 --speed 1.35
```

Flags that matter:

- `--crop` is the region of the RViz window that ends up in the GIF (the panel plus the 3D view).
  `--capture` is the region read from X, which should cover `--crop`.
- `--hold-fps` is the *target* capture rate. It is a ceiling, not a promise: each capture costs
  about 45 ms, so on a big window the real rate lands lower.
- `--speed` scales the finished GIF against real time. Each frame keeps the interval it was
  actually captured at, so `1.0` plays at true speed regardless of how fast capture ran. Earlier
  versions resampled to a fixed fps and silently compressed a 4 s run into 1 s.
- `speed_scale` on the launch is what actually slows the *arm*; MoveIt's velocity scaling only
  changes how the trajectory is time-parameterised, and `trajectory_bridge` re-times it anyway.

The recorder exits non-zero if the final joint error exceeds 0.05 rad, so a failed run cannot be
mistaken for a good take.

### Window capture

Screenshots on this setup are awkward enough that the tools in `tools/win_capture/` exist:

| Tool | Purpose |
|---|---|
| `xgrab.py` | capture a window and locate windows by walking the X window tree |
| `pngrab.c` | fast capture: `XShmGetImage` + raw RGB to stdout, ~23 fps at 1900×1190 |
| `win_grab.c` | capture by window title, writes PPM |
| `win_id_resize.c` | resize/move a client window by ID |
| `list_windows.py` | list windows above a size, including reparented client windows |

Three things that cost time to find out:

- **Root-window reads are refused.** `xwd -root` and `PIL.ImageGrab` fail with `BadMatch` under
  XWayland. Capturing a *client* window works.
- **Do not find windows by title.** Mutter's frame window carries the same title as the client, so
  a title search can return the decorated frame and shift every crop coordinate. Use the window ID
  and check the class hint (`rviz2`, not `mutter-x11-frames`).
- **`XGetImage` is slow in a way that is easy to misread.** A full 1900×1190 window takes ~340 ms
  through `XGetImage`, which looks like a slow X server but is really per-row protocol overhead.
  `XShmGetImage` does the same capture in 5 ms.

## Existing and hardware footage

Hardware clips can be added alongside the generated figures:

1. Convert the clip to a repository-sized GIF:

   ```bash
   python3 tools/make_figure_from_video.py <clip>.mp4 docs/figures/hardware.gif \
           --start 10 --duration 8 --width 760 --fps 12
   ```

   `ffmpeg` is not installed here, so the converter uses OpenCV + Pillow. It accepts anything
   OpenCV can decode (mp4, mov, avi, mkv, m4v). Add `--poster --at 14` for a still frame, or
   `--mp4` to recompress to h264.

2. Add a centred image block to `README.md` next to the existing figures:

   ```html
   <p align="center">
     <img src="docs/figures/hardware.gif" width="760" alt="..."/>
   </p>
   ```

### Before you commit

Keep each GIF under about 3 MB; GitHub serves it as-is, and a 10 MB autoplay GIF is unpleasant on a
laptop and unusable on a phone. If one is too big, in this order:

1. Shorten `--duration` to 6–8 s. A short loop that shows the motion beats a long one.
2. Reduce `--width` to 640 — still readable in the README's content column.
3. Lower `--fps` to 10, then `--colors` to 64.

For reference, `demo.gif` is 1.2 MB (85 frames, 760×480) and `demo_rviz.gif` is 0.2 MB (24 frames,
900×517, 7.5 s).
