# Testing

Two suites, both runnable without hardware. They cover different layers and are meant to be run
together.

```bash
source /opt/ros/jazzy/setup.bash
cd miku-arm-ros2 && colcon build --symlink-install && source install/setup.bash

ros2 run miku_sim run_serial_hil_test.sh    # 7 checks — the serial protocol
ros2 run miku_sim run_sim_e2e_test.sh       # 6 checks — the control pipeline
```

Both exit non-zero if any check fails and print a pass/fail table.

---

## 1. `run_serial_hil_test.sh` — the serial protocol

### What is actually under test

The **real `hardware` binary**, unmodified, talking to `virtual_motor_board.py` over a `socat` PTY
pair. Nothing is stubbed: the same `serial::Serial` implementation, the same frame packing, the same
`/Arm_rx` parsing that runs against the robot.

```
hardware (real binary)  ──PTY──  virtual_motor_board.py
   /dev/ttyACM0-ish                  frame parser + motor model
```

The script creates the PTY pair itself, starts the board, points the node at it with
`-p serial_port:=`, runs seven checks, and tears everything down on exit.

### The checks

| # | Check | What it asserts |
|---|---|---|
| 1 | Serial opens | the in-tree POSIX `serial::Serial` initialises against a PTY |
| 2 | Bidirectional flow | ≥ 50 messages on `/Arm_rx`; both 50 B down and 46 B up frames parse |
| 3 | Position-mode accuracy | six joints reach setpoint to **< 0.002 rad**, proving the ×1000 fixed-point encode/decode is lossless |
| 4 | MIT torque feed-forward | `±0.5` and `+0.25 N·m` come back with correct sign and magnitude |
| 5 | Gravity droop vs feed-forward | with a 0.45 N·m load, position mode sags and MIT + feed-forward does not |
| 6 | Gripper contact | the claw stalls at −1.2 rad with torque rising to 0.6 N·m — the signature `ClawController` keys on |
| 7 | Board unplugged mid-run | killing the board does not kill the node |

Check 3 is the one that matters most for the port: any error in byte order, field offset, or the
×1000 convention would show up as a constant offset here.

Check 7 reproduces the failure the original code was written around — an MCU reset raising
`IOException`. Note this check currently passes via the weaker branch (the node survives and logs
the fault); the "重连成功" message is not always observed, because the reconnect path depends on how
the PTY surfaces the disconnection. It is reported honestly rather than asserted.

### Tuning

`--no-gravity` turns off the simulated load, which check 3 uses so that following accuracy is
measured without a disturbance. `--no-object` leaves the gripper empty, which exercises the
*failed to grasp* branch instead.

Environment: `ROS_DOMAIN_ID` (default 71) and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` are forced,
so the suite does not collide with other ROS 2 systems on the same machine.

---

## 2. `run_sim_e2e_test.sh` — the control pipeline

### What is actually under test

Real algorithm nodes — `arm_control_node`, `teach_one_node`, `trajectory_track` — closing the loop
against `sim_motor_board`, with `robot_state_publisher` turning the resulting `/joint_states` into TF.

```
arm_control_node ──Arm_tx──> sim_motor_board ──/Arm_rx──> arm_control_node
                                    └──/joint_states──> robot_state_publisher ──> TF
```

### The checks

| # | Check | What it asserts |
|---|---|---|
| 1 | Link up | `/joint_states` averaging ~100 Hz |
| 2 | TF tree | `base_link → link_1 … → link_6` all present, from both `/tf` and `/tf_static` |
| 3 | IK closed loop | a target pose is solved and the **simulated arm measurably moves** (> 0.05 rad) |
| 4 | Gravity compensation | under load, `teach_one_node` holds attitude — drift < 0.6 rad over 12 s |
| 5 | Teach-path replay | a real recording loads and is commanded out point by point |
| 6 | Gripper FSM | commanding closure drives the claw onto the object and stalls it |

### Implementation notes

Two things to know before editing this suite:

- **Subscribe first, then publish.** ROS 2 discovery is asynchronous, so one-shot
  `ros2 topic echo --once` probes intermittently captured nothing. Checks 2–4 start a subscriber
  before the publisher and read the result at the end of the window.
- **Reset the CLI daemon.** `ros2 daemon stop` runs at the top of the script. A daemon holding a
  stale graph from a crashed run makes python subscribers receive nothing, which is indistinguishable
  from a broken robot. This caused the TF check to fail intermittently.

Environment: `ROS_DOMAIN_ID` (default 72), same isolation as above.

---

## Regenerating the figures

The README figures come from real runs.

`docs/figures/demo.gif` is rendered by `tools/record_demo.py`, which loads the URDF and STL meshes
and poses them from recorded joint states. RViz2 cannot be recorded headlessly here: with no X
server, Ogre fails to create a GLX window.

```bash
# 1. start the simulated arm
ros2 launch miku_sim sim.launch.py use_rviz:=false &

# 2. capture joint states while a recorded teach path plays back
python3 tools/capture_joint_states.py 50 /tmp/capture.json &
sleep 2
printf '2025-10-11_17-38-34\n\n' | ros2 run hardware trajectory_track

# 3. render
python3 tools/record_demo.py /tmp/capture.json --frames 85 --fps 15 \
        --size 760x480 --outdir docs/figures
```

`docs/figures/architecture.png` is drawn from the code by `tools/make_architecture.py`, not from a
run. After editing it, check the layout:

```bash
python3 tools/check_figure.py      # no text out of bounds, across borders, or overlapping
```

The canvas is fixed at 8.6 × 2.05 in on purpose: the README column is about 800 px wide, so a taller
figure is scaled down until the labels are unreadable. The generator asserts its own width and
height budget and fails rather than emitting a broken figure. Captions and panel data for the
figures carry measured values; re-measure them if the simulator model changes.

`docs/figures/demo_rviz.gif` is recorded from a live RViz2 window by `tools/rviz_demo.py`. It needs
a real display, so it cannot be regenerated headlessly. See [`RECORDING.md`](RECORDING.md) for the
command and for why the recorder drives MoveIt's action interface instead of synthesising clicks.

## Coverage

Both suites run without hardware. Together they cover the serial protocol and the control pipeline.

Not covered:

- **The physical arm.** Protocol framing, field offsets, scaling and the control law are verified;
  stiffness, friction and the actual gravity load are not. The gains in
  `arm_control_params.yaml` carry over from the original tuning.
- **The RealSense D435.** `deep_camera` and `aruco` are exercised with synthetic images
  (750 mm depth, marker `ID=341`, pose solved), not with sensor data.
- **Joint limits.** `KDL::ChainIkSolverPos_LMA` does not consider them, matching upstream.
  Unreachable targets log `IK 失败，本步跳过` and are skipped.
- **GUI interaction.** Qt5 highgui needs an X display; the mouse-pick and ESC paths in
  `deep_camera` are not exercised by the suites.
- **The MoveIt demo's GUI.** `trajectory_bridge` and the `FollowJointTrajectory` link to it are
  covered indirectly — `tools/rviz_demo.py` runs the same `/move_action` path the panel's
  **Execute** button uses and fails if the final joint error exceeds 0.05 rad — but clicking the
  panel itself is not. Synthetic button events are not delivered to Qt on this Wayland/XWayland
  setup (see `RECORDING.md`). Collision checking against a cluttered scene is also untested; the
  planning scene is empty.


## Adding a check

Both suites are plain bash with a `pass`/`fail` helper and an exit-code summary, so a new check is a
new numbered block. Two rules:

1. Sample a window or an extremum; never assert on a single sample.
2. Kill the previous check's processes before starting the next. `trajectory_track` has a 60 s
   timeout and keeps publishing mode-2 frames into the next check if only its wrapper is killed.
