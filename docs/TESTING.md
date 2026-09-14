# Testing

Two suites, both runnable with no hardware. They answer different questions and are meant to be run
together — between them they cover everything that could otherwise only be checked on the robot.

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

Two things in this suite were hard-won and are worth knowing before editing it:

- **Subscribe first, then publish.** Earlier versions used one-shot `ros2 topic echo --once` probes
  and intermittently captured nothing, because ROS 2 discovery is asynchronous and the probe window
  raced it. Checks 2–4 now start a subscriber *before* the publisher and read the result at the end
  of the window.
- **Reset the CLI daemon.** `ros2 daemon stop` runs at the top of the script. A daemon carrying a
  stale graph from a previous crashed run makes python subscribers receive nothing at all — which
  looks exactly like a broken robot. This was the actual cause of the TF check flapping; with the
  reset it passes consistently.

Environment: `ROS_DOMAIN_ID` (default 72), same isolation as above.

---

## Regenerating the figures

The README figures come from real runs, not from drawing code alone. `tools/make_hero.py`
in particular needs captured data.

```bash
# 1. start the simulated arm
ros2 launch miku_sim sim.launch.py use_rviz:=false &

# 2. capture joint states while a recorded teach path plays back
python3 tools/capture_joint_states.py 22 /tmp/capture_traj.json &
sleep 2
NAME=$(basename "$(ls install/hardware/share/hardware/teach_path/*.txt | head -1)" .txt)
printf '%s\n\n' "$NAME" | ros2 run hardware trajectory_track

# 3. rebuild the figures
python3 tools/make_hero.py /tmp/capture_traj.json docs/figures/hero.png
python3 tools/make_architecture.py
```

`make_architecture.py` draws from the code, not from a run — if you rename a topic, update it there.
`make_hero.py` panels C and D carry values measured by the HIL suite; if you change the simulator's
model, re-measure them rather than leaving stale numbers in the figure.

## Adding a check

Both suites are plain bash with a `pass`/`fail` helper and an exit-code summary, so a new check is a
new numbered block. Two rules, both learned the hard way:

1. Never assert on a *single* sample. Sample a window, or assert on an extremum.
2. Clean up the previous check's processes before the next one starts. `trajectory_track` has a
   60 s timeout and will keep publishing mode-2 frames into the next check if you only `kill` the
   wrapper.
