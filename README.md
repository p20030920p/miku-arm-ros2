<div align="center">

# miku-arm-ros2

**A Damiao-motor 6-axis arm, ported from ROS 1 Noetic to ROS 2 Jazzy, and verified without the robot**

<sub>catkin → ament · <b>roscpp</b> → rclcpp · the driver board's binary protocol reproduced byte-for-byte · full pipeline closed in simulation</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#quick-start)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![Packages](https://img.shields.io/badge/colcon-7%20packages-success)](#quick-start)
[![Protocol](https://img.shields.io/badge/serial%20protocol-7%2F7-success)](#verification)
[![Simulation](https://img.shields.io/badge/simulation%20e2e-6%2F6-success)](#verification)

[Quick start](#quick-start) &nbsp;•&nbsp; [The two substitutions](#the-two-substitutions) &nbsp;•&nbsp; [Verification](#verification) &nbsp;•&nbsp; [Docs](#documentation)

*English &nbsp;|&nbsp; [中文](README_CN.md)*

</div>

<p align="center">
  <img src="docs/figures/demo.gif" width="760"
       alt="The simulated arm executing a recorded teach path, driven by real joint-state playback"/>
</p>

*Simulation. Recorded teach path replayed through the arm; mesh from the URDF, poses from
`/joint_states` during a playback run. 50 s at 100 Hz.*

<p align="center">
  <img src="docs/figures/hardware.gif" width="372" alt="Hardware demo — video pending"/>
  &nbsp;
  <img src="docs/figures/rviz2.gif" width="372" alt="RViz2 screen recording — pending"/>
</p>

*Hardware (left) and RViz2 (right) — see [`docs/MEDIA.md`](docs/MEDIA.md).*

Six Damiao motors and a gripper on one MCU board, reached over `/dev/ttyACM0` with a 50-byte down /
46-byte up binary protocol. The ROS 1 workspace is in [`reference/`](reference/); `src/` is the
Jazzy port.

| | |
|---|---|
| **Port** | 5 catkin → 7 ament packages · 21 executables · 8 082 lines |
| **Protocol** | **7/7** checks, six joints to **< 0.002 rad** |
| **Pipeline** | **6/6** checks at **100 Hz** |
| **Preserved** | every gain and offset; 3 upstream bugs fixed |
| **Not verified** | the arm, RealSense, GUI — [below](#what-is-not-verified) |

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/p20030920p/miku-arm-ros2.git && cd miku-arm-ros2
colcon build --symlink-install
source install/setup.bash
```

No hardware needed — start the simulated arm, then replay a real recording through it:

```bash
ros2 launch miku_sim sim.launch.py                    # simulated motors + RViz2
ros2 run hardware trajectory_track                    # prompts for a file in teach_path/
```

Against the arm, the same nodes with the serial device attached:

```bash
ros2 launch miku_dummy display.launch.py              # robot_state_publisher + RViz2
ros2 run hardware hardware                            # -p serial_port:=/dev/ttyACM0
ros2 run arm_control teach_one_node                   # gravity-compensated hand-guiding
```

The three `./launch_*.sh` scripts wrap these and home the arm on exit.

## The two substitutions

Two pieces stand in for the arm, covering different layers.

**`virtual_motor_board.py`** implements the driver board's side of the wire: `0x86C1` / `0x86C2`
framing, field offsets, the ×1000 fixed point. Over a `socat` PTY it tests the serial protocol by
running the real `hardware` binary against it.

**`sim_motor_board`** replaces the board on the ROS side and publishes `/joint_states`, which closes
the control loop in simulation.

![The real and simulated paths share every algorithm node; only the motor interface differs](docs/figures/architecture.png)

The controller does not use MoveIt or `ros2_control`; it runs its own KDL kinematics and commands
the motors in MIT mode. Only the motor interface therefore differs between the two paths.

## Verification

```bash
ros2 run miku_sim run_serial_hil_test.sh    # 7 checks
ros2 run miku_sim run_sim_e2e_test.sh       # 6 checks
```

`run_serial_hil_test.sh` — the real `hardware` binary against the virtual board:

| Check | Result |
|---|---|
| Board init, bidirectional frame flow | ✅ |
| Six-joint setpoint accuracy | **< 0.002 rad** |
| MIT torque feed-forward, sign and magnitude | ✅ |
| Gravity droop removed by feed-forward | ✅ |
| Gripper stalls on contact | ✅ |
| Board unplugged mid-run | node survives |

`run_sim_e2e_test.sh` — the full pipeline, closed in simulation:

| Check | Result |
|---|---|
| `/joint_states` rate | **100 Hz** |
| TF tree `base_link → link_6` | complete |
| IK closed loop moves the arm | ✅ |
| Gravity-compensated hover drift | **0.0000 rad** |
| Real teach file replays | **7 325 points** |
| Gripper FSM reaches *grasped* | ✅ |

Setpoint accuracy covers byte order, field offsets and the ×1000 scaling: an error in any of them
appears there as a constant offset.

### What is *not* verified

- **The physical arm.** Protocol and control law are verified; stiffness, friction and the real
  gravity load are not. The gains in `arm_control_params.yaml` come from the original tuning and
  are unverified.
- **The RealSense D435.** No camera available. `deep_camera` and `aruco` were tested with synthetic
  images (750 mm depth, marker `ID=341`, pose solved), not real sensor data.
- **Joint limits.** `KDL::ChainIkSolverPos_LMA` ignores them, as upstream. Unreachable targets log
  `IK 失败，本步跳过` and are skipped.
- **GUI interaction.** Qt5 highgui creates no X window here, so the mouse-pick and ESC paths in
  `deep_camera` are untested.

## Layout

```
src/
  arm_control/               KDL FK/IK, planner, gravity comp, claw FSM
  hardware/                  serial node, replay, teaching record, tests
  miku_sim/                  virtual driver board, simulated motors, tests
  miku_dummy/                URDF, meshes, RViz2 config
  miku_dummy_moveit_config/  MoveIt 2 config (SRDF + planner params)
  aruco/                     detector, ROS 2 node, marker-production material
  deep_camera/               RealSense RGB-D capture, pose estimation
docs/                        PORTING.md, TESTING.md, OVERVIEW.md, figures/
reference/ros1-original/     the ROS 1 workspace, unmodified (COLCON_IGNORE)
tools/                       demo recorder, figure generators, video import
```

## Documentation

| | |
|---|---|
| [`docs/OVERVIEW.md`](docs/OVERVIEW.md) | why a port; what the arm does; how the substitutes work |
| [`docs/PORTING.md`](docs/PORTING.md) | the ROS 1 → ROS 2 mapping rules, and where they did not apply |
| [`docs/TESTING.md`](docs/TESTING.md) | what each check asserts; re-recording the figures |
| [`docs/MEDIA.md`](docs/MEDIA.md) | adding real hardware footage; the two reserved slots |
| [`CHANGELOG.md`](CHANGELOG.md) | including the three upstream defects fixed |

## License

MIT. The bundled ArUco detector is MIT, © 2017 Tentone — see [`src/aruco/LICENSE`](src/aruco/LICENSE).
