# Overview

Why this exists, what the arm does, and how it is verified without being plugged in.

## Why a port

The original is a ROS 1 Noetic catkin workspace driving a 6-axis arm. Noetic is not supported on
Ubuntu 24.04 and cannot be installed, so the code moved to ROS 2 Jazzy rather than being rehosted
behind a container or a second machine.

Where ROS 2 has no equivalent, the change is marked `// ROS 2 差异：` in the source and listed in
[`PORTING.md`](PORTING.md).

## What the arm does

Six Damiao motors plus a gripper, all on one MCU driver board, reached over `/dev/ttyACM0`. The MCU
speaks a fixed binary protocol: a 50-byte command frame down, a 46-byte feedback frame up, every
physical quantity scaled ×1000 into an `int16`. Field offsets and frame headers are documented at the
top of [`virtual_motor_board.py`](../src/miku_sim/scripts/virtual_motor_board.py), which is the
executable specification of the protocol as implemented.

Two control modes carry the design:

| Mode | Behaviour | Used for |
|---|---|---|
| `mode=2` | velocity-limited position control | homing, trajectory replay |
| `mode=1` | MIT — position stiffness `kp`, damping `kd`, plus a torque feed-forward | compliant holding, hand-guided teaching |

With `kp = 0` in MIT mode the arm is back-drivable and the gravity compensator holds it up, which
is how hand-guided teaching works: a 1 kHz recorder captures the path. The compensator computes
joint torques from the URDF inertial model via KDL and scales them by six calibrated gains.

The vision path is optional: a RealSense D435 feeds `deep_camera`, which detects ArUco markers,
reads depth at the marker and solves pose with `solvePnP`. Marker production files (PDF, SVGs,
encoding layout) are in [`src/aruco/aruco_set/`](../src/aruco/aruco_set/).

## Why the original is unusually testable

It never went through MoveIt or `ros2_control`. The whole controller is ordinary ROS nodes over two
topics:

| Topic | Type | |
|---|---|---|
| `Arm_tx` | `arm_control/msg/ArmMsg` | setpoints down: 7 motors × position/velocity/torque, plus `kp`, `kd`, mode |
| `Arm_rx` | `arm_control/msg/ArmMsg` | measured state up |

Everything between a target pose and the bytes on the wire is above `Arm_tx`; everything between
those bytes and the arm moving is below it. The two halves can therefore be verified separately.

## The two substitutions

### `virtual_motor_board.py` — the protocol

Implements the MCU's side of the wire: frame parsing, the ×1000 decode, a motor model and frame
generation, on the far end of a `socat` PTY pair. The real `hardware` binary runs against it, using
the same serial code and frame packing as on the robot.

The motor model applies gravity in both modes: as a stiffness-limited steady-state droop in position
mode, and as a load in MIT mode. Without it the gravity compensator has nothing to compensate.

### `sim_motor_board` — the control law

Replaces the board on the ROS side: subscribes to `Arm_tx`, models the motors, publishes `/Arm_rx`
back (closing the loop for the algorithm nodes) and `/joint_states` out (so `robot_state_publisher`
produces TF and RViz2 draws the arm).

The demo GIF renders the URDF meshes from the same recorded joint data.

## Why not Gazebo

`gz sim` 8.11 is installed and runs, but `gz_ros2_control`'s `GazeboSimSystem` plugin is only
registered inside the `gz-sim` process and cannot be loaded by a standalone `controller_manager`
(`plugin does not exist`). The controller does not use `ros2_control` anyway, so dropping it keeps
the simulated path structurally identical to the real one.

RViz2 could not be recorded either: with no X server, Ogre cannot create a GLX window
(`Invalid parentWindowHandle (wrong server or screen)`). The demo is rendered in VTK from the same
URDF and STL meshes, driven by recorded `/joint_states`
([`../tools/record_demo.py`](../tools/record_demo.py)).

## What the port changed

Three upstream defects were fixed; each had disabled a feature. Details in
[`../CHANGELOG.md`](../CHANGELOG.md):

- `hardware.cpp` caught `SerialException` before `IOException`, making the latter unreachable, so a
  vanished device was misreported.
- `aruco` read `topic_marker_remove` into the *register* variable, so marker removal never worked.
- `kinematics_solver.cpp` tested `getChain(...) < 0`, but orocos_kdl 1.5 returns `bool`, so the test
  never fired.

Two things were left unchanged:

- **Joint limits** are still ignored by `KDL::ChainIkSolverPos_LMA`. Changing that would alter which
  targets the arm accepts.
- **Gains**: the six gravity-compensation coefficients carry over from the original calibration.
