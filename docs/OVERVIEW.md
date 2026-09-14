# Overview

Why this exists, what the arm does, and how it is verified without being plugged in.

## Why a port

The original is a ROS 1 Noetic catkin workspace driving a 6-axis arm. Noetic is not supported on
Ubuntu 24.04 and cannot be installed, so the code moved to ROS 2 Jazzy rather than being rehosted
behind a container or a second machine.

It is a translation, not a redesign. Where ROS 2 genuinely has no equivalent, the change is marked
`// ROS 2 差异：` in the source and explained in [`PORTING.md`](PORTING.md).

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

With `kp = 0` in MIT mode the arm is back-drivable, and the gravity compensator alone holds it up.
That is how teaching works: you move the arm by hand and a 1 kHz recorder captures the path. The
compensator computes joint torques from the URDF's inertial model via KDL and scales them by six
hand-calibrated gains, one per joint.

The vision path is separate and optional: a RealSense D435 feeds `deep_camera`, which detects ArUco
markers, reads depth at the marker, and solves pose with `solvePnP`. The marker set is real hardware
— the production PDF, SVGs and the encoding layout are in [`src/aruco/aruco_set/`](../src/aruco/aruco_set/).

## Why the original is unusually testable

It never went through MoveIt or `ros2_control`. The whole controller is ordinary ROS nodes over two
topics:

| Topic | Type | |
|---|---|---|
| `Arm_tx` | `arm_control/msg/ArmMsg` | setpoints down: 7 motors × position/velocity/torque, plus `kp`, `kd`, mode |
| `Arm_rx` | `arm_control/msg/ArmMsg` | measured state up |

Everything between "a target pose" and "bytes on the wire" lives above `Arm_tx`. Everything between
"bytes on the wire" and "the arm moved" lives below it. That clean seam is what makes it possible to
verify the two halves independently — which is exactly what the two substitutions do.

## The two substitutions

### `virtual_motor_board.py` — the protocol

Implements the MCU's side of the wire: frame parsing, the ×1000 decode, a motor model, and frame
generation. It sits on the far end of a `socat` PTY pair, so the **real `hardware` binary** runs
against it unmodified — same `serial::Serial`, same packing code, same `/Arm_rx` parsing that runs
against the robot.

The motor model deliberately includes gravity in **both** modes, as a stiffness-limited steady-state
droop in position mode and as a load in MIT mode. Without that, the gravity compensator has nothing
to compensate and the test proves nothing. The first version of this simulator made exactly that
mistake.

### `sim_motor_board` — the control law

Replaces the board on the ROS side: subscribes to `Arm_tx`, models the motors, publishes `/Arm_rx`
back (closing the loop for the algorithm nodes) and `/joint_states` out (so `robot_state_publisher`
produces TF and RViz2 draws the arm).

This is what the demo GIF shows — the URDF meshes driven by real recorded joint data, so the motion
is a real playback rather than an animation.

## Why not Gazebo

`gz sim` 8.11 is installed and runs. But `gz_ros2_control`'s `GazeboSimSystem` plugin is only
registered inside the `gz-sim` process and cannot be loaded by a standalone `controller_manager`
(verified: `plugin does not exist`). Since the original never used `ros2_control`, dropping it costs
nothing — and it keeps the simulated path structurally identical to the real one, which is the whole
point.

RViz2 was likewise unusable for recording: with no X server available, Ogre cannot create a GLX
window (`Invalid parentWindowHandle (wrong server or screen)`). The demo is therefore rendered in VTK
from the same URDF and the same STL meshes, driven by recorded `/joint_states` — see
[`../tools/record_demo.py`](../tools/record_demo.py).

## What the port changed

Beyond the mechanical ROS 1 → ROS 2 translation, three upstream defects were fixed. Each had
silently disabled a feature, and each is described in [`../CHANGELOG.md`](../CHANGELOG.md):

- `hardware.cpp` caught `SerialException` before `IOException`, making the latter unreachable, so a
  vanished device was misreported.
- `aruco` read `topic_marker_remove` into the *register* variable, so marker removal never worked.
- `kinematics_solver.cpp` tested `getChain(...) < 0`, but orocos_kdl 1.5 returns `bool`, so the test
  never fired.

Two deliberate non-changes:

- **Joint limits are still ignored.** `KDL::ChainIkSolverPos_LMA` does not consider them. Fixing that
  would change which targets the arm accepts, which is a behavioural decision for the owner.
- **Gains are unchanged.** The six gravity-compensation coefficients were calibrated on the physical
  arm. They are carried over as-is.
