<div align="center">

# miku-arm-ros2

**ROS 2 Jazzy driver and control stack for a 6-axis Damiao-motor arm with a gripper**

<sub>KDL inverse kinematics · gravity compensation · trajectory replay and teaching · ArUco / RealSense pose estimation</sub>

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)](#build)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](#build)
[![License](https://img.shields.io/badge/license-MIT-3DA639)](LICENSE)

[Build](#build) &nbsp;•&nbsp; [Hardware](#with-hardware) &nbsp;•&nbsp; [Simulation](#without-hardware) &nbsp;•&nbsp; [Verification](#verification) &nbsp;•&nbsp; [Provenance](#provenance)

*English &nbsp;|&nbsp; [中文](README_CN.md)*

</div>

<p align="center">
  <img src="docs/figures/demo.gif" width="760"
       alt="Recorded teach path replayed on the simulated arm"/>
</p>

*Recorded teach path replayed in simulation: mesh from the URDF, poses from `/joint_states`
during playback. 50 s at 100 Hz.*

<p align="center">
  <img src="docs/figures/demo_hardware.gif" width="400"
       alt="The real arm on the bench following a goal dragged in the RViz2 MotionPlanning panel"/>
</p>

*Hardware demo. The self-built arm on the bench, following a goal dragged in the RViz2
MotionPlanning panel through the same `trajectory_bridge` path as the simulation above.*

The arm is six Damiao motors and a gripper on one MCU board, driven over `/dev/ttyACM0` with a
50-byte down / 46-byte up binary protocol. The controller uses neither MoveIt nor `ros2_control`: it
runs its own KDL kinematics and commands the motors in MIT mode.

## Build

Requires Ubuntu 24.04 and ROS 2 Jazzy.

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/p20030920p/miku-arm-ros2.git
cd miku-arm-ros2
colcon build --symlink-install
source install/setup.bash
```

## With hardware

Start the model display and the serial node, then choose a controller:

```bash
ros2 launch miku_dummy display.launch.py     # robot_state_publisher + RViz2
ros2 run hardware hardware                   # serial node
```

```bash
# keyboard-driven Cartesian targets
ros2 run arm_control arm_control_node

# hand-guided teaching: gravity compensation only, kp = 0
ros2 run arm_control teach_one_node

# replay a recorded trajectory from hardware/teach_path
ros2 run hardware trajectory_track
```

The serial device defaults to `/dev/ttyACM0` at 115200 baud and can be overridden:

```bash
ros2 run hardware hardware --ros-args -p serial_port:=/dev/ttyACM1
```

`./launch_hardware.sh`, `./launch_arm_teach_one_node.sh` and `./launch_claw_test.sh` wrap these
sequences and home the arm on exit.

## Without hardware

Start the simulated arm, then drive it with the same controller nodes:

```bash
ros2 launch miku_sim sim.launch.py           # simulated motors + RViz2
ros2 run hardware trajectory_track           # replay a recorded trajectory
```

### Planning with MoveIt 2

```bash
ros2 launch miku_moveit_demo moveit_demo.launch.py
```

This brings up the simulated motors, `move_group`, `trajectory_bridge` and RViz2. Drag the
interactive marker in the 3D view to set a goal, then use **Plan** and **Execute** in the
MotionPlanning panel. `trajectory_bridge` publishes the trajectory as `ArmMsg(mode=2)` on
`/Arm_tx` at 50 Hz; `speed_scale:=0.2` slows playback for demonstration, and `record:=true`
switches RViz to the single-panel layout used for recording.

## Verification

```bash
ros2 run miku_sim run_serial_hil_test.sh     # serial protocol, 7 checks
ros2 run miku_sim run_sim_e2e_test.sh        # control pipeline, 6 checks
```

Both suites run CPU only — 13 checks in total, with no hardware attached — on a thin-and-light
laptop (Huawei MateBook 14, Intel Core i5-1240P); nothing in the stack needs a GPU.

| Suite | What it establishes |
|---|---|
| `run_serial_hil_test.sh` | board init and bidirectional frame flow; six-joint setpoint accuracy < 0.002 rad; MIT torque feed-forward sign and magnitude; gravity droop removed by feed-forward; gripper stalls on contact; node survives the board being unplugged mid-run |
| `run_sim_e2e_test.sh` | `/joint_states` at 100 Hz; complete TF tree `base_link → link_6`; IK closed loop moves the arm; gravity-compensated hover drift 0.0000 rad; a recorded teach file replays (7 325 points); the gripper FSM reaches *grasped* |

What each check asserts, and which parts are not covered: [`docs/TESTING.md`](docs/TESTING.md).

## Provenance

The original is a ROS 1 Noetic catkin workspace, kept unmodified in [`reference/`](reference/) under a
`COLCON_IGNORE`. This repository is its ROS 2 Jazzy port, plus the MoveIt 2 configuration and demo,
the two simulation substitutes and their tests, and the fixes in [`CHANGELOG.md`](CHANGELOG.md) —
among them a MoveIt joint-limit file with `max_acceleration: 0`, which produced trajectories that
were planned but never executed.

## Docs

Each package, and the reference documents:

- [`arm_control`](src/arm_control) — kinematics, gravity compensator, linear planner, gripper FSM, control nodes
- [`hardware`](src/hardware) — serial node, trajectory replay, teaching recorder, test nodes
- [`miku_sim`](src/miku_sim) — simulated motors, virtual driver board, both test suites
- [`deep_camera`](src/deep_camera) · [`aruco`](src/aruco) — RealSense RGB-D capture and ArUco pose estimation
- [`miku_dummy`](src/miku_dummy) · [`miku_dummy_moveit_config`](src/miku_dummy_moveit_config) · [`miku_moveit_demo`](src/miku_moveit_demo) — URDF, meshes, MoveIt 2 configuration and the planning demo

[`docs/OVERVIEW.md`](docs/OVERVIEW.md) control pipeline, topics, simulation design ·
[`docs/PORTING.md`](docs/PORTING.md) ROS 1 → ROS 2 mapping rules ·
[`docs/TESTING.md`](docs/TESTING.md) test suites, coverage and gaps ·
[`docs/RECORDING.md`](docs/RECORDING.md) recording demos ·
[`CHANGELOG.md`](CHANGELOG.md) release history.

## License

MIT. The bundled ArUco detector is MIT, © 2017 Tentone — see [`src/aruco/LICENSE`](src/aruco/LICENSE).
