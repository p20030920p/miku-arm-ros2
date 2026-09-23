# Changelog

## [Unreleased]

### Added
- `docs/figures/demo_hardware.gif` — the real arm on the bench following a goal dragged in the RViz2
  MotionPlanning panel. It takes the front-page slot that `demo_rviz.gif` held, and was made with the
  converter below from a hand-held phone take.
- `miku_sim` package: `virtual_motor_board.py`, a byte-faithful peer for the Damiao driver board's
  binary protocol, and `sim_motor_board`, a ROS-side motor substitute that publishes `/joint_states`.
- `run_serial_hil_test.sh` — 7 checks driving the real `hardware` binary against the virtual board
  over a `socat` PTY. No hardware required.
- `run_sim_e2e_test.sh` — 6 checks closing the full pipeline in simulation, including teach-path
  replay of a recorded trajectory.
- `miku_dummy_moveit_config` — MoveIt 2 configuration for `miku_arm`.
- `miku_moveit_demo` — MoveIt 2 planning demo: `trajectory_bridge` serves
  `FollowJointTrajectory` and republishes trajectories as `ArmMsg(mode=2)`, so **Plan** and
  **Execute** in the RViz2 MotionPlanning panel drive the arm in simulation.
- `tools/` — figure generators, a joint-state capture tool, and `rviz_demo.py`, which records the
  MoveIt demo from the RViz2 window.
- `docs/figures/demo_rviz.gif` — the MoveIt planning and execution demo.
- Real teach recordings (38 610 lines) under `src/hardware/teach_path/`.
- ArUco marker production material (encoding layout, SVGs, A4 sheet) under `src/aruco/aruco_set/`.
- `reference/ros1-original/` — the ROS 1 workspace, unmodified, with a `COLCON_IGNORE`.

### Changed
- **The architecture figure is off the front page**: the READMEs no longer show
  `docs/figures/architecture.png`. The file and its generator (`tools/make_architecture.py`, checked by
  `tools/check_figure.py`) stay in the repository. No code change.`n- **The verification section keeps the total**: dropping the front-page table also dropped the only`n  place the aggregate count was written, so the two suites now state it in prose (13 checks, no hardware`n  attached). No code change.`n- **The front page drops the summary table and the simulator rationale**: the
  package/control/mode/verification/porting table restated what the sections below it already say, and the
  two paragraphs on why the simulators exist duplicated `docs/OVERVIEW.md`. The architecture figure
  stays, next to the hardware interface it diagrams.`n- **README restructured as library documentation**: the two verification tables became one, the
  packages and documentation tables are merged, a Provenance section states what the port added over
  the unmodified ROS 1 workspace, and the verification section names the machine the checks ran on
  (Huawei MateBook 14 2022, Intel Core i5-1240P, CPU only). No code change.
- `tools/make_figure_from_video.py` gained `--crop x0,y0,x1,y1`, applied before the resize, so a
  portrait phone take can be cut to the bench instead of published as a 400 × 711 strip.
- `hardware` serial device and baud rate are now node parameters (`serial_port`,
  `serial_baudrate`). Defaults are unchanged from the previously hard-coded `/dev/ttyACM0` and
  115200, so real-robot behaviour is identical.
- `ArmMsg.Task_status` renamed to `task_status` — ROS 2's IDL generator requires lower-case field
  names.

### Fixed
- `tools/make_figure_from_video.py` measured `--duration` from the start of the file rather than
  from the seek point, so `--start 10 --duration 8` produced an 18 s clip. It now produces 8 s.
- `miku_dummy_moveit_config/config/joint_limits.yaml`: every joint had
  `has_acceleration_limits: false` and `max_acceleration: 0`. MoveIt's
  `AddTimeOptimalParameterization` adapter then failed with "No acceleration limit was defined for
  joint joint_N", producing trajectories with zero timestamps that were planned but never
  executed. Real limits are now set.
- `hardware.cpp`: `catch (SerialException&)` preceded `catch (IOException&)`, making the latter
  branch unreachable, so a vanished device was misreported. Order swapped.
- `aruco`: the `topic_marker_remove` parameter was read into the *register* variable, leaving
  removal non-functional. Each parameter now reads into its own variable.
- `kinematics_solver.cpp`: `KDL::Tree::getChain` returns `bool` in orocos_kdl 1.5 (it returned
  `int`, with `< 0` meaning failure), so the old test never fired. Now negated.
- `arm_control` URDFs referenced meshes from packages that did not contain them; meshes moved into
  the package and references rewritten so each URDF resolves standalone.

## [0.1.0] — initial port

Complete ROS 1 Noetic → ROS 2 Jazzy port of the original five catkin packages: `arm_control`,
`hardware`, `deep_camera`, `aruco`, `miku_dummy`.
