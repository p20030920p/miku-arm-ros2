# Changelog

## [Unreleased]

### Added
- `miku_sim` package: `virtual_motor_board.py`, a byte-faithful peer for the Damiao driver board's
  binary protocol, and `sim_motor_board`, a ROS-side motor substitute that publishes `/joint_states`.
- `run_serial_hil_test.sh` — 7 checks driving the real `hardware` binary against the virtual board
  over a `socat` PTY. No hardware required.
- `run_sim_e2e_test.sh` — 6 checks closing the full pipeline in simulation, including teach-path
  replay of a recorded trajectory.
- `miku_dummy_moveit_config` — MoveIt 2 configuration for `miku_arm`.
- `tools/` — figure generators and a joint-state capture tool.
- Real teach recordings (38 610 lines) under `src/hardware/teach_path/`.
- ArUco marker production material (encoding layout, SVGs, A4 sheet) under `src/aruco/aruco_set/`.
- `reference/ros1-original/` — the ROS 1 workspace, unmodified, with a `COLCON_IGNORE`.

### Changed
- `hardware` serial device and baud rate are now node parameters (`serial_port`,
  `serial_baudrate`). Defaults are unchanged from the previously hard-coded `/dev/ttyACM0` and
  115200, so real-robot behaviour is identical.
- `ArmMsg.Task_status` renamed to `task_status` — ROS 2's IDL generator requires lower-case field
  names.

### Fixed
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
