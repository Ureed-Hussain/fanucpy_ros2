# Changelog

All notable project changes are documented here.

## Unreleased

- Clarified controller-side MAPPDK version/capability diagnostics, including
  `INTP-310` recovery and persistent-connection behavior after TP programs.
- Power-command failures no longer print a misleading `0.000 W` value, and an
  unsupported `ins_pwr` command now identifies a mismatched controller build.
- Added an atomic, configurable post-program MAPPDK socket recycle and live
  state verification so sequential TP actions do not reuse a stale session.
- Added `fanucpy_ros2_controller` with an experimental child-task replacement
  for the controller-side `MAPPDKCALL` routine and review/deployment guidance.

## 0.8.0 - 2026-08-24

- Added typed numeric-register get/set services.
- Added RDO/DOUT get/set services and a configured gripper `SetBool` service.
- Added an instantaneous controller-power service returning watts.
- Added a double-gated, explicitly allowlisted `RunProgram` action for FANUC
  teach-pendant programs.
- Kept raw socket commands and unrestricted system-variable writes private.
- Added the `fanucpy_controller_tools` Python example and complete README usage.
- Published controller utility gates, TP allowlist, and gripper output in
  `DriverStatus`.

## 0.7.0 - 2026-08-21

- Added an opt-in `goal_only` trajectory execution mode that commands exactly
  the final validated MoveIt joint target.
- Kept `stop_at_waypoints` as the default and added a separate
  `allow_goal_only_execution` safety gate.
- Added a configurable per-joint current-to-goal distance cap, defaulting to
  `0.35 rad` for the M-10iA.
- Published the active goal-only gate and distance cap in `DriverStatus`.
- Documented that direct-final execution bypasses MoveIt's collision-checked
  intermediate path.

## 0.6.0 - 2026-08-21

- Added `fanuc_m10ia_moveit_config` with matching mock and real MoveIt 2 modes.
- Added the `manipulator` planning group, KDL kinematics, OMPL planning,
  conservative scaling defaults, and an M-10iA self-collision matrix.
- Made both modes expose
  `/fanuc_arm_controller/follow_joint_trajectory`, allowing student algorithms
  to switch environments without controller-name changes.
- Replaced the locally fabricated visualization default with MoveIt's
  BSD-licensed ROS-Industrial M-10iA CAD description.
- Kept real motion disabled by default in the combined MoveIt launch.

## 0.5.0 - 2026-08-21

- Added `fanucpy_ros2_trajectory_controller` with the standard MoveIt 2
  `control_msgs/action/FollowJointTrajectory` interface.
- Added strict trajectory joint-name, finite-value, model-limit, timing-order,
  waypoint-spacing, start-state, and measured-tolerance validation.
- Added exact-stop joint waypoint execution through the driver's sole fanucpy
  connection, with standard desired/actual/error feedback.
- Added an M-10iA MoveIt controller YAML and conservative 5% default/10% maximum
  joint velocity configuration.
- Documented the current stop-at-waypoints timing and cancellation limitations.

## 0.4.0 - 2026-08-21

- Replaced fixed speed presets with 10/25/50/75/100% presets derived from the
  active robot configuration.
- Added driver limit metadata to the transient-local status interface and made
  keyboard teleop synchronize its limits automatically.
- Set the example M-10iA Cartesian velocity ceiling to 2000 mm/s, with
  explicit manual/site verification warnings.
- Added `fanucpy_ros2_visualization` with live and visualization-only RViz2
  launches and selectable external FANUC xacro descriptions.

## 0.3.0 - 2026-08-21

- Added per-goal Cartesian velocity requests with a driver-enforced maximum.
- Added keyboard velocity controls and 25/50/100/150/250 mm/s presets.

## 0.2.0 - 2026-08-21

- Added the bounded `JogCartesian` ROS 2 action.
- Added a disabled-by-default driver motion gate and conservative motion limits.
- Added the guarded Cartesian keyboard teleop example.
- Raised the configurable translation ceiling to 50 mm and added step presets.
- Added offline motion validation, transport, and keyboard-layout tests.

## 0.1.0 - 2026-08-21

- Created the ROS 2 Humble workspace.
- Added connection lifecycle and automatic reconnect behavior.
- Added standard joint-state and Cartesian-state publishing.
- Added a parameterized bringup launch file.
- Added offline transport and conversion tests.
