# Driver interface reference

The bringup launch places FANUC-specific interfaces in the `/fanuc` namespace.
The standard joint-state topic remains global for compatibility with
`robot_state_publisher` and RViz.

## Published topics

| Topic | Type | Units and purpose |
| --- | --- | --- |
| `/fanuc/driver_status` | `fanucpy_ros2_interfaces/msg/DriverStatus` | Connection lifecycle and last diagnostic message |
| `/joint_states` | `sensor_msgs/msg/JointState` | Six joint positions in radians |
| `/fanuc/cartesian_state` | `fanucpy_ros2_interfaces/msg/CartesianState` | Native controller XYZ in mm and WPR in degrees |
| `/fanuc/cartesian_pose` | `geometry_msgs/msg/PoseStamped` | XYZ in metres and normalized quaternion orientation |

`driver_status` uses reliable, transient-local QoS so a newly started monitor
receives the latest connection state. Its states are `DISCONNECTED` (0),
`CONNECTING` (1), `CONNECTED` (2), and `ERROR` (3). It also publishes the
motion gate and active translation, rotation, default-velocity, and
maximum-velocity limits so clients do not duplicate robot-specific values.
The status also reports the controller-write gate, effective TP-program gate,
allowlist, and configured gripper output.

## Services

| Service | Type | Gate and behavior |
| --- | --- | --- |
| `/fanuc/get_numeric_register` | `fanucpy_ros2_interfaces/srv/GetNumericRegister` | Connected controller; returns integer or float type explicitly |
| `/fanuc/set_numeric_register` | `fanucpy_ros2_interfaces/srv/SetNumericRegister` | Requires `enable_controller_writes` |
| `/fanuc/get_digital_output` | `fanucpy_ros2_interfaces/srv/GetDigitalOutput` | Connected controller; supports RDO=1 or DOUT=2 |
| `/fanuc/set_digital_output` | `fanucpy_ros2_interfaces/srv/SetDigitalOutput` | Requires `enable_controller_writes` |
| `/fanuc/set_gripper` | `std_srvs/srv/SetBool` | Requires `enable_controller_writes`; uses `ee_do_type` and `ee_do_num` |
| `/fanuc/get_power` | `fanucpy_ros2_interfaces/srv/GetPower` | Connected controller; converts controller kW to watts |

Numeric registers are validated in `1..999`; integer writes are signed 32-bit
and floating-point writes must be finite. The supplied MAPPDK parser accepts
single-digit RDO numbers `1..9` and five-digit DOUT numbers `1..99999`.

Write services share the controller-command reservation and are rejected while
MoveIt, teleop, another write, or a TP program is executing.

## Action

`/fanuc/jog_cartesian` uses
`fanucpy_ros2_interfaces/action/JogCartesian`. Each goal contains a relative
XYZ offset in millimetres, a relative WPR offset in degrees, and a requested
linear velocity in mm/s. The driver
reads the live Cartesian pose immediately before executing the offset, runs
the move with `CNT=0` and linear motion, and returns the resulting target.

The action rejects commands when motion is disabled, the robot is disconnected,
another movement is active, the requested frame differs from `frame_id`, or an
offset exceeds a configured per-axis limit. The interface is generated for
both Python and C++ clients.

MAPPDK/fanucpy does not expose a reliable motion-abort operation. ROS action
cancellation is therefore rejected. Use teach-pendant HOLD or the emergency
stop to interrupt active robot motion.

### TP programs

`/fanuc/run_program` uses `fanucpy_ros2_interfaces/action/RunProgram`. A goal is
accepted only when the controller is connected, motion is enabled,
`enable_program_execution` is true, the normalized name appears in
`allowed_tp_programs`, and no other controller command is reserved. Names are
uppercase-normalized and limited to 1..32 letters, digits, or underscores,
beginning with a letter.

MAPPDK offers no dependable remote program abort, so cancellation is rejected.
A socket timeout only stops the host from waiting; it is not a controller HOLD.
By default, the driver atomically replaces the MAPPDK socket after a successful
program call and verifies joint/Cartesian state before allowing another
controller command. This works around controller builds that stop servicing a
socket after `mappdkcall`, while the ROS node and action interface remain live.

### MoveIt joint trajectory

`/fanuc_arm_controller/follow_joint_trajectory` uses the standard
`control_msgs/action/FollowJointTrajectory` interface. It is available from the
driver process and shares the same motion gate, fanucpy connection, and command
reservation as `/fanuc/jog_cartesian`.

The default `stop_at_waypoints` executor validates and executes each planned
joint point using an exact-stop FANUC joint move. The separately gated
`goal_only` executor validates the request but commands only its final joint
target. Goal-only execution is faster, but its direct FANUC move does not
follow MoveIt's collision-checked intermediate path. See
`docs/moveit_trajectory_controller.md` before using either mode on a real
robot.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `robot_ip` | `192.0.2.10` | Documentation address; replace with the controller IPv4 address or hostname |
| `robot_port` | `18735` | MAPPDK server TCP port |
| `robot_model` | `FANUC M-10iA` in the site YAML | Model label passed to fanucpy and status messages |
| `socket_timeout_sec` | `5.0` | Maximum blocking socket-operation time |
| `reconnect_delay_sec` | `2.0` | Wait between reconnection attempts |
| `state_poll_rate_hz` | `5.0` | State sampling frequency |
| `frame_id` | `fanuc_world` | Cartesian message frame label |
| `joint_names` | `joint_1` through `joint_6` | Ordered ROS joint names |
| `ee_do_type` | `RDO` | Output type used by `/fanuc/set_gripper` (`RDO` or `DOUT`) |
| `ee_do_num` | `7` | End-effector output used by `/fanuc/set_gripper` |
| `enable_controller_writes` | `false` | Gate for numeric-register, digital-output, and gripper writes |
| `enable_program_execution` | `false` | Separate TP-program execution gate |
| `allowed_tp_programs` | `[""]` | Explicit TP-program allowlist; empty-string sentinel permits nothing |
| `recycle_connection_after_program` | `true` | Replace and verify the MAPPDK socket after a successful TP program |
| `program_reconnect_timeout_sec` | `15.0` | Maximum time allowed for post-program socket recovery |
| `program_state_probe_timeout_sec` | `5.0` | Temporary socket timeout for the recovery state check |
| `enable_motion_commands` | `false` | Explicit gate for all robot motion actions |
| `max_translation_step_mm` | `50.0` | Maximum absolute jog offset on each XYZ axis |
| `max_rotation_step_deg` | `2.0` | Maximum absolute jog offset on each WPR axis |
| `cartesian_velocity_mm_s` | `25` | Linear jog velocity passed to fanucpy |
| `max_cartesian_velocity_mm_s` | `2000` | Robot/site-specific maximum per-goal velocity accepted by the driver |
| `cartesian_acceleration_percent` | `20` | Jog acceleration percentage |
| `trajectory_action_name` | `/fanuc_arm_controller/follow_joint_trajectory` | Standard MoveIt action name |
| `max_trajectory_points` | `500` | Maximum accepted trajectory waypoint count |
| `max_joint_step_rad` | `0.35` | Maximum adjacent per-joint waypoint distance |
| `trajectory_execution_mode` | `stop_at_waypoints` | `stop_at_waypoints` or explicitly selected `goal_only` |
| `allow_goal_only_execution` | `false` | Second opt-in gate required before direct-final-target motion |
| `goal_only_max_joint_delta_rad` | `0.35` | Maximum current-to-final distance accepted on each joint in `goal_only` |
| `trajectory_start_tolerance_rad` | `0.05` | Required first-point/current-state agreement |
| `trajectory_path_tolerance_rad` | `0.05` | Intermediate measured joint tolerance |
| `trajectory_goal_tolerance_rad` | `0.02` | Final measured joint tolerance |
| `joint_velocity_percent` | `5` | Fallback FANUC joint velocity percentage |
| `max_joint_velocity_percent` | `10` | Driver-enforced trajectory velocity cap |
| `joint_acceleration_percent` | `20` | FANUC joint acceleration percentage |

The driver validates all parameters before starting its worker. Runtime
parameter changes are intentionally not accepted in this milestone; restart
bringup after changing controller connection settings.

`max_cartesian_velocity_mm_s` is configuration, not controller discovery.
fanucpy/MAPPDK does not report a trustworthy model maximum through this API.
Set it from the exact robot manual and the site's approved operating limit;
never reuse the M-10iA value blindly for another model or installation.

## Frame caution

The Cartesian position is whatever coordinate context MAPPDK reports. Confirm
UFRAME 8 and UTOOL 8 on the controller before treating `fanuc_world` as a
calibrated ROS frame. The frame name is a label, not an automatic transform.
