# Architecture

## Connection ownership

Only `fanucpy_ros2_driver` may import `fanucpy` and connect to TCP port 18735.
Vision, calibration, teaching, Python, and C++ applications communicate with
the driver through ROS interfaces.

```text
Python and C++ student nodes
             |
      ROS 2 interfaces
             |
   fanucpy_ros2_driver
       (one TCP owner)
       |           \
       |        /joint_states
       |              |
       |      robot_state_publisher
       |              |
       |             RViz2
       |
      fanucpy / MAPPDK
             |
      FANUC controller
```

## Interface rules

- Topics are used for continuous state streams.
- Services will be used for short register and I/O operations.
- Actions will be used for motion and monitored program execution.
- Native FANUC units are always named explicitly with `_mm` and `_deg`.
- Standard ROS pose interfaces use metres, radians/quaternions, and a frame ID.
- A command is never reported as complete merely because it was accepted.

## Threading

The driver polls state in a background worker so blocking socket reads do not
block the ROS executor. `FanucpyTransport` serializes every controller call.
Future service and action callbacks must use that transport and must never
access the underlying `fanucpy.Robot` object directly.

The visualization package is controller-independent. It consumes only standard
ROS state, so RViz, teleop, Python nodes, and C++ nodes never create additional
fanucpy connections.

## Optional perception boundary

The external `danger_vision` detector remains responsible for the camera,
YOLO checkpoint, tracking, homography, and schema-version-2 JSON output. The
`fanucpy_ros2_vision_bridge` validates each atomic batch and republishes it as
typed `VisionDetectionArray` data.

```text
camera + external danger_vision detector
                 |
       /danger/model/observations
                 |
      fanucpy_ros2_vision_bridge
                 |
       /fanuc/vision/detections
                 |
     exact-label counts + projected
       pixels + calibrated XY
                 |
       Ollama observation answers
```

The observation stream contains both normal and dangerous visual objects and
marks whether projected-pixel, conveyor-plane, and calibrated eye-to-hand XY
geometry is valid. The default 2D affine values reproduce the active
`phase_two.py` laboratory calibration and are configuration parameters rather
than a universal camera-to-robot transform.
The original dangerous-only `/danger/model/detections` stream remains isolated
for existing conveyor-planner consumers. The bridge has no dependency on
`fanucpy`, does not open a controller socket, and cannot command the driver.
Target selection, Z estimation, grasp planning, and physical execution are
deliberately outside this observation stage.

## Unified assistant and guarded workcell workflows

`fanucpy_ros2_assistant` composes the existing task-planner client with fresh
typed vision observations. Ordinary jog, absolute target, joint, TP-program,
status, and vision questions continue through their existing implementations.
An object-directed request such as `go to battery one` uses a deterministic
path instead of asking Ollama to reproduce numeric coordinates:

```text
human object request
        |
fresh typed detection + deterministic object alias
        |
calibrated X/Y + current robot Z/W/P/R
        |
dry-run preview
        |
CLI permission + assistant gate + two driver gates + typed confirmation
        |
new vision/state samples + same track/label/identity/calibration check
        |
/fanuc/move_cartesian
```

The assistant can also compose the existing actions into separately gated,
deterministic workcell workflows. A motion-only pick approaches a selected
object X/Y at configured Z, descends only Z, and retracts only Z, verifying
fresh Cartesian feedback before each later stage. A named drop command sends a
configured absolute pose. Home wording calls a configured, driver-allowlisted
TP program. These workflows do not use Ollama to generate numeric targets.

This stage assumes a stationary scene and has no connection to gripper
control. It is not a moving-object interceptor, path planner, reachability
solver, grasp planner, or collision checker.

## MoveIt execution

`fanucpy_ros2_trajectory_controller` contains the standard action server and
validation logic, but the server is embedded in `fanucpy_ros2_driver`. This is
intentional: MoveIt trajectories and Cartesian teleop share the same motion
reservation and serialized transport rather than creating another controller
socket.

```text
MoveIt move_group
       |
control_msgs/FollowJointTrajectory
       |
fanucpy_ros2_trajectory_controller
       |
fanucpy_ros2_driver transport lock
       |
fanucpy / MAPPDK / physical FANUC
```

`fanuc_m10ia_moveit_config` uses the same planning group, model, joint-state
topic, and action path in mock and real modes. Mock mode supplies the action
with ros2_control `GenericSystem`; real mode supplies it with the embedded
fanucpy action server. Student planning code is therefore environment-neutral.

## Controller utility interfaces

Numeric registers, RDO/DOUT, configured gripper output, and power are exposed
as short ROS services. TP programs use an action because execution may block
for an extended period. Program execution shares the driver's motion
reservation with MoveIt and Cartesian teleop.

Raw `send_cmd`, connection lifecycle, and unrestricted system-variable writes
remain private. This prevents student nodes from bypassing validation or
desynchronizing the driver's sole controller socket.

Later milestones will add matching C++ examples, gripper-aware manipulation,
and a fake MAPPDK server for integration testing.
