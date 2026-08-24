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

Later milestones will add richer Cartesian target actions, matching C++
examples, vision-node migration, and a fake MAPPDK server for integration
testing.
