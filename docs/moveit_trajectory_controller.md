# MoveIt 2 real-robot trajectory controller

`fanucpy_ros2_trajectory_controller` supplies the standard
`control_msgs/action/FollowJointTrajectory` interface expected by MoveIt 2.
The action server is embedded in `fanucpy_ros2_driver`, so trajectory execution,
keyboard jogging, state polling, and future I/O services share the same fanucpy
connection and command lock.

## Install the standard ROS interface

ROS 2 Humble does not install `control_msgs` in every desktop configuration.
Install it once before building:

```bash
sudo apt update
sudo apt install ros-humble-control-msgs
```

A full MoveIt development computer also needs MoveIt 2, its configuration
helper, the exact M-10iA description, and mock-mode ros2_control:

```bash
sudo apt install \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-resources-fanuc-description \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

## Execution contract

The controller provides:

```text
/fanuc_arm_controller/follow_joint_trajectory
  control_msgs/action/FollowJointTrajectory
```

It accepts trajectories containing `joint_1` through `joint_6` in any order,
then reorders them into the driver order. Before accepting a goal it checks:

- the driver motion gate and FANUC connection;
- unique and complete joint names;
- finite positions, velocities, accelerations, and durations;
- model-specific joint position and velocity limits;
- strictly increasing `time_from_start` values;
- the maximum number of trajectory points;
- maximum joint distance between adjacent points;
- agreement between the first point and live `/joint_states`;
- exclusive command ownership against teleop and other trajectories.

Each point is sent as a FANUC joint move with `CNT=0`. The controller reads the
measured joints after each move, publishes standard desired/actual/error action
feedback, checks path or goal position tolerance, and updates `/joint_states`
for RViz and MoveIt.

## Execution modes

The safe default is `stop_at_waypoints`: every validated MoveIt point is sent
as a separate `CNT=0` joint command.

The optional `goal_only` mode sends exactly one physical motion command: the
last joint point in the MoveIt trajectory. The driver still validates joint
names, finite values, model limits, waypoint spacing, trajectory start state,
and the configured current-to-final joint distance. However, the physical move
is direct joint interpolation and does not follow the intermediate path that
MoveIt collision-checked. It can therefore collide with unmodelled or modelled
objects even when the displayed MoveIt trajectory is collision-free.

Goal-only requires all three deliberate selections:

```text
enable_motion_commands:=true
trajectory_execution_mode:=goal_only
allow_goal_only_execution:=true
```

The M-10iA default additionally rejects a direct move when any joint differs
from its live position by more than `0.35 rad` (about 20 degrees).

## Important protocol limitations

The current fanucpy/MAPPDK protocol exposes blocking point moves, not buffered
timed trajectories. Therefore:

- every planned waypoint is retained, but the robot stops at every waypoint;
- `time_from_start` selects an approximate FANUC speed percentage but cannot be
  reproduced accurately;
- non-zero trajectory header timestamps are rejected;
- per-goal and component tolerances are rejected in favor of configured driver
  tolerances;
- multi-DOF trajectories are rejected;
- action cancellation is rejected because MAPPDK provides no dependable remote
  motion abort. Use teach-pendant HOLD or the emergency stop.

The first bullet applies to `stop_at_waypoints`; `goal_only` intentionally
skips those intermediate points and consequently gives up their planned path.

The supplied MoveIt controller YAML disables execution-duration monitoring so
MoveIt does not cancel a safe but slower exact-stop execution. Controller-side
trajectory buffering is required before enabling accurate trajectory timing or
continuous blending.

## Build and start the real controller

```bash
source /opt/ros/humble/setup.bash
cd ~/fanucpy_ros2

colcon build --symlink-install
source install/setup.bash
```

Start the physical robot connection with motion deliberately enabled:

```bash
# Replace this address if your controller uses a different one.
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=192.168.0.177 \
  enable_motion_commands:=true \
  joint_velocity_percent:=5 \
  max_joint_velocity_percent:=10
```

Verify the standard interface without sending motion:

```bash
ros2 action list -t | grep fanuc_arm_controller
ros2 topic echo /fanuc/driver_status --once
```

Expected action:

```text
/fanuc_arm_controller/follow_joint_trajectory
  [control_msgs/action/FollowJointTrajectory]
```

## Use the supplied M-10iA MoveIt configuration

`fanuc_m10ia_moveit_config` already connects MoveIt to this action and provides
matching `mode:=mock` and `mode:=real` launches. Follow
`docs/moveit2_quickstart.md` for the complete procedure.

## Connect another model-specific MoveIt configuration

The installed controller file is:

```text
$(ros2 pkg prefix fanucpy_ros2_trajectory_controller)/share/
  fanucpy_ros2_trajectory_controller/config/moveit_controllers.yaml
```

Use its contents as the `moveit_controllers.yaml` in the model-specific MoveIt
configuration. The MoveIt model must use exactly:

- planning group: `manipulator`;
- base frame: `base_link`;
- tool frame: `tool0`;
- joints: `joint_1` through `joint_6`;
- the same joint signs, zero positions, limits, and robot geometry as the real
  mechanical unit.

The student algorithm continues to call MoveIt normally. Only the launch-time
controller changes between simulation and the real robot.

## Parameters

| Parameter | M-10iA default | Purpose |
| --- | --- | --- |
| `trajectory_action_name` | `/fanuc_arm_controller/follow_joint_trajectory` | Standard action path used by MoveIt |
| `max_trajectory_points` | `500` | Upper bound on accepted waypoint count |
| `max_joint_step_rad` | `0.35` | Maximum per-joint distance between adjacent points |
| `trajectory_execution_mode` | `stop_at_waypoints` | Safe default or `goal_only` direct-final mode |
| `allow_goal_only_execution` | `false` | Required second gate for `goal_only` |
| `goal_only_max_joint_delta_rad` | `0.35` | Maximum direct current-to-final distance per joint |
| `trajectory_start_tolerance_rad` | `0.05` | First-point agreement with current state |
| `trajectory_path_tolerance_rad` | `0.05` | Measured intermediate waypoint tolerance |
| `trajectory_goal_tolerance_rad` | `0.02` | Measured final waypoint tolerance |
| `joint_velocity_percent` | `5` | Fallback FANUC joint-speed percentage |
| `max_joint_velocity_percent` | `10` | Hard trajectory speed cap |
| `joint_acceleration_percent` | `20` | FANUC joint acceleration percentage |

The lower, upper, and velocity limit arrays are stored in the selected robot
YAML. Never use the M-10iA arrays for a different FANUC model.
