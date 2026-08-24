# MoveIt 2: same algorithm in mock and real modes

`fanuc_m10ia_moveit_config` is the model-specific layer between a student
MoveIt application and either a mock M-10iA or the physical fanucpy driver.
The planning API does not change between modes.

| Interface | Mock mode | Real mode |
| --- | --- | --- |
| Robot model | ROS-Industrial/MoveIt M-10iA | Same model |
| Planning group | `manipulator` | `manipulator` |
| Base and tip | `base_link` to `tool0` | Same |
| Joint state | `/joint_states` | `/joint_states` |
| Trajectory action | `/fanuc_arm_controller/follow_joint_trajectory` | Same |
| Backend | ros2_control `GenericSystem` | fanucpy/MAPPDK |

## Install dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-control-msgs \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-resources-fanuc-description \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

The description dependency contains the exact M-10iA mesh and kinematic model
used by this configuration. It is BSD-licensed and released for ROS 2 Humble.
It is an open-source community model, not an OEM-certified digital twin. Check
joint limits, mounting, tool transform, and collision geometry against the
physical workcell.

Build from the system ROS Python environment:

```bash
source /opt/ros/humble/setup.bash
cd ~/fanucpy_ros2

rosdep install --from-paths src --ignore-src -r -y
PATH=/usr/bin:/bin /usr/bin/colcon build --symlink-install
source install/setup.bash
```

## 1. Test the algorithm with mock hardware

```bash
ros2 launch fanuc_m10ia_moveit_config fanuc_m10ia_moveit.launch.py \
  mode:=mock
```

In another sourced terminal, verify the stable interfaces:

```bash
ros2 topic echo /joint_states --once
ros2 action list -t | grep fanuc_arm_controller
ros2 node list | grep move_group
```

The action should be:

```text
/fanuc_arm_controller/follow_joint_trajectory
  [control_msgs/action/FollowJointTrajectory]
```

Use the RViz MotionPlanning panel to select `manipulator`, plan, and execute.
The mock arm should move and publish its new joint state.

## 2. Connect MoveIt to live robot state without motion

Stop mock mode first. Only one launch should own an action of the same name.
Start real mode with its motion gate left at the safe default:

```bash
ros2 launch fanuc_m10ia_moveit_config fanuc_m10ia_moveit.launch.py \
  mode:=real \
  robot_ip:=192.0.2.10  # Replace with your controller address.
```

Verify connection and model tracking:

```bash
ros2 topic echo /fanuc/driver_status --once
ros2 topic echo /joint_states --once
ros2 action list -t | grep fanuc_arm_controller
```

RViz should follow the measured physical joints. Planning is available, but
the driver rejects execution while `motion_commands_enabled` is `false`.

## 3. Supervised physical execution

Do this only after validating the robot model, active UFRAME/UTOOL, workcell
obstacles, trajectory, pendant override, and emergency-stop access. Stop the
state-only launch and restart it deliberately:

```bash
# Replace the documentation address with your controller address.
ros2 launch fanuc_m10ia_moveit_config fanuc_m10ia_moveit.launch.py \
  mode:=real \
  robot_ip:=192.0.2.10 \
  enable_motion_commands:=true \
  joint_velocity_percent:=5 \
  max_joint_velocity_percent:=10
```

Plan first and inspect the full path in RViz before pressing Execute. The
current bridge executes validated trajectories as blocking `CNT=0` joint
moves. It preserves waypoints but does not reproduce MoveIt's timing and cannot
reliably cancel controller motion. Use teach-pendant HOLD or emergency stop.

## Optional direct-final-target mode

For a small, independently verified joint move, `goal_only` commands exactly
the last MoveIt joint point and skips every intermediate command:

```bash
# Replace the documentation address with your controller address.
ros2 launch fanuc_m10ia_moveit_config fanuc_m10ia_moveit.launch.py \
  mode:=real \
  robot_ip:=192.0.2.10 \
  enable_motion_commands:=true \
  trajectory_execution_mode:=goal_only \
  allow_goal_only_execution:=true \
  goal_only_max_joint_delta_rad:=0.35 \
  joint_velocity_percent:=5 \
  max_joint_velocity_percent:=10
```

This does not make MoveIt execute faster along its planned path. It replaces
that path with one direct FANUC joint move to the final configuration. Use it
only when the direct swept motion has been checked against the complete
workcell. The driver rejects targets beyond the configured per-joint delta.

## Use the same student program

A Python or C++ application should continue to target MoveIt's
`manipulator` group. It should not call fanucpy and should not select a separate
controller for real mode. The launch determines which backend serves the same
standard action.

If the workcell has a non-identity base transform, pass the measured mounting
transform in metres and radians:

```bash
ros2 launch fanuc_m10ia_moveit_config fanuc_m10ia_moveit.launch.py \
  mode:=real \
  base_x:=0.0 base_y:=0.0 base_z:=0.0 \
  base_roll:=0.0 base_pitch:=0.0 base_yaw:=0.0
```

Do not estimate these values visually when collision checking depends on them;
use a calibrated workcell transform.
