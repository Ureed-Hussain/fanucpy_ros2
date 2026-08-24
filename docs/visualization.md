# Live RViz2 visualization

`fanucpy_ros2_visualization` displays a model-specific FANUC description and
updates it from the driver's standard `/joint_states` topic. It never connects
to the controller itself, so `fanucpy_ros2_driver` remains the sole owner of
the MAPPDK socket.

## Model requirement

A realistic view requires the exact robot's mesh-based URDF/xacro description.
There is no geometrically accurate generic model for every FANUC arm. The
visualization launch is therefore model-selectable: supply any installed ROS 2
description package whose moving joints are named `joint_1` through `joint_6`.

The default uses the M-10iA model released with MoveIt's ROS 2 resources. It
was adapted from the ROS-Industrial support package and includes CAD visual and
collision meshes:

| Robot | Description package | Description file |
| --- | --- | --- |
| FANUC M-10iA | `moveit_resources_fanuc_description` | `urdf/fanuc.urdf` |

Install it once, then source ROS and this workspace:

```bash
sudo apt install ros-humble-moveit-resources-fanuc-description
```

```bash
source /opt/ros/humble/setup.bash
source ~/fanucpy_ros2/install/setup.bash
```

## Start connection and RViz together

Stop an already running FANUC driver first, because only one process may own
the controller connection. Then run:

```bash
ros2 launch fanucpy_ros2_visualization fanuc_live_rviz.launch.py \
  robot_ip:=192.0.2.10  # Replace with your controller address.
```

This starts the driver in state-only mode, `robot_state_publisher`, and RViz2.
The default 10 Hz state polling provides a responsive live display without
opening a second robot connection.

To use keyboard motion, restart the launch with the deliberate motion gate:

```bash
# Replace the documentation address with your controller address.
ros2 launch fanucpy_ros2_visualization fanuc_live_rviz.launch.py \
  robot_ip:=192.0.2.10 \
  enable_motion_commands:=true
```

In another sourced terminal:

```bash
ros2 run fanucpy_ros2_examples fanucpy_keyboard_teleop
```

RViz follows all joint movement published by the driver, whether the motion
originates in keyboard teleop or a later Python/C++ action client.

## Attach RViz to a driver that is already running

Use the visualization-only launch to avoid a second controller connection:

```bash
ros2 launch fanucpy_ros2_visualization fanuc_visualization.launch.py
```

## Select another FANUC model

The same arguments accept another installed ROS 2 FANUC description package:

```bash
ros2 launch fanucpy_ros2_visualization fanuc_visualization.launch.py \
  description_package:=YOUR_DESCRIPTION_PACKAGE \
  description_file:=urdf/YOUR_ROBOT.xacro
```

Before using a new model, verify its joint names, joint signs, zero pose, base
frame, mechanical limits, and mesh license. A realistic-looking mesh alone is
not proof that its kinematics match the physical robot. The M-10iA resource is
an open-source community model, not a controller-exported or OEM-certified
digital twin; verify it against the exact mechanical unit before real motion.
