# FANUC M-10iA MoveIt 2 configuration

This package gives mock and real operation the same MoveIt planning group,
joint names, robot model, and standard trajectory action. Student applications
can therefore switch environments by changing only the launch argument:

```text
mode:=mock  -> ros2_control GenericSystem
mode:=real  -> fanucpy_ros2_driver
```

Both modes provide:

```text
planning group: manipulator
base link:      base_link
tip link:       tool0
action:         /fanuc_arm_controller/follow_joint_trajectory
joint states:   /joint_states
```

The mesh and kinematic description comes from the separately installed,
BSD-licensed `moveit_resources_fanuc_description` ROS 2 package. That model was
adapted from ROS-Industrial's M-10iA support package; this repository does not
copy or relicense its meshes.

Real motion is disabled by default. Read `docs/moveit2_quickstart.md` at the
workspace root before enabling execution.

Real mode also offers a separately gated `goal_only` executor. It commands only
the final MoveIt joint target and does not preserve the collision-checked path;
it is never enabled by default.
