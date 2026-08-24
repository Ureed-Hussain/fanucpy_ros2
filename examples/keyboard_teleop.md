# Cartesian keyboard teleop

This example resembles TurtleBot keyboard teleoperation at the user interface,
but it intentionally sends discrete Cartesian steps instead of continuous
velocity. `fanucpy` and MAPPDK expose blocking point moves, not a velocity stream.

## Safety behavior

- Driver motion is disabled by default.
- The keyboard starts disarmed and requires `SPACE` to arm.
- Default steps are 1 mm translation and 0.5 degrees rotation.
- The driver rejects translation components above 50 mm and rotation components
  above 2 degrees with the default configuration.
- Only one action goal can execute at a time.
- Every target is calculated from a fresh controller pose.
- Movement uses linear mode, an operator-selected velocity capped by the
  selected robot configuration, 20% acceleration, and `CNT=0`.

Keyboard disarm, quitting the client, and ROS action cancellation do not stop an
active controller movement. MAPPDK/fanucpy provides no dependable remote abort
for this operation. Keep teach-pendant HOLD and the emergency stop available.

## Supervised first test

1. Put the robot in the operating mode required by your approved lab procedure.
2. Reduce the controller override and clear the safeguarded workspace.
3. Confirm MAPPDK, UFRAME 8, and UTOOL 8 on the teach pendant.
4. Have a trained operator hold the pendant and watch the robot.
5. Stop the existing state-only bringup with `Ctrl-C`.
6. Start motion-enabled bringup in terminal 1:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/fanucpy_ros2/install/setup.bash

   # Replace the documentation address with your controller address.
   ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
     robot_ip:=192.0.2.10 \
     enable_motion_commands:=true
   ```

7. Wait for `Connected to FANUC controller`, then start terminal 2:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/fanucpy_ros2/install/setup.bash

   ros2 run fanucpy_ros2_examples fanucpy_keyboard_teleop
   ```

8. Press `P` and compare XYZ/WPR with the teach pendant.
9. Select a direction that the pendant operator has confirmed is clear.
10. Press `SPACE` to arm, then press that movement key once.
11. Compare the new pose with the expected 1 mm or 0.5 degree change.
12. Press `SPACE` again to disarm before discussing or changing settings.

Do not select the 50 mm preset for the first supervised movement. Increase the
step only after the operator has verified the active frame and available path.

## Keyboard layout

| Keys | Command in active FANUC user frame |
| --- | --- |
| `W` / `S` | +Y / -Y |
| `A` / `D` | -X / +X |
| `R` / `F` | +Z / -Z |
| `U` / `O` | +W / -W |
| `I` / `K` | +P / -P |
| `J` / `L` | +R / -R |
| `+` / `-` | Increase/decrease translation step |
| `1` / `2` / `3` / `4` / `5` | Select 1 / 5 / 10 / 25 / 50 mm translation |
| `,` / `.` | Decrease/increase velocity by 5% of the configured maximum |
| `6` / `7` / `8` / `9` / `0` | Select 10% / 25% / 50% / 75% / 100% of the configured maximum |
| `]` / `[` | Increase/decrease rotation step |
| `P` | Print controller Cartesian state |
| `SPACE` | Arm/disarm future commands |
| `H` | Print help |
| `Q` | Quit after the current command finishes |

## Selecting an exact startup step

Any initial step up to the configured limit can be supplied as a ROS parameter:

```bash
ros2 run fanucpy_ros2_examples fanucpy_keyboard_teleop \
  --ros-args -p translation_step_mm:=20.0
```

The driver ceiling can be lowered or raised up to 100 mm when bringup starts:

```bash
# Replace the documentation address with your controller address.
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=192.0.2.10 \
  enable_motion_commands:=true \
  max_translation_step_mm:=50.0
```

The teleop learns the active limits from `/fanuc/driver_status`; the driver
remains authoritative and rejects goals above its configured ceiling.

An exact startup velocity can also be selected:

```bash
ros2 run fanucpy_ros2_examples fanucpy_keyboard_teleop \
  --ros-args -p cartesian_velocity_mm_s:=75
```

The physical controller override still scales or limits the resulting robot
speed. The M-10iA site configuration uses a 2000 mm/s ceiling, so key `0`
selects 2000 mm/s and the other speed presets scale below it. Verify this value
against the mechanical manual and the approved site risk assessment. Begin at
25 mm/s and increase only after verifying the motion path.
