# fanucpy_ros2_task_planner

This package is an isolated natural-language client for the existing
`fanucpy_ros2` ROS 2 actions. It does not change the driver, controller
protocol, or MoveIt integration.

The deterministic client intentionally supports a small prompt grammar. It
does not use an LLM and never guesses an unsupported instruction. An optional local
Ollama conversation is also available. It can propose validated relative
Cartesian jogs, direct absolute Cartesian targets, bounded absolute joint
targets, and allowlisted TP-program calls. It cannot bypass the deterministic
execution boundary or enable a disabled driver gate.

The Ollama client also accepts short partial absolute targets. `move X 300`
sets X to 300 mm while the ROS 2 client preserves Y/Z/W/P/R from fresh robot
feedback. In contrast, `move X by 10 mm` remains a relative jog.

When the optional `fanucpy_ros2_vision_bridge` is running, the Ollama client
can answer observation questions such as `what do you see?` from fresh exact
detector-label counts on `/fanuc/vision/detections`. It reports both normal and
dangerous categories supplied by the detector. Missing and stale vision are
reported as unavailable, not as an empty scene. It can also answer questions
such as `tell me the coordinates of battery one` using projected `(u, v)`
pixels followed by calibrated eye-to-hand `(X, Y)` millimetres. Object numbers
are assigned in ascending track-ID order for each fresh frame; an explicit
track ID is the unambiguous reference within that frame. Vision questions
cannot produce robot commands. Coordinate selection and numeric formatting are
resolved deterministically from the typed ROS message rather than delegated to
the language model.

For local-model installation, architecture, conversation commands, and test
instructions, see [OLLAMA.md](OLLAMA.md).

## Coordinate mapping

All commands are relative offsets in the driver's configured active FANUC user
frame, which defaults to `fanuc_world`.

| Prompt term | Jog component |
| --- | --- |
| `left` / `right` | `-X` / `+X` |
| `backward` / `forward` | `-Y` / `+Y` |
| `down` / `up` | `-Z` / `+Z` |
| `roll`, `W`, or `X` | `W` rotation |
| `pitch`, `P`, or `Y` | `P` rotation |
| `yaw`, `R`, or `Z` | `R` rotation |

Positive rotation follows the controller W/P/R convention. `clockwise` is
mapped to a negative rotation and `counterclockwise` to a positive rotation.
When no axis is included, clockwise/counterclockwise uses yaw/R. Always review
the printed W/P/R offset because rotational viewpoint terminology is
inherently ambiguous.

## Supported prompts

Translation defaults to 10 mm when no distance is provided. Rotation defaults
to 1 degree. The defaults and dry-run limits are configurable in
`config/prompt_control.yaml`.

Examples:

```text
move left
move right 5 mm
move up 1 cm and forward 10 mm
move down 5 mm and rotate pitch negative 1 degree
rotate yaw positive 1 degree
rotate clockwise 0.5 degrees around z
move left 10 mm at 20 mm/s
```

A prompt may command each X/Y/Z/W/P/R component at most once. Unknown words,
negative distances, unsupported units, duplicate axes, and movements above the
configured limit are rejected as a complete command.

## Dry-run test

Build and source the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/Ros2_Fanucpy
colcon build --symlink-install --packages-up-to fanucpy_ros2_task_planner
source install/setup.bash
```

Parse and preview a prompt without contacting or moving the robot:

```bash
ros2 run fanucpy_ros2_task_planner fanucpy_prompt \
  "move left 10 mm and up 5 mm at 20 mm/s"
```

Dry-run mode is the default. It prints the exact `[dX dY dZ dW dP dR]` goal and
does not wait for a driver.

To load a different local configuration:

```bash
ros2 run fanucpy_ros2_task_planner fanucpy_prompt \
  "rotate yaw positive 0.5 degrees" \
  --ros-args \
  --params-file install/fanucpy_ros2_task_planner/share/fanucpy_ros2_task_planner/config/prompt_control.yaml
```

## Supervised real execution

Industrial robot motion can cause serious injury or equipment damage. Verify
the active user frame, tool frame, robot model, software limits, controller
speed, physical workspace, and emergency-stop access before enabling motion.

Start the existing driver in a separate terminal with its motion gate enabled:

```bash
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=CONTROLLER_IP_OR_HOSTNAME \
  enable_motion_commands:=true
```

Then request one motion:

```bash
ros2 run fanucpy_ros2_task_planner fanucpy_prompt \
  --execute "move left 5 mm at 10 mm/s"
```

Before sending anything, the client:

1. Parses and prints the exact Cartesian offset.
2. Waits for a connected `/fanuc/driver_status` message.
3. Verifies that the driver's motion gate is enabled.
4. Checks the plan against the limits reported by the live driver.
5. Verifies that `/fanuc/jog_cartesian` is available.
6. Requires the operator to type the exact word `EXECUTE`.

Terminal interruption does not guarantee that controller motion stops. Use the
teach-pendant HOLD or emergency stop whenever active motion must be stopped.
