# fanucpy_ros2_assistant

This package provides one conversational entry point for the existing
`fanucpy_ros2_task_planner` and the typed observations produced by
`fanucpy_ros2_vision_bridge`.

It also offers local package-specific help and reviewed single-step JSON task
files. These features do not require an Ollama response, a vision model, or
an internet connection. Status needs live ROS feedback; physical execution
still needs the robot, independent gates and typed confirmation.

It retains all existing local-Ollama functions:

- relative Cartesian jogs;
- partial or complete absolute Cartesian targets;
- six-joint absolute targets;
- allowlisted TP-program calls;
- robot status and current-position questions;
- object counts and projected/eye-to-hand coordinate questions.

It also provides deterministic, separately gated workcell workflows:

- `go to battery one` aligns the robot X/Y with a selected visible object while
  preserving fresh current Z and W/P/R;
- `pick battery one` approaches the selected X/Y at Z = -190 mm, descends only
  Z to -238 mm, and retracts only Z to -190 mm;
- `drop it at 200 mm/s` or `go to the drop position` moves to the configured
  X/Y/Z/W/P/R drop pose at the requested or default speed; and
- `go home` calls the configured allowlisted `HOME_P` TP program.

The pick workflow is motion only. It does not close a gripper, determine object
height, or prove that the configured Z values are safe for the installed tool.

## Private vision model not included

The trained checkpoint used for the laboratory demonstration is a company
asset. It is intentionally not stored, distributed, downloaded, or licensed
by this repository. Model-weight file types are excluded by the repository's
`.gitignore` to reduce the risk of accidental publication.

Vision support is therefore an integration boundary, not a bundled detector.
Users may connect their own detector and properly licensed model when it
publishes the documented observation schema, or adapt
`fanucpy_ros2_vision_bridge` to another perception source. All non-vision
assistant functions remain available without a model. A local model path must
be supplied explicitly when the optional external detector is launched.

## Data flow

```text
danger_vision /danger/model/observations
                  |
        fanucpy_ros2_vision_bridge
                  |
        /fanuc/vision/detections
                  |
         fanucpy_ros2_assistant
          |                 |
 deterministic object      existing local-Ollama
 selection + workflows     task interpretation
          |                 |
          +-------- guarded ROS 2 actions --------+
```

Ollama does not copy or calculate visual target coordinates. Object selection,
frame checking, calibration checking, target construction, and numeric
validation are deterministic. This prevents a language-model response from
changing measured X/Y values or selecting a different object.

## Supported object wording

Examples include:

```text
go to battery one
align with the second battery at 25 mm/s
move above bottle 1
position over track ID 12
go to cardboard
go to pet bottle clear food one
pick up battery two at 25 mm/s
grab track ID 12
take the robot to the drop position
drop it at 200 mm/s
could you drop it at a speed of 200 millimeters per second?
go to drop position at max speed
please return the robot home
back home please
```

`battery one`, `bottle one`, and `object one` are current-frame aliases ordered
by track ID. `track ID N` is preferred when an exact tracked object matters.
The requested class must be visible in a fresh normalized batch. Ambiguous,
missing, stale, invalid, or uncalibrated targets are rejected.

Rough pick wording such as `pick`, `pick up`, `grab`, `grasp`, `collect`, and
`retrieve` is recognized. A generic request such as `pick the object` works
only when exactly one object is visible; otherwise the assistant asks for an
object name, number, or track ID.

`drop it`, `drop off the object`, and `go to the drop position` all mean the
saved drop pose. The pronoun does not select a detected object or establish that
the robot is holding anything. These commands do not release an object or
operate a gripper. The assistant explains this interpretation and shows the
exact target before asking for confirmation. No new object selection is needed
for a fixed drop target. `go home`, `back home`, and `home` preview the configured
TP call, rather than guessing a Cartesian home position.

These named workflows are interpreted locally before querying Ollama. Ordinary
robot, vision, and conversational questions still use the existing task planner
where appropriate. The assistant does not accept arbitrary instructions as
equivalent to these workflows: requests for a different drop location, combined
pick/drop/home instructions, or an additional gripper release are rejected.
Ask for each workflow separately. Informational questions such as `where is the
drop position?` only preview, even with `--execute`. Negated workflow requests
send nothing; they are not a robot stop mechanism.

### Speed wording

For object alignment, pick motion, and the saved drop pose:

- `at 200 mm/s`, `at 200mm/s`, and
  `at a speed of 200 millimeters per second` specify the commanded speed;
- `at max speed`, `at maximum speed`, and `at full speed` use the connected
  driver's configured `max_cartesian_velocity_mm_s`, not an inferred physical
  maximum. A newly received Cartesian sample and a connected driver status with
  a positive speed limit are required. The numeric speed is shown in the
  preview. Controller limits and the pendant override still apply;
- omitting speed uses `default_vision_velocity_mm_s` for alignment/pick or
  `drop_velocity_mm_s` for drop (both default to 25 mm/s). Speed is not carried
  over from the previous prompt;
- speeds must be whole mm/s values within the allowed range. Conflicting speeds
  and imprecise modifiers such as `faster` or `slowly` are rejected instead of
  silently changing or guessing the requested speed.

Home calls a TP program, whose motion and speed are defined on the controller.
`go home at 200 mm/s` is therefore rejected with an explanation, not silently
treated as a speed-controlled TP call. Use `go home` after reviewing the program.

Object-relative offsets, object-height estimation, orientation generation,
moving-conveyor interception, grasp verification, and collision planning are
not implemented.

## Package help

The assistant can explain this package without searching manuals or opening
your files. This first version is a compact, reviewed guide, not an arbitrary
code debugger or official FANUC support service. It covers connection errors,
vision availability, units/frames, execution gates, TP calls, motion-only pick,
Ollama configuration, build/source issues and saved tasks.

```text
You> hello
You> why vision not working?
You> how do I fix it?
You> explain the coordinate units
You> /ask why does go home fail?
You> tell me more
```

Recognized help questions run through a read-only handler **before** motion
interpretation, even with `--execute`. `/ask QUESTION` explicitly selects
that handler for any wording, including quoted motion commands. Replies name
the relevant package documentation and distinguish general checks from live
observations. Suggested terminal checks are printed, never executed. `/more`
or `tell me more` gives details for the last help topic. `/clear` clears this
context and model history; it does not change controller state.

Use `/status` for actual feedback. For source-specific problems, provide the
exact command/error and a redacted excerpt; this help mode does not read or
modify local code. The model's conversational scope is also restricted to
this package, but generated explanations are not guaranteed correct.

Simple acknowledgements (`yes`, `do it`, `again`, `run it`) do not repeat robot
commands. Name the task/movement explicitly. `stop` in this chat is **not** a
robot stop operation; use pendant HOLD or emergency stop for physical motion.

## ROS example programs

`/examples` lists reviewed ROS executables and launch files that the assistant
can open in its current terminal. This is separate from `/tasks`: tasks contain
one bounded robot action, while a program such as keyboard teleop or MoveIt
keeps running and has its own user interface.

The installed registry provides:

| Name | Fixed program |
| --- | --- |
| `teleop` | `ros2 run fanucpy_ros2_examples fanucpy_keyboard_teleop` |
| `moveit2` | A fixed real-mode M-10iA MoveIt wrapper that reuses the existing driver |

```text
You> /examples
You> preview teleop example
You> run teleop example
You> please run the keyboard teleop
You> run moveit2 example
You> run movite2 example
```

The common `movite`/`movite2` spelling is an explicit alias. There is no fuzzy
model-generated command. Prompt-time command-line arguments, changed MoveIt
mode, chained programs and background execution are rejected.

Preview works without `--execute` and starts nothing. Opening a program needs
all of the following:

1. assistant CLI `--execute`;
2. the exact canonical name in `allowed_examples`;
3. a connected existing driver with new Cartesian feedback;
4. no known conflicting teleop/MoveIt/mock nodes; and
5. the exact `LAUNCH teleop` or `LAUNCH moveit2` confirmation.

The child program takes over the terminal. The assistant cannot accept another
prompt until it exits. In teleop, press `SPACE` to arm/disarm future key
commands and `Q` to exit after the current jog. In MoveIt, closing RViz does
not necessarily close every launch process; press `Ctrl+C` in this terminal
after robot motion has stopped to close the MoveIt launch and return to the
assistant. A terminal interrupt is not a controller stop.

MoveIt uses `fanuc_m10ia_moveit_existing_driver.launch.py`, which fixes
`mode:=real`, `start_driver:=false`, and `use_rviz:=true`. The wrapper cannot be
overridden from the prompt. It therefore does not start a second FANUC driver
or mock joint-state publisher. The assistant also checks for exactly one
`fanucpy_driver`, one `/joint_states` publisher from that driver, fresh joint
feedback, and common conflicting node names before and after confirmation.
These graph checks reduce accidental duplicates but are not a safety lock.

The reviewed registry is
`fanucpy_ros2_assistant/config/examples.json`. A custom
`example_registry_file` can list more fixed `ros2 run`/`ros2 launch` entries;
each requires a unique name/alias, literal package/target/arguments and an
`allowed_examples` entry. The complete registry is validated and snapshotted
at startup, and its path/hash are displayed before launch. No shell is used,
and the language model cannot add an executable or argument. A locally edited
registry is trusted configuration, so review it like code and restart the
assistant after changing it.

Each registry entry uses this shape:

```json
{
  "name": "my_example",
  "aliases": ["my example"],
  "description": "Explain exactly what this reviewed program starts.",
  "kind": "run",
  "package": "my_ros_package",
  "target": "my_installed_executable",
  "arguments": [],
  "requires_driver": true,
  "requires_joint_states": false,
  "conflict_nodes": ["another_exclusive_node"]
}
```

Use `kind: "launch"` with an installed launch filename for launch files.
Arguments are fixed strings in the registry and cannot be added by a prompt.
`requires_driver` requires a connected existing `fanucpy_driver` and fresh
Cartesian feedback. `requires_joint_states` additionally requires fresh joint
feedback and exactly one `/joint_states` publisher from that driver. List every
known mutually exclusive node in `conflict_nodes`; this check is best-effort,
so the example itself must still reject unsafe or duplicate operation.

## Named task files

An example file is **declarative JSON**, not an executable Python/shell file.
The assistant translates its validated fields into an existing guarded ROS
action. This task feature never starts a subprocess, dynamically loads code,
or accepts model-generated shell commands. The separately allowlisted ROS
program registry above can open only its literal reviewed entries; arbitrary
`.py`/shell files are not accepted by either feature.

The installed `tasks/` directory contains:

| Identifier | Operation |
| --- | --- |
| `robot_status` | Read current robot/vision status; no motion |
| `move_left_demo` | One relative -X jog, 5 mm at 20 mm/s, in `fanuc_world` |
| `home_demo` | One explicit call to `HOME_P`, subject to the driver TP allowlist |

These are examples, not workcell safety approvals. No saved motion task is
allowlisted for execution by default. `home_demo` uses the exact program in
its file, unlike `go home`, which uses `home_program_name` and its own workflow
gate. A task TP call still requires the driver motion/TP gates and live program
allowlist, plus the task allowlist and interactive confirmation.

```text
You> /tasks
You> preview move_left_demo
You> please run the move left example
You> could you execute the file move_left_demo.json for me?
You> run robot_status
```

`preview` always sends nothing. Without `--execute`, motion requests are dry
runs. With execution permitted, the task must also appear in `allowed_tasks`.
Its exact numeric plan is displayed before the existing typed confirmation;
driver gates, live feedback, limits and TP allowlists still apply. No timeout,
failure or interruption triggers an automatic retry.

Names are matched exactly, or by their unique spaced form (`move_left_demo`
matches `move left example`). Task IDs and numeric values are never spell-
corrected or inferred by the model. Unknown/ambiguous names, arbitrary paths,
negation, chained task requests and prompt-time speed/target overrides are
rejected, not forwarded to Ollama. For example, `run move_left_demo at 200
mm/s` does not silently replace the file's 20 mm/s speed. Use an explicit
ordinary movement request or review/edit the file outside the assistant.

### Task format and personal tasks

To add a reviewed task, create a JSON file in an explicitly chosen private
task directory. `task_directory` replaces, rather than merges with, the
installed examples. `allowed_tasks` is a list of exact identifiers in that
directory. No entire-workspace or home-directory scan is performed.

For example, the bundled `move_left_demo.json` contains:

```json
{
  "schema_version": 1,
  "name": "move_left_demo",
  "description": "One relative -X jog of 5 mm at 20 mm/s; review workcell clearance first.",
  "action": "cartesian_jog",
  "frame_id": "fanuc_world",
  "delta_x_mm": -5.0,
  "delta_y_mm": 0.0,
  "delta_z_mm": 0.0,
  "delta_w_deg": 0.0,
  "delta_p_deg": 0.0,
  "delta_r_deg": 0.0,
  "velocity_mm_s": 20
}
```

All files require `schema_version: 1`, `name`, `description` and `action`.
The lowercase `name` must match the filename without `.json`.

- `status`: no additional fields.
- `cartesian_jog`: all six `delta_*` fields above, `frame_id` and
  `velocity_mm_s`.
- `cartesian_target`: all six `x_mm`, `y_mm`, `z_mm`, `w_deg`, `p_deg`, `r_deg`
  fields, `frame_id` and `velocity_mm_s`. This is one full absolute target;
  no collision planning or relative jog-step limit applies.
- `tp_program`: only the additional uppercase `program_name` field.

Cartesian speed is integer mm/s; zero selects the driver's configured default.
Frames must match the assistant command frame; no TF conversion occurs.
Missing/extra/duplicate fields, nonfinite values, booleans in numeric fields,
multi-step sequences and unsupported actions are rejected. Up to 64 files of
16 KiB each are accepted, with no symlink escape outside the chosen directory.
The catalog is snapshotted at startup, and the source path and SHA-256 are
shown with the preview. Edit and review files, then restart to load changes.
No signature/trust verification is implied by displaying a hash.

### Try help and tasks without hardware

After building and sourcing (below), start only the assistant; do not launch
bringup just to try the guide or previews:

```bash
ros2 run fanucpy_ros2_assistant fanucpy_assistant
```

Try `/ask how do I run a task?`, `/tasks`, and `preview move_left_demo`.
Without a driver, current position is unavailable; a saved relative offset
can still be shown. No action is sent. Camera/detector nodes and Ollama are
not needed for these deterministic paths.

For a supervised motion test, review the example and workcell first. Keep
your existing single driver running with the appropriate motion gate, then
start the assistant with permission for **only** the reviewed example:

```bash
ros2 run fanucpy_ros2_assistant fanucpy_assistant --execute \
  --ros-args -p allowed_tasks:='["move_left_demo"]'
```

Say `please run the move left example`, review the live pose/target, and type
the requested confirmation only if the full movement is safe. Adding
`--params-file` for your existing assistant configuration is supported; keep
that configuration and its workcell-specific frames/limits when relevant.
Do not start a second driver, enable unrelated gates, or assume a 5 mm move
is safe merely because it is small.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Ros2_Fanucpy
colcon build --symlink-install --packages-up-to fanucpy_ros2_assistant
source install/setup.bash
```

## Start the supporting nodes

Start the camera and the external detector first. For the current laboratory
workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/vision_ws_1/install/setup.bash
ros2 run danger_vision danger_model_size_node --ros-args \
  -p model_path:=/absolute/path/to/model.pt
```

The detector and checkpoint are external inputs. This repository does not
contain, download, or license a trained vision model.

The assistant support launch starts the existing FANUC driver and vision
bridge. Every physical command gate defaults to false:

```bash
source /opt/ros/humble/setup.bash
source ~/Ros2_Fanucpy/install/setup.bash
ros2 launch fanucpy_ros2_assistant assistant_support.launch.py \
  robot_ip:=192.168.0.177
```

### Optional two-terminal integration

`assistant_support.launch.py` can also include an external camera launch and
external detector node. Both are disabled by default so that installing this
repository does not create undeclared camera/model dependencies. Source those
external workspaces first and explicitly identify their package, executable,
and local checkpoint:

```bash
ros2 launch fanucpy_ros2_assistant assistant_support.launch.py \
  start_camera:=true \
  camera_package:=flir_launch \
  camera_launch_file:=flir_fast.launch.py \
  start_detector:=true \
  detector_package:=danger_vision \
  detector_executable:=danger_model_size_node \
  detector_model_path:=/absolute/path/to/model.pt \
  robot_ip:=192.168.0.177
```

This one launch owns the camera, detector, single FANUC driver, and vision
bridge. Do not run their individual commands at the same time. The second
terminal remains the interactive assistant because terminal input must stay
attached directly to `fanucpy_assistant`.

## Dry-run conversation

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Ros2_Fanucpy/install/setup.bash
ros2 run fanucpy_ros2_assistant fanucpy_assistant \
  --ros-args \
  --params-file ~/Ros2_Fanucpy/install/fanucpy_ros2_assistant/share/fanucpy_ros2_assistant/config/assistant.yaml
```

Then try:

```text
You> what do you see?
You> tell me the coordinates of battery one
You> go to battery one
You> please pick up battery one at 25 mm/s
You> take it to the drop position
You> drop it at 200 mm/s
You> where is the drop position?
You> please return the robot home
```

Dry-run mode prints exact targets and never sends an action or calls a TP
program. The pick preview shows all three stages. The fixed drop target is
`[237, 720, -167, -179, 60, -175]` in mm/degrees by default, and the home
request resolves to `HOME_P`.

## Supervised physical execution

Industrial robot motion can cause serious injury or equipment damage. Use this
stage only with a stationary conveyor and stationary object, inside an
appropriately safeguarded workcell, at reduced speed, with qualified
supervision and immediate access to pendant HOLD and emergency stop.

Start support bringup with the driver motion gates enabled. Enable and
allowlist TP programs as well if the conversational home command is required:

If support bringup is already running, do not launch a second driver or bridge.
When the robot is stationary, shut down that support launch before restarting
with changed flags. Leave the camera and detector running.

```bash
ros2 launch fanucpy_ros2_assistant assistant_support.launch.py \
  robot_ip:=192.168.0.177 \
  motion_socket_timeout_sec:=60.0 \
  enable_motion_commands:=true \
  enable_absolute_cartesian_commands:=true \
  enable_program_execution:=true \
  allowed_tp_programs:='["HOME_P"]'
```

`motion_socket_timeout_sec` applies only while `fanucpy` waits for a blocking
controller motion to finish. Normal state reads retain the shorter
`socket_timeout_sec`. A host timeout does not stop physical motion, so a timed
out target must never be submitted again automatically.

Start the assistant with CLI execution permission and only the workflow gates
that have been validated for the workcell:

```bash
ros2 run fanucpy_ros2_assistant fanucpy_assistant --execute \
  --ros-args \
  --params-file ~/Ros2_Fanucpy/install/fanucpy_ros2_assistant/share/fanucpy_ros2_assistant/config/assistant.yaml \
  -p enable_vision_guided_motion:=true \
  -p enable_pick_motion_sequence:=true \
  -p enable_drop_target_motion:=true \
  -p enable_home_program_call:=true
```

The workflow parameters are read at assistant startup. If a preview says a gate
is disabled, exit the idle assistant with `/quit` and restart it with that
`-p enable_...:=true` override after `--ros-args` and the parameter file, keeping
the other required options. Do not remove the gate or turn it on permanently in
the default YAML just to suppress the message. First omit `--execute` to check
the interpreted targets and speeds without sending commands.

The confirmation tokens are intentionally specific:

- visual XY alignment: `MOVE TO TRACK N`;
- the complete three-stage pick motion: `PICK TRACK N`;
- the configured drop pose: `MOVE TO DROP`; and
- the home TP call: `RUN HOME_P`.

A visual motion therefore requires:

1. `enable_motion_commands:=true` in the driver;
2. `enable_absolute_cartesian_commands:=true` in the driver;
3. `enable_vision_guided_motion:=true` in the assistant;
4. assistant CLI `--execute`;
5. its exact interactive confirmation.

After confirmation, the assistant requires a new vision batch and a new robot
state. It reacquires the same track ID, label, identity source, frame, and
calibration. If its XY target changed by more than
`max_vision_target_refresh_shift_mm`, execution is blocked and a new prompt is
required.

## Parameters

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `enable_vision_guided_motion` | `false` | Separate physical vision-motion gate |
| `default_vision_velocity_mm_s` | `25` | Velocity when the prompt omits one |
| `max_vision_xy_travel_mm` | `500.0` | Maximum horizontal current-to-target distance |
| `max_vision_target_refresh_shift_mm` | `5.0` | Maximum target change after confirmation |
| `vision_refresh_timeout_sec` | `2.0` | Wait for post-confirmation vision/state |
| `vision_xy_offset_mm` | `[0.0, 0.0]` | Workcell-specific calibrated TCP X/Y offset |
| `enable_pick_motion_sequence` | `false` | Additional gate for approach/down/up motion |
| `pick_approach_z_mm` | `-190.0` | Object approach and alignment Z |
| `pick_descend_z_mm` | `-238.0` | Fixed descent Z |
| `pick_retract_z_mm` | `-190.0` | Fixed retraction Z |
| `max_pick_approach_z_change_mm` | `100.0` | Maximum current-Z to approach-Z change |
| `max_pick_vertical_stage_mm` | `50.0` | Maximum descent or retraction distance |
| `pick_pose_tolerance_mm` | `2.0` | XYZ feedback tolerance before the next stage |
| `pick_orientation_tolerance_deg` | `1.0` | WPR feedback tolerance before the next stage |
| `enable_drop_target_motion` | `false` | Separate gate for the named drop target |
| `drop_target_mm_deg` | `[237, 720, -167, -179, 60, -175]` | Configured drop pose in mm/degrees |
| `drop_velocity_mm_s` | `25` | Drop-target velocity when the prompt omits speed |
| `enable_home_program_call` | `false` | Separate assistant gate for home |
| `home_program_name` | `HOME_P` | Allowlisted TP program called by home wording |

The active bridge calibration supplies only X and Y. The zero default TCP
offset does not prove that the detected object center equals the required tool
center. Measure and validate the offset for the installed tool before physical
testing.

## Safety boundary and limitations

This package is not a safety-certified control system. It does not calculate
reachability, inverse kinematics, singularities, collisions, conveyor motion,
object height, tool clearance, or a safe Cartesian path. The direct targets
are sent through the existing absolute Cartesian action only after all gates
pass. One pick confirmation authorizes three sequential targets; fresh
Cartesian feedback is checked after every stage, and a failed or uncertain
stage prevents later stages from being sent. The object is not re-detected
during descent because it may be occluded, so the calibrated X/Y is locked
immediately before execution and the scene must remain stationary.

Users remain responsible for validating calibration, active frames, tooling,
the configured -190/-238 mm Z values, orientation, speed, robot limits, the
entire swept path, and the physical workspace. A timeout does not stop the
physical robot and must not be followed by automatically repeating a target.
