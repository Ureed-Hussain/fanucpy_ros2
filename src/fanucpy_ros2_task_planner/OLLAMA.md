# Local Ollama conversation

`fanucpy_ollama` adds an optional conversational interface to the deterministic
task planner. It calls a local Ollama `/api/chat` endpoint and requires a JSON
response matching a strict schema. The model cannot send raw socket commands,
change driver parameters, enable motion, or bypass the existing ROS 2 action
validation and driver gates.

The model is used only to interpret rough language. Current position, target
position, live limits, connection state, motion/program gating, confirmation,
and ROS 2 execution remain deterministic.

Object-coordinate questions are also deterministic. The client recognizes the
requested object and coordinate representation directly from the fresh typed
vision message, then prints the measured values without asking the model to
copy or calculate them. This prevents a conversational response from swapping
objects or altering numeric coordinates.

## Data flow

```text
User conversation
       |
Local Ollama model (structured JSON only)
       ^
Fresh labels and coordinates from /fanuc/vision/detections (optional)
       |
Schema, allowlist, and numeric validation
       |
Current/target position preview
       |
Live driver-limit check + typed confirmation
       |
/fanuc/jog_cartesian, /fanuc/move_cartesian, FollowJointTrajectory,
or /fanuc/run_program
       |
Existing fanucpy_ros2 driver
```

The default endpoint is loopback-only at `http://127.0.0.1:11434`. No
controller address is sent to Ollama. The context contains only driver state,
motion/program-gate state, command frame, Cartesian and joint feedback,
configured limits, the TP-program allowlist, and fresh exact-label vision
counts and object records when the optional bridge is active. Each object
record can contain projected pixels and calibrated eye-to-hand XY. Images are
not sent to Ollama.

## Install Ollama and download a model

Follow the official [Ollama Linux installation](https://docs.ollama.com/linux).
The documented installer command is:

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

Download the default local model:

```bash
ollama pull qwen3:8b
```

Start Ollama if it is not already running as a service:

```bash
ollama serve
```

The package uses Python's standard HTTP library. The optional Python `ollama`
package is not required.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Ros2_Fanucpy
colcon build --symlink-install --packages-up-to fanucpy_ros2_task_planner
source install/setup.bash
```

## Start a dry-run conversation

Start the normal state driver. The model can still interpret a relative prompt
without feedback, but live state is required to show current and calculated
target coordinates.

```bash
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=CONTROLLER_IP_OR_HOSTNAME
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Ros2_Fanucpy/install/setup.bash
ros2 run fanucpy_ros2_task_planner fanucpy_ollama
```

Example conversation:

```text
You> please move to the left side at 50 mm at 200 mm/s
Assistant: I interpreted this as a 50 mm movement in negative X at 200 mm/s.
Current [fanuc_world] [X Y Z W P R]: [...]
Offset [dX dY dZ dW dP dR]: [-50.000, ...] [mm, deg]
Calculated target [fanuc_world] [X Y Z W P R]: [...]
DRY RUN: the model plan was validated but not sent.
```

The exact wording of the model response can vary. The numeric plan, current
state, and calculated target are printed independently by the ROS 2 client.

## Supported conversational actions

| User intent | Required prompt information | ROS 2 interface |
| --- | --- | --- |
| Relative Cartesian jog | Direction or X/Y/Z/W/P/R offset | `/fanuc/jog_cartesian` |
| Absolute Cartesian target | One or more explicit X/Y/Z/W/P/R values | `/fanuc/move_cartesian` |
| Absolute joint target | All six J1/J2/J3/J4/J5/J6 values in degrees | `/fanuc_arm_controller/follow_joint_trajectory` |
| TP-program call | Exact name in the live driver allowlist | `/fanuc/run_program` |

## Optional vision questions

The separate `fanucpy_ros2_vision_bridge` package converts the atomic
schema-version-2 JSON batch from `/danger/model/observations` into
`/fanuc/vision/detections`. The observation topic contains both normal and
dangerous visual objects; the original `/danger/model/detections` remains
dangerous-only for existing planner consumers. Start the existing camera,
projection, and `danger_model_size_node` processes first, then launch the
adapter:

```bash
ros2 launch fanucpy_ros2_vision_bridge danger_vision_bridge.launch.py
```

Keep the detector publishing continuously. In another terminal, verify that
the normalized batch is current:

```bash
ros2 topic echo /fanuc/vision/status --once
ros2 topic echo /fanuc/vision/detections --once
```

Then ask Ollama naturally:

```text
You> what are you see?
Assistant: I can see two batteries (battery), categorized as dangerous, and
one clear food-grade PET bottle (pet-bottle-clear-food), categorized as normal.
```

You can also ask for one detected object's coordinates:

```text
You> tell me the coordinates of battery one
Assistant: Battery 1 (battery; dangerous; track ID 10). Its projected center is
(u=200.00, v=100.00) px. Its eye-to-hand position is
(X=-475.339, Y=1107.014) mm in fanuc_world, using phase_two_active_affine. This
2D calibration does not provide Z.

You> where is bottle 1?
You> give me only the projected pixel of battery two
You> what are the eye-to-hand coordinates of track 30?
```

The numeric values depend on the latest detector frame.
`battery one`, `battery two`, `object one`, and similar aliases are assigned in
ascending track-ID order and are recomputed on every fresh frame. Bottle class
labels such as `pet-bottle-clear-food` also receive short aliases such as
`bottle one`. Use `track ID N` when you need an unambiguous object reference
within the displayed frame.

The count-answer wording can vary, but every dangerous and normal count and each
parenthesized class name must come from the latest detector batch. The client
distinguishes fresh empty, stale, malformed, and unavailable perception.
Vision observations use
`action="none"`; a model response that tries to turn an explicit vision
question into motion is rejected locally. Images and object coordinates are
not converted into commands, and this phase does not implement pick-and-place.
If either coordinate validity flag is false, the assistant reports that result
as unavailable rather than displaying its numeric zero placeholder. The
eye-to-hand affine supplies X and Y only; it does not supply Z or orientation.
The displayed category is detector configuration, not a safety determination.

Examples:

```text
You> move X 300
You> set X to -350 and Z to -190 at 100 mm/s
You> move to X -350 Y 800 Z -190 W -179 P 60 R -175 at 100 mm/s
You> move joints to J1 5 J2 10 J3 -20 J4 0 J5 15 J6 0 at 5 percent
You> run TP program HOME_P
```

Absolute Cartesian commands use a dedicated direct controller action. The
relative jog translation/rotation limits do not apply, and the client does not
divide a target into multiple moves. The separate
`enable_absolute_cartesian_commands` driver gate must be enabled for execution.
The user may name only the axes that need to change. For example, `move X 300`
means an absolute X target of 300 mm; the client copies Y/Z/W/P/R from fresh
Cartesian feedback immediately before preview and again immediately before
execution. `move X by 10 mm` remains a relative jog. A partial absolute target
cannot execute when fresh Cartesian feedback is unavailable.

Optional operator-configured coordinate bounds can be enabled in the driver,
but they are not a reachability or collision model. The client does not
transform frames, run inverse kinematics, or perform collision checking.

Joint commands are absolute and require all six values. The client validates
joint position limits, maximum current-to-target delta, and the driver-reported
joint-velocity cap. It sends a two-point trajectory containing the measured
start and requested target; the existing trajectory action independently
validates it again.

The task-planner joint limits and `max_joint_command_delta_rad` should match the
corresponding robot/driver configuration. A mismatch cannot bypass the driver:
the trajectory action performs its own final validation and may reject the
goal.

TP program names are normalized using the same syntax as the driver, but a
syntactically valid name is not enough: execution requires an exact match in
the live `allowed_tp_programs` list. The language client never retries a TP
program automatically.

An allowlisted TP program can be interpreted and previewed while the program
gate is disabled. Real execution remains blocked until both
`enable_motion_commands` and `enable_program_execution` are true in the driver.

Useful conversation commands:

```text
/status   show live state without calling Ollama
/clear    clear conversation history
/help     show examples and commands
/quit     exit
```

Conversation history allows follow-ups such as `do the same movement again`,
but every returned movement is independently validated and confirmed.

For one dry-run request without entering the conversation loop:

```bash
ros2 run fanucpy_ros2_task_planner fanucpy_ollama \
  "please move left side at 5 mm at 20 mm/s"
```

## Supervised execution

Industrial robot motion can cause serious injury or equipment damage. Test at
reduced controller speed inside an appropriately safeguarded workspace with
qualified supervision, pendant HOLD, and emergency-stop access.

Start the existing driver with its separate motion gate enabled:

```bash
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=CONTROLLER_IP_OR_HOSTNAME \
  enable_motion_commands:=true
```

Start the conversation with execution permitted:

```bash
ros2 run fanucpy_ros2_task_planner fanucpy_ollama --execute
```

Direct absolute Cartesian execution also requires bringup to include:

```text
enable_absolute_cartesian_commands:=true
```

Every movement still requires the exact typed confirmation `EXECUTE`. The
model cannot provide this confirmation itself.

TP programs use a stronger action-specific confirmation. For `HOME_P`, the
operator must type exactly:

```text
RUN HOME_P
```

To make that program available, both driver gates and the explicit allowlist
must be set when bringup starts:

```bash
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=CONTROLLER_IP_OR_HOSTNAME \
  enable_motion_commands:=true \
  enable_program_execution:=true \
  allowed_tp_programs:='["HOME_P"]'
```

The named TP program is controller-side logic. The model does not create,
inspect, or modify it. Review the TP program on the teach pendant before adding
it to the allowlist.

Start with a small request:

```text
You> please move right by 5 mm at 10 mm/s
```

For rotation, prefer explicit W/P/R meaning:

```text
You> rotate yaw positive by 0.5 degrees
```

Clockwise terminology depends on viewpoint. The model maps unspecified
clockwise rotation to negative yaw/R, but the operator must review the printed
W/P/R plan before confirmation.

## Configuration

Defaults are in `config/ollama_prompt.yaml`. A different installed model can be
selected from the command line:

```bash
ros2 run fanucpy_ros2_task_planner fanucpy_ollama \
  --model qwen3:4b
```

The same value can be supplied as a ROS parameter:

```bash
ros2 run fanucpy_ros2_task_planner fanucpy_ollama \
  --ros-args -p ollama_model:=qwen3:8b
```

Remote model endpoints are rejected by default. They should not be enabled
without reviewing authentication, transport security, privacy, availability,
and the effect of network latency. Regardless of endpoint location, model
output remains untrusted and must pass the local schema and motion validation.

## Current scope

The model may select only one of four bounded actions per prompt: relative
Cartesian jog, absolute Cartesian target, absolute joint target, or an exact
allowlisted TP-program call. Gripper commands, digital I/O writes, object
detection execution, and pick-and-place remain outside this phase. A separate
optional bridge now supplies fresh detector-label counts for observation-only
questions. It may describe fresh projected-pixel and calibrated eye-to-hand XY
observations, but it does not receive Z, orientation, or a grasp pose and must
not invent them. The package deliberately provides no vision-to-motion path.
