# fanucpy_ros2


https://github.com/user-attachments/assets/00c2d331-1b5b-4cf0-913a-09fb140dd65a


`fanucpy_ros2` is a student-friendly ROS 2 integration layer for
[`fanucpy`](https://github.com/torayeff/fanucpy). It gives ROS 2 nodes a single,
well-defined connection to a FANUC robot controller and exposes robot state
using standard ROS messages.

This package is made especially for FANUC controllers that do not provide a
ROS 2-compatible interface. It helps students control a physical robot while
working in a familiar ROS 2 environment.

This repository is a multi-package ROS 2 workspace. Its packages use lowercase
names following ROS package conventions.

## Status

The current release provides:

- one process that owns the `fanucpy` TCP connection;
- automatic reconnect after communication failures;
- `/fanuc/driver_status` connection-state messages;
- `/joint_states` in radians;
- `/fanuc/cartesian_state` in native FANUC millimetres and WPR degrees;
- `/fanuc/cartesian_pose` as a stamped SI-unit ROS pose;
- an explicitly enabled, bounded Cartesian jog action;
- a guarded keyboard teleop example;
- ROS services for numeric registers, RDO/DOUT, gripper, and power;
- a double-gated, allowlisted TP-program action;
- one controller-utilities example covering every new interface;
- robot-specific velocity ceilings shared automatically with teleop;
- model-selectable live RViz2 visualization using realistic mesh descriptions;
- a MoveIt-compatible standard `FollowJointTrajectory` real-robot bridge;
- matching mock and real M-10iA MoveIt 2 environments;
- a versioned MAPPDK controller bundle with integrity checks and provenance;
- the BSD-licensed ROS-Industrial/MoveIt M-10iA CAD model as the default;
- one bringup launch file and one YAML configuration file.

C++ examples will be added incrementally on top of the language-neutral
interfaces generated in this release.

## Scope and official alternative

This community project is intended for teaching and for existing ROS 2 Humble
installations that communicate through `fanucpy` and the controller-side
MAPPDK socket server. It is not the official FANUC ROS 2 driver and does not
provide a high-bandwidth streaming `ros2_control` hardware interface.

For a new deployment, also evaluate FANUC Corporation's official
[`fanuc_driver`](https://github.com/FANUC-CORPORATION/fanuc_driver). Its main
branch targets newer ROS 2 releases and the project provides a Humble branch.
Choose the driver whose controller options, ROS release, performance, support,
and workcell safety requirements match the installation.

## Safety

This software is not a safety system. It must not replace the teach pendant,
emergency stop, safety fence, DCS, controller limits, or a qualified risk
assessment. Perform first tests in a controlled area, at low override, with a
trained operator ready to stop the robot.

The project is not affiliated with or endorsed by FANUC Corporation. FANUC is
a trademark of its respective owner.

## Controller requirements

The controller-side MAPPDK software must be installed and running. A complete,
versioned copy of the bundle supplied by the project author is now installed by
`fanucpy_ros2_controller`; it is never transferred to a robot automatically.
The upstream setup requires:

- R632 - KAREL;
- R648 - User Socket Messaging;
- server tag S8 configured on TCP port `18735` by default.

MAPPDK reserves UFRAME 8, UTOOL 8, R[81-83], and PR[81]. Do not reuse these
resources in student programs. See
[`docs/controller_setup.md`](docs/controller_setup.md) for bundle verification,
installation, compatibility warnings, and the complete pre-flight checklist.

## Host requirements

- Ubuntu 22.04
- ROS 2 Humble
- system Python 3.10
- `fanucpy` 0.1.14 or a compatible 0.1.x release

Do not run this workspace with the currently configured Conda Python 3.13.
ROS 2 Humble's `rclpy` extension is built for Python 3.10.

### Source-controlled fanucpy extension

The driver's
[`robot.py`](src/fanucpy_ros2_driver/fanucpy_ros2_driver/robot.py) subclasses
the installed `fanucpy.Robot` and contains the project-specific, validated
numeric-register support. It also retains the supplied six-axis position-
register methods after correcting their validation, typing, response handling,
and command-forwarding bug. Students do not need to edit or replace
`site-packages/fanucpy/robot.py`; the ROS driver loads this compatibility class
automatically while retaining upstream fanucpy for all other operations.

These extension methods require matching commands in the controller's compiled
MAPPDK server. The controller source supplied in `fanucpy.zip` is an unmodified
upstream baseline and does not contain the numeric- or position-register command
handlers. Keep a working extended controller binary when those services are
required; see the controller setup guide before replacing any `.PC` file.

## Build

Open a terminal outside Conda, or run `conda deactivate` until the prompt no
longer shows a Conda environment. Then:

```bash
source /opt/ros/humble/setup.bash
cd ~/fanucpy_ros2

/usr/bin/python3 -m pip show fanucpy
# Only when the previous command reports that fanucpy is missing:
/usr/bin/python3 -m pip install --user "fanucpy>=0.1.14,<0.2"

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

On a machine where Conda remains first on `PATH`, use this equivalent build
command to guarantee compatibility with ROS 2 Humble:

```bash
PATH=/usr/bin:/bin /usr/bin/colcon build \
  --symlink-install \
  --cmake-clean-cache
```

## Offline verification

These tests use a fake robot object and never open a controller connection:

```bash
source /opt/ros/humble/setup.bash
cd ~/fanucpy_ros2
source install/setup.bash

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PATH=/usr/bin:/bin \
  /usr/bin/colcon test --packages-select fanucpy_ros2_driver \
  --return-code-on-test-failure
PATH=/usr/bin:/bin /usr/bin/colcon test-result --verbose
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` isolates the workspace from unrelated
user-installed pytest plugins. It does not disable any project test.

## Bring up the robot connection

The tested M-10iA controller address is `192.168.0.177`, and the commands below
use that address. Replace it when your controller uses a different address.
The checked-in configuration uses the tested address. Students using another
controller must replace it in their site configuration or pass `robot_ip`:

```bash
source /opt/ros/humble/setup.bash
source ~/fanucpy_ros2/install/setup.bash

ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=192.168.0.177
```

Successful startup includes a log similar to:

```text
Connected to FANUC controller at <controller-address>:18735
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/fanucpy_ros2/install/setup.bash

ros2 topic echo /fanuc/driver_status --once
ros2 topic echo /joint_states --once
ros2 topic echo /fanuc/cartesian_state --once
ros2 topic echo /fanuc/cartesian_pose --once
```

For a quick topic-rate check:

```bash
ros2 topic hz /joint_states
```

## Registers, I/O, power, gripper, and TP programs

Students should not import `fanucpy` or create a second TCP connection. The
driver exposes these controller functions as normal ROS 2 services and an
action:

| Interface | Type | Purpose |
| --- | --- | --- |
| `/fanuc/get_numeric_register` | `GetNumericRegister` service | Read R[1..999] |
| `/fanuc/set_numeric_register` | `SetNumericRegister` service | Write a typed integer/float register |
| `/fanuc/get_digital_output` | `GetDigitalOutput` service | Read RDO or DOUT |
| `/fanuc/set_digital_output` | `SetDigitalOutput` service | Write RDO or DOUT |
| `/fanuc/set_gripper` | `std_srvs/SetBool` service | Write configured `ee_do_type`/`ee_do_num` |
| `/fanuc/get_power` | `GetPower` service | Read instantaneous watts |
| `/fanuc/run_program` | `RunProgram` action | Execute one allowlisted TP program |

Read-only first check:

```bash
ros2 run fanucpy_ros2_examples fanucpy_controller_tools power
ros2 run fanucpy_ros2_examples fanucpy_controller_tools get-register 101
ros2 run fanucpy_ros2_examples fanucpy_controller_tools get-output rdo 7
```

Power monitoring is optional. `Power read failed: wrong-command` means the
controller's compiled MAPPDK server does not implement `ins_pwr`; it is not a
valid `0 W` reading. Install one matched MAPPDK controller build or leave this
optional service unused.

Register, output, and gripper writes remain blocked unless bringup includes
`enable_controller_writes:=true`. TP programs require the normal motion gate,
a second program gate, and an explicit allowlist. For a reviewed `HOME_P`
program:

```bash
# Terminal 1
# Replace this address if your controller uses a different one.
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=192.168.0.177 \
  socket_timeout_sec:=60.0 \
  enable_motion_commands:=true \
  enable_program_execution:=true \
  allowed_tp_programs:='["HOME_P"]'

# Terminal 2, after all physical-robot safety checks
ros2 run fanucpy_ros2_examples fanucpy_controller_tools \
  run-program HOME_P
```

Program cancellation and a socket timeout do not physically stop a TP
program. Use teach-pendant HOLD or the emergency stop. The full example,
including write commands, is in `examples/controller_utilities.md`.

After a TP program reports success, the driver atomically recycles its MAPPDK
socket and verifies state before allowing the next command. The ROS node stays
running, so consecutive program actions use a healthy controller session. If
recovery still reports `Connection refused`, or `INTP-310` appears, the KAREL
server itself stopped. Record the full `INTP-310 (PROGRAM, line)` alarm, fix the
named program, then use `FCTN -> ABORT ALL` and run `MAPPDK` again. See
`docs/controller_setup.md` for the checklist.

## Keyboard teleop

Keyboard motion is disabled during normal bringup. Follow the supervised test
procedure in `examples/keyboard_teleop.md`; do not enable motion merely to test
the state connection.

```bash
# Terminal 1: restart bringup with the motion gate explicitly enabled.
# Replace this address if your controller uses a different one.
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=192.168.0.177 \
  enable_motion_commands:=true

# Terminal 2: start the interactive keyboard client.
ros2 run fanucpy_ros2_examples fanucpy_keyboard_teleop
```

The keyboard starts locally disarmed. Press `SPACE` only after checking the
controller frame, low override, work area, and pendant. The default translation
step is 1 mm and the default rotation step is 0.5 degrees. Number keys select
1, 5, 10, 25, or 50 mm translation steps. Keys `6` through `0` select 10%,
25%, 50%, 75%, or 100% of the maximum Cartesian velocity published by the
selected robot configuration. The example M-10iA configuration currently sets
that site-defined ceiling to 2000 mm/s; verify it for the exact robot and
workcell. The controller override still applies.

## Live RViz2 visualization

The visualization package renders the selected robot's mesh-based description
and updates it from the driver's `/joint_states` output. The default is the
BSD-licensed M-10iA model released for ROS 2 by MoveIt:

```bash
source /opt/ros/humble/setup.bash
source ~/fanucpy_ros2/install/setup.bash

ros2 launch fanucpy_ros2_visualization fanuc_live_rviz.launch.py \
  robot_ip:=192.168.0.177
```

Use `fanuc_visualization.launch.py` instead when the driver is already running.
Selection commands for other installed FANUC descriptions are in
`docs/visualization.md`.

## MoveIt 2 execution bridge

MoveIt 2 can send planned joint trajectories to the standard action:

```text
/fanuc_arm_controller/follow_joint_trajectory
```

Install the ROS and MoveIt dependencies before building:

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

The first implementation safely preserves every validated MoveIt waypoint as
an exact-stop FANUC joint move. Because fanucpy/MAPPDK has no buffered timed
trajectory protocol, it does not yet reproduce trajectory timing or continuous
blending. See `docs/moveit_trajectory_controller.md` for the execution contract
and `docs/moveit2_quickstart.md` for the matching mock/real workflow.

An explicitly double-gated `goal_only` mode is also available for bounded
direct joint moves. It sends only the final MoveIt point and therefore does not
retain MoveIt's collision-checked path.

Start the safe mock environment first:

```bash
ros2 launch fanuc_m10ia_moveit_config fanuc_m10ia_moveit.launch.py \
  mode:=mock
```

Start MoveIt with live robot state while physical commands remain blocked:

```bash
ros2 launch fanuc_m10ia_moveit_config fanuc_m10ia_moveit.launch.py \
  mode:=real \
  robot_ip:=192.168.0.177
```

## Workspace packages

| Package | Purpose |
| --- | --- |
| `fanucpy_ros2_interfaces` | Language-neutral ROS messages, services, and actions |
| `fanucpy_ros2_controller` | Versioned MAPPDK controller bundle, integrity manifest, and experimental patch |
| `fanucpy_ros2_driver` | Python connection and state driver; sole socket owner |
| `fanucpy_ros2_bringup` | Launch files and site configuration |
| `fanucpy_ros2_examples` | Guarded student-facing Python examples |
| `fanucpy_ros2_visualization` | Selectable robot description, transforms, and RViz2 |
| `fanucpy_ros2_trajectory_controller` | Standard MoveIt `FollowJointTrajectory` execution bridge |
| `fanuc_m10ia_moveit_config` | Matching M-10iA MoveIt setup for mock and physical operation |

See `docs/architecture.md` before contributing a new command or example.
The complete topic and parameter contract is in `docs/interfaces.md`.

## Author

Muhammad Ureed Hussain<br>
IMViA Laboratory, France
