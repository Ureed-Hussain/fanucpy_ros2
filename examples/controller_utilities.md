# FANUC controller utilities

`fanucpy_controller_tools` is one ROS 2 Python example covering numeric
registers, RDO/DOUT, the configured gripper output, instantaneous power, and
allowlisted TP programs. It never imports `fanucpy` and never opens a second
controller socket.

## Read-only first check

Start normal state-only bringup. Read operations do not require a write or
motion gate:

```bash
# Replace the documentation address with your controller address.
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=192.0.2.10 \
  socket_timeout_sec:=30.0
```

In a sourced second terminal:

```bash
ros2 run fanucpy_ros2_examples fanucpy_controller_tools power
ros2 run fanucpy_ros2_examples fanucpy_controller_tools get-register 101
ros2 run fanucpy_ros2_examples fanucpy_controller_tools get-output rdo 7
ros2 run fanucpy_ros2_examples fanucpy_controller_tools get-output dout 42
```

The power service is optional because it depends on the controller-side
`ins_pwr` MAPPDK command. If it reports that the command is unsupported, do
not interpret the response as `0 W`: the installed `MAPPDK_SERVER.PC` does not
provide that command. Compile and deploy one matching set of MAPPDK controller
files, or leave power monitoring unused. Register and I/O services can still
work when this optional command is absent.

## Register, output, and gripper writes

Writes are rejected unless bringup was explicitly started with:

```text
enable_controller_writes:=true
```

After checking that the chosen registers and outputs are not reserved or used
by another controller program:

```bash
ros2 run fanucpy_ros2_examples fanucpy_controller_tools \
  set-register 101 integer 42
ros2 run fanucpy_ros2_examples fanucpy_controller_tools \
  set-register 102 float 12.5
ros2 run fanucpy_ros2_examples fanucpy_controller_tools \
  set-output rdo 7 true
ros2 run fanucpy_ros2_examples fanucpy_controller_tools gripper false
```

RDO numbers are limited to `1..9` by the supplied MAPPDK KAREL parser. DOUT
numbers use the five-digit MAPPDK field and are validated in `1..99999`.

## TP program execution

A TP program may move the robot or actuate tooling. The driver requires all of
the following before accepting an action goal:

- `enable_motion_commands:=true`;
- `enable_program_execution:=true`;
- the requested name in `allowed_tp_programs`;
- a connected controller;
- no active teleop, MoveIt, utility write, or TP-program command.

For a reviewed program named `HOME_P`:

```bash
# Replace the documentation address with your controller address.
ros2 launch fanucpy_ros2_bringup fanucpy_bringup.launch.py \
  robot_ip:=192.0.2.10 \
  socket_timeout_sec:=60.0 \
  enable_motion_commands:=true \
  enable_program_execution:=true \
  allowed_tp_programs:='["HOME_P"]'
```

Then, only after the normal physical-robot safety checks:

```bash
ros2 run fanucpy_ros2_examples fanucpy_controller_tools \
  run-program HOME_P
```

Program action cancellation is rejected because MAPPDK does not provide a
dependable remote program abort. A socket timeout is also not a physical HOLD.
Use the teach-pendant HOLD or emergency stop when motion must be interrupted.

After a successful TP program, the default
`recycle_connection_after_program:=true` behavior closes only the completed
MAPPDK socket session, reconnects, and verifies live state while retaining the
same ROS node and command lock. A successful command should therefore report:

```text
success; MAPPDK socket session recycled and state verified
```

This supports consecutive `run-program` calls without allowing the polling
thread to send state commands on the stale session. If recovery instead
reports `Connection refused`, or the complete teach-pendant alarm contains
`INTP-310`, the KAREL server itself stopped and cannot be restarted through its
closed socket. Inspect the alarm's program and line shown in parentheses.

- If the alarm names `HOME_P`, fix the invalid array/register indirection in
  that TP program.
- If it names `MAPPDK_SERVER` or another MAPPDK routine, compile and install
  all MAPPDK `.PC` files together from the same source revision. Do not mix an
  older server binary with modified command sources.
- After an abort, clear the fault, use `FCTN -> ABORT ALL`, and start `MAPPDK`
  again. The ROS driver will reconnect automatically once the controller
  server is listening, but it cannot remotely restart an aborted KAREL server
  over the closed socket.

For a line-specific diagnosis, record the entire alarm in the form
`INTP-310 (PROGRAM_NAME, line)` and export the named TP program as an `.LS`
file.

## Direct ROS 2 CLI equivalents

The example is optional. Any Python or C++ ROS client can use the generated
interfaces, and the standard CLI can inspect them:

```bash
ros2 service list -t | grep /fanuc/
ros2 action list -t | grep /fanuc/run_program
ros2 interface show fanucpy_ros2_interfaces/action/RunProgram
```
