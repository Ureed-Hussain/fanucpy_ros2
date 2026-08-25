# Controller pre-flight checklist

Complete this checklist on the teach pendant before launching the ROS driver.

## Software options

Under `MENU -> NEXT -> STATUS -> Version ID -> ORDER FI`, confirm:

- R632 - KAREL;
- R648 - User Socket Messaging.

## Network and server

- The controller and ROS computer are on the same subnet.
- The controller responds to `ping` from the ROS computer.
- Host communication server tag S8 uses protocol `SM`.
- S8 listens on TCP port 18735.
- S8 is in the `STARTED` state.

## MAPPDK

- The MAPPDK `.PC` and `.LS` files are installed.
- The `MAPPDK` teach-pendant program is running.
- UFRAME 8 and UTOOL 8 are configured as expected.
- R[81], R[82], R[83], and PR[81] are reserved for MAPPDK.

Install the controller files as one matched build. A `.KL` source file copied
to a controller does not update an already compiled `.PC` binary, and mixing
files from different MAPPDK revisions can expose different command sets.

### Repository controller bundle

`fanucpy_ros2_controller` installs the complete controller bundle supplied in
the project's `fanucpy.zip`. After building and sourcing the workspace, find it
with:

```bash
ros2 pkg prefix --share fanucpy_ros2_controller
```

Open the returned package directory and verify its `fanuc_driver` subdirectory:

```bash
cd <package-share>/fanuc_driver
sha256sum --check SHA256SUMS
```

The ready-to-load set contains `mappdk.ls`, `mappdk_server.pc`,
`mappdk_logger.pc`, `mappdk_move.ls`, and `mappdk_movel.ls`. The `.KL` files are
the matching sources for review and translation with FANUC tooling. Back up the
controller first, stop the running server with `FCTN -> ABORT ALL`, transfer
the reviewed matched set using the site's approved USB or FTP procedure, and
run `MAPPDK` again.

The supplied bundle is the upstream MAPPDK baseline. Its `mappdk_cmd.kl` does
not implement the project's `setregint`, `setregflt`, `getreg`, `setpr`, or
`getpr` extension commands. Replacing a working custom `MAPPDK_SERVER.PC` with
the baseline binary may remove register support. The ROS driver reports
`wrong-command` when a compiled controller server lacks a requested capability.
Do not infer controller support only from methods present in Python.

## Controller aborts and reconnects

The ROS driver automatically retries after ordinary network loss. For TP
programs it also recycles the completed socket session by default and verifies
state before accepting the next controller command. It cannot restart
`MAPPDK_SERVER` if the KAREL task itself has aborted.

For `INTP-310`, record the complete alarm including `(program, line)`; it means
that the named controller program used an invalid array subscript. If the name
is a user TP program, inspect that program's array or indirect-register access.
If the name is an MAPPDK routine, rebuild and reinstall the complete MAPPDK
controller file set. Then clear the alarm, select `FCTN -> ABORT ALL`, and run
`MAPPDK` again. Once port 18735 is listening, the ROS driver reconnects without
being relaunched.

An `ins_pwr` response of `wrong-command` is a controller capability/version
mismatch, not a valid zero-power measurement. Power monitoring may be left
unused, or the matching MAPPDK source must be translated and its compiled
server installed.

## First ROS test

- Use a low controller override.
- Keep the workspace clear.
- Have a trained operator at the pendant.
- Start with the state-only bringup; it does not send a motion command.
