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
