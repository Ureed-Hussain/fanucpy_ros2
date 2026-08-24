# Experimental FANUC controller-side program-call patch

This package installs source material for an experimental FANUC controller
patch. It does not install or run controller code automatically. The patch has
not yet been translated with proprietary FANUC tooling or validated on a
controller; treat it as a starting point for controller-side review, not as a
ready-to-deploy binary.

## Why this patch exists

The original fanucpy MAPPDK `MAPPDKCALL` routine uses `CALL_PROGLIN`, which
executes the requested TP program in the current MAPPDK server task. On the
development M-10iA controller, the TP program completes and returns `success`,
but port 18735 then stops listening. ROS logs show a state-read timeout followed
by `Connection refused`.

`karel/mappdk_call_child.kl.inc` replaces that routine with `RUN_TASK` and
`GET_TSK_INFO`. The user TP program runs as a separate child task while the
MAPPDK server remains the parent socket task. The command still blocks until
the child is finished, preserving the ROS `RunProgram` action contract.

## Build requirement

A FANUC KAREL translator is proprietary controller tooling. This Linux
workspace does not contain it and therefore cannot produce
`MAPPDK_SERVER.PC`. Use the KAREL translator supplied with the controller or
ROBOGUIDE on a supported Windows system.

Start from one matching fanucpy controller-source revision. Do not combine a
`.PC` binary from one revision with `.KL` includes from another.

1. Back up the controller and its current `MAPPDK_SERVER.PC`.
2. Replace the complete `MAPPDKCALL` routine in `mappdk_cmd.kl` with
   `karel/mappdk_call_child.kl.inc`.
3. Add `%ENVIRONMENT MULTI` to `mappdk_server.kl` with its other translator
   directives.
4. Translate `mappdk_server.kl` for the exact controller software version.
5. On the pendant, stop motion safely and use `FCTN -> ABORT ALL`.
6. Load the newly generated `MAPPDK_SERVER.PC` and run `MAPPDK` again.

Do not load an unreviewed `.PC` on a production robot. Verify the controller
software version, robot group configuration, UFRAME/UTOOL, UOP mode, and site
risk controls first. `RUN_TASK` is appropriate here because the MAPPDK parent
has `%NOLOCKGROUP`; the TP child uses its own configured motion group.

## No-motion validation

Before calling a motion program, create and allowlist a TP program named
`ROS_NOP` containing only a short `WAIT` instruction. Call it twice and confirm
that `/fanuc/driver_status` remains `CONNECTED`. Only then repeat the test with
an independently reviewed motion program.

The ROS driver also recycles and verifies the TCP session after every TP call.
That host-side recovery remains useful, but it cannot restart an aborted or
stopped KAREL server.
