# FANUC MAPPDK controller bundle

This ROS package installs the complete MAPPDK controller bundle supplied in
the project's `fanucpy.zip`, together with review guidance and an optional
experimental TP-program patch. It never transfers files to a controller or
runs controller code automatically.

## Installed contents

- `fanuc_driver/`: matched KAREL/TP sources and the supplied compiled `.PC`
  files;
- `fanuc_driver/SHA256SUMS`: integrity manifest for the ten supplied files;
- `UPSTREAM.md`: exact upstream revision and license provenance;
- `karel/mappdk_call_child.kl.inc`: optional project patch for TP programs that
  stop the parent socket task.

After building and sourcing the workspace, locate the bundle with:

```bash
ros2 pkg prefix --share fanucpy_ros2_controller
```

The controller files are under:

```text
<package-share>/fanuc_driver/
```

Verify a copied or installed bundle before using it:

```bash
cd <package-share>/fanuc_driver
sha256sum --check SHA256SUMS
```

## Controller installation

The ready-to-load upstream set is:

- `mappdk.ls`;
- `mappdk_server.pc`;
- `mappdk_logger.pc`;
- `mappdk_move.ls`;
- `mappdk_movel.ls`.

The `.KL` files are source material. Loading a `.KL` file does not replace an
already compiled `.PC`; translating KAREL requires FANUC tooling compatible
with the exact controller software.

Back up the controller first, verify R632 (KAREL) and R648 (User Socket
Messaging), and install all files as one matched build. Do not mix a `.PC`
binary with sources from another revision. Use `FCTN -> ABORT ALL` before
replacing a running server, then load the reviewed files and run `MAPPDK` from
the teach pendant.

## Register-extension compatibility

The ZIP's modified Python `robot.py` adds these wire commands:

- `setregint`, `setregflt`, and `getreg` for R[];
- `setpr` and `getpr` for six-axis PR[].

Those commands are preserved, validated, and corrected in the ROS driver's
source-controlled `robot.py`. However, the `mappdk_cmd.kl` supplied in the same
ZIP does not implement those command names. A controller running that baseline
server will reply `wrong-command`; a separately extended controller server is
required. Do not replace a working custom `.PC` that already supports register
commands with this baseline binary unless loss of those commands is intended.

The ROS package therefore exposes numeric-register errors honestly instead of
pretending a write succeeded. Position-register methods are retained at the
Python compatibility layer but are not yet exposed as public ROS services.

## Experimental TP-program child-task patch

The upstream `MAPPDKCALL` routine uses `CALL_PROGLIN`, which may execute a TP
program in the server task. On the development M-10iA, the TP program completed
but port 18735 then stopped listening.

`karel/mappdk_call_child.kl.inc` is a review candidate that uses `RUN_TASK` and
`GET_TSK_INFO` so the TP program runs as a separate child. It has not been
translated or validated on a controller in this Linux workspace.

To evaluate it:

1. Replace the complete `MAPPDKCALL` routine in `mappdk_cmd.kl` with the
   include's routine.
2. Add `%ENVIRONMENT MULTI` to `mappdk_server.kl` with the translator
   directives.
3. Translate the complete matched source set with FANUC/ROBOGUIDE tooling for
   the exact controller version.
4. First test an allowlisted no-motion TP program such as `ROS_NOP` twice and
   confirm the ROS driver remains connected.

Do not load an unreviewed `.PC` on a production robot. Verify the controller
version, motion-group configuration, UFRAME/UTOOL, UOP mode, and site risk
controls first.
