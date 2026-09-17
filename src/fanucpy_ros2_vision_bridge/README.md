# fanucpy_ros2_vision_bridge

This optional package converts external perception output into typed
`fanucpy_ros2` messages. It does not load a vision model, connect to the FANUC
controller, or command robot motion.

The first adapter consumes the observation-only schema-version-2 JSON batch
published by the experimental `danger_vision` detector. That batch contains
both dangerous and normal objects that passed its confidence and area filters:

```text
/danger/model/observations           std_msgs/msg/String
                  |
                  v
/fanuc/vision/detections  fanucpy_ros2_interfaces/msg/VisionDetectionArray
/fanuc/vision/status                  std_msgs/msg/String
```

Labels and the `is_danger` classification are copied exactly from the detector
payload. The package therefore works with the class names stored in the
external YOLO checkpoint without bundling that checkpoint or depending on
Ultralytics. The dangerous-only `/danger/model/detections` topic remains
separate for existing conveyor planner consumers.

`geometry_valid` indicates whether conveyor-plane coordinates and dimensions
are available. Visual-only detections remain visible to observation clients,
but their zero coordinate placeholders must never be used as measurements.
`projected_center_valid` independently identifies a measured center in the
projected conveyor image. The bridge also supplies a validity flag for the
calibrated eye-to-hand XY result.

## Coordinate outputs

Each valid object exposes two coordinate representations in this order:

| Representation | Fields | Units |
| --- | --- | --- |
| Projected conveyor image | `projected_u_px`, `projected_v_px` | pixels |
| Eye-to-hand result | `eye_to_hand_x_mm`, `eye_to_hand_y_mm` | millimetres |

The default eye-to-hand parameters reproduce the currently active affine
calibration in the laboratory `danger_vision/phase_two.py`:

```text
[X_robot_mm]   [-0.01645279  -1.00536662] [X_conveyor_mm]   [-170.43887235]
[Y_robot_mm] = [-1.00400079  -0.00456943] [Y_conveyor_mm] + [1309.18504954]
```

For a detector batch whose `finish_edge` is `top`, `Y_conveyor_mm` is
`center_s_m * 1000`. For `bottom`, it is
`(conveyor_length_m - center_s_m) * 1000`. The output frame defaults to
`fanuc_world`, and the calibration is identified as
`phase_two_active_affine` in every array message.

This is a two-dimensional calibration. It does not provide robot Z, tool
orientation, a grasp pose, reachability, or collision validation. Before using
the values in another workcell, replace the six affine values and calibration
name in `config/danger_vision_bridge.yaml` with a calibration measured for
that camera, conveyor, robot base, and active FANUC frame.

The normal/dangerous flag is an application classification supplied by the
external detector, not a safety assessment. In the current laboratory detector,
the default dangerous labels are `battery`, `pile`, `gas-bottle`, and `n2o`;
all other checkpoint classes are normal by default. Review and override that
policy for each workcell.

## Start the bridge

Start the camera and external detector first. For the current laboratory
detector, provide the trained model path to `danger_model_size_node` in the
`vision_ws_1` workspace.

Build and source this workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/Ros2_Fanucpy
colcon build --symlink-install --packages-up-to fanucpy_ros2_vision_bridge
source install/setup.bash
```

Launch the bridge:

```bash
ros2 launch fanucpy_ros2_vision_bridge danger_vision_bridge.launch.py
```

Verify the normalized output:

```bash
ros2 topic echo /fanuc/vision/status --once
ros2 topic echo /fanuc/vision/detections --once
```

The bridge rejects malformed JSON, unsupported schema versions, non-finite
coordinates, invalid confidence values, duplicate track IDs, and oversized
input batches. Empty valid batches are published as an empty detection array;
they are different from unavailable or malformed perception data.

## Current boundary

The bridge publishes observations and a calibrated 2D XY result only. It does
not select a grasp, create a robot target, or invoke any driver command. Those
steps remain outside this observation-only stage.
