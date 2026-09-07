# Milestone 4 Runbook

Commands below assume your shell already loaded ROS 2 and the local workspace.
Detailed notes live in `docs/milestones/milestone4.md`.

Milestone 4 uses OptiTrack as the default ownship pose source and adds a target
map that predicts short target-drone dropouts. The current camera path is the
ZED 2i through the Stereolabs ZED SDK; moving target prediction is handled by
`target_map_node`.

For the current `cdrone3` OptiTrack setup, raw `RigidBody3` X/Y is rotated 180
degrees by the external pose adapter before any flight, perimeter, tracker,
target-map, or metrics consumer uses it. This is configured with
Milestone 4's default `frame_rpy_rad: [0.0, 0.0, 3.141592653589793]`.
Do not use `rpy_offset_rad` for this correction; that knob is only for body
orientation offset after the source frame has been transformed.

The ESCs are currently detached from the drone. With ESCs detached, use this
runbook for camera/perception/target-map bench validation only. Do not call the
start service unless the vehicle has been intentionally reconfigured for a
supervised flight test; the start service can arm and command takeoff/flight
behavior.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## ZED 2i Preflight

Confirm the ZED 2i is attached over USB and the SDK imports:

```bash
lsusb | rg -i 'stereolabs|zed'
python3 -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
uname -m
lsb_release -a
cat /etc/nv_tegra_release
```

Expected on the current Jetson:

- `lsusb` shows `STEREOLABS ZED 2i`
- Python prints a ZED SDK version, currently validated with `5.4.0`
- architecture is `aarch64`
- Ubuntu is `22.04`
- L4T/JetPack matches the Stereolabs Jetson installer you are using

If `import pyzed.sl` fails, install the ZED SDK before launching Milestone 4:

```bash
sudo apt install zstd
```

Download the Jetson/Linux ZED SDK installer that matches this Jetson's
Ubuntu/L4T/JetPack from:

- https://www.stereolabs.com/developers/release
- https://docs.stereolabs.com/docs/development/zed-sdk/linux

Run the downloaded installer, then rerun:

```bash
python3 -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
```

Keep the Python runtime compatible with ROS, PyTorch, TensorRT, Norfair, and
ZED. The ZED 5.x wheel declares `numpy>=2`, but this Jetson's ROS/PyTorch stack
needs NumPy 1.x. Use the no-dependency ZED install path that has been bench
tested with the ZED 2i:

```bash
python3 -m pip install --user --force-reinstall numpy==1.26.4 sympy==1.13.1
python3 -m pip install --user --force-reinstall --no-deps \
  /home/jetson/Downloads/zed_sdk/wheels/pyzed-5.4-cp310-cp310-linux_aarch64.whl
```

If you need Python virtual environments for future isolated probes:

```bash
sudo apt update
sudo apt install -y python3.10-venv
```

## ZED 2i Image Smoke Test

Do this before launching the tracker or any flight behavior:

```bash
cd /home/jetson/cdrone_control
python3 scripts/capture_zed2i_smoke.py \
  --rotate-180 \
  --output-dir /home/jetson/output_dump/zed2i_smoke
```

Expected outputs:

- `/home/jetson/output_dump/zed2i_smoke/rgb.png`
- `/home/jetson/output_dump/zed2i_smoke/depth.png`
- `/home/jetson/output_dump/zed2i_smoke/metadata.json`

If this fails with `ZED SDK Python import failed`, fix the SDK install before
continuing. Milestone 4 should not be flight-tested until the smoke test saves
a valid RGB image, depth visualization, and metadata file.

The ZED 2i is currently mounted upside down on cdrone3. Milestone 4 uses
`zed_rotate_180: true` in `zed2i_tracking.yaml`, which rotates both RGB and
depth frames before detection/depth lookup and rotates the camera intrinsics
principal point before body/world projection. Keep the smoke test's
`--rotate-180` option aligned with that runtime config.

## ZED 2i Floor-Mark Depth Ladder

Use this non-flight check when ZED depth looks wrong in Milestone 4. It uses
floor marks as truth, not OptiTrack, and records both raw center depth and
detector-bbox depth with `models/best.engine`.

Keep the target drone stationary at the requested mark. Capture the first range
at `1.2 m`:

```bash
cd /home/jetson/cdrone_control
python3 scripts/capture_zed2i_depth_ladder.py \
  --range-m 1.2 \
  --output-dir /home/jetson/output_dump/zed2i_depth_ladder \
  --rotate-180 \
  --model-path /home/jetson/cdrone_control/models/best.engine
```

If the overlay shows the yellow ROI marker is not on the drone body, add
`--roi-center-x-px <x> --roi-center-y-px <y>` and retake the range. If
`models/best.engine` reports no accepted detector boxes, use the older detector
for bbox-depth diagnosis:

```bash
python3 scripts/capture_zed2i_depth_ladder.py \
  --range-m 4.8 \
  --output-dir /home/jetson/output_dump/zed2i_depth_ladder \
  --rotate-180 \
  --model-path /home/jetson/cdrone_control/models/16_k_and_drone_studio_realsense_images_model.engine \
  --roi-center-x-px 570 \
  --roi-center-y-px 345
```

Repeat for `2.4`, `3.6`, `4.8`, `6.0`, and `7.2` after moving the target drone
to each marked range. Continue by `1.2 m` increments only while the previous
range still has useful detections and depth samples; stop after two consecutive
unusable ranges or at `9.6 m`.

After one or more captures, generate the floor-mark summary:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
zed_depth_ladder_report /home/jetson/output_dump/zed2i_depth_ladder
```

Outputs:

- `/home/jetson/output_dump/zed2i_depth_ladder/<range>/depth_ladder_frames.csv`
- `/home/jetson/output_dump/zed2i_depth_ladder/<range>/summary.json`
- `/home/jetson/output_dump/zed2i_depth_ladder/zed_depth_ladder_summary.csv`
- `/home/jetson/output_dump/zed2i_depth_ladder/zed_depth_ladder_summary.md`

Interpretation:

- If center and bbox median depth are both biased at all ranges, compare ZED
  depth modes before changing tracker math:
  `--depth-mode NEURAL`, `--depth-mode NEURAL_PLUS`, and `--depth-mode QUALITY`.
- If center depth is good but bbox/tracker depth is wrong, tune the detector ROI
  sampling before changing ZED SDK settings.
- If depth is good but body/world range is wrong in tracker-only validation,
  inspect projection, `zed_rotate_180`, and camera-to-body extrinsics.

Before calling the start service, run a tracker-only ZED validation. This opens
the ZED 2i and loads the TensorRT detector, but it does not arm or command the
vehicle:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
timeout 30s ros2 launch drone_bringup tracking_only.launch.py \
  source_mode:=zed_sdk \
  zed_rotate_180:=true \
  tracker_config_file:=/home/jetson/cdrone_control/ros2/install/drone_vision_pkg/share/drone_vision_pkg/config/zed2i_tracking.yaml \
  publish_world_track_compare:=false \
  experiment_tag:=milestone4_tracker_only_preflight
```

Do not run the start service unless this tracker-only launch stays alive until
the timeout and logs `rotate_180=on` without NumPy, Torch, TensorRT, or ZED
exceptions.

## Generate Visualizations

```bash
cd /home/jetson/cdrone_control
python3 scripts/plot_milestone4_prediction_demo.py
```

Outputs:

- `docs/generated/milestone4_target_memory_architecture.svg`
- `docs/generated/milestone4_prediction_path.svg`

Mission recording also saves MP4 visualizations automatically while the demo is
active:

- `mission_recordings/mission_<drone>_<timestamp>_<state>_combined.mp4`
- `mission_recordings/mission_<drone>_<timestamp>_<state>_yolo.mp4`
- `mission_recordings/mission_<drone>_<timestamp>_<state>_target_map.mp4`

Tracking metrics also save CSVs and a cleaned-path MP4 automatically when the
flight/demo exits the active state:

- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/tracking_samples.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/tracking_frames.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/cleaned_target_path.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/tracking_summary.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/cleaned_target_path.mp4`

The cleaned-track postprocess runs by default. It waits up to `5 s` for the
nearest matching `mission_<drone>_*_combined.mp4`, then renders
`cleaned_target_path.mp4` with the same FPS, frame count, and duration as that
combined recording. `cleaned_target_path.csv` stays point-based, but its
`timestamp_s` column is relative and starts from `0.0` instead of using absolute
ROS time.

For Milestone 4, the combined video replaces the old depth-map quadrant with a
normal detector-only YOLO quadrant. The YOLO and combined videos also include a
second annotated tile with target-map burn-in overlays when
`/cdrone/cdrone3/target_map/world_tracks` is live. The target-map MP4 is a
top-down recording of the defense drone path and target-map tracks. The separate
depth MP4 is disabled by default for Milestone 4.

## Launch

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=1 \
  zed_rotate_180:=true \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5 \
  rpy_offset_rad:='[0.0, 0.0, -3.141592653589793]'
```

Milestone 4 defaults to `tracking_source_mode:=zed_sdk`,
`tracker_config_file:=.../zed2i_tracking.yaml`, and
`demo_config:=.../milestone4_demo.yaml`. It opens the ZED 2i directly through
`pyzed.sl`, retrieves left color plus depth frames, and keeps OptiTrack as the
ownship pose source through the corrected external-pose adapter topic
`/cdrone/cdrone3/external_pose/input_pose`. The default ZED config uses `NEURAL` depth, `0.4 m`
minimum depth, and explicit 180 degree frame rotation for the current
upside-down camera mount. The launch default and YAML default both use a
`2.0 m` takeoff altitude unless explicitly overridden.

Before calling the start service, verify the corrected pose is inside the
studio boundary. Raw VRPN may show the unrotated OptiTrack frame; the adapter
output is the pose used downstream:

```bash
ros2 topic echo --once /cdrone/cdrone3/vrpn_mocap/RigidBody3/pose
ros2 topic echo --once /cdrone/cdrone3/external_pose/input_pose
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/pose
ros2 param get /cdrone/cdrone3/external_pose_adapter_node frame_rpy_rad
```

For the current studio calibration, a raw pose near `x=11.06, y=-0.53` should
become approximately `x=-11.06, y=0.53` on
`/cdrone/cdrone3/external_pose/input_pose`. If the adapter output or MAVROS
local pose still reports the unrotated XY, stop and fix the pose transform
before starting the demo.

Check heading separately. If the drone is physically facing `+Y`, the corrected
pose yaw should match that direction in the active map frame before flight. Use
`rpy_offset_rad` only to correct body heading after the XY frame transform has
been verified. The launch command above carries forward the same yaw offset
used by the validated Milestone 1 hover run.

The Milestone 4 demo config sets `max_target_distance_m: 15.0`, so a detected
target drone can become a follow candidate at 15 m if it also passes the
confidence, front-facing, lateral, vertical, perimeter, and minimum-distance
safety gates. Milestone 3 keeps its existing `6.0 m` config unless you override
`demo_config`.

`follow_distance_m` is not the chase-start threshold. It is the desired standoff
distance after the drone has already started chasing. With the default
`follow_distance_m: 1.5`, cdrone3 will chase an eligible detected target from
up to 15 m away, then slow/hold around 1.5 m from the target.

Milestone 4 enables tracking metrics by default. To disable only the CSV/MP4
metrics logger:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=2 \
  zed_rotate_180:=true \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5 \
  enable_tracking_metrics:=false
```

To override postprocessing for a run:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=2 \
  zed_rotate_180:=true \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5 \
  tracking_metrics_reference_video:=/home/jetson/cdrone_control/mission_recordings/<combined_video>.mp4
```

Set `tracking_metrics_postprocess_cleaned_tracks:=false` only if you want to
skip cleaned-track generation during the flight and rerun it later.

The ZED 2i tracker config leaves camera-to-body extrinsics at zero. If the ZED
2i is mounted differently than the old D455, calibrate and update
`camera_offset_body_m` and `camera_rpy_body_rad` before trusting metric body or
world target positions. If the camera is physically remounted upright later,
set `zed_rotate_180: false`, recapture the smoke images without `--rotate-180`,
and recheck the projection/extrinsics.

Useful tuning knobs, already included in the launch command above:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=2 \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5 \
  rpy_offset_rad:='[0.0, 0.0, -3.141592653589793]'
```

## Start

Do not run this while the ESCs are detached unless you are deliberately testing
the mission state machine with safe vehicle-side protections in place. For a
normal flight configuration, this starts the Milestone 4 mission through the
Milestone 3 sequence node and can arm/command the vehicle:

```bash
ros2 service call /cdrone/cdrone3/demo/milestone3_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
ros2 service call /cdrone/cdrone3/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```

## Live Checks

```bash
ros2 topic echo /cdrone/cdrone3/perception/status
ros2 topic echo /cdrone/cdrone3/perception/world_tracks
ros2 topic echo /cdrone/cdrone3/target_map/world_tracks
ros2 topic echo /cdrone/cdrone3/target_map/tracks
ros2 topic echo /cdrone/cdrone3/engagement/state
```

Healthy signs:

- raw perception tracks appear while the target drone is visible
- perception status shows tracker FPS and detector counts once ZED frames arrive
- target-map tracks keep a stable `track_id` across brief detector dropouts
- predicted tracks show `source: 2`, rising `last_observed_age_s`, and rising
  `position_uncertainty_m`
- the engagement state reports `target_visible: false` during predicted holds
- saved mission MP4s include `*_target_map.mp4`, and the `*_yolo.mp4` overlay
  shows map IDs, source labels, age, uncertainty, path tails, and velocity arrows
- Milestone 4 combined MP4 uses YOLO/RGB/YOLO+target-map/stats tiles, without a
  depth tile
- `cleaned_target_path.mp4` has the same frame count/FPS as the combined MP4,
  and its overlay time starts at `t=0.00s`

## Rerun Cleaned-Track Postprocess

After a recording is done, you usually do not need to run anything manually.
Use this only to backfill old metrics runs or force a specific combined video:

```bash
ros2 run drone_vision_pkg tracking_metrics_postprocess \
  --run-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/<tracking_run_dir> \
  --reference-video /home/jetson/cdrone_control/mission_recordings/<combined_video>.mp4
```

If `--reference-video` is omitted, the script auto-discovers the nearest
`mission_<drone>_*_combined.mp4` from `/home/jetson/cdrone_control/mission_recordings`.

## Compare With Milestone 3

Run one Milestone 3 flight with `enable_tracking_metrics:=true`, then one
Milestone 4 flight with the default metrics logger enabled. After both runs:

```bash
ros2 run drone_vision_pkg milestone_tracking_compare_report \
  --milestone3-run-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/<milestone3_run_dir> \
  --milestone4-run-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/<milestone4_run_dir> \
  --output-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/milestone3_vs_milestone4_report
```

The comparison report writes:

- `comparison_summary.csv`
- `comparison_summary.json`
- `comparison.md`

## RViz Markers

```bash
rviz2
```

Add a `MarkerArray` display and set the topic to:

```text
/cdrone/cdrone3/target_map/markers
```

Color meaning:

- green: detected
- yellow: tracker-held
- orange: target-map predicted

## ZED 2i Pose Note

This Milestone 4 run still uses OptiTrack for ownship pose. The ZED 2i SDK path
added here is for target perception depth and RGB only. If ZED VIO is evaluated
later as an ownship pose source, keep it log-only first and switch mission pose
topics only after pose quality and frame alignment are verified.
