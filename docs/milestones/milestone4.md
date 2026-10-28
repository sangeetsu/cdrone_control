# Milestone 4

Milestone 4 uses one ZED 2i for ownship vision pose and target color/depth,
with a target-memory layer between detection and the follow controller.

## What Milestone 4 Adds

The defense drone now has an internal target map for short target-drone
dropouts:

1. ZED + YOLO publishes body-frame and local-odometry target tracks.
2. `target_map_node` keeps stable map IDs across detector ID changes.
3. Missing detections are predicted with a constant-velocity model.
4. Prediction confidence decays and position uncertainty grows over time.
5. The controller consumes `/cdrone/<drone_id>/target_map/tracks`, not raw
   detector tracks.
6. Predicted tracks are accepted only for a short cautious hold.

ZED VIO is fused into PX4 for local position. MAVROS publishes that local pose
in a frame named `map`; it has no surveyed room or GPS alignment. Target memory
uses the same frame to predict moving drones through short detector gaps.
The ZED starts in the Milestone 1 `PERFORMANCE` depth mode and samples
detector frames at up to 15 Hz. Its SDK depth and pose computation is capped at
20 Hz, and the combined camera/detector process starts 15 s after MAVROS to
spread startup load. Inference waits for five seconds of stable ZED
tracking. See the runbook's restart diagnosis
before flight if the Jetson or FCU power cycles during launch.
The October 20 abrupt reset left empty ROS log files. The Jetson's `OC1`
undervoltage counter has since exceeded 85,000 and increased on the current
boot, so the shared supply path needs direct measurement under ZED and GPU
load. The combined process now syncs startup markers and power snapshots to
`~/.ros/log/milestone4_startup_trace.log` for post-reset diagnosis. The active
ZED depth mode is `PERFORMANCE` with `GEN_3` tracking; the SDK neural depth
mode and SDK object detector are not enabled. `best.engine` uses the GPU
separately.
The next pasted trace captured 4.568–4.664 V module input and thousands of
new `OC1` events before `best.engine` finished TensorRT warmup. The ZED had
not opened. The combined process now stops before heavy startup and again
before engine warmup if module voltage is below 4.85 V or `OC1` is rising.
The gate leaves 0.10 V of margin above the module's 4.75 V minimum.
This preflight cannot replace a physical load measurement or correct the
power path.
The older `Demo_ASU_News` branch used a different detector engine with ZED
depth and OptiTrack pose. The current route uses `models/best.engine` as an
OBB drone detector, adds ZED positional tracking, and passes color/depth arrays
directly to the detector without full ROS image transport. ZED-local Milestone
4 rejects `.pt` and `.onnx` runtime models; the supplied engine has no
embedded metadata and needs a known-target bench check on this Jetson.
The one-terminal diagnostic launch starts separate MAVROS, ZED VIO, and
detector processes; the full mission remains one launch.
The combined mission warms `best.engine` before opening the ZED, so the first
TensorRT context allocation is separated from camera depth and VIO startup.
The camera callback publishes each vision pose before retrieving color/depth
for detection. Its timing log distinguishes stale SDK frames from image retrieval
overhead; failed USB grabs back off, failed image retrieval is rejected, and
flight readiness still rejects stale vision pose.
Video writing and ReID/camera-motion estimation are off in the default
performance profile. CSV metrics and short target prediction remain enabled.

## Onboard Optical Flow And Control

Milestone 4 already uses body-frame velocity commands while following a target:
`compute_follow_command` in the milestone 3 sequence controller produces the
command, and `mavros_velocity_node` streams it to MAVROS. The target map is target
memory, not a PX4 waypoint mission. A waypoint-to-velocity rewrite is unnecessary.

The local mission uses position setpoints for OFFBOARD takeoff and a separate
velocity stream for pursuit. A 20 m takeoff-centered excursion guard replaces
the surveyed studio perimeter; target eligibility also ends at 20 m. Target
history is cleared when ownship pose jumps or becomes stale. Pursuit height is
limited to 1.0–2.5 m above takeoff, and detected-track horizontal commands
are capped at 1.2 m/s. The 20 m limit uses local odometry; it does not sense
obstacles or establish a surveyed boundary, so the flight area needs room for
the full path and stopping margin. See the
[Milestone 4 runbook](../../milestone4_runbook.md) for bench and flight gates.

The ZED-local mission can arm and climb without a visible target once ZED,
detector status, and PX4 local odometry pass readiness. After position-to-velocity
handoff it requires 2 continuous seconds within 0.10 m of takeoff height with
horizontal speed at most 0.25 m/s and vertical speed at most 0.15 m/s. Only
then does it enter `SEARCH`, where zero velocity is sent until a confirmed
target appears or the operator aborts. New map tracks need three coherent
observations spanning at least 0.2 s; associations allow 2 m/s motion plus
0.5 m measurement allowance, including when detector IDs match. One rejected
ZED pose enters a 0.5 s hold and can recover to `SEARCH` after two accepted
poses with valid PX4 odometry. Persistent ZED jumps and PX4 odometry faults
request landing. The October 23 abort evidence and log checks are in
[Milestone 4 troubleshooting](../../milestone4_troubleshooting.md).

Generated diagrams:

- `docs/generated/milestone4_target_memory_architecture.svg`
- `docs/generated/milestone4_prediction_path.svg`

Optional mission recording outputs (`tracker_record_mission_video:=true`):

- `*_combined.mp4`: Milestone 4 mosaic with detector-only YOLO, RGB,
  YOLO+target-map, and stats tiles; the depth tile is removed for this milestone
- `*_yolo.mp4`: RGB detector overlay plus target-map ID/source/age/uncertainty,
  path tail, and velocity arrow
- `*_target_map.mp4`: top-down map recording of defense drone path and target-map
  tracks over the mission

Milestone 3 keeps its original depth/RGB/YOLO/stats recording layout. Milestone 4
sets `recording_save_depth_video:=false` and
`recording_replace_depth_tile_with_yolo:=true` through launch.

Tracking metrics outputs:

- `tracking_samples.csv`: raw visual and target-map samples
- `tracking_frames.csv`: per-frame coverage/dropout/bridge state
- `cleaned_target_path.csv`: smoothed estimated target path with relative
  timestamps starting from zero
- `tracking_summary.csv`: one-row comparison metrics
- `cleaned_target_path.mp4` when `tracking_metrics_write_video:=true`:
  post-flight map visualization of ownship, raw
  visual tracks, target-map observations, predictions, and the cleaned path,
  rendered with the same FPS, frame count, and duration as the matching combined
  mission recording when that MP4 is available

These metrics are visual-only. Ownship pose comes from PX4 local odometry;
target drones need no mocap rigid bodies.

## Interfaces

`TargetTrack.msg` and `WorldTargetTrack.msg` include prediction metadata:

- `detector_track_id`
- `source`: `SOURCE_DETECTED`, `SOURCE_HELD`, or `SOURCE_PREDICTED`
- `last_observed_age_s`
- `prediction_horizon_s`
- `position_uncertainty_m`
- `velocity_uncertainty_mps`

Important topics for the current `cdrone3` branch:

```bash
/cdrone/cdrone3/perception/world_tracks
/cdrone/cdrone3/target_map/world_tracks
/cdrone/cdrone3/target_map/tracks
/cdrone/cdrone3/target_map/markers
/cdrone/cdrone3/engagement/state
```

The detector mission recorder subscribes to `/cdrone/cdrone3/target_map/world_tracks`
by default for MP4 burn-in overlays. Override
`recording_target_map_world_topic` only when the target-map world topic is also
customized.

The controller still publishes:

```bash
/cdrone/cdrone3/control/cmd_vel_body
```

## Prediction And Control Defaults

Target-map defaults:

- prediction publication horizon: `3.0 s`
- stale prune window: `5.0 s`
- association gate: `0.8 m`
- confidence decay: `0.25 / s`
- position uncertainty growth: `0.35 m/s`

Control defaults for predicted tracks:

- predicted control enabled only by `milestone4_demo.launch.py`
- predicted hold limit: `1.5 s`
- max predicted position uncertainty: `0.75 m`
- predicted XY command cap: `0.25 m/s`
- predicted Z command cap: `0.15 m/s`
- predicted yaw-rate cap: `0.25 rad/s`

Milestone 3 launch defaults keep predicted-track control disabled.

## Launch

Default ZED local-odometry bench launch:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  pose_source:=zed_local enable_flight:=false \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml
```

Tune prediction behavior from launch:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=1 \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=0.6 \
  target_map_prediction_horizon_s:=2.5
```

Legacy OptiTrack path:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  pose_source:=optitrack tracking_source_mode:=zed_sdk \
  frame_rpy_rad:='[0.0, 0.0, 3.141592653589793]'
```

## Validation

Targeted checks used for this milestone:

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
python3 -m pytest src/drone_vision_pkg/test/test_target_memory.py src/drone_vision_pkg/test/test_target_map_health.py src/drone_vision_pkg/test/test_zed_sdk_helpers.py src/drone_vision_pkg/test/test_best_engine_tracker.py src/drone_vision_pkg/test/test_zed_local_combined_startup.py -q
python3 -m pytest src/drone_vision_pkg/test/test_tracking_metrics.py -q
python3 -m pytest src/drone_control_pkg/test/test_milestone2_demo_logic.py -q
python3 -m pytest src/drone_control_pkg/test/test_follow_utils.py src/drone_control_pkg/test/test_local_flight.py src/drone_control_pkg/test/test_milestone4_engine_launch.py src/drone_control_pkg/test/test_zed_vio_node_resilience.py -q
ros2 launch drone_bringup milestone4_demo.launch.py --show-args
ros2 launch drone_bringup milestone4_diagnostics.launch.py --show-args
```

Bench acceptance:

- when the target drone is visible, target-map output uses detected or held
  source metadata
- when the target drone leaves frame briefly, target-map output continues with
  `SOURCE_PREDICTED`, increasing `last_observed_age_s` and uncertainty
- when the target drone reappears near the prediction, the same map ID is reused
- after the configured hold/uncertainty limits, control returns to search or zero
  command instead of chasing stale predictions
- when optional video is enabled, the mission recording folder contains
  `*_target_map.mp4`; overlays show target-map labels and uncertainty circles
  during predicted gaps, and the combined video uses a YOLO tile instead of
  a depth quadrant
- the tracking metrics folder contains the CSV bundle; when video is enabled,
  `cleaned_target_path.mp4` starts at `t=0.00s` and matches the combined MP4
  frame count/FPS when the reference recording is available
- compared with a Milestone 3 metrics run, Milestone 4 should show higher
  target-map coverage, shorter no-track gaps, bridged raw-detector dropouts,
  stable map IDs, and bounded reacquisition residuals
- ZED `OK`, valid PX4 vision-fused local position/velocity, synchronized
  color/depth, a running detector, and successful `/demo/milestone4_check`
  before flight; a visible target is optional for takeoff readiness
- `best.engine` yields at least 10 Hz detected 3D tracks for the moving-target
  flight, no gap above 0.5 s,
  and valid range in at least 90% of frames at 20 m with error within 2 m
- a moving target remains trackable as the pursuing drone moves; a stale pose
  or estimator reset clears target history and blocks pursuit
