# Milestone 4

Milestone 4 adds a target-memory layer between RealSense perception and the
existing follow/demo controller.

## What Milestone 4 Adds

The defense drone now has an internal target map for short target-drone
dropouts:

1. RealSense + YOLO publishes body-frame and world-frame target tracks.
2. `target_map_node` keeps stable map IDs across detector ID changes.
3. Missing detections are predicted with a constant-velocity model.
4. Prediction confidence decays and position uncertainty grows over time.
5. The controller consumes `/cdrone/<drone_id>/target_map/tracks`, not raw
   detector tracks.
6. Predicted tracks are accepted only for a short cautious hold.

SLAM/VSLAM is useful here as the defense drone localization source. It does not
replace target-state estimation for a moving target drone. In the default studio
flow, OptiTrack supplies the `map` frame. Isaac VSLAM remains the non-mocap pose
alternative when the D455 VSLAM path is validated for the airframe.

Generated diagrams:

- `docs/generated/milestone4_target_memory_architecture.svg`
- `docs/generated/milestone4_prediction_path.svg`

Mission recording outputs:

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
- `cleaned_target_path.mp4`: post-flight map visualization of ownship, raw
  visual tracks, target-map observations, predictions, and the cleaned path,
  rendered with the same FPS, frame count, and duration as the matching combined
  mission recording when that MP4 is available

These metrics are visual-only by default. The defense drone ownship pose is
`RigidBody3`, but attack drones do not need OptiTrack rigid bodies.

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

The RealSense mission recorder subscribes to `/cdrone/cdrone3/target_map/world_tracks`
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

Default OptiTrack-backed Milestone 4:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=1
```

Tune prediction behavior from launch:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=1 \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=0.6 \
  target_map_prediction_horizon_s:=2.5
```

Isaac VSLAM pose path, after the D455 VSLAM container is already publishing:

```bash
./scripts/launch_isaac_vslam_d455_in_container.sh

ros2 launch drone_bringup milestone4_demo.launch.py \
  pose_source:=realsense_pose \
  source_pose_topic:=/visual_slam/tracking/vo_pose \
  tracking_pose_topic:=/visual_slam/tracking/vo_pose \
  target_map_ownship_pose_topic:=/visual_slam/tracking/vo_pose
```

## Validation

Targeted checks used for this milestone:

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
python3 -m pytest src/drone_vision_pkg/test/test_target_memory.py -q
python3 -m pytest src/drone_vision_pkg/test/test_tracking_metrics.py -q
python3 -m pytest src/drone_control_pkg/test/test_milestone2_demo_logic.py -q
python3 -m pytest src/drone_control_pkg/test/test_follow_utils.py -q
ros2 launch drone_bringup milestone4_demo.launch.py --show-args
```

Bench acceptance:

- when the target drone is visible, target-map output uses detected or held
  source metadata
- when the target drone leaves frame briefly, target-map output continues with
  `SOURCE_PREDICTED`, increasing `last_observed_age_s` and uncertainty
- when the target drone reappears near the prediction, the same map ID is reused
- after the configured hold/uncertainty limits, control returns to search or zero
  command instead of chasing stale predictions
- the mission recording folder contains `*_target_map.mp4`, and the camera
  overlay videos show target-map labels and uncertainty circles during predicted
  gaps
- Milestone 4 combined video does not include a depth quadrant; the upper-left
  tile is normal YOLO detector output
- the tracking metrics folder contains the CSV bundle and
  `cleaned_target_path.mp4`; the cleaned MP4 starts at `t=0.00s` and matches the
  combined MP4 frame count/FPS when the reference recording is available
- compared with a Milestone 3 metrics run, Milestone 4 should show higher
  target-map coverage, shorter no-track gaps, bridged raw-detector dropouts,
  stable map IDs, and bounded reacquisition residuals
