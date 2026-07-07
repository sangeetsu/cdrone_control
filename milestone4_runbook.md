# Milestone 4 Runbook

Commands below assume your shell already loaded ROS 2 and the local workspace.
Detailed notes live in `docs/milestones/milestone4.md`.

Milestone 4 uses OptiTrack as the default ownship pose source and adds a target
map that predicts short target-drone dropouts. Isaac VSLAM can be used later as
the pose source, but moving target prediction is handled by `target_map_node`.

For the current `cdrone3` OptiTrack setup, raw `RigidBody3` X/Y is rotated 180
degrees by the external pose adapter before any flight, perimeter, tracker,
target-map, or metrics consumer uses it.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

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
  required_completion_count:=1
```

Milestone 4 enables tracking metrics by default. To disable only the CSV/MP4
metrics logger:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=1 \
  enable_tracking_metrics:=false
```

To override postprocessing for a run:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=1 \
  tracking_metrics_reference_video:=/home/jetson/cdrone_control/mission_recordings/<combined_video>.mp4
```

Set `tracking_metrics_postprocess_cleaned_tracks:=false` only if you want to
skip cleaned-track generation during the flight and rerun it later.

Useful tuning knobs:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=2 \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5
```

## Start

```bash
ros2 service call /cdrone/cdrone3/demo/milestone3_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
ros2 service call /cdrone/cdrone3/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```

## Live Checks

```bash
ros2 topic echo /cdrone/cdrone3/perception/world_tracks
ros2 topic echo /cdrone/cdrone3/target_map/world_tracks
ros2 topic echo /cdrone/cdrone3/target_map/tracks
ros2 topic echo /cdrone/cdrone3/engagement/state
```

Healthy signs:

- raw perception tracks appear while the target drone is visible
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

## VSLAM Pose Alternative

After Isaac VSLAM is already publishing `/visual_slam/tracking/vo_pose`:

```bash
./scripts/launch_isaac_vslam_d455_in_container.sh

ros2 launch drone_bringup milestone4_demo.launch.py \
  pose_source:=realsense_pose \
  source_pose_topic:=/visual_slam/tracking/vo_pose \
  tracking_pose_topic:=/visual_slam/tracking/vo_pose \
  target_map_ownship_pose_topic:=/visual_slam/tracking/vo_pose
```

Use OptiTrack first for validation; use VSLAM only after pose quality and frame
alignment are verified.
