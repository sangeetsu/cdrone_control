# Milestone 4 Runbook

Commands below assume your shell already loaded ROS 2 and the local workspace.
Detailed notes live in `docs/milestones/milestone4.md`.

Milestone 4 uses OptiTrack as the default ownship pose source and adds a target
map that predicts short target-drone dropouts. The current camera path is the
ZED 2i through the Stereolabs ZED SDK; moving target prediction is handled by
`target_map_node`.

For the current `cdrone3` OptiTrack setup, raw `RigidBody3` X/Y is rotated 180
degrees by the external pose adapter before any flight, perimeter, tracker,
target-map, or metrics consumer uses it.

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
  tracking_source_mode:=zed_sdk \
  tracker_config_file:=/home/jetson/cdrone_control/ros2/src/drone_vision_pkg/config/zed2i_tracking.yaml \
  required_completion_count:=2 \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5
```

This starts the existing tracker with `source_mode:=zed_sdk`, opens the ZED 2i
directly through `pyzed.sl`, retrieves left color plus depth frames, and keeps
OptiTrack as the ownship pose source.

Milestone 4 enables tracking metrics by default. To disable only the CSV/MP4
metrics logger:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  tracking_source_mode:=zed_sdk \
  tracker_config_file:=/home/jetson/cdrone_control/ros2/src/drone_vision_pkg/config/zed2i_tracking.yaml \
  required_completion_count:=2 \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5 \
  enable_tracking_metrics:=false
```

To override postprocessing for a run:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  tracking_source_mode:=zed_sdk \
  tracker_config_file:=/home/jetson/cdrone_control/ros2/src/drone_vision_pkg/config/zed2i_tracking.yaml \
  required_completion_count:=2 \
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
world target positions.

Useful tuning knobs, already included in the launch command above:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  required_completion_count:=2 \
  predicted_track_hold_s:=1.0 \
  max_predicted_position_uncertainty_m:=1.0 \
  target_map_prediction_horizon_s:=5
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
