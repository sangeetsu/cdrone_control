# Milestone 3 Runbook

Commands below assume your shell already loaded ROS 2 and the local workspace.
Detailed notes live in `docs/milestones/milestone3.md`.

The active `drone_id`, MAVROS namespace, and mocap defaults come from
`ros2/src/drone_bringup/config/droneid_config.yaml`. The checked-in commands use
the current `cdrone3` branch values.

Current OptiTrack frame correction: `RigidBody3` is still the raw defense-drone
source, but the external pose adapter applies `frame_rpy_rad: [0, 0, pi]` so the
published ownship pose flips raw X/Y into the saved studio map frame. Tracking,
target-map, metrics, PX4, and perimeter checks should consume
`/cdrone/cdrone3/external_pose/input_pose`, not raw `RigidBody3`.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Launch

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1
```

To record Milestone 3 visual-tracking metrics for comparison with Milestone 4,
enable the metrics logger:

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1 \
  enable_tracking_metrics:=true \
  tracking_metrics_experiment_tag:=milestone3
```

Metrics outputs are written under:

```text
mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone3/
```

Each run directory contains CSV files plus `cleaned_target_path.mp4`.

PX4 speed profile selection for Milestone 3:

- `speed_profile:=indoor` for the current conservative profile
- `speed_profile:=fun` for the new midpoint profile
- `speed_profile:=default` for the faster baseline profile

Example:

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1 \
  speed_profile:=fun
```

If you want to push the same named profile directly to PX4 outside the Milestone
3 launch flow, use:

```bash
python3 /home/jetson/cdrone_control/scripts/apply_px4_speed_profile.py indoor
python3 /home/jetson/cdrone_control/scripts/apply_px4_speed_profile.py fun
python3 /home/jetson/cdrone_control/scripts/apply_px4_speed_profile.py default
```

Helpful helper commands:

```bash
python3 /home/jetson/cdrone_control/scripts/apply_px4_speed_profile.py --list
python3 /home/jetson/cdrone_control/scripts/apply_px4_speed_profile.py fun --dry-run
/home/jetson/cdrone_control/scripts/restore_hover_baseline_params.sh
```

Use the restore script if you want to return PX4 to the validated baseline
profile after testing or after an interrupted run.

## Start

```bash
ros2 service call /cdrone/cdrone3/demo/milestone3_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
ros2 service call /cdrone/cdrone3/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```

## Tracking Metrics

Milestone 3 metrics use raw visual world tracks only. Save the run directory
printed by `tracking_metrics_node`; it will be the `--milestone3-run-dir` input
for the Milestone 3 vs Milestone 4 comparison report.

Expected files:

- `tracking_samples.csv`
- `tracking_frames.csv`
- `cleaned_target_path.csv`
- `tracking_summary.csv`
- `cleaned_target_path.mp4`

## Speed Tuning

PX4 post-OFFBOARD profiles live in
`ros2/src/drone_bringup/config/indoor_speed_profile.yaml`,
`fun_speed_profile.yaml`, and `default_speed_profile.yaml`. Controller-side
follow and yaw response still comes from
`ros2/src/drone_bringup/config/milestone3_demo.yaml`. The detailed values,
units, and tuning guidance live in `docs/milestones/milestone3.md`.
