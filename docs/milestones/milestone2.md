# Milestone 2

Commands in this note assume your shell has already loaded ROS 2 and the local
workspace from `~/.bashrc`.

## Current Demo Definition

Milestone 2 is the studio demo where our drone starts from a supervised staging
hover, detects small target drones with the active RealSense tracker, then runs
one airborne autonomous sequence that services targets at short standoff range.

Fixed assumptions for this revision:

- venue: `ros2/src/drone_bringup/config/drone_studio_perimeter.yaml`
- ownship pose source: OptiTrack through the active external-pose bridge
- target behavior: mostly stationary indoor loiter, not free maneuvering
- light behavior: flashlight always on and tethered, with no active light
  command in v1

## Status

Use these labels literally:

- `implemented`: code or launch surface exists in the active workspace
- `bench-validated`: backed by current repo evidence
- `new work required`: still not active or not yet validated for this demo

| Capability | Status | Notes |
| --- | --- | --- |
| `drone_vision_pkg` + `tracking_only.launch.py` | `implemented` | Active tracker path under `ros2/src/drone_vision_pkg` and `ros2/src/drone_bringup/launch/tracking_only.launch.py`. |
| Direct D455 tracking with typed `/perception/tracks` and `/perception/world_tracks` | `implemented` | Active `realsense_tracker_node` publishes `TargetTrackArray` and `WorldTargetTrackArray`. |
| Repo-local model auto-resolution | `implemented` | `drone_vision_pkg/model_paths.py` now prefers `cdrone_control/models`. |
| World-track compare bundle + offline report | `implemented` | `world_track_compare_report` is active in `drone_vision_pkg`. |
| Report unit test | `bench-validated` | `python3 -m pytest ros2/src/drone_vision_pkg/test/test_world_track_compare_report.py -q` passed on April 20, 2026. |
| April 20, 2026 static depth ladder with the new engine | `bench-validated` | Follow-ready at mean truth ranges `1.34 m` and `2.52 m`; not follow-ready at `3.74 m`; poor beyond `4.97 m`. |
| Generic `target_follow.launch.py` backend | `implemented` | Active single-target body-frame follow controller exists, but it is still a generic backend and still needs flight tuning. |
| New milestone 2 sequential demo path | `implemented` | `milestone2_demo.launch.py` and `milestone2_demo_sequence_node.py` now exist in the active workspace. |
| Sequential target completion for one airborne run | `implemented`, not yet `bench-validated` | The new node locks one `track_id`, dwells at standoff, excludes completed targets, and advances automatically. |
| Follow-path perimeter guard in the milestone 2 demo path | `implemented`, not yet `bench-validated` | The new milestone 2 node reuses `PerimeterGuard` and predicts commanded motion against the studio boundary and pillar keep-out. |
| Active light on/off control | `new work required` | `LightCommand.msg` exists, but the live light controller remains archived. |
| Generic active follow path with perimeter guard | `new work required` | The older `target_follow_controller_node.py` is still the single-target backend and still does not do perimeter prediction. |
| `combined.launch.py` | `new work required` | Still scaffolding only. |
| Multi-target mocap compare tooling | `new work required` | Current world-track compare flow still assumes one `compare_pose_topic`, so it is not the right validator for the multi-target smoke test. |

## Feasible Setup

The April 20, 2026 depth ladder is the hard perception envelope for this
revision. With the current engine:

- follow-ready at mean truth range `1.342 m`
- follow-ready at mean truth range `2.516 m`
- not follow-ready at mean truth range `3.736 m`
- largely collapsed by mean truth range `4.969 m`

That means the feasible studio geometry for this demo is:

- stage ownship at a perimeter-safe hover first
- place targets on a forward arc
- initial target loiter range: `2.2-2.4 m`
- standoff success condition: `1.5 +/- 0.2 m` for `>= 3.0 s`
- do not plan around initial range `>= 3.6 m`
- targets should remain quasi-static with only small loiter drift
- all targets do not need to be in frame simultaneously at startup

## Launch Surface

Main launch:

```bash
MODEL_PATH=/home/jetson/cdrone_control/models/16_k_and_drone_studio_realsense_images_model.engine
ros2 launch drone_bringup milestone2_demo.launch.py \
  model_path:="$MODEL_PATH" \
  required_completion_count:=1
```

What this launch starts:

- OptiTrack external-pose bridge through `external_pose_px4_bridge.launch.py`
- direct RealSense tracker through `tracking_only.launch.py`
- `mavros_velocity_node`
- `milestone2_demo_sequence_node`

Useful target-count examples:

- first flight with one target drone: `required_completion_count:=1`
- two-target follow-up test: `required_completion_count:=2`
- three-target video attempt: `required_completion_count:=3`

Important defaults in `milestone2_demo.yaml`:

- `follow_distance_m = 1.5`
- `follow_distance_tolerance_m = 0.2`
- `dwell_time_s = 3.0`
- `max_target_distance_m = 2.6`
- `min_safe_distance_m = 1.0`
- `max_vel_xy_mps = 0.6`
- `max_vel_z_mps = 0.3`
- `max_yaw_rate_rps = 0.4`

## Services And Topics

Start the airborne sequence after supervised takeoff and staging:

```bash
ros2 service call /cdrone/cdrone4/demo/milestone2_start std_srvs/srv/Trigger "{}"
```

Abort the sequence:

```bash
ros2 service call /cdrone/cdrone4/demo/milestone2_abort std_srvs/srv/Trigger "{}"
```

Useful live topics:

```bash
ros2 topic echo /cdrone/cdrone4/perception/tracks
ros2 topic echo /cdrone/cdrone4/engagement/state
ros2 topic echo /cdrone/cdrone4/mavros/state
```

The operator-facing state message is `/cdrone/cdrone4/engagement/state`
(`drone_msgs/EngagementState`), which now includes:

- `dwell_elapsed_s`
- `completed_targets_count`
- `required_targets_count`

## Build And Validation

If you have not rebuilt since the milestone 2 changes:

```bash
cd /home/jetson/cdrone_control/ros2
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
```

Targeted checks used for this revision:

```bash
python3 -m pytest ros2/src/drone_control_pkg/test/test_milestone2_demo_logic.py -q
python3 -m pytest ros2/src/drone_vision_pkg/test/test_world_track_compare_report.py -q
ros2 launch drone_bringup milestone2_demo.launch.py --show-args
```

## Bench Gates Before Flight

1. Keep the April 20, 2026 single-target ladder as the authoritative range
   bound until new evidence exists.
2. Run a single-target airborne handoff first at `2.2-2.4 m`.
3. Use a multi-target smoke test to check `TargetTrackArray` multiplicity,
   `track_id` stability, and automatic handoff.
4. Force one perimeter-conflict case and confirm hold or target-skip behavior.

Do not use the current world-track compare report as the only validator for the
multi-target smoke test. It still assumes one `compare_pose_topic`.

## Flight Gates And Video Acceptance

Before any milestone 2 attempt in a session:

1. Milestone 1 staged takeoff must pass cleanly.
2. A short perimeter-safe goto must pass cleanly.
3. If hover is unstable or `OFFBOARD` handoff is inconsistent, stop.

Video acceptance for the full three-target demo:

1. One supervised takeoff and staging phase, then one airborne autonomous run.
2. Three targets serviced without operator retargeting.
3. Each target held with a valid visible track at `1.5 +/- 0.2 m` for `>= 3.0 s`.
4. No perimeter violation, pillar incursion, or manual recovery during the sequence.

## Current Limits

This revision does not claim the whole studio video is already flight-proven.

The active code path now exists for:

- sequential target servicing
- target completion after standoff plus dwell
- automatic advance to the next target
- perimeter prediction in the milestone 2 demo path

Still pending validation:

- multi-target bench validation with real targets in one scene
- single-target airborne handoff validation
- full three-target airborne validation in one continuous run
- any active light control beyond the always-on tethered flashlight
