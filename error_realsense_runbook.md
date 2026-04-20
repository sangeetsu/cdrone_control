# Error RealSense Runbook

This runbook is for bench-testing the RealSense tracker error pipeline and
generating the depth-vs-error report bundle reliably.

It assumes:

- the active tracker node is `realsense_tracker_node`
- ownship pose comes from `/vrpn_mocap/RigidBody3/pose`
- target reference pose comes from `/vrpn_mocap/RigidBody2/pose`
- run bundles are written under `/home/jetson/output_dump`
- the first reporting pass uses `RigidBody2` directly as ground truth

## What This Test Produces

Each run creates a new directory:

```text
/home/jetson/output_dump/YYYYMMDD_HHMMSS[_experiment_tag]/
```

Expected files inside each run directory:

- `run_manifest.json`
- `world_track_compare_tracks.csv`
- `world_track_compare_frames.csv`
- `world_track_compare_*.jpg`

After report generation, the same run directory should also contain:

- `report/depth_vs_error.png`
- `report/depth_binned_error.png`
- `report/truth_range_vs_error.png`
- `report/coverage_vs_range.png`
- `report/summary.json`
- `report/summary.md`

## Preflight

Run these steps before every bench session.

### 1. Build the ROS packages

From `/home/jetson/cdrone_control/ros2`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
source install/setup.bash
```

Pass criteria:

- build finishes without errors
- `source install/setup.bash` succeeds

### 2. Confirm the D455 is visible on the host

```bash
/home/jetson/cdrone_control/scripts/check_realsense_host_access.sh
```

Pass criteria:

- `lsusb` shows the D455
- `rs-enumerate-devices` sees the camera

If host access is broken:

```bash
sudo /home/jetson/cdrone_control/scripts/install_realsense_host_access.sh
```

Then physically unplug and replug the camera once.

### 3. Confirm OptiTrack topics are live

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 topic hz /vrpn_mocap/RigidBody3/pose
ros2 topic hz /vrpn_mocap/RigidBody2/pose
```

Pass criteria:

- both topics are publishing steadily
- frame dropouts are not obvious before tracker launch

### 4. Confirm the report command is installed

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
world_track_compare_report --help
```

Pass criteria:

- help text prints without import errors

## Tracker Launch

Use one `experiment_tag` per scenario.

Example launch:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_bringup tracking_only.launch.py \
  experiment_tag:=static_depth_3m
```

Expected startup signs:

- log shows `Started direct RealSense pipeline`
- log shows `RealSense tracker started`
- log includes a `run_dir=...` path

Optional live checks in another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 topic echo /cdrone/drone01/perception/status
ros2 topic echo /cdrone/drone01/perception/world_tracks
```

Healthy signs:

- `tracker_fps` is near `10`
- `inference_latency_ms` is stable enough for repeated runs
- world tracks appear when the target drone is visible

## Finding the Active Run Directory

The newest run directory is usually the one for the active test:

```bash
find /home/jetson/output_dump -maxdepth 1 -mindepth 1 -type d | sort | tail -n 1
```

You can save it in a variable:

```bash
LATEST_RUN=$(find /home/jetson/output_dump -maxdepth 1 -mindepth 1 -type d | sort | tail -n 1)
echo "$LATEST_RUN"
```

Before stopping the tracker, confirm the run directory is filling:

```bash
ls -lah "$LATEST_RUN"
```

Healthy signs:

- `run_manifest.json` exists
- JPEGs are appearing during the run
- CSV files exist and grow during a successful comparison run

## Standard Test Order

Keep these fixed across the whole benchmark set:

- same tracker config
- same detector model
- same camera settings
- same lighting region
- same pose topics

Run the scenarios in this order.

### Scenario 1: Static Depth Ladder

Goal:

- measure how error changes with distance when the target is as stable as possible

Procedure:

1. Launch with `experiment_tag:=static_depth_<distance>m`
2. Place the target centered in the frame
3. Hold at each depth for `20 s`
4. Repeat for `1.5, 2, 3, 4, 5, 6, 7, 8 m`

Recommended tags:

- `static_depth_1p5m`
- `static_depth_2m`
- `static_depth_3m`
- `static_depth_4m`
- `static_depth_5m`
- `static_depth_6m`
- `static_depth_7m`
- `static_depth_8m`

### Scenario 2: Axial Motion

Goal:

- measure error and dropouts during approach and retreat

Procedure:

1. Launch with `experiment_tag:=axial_in_01`
2. Move slowly from `8 m` to `2 m`
3. Stop the run
4. Launch with `experiment_tag:=axial_out_01`
5. Move slowly from `2 m` to `8 m`
6. Repeat until you have `3` inward and `3` outward runs

Recommended tags:

- `axial_in_01`, `axial_in_02`, `axial_in_03`
- `axial_out_01`, `axial_out_02`, `axial_out_03`

### Scenario 3: Lateral Sweep

Goal:

- measure how lateral motion affects range-conditioned detection quality

Procedure:

1. Hold distance fixed at `3 m`
2. Sweep left-right at roughly constant speed
3. Repeat `3` runs
4. Repeat the same process at `5 m`
5. Repeat the same process at `7 m`

Recommended tags:

- `lat_3m_01`, `lat_3m_02`, `lat_3m_03`
- `lat_5m_01`, `lat_5m_02`, `lat_5m_03`
- `lat_7m_01`, `lat_7m_02`, `lat_7m_03`

### Scenario 4: Reacquisition

Goal:

- measure loss and reacquisition behavior under partial occlusion

Procedure:

1. Launch with `experiment_tag:=reacq_3to5m_01`
2. Operate between `3-5 m`
3. Introduce partial occlusion and re-entry several times in one run
4. Repeat for `3` runs

Recommended tags:

- `reacq_3to5m_01`
- `reacq_3to5m_02`
- `reacq_3to5m_03`

## Per-Run Execution Checklist

For every run:

1. Start the tracker with the correct `experiment_tag`
2. Wait until the startup logs settle
3. Confirm the latest run directory exists
4. Perform the scenario exactly once
5. Stop the tracker cleanly with `Ctrl+C`
6. Confirm the run directory contains manifest, CSVs, and JPEGs
7. Generate the report
8. Save the report outputs and note pass/fail

## Report Generation

After stopping a run:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
LATEST_RUN=$(find /home/jetson/output_dump -maxdepth 1 -mindepth 1 -type d | sort | tail -n 1)
world_track_compare_report --run-dir "$LATEST_RUN"
```

Optional custom bin size:

```bash
world_track_compare_report --run-dir "$LATEST_RUN" --depth-bin-size 0.25
```

Pass criteria:

- command exits successfully
- `report/summary.json` exists
- all four PNG plots exist

## What To Inspect After Each Run

Open:

- `report/summary.md`
- `report/depth_vs_error.png`
- `report/depth_binned_error.png`
- `report/truth_range_vs_error.png`
- `report/coverage_vs_range.png`

Minimum checks:

- there are non-zero frame rows for a real run
- there are non-zero track rows when comparisons were actually produced
- `depth_vs_error.png` is populated for successful comparison runs
- `coverage_vs_range.png` is not flat at zero when tracking is healthy

## Pass / Fail Gates For Follow Readiness

Use these as the current first-pass gate:

- controller-valid ratio `>= 0.90`
- longest no-valid-track gap `< 0.25 s`
- p90 forward body error `<= 0.25 m`
- p90 lateral body error `<= 0.15 m`
- p90 vertical body error `<= 0.15 m`

The report already computes these in `summary.json` and `summary.md`.

If any of these fail, do not treat that operating band as ready for live
follow.

## Interpreting Common Outcomes

### Case 1: JPEGs exist but CSV track rows are zero

Meaning:

- the compare pipeline ran
- the node was alive
- but no valid world-track comparisons were produced

Likely causes:

- no world tracks were available
- ownship pose was missing
- reference pose was missing
- target was not detected or not projected successfully

Checks:

```bash
cat "$LATEST_RUN/report/summary.json"
```

Look at:

- `frame_rows`
- `track_rows`
- `world_track_frame_ratio`
- `controller_valid_track_ratio`

### Case 2: Reference available but `frame_match` is false

Meaning:

- the tracker world frame and reference pose frame differ
- world-coordinate error is not directly trustworthy until frames are aligned

Action:

- fix the frame mismatch before using the error plots for decisions

### Case 3: Error rises sharply with distance

Meaning:

- the current detect-depth-project stack is degrading at longer range

Action:

- use the depth-binned and truth-range plots to define the safe operating band

### Case 4: Controller-valid ratio is low but raw detections exist

Meaning:

- perception may still be publishing tracks
- but they are failing the follow controller safety gates

Likely causes:

- low confidence
- distance too large
- target not sufficiently in front
- body-frame lateral or vertical error too large

## Quick Bench Troubleshooting

### Tracker starts but no camera data arrives

Run:

```bash
/home/jetson/cdrone_control/scripts/check_realsense_host_access.sh
```

If needed:

```bash
sudo /home/jetson/cdrone_control/scripts/install_realsense_host_access.sh
```

Then unplug and replug the D455.

### Tracker starts but no world tracks appear

Check:

```bash
ros2 topic echo /cdrone/drone01/perception/tracks
ros2 topic echo /cdrone/drone01/perception/world_tracks
ros2 topic hz /vrpn_mocap/RigidBody3/pose
```

Interpretation:

- body tracks but no world tracks usually means ownship pose is missing
- no body tracks usually means detection/tracking/depth failed first

### Reports generate but plots are empty

Check:

```bash
LATEST_RUN=$(find /home/jetson/output_dump -maxdepth 1 -mindepth 1 -type d | sort | tail -n 1)
wc -l "$LATEST_RUN/world_track_compare_tracks.csv"
wc -l "$LATEST_RUN/world_track_compare_frames.csv"
```

Interpretation:

- frame CSV only means the run was logged but no valid comparisons happened
- empty track CSV means depth-vs-error plots cannot populate yet

## Suggested Session Log

For each run, record:

- date and time
- experiment tag
- lighting notes
- model path
- whether OptiTrack looked healthy
- whether the target stayed centered when intended
- whether report generation succeeded
- whether the run passed the follow-readiness gate

## One-Run Example

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash

ros2 launch drone_bringup tracking_only.launch.py \
  experiment_tag:=static_depth_3m
```

After the `20 s` hold completes, stop the node and run:

```bash
LATEST_RUN=$(find /home/jetson/output_dump -maxdepth 1 -mindepth 1 -type d | sort | tail -n 1)
world_track_compare_report --run-dir "$LATEST_RUN"
cat "$LATEST_RUN/report/summary.md"
```

If that run looks healthy, move to the next tag and repeat.
