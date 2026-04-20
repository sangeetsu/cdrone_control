# Tracking Experiment Protocol

This workflow uses the run-scoped world-track compare bundle produced by
`realsense_tracker_node` and the offline
`world_track_compare_report` command.

## Launch Pattern

From `cdrone_control/ros2`:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_bringup tracking_only.launch.py \
  experiment_tag:=static_depth_3m
```

Artifacts will be written under the configured
`world_track_compare_frame_dump_dir` as:

```text
/home/jetson/output_dump/YYYYMMDD_HHMMSS[_experiment_tag]/
```

Each run bundle contains:

- `run_manifest.json`
- `world_track_compare_tracks.csv`
- `world_track_compare_frames.csv`
- `world_track_compare_*.jpg`

## Report Generation

Generate the offline report after each run:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
world_track_compare_report \
  --run-dir /home/jetson/output_dump/20260416_180000_static_depth_3m
```

By default the report is written to:

```text
<run-dir>/report/
```

The report outputs:

- `depth_vs_error.png`
- `depth_binned_error.png`
- `truth_range_vs_error.png`
- `coverage_vs_range.png`
- `summary.json`
- `summary.md`

## Benchmark Scenarios

Run one experiment tag per scenario:

1. Static depth ladder at `1.5, 2, 3, 4, 5, 6, 7, 8 m`, `20 s` each.
2. Axial approach and retreat `8 -> 2 m` and `2 -> 8 m`, `3` repeats.
3. Lateral sweeps at `3, 5, 7 m`, `3` repeats each.
4. Reacquisition with partial occlusion and re-entry at `3-5 m`, `3` repeats.

Keep the tracker config, detector model, lighting region, and pose topics fixed
across all runs in the benchmark set.

## What To Check

For each run, review:

- world-track frame ratio
- controller-valid frame ratio
- median, p90, and p95 position error
- longest no-valid-track gap
- coverage vs range
- follow-readiness summary in `summary.json` / `summary.md`

The current follow-readiness gate is:

- controller-valid ratio `>= 0.90`
- longest no-valid-track gap `< 0.25 s`
- p90 forward body error `<= 0.25 m`
- p90 lateral body error `<= 0.15 m`
- p90 vertical body error `<= 0.15 m`
