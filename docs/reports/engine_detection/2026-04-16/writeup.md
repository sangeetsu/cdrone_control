# RealSense Static-Depth Evaluation of `best_large_640_100e_77k_fp16.engine`

## Purpose

This note is meant to give a concrete, data-backed summary of how the current TensorRT engine behaves in the RealSense tracking pipeline. The goal is to show where tracking quality degrades, where it fully fails, and why the current evidence points much more strongly to model/engine detectability than to mocap, logging, or evaluation bugs.

This is a dated historical report from the April 16, 2026 bench campaign, so it
intentionally records the then-active ownship rigid body rather than the newer
`cdrone4` defaults.

Engine under test:

- `/home/jetson/cdrone_yolo/models/best_large_640_100e_77k_fp16.engine`

## Test Setup

- Camera pipeline: direct RealSense, color `848x480@15`, depth `848x480@15`, aligned depth to color
- Reference target: `RigidBody2`
- Ownship pose: `RigidBody3`
- World/error evaluation: tracker world estimate compared against mocap reference pose
- Primary metrics:
  - ground-truth range from camera to target: `truth_range_m`
  - ground-truth forward depth in camera frame: `truth_depth_z_m`
  - estimated range/depth from tracker: `distance_m`, `depth_z_m`
  - world-position error norm: `error_norm_m`
  - detection fraction, world-track fraction, controller-valid fraction
- Comparison method for the table below:
  - for each run, I used the final `180` frame rows as the static hold window
  - track rows were matched to that same time window by `timestamp_s`
  - this avoids accidentally mixing older tracked rows into later miss windows

## Main Finding

The current engine tracks reliably only at short range in this setup. It remains usable through roughly `4.0 m`, with growing position error, then collapses between `4.05 m` and `5.02 m`.

The strongest evidence is the `5.0 m` and `6.0 m` holds:

- mocap remained healthy and continuously published valid target pose and ownship pose
- ground-truth range and depth were stable and logged correctly
- despite that, the final windows at `5.02 m`, `5.91 m`, `5.91 m with props`, and `5.91 m in reduced light` all had:
  - detection fraction `0.0`
  - world-track fraction `0.0`
  - controller-valid fraction `0.0`

That means the failure mode is not "bad ground truth" or "logger broke." It is "the detector never fired" under those conditions.

## Static Depth Ladder Summary

| Nominal hold | Run ID | Ground-truth range mean (m) | Ground-truth depth mean (m) | Detection fraction | World-track fraction | Controller-valid fraction | Estimated depth mean (m) | P90 error (m) | Notes |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1.5 m | `20260416_225715_static_depth_1p5m_rerun` | 1.495 | 1.489 | 1.000 | 1.000 | 1.000 | 1.325 | 0.205 | clean rerun |
| 2.0 m | `20260416_225910_static_depth_2m` | 1.904 | 1.896 | 1.000 | 1.000 | 1.000 | 1.768 | 0.188 | final hold good; file also contains earlier stale-`RigidBody2` segment |
| 3.0 m | `20260416_230755_static_depth_3m` | 3.156 | 3.152 | 1.000 | 1.000 | 1.000 | 3.131 | 0.245 | tracking still solid, but error is growing |
| 4.0 m | `20260416_230929_static_depth_4m` | 4.045 | 4.033 | 0.994 | 1.000 | 1.000 | 4.136 | 0.308 | still tracks, but error is materially worse |
| 5.0 m | `20260416_231100_static_depth_5m` | 5.022 | 5.013 | 0.000 | 0.000 | 0.000 | n/a | n/a | final hold is a pure miss window |
| 6.0 m | `20260416_231702_static_depth_6m` | 5.914 | 5.906 | 0.000 | 0.000 | 0.000 | n/a | n/a | pure miss case |
| 6.0 m + props | `20260416_232411_static_depth_6m_props` | 5.915 | 5.906 | 0.000 | 0.000 | 0.000 | n/a | n/a | propellers spinning did not recover detection |
| 6.0 m + low light | `20260416_232926_static_depth_6m_lowlight` | 5.915 | 5.906 | 0.000 | 0.000 | 0.000 | n/a | n/a | reduced lighting did not recover detection |

Source CSV for this table:

- The original CSV bundle is not retained in this repo snapshot; use the
  per-run reports linked later in this document for the preserved evidence.

## What The Error Trend Says

Before the detector fully fails, error is already trending in the wrong direction:

- `1.5 m`: `p90 error = 0.205 m`
- `2.0 m`: `p90 error = 0.188 m` in the final corrected hold window
- `3.0 m`: `p90 error = 0.245 m`
- `4.0 m`: `p90 error = 0.308 m`

So the pattern is:

1. Tracking works at short range.
2. Error grows by `3-4 m`.
3. Detection/track coverage collapses completely by `~5.0 m`.

The range/error plot makes that progression obvious:

- The original range/error plot asset is not retained in this repo snapshot.

The coverage plot shows the cliff between `4.05 m` and `5.02 m`:

- The original coverage plot asset is not retained in this repo snapshot.

## Why This Looks Like A Model/Engine Limitation

The current evidence points toward detectability/model performance, not the evaluation stack:

- At `5.02 m`, the final `180` frames had stable truth range and depth, but:
  - `detections_count = 0`
  - `active_tracks_count = 0`
  - `world_tracks_count = 0`
- At `5.91 m`, the same thing happened in three separate conditions:
  - baseline
  - propellers spinning
  - reduced lighting
- Mocap was live in those runs, and the target was still being located in world space:
  - the frame CSVs still recorded valid `truth_range_m` and `truth_depth_z_m`
  - the target and ownship poses were both available
- That means the problem is upstream of world projection and comparison:
  - if the detector does not fire, there is no YOLO detection, no Norfair track, and no world track

In other words: once the target gets to about `5-6 m` in this camera geometry, the current engine is not reliably producing even the first-stage detection needed by the rest of the pipeline.

## Representative Frames

Detected cases:

- `1.5 m` detected: [world_track_compare_1776405532_450752252.jpg](/home/jetson/output_dump/20260416_225715_static_depth_1p5m_rerun/world_track_compare_1776405532_450752252.jpg)
- `3.0 m` detected: [world_track_compare_1776406151_757768359.jpg](/home/jetson/output_dump/20260416_230755_static_depth_3m/world_track_compare_1776406151_757768359.jpg)
- `4.0 m` detected: [world_track_compare_1776406245_065471845.jpg](/home/jetson/output_dump/20260416_230929_static_depth_4m/world_track_compare_1776406245_065471845.jpg)

Miss cases:

- `5.0 m` miss: [world_track_compare_1776406605_698359290.jpg](/home/jetson/output_dump/20260416_231100_static_depth_5m/world_track_compare_1776406605_698359290.jpg)
- `6.0 m` miss: [world_track_compare_1776406790_125408363.jpg](/home/jetson/output_dump/20260416_231702_static_depth_6m/world_track_compare_1776406790_125408363.jpg)
- `6.0 m + props` miss: [world_track_compare_1776407138_958219294.jpg](/home/jetson/output_dump/20260416_232411_static_depth_6m_props/world_track_compare_1776407138_958219294.jpg)
- `6.0 m + low light` miss: [world_track_compare_1776408109_688924127.jpg](/home/jetson/output_dump/20260416_232926_static_depth_6m_lowlight/world_track_compare_1776408109_688924127.jpg)

The key visual point is that the target is still centered and still has valid mocap at `5-6 m`, but the detector output remains empty.

## Per-Run Reports

- `1.5 m`: [summary.md](/home/jetson/output_dump/20260416_225715_static_depth_1p5m_rerun/report/summary.md:1)
- `2.0 m`: [summary.md](/home/jetson/output_dump/20260416_225910_static_depth_2m/report/summary.md:1)
- `3.0 m`: [summary.md](/home/jetson/output_dump/20260416_230755_static_depth_3m/report/summary.md:1)
- `4.0 m`: [summary.md](/home/jetson/output_dump/20260416_230929_static_depth_4m/report/summary.md:1)
- `5.0 m`: [summary.md](/home/jetson/output_dump/20260416_231100_static_depth_5m/report/summary.md:1)
- `6.0 m`: [summary.md](/home/jetson/output_dump/20260416_231702_static_depth_6m/report/summary.md:1)
- `6.0 m + props`: [summary.md](/home/jetson/output_dump/20260416_232411_static_depth_6m_props/report/summary.md:1)
- `6.0 m + low light`: [summary.md](/home/jetson/output_dump/20260416_232926_static_depth_6m_lowlight/report/summary.md:1)

## Short Version To Send With This

The current `best_large_640_100e_77k_fp16.engine` appears to be the limiting factor for long-range detectability in our RealSense setup. In static bench tests:

- tracking is good at `1.5 m`
- still present but less accurate at `3-4 m`
- fully collapses by `~5.0 m`
- remains a full miss at `~5.9 m`
- prop motion and reduced lighting did not recover detection at `~5.9 m`

Ground-truth pose was still valid during the miss runs, so this is not a mocap or logging issue. The detector simply did not fire at those distances.

## Suggested Follow-Up For The Model/Engine Owner

- Re-run the same `5-6 m` clips against the original pre-TensorRT model and compare raw detections against the TRT engine.
- Check whether the training set actually contains enough `4-6 m` small-target examples from a low RealSense viewpoint.
- Test a higher input resolution or crop/tiling strategy, since the target is very small in-frame at `5-6 m`.
- Verify preprocessing parity between training/export/inference:
  - resize policy
  - letterboxing
  - color order
  - normalization
  - confidence and NMS thresholds
- If possible, dump raw detection logits or pre-NMS candidates on the `5-6 m` bench frames to determine whether the failure is:
  - no candidate at all
  - weak candidate below confidence threshold
  - NMS suppression
