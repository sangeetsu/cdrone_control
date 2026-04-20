# RealSense Bench Check of 16_k_and_drone_studio_realsense_images_model.engine

## Purpose

Ground truth came from mocap. In the April 20, 2026 static-depth ladder, this new engine tracked cleanly through 2.4 m, lost follow readiness by 3.6 m, collapsed sharply at 4.8 m, and stayed unreliable at 6.0 m and 7.2 m. The 6.0 m rerun did recover intermittent detections, but coverage and error were still far outside the follow gate.

## Static-depth ladder summary

Hit fractions below are computed across the full run. Range, depth, and error metrics use the final steady window.

| Nominal hold | Run ID | Ground-truth range mean (m) | Ground-truth depth mean (m) | Detection fraction | World-track fraction | Controller-valid fraction | Estimated depth mean (m) | P90 error (m) | Notes |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1.2 m | `20260420_061303_static_depth_1p2m_rerun` | 1.342 | 1.326 | 1.000 | 0.998 | 0.998 | 1.158 | 0.203 | Short-range detection is stable and comfortably inside the follow gate. |
| 2.4 m | `20260420_061624_static_depth_2p4m` | 2.516 | 2.503 | 0.952 | 0.997 | 0.997 | 2.416 | 0.224 | Tracking is still stable here and remains follow-ready. |
| 3.6 m | `20260420_061842_static_depth_3p6m` | 3.736 | 3.730 | 1.000 | 0.997 | 0.997 | 3.769 | 0.327 | Coverage is still high, but error has grown beyond the follow gate. |
| 4.8 m | `20260420_062105_static_depth_4p8m` | 4.969 | 4.962 | 0.071 | 0.026 | 0.026 | n/a | n/a | Only brief detections remained at this depth, with world tracks appearing only sporadically. |
| 6.0 m | `20260420_062552_static_depth_6p0m_rerun` | 6.170 | 6.161 | 0.313 | 0.415 | 0.415 | 6.697 | 0.896 | The rerun recovered some detections, but coverage and error stayed far outside the follow gate. |
| 7.2 m | `20260420_062851_static_depth_7p2m` | 7.414 | 7.401 | 0.162 | 0.096 | 0.000 | 8.755 | 1.900 | Detections still appear occasionally, but usable tracks are effectively gone. |

Source CSV for this table:

- `/home/jetson/cdrone_control/engine_model_report_assets_2026-04-20/static_depth_ladder_summary.csv`

Overview plots:

- Tracking coverage vs range: `/home/jetson/cdrone_control/engine_model_report_assets_2026-04-20/range_vs_tracking_fraction.png`
- Error vs range: `/home/jetson/cdrone_control/engine_model_report_assets_2026-04-20/range_vs_error_p90.png`

Representative runs:

- 1.2 m: `/home/jetson/output_dump/20260420_061303_static_depth_1p2m_rerun/report/summary.md`
- 2.4 m: `/home/jetson/output_dump/20260420_061624_static_depth_2p4m/report/summary.md`
- 3.6 m: `/home/jetson/output_dump/20260420_061842_static_depth_3p6m/report/summary.md`
- 4.8 m: `/home/jetson/output_dump/20260420_062105_static_depth_4p8m/report/summary.md`
- 6.0 m: `/home/jetson/output_dump/20260420_062552_static_depth_6p0m_rerun/report/summary.md`
- 7.2 m: `/home/jetson/output_dump/20260420_062851_static_depth_7p2m/report/summary.md`
