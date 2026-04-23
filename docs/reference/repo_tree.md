# Repo Tree

This file documents the post-cleanup repo layout.

Design intent:

- keep operator runbooks at the repo root for quick access
- move longer-lived reference material under `docs/`
- move outdated or superseded notes under `docs/archived/`
- keep runtime assets outside `docs/` so documentation and operational data do
  not get mixed together

```text
cdrone_control/
|-- README.md
|-- milestone1_runbook.md
|-- milestone2_runbook.md
|-- milestone3_runbook.md
|-- error_realsense_runbook.md
|-- archive/
|   `-- legacy_stack/
|-- backup/
|-- docker/
|-- docs/
|   |-- archived/
|   |   |-- README.md
|   |   |-- artifacts/
|   |   `-- *.md
|   |-- generated/
|   |   `-- drone_studio_perimeter_2d.svg
|   |-- guides/
|   |   |-- setup/
|   |   `-- vio/
|   |-- milestones/
|   |   |-- milestone2.md
|   |   `-- milestone3.md
|   |-- notes/
|   |   `-- offboard_indoor_blocker.md
|   |-- plans/
|   |   `-- milestone_planner.md
|   |-- protocols/
|   |   `-- tracking_experiment_protocol.md
|   |-- reference/
|   |   |-- d455_vio_contract.md
|   |   |-- isaac_ros_release32.md
|   |   `-- repo_tree.md
|   `-- reports/
|       `-- engine_detection/
|-- ds_dataset/
|-- external/
|-- librealsense/
|-- models/
|-- ros2/
|   |-- src/
|   |   |-- drone_bringup/
|   |   |-- drone_control_pkg/
|   |   |-- drone_msgs/
|   |   |-- drone_vision_pkg/
|   |   `-- ros2_poselib/
|   |-- build/
|   |-- install/
|   `-- log/
|-- scripts/
|-- setup_jetson.sh
|-- temp_outputs/
`-- test_mavlink_router_connection.sh
```

Notes:

- `docs/reports/` is for dated, human-readable experiment output and any assets
  that need to travel with those writeups.
- `models/`, `ds_dataset/`, and `temp_outputs/` were intentionally left outside
  `docs/` because they are runtime data, datasets, or generated artifacts.
- If the repo keeps growing, the next useful organization step would be a
  top-level `data/` or `artifacts/` umbrella for non-doc assets.
