# cdrone_control

ROS 2 and PX4 workspace for indoor `cdrone` experiments on a Jetson Orin Nano.
This branch is the `cdrone3` clone, and the active stack is organized around a
single per-drone identity file instead of scattered literals.

## Source Of Truth

All per-drone identity, network, and mocap defaults live in
`ros2/src/drone_bringup/config/droneid_config.yaml`.

Current branch defaults:

- `drone_id: cdrone3`
- `hostname: cdrone3`
- `local_ip: 192.168.0.194`
- `mavros_namespace: /cdrone/cdrone3/mavros`
- `rigid_body_name: RigidBody3`
- `ownship_pose_topic: /vrpn_mocap/RigidBody3/pose`
- `compare_pose_topic: /vrpn_mocap/RigidBody2/pose`
- `optitrack_server: 192.168.0.217`
- `optitrack_port: 3883`
- peer metadata for `cdrone4: 192.168.0.121`

When cloning this repo for another drone, update `droneid_config.yaml` first.
The active bringup launches, tracker launch path, and helper scripts now read
from that file.

## Current Features

- OptiTrack / VRPN ownship pose bridge into PX4 through MAVROS
- Hover, goto, and circle demos with perimeter checks
- Target-follow control backend using the active `/cdrone/<drone_id>/...` topic tree
- Milestone 2 and Milestone 3 demo launch flows
- Milestone 4 target-memory flow for short target-drone dropout prediction
- RealSense tracking-only bench pipeline for world-track evaluation
- Retained D455 / VSLAM bridge path for future ownship-pose work
- Archived legacy autonomy stack preserved under `archive/legacy_stack/`

## Current Priorities

- Keep the external-pose flight path reliable for hover, goto, and circle work
- Validate the target-follow stack against the current OptiTrack-backed ownship pose
- Keep the tracking bench workflow reproducible for model evaluation
- Treat D455 / VIO as secondary work until the current control loop is stable

## Runbooks

These stay in the repo root on purpose because they are operator-facing entry
points for live testing.

- [milestone1_runbook.md](milestone1_runbook.md): hover, goto, and circle demo flow
- [milestone2_runbook.md](milestone2_runbook.md): Milestone 2 demo flow
- [milestone3_runbook.md](milestone3_runbook.md): Milestone 3 demo flow
- [milestone4_runbook.md](milestone4_runbook.md): Milestone 4 target-memory flow
- [error_realsense_runbook.md](error_realsense_runbook.md): RealSense tracking and report workflow

Each runbook assumes the active values come from `droneid_config.yaml`, and the
root operator-facing runbooks are aligned to the current `cdrone3` defaults.

## Documentation

- [docs/reference/repo_tree.md](docs/reference/repo_tree.md): repo layout after the cleanup pass
- [docs/notes/offboard_indoor_blocker.md](docs/notes/offboard_indoor_blocker.md): current OFFBOARD blocker context
- [docs/plans/milestone_planner.md](docs/plans/milestone_planner.md): active implementation plan
- [docs/milestones/milestone2.md](docs/milestones/milestone2.md): milestone 2 notes and validation context
- [docs/milestones/milestone3.md](docs/milestones/milestone3.md): milestone 3 notes and validation context
- [docs/milestones/milestone4.md](docs/milestones/milestone4.md): milestone 4 target-memory notes
- [docs/protocols/tracking_experiment_protocol.md](docs/protocols/tracking_experiment_protocol.md): tracking experiment workflow
- [docs/guides/setup/host_setup.md](docs/guides/setup/host_setup.md): host-side setup notes
- [docs/guides/vio/isaac_vslam_d455.md](docs/guides/vio/isaac_vslam_d455.md): retained VSLAM guide
- [docs/reference/d455_vio_contract.md](docs/reference/d455_vio_contract.md): VIO interface notes
- [docs/reference/isaac_ros_release32.md](docs/reference/isaac_ros_release32.md): pinned Isaac ROS reference
- [docs/reports/](docs/reports/): dated experiment reports and report assets
- [docs/archived/](docs/archived/): legacy or superseded notes kept for history

## Repo Layout

- `ros2/src/`: active ROS 2 packages and launch/config files
- `scripts/`: bench helpers, setup helpers, and capture scripts
- `docs/`: organized documentation, notes, reports, and references
- `archive/legacy_stack/`: old autonomy-heavy stack kept only as reference
- `models/`: detector engines and model assets used at runtime
- `ds_dataset/`: dataset exports and related scripts
- `temp_outputs/`: generated experiment output bundles
- `backup/`: old build/install/log snapshots

Documentation-related assets belong under `docs/`. Runtime datasets, models,
and generated outputs stay outside `docs/` because they are operational assets,
not narrative documentation. If you want another cleanup pass later, a dedicated
top-level `data/` or `artifacts/` directory would be a better home than nesting
those under `docs/`.

## Future Work

- Validate `cdrone3` and `cdrone4` flying concurrently on the same LAN and mocap system
- Add an explicit host pairing / SSH / hostname runbook for new clones
- Push more setup into reusable per-drone config instead of host-local memory
- Flight-tune and validate the target-follow controller
- Decide whether D455 / VIO should become a primary ownship-pose path later
