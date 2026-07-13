# cdrone_control Theory And File Guide

Generated for branch `cdrone3_internal_tracking` at tracked commit `d14e83d`.

This document explains the purpose, theory, architecture, workflows, and tracked-file contents of the cdrone_control repository. It intentionally covers tracked Git paths only. Local untracked recordings, generated build trees, untracked runtime logs, and deployment archives such as `mission_recordings/`, untracked `runtime_logs/`, and `cdrone_deploy_v1.tar.gz` are outside this guide except where their tracked templates or scripts are documented.

## Table Of Contents

- [Purpose](#purpose)
- [Core Theory](#core-theory)
- [Architecture](#architecture)
- [Workflow Model](#workflow-model)
- [Tracked File Coverage](#tracked-file-coverage)
- [File Catalog](#file-catalog)
- [Maintenance Suggestions](#maintenance-suggestions)

## Purpose

`cdrone_control` is a ROS 2 and PX4 workspace for indoor drone experiments on Jetson-class companion computers. The current branch is specialized for `cdrone3`: one drone in a multi-drone indoor lab setup where identity, network addresses, MAVROS namespaces, and OptiTrack rigid-body names must be consistent across launch files, scripts, control nodes, and perception nodes.

The repository exists to make indoor flight experiments repeatable. It keeps the flight-control side, target-tracking side, runbooks, dataset tooling, Docker/setup helpers, and archived predecessor stack in one place. The active work centers on externally referenced pose control, RealSense target tracking, target-follow behaviors, and milestone demos that can be run in a lab with PX4/MAVROS, OptiTrack/VRPN, and D455/Jetson hardware.

The practical purpose is not only to store code. It is to preserve the exact assumptions that make flight tests safe: the active drone identity, the coordinate frames, topic names, perimeter limits, launch defaults, controller state machines, and operator procedures. That is why this guide documents code, configs, tests, reports, and archived context together.

## Core Theory

### Indoor Drone Autonomy

Indoor drone autonomy differs from outdoor GPS flight because global position is not available from GNSS. The stack therefore supplies PX4 with external pose estimates and constrains motion inside a known volume. The control loop depends on consistent transforms: the flight controller needs a trustworthy estimate of ownship pose, the perception stack needs camera-frame detections projected into body or world coordinates, and the behavior layer needs safety checks before it sends setpoints.

### PX4 And MAVROS

PX4 is the autopilot firmware responsible for low-level stabilization, arming, mode handling, failsafes, and actuator control. MAVROS is the ROS bridge that exposes PX4 state, services, setpoint topics, and parameter APIs. In this repo, Python ROS nodes generally do not control motors directly. They publish position/velocity/manual setpoints, call MAVROS services such as arming and mode changes, and monitor MAVROS state freshness so PX4 remains the final authority on flight safety.

### ROS 2

ROS 2 provides process composition, typed topics, services, launch descriptions, parameters, and package boundaries. The active stack uses ROS packages for bringup (`drone_bringup`), control (`drone_control_pkg`), perception (`drone_vision_pkg`), custom messages (`drone_msgs`), and a small pose utility library (`ros2_poselib`). Launch files wire these packages together and inject per-drone defaults from config files.

### OptiTrack, VRPN, And External Pose

OptiTrack provides motion-capture estimates for rigid bodies in the flight arena. VRPN publishes those poses into ROS. The external-pose bridge path adapts the mocap pose into MAVROS/PX4-compatible topics and frames. The central challenge is maintaining frame consistency, timestamp freshness, and namespace correctness so the vehicle receives the pose for the correct rigid body and does not fly using stale or cross-drone data.

### RealSense Tracking

The RealSense D455 supplies RGB/depth streams for target detection and tracking. The perception stack loads a detector model, filters detections, estimates body/world target state, publishes custom target-track messages, and optionally records metrics. Depth and camera projection convert image-space detections into metric estimates, while resilience logic handles short dropouts and appearance-based track continuity.

### Target Following And Target Memory

Target following translates a target track into velocity or position commands subject to standoff, altitude, yaw, and speed constraints. Target memory extends this by retaining recent target state so the system can bridge short tracking dropouts. The theory is deliberately conservative: prediction is useful only while it is fresh and bounded; after timeout or uncertainty growth, the controller must slow, hold, return, or abort rather than chase stale data.

### Safety Perimeter

Indoor tests require software boundaries in addition to pilot discipline. Perimeter configs define allowed polygons, altitude ceilings/floors, and margins. Control nodes use these configs to reject unsafe goals and abort or hold when runtime state violates constraints. The perimeter system is a guardrail around autonomous behaviors, not a replacement for PX4 failsafes or human supervision.

### cdrone3 Identity Flow

The active identity source is `ros2/src/drone_bringup/config/droneid_config.yaml`. Launch files and helper scripts load it to derive the drone id, hostname, local IP, MAVROS namespace, cdrone namespace, ownship mocap topic, compare pose topic, rigid-body name, and peer metadata. This prevents the most dangerous class of multi-drone mistakes: hidden literals that accidentally command or observe the wrong vehicle.

## Architecture

### Package Roles

- `drone_bringup`: 38 tracked paths.
- `drone_control_pkg`: 48 tracked paths.
- `drone_msgs`: 12 tracked paths.
- `drone_vision_pkg`: 27 tracked paths.
- `ros2_poselib`: 12 tracked paths.

`drone_bringup` is the orchestration layer. It contains launch files and shared YAML defaults that decide which nodes start together and which topics/parameters they receive.

`drone_control_pkg` owns flight-facing behavior: MAVROS setup, demo state machines, external-pose adapters, target-follow controllers, perimeter checks, speed profiles, and optical-flow comparison utilities.

`drone_vision_pkg` owns camera-facing behavior: RealSense tracking, projection, target memory, world-track mapping, metrics generation, and report tooling.

`drone_msgs` defines custom message interfaces so perception, control, and monitoring agree on typed contracts for targets, status, alerts, and coordination.

`ros2_poselib` provides pose math utilities used by the ROS stack.

### Data Flow

1. Mocap or VIO produces ownship pose.
2. External-pose bridge/adapters convert pose into the frame and topic shape expected by MAVROS/PX4.
3. RealSense tracking produces target observations in image, body, or world frames.
4. Target memory/map/metrics nodes refine or evaluate target state.
5. Demo sequence and follow-controller nodes combine ownship state, target state, config, and safety perimeter checks.
6. MAVROS setpoint/service topics carry the requested mode, arm state, velocity, pose, or manual-control command toward PX4.

### Runtime Safety Model

The control code repeatedly checks freshness, mode, arming state, pose availability, perimeter limits, timeout budgets, service-call results, and operator start/abort services. Most demo nodes are state machines: they move through preflight, setup, takeoff/hover/follow/return/land states and fall into abort/hold/land paths when a guard fails. This code structure is important because a lab demo must behave predictably when a sensor drops out or PX4 refuses a request.

### Test Strategy

The repo uses Python unit tests for math helpers, state transition helpers, target memory, tracking metrics, speed-profile configs, and report generation. Lint placeholder tests are present in ROS packages. The tests are not flight certification; they preserve logic-level behavior so live testing starts from a known baseline.

## Workflow Model

The operator-facing flow starts with setup docs and root runbooks, then launches bringup files with cdrone3 defaults. During development, helpers in `scripts/` calibrate perimeters, check RealSense/VIO/PX4Flow topics, apply PX4 speed profiles, capture logs, and generate reports. Dataset tools under `ds_dataset/` record and export training data. Reports under `docs/reports/` retain experiment evidence, while `archive/legacy_stack/` preserves the previous autonomy stack for design reference rather than active operation.

## Tracked File Coverage

- Tracked paths documented: 348.
- Generated from Git commit: `d14e83d`.
- Branch at generation time: `cdrone3_internal_tracking`.
- Scope: tracked Git paths only; untracked local recordings/logs/build products are intentionally excluded.

### Counts By Area

- Archived legacy stack: 72
- Documentation, references, and reports: 49
- Active ROS 2 package: drone_control_pkg: 48
- Active ROS 2 package: drone_bringup: 38
- Utility scripts: 29
- Active ROS 2 package: drone_vision_pkg: 27
- Root files and operator runbooks: 25
- Dataset capture and export workspace: 17
- Tracked runtime logs: 13
- Active ROS 2 package: drone_msgs: 12
- Active ROS 2 package: ros2_poselib: 12
- Docker and host setup assets: 5
- Model and runtime assets: 1

### Counts By Extension

- `.py`: 139
- `.md`: 55
- `.yaml`: 28
- `.sh`: 27
- `[no extension]`: 22
- `.log`: 13
- `.msg`: 10
- `.xml`: 9
- `.cfg`: 7
- `.jpg`: 7
- `.txt`: 6
- `.png`: 4
- `.docx`: 3
- `.svg`: 3
- `.Dockerfile`: 2
- `.html`: 2
- `.rules`: 2
- `.csv`: 1
- `.dsc`: 1
- `.engine`: 1
- `.gz`: 1
- `.params`: 1
- `.parm`: 1
- `.xz`: 1
- `.yml`: 1
- `Dockerfile`: 1

## File Catalog

## Active ROS 2 package: drone_bringup

### `ros2/src/drone_bringup/CMakeLists.txt`

- File type: CMake build manifest.
- Tracked size: 881 bytes; 26 decoded lines.
- Purpose: CMake/ament build recipe for a ROS package.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: colcon/ament_cmake during build.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 26 total, 21 non-empty.
  - Example: `cmake_minimum_required(VERSION 3.8)`
  - Example: `project(drone_bringup)`
  - Example: `if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")`

### `ros2/src/drone_bringup/config/default_speed_profile.yaml`

- File type: YAML configuration file.
- Tracked size: 409 bytes; 14 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 12 documented key/value entries.
- Key map:
  - `name`: `default_speed_profile`
  - `description`: `PX4 motion profile matching the faster baseline values used before the indoor profile.`
  - `parameters`: mapping
  - `parameters.MPC_XY_CRUISE`: `5.0`
  - `parameters.MPC_XY_VEL_MAX`: `12.0`
  - `parameters.MPC_ACC_HOR`: `3.0`
  - `parameters.MPC_JERK_AUTO`: `4.0`
  - `parameters.MPC_JERK_MAX`: `8.0`
  - `parameters.MPC_Z_V_AUTO_UP`: `3.0`
  - `parameters.MPC_Z_V_AUTO_DN`: `1.5`
  - `parameters.MPC_Z_VEL_MAX_DN`: `1.5`
  - `parameters.MPC_LAND_SPEED`: `0.7`

### `ros2/src/drone_bringup/config/drone_studio_perimeter.yaml`

- File type: YAML configuration file.
- Tracked size: 1671 bytes; 65 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 55 documented key/value entries.
- Key map:
  - `version`: `1`
  - `name`: `drone_studio`
  - `frame_id`: `map`
  - `source_pose_topic`: `/cdrone/cdrone3/mavros/local_position/pose`
  - `finalized_from`: `perimeter.yaml`
  - `finalized_at_utc`: `2026-04-13T01:05:30+00:00`
  - `notes`: list[4]
  - `notes[0]`: `boundary XY was sampled on the floor and is the intended safe inner fly boundary`
  - `notes[1]`: `pillar XY was sampled at elevated marker height because floor detection was wavy`
  - `notes[2]`: `pillar Z is intentionally ignored for enforcement; the pillar keep-out is XY-only`
  - `notes[3]`: `the flight ceiling is 4.5 m above the measured floor reference from the boundary samples`
  - `floor_reference_m`: mapping
  - `floor_reference_m.sampled_floor_z_mean_m`: `0.1436`
  - `floor_reference_m.sampled_floor_z_min_m`: `0.1314`
  - `floor_reference_m.sampled_floor_z_max_m`: `0.1511`
  - `altitude_limits_m`: mapping
  - `altitude_limits_m.min_z_m`: `1.0`
  - `altitude_limits_m.max_z_m`: `4.6436`
  - `altitude_limits_m.min_height_above_floor_m`: `0.8564`
  - `altitude_limits_m.max_height_above_floor_m`: `4.5`
  - `boundary_polygon_xy_m`: list[4]
  - `boundary_polygon_xy_m[0]`: mapping
  - `boundary_polygon_xy_m[0].name`: `north_west`
  - `boundary_polygon_xy_m[0].x_m`: `-12.8963`
  - `boundary_polygon_xy_m[0].y_m`: `12.3315`
  - `boundary_polygon_xy_m[1]`: mapping
  - `boundary_polygon_xy_m[1].name`: `north_east`
  - `boundary_polygon_xy_m[1].x_m`: `5.2392`
  - `boundary_polygon_xy_m[1].y_m`: `12.7477`
  - `boundary_polygon_xy_m[2]`: mapping
  - `boundary_polygon_xy_m[2].name`: `south_east`
  - `boundary_polygon_xy_m[2].x_m`: `3.8063`
  - `boundary_polygon_xy_m[2].y_m`: `-4.0075`
  - `boundary_polygon_xy_m[3]`: mapping
  - `boundary_polygon_xy_m[3].name`: `south_west`
  - `boundary_polygon_xy_m[3].x_m`: `-14.2857`
  - `boundary_polygon_xy_m[3].y_m`: `-4.3622`
  - `keep_out_polygons_xy_m`: list[1]
  - `keep_out_polygons_xy_m[0]`: mapping
  - `keep_out_polygons_xy_m[0].name`: `studio_pillar`
  - `keep_out_polygons_xy_m[0].enforce_xy_only`: `True`
  - `keep_out_polygons_xy_m[0].vertices_xy_m`: list[4]
  - `keep_out_polygons_xy_m[0].vertices_xy_m[0]`: mapping
  - `keep_out_polygons_xy_m[0].vertices_xy_m[1]`: mapping
  - `keep_out_polygons_xy_m[0].vertices_xy_m[2]`: mapping
  - `keep_out_polygons_xy_m[0].vertices_xy_m[3]`: mapping
  - `geometry_summary`: mapping
  - `geometry_summary.boundary_area_m2`: `302.399`
  - `geometry_summary.pillar_keep_out_area_m2`: `14.937`
  - `geometry_summary.min_pillar_vertex_to_boundary_m`: `5.756`
  - `recommended_policy`: mapping
  - `recommended_policy.reject_goals_outside_boundary`: `True`
  - `recommended_policy.reject_goals_inside_keep_out`: `True`
  - `recommended_policy.block_motion_toward_violation`: `True`
  - `recommended_policy.hold_zero_velocity_on_violation`: `True`

### `ros2/src/drone_bringup/config/droneid_config.yaml`

- File type: YAML configuration file.
- Tracked size: 865 bytes; 30 decoded lines.
- Purpose: Central cdrone identity and network/mocap default file used by launches and helper scripts.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 28 documented key/value entries.
- Key map:
  - `identity`: mapping
  - `identity.drone_id`: `cdrone3`
  - `identity.hostname`: `cdrone3`
  - `identity.drone_namespace`: `/cdrone/cdrone3`
  - `identity.mavros_namespace`: `/cdrone/cdrone3/mavros`
  - `network`: mapping
  - `network.local_ip`: `192.168.0.194`
  - `network.peer_drone_ips`: mapping
  - `network.peer_drone_ips.cdrone4`: `192.168.0.121`
  - `network.optitrack_server`: `192.168.0.217`
  - `network.optitrack_port`: `3883`
  - `mocap`: mapping
  - `mocap.pose_source`: `optitrack`
  - `mocap.mocap_namespace`: `/cdrone/cdrone3/vrpn_mocap`
  - `mocap.rigid_body_name`: `RigidBody3`
  - `mocap.map_frame`: `map`
  - `mocap.frame_rpy_rad`: list[3]
  - `mocap.frame_rpy_rad[0]`: `0.0`
  - `mocap.frame_rpy_rad[1]`: `0.0`
  - `mocap.frame_rpy_rad[2]`: `3.141592653589793`
  - `mocap.vrpn_update_freq`: `100.0`
  - `mocap.vrpn_refresh_freq`: `1.0`
  - `mocap.vrpn_sensor_data_qos`: `True`
  - `mocap.source_best_effort`: `True`
  - `mocap.use_vrpn_timestamps`: `False`
  - `tracking`: mapping
  - `tracking.ownship_pose_topic`: `/cdrone/cdrone3/external_pose/input_pose`
  - `tracking.compare_pose_topic`: `/cdrone/cdrone3/vrpn_mocap/RigidBody2/pose`

### `ros2/src/drone_bringup/config/fun_speed_profile.yaml`

- File type: YAML configuration file.
- Tracked size: 411 bytes; 14 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 12 documented key/value entries.
- Key map:
  - `name`: `fun_speed_profile`
  - `description`: `Midpoint PX4 motion profile between the conservative indoor settings and the faster baseline.`
  - `parameters`: mapping
  - `parameters.MPC_XY_CRUISE`: `3.0`
  - `parameters.MPC_XY_VEL_MAX`: `6.0`
  - `parameters.MPC_ACC_HOR`: `2.0`
  - `parameters.MPC_JERK_AUTO`: `2.5`
  - `parameters.MPC_JERK_MAX`: `5.0`
  - `parameters.MPC_Z_V_AUTO_UP`: `1.8`
  - `parameters.MPC_Z_V_AUTO_DN`: `1.0`
  - `parameters.MPC_Z_VEL_MAX_DN`: `1.0`
  - `parameters.MPC_LAND_SPEED`: `0.5`

### `ros2/src/drone_bringup/config/hover_baseline_profile.yaml`

- File type: YAML configuration file.
- Tracked size: 521 bytes; 19 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 18 documented key/value entries.
- Key map:
  - `name`: `hover_baseline_profile`
  - `description`: `Validated hover baseline PX4 params restored between manual hover and speed-profile tests.`
  - `parameters`: mapping
  - `parameters.MIS_TAKEOFF_ALT`: `1.0`
  - `parameters.MPC_XY_VEL_MAX`: `12.0`
  - `parameters.MPC_VEL_MANUAL`: `10.0`
  - `parameters.MPC_JERK_MAX`: `8.0`
  - `parameters.MPC_XY_CRUISE`: `5.0`
  - `parameters.MPC_JERK_AUTO`: `4.0`
  - `parameters.MPC_Z_V_AUTO_UP`: `3.0`
  - `parameters.MPC_ACC_HOR`: `3.0`
  - `parameters.MPC_Z_VEL_MAX_DN`: `1.5`
  - `parameters.MPC_Z_V_AUTO_DN`: `1.5`
  - `parameters.MPC_TKO_SPEED`: `1.5`
  - `parameters.MPC_LAND_SPEED`: `0.7`
  - `parameters.CAL_ACC0_XOFF`: `-0.0842566192150116`
  - `parameters.CAL_ACC0_YOFF`: `-0.051904574036598206`
  - `parameters.CAL_ACC0_ZOFF`: `0.0773451253771782`

### `ros2/src/drone_bringup/config/indoor_speed_profile.yaml`

- File type: YAML configuration file.
- Tracked size: 311 bytes; 13 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 12 documented key/value entries.
- Key map:
  - `name`: `indoor_slow_v1`
  - `description`: `Conservative PX4 multicopter motion profile for indoor mocap tests.`
  - `parameters`: mapping
  - `parameters.MPC_XY_CRUISE`: `1.0`
  - `parameters.MPC_XY_VEL_MAX`: `1.5`
  - `parameters.MPC_ACC_HOR`: `1.0`
  - `parameters.MPC_JERK_AUTO`: `1.0`
  - `parameters.MPC_JERK_MAX`: `2.0`
  - `parameters.MPC_Z_V_AUTO_UP`: `0.8`
  - `parameters.MPC_Z_V_AUTO_DN`: `0.5`
  - `parameters.MPC_Z_VEL_MAX_DN`: `0.5`
  - `parameters.MPC_LAND_SPEED`: `0.35`

### `ros2/src/drone_bringup/config/milestone2_demo.yaml`

- File type: YAML configuration file.
- Tracked size: 1200 bytes; 47 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 40 documented key/value entries.
- Key map:
  - `milestone2_demo_sequence_node`: mapping
  - `milestone2_demo_sequence_node.ros__parameters`: mapping
  - `milestone2_demo_sequence_node.ros__parameters.scenario_id`: `milestone2_demo_v1`
  - `milestone2_demo_sequence_node.ros__parameters.required_completion_count`: `1`
  - `milestone2_demo_sequence_node.ros__parameters.dwell_time_s`: `3.0`
  - `milestone2_demo_sequence_node.ros__parameters.require_mavros_connected`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.require_armed`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.require_offboard`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.require_companion_active`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.publish_zero_on_block`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.block_target_on_perimeter_violation`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.state_timeout_s`: `1.0`
  - `milestone2_demo_sequence_node.ros__parameters.local_pose_timeout_s`: `0.5`
  - `milestone2_demo_sequence_node.ros__parameters.companion_status_timeout_s`: `0.5`
  - `milestone2_demo_sequence_node.ros__parameters.track_timeout_s`: `0.5`
  - `milestone2_demo_sequence_node.ros__parameters.track_gc_s`: `2.5`
  - `milestone2_demo_sequence_node.ros__parameters.min_track_confidence`: `0.35`
  - `milestone2_demo_sequence_node.ros__parameters.max_target_distance_m`: `2.6`
  - `milestone2_demo_sequence_node.ros__parameters.require_target_in_front`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.max_abs_target_y_m`: `4.0`
  - `milestone2_demo_sequence_node.ros__parameters.max_abs_target_z_m`: `2.5`
  - `milestone2_demo_sequence_node.ros__parameters.min_safe_distance_m`: `1.0`
  - `milestone2_demo_sequence_node.ros__parameters.follow_distance_m`: `1.0`
  - `milestone2_demo_sequence_node.ros__parameters.follow_distance_tolerance_m`: `0.2`
  - `milestone2_demo_sequence_node.ros__parameters.lateral_deadband_m`: `0.15`
  - `milestone2_demo_sequence_node.ros__parameters.vertical_deadband_m`: `0.15`
  - `milestone2_demo_sequence_node.ros__parameters.yaw_deadband_rad`: `0.08`
  - `milestone2_demo_sequence_node.ros__parameters.kp_xy`: `0.45`
  - `milestone2_demo_sequence_node.ros__parameters.kp_z`: `0.3`
  - `milestone2_demo_sequence_node.ros__parameters.kp_yaw`: `0.8`
  - `milestone2_demo_sequence_node.ros__parameters.max_vel_xy_mps`: `0.6`
  - `milestone2_demo_sequence_node.ros__parameters.max_vel_z_mps`: `0.3`
  - `milestone2_demo_sequence_node.ros__parameters.max_yaw_rate_rps`: `0.4`
  - `milestone2_demo_sequence_node.ros__parameters.enable_perimeter_guard`: `True`
  - `milestone2_demo_sequence_node.ros__parameters.perimeter_segment_sample_step_m`: `0.1`
  - `milestone2_demo_sequence_node.ros__parameters.perimeter_boundary_tolerance_m`: `0.05`
  - `milestone2_demo_sequence_node.ros__parameters.perimeter_boundary_margin_m`: `1.0`
  - `milestone2_demo_sequence_node.ros__parameters.perimeter_keep_out_margin_m`: `0.3`
  - `milestone2_demo_sequence_node.ros__parameters.perimeter_ceiling_tolerance_m`: `0.05`
  - `milestone2_demo_sequence_node.ros__parameters.projected_path_horizon_s`: `0.75`

### `ros2/src/drone_bringup/config/milestone3_demo.yaml`

- File type: YAML configuration file.
- Tracked size: 1575 bytes; 59 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 51 documented key/value entries.
- Key map:
  - `milestone3_demo_sequence_node`: mapping
  - `milestone3_demo_sequence_node.ros__parameters`: mapping
  - `milestone3_demo_sequence_node.ros__parameters.scenario_id`: `milestone3_demo_v1`
  - `milestone3_demo_sequence_node.ros__parameters.required_completion_count`: `1`
  - `milestone3_demo_sequence_node.ros__parameters.takeoff_altitude_m`: `2.6`
  - `milestone3_demo_sequence_node.ros__parameters.takeoff_rate_m_s`: `0.5`
  - `milestone3_demo_sequence_node.ros__parameters.dwell_time_s`: `3.0`
  - `milestone3_demo_sequence_node.ros__parameters.offboard_warmup_s`: `1.5`
  - `milestone3_demo_sequence_node.ros__parameters.stage_hover_duration_s`: `2.0`
  - `milestone3_demo_sequence_node.ros__parameters.require_mavros_connected`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.require_companion_active`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.land_on_complete`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.land_on_abort`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.return_to_takeoff_on_complete`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.return_position_tolerance_m`: `0.2`
  - `milestone3_demo_sequence_node.ros__parameters.return_yaw_tolerance_rad`: `0.1`
  - `milestone3_demo_sequence_node.ros__parameters.return_hold_duration_s`: `1.5`
  - `milestone3_demo_sequence_node.ros__parameters.publish_zero_on_block`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.block_target_on_perimeter_violation`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.state_timeout_s`: `3.0`
  - `milestone3_demo_sequence_node.ros__parameters.local_pose_timeout_s`: `0.5`
  - `milestone3_demo_sequence_node.ros__parameters.local_pose_timeout_during_param_sync_s`: `2.0`
  - `milestone3_demo_sequence_node.ros__parameters.companion_status_timeout_s`: `0.5`
  - `milestone3_demo_sequence_node.ros__parameters.track_timeout_s`: `0.5`
  - `milestone3_demo_sequence_node.ros__parameters.track_gc_s`: `2.5`
  - `milestone3_demo_sequence_node.ros__parameters.min_track_confidence`: `0.35`
  - `milestone3_demo_sequence_node.ros__parameters.max_target_distance_m`: `6.0`
  - `milestone3_demo_sequence_node.ros__parameters.require_target_in_front`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.max_abs_target_y_m`: `4.0`
  - `milestone3_demo_sequence_node.ros__parameters.max_abs_target_z_m`: `2.5`
  - `milestone3_demo_sequence_node.ros__parameters.min_safe_distance_m`: `1.0`
  - `milestone3_demo_sequence_node.ros__parameters.follow_distance_m`: `1.5`
  - `milestone3_demo_sequence_node.ros__parameters.follow_distance_tolerance_m`: `0.2`
  - `milestone3_demo_sequence_node.ros__parameters.lateral_deadband_m`: `0.15`
  - `milestone3_demo_sequence_node.ros__parameters.vertical_deadband_m`: `0.15`
  - `milestone3_demo_sequence_node.ros__parameters.yaw_deadband_rad`: `0.08`
  - `milestone3_demo_sequence_node.ros__parameters.kp_xy`: `0.45`
  - `milestone3_demo_sequence_node.ros__parameters.kp_z`: `0.3`
  - `milestone3_demo_sequence_node.ros__parameters.kp_yaw`: `0.8`
  - `milestone3_demo_sequence_node.ros__parameters.max_vel_xy_mps`: `0.6`
  - `milestone3_demo_sequence_node.ros__parameters.max_vel_z_mps`: `0.3`
  - `milestone3_demo_sequence_node.ros__parameters.max_yaw_rate_rps`: `0.4`
  - `milestone3_demo_sequence_node.ros__parameters.enable_perimeter_guard`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.perimeter_segment_sample_step_m`: `0.1`
  - `milestone3_demo_sequence_node.ros__parameters.perimeter_boundary_tolerance_m`: `0.05`
  - `milestone3_demo_sequence_node.ros__parameters.perimeter_boundary_margin_m`: `0.8`
  - `milestone3_demo_sequence_node.ros__parameters.perimeter_keep_out_margin_m`: `0.3`
  - `milestone3_demo_sequence_node.ros__parameters.perimeter_ceiling_tolerance_m`: `0.1`
  - `milestone3_demo_sequence_node.ros__parameters.projected_path_horizon_s`: `0.75`
  - `milestone3_demo_sequence_node.ros__parameters.use_speed_profile`: `True`
  - `milestone3_demo_sequence_node.ros__parameters.restore_speed_profile_on_exit`: `True`

### `ros2/src/drone_bringup/config/milestone3_takeoff_baseline_profile.yaml`

- File type: YAML configuration file.
- Tracked size: 373 bytes; 14 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 13 documented key/value entries.
- Key map:
  - `name`: `milestone3_takeoff_baseline_v1`
  - `description`: `Baseline PX4 motion params restored before takeoff to clear any stale indoor profile values.`
  - `parameters`: mapping
  - `parameters.MPC_XY_CRUISE`: `5.0`
  - `parameters.MPC_XY_VEL_MAX`: `12.0`
  - `parameters.MPC_ACC_HOR`: `3.0`
  - `parameters.MPC_JERK_AUTO`: `4.0`
  - `parameters.MPC_JERK_MAX`: `8.0`
  - `parameters.MPC_Z_V_AUTO_UP`: `3.0`
  - `parameters.MPC_Z_V_AUTO_DN`: `1.5`
  - `parameters.MPC_Z_VEL_MAX_DN`: `1.5`
  - `parameters.MPC_LAND_SPEED`: `0.7`
  - `parameters.MPC_TKO_SPEED`: `1.5`

### `ros2/src/drone_bringup/config/minjerk_waypoints.yaml`

- File type: YAML configuration file.
- Tracked size: 466 bytes; 30 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 32 documented key/value entries.
- Key map:
  - `name`: `minjerk_of_compare_default`
  - `frame_id`: `map`
  - `defaults`: mapping
  - `defaults.cruise_speed_mps`: `0.35`
  - `defaults.min_segment_duration_s`: `2.0`
  - `defaults.hold_s`: `0.5`
  - `defaults.final_hold_s`: `2.0`
  - `waypoints`: list[4]
  - `waypoints[0]`: mapping
  - `waypoints[0].name`: `east`
  - `waypoints[0].x_m`: `0.75`
  - `waypoints[0].y_m`: `0.0`
  - `waypoints[0].z_m`: `1.8`
  - `waypoints[0].yaw_deg`: `0.0`
  - `waypoints[1]`: mapping
  - `waypoints[1].name`: `north_east`
  - `waypoints[1].x_m`: `0.75`
  - `waypoints[1].y_m`: `0.75`
  - `waypoints[1].z_m`: `1.8`
  - `waypoints[1].yaw_deg`: `0.0`
  - `waypoints[2]`: mapping
  - `waypoints[2].name`: `north`
  - `waypoints[2].x_m`: `0.0`
  - `waypoints[2].y_m`: `0.75`
  - `waypoints[2].z_m`: `1.8`
  - `waypoints[2].yaw_deg`: `0.0`
  - `waypoints[3]`: mapping
  - `waypoints[3].name`: `home_box`
  - `waypoints[3].x_m`: `0.0`
  - `waypoints[3].y_m`: `0.0`
  - `waypoints[3].z_m`: `1.8`
  - `waypoints[3].yaw_deg`: `0.0`

### `ros2/src/drone_bringup/config/minjerk_waypoints_drift.yaml`

- File type: YAML configuration file.
- Tracked size: 1700 bytes; 92 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 104 documented key/value entries.
- Key map:
  - `name`: `minjerk_of_compare_takeoff_centered_drift`
  - `frame_id`: `map`
  - `defaults`: mapping
  - `defaults.cruise_speed_mps`: `0.28`
  - `defaults.min_segment_duration_s`: `3.0`
  - `defaults.hold_s`: `0.5`
  - `defaults.final_hold_s`: `3.0`
  - `waypoints`: list[16]
  - `waypoints[0]`: mapping
  - `waypoints[0].name`: `takeoff_column`
  - `waypoints[0].x_m`: `-9.545`
  - `waypoints[0].y_m`: `-2.391`
  - `waypoints[0].z_m`: `1.8`
  - `waypoints[0].yaw_deg`: `0.0`
  - `waypoints[1]`: mapping
  - `waypoints[1].name`: `east_short`
  - `waypoints[1].x_m`: `-9.045`
  - `waypoints[1].y_m`: `-2.391`
  - `waypoints[1].z_m`: `1.8`
  - `waypoints[1].yaw_deg`: `0.0`
  - `waypoints[2]`: mapping
  - `waypoints[2].name`: `north_east`
  - `waypoints[2].x_m`: `-9.045`
  - `waypoints[2].y_m`: `-1.891`
  - `waypoints[2].z_m`: `1.8`
  - `waypoints[2].yaw_deg`: `0.0`
  - `waypoints[3]`: mapping
  - `waypoints[3].name`: `north_short`
  - `waypoints[3].x_m`: `-9.545`
  - `waypoints[3].y_m`: `-1.891`
  - `waypoints[3].z_m`: `1.8`
  - `waypoints[3].yaw_deg`: `0.0`
  - `waypoints[4]`: mapping
  - `waypoints[4].name`: `west_short`
  - `waypoints[4].x_m`: `-10.045`
  - `waypoints[4].y_m`: `-1.891`
  - `waypoints[4].z_m`: `1.8`
  - `waypoints[4].yaw_deg`: `0.0`
  - `waypoints[5]`: mapping
  - `waypoints[5].name`: `south_west`
  - `waypoints[5].x_m`: `-10.045`
  - `waypoints[5].y_m`: `-2.391`
  - `waypoints[5].z_m`: `1.8`
  - `waypoints[5].yaw_deg`: `0.0`
  - `waypoints[6]`: mapping
  - `waypoints[6].name`: `return_takeoff_column`
  - `waypoints[6].x_m`: `-9.545`
  - `waypoints[6].y_m`: `-2.391`
  - `waypoints[6].z_m`: `1.8`
  - `waypoints[6].yaw_deg`: `0.0`
  - `waypoints[7]`: mapping
  - `waypoints[7].name`: `east_long`
  - `waypoints[7].x_m`: `-8.745`
  - `waypoints[7].y_m`: `-2.391`
  - `waypoints[7].z_m`: `1.8`
  - `waypoints[7].yaw_deg`: `0.0`
  - `waypoints[8]`: mapping
  - `waypoints[8].name`: `north_long`
  - `waypoints[8].x_m`: `-8.745`
  - `waypoints[8].y_m`: `-1.591`
  - `waypoints[8].z_m`: `1.8`
  - `waypoints[8].yaw_deg`: `0.0`
  - `waypoints[9]`: mapping
  - `waypoints[9].name`: `west_long`
  - `waypoints[9].x_m`: `-10.345`
  - `waypoints[9].y_m`: `-1.591`
  - `waypoints[9].z_m`: `1.8`
  - `waypoints[9].yaw_deg`: `0.0`
  - `waypoints[10]`: mapping
  - `waypoints[10].name`: `south_long`
  - `waypoints[10].x_m`: `-10.345`
  - `waypoints[10].y_m`: `-2.391`
  - `waypoints[10].z_m`: `1.8`
  - `waypoints[10].yaw_deg`: `0.0`
  - `waypoints[11]`: mapping
  - `waypoints[11].name`: `diagonal_ne`
  - `waypoints[11].x_m`: `-8.945`
  - `waypoints[11].y_m`: `-1.791`
  - `waypoints[11].z_m`: `1.8`
  - `waypoints[11].yaw_deg`: `0.0`
  - `waypoints[12]`: mapping
  - `waypoints[12].name`: `diagonal_sw`
  - `waypoints[12].x_m`: `-10.145`
  - `waypoints[12].y_m`: `-2.291`
  - `waypoints[12].z_m`: `1.8`
  - `waypoints[12].yaw_deg`: `0.0`
  - `waypoints[13]`: mapping
  - `waypoints[13].name`: `diagonal_nw`
  - `waypoints[13].x_m`: `-10.145`
  - `waypoints[13].y_m`: `-1.791`
  - `waypoints[13].z_m`: `1.8`
  - `waypoints[13].yaw_deg`: `0.0`
  - `waypoints[14]`: mapping
  - `waypoints[14].name`: `diagonal_se`
  - `waypoints[14].x_m`: `-8.945`
  - `waypoints[14].y_m`: `-2.291`
  - `waypoints[14].z_m`: `1.8`
  - `waypoints[14].yaw_deg`: `0.0`
  - `waypoints[15]`: mapping
  - `waypoints[15].name`: `final_takeoff_column`
  - `waypoints[15].x_m`: `-9.545`
  - `waypoints[15].y_m`: `-2.391`
  - `waypoints[15].z_m`: `1.8`
  - `waypoints[15].yaw_deg`: `0.0`

### `ros2/src/drone_bringup/config/optitrack_defaults.yaml`

- File type: YAML configuration file.
- Tracked size: 415 bytes; 16 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 16 documented key/value entries.
- Key map:
  - `optitrack_server`: `192.168.0.217`
  - `optitrack_port`: `3883`
  - `map_frame`: `map`
  - `vrpn_update_freq`: `100.0`
  - `vrpn_refresh_freq`: `1.0`
  - `vrpn_sensor_data_qos`: `True`
  - `source_best_effort`: `True`
  - `use_vrpn_timestamps`: `False`
  - `global_origin_latitude_deg`: `0.0`
  - `global_origin_longitude_deg`: `0.0`
  - `global_origin_altitude_m`: `17.1637`
  - `home_position_x_m`: `0.0`
  - `home_position_y_m`: `0.0`
  - `home_position_z_m`: `0.0`
  - `home_approach_z_m`: `1.0`
  - `reference_retry_period_s`: `1.0`

### `ros2/src/drone_bringup/config/perimeter.yaml`

- File type: YAML configuration file.
- Tracked size: 1789 bytes; 86 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 90 documented key/value entries.
- Key map:
  - `version`: `1`
  - `frame_id`: `map`
  - `calibration_source_topic`: `/cdrone/cdrone3/mavros/local_position/pose`
  - `generated_at_utc`: `2026-04-13T01:04:44.013932+00:00`
  - `capture_semantics`: mapping
  - `capture_semantics.boundary_polygon_xy_m`: `Safe inner boundary to enforce directly, not the physical wall line`
  - `capture_semantics.keep_out_polygons_xy_m`: `Blocked XY polygons the drone must not enter`
  - `altitude_limits_m`: mapping
  - `altitude_limits_m.min_z_m`: `0.35`
  - `altitude_limits_m.max_z_m`: `1.9`
  - `boundary_polygon_xy_m`: list[4]
  - `boundary_polygon_xy_m[0]`: mapping
  - `boundary_polygon_xy_m[0].name`: `north_west`
  - `boundary_polygon_xy_m[0].x_m`: `-12.8963`
  - `boundary_polygon_xy_m[0].y_m`: `12.3315`
  - `boundary_polygon_xy_m[1]`: mapping
  - `boundary_polygon_xy_m[1].name`: `north_east`
  - `boundary_polygon_xy_m[1].x_m`: `5.2392`
  - `boundary_polygon_xy_m[1].y_m`: `12.7477`
  - `boundary_polygon_xy_m[2]`: mapping
  - `boundary_polygon_xy_m[2].name`: `south_east`
  - `boundary_polygon_xy_m[2].x_m`: `3.8063`
  - `boundary_polygon_xy_m[2].y_m`: `-4.0075`
  - `boundary_polygon_xy_m[3]`: mapping
  - `boundary_polygon_xy_m[3].name`: `south_west`
  - `boundary_polygon_xy_m[3].x_m`: `-14.2857`
  - `boundary_polygon_xy_m[3].y_m`: `-4.3622`
  - `keep_out_polygons_xy_m`: list[1]
  - `keep_out_polygons_xy_m[0]`: mapping
  - `keep_out_polygons_xy_m[0].name`: `studio_pillar`
  - `keep_out_polygons_xy_m[0].vertices_xy_m`: list[4]
  - `keep_out_polygons_xy_m[0].vertices_xy_m[0]`: mapping
  - `keep_out_polygons_xy_m[0].vertices_xy_m[1]`: mapping
  - `keep_out_polygons_xy_m[0].vertices_xy_m[2]`: mapping
  - `keep_out_polygons_xy_m[0].vertices_xy_m[3]`: mapping
  - `recorded_samples`: mapping
  - `recorded_samples.boundary_vertices`: list[4]
  - `recorded_samples.boundary_vertices[0]`: mapping
  - `recorded_samples.boundary_vertices[0].name`: `north_west`
  - `recorded_samples.boundary_vertices[0].x_m`: `-12.8963`
  - `recorded_samples.boundary_vertices[0].y_m`: `12.3315`
  - `recorded_samples.boundary_vertices[0].z_m`: `0.1454`
  - `recorded_samples.boundary_vertices[0].yaw_rad`: `0.0422`
  - `recorded_samples.boundary_vertices[1]`: mapping
  - `recorded_samples.boundary_vertices[1].name`: `north_east`
  - `recorded_samples.boundary_vertices[1].x_m`: `5.2392`
  - `recorded_samples.boundary_vertices[1].y_m`: `12.7477`
  - `recorded_samples.boundary_vertices[1].z_m`: `0.1314`
  - `recorded_samples.boundary_vertices[1].yaw_rad`: `-0.0344`
  - `recorded_samples.boundary_vertices[2]`: mapping
  - `recorded_samples.boundary_vertices[2].name`: `south_east`
  - `recorded_samples.boundary_vertices[2].x_m`: `3.8063`
  - `recorded_samples.boundary_vertices[2].y_m`: `-4.0075`
  - `recorded_samples.boundary_vertices[2].z_m`: `0.1463`
  - `recorded_samples.boundary_vertices[2].yaw_rad`: `0.0587`
  - `recorded_samples.boundary_vertices[3]`: mapping
  - `recorded_samples.boundary_vertices[3].name`: `south_west`
  - `recorded_samples.boundary_vertices[3].x_m`: `-14.2857`
  - `recorded_samples.boundary_vertices[3].y_m`: `-4.3622`
  - `recorded_samples.boundary_vertices[3].z_m`: `0.1511`
  - `recorded_samples.boundary_vertices[3].yaw_rad`: `0.0695`
  - `recorded_samples.pillar_vertices`: list[4]
  - `recorded_samples.pillar_vertices[0]`: mapping
  - `recorded_samples.pillar_vertices[0].name`: `pillar_nw`
  - `recorded_samples.pillar_vertices[0].x_m`: `-7.6159`
  - `recorded_samples.pillar_vertices[0].y_m`: `6.3784`
  - `recorded_samples.pillar_vertices[0].z_m`: `1.6856`
  - `recorded_samples.pillar_vertices[0].yaw_rad`: `0.0184`
  - `recorded_samples.pillar_vertices[1]`: mapping
  - `recorded_samples.pillar_vertices[1].name`: `pillar_ne`
  - `recorded_samples.pillar_vertices[1].x_m`: `-3.4622`
  - `recorded_samples.pillar_vertices[1].y_m`: `6.1128`
  - `recorded_samples.pillar_vertices[1].z_m`: `1.696`
  - `recorded_samples.pillar_vertices[1].yaw_rad`: `0.0372`
  - `recorded_samples.pillar_vertices[2]`: mapping
  - `recorded_samples.pillar_vertices[2].name`: `pillar_se`
  - `recorded_samples.pillar_vertices[2].x_m`: `-3.6881`
  - `recorded_samples.pillar_vertices[2].y_m`: `2.5966`
  - `recorded_samples.pillar_vertices[2].z_m`: `1.7529`
  - `recorded_samples.pillar_vertices[2].yaw_rad`: `0.041`
  - `recorded_samples.pillar_vertices[3]`: mapping
  - `recorded_samples.pillar_vertices[3].name`: `pillar_sw`
  - `recorded_samples.pillar_vertices[3].x_m`: `-7.5636`
  - `recorded_samples.pillar_vertices[3].y_m`: `2.456`
  - `recorded_samples.pillar_vertices[3].z_m`: `1.7347`
  - `recorded_samples.pillar_vertices[3].yaw_rad`: `0.0551`
  - `recommended_policy`: mapping
  - `recommended_policy.reject_goals_outside_boundary`: `True`
  - `recommended_policy.block_motion_toward_violation`: `True`
  - `recommended_policy.hold_zero_velocity_on_violation`: `True`

### `ros2/src/drone_bringup/config/px4_config.yaml`

- File type: YAML configuration file.
- Tracked size: 8342 bytes; 284 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 184 documented key/value entries.
- Key map:
  - `/**`: mapping
  - `/**.ros__parameters`: mapping
  - `/**.ros__parameters.startup_px4_usb_quirk`: `False`
  - `/**/sys`: mapping
  - `/**/sys.ros__parameters`: mapping
  - `/**/sys.ros__parameters.min_voltage`: list[1]
  - `/**/sys.ros__parameters.min_voltage[0]`: `10.0`
  - `/**/sys.ros__parameters.disable_diag`: `False`
  - `/**/sys.ros__parameters.heartbeat_rate`: `1.0`
  - `/**/sys.ros__parameters.heartbeat_mav_type`: `ONBOARD_CONTROLLER`
  - `/**/sys.ros__parameters.conn_timeout`: `10.0`
  - `/**/time`: mapping
  - `/**/time.ros__parameters`: mapping
  - `/**/time.ros__parameters.time_ref_source`: `fcu`
  - `/**/time.ros__parameters.timesync_mode`: `MAVLINK`
  - `/**/time.ros__parameters.timesync_avg_alpha`: `0.6`
  - `/**/time.ros__parameters.timesync_rate`: `10.0`
  - `/**/time.ros__parameters.system_time_rate`: `1.0`
  - `/**/tdr_radio`: mapping
  - `/**/tdr_radio.ros__parameters`: mapping
  - `/**/tdr_radio.ros__parameters.low_rssi`: `40`
  - `/**/cmd`: mapping
  - `/**/cmd.ros__parameters`: mapping
  - `/**/cmd.ros__parameters.use_comp_id_system_control`: `False`
  - `/**/global_position`: mapping
  - `/**/global_position.ros__parameters`: mapping
  - `/**/global_position.ros__parameters.frame_id`: `map`
  - `/**/global_position.ros__parameters.child_frame_id`: `base_link`
  - `/**/global_position.ros__parameters.rot_covariance`: `99999.0`
  - `/**/global_position.ros__parameters.gps_uere`: `1.0`
  - `/**/global_position.ros__parameters.use_relative_alt`: `True`
  - `/**/global_position.ros__parameters.tf.send`: `False`
  - `/**/global_position.ros__parameters.tf.frame_id`: `map`
  - `/**/global_position.ros__parameters.tf.global_frame_id`: `earth`
  - `/**/global_position.ros__parameters.tf.child_frame_id`: `base_link`
  - `/**/imu`: mapping
  - `/**/imu.ros__parameters`: mapping
  - `/**/imu.ros__parameters.frame_id`: `base_link`
  - `/**/imu.ros__parameters.linear_acceleration_stdev`: `0.0003`
  - `/**/imu.ros__parameters.angular_velocity_stdev`: `0.0003490659`
  - `/**/imu.ros__parameters.orientation_stdev`: `1.0`
  - `/**/imu.ros__parameters.magnetic_stdev`: `0.0`
  - `/**/local_position`: mapping
  - `/**/local_position.ros__parameters`: mapping
  - `/**/local_position.ros__parameters.frame_id`: `map`
  - `/**/local_position.ros__parameters.tf.send`: `False`
  - `/**/local_position.ros__parameters.tf.frame_id`: `map`
  - `/**/local_position.ros__parameters.tf.child_frame_id`: `base_link`
  - `/**/local_position.ros__parameters.tf.send_fcu`: `False`
  - `/**/setpoint_accel`: mapping
  - `/**/setpoint_accel.ros__parameters`: mapping
  - `/**/setpoint_accel.ros__parameters.send_force`: `False`
  - `/**/setpoint_attitude`: mapping
  - `/**/setpoint_attitude.ros__parameters`: mapping
  - `/**/setpoint_attitude.ros__parameters.reverse_thrust`: `False`
  - `/**/setpoint_attitude.ros__parameters.use_quaternion`: `False`
  - `/**/setpoint_attitude.ros__parameters.tf.listen`: `False`
  - `/**/setpoint_attitude.ros__parameters.tf.frame_id`: `map`
  - `/**/setpoint_attitude.ros__parameters.tf.child_frame_id`: `target_attitude`
  - `/**/setpoint_attitude.ros__parameters.tf.rate_limit`: `50.0`
  - `/**/setpoint_raw`: mapping
  - `/**/setpoint_raw.ros__parameters`: mapping
  - `/**/setpoint_raw.ros__parameters.thrust_scaling`: `1.0`
  - `/**/setpoint_position`: mapping
  - `/**/setpoint_position.ros__parameters`: mapping
  - `/**/setpoint_position.ros__parameters.tf.listen`: `False`
  - `/**/setpoint_position.ros__parameters.tf.frame_id`: `map`
  - `/**/setpoint_position.ros__parameters.tf.child_frame_id`: `target_position`
  - `/**/setpoint_position.ros__parameters.tf.rate_limit`: `50.0`
  - `/**/setpoint_position.ros__parameters.mav_frame`: `LOCAL_NED`
  - `/**/guided_target`: mapping
  - `/**/guided_target.ros__parameters`: mapping
  - `/**/guided_target.ros__parameters.tf.listen`: `False`
  - `/**/guided_target.ros__parameters.tf.frame_id`: `map`
  - `/**/guided_target.ros__parameters.tf.child_frame_id`: `target_position`
  - `/**/guided_target.ros__parameters.tf.rate_limit`: `50.0`
  - `/**/setpoint_velocity`: mapping
  - `/**/setpoint_velocity.ros__parameters`: mapping
  - `/**/setpoint_velocity.ros__parameters.mav_frame`: `BODY_NED`
  - `/**/mission`: mapping
  - `/**/mission.ros__parameters`: mapping
  - `/**/mission.ros__parameters.pull_after_gcs`: `True`
  - `/**/mission.ros__parameters.use_mission_item_int`: `True`
  - `/**/distance_sensor`: mapping
  - `/**/distance_sensor.ros__parameters`: mapping
  - `/**/distance_sensor.ros__parameters.config`: `rangefinder_pub: id: 0 frame_id: "lidar" #orientation: PITCH_270 # sended by FCU field_of_view: 0.0 # XXX TODO send_t...`
  - `/**/image`: mapping
  - `/**/image.ros__parameters`: mapping
  - `/**/image.ros__parameters.frame_id`: `px4flow`
  - `/**/fake_gps`: mapping
  - `/**/fake_gps.ros__parameters`: mapping
  - `/**/fake_gps.ros__parameters.use_mocap`: `True`
  - `/**/fake_gps.ros__parameters.mocap_transform`: `False`
  - `/**/fake_gps.ros__parameters.mocap_withcovariance`: `False`
  - `/**/fake_gps.ros__parameters.use_vision`: `False`
  - `/**/fake_gps.ros__parameters.use_hil_gps`: `True`
  - `/**/fake_gps.ros__parameters.gps_id`: `4`
  - `/**/fake_gps.ros__parameters.geo_origin.lat`: `47.3667`
  - `/**/fake_gps.ros__parameters.geo_origin.lon`: `8.55`
  - `/**/fake_gps.ros__parameters.geo_origin.alt`: `408.0`
  - `/**/fake_gps.ros__parameters.eph`: `2.0`
  - `/**/fake_gps.ros__parameters.epv`: `2.0`
  - `/**/fake_gps.ros__parameters.horiz_accuracy`: `0.5`
  - `/**/fake_gps.ros__parameters.vert_accuracy`: `0.5`
  - `/**/fake_gps.ros__parameters.speed_accuracy`: `0.0`
  - `/**/fake_gps.ros__parameters.satellites_visible`: `6`
  - `/**/fake_gps.ros__parameters.fix_type`: `3`
  - `/**/fake_gps.ros__parameters.tf.listen`: `False`
  - `/**/fake_gps.ros__parameters.tf.send`: `False`
  - `/**/fake_gps.ros__parameters.tf.frame_id`: `map`
  - `/**/fake_gps.ros__parameters.tf.child_frame_id`: `fix`
  - `/**/fake_gps.ros__parameters.tf.rate_limit`: `10.0`
  - `/**/fake_gps.ros__parameters.gps_rate`: `5.0`
  - `/**/landing_target`: mapping
  - `/**/landing_target.ros__parameters`: mapping
  - `/**/landing_target.ros__parameters.listen_lt`: `False`
  - `/**/landing_target.ros__parameters.mav_frame`: `LOCAL_NED`
  - `/**/landing_target.ros__parameters.land_target_type`: `VISION_FIDUCIAL`
  - `/**/landing_target.ros__parameters.image.width`: `640`
  - `/**/landing_target.ros__parameters.image.height`: `480`
  - ... 64 additional nested entries omitted from this generated map; inspect the file for exhaustive scalar values.

### `ros2/src/drone_bringup/config/px4_params.yaml`

- File type: YAML configuration file.
- Tracked size: 241 bytes; 5 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 3 documented key/value entries.
- Key map:
  - `mavros_node`: mapping
  - `mavros_node.ros__parameters`: mapping
  - `mavros_node.ros__parameters.fcu_url`: `udp://:14540@127.0.0.1:14550`

### `ros2/src/drone_bringup/config/px4_pluginlists.yaml`

- File type: YAML configuration file.
- Tracked size: 516 bytes; 20 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 6 documented key/value entries.
- Key map:
  - `/**`: mapping
  - `/**.ros__parameters`: mapping
  - `/**.ros__parameters.plugin_denylist`: list[3]
  - `/**.ros__parameters.plugin_denylist[0]`: `guided_target`
  - `/**.ros__parameters.plugin_denylist[1]`: `wheel_odometry`
  - `/**.ros__parameters.plugin_denylist[2]`: `odometry`

### `ros2/src/drone_bringup/config/realsense_d455_tracking.yaml`

- File type: YAML configuration file.
- Tracked size: 347 bytes; 15 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 15 documented key/value entries.
- Key map:
  - `camera_namespace`: `camera`
  - `camera_name`: `camera`
  - `enable_color`: `True`
  - `enable_depth`: `True`
  - `enable_infra1`: `False`
  - `enable_infra2`: `False`
  - `enable_gyro`: `False`
  - `enable_accel`: `False`
  - `enable_sync`: `True`
  - `pointcloud.enable`: `False`
  - `align_depth.enable`: `True`
  - `publish_tf`: `False`
  - `initial_reset`: `False`
  - `rgb_camera.color_profile`: `848x480x15`
  - `depth_module.depth_profile`: `848x480x15`

### `ros2/src/drone_bringup/config/realsense_d455_vio.yaml`

- File type: YAML configuration file.
- Tracked size: 448 bytes; 20 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 20 documented key/value entries.
- Key map:
  - `camera_namespace`: `camera`
  - `camera_name`: `d455`
  - `enable_color`: `False`
  - `enable_depth`: `False`
  - `enable_infra1`: `True`
  - `enable_infra2`: `True`
  - `enable_gyro`: `True`
  - `enable_accel`: `True`
  - `enable_sync`: `True`
  - `unite_imu_method`: `2`
  - `pointcloud.enable`: `False`
  - `align_depth.enable`: `False`
  - `publish_tf`: `True`
  - `tf_publish_rate`: `30.0`
  - `initial_reset`: `False`
  - `depth_module.depth_profile`: `640x360x90`
  - `depth_module.infra_profile`: `640x360x90`
  - `depth_module.emitter_enabled`: `0`
  - `gyro_fps`: `200`
  - `accel_fps`: `200`

### `ros2/src/drone_bringup/config/target_follow_safety.yaml`

- File type: YAML configuration file.
- Tracked size: 948 bytes; 38 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 31 documented key/value entries.
- Key map:
  - `target_follow_controller_node`: mapping
  - `target_follow_controller_node.ros__parameters`: mapping
  - `target_follow_controller_node.ros__parameters.follow_enabled_on_startup`: `False`
  - `target_follow_controller_node.ros__parameters.require_mavros_connected`: `True`
  - `target_follow_controller_node.ros__parameters.require_armed`: `True`
  - `target_follow_controller_node.ros__parameters.require_offboard`: `True`
  - `target_follow_controller_node.ros__parameters.require_companion_active`: `True`
  - `target_follow_controller_node.ros__parameters.publish_zero_on_block`: `True`
  - `target_follow_controller_node.ros__parameters.publish_zero_on_lost_target`: `True`
  - `target_follow_controller_node.ros__parameters.state_timeout_s`: `1.0`
  - `target_follow_controller_node.ros__parameters.local_pose_timeout_s`: `0.5`
  - `target_follow_controller_node.ros__parameters.companion_status_timeout_s`: `0.5`
  - `target_follow_controller_node.ros__parameters.track_timeout_s`: `0.5`
  - `target_follow_controller_node.ros__parameters.track_gc_s`: `2.5`
  - `target_follow_controller_node.ros__parameters.min_track_confidence`: `0.35`
  - `target_follow_controller_node.ros__parameters.max_target_distance_m`: `8.0`
  - `target_follow_controller_node.ros__parameters.require_target_in_front`: `True`
  - `target_follow_controller_node.ros__parameters.max_abs_target_y_m`: `4.0`
  - `target_follow_controller_node.ros__parameters.max_abs_target_z_m`: `2.5`
  - `target_follow_controller_node.ros__parameters.min_safe_distance_m`: `1.0`
  - `target_follow_controller_node.ros__parameters.follow_distance_m`: `3.0`
  - `target_follow_controller_node.ros__parameters.follow_distance_tolerance_m`: `0.25`
  - `target_follow_controller_node.ros__parameters.lateral_deadband_m`: `0.15`
  - `target_follow_controller_node.ros__parameters.vertical_deadband_m`: `0.15`
  - `target_follow_controller_node.ros__parameters.yaw_deadband_rad`: `0.08`
  - `target_follow_controller_node.ros__parameters.kp_xy`: `0.45`
  - `target_follow_controller_node.ros__parameters.kp_z`: `0.3`
  - `target_follow_controller_node.ros__parameters.kp_yaw`: `0.8`
  - `target_follow_controller_node.ros__parameters.max_vel_xy_mps`: `1.0`
  - `target_follow_controller_node.ros__parameters.max_vel_z_mps`: `0.5`
  - `target_follow_controller_node.ros__parameters.max_yaw_rate_rps`: `0.6`

### `ros2/src/drone_bringup/launch/altctl_demo.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 5453 bytes; 131 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `launch_setup(context, *args, **kwargs)` (lines 17-100): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration, IncludeLaunchDescription, Node, PythonLaunchDescriptionSource, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 103-131): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, default_arg.

### `ros2/src/drone_bringup/launch/bench_offboard.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 4857 bytes; 114 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `launch_setup(context, *args, **kwargs)` (lines 14-83): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration, IncludeLaunchDescription, Node, PythonLaunchDescriptionSource, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 87-114): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, default_arg.

### `ros2/src/drone_bringup/launch/combined.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 600 bytes; 19 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import IncludeLaunchDescription; from launch.launch_description_sources import PythonLaunchDescriptionSource`
- Top-level functions:
  - `generate_launch_description()` (lines 9-19): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchDescription, IncludeLaunchDescription, PythonLaunchDescriptionSource, os.path.join.

### `ros2/src/drone_bringup/launch/drone.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 1762 bytes; 49 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - third-party: `import yaml`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, OpaqueFunction; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `launch_setup(context, *args, **kwargs)` (lines 12-35): No docstring; behavior is described from its body and call sites.
    Code map: 1 context managers; 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, os.path.join, mavros_param_dict.get.get, Node, yaml.safe_load, mavros_param_dict.get, LaunchConfiguration.
  - `generate_launch_description()` (lines 38-49): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 16996 bytes; 393 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.conditions import IfCondition; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_load_optitrack_defaults() -> dict[str, object]` (lines 17-18): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults.
  - `_join_topic(namespace: str, leaf: str) -> str` (lines 21-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: str.strip, namespace.rstrip, str.strip.lstrip, namespace.startswith.
  - `launch_setup(context, *args, **kwargs)` (lines 30-248): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 return points; 1 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, LaunchConfiguration, LaunchConfiguration.perform.strip, LaunchConfiguration.perform.strip.lower, _join_topic, IncludeLaunchDescription, nodes.append, PythonLaunchDescriptionSource, Node, LaunchConfiguration.perform, os.path.join, Dict.items, IfCondition, ValueError, ParameterValue.
  - `generate_launch_description()` (lines 251-393): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _load_optitrack_defaults, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/milestone2_demo.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 14131 bytes; 330 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_load_optitrack_defaults() -> dict[str, object]` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults.
  - `launch_setup(context, *args, **kwargs)` (lines 20-152): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration, IncludeLaunchDescription, Node, PythonLaunchDescriptionSource, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 155-330): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, _load_optitrack_defaults, os.path.join, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/milestone3_demo.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 31144 bytes; 720 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from pathlib import Path; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level constants/state: `_SPEED_PROFILE_FILENAME_BY_NAME`.
- Top-level functions:
  - `_load_optitrack_defaults() -> dict[str, object]` (lines 30-31): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults.
  - `_launch_arg_as_bool(context, name: str, default: bool = False) -> bool` (lines 34-38): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: LaunchConfiguration.perform.strip.lower, LaunchConfiguration.perform.strip, LaunchConfiguration.perform, LaunchConfiguration.
  - `launch_setup(context, *args, **kwargs)` (lines 41-395): No docstring; behavior is described from its body and call sites.
    Code map: 7 conditional branches; 1 return points; 1 raise statements.
    Notable calls: get_package_share_directory, Path.resolve, LaunchConfiguration, LaunchConfiguration.perform.strip, LaunchConfiguration.perform.strip.lower, IncludeLaunchDescription, Node, _launch_arg_as_bool, _SPEED_PROFILE_FILENAME_BY_NAME.get, os.path.join, PythonLaunchDescriptionSource, led_script_path.is_file, tracking_metrics_actions.append, Path, LaunchConfiguration.perform, ', '.join, ValueError, Dict.items....
  - `generate_launch_description()` (lines 398-720): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, _load_optitrack_defaults, os.path.join, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/milestone4_demo.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 23327 bytes; 513 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_load_defaults() -> dict[str, object]` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults.
  - `_join_topic(namespace: str, leaf: str) -> str` (lines 20-26): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: str.strip, namespace.rstrip, str.strip.lstrip, namespace.startswith.
  - `_cdrone_topic(drone_namespace: str, drone_id: str, leaf: str) -> str` (lines 29-34): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: str.strip.rstrip, str.strip.strip, _join_topic, _join_topic.rstrip, str.strip, namespace.endswith.
  - `launch_setup(context, *args, **kwargs)` (lines 37-273): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration.perform.strip, Node, IncludeLaunchDescription, _cdrone_topic, PythonLaunchDescriptionSource, LaunchConfiguration.perform, LaunchConfiguration, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 276-513): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, _load_defaults, os.path.join, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/minjerk_of_compare.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 12471 bytes; 291 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_cdrone_topic(defaults: dict[str, object], leaf: str) -> str` (lines 17-20): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip.strip, str.strip, leaf.strip, defaults.get.
  - `launch_setup(context, *args, **kwargs)` (lines 23-159): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration, IncludeLaunchDescription, Node, PythonLaunchDescriptionSource, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 162-291): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: load_drone_launch_defaults, get_package_share_directory, os.path.join, _cdrone_topic, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg, defaults.get.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/position_circle_demo.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 17409 bytes; 331 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `generate_launch_description()` (lines 11-331): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, load_drone_launch_defaults, IncludeLaunchDescription, LaunchDescription, PythonLaunchDescriptionSource, os.path.join, Dict.items, DeclareLaunchArgument, default_arg, LaunchConfiguration.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/position_goto_demo.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 24320 bytes; 523 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_load_optitrack_defaults() -> dict[str, object]` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults.
  - `launch_setup(context, *args, **kwargs)` (lines 20-297): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration, IncludeLaunchDescription, Node, PythonLaunchDescriptionSource, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 300-523): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: _load_optitrack_defaults, get_package_share_directory, os.path.join, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/position_hover_demo.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 17283 bytes; 370 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_load_optitrack_defaults() -> dict[str, object]` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults.
  - `launch_setup(context, *args, **kwargs)` (lines 20-202): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration, IncludeLaunchDescription, Node, PythonLaunchDescriptionSource, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 205-370): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: _load_optitrack_defaults, get_package_share_directory, os.path.join, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/realsense_d455.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 2993 bytes; 80 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; from ament_index_python.packages import get_package_share_directory`
  - third-party: `import yaml`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration, PathJoinSubstitution; from launch_ros.substitutions import FindPackageShare`
- Top-level functions:
  - `_load_profile(path: str) -> dict[str, object]` (lines 12-19): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 return points; 2 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: Path, profile_path.exists, FileNotFoundError, yaml.safe_load, isinstance, ValueError, profile_path.read_text.
  - `_to_launch_value(value: object) -> str` (lines 22-25): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: isinstance.
  - `launch_setup(context, *args, **kwargs)` (lines 28-64): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, LaunchConfiguration.perform.strip, _load_profile, PathJoinSubstitution, _to_launch_value, IncludeLaunchDescription, Path, LaunchConfiguration.perform, FindPackageShare, profile.items, PythonLaunchDescriptionSource, launch_arguments.items, LaunchConfiguration.
  - `generate_launch_description()` (lines 67-80): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, Path.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/target_follow.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 11056 bytes; 251 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; from launch_ros.parameter_descriptions import ParameterValue`
  - local/project: `from drone_control_pkg.deployment_config import default_arg as _default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_load_optitrack_defaults() -> dict[str, object]` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults.
  - `launch_setup(context, *args, **kwargs)` (lines 20-120): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchConfiguration, IncludeLaunchDescription, Node, PythonLaunchDescriptionSource, os.path.join, Dict.items, ParameterValue.
  - `generate_launch_description()` (lines 123-251): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, _load_optitrack_defaults, os.path.join, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, _default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/tracking_only.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 5887 bytes; 131 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `_vision_launch_path(package_name: str, leaf_name: str) -> str` (lines 14-15): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: os.path.join, get_package_share_directory.
  - `generate_launch_description()` (lines 18-131): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, load_drone_launch_defaults, LaunchDescription, DeclareLaunchArgument, IncludeLaunchDescription, PythonLaunchDescriptionSource, default_arg, os.path.join, _vision_launch_path, Dict.items, LaunchConfiguration.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_bringup/launch/vslam_only.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 602 bytes; 19 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import IncludeLaunchDescription; from launch.launch_description_sources import PythonLaunchDescriptionSource`
- Top-level functions:
  - `generate_launch_description()` (lines 9-19): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, LaunchDescription, IncludeLaunchDescription, PythonLaunchDescriptionSource, os.path.join.

### `ros2/src/drone_bringup/launch/vslam_px4_bridge.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 2421 bytes; 53 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `generate_launch_description()` (lines 11-53): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: get_package_share_directory, load_drone_launch_defaults, IncludeLaunchDescription, LaunchDescription, PythonLaunchDescriptionSource, os.path.join, Dict.items, DeclareLaunchArgument, default_arg, LaunchConfiguration.

### `ros2/src/drone_bringup/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 1304 bytes; 33 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_bringup`.
- Dependencies: `buildtool_depend:ament_cmake, exec_depend:launch, exec_depend:launch_ros, exec_depend:mavros, exec_depend:mavros_extras, exec_depend:vrpn_mocap, exec_depend:realsense2_camera, exec_depend:cv_bridge, exec_depend:image_transport, exec_depend:imu_filter_madgwick, exec_depend:python3-yaml, exec_depend:tf2_geometry_msgs, exec_depend:tf2_ros, exec_depend:drone_control_pkg, exec_depend:drone_vision_pkg, exec_depend:ros2_poselib, test_depend:ament_lint_auto, test_depend:ament_lint_common`.
- Build type: `ament_cmake`.

## Active ROS 2 package: drone_control_pkg

### `ros2/src/drone_control_pkg/config/px4_config.yaml`

- File type: YAML configuration file.
- Tracked size: 8342 bytes; 284 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 184 documented key/value entries.
- Key map:
  - `/**`: mapping
  - `/**.ros__parameters`: mapping
  - `/**.ros__parameters.startup_px4_usb_quirk`: `False`
  - `/**/sys`: mapping
  - `/**/sys.ros__parameters`: mapping
  - `/**/sys.ros__parameters.min_voltage`: list[1]
  - `/**/sys.ros__parameters.min_voltage[0]`: `10.0`
  - `/**/sys.ros__parameters.disable_diag`: `False`
  - `/**/sys.ros__parameters.heartbeat_rate`: `1.0`
  - `/**/sys.ros__parameters.heartbeat_mav_type`: `ONBOARD_CONTROLLER`
  - `/**/sys.ros__parameters.conn_timeout`: `10.0`
  - `/**/time`: mapping
  - `/**/time.ros__parameters`: mapping
  - `/**/time.ros__parameters.time_ref_source`: `fcu`
  - `/**/time.ros__parameters.timesync_mode`: `MAVLINK`
  - `/**/time.ros__parameters.timesync_avg_alpha`: `0.6`
  - `/**/time.ros__parameters.timesync_rate`: `10.0`
  - `/**/time.ros__parameters.system_time_rate`: `1.0`
  - `/**/tdr_radio`: mapping
  - `/**/tdr_radio.ros__parameters`: mapping
  - `/**/tdr_radio.ros__parameters.low_rssi`: `40`
  - `/**/cmd`: mapping
  - `/**/cmd.ros__parameters`: mapping
  - `/**/cmd.ros__parameters.use_comp_id_system_control`: `False`
  - `/**/global_position`: mapping
  - `/**/global_position.ros__parameters`: mapping
  - `/**/global_position.ros__parameters.frame_id`: `map`
  - `/**/global_position.ros__parameters.child_frame_id`: `base_link`
  - `/**/global_position.ros__parameters.rot_covariance`: `99999.0`
  - `/**/global_position.ros__parameters.gps_uere`: `1.0`
  - `/**/global_position.ros__parameters.use_relative_alt`: `True`
  - `/**/global_position.ros__parameters.tf.send`: `False`
  - `/**/global_position.ros__parameters.tf.frame_id`: `map`
  - `/**/global_position.ros__parameters.tf.global_frame_id`: `earth`
  - `/**/global_position.ros__parameters.tf.child_frame_id`: `base_link`
  - `/**/imu`: mapping
  - `/**/imu.ros__parameters`: mapping
  - `/**/imu.ros__parameters.frame_id`: `base_link`
  - `/**/imu.ros__parameters.linear_acceleration_stdev`: `0.0003`
  - `/**/imu.ros__parameters.angular_velocity_stdev`: `0.0003490659`
  - `/**/imu.ros__parameters.orientation_stdev`: `1.0`
  - `/**/imu.ros__parameters.magnetic_stdev`: `0.0`
  - `/**/local_position`: mapping
  - `/**/local_position.ros__parameters`: mapping
  - `/**/local_position.ros__parameters.frame_id`: `map`
  - `/**/local_position.ros__parameters.tf.send`: `False`
  - `/**/local_position.ros__parameters.tf.frame_id`: `map`
  - `/**/local_position.ros__parameters.tf.child_frame_id`: `base_link`
  - `/**/local_position.ros__parameters.tf.send_fcu`: `False`
  - `/**/setpoint_accel`: mapping
  - `/**/setpoint_accel.ros__parameters`: mapping
  - `/**/setpoint_accel.ros__parameters.send_force`: `False`
  - `/**/setpoint_attitude`: mapping
  - `/**/setpoint_attitude.ros__parameters`: mapping
  - `/**/setpoint_attitude.ros__parameters.reverse_thrust`: `False`
  - `/**/setpoint_attitude.ros__parameters.use_quaternion`: `False`
  - `/**/setpoint_attitude.ros__parameters.tf.listen`: `False`
  - `/**/setpoint_attitude.ros__parameters.tf.frame_id`: `map`
  - `/**/setpoint_attitude.ros__parameters.tf.child_frame_id`: `target_attitude`
  - `/**/setpoint_attitude.ros__parameters.tf.rate_limit`: `50.0`
  - `/**/setpoint_raw`: mapping
  - `/**/setpoint_raw.ros__parameters`: mapping
  - `/**/setpoint_raw.ros__parameters.thrust_scaling`: `1.0`
  - `/**/setpoint_position`: mapping
  - `/**/setpoint_position.ros__parameters`: mapping
  - `/**/setpoint_position.ros__parameters.tf.listen`: `False`
  - `/**/setpoint_position.ros__parameters.tf.frame_id`: `map`
  - `/**/setpoint_position.ros__parameters.tf.child_frame_id`: `target_position`
  - `/**/setpoint_position.ros__parameters.tf.rate_limit`: `50.0`
  - `/**/setpoint_position.ros__parameters.mav_frame`: `LOCAL_NED`
  - `/**/guided_target`: mapping
  - `/**/guided_target.ros__parameters`: mapping
  - `/**/guided_target.ros__parameters.tf.listen`: `False`
  - `/**/guided_target.ros__parameters.tf.frame_id`: `map`
  - `/**/guided_target.ros__parameters.tf.child_frame_id`: `target_position`
  - `/**/guided_target.ros__parameters.tf.rate_limit`: `50.0`
  - `/**/setpoint_velocity`: mapping
  - `/**/setpoint_velocity.ros__parameters`: mapping
  - `/**/setpoint_velocity.ros__parameters.mav_frame`: `BODY_NED`
  - `/**/mission`: mapping
  - `/**/mission.ros__parameters`: mapping
  - `/**/mission.ros__parameters.pull_after_gcs`: `True`
  - `/**/mission.ros__parameters.use_mission_item_int`: `True`
  - `/**/distance_sensor`: mapping
  - `/**/distance_sensor.ros__parameters`: mapping
  - `/**/distance_sensor.ros__parameters.config`: `rangefinder_pub: id: 0 frame_id: "lidar" #orientation: PITCH_270 # sended by FCU field_of_view: 0.0 # XXX TODO send_t...`
  - `/**/image`: mapping
  - `/**/image.ros__parameters`: mapping
  - `/**/image.ros__parameters.frame_id`: `px4flow`
  - `/**/fake_gps`: mapping
  - `/**/fake_gps.ros__parameters`: mapping
  - `/**/fake_gps.ros__parameters.use_mocap`: `True`
  - `/**/fake_gps.ros__parameters.mocap_transform`: `False`
  - `/**/fake_gps.ros__parameters.mocap_withcovariance`: `False`
  - `/**/fake_gps.ros__parameters.use_vision`: `False`
  - `/**/fake_gps.ros__parameters.use_hil_gps`: `True`
  - `/**/fake_gps.ros__parameters.gps_id`: `4`
  - `/**/fake_gps.ros__parameters.geo_origin.lat`: `47.3667`
  - `/**/fake_gps.ros__parameters.geo_origin.lon`: `8.55`
  - `/**/fake_gps.ros__parameters.geo_origin.alt`: `408.0`
  - `/**/fake_gps.ros__parameters.eph`: `2.0`
  - `/**/fake_gps.ros__parameters.epv`: `2.0`
  - `/**/fake_gps.ros__parameters.horiz_accuracy`: `0.5`
  - `/**/fake_gps.ros__parameters.vert_accuracy`: `0.5`
  - `/**/fake_gps.ros__parameters.speed_accuracy`: `0.0`
  - `/**/fake_gps.ros__parameters.satellites_visible`: `6`
  - `/**/fake_gps.ros__parameters.fix_type`: `3`
  - `/**/fake_gps.ros__parameters.tf.listen`: `False`
  - `/**/fake_gps.ros__parameters.tf.send`: `False`
  - `/**/fake_gps.ros__parameters.tf.frame_id`: `map`
  - `/**/fake_gps.ros__parameters.tf.child_frame_id`: `fix`
  - `/**/fake_gps.ros__parameters.tf.rate_limit`: `10.0`
  - `/**/fake_gps.ros__parameters.gps_rate`: `5.0`
  - `/**/landing_target`: mapping
  - `/**/landing_target.ros__parameters`: mapping
  - `/**/landing_target.ros__parameters.listen_lt`: `False`
  - `/**/landing_target.ros__parameters.mav_frame`: `LOCAL_NED`
  - `/**/landing_target.ros__parameters.land_target_type`: `VISION_FIDUCIAL`
  - `/**/landing_target.ros__parameters.image.width`: `640`
  - `/**/landing_target.ros__parameters.image.height`: `480`
  - ... 64 additional nested entries omitted from this generated map; inspect the file for exhaustive scalar values.

### `ros2/src/drone_control_pkg/config/px4_params.yaml`

- File type: YAML configuration file.
- Tracked size: 66 bytes; 3 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 3 documented key/value entries.
- Key map:
  - `mavros_node`: mapping
  - `mavros_node.ros__parameters`: mapping
  - `mavros_node.ros__parameters.fcu_url`: `/dev/ttyACM0:921600`

### `ros2/src/drone_control_pkg/config/px4_pluginlists.yaml`

- File type: YAML configuration file.
- Tracked size: 317 bytes; 18 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 4 documented key/value entries.
- Key map:
  - `/**`: mapping
  - `/**.ros__parameters`: mapping
  - `/**.ros__parameters.plugin_denylist`: list[1]
  - `/**.ros__parameters.plugin_denylist[0]`: `wheel_odometry`

### `ros2/src/drone_control_pkg/drone_control_pkg/__init__.py`

- File type: Python source file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: Provides executable or importable Python behavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `ros2/src/drone_control_pkg/drone_control_pkg/altctl_demo_sequence_node.py`

- File type: Python source file.
- Tracked size: 22914 bytes; 619 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes AltctlDemoSequenceNode; defines functions clamp, quaternion_to_yaw_rad, project_forward_distance, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; from typing import Optional, Tuple; from std_srvs.srv import Trigger`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import ManualControl, State; from mavros_msgs.srv import CommandBool, SetMode; from rclpy.node import Node; from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy; from std_msgs.msg import String`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.topic_utils import cdrone_topic, join_topic`
- Classes:
  - `AltctlDemoSequenceNode` (lines 42-601, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `publish_rate_hz, drone_id, mavros_namespace, target_altitude_m, forward_distance_m, hover_after_takeoff_s, hover_after_translate_s, arm_zero_throttle_hold_s, takeoff_throttle_delta, hover_throttle_center, forward_stick_cmd, altitude_tolerance_m, position_tolerance_m, stage_timeout_s, land_detect_altitude_m, manual_control_topic, state_topic, local_pose_topic, start_service_name, abort_service_name, status_topic, mode_service, arm_service, pose_timeout_s, state_timeout_s, touchdown_dwell_s, altitude_kp, latest_state, latest_pose, last_state_time_s, last_pose_time_s, demo_state, stage_started_s, forward_start_xy, forward_heading_rad, touchdown_started_s, pending_mode_name, mode_future, pend...` plus more.
    - `__init__(self) -> None` (lines 59-209): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, join_topic, State, PoseStamped, self.now_s, self.create_publisher, QoSProfile, self.create_subscription, self.create_client, self.create_service, self.create_timer, self.publish_status, configured_drone_id, configured_mavros_namespace, str.strip, cdrone_topic, self.get_parameter.
    - `now_s(self) -> float` (lines 211-212): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `state_callback(self, msg: State) -> None` (lines 214-216): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 218-220): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `current_altitude_m(self) -> float` (lines 222-223): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_xy(self) -> Tuple[float, float]` (lines 225-229): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_yaw_rad(self) -> float` (lines 231-233): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: quaternion_to_yaw_rad.
    - `pose_fresh(self) -> bool` (lines 235-236): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `state_fresh(self) -> bool` (lines 238-239): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `is_altctl_mode(self) -> bool` (lines 241-242): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `is_land_mode(self) -> bool` (lines 244-245): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `startable(self) -> Tuple[bool, str]` (lines 247-264): No docstring; behavior is described from its body and call sites.
      Code map: 8 conditional branches; 9 return points.
      Notable calls: self.mode_client.wait_for_service, self.arm_client.wait_for_service, self.state_fresh, self.pose_fresh.
    - `handle_start_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 266-284): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.startable, self.transition_to.
    - `handle_abort_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 286-298): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.enter_abort.
    - `publish_status(self) -> None` (lines 300-303): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: String, self.status_pub.publish.
    - `publish_manual(self, x_cmd: float = 0.0, y_cmd: float = 0.0, throttle_cmd: Optional[float] = None, yaw_cmd: float = 0.0) -> None` (lines 305-321): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: ManualControl, self.manual_pub.publish, clamp.
    - `transition_to(self, new_state: str, reason: str = '') -> None` (lines 323-338): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches.
      Notable calls: self.now_s, self.publish_status, self.current_xy, self.current_yaw_rad.
    - `enter_abort(self, reason: str) -> None` (lines 340-351): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.publish_manual, self.transition_to.
    - `stage_elapsed_s(self) -> float` (lines 353-354): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `stage_timeout_limit_s(self) -> Optional[float]` (lines 356-365): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
    - `forward_progress_m(self) -> float` (lines 367-374): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: project_forward_distance, self.current_xy.
    - `altitude_hold_throttle(self, target_altitude_m: float) -> float` (lines 376-383): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: clamp, self.current_altitude_m.
    - `request_mode(self, mode_name: str) -> None` (lines 385-396): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: requests PX4/MAVROS mode changes.
      Notable calls: SetMode.Request, self.mode_client.call_async, self.mode_client.wait_for_service, self.enter_abort.
    - `request_arm(self, arm_value: bool) -> None` (lines 398-411): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: arms/disarms through MAVROS.
      Notable calls: CommandBool.Request, self.arm_client.call_async, self.arm_client.wait_for_service, self.enter_abort.
    - `poll_service_futures(self) -> None` (lines 413-441): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 2 try/except blocks.
      Notable calls: self.mode_future.done, self.arm_future.done, self.mode_future.result, self.arm_future.result, self.enter_abort, action.capitalize.
    - `run_safety_checks(self) -> None` (lines 443-474): No docstring; behavior is described from its body and call sites.
      Code map: 8 conditional branches; 6 return points.
      Notable calls: self.stage_timeout_limit_s, self.state_fresh, self.enter_abort, self.pose_fresh, self.is_altctl_mode, self.stage_elapsed_s, self.is_land_mode.
    - `step_state_machine(self) -> None` (lines 476-560): No docstring; behavior is described from its body and call sites.
      Code map: 29 conditional branches; 13 return points.
      Notable calls: self.is_altctl_mode, self.is_land_mode, self.transition_to, self.request_arm, self.current_altitude_m, self.stage_elapsed_s, self.forward_progress_m, self.request_mode, self.now_s.
    - `publish_manual_for_state(self) -> None` (lines 562-594): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 3 return points.
      Notable calls: self.publish_manual, self.altitude_hold_throttle, clamp.
    - `timer_callback(self) -> None` (lines 596-601): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.poll_service_futures, self.run_safety_checks, self.step_state_machine, self.publish_manual_for_state, self.publish_status.
- Top-level functions:
  - `clamp(value: float, min_v: float, max_v: float) -> float` (lines 22-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `quaternion_to_yaw_rad(w: float, x: float, y: float, z: float) -> float` (lines 26-29): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.atan2.
  - `project_forward_distance(start_xy: Tuple[float, float], current_xy: Tuple[float, float], heading_rad: float) -> float` (lines 32-39): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.cos, math.sin.
  - `main(args = None) -> None` (lines 604-615): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, AltctlDemoSequenceNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, publishes messages, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, handles pose messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/bench_vision_pose_node.py`

- File type: Python source file.
- Tracked size: 4343 bytes; 128 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes BenchVisionPoseNode; defines functions quaternion_from_euler, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from math import cos, sin`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import CompanionProcessStatus; from rclpy.node import Node`
  - local/project: `from drone_control_pkg.deployment_config import configured_mavros_namespace; from drone_control_pkg.topic_utils import join_topic`
- Classes:
  - `BenchVisionPoseNode` (lines 33-111, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `publish_rate_hz, frame_id, x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad, publish_companion_status, mavros_namespace, pose_pub, status_pub, timer, declare_parameter, create_publisher, create_timer, publish_loop, get_parameter, get_logger, get_clock`.
    - `__init__(self) -> None` (lines 34-82): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, uses timer callbacks, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, self.create_publisher, self.create_timer, configured_mavros_namespace, join_topic, self.get_parameter.
    - `publish_loop(self) -> None` (lines 84-111): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: publishes messages, handles pose messages.
      Notable calls: quaternion_from_euler, PoseStamped, self.pose_pub.publish, CompanionProcessStatus, self.status_pub.publish.
- Top-level functions:
  - `quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]` (lines 12-30): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: cos, sin.
  - `main(args = None) -> None` (lines 114-124): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, BenchVisionPoseNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, uses timer callbacks, publishes messages, handles pose messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/deployment_config.py`

- File type: Python source file.
- Tracked size: 6281 bytes; 185 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines functions _share_config_path, _load_yaml_mapping, _merge_mappings, _flatten_config_sections, _normalize_ns, _join_topic, load_drone_launch_defaults, get_default_value, default_arg, configured_drone_id and more.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from functools import lru_cache; from pathlib import Path; from typing import Any; from ament_index_python.packages import get_package_share_directory`
  - third-party: `import yaml`
- Top-level constants/state: `_DRONE_BRINGUP_PACKAGE, _SHARED_DEFAULTS_FILE, _DRONE_ID_CONFIG_FILE, _CONFIG_SECTIONS_TO_FLATTEN`.
- Top-level functions:
  - `_share_config_path(filename: str) -> Path` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Path, get_package_share_directory.
  - `_load_yaml_mapping(path: Path, *, required: bool) -> dict[str, Any]` (lines 20-29): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 2 return points; 2 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: path.is_file, yaml.safe_load, isinstance, ValueError, FileNotFoundError, path.read_text.
  - `_merge_mappings(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]` (lines 32-40): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points.
    Notable calls: override.items, merged.get, isinstance, _merge_mappings.
  - `_flatten_config_sections(config: dict[str, Any]) -> dict[str, Any]` (lines 43-50): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points.
    Notable calls: config.items, isinstance, _merge_mappings.
  - `_normalize_ns(namespace: object) -> str` (lines 53-59): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: str.strip, namespace.rstrip, namespace.startswith.
  - `_join_topic(namespace: object, leaf: object) -> str` (lines 62-65): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _normalize_ns, str.strip.lstrip, str.strip.
  - `load_drone_launch_defaults() -> dict[str, Any]` (lines 69-109): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: lru_cache, _load_yaml_mapping, _flatten_config_sections, _merge_mappings, str.strip.strip, _normalize_ns, str.strip, _share_config_path, defaults.get, _join_topic.
  - `get_default_value(key: str, fallback: Any) -> Any` (lines 112-117): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: load_drone_launch_defaults, defaults.get.
  - `default_arg(defaults: dict[str, Any], key: str, fallback: Any) -> str` (lines 120-126): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: defaults.get, isinstance.
  - `configured_drone_id() -> str` (lines 129-130): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip, get_default_value.
  - `configured_hostname() -> str` (lines 133-134): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip, get_default_value, configured_drone_id.
  - `configured_local_ip() -> str` (lines 137-138): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip, get_default_value.
  - `configured_mavros_namespace() -> str` (lines 141-146): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip, configured_drone_id.strip.strip, configured_drone_id.strip, get_default_value, configured_drone_id.
  - `configured_drone_namespace() -> str` (lines 149-154): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip, configured_drone_id.strip.strip, _normalize_ns, configured_drone_id.strip, get_default_value, configured_drone_id.
  - `configured_mocap_namespace() -> str` (lines 157-161): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip, _join_topic, _normalize_ns, configured_drone_namespace, get_default_value.
  - `configured_rigid_body_name() -> str` (lines 164-165): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip, get_default_value.
  - `configured_ownship_pose_topic() -> str` (lines 168-175): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip, _join_topic, configured_mocap_namespace, get_default_value, configured_rigid_body_name.
  - `configured_compare_pose_topic() -> str` (lines 178-185): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip, _join_topic, configured_mocap_namespace, get_default_value.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/drone_control_node.py`

- File type: Python source file.
- Tracked size: 18940 bytes; 469 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes DroneControllerNode; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from collections import deque`
  - third-party: `import numpy as np; from scipy.spatial.transform import Rotation as R`
  - ROS/runtime: `import rclpy; from rclpy.node import Node; from mavros_msgs.msg import State, AttitudeTarget; from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion; from std_msgs.msg import String; from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong, MessageInterval`
  - local/project: `from ros2_poselib.poselib import Pose3D; from drone_control_pkg.deployment_config import configured_mavros_namespace; from drone_control_pkg.topic_utils import join_topic`
- Classes:
  - `DroneControllerNode` (lines 34-453, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `mavros_namespace, drone_state_queue, drone_local_pos_queue, command_topic, mode_topic, arm_topic, takeoff_topic, message_interval_topic, state_topic, setpoint_position_topic, local_position_topic, cmd_cli, mode_cli, arm_cli, takeoff_cli, message_interval_cli, state_sub, target_pub, pos_sub, messages_to_request, flight_plan, flight_plan_actions, current_action_index, action_start_time, state, timer, declare_parameter, create_client, create_subscription, state_callback, create_publisher, local_position_callback, create_timer, main_loop, takeoff_callback, mode_change_callback, arm_callback, change_mode, set_all_message_interval, hover` plus more.
    - `__init__(self)` (lines 35-138): The `DroneControllerNode` class is responsible for controlling the behavior of a drone using ROS2.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, calls ROS services, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, deque, join_topic, self.create_client, rclpy.qos.QoSProfile, self.create_subscription, self.create_publisher, self.create_timer, configured_mavros_namespace, self.get_parameter, self.flight_plan.keys.
    - `local_position_callback(self, msg: PoseStamped) -> None` (lines 140-149): Creates a `Pose3D` object from the local position message received from the drone then appends it to `drone_local_pos_queue`.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.drone_local_pos_queue.append, Pose3D.from_msg.
    - `state_callback(self, msg: State) -> None` (lines 151-158): Appends the received state message to `drone_state_queue`.
      Code map: straight-line helper logic.
      Notable calls: self.drone_state_queue.append.
    - `set_all_message_interval(self) -> None` (lines 160-180): Requests data from the drone flight controller in the form of MavLink messages.
      Code map: 1 for loops.
      Notable calls: MessageInterval.Request, self.message_interval_cli.call_async, future.add_done_callback, self.message_interval_callback.
    - `message_interval_callback(self, future, message_id)` (lines 182-204): Handles the callback for the set_message_interval service call.
      Code map: 1 conditional branches; 1 try/except blocks.
      Notable calls: future.result.
    - `takeoff(self, target_alt: float) -> None` (lines 206-220): Makes drone takeoff until it reaches target altitude.
      Code map: straight-line helper logic.
      Runtime interactions: requests takeoff/land through MAVROS.
      Notable calls: CommandTOL.Request, self.takeoff_cli.call_async, future.add_done_callback.
    - `takeoff_callback(self, future)` (lines 222-237): Handles the callback for the takeoff service call.
      Code map: 1 conditional branches; 1 try/except blocks.
      Notable calls: future.result.
    - `change_mode(self, new_mode: str) -> None` (lines 239-254): Sends an asynchronous request to changes the mode of the drone using the SetMode service.
      Code map: straight-line helper logic.
      Runtime interactions: requests PX4/MAVROS mode changes.
      Notable calls: SetMode.Request, self.mode_cli.call_async, future.add_done_callback.
    - `mode_change_callback(self, future)` (lines 256-271): Handles the result of an asynchronous service call to change the mode of the drone.
      Code map: 1 conditional branches; 1 try/except blocks.
      Notable calls: future.result.
    - `arm(self) -> None` (lines 273-284): Initiates the arming process for the system.
      Code map: straight-line helper logic.
      Runtime interactions: arms/disarms through MAVROS.
      Notable calls: CommandBool.Request, self.arm_cli.call_async, future.add_done_callback.
    - `arm_callback(self, future)` (lines 286-301): Handles the result of an asynchronous service call to arm the drone.
      Code map: 1 conditional branches; 1 try/except blocks.
      Notable calls: future.result.
    - `to_local_pose(self, target_pose: Pose3D) -> None` (lines 303-304): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: self.target_pub.publish, target_pose.to_msg.
    - `is_guided(self)` (lines 306-315): Checks if the drone is currently in guided mode.
      Code map: 1 conditional branches; 2 return points.
    - `main_loop(self)` (lines 317-356): Controls the main loop of the drone state machine.
      Code map: 10 conditional branches.
      Notable calls: self.set_all_message_interval, self.change_mode, self.is_guided, self.arm, self.takeoff, self.has_reached_altitude, self.execute_current_action, self.is_landed.
    - `has_reached_altitude(self, target_altitude: float, tolerance: float = 0.1)` (lines 358-375): Determines if the drone has reached `target_altitude` within set `tolerance`.
      Code map: 1 conditional branches; 2 return points.
    - `is_landed(self)` (lines 377-393): Determines if the drone has landed by checking its altitude.
      Code map: 1 conditional branches; 2 return points.
    - `execute_current_action(self)` (lines 395-413): Executes the current action in the flight plan.
      Code map: 3 conditional branches; 1 return points.
      Notable calls: self.hover, self.land.
    - `hover(self, duration: float)` (lines 415-437): Makes the drone keep its pose `hover` for a set duration.
      Code map: 3 conditional branches.
      Notable calls: self.to_local_pose.
    - `land(self)` (lines 439-453): Initiates the landing process for the drone.
      Code map: straight-line helper logic.
      Notable calls: self.change_mode.
- Top-level functions:
  - `main(args = None)` (lines 456-465): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks.
    Notable calls: rclpy.init, DroneControllerNode, rclpy.spin.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, calls ROS services, publishes messages, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages, uses NumPy arrays/math.

### `ros2/src/drone_control_pkg/drone_control_pkg/drone_setup_node.py`

- File type: Python source file.
- Tracked size: 8471 bytes; 220 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes DroneSetup; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from geographic_msgs.msg import GeoPointStamped`
  - ROS/runtime: `import rclpy; from mavros_msgs.msg import HomePosition, State; from rclpy import qos; from rclpy.node import Node`
  - local/project: `from drone_control_pkg.deployment_config import configured_mavros_namespace; from drone_control_pkg.topic_utils import join_topic`
- Top-level constants/state: `STATE_QOS, PUB_QOS`.
- Classes:
  - `DroneSetup` (lines 26-203, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `mavros_namespace, global_origin_latitude_deg, global_origin_longitude_deg, global_origin_altitude_m, home_position_x_m, home_position_y_m, home_position_z_m, home_approach_z_m, retry_period_s, state_topic, home_position_set_topic, home_position_topic, global_origin_set_topic, global_origin_topic, connected, last_connect_time_s, last_home_position_time_s, last_global_origin_time_s, publish_attempt_count, setup_complete_logged, set_home_pub, set_gp_pub, timer, declare_parameter, create_subscription, state_callback, home_position_callback, global_origin_callback, create_publisher, create_timer, timer_callback, now_s, home_position_ready, global_origin_ready, get_parameter, get_logger, make_g...`.
    - `__init__(self) -> None` (lines 27-119): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks.
      Notable calls: super.__init__, self.declare_parameter, join_topic, self.create_subscription, self.create_publisher, self.create_timer, configured_mavros_namespace, self.get_parameter.
    - `now_s(self) -> float` (lines 121-122): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `state_callback(self, msg: State) -> None` (lines 124-141): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
      Notable calls: self.now_s.
    - `home_position_callback(self, msg: HomePosition) -> None` (lines 143-145): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `global_origin_callback(self, msg: GeoPointStamped) -> None` (lines 147-149): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `home_position_ready(self) -> bool` (lines 151-152): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `global_origin_ready(self) -> bool` (lines 154-155): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `make_home_position_msg(self) -> HomePosition` (lines 157-169): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: HomePosition.
    - `make_global_origin_msg(self) -> GeoPointStamped` (lines 171-178): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: GeoPointStamped.
    - `timer_callback(self) -> None` (lines 180-203): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 2 return points.
      Runtime interactions: publishes messages.
      Notable calls: self.home_position_ready, self.global_origin_ready, self.set_gp_pub.publish, self.set_home_pub.publish, self.make_global_origin_msg, self.make_home_position_msg.
- Top-level functions:
  - `main(args = None) -> None` (lines 206-216): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, DroneSetup, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/external_pose_adapter_node.py`

- File type: Python source file.
- Tracked size: 8952 bytes; 267 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes ExternalPoseAdapterNode; defines functions _normalize_quaternion, _quaternion_from_euler, _quaternion_multiply, _rotate_vector, _transform_pose_components, _stamp_is_zero, _parse_vector, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import ast; from copy import deepcopy; from math import cos, sin, sqrt; from typing import Iterable`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from rclpy.node import Node; from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id; from drone_control_pkg.topic_utils import external_pose_input_topic`
- Classes:
  - `ExternalPoseAdapterNode` (lines 132-254, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, source_pose_topic, output_pose_topic, map_frame, timeout_s, source_best_effort, position_offset_m, frame_rpy_rad, rpy_offset_rad, frame_offset_q, orientation_offset_q, last_source_s, last_timeout_warn_s, pose_pub, timeout_timer, declare_parameter, create_subscription, pose_callback, create_publisher, create_timer, check_source_timeout, now_s, get_parameter, get_logger, get_clock`.
    - `__init__(self) -> None` (lines 133-200): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, str.strip, _parse_vector, _quaternion_from_euler, QoSProfile, self.create_subscription, self.create_publisher, self.create_timer, configured_drone_id, external_pose_input_topic, self.get_parameter.
    - `now_s(self) -> float` (lines 202-203): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 205-238): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: publishes messages, handles pose messages.
      Notable calls: self.now_s, deepcopy, _stamp_is_zero, _normalize_quaternion, _transform_pose_components, self.pose_pub.publish.
    - `check_source_timeout(self) -> None` (lines 240-254): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: self.now_s.
- Top-level functions:
  - `_normalize_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]` (lines 17-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: sqrt.
  - `_quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]` (lines 26-45): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: cos, sin, _normalize_quaternion.
  - `_quaternion_multiply(q0: tuple[float, float, float, float], q1: tuple[float, float, float, float]) -> tuple[float, float, float, float]` (lines 48-59): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _normalize_quaternion.
  - `_rotate_vector(q: tuple[float, float, float, float], vector: tuple[float, float, float]) -> tuple[float, float, float]` (lines 62-83): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _normalize_quaternion.
  - `_transform_pose_components(position_xyz: tuple[float, float, float], orientation_xyzw: tuple[float, float, float, float], *, frame_offset_q: tuple[float, float, float, float], position_offset_m: tuple[float, float, float], orientation_offset_q: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]` (lines 86-106): Apply source-frame and body-frame corrections to a pose.
    Code map: 1 return points.
    Notable calls: _normalize_quaternion, _rotate_vector, _quaternion_multiply.
  - `_stamp_is_zero(msg: PoseStamped) -> bool` (lines 109-110): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: handles pose messages.
  - `_parse_vector(value: object, *, expected_len: int, name: str) -> tuple[float, ...]` (lines 113-129): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 return points; 2 raise statements.
    Notable calls: isinstance, ast.literal_eval, ValueError, value.strip.
  - `main(args = None) -> None` (lines 257-267): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, ExternalPoseAdapterNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/external_pose_bridge_node.py`

- File type: Python source file.
- Tracked size: 5343 bytes; 146 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes ExternalPoseBridgeNode; defines functions _stamp_is_zero, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from copy import deepcopy`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import CompanionProcessStatus; from rclpy.node import Node`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.topic_utils import external_pose_input_topic, join_topic, legacy_vio_input_topic`
- Classes:
  - `ExternalPoseBridgeNode` (lines 25-133, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, mavros_namespace, publish_rate_hz, input_timeout_s, publish_companion_status, input_pose_topic, legacy_input_pose_topic, output_pose_topic, restamp_with_local_clock, output_status_topic, latest_pose, last_pose_s, last_input_topic, subscribed_topics, pose_pub, status_pub, timer, declare_parameter, _create_input_subscriptions, create_publisher, create_timer, publish_loop, _subscribe, create_subscription, now_s, _publish_status, get_parameter, get_logger, pose_callback, get_clock`.
    - `__init__(self) -> None` (lines 26-79): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, uses timer callbacks, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, str.strip, join_topic, PoseStamped, self._create_input_subscriptions, self.create_publisher, self.create_timer, configured_drone_id, configured_mavros_namespace, legacy_vio_input_topic, self.get_parameter, ', '.join.
    - `_create_input_subscriptions(self) -> None` (lines 81-89): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Notable calls: external_pose_input_topic, self._subscribe.
    - `_subscribe(self, topic: str) -> None` (lines 91-98): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: creates subscriptions, handles pose messages.
      Notable calls: self.subscribed_topics.append, self.create_subscription, self.pose_callback.
    - `now_s(self) -> float` (lines 100-101): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `pose_callback(self, msg: PoseStamped, source_topic: str) -> None` (lines 103-106): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `publish_loop(self) -> None` (lines 108-120): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 return points.
      Runtime interactions: publishes messages.
      Notable calls: self.now_s, deepcopy, self.pose_pub.publish, _stamp_is_zero, self._publish_status.
    - `_publish_status(self, active: bool) -> None` (lines 122-133): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: CompanionProcessStatus, self.status_pub.publish.
- Top-level functions:
  - `_stamp_is_zero(msg: PoseStamped) -> bool` (lines 21-22): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: handles pose messages.
  - `main(args = None) -> None` (lines 136-146): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, ExternalPoseBridgeNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/external_pose_debug_node.py`

- File type: Python source file.
- Tracked size: 18337 bytes; 467 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes ExternalPoseDebugNode; defines functions _yaw_from_quaternion, _angle_diff_deg, _round_or_none, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import json; from copy import deepcopy; from math import atan2, pi; from geographic_msgs.msg import GeoPointStamped`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import HomePosition; from rclpy import qos; from rclpy.node import Node; from std_msgs.msg import String`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace, configured_ownship_pose_topic; from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic, join_topic`
- Classes:
  - `ExternalPoseDebugNode` (lines 42-454, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, mavros_namespace, source_pose_topic, adapter_output_pose_topic, vision_pose_topic, local_pose_topic, global_origin_topic, home_position_topic, publish_rate_hz, history_window_s, hold_timeout_s, warn_gap_s, warn_position_error_m, warn_yaw_error_deg, source_best_effort, summary_topic, held_pose_topic, source_samples, adapter_samples, vision_samples, local_samples, latest_home, latest_origin, last_warning_s, summary_pub, held_pose_pub, timer, declare_parameter, create_subscription, _origin_callback, _home_callback, create_publisher, create_timer, publish_debug, now_s, _latest_pose_stats, _gap_stats, _maybe_warn, _pose_delta, _make_pose_callback` plus more.
    - `__init__(self) -> None` (lines 43-174): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, str.strip, cdrone_topic, qos.QoSProfile, self.create_subscription, self.create_publisher, self.create_timer, configured_drone_id, configured_mavros_namespace, configured_ownship_pose_topic, external_pose_input_topic, join_topic, self._make_pose_callback, self.get_parameter.
    - `now_s(self) -> float` (lines 176-177): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `_make_pose_callback(self, store: list[tuple[float, PoseStamped]])` (lines 179-184): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages.
      Notable calls: store.append, self._trim_pose_store, self.now_s, deepcopy.
    - `_trim_pose_store(self, store: list[tuple[float, PoseStamped]]) -> None` (lines 186-191): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 while loops.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s, store.pop.
    - `_origin_callback(self, msg: GeoPointStamped) -> None` (lines 193-194): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s, deepcopy.
    - `_home_callback(self, msg: HomePosition) -> None` (lines 196-197): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s, deepcopy.
    - `_latest_pose_stats(self, store: list[tuple[float, PoseStamped]]) -> dict[str, float] | None` (lines 199-227): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s, _yaw_from_quaternion.
    - `_gap_stats(self, store: list[tuple[float, PoseStamped]]) -> tuple[float | None, float | None] | tuple[None, None]` (lines 229-238): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `_pose_delta(self, source: dict[str, float] | None, target: dict[str, float] | None) -> dict[str, float] | None` (lines 240-250): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: _angle_diff_deg.
    - `_rounded_pose_stats(self, stats: dict[str, float] | None) -> dict[str, float | None] | None` (lines 252-265): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: _round_or_none.
    - `_rounded_delta(self, delta: dict[str, float] | None) -> dict[str, float | None] | None` (lines 267-277): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: _round_or_none.
    - `publish_debug(self) -> None` (lines 279-370): No docstring; behavior is described from its body and call sites.
      Code map: 6 conditional branches.
      Runtime interactions: publishes messages.
      Notable calls: self.now_s, self._latest_pose_stats, self._gap_stats, String, json.dumps, self.summary_pub.publish, self._maybe_warn, _round_or_none, self._rounded_pose_stats, self._rounded_delta, deepcopy, self.held_pose_pub.publish, self._pose_delta.
    - `_maybe_warn(self, now_s: float, source_stats: dict[str, float] | None, max_gap_s: float | None, local_stats: dict[str, float] | None, reference_stats: dict[str, float] | None, reference_label: str) -> None` (lines 372-454): No docstring; behavior is described from its body and call sites.
      Code map: 11 conditional branches; 2 return points.
      Notable calls: self._pose_delta, warn_parts.append, '; '.join.
- Top-level functions:
  - `_yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float` (lines 23-24): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: atan2.
  - `_angle_diff_deg(a_rad: float, b_rad: float) -> float` (lines 27-33): No docstring; behavior is described from its body and call sites.
    Code map: 2 while loops; 1 return points.
  - `_round_or_none(value: float | None, digits: int = 4) -> float | None` (lines 36-39): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `main(args = None) -> None` (lines 457-467): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, ExternalPoseDebugNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/follow_utils.py`

- File type: Python source file.
- Tracked size: 8953 bytes; 310 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes TrackSnapshot, FollowCommand; defines functions clamp, apply_deadband, wrap_angle_rad, clamp_follow_command_altitude, target_bearing_rad, world_error_to_body_frame, score_track, track_is_valid, clamp_follow_command_velocity, distance_in_standoff_window and more.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; from dataclasses import dataclass, replace`
- Top-level constants/state: `TRACK_SOURCE_DETECTED, TRACK_SOURCE_HELD, TRACK_SOURCE_PREDICTED`.
- Classes:
  - `TrackSnapshot` (lines 25-43, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `FollowCommand` (lines 47-56, bases: `object`): No class docstring; role is inferred from methods and base classes.
- Top-level functions:
  - `clamp(value: float, min_v: float, max_v: float) -> float` (lines 12-13): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `apply_deadband(value: float, deadband: float) -> float` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `wrap_angle_rad(angle_rad: float) -> float` (lines 20-21): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.atan2, math.sin, math.cos.
  - `clamp_follow_command_altitude(command: FollowCommand, *, current_altitude_m: float, projected_horizon_s: float, min_z_m: float | None, max_z_m: float | None) -> FollowCommand` (lines 59-87): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 3 return points.
    Notable calls: replace.
  - `target_bearing_rad(track: TrackSnapshot) -> float` (lines 90-91): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.atan2.
  - `world_error_to_body_frame(*, dx_world_m: float, dy_world_m: float, yaw_rad: float) -> tuple[float, float]` (lines 94-105): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.cos, math.sin.
  - `score_track(track: TrackSnapshot) -> float` (lines 108-126): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.sqrt.
  - `track_is_valid(track: TrackSnapshot, *, min_track_confidence: float, max_target_distance_m: float, require_target_in_front: bool, max_abs_target_y_m: float, max_abs_target_z_m: float, allow_predicted_tracks: bool = False, max_predicted_track_age_s: float = 0.0, max_predicted_position_uncertainty_m: float = 0.0) -> bool` (lines 129-161): No docstring; behavior is described from its body and call sites.
    Code map: 9 conditional branches; 9 return points.
  - `clamp_follow_command_velocity(command: FollowCommand, *, max_vel_xy_mps: float, max_vel_z_mps: float, max_yaw_rate_rps: float) -> FollowCommand` (lines 164-196): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: math.hypot, clamp, replace.
  - `distance_in_standoff_window(distance_m: float, desired_distance_m: float, tolerance_m: float) -> bool` (lines 199-206): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `compute_follow_command(track: TrackSnapshot, *, follow_distance_m: float, follow_distance_tolerance_m: float, lateral_deadband_m: float, vertical_deadband_m: float, yaw_deadband_rad: float, kp_xy: float, kp_z: float, kp_yaw: float, max_vel_xy_mps: float, max_vel_z_mps: float, max_yaw_rate_rps: float, min_safe_distance_m: float) -> FollowCommand` (lines 209-257): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: apply_deadband, clamp, FollowCommand, target_bearing_rad.
  - `compute_return_to_point_command(*, current_xy: tuple[float, float], current_altitude_m: float, current_yaw_rad: float, target_xy: tuple[float, float], target_altitude_m: float, target_yaw_rad: float, xy_deadband_m: float, z_deadband_m: float, yaw_deadband_rad: float, kp_xy: float, kp_z: float, kp_yaw: float, max_vel_xy_mps: float, max_vel_z_mps: float, max_yaw_rate_rps: float) -> FollowCommand` (lines 260-310): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: world_error_to_body_frame, apply_deadband, FollowCommand, wrap_angle_rad, clamp.

### `ros2/src/drone_control_pkg/drone_control_pkg/keyboard_teleop_node.py`

- File type: Python source file.
- Tracked size: 20409 bytes; 583 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes KeyboardTeleopNode; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Keyboard teleop for cdrone_control via MAVROS.
- Imports by role:
  - standard library: `import select; import sys; import termios; import time; import tty; from typing import Dict, Tuple`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped, TwistStamped; from mavros_msgs.msg import ManualControl; from mavros_msgs.msg import State; from mavros_msgs.srv import CommandBool, CommandLong, SetMode; from rclpy.node import Node; from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy; from std_msgs.msg import Bool`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.topic_utils import cdrone_topic, join_topic`
- Top-level constants/state: `MOVE_BINDINGS, SPEED_BINDINGS, MANUAL_INTERFACE, VELOCITY_INTERFACE, POSITION_HOLD_MODE`.
- Classes:
  - `KeyboardTeleopNode` (lines 56-565, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `command_interface, publish_rate_hz, linear_speed, angular_speed, manual_xy, manual_yaw, throttle_center, throttle_step, arm_throttle_hold_sec, disarm_request_delay_sec, local_pose_timeout_s, drone_id, mavros_namespace, cmd_vel_topic, estop_topic, manual_control_topic, arm_service, mode_service, command_service, state_topic, local_pose_topic, velocity_pub, manual_pub, estop_pub, arm_client, mode_client, cmd_client, state_sub, local_pose_sub, x, y, z, yaw, force_zero_throttle_until_s, pending_disarm_timer, latest_state, last_local_pose_s, timer, settings, declare_parameter` plus more.
    - `__init__(self)` (lines 57-173): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, calls ROS services, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, handles pose messages, handles velocity setpoints.
      Notable calls: super.__init__, self.declare_parameter, str.strip.lower, join_topic, self.create_publisher, self.create_client, self.create_subscription, QoSProfile, State, self.create_timer, termios.tcgetattr, self.print_usage, configured_drone_id, configured_mavros_namespace, str.strip, cdrone_topic, self.get_parameter.
    - `print_usage(self)` (lines 175-255): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
    - `get_key(self)` (lines 257-265): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: tty.setraw, select.select, termios.tcsetattr, sys.stdin.fileno, sys.stdin.read.
    - `now_s(self) -> float` (lines 267-268): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: time.monotonic.
    - `state_callback(self, msg: State) -> None` (lines 270-271): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `local_pose_callback(self, _msg: PoseStamped) -> None` (lines 273-274): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `publish_current_command(self)` (lines 276-280): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.publish_manual_control, self.publish_velocity.
    - `publish_velocity(self)` (lines 282-290): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages, handles velocity setpoints.
      Notable calls: TwistStamped, self.velocity_pub.publish.
    - `publish_manual_control(self)` (lines 292-300): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: ManualControl, self.current_manual_throttle, self.manual_pub.publish.
    - `current_manual_throttle(self) -> float` (lines 302-308): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: time.monotonic.
    - `hold_zero_throttle_for_arming(self, action: str)` (lines 310-326): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.publish_current_command, time.monotonic.
    - `stop_drone(self)` (lines 328-337): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.publish_current_command.
    - `apply_axis_command(self, key: str)` (lines 339-343): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: setattr, self.publish_current_command, self.log_current_command.
    - `adjust_speed(self, scale: float)` (lines 345-360): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
    - `log_current_command(self)` (lines 362-376): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.current_manual_throttle.
    - `arm_drone(self)` (lines 378-386): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: arms/disarms through MAVROS.
      Notable calls: self.hold_zero_throttle_for_arming, CommandBool.Request, self.arm_client.call_async, future.add_done_callback, self.arm_client.wait_for_service.
    - `force_arm_drone(self)` (lines 388-398): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.hold_zero_throttle_for_arming, CommandLong.Request, self.cmd_client.call_async, future.add_done_callback, self.cmd_client.wait_for_service.
    - `_force_arm_callback(self, future)` (lines 400-408): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 try/except blocks.
      Notable calls: future.result.
    - `disable_rc_override(self)` (lines 410-423): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: sets PX4/MAVROS parameters.
      Notable calls: CommandLong.Request, self.cmd_client.call_async, self.cmd_client.wait_for_service, join_topic.
    - `disarm_drone(self)` (lines 425-434): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Notable calls: self.hold_zero_throttle_for_arming, self.send_disarm_request, self.arm_client.wait_for_service, self.schedule_delayed_disarm.
    - `schedule_delayed_disarm(self)` (lines 436-449): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: uses timer callbacks.
      Notable calls: self.create_timer, self.pending_disarm_timer.cancel, self.destroy_timer.
    - `_delayed_disarm_callback(self)` (lines 451-457): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.send_disarm_request, self.pending_disarm_timer.cancel, self.destroy_timer.
    - `send_disarm_request(self)` (lines 459-463): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: arms/disarms through MAVROS.
      Notable calls: CommandBool.Request, self.arm_client.call_async, future.add_done_callback.
    - `_arm_callback(self, future)` (lines 465-473): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 try/except blocks.
      Notable calls: future.result.
    - `set_mode(self, mode_name: str)` (lines 475-482): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: requests PX4/MAVROS mode changes.
      Notable calls: SetMode.Request, self.mode_client.call_async, future.add_done_callback, self.mode_client.wait_for_service, self._mode_callback.
    - `_mode_callback(self, future, mode_name)` (lines 484-492): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 try/except blocks.
      Notable calls: future.result.
    - `warn_if_posctl_pose_unready(self)` (lines 494-512): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 return points.
      Notable calls: self.now_s.
    - `run(self)` (lines 514-565): No docstring; behavior is described from its body and call sites.
      Code map: 13 conditional branches; 1 while loops; 1 try/except blocks.
      Notable calls: rclpy.ok, self.stop_drone, termios.tcsetattr, self.get_key, rclpy.spin_once, self.apply_axis_command, self.adjust_speed, self.force_arm_drone, self.arm_drone, self.set_mode, self.disarm_drone, self.disable_rc_override, self.warn_if_posctl_pose_unready, repr.
- Top-level functions:
  - `main(args = None)` (lines 568-579): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, KeyboardTeleopNode, node.run, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, calls ROS services, publishes messages, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, sets PX4/MAVROS parameters, handles pose messages, handles velocity setpoints.

### `ros2/src/drone_control_pkg/drone_control_pkg/mavros_velocity_node.py`

- File type: Python source file.
- Tracked size: 8166 bytes; 223 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes MavrosVelocityNode; defines functions clamp, offboard_velocity_gate_open, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from copy import deepcopy`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped, TwistStamped; from mavros_msgs.msg import State; from rclpy.node import Node; from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy; from std_msgs.msg import Bool`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.topic_utils import cdrone_topic, join_topic`
- Classes:
  - `MavrosVelocityNode` (lines 31-206, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `publish_rate_hz, watchdog_timeout_s, max_vel_xy_mps, max_vel_z_mps, max_yaw_rate_rps, require_guided_mode, drone_id, mavros_namespace, cmd_vel_topic, estop_topic, state_topic, local_pose_topic, output_topic, latest_cmd, latest_cmd_time_s, estop, state, local_pose, cmd_sub, estop_sub, state_sub, local_pose_sub, velocity_pub, timer, declare_parameter, create_subscription, cmd_callback, estop_callback, state_callback, local_pose_callback, create_publisher, create_timer, publish_loop, now_s, _guided_gate_open, _last_gate_warn, get_parameter, get_logger, _zero_cmd, latest_cmd_is_effectively_zero` plus more.
    - `__init__(self) -> None` (lines 32-113): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages, handles velocity setpoints.
      Notable calls: super.__init__, self.declare_parameter, TwistStamped, State, PoseStamped, self.create_subscription, QoSProfile, self.create_publisher, self.create_timer, configured_drone_id, configured_mavros_namespace, str.strip, cdrone_topic, join_topic, self.get_parameter.
    - `now_s(self) -> float` (lines 115-116): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `latest_cmd_is_effectively_zero(self) -> bool` (lines 118-125): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `cmd_callback(self, msg: TwistStamped) -> None` (lines 127-142): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles velocity setpoints.
      Notable calls: deepcopy, clamp, self.now_s.
    - `estop_callback(self, msg: Bool) -> None` (lines 144-145): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `state_callback(self, msg: State) -> None` (lines 147-155): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
    - `local_pose_callback(self, msg: PoseStamped) -> None` (lines 157-158): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
    - `_guided_gate_open(self) -> bool` (lines 160-165): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: offboard_velocity_gate_open.
    - `_zero_cmd(self) -> TwistStamped` (lines 167-170): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles velocity setpoints.
      Notable calls: TwistStamped.
    - `publish_loop(self) -> None` (lines 172-206): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 3 return points.
      Runtime interactions: publishes messages.
      Notable calls: self.now_s, deepcopy, self.velocity_pub.publish, self._guided_gate_open, self._zero_cmd, hasattr, self.latest_cmd_is_effectively_zero, getattr.
- Top-level functions:
  - `clamp(value: float, min_v: float, max_v: float) -> float` (lines 19-20): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `offboard_velocity_gate_open(*, require_guided_mode: bool, armed: bool, mode: str) -> bool` (lines 23-28): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `main(args = None) -> None` (lines 209-219): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, MavrosVelocityNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages, handles velocity setpoints.

### `ros2/src/drone_control_pkg/drone_control_pkg/milestone2_demo_logic.py`

- File type: Python source file.
- Tracked size: 3233 bytes; 103 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines functions select_sequential_target, update_dwell_progress, project_body_velocity_to_world_xy.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; from typing import Iterable, Optional`
  - local/project: `from drone_control_pkg.follow_utils import TrackSnapshot, score_track, track_is_valid`
- Top-level functions:
  - `select_sequential_target(tracks: Iterable[TrackSnapshot], *, active_track_id: Optional[int], excluded_track_ids: set[int], now_s: float, track_timeout_s: float, min_track_confidence: float, max_target_distance_m: float, require_target_in_front: bool, max_abs_target_y_m: float, max_abs_target_z_m: float, allow_predicted_tracks: bool = False, max_predicted_track_age_s: float = 0.0, max_predicted_position_uncertainty_m: float = 0.0) -> Optional[TrackSnapshot]` (lines 9-64): No docstring; behavior is described from its body and call sites.
    Code map: 6 conditional branches; 1 for loops; 6 return points.
    Notable calls: candidates.sort, track_is_valid, is_candidate, score_track.
  - `update_dwell_progress(dwell_started_s: Optional[float], *, in_standoff_window: bool, now_s: float, dwell_time_s: float) -> tuple[Optional[float], float, float, bool]` (lines 67-84): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
  - `project_body_velocity_to_world_xy(*, current_xy: tuple[float, float], yaw_rad: float, vx_mps: float, vy_mps: float, horizon_s: float) -> tuple[float, float]` (lines 87-103): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.cos, math.sin.

### `ros2/src/drone_control_pkg/drone_control_pkg/milestone2_demo_sequence_node.py`

- File type: Python source file.
- Tracked size: 32146 bytes; 796 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes Milestone2DemoSequenceNode; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; import os; from typing import Dict, Optional; from std_srvs.srv import Trigger`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped, TwistStamped; from mavros_msgs.msg import CompanionProcessStatus, State; from rclpy.node import Node; from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy; from std_msgs.msg import Bool`
  - local/project: `from drone_msgs.msg import EngagementState, TargetTrackArray; from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.follow_utils import FollowCommand, TrackSnapshot, compute_follow_command, distance_in_standoff_window, target_bearing_rad; from drone_control_pkg.milestone2_demo_logic import project_body_velocity_to_world_xy, select_sequential_target, update_dwell_progress; from drone_control_pkg.perimeter_utils import Perimeter...`
- Classes:
  - `Milestone2DemoSequenceNode` (lines 36-779, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `scenario_id, publish_rate_hz, drone_id, mavros_namespace, required_completion_count, dwell_time_s, follow_distance_m, follow_distance_tolerance_m, lateral_deadband_m, vertical_deadband_m, yaw_deadband_rad, track_timeout_s, track_gc_s, state_timeout_s, local_pose_timeout_s, companion_status_timeout_s, min_track_confidence, max_target_distance_m, require_target_in_front, max_abs_target_y_m, max_abs_target_z_m, min_safe_distance_m, require_mavros_connected, require_armed, require_offboard, require_companion_active, publish_zero_on_block, block_target_on_perimeter_violation, projected_path_horizon_s, kp_xy, kp_z, kp_yaw, max_vel_xy_mps, max_vel_z_mps, max_yaw_rate_rps, enable_perimeter_guard,...` plus more.
    - `__init__(self) -> None` (lines 39-298): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, handles pose messages, handles velocity setpoints, handles target track messages.
      Notable calls: super.__init__, self.declare_parameter, State, PoseStamped, self.load_perimeter_guard, QoSProfile, self.create_subscription, self.create_publisher, self.create_service, self.create_timer, self.publish_engagement_state, configured_drone_id, configured_mavros_namespace, str.strip, cdrone_topic, join_topic, self.get_parameter, self.now_s.
    - `now_s(self) -> float` (lines 300-301): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `load_perimeter_guard(self) -> None` (lines 303-329): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 3 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: PerimeterGuard.load_from_yaml, os.path.basename.
    - `state_callback(self, msg: State) -> None` (lines 331-333): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `local_pose_callback(self, msg: PoseStamped) -> None` (lines 335-337): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `companion_status_callback(self, msg: CompanionProcessStatus) -> None` (lines 339-348): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `estop_callback(self, msg: Bool) -> None` (lines 350-351): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `tracks_callback(self, msg: TargetTrackArray) -> None` (lines 353-382): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 for loops.
      Runtime interactions: handles target track messages.
      Notable calls: self.now_s, TrackSnapshot, seen_ids.add, self.tracks.keys, self.tracks.pop.
    - `current_frame_id(self) -> str` (lines 384-385): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.strip.
    - `current_xy(self) -> tuple[float, float]` (lines 387-391): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_altitude_m(self) -> float` (lines 393-394): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_yaw_rad(self) -> float` (lines 396-404): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: math.atan2.
    - `state_fresh(self) -> bool` (lines 406-407): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `pose_fresh(self) -> bool` (lines 409-410): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `companion_status_fresh(self) -> bool` (lines 412-415): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `zero_cmd(self) -> TwistStamped` (lines 417-420): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles velocity setpoints.
      Notable calls: TwistStamped.
    - `publish_zero_command(self) -> None` (lines 422-423): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: self.cmd_pub.publish, self.zero_cmd.
    - `publish_follow_command(self, command: FollowCommand) -> None` (lines 425-432): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages, handles velocity setpoints.
      Notable calls: TwistStamped, self.cmd_pub.publish.
    - `clear_active_target(self) -> None` (lines 434-438): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `set_demo_state(self, new_state: str, reason: str = '') -> None` (lines 440-448): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 return points.
    - `control_block_reason(self) -> str` (lines 450-468): No docstring; behavior is described from its body and call sites.
      Code map: 9 conditional branches; 9 return points.
      Notable calls: self.state_fresh, str.upper, self.pose_fresh, self.companion_status_fresh.
    - `perimeter_runtime_violation_reason(self) -> Optional[str]` (lines 470-496): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Notable calls: self.current_frame_id, self.current_xy, self.perimeter_guard.xy_violation_reason, self.perimeter_guard.runtime_ceiling_violation_reason, self.current_altitude_m, self.perimeter_guard.frame_matches.
    - `command_projection_violation_reason(self, command: FollowCommand) -> Optional[str]` (lines 498-527): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: self.current_xy, project_body_velocity_to_world_xy, self.perimeter_guard.segment_violation_reason, self.perimeter_guard.goal_altitude_violation_reason, self.current_altitude_m, self.current_yaw_rad.
    - `select_target(self, now_s: float) -> Optional[TrackSnapshot]` (lines 529-561): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 return points.
      Notable calls: select_sequential_target, self.tracks.values.
    - `startable(self) -> tuple[bool, str]` (lines 563-574): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Notable calls: self.control_block_reason, self.perimeter_runtime_violation_reason.
    - `handle_start_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 576-600): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.startable, self.completed_track_ids.clear, self.blocked_track_ids.clear, self.set_demo_state.
    - `handle_abort_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 602-616): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.enter_abort.
    - `enter_abort(self, reason: str) -> None` (lines 618-623): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.clear_active_target, self.publish_zero_command, self.set_demo_state.
    - `publish_engagement_state(self, *, now_s: float, target: Optional[TrackSnapshot]) -> None` (lines 625-653): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: EngagementState, self.state_pub.publish, target_bearing_rad.
    - `timer_callback(self) -> None` (lines 655-779): No docstring; behavior is described from its body and call sites.
      Code map: 11 conditional branches; 7 return points.
      Notable calls: self.now_s, self.perimeter_runtime_violation_reason, self.control_block_reason, self.select_target, compute_follow_command, self.command_projection_violation_reason, self.publish_follow_command, distance_in_standoff_window, update_dwell_progress, self.publish_engagement_state, self.publish_zero_command, self.enter_abort, self.set_demo_state, self.clear_active_target, self.completed_track_ids.add, self.blocked_track_ids.add.
- Top-level functions:
  - `main(args = None) -> None` (lines 782-792): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, Milestone2DemoSequenceNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, publishes messages, handles pose messages, handles velocity setpoints, handles target track messages, loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/milestone3_demo_sequence_node.py`

- File type: Python source file.
- Tracked size: 113230 bytes; 2652 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes Milestone3DemoSequenceNode; defines functions parameter_value_to_float, parameter_value_is_declared_numeric, normalize_takeoff_strategy, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; import os; from typing import Dict, Optional, Tuple; from geographic_msgs.msg import GeoPointStamped; from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue; from rcl_interfaces.srv import GetParameters, SetParameters; from std_srvs.srv import Trigger`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped, TwistStamped; from mavros_msgs.msg import CompanionProcessStatus, HomePosition, State; from mavros_msgs.srv import CommandBool, CommandTOLLocal, ParamPull, SetMode; from rclpy.node import Node; from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy; from std_msgs.msg import Bool`
  - local/project: `from drone_msgs.msg import EngagementState, TargetTrackArray; from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.follow_utils import FollowCommand, TRACK_SOURCE_PREDICTED, TrackSnapshot, clamp_follow_command_altitude, clamp_follow_command_velocity, compute_follow_command, compute_return_to_point_command, distance_in_standoff_window, target_bearing_rad, wrap_angle_rad; from drone_control_pkg.milestone2_demo_logic import proj...`
- Classes:
  - `Milestone3DemoSequenceNode` (lines 88-2635, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `scenario_id, publish_rate_hz, drone_id, mavros_namespace, takeoff_altitude_m, takeoff_rate_m_s, takeoff_strategy, altitude_tolerance_m, touchdown_altitude_m, touchdown_dwell_s, stage_timeout_s, arm_zero_throttle_hold_s, offboard_warmup_s, stage_hover_duration_s, climb_handoff_timeout_s, local_pose_timeout_s, local_pose_timeout_during_param_sync_s, state_timeout_s, state_timeout_during_param_sync_s, connection_loss_timeout_s, connection_loss_timeout_during_param_sync_s, mode_request_retry_interval_s, companion_status_timeout_s, require_mavros_connected, require_companion_active, restore_takeoff_alt_on_exit, takeoff_param_id, param_pull_force, param_pull_retry_delay_s, param_sync_timeout_s,...` plus more.
    - `__init__(self) -> None` (lines 101-597): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages, handles velocity setpoints, handles target track messages.
      Notable calls: super.__init__, self.declare_parameter, normalize_takeoff_strategy, str.strip, self.now_s, State, PoseStamped, self.create_publisher, QoSProfile, self.create_subscription, self.create_client, self.create_service, self.create_timer, self.load_perimeter_guard, self.load_pre_takeoff_profile, self.load_speed_profile, self.publish_engagement_state, configured_drone_id....
    - `now_s(self) -> float` (lines 599-600): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `state_callback(self, msg: State) -> None` (lines 602-613): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
      Notable calls: self.now_s.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 615-617): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `home_position_callback(self, msg: HomePosition) -> None` (lines 619-621): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `global_origin_callback(self, msg: GeoPointStamped) -> None` (lines 623-625): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `companion_status_callback(self, msg: CompanionProcessStatus) -> None` (lines 627-636): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `estop_callback(self, msg: Bool) -> None` (lines 638-639): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `tracks_callback(self, msg: TargetTrackArray) -> None` (lines 641-686): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 for loops.
      Runtime interactions: handles target track messages.
      Notable calls: self.now_s, TrackSnapshot, seen_ids.add, self.tracks.keys, self.tracks.pop, getattr.
    - `load_perimeter_guard(self) -> None` (lines 688-714): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 3 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: PerimeterGuard.load_from_yaml, os.path.basename.
    - `load_pre_takeoff_profile(self) -> None` (lines 716-740): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 try/except blocks; 2 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: Px4ParamProfile.load_from_yaml, self.pre_takeoff_profile.parameters.keys.
    - `load_speed_profile(self) -> None` (lines 742-766): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 3 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: Px4ParamProfile.load_from_yaml, self.speed_profile.parameters.keys.
    - `current_altitude_m(self) -> float` (lines 768-769): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_xy(self) -> Tuple[float, float]` (lines 771-775): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_yaw_rad(self) -> float` (lines 777-785): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: math.atan2.
    - `has_takeoff_return_target(self) -> bool` (lines 787-792): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `return_target_reached(self) -> bool` (lines 794-818): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points; 2 assertions.
      Notable calls: self.current_xy, math.hypot, self.has_takeoff_return_target, wrap_angle_rad, self.current_altitude_m, self.takeoff_target_altitude_m, self.current_yaw_rad.
    - `current_frame_id(self) -> str` (lines 820-821): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.strip.
    - `pose_fresh(self) -> bool` (lines 823-833): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `active_state_timeout_s(self) -> float` (lines 835-848): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `state_fresh(self) -> bool` (lines 850-851): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.active_state_timeout_s, self.now_s.
    - `companion_status_fresh(self) -> bool` (lines 853-856): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `home_position_ready(self) -> bool` (lines 858-859): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `global_origin_ready(self) -> bool` (lines 861-862): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `active_connection_loss_timeout_s(self) -> float` (lines 864-880): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `mode_matches(self, mode_name: str) -> bool` (lines 882-883): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `mode_request_retry_ready(self) -> bool` (lines 885-890): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.now_s.
    - `is_land_mode(self) -> bool` (lines 892-893): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `is_takeoff_mode(self) -> bool` (lines 895-896): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `is_takeoff_handoff_mode(self) -> bool` (lines 898-900): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `takeoff_target_altitude_m(self) -> float` (lines 902-905): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `takeoff_altitude_reached(self) -> bool` (lines 907-910): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.current_altitude_m, self.takeoff_target_altitude_m.
    - `touchdown_threshold_altitude_m(self) -> float` (lines 912-915): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `zero_cmd(self) -> TwistStamped` (lines 917-920): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles velocity setpoints.
      Notable calls: TwistStamped.
    - `publish_zero_command(self) -> None` (lines 922-923): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: self.cmd_pub.publish, self.zero_cmd.
    - `publish_follow_command(self, command: FollowCommand) -> None` (lines 925-932): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages, handles velocity setpoints.
      Notable calls: TwistStamped, self.cmd_pub.publish.
    - `publish_takeoff_hold_command(self) -> None` (lines 934-966): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Notable calls: self.current_altitude_m, self.current_yaw_rad, compute_return_to_point_command, self.clamp_command_to_perimeter_altitude, self.command_projection_violation_reason, self.publish_follow_command, self.current_xy, self.publish_zero_command, self.takeoff_target_altitude_m.
    - `clear_active_target(self) -> None` (lines 968-973): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `set_demo_state(self, new_state: str, reason: str = '') -> None` (lines 975-995): No docstring; behavior is described from its body and call sites.
      Code map: 7 conditional branches.
      Notable calls: self.now_s, self.current_xy, self.current_yaw_rad.
    - `stage_elapsed_s(self) -> float` (lines 997-998): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `stage_timeout_limit_s(self) -> Optional[float]` (lines 1000-1015): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
    - `should_restore_speed_profile(self) -> bool` (lines 1017-1022): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `speed_profile_apply_complete(self) -> bool` (lines 1024-1031): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: all.
    - `next_speed_profile_param_to_apply(self) -> Optional[str]` (lines 1033-1040): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
    - `next_speed_profile_param_to_restore(self) -> Optional[str]` (lines 1042-1046): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
    - `pre_takeoff_profile_apply_complete(self) -> bool` (lines 1048-1055): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: all.
    - `next_pre_takeoff_param_to_apply(self) -> Optional[str]` (lines 1057-1064): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
    - `skip_pre_takeoff_profile_param(self, param_name: str, reason: str) -> None` (lines 1066-1073): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.pre_takeoff_profile_skipped_names.add, self.pre_takeoff_profile_applied_names.discard.
    - `skip_speed_profile_param(self, param_name: str, reason: str) -> None` (lines 1075-1084): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.speed_profile_skipped_names.add, self.speed_profile_original_values.pop, self.speed_profile_changed_names.discard, self.speed_profile_applied_names.discard.
    - `is_missing_param_reason(reason: str) -> bool` (lines 1087-1089): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.lower.
    - `skip_takeoff_param_restore(self, reason: str) -> None` (lines 1091-1097): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `perimeter_start_violation_reason(self) -> Optional[str]` (lines 1099-1122): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Notable calls: self.current_frame_id, self.current_xy, self.perimeter_guard.xy_violation_reason, self.perimeter_guard.goal_altitude_violation_reason, self.takeoff_target_altitude_m, self.perimeter_guard.frame_matches.
    - `perimeter_runtime_violation_reason(self) -> Optional[str]` (lines 1124-1149): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Notable calls: self.current_frame_id, self.current_xy, self.perimeter_guard.xy_violation_reason, self.perimeter_guard.runtime_ceiling_violation_reason, self.current_altitude_m, self.perimeter_guard.frame_matches.
    - `command_projection_violation_reason(self, command: FollowCommand) -> Optional[str]` (lines 1151-1178): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: self.current_xy, project_body_velocity_to_world_xy, self.perimeter_guard.segment_violation_reason, self.perimeter_guard.goal_altitude_violation_reason, self.current_altitude_m, self.current_yaw_rad.
    - `clamp_command_to_perimeter_altitude(self, command: FollowCommand) -> FollowCommand` (lines 1180-1207): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: clamp_follow_command_altitude, self.now_s, self.current_altitude_m.
    - `control_block_reason(self) -> str` (lines 1209-1227): No docstring; behavior is described from its body and call sites.
      Code map: 9 conditional branches; 9 return points.
      Notable calls: self.state_fresh, str.upper, self.pose_fresh, self.companion_status_fresh.
    - `select_target(self, now_s: float) -> Optional[TrackSnapshot]` (lines 1229-1266): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 return points.
      Notable calls: select_sequential_target, self.tracks.values, self.clear_active_target.
    - `startable(self) -> Tuple[bool, str]` (lines 1268-1321): No docstring; behavior is described from its body and call sites.
      Code map: 20 conditional branches; 20 return points.
      Notable calls: self.perimeter_start_violation_reason, self.mode_client.wait_for_service, self.arm_client.wait_for_service, self.param_get_client.wait_for_service, self.param_pull_client.wait_for_service, self.param_set_client.wait_for_service, self.state_fresh, self.pose_fresh, self.global_origin_ready, self.home_position_ready, self.takeoff_client.wait_for_service, self.companion_status_fresh.
    - `handle_start_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 1323-1376): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.startable, self.current_xy, self.current_altitude_m, self.current_yaw_rad, self.completed_track_ids.clear, self.blocked_track_ids.clear, self.clear_active_target, self.set_demo_state.
    - `handle_abort_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 1378-1391): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.enter_abort.
    - `enter_abort(self, reason: str) -> None` (lines 1393-1400): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.clear_active_target, self.publish_zero_command, self.set_demo_state.
    - `request_mode(self, mode_name: str) -> None` (lines 1402-1413): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: requests PX4/MAVROS mode changes.
      Notable calls: SetMode.Request, self.mode_client.call_async, self.now_s, self.mode_client.wait_for_service, self.enter_abort.
    - `request_arm(self, arm_value: bool) -> None` (lines 1415-1426): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: arms/disarms through MAVROS.
      Notable calls: CommandBool.Request, self.arm_client.call_async, self.arm_client.wait_for_service, self.enter_abort.
    - `request_takeoff(self) -> None` (lines 1428-1456): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: requests takeoff/land through MAVROS.
      Notable calls: self.current_xy, self.takeoff_target_altitude_m, CommandTOLLocal.Request, self.current_yaw_rad, self.takeoff_client.call_async, self.takeoff_client.wait_for_service, self.enter_abort.
    - `request_param_get(self, *, param_name: Optional[str] = None, kind: str = 'GET_ORIGINAL') -> None` (lines 1458-1478): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: str.strip, GetParameters.Request, self.param_get_client.call_async, self.enter_abort, self.param_get_client.wait_for_service.
    - `request_param_pull(self) -> None` (lines 1480-1493): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Notable calls: ParamPull.Request, self.param_pull_client.call_async, self.param_pull_client.wait_for_service, self.enter_abort.
    - `schedule_param_pull_retry(self, reason: str) -> None` (lines 1495-1503): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.now_s.
    - `request_param_set(self, value: float, *, kind: str, param_name: Optional[str] = None) -> None` (lines 1505-1537): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: str.strip, SetParameters.Request, self.param_set_client.call_async, self.enter_abort, self.param_set_client.wait_for_service, Parameter, ParameterValue.
    - `poll_service_futures(self) -> None` (lines 1539-1826): No docstring; behavior is described from its body and call sites.
      Code map: 38 conditional branches; 5 try/except blocks; 14 return points.
      Notable calls: self.mode_future.done, self.arm_future.done, self.takeoff_future.done, self.param_pull_future.done, self.param_future.done, self.mode_future.result, self.arm_future.result, self.takeoff_future.result, self.param_pull_future.result, self.param_future.result, self.enter_abort, self.is_land_mode, self.schedule_param_pull_retry, parameter_value_to_float, math.isclose, self.current_altitude_m, self.touchdown_threshold_altitude_m, parameter_value_is_declared_numeric....
    - `run_safety_checks(self) -> None` (lines 1828-1936): No docstring; behavior is described from its body and call sites.
      Code map: 20 conditional branches; 12 return points.
      Notable calls: self.perimeter_runtime_violation_reason, self.stage_timeout_limit_s, self.state_fresh, self.enter_abort, self.now_s, self.active_connection_loss_timeout_s, requires_runtime_tracking_guards, self.pose_fresh, self.companion_status_fresh, self.is_takeoff_mode, self.is_takeoff_handoff_mode, self.mode_matches, self.stage_elapsed_s.
    - `publish_engagement_state(self, *, now_s: float, target: Optional[TrackSnapshot]) -> None` (lines 1938-1983): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches.
      Runtime interactions: publishes messages.
      Notable calls: EngagementState, self.state_pub.publish, target_bearing_rad, self.control_block_reason.
    - `step_state_machine(self) -> Optional[TrackSnapshot]` (lines 1985-2627): No docstring; behavior is described from its body and call sites.
      Code map: 121 conditional branches; 68 return points; 2 assertions.
      Notable calls: self.now_s, self.publish_zero_command, self.takeoff_altitude_reached, self.pre_takeoff_profile_apply_complete, self.next_pre_takeoff_param_to_apply, self.request_param_set, self.is_takeoff_mode, self.current_altitude_m, self.takeoff_target_altitude_m, self.is_takeoff_handoff_mode, self.mode_matches, self.speed_profile_apply_complete, self.next_speed_profile_param_to_apply, math.isclose, self.control_block_reason, self.select_target, compute_follow_command, self.clamp_command_to_perimeter_altitude....
    - `timer_callback(self) -> None` (lines 2629-2635): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.poll_service_futures, self.run_safety_checks, self.step_state_machine, self.publish_engagement_state, self.tracks.get, self.now_s.
- Top-level functions:
  - `parameter_value_to_float(value: ParameterValue) -> float` (lines 51-60): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 5 return points.
  - `parameter_value_is_declared_numeric(value: ParameterValue) -> bool` (lines 63-67): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `normalize_takeoff_strategy(value: str) -> str` (lines 70-85): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Notable calls: str.strip.upper, ValueError, str.strip.
  - `main(args = None) -> None` (lines 2638-2648): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, Milestone3DemoSequenceNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, publishes messages, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages, handles velocity setpoints, handles target track messages, loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/milestone3_state_logic.py`

- File type: Python source file.
- Tracked size: 766 bytes; 30 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines functions completion_next_state, post_return_next_state, requires_runtime_tracking_guards.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations`
- Top-level constants/state: `POST_LANDING_CLEANUP_STATES`.
- Top-level functions:
  - `completion_next_state(*, return_to_takeoff_on_complete: bool, has_takeoff_return_target: bool, land_on_complete: bool) -> str` (lines 10-20): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
  - `post_return_next_state(*, land_on_complete: bool) -> str` (lines 23-24): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `requires_runtime_tracking_guards(demo_state: str, *, armed: bool) -> bool` (lines 27-30): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.

### `ros2/src/drone_control_pkg/drone_control_pkg/minjerk_waypoint_mission_node.py`

- File type: Python source file.
- Tracked size: 615 bytes; 30 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes MinJerkWaypointMissionNode; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations`
  - ROS/runtime: `import rclpy`
  - local/project: `from drone_control_pkg.position_goto_demo_sequence_node import PositionGotoDemoSequenceNode`
- Classes:
  - `MinJerkWaypointMissionNode` (lines 10-12, bases: `PositionGotoDemoSequenceNode`): No class docstring; role is inferred from methods and base classes.
    - `__init__(self) -> None` (lines 11-12): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: super.__init__.
- Top-level functions:
  - `main(args = None) -> None` (lines 15-26): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, MinJerkWaypointMissionNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.

### `ros2/src/drone_control_pkg/drone_control_pkg/minjerk_waypoint_utils.py`

- File type: Python source file.
- Tracked size: 6638 bytes; 242 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes TrajectoryPose, Waypoint, WaypointMission, MinJerkSegment; defines functions angle_diff_rad, yaw_rad_from_deg, yaw_rad_from_quaternion, minimum_jerk_blend, load_waypoint_mission, build_minjerk_segments, sample_minjerk_segment, segment_duration_from_speed, interpolate_pose, _parse_waypoint and more.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; from dataclasses import dataclass; from pathlib import Path; from typing import Any, Optional`
  - third-party: `import yaml`
- Classes:
  - `TrajectoryPose` (lines 42-46, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `Waypoint` (lines 50-57, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `WaypointMission` (lines 61-68, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `MinJerkSegment` (lines 72-77, bases: `object`): No class docstring; role is inferred from methods and base classes.
- Top-level functions:
  - `angle_diff_rad(target_rad: float, source_rad: float) -> float` (lines 11-17): No docstring; behavior is described from its body and call sites.
    Code map: 2 while loops; 1 return points.
  - `yaw_rad_from_deg(value: object) -> float` (lines 20-21): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.radians.
  - `yaw_rad_from_quaternion(x: float, y: float, z: float, w: float) -> float` (lines 24-33): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.atan2.
  - `minimum_jerk_blend(progress: float) -> float` (lines 36-38): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `load_waypoint_mission(path: str) -> WaypointMission` (lines 80-117): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 1 return points; 4 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: Path.expanduser, loaded.get, _float_default, WaypointMission, config_path.is_file, FileNotFoundError, yaml.safe_load, isinstance, ValueError, Path, config_path.read_text, _parse_waypoint, str.strip.
  - `build_minjerk_segments(start_pose: TrajectoryPose, mission: WaypointMission) -> tuple[MinJerkSegment, ...]` (lines 120-165): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 for loops; 1 return points.
    Notable calls: TrajectoryPose, segments.append, segment_duration_from_speed, MinJerkSegment, angle_diff_rad.
  - `sample_minjerk_segment(segment: MinJerkSegment, elapsed_s: float) -> TrajectoryPose` (lines 168-173): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: minimum_jerk_blend, interpolate_pose.
  - `segment_duration_from_speed(start: TrajectoryPose, end: TrajectoryPose, *, cruise_speed_mps: float, min_segment_duration_s: float) -> float` (lines 176-188): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.sqrt.
  - `interpolate_pose(start: TrajectoryPose, end: TrajectoryPose, blend: float) -> TrajectoryPose` (lines 191-202): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: TrajectoryPose.
  - `_parse_waypoint(raw: object, *, default_hold_s: float) -> Waypoint` (lines 205-225): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 return points; 1 raise statements.
    Notable calls: Waypoint, isinstance, ValueError, yaw_rad_from_deg, _optional_float, str.strip, raw.get.
  - `_float_default(mapping: dict[str, Any], key: str, fallback: float) -> float` (lines 228-236): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: mapping.get.
  - `_optional_float(value: object) -> Optional[float]` (lines 239-242): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/mocap_of_compare_logger_node.py`

- File type: Python source file.
- Tracked size: 18337 bytes; 511 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes MocapOfCompareLoggerNode; defines functions _pose_x, _pose_y, _pose_z, _csv_float, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import csv; import math; from copy import deepcopy; from datetime import datetime, timezone; from pathlib import Path; from typing import Optional`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import OpticalFlowRad; from rclpy import qos; from rclpy.node import Node; from sensor_msgs.msg import Range; from std_msgs.msg import String`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace, configured_ownship_pose_topic; from drone_control_pkg.minjerk_waypoint_utils import yaw_rad_from_quaternion; from drone_control_pkg.optical_flow_dead_reckon import OpticalFlowDeadReckoner, OpticalFlowSample, OpticalFlowUpdate; from drone_control_pkg.topic_utils import cdrone_topic, join_topic`
- Top-level constants/state: `CSV_FIELDNAMES, ACTIVE_STATES`.
- Classes:
  - `MocapOfCompareLoggerNode` (lines 101-470, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, mavros_namespace, mocap_pose_topic, flow_rad_topic, flow_range_topic, setpoint_topic, mission_state_topic, of_pose_topic, comparison_frame_id, publish_rate_hz, source_best_effort, reset_on_active_state, run_id, run_dir, csv_path, csv_handle, csv_writer, dead_reckoner, latest_mocap, latest_setpoint, latest_range, latest_flow, latest_update, latest_state, was_active, origin_reset_pose, of_pose_pub, timer, declare_parameter, create_subscription, mocap_callback, flow_callback, range_callback, setpoint_callback, state_callback, create_publisher, create_timer, timer_callback, _flow_range_m, _pose_yaw` plus more.
    - `__init__(self) -> None` (lines 102-260): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, str.strip, self.run_dir.mkdir, self.csv_path.open, csv.DictWriter, self.csv_writer.writeheader, OpticalFlowDeadReckoner, qos.QoSProfile, self.create_subscription, self.create_publisher, self.create_timer, configured_drone_id, configured_mavros_namespace, configured_ownship_pose_topic, join_topic, cdrone_topic, datetime.now.strftime....
    - `now_s(self) -> float` (lines 262-263): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `mocap_callback(self, msg: PoseStamped) -> None` (lines 265-268): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: handles pose messages.
      Notable calls: deepcopy, self.reset_dead_reckoner.
    - `range_callback(self, msg: Range) -> None` (lines 270-271): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: deepcopy.
    - `setpoint_callback(self, msg: PoseStamped) -> None` (lines 273-274): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: deepcopy.
    - `state_callback(self, msg: String) -> None` (lines 276-282): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: str.strip, self.reset_dead_reckoner.
    - `flow_callback(self, msg: OpticalFlowRad) -> None` (lines 284-303): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Notable calls: self._flow_range_m, OpticalFlowSample, self._pose_yaw, self.dead_reckoner.update, self.publish_of_pose, self.reset_dead_reckoner.
    - `reset_dead_reckoner(self) -> None` (lines 305-327): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Notable calls: self._pose_yaw, self.dead_reckoner.reset, deepcopy.
    - `publish_of_pose(self) -> None` (lines 329-342): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: publishes messages, handles pose messages.
      Notable calls: PoseStamped, math.sin, math.cos, self.of_pose_pub.publish.
    - `timer_callback(self) -> None` (lines 344-351): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Notable calls: self.csv_writer.writerow, self.csv_handle.flush, self.publish_of_pose, self.reset_dead_reckoner, self._csv_row.
    - `destroy_node(self) -> bool` (lines 353-358): No docstring; behavior is described from its body and call sites.
      Code map: 1 try/except blocks; 1 return points.
      Notable calls: self.csv_handle.flush, self.csv_handle.close, super.destroy_node.
    - `_csv_row(self) -> dict[str, object]` (lines 360-439): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self._frame_match, self._pose_yaw, math.hypot, math.sqrt, self.now_s, datetime.now.isoformat, _pose_x, _pose_y, _pose_z, _csv_float, datetime.now.
    - `_flow_range_m(self, msg: OpticalFlowRad) -> float` (lines 441-446): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: math.isfinite.
    - `_pose_yaw(self, msg: Optional[PoseStamped]) -> float` (lines 448-452): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: handles pose messages.
      Notable calls: yaw_rad_from_quaternion.
    - `_frame_match(self, mocap: Optional[PoseStamped], setpoint: Optional[PoseStamped], of_pose: object) -> bool` (lines 454-470): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Runtime interactions: handles pose messages.
      Notable calls: str.strip, getattr.
- Top-level functions:
  - `_pose_x(msg: Optional[PoseStamped]) -> str` (lines 473-474): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: handles pose messages.
    Notable calls: _csv_float.
  - `_pose_y(msg: Optional[PoseStamped]) -> str` (lines 477-478): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: handles pose messages.
    Notable calls: _csv_float.
  - `_pose_z(msg: Optional[PoseStamped]) -> str` (lines 481-482): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: handles pose messages.
    Notable calls: _csv_float.
  - `_csv_float(value: object) -> str` (lines 485-494): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 try/except blocks; 4 return points.
    Notable calls: math.isfinite.
  - `main(args = None) -> None` (lines 497-507): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, MocapOfCompareLoggerNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/of_compare_stats.py`

- File type: Python source file.
- Tracked size: 23133 bytes; 654 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines functions generate_stats, write_report, render_markdown, main, _read_rows, _eligible_rows, _duration_s, _time_range, _quality_summary, _range_summary and more; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import csv; import json; import math; import os; from collections import Counter; from pathlib import Path; from statistics import mean, median; from typing import Iterable, Optional, Sequence`
- Top-level constants/state: `ERROR_COLUMNS`.
- Top-level functions:
  - `generate_stats(csv_path: str | Path) -> dict[str, object]` (lines 17-62): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 return points.
    Notable calls: Path.expanduser, _read_rows, _eligible_rows, _numeric_values, _series_summary, _duration_s, _time_range, _value_counts, _reject_reason_counts, _quality_summary, _range_summary, _drift_summary, Path, row.get, _truthy.
  - `write_report(csv_path: str | Path, *, output_dir: str | Path | None = None, plots: bool = False) -> dict[str, object]` (lines 65-92): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Path.expanduser, generate_stats, report_dir.mkdir, summary_path.write_text, markdown_path.write_text, path.relative_to.as_posix, _write_plots, render_markdown, Path, _write_timeseries_csv, json.dumps, path.relative_to, _read_rows.
  - `render_markdown(summary: dict[str, object]) -> str` (lines 95-186): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 for loops; 1 return points.
    Notable calls: summary.get, lines.extend, lines.append, '\n'.join, errors.get, column.removeprefix.removesuffix, _fmt, counts.get, column.removeprefix, stats.get, z_errors.get, drift.get, partial_drift.get, range_summary.get, quality.get.
  - `main(argv: Optional[list[str]] = None) -> int` (lines 189-239): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, write_report, summary.get, _fmt, ','.join, err_xy.get, err_3d.get.
  - `_read_rows(path: Path) -> list[dict[str, str]]` (lines 242-244): No docstring; behavior is described from its body and call sites.
    Code map: 1 context managers; 1 return points.
    Notable calls: path.open, csv.DictReader.
  - `_eligible_rows(rows: Sequence[dict[str, str]], *, columns: Sequence[str], require_sample_valid: bool = True) -> list[dict[str, str]]` (lines 247-261): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 for loops; 1 return points.
    Notable calls: all, _truthy, result.append, row.get, _float_or_none.
  - `_duration_s(rows: Sequence[dict[str, str]]) -> Optional[float]` (lines 264-268): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: _numeric_values, row.get.
  - `_time_range(rows: Sequence[dict[str, str]]) -> dict[str, object]` (lines 271-277): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: rows.get.
  - `_quality_summary(rows: Sequence[dict[str, str]]) -> dict[str, Optional[float]]` (lines 280-282): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _numeric_values, _series_min_median_mean, row.get.
  - `_range_summary(rows: Sequence[dict[str, str]]) -> dict[str, Optional[float]]` (lines 285-289): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: _numeric_values, _series_summary, row.get.
  - `_reject_reason_counts(rows: Sequence[dict[str, str]]) -> dict[str, int]` (lines 292-297): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 return points.
    Notable calls: Counter, counter.most_common, row.get.
  - `_value_counts(values: Iterable[object]) -> dict[str, int]` (lines 300-302): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Counter, counter.most_common.
  - `_drift_summary(rows: Sequence[dict[str, str]]) -> dict[str, Optional[float]]` (lines 305-337): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: _relative_times, _numeric_values, _linear_slope, row.get.
  - `_write_plots(csv_path: Path, report_dir: Path) -> list[str]` (lines 340-362): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks; 2 return points.
    Notable calls: _read_rows, plot_dir.mkdir, os.environ.setdefault, Path.mkdir, outputs.extend, matplotlib.use, _plot_errors, _plot_drift, _plot_trajectory, _plot_flow_health, path.relative_to.as_posix, Path, error_path.write_text, error_path.relative_to.as_posix, path.relative_to, error_path.relative_to.
  - `_plot_errors(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]` (lines 365-399): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: _eligible_rows, _relative_times, fig.suptitle, fig.tight_layout, fig.savefig, plt.close, plt.subplots, axes.plot, axes.set_ylabel, axes.legend, axes.grid, axes.set_xlabel, ax.plot, ax.set_ylabel, ax.set_xlabel, ax.legend, ax.grid, _series.
  - `_plot_drift(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]` (lines 402-428): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 2 return points.
    Notable calls: _eligible_rows, _relative_times, _series, plt.subplots, ax.plot, ax.set_title, ax.set_xlabel, ax.set_ylabel, ax.legend, ax.grid, fig.tight_layout, fig.savefig, plt.close, _linear_slope, mean.
  - `_plot_trajectory(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]` (lines 431-457): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: _eligible_rows, plt.subplots, ax.plot, ax.set_title, ax.set_xlabel, ax.set_ylabel, ax.axis, ax.legend, ax.grid, fig.tight_layout, fig.savefig, plt.close, _series.
  - `_plot_flow_health(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]` (lines 460-495): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: _relative_times, _series, plt.subplots, axes.plot, axes.axhline, axes.set_ylabel, axes.legend, axes.grid, axes.set_xlabel, fig.suptitle, fig.tight_layout, fig.savefig, plt.close, _float_or_none, row.get.
  - `_write_timeseries_csv(rows: Sequence[dict[str, str]], report_dir: Path) -> list[Path]` (lines 498-533): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 context managers; 2 return points.
    Notable calls: _eligible_rows, _relative_times, path.open, csv.DictWriter, writer.writeheader, writer.writerow, row.get.
  - `_series(rows: Sequence[dict[str, str]], column: str) -> list[float]` (lines 536-537): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _float_or_none, row.get.
  - `_relative_times(rows: Sequence[dict[str, str]]) -> list[float]` (lines 540-554): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 2 return points.
    Notable calls: _float_or_none, row.get, times.append.
  - `_series_summary(values: Sequence[float]) -> dict[str, Optional[float]]` (lines 557-586): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: mean, median, math.sqrt, _percentile, math.isfinite.
  - `_series_min_median_mean(values: Sequence[float]) -> dict[str, Optional[float]]` (lines 589-598): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: median, mean, math.isfinite.
  - `_linear_slope(times: Sequence[float], values: Sequence[float]) -> Optional[float]` (lines 601-609): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
    Notable calls: mean.
  - `_percentile(values: Sequence[float], fraction: float) -> float` (lines 612-622): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
    Notable calls: sorted, math.floor, math.ceil.
  - `_numeric_values(values: Iterable[object]) -> list[float]` (lines 625-631): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points.
    Notable calls: _float_or_none, result.append.
  - `_float_or_none(value: object) -> Optional[float]` (lines 634-643): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 try/except blocks; 4 return points.
    Notable calls: math.isfinite.
  - `_truthy(value: object) -> bool` (lines 646-647): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip.lower, str.strip.
  - `_fmt(value: object) -> str` (lines 650-654): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: _float_or_none.
- Whole-file runtime themes: defines a CLI.

### `ros2/src/drone_control_pkg/drone_control_pkg/optical_flow_dead_reckon.py`

- File type: Python source file.
- Tracked size: 5929 bytes; 194 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes OpticalFlowSample, OpticalFlowPose, OpticalFlowUpdate, OpticalFlowDeadReckoner.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; from dataclasses import dataclass; from typing import Optional`
- Classes:
  - `OpticalFlowSample` (lines 9-17, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `OpticalFlowPose` (lines 21-26, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `OpticalFlowUpdate` (lines 30-38, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `OpticalFlowDeadReckoner` (lines 41-194, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `frame_id, quality_min, range_min_m, range_max_m, gyro_compensation_gain, flow_scale_x, flow_scale_y, pose, origin_pose, origin_range_m, _reject_reason, _gyro_compensation, _z_from_range`.
    - `__init__(self, *, frame_id: str = 'map', quality_min: int = 10, range_min_m: float = 0.2, range_max_m: float = 5.0, gyro_compensation_gain: float = 1.0, flow_scale_x: float = 1.0, flow_scale_y: float = 1.0) -> None` (lines 42-62): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `reset(self, *, x_m: float, y_m: float, z_m: float, yaw_rad: float, range_m: Optional[float] = None, frame_id: str = '') -> OpticalFlowPose` (lines 64-89): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: OpticalFlowPose, math.isfinite.
    - `update(self, sample: OpticalFlowSample, *, yaw_rad: float) -> OpticalFlowUpdate` (lines 91-161): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: self._reject_reason, math.cos, math.sin, OpticalFlowPose, OpticalFlowUpdate, self._gyro_compensation, self._z_from_range.
    - `_z_from_range(self, range_m: float) -> float` (lines 163-170): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 4 return points.
      Notable calls: math.isfinite.
    - `_reject_reason(self, sample: OpticalFlowSample) -> str` (lines 172-185): No docstring; behavior is described from its body and call sites.
      Code map: 6 conditional branches; 7 return points.
      Notable calls: math.isfinite.
    - `_gyro_compensation(self, value: float) -> float` (lines 187-194): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: math.isfinite.

### `ros2/src/drone_control_pkg/drone_control_pkg/perimeter_utils.py`

- File type: Python source file.
- Tracked size: 9087 bytes; 286 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes PolygonRegion, AltitudeLimits, PerimeterGuard; defines functions _distance_point_to_segment, _point_in_polygon, _distance_to_polygon_edges, _optional_float, _parse_vertices.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; from dataclasses import dataclass; from pathlib import Path; from typing import Optional`
  - third-party: `import yaml`
- Classes:
  - `PolygonRegion` (lines 82-84, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `AltitudeLimits` (lines 88-90, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `PerimeterGuard` (lines 94-258, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `keep_outs, xy_violation_reason, altitude_limits, boundary, frame_id`.
    - `load_from_yaml(cls, path: str) -> 'PerimeterGuard'` (lines 102-151): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 1 for loops; 1 return points; 5 raise statements.
      Runtime interactions: loads or writes YAML.
      Notable calls: Path.expanduser, PolygonRegion, AltitudeLimits, cls, config_path.is_file, FileNotFoundError, yaml.safe_load, isinstance, ValueError, str.strip, loaded.get, keep_outs.append, Path, config_path.read_text, _parse_vertices, _optional_float, altitude_limits_raw.get, item.get.
    - `frame_matches(self, frame_id: str) -> bool` (lines 153-158): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: str.strip.
    - `xy_violation_reason(self, x_m: float, y_m: float, *, boundary_tolerance_m: float = 0.0, boundary_margin_m: float = 0.0, keep_out_margin_m: float = 0.0) -> Optional[str]` (lines 160-194): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 for loops; 4 return points.
      Notable calls: _point_in_polygon, _distance_to_polygon_edges.
    - `segment_violation_reason(self, start_xy: tuple[float, float], end_xy: tuple[float, float], *, sample_step_m: float = 0.1, boundary_tolerance_m: float = 0.0, boundary_margin_m: float = 0.0, keep_out_margin_m: float = 0.0) -> Optional[str]` (lines 196-224): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
      Notable calls: math.hypot, self.xy_violation_reason, math.ceil.
    - `goal_altitude_violation_reason(self, goal_z_m: float) -> Optional[str]` (lines 226-243): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
    - `runtime_ceiling_violation_reason(self, current_z_m: float, *, ceiling_tolerance_m: float = 0.0) -> Optional[str]` (lines 245-258): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
- Top-level functions:
  - `_distance_point_to_segment(point_xy: tuple[float, float], start_xy: tuple[float, float], end_xy: tuple[float, float]) -> float` (lines 11-35): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
    Notable calls: math.hypot.
  - `_point_in_polygon(point_xy: tuple[float, float], vertices_xy: tuple[tuple[float, float], ...], *, edge_tolerance_m: float = 0.0) -> bool` (lines 38-64): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 for loops; 2 return points.
    Notable calls: _distance_point_to_segment.
  - `_distance_to_polygon_edges(point_xy: tuple[float, float], vertices_xy: tuple[tuple[float, float], ...]) -> float` (lines 67-78): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _distance_point_to_segment.
  - `_optional_float(value: object) -> Optional[float]` (lines 261-264): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `_parse_vertices(raw_vertices: object, *, region_name: str) -> tuple[tuple[float, float], ...]` (lines 267-286): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 for loops; 1 return points; 3 raise statements.
    Notable calls: isinstance, ValueError, vertices.append.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/position_goto_demo_sequence_node.py`

- File type: Python source file.
- Tracked size: 107879 bytes; 2524 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes PositionGotoDemoSequenceNode; defines functions clamp, normalize_takeoff_strategy, normalize_demo_mode, parameter_value_to_float, parameter_value_is_declared_numeric, quaternion_from_yaw, polygon_centroid_xy, polygon_max_radius_from_center, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; import os; from typing import Optional, Tuple; from geographic_msgs.msg import GeoPointStamped; from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue; from rcl_interfaces.srv import GetParameters, SetParameters; from std_srvs.srv import Trigger`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import CompanionProcessStatus, HomePosition, State; from mavros_msgs.srv import CommandBool, CommandTOLLocal, ParamPull, SetMode; from rclpy.node import Node; from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy; from std_msgs.msg import String`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace, configured_ownship_pose_topic; from drone_control_pkg.minjerk_waypoint_utils import MinJerkSegment, TrajectoryPose, WaypointMission, build_minjerk_segments, load_waypoint_mission, sample_minjerk_segment; from drone_control_pkg.perimeter_utils import PerimeterGuard; from drone_control_pkg.px4_param_profile import Px4ParamProfile; from drone_control_pkg.topic_utils import cdrone_topic, join_topic`
- Classes:
  - `PositionGotoDemoSequenceNode` (lines 149-2506, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `publish_rate_hz, drone_id, mavros_namespace, demo_mode, takeoff_altitude_m, takeoff_rate_m_s, takeoff_strategy, altitude_tolerance_m, touchdown_altitude_m, touchdown_dwell_s, stage_timeout_s, arm_zero_throttle_hold_s, local_pose_timeout_s, local_pose_timeout_during_param_sync_s, state_timeout_s, state_timeout_during_param_sync_s, connection_loss_timeout_s, connection_loss_timeout_during_param_sync_s, mode_request_retry_interval_s, companion_status_timeout_s, require_companion_active, restore_takeoff_alt_on_exit, takeoff_param_id, param_pull_force, param_pull_retry_delay_s, param_sync_timeout_s, use_speed_profile, speed_profile_config, restore_speed_profile_on_exit, offboard_setpoint_warmu...` plus more.
    - `__init__(self, node_name: str = 'position_goto_demo_sequence_node') -> None` (lines 163-593): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, normalize_demo_mode, normalize_takeoff_strategy, str.strip, State, PoseStamped, self.now_s, self.create_publisher, QoSProfile, self.create_subscription, self.create_client, self.create_service, self.create_timer, self.load_waypoint_mission, self.load_perimeter_guard, self.load_speed_profile, self.publish_status....
    - `now_s(self) -> float` (lines 595-596): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `state_callback(self, msg: State) -> None` (lines 598-609): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
      Notable calls: self.now_s.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 611-613): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `mocap_pose_callback(self, msg: PoseStamped) -> None` (lines 615-617): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `home_position_callback(self, msg: HomePosition) -> None` (lines 619-621): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `global_origin_callback(self, msg: GeoPointStamped) -> None` (lines 623-625): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `companion_status_callback(self, msg: CompanionProcessStatus) -> None` (lines 627-636): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `current_altitude_m(self) -> float` (lines 638-639): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_xy(self) -> Tuple[float, float]` (lines 641-645): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_yaw_rad(self) -> float` (lines 647-655): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: math.atan2.
    - `pose_fresh(self) -> bool` (lines 657-661): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `mocap_pose_fresh(self) -> bool` (lines 663-667): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `current_mocap_frame_id(self) -> str` (lines 669-670): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.strip.
    - `active_state_timeout_s(self) -> float` (lines 672-682): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `state_fresh(self) -> bool` (lines 684-685): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.active_state_timeout_s, self.now_s.
    - `companion_status_fresh(self) -> bool` (lines 687-690): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `home_position_ready(self) -> bool` (lines 692-693): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `global_origin_ready(self) -> bool` (lines 695-696): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `active_connection_loss_timeout_s(self) -> float` (lines 698-710): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `mode_matches(self, mode_name: str) -> bool` (lines 712-713): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `mode_request_retry_ready(self) -> bool` (lines 715-720): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.now_s.
    - `is_land_mode(self) -> bool` (lines 722-723): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `is_takeoff_mode(self) -> bool` (lines 725-726): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `is_takeoff_handoff_mode(self) -> bool` (lines 728-730): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `takeoff_target_altitude_m(self) -> float` (lines 732-735): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `touchdown_threshold_altitude_m(self) -> float` (lines 737-740): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `current_frame_id(self) -> str` (lines 742-743): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.strip.
    - `goal_xy(self) -> Tuple[float, float]` (lines 745-746): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `circle_center_xy(self) -> Tuple[float, float]` (lines 748-749): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `circle_direction_sign(self) -> float` (lines 751-752): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `circle_total_angle_rad(self) -> float` (lines 754-755): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `circle_total_duration_s(self) -> float` (lines 757-760): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.circle_total_angle_rad.
    - `circle_point_xy(self, angle_rad: float) -> Tuple[float, float]` (lines 762-766): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: math.cos, math.sin.
    - `circle_entry_pose(self) -> Optional[PoseStamped]` (lines 768-777): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: handles pose messages.
      Notable calls: self.circle_point_xy, self.make_pose_setpoint, self.current_yaw_rad.
    - `circle_entry_xy(self) -> Optional[Tuple[float, float]]` (lines 779-782): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.circle_point_xy.
    - `configure_circle_keep_out_orbit(self) -> Optional[str]` (lines 784-815): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 4 return points.
      Notable calls: next, polygon_centroid_xy, polygon_max_radius_from_center.
    - `choose_circle_entry_angle(self, start_xy: Tuple[float, float]) -> Tuple[Optional[float], Optional[float]]` (lines 817-850): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 for loops; 2 return points.
      Notable calls: self.circle_point_xy, self.perimeter_guard.segment_violation_reason, math.hypot.
    - `log_circle_plan(self, start_xy: Tuple[float, float], entry_distance_m: Optional[float] = None) -> None` (lines 852-879): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
      Notable calls: self.circle_entry_xy.
    - `load_waypoint_mission(self) -> None` (lines 881-907): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 3 return points.
      Notable calls: load_waypoint_mission.
    - `load_perimeter_guard(self) -> None` (lines 909-935): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 3 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: PerimeterGuard.load_from_yaml, os.path.basename.
    - `load_speed_profile(self) -> None` (lines 937-961): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 3 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: Px4ParamProfile.load_from_yaml, self.speed_profile.parameters.keys.
    - `should_restore_speed_profile(self) -> bool` (lines 963-968): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `speed_profile_apply_complete(self) -> bool` (lines 970-977): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: all.
    - `next_speed_profile_param_to_apply(self) -> Optional[str]` (lines 979-986): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
    - `next_speed_profile_param_to_restore(self) -> Optional[str]` (lines 988-992): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
    - `perimeter_start_violation_reason(self, start_xy: Tuple[float, float]) -> Optional[str]` (lines 994-1143): No docstring; behavior is described from its body and call sites.
      Code map: 23 conditional branches; 2 for loops; 24 return points.
      Notable calls: self.current_frame_id, self.perimeter_guard.xy_violation_reason, self.perimeter_guard.goal_altitude_violation_reason, self.perimeter_guard.segment_violation_reason, self.perimeter_guard.frame_matches, self.configure_circle_keep_out_orbit, self.choose_circle_entry_angle, self.log_circle_plan, self.goal_xy, self.circle_point_xy.
    - `perimeter_runtime_violation_reason(self) -> Optional[str]` (lines 1145-1163): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: self.current_xy, self.perimeter_guard.xy_violation_reason, self.perimeter_guard.runtime_ceiling_violation_reason, self.current_altitude_m.
    - `publish_status(self) -> None` (lines 1165-1168): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: String, self.status_pub.publish.
    - `startable(self) -> Tuple[bool, str]` (lines 1170-1285): No docstring; behavior is described from its body and call sites.
      Code map: 36 conditional branches; 30 return points.
      Notable calls: self.current_xy, self.perimeter_start_violation_reason, self.mode_client.wait_for_service, self.arm_client.wait_for_service, self.param_get_client.wait_for_service, self.param_pull_client.wait_for_service, self.param_set_client.wait_for_service, self.state_fresh, self.pose_fresh, self.current_mocap_frame_id, self.current_frame_id, self.global_origin_ready, self.home_position_ready, math.hypot, self.choose_circle_entry_angle, self.takeoff_client.wait_for_service, self.mocap_pose_fresh, self.companion_status_fresh....
    - `handle_start_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 1287-1337): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.startable, self.current_xy, self.current_altitude_m, self.transition_to.
    - `handle_abort_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 1339-1351): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.enter_abort.
    - `transition_to(self, new_state: str, reason: str = '') -> None` (lines 1353-1373): No docstring; behavior is described from its body and call sites.
      Code map: 7 conditional branches.
      Notable calls: self.now_s, self.publish_status.
    - `enter_abort(self, reason: str) -> None` (lines 1375-1380): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.transition_to.
    - `stage_elapsed_s(self) -> float` (lines 1382-1383): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `stage_timeout_limit_s(self) -> Optional[float]` (lines 1385-1408): No docstring; behavior is described from its body and call sites.
      Code map: 9 conditional branches; 10 return points.
      Notable calls: self.circle_total_duration_s.
    - `request_mode(self, mode_name: str) -> None` (lines 1410-1421): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: requests PX4/MAVROS mode changes.
      Notable calls: SetMode.Request, self.mode_client.call_async, self.now_s, self.mode_client.wait_for_service, self.enter_abort.
    - `request_arm(self, arm_value: bool) -> None` (lines 1423-1434): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: arms/disarms through MAVROS.
      Notable calls: CommandBool.Request, self.arm_client.call_async, self.arm_client.wait_for_service, self.enter_abort.
    - `request_takeoff(self) -> None` (lines 1436-1464): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: requests takeoff/land through MAVROS.
      Notable calls: self.current_xy, self.takeoff_target_altitude_m, CommandTOLLocal.Request, self.current_yaw_rad, self.takeoff_client.call_async, self.takeoff_client.wait_for_service, self.enter_abort.
    - `request_param_get(self, *, param_name: Optional[str] = None, kind: str = 'GET_ORIGINAL') -> None` (lines 1466-1486): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: str.strip, GetParameters.Request, self.param_get_client.call_async, self.enter_abort, self.param_get_client.wait_for_service.
    - `request_param_pull(self) -> None` (lines 1488-1501): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Notable calls: ParamPull.Request, self.param_pull_client.call_async, self.param_pull_client.wait_for_service, self.enter_abort.
    - `schedule_param_pull_retry(self, reason: str) -> None` (lines 1503-1511): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.now_s.
    - `request_param_set(self, value: float, *, kind: str, param_name: Optional[str] = None) -> None` (lines 1513-1545): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: str.strip, SetParameters.Request, self.param_set_client.call_async, self.enter_abort, self.param_set_client.wait_for_service, Parameter, ParameterValue.
    - `skip_speed_profile_param(self, param_name: str, reason: str) -> None` (lines 1547-1556): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.speed_profile_skipped_names.add, self.speed_profile_original_values.pop, self.speed_profile_changed_names.discard, self.speed_profile_applied_names.discard.
    - `poll_service_futures(self) -> None` (lines 1558-1828): No docstring; behavior is described from its body and call sites.
      Code map: 35 conditional branches; 5 try/except blocks; 12 return points.
      Notable calls: self.mode_future.done, self.arm_future.done, self.takeoff_future.done, self.param_pull_future.done, self.param_future.done, self.mode_future.result, self.arm_future.result, self.takeoff_future.result, self.param_pull_future.result, self.param_future.result, self.enter_abort, self.is_land_mode, self.schedule_param_pull_retry, parameter_value_to_float, math.isclose, self.current_altitude_m, self.touchdown_threshold_altitude_m, parameter_value_is_declared_numeric....
    - `make_pose_setpoint(self, x_m: float, y_m: float, z_m: float, yaw_rad: float) -> PoseStamped` (lines 1830-1848): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages.
      Notable calls: PoseStamped, quaternion_from_yaw.
    - `current_hold_pose(self) -> PoseStamped` (lines 1850-1856): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages.
      Notable calls: PoseStamped.
    - `goal_pose(self) -> PoseStamped` (lines 1858-1865): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages.
      Notable calls: self.make_pose_setpoint, self.current_yaw_rad.
    - `trajectory_pose_from_current(self) -> TrajectoryPose` (lines 1867-1873): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: TrajectoryPose, self.current_yaw_rad.
    - `pose_from_trajectory(self, pose: TrajectoryPose) -> PoseStamped` (lines 1875-1881): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages.
      Notable calls: self.make_pose_setpoint.
    - `start_waypoint_plan(self) -> bool` (lines 1883-1907): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: self.trajectory_pose_from_current, build_minjerk_segments, self.now_s, self.pose_from_trajectory, self.enter_abort.
    - `update_waypoint_setpoint(self) -> bool` (lines 1909-1933): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Notable calls: self.now_s, self.pose_from_trajectory, sample_minjerk_segment.
    - `circle_pose(self, angle_rad: float) -> PoseStamped` (lines 1935-1942): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages.
      Notable calls: self.circle_point_xy, self.make_pose_setpoint, self.current_yaw_rad.
    - `publish_setpoint_for_state(self) -> None` (lines 1944-1953): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: publishes messages, handles pose messages.
      Notable calls: PoseStamped, self.position_setpoint_pub.publish.
    - `goal_position_error_m(self) -> Optional[float]` (lines 1955-1961): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: math.sqrt.
    - `run_safety_checks(self) -> None` (lines 1963-2053): No docstring; behavior is described from its body and call sites.
      Code map: 18 conditional branches; 10 return points.
      Notable calls: self.perimeter_runtime_violation_reason, self.stage_timeout_limit_s, self.state_fresh, self.enter_abort, self.now_s, self.active_connection_loss_timeout_s, self.pose_fresh, self.companion_status_fresh, self.is_takeoff_mode, self.is_takeoff_handoff_mode, self.mode_matches, self.stage_elapsed_s.
    - `step_state_machine(self) -> None` (lines 2055-2499): No docstring; behavior is described from its body and call sites.
      Code map: 109 conditional branches; 63 return points.
      Notable calls: self.speed_profile_apply_complete, self.next_speed_profile_param_to_apply, math.isclose, self.request_param_set, self.is_takeoff_mode, self.current_altitude_m, self.takeoff_target_altitude_m, self.is_takeoff_handoff_mode, self.mode_matches, self.goal_position_error_m, self.circle_total_duration_s, self.circle_pose, self.update_waypoint_setpoint, self.is_land_mode, self.next_speed_profile_param_to_restore, self.speed_profile_original_values.get, self.should_restore_speed_profile, self.transition_to....
    - `timer_callback(self) -> None` (lines 2501-2506): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.poll_service_futures, self.run_safety_checks, self.step_state_machine, self.publish_setpoint_for_state, self.publish_status.
- Top-level functions:
  - `clamp(value: float, min_v: float, max_v: float) -> float` (lines 37-38): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `normalize_takeoff_strategy(value: str) -> str` (lines 41-56): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Notable calls: str.strip.upper, ValueError, str.strip.
  - `normalize_demo_mode(value: str) -> str` (lines 59-77): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Notable calls: str.strip.lower, ValueError, str.strip.
  - `parameter_value_to_float(value: ParameterValue) -> float` (lines 80-89): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 5 return points.
  - `parameter_value_is_declared_numeric(value: ParameterValue) -> bool` (lines 92-96): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]` (lines 99-101): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.sin, math.cos.
  - `polygon_centroid_xy(vertices_xy: tuple[tuple[float, float], ...]) -> tuple[float, float]` (lines 104-122): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 2 return points.
    Notable calls: math.isclose.
  - `polygon_max_radius_from_center(center_xy: tuple[float, float], vertices_xy: tuple[tuple[float, float], ...]) -> float` (lines 125-146): No docstring; behavior is described from its body and call sites.
    Code map: 2 for loops; 1 return points.
    Notable calls: math.hypot.
  - `main(args = None) -> None` (lines 2509-2520): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, PositionGotoDemoSequenceNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, publishes messages, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages, loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/position_hover_demo_sequence_node.py`

- File type: Python source file.
- Tracked size: 74332 bytes; 1746 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes PositionHoverDemoSequenceNode; defines functions clamp, normalize_hover_mode, normalize_takeoff_strategy, parameter_value_to_float, parameter_value_is_declared_numeric, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; from typing import Optional, Tuple; from geographic_msgs.msg import GeoPointStamped; from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue; from rcl_interfaces.srv import GetParameters, SetParameters; from std_srvs.srv import Trigger`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import CompanionProcessStatus, HomePosition, ManualControl, State; from mavros_msgs.srv import CommandBool, CommandTOLLocal, ParamPull, SetMode; from rclpy.node import Node; from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy; from std_msgs.msg import String`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.px4_param_profile import Px4ParamProfile; from drone_control_pkg.topic_utils import cdrone_topic, join_topic`
- Classes:
  - `PositionHoverDemoSequenceNode` (lines 82-1728, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `publish_rate_hz, drone_id, mavros_namespace, takeoff_altitude_m, takeoff_rate_m_s, takeoff_strategy, auto_takeoff_fallback_enabled, auto_takeoff_fallback_timeout_s, auto_takeoff_fallback_min_delta_m, hover_duration_s, hover_mode, altitude_tolerance_m, start_altitude_limit_m, touchdown_altitude_m, touchdown_dwell_s, stage_timeout_s, arm_zero_throttle_hold_s, manual_hover_throttle_center, local_pose_timeout_s, local_pose_timeout_during_param_sync_s, state_timeout_s, state_timeout_during_param_sync_s, connection_loss_timeout_s, connection_loss_timeout_during_param_sync_s, mode_request_retry_interval_s, companion_status_timeout_s, require_companion_active, max_horizontal_excursion_m, restore_...` plus more.
    - `__init__(self) -> None` (lines 85-402): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages.
      Notable calls: super.__init__, self.declare_parameter, normalize_takeoff_strategy, normalize_hover_mode, str.strip, State, PoseStamped, self.now_s, self.create_publisher, QoSProfile, self.create_subscription, self.create_client, self.create_service, self.create_timer, self.load_speed_profile, self.publish_status, configured_drone_id, configured_mavros_namespace....
    - `now_s(self) -> float` (lines 404-405): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `state_callback(self, msg: State) -> None` (lines 407-418): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
      Notable calls: self.now_s.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 420-422): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `home_position_callback(self, msg: HomePosition) -> None` (lines 424-426): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `global_origin_callback(self, msg: GeoPointStamped) -> None` (lines 428-430): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `companion_status_callback(self, msg: CompanionProcessStatus) -> None` (lines 432-441): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `current_altitude_m(self) -> float` (lines 443-444): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `current_xy(self) -> Tuple[float, float]` (lines 446-450): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `horizontal_excursion_m(self) -> float` (lines 452-459): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.current_xy, math.hypot.
    - `takeoff_target_altitude_m(self) -> float` (lines 461-464): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `touchdown_threshold_altitude_m(self) -> float` (lines 466-469): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `current_altitude_above_home_m(self) -> Optional[float]` (lines 471-474): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.current_altitude_m.
    - `takeoff_origin_above_home_m(self) -> Optional[float]` (lines 476-479): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `current_yaw_rad(self) -> float` (lines 481-489): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: math.atan2.
    - `pose_fresh(self) -> bool` (lines 491-495): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `active_state_timeout_s(self) -> float` (lines 497-507): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `state_fresh(self) -> bool` (lines 509-510): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.active_state_timeout_s, self.now_s.
    - `companion_status_fresh(self) -> bool` (lines 512-515): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `home_position_ready(self) -> bool` (lines 517-518): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `global_origin_ready(self) -> bool` (lines 520-521): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `active_connection_loss_timeout_s(self) -> float` (lines 523-535): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `mode_matches(self, mode_name: str) -> bool` (lines 537-538): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `mode_request_retry_ready(self) -> bool` (lines 540-545): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.now_s.
    - `is_land_mode(self) -> bool` (lines 547-548): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `is_takeoff_mode(self) -> bool` (lines 550-551): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: str.upper.
    - `is_takeoff_handoff_mode(self) -> bool` (lines 553-554): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.mode_matches.
    - `publish_status(self) -> None` (lines 556-559): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: String, self.status_pub.publish.
    - `load_speed_profile(self) -> None` (lines 561-587): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 3 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: Px4ParamProfile.load_from_yaml, self.speed_profile.parameters.keys.
    - `should_restore_speed_profile(self) -> bool` (lines 589-594): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `speed_profile_apply_complete(self) -> bool` (lines 596-603): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: all.
    - `next_speed_profile_param_to_apply(self) -> Optional[str]` (lines 605-612): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
    - `next_speed_profile_param_to_restore(self) -> Optional[str]` (lines 614-618): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 2 return points.
    - `publish_manual(self, throttle_cmd: float) -> None` (lines 620-628): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: ManualControl, self.manual_pub.publish, clamp.
    - `startable(self) -> Tuple[bool, str]` (lines 630-675): No docstring; behavior is described from its body and call sites.
      Code map: 18 conditional branches; 18 return points.
      Notable calls: self.mode_client.wait_for_service, self.arm_client.wait_for_service, self.param_get_client.wait_for_service, self.param_pull_client.wait_for_service, self.param_set_client.wait_for_service, self.state_fresh, self.pose_fresh, self.global_origin_ready, self.home_position_ready, self.takeoff_client.wait_for_service, self.companion_status_fresh.
    - `handle_start_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 677-729): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Notable calls: self.startable, self.current_xy, self.current_altitude_m, self.takeoff_origin_above_home_m, self.transition_to.
    - `handle_abort_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response` (lines 731-743): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.enter_abort.
    - `transition_to(self, new_state: str, reason: str = '') -> None` (lines 745-757): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches.
      Notable calls: self.now_s, self.publish_status.
    - `enter_abort(self, reason: str) -> None` (lines 759-764): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.transition_to.
    - `stage_elapsed_s(self) -> float` (lines 766-767): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `stage_timeout_limit_s(self) -> Optional[float]` (lines 769-782): No docstring; behavior is described from its body and call sites.
      Code map: 6 conditional branches; 7 return points.
    - `request_mode(self, mode_name: str) -> None` (lines 784-795): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: requests PX4/MAVROS mode changes.
      Notable calls: SetMode.Request, self.mode_client.call_async, self.now_s, self.mode_client.wait_for_service, self.enter_abort.
    - `request_arm(self, arm_value: bool) -> None` (lines 797-808): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: arms/disarms through MAVROS.
      Notable calls: CommandBool.Request, self.arm_client.call_async, self.arm_client.wait_for_service, self.enter_abort.
    - `request_takeoff(self) -> None` (lines 810-838): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: requests takeoff/land through MAVROS.
      Notable calls: self.current_xy, self.takeoff_target_altitude_m, CommandTOLLocal.Request, self.current_yaw_rad, self.takeoff_client.call_async, self.takeoff_client.wait_for_service, self.enter_abort.
    - `request_param_get(self, *, param_name: Optional[str] = None, kind: str = 'GET_ORIGINAL') -> None` (lines 840-860): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: str.strip, GetParameters.Request, self.param_get_client.call_async, self.enter_abort, self.param_get_client.wait_for_service.
    - `request_param_pull(self) -> None` (lines 862-875): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Notable calls: ParamPull.Request, self.param_pull_client.call_async, self.param_pull_client.wait_for_service, self.enter_abort.
    - `schedule_param_pull_retry(self, reason: str) -> None` (lines 877-885): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.now_s.
    - `request_param_set(self, value: float, *, kind: str, param_name: Optional[str] = None) -> None` (lines 887-919): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: str.strip, SetParameters.Request, self.param_set_client.call_async, self.enter_abort, self.param_set_client.wait_for_service, Parameter, ParameterValue.
    - `skip_speed_profile_param(self, param_name: str, reason: str) -> None` (lines 921-930): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.speed_profile_skipped_names.add, self.speed_profile_original_values.pop, self.speed_profile_changed_names.discard, self.speed_profile_applied_names.discard.
    - `poll_service_futures(self) -> None` (lines 932-1221): No docstring; behavior is described from its body and call sites.
      Code map: 37 conditional branches; 5 try/except blocks; 12 return points.
      Notable calls: self.mode_future.done, self.arm_future.done, self.takeoff_future.done, self.param_pull_future.done, self.param_future.done, self.mode_future.result, self.arm_future.result, self.takeoff_future.result, self.param_pull_future.result, self.param_future.result, self.enter_abort, self.is_land_mode, self.schedule_param_pull_retry, parameter_value_to_float, math.isclose, self.current_altitude_m, self.touchdown_threshold_altitude_m, parameter_value_is_declared_numeric....
    - `run_safety_checks(self) -> None` (lines 1223-1317): No docstring; behavior is described from its body and call sites.
      Code map: 18 conditional branches; 10 return points.
      Notable calls: self.stage_timeout_limit_s, self.state_fresh, self.enter_abort, self.now_s, self.active_connection_loss_timeout_s, self.pose_fresh, self.companion_status_fresh, self.horizontal_excursion_m, self.mode_matches, self.is_takeoff_mode, self.is_takeoff_handoff_mode, self.stage_elapsed_s.
    - `step_state_machine(self) -> None` (lines 1319-1712): No docstring; behavior is described from its body and call sites.
      Code map: 89 conditional branches; 50 return points.
      Notable calls: self.speed_profile_apply_complete, self.next_speed_profile_param_to_apply, math.isclose, self.request_param_set, self.is_takeoff_mode, self.current_altitude_m, self.takeoff_target_altitude_m, self.current_altitude_above_home_m, self.is_takeoff_handoff_mode, self.mode_matches, self.is_land_mode, self.next_speed_profile_param_to_restore, self.speed_profile_original_values.get, self.should_restore_speed_profile, self.transition_to, self.request_param_pull, self.request_param_get, self.speed_profile_applied_names.add....
    - `publish_manual_for_state(self) -> None` (lines 1714-1721): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 return points.
      Notable calls: self.publish_manual.
    - `timer_callback(self) -> None` (lines 1723-1728): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.poll_service_futures, self.run_safety_checks, self.step_state_machine, self.publish_manual_for_state, self.publish_status.
- Top-level functions:
  - `clamp(value: float, min_v: float, max_v: float) -> float` (lines 26-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `normalize_hover_mode(value: str) -> str` (lines 30-42): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Notable calls: str.strip.upper, ValueError, str.strip.
  - `normalize_takeoff_strategy(value: str) -> str` (lines 45-60): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Notable calls: str.strip.upper, ValueError, str.strip.
  - `parameter_value_to_float(value: ParameterValue) -> float` (lines 63-72): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 5 return points.
  - `parameter_value_is_declared_numeric(value: ParameterValue) -> bool` (lines 75-79): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `main(args = None) -> None` (lines 1731-1742): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, PositionHoverDemoSequenceNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, calls ROS services, publishes messages, requests PX4/MAVROS mode changes, arms/disarms through MAVROS, requests takeoff/land through MAVROS, handles pose messages, loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/px4_param_profile.py`

- File type: Python source file.
- Tracked size: 1571 bytes; 45 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes Px4ParamProfile.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass; from pathlib import Path`
  - third-party: `import yaml`
- Classes:
  - `Px4ParamProfile` (lines 10-45, bases: `object`): No class docstring; role is inferred from methods and base classes.
    - `load_from_yaml(cls, path: str) -> 'Px4ParamProfile'` (lines 17-45): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 for loops; 1 return points; 4 raise statements.
      Runtime interactions: loads or writes YAML.
      Notable calls: Path.expanduser, loaded.get, raw_parameters.items, str.strip, cls, config_path.is_file, FileNotFoundError, yaml.safe_load, isinstance, ValueError, Path, config_path.read_text.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_control_pkg/drone_control_pkg/target_follow_controller_node.py`

- File type: Python source file.
- Tracked size: 18526 bytes; 467 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines classes TargetFollowControllerNode; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from typing import Dict, Optional; from std_srvs.srv import SetBool`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped, TwistStamped; from mavros_msgs.msg import CompanionProcessStatus, State; from rclpy.node import Node; from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy; from std_msgs.msg import Bool, String`
  - local/project: `from drone_msgs.msg import TargetTrackArray; from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.follow_utils import TrackSnapshot, compute_follow_command, score_track, track_is_valid; from drone_control_pkg.topic_utils import cdrone_topic, join_topic`
- Classes:
  - `TargetFollowControllerNode` (lines 26-450, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `publish_rate_hz, drone_id, mavros_namespace, follow_enabled, follow_distance_m, follow_distance_tolerance_m, lateral_deadband_m, vertical_deadband_m, yaw_deadband_rad, track_timeout_s, track_gc_s, state_timeout_s, local_pose_timeout_s, companion_status_timeout_s, min_track_confidence, max_target_distance_m, require_target_in_front, max_abs_target_y_m, max_abs_target_z_m, min_safe_distance_m, require_mavros_connected, require_armed, require_offboard, require_companion_active, publish_zero_on_block, publish_zero_on_lost_target, kp_xy, kp_z, kp_yaw, max_vel_xy_mps, max_vel_z_mps, max_yaw_rate_rps, tracks_topic, cmd_vel_topic, estop_topic, state_topic, local_pose_topic, companion_status_topic...` plus more.
    - `__init__(self) -> None` (lines 27-230): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, handles pose messages, handles velocity setpoints, handles target track messages.
      Notable calls: super.__init__, self.declare_parameter, State, PoseStamped, QoSProfile, self.create_subscription, self.create_publisher, self.create_service, self.create_timer, self.publish_status, configured_drone_id, configured_mavros_namespace, str.strip, cdrone_topic, join_topic, self.get_parameter.
    - `now_s(self) -> float` (lines 232-233): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `state_callback(self, msg: State) -> None` (lines 235-237): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `local_pose_callback(self, msg: PoseStamped) -> None` (lines 239-241): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `companion_status_callback(self, msg: CompanionProcessStatus) -> None` (lines 243-252): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
    - `estop_callback(self, msg: Bool) -> None` (lines 254-255): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `tracks_callback(self, msg: TargetTrackArray) -> None` (lines 257-302): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 for loops.
      Runtime interactions: handles target track messages.
      Notable calls: self.now_s, TrackSnapshot, seen_ids.add, self.tracks.keys, self.tracks.pop, getattr.
    - `handle_enable_request(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response` (lines 304-316): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: publishes messages.
      Notable calls: self.cmd_pub.publish, self.zero_cmd.
    - `state_fresh(self) -> bool` (lines 318-319): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `pose_fresh(self) -> bool` (lines 321-322): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `companion_status_fresh(self) -> bool` (lines 324-327): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.now_s.
    - `track_is_valid(self, track: TrackSnapshot, now_s: float) -> bool` (lines 329-339): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: track_is_valid.
    - `select_target(self, now_s: float) -> Optional[TrackSnapshot]` (lines 341-365): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: candidates.sort, self.tracks.get, self.track_is_valid, self.tracks.values, score_track.
    - `control_block_reason(self, now_s: float) -> str` (lines 367-388): No docstring; behavior is described from its body and call sites.
      Code map: 10 conditional branches; 10 return points.
      Notable calls: self.state_fresh, str.upper, self.pose_fresh, self.companion_status_fresh.
    - `zero_cmd(self) -> TwistStamped` (lines 390-393): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles velocity setpoints.
      Notable calls: TwistStamped.
    - `publish_status(self, status: str) -> None` (lines 395-401): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: publishes messages.
      Notable calls: String, self.status_pub.publish.
    - `publish_follow_command(self, track: TrackSnapshot) -> None` (lines 403-432): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages, handles velocity setpoints.
      Notable calls: compute_follow_command, TwistStamped, self.cmd_pub.publish, self.publish_status.
    - `control_loop(self) -> None` (lines 434-450): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 2 return points.
      Runtime interactions: publishes messages.
      Notable calls: self.now_s, self.control_block_reason, self.select_target, self.publish_follow_command, self.publish_status, self.cmd_pub.publish, self.zero_cmd.
- Top-level functions:
  - `main(args = None) -> None` (lines 453-463): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, TargetFollowControllerNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, exposes ROS services, publishes messages, handles pose messages, handles velocity setpoints, handles target track messages.

### `ros2/src/drone_control_pkg/drone_control_pkg/topic_utils.py`

- File type: Python source file.
- Tracked size: 965 bytes; 35 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: defines functions normalize_ns, join_topic, cdrone_ns, cdrone_topic, external_pose_input_topic, legacy_vio_input_topic.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations`
- Top-level functions:
  - `normalize_ns(namespace: str) -> str` (lines 4-10): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: str.strip, namespace.rstrip, namespace.startswith.
  - `join_topic(namespace: str, leaf: str) -> str` (lines 13-16): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: normalize_ns, str.strip.lstrip, str.strip.
  - `cdrone_ns(drone_id: str) -> str` (lines 19-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip.strip, str.strip.
  - `cdrone_topic(drone_id: str, leaf: str) -> str` (lines 26-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: join_topic, cdrone_ns.
  - `external_pose_input_topic(drone_id: str) -> str` (lines 30-31): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: cdrone_topic.
  - `legacy_vio_input_topic(drone_id: str) -> str` (lines 34-35): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: cdrone_topic.

### `ros2/src/drone_control_pkg/drone_control_pkg/vio_bridge_node.py`

- File type: Python source file.
- Tracked size: 166 bytes; 4 decoded lines.
- Purpose: Control-side Python module or ROS node for PX4/MAVROS integration, demos, safety, or target following.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - local/project: `from drone_control_pkg.external_pose_bridge_node import ExternalPoseBridgeNode as VioBridgeNode; from drone_control_pkg.external_pose_bridge_node import main`

### `ros2/src/drone_control_pkg/launch/mavros.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 1810 bytes; 48 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument; from launch.substitutions import LaunchConfiguration; from launch.launch_context import LaunchContext; from launch.events.process.process_exited import ProcessExited; from launch.event_handlers.on_process_exit import OnProcessExit; from launch.actions import DeclareLaunchArgument, OpaqueFunction; from launch_ros.actions import Node`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `launch_setup(context, *args, **kwargs)` (lines 15-33): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, LaunchConfiguration, Node.
  - `generate_launch_description()` (lines 36-48): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: load_drone_launch_defaults, LaunchDescription, DeclareLaunchArgument, OpaqueFunction, default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_control_pkg/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 1137 bytes; 32 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_control_pkg`.
- Dependencies: `exec_depend:rclpy, exec_depend:mavros, exec_depend:python3-numpy, exec_depend:python3-scipy, exec_depend:python3-yaml, exec_depend:mavros_msgs, exec_depend:std_msgs, exec_depend:std_srvs, exec_depend:geometry_msgs, exec_depend:sensor_msgs, exec_depend:geographic_msgs, exec_depend:drone_msgs, test_depend:ament_copyright, test_depend:ament_flake8, test_depend:ament_pep257, test_depend:python3-pytest`.
- Build type: `ament_python`.

### `ros2/src/drone_control_pkg/resource/drone_control_pkg`

- File type: Text file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Tracked non-text asset used by documentation, runtime, or dependency setup.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 0.

### `ros2/src/drone_control_pkg/setup.cfg`

- File type: INI/setup configuration.
- Tracked size: 103 bytes; 4 decoded lines.
- Purpose: [develop] script_dir=$base/lib/drone_control_pkg [install] install_scripts=$base/lib/drone_control_pkg
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Config sections: `develop, install`.

### `ros2/src/drone_control_pkg/setup.py`

- File type: Python source file.
- Tracked size: 2745 bytes; 65 decoded lines.
- Purpose: Python package installer that registers package data and console script entry points.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: colcon/ament_python during build and ROS 2 console script lookup at runtime.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from setuptools import find_packages, setup; import os; import glob`
- Top-level constants/state: `package_name`.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_control_pkg/test/test_copyright.py`

- File type: Python source file.
- Tracked size: 967 bytes; 27 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_copyright.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_copyright.main import main; import pytest`
- Top-level functions:
  - `test_copyright()` (lines 25-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: pytest.mark.skip, main.

### `ros2/src/drone_control_pkg/test/test_external_pose_adapter.py`

- File type: Python source file.
- Tracked size: 1407 bytes; 44 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_frame_yaw_pi_flips_x_and_y_position, test_body_offset_is_applied_after_frame_rotation.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from math import pi; from pathlib import Path; import sys; import pytest`
  - local/project: `from drone_control_pkg.external_pose_adapter_node import _quaternion_from_euler, _transform_pose_components`
- Top-level functions:
  - `test_frame_yaw_pi_flips_x_and_y_position() -> None` (lines 17-30): No docstring; behavior is described from its body and call sites.
    Code map: 5 assertions.
    Notable calls: _transform_pose_components, _quaternion_from_euler, pytest.approx.
  - `test_body_offset_is_applied_after_frame_rotation() -> None` (lines 33-44): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: _transform_pose_components, _quaternion_from_euler, pytest.approx.

### `ros2/src/drone_control_pkg/test/test_flake8.py`

- File type: Python source file.
- Tracked size: 877 bytes; 25 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_flake8.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_flake8.main import main_with_errors; import pytest`
- Top-level functions:
  - `test_flake8()` (lines 21-25): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main_with_errors, '\n'.join.

### `ros2/src/drone_control_pkg/test/test_follow_utils.py`

- File type: Python source file.
- Tracked size: 3732 bytes; 145 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions make_command, test_clamp_follow_command_altitude_limits_descent_at_floor, test_clamp_follow_command_altitude_preserves_safe_command, test_clamp_follow_command_altitude_limits_ascent_at_ceiling, test_compute_return_to_point_command_moves_forward_toward_world_x_target, test_compute_return_to_point_command_rotates_world_error_into_body_frame, test_clamp_follow_command_velocity_caps_predicted_control.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import sys; import pytest`
  - local/project: `from drone_control_pkg.follow_utils import FollowCommand, clamp_follow_command_velocity, clamp_follow_command_altitude, compute_return_to_point_command`
- Top-level functions:
  - `make_command(*, vz: float) -> FollowCommand` (lines 16-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: FollowCommand.
  - `test_clamp_follow_command_altitude_limits_descent_at_floor() -> None` (lines 30-41): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: make_command, clamp_follow_command_altitude, pytest.approx.
  - `test_clamp_follow_command_altitude_preserves_safe_command() -> None` (lines 44-55): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: make_command, clamp_follow_command_altitude.
  - `test_clamp_follow_command_altitude_limits_ascent_at_ceiling() -> None` (lines 58-69): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: make_command, clamp_follow_command_altitude, pytest.approx.
  - `test_compute_return_to_point_command_moves_forward_toward_world_x_target() -> None` (lines 72-94): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: compute_return_to_point_command, pytest.approx.
  - `test_compute_return_to_point_command_rotates_world_error_into_body_frame() -> None` (lines 97-119): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: compute_return_to_point_command, pytest.approx.
  - `test_clamp_follow_command_velocity_caps_predicted_control() -> None` (lines 122-145): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: FollowCommand, clamp_follow_command_velocity, pytest.approx.

### `ros2/src/drone_control_pkg/test/test_mavros_velocity_node.py`

- File type: Python source file.
- Tracked size: 779 bytes; 22 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_offboard_velocity_gate_requires_armed_offboard_when_enabled, test_offboard_velocity_gate_can_be_disabled_for_bench_use.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - local/project: `from drone_control_pkg.mavros_velocity_node import offboard_velocity_gate_open`
- Top-level functions:
  - `test_offboard_velocity_gate_requires_armed_offboard_when_enabled()` (lines 4-16): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: offboard_velocity_gate_open.
  - `test_offboard_velocity_gate_can_be_disabled_for_bench_use()` (lines 19-22): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: offboard_velocity_gate_open.

### `ros2/src/drone_control_pkg/test/test_milestone2_demo_logic.py`

- File type: Python source file.
- Tracked size: 5117 bytes; 192 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions make_track, test_select_sequential_target_keeps_active_lock_when_valid, test_select_sequential_target_skips_excluded_and_stale_tracks, test_select_sequential_target_rejects_predicted_by_default, test_select_sequential_target_allows_bounded_predicted_track, test_update_dwell_progress_completes_after_required_duration, test_project_body_velocity_to_world_xy_rotates_body_axes.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import sys; import pytest`
  - local/project: `from drone_control_pkg.follow_utils import TRACK_SOURCE_PREDICTED, TrackSnapshot; from drone_control_pkg.milestone2_demo_logic import project_body_velocity_to_world_xy, select_sequential_target, update_dwell_progress`
- Top-level functions:
  - `make_track(track_id: int, *, x_b_m: float, y_b_m: float = 0.0, z_b_m: float = 0.0, vx_b_mps: float = 0.0, vy_b_mps: float = 0.0, vz_b_mps: float = 0.0, distance_m: float | None = None, confidence: float = 0.9, last_seen_s: float = 10.0, source: int = 0, last_observed_age_s: float = 0.0, position_uncertainty_m: float = 0.0) -> TrackSnapshot` (lines 16-48): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: TrackSnapshot.
  - `test_select_sequential_target_keeps_active_lock_when_valid()` (lines 51-71): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: select_sequential_target, make_track.
  - `test_select_sequential_target_skips_excluded_and_stale_tracks()` (lines 74-95): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: select_sequential_target, make_track.
  - `test_select_sequential_target_rejects_predicted_by_default()` (lines 98-122): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: select_sequential_target, make_track.
  - `test_select_sequential_target_allows_bounded_predicted_track()` (lines 125-153): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: select_sequential_target, make_track.
  - `test_update_dwell_progress_completes_after_required_duration()` (lines 156-179): No docstring; behavior is described from its body and call sites.
    Code map: 8 assertions.
    Notable calls: update_dwell_progress, pytest.approx.
  - `test_project_body_velocity_to_world_xy_rotates_body_axes()` (lines 182-192): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: project_body_velocity_to_world_xy, pytest.approx.

### `ros2/src/drone_control_pkg/test/test_milestone3_state_logic.py`

- File type: Python source file.
- Tracked size: 1707 bytes; 49 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_runtime_tracking_guards_stay_enabled_in_flight_states, test_runtime_tracking_guards_disable_only_for_post_landing_cleanup, test_completion_prefers_return_to_start_when_configured, test_completion_falls_back_to_landing_without_return_target, test_post_return_next_state_honors_land_setting.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import sys`
  - local/project: `from drone_control_pkg.milestone3_state_logic import completion_next_state, post_return_next_state, requires_runtime_tracking_guards`
- Top-level functions:
  - `test_runtime_tracking_guards_stay_enabled_in_flight_states() -> None` (lines 13-16): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: requires_runtime_tracking_guards.
  - `test_runtime_tracking_guards_disable_only_for_post_landing_cleanup() -> None` (lines 19-22): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: requires_runtime_tracking_guards.
  - `test_completion_prefers_return_to_start_when_configured() -> None` (lines 25-33): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: completion_next_state.
  - `test_completion_falls_back_to_landing_without_return_target() -> None` (lines 36-44): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: completion_next_state.
  - `test_post_return_next_state_honors_land_setting() -> None` (lines 47-49): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: post_return_next_state.

### `ros2/src/drone_control_pkg/test/test_minjerk_waypoint_utils.py`

- File type: Python source file.
- Tracked size: 2065 bytes; 87 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_minimum_jerk_blend_has_smooth_endpoints, test_waypoint_yaml_and_segment_sampling, test_yaw_uses_shortest_unwrapped_path.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import math; import sys; import pytest`
  - local/project: `from drone_control_pkg.minjerk_waypoint_utils import TrajectoryPose, build_minjerk_segments, load_waypoint_mission, minimum_jerk_blend, sample_minjerk_segment`
- Top-level functions:
  - `test_minimum_jerk_blend_has_smooth_endpoints() -> None` (lines 18-21): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: minimum_jerk_blend, pytest.approx.
  - `test_waypoint_yaml_and_segment_sampling(tmp_path: Path) -> None` (lines 24-64): No docstring; behavior is described from its body and call sites.
    Code map: 6 assertions.
    Runtime interactions: loads or writes YAML.
    Notable calls: config_path.write_text, load_waypoint_mission, build_minjerk_segments, sample_minjerk_segment, TrajectoryPose, pytest.approx.
  - `test_yaw_uses_shortest_unwrapped_path(tmp_path: Path) -> None` (lines 67-87): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Runtime interactions: loads or writes YAML.
    Notable calls: config_path.write_text, load_waypoint_mission, build_minjerk_segments, TrajectoryPose, math.degrees, pytest.approx, math.radians.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_control_pkg/test/test_of_compare_stats.py`

- File type: Python source file.
- Tracked size: 2288 bytes; 77 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_generate_stats_filters_invalid_rows, test_write_report_creates_summary_files.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import csv; import sys; import pytest`
  - local/project: `from drone_control_pkg.of_compare_stats import generate_stats, write_report`
- Top-level functions:
  - `test_generate_stats_filters_invalid_rows(tmp_path: Path) -> None` (lines 12-62): No docstring; behavior is described from its body and call sites.
    Code map: 1 context managers; 4 assertions.
    Notable calls: generate_stats, csv_path.open, csv.DictWriter, writer.writeheader, writer.writerow, pytest.approx.
  - `test_write_report_creates_summary_files(tmp_path: Path) -> None` (lines 65-77): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: csv_path.write_text, write_report, BinOp.exists.

### `ros2/src/drone_control_pkg/test/test_optical_flow_dead_reckon.py`

- File type: Python source file.
- Tracked size: 3384 bytes; 110 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions make_sample, test_dead_reckon_maps_flow_to_body_xy, test_dead_reckon_rotates_body_motion_with_mocap_yaw, test_dead_reckon_rejects_low_quality_without_xy_integration, test_dead_reckon_uses_raw_flow_when_gyro_terms_are_nan, test_dead_reckon_rejects_nonfinite_flow_without_nan_pose.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import math; import sys; import pytest`
  - local/project: `from drone_control_pkg.optical_flow_dead_reckon import OpticalFlowDeadReckoner, OpticalFlowSample`
- Top-level functions:
  - `make_sample(**overrides) -> OpticalFlowSample` (lines 15-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: values.update, OpticalFlowSample.
  - `test_dead_reckon_maps_flow_to_body_xy() -> None` (lines 30-43): No docstring; behavior is described from its body and call sites.
    Code map: 5 assertions.
    Notable calls: OpticalFlowDeadReckoner, reckoner.reset, reckoner.update, make_sample, pytest.approx.
  - `test_dead_reckon_rotates_body_motion_with_mocap_yaw() -> None` (lines 46-58): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: OpticalFlowDeadReckoner, reckoner.reset, reckoner.update, make_sample, pytest.approx.
  - `test_dead_reckon_rejects_low_quality_without_xy_integration() -> None` (lines 61-73): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: OpticalFlowDeadReckoner, reckoner.reset, reckoner.update, make_sample, pytest.approx.
  - `test_dead_reckon_uses_raw_flow_when_gyro_terms_are_nan() -> None` (lines 76-95): No docstring; behavior is described from its body and call sites.
    Code map: 5 assertions.
    Notable calls: OpticalFlowDeadReckoner, reckoner.reset, reckoner.update, make_sample, pytest.approx.
  - `test_dead_reckon_rejects_nonfinite_flow_without_nan_pose() -> None` (lines 98-110): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: OpticalFlowDeadReckoner, reckoner.reset, reckoner.update, make_sample, pytest.approx.

### `ros2/src/drone_control_pkg/test/test_pep257.py`

- File type: Python source file.
- Tracked size: 802 bytes; 23 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_pep257.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_pep257.main import main; import pytest`
- Top-level functions:
  - `test_pep257()` (lines 21-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main.

### `ros2/src/drone_control_pkg/test/test_speed_profile_configs.py`

- File type: Python source file.
- Tracked size: 2050 bytes; 63 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions _bringup_config_dir, test_indoor_speed_profile_does_not_override_takeoff_speed, test_milestone3_takeoff_baseline_profile_restores_takeoff_speed, test_default_speed_profile_matches_faster_baseline_motion_limits, test_fun_speed_profile_sits_between_indoor_and_default.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path`
  - local/project: `from drone_control_pkg.px4_param_profile import Px4ParamProfile`
- Top-level functions:
  - `_bringup_config_dir() -> Path` (lines 6-8): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Path.resolve, Path.
  - `test_indoor_speed_profile_does_not_override_takeoff_speed()` (lines 11-18): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Runtime interactions: loads or writes YAML.
    Notable calls: Px4ParamProfile.load_from_yaml, _bringup_config_dir.
  - `test_milestone3_takeoff_baseline_profile_restores_takeoff_speed()` (lines 21-28): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Runtime interactions: loads or writes YAML.
    Notable calls: Px4ParamProfile.load_from_yaml, _bringup_config_dir.
  - `test_default_speed_profile_matches_faster_baseline_motion_limits()` (lines 31-38): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Runtime interactions: loads or writes YAML.
    Notable calls: Px4ParamProfile.load_from_yaml, _bringup_config_dir.
  - `test_fun_speed_profile_sits_between_indoor_and_default()` (lines 41-63): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 assertions.
    Runtime interactions: loads or writes YAML.
    Notable calls: Px4ParamProfile.load_from_yaml, _bringup_config_dir.
- Whole-file runtime themes: loads or writes YAML.

## Active ROS 2 package: drone_msgs

### `ros2/src/drone_msgs/CMakeLists.txt`

- File type: CMake build manifest.
- Tracked size: 727 bytes; 28 decoded lines.
- Purpose: CMake/ament build recipe for a ROS package.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: colcon/ament_cmake during build.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 28 total, 23 non-empty.
  - Example: `cmake_minimum_required(VERSION 3.8)`
  - Example: `project(drone_msgs)`
  - Example: `if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")`

### `ros2/src/drone_msgs/msg/EngagementState.msg`

- File type: ROS message definition.
- Tracked size: 401 bytes; 17 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `string scenario_id`
  - `string state`
  - `bool autonomy_enabled`
  - `bool autonomy_ready`
  - `string blocked_reason`
  - `bool estop_latched`
  - `bool obstacle_blocked`
  - `bool min_distance_gate_active`
  - `int32 active_track_id`
  - `float32 active_distance_m`
  - `float32 active_bearing_rad`
  - `bool target_visible`
  - `float32 dwell_remaining_s`
  - `float32 dwell_elapsed_s`
  - `int32 completed_targets_count`
  - `int32 required_targets_count`

### `ros2/src/drone_msgs/msg/FlightStatus.msg`

- File type: ROS message definition.
- Tracked size: 828 bytes; 42 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `string drone_id`
  - `string hostname`
  - `bool connected`
  - `bool armed`
  - `bool guided`
  - `string mode`
  - `bool autonomy_enabled`
  - `bool autonomy_ready`
  - `bool autonomy_blocked`
  - `string autonomy_blocked_reason`
  - `bool estop`
  - `bool obstacle_blocked`
  - `bool min_distance_gate_active`
  - `bool target_visible`
  - `bool watchdog_healthy`
  - `float32 altitude_m`
  - `float32 x_m`
  - `float32 y_m`
  - `float32 z_m`
  - `float32 roll_rad`
  - `float32 pitch_rad`
  - `float32 yaw_rad`
  - `float32 vx_mps`
  - `float32 vy_mps`
  - `float32 vz_mps`
  - `float32 cmd_vx_mps`
  - `float32 cmd_vy_mps`
  - `float32 cmd_vz_mps`
  - `float32 cmd_yaw_rate_rps`
  - `string behavior_state`
  - `int32 active_track_id`
  - `float32 target_distance_m`
  - `float32 target_bearing_rad`
  - `float32 battery_voltage_v`
  - `float32 battery_remaining_pct`
  - `float32 vio_age_s`
  - `float32 tracks_age_s`
  - `float32 mavros_age_s`
  - `float32 cmd_age_s`
  - `float32 perception_fps`
  - `float32 inference_latency_ms`

### `ros2/src/drone_msgs/msg/LightCommand.msg`

- File type: ROS message definition.
- Tracked size: 86 bytes; 4 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `bool enabled`
  - `float32 intensity_0_to_1`
  - `float32 strobe_hz`

### `ros2/src/drone_msgs/msg/PeerState.msg`

- File type: ROS message definition.
- Tracked size: 218 bytes; 14 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `string drone_id`
  - `string hostname`
  - `string self_ip`
  - `bool connected`
  - `bool armed`
  - `bool autonomy_enabled`
  - `string mode`
  - `float32 x_m`
  - `float32 y_m`
  - `float32 z_m`
  - `float32 vx_mps`
  - `float32 vy_mps`
  - `float32 vz_mps`

### `ros2/src/drone_msgs/msg/PerceptionStatus.msg`

- File type: ROS message definition.
- Tracked size: 184 bytes; 8 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `string drone_id`
  - `float32 tracker_fps`
  - `float32 inference_latency_ms`
  - `int32 left_detections`
  - `int32 right_detections`
  - `int32 paired_detections`
  - `int32 active_tracks`

### `ros2/src/drone_msgs/msg/SystemAlert.msg`

- File type: ROS message definition.
- Tracked size: 192 bytes; 11 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `uint8 SEVERITY_INFO=0`
  - `uint8 SEVERITY_WARN=1`
  - `uint8 SEVERITY_ERROR=2`
  - `uint8 SEVERITY_FATAL=3`
  - `builtin_interfaces/Time stamp`
  - `string drone_id`
  - `uint8 severity`
  - `string code`
  - `string message`
  - `bool latched`

### `ros2/src/drone_msgs/msg/TargetTrack.msg`

- File type: ROS message definition.
- Tracked size: 437 bytes; 21 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `uint8 SOURCE_DETECTED=0`
  - `uint8 SOURCE_HELD=1`
  - `uint8 SOURCE_PREDICTED=2`
  - `int32 track_id`
  - `int32 detector_track_id`
  - `uint8 source`
  - `float32 x_b_m`
  - `float32 y_b_m`
  - `float32 z_b_m`
  - `float32 vx_b_mps`
  - `float32 vy_b_mps`
  - `float32 vz_b_mps`
  - `float32 distance_m`
  - `float32 confidence`
  - `float32 bbox_area_px`
  - `bool inbound`
  - `float32 last_observed_age_s`
  - `float32 prediction_horizon_s`
  - `float32 position_uncertainty_m`
  - `float32 velocity_uncertainty_mps`

### `ros2/src/drone_msgs/msg/TargetTrackArray.msg`

- File type: ROS message definition.
- Tracked size: 51 bytes; 2 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `TargetTrack[] tracks`

### `ros2/src/drone_msgs/msg/WorldTargetTrack.msg`

- File type: ROS message definition.
- Tracked size: 425 bytes; 21 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `uint8 SOURCE_DETECTED=0`
  - `uint8 SOURCE_HELD=1`
  - `uint8 SOURCE_PREDICTED=2`
  - `int32 track_id`
  - `int32 detector_track_id`
  - `uint8 source`
  - `float32 x_m`
  - `float32 y_m`
  - `float32 z_m`
  - `float32 vx_mps`
  - `float32 vy_mps`
  - `float32 vz_mps`
  - `float32 distance_m`
  - `float32 confidence`
  - `float32 bbox_area_px`
  - `bool inbound`
  - `float32 last_observed_age_s`
  - `float32 prediction_horizon_s`
  - `float32 position_uncertainty_m`
  - `float32 velocity_uncertainty_mps`

### `ros2/src/drone_msgs/msg/WorldTargetTrackArray.msg`

- File type: ROS message definition.
- Tracked size: 72 bytes; 3 decoded lines.
- Purpose: Custom ROS interface message used to type perception, status, command, or coordination topics.
- Main responsibilities: Defines public fields for strongly typed ROS message exchange.
- Important dependencies or consumers: rosidl code generation plus Python/C++ nodes that publish or subscribe to the generated message types.
- When to touch it: Touch this only when changing public ROS message contracts; rebuild dependent packages afterward.
- Message fields:
  - `builtin_interfaces/Time stamp`
  - `string frame_id`
  - `WorldTargetTrack[] tracks`

### `ros2/src/drone_msgs/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 862 bytes; 25 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_msgs`.
- Dependencies: `buildtool_depend:ament_cmake, buildtool_depend:rosidl_default_generators, depend:builtin_interfaces, exec_depend:rosidl_default_runtime, test_depend:ament_lint_auto, test_depend:ament_lint_common`.
- Build type: `ament_cmake`.

## Active ROS 2 package: drone_vision_pkg

### `ros2/src/drone_vision_pkg/config/target_map.yaml`

- File type: YAML configuration file.
- Tracked size: 840 bytes; 25 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 23 documented key/value entries.
- Key map:
  - `target_map_node`: mapping
  - `target_map_node.ros__parameters`: mapping
  - `target_map_node.ros__parameters.publish_rate_hz`: `10.0`
  - `target_map_node.ros__parameters.world_frame`: `map`
  - `target_map_node.ros__parameters.world_tracks_topic`: ``
  - `target_map_node.ros__parameters.ownship_pose_topic`: ``
  - `target_map_node.ros__parameters.target_map_world_topic`: ``
  - `target_map_node.ros__parameters.target_map_tracks_topic`: ``
  - `target_map_node.ros__parameters.target_map_markers_topic`: ``
  - `target_map_node.ros__parameters.association_gate_m`: `0.8`
  - `target_map_node.ros__parameters.association_uncertainty_scale`: `1.0`
  - `target_map_node.ros__parameters.max_prediction_horizon_s`: `3.0`
  - `target_map_node.ros__parameters.prune_after_s`: `5.0`
  - `target_map_node.ros__parameters.observation_fresh_s`: `0.25`
  - `target_map_node.ros__parameters.confidence_decay_per_s`: `0.25`
  - `target_map_node.ros__parameters.observed_position_uncertainty_m`: `0.15`
  - `target_map_node.ros__parameters.observed_velocity_uncertainty_mps`: `0.35`
  - `target_map_node.ros__parameters.position_process_noise_mps`: `0.35`
  - `target_map_node.ros__parameters.velocity_process_noise_mps`: `0.25`
  - `target_map_node.ros__parameters.velocity_blend_alpha`: `0.55`
  - `target_map_node.ros__parameters.max_velocity_mps`: `8.0`
  - `target_map_node.ros__parameters.history_length`: `30`
  - `target_map_node.ros__parameters.marker_lifetime_s`: `0.5`

### `ros2/src/drone_vision_pkg/config/tracking_only.yaml`

- File type: YAML configuration file.
- Tracked size: 2063 bytes; 58 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 62 documented key/value entries.
- Key map:
  - `realsense_tracker_node`: mapping
  - `realsense_tracker_node.ros__parameters`: mapping
  - `realsense_tracker_node.ros__parameters.source_mode`: `direct`
  - `realsense_tracker_node.ros__parameters.tracker_rate_hz`: `10.0`
  - `realsense_tracker_node.ros__parameters.device_id`: `0`
  - `realsense_tracker_node.ros__parameters.model_path`: ``
  - `realsense_tracker_node.ros__parameters.model_input_size`: `640`
  - `realsense_tracker_node.ros__parameters.detection_conf_threshold`: `0.4`
  - `realsense_tracker_node.ros__parameters.norfair_distance_threshold`: `80.0`
  - `realsense_tracker_node.ros__parameters.track_hit_counter_max`: `12`
  - `realsense_tracker_node.ros__parameters.max_valid_depth_m`: `20.0`
  - `realsense_tracker_node.ros__parameters.depth_window_px`: `5`
  - `realsense_tracker_node.ros__parameters.publish_world_tracks`: `True`
  - `realsense_tracker_node.ros__parameters.publish_world_track_compare`: `True`
  - `realsense_tracker_node.ros__parameters.save_world_track_compare_frames`: `True`
  - `realsense_tracker_node.ros__parameters.save_world_track_compare_csv`: `True`
  - `realsense_tracker_node.ros__parameters.save_world_track_compare_frame_csv`: `True`
  - `realsense_tracker_node.ros__parameters.world_track_compare_frame_dump_interval_s`: `3.0`
  - `realsense_tracker_node.ros__parameters.world_track_compare_csv_dump_interval_s`: `3.0`
  - `realsense_tracker_node.ros__parameters.world_track_compare_frame_dump_dir`: `/home/jetson/output_dump`
  - `realsense_tracker_node.ros__parameters.world_track_compare_csv_filename`: `world_track_compare_tracks.csv`
  - `realsense_tracker_node.ros__parameters.experiment_tag`: ``
  - `realsense_tracker_node.ros__parameters.world_frame`: `map`
  - `realsense_tracker_node.ros__parameters.camera_offset_body_m`: list[3]
  - `realsense_tracker_node.ros__parameters.camera_offset_body_m[0]`: `0.0`
  - `realsense_tracker_node.ros__parameters.camera_offset_body_m[1]`: `0.0`
  - `realsense_tracker_node.ros__parameters.camera_offset_body_m[2]`: `0.0`
  - `realsense_tracker_node.ros__parameters.camera_rpy_body_rad`: list[3]
  - `realsense_tracker_node.ros__parameters.camera_rpy_body_rad[0]`: `0.0`
  - `realsense_tracker_node.ros__parameters.camera_rpy_body_rad[1]`: `0.0`
  - `realsense_tracker_node.ros__parameters.camera_rpy_body_rad[2]`: `0.0`
  - `realsense_tracker_node.ros__parameters.align_depth_to_color`: `True`
  - `realsense_tracker_node.ros__parameters.direct_color_width`: `848`
  - `realsense_tracker_node.ros__parameters.direct_color_height`: `480`
  - `realsense_tracker_node.ros__parameters.direct_color_fps`: `15`
  - `realsense_tracker_node.ros__parameters.direct_depth_width`: `848`
  - `realsense_tracker_node.ros__parameters.direct_depth_height`: `480`
  - `realsense_tracker_node.ros__parameters.direct_depth_fps`: `15`
  - `realsense_tracker_node.ros__parameters.record_mission_video`: `True`
  - `realsense_tracker_node.ros__parameters.recording_output_dir`: `/home/jetson/cdrone_control/mission_recordings`
  - `realsense_tracker_node.ros__parameters.recording_fps`: `10.0`
  - `realsense_tracker_node.ros__parameters.recording_width`: `1280`
  - `realsense_tracker_node.ros__parameters.recording_height`: `720`
  - `realsense_tracker_node.ros__parameters.recording_fourcc`: `mp4v`
  - `realsense_tracker_node.ros__parameters.recording_depth_max_m`: `6.0`
  - `realsense_tracker_node.ros__parameters.recording_save_depth_video`: `True`
  - `realsense_tracker_node.ros__parameters.recording_replace_depth_tile_with_yolo`: `False`
  - `realsense_tracker_node.ros__parameters.recording_state_topic`: ``
  - `realsense_tracker_node.ros__parameters.recording_mavros_state_topic`: ``
  - `realsense_tracker_node.ros__parameters.recording_local_pose_topic`: ``
  - `realsense_tracker_node.ros__parameters.recording_flow_range_topic`: ``
  - `realsense_tracker_node.ros__parameters.recording_target_map_world_topic`: ``
  - `realsense_tracker_node.ros__parameters.recording_target_map_history_s`: `3.0`
  - `realsense_tracker_node.ros__parameters.color_topic`: `/camera/color/image_raw`
  - `realsense_tracker_node.ros__parameters.depth_topic`: `/camera/aligned_depth_to_color/image_raw`
  - `realsense_tracker_node.ros__parameters.camera_info_topic`: `/camera/color/camera_info`
  - `realsense_tracker_node.ros__parameters.pose_topic`: ``
  - `realsense_tracker_node.ros__parameters.compare_pose_topic`: ``
  - `realsense_tracker_node.ros__parameters.tracks_topic`: ``
  - `realsense_tracker_node.ros__parameters.world_tracks_topic`: ``
  - `realsense_tracker_node.ros__parameters.world_track_compare_topic`: ``
  - `realsense_tracker_node.ros__parameters.perception_status_topic`: ``

### `ros2/src/drone_vision_pkg/drone_vision_pkg/__init__.py`

- File type: Python source file.
- Tracked size: 60 bytes; 1 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Tracking-first perception package for cdrone_control.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/model_paths.py`

- File type: Python source file.
- Tracked size: 2365 bytes; 78 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines functions _candidate_roots, _candidate_model_paths, _repo_root, resolve_model_path.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import os; from pathlib import Path`
- Top-level constants/state: `DEFAULT_MODEL_CANDIDATES`.
- Top-level functions:
  - `_candidate_roots() -> tuple[Path, ...]` (lines 15-25): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _repo_root, os.environ.get.strip, Path, os.environ.get.
  - `_candidate_model_paths() -> tuple[Path, ...]` (lines 28-40): No docstring; behavior is described from its body and call sites.
    Code map: 3 for loops; 1 return points.
    Notable calls: _repo_root, _candidate_roots, candidates.append.
  - `_repo_root() -> Path` (lines 43-50): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 2 return points.
    Notable calls: Path.resolve, Path, BinOp.is_dir.
  - `resolve_model_path(requested_path: str) -> str` (lines 53-78): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 1 for loops; 3 return points; 3 raise statements.
    Notable calls: str.strip, os.environ.get.strip, _candidate_model_paths, FileNotFoundError, Path.expanduser, path.exists, candidate.exists, os.environ.get, Path.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/projection.py`

- File type: Python source file.
- Tracked size: 6596 bytes; 208 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes CameraIntrinsics, RealsenseProjection; defines functions _normalize_quaternion, _quaternion_to_rotation_matrix, _rotation_matrix_from_rpy.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
  - third-party: `import numpy as np`
  - ROS/runtime: `from geometry_msgs.msg import PoseStamped; from sensor_msgs.msg import CameraInfo`
- Top-level constants/state: `OPTICAL_TO_FLU`.
- Classes:
  - `CameraIntrinsics` (lines 21-25, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `RealsenseProjection` (lines 90-208, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `world_frame, pose_frame_id, intrinsics, drone_position_m, world_from_body_rotation, camera_offset_body_m, camera_to_body_rotation, set_intrinsics, frame_id, pixel_depth_to_camera_frame, camera_to_body_frame, pixel_depth_to_body, body_to_world, world_to_body, body_to_camera_frame, has_pose`.
    - `__init__(self, camera_offset_body_m: list[float], camera_rpy_body_rad: list[float], world_frame: str) -> None` (lines 91-104): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: str.strip, np.asarray, _rotation_matrix_from_rpy.
    - `frame_id(self) -> str` (lines 107-108): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `has_intrinsics(self) -> bool` (lines 110-111): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `has_pose(self) -> bool` (lines 113-117): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `set_intrinsics(self, fx: float, fy: float, cx: float, cy: float) -> None` (lines 119-127): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: CameraIntrinsics.
    - `update_camera_info(self, msg: CameraInfo) -> None` (lines 129-134): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.set_intrinsics.
    - `update_pose(self, msg: PoseStamped) -> None` (lines 136-147): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages, uses NumPy arrays/math.
      Notable calls: np.array, _quaternion_to_rotation_matrix, str.strip.
    - `pixel_depth_to_camera_frame(self, x_px: float, y_px: float, depth_m: float) -> np.ndarray | None` (lines 149-160): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: np.array.
    - `camera_to_body_frame(self, camera_point_m: np.ndarray) -> np.ndarray` (lines 162-163): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: uses NumPy arrays/math.
    - `body_to_camera_frame(self, body_point_m: np.ndarray) -> np.ndarray` (lines 165-168): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: np.asarray.
    - `pixel_depth_to_body(self, x_px: float, y_px: float, depth_m: float) -> np.ndarray | None` (lines 170-179): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: self.pixel_depth_to_camera_frame, self.camera_to_body_frame.
    - `pixel_depth_to_world(self, x_px: float, y_px: float, depth_m: float) -> np.ndarray | None` (lines 181-190): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: self.pixel_depth_to_body, self.body_to_world, self.has_pose.
    - `body_to_world(self, body_point_m: np.ndarray) -> np.ndarray | None` (lines 192-195): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: self.has_pose.
    - `world_to_body(self, world_point_m: np.ndarray) -> np.ndarray | None` (lines 197-202): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: np.asarray, self.has_pose.
    - `world_to_camera(self, world_point_m: np.ndarray) -> np.ndarray | None` (lines 204-208): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: self.world_to_body, self.body_to_camera_frame.
- Top-level functions:
  - `_normalize_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]` (lines 28-37): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.sqrt.
  - `_quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray` (lines 40-63): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: _normalize_quaternion, np.array.
  - `_rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray` (lines 66-87): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.array, np.cos, np.sin.
- Whole-file runtime themes: handles pose messages, uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/realsense_tracker_node.py`

- File type: Python source file.
- Tracked size: 134335 bytes; 3396 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes TrackVelocity, RealsenseTrackerNode; defines functions led_color_name_for_engagement_state, led_bgr_for_color_name, is_active_mission_state, make_depth_colormap, get_centroid, map_point_between_frames, compute_mean_depth, bbox_to_dict, point_to_dict, point3_to_dict and more; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import csv; from dataclasses import dataclass; import json; import math; from pathlib import Path; import time`
  - third-party: `import numpy as np`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from mavros_msgs.msg import State; from rclpy.node import Node; from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data; from sensor_msgs.msg import CameraInfo, Image, Range; from std_msgs.msg import String`
  - local/project: `from drone_control_pkg.deployment_config import configured_compare_pose_topic, configured_drone_id, configured_mavros_namespace; from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic, join_topic; from drone_msgs.msg import EngagementState, PerceptionStatus, TargetTrack, TargetTrackArray, WorldTargetTrack, WorldTargetTrackArray; from drone_vision_pkg.model_paths import resolve_model_path; from drone_vision_pkg.projection import RealsenseProjection; from drone_vision...`
- Top-level constants/state: `STARTUP_LED_STATES, RETURN_LED_STATES, LANDING_LED_STATES, MISSION_TERMINAL_STATES, LED_RGB_BY_NAME, TARGET_TRACK_SOURCE_DETECTED, TARGET_TRACK_SOURCE_HELD, TARGET_TRACK_SOURCE_PREDICTED, TARGET_MAP_SOURCE_LABELS, TARGET_MAP_SOURCE_BGR`.
- Classes:
  - `TrackVelocity` (lines 94-96, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `RealsenseTrackerNode` (lines 354-3383, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, source_mode, tracker_rate_hz, device_id, model_input_size, conf_threshold, norfair_distance_threshold, track_hit_counter_max, enable_reid, reid_histogram_bins, reid_distance_threshold, reid_hit_counter_max, enable_motion_estimator, publish_track_hold_s, publish_track_hold_max_extrapolation_m, max_valid_depth_m, depth_window_px, publish_world_tracks, publish_world_track_compare, save_world_track_compare_frames, save_world_track_compare_csv, save_world_track_compare_frame_csv, world_track_compare_frame_dump_interval_s, world_track_compare_csv_dump_interval_s, world_track_compare_root_dir, world_track_compare_tracks_csv_filename, experiment_tag, run_started_wall_time_s, world_track...` plus more.
    - `__init__(self) -> None` (lines 355-848): No docstring; behavior is described from its body and call sites.
      Code map: 12 conditional branches; 4 raise statements.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages, handles target track messages, handles world-frame target tracks, uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: super.__init__, self.declare_parameter, str.strip.lower, Path, sanitize_experiment_tag, time.time, make_run_id, str.strip, resolve_model_path, RealsenseProjection, YOLO, self._build_tracker, EngagementState, State, QoSProfile, self.create_publisher, self.create_timer, configured_drone_id....
    - `_build_tracker(self) -> Tracker` (lines 850-876): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 try/except blocks; 2 return points.
      Notable calls: tracker_kwargs.update, Tracker, tracker_kwargs.pop.
    - `now_s(self) -> float` (lines 878-879): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `destroy_node(self) -> bool` (lines 881-895): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 try/except blocks; 1 return points.
      Notable calls: self._stop_mission_recording, self._flush_world_track_compare_csv, self._write_world_track_compare_manifest, super.destroy_node, self.rs_pipeline.stop.
    - `color_callback(self, msg: Image) -> None` (lines 897-898): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `depth_callback(self, msg: Image) -> None` (lines 900-901): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `camera_info_callback(self, msg: CameraInfo) -> None` (lines 903-904): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.projector.update_camera_info.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 906-907): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.projector.update_pose.
    - `compare_pose_callback(self, msg: PoseStamped) -> None` (lines 909-910): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
    - `engagement_state_callback(self, msg: EngagementState) -> None` (lines 912-918): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.now_s, is_active_mission_state, self._start_mission_recording, self._stop_mission_recording.
    - `mavros_state_callback(self, msg: State) -> None` (lines 920-922): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `local_pose_callback(self, msg: PoseStamped) -> None` (lines 924-942): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: handles pose messages, uses NumPy arrays/math.
      Notable calls: self.now_s, self.mission_ownship_history.append, np.array.
    - `flow_range_callback(self, msg: Range) -> None` (lines 944-946): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `target_map_world_callback(self, msg: WorldTargetTrackArray) -> None` (lines 948-990): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 for loops.
      Runtime interactions: handles target track messages, handles world-frame target tracks, uses NumPy arrays/math.
      Notable calls: self.now_s, seen_ids.add, self.target_map_world_history.setdefault, history.append, self.target_map_world_history.keys, self.mission_target_map_world_history.setdefault, mission_history.append, np.array, self.target_map_world_history.pop.
    - `_mission_recording_output_size(self) -> tuple[int, int]` (lines 992-999): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
    - `_start_mission_recording(self, state: str) -> None` (lines 1001-1084): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 2 for loops; 1 try/except blocks; 4 return points.
      Runtime interactions: uses OpenCV image processing.
      Notable calls: self._mission_recording_output_size, time.time, time.strftime, ''.join.strip, BoolOp.ljust, cv2.VideoWriter_fourcc, stream_specs.items, output_dir.mkdir, time.localtime, cv2.VideoWriter, ''.join, writer.isOpened, writer.release, writers.values, ', '.join, opened_writer.release, ch.isalnum, ch.lower....
    - `_stop_mission_recording(self, reason: str) -> None` (lines 1086-1104): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 1 return points.
      Notable calls: writers.values, writer.release, ', '.join, output_paths.items.
    - `_maybe_record_mission_frame(self, *, frame: np.ndarray, depth_frame: np.ndarray | None, body_tracks: list[TargetTrack], body_track_debug: dict[int, dict[str, object]], detections_count: int, active_tracks_count: int, tracker_fps: float, now_s: float) -> None` (lines 1106-1147): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 1 for loops; 3 return points.
      Runtime interactions: handles target track messages, uses NumPy arrays/math.
      Notable calls: is_active_mission_state, self._compose_mission_recording_frames, output_frames.items, self._start_mission_recording, writers.get, writer.write.
    - `_compose_mission_recording_frames(self, *, frame: np.ndarray, depth_frame: np.ndarray | None, body_tracks: list[TargetTrack], body_track_debug: dict[int, dict[str, object]], detections_count: int, active_tracks_count: int, tracker_fps: float) -> dict[str, np.ndarray]` (lines 1149-1232): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: handles target track messages, uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: self._mission_recording_output_size, cv2.resize, make_depth_colormap, self._make_stats_panel, self._make_target_map_recording_frame, self._draw_tile_label, np.vstack, self._make_yolo_overlay, np.hstack.
    - `_make_yolo_overlay(self, frame: np.ndarray, body_track_debug: dict[int, dict[str, object]], *, include_target_map: bool) -> np.ndarray` (lines 1234-1271): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 for loops; 1 return points.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: frame.copy, body_track_debug.items, debug_info.get, cv2.rectangle, cv2.putText, self._draw_target_map_video_overlay, isinstance, label_parts.append, ' '.join, yolo_bbox.get.
    - `_project_world_to_pixel(self, world_point_m: np.ndarray, *, image_shape: tuple[int, ...]) -> tuple[int, int, float] | None` (lines 1273-1303): No docstring; behavior is described from its body and call sites.
      Code map: 6 conditional branches; 7 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: self.projector.world_to_camera, self.projector.has_pose, self.projector.has_intrinsics, np.isfinite.
    - `_draw_target_map_video_overlay(self, image: np.ndarray) -> None` (lines 1305-1410): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 2 for loops; 2 return points.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: self.now_s, cv2.rectangle, cv2.putText, TARGET_MAP_SOURCE_BGR.get, np.array, self._project_world_to_pixel, self._draw_target_map_history, self._uncertainty_radius_px, cv2.circle, TARGET_MAP_SOURCE_LABELS.get, getattr, cv2.arrowedLine.
    - `_uncertainty_radius_px(self, uncertainty_m: float, *, depth_z_m: float) -> int` (lines 1412-1419): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
    - `_draw_target_map_history(self, image: np.ndarray, *, track_id: int, color: tuple[int, int, int]) -> None` (lines 1421-1444): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 for loops; 1 return points.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: self.target_map_world_history.get, self._project_world_to_pixel, points.append, cv2.line.
    - `_make_target_map_recording_frame(self, *, size: tuple[int, int]) -> np.ndarray` (lines 1446-1475): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: np.full, self._draw_tile_label, self._recording_map_points, self._recording_map_bounds, self._draw_recording_map_grid, self._draw_recording_map_ownship, self._draw_recording_map_targets, self._draw_recording_map_legend, cv2.putText.
    - `_recording_map_points(self) -> list[np.ndarray]` (lines 1477-1497): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 for loops; 1 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: points.extend, self.mission_target_map_world_history.values, points.append, np.array, np.all, np.isfinite.
    - `_recording_map_bounds(self, points: list[np.ndarray]) -> tuple[float, float, float, float]` (lines 1499-1510): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: uses NumPy arrays/math.
    - `_map_world_to_pixel(self, point_m: np.ndarray, *, map_rect: tuple[int, int, int, int], bounds: tuple[float, float, float, float]) -> tuple[int, int]` (lines 1512-1530): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: uses NumPy arrays/math.
    - `_draw_recording_map_grid(self, image: np.ndarray, *, map_rect: tuple[int, int, int, int], bounds: tuple[float, float, float, float]) -> None` (lines 1532-1589): No docstring; behavior is described from its body and call sites.
      Code map: 2 while loops.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: cv2.rectangle, cv2.putText, math.floor, self._map_world_to_pixel, cv2.line, np.array.
    - `_draw_recording_map_ownship(self, image: np.ndarray, *, map_rect: tuple[int, int, int, int], bounds: tuple[float, float, float, float]) -> None` (lines 1591-1636): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 for loops; 1 return points.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: np.array, self._map_world_to_pixel, cv2.circle, cv2.putText, cv2.line.
    - `_draw_recording_map_targets(self, image: np.ndarray, *, map_rect: tuple[int, int, int, int], bounds: tuple[float, float, float, float]) -> None` (lines 1638-1702): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 for loops.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: sorted, self.mission_target_map_world_history.items, source_by_track_id.get, TARGET_MAP_SOURCE_BGR.get, np.array, self._map_world_to_pixel, cv2.circle, cv2.putText, getattr, cv2.line, self._map_uncertainty_radius_px, TARGET_MAP_SOURCE_LABELS.get.
    - `_map_uncertainty_radius_px(self, uncertainty_m: float, *, map_rect: tuple[int, int, int, int], bounds: tuple[float, float, float, float]) -> int` (lines 1704-1716): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `_draw_recording_map_legend(self, image: np.ndarray, *, map_rect: tuple[int, int, int, int]) -> None` (lines 1718-1759): No docstring; behavior is described from its body and call sites.
      Code map: 1 for loops.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: cv2.rectangle, cv2.circle, cv2.putText.
    - `_draw_tile_label(self, image: np.ndarray, label: str, *, accent_bgr: tuple[int, int, int] | None = None) -> None` (lines 1761-1780): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: cv2.rectangle, cv2.putText.
    - `_make_stats_panel(self, *, size: tuple[int, int], body_tracks: list[TargetTrack], detections_count: int, active_tracks_count: int, tracker_fps: float) -> np.ndarray` (lines 1782-1837): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 1 return points.
      Runtime interactions: handles target track messages, uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: np.full, led_color_name_for_engagement_state, led_bgr_for_color_name, self._draw_tile_label, cv2.circle, self._mission_stats_lines, cv2.putText.
    - `_mission_stats_lines(self, *, body_tracks: list[TargetTrack], detections_count: int, active_tracks_count: int, tracker_fps: float, led_color_name: str) -> list[tuple[str, str, tuple[int, int, int]]]` (lines 1839-1926): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Runtime interactions: handles target track messages.
      Notable calls: self.now_s, self._select_stats_track, lines.extend, lines.append, led_bgr_for_color_name, self._format_target_stats, self._format_target_map_recording_stats, self._format_pose_stats, self._format_flow_range_stats.
    - `_format_target_map_recording_stats(self, *, now_s: float) -> str` (lines 1928-1945): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: getattr.
    - `_select_stats_track(self, body_tracks: list[TargetTrack]) -> TargetTrack | None` (lines 1947-1955): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 for loops; 3 return points.
      Runtime interactions: handles target track messages.
    - `_format_target_stats(self, track: TargetTrack | None, engagement: EngagementState) -> str` (lines 1957-1972): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Runtime interactions: handles target track messages.
    - `_format_pose_stats(self, pose: PoseStamped | None, *, now_s: float) -> str` (lines 1974-1982): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Runtime interactions: handles pose messages.
    - `_format_flow_range_stats(self, flow_range: Range | None, *, now_s: float) -> str` (lines 1984-1988): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `process_latest_frame(self) -> None` (lines 1990-2119): No docstring; behavior is described from its body and call sites.
      Code map: 9 conditional branches; 4 return points.
      Runtime interactions: publishes messages, handles target track messages, handles world-frame target tracks, uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: time.perf_counter, self.model.predict, yolo_boxes_to_detections, self.now_s, self._build_body_tracks, self._build_world_tracks, TargetTrackArray, self.tracks_pub.publish, WorldTargetTrackArray, self.world_tracks_pub.publish, self._publish_world_track_compare, self._maybe_dump_world_track_compare_frame, self._maybe_append_world_track_compare_tracks_csv, self._maybe_append_world_track_compare_frames_csv, PerceptionStatus, self.status_pub.publish, self._maybe_record_mission_frame, self._read_direct_frame....
    - `_start_direct_pipeline(self) -> None` (lines 2121-2164): No docstring; behavior is described from its body and call sites.
      Code map: 1 try/except blocks.
      Notable calls: rs.config, config.enable_stream, rs.pipeline, self.rs_pipeline.start, profile.get_stream.as_video_stream_profile, color_profile.get_intrinsics, self.projector.set_intrinsics, rs.align, profile.get_device.first_depth_sensor, depth_sensor.get_depth_scale, profile.get_stream, profile.get_device.
    - `_read_direct_frame(self) -> tuple[np.ndarray, np.ndarray | None, tuple[int, int]] | None` (lines 2166-2202): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 try/except blocks; 4 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: frames.get_color_frame, frames.get_depth_frame, np.asanyarray, self.rs_pipeline.wait_for_frames, self.rs_align.process, self._warn_throttled, color_frame.get_data, color_frame.get_frame_number, np.asanyarray.astype, depth_frame.get_data.
    - `_build_body_tracks(self, tracked_objects: list, frame: np.ndarray, depth_frame: np.ndarray | None, stamp_msg, now_s: float) -> tuple[list[TargetTrack], dict[int, dict[str, object]]]` (lines 2204-2325): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 1 for loops; 2 return points.
      Runtime interactions: handles target track messages, uses NumPy arrays/math.
      Notable calls: self._cache_fresh_body_tracks, self._append_held_body_tracks, self._garbage_collect_history, self.projector.has_intrinsics, self._warn_throttled, getattr, norfair_active_track_ids.add, np.asarray, get_centroid, map_point_between_frames, compute_mean_depth, self.projector.pixel_depth_to_body, self._estimate_velocity, detection.data.get, TargetTrack, track_messages.append, np.linalg.norm, point3_to_dict....
    - `_message_to_body_track_state(self, track: TargetTrack, *, now_s: float) -> BodyTrackState` (lines 2327-2357): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles target track messages.
      Notable calls: BodyTrackState, getattr.
    - `_body_track_state_to_msg(self, state: BodyTrackState, stamp_msg) -> TargetTrack` (lines 2359-2381): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles target track messages, uses NumPy arrays/math.
      Notable calls: TargetTrack, np.linalg.norm, np.array.
    - `_cache_fresh_body_tracks(self, body_tracks: list[TargetTrack], body_track_debug: dict[int, dict[str, object]], *, now_s: float) -> None` (lines 2383-2412): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 for loops.
      Runtime interactions: handles target track messages.
      Notable calls: fresh_track_ids.add, self._message_to_body_track_state, self.cached_body_tracks.keys, body_track_debug.get, self.cached_body_tracks.pop, self.cached_body_track_debug.pop.
    - `_append_held_body_tracks(self, *, body_tracks: list[TargetTrack], body_track_debug: dict[int, dict[str, object]], active_track_ids: set[int], stamp_msg, now_s: float) -> tuple[list[TargetTrack], dict[int, dict[str, object]]]` (lines 2414-2442): No docstring; behavior is described from its body and call sites.
      Code map: 1 for loops; 1 return points.
      Runtime interactions: handles target track messages.
      Notable calls: select_held_track_states, held_tracks.items, body_tracks.append, self._body_track_state_to_msg, self.cached_body_track_debug.get.
    - `_build_world_tracks(self, body_tracks: list[TargetTrack], stamp_msg, now_s: float) -> list[WorldTargetTrack]` (lines 2444-2513): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 for loops; 3 return points.
      Runtime interactions: handles target track messages, handles world-frame target tracks, uses NumPy arrays/math.
      Notable calls: self._garbage_collect_history, self.projector.has_pose, np.array, self.projector.body_to_world, self._estimate_velocity, WorldTargetTrack, world_tracks.append, active_track_ids.add, self._warn_throttled, getattr.
    - `_estimate_velocity(self, track_id: int, point_m: np.ndarray, now_s: float, history: dict[int, tuple[np.ndarray, float]]) -> TrackVelocity` (lines 2515-2533): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: history.get, np.zeros, TrackVelocity, point_m.copy, np.linalg.norm, np.dot.
    - `_garbage_collect_history(self, history: dict[int, tuple[np.ndarray, float]], active_track_ids: set[int]) -> None` (lines 2535-2544): No docstring; behavior is described from its body and call sites.
      Code map: 1 for loops.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: history.pop.
    - `_warn_throttled(self, message: str, attr_name: str) -> None` (lines 2546-2552): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s, setattr, getattr.
    - `_ensure_output_dump_dir(self) -> None` (lines 2554-2565): No docstring; behavior is described from its body and call sites.
      Code map: 1 try/except blocks.
      Notable calls: self.world_track_compare_root_dir.mkdir, self.world_track_compare_run_dir.mkdir.
    - `_ensure_world_track_compare_csv_ready(self, *, path: Path, header: list[str], ready_attr: str, enabled_attr: str, log_label: str) -> bool` (lines 2567-2594): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 try/except blocks; 1 context managers; 4 return points.
      Notable calls: getattr, path.exists, setattr, path.open, csv.writer, writer.writerow, path.stat.
    - `_ensure_world_track_compare_tracks_csv_ready(self) -> bool` (lines 2596-2603): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self._ensure_world_track_compare_csv_ready.
    - `_ensure_world_track_compare_frames_csv_ready(self) -> bool` (lines 2605-2612): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self._ensure_world_track_compare_csv_ready.
    - `_write_world_track_compare_manifest(self, completed: bool = False) -> None` (lines 2614-2712): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 try/except blocks; 1 context managers; 3 return points.
      Notable calls: self.world_track_compare_run_dir.exists, time.time, isoformat_wall_time, self.world_track_compare_manifest_path.open, json.dump.
    - `_controller_track_sample(self, track: TargetTrack) -> ControllerTrackSample` (lines 2714-2729): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles target track messages.
      Notable calls: ControllerTrackSample.
    - `_publish_world_track_compare(self, world_tracks: list[WorldTargetTrack], body_tracks: list[TargetTrack], stamp_msg, body_track_debug: dict[int, dict[str, object]], detections_count: int, active_tracks_count: int, tracker_fps: float) -> dict[str, object] | None` (lines 2731-2893): No docstring; behavior is described from its body and call sites.
      Code map: 7 conditional branches; 1 for loops; 3 return points.
      Runtime interactions: publishes messages, handles target track messages, handles world-frame target tracks, uses NumPy arrays/math.
      Notable calls: self.projector.has_pose, select_best_controller_track, np.array, point3_to_dict, String, json.dumps, self.world_track_compare_pub.publish, str.strip, self.projector.world_to_body, self.projector.world_to_camera, self._warn_throttled, body_track_debug.get, debug_info.get, comparisons.append, self._controller_track_sample, np.linalg.norm, isinstance, payload.get....
    - `_maybe_dump_world_track_compare_frame(self, frame: np.ndarray, compare_payload: dict[str, object] | None, now_s: float) -> None` (lines 2895-3012): No docstring; behavior is described from its body and call sites.
      Code map: 8 conditional branches; 3 for loops; 2 return points.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: frame.copy, compare_payload.get, isinstance, self._world_track_compare_overlay_lines, cv2.rectangle, cv2.imwrite, cv2.putText, comparison.get, stamp.get, cv2.circle, yolo_bbox.get, norfair_bbox.get, point.get.
    - `_track_compare_rows_from_payload(self, compare_payload: dict[str, object] | None) -> list[list[object]]` (lines 3014-3151): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 for loops; 4 return points.
      Notable calls: compare_payload.get, isinstance, stamp.get, dict_or_empty, rows.append, comparison.get, csv_value, track_position.get, delta.get, body_position.get, yolo_bbox.get, norfair_bbox.get, yolo_centroid.get, norfair_centroid.get, reference_position.get, reference_body_position.get, body_delta.get.
    - `_frame_compare_row_from_payload(self, compare_payload: dict[str, object] | None) -> list[object] | None` (lines 3153-3189): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: compare_payload.get, isinstance, stamp.get, csv_value.
    - `_maybe_append_world_track_compare_tracks_csv(self, compare_payload: dict[str, object] | None) -> None` (lines 3191-3204): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 return points.
      Notable calls: self._track_compare_rows_from_payload, self.pending_world_track_compare_track_rows.extend, self.now_s, self._flush_world_track_compare_csv.
    - `_maybe_append_world_track_compare_frames_csv(self, compare_payload: dict[str, object] | None) -> None` (lines 3206-3219): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 return points.
      Notable calls: self._frame_compare_row_from_payload, self.pending_world_track_compare_frame_rows.append, self.now_s, self._flush_world_track_compare_csv.
    - `_flush_world_track_compare_csv(self, *, now_s: float | None = None, force: bool = False) -> None` (lines 3221-3299): No docstring; behavior is described from its body and call sites.
      Code map: 11 conditional branches; 2 try/except blocks; 2 context managers; 3 return points.
      Notable calls: self.pending_world_track_compare_track_rows.clear, self.pending_world_track_compare_frame_rows.clear, self._ensure_world_track_compare_tracks_csv_ready, self._ensure_world_track_compare_frames_csv_ready, self.now_s, self.world_track_compare_tracks_csv_path.open, csv.writer, writer.writerows, self.world_track_compare_frames_csv_path.open.
    - `_world_track_compare_overlay_lines(self, compare_payload: dict[str, object]) -> list[str]` (lines 3301-3383): No docstring; behavior is described from its body and call sites.
      Code map: 6 conditional branches; 1 for loops; 3 return points.
      Notable calls: compare_payload.get, isinstance, lines.append, comparison.get, reference_position.get, track_position.get, delta.get, yolo_centroid.get, norfair_centroid.get.
- Top-level functions:
  - `led_color_name_for_engagement_state(*, state: str, blocked_reason: str = '', estop_latched: bool = False) -> str` (lines 149-175): No docstring; behavior is described from its body and call sites.
    Code map: 10 conditional branches; 11 return points.
    Notable calls: str.strip.upper, str.strip.
  - `led_bgr_for_color_name(color_name: str) -> tuple[int, int, int]` (lines 178-180): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: LED_RGB_BY_NAME.get.
  - `is_active_mission_state(state: str) -> bool` (lines 183-185): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip.upper, str.strip.
  - `make_depth_colormap(depth_frame: np.ndarray | None, *, max_depth_m: float, output_size: tuple[int, int]) -> np.ndarray` (lines 188-209): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 2 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: np.asarray, np.nan_to_num, np.clip, BinOp.astype, cv2.applyColorMap, np.any, cv2.resize, np.zeros.
  - `get_centroid(points: np.ndarray) -> np.ndarray` (lines 212-224): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.asarray, np.mean, np.array.
  - `map_point_between_frames(point: np.ndarray, src_shape: tuple[int, ...] | None, dst_shape: tuple[int, ...] | None) -> np.ndarray` (lines 227-240): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.array, np.asarray.
  - `compute_mean_depth(depth_frame: np.ndarray | None, center: np.ndarray, radius_px: int, max_valid_depth_m: float) -> float` (lines 243-269): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 5 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: depth_frame.astype, np.any, np.median, np.nanmax, np.isfinite.
  - `bbox_to_dict(bbox: list[float] | tuple[float, float, float, float] | None) -> dict[str, float] | None` (lines 272-282): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `point_to_dict(point: np.ndarray | list[float] | tuple[float, float] | None) -> dict[str, float] | None` (lines 285-290): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
  - `point3_to_dict(point: np.ndarray | list[float] | tuple[float, float, float] | None) -> dict[str, float] | None` (lines 293-302): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
  - `isoformat_wall_time(timestamp_s: float) -> str` (lines 305-306): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: time.strftime, time.localtime.
  - `dict_or_empty(value: object) -> dict[str, object]` (lines 309-312): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: isinstance.
  - `yolo_boxes_to_detections(boxes, *, frame: np.ndarray | None = None, enable_reid: bool = False, reid_histogram_bins: int = 32) -> list[Detection]` (lines 315-351): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 for loops; 3 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: boxes.xyxy.cpu.numpy, boxes.conf.cpu.numpy, np.ones, detections.append, boxes.xyxy.cpu, Detection, compute_lab_histogram_embedding, boxes.conf.cpu, np.array.
  - `main(args = None) -> None` (lines 3386-3396): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, RealsenseTrackerNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages, handles target track messages, handles world-frame target tracks, uses OpenCV image processing, uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/target_map_node.py`

- File type: Python source file.
- Tracked size: 20583 bytes; 569 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes TargetMapNode; defines functions _stamp_to_float_s, _point_from_array, _track_source_name, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations`
  - third-party: `import numpy as np`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import Point, PoseStamped; from rclpy.node import Node; from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy; from visualization_msgs.msg import Marker, MarkerArray`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace; from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic; from drone_msgs.msg import TargetTrack, TargetTrackArray, WorldTargetTrack, WorldTargetTrackArray; from drone_vision_pkg.target_memory import TRACK_SOURCE_DETECTED, TRACK_SOURCE_HELD, TRACK_SOURCE_PREDICTED, TargetMemory, TargetMemoryTrack, WorldTrackObservation, normalized_quaternion_to_rotation_matrix, world_to_b...`
- Classes:
  - `TargetMapNode` (lines 50-555, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, mavros_namespace, publish_rate_hz, world_frame, observation_fresh_s, marker_lifetime_s, world_tracks_topic, ownship_pose_topic, target_map_world_topic, target_map_tracks_topic, target_map_markers_topic, memory, latest_pose, last_pose_warn_s, last_tracks_stamp_s, world_pub, tracks_pub, markers_pub, timer, declare_parameter, create_subscription, world_tracks_callback, ownship_pose_callback, create_publisher, create_timer, publish_target_map, now_s, _publish_world_tracks, _publish_body_tracks, _publish_markers, _marker_color, _base_marker, _track_to_world_msg, get_parameter, get_logger, _warn_pose_missing, _track_to_body_msg, _position_marker, _uncertainty_marker, _path_marker` plus more.
    - `__init__(self) -> None` (lines 51-187): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages, handles target track messages, handles world-frame target tracks.
      Notable calls: super.__init__, self.declare_parameter, TargetMemory, QoSProfile, self.create_subscription, self.create_publisher, self.create_timer, configured_drone_id, configured_mavros_namespace, str.strip, cdrone_topic, external_pose_input_topic, self.get_parameter.
    - `now_s(self) -> float` (lines 189-190): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `ownship_pose_callback(self, msg: PoseStamped) -> None` (lines 192-193): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
    - `world_tracks_callback(self, msg: WorldTargetTrackArray) -> None` (lines 195-241): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops.
      Runtime interactions: handles target track messages, handles world-frame target tracks, uses NumPy arrays/math.
      Notable calls: self.now_s, self.memory.update, _stamp_to_float_s, observations.append, getattr, WorldTrackObservation, np.array.
    - `publish_target_map(self) -> None` (lines 243-253): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s, self.memory.snapshot, self._publish_world_tracks, self._publish_body_tracks, self._publish_markers, self.memory.prune.
    - `_publish_world_tracks(self, tracks: list[TargetMemoryTrack], *, stamp_msg, now_s: float) -> None` (lines 255-269): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages, handles target track messages, handles world-frame target tracks.
      Notable calls: WorldTargetTrackArray, self.world_pub.publish, self._track_to_world_msg.
    - `_publish_body_tracks(self, tracks: list[TargetMemoryTrack], *, stamp_msg, now_s: float) -> None` (lines 271-319): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 for loops; 1 return points.
      Runtime interactions: publishes messages, handles target track messages, uses NumPy arrays/math.
      Notable calls: TargetTrackArray, np.array, normalized_quaternion_to_rotation_matrix, self.tracks_pub.publish, world_to_body_position, body_tracks.append, self._warn_pose_missing, self._track_to_body_msg.
    - `_track_to_world_msg(self, track: TargetMemoryTrack, *, stamp_msg, now_s: float) -> WorldTargetTrack` (lines 321-355): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles target track messages, handles world-frame target tracks.
      Notable calls: track.observed_age_s, track.output_source, WorldTargetTrack, track.decayed_confidence.
    - `_track_to_body_msg(self, track: TargetMemoryTrack, *, body_position_m: np.ndarray, body_velocity_mps: np.ndarray, stamp_msg, now_s: float) -> TargetTrack` (lines 357-400): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: handles target track messages, uses NumPy arrays/math.
      Notable calls: track.observed_age_s, track.output_source, TargetTrack, track.decayed_confidence, np.linalg.norm, np.dot.
    - `_publish_markers(self, tracks: list[TargetMemoryTrack], *, stamp_msg, now_s: float) -> None` (lines 402-429): No docstring; behavior is described from its body and call sites.
      Code map: 1 for loops.
      Runtime interactions: publishes messages.
      Notable calls: MarkerArray, Marker, markers.markers.append, self.markers_pub.publish, track.output_source, markers.markers.extend, self._position_marker, self._uncertainty_marker, self._path_marker, self._label_marker.
    - `_base_marker(self, track: TargetMemoryTrack, marker_id: int, source: int, *, stamp_msg) -> Marker` (lines 431-455): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: Marker, self._marker_color.
    - `_position_marker(self, track: TargetMemoryTrack, source: int, *, stamp_msg) -> Marker` (lines 457-470): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self._base_marker, _point_from_array.
    - `_uncertainty_marker(self, track: TargetMemoryTrack, source: int, *, stamp_msg) -> Marker` (lines 472-492): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self._base_marker, _point_from_array.
    - `_path_marker(self, track: TargetMemoryTrack, source: int, *, stamp_msg) -> Marker` (lines 494-512): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self._base_marker, _point_from_array, marker.points.append.
    - `_label_marker(self, track: TargetMemoryTrack, source: int, *, stamp_msg, now_s: float) -> Marker` (lines 514-536): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: self._base_marker, _point_from_array, np.array, _track_source_name, track.observed_age_s.
    - `_marker_color(self, source: int) -> tuple[float, float, float, float]` (lines 538-545): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 4 return points.
    - `_warn_pose_missing(self) -> None` (lines 547-555): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s.
- Top-level functions:
  - `_stamp_to_float_s(stamp) -> float` (lines 28-29): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `_point_from_array(position_m: np.ndarray) -> Point` (lines 32-37): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: Point.
  - `_track_source_name(source: int) -> str` (lines 40-47): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 4 return points.
  - `main(args = None) -> None` (lines 558-565): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks.
    Notable calls: rclpy.init, TargetMapNode, rclpy.spin, node.destroy_node, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages, handles target track messages, handles world-frame target tracks, uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/target_memory.py`

- File type: Python source file.
- Tracked size: 13717 bytes; 376 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes WorldTrackObservation, TargetMemoryTrack, TargetMemory; defines functions finite_vector, normalized_quaternion_to_rotation_matrix, world_to_body_position.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass, field; import math`
  - third-party: `import numpy as np`
- Top-level constants/state: `TRACK_SOURCE_DETECTED, TRACK_SOURCE_HELD, TRACK_SOURCE_PREDICTED`.
- Classes:
  - `WorldTrackObservation` (lines 62-77, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `TargetMemoryTrack` (lines 81-143, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `position_m, position_uncertainty_m, velocity_uncertainty_mps, last_update_s, source, observed_age_s, velocity_mps, last_observed_s, last_observation_received_s, confidence`.
    - `predict_to(self, now_s: float, *, position_process_noise_mps: float, velocity_process_noise_mps: float) -> None` (lines 100-121): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `observed_age_s(self, now_s: float) -> float` (lines 123-124): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `output_source(self, now_s: float, *, observation_fresh_s: float) -> int` (lines 126-133): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: self.observed_age_s.
    - `decayed_confidence(self, now_s: float, *, confidence_decay_per_s: float) -> float` (lines 135-143): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.observed_age_s.
  - `TargetMemory` (lines 146-376, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `association_gate_m, association_uncertainty_scale, max_prediction_horizon_s, prune_after_s, confidence_decay_per_s, observed_position_uncertainty_m, observed_velocity_uncertainty_mps, position_process_noise_mps, velocity_process_noise_mps, velocity_blend_alpha, max_velocity_mps, history_length, tracks, next_map_id, predict_all, prune, _append_history, _bounded_velocity, _find_match, _create_track, _update_track`.
    - `__init__(self, *, association_gate_m: float = 0.8, association_uncertainty_scale: float = 1.0, max_prediction_horizon_s: float = 3.0, prune_after_s: float = 5.0, confidence_decay_per_s: float = 0.25, observed_position_uncertainty_m: float = 0.15, observed_velocity_uncertainty_mps: float = 0.35, position_process_noise_mps: float = 0.35, velocity_process_noise_mps: float = 0.25, velocity_blend_alpha: float = 0.55, max_velocity_mps: float = 8.0, history_length: int = 30) -> None` (lines 147-176): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `update(self, observations: list[WorldTrackObservation], *, now_s: float) -> None` (lines 178-202): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 for loops.
      Notable calls: self.predict_all, sorted, self.prune, self.tracks.keys, self._find_match, available_track_ids.discard, self._create_track, self._update_track, finite_vector.
    - `predict_all(self, now_s: float) -> None` (lines 204-210): No docstring; behavior is described from its body and call sites.
      Code map: 1 for loops.
      Notable calls: self.tracks.values, track.predict_to.
    - `snapshot(self, *, now_s: float, observation_fresh_s: float) -> list[TargetMemoryTrack]` (lines 212-230): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 for loops; 1 return points.
      Notable calls: self.predict_all, self.tracks.values, visible_tracks.sort, visible_tracks.append, track.observed_age_s, track.output_source, self._append_history.
    - `prune(self, now_s: float) -> None` (lines 232-240): No docstring; behavior is described from its body and call sites.
      Code map: 1 for loops.
      Notable calls: self.tracks.pop, self.tracks.items, track.observed_age_s.
    - `_find_match(self, observation: WorldTrackObservation, available_track_ids: set[int], *, now_s: float) -> int | None` (lines 242-277): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 for loops; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: track.observed_age_s, np.linalg.norm.
    - `_create_track(self, observation: WorldTrackObservation, *, now_s: float) -> int` (lines 279-317): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: TargetMemoryTrack, self._append_history, np.asarray.copy, self._bounded_velocity, np.asarray.
    - `_update_track(self, map_id: int, observation: WorldTrackObservation, *, now_s: float) -> None` (lines 319-361): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: track.position_m.copy, self._bounded_velocity, np.asarray.copy, self._append_history, np.linalg.norm, np.asarray.
    - `_bounded_velocity(self, velocity_mps: np.ndarray) -> np.ndarray` (lines 363-371): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: np.asarray.copy, finite_vector, np.zeros, np.linalg.norm, np.asarray.
    - `_append_history(self, track: TargetMemoryTrack, *, now_s: float) -> None` (lines 373-376): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: track.history.append, track.position_m.copy.
- Top-level functions:
  - `finite_vector(values: np.ndarray, *, expected_size: int = 3) -> bool` (lines 14-16): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.asarray, np.all, np.isfinite.
  - `normalized_quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray` (lines 19-47): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: math.sqrt, np.array.
  - `world_to_body_position(world_position_m: np.ndarray, ownship_position_m: np.ndarray, world_from_body_rotation: np.ndarray) -> np.ndarray` (lines 50-58): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.asarray.
- Whole-file runtime themes: uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/tracker_resilience.py`

- File type: Python source file.
- Tracked size: 6418 bytes; 206 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes BodyTrackState; defines functions clamp_bbox_to_image, compute_lab_histogram_embedding, histogram_correlation_distance, latest_track_embedding, tracked_object_reid_distance, extrapolate_body_track_state, select_held_track_states.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass; import math; from typing import Mapping`
  - third-party: `import numpy as np`
- Top-level constants/state: `TRACK_SOURCE_DETECTED, TRACK_SOURCE_HELD, TRACK_SOURCE_PREDICTED`.
- Classes:
  - `BodyTrackState` (lines 20-37, bases: `object`): No class docstring; role is inferred from methods and base classes.
- Top-level functions:
  - `clamp_bbox_to_image(bbox: list[float] | tuple[float, float, float, float] | None, image_shape: tuple[int, ...]) -> tuple[int, int, int, int] | None` (lines 40-62): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 4 return points.
    Notable calls: math.floor, math.ceil.
  - `compute_lab_histogram_embedding(image: np.ndarray | None, bbox: list[float] | tuple[float, float, float, float] | None, *, bins_per_channel: int) -> np.ndarray | None` (lines 65-93): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 5 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: clamp_bbox_to_image, cv2.cvtColor, cv2.calcHist, cv2.normalize.flatten, histogram.astype, cv2.normalize.
  - `histogram_correlation_distance(first_embedding: np.ndarray | None, second_embedding: np.ndarray | None) -> float` (lines 96-111): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: cv2.compareHist, math.isfinite, np.asarray.
  - `latest_track_embedding(track: object) -> np.ndarray | None` (lines 114-124): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 3 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: getattr, reversed, np.asarray.
  - `tracked_object_reid_distance(first_track: object, second_track: object) -> float` (lines 127-131): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: histogram_correlation_distance, latest_track_embedding.
  - `extrapolate_body_track_state(track_state: BodyTrackState, *, now_s: float, max_extrapolation_m: float) -> BodyTrackState` (lines 134-176): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.array, BodyTrackState, np.linalg.norm.
  - `select_held_track_states(*, active_track_ids: set[int], fresh_track_ids: set[int], cached_tracks: Mapping[int, BodyTrackState], now_s: float, hold_window_s: float, max_extrapolation_m: float) -> dict[int, BodyTrackState]` (lines 179-206): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 1 for loops; 2 return points.
    Notable calls: sorted, cached_tracks.get, extrapolate_body_track_state.
- Whole-file runtime themes: uses OpenCV image processing, uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/tracking_metrics.py`

- File type: Python source file.
- Tracked size: 81612 bytes; 2281 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes TrackSample, FrameSample, CleanedPathPoint, ReferenceVideoInfo, CleanedTrackPostprocessResult, TrackingMetricsRun, TrackingMetricsNode; defines functions is_active_mission_state, source_label, finite_float, optional_float, safe_ratio, percentile, stamp_to_s, make_run_id, sample_from_world_track, select_active_track and more; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import csv; from dataclasses import dataclass, replace; import json; import math; from pathlib import Path; import re; import time; from typing import Any, Iterable`
  - third-party: `import numpy as np`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from rclpy.node import Node; from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy`
  - local/project: `from drone_control_pkg.deployment_config import configured_drone_id, configured_ownship_pose_topic; from drone_control_pkg.topic_utils import cdrone_topic; from drone_msgs.msg import EngagementState, WorldTargetTrack, WorldTargetTrackArray`
- Top-level constants/state: `TRACK_SOURCE_DETECTED, TRACK_SOURCE_HELD, TRACK_SOURCE_PREDICTED, MISSION_TERMINAL_STATES, SAMPLES_CSV, FRAMES_CSV, CLEANED_PATH_CSV, SUMMARY_CSV, MANIFEST_JSON, CLEANED_PATH_MP4, REFERENCE_VIDEO_PATTERN, MAX_REFERENCE_START_DELTA_S, CLEANED_VIDEO_HISTORY_WINDOW_S, SAMPLE_FIELDNAMES, FRAME_FIELDNAMES, CLEANED_PATH_FIELDNAMES, SUMMARY_FIELDNAMES`.
- Classes:
  - `TrackSample` (lines 229-244, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `FrameSample` (lines 248-264, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `CleanedPathPoint` (lines 268-276, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `ReferenceVideoInfo` (lines 280-291, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `fps, frame_count`.
    - `duration_s(self) -> float` (lines 288-291): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
  - `CleanedTrackPostprocessResult` (lines 295-321, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `reference_video, time_origin_s, time_origin_source, video_synced_to_reference, video_fps, video_frame_count, fallback_reason, cleaned_path_csv, cleaned_path_mp4`.
    - `to_manifest_dict(self) -> dict[str, object]` (lines 306-321): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
  - `TrackingMetricsRun` (lines 453-718, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `run_id, experiment_tag, max_reacquisition_gap_s, simulated_dropout_horizon_s, smoothing_window, samples, frames, ownship_history, raw_dropout_start_s, bridge_start_s, longest_raw_dropout_s, longest_bridge_s, last_raw_track_id, last_target_map_track_id, raw_id_switch_count, target_map_id_switch_count, last_map_id_before_dropout, last_predicted_map_sample, raw_reacquisition_count, target_map_same_id_reacquisition_count, reacquisition_residuals_m, reacquisition_uncertainties_m, cleaned_path, _handle_raw_reacquisition`.
    - `__init__(self, *, run_id: str, experiment_tag: str, max_reacquisition_gap_s: float = 3.0, simulated_dropout_horizon_s: float = 1.0, smoothing_window: int = 5) -> None` (lines 454-484): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: uses NumPy arrays/math.
    - `add_samples(self, samples: list[TrackSample]) -> None` (lines 486-487): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.samples.extend.
    - `add_ownship_position(self, timestamp_s: float, position_m: np.ndarray) -> None` (lines 489-493): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: np.asarray, self.ownship_history.append, np.all, position.copy, np.isfinite.
    - `sample_frame(self, *, timestamp_s: float, state: str, raw_tracks: list[TrackSample], target_map_tracks: list[TrackSample]) -> FrameSample` (lines 495-587): No docstring; behavior is described from its body and call sites.
      Code map: 10 conditional branches; 1 return points.
      Notable calls: select_active_track, FrameSample, self.frames.append, self._handle_raw_reacquisition.
    - `_handle_raw_reacquisition(self, *, timestamp_s: float, active_raw: TrackSample, active_map: TrackSample | None) -> None` (lines 589-620): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 3 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: math.isfinite, np.linalg.norm, self.reacquisition_residuals_m.append, self.reacquisition_uncertainties_m.append.
    - `cleaned_path(self) -> list[CleanedPathPoint]` (lines 622-626): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: clean_target_path.
    - `summary(self) -> dict[str, object]` (lines 628-718): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: longest_false_coverage_gap_s, simulated_dropout_backtest, self.cleaned_path, _csv_optional, safe_ratio, percentile.
  - `TrackingMetricsNode` (lines 1630-1889, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, experiment_tag, output_dir, raw_world_tracks_topic, target_map_world_tracks_topic, ownship_pose_topic, engagement_state_topic, metrics_rate_hz, track_stale_s, max_reacquisition_gap_s, simulated_dropout_horizon_s, path_smoothing_window, write_video, video_fps, video_width, video_height, video_fourcc, postprocess_cleaned_tracks, reference_video, reference_recording_dir, reference_video_wait_s, current_run, current_run_dir, latest_raw_tracks, latest_map_tracks, last_raw_tracks_s, last_map_tracks_s, latest_state, last_finalize_wall_s, timer, declare_parameter, create_subscription, raw_tracks_callback, ownship_pose_callback, engagement_state_callback, create_timer, timer_callback, _m...` plus more.
    - `__init__(self) -> None` (lines 1631-1752): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates subscriptions, uses timer callbacks, handles pose messages, handles target track messages, handles world-frame target tracks.
      Notable calls: super.__init__, self.declare_parameter, str.strip, Path.expanduser, QoSProfile, self.create_subscription, self.create_timer, configured_drone_id, configured_ownship_pose_topic, cdrone_topic, self.get_parameter, Path.
    - `now_s(self) -> float` (lines 1754-1755): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `raw_tracks_callback(self, msg: WorldTargetTrackArray) -> None` (lines 1757-1766): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: handles target track messages, handles world-frame target tracks.
      Notable calls: self._message_time_s, self.now_s, sample_from_world_track, self.current_run.add_samples.
    - `target_map_tracks_callback(self, msg: WorldTargetTrackArray) -> None` (lines 1768-1781): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: handles target track messages, handles world-frame target tracks.
      Notable calls: self._message_time_s, self.now_s, sample_from_world_track, self.current_run.add_samples.
    - `ownship_pose_callback(self, msg: PoseStamped) -> None` (lines 1783-1797): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Runtime interactions: handles pose messages, uses NumPy arrays/math.
      Notable calls: self._stamp_or_now_s, self.current_run.add_ownship_position, np.array, finite_float.
    - `engagement_state_callback(self, msg: EngagementState) -> None` (lines 1799-1804): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
      Notable calls: is_active_mission_state, self._start_run_if_needed, self._finalize_current_run.
    - `timer_callback(self) -> None` (lines 1806-1828): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: self.now_s, run.sample_frame.
    - `destroy_node(self) -> bool` (lines 1830-1833): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: super.destroy_node, self._finalize_current_run.
    - `_message_time_s(self, msg: WorldTargetTrackArray) -> float` (lines 1835-1837): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles target track messages, handles world-frame target tracks.
      Notable calls: stamp_to_s, self.now_s.
    - `_stamp_or_now_s(self, stamp: Any) -> float` (lines 1839-1841): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: stamp_to_s, self.now_s.
    - `_start_run_if_needed(self) -> None` (lines 1843-1861): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: time.time, make_run_id, TrackingMetricsRun.
    - `_finalize_current_run(self, *, reason: str) -> None` (lines 1863-1889): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: write_run_outputs, time.time, ', '.join, artifacts.items.
- Top-level functions:
  - `is_active_mission_state(state: str) -> bool` (lines 143-145): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip.upper, str.strip.
  - `source_label(source: int) -> str` (lines 148-155): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 4 return points.
  - `finite_float(value: Any, default: float = 0.0) -> float` (lines 158-163): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks; 2 return points.
    Notable calls: math.isfinite.
  - `optional_float(value: Any) -> float | None` (lines 166-176): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 try/except blocks; 4 return points.
    Notable calls: str.strip, math.isfinite.
  - `safe_ratio(numerator: int | float, denominator: int | float) -> float | None` (lines 179-183): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `percentile(values: Iterable[float], q: float) -> float | None` (lines 186-194): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.percentile, math.isfinite, np.asarray.
  - `stamp_to_s(stamp: Any) -> float` (lines 197-200): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: getattr.
  - `make_run_id(*, drone_id: str, experiment_tag: str, started_wall_time_s: float | None = None) -> str` (lines 203-225): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 return points.
    Notable calls: time.strftime, ''.join.strip, '_'.join, time.time, time.localtime, parts.append, ''.join, ch.isalnum, ch.lower.
  - `sample_from_world_track(track: WorldTargetTrack, *, timestamp_s: float, sample_kind: str) -> TrackSample` (lines 324-373): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: handles target track messages, handles world-frame target tracks, uses NumPy arrays/math.
    Notable calls: TrackSample, getattr, np.array, finite_float.
  - `select_active_track(samples: list[TrackSample]) -> TrackSample | None` (lines 376-386): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: sorted.
  - `sample_to_csv_row(sample: TrackSample, *, run_id: str, experiment_tag: str) -> dict[str, object]` (lines 389-418): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: source_label.
  - `frame_to_csv_row(frame: FrameSample, *, run_id: str, experiment_tag: str) -> dict[str, object]` (lines 421-450): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `_csv_optional(value: float | None) -> str` (lines 721-722): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `longest_false_coverage_gap_s(frames: list[FrameSample]) -> float` (lines 725-741): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 1 for loops; 2 return points.
  - `dominant_track_id(samples: list[TrackSample]) -> int | None` (lines 744-750): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 2 return points.
    Notable calls: counts.get, sorted, counts.items.
  - `clean_target_path(samples: list[TrackSample], *, smoothing_window: int = 5) -> list[CleanedPathPoint]` (lines 753-796): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 3 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: dominant_track_id, sorted, np.asarray, np.mean, cleaned.append, CleanedPathPoint, sample.position_m.copy.
  - `cleaned_path_to_csv_row(point: CleanedPathPoint, *, run_id: str, experiment_tag: str) -> dict[str, object]` (lines 799-821): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: source_label.
  - `simulated_dropout_backtest(samples: list[TrackSample], *, horizon_s: float = 1.0, max_velocity_seed_dt_s: float = 0.75) -> list[float]` (lines 824-870): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 1 for loops; 3 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: dominant_track_id, sorted, next, math.isfinite, np.linalg.norm, errors.append.
  - `write_csv(path: Path, fieldnames: list[str], rows: Iterable[dict[str, object]]) -> None` (lines 873-879): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 context managers.
    Notable calls: path.parent.mkdir, path.open, csv.DictWriter, writer.writeheader, writer.writerow.
  - `parse_recording_timestamp_s(name: str) -> float | None` (lines 882-891): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks; 3 return points.
    Notable calls: re.search, match.groups, time.mktime, time.strptime.
  - `read_reference_video_info(path: Path | str) -> ReferenceVideoInfo | None` (lines 894-918): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 1 try/except blocks; 5 return points.
    Runtime interactions: uses OpenCV image processing.
    Notable calls: Path.expanduser, cv2.VideoCapture, ReferenceVideoInfo, video_path.exists, capture.release, Path, capture.isOpened, capture.get.
  - `find_nearest_reference_video(*, run_dir: Path | str, recording_dir: Path | str, run_id: str = '', drone_id: str = '') -> Path | None` (lines 921-963): No docstring; behavior is described from its body and call sites.
    Code map: 7 conditional branches; 1 try/except blocks; 7 return points.
    Notable calls: Path.expanduser, str.strip, Path, parse_recording_timestamp_s, score, recordings.exists, run_path.exists, time.time, sorted, recordings.glob, run_path.stat, candidate.stat.
  - `resolve_reference_video_info(*, run_dir: Path | str, run_id: str, drone_id: str, reference_video: Path | str | None, reference_recording_dir: Path | str | None, wait_s: float) -> tuple[ReferenceVideoInfo | None, str]` (lines 966-1006): No docstring; behavior is described from its body and call sites.
    Code map: 7 conditional branches; 1 while loops; 2 return points.
    Notable calls: time.monotonic, str.strip, time.sleep, Path.expanduser, read_reference_video_info, candidate.exists, find_nearest_reference_video, Path.
  - `cleaned_time_origin_s(*, cleaned_path: list[CleanedPathPoint], all_samples: list[TrackSample], ownship_history: list[tuple[float, np.ndarray]]) -> tuple[float, str]` (lines 1009-1021): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 4 return points.
    Runtime interactions: uses NumPy arrays/math.
  - `_relative_time(timestamp_s: float, origin_s: float) -> float` (lines 1024-1026): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `normalized_cleaned_path(cleaned_path: list[CleanedPathPoint], *, origin_s: float) -> list[CleanedPathPoint]` (lines 1029-1037): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: replace, _relative_time.
  - `normalized_track_samples(samples: list[TrackSample], *, origin_s: float) -> list[TrackSample]` (lines 1040-1048): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: replace, _relative_time.
  - `normalized_ownship_history(history: list[tuple[float, np.ndarray]], *, origin_s: float) -> list[tuple[float, np.ndarray]]` (lines 1051-1059): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: _relative_time, position.copy.
  - `write_cleaned_track_outputs(*, run_id: str, experiment_tag: str, drone_id: str, run_dir: Path, cleaned_path: list[CleanedPathPoint], all_samples: list[TrackSample], ownship_history: list[tuple[float, np.ndarray]], write_video: bool, video_fps: float, video_width: int, video_height: int, video_fourcc: str, reference_video: Path | str | None = None, reference_recording_dir: Path | str | None = None, reference_video_wait_s: float = 0.0) -> CleanedTrackPostprocessResult` (lines 1062-1163): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: run_dir.mkdir, cleaned_time_origin_s, normalized_cleaned_path, normalized_track_samples, normalized_ownship_history, write_csv, CleanedTrackPostprocessResult, cleaned_path_to_csv_row, resolve_reference_video_info, write_cleaned_path_video.
  - `write_run_outputs(run: TrackingMetricsRun, *, run_dir: Path, drone_id: str = '', write_video: bool = True, video_fps: float = 10.0, video_width: int = 1280, video_height: int = 720, video_fourcc: str = 'mp4v', postprocess_cleaned_tracks: bool = True, reference_video: Path | str | None = None, reference_recording_dir: Path | str | None = None, reference_video_wait_s: float = 0.0) -> dict[str, str]` (lines 1166-1259): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 return points.
    Notable calls: run_dir.mkdir, run.summary, write_csv, manifest_path.write_text, sample_to_csv_row, frame_to_csv_row, write_cleaned_track_outputs, postprocess_result.to_manifest_dict, time.strftime, sorted, time.localtime, json.dumps, run.cleaned_path, time.time.
  - `_history_start_s(frame_time_s: float, history_window_s: float) -> float` (lines 1262-1263): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `_timestamp_in_history_window(timestamp_s: float, *, frame_time_s: float, history_window_s: float) -> bool` (lines 1266-1276): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _history_start_s.
  - `samples_in_history_window(samples: list[TrackSample], *, frame_time_s: float, history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S) -> list[TrackSample]` (lines 1279-1293): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _timestamp_in_history_window.
  - `positions_in_history_window(history: list[tuple[float, np.ndarray]], *, frame_time_s: float, history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S) -> list[np.ndarray]` (lines 1296-1310): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: _timestamp_in_history_window.
  - `cleaned_points_in_history_window(cleaned_path: list[CleanedPathPoint], *, frame_time_s: float, history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S) -> list[CleanedPathPoint]` (lines 1313-1327): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: _timestamp_in_history_window.
  - `write_cleaned_path_video(output_path: Path, *, cleaned_path: list[CleanedPathPoint], all_samples: list[TrackSample], ownship_history: list[tuple[float, np.ndarray]], fps: float = 10.0, width: int = 1280, height: int = 720, fourcc_text: str = 'mp4v', frame_count: int | None = None, history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S) -> bool` (lines 1330-1499): No docstring; behavior is described from its body and call sites.
    Code map: 8 conditional branches; 1 for loops; 1 while loops; 4 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: output_path.parent.mkdir, cv2.VideoWriter_fourcc, cv2.VideoWriter, all_points.extend, map_bounds, sorted, writer.release, writer.isOpened, samples_in_history_window, np.full, draw_grid, draw_history_polyline, draw_sample_points, draw_video_overlay, writer.write, str.ljust, _timestamp_in_history_window, cleaned_points_in_history_window....
  - `map_bounds(points: list[np.ndarray]) -> tuple[float, float, float, float]` (lines 1502-1513): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.asarray, np.min, np.max.
  - `world_to_pixel(position_m: np.ndarray, bounds: tuple[float, float, float, float], width: int, height: int) -> tuple[int, int]` (lines 1516-1528): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
  - `world_radius_to_px(radius_m: float, bounds: tuple[float, float, float, float], width: int, height: int) -> int` (lines 1531-1540): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `draw_grid(frame: np.ndarray, bounds: tuple[float, float, float, float]) -> None` (lines 1543-1555): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 for loops; 1 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: np.linspace, world_to_pixel, cv2.line, np.array.
  - `draw_history_polyline(frame: np.ndarray, positions: list[np.ndarray], bounds: tuple[float, float, float, float], *, color: tuple[int, int, int], thickness: int) -> None` (lines 1558-1571): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: world_to_pixel, cv2.line.
  - `draw_sample_points(frame: np.ndarray, samples: list[TrackSample], bounds: tuple[float, float, float, float], *, color: tuple[int, int, int], radius: int) -> None` (lines 1574-1587): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: world_to_pixel, cv2.circle.
  - `draw_video_overlay(frame: np.ndarray, *, point: CleanedPathPoint | None, index: int, total: int, display_time_s: float) -> None` (lines 1590-1627): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 1 return points.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: cv2.rectangle, cv2.putText, source_label.
  - `main(args: list[str] | None = None) -> None` (lines 1892-1899): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks.
    Notable calls: rclpy.init, TrackingMetricsNode, rclpy.spin, node.destroy_node, rclpy.shutdown.
  - `_csv_int(value: Any, default: int = 0) -> int` (lines 1902-1906): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks; 2 return points.
    Notable calls: str.strip.
  - `_csv_bool(value: Any) -> bool` (lines 1909-1910): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip.lower, str.strip.
  - `_drone_id_from_run_id(run_id: str) -> str` (lines 1913-1915): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: re.match, match.group.
  - `_default_reference_recording_dir(run_dir: Path) -> Path` (lines 1918-1921): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: Path.
  - `read_tracking_samples_csv(run_dir: Path | str) -> tuple[list[TrackSample], str, str]` (lines 1924-1986): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 1 for loops; 1 context managers; 1 return points; 1 raise statements.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: Path.expanduser, samples_path.exists, FileNotFoundError, samples_path.open, csv.DictReader, Path, _csv_int, samples.append, str.strip, row.get, TrackSample, finite_float, np.array, _csv_bool.
  - `update_tracking_manifest(*, run_dir: Path, run_id: str, experiment_tag: str, result: CleanedTrackPostprocessResult) -> Path` (lines 1989-2034): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 try/except blocks; 1 return points.
    Notable calls: manifest_path.exists, manifest.get, files.update, manifest.update, manifest_path.write_text, isinstance, json.loads, time.strftime, result.to_manifest_dict, json.dumps, manifest_path.read_text, time.localtime, time.time.
  - `postprocess_tracking_run_dir(*, run_dir: Path | str, reference_video: Path | str | None = None, reference_recording_dir: Path | str | None = None, reference_video_wait_s: float = 0.0, smoothing_window: int = 5, write_video: bool = True, video_fps: float = 10.0, video_width: int = 1280, video_height: int = 720, video_fourcc: str = 'mp4v') -> dict[str, object]` (lines 2037-2090): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: Path.expanduser, read_tracking_samples_csv, clean_target_path, write_cleaned_track_outputs, update_tracking_manifest, _default_reference_recording_dir, Path, str.strip, _drone_id_from_run_id.
  - `build_postprocess_arg_parser() -> argparse.ArgumentParser` (lines 2093-2107): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument.
  - `postprocess_main(argv: list[str] | None = None) -> int` (lines 2110-2132): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 return points.
    Notable calls: build_postprocess_arg_parser, parser.parse_args, postprocess_tracking_run_dir, artifacts.get.
  - `read_summary_csv(run_dir: Path) -> dict[str, str]` (lines 2135-2143): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 context managers; 1 return points; 2 raise statements.
    Notable calls: path.exists, FileNotFoundError, path.open, ValueError, csv.DictReader, rows.items.
  - `compare_tracking_runs(*, milestone3_run_dir: str | Path, milestone4_run_dir: str | Path, output_dir: str | Path) -> dict[str, object]` (lines 2146-2210): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 return points.
    Notable calls: Path.expanduser.resolve, out_dir.mkdir, read_summary_csv, write_csv, json_path.write_text, write_comparison_markdown, optional_float, rows.append, Path.expanduser, m3.get, m4.get, json.dumps, Path.
  - `write_comparison_markdown(report: dict[str, object], path: Path) -> None` (lines 2213-2259): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops.
    Notable calls: report.get, isinstance, lines.extend, path.write_text, lines.append, '\n'.join, row.get.
  - `build_compare_arg_parser() -> argparse.ArgumentParser` (lines 2262-2269): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument.
  - `compare_main(argv: list[str] | None = None) -> int` (lines 2272-2281): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: build_compare_arg_parser, parser.parse_args, compare_tracking_runs.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates subscriptions, uses timer callbacks, handles pose messages, handles target track messages, handles world-frame target tracks, uses OpenCV image processing, uses NumPy arrays/math, defines a CLI.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/world_track_compare_common.py`

- File type: Python source file.
- Tracked size: 5955 bytes; 227 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes ControllerTrackSample; defines functions sanitize_experiment_tag, make_run_id, csv_value, controller_track_is_valid, controller_track_score, select_best_controller_track.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Shared helpers for world-track comparison logging and reporting.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass; import math; import re; import time`
- Top-level constants/state: `RUN_MANIFEST_FILENAME, TRACKS_CSV_FILENAME_DEFAULT, FRAMES_CSV_FILENAME, REPORT_DIRNAME, TRACKS_CSV_FIELDNAMES, FRAMES_CSV_FIELDNAMES, FOLLOW_CONTROLLER_GATES`.
- Classes:
  - `ControllerTrackSample` (lines 110-123, bases: `object`): Track fields needed to mirror the follow controller selection logic.
- Top-level functions:
  - `sanitize_experiment_tag(tag: str) -> str` (lines 126-130): Return a filesystem-safe experiment tag.
    Code map: 1 return points.
    Notable calls: re.sub, clean_tag.strip, str.strip.
  - `make_run_id(*, started_wall_time_s: float | None = None, experiment_tag: str = '') -> str` (lines 133-149): Create a stable run identifier for output bundle directories.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: time.strftime, sanitize_experiment_tag, time.time, time.localtime.
  - `csv_value(value)` (lines 152-155): Return an empty string for missing CSV values.
    Code map: 1 return points.
  - `controller_track_is_valid(track: ControllerTrackSample, *, min_track_confidence: float = FOLLOW_CONTROLLER_GATES['min_track_confidence'], max_target_distance_m: float = FOLLOW_CONTROLLER_GATES['max_target_distance_m'], require_target_in_front: bool = FOLLOW_CONTROLLER_GATES['require_target_in_front'], max_abs_target_y_m: float = FOLLOW_CONTROLLER_GATES['max_abs_target_y_m'], max_abs_target_z_m: float = FOLLOW_CONTROLLER_GATES['max_abs_target_z_m']) -> bool` (lines 158-181): Apply the controller's track validity gates to a candidate.
    Code map: 5 conditional branches; 6 return points.
  - `controller_track_score(track: ControllerTrackSample) -> float` (lines 184-204): Score a candidate track the same way as the follow controller.
    Code map: 1 return points.
    Notable calls: math.sqrt.
  - `select_best_controller_track(tracks: list[ControllerTrackSample]) -> tuple[ControllerTrackSample | None, float | None]` (lines 207-227): Return the controller-valid track with the highest selection score.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: scored_tracks.sort, controller_track_is_valid, controller_track_score.

### `ros2/src/drone_vision_pkg/drone_vision_pkg/world_track_compare_report.py`

- File type: Python source file.
- Tracked size: 28308 bytes; 917 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines functions optional_float, optional_int, optional_bool, safe_mean, percentile, load_json, read_typed_csv, first_existing_path, resolve_artifacts, finite_series and more; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Offline reporting for world-track comparison experiment bundles.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import csv; import json; from pathlib import Path; from typing import Any`
  - third-party: `import matplotlib; import numpy as np; from matplotlib import pyplot as plt`
  - local/project: `from drone_vision_pkg.world_track_compare_common import FOLLOW_CONTROLLER_GATES, FRAMES_CSV_FILENAME, REPORT_DIRNAME, RUN_MANIFEST_FILENAME, TRACKS_CSV_FILENAME_DEFAULT`
- Top-level constants/state: `TRACK_FLOAT_FIELDS, TRACK_INT_FIELDS, TRACK_BOOL_FIELDS, FRAME_FLOAT_FIELDS, FRAME_INT_FIELDS, FRAME_BOOL_FIELDS`.
- Top-level functions:
  - `optional_float(value: Any) -> float | None` (lines 106-114): Parse a CSV value into a float or None.
    Code map: 2 conditional branches; 3 return points.
    Notable calls: str.strip.
  - `optional_int(value: Any) -> int | None` (lines 117-125): Parse a CSV value into an int or None.
    Code map: 2 conditional branches; 3 return points.
    Notable calls: str.strip.
  - `optional_bool(value: Any) -> bool | None` (lines 128-140): Parse a CSV value into a bool or None.
    Code map: 4 conditional branches; 4 return points; 1 raise statements.
    Notable calls: str.strip.lower, ValueError, str.strip.
  - `safe_mean(values: list[float]) -> float | None` (lines 143-148): Return the mean of a list or None when empty.
    Code map: 1 conditional branches; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.mean, np.asarray.
  - `percentile(values: list[float], q: float) -> float | None` (lines 151-156): Return a percentile from a list of floats.
    Code map: 1 conditional branches; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.percentile, np.asarray.
  - `load_json(path: Path) -> dict[str, Any]` (lines 159-166): Load a JSON file if it exists.
    Code map: 1 conditional branches; 1 context managers; 2 return points.
    Notable calls: path.exists, path.open, json.load, isinstance.
  - `read_typed_csv(path: Path, *, float_fields: set[str], int_fields: set[str], bool_fields: set[str]) -> list[dict[str, Any]]` (lines 169-198): Read a CSV into typed dictionaries.
    Code map: 5 conditional branches; 2 for loops; 1 context managers; 2 return points.
    Notable calls: path.exists, path.open, csv.DictReader, row.items, rows.append, optional_float, optional_int, optional_bool.
  - `first_existing_path(paths: list[Path]) -> Path | None` (lines 201-207): Return the first existing path from a list.
    Code map: 1 conditional branches; 1 for loops; 2 return points.
    Notable calls: path.exists.
  - `resolve_artifacts(run_dir: Path) -> dict[str, Path | None]` (lines 210-233): Resolve the manifest and CSV files for a run bundle.
    Code map: 2 conditional branches; 1 return points.
    Notable calls: load_json, manifest_files.get, isinstance, manifest.get, tracks_candidates.insert, frames_candidates.insert, first_existing_path, Path, manifest_path.exists.
  - `finite_series(rows: list[dict[str, Any]], key: str) -> list[float]` (lines 236-247): Extract a finite float series from a list of rows.
    Code map: 2 conditional branches; 1 for loops; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: row.get, np.isfinite, values.append.
  - `finite_xy(rows: list[dict[str, Any]], x_key: str, y_key: str) -> tuple[list[float], list[float]]` (lines 250-270): Extract paired finite series from a list of rows.
    Code map: 2 conditional branches; 1 for loops; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: row.get, xs.append, ys.append, np.isfinite.
  - `empty_plot(output_path: Path, title: str, message: str) -> None` (lines 273-282): Write a placeholder plot when no usable data exists.
    Code map: straight-line helper logic.
    Notable calls: plt.subplots, ax.axis, ax.text, ax.set_title, fig.tight_layout, fig.savefig, plt.close.
  - `scatter_plot(xs: list[float], ys: list[float], *, output_path: Path, title: str, xlabel: str, ylabel: str) -> None` (lines 285-308): Save a scatter plot or a placeholder when empty.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: plt.subplots, ax.scatter, ax.set_title, ax.set_xlabel, ax.set_ylabel, ax.grid, fig.tight_layout, fig.savefig, plt.close, empty_plot.
  - `bins_for(values: list[float], bin_size: float) -> np.ndarray` (lines 311-320): Build inclusive bin edges for a series.
    Code map: 1 conditional branches; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.arange, np.floor, np.ceil.
  - `depth_binned_error_plot(track_rows: list[dict[str, Any]], *, depth_bin_size: float, output_path: Path) -> None` (lines 323-374): Plot median and p90 error by estimated depth bin.
    Code map: 3 conditional branches; 1 for loops; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: finite_xy, bins_for, plt.subplots, ax.plot, ax.set_title, ax.set_xlabel, ax.set_ylabel, ax.grid, ax.legend, fig.tight_layout, fig.savefig, plt.close, empty_plot, centers.append, medians.append, p90s.append, np.median, np.percentile....
  - `coverage_by_range(frame_rows: list[dict[str, Any]], *, depth_bin_size: float) -> list[dict[str, Any]]` (lines 377-422): Compute range-binned frame coverage metrics.
    Code map: 2 conditional branches; 1 for loops; 2 return points.
    Notable calls: bins_for, summaries.append, row.get.
  - `coverage_plot(frame_rows: list[dict[str, Any]], *, depth_bin_size: float, output_path: Path) -> list[dict[str, Any]]` (lines 425-469): Plot world-track and controller-valid coverage by range.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: coverage_by_range, plt.subplots, ax.plot, ax.set_ylim, ax.set_title, ax.set_xlabel, ax.set_ylabel, ax.grid, ax.legend, fig.tight_layout, fig.savefig, plt.close, empty_plot.
  - `longest_false_run_s(frame_rows: list[dict[str, Any]], *, key: str, only_with_truth_range: bool = False) -> float | None` (lines 472-505): Return the longest consecutive false run for a frame-level boolean key.
    Code map: 5 conditional branches; 1 for loops; 2 return points.
    Notable calls: sorted, ordered_rows.get, row.get.
  - `jitter_m(rows: list[dict[str, Any]], axis_keys: tuple[str, str, str]) -> float | None` (lines 508-524): Estimate jitter as combined standard deviation across three axes.
    Code map: 3 conditional branches; 1 for loops; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.asarray, np.std, any, valid_vectors.append, np.linalg.norm, row.get, np.all, np.isfinite.
  - `follow_readiness_summary(track_rows: list[dict[str, Any]], frame_rows: list[dict[str, Any]]) -> dict[str, Any]` (lines 527-621): Evaluate follow-readiness in the controller's operating range.
    Code map: 2 conditional branches; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: longest_false_run_s, percentile, all, np.mean, np.asarray, row.get.
  - `build_summary(*, run_dir: Path, manifest: dict[str, Any], track_rows: list[dict[str, Any]], frame_rows: list[dict[str, Any]], coverage_rows: list[dict[str, Any]], plot_paths: dict[str, str]) -> dict[str, Any]` (lines 624-706): Build the JSON summary for a report bundle.
    Code map: 1 return points.
    Notable calls: finite_series, manifest.get, follow_readiness_summary, isinstance, safe_mean, percentile, longest_false_run_s, jitter_m, row.get.
  - `write_summary_markdown(summary: dict[str, Any], output_path: Path) -> None` (lines 709-769): Write a concise markdown report next to the plots.
    Code map: straight-line helper logic.
    Notable calls: summary.get, output_path.write_text, '\n'.join, counts.get, rates.get, errors.get, readiness.get, timing.get.
  - `generate_report(run_dir: str | Path, *, depth_bin_size: float = 0.5, output_dir: str | Path | None = None) -> dict[str, Any]` (lines 772-871): Generate a full offline report for a run bundle.
    Code map: 2 conditional branches; 1 return points; 1 raise statements.
    Notable calls: Path.expanduser.resolve, output_dir.mkdir, resolve_artifacts, finite_xy, scatter_plot, depth_binned_error_plot, coverage_plot, build_summary, summary_json_path.write_text, write_summary_markdown, run_dir.exists, FileNotFoundError, load_json, read_typed_csv, Path.expanduser, json.dumps, Path.
  - `build_arg_parser() -> argparse.ArgumentParser` (lines 874-896): Build the CLI parser for the report generator.
    Code map: 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument.
  - `main(argv: list[str] | None = None) -> int` (lines 899-913): Run the offline report CLI.
    Code map: 1 return points.
    Notable calls: build_arg_parser, parser.parse_args, generate_report, summary.get.
- Whole-file runtime themes: uses NumPy arrays/math, defines a CLI.

### `ros2/src/drone_vision_pkg/launch/combined.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 452 bytes; 16 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: declares operator-facing launch arguments.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, LogInfo`
- Top-level functions:
  - `generate_launch_description()` (lines 5-16): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: LaunchDescription, DeclareLaunchArgument, LogInfo.

### `ros2/src/drone_vision_pkg/launch/tracking_only.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 5200 bytes; 119 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node`
  - local/project: `from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults`
- Top-level functions:
  - `generate_launch_description()` (lines 10-119): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: load_drone_launch_defaults, Node, LaunchDescription, Path.resolve, LaunchConfiguration, DeclareLaunchArgument, Path, default_arg.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_vision_pkg/launch/vslam_only.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 454 bytes; 16 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: declares operator-facing launch arguments.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, LogInfo`
- Top-level functions:
  - `generate_launch_description()` (lines 5-16): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: LaunchDescription, DeclareLaunchArgument, LogInfo.

### `ros2/src/drone_vision_pkg/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 1299 bytes; 34 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_vision_pkg`.
- Dependencies: `exec_depend:rclpy, exec_depend:launch, exec_depend:launch_ros, exec_depend:python3-numpy, exec_depend:python3-opencv, exec_depend:python3-matplotlib, exec_depend:python3-yaml, exec_depend:cv_bridge, exec_depend:std_msgs, exec_depend:geometry_msgs, exec_depend:sensor_msgs, exec_depend:mavros_msgs, exec_depend:visualization_msgs, exec_depend:drone_msgs, exec_depend:drone_control_pkg, test_depend:ament_copyright, test_depend:ament_flake8, test_depend:ament_pep257, test_depend:python3-pytest`.
- Build type: `ament_python`.

### `ros2/src/drone_vision_pkg/resource/drone_vision_pkg`

- File type: Text file.
- Tracked size: 17 bytes; 1 decoded lines.
- Purpose: drone_vision_pkg
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `ros2/src/drone_vision_pkg/setup.cfg`

- File type: INI/setup configuration.
- Tracked size: 101 bytes; 4 decoded lines.
- Purpose: [develop] script_dir=$base/lib/drone_vision_pkg [install] install_scripts=$base/lib/drone_vision_pkg
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Config sections: `develop, install`.

### `ros2/src/drone_vision_pkg/setup.py`

- File type: Python source file.
- Tracked size: 1559 bytes; 44 decoded lines.
- Purpose: Python package installer that registers package data and console script entry points.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: colcon/ament_python during build and ROS 2 console script lookup at runtime.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from setuptools import find_packages, setup; import glob; import os`
- Top-level constants/state: `package_name`.
- Whole-file runtime themes: loads or writes YAML.

### `ros2/src/drone_vision_pkg/test/test_copyright.py`

- File type: Python source file.
- Tracked size: 890 bytes; 26 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_copyright.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_copyright.main import main; import pytest`
- Top-level functions:
  - `test_copyright()` (lines 24-26): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: pytest.mark.skip, main.

### `ros2/src/drone_vision_pkg/test/test_flake8.py`

- File type: Python source file.
- Tracked size: 878 bytes; 25 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_flake8.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_flake8.main import main_with_errors; import pytest`
- Top-level functions:
  - `test_flake8()` (lines 21-25): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main_with_errors, '\n'.join.

### `ros2/src/drone_vision_pkg/test/test_mission_recording_helpers.py`

- File type: Python source file.
- Tracked size: 2846 bytes; 80 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_led_color_mapping_matches_milestone_states, test_mission_state_activity_gate, test_led_bgr_for_color_name_returns_cv2_order, test_depth_colormap_has_requested_size_and_masks_invalid_depth.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import sys`
  - third-party: `import numpy as np`
  - local/project: `from drone_vision_pkg.realsense_tracker_node import is_active_mission_state, led_bgr_for_color_name, led_color_name_for_engagement_state, make_depth_colormap`
- Top-level functions:
  - `test_led_color_mapping_matches_milestone_states() -> None` (lines 30-50): No docstring; behavior is described from its body and call sites.
    Code map: 11 assertions.
    Notable calls: led_color_name_for_engagement_state.
  - `test_mission_state_activity_gate() -> None` (lines 53-58): No docstring; behavior is described from its body and call sites.
    Code map: 5 assertions.
    Notable calls: is_active_mission_state.
  - `test_led_bgr_for_color_name_returns_cv2_order() -> None` (lines 61-63): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Runtime interactions: uses OpenCV image processing.
    Notable calls: led_bgr_for_color_name.
  - `test_depth_colormap_has_requested_size_and_masks_invalid_depth() -> None` (lines 66-80): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.array, make_depth_colormap, np.all, np.any.
- Whole-file runtime themes: uses OpenCV image processing, uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/test/test_pep257.py`

- File type: Python source file.
- Tracked size: 803 bytes; 23 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_pep257.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_pep257.main import main; import pytest`
- Top-level functions:
  - `test_pep257()` (lines 21-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main.

### `ros2/src/drone_vision_pkg/test/test_target_memory.py`

- File type: Python source file.
- Tracked size: 3887 bytes; 131 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions make_observation, test_target_memory_predicts_during_dropout_with_decay_and_uncertainty, test_target_memory_reacquires_near_prediction_with_stable_map_id, test_target_memory_prunes_stale_tracks_after_prediction_window, test_world_to_body_position_uses_inverse_ownship_rotation.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import sys; import pytest`
  - third-party: `import numpy as np`
  - local/project: `from drone_vision_pkg.target_memory import TRACK_SOURCE_DETECTED, TRACK_SOURCE_PREDICTED, TargetMemory, WorldTrackObservation, normalized_quaternion_to_rotation_matrix, world_to_body_position`
- Top-level functions:
  - `make_observation(*, track_id: int, detector_track_id: int | None = None, x_m: float, y_m: float = 0.0, z_m: float = 1.0, vx_mps: float = 0.0, vy_mps: float = 0.0, vz_mps: float = 0.0, confidence: float = 0.9, now_s: float = 0.0) -> WorldTrackObservation` (lines 19-48): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: WorldTrackObservation, np.array, np.linalg.norm.
  - `test_target_memory_predicts_during_dropout_with_decay_and_uncertainty() -> None` (lines 51-72): No docstring; behavior is described from its body and call sites.
    Code map: 6 assertions.
    Notable calls: TargetMemory, memory.update, memory.snapshot, track.output_source, pytest.approx, track.decayed_confidence, make_observation.
  - `test_target_memory_reacquires_near_prediction_with_stable_map_id() -> None` (lines 75-100): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: TargetMemory, memory.update, memory.snapshot, make_observation.
  - `test_target_memory_prunes_stale_tracks_after_prediction_window() -> None` (lines 103-112): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: TargetMemory, memory.update, memory.prune, memory.snapshot, make_observation.
  - `test_world_to_body_position_uses_inverse_ownship_rotation() -> None` (lines 115-131): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: normalized_quaternion_to_rotation_matrix, world_to_body_position, np.array, pytest.approx.
- Whole-file runtime themes: uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/test/test_tracker_resilience.py`

- File type: Python source file.
- Tracked size: 5267 bytes; 168 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines classes StubDetection, StubTrack; defines functions test_histogram_embedding_prefers_similar_appearance, test_tracked_object_reid_distance_falls_back_to_past_detections, test_select_held_track_states_caps_extrapolation_and_expires, test_norfair_reid_recovers_same_track_id_after_brief_dropout.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from pathlib import Path; import sys; import pytest`
  - third-party: `import numpy as np; from norfair import Detection, Tracker`
  - local/project: `from drone_vision_pkg.tracker_resilience import BodyTrackState, compute_lab_histogram_embedding, select_held_track_states, tracked_object_reid_distance`
- Classes:
  - `StubDetection` (lines 18-20, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `embedding`.
    - `__init__(self, embedding)` (lines 19-20): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
  - `StubTrack` (lines 23-26, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `last_detection, past_detections`.
    - `__init__(self, last_detection, past_detections = None)` (lines 24-26): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
- Top-level functions:
  - `test_histogram_embedding_prefers_similar_appearance() -> None` (lines 29-59): No docstring; behavior is described from its body and call sites.
    Code map: 5 assertions.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.zeros, compute_lab_histogram_embedding, StubTrack, StubDetection, tracked_object_reid_distance.
  - `test_tracked_object_reid_distance_falls_back_to_past_detections() -> None` (lines 62-75): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.array, StubTrack, StubDetection, tracked_object_reid_distance.
  - `test_select_held_track_states_caps_extrapolation_and_expires() -> None` (lines 78-131): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: select_held_track_states, BodyTrackState, pytest.approx, held_tracks.keys.
  - `test_norfair_reid_recovers_same_track_id_after_brief_dropout() -> None` (lines 134-168): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points; 2 assertions.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: Tracker, np.array, tracker.update, Detection, make_detection.
- Whole-file runtime themes: uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/test/test_tracking_metrics.py`

- File type: Python source file.
- Tracked size: 14988 bytes; 455 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions write_reference_video, sample, test_tracking_run_counts_prediction_bridge_and_reacquisition, test_clean_target_path_prefers_target_map_and_smooths, test_video_history_window_filters_tracks_older_than_ten_seconds, test_simulated_dropout_backtest_uses_visible_raw_motion, test_write_run_outputs_creates_csv_bundle, test_cleaned_path_csv_timestamps_start_at_zero, test_write_run_outputs_creates_mp4_when_opencv_available, test_cleaned_path_mp4_matches_reference_video_metadata and more.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import csv; import json; from pathlib import Path; import sys; import pytest`
  - third-party: `import numpy as np`
  - local/project: `from drone_vision_pkg import tracking_metrics as metrics; from drone_vision_pkg.tracking_metrics import SUMMARY_FIELDNAMES, TrackSample, TrackingMetricsRun, clean_target_path, compare_tracking_runs, simulated_dropout_backtest, write_csv, write_run_outputs`
- Top-level functions:
  - `write_reference_video(path: Path, *, fps: float, frame_count: int) -> None` (lines 26-37): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 try/except blocks; 2 assertions.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: metrics.cv2.VideoWriter_fourcc, metrics.cv2.VideoWriter, writer.isOpened, writer.release, np.full, writer.write.
  - `sample(timestamp_s: float, *, sample_kind: str, track_id: int, x_m: float, source: int = metrics.TRACK_SOURCE_DETECTED, uncertainty_m: float = 0.2, confidence: float = 0.9) -> TrackSample` (lines 40-66): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: TrackSample, np.array, np.zeros.
  - `test_tracking_run_counts_prediction_bridge_and_reacquisition() -> None` (lines 69-140): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 12 assertions.
    Notable calls: TrackingMetricsRun, run.summary, run.add_samples, run.sample_frame, sample.
  - `test_clean_target_path_prefers_target_map_and_smooths() -> None` (lines 143-158): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: clean_target_path, np.isclose, sample.
  - `test_video_history_window_filters_tracks_older_than_ten_seconds() -> None` (lines 161-190): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: clean_target_path, metrics.samples_in_history_window, metrics.cleaned_points_in_history_window, metrics.positions_in_history_window, sample, np.array.
  - `test_simulated_dropout_backtest_uses_visible_raw_motion() -> None` (lines 193-204): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: simulated_dropout_backtest, sample, pytest.approx.
  - `test_write_run_outputs_creates_csv_bundle(tmp_path: Path) -> None` (lines 207-224): No docstring; behavior is described from its body and call sites.
    Code map: 4 assertions.
    Notable calls: TrackingMetricsRun, run.add_samples, run.sample_frame, write_run_outputs, Path.exists, sample, Path.
  - `test_cleaned_path_csv_timestamps_start_at_zero(tmp_path: Path) -> None` (lines 227-254): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 context managers; 3 assertions.
    Notable calls: TrackingMetricsRun, write_run_outputs, json.loads, run.add_samples, run.sample_frame, Path.open, pytest.approx, BinOp.read_text, sample, csv.DictReader, Path.
  - `test_write_run_outputs_creates_mp4_when_opencv_available(tmp_path: Path) -> None` (lines 257-281): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 2 assertions.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: TrackingMetricsRun, write_run_outputs, Path.exists, pytest.skip, run.add_samples, run.add_ownship_position, run.sample_frame, sample, np.array, Path, Path.stat.
  - `test_cleaned_path_mp4_matches_reference_video_metadata(tmp_path: Path) -> None` (lines 284-332): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 try/except blocks; 4 assertions.
    Runtime interactions: uses OpenCV image processing.
    Notable calls: write_reference_video, TrackingMetricsRun, write_run_outputs, metrics.cv2.VideoCapture, json.loads, pytest.skip, run.add_samples, run.sample_frame, capture.release, BinOp.read_text, sample, capture.get, pytest.approx.
  - `test_find_nearest_reference_video_uses_run_timestamp(tmp_path: Path) -> None` (lines 335-364): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: recording_dir.mkdir, far.touch, near.touch, other_drone.touch, run_dir.mkdir, metrics.find_nearest_reference_video.
  - `test_postprocess_main_generates_cleaned_outputs(tmp_path: Path) -> None` (lines 367-415): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks; 1 context managers; 4 assertions.
    Runtime interactions: uses OpenCV image processing.
    Notable calls: run_dir.mkdir, write_csv, write_reference_video, metrics.postprocess_main, metrics.cv2.VideoCapture, pytest.skip, sample, metrics.sample_to_csv_row, BinOp.open, capture.release, csv.DictReader, capture.get, pytest.approx.
  - `test_compare_tracking_runs_writes_comparison_files(tmp_path: Path) -> None` (lines 418-455): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: m3_dir.mkdir, m4_dir.mkdir, m3_summary.update, m4_summary.update, write_csv, compare_tracking_runs, Path.exists, Path.
- Whole-file runtime themes: uses OpenCV image processing, uses NumPy arrays/math.

### `ros2/src/drone_vision_pkg/test/test_world_track_compare_report.py`

- File type: Python source file.
- Tracked size: 12006 bytes; 486 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions write_csv, write_manifest, test_projection_inverse_helpers_round_trip, test_generate_report_for_empty_run, test_generate_report_for_clean_hit_run, test_generate_report_for_mixed_run.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: Tests for world-track comparison helpers and reporting.
- Imports by role:
  - standard library: `from __future__ import annotations; import csv; import json; from pathlib import Path`
  - third-party: `import numpy as np`
  - ROS/runtime: `from geometry_msgs.msg import PoseStamped`
  - local/project: `from drone_vision_pkg.projection import RealsenseProjection; from drone_vision_pkg.world_track_compare_common import FRAMES_CSV_FIELDNAMES, RUN_MANIFEST_FILENAME, TRACKS_CSV_FIELDNAMES, TRACKS_CSV_FILENAME_DEFAULT; from drone_vision_pkg.world_track_compare_report import generate_report`
- Top-level functions:
  - `write_csv(path: Path, header: list[str], rows: list[list[object]]) -> None` (lines 22-28): Write a CSV file for a synthetic report test.
    Code map: 1 context managers.
    Notable calls: path.open, csv.writer, writer.writerow, writer.writerows.
  - `write_manifest(run_dir: Path, run_id: str, experiment_tag: str) -> None` (lines 31-45): Write a minimal manifest file for a synthetic run.
    Code map: straight-line helper logic.
    Notable calls: BinOp.write_text, json.dumps.
  - `test_projection_inverse_helpers_round_trip() -> None` (lines 48-79): Projection helpers should invert camera/body/world transforms.
    Code map: 8 assertions.
    Runtime interactions: handles pose messages, uses NumPy arrays/math.
    Notable calls: RealsenseProjection, PoseStamped, projection.update_pose, np.array, projection.camera_to_body_frame, projection.body_to_camera_frame, np.allclose, projection.body_to_world, projection.world_to_body, projection.world_to_camera, np.isclose, np.linalg.norm.
  - `test_generate_report_for_empty_run(tmp_path: Path) -> None` (lines 82-155): The report should handle runs with only frame rows.
    Code map: 6 assertions.
    Notable calls: run_dir.mkdir, write_manifest, write_csv, generate_report, BinOp.exists.
  - `test_generate_report_for_clean_hit_run(tmp_path: Path) -> None` (lines 158-335): A clean run should yield a ready-for-follow summary.
    Code map: 3 assertions.
    Notable calls: run_dir.mkdir, write_manifest, write_csv, generate_report.
  - `test_generate_report_for_mixed_run(tmp_path: Path) -> None` (lines 338-486): A mixed run should preserve misses, mismatches, and dropout stats.
    Code map: 3 assertions.
    Notable calls: run_dir.mkdir, write_manifest, write_csv, generate_report.
- Whole-file runtime themes: handles pose messages, uses NumPy arrays/math.

## Active ROS 2 package: ros2_poselib

### `ros2/src/ros2_poselib/__init__.py`

- File type: Python source file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Tracked non-text asset used by documentation, runtime, or dependency setup.
- Main responsibilities: Provides executable or importable Python behavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `ros2/src/ros2_poselib/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 801 bytes; 23 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `ros2_poselib`.
- Dependencies: `exec_depend:rclpy, exec_depend:python3-numpy, exec_depend:python3-scipy, exec_depend:geometry_msgs, test_depend:ament_copyright, test_depend:ament_flake8, test_depend:ament_pep257, test_depend:python3-pytest`.
- Build type: `ament_python`.

### `ros2/src/ros2_poselib/resource/ros2_poselib`

- File type: Text file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Tracked non-text asset used by documentation, runtime, or dependency setup.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 0.

### `ros2/src/ros2_poselib/ros2_poselib/__init__.py`

- File type: Python source file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Tracked non-text asset used by documentation, runtime, or dependency setup.
- Main responsibilities: Provides executable or importable Python behavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `ros2/src/ros2_poselib/ros2_poselib/poselib/__init__.py`

- File type: Python source file.
- Tracked size: 45 bytes; 1 decoded lines.
- Purpose: from ros2_poselib.poselib._pose import Pose3D
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - local/project: `from ros2_poselib.poselib._pose import Pose3D`

### `ros2/src/ros2_poselib/ros2_poselib/poselib/_pose.py`

- File type: Python source file.
- Tracked size: 5074 bytes; 140 decoded lines.
- Purpose: from typing import Any import numpy as np from numpy import floating from scipy.spatial.transform import Rotation as R from dataclasses import dataclass from rclpy.time import Time from geometry_msgs.msg import PoseStamped @dataclass class Pose3D: """ Repre...
- Main responsibilities: defines classes Pose3D; defines functions generate_rand_pose_msg.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from typing import Any; from dataclasses import dataclass`
  - third-party: `import numpy as np; from numpy import floating; from scipy.spatial.transform import Rotation as R`
  - ROS/runtime: `from rclpy.time import Time; from geometry_msgs.msg import PoseStamped`
- Classes:
  - `Pose3D` (lines 13-114, bases: `object`): Represents a 3D pose with a timestamp, frame identifier, position, and orientation.
    State touched: `frame_id, position, timestamp, orientation`.
    - `from_msg(cls, msg: PoseStamped)` (lines 36-51): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages, uses NumPy arrays/math.
      Notable calls: cls, Time.from_msg, np.array, R.from_quat.
    - `to_msg(self) -> PoseStamped` (lines 53-64): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: handles pose messages.
      Notable calls: PoseStamped, self.timestamp.to_msg, self.orientation.as_quat.
    - `pos_diff(self, pose: 'Pose3D') -> floating[Any]` (lines 66-75): Returns the Euclidean distance between the current position and the provided pose's position.' Args: pose (Pose3D): The pose to which the difference is calculated.
      Code map: 1 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: np.linalg.norm.
    - `orient_diff(self, pose: 'Pose3D') -> floating[Any]` (lines 77-86): Returns the difference in orientation between the current pose and the provided pose.
      Code map: 1 return points.
      Notable calls: self.orientation.inv.
    - `is_near(self, pose: 'Pose3D', pos_threshold: float = 1e-06, orientation_threshold: float = 0.3) -> bool` (lines 88-114): Compares two poses to check if they are close enough to each other.
      Code map: 1 conditional branches; 1 return points; 1 raise statements.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: np.linalg.norm, orientation_diff.magnitude, ValueError, self.orientation.inv.
- Top-level functions:
  - `generate_rand_pose_msg() -> PoseStamped` (lines 117-140): Generates a random PoseStamped message with randomized position and orientation values.
    Code map: 1 return points.
    Runtime interactions: handles pose messages, uses NumPy arrays/math.
    Notable calls: PoseStamped, Time.to_msg, np.random.random_sample, R.random.as_quat, Time, R.random, np.random.randint.
- Whole-file runtime themes: handles pose messages, uses NumPy arrays/math.

### `ros2/src/ros2_poselib/setup.cfg`

- File type: INI/setup configuration.
- Tracked size: 93 bytes; 4 decoded lines.
- Purpose: [develop] script_dir=$base/lib/ros2_poselib [install] install_scripts=$base/lib/ros2_poselib
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Config sections: `develop, install`.

### `ros2/src/ros2_poselib/setup.py`

- File type: Python source file.
- Tracked size: 658 bytes; 25 decoded lines.
- Purpose: Python package installer that registers package data and console script entry points.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: colcon/ament_python during build and ROS 2 console script lookup at runtime.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from setuptools import find_packages, setup`
- Top-level constants/state: `package_name`.

### `ros2/src/ros2_poselib/test/__init__.py`

- File type: Python source file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: Provides executable or importable Python behavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `ros2/src/ros2_poselib/test/test_copyright.py`

- File type: Python source file.
- Tracked size: 962 bytes; 25 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_copyright.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_copyright.main import main; import pytest`
- Top-level functions:
  - `test_copyright()` (lines 23-25): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: pytest.mark.skip, main.

### `ros2/src/ros2_poselib/test/test_flake8.py`

- File type: Python source file.
- Tracked size: 884 bytes; 25 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_flake8.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_flake8.main import main_with_errors; import pytest`
- Top-level functions:
  - `test_flake8()` (lines 21-25): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main_with_errors, '\n'.join.

### `ros2/src/ros2_poselib/test/test_pep257.py`

- File type: Python source file.
- Tracked size: 803 bytes; 23 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_pep257.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_pep257.main import main; import pytest`
- Top-level functions:
  - `test_pep257()` (lines 21-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main.

## Archived legacy stack

### `archive/legacy_stack/README.md`

- File type: Markdown documentation.
- Tracked size: 955 bytes; 35 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Legacy Stack Archive
    - Why It Exists
    - What Was Archived
    - How To Use It
- Opening content: This directory holds the parts of `cdrone_control` that were removed from the active workspace during the VIO-first rehaul.

### `archive/legacy_stack/docker/jetson/Dockerfile`

- File type: Dockerfile.
- Tracked size: 1101 bytes; 40 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Docker instructions:
  - `ARG IMAGE_NAME=dev:mavros2`
  - `FROM ${IMAGE_NAME}`
  - `RUN apt-get update && apt-get install -y \`
  - `RUN pip install \`
  - `COPY ros2/src/drone_msgs /root/ros2_ws/src/drone_msgs`
  - `COPY ros2/src/drone_bringup /root/ros2_ws/src/drone_bringup`
  - `COPY ros2/src/drone_behavior_pkg /root/ros2_ws/src/drone_behavior_pkg`
  - `COPY ros2/src/drone_control_pkg /root/ros2_ws/src/drone_control_pkg`
  - `COPY ros2/src/drone_light_pkg /root/ros2_ws/src/drone_light_pkg`
  - `COPY ros2/src/drone_vision_pkg /root/ros2_ws/src/drone_vision_pkg`
  - `WORKDIR /root/ros2_ws`
  - `RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \`
  - `RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \`
  - `CMD ["bash"]`

### `archive/legacy_stack/docker/jetson/docker-compose.yml`

- File type: YAML configuration file.
- Tracked size: 652 bytes; 27 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- YAML shape: `dict` with 32 documented key/value entries.
- Key map:
  - `services`: mapping
  - `services.mavros_container`: mapping
  - `services.mavros_container.image`: `dev:mavros2`
  - `services.mavros_container.privileged`: `True`
  - `services.mavros_container.ipc`: `host`
  - `services.mavros_container.pid`: `host`
  - `services.mavros_container.volumes`: list[2]
  - `services.mavros_container.volumes[0]`: `/dev:/dev`
  - `services.mavros_container.volumes[1]`: `./mavros_entrypoint.sh:/mavros_entrypoint.sh`
  - `services.mavros_container.entrypoint`: list[3]
  - `services.mavros_container.entrypoint[0]`: `/bin/bash`
  - `services.mavros_container.entrypoint[1]`: `-c`
  - `services.mavros_container.entrypoint[2]`: `/mavros_entrypoint.sh`
  - `services.autonomy_container`: mapping
  - `services.autonomy_container.image`: `xtrana:ros2`
  - `services.autonomy_container.runtime`: `nvidia`
  - `services.autonomy_container.privileged`: `True`
  - `services.autonomy_container.ipc`: `host`
  - `services.autonomy_container.pid`: `host`
  - `services.autonomy_container.depends_on`: list[1]
  - `services.autonomy_container.depends_on[0]`: `mavros_container`
  - `services.autonomy_container.environment`: list[3]
  - `services.autonomy_container.environment[0]`: `NVIDIA_DRIVER_CAPABILITIES=all`
  - `services.autonomy_container.environment[1]`: `SOURCE_MODE=csi`
  - `services.autonomy_container.environment[2]`: `SCENARIO=intercept_illuminate_v1`
  - `services.autonomy_container.volumes`: list[2]
  - `services.autonomy_container.volumes[0]`: `/dev:/dev`
  - `services.autonomy_container.volumes[1]`: `./entrypoint.sh:/entrypoint.sh`
  - `services.autonomy_container.entrypoint`: list[3]
  - `services.autonomy_container.entrypoint[0]`: `/bin/bash`
  - `services.autonomy_container.entrypoint[1]`: `-c`
  - `services.autonomy_container.entrypoint[2]`: `/entrypoint.sh`

### `archive/legacy_stack/docker/jetson/entrypoint.sh`

- File type: Shell script.
- Tracked size: 342 bytes; 15 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Shell safety flags: `set -e`.
- Command map: `source(2), set(1), sleep(1), ros2(1)`.

### `archive/legacy_stack/docs/FINAL_INSTRUCTIONS.md`

- File type: Markdown documentation.
- Tracked size: 12407 bytes; 432 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - MAVROS Keyboard Teleop - Final Instructions
    - 📋 Quick Reference
      - What We Built
      - Why MAVROS Instead of ros2_px4_teleop_example?
    - 🚀 Daily Operation (After Initial Setup)
      - Start System (3 Terminals)
      - Control Your Drone
      - Stop System
    - 📁 File Locations
      - Key Configuration Files
      - Documentation Files
    - ⚠️ First Time Testing? READ THIS
    - 🔧 Troubleshooting Quick Reference
      - MAVROS Won't Connect
  - Check mavlink-router service (should be active)
  - Restart mavlink-router if needed
  - Check USB connection to FC
  - Verify MAVROS config uses UDP
- Opening content: **System:** Jetson Orin Nano + ARK PAB + ARKV6X + PX4 1.16.0 **Method:** MAVROS (MAVLink) - No external modes needed **Status:** Ready for props-off testing

### `archive/legacy_stack/docs/JETSON_PX4_SETUP.md`

- File type: Markdown documentation.
- Tracked size: 9273 bytes; 371 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - cdrone_control + PX4 Setup Guide (MAVROS-based)
    - 🔄 Differences from ros2_px4_teleop_example
    - ✅ What We Learned from teleop_example That Still Applies
      - 1. **Serial Device Paths**
      - 2. **Branch Alignment**
      - 3. **Multiple Agent Conflicts**
      - 4. **PX4 Configuration**
    - 🚀 Quick Start: cdrone_control
      - Setup (One-Time)
      - Running MAVROS (Every Time)
  - Launch MAVROS only (test connection)
      - Full Autonomy Stack
  - Launch full stack (MAVROS + control + vision + behavior)
    - 📋 Comparison: What Works Now vs Before
      - ✅ Confirmed Working (from teleop testing):
      - ⚠️ What's Different:
    - 🔧 Configuration Files to Review
      - PX4 Parameters (set in QGC if needed):
- Opening content: **Hardware:** Jetson Orin Nano + ARK Jetson PAB Carrier + ARKV6X + PX4 1.16.0 **Communication:** MAVROS (MAVLink-ROS2 bridge) **Advantage:** Works without PX4 external modes - uses standard MAVLink

### `archive/legacy_stack/docs/MAVROS_CONTROL_GUIDE.md`

- File type: Markdown documentation.
- Tracked size: 9067 bytes; 335 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - MAVROS Control Capabilities for cdrone_control
    - 🔌 System Architecture
    - ✅ Confirmed Capabilities
      - 1. **Arming/Disarming** ✓
  - In your code
  - Arm
  - Disarm
      - 2. **Mode Changes (Including STABILIZED)** ✓
  - In your code
  - Set STABILIZED mode
  - Set OFFBOARD mode
      - 3. **Velocity Control** ✓
        - Layer 1: mavros_velocity_node (Safety Wrapper)
        - Layer 2: Direct MAVROS
      - 4. **Position Control** ✓
    - 🆕 New Keyboard Teleop Node
      - Features:
      - Build and Run:
- Opening content: **Hardware:** Jetson Orin Nano + ARK PAB Carrier + ARKV6X + PX4 1.16.0 **Communication:** MAVROS (MAVLink-ROS2 bridge) via mavlink-router

### `archive/legacy_stack/docs/ark_orin_multi_drone_plan.md`

- File type: Markdown documentation.
- Tracked size: 8348 bytes; 188 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - ARK Orin Multi-Drone Stereo Tracking and Sequential Illumination Plan
    - Summary
    - Scope and Success Criteria
    - Architecture
      - Nodes
      - Data Flow
    - Public APIs / Interfaces / Types
      - New ROS2 Message Package
      - ROS Topics
      - Config and Scenario Interfaces
    - File-Level Plan
      - Create
      - Modify
    - Core Algorithms and Behaviors
      - Stereo Tracker Pipeline
      - Target Ranking (Inbound+Nearest)
      - Scenario Engine
      - Engagement Completion Rule
- Opening content: `TargetTrack.msg` - `builtin_interfaces/Time stamp` - `int32 track_id` - `float32 x_b_m` - `float32 y_b_m` - `float32 z_b_m` - `float32 vx_b_mps` - `float32 vy_b_mps` - `float32 vz_b_mps` - `float32 distance_m` - `float32 confidence` - `float32 bbox_area_px` - `bool inbound`

### `archive/legacy_stack/docs/cdrone_control_plan.md`

- File type: Markdown documentation.
- Tracked size: 11786 bytes; 233 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - ARK Orin Multi-Drone Stereo Tracking and Sequential Illumination Plan
    - Summary
    - Scope and Success Criteria
    - Architecture (Decision-Complete)
      - Nodes
      - Data Flow
    - Public APIs / Interfaces / Types
      - New ROS2 Message Package
      - ROS Topics
      - Config and Scenario Interfaces
    - File-Level Implementation Plan
      - Create
      - Modify
    - Core Algorithms and Behavior Definitions
      - Stereo Tracker Pipeline
      - Target Ranking (Inbound+Nearest)
      - Scenario Engine (YAML + Registry)
      - Engagement Completion Rule
- Opening content: `TargetTrack.msg` - `builtin_interfaces/Time stamp` - `int32 track_id` - `float32 x_b_m` - `float32 y_b_m` - `float32 z_b_m` - `float32 vx_b_mps` - `float32 vy_b_mps` - `float32 vz_b_mps` - `float32 distance_m` - `float32 confidence` - `float32 bbox_area_px` - `bool inbound`

### `archive/legacy_stack/docs/demo.md`

- File type: Markdown documentation.
- Tracked size: 4402 bytes; 185 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - ALTCTL Demo Flight
    - What It Does
    - Important Notes
    - Preflight
    - Build
    - Launch
    - Start The Demo
    - Abort The Demo
    - Monitor State
    - Parameter Overrides
    - Recommended First Run
    - Troubleshooting
    - Related Files
- Opening content: This document explains how to run the non-invasive automated demo flight that uses the existing `ALTCTL + MANUAL_CONTROL` path. It does not use the autonomy stack and it does not depend on `OFFBOARD`.

### `archive/legacy_stack/docs/intellisense_differences.md`

- File type: Markdown documentation.
- Tracked size: 9845 bytes; 330 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Intel RealSense D435 vs D435i vs D455
    - Short Bottom Line
    - Why These Three Get Confused
    - Hardware Differences
      - 1. IMU
      - 2. Depth Range Behavior
      - 3. Minimum Distance
      - 4. Stereo Baseline and Geometry
      - 5. RGB Camera and FOV Match
      - 6. Physical Size
    - Software Stack Differences
      - 1. Base SDK Support
      - 2. Available Data Streams
      - 3. Visual-Inertial Readiness
      - 4. Firmware and Version Sensitivity
    - Development Differences
      - 1. Basic Bring-Up
      - 2. SLAM / VIO Development
- Opening content: Source references: - D435 product page: https://www.intelrealsense.com/depth-camera-d435/ - D435 Intel specs page: https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html - D435i product page: https://www.realsenseai.co...

### `archive/legacy_stack/docs/l2_auto_plan.md`

- File type: Markdown documentation.
- Tracked size: 8560 bytes; 102 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Level 2 Autonomy, Monitoring Dashboard, and Weekly Roadmap
    - Summary
    - Problems To Solve Before Level 2
    - Key Changes
    - Public Interfaces
    - Weekly Roadmap
    - Test Plan
    - Assumptions and Defaults

### `archive/legacy_stack/docs/l2_single_drone_intercept_plan.md`

- File type: Markdown documentation.
- Tracked size: 7483 bytes; 211 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Level 2 Single-Drone Predictive Intercept Plan
    - Summary
    - Current Gaps
    - Target Architecture
      - Perception and estimation
      - Guidance and control
      - Behavior state machine
    - Interface Changes
      - New messages
      - `PredictedTrack` fields
      - New topics
    - Implementation Steps
      - 1. Split raw tracking from prediction
      - 2. Add `track_predictor_node`
      - 3. Update behavior ranking
      - 4. Replace reactive approach math
      - 5. Extend engagement state and dashboard visibility
    - Safety Constraints
- Opening content: This document defines the next autonomy upgrade for one drone only. The goal is to replace the current reactive follow behavior with predictive interception while keeping the existing Level 2 infrastructure intact:

### `archive/legacy_stack/docs/l2_swarm_no_comms_plan.md`

- File type: Markdown documentation.
- Tracked size: 8003 bytes; 233 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Level 2 Swarm Scaling Plan Without Inter-Drone Communication
    - Summary
    - Problem Framing
    - Design Principles
    - Swarm Policy
      - Shared mission assumptions
      - Patrol geometry
      - Patrol state machine
    - Local Target Handling
      - Target classes
      - Friendly detection
      - Engagement rules
      - Claiming a target without communication
      - Breach override
    - Required Interface and Message Changes
      - Track classification and intercept data
      - Swarm configuration parameters
    - Implementation Steps
- Opening content: This document defines how the single-drone predictive intercept stack should scale to multiple defensive drones when the swarm has no runtime communication between agents. Each drone runs the same software, but with its own `drone_id` and a small amount of preloaded identity/c...

### `archive/legacy_stack/docs/l2_test1.md`

- File type: Markdown documentation.
- Tracked size: 12005 bytes; 439 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Level 2 Test 1 Implementation Summary
    - Scope
    - High-Level Result
    - Files Added
      - New messages
      - New monitor package
      - New behavior/state-machine files
      - New control files
      - New launch file
    - Message and Interface Changes
      - `drone_msgs`
    - Behavior Layer Changes
      - State machine migration
      - New behaviors
      - Updated behaviors
      - `engagement_manager_node.py`
      - `math_utils.py`
    - Health and Safety Changes
- Opening content: This document summarizes the code changes implemented for the first Level 2 autonomy integration pass. The goal of this pass was to move the repo from:

### `archive/legacy_stack/docs/l2_test1_todo.md`

- File type: Markdown documentation.
- Tracked size: 5773 bytes; 166 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Level 2 Test 1 TODO
    - Immediate Build/Environment Tasks
    - VIO Integration Tasks
    - Topic and Namespacing Validation
    - Perception Validation
    - Autonomy Logic Validation
    - Health and Alert Validation
    - Dashboard Validation
    - Bench Test Sequence
    - Flight Test Prerequisites
    - Known Code Cleanup Still Needed
    - Swarm/Level 4 Work Not Yet Implemented
    - Suggested First Commands On A Real Target Machine
  - Build
  - Bring up stack
  - Bring up remote dashboard
  - Inspect monitor output
- Opening content: - Install or source a real ROS 2 Humble environment on the target machine. - Run a full `colcon build` for: - `drone_msgs` - `drone_behavior_pkg` - `drone_control_pkg` - `drone_light_pkg` - `drone_monitor_pkg` - `drone_vision_pkg` - `drone_bringup` - Fix any ROS message-genera...

### `archive/legacy_stack/docs/prompts.txt`

- File type: Plain text file.
- Tracked size: 757 bytes; 4 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 4 total, 3 non-empty.
  - Example: `Spawn a subagent to go through this repo and create a memory.md for remembering what this repo used to have, I'm going for a complete rehaul. The second subagent can figure out a plan on how to rehaul this repo to get...`
  - Example: `something like this but specific to our drone and setup (this jetson carrier board is still esc/motorless so we will test later.`
  - Example: `A third subagent can go through the plan and verify that the plan is actually viable by cross checking with online sources that there are no version mismatches or bugs that can't be bypassed.`

### `archive/legacy_stack/docs/props_off_test.md`

- File type: Markdown documentation.
- Tracked size: 6916 bytes; 281 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Props-Off Bench Testing Guide
    - Safety First
    - Quick Start
    - What Changed
    - MAVLink Routing
    - Pre-Test Checklist
      - Hardware
      - Software
    - Test Procedure
      - Phase 1: MAVROS Connection
      - Phase 2: Keyboard Teleop
      - Phase 3: Verify Manual-Control Messages
      - Phase 4: Mode Change Test
      - Phase 5: Arming Test
      - Phase 6: Stick Response Test
      - Phase 7: Optional STABILIZED Check
    - Post-Test Checklist
    - Troubleshooting
- Opening content: **Current recommended indoor bench method:** use `keyboard_teleop_node` in `ALTCTL` via MAVROS `ManualControl`.

### `archive/legacy_stack/docs/props_on_test.md`

- File type: Markdown documentation.
- Tracked size: 4695 bytes; 188 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Props-On Flight Test Guide
    - Safety Rules
    - Before You Do This
    - Hardware Checklist
    - Software Checklist
    - Launch
    - Mode and Arming Sequence
    - First Takeoff
    - Basic Control Test
    - Landing
    - Abort / Stop Criteria
    - What Not To Do Yet
    - After Flight
    - References
- Opening content: **Scope:** first low-altitude manual flight with propellers installed.

### `archive/legacy_stack/docs/vio_todo.md`

- File type: Markdown documentation.
- Tracked size: 1374 bytes; 23 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - VIO TODO
    - Goal
    - Current State
    - Integration Choices
    - TODO

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/config/scenarios/intercept_illuminate_v1.yaml`

- File type: YAML configuration file.
- Tracked size: 266 bytes; 14 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 13 documented key/value entries.
- Key map:
  - `scenario_id`: `intercept_illuminate_v1`
  - `states`: list[6]
  - `states[0]`: `SEARCH`
  - `states[1]`: `ALIGN`
  - `states[2]`: `APPROACH`
  - `states[3]`: `FOLLOW_STANDOFF`
  - `states[4]`: `LOST_TARGET_HOLD`
  - `states[5]`: `FAILSAFE_HOLD`
  - `policy`: mapping
  - `policy.ranking`: `inbound_nearest_hybrid`
  - `policy.follow_distance_m`: `3.0`
  - `policy.follow_distance_tolerance_m`: `0.4`
  - `policy.target_cooldown_s`: `5.0`

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/config/scenarios/track_follow_v1.yaml`

- File type: YAML configuration file.
- Tracked size: 258 bytes; 14 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 13 documented key/value entries.
- Key map:
  - `scenario_id`: `track_follow_v1`
  - `states`: list[6]
  - `states[0]`: `SEARCH`
  - `states[1]`: `ALIGN`
  - `states[2]`: `APPROACH`
  - `states[3]`: `FOLLOW_STANDOFF`
  - `states[4]`: `LOST_TARGET_HOLD`
  - `states[5]`: `FAILSAFE_HOLD`
  - `policy`: mapping
  - `policy.ranking`: `inbound_nearest_hybrid`
  - `policy.follow_distance_m`: `3.0`
  - `policy.follow_distance_tolerance_m`: `0.4`
  - `policy.target_cooldown_s`: `5.0`

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/__init__.py`

- File type: Python source file.
- Tracked size: 31 bytes; 1 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behavior_registry.py`

- File type: Python source file.
- Tracked size: 1577 bytes; 46 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes BehaviorAction, FailsafeHoldBehavior; defines functions build_behavior_registry.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass; from typing import Dict, Protocol`
  - local/project: `from drone_behavior_pkg.behaviors.align import AlignBehavior; from drone_behavior_pkg.behaviors.approach import ApproachBehavior; from drone_behavior_pkg.behaviors.follow_standoff import FollowStandoffBehavior; from drone_behavior_pkg.behaviors.lost_target_hold import LostTargetHoldBehavior; from drone_behavior_pkg.behaviors.search import SearchBehavior`
- Classes:
  - `BehaviorAction` (lines 13-17, bases: `Protocol`): No class docstring; role is inferred from methods and base classes.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 16-17): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
  - `FailsafeHoldBehavior` (lines 21-35, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `name`.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 24-35): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: manager.publish_zero_velocity, manager.publish_light, manager.clear_active_target.
- Top-level functions:
  - `build_behavior_registry() -> Dict[str, BehaviorAction]` (lines 38-46): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: SearchBehavior, AlignBehavior, ApproachBehavior, FollowStandoffBehavior, LostTargetHoldBehavior, FailsafeHoldBehavior.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/__init__.py`

- File type: Python source file.
- Tracked size: 31 bytes; 1 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/advance_queue.py`

- File type: Python source file.
- Tracked size: 433 bytes; 15 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes AdvanceQueueBehavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
- Classes:
  - `AdvanceQueueBehavior` (lines 7-15, bases: `object`): No class docstring; role is inferred from methods and base classes.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 10-15): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: manager.mark_active_target_completed, manager.publish_zero_velocity, manager.publish_light, manager.clear_active_target.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/align.py`

- File type: Python source file.
- Tracked size: 950 bytes; 28 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes AlignBehavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
- Classes:
  - `AlignBehavior` (lines 7-28, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `name`.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 10-28): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Notable calls: manager.get_active_target, manager.compute_obstacle_blocked, manager.publish_align_for_target, manager.publish_light, manager.publish_zero_velocity, manager.active_target_lost_for.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/approach.py`

- File type: Python source file.
- Tracked size: 1058 bytes; 31 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes ApproachBehavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
- Classes:
  - `ApproachBehavior` (lines 7-31, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `name`.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 10-31): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 5 return points.
      Notable calls: manager.get_active_target, manager.compute_obstacle_blocked, manager.publish_velocity_for_target, manager.publish_light, manager.publish_zero_velocity, manager.active_target_lost_for.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/follow_standoff.py`

- File type: Python source file.
- Tracked size: 936 bytes; 28 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes FollowStandoffBehavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
- Classes:
  - `FollowStandoffBehavior` (lines 7-28, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `name`.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 10-28): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 4 return points.
      Notable calls: manager.get_active_target, manager.compute_obstacle_blocked, manager.publish_velocity_for_target, manager.publish_light, manager.publish_zero_velocity, manager.active_target_lost_for.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/illuminate.py`

- File type: Python source file.
- Tracked size: 972 bytes; 25 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes IlluminateBehavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
- Classes:
  - `IlluminateBehavior` (lines 7-25, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `name`.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 10-25): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 4 return points.
      Notable calls: manager.get_active_target, manager.publish_velocity_for_target, manager.publish_light, manager.publish_zero_velocity, manager.active_target_lost_for.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/lost_target_hold.py`

- File type: Python source file.
- Tracked size: 610 bytes; 21 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes LostTargetHoldBehavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
- Classes:
  - `LostTargetHoldBehavior` (lines 7-21, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `name`.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 10-21): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 3 return points.
      Notable calls: manager.publish_zero_velocity, manager.publish_light, manager.get_active_target, manager.active_target_lost_for, manager.clear_active_target.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/search.py`

- File type: Python source file.
- Tracked size: 498 bytes; 17 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes SearchBehavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass`
- Classes:
  - `SearchBehavior` (lines 7-17, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `name`.
    - `step(self, manager: 'EngagementManagerNode', now_s: float) -> str` (lines 10-17): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: manager.select_target, manager.publish_zero_velocity, manager.publish_light, manager.set_active_target.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/engagement_manager_node.py`

- File type: Python source file.
- Tracked size: 19357 bytes; 465 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes EngagementManagerNode; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from pathlib import Path; from typing import Dict, Optional; from ament_index_python.packages import get_package_share_directory`
  - third-party: `import yaml`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped, TwistStamped; from mavros_msgs.msg import State; from rclpy.node import Node; from std_msgs.msg import Bool`
  - local/project: `from drone_behavior_pkg.behavior_registry import build_behavior_registry; from drone_behavior_pkg.math_utils import TrackSnapshot, clamp, compute_velocity_command_details, score_track, target_bearing_rad; from drone_behavior_pkg.topic_utils import cdrone_topic, join_topic; from drone_msgs.msg import EngagementState, LightCommand, TargetTrack, TargetTrackArray`
- Classes:
  - `EngagementManagerNode` (lines 26-449, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, mavros_namespace, control_rate_hz, follow_distance_m, follow_distance_tolerance_m, align_yaw_tolerance_rad, target_cooldown_s, min_safe_distance_m, lost_target_timeout_s, reacquire_timeout_s, vio_timeout_s, track_gc_s, kp_xy, kp_z, kp_yaw, max_vel_xy_mps, max_vel_z_mps, max_yaw_rate_rps, obstacle_forward_distance_m, obstacle_lateral_band_m, obstacle_vertical_band_m, light_default_intensity, tracks_topic, engagement_state_topic, cmd_vel_topic, light_cmd_topic, estop_topic, estop_reset_topic, autonomy_enable_topic, local_pose_topic, vio_pose_topic, mavros_state_topic, scenario, scenario_id, state, behaviors, tracks, cooldown_until, active_track_id, active_target_lost_start_s` plus more.
    - `__init__(self) -> None` (lines 27-192): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages, handles velocity setpoints, handles target track messages, loads or writes YAML.
      Notable calls: super.__init__, self.declare_parameter, join_topic, Path, self._load_scenario, build_behavior_registry, PoseStamped, State, self.create_subscription, self.create_publisher, self.create_timer, str.strip, cdrone_topic, self.scenario.get, self.get_parameter, get_package_share_directory.
    - `_load_scenario(self, path: Path) -> dict` (lines 194-211): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 context managers; 2 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: path.exists, path.open, yaml.safe_load.
    - `now_s(self) -> float` (lines 213-214): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `tracks_callback(self, msg: TargetTrackArray) -> None` (lines 216-241): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 2 for loops.
      Runtime interactions: handles target track messages.
      Notable calls: self.now_s, TrackSnapshot, seen_ids.add, self.tracks.keys, self.tracks.pop.
    - `estop_callback(self, msg: Bool) -> None` (lines 243-246): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
    - `estop_reset_callback(self, msg: Bool) -> None` (lines 248-254): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches.
    - `autonomy_enable_callback(self, msg: Bool) -> None` (lines 256-257): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `local_pose_callback(self, msg: PoseStamped) -> None` (lines 259-260): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
    - `vio_pose_callback(self, _msg: PoseStamped) -> None` (lines 262-263): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `mavros_state_callback(self, msg: State) -> None` (lines 265-266): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `select_target(self, now_s: float) -> Optional[TrackSnapshot]` (lines 268-282): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 for loops; 2 return points.
      Notable calls: self.tracks.values, candidates.sort, self.cooldown_until.get, candidates.append, score_track.
    - `set_active_target(self, track_id: int, now_s: float) -> None` (lines 284-288): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `clear_active_target(self) -> None` (lines 290-292): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `mark_active_target_completed(self, now_s: float) -> None` (lines 294-300): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
    - `get_active_target(self, now_s: float) -> Optional[TrackSnapshot]` (lines 302-311): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 3 return points.
      Notable calls: self.tracks.get.
    - `active_target_lost_for(self, now_s: float) -> float` (lines 313-316): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
    - `publish_zero_velocity(self) -> None` (lines 318-321): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages, handles velocity setpoints.
      Notable calls: TwistStamped, self.cmd_pub.publish.
    - `publish_light(self, enabled: bool, intensity: float, strobe_hz: float) -> None` (lines 323-329): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: LightCommand, self.light_pub.publish, clamp.
    - `publish_velocity_for_target(self, target: TrackSnapshot, desired_distance_m: float) -> None` (lines 331-353): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages, handles velocity setpoints.
      Notable calls: compute_velocity_command_details, TwistStamped, self.cmd_pub.publish.
    - `publish_align_for_target(self, target: TrackSnapshot) -> float` (lines 355-365): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: publishes messages, handles velocity setpoints.
      Notable calls: target_bearing_rad, clamp, TwistStamped, self.cmd_pub.publish.
    - `compute_obstacle_blocked(self, active_target: Optional[TrackSnapshot]) -> bool` (lines 367-380): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 1 for loops; 2 return points.
      Notable calls: self.tracks.values, self.now_s.
    - `autonomy_block_reason(self, now_s: float) -> str` (lines 382-395): No docstring; behavior is described from its body and call sites.
      Code map: 6 conditional branches; 7 return points.
    - `_publish_engagement_state(self, now_s: float) -> None` (lines 397-416): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: publishes messages.
      Notable calls: EngagementState, self.get_active_target, self.state_pub.publish, target_bearing_rad.
    - `control_loop(self) -> None` (lines 418-449): No docstring; behavior is described from its body and call sites.
      Code map: 6 conditional branches; 1 for loops.
      Notable calls: self.now_s, self.compute_obstacle_blocked, self.autonomy_block_reason, self._publish_engagement_state, self.cooldown_until.keys, self.get_active_target, self.behaviors.step, self.cooldown_until.pop, self.publish_zero_velocity, self.publish_light, self.behaviors.get, behavior.step.
- Top-level functions:
  - `main(args = None) -> None` (lines 452-461): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks.
    Notable calls: rclpy.init, EngagementManagerNode, rclpy.spin, node.destroy_node, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages, handles velocity setpoints, handles target track messages, loads or writes YAML.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/health_monitor_node.py`

- File type: Python source file.
- Tracked size: 5629 bytes; 148 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes HealthMonitorNode; defines functions main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from typing import Optional`
  - ROS/runtime: `import rclpy; from rclpy.node import Node; from geometry_msgs.msg import PoseStamped; from std_msgs.msg import Bool`
  - local/project: `from drone_behavior_pkg.topic_utils import cdrone_topic, join_topic; from drone_msgs.msg import EngagementState, TargetTrackArray`
- Classes:
  - `HealthMonitorNode` (lines 14-132, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, mavros_namespace, check_rate_hz, tracks_timeout_s, engagement_timeout_s, vio_timeout_s, startup_grace_s, tracks_topic, engagement_state_topic, estop_topic, autonomy_enable_topic, vio_pose_topic, last_tracks_time_s, last_engagement_time_s, last_vio_time_s, last_estop_state, start_time_s, autonomy_enabled, tracks_sub, engagement_sub, vio_sub, autonomy_enable_sub, estop_pub, timer, declare_parameter, now_s, create_subscription, tracks_callback, engagement_callback, vio_callback, autonomy_enable_callback, create_publisher, create_timer, check_health, get_parameter, get_logger, get_clock`.
    - `__init__(self) -> None` (lines 15-79): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages, handles target track messages.
      Notable calls: super.__init__, self.declare_parameter, join_topic, self.now_s, self.create_subscription, self.create_publisher, self.create_timer, str.strip, cdrone_topic, self.get_parameter.
    - `now_s(self) -> float` (lines 81-82): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `tracks_callback(self, _msg: TargetTrackArray) -> None` (lines 84-85): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles target track messages.
      Notable calls: self.now_s.
    - `engagement_callback(self, _msg: EngagementState) -> None` (lines 87-88): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `vio_callback(self, _msg: PoseStamped) -> None` (lines 90-91): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `autonomy_enable_callback(self, msg: Bool) -> None` (lines 93-94): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `check_health(self) -> None` (lines 96-132): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 3 return points.
      Runtime interactions: publishes messages.
      Notable calls: self.now_s, Bool, self.estop_pub.publish.
- Top-level functions:
  - `main(args = None) -> None` (lines 135-144): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks.
    Notable calls: rclpy.init, HealthMonitorNode, rclpy.spin, node.destroy_node, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages, handles target track messages.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/math_utils.py`

- File type: Python source file.
- Tracked size: 2896 bytes; 111 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes TrackSnapshot, VelocityCommand; defines functions clamp, score_track, target_bearing_rad, compute_velocity_command, compute_velocity_command_details.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass; import math; from typing import Tuple`
- Classes:
  - `TrackSnapshot` (lines 14-26, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `VelocityCommand` (lines 30-35, bases: `object`): No class docstring; role is inferred from methods and base classes.
- Top-level functions:
  - `clamp(value: float, min_v: float, max_v: float) -> float` (lines 9-10): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `score_track(track: TrackSnapshot) -> float` (lines 38-47): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.sqrt.
  - `target_bearing_rad(track: TrackSnapshot) -> float` (lines 50-51): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: math.atan2.
  - `compute_velocity_command(track: TrackSnapshot, desired_distance_m: float, kp_xy: float, kp_z: float, kp_yaw: float, max_vel_xy_mps: float, max_vel_z_mps: float, max_yaw_rate_rps: float, min_safe_distance_m: float) -> Tuple[float, float, float, float]` (lines 54-76): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: compute_velocity_command_details.
  - `compute_velocity_command_details(track: TrackSnapshot, desired_distance_m: float, kp_xy: float, kp_z: float, kp_yaw: float, max_vel_xy_mps: float, max_vel_z_mps: float, max_yaw_rate_rps: float, min_safe_distance_m: float) -> VelocityCommand` (lines 79-111): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: target_bearing_rad, clamp, VelocityCommand.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/drone_behavior_pkg/topic_utils.py`

- File type: Python source file.
- Tracked size: 744 bytes; 27 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines functions normalize_ns, join_topic, cdrone_ns, cdrone_topic.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations`
- Top-level functions:
  - `normalize_ns(namespace: str) -> str` (lines 4-10): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: str.strip, namespace.rstrip, namespace.startswith.
  - `join_topic(namespace: str, leaf: str) -> str` (lines 13-16): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: normalize_ns, str.strip.lstrip, str.strip.
  - `cdrone_ns(drone_id: str) -> str` (lines 19-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip.strip, str.strip.
  - `cdrone_topic(drone_id: str, leaf: str) -> str` (lines 26-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: join_topic, cdrone_ns.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 958 bytes; 26 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_behavior_pkg`.
- Dependencies: `exec_depend:rclpy, exec_depend:geometry_msgs, exec_depend:std_msgs, exec_depend:mavros_msgs, exec_depend:drone_msgs, exec_depend:python3-yaml, exec_depend:numpy, test_depend:ament_copyright, test_depend:ament_flake8, test_depend:ament_pep257, test_depend:python3-pytest`.
- Build type: `ament_python`.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/resource/drone_behavior_pkg`

- File type: Text file.
- Tracked size: 19 bytes; 1 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/setup.cfg`

- File type: INI/setup configuration.
- Tracked size: 105 bytes; 4 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Config sections: `develop, install`.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/setup.py`

- File type: Python source file.
- Tracked size: 1186 bytes; 37 decoded lines.
- Purpose: Python package installer that registers package data and console script entry points.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: colcon/ament_python during build and ROS 2 console script lookup at runtime.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from setuptools import find_packages, setup; import glob; import os`
- Top-level constants/state: `package_name`.
- Whole-file runtime themes: loads or writes YAML.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/test/test_behavior_math.py`

- File type: Python source file.
- Tracked size: 1462 bytes; 62 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_score_prefers_inbound_for_equal_distance, test_min_safe_distance_blocks_forward_closing.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - local/project: `from drone_behavior_pkg.math_utils import TrackSnapshot, compute_velocity_command, score_track`
- Top-level functions:
  - `test_score_prefers_inbound_for_equal_distance()` (lines 4-33): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: TrackSnapshot, score_track.
  - `test_min_safe_distance_blocks_forward_closing()` (lines 36-62): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: TrackSnapshot, compute_velocity_command.

### `archive/legacy_stack/ros2/src/drone_behavior_pkg/test/test_behavior_transitions.py`

- File type: Python source file.
- Tracked size: 2656 bytes; 90 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines classes DummyManager; defines functions _target, test_align_transitions_to_approach_when_bearing_small, test_approach_transitions_to_follow_standoff_when_in_range, test_follow_standoff_stays_active_with_visible_target, test_lost_target_hold_returns_to_search_after_timeout.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - local/project: `from drone_behavior_pkg.behaviors.align import AlignBehavior; from drone_behavior_pkg.behaviors.approach import ApproachBehavior; from drone_behavior_pkg.behaviors.follow_standoff import FollowStandoffBehavior; from drone_behavior_pkg.behaviors.lost_target_hold import LostTargetHoldBehavior; from drone_behavior_pkg.math_utils import TrackSnapshot`
- Classes:
  - `DummyManager` (lines 8-43, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `follow_distance_m, follow_distance_tolerance_m, lost_target_timeout_s, light_default_intensity, align_yaw_tolerance_rad, reacquire_timeout_s, _target, _lost_for, _obstacle_blocked, cleared`.
    - `__init__(self)` (lines 9-19): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `get_active_target(self, _now_s)` (lines 21-22): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `active_target_lost_for(self, _now_s)` (lines 24-25): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `compute_obstacle_blocked(self, _target)` (lines 27-28): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `publish_zero_velocity(self)` (lines 30-31): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `publish_light(self, _enabled, _intensity, _strobe_hz)` (lines 33-34): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `publish_align_for_target(self, _target)` (lines 36-37): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `publish_velocity_for_target(self, _target, desired_distance_m)` (lines 39-40): No docstring; behavior is described from its body and call sites.
      Code map: 1 assertions.
    - `clear_active_target(self)` (lines 42-43): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
- Top-level functions:
  - `_target(distance_m: float, y_b_m: float = 0.0) -> TrackSnapshot` (lines 46-60): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: TrackSnapshot.
  - `test_align_transitions_to_approach_when_bearing_small()` (lines 63-67): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: DummyManager, _target, AlignBehavior, behavior.step.
  - `test_approach_transitions_to_follow_standoff_when_in_range()` (lines 70-74): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: DummyManager, _target, ApproachBehavior, behavior.step.
  - `test_follow_standoff_stays_active_with_visible_target()` (lines 77-81): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: DummyManager, _target, FollowStandoffBehavior, behavior.step.
  - `test_lost_target_hold_returns_to_search_after_timeout()` (lines 84-90): No docstring; behavior is described from its body and call sites.
    Code map: 2 assertions.
    Notable calls: DummyManager, LostTargetHoldBehavior, behavior.step.

### `archive/legacy_stack/ros2/src/drone_bringup/config/autonomy_params.yaml`

- File type: YAML configuration file.
- Tracked size: 2417 bytes; 96 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- YAML shape: `dict` with 96 documented key/value entries.
- Key map:
  - `stereo_tracker_node`: mapping
  - `stereo_tracker_node.ros__parameters`: mapping
  - `stereo_tracker_node.ros__parameters.drone_id`: `drone01`
  - `stereo_tracker_node.ros__parameters.source_mode`: `csi`
  - `stereo_tracker_node.ros__parameters.left_sensor_id`: `2`
  - `stereo_tracker_node.ros__parameters.right_sensor_id`: `3`
  - `stereo_tracker_node.ros__parameters.left_url`: `rtsp://127.0.0.1:5600/camera1`
  - `stereo_tracker_node.ros__parameters.right_url`: `rtsp://127.0.0.1:5600/camera2`
  - `stereo_tracker_node.ros__parameters.capture_width`: `1920`
  - `stereo_tracker_node.ros__parameters.capture_height`: `1080`
  - `stereo_tracker_node.ros__parameters.display_width`: `640`
  - `stereo_tracker_node.ros__parameters.display_height`: `640`
  - `stereo_tracker_node.ros__parameters.framerate`: `30`
  - `stereo_tracker_node.ros__parameters.flip_method`: `0`
  - `stereo_tracker_node.ros__parameters.model_path`: `/home/sangeetsu/cdrone_yolo/non_xai_best.engine`
  - `stereo_tracker_node.ros__parameters.device_id`: `0`
  - `stereo_tracker_node.ros__parameters.model_input_size`: `640`
  - `stereo_tracker_node.ros__parameters.detection_conf_threshold`: `0.35`
  - `stereo_tracker_node.ros__parameters.epipolar_tolerance_px`: `10.0`
  - `stereo_tracker_node.ros__parameters.tracker_rate_hz`: `20.0`
  - `stereo_tracker_node.ros__parameters.calibration_path`: `/home/sangeetsu/cdrone_yolo/stereo_calibration.npz`
  - `stereo_tracker_node.ros__parameters.camera_body_translation_m`: list[3]
  - `stereo_tracker_node.ros__parameters.camera_body_translation_m[0]`: `0.0`
  - `stereo_tracker_node.ros__parameters.camera_body_translation_m[1]`: `0.0`
  - `stereo_tracker_node.ros__parameters.camera_body_translation_m[2]`: `0.0`
  - `stereo_tracker_node.ros__parameters.camera_body_rpy_rad`: list[3]
  - `stereo_tracker_node.ros__parameters.camera_body_rpy_rad[0]`: `0.0`
  - `stereo_tracker_node.ros__parameters.camera_body_rpy_rad[1]`: `0.0`
  - `stereo_tracker_node.ros__parameters.camera_body_rpy_rad[2]`: `0.0`
  - `stereo_tracker_node.ros__parameters.norfair_distance_threshold`: `1.25`
  - `stereo_tracker_node.ros__parameters.rtsp_transport`: `udp`
  - `vio_bridge_node`: mapping
  - `vio_bridge_node.ros__parameters`: mapping
  - `vio_bridge_node.ros__parameters.drone_id`: `drone01`
  - `vio_bridge_node.ros__parameters.publish_rate_hz`: `30.0`
  - `vio_bridge_node.ros__parameters.input_timeout_s`: `0.25`
  - `vio_bridge_node.ros__parameters.publish_companion_status`: `True`
  - `vio_bridge_node.ros__parameters.input_pose_topic`: ``
  - `engagement_manager_node`: mapping
  - `engagement_manager_node.ros__parameters`: mapping
  - `engagement_manager_node.ros__parameters.drone_id`: `drone01`
  - `engagement_manager_node.ros__parameters.control_rate_hz`: `20.0`
  - `engagement_manager_node.ros__parameters.follow_distance_m`: `3.0`
  - `engagement_manager_node.ros__parameters.follow_distance_tolerance_m`: `0.4`
  - `engagement_manager_node.ros__parameters.align_yaw_tolerance_rad`: `0.12`
  - `engagement_manager_node.ros__parameters.target_cooldown_s`: `15.0`
  - `engagement_manager_node.ros__parameters.min_safe_distance_m`: `0.8`
  - `engagement_manager_node.ros__parameters.lost_target_timeout_s`: `0.75`
  - `engagement_manager_node.ros__parameters.reacquire_timeout_s`: `2.0`
  - `engagement_manager_node.ros__parameters.vio_timeout_s`: `0.5`
  - `engagement_manager_node.ros__parameters.track_gc_s`: `2.5`
  - `engagement_manager_node.ros__parameters.kp_xy`: `0.45`
  - `engagement_manager_node.ros__parameters.kp_z`: `0.3`
  - `engagement_manager_node.ros__parameters.kp_yaw`: `0.8`
  - `engagement_manager_node.ros__parameters.max_vel_xy_mps`: `1.5`
  - `engagement_manager_node.ros__parameters.max_vel_z_mps`: `0.8`
  - `engagement_manager_node.ros__parameters.max_yaw_rate_rps`: `0.6`
  - `engagement_manager_node.ros__parameters.obstacle_forward_distance_m`: `1.5`
  - `engagement_manager_node.ros__parameters.obstacle_lateral_band_m`: `0.8`
  - `engagement_manager_node.ros__parameters.obstacle_vertical_band_m`: `0.8`
  - `engagement_manager_node.ros__parameters.light_default_intensity`: `0.9`
  - `health_monitor_node`: mapping
  - `health_monitor_node.ros__parameters`: mapping
  - `health_monitor_node.ros__parameters.drone_id`: `drone01`
  - `health_monitor_node.ros__parameters.check_rate_hz`: `5.0`
  - `health_monitor_node.ros__parameters.tracks_timeout_s`: `1.0`
  - `health_monitor_node.ros__parameters.engagement_timeout_s`: `2.0`
  - `health_monitor_node.ros__parameters.vio_timeout_s`: `0.5`
  - `health_monitor_node.ros__parameters.startup_grace_s`: `5.0`
  - `mavros_velocity_node`: mapping
  - `mavros_velocity_node.ros__parameters`: mapping
  - `mavros_velocity_node.ros__parameters.drone_id`: `drone01`
  - `mavros_velocity_node.ros__parameters.publish_rate_hz`: `20.0`
  - `mavros_velocity_node.ros__parameters.watchdog_timeout_s`: `0.5`
  - `mavros_velocity_node.ros__parameters.max_vel_xy_mps`: `1.5`
  - `mavros_velocity_node.ros__parameters.max_vel_z_mps`: `0.8`
  - `mavros_velocity_node.ros__parameters.max_yaw_rate_rps`: `0.6`
  - `mavros_velocity_node.ros__parameters.require_guided_mode`: `True`
  - `light_controller_node`: mapping
  - `light_controller_node.ros__parameters`: mapping
  - `light_controller_node.ros__parameters.drone_id`: `drone01`
  - `light_controller_node.ros__parameters.light_gpio_pin`: `33`
  - `light_controller_node.ros__parameters.light_pwm_hz`: `200`
  - `light_controller_node.ros__parameters.light_default_intensity`: `0.9`
  - `telemetry_aggregator_node`: mapping
  - `telemetry_aggregator_node.ros__parameters`: mapping
  - `telemetry_aggregator_node.ros__parameters.drone_id`: `drone01`
  - `telemetry_aggregator_node.ros__parameters.hostname`: `cdrone-orin`
  - `telemetry_aggregator_node.ros__parameters.self_ip`: `127.0.0.1`
  - `telemetry_aggregator_node.ros__parameters.publish_rate_hz`: `5.0`
  - `telemetry_aggregator_node.ros__parameters.dashboard_rate_hz`: `5.0`
  - `telemetry_aggregator_node.ros__parameters.battery_low_threshold_pct`: `0.2`
  - `telemetry_aggregator_node.ros__parameters.vio_timeout_s`: `0.5`
  - `telemetry_aggregator_node.ros__parameters.tracks_timeout_s`: `1.0`
  - `telemetry_aggregator_node.ros__parameters.mavros_timeout_s`: `1.0`
  - `telemetry_aggregator_node.ros__parameters.cmd_timeout_s`: `0.5`

### `archive/legacy_stack/ros2/src/drone_bringup/launch/autonomy_stack.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 4670 bytes; 128 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: includes lower-level launches, declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `import os; from ament_index_python.packages import get_package_share_directory`
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction; from launch.launch_description_sources import PythonLaunchDescriptionSource; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node`
- Top-level functions:
  - `launch_setup(context, *args, **kwargs)` (lines 11-110): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: LaunchConfiguration.perform, LaunchConfiguration, get_package_share_directory, os.path.join, IncludeLaunchDescription, Node, os.path.exists, PythonLaunchDescriptionSource, Dict.items.
  - `generate_launch_description()` (lines 113-128): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: get_package_share_directory, os.path.join, LaunchDescription, DeclareLaunchArgument, OpaqueFunction.
- Whole-file runtime themes: loads or writes YAML.

### `archive/legacy_stack/ros2/src/drone_bringup/launch/dashboard.launch.py`

- File type: ROS 2 Python launch file.
- Tracked size: 1148 bytes; 35 decoded lines.
- Purpose: ROS 2 launch description that composes nodes, includes other launches, declares arguments, and maps runtime parameters.
- Main responsibilities: declares operator-facing launch arguments, starts ROS nodes, passes typed parameters.
- Important dependencies or consumers: ROS 2 launch CLI, runbooks, and higher-level bringup launch files.
- When to touch it: Touch this when changing runtime topology, topic names, default parameters, hardware identity, or demo behavior.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - ROS/runtime: `from launch import LaunchDescription; from launch.actions import DeclareLaunchArgument, OpaqueFunction; from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node`
- Top-level functions:
  - `launch_setup(context, *args, **kwargs)` (lines 7-24): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: LaunchConfiguration, Node.
  - `generate_launch_description()` (lines 27-35): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: LaunchDescription, DeclareLaunchArgument, OpaqueFunction.

### `archive/legacy_stack/ros2/src/drone_light_pkg/drone_light_pkg/__init__.py`

- File type: Python source file.
- Tracked size: 31 bytes; 1 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `archive/legacy_stack/ros2/src/drone_light_pkg/drone_light_pkg/light_controller_node.py`

- File type: Python source file.
- Tracked size: 4549 bytes; 137 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes LightState, LightControllerNode; defines functions cdrone_topic, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass; from typing import Optional`
  - ROS/runtime: `import rclpy; from rclpy.node import Node; from std_msgs.msg import Bool`
  - local/project: `from drone_msgs.msg import LightCommand; from drone_light_pkg.light_utils import duty_from_intensity`
- Classes:
  - `LightState` (lines 25-28, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `LightControllerNode` (lines 31-121, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `pin, pwm_hz, default_intensity, drone_id, light_cmd_topic, estop_topic, estop, state, last_duty, pwm, light_sub, estop_sub, timer, declare_parameter, _init_hardware, create_subscription, light_cmd_callback, estop_callback, create_timer, apply_output, get_parameter, get_logger, get_clock`.
    - `__init__(self) -> None` (lines 32-69): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates subscriptions, uses timer callbacks.
      Notable calls: super.__init__, self.declare_parameter, LightState, self._init_hardware, self.create_subscription, self.create_timer, str.strip, cdrone_topic, self.get_parameter.
    - `_init_hardware(self) -> None` (lines 71-83): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: GPIO.setwarnings, GPIO.setmode, GPIO.setup, GPIO.PWM, self.pwm.start.
    - `estop_callback(self, msg: Bool) -> None` (lines 85-89): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
    - `light_cmd_callback(self, msg: LightCommand) -> None` (lines 91-94): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `apply_output(self) -> None` (lines 96-113): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 return points.
      Notable calls: self.pwm.ChangeDutyCycle, duty_from_intensity.
    - `destroy_node(self) -> bool` (lines 115-121): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Notable calls: super.destroy_node, self.pwm.ChangeDutyCycle, self.pwm.stop, GPIO.cleanup.
- Top-level functions:
  - `cdrone_topic(drone_id: str, leaf: str) -> str` (lines 19-22): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip.strip, str.strip, leaf.lstrip.
  - `main(args = None) -> None` (lines 124-133): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks.
    Notable calls: rclpy.init, LightControllerNode, rclpy.spin, node.destroy_node, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates subscriptions, uses timer callbacks.

### `archive/legacy_stack/ros2/src/drone_light_pkg/drone_light_pkg/light_utils.py`

- File type: Python source file.
- Tracked size: 127 bytes; 3 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines functions duty_from_intensity.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Top-level functions:
  - `duty_from_intensity(intensity: float) -> float` (lines 1-3): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.

### `archive/legacy_stack/ros2/src/drone_light_pkg/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 792 bytes; 22 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_light_pkg`.
- Dependencies: `exec_depend:rclpy, exec_depend:std_msgs, exec_depend:drone_msgs, test_depend:ament_copyright, test_depend:ament_flake8, test_depend:ament_pep257, test_depend:python3-pytest`.
- Build type: `ament_python`.

### `archive/legacy_stack/ros2/src/drone_light_pkg/resource/drone_light_pkg`

- File type: Text file.
- Tracked size: 16 bytes; 1 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `archive/legacy_stack/ros2/src/drone_light_pkg/setup.cfg`

- File type: INI/setup configuration.
- Tracked size: 99 bytes; 4 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Config sections: `develop, install`.

### `archive/legacy_stack/ros2/src/drone_light_pkg/setup.py`

- File type: Python source file.
- Tracked size: 751 bytes; 25 decoded lines.
- Purpose: Python package installer that registers package data and console script entry points.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: colcon/ament_python during build and ROS 2 console script lookup at runtime.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from setuptools import find_packages, setup`
- Top-level constants/state: `package_name`.

### `archive/legacy_stack/ros2/src/drone_light_pkg/test/test_light_math.py`

- File type: Python source file.
- Tracked size: 235 bytes; 7 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_duty_from_intensity_clamped.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - local/project: `from drone_light_pkg.light_utils import duty_from_intensity`
- Top-level functions:
  - `test_duty_from_intensity_clamped()` (lines 4-7): No docstring; behavior is described from its body and call sites.
    Code map: 3 assertions.
    Notable calls: duty_from_intensity.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/drone_monitor_pkg/__init__.py`

- File type: Python source file.
- Tracked size: 31 bytes; 1 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/drone_monitor_pkg/dashboard_tui_node.py`

- File type: Python source file.
- Tracked size: 6704 bytes; 179 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes DashboardTuiNode; defines functions safe_addstr, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import curses; import threading; import time; from collections import OrderedDict`
  - ROS/runtime: `import rclpy; from rclpy.executors import SingleThreadedExecutor; from rclpy.node import Node`
  - local/project: `from drone_monitor_pkg.topic_utils import cdrone_topic; from drone_msgs.msg import FlightStatus, SystemAlert`
- Classes:
  - `DashboardTuiNode` (lines 26-160, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, dashboard_rate_hz, flight_status_topic, alerts_topic, latest_status, alerts, _stop, declare_parameter, create_subscription, status_callback, alert_callback, _draw_loop, _render, get_parameter, get_logger`.
    - `__init__(self) -> None` (lines 27-55): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates subscriptions.
      Notable calls: super.__init__, self.declare_parameter, OrderedDict, self.create_subscription, str.strip, cdrone_topic, self.get_parameter.
    - `status_callback(self, msg: FlightStatus) -> None` (lines 57-58): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `alert_callback(self, msg: SystemAlert) -> None` (lines 60-66): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 while loops.
      Notable calls: self.alerts.pop, self.alerts.popitem.
    - `stop(self) -> None` (lines 68-69): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `run_ui(self) -> None` (lines 71-72): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: curses.wrapper.
    - `_draw_loop(self, screen) -> None` (lines 74-88): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 while loops.
      Notable calls: curses.curs_set, screen.nodelay, screen.getch, screen.erase, self._render, screen.refresh, time.sleep, ord.
    - `_render(self, screen) -> None` (lines 90-160): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 4 for loops; 2 return points.
      Notable calls: safe_addstr, self.alerts.values, severity_text.get.
- Top-level functions:
  - `safe_addstr(window, row: int, col: int, text: str, attr: int = 0) -> None` (lines 16-23): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: window.getmaxyx, window.addstr.
  - `main(args = None) -> None` (lines 163-179): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, DashboardTuiNode, SingleThreadedExecutor, executor.add_node, threading.Thread, spin_thread.start, node.run_ui, node.stop, executor.shutdown, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates subscriptions.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/drone_monitor_pkg/telemetry_aggregator_node.py`

- File type: Python source file.
- Tracked size: 17503 bytes; 445 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines classes AlertRecord, TelemetryAggregatorNode; defines functions quaternion_to_euler, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import math; import socket; from dataclasses import dataclass; from typing import Dict, Optional`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped, TwistStamped; from mavros_msgs.msg import State; from rclpy.node import Node; from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy; from sensor_msgs.msg import BatteryState; from std_msgs.msg import Bool`
  - local/project: `from drone_monitor_pkg.topic_utils import cdrone_ns, cdrone_topic, join_topic; from drone_msgs.msg import EngagementState, FlightStatus, PeerState, PerceptionStatus, SystemAlert, TargetTrackArray`
- Classes:
  - `AlertRecord` (lines 28-31, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `TelemetryAggregatorNode` (lines 51-432, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `drone_id, hostname, self_ip, publish_rate_hz, mavros_ns, battery_low_threshold_pct, vio_timeout_s, tracks_timeout_s, mavros_timeout_s, cmd_timeout_s, autonomy_enable_topic, estop_topic, cmd_vel_topic, tracks_topic, perception_status_topic, engagement_state_topic, obstacle_blocked_topic, flight_status_topic, alerts_topic, peer_state_topic, latest_state, latest_pose, latest_velocity, latest_cmd, latest_battery, latest_engagement, latest_perception_status, latest_tracks, autonomy_enabled, estop, obstacle_blocked, last_state_s, last_pose_s, last_velocity_s, last_cmd_s, last_battery_s, last_engagement_s, last_tracks_s, last_perception_status_s, last_vio_s` plus more.
    - `__init__(self) -> None` (lines 52-201): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, handles pose messages, handles velocity setpoints, handles target track messages.
      Notable calls: super.__init__, self.declare_parameter, self._topic_param, State, PoseStamped, TwistStamped, BatteryState, EngagementState, PerceptionStatus, TargetTrackArray, QoSProfile, self.create_subscription, self.create_publisher, self.create_timer, socket.gethostname, cdrone_topic, self._mavros, self.get_parameter....
    - `_topic_param(self, param_name: str, default: str) -> str` (lines 203-205): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: reads ROS parameters.
      Notable calls: str.strip, self.get_parameter.
    - `_mavros(self, leaf: str) -> str` (lines 207-208): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: join_topic.
    - `now_s(self) -> float` (lines 210-211): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
    - `state_callback(self, msg: State) -> None` (lines 213-215): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 217-219): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `velocity_callback(self, msg: TwistStamped) -> None` (lines 221-223): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles velocity setpoints.
      Notable calls: self.now_s.
    - `vio_pose_callback(self, _msg: PoseStamped) -> None` (lines 225-226): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
      Notable calls: self.now_s.
    - `battery_callback(self, msg: BatteryState) -> None` (lines 228-230): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `autonomy_enable_callback(self, msg: Bool) -> None` (lines 232-233): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `estop_callback(self, msg: Bool) -> None` (lines 235-236): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `obstacle_callback(self, msg: Bool) -> None` (lines 238-239): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `cmd_callback(self, msg: TwistStamped) -> None` (lines 241-243): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles velocity setpoints.
      Notable calls: self.now_s.
    - `tracks_callback(self, msg: TargetTrackArray) -> None` (lines 245-247): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles target track messages.
      Notable calls: self.now_s.
    - `perception_status_callback(self, msg: PerceptionStatus) -> None` (lines 249-251): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `engagement_state_callback(self, msg: EngagementState) -> None` (lines 253-255): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.now_s.
    - `_age(self, value: Optional[float], fallback: float = 9999.0) -> float` (lines 257-260): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 2 return points.
      Notable calls: self.now_s.
    - `_update_alert(self, code: str, active: bool, severity: int, message: str) -> None` (lines 262-283): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points.
      Runtime interactions: publishes messages.
      Notable calls: self.alerts.get, SystemAlert, self.alert_pub.publish, AlertRecord.
    - `publish_loop(self) -> None` (lines 285-432): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches.
      Runtime interactions: publishes messages.
      Notable calls: FlightStatus, quaternion_to_euler, self.flight_status_pub.publish, PeerState, self.peer_state_pub.publish, self._update_alert, getattr, self._age.
- Top-level functions:
  - `quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]` (lines 34-48): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: math.atan2, math.copysign, math.asin.
  - `main(args = None) -> None` (lines 435-445): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks.
    Notable calls: rclpy.init, TelemetryAggregatorNode, rclpy.spin, node.destroy_node, rclpy.ok, rclpy.shutdown.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, creates subscriptions, uses timer callbacks, publishes messages, handles pose messages, handles velocity setpoints, handles target track messages.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/drone_monitor_pkg/topic_utils.py`

- File type: Python source file.
- Tracked size: 744 bytes; 27 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: defines functions normalize_ns, join_topic, cdrone_ns, cdrone_topic.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations`
- Top-level functions:
  - `normalize_ns(namespace: str) -> str` (lines 4-10): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: str.strip, namespace.rstrip, namespace.startswith.
  - `join_topic(namespace: str, leaf: str) -> str` (lines 13-16): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: normalize_ns, str.strip.lstrip, str.strip.
  - `cdrone_ns(drone_id: str) -> str` (lines 19-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: str.strip.strip, str.strip.
  - `cdrone_topic(drone_id: str, leaf: str) -> str` (lines 26-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: join_topic, cdrone_ns.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 926 bytes; 25 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_monitor_pkg`.
- Dependencies: `exec_depend:rclpy, exec_depend:geometry_msgs, exec_depend:sensor_msgs, exec_depend:std_msgs, exec_depend:mavros_msgs, exec_depend:drone_msgs, test_depend:ament_copyright, test_depend:ament_flake8, test_depend:ament_pep257, test_depend:python3-pytest`.
- Build type: `ament_python`.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/resource/drone_monitor_pkg`

- File type: Text file.
- Tracked size: 18 bytes; 1 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/setup.cfg`

- File type: INI/setup configuration.
- Tracked size: 103 bytes; 4 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Config sections: `develop, install`.

### `archive/legacy_stack/ros2/src/drone_monitor_pkg/setup.py`

- File type: Python source file.
- Tracked size: 867 bytes; 26 decoded lines.
- Purpose: Python package installer that registers package data and console script entry points.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: colcon/ament_python during build and ROS 2 console script lookup at runtime.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from setuptools import find_packages, setup`
- Top-level constants/state: `package_name`.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/drone_vision_pkg/__init__.py`

- File type: Python source file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: Provides executable or importable Python behavior.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/drone_vision_pkg/stereo_tracker_node.py`

- File type: Python source file.
- Tracked size: 17355 bytes; 451 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes StereoTrackerNode; defines functions cdrone_topic, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from pathlib import Path; from typing import Dict, List, Tuple; import os; import time`
  - third-party: `import numpy as np`
  - ROS/runtime: `import rclpy; from rclpy.node import Node`
  - local/project: `from drone_vision_pkg.stereo_utils import StereoDetection, build_csi_pipeline, camera_to_body, match_detections_epipolar, rpy_to_rotation_matrix, triangulate_point; from drone_msgs.msg import PerceptionStatus, TargetTrack, TargetTrackArray`
- Classes:
  - `StereoTrackerNode` (lines 57-430, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `source_mode, left_sensor_id, right_sensor_id, left_url, right_url, capture_width, capture_height, display_width, display_height, framerate, flip_method, model_path, device_id, model_input_size, conf_thresh, epipolar_tol, tracker_rate_hz, calibration_path, translation, rotation, norfair_distance_threshold, drone_id, tracks_topic, perception_status_topic, model, tracker, motion_estimator, left_cap, right_cap, prev_positions, tracks_pub, perception_status_pub, last_process_time_s, timer, p1, p2, map1_x, map1_y, map2_x, map2_y` plus more.
    - `__init__(self) -> None` (lines 58-158): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 1 raise statements.
      Runtime interactions: declares ROS parameters, reads ROS parameters, creates publishers, uses timer callbacks, handles target track messages, uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: super.__init__, self.declare_parameter, str.strip.lower, np.array, rpy_to_rotation_matrix, self._load_calibration, YOLO, self._build_tracker, self._open_capture, self.create_publisher, self.create_timer, RuntimeError, str.strip, cdrone_topic, MotionEstimator, self.get_parameter.
    - `_build_tracker(self)` (lines 160-178): No docstring; behavior is described from its body and call sites.
      Code map: 1 try/except blocks; 2 return points.
      Notable calls: Tracker.
    - `_load_calibration(self, calibration_path: str) -> None` (lines 180-192): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 raise statements.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: Path, np.load, path.exists, FileNotFoundError.
    - `_open_capture(self, sensor_id: int, url: str)` (lines 194-212): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 return points; 1 raise statements.
      Runtime interactions: uses OpenCV image processing.
      Notable calls: build_csi_pipeline, cv2.VideoCapture, cap.isOpened, RuntimeError.
    - `_extract_detections(self, image: np.ndarray) -> List[StereoDetection]` (lines 214-247): No docstring; behavior is described from its body and call sites.
      Code map: 2 conditional branches; 1 for loops; 3 return points.
      Runtime interactions: uses NumPy arrays/math.
      Notable calls: self.model.predict, boxes.xyxy.cpu.numpy, boxes.conf.cpu.numpy, np.ones, detections.append, boxes.xyxy.cpu, StereoDetection, boxes.conf.cpu.
    - `_publish_tracks(self, tracked_objects) -> None` (lines 249-299): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 1 for loops.
      Runtime interactions: publishes messages, handles target track messages, uses NumPy arrays/math.
      Notable calls: TargetTrackArray, self.tracks_pub.publish, np.array.reshape, TargetTrack, msg.tracks.append, np.zeros, pos.copy, np.linalg.norm, getattr, isinstance, np.array, np.dot, data.get.
    - `_publish_perception_status(self, left_count: int, right_count: int, paired_count: int, active_tracks: int, inference_latency_ms: float) -> None` (lines 301-325): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: publishes messages.
      Notable calls: PerceptionStatus, self.perception_status_pub.publish.
    - `get_hist(self, image)` (lines 327-335): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: uses OpenCV image processing.
      Notable calls: cv2.calcHist, cv2.normalize.flatten, cv2.cvtColor, cv2.normalize.
    - `embedding_distance(self, matched_not_init_trackers, unmatched_trackers)` (lines 337-357): No docstring; behavior is described from its body and call sites.
      Code map: 4 conditional branches; 2 for loops; 3 return points.
      Runtime interactions: uses OpenCV image processing.
      Notable calls: reversed, cv2.compareHist.
    - `process_frame(self) -> None` (lines 359-430): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 2 for loops; 2 try/except blocks; 1 return points.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: time.perf_counter, self.left_cap.read, self.right_cap.read, cv2.remap, self._extract_detections, match_detections_epipolar, self._publish_tracks, self._publish_perception_status, self._open_capture, camera_to_body, detections_3d.append, get_cutout, self.motion_estimator.update, self.tracker.update, self.left_cap.release, self.right_cap.release, triangulate_point, Detection....
- Top-level functions:
  - `cdrone_topic(drone_id: str, leaf: str) -> str` (lines 51-54): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.strip.strip, str.strip, leaf.lstrip.
  - `main(args = None) -> None` (lines 433-447): No docstring; behavior is described from its body and call sites.
    Code map: 2 try/except blocks.
    Notable calls: rclpy.init, StereoTrackerNode, rclpy.spin, node.destroy_node, rclpy.shutdown, node.left_cap.release, node.right_cap.release.
- Whole-file runtime themes: declares ROS parameters, reads ROS parameters, creates publishers, uses timer callbacks, publishes messages, handles target track messages, uses OpenCV image processing, uses NumPy arrays/math.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/drone_vision_pkg/stereo_utils.py`

- File type: Python source file.
- Tracked size: 3454 bytes; 111 decoded lines.
- Purpose: Vision-side Python module or ROS node for RealSense tracking, target memory, mapping, metrics, or reports.
- Main responsibilities: defines classes StereoDetection; defines functions build_csi_pipeline, rpy_to_rotation_matrix, match_detections_epipolar, triangulate_point, camera_to_body.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from dataclasses import dataclass; from typing import List, Sequence, Tuple; import math`
  - third-party: `import numpy as np`
- Classes:
  - `StereoDetection` (lines 21-26, bases: `object`): No class docstring; role is inferred from methods and base classes.
- Top-level functions:
  - `build_csi_pipeline(sensor_id: int, capture_width: int, capture_height: int, display_width: int, display_height: int, framerate: int, flip_method: int) -> str` (lines 29-45): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `rpy_to_rotation_matrix(rpy: Sequence[float]) -> np.ndarray` (lines 48-57): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.array, math.cos, math.sin.
  - `match_detections_epipolar(left: Sequence[StereoDetection], right: Sequence[StereoDetection], y_tolerance_px: float) -> List[Tuple[int, int]]` (lines 60-95): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 4 for loops; 3 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.full, linear_sum_assignment, pairs.append, used_right.add, np.argmin.
  - `triangulate_point(p1: np.ndarray, p2: np.ndarray, left_uv: Tuple[float, float], right_uv: Tuple[float, float]) -> np.ndarray` (lines 98-107): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
    Notable calls: np.array.reshape, cv2.triangulatePoints, point_h.reshape, RuntimeError, np.array.
  - `camera_to_body(point_cam: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray` (lines 110-111): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: uses NumPy arrays/math.
- Whole-file runtime themes: uses OpenCV image processing, uses NumPy arrays/math.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/package.xml`

- File type: XML ROS package manifest.
- Tracked size: 944 bytes; 26 decoded lines.
- Purpose: ROS package manifest declaring package metadata, build type, and runtime/test dependencies.
- Main responsibilities: Declares dependency metadata required for ROS package discovery and build resolution.
- Important dependencies or consumers: ROS build tools, rosdep, colcon, and package installers.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- XML root: `package`.
- ROS package name: `drone_vision_pkg`.
- Dependencies: `exec_depend:rclpy, exec_depend:cv2, exec_depend:numpy, exec_depend:scipy, exec_depend:norfair, exec_depend:ultralytics, exec_depend:drone_msgs, test_depend:ament_copyright, test_depend:ament_flake8, test_depend:ament_pep257, test_depend:python3-pytest`.
- Build type: `ament_python`.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/resource/drone_vision_pkg`

- File type: Text file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 0.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/setup.cfg`

- File type: INI/setup configuration.
- Tracked size: 101 bytes; 4 decoded lines.
- Purpose: Archived prior implementation/reference material retained for historical context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Config sections: `develop, install`.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/setup.py`

- File type: Python source file.
- Tracked size: 1170 bytes; 35 decoded lines.
- Purpose: Python package installer that registers package data and console script entry points.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: colcon/ament_python during build and ROS 2 console script lookup at runtime.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from setuptools import find_packages, setup; import os; import glob`
- Top-level constants/state: `package_name`.
- Whole-file runtime themes: loads or writes YAML.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/test/test_copyright.py`

- File type: Python source file.
- Tracked size: 967 bytes; 27 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_copyright.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_copyright.main import main; import pytest`
- Top-level functions:
  - `test_copyright()` (lines 25-27): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: pytest.mark.skip, main.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/test/test_flake8.py`

- File type: Python source file.
- Tracked size: 877 bytes; 25 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_flake8.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_flake8.main import main_with_errors; import pytest`
- Top-level functions:
  - `test_flake8()` (lines 21-25): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main_with_errors, '\n'.join.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/test/test_pep257.py`

- File type: Python source file.
- Tracked size: 802 bytes; 23 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_pep257.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from ament_pep257.main import main; import pytest`
- Top-level functions:
  - `test_pep257()` (lines 21-23): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: main.

### `archive/legacy_stack/ros2/src/drone_vision_pkg/test/test_stereo_matcher.py`

- File type: Python source file.
- Tracked size: 583 bytes; 14 decoded lines.
- Purpose: Automated pytest/ament test coverage for the adjacent package behavior.
- Main responsibilities: defines functions test_epipolar_matching_prefers_disparity_pairs.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - local/project: `from drone_vision_pkg.stereo_utils import StereoDetection, match_detections_epipolar`
- Top-level functions:
  - `test_epipolar_matching_prefers_disparity_pairs()` (lines 4-14): No docstring; behavior is described from its body and call sites.
    Code map: 1 assertions.
    Notable calls: match_detections_epipolar, StereoDetection.

## Dataset capture and export workspace

### `ds_dataset/.gitignore`

- File type: Text file.
- Tracked size: 514 bytes; 24 decoded lines.
- Purpose: sessions/* !sessions/.gitkeep runtime/* !runtime/.gitkeep scripts/__pycache__/ exports/yolo/images/train/* !exports/yolo/images/train/.gitkeep exports/yolo/images/val/* !exports/yolo/images/val/.gitkeep exports/yolo/images/test/* !exports/yolo/images/test/....
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 24.

### `ds_dataset/README.md`

- File type: Markdown documentation.
- Tracked size: 3791 bytes; 118 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - DS Dataset Workspace
    - Layout
    - Manual Flight Workflow
    - Annotation Convention
    - Export Later For YOLO Training
    - Notes
- Opening content: This folder is the capture workspace for collecting a better small-drone RGB dataset from the RealSense D455 while the main drone is being flown manually.

### `ds_dataset/exports/yolo/data.yaml`

- File type: YAML configuration file.
- Tracked size: 136 bytes; 6 decoded lines.
- Purpose: Runtime configuration for launch arguments, PX4/MAVROS behavior, tracking, perimeters, speed profiles, or datasets.
- Main responsibilities: Stores structured defaults that keep runtime behavior configurable without code edits.
- Important dependencies or consumers: Launch files, nodes, scripts, or operators that load runtime parameters.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- YAML shape: `dict` with 6 documented key/value entries.
- Key map:
  - `path`: `/home/jetson/cdrone_control/ds_dataset/exports/yolo`
  - `train`: `images/train`
  - `val`: `images/val`
  - `test`: `images/test`
  - `names`: mapping
  - `names.0`: `small_drone`

### `ds_dataset/exports/yolo/images/test/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Text lines: 1.

### `ds_dataset/exports/yolo/images/train/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `ds_dataset/exports/yolo/images/val/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `ds_dataset/exports/yolo/labels/test/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch this when behavior changes require updated regression coverage.
- Text lines: 1.

### `ds_dataset/exports/yolo/labels/train/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `ds_dataset/exports/yolo/labels/val/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `ds_dataset/exports/yolo/manifests/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `ds_dataset/runtime/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

### `ds_dataset/scripts/export_yolo_dataset.py`

- File type: Python source file.
- Tracked size: 7460 bytes; 240 decoded lines.
- Purpose: Dataset capture/export helper for RealSense-based training data workflows.
- Main responsibilities: defines functions default_dataset_root, stable_split_key, choose_split, ensure_output_dirs, resolve_sessions, transfer_file, write_data_yaml, build_parser, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import csv; import hashlib; import os; import shutil; from pathlib import Path`
- Top-level constants/state: `FRAME_CSV_NAME, EXPORT_MANIFEST_NAME`.
- Top-level functions:
  - `default_dataset_root() -> Path` (lines 16-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Path.resolve, Path.
  - `stable_split_key(seed: int, key: str) -> float` (lines 20-22): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: hashlib.sha256.hexdigest, hashlib.sha256, JoinedStr.encode.
  - `choose_split(score: float, val_fraction: float, test_fraction: float) -> str` (lines 25-30): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
  - `ensure_output_dirs(output_root: Path) -> None` (lines 33-43): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops.
    Notable calls: Path, BinOp.mkdir.
  - `resolve_sessions(dataset_root: Path, requested_sessions: list[str]) -> list[Path]` (lines 46-50): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: sorted, sessions_root.iterdir, path.is_dir.
  - `transfer_file(src: Path, dst: Path, mode: str) -> None` (lines 53-63): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 1 raise statements.
    Notable calls: dst.exists, dst.is_symlink, dst.unlink, shutil.copy2, dst.symlink_to, os.link, ValueError.
  - `write_data_yaml(output_root: Path, class_name: str) -> None` (lines 66-81): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Runtime interactions: loads or writes YAML.
    Notable calls: yaml_path.write_text, '\n'.join.
  - `build_parser() -> argparse.ArgumentParser` (lines 84-143): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML, defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, default_dataset_root.
  - `main() -> int` (lines 146-236): No docstring; behavior is described from its body and call sites.
    Code map: 9 conditional branches; 2 for loops; 2 context managers; 3 return points; 2 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: build_parser.parse_args, args.dataset_root.resolve, args.output_root.resolve, resolve_sessions, write_data_yaml, SystemExit, ensure_output_dirs, manifest_path.open, csv.DictWriter, writer.writeheader, writer.writerows, build_parser, frames_csv_path.exists, frames_csv_path.open, csv.DictReader, choose_split, manifest_rows.append, rgb_path.exists....
- Whole-file runtime themes: loads or writes YAML, defines a CLI.

### `ds_dataset/scripts/record_realsense_dataset.py`

- File type: Python source file.
- Tracked size: 25556 bytes; 729 decoded lines.
- Purpose: Dataset capture/export helper for RealSense-based training data workflows.
- Main responsibilities: defines classes SessionPaths, StopRequested, RealsenseDatasetRecorder; defines functions isoformat_now, sanitize_tag, default_dataset_root, build_session_name, load_optional_module, build_parser, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import csv; import json; import os; import signal; import sys; import time; from dataclasses import dataclass; from datetime import datetime, timezone; from pathlib import Path; from typing import Any`
- Top-level constants/state: `DEFAULT_COLOR_WIDTH, DEFAULT_COLOR_HEIGHT, DEFAULT_COLOR_FPS, DEFAULT_DEPTH_WIDTH, DEFAULT_DEPTH_HEIGHT, DEFAULT_DEPTH_FPS, DEFAULT_SAVE_FPS, DEFAULT_STATUS_INTERVAL_S, DEFAULT_WARMUP_FRAMES, SCHEMA_VERSION, FRAME_CSV_FIELDNAMES, ANNOTATION_INDEX_FIELDNAMES`.
- Classes:
  - `SessionPaths` (lines 100-107, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `StopRequested` (lines 110-111, bases: `Exception`): No class docstring; role is inferred from methods and base classes.
  - `RealsenseDatasetRecorder` (lines 114-576, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `args, dataset_root, sessions_root, runtime_dir, session_name, session_paths, status_path, pid, capture_started_wall_s, capture_started_monotonic_s, stop_requested, stop_reason, total_frames_seen, total_frames_saved, total_frames_skipped, last_saved_monotonic_s, last_saved_wall_s, last_status_monotonic_s, frames_file, annotation_file, manifest, _prepare_session_paths, _write_annotation_index_row, _write_manifest, _write_status, _initialize_metadata_files, _should_stop_from_limits, _save_frame, _log_status`.
    - `__init__(self, args: argparse.Namespace) -> None` (lines 115-185): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Runtime interactions: defines a CLI.
      Notable calls: args.dataset_root.resolve, args.runtime_dir.resolve, self._prepare_session_paths, self.runtime_dir.mkdir, os.getpid, time.time, time.monotonic, build_session_name, args.status_path.resolve, isoformat_now.
    - `_prepare_session_paths(self) -> SessionPaths` (lines 187-206): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 for loops; 1 return points; 1 raise statements.
      Notable calls: session_dir.exists, SessionPaths, FileExistsError, path.mkdir.
    - `request_stop(self, reason: str) -> None` (lines 208-210): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `_write_manifest(self) -> None` (lines 212-221): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.session_paths.manifest_path.write_text, json.dumps.
    - `_write_status(self, state: str) -> None` (lines 223-246): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self.status_path.write_text, isoformat_now, json.dumps.
    - `_write_annotation_index_row(self, writer: csv.DictWriter, frame_index: int, stem: str) -> None` (lines 248-259): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: writer.writerow.
    - `_initialize_metadata_files(self) -> tuple[Any, csv.DictWriter, Any, csv.DictWriter]` (lines 261-283): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Notable calls: self.session_paths.frames_csv_path.open, csv.DictWriter, frames_writer.writeheader, self.session_paths.annotation_index_path.open, annotation_writer.writeheader.
    - `_save_frame(self, cv2, color_image, depth_image, frame_index: int, color_frame_number: int, color_timestamp_ms: float, depth_frame_number: int | str, depth_timestamp_ms: float | str, frames_writer: csv.DictWriter, annotation_writer: csv.DictWriter) -> None` (lines 285-351): No docstring; behavior is described from its body and call sites.
      Code map: 5 conditional branches; 2 raise statements.
      Runtime interactions: uses OpenCV image processing.
      Notable calls: time.time, time.monotonic, frames_writer.writerow, self._write_annotation_index_row, Path, cv2.imwrite, RuntimeError, depth_relpath.as_posix, self.frames_file.flush, self.annotation_file.flush, isoformat_now, rgb_relpath.as_posix, label_relpath.as_posix.
    - `_log_status(self) -> None` (lines 353-369): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches; 1 return points.
      Notable calls: time.monotonic, self._write_manifest, self._write_status.
    - `_should_stop_from_limits(self) -> None` (lines 371-377): No docstring; behavior is described from its body and call sites.
      Code map: 3 conditional branches; 2 raise statements.
      Notable calls: StopRequested, time.monotonic.
    - `run(self) -> int` (lines 379-576): No docstring; behavior is described from its body and call sites.
      Code map: 15 conditional branches; 1 for loops; 1 while loops; 2 try/except blocks; 2 return points; 3 raise statements.
      Runtime interactions: uses OpenCV image processing, uses NumPy arrays/math.
      Notable calls: self._write_manifest, self._write_status, load_optional_module, self._initialize_metadata_files, isoformat_now, rs.config, config.enable_stream, rs.pipeline, pipeline.start, profile.get_device, profile.get_stream.as_video_stream_profile, color_profile.get_intrinsics, frames_file.close, annotation_file.close, config.enable_device, rs.align, device.first_depth_sensor, pipeline.wait_for_frames....
- Top-level functions:
  - `isoformat_now(timestamp_s: float | None = None) -> str` (lines 59-66): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: datetime.fromtimestamp.astimezone.isoformat, time.time, datetime.fromtimestamp.astimezone, datetime.fromtimestamp.
  - `sanitize_tag(value: str) -> str` (lines 69-77): No docstring; behavior is described from its body and call sites.
    Code map: 1 while loops; 1 return points.
    Notable calls: ''.join, cleaned.strip, cleaned.replace, value.strip, char.isalnum.
  - `default_dataset_root() -> Path` (lines 80-81): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Path.resolve, Path.
  - `build_session_name(tag: str) -> str` (lines 84-86): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: datetime.now.strftime, datetime.now, sanitize_tag.
  - `load_optional_module(module_name: str, package_name: str)` (lines 89-96): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks; 1 return points; 1 raise statements.
    Notable calls: __import__, RuntimeError.
  - `build_parser() -> argparse.ArgumentParser` (lines 579-707): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, default_dataset_root.
  - `main() -> int` (lines 710-725): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks; 2 return points.
    Notable calls: build_parser, parser.parse_args, RealsenseDatasetRecorder, signal.signal, recorder.request_stop, recorder.run.
- Whole-file runtime themes: uses OpenCV image processing, uses NumPy arrays/math, defines a CLI.

### `ds_dataset/scripts/start_capture.sh`

- File type: Shell script.
- Tracked size: 2318 bytes; 86 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Defined shell functions: `find_running_recorders`.
- Command map: `echo(15), if(6), fi(6), exit(3), rm(2), set(1), mkdir(1), ps(1), source(1), shift(1), nohup(1), --dataset-root(1), --runtime-dir(1), --tag(1), --session-name(1), --status-path(1), cat(1), EOF(1), sleep(1), tail(1)`.

### `ds_dataset/scripts/status_capture.sh`

- File type: Shell script.
- Tracked size: 1543 bytes; 54 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Defined shell functions: `find_running_recorders`.
- Command map: `echo(7), if(4), fi(4), exit(3), set(1), ps(1), source(1), python3(1), import(1), path(1), with(1), data(1), mapfile(1), printf(1)`.

### `ds_dataset/scripts/stop_capture.sh`

- File type: Shell script.
- Tracked size: 1698 bytes; 67 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Defined shell functions: `find_running_recorders`.
- Command map: `echo(10), if(8), fi(8), exit(3), rm(2), set(1), ps(1), source(1), mapfile(1), printf(1), kill(1), for(1), break(1), sleep(1), done(1)`.

### `ds_dataset/sessions/.gitkeep`

- File type: Empty directory placeholder.
- Tracked size: 1 bytes; 1 decoded lines.
- Purpose: Keeps an otherwise-empty generated-data directory present in Git.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Git only; runtime tools write generated data next to it.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1.

## Docker and host setup assets

### `docker/amd64/install_geographiclib_datasets.sh`

- File type: Shell script.
- Tracked size: 1243 bytes; 41 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Defined shell functions: `run_get`.
- Command map: `echo(5), if(4), fi(4), local(3), run_get(3), return(2), exit(1), elif(1), geographiclib-datasets-download(1), else(1)`.

### `docker/amd64/mavros.Dockerfile`

- File type: Dockerfile.
- Tracked size: 1103 bytes; 38 decoded lines.
- Purpose: Container or host setup asset for MAVROS/ROS/Jetson workflows.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Docker instructions:
  - `FROM ros:humble`
  - `ENV ROS_DISTRO=humble`
  - `RUN apt-get update && apt-get install -y --no-install-recommends\`
  - `COPY install_geographiclib_datasets.sh /bash_scripts/`
  - `RUN chmod +x /bash_scripts/install_geographiclib_datasets.sh`
  - `RUN /bash_scripts/install_geographiclib_datasets.sh`
  - `RUN pip install numpy==1.24.4 transforms3d==0.4.1 setuptools==58.2.0`
  - `COPY drone_control_pkg /ros2_ws/src/drone_control_pkg`
  - `WORKDIR /opt/ros/${ROS_DISTRO}`
  - `RUN . /setup.sh && \`
  - `RUN echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc`
  - `COPY ros_entrypoint.sh /bash_scripts/`
  - `RUN chmod +x /bash_scripts/ros_entrypoint.sh`
  - `ENTRYPOINT [ "/bash_scripts/ros_entrypoint.sh" ]`
  - `CMD ["ros2", "launch", "mavros", "apm.launch"]`

### `docker/amd64/ros_entrypoint.sh`

- File type: Shell script.
- Tracked size: 95 bytes; 6 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Shell safety flags: `set -e`.
- Command map: `set(1), source(1), exec(1)`.

### `docker/jetson/mavros.Dockerfile`

- File type: Dockerfile.
- Tracked size: 785 bytes; 31 decoded lines.
- Purpose: Container or host setup asset for MAVROS/ROS/Jetson workflows.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Docker instructions:
  - `ARG ROS_VERSION=humble`
  - `ARG ROS_IMAGE=-ros-base-jammy`
  - `FROM ros:$ROS_VERSION$ROS_IMAGE`
  - `RUN apt-get update && apt-get install -y \`
  - `RUN pip install numpy transforms3d setuptools==58.2.0`
  - `RUN . /opt/ros/$ROS_DISTRO/setup.sh && \`
  - `COPY drone_control_pkg /root/ros2_ws/src`
  - `WORKDIR /root/ros2_ws/`
  - `RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \`
  - `RUN . /opt/ros/$ROS_DISTRO/setup.sh && \`
  - `RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \`
  - `CMD ["bash"]`

### `docker/jetson/mavros_entrypoint.sh`

- File type: Shell script.
- Tracked size: 376 bytes; 15 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Command map: `ros2(2), source(1), sleep(1), wait(1)`.

## Documentation, references, and reports

### `docs/archived/ARK_PAB_FLASH_CHECKLIST.md`

- File type: Markdown documentation.
- Tracked size: 16596 bytes; 694 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - ARK PAB Jetson Flash Checklist
    - 0. What You Need Before You Start
    - 1. On The Host PC, Make A Fresh Workspace
    - 2. Confirm The Host PC Is Ubuntu 22.04
    - 3. Run ARK Setup
    - 4. Build The Kernel And ARK Device Tree
    - 5. Optional: Preload Wi-Fi Before Flashing
    - 6. Optional: Generate A Reusable Flash Package
  - SD card image package
  - Non-super package
    - 7. Put The Jetson In Force Recovery Mode
    - 8. Flash The Jetson
    - 9. First Boot After Flash
      - Option A: SSH over mDNS
      - Option B: SSH over micro-USB RNDIS
    - 10. Optional: Give The Jetson Internet Over Micro-USB
    - 11. Optional: Add Wi-Fi After Flash Instead
    - 12. Optional: Enable Super Mode And Max Clocks
- Opening content: This file is a literal, follow-along checklist for **your exact case**:

### `docs/archived/README.md`

- File type: Markdown documentation.
- Tracked size: 514 bytes; 16 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Archived Root Docs
- Opening content: This directory holds older markdown files that used to live in the repo root.

### `docs/archived/artifacts/output.txt`

- File type: Plain text file.
- Tracked size: 103579 bytes; 1086 decoded lines.
- Purpose: [INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-08-17-33-57-696322-jetson-6480 [INFO] [launch]: Default logging verbosity is set to INFO [INFO] [mavros_node-1]: process started with pid [6482] [INFO] [drone_setup_node-2]: pro...
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 1086 total, 1086 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-08-17-33-57-696322-jetson-6480`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [6482]`

### `docs/archived/hackster.md`

- File type: Markdown documentation.
- Tracked size: 11376 bytes; 449 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Hackster Summary: GPS-Denied Drone With NVIDIA Jetson Orin Nano
    - Overview
    - Hardware Used
    - Stated Software / Version Requirements
    - System Architecture
    - Jetson Orin Nano Setup
      - Wiring
      - JetPack Verification and Performance Setup
    - SSD Setup
    - Isaac ROS Setup
      - Dependencies and Workspace
      - Cloning Isaac ROS and VSLAM-UAV
      - Running Isaac ROS
      - RealSense Validation
    - Camera IMU Setup
      - IMU Calibration
      - IMU Noise Parameter Estimation
    - PX4 Flight Controller Setup
- Opening content: Source: - Hackster article: https://www.hackster.io/bandofpv/gps-denied-drone-with-nvidia-jetson-orin-nano-9f3417 - Author: Andrew Bernas - Published: June 22, 2025

### `docs/archived/memory.md`

- File type: Markdown documentation.
- Tracked size: 10615 bytes; 142 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Repo Memory
    - Rehaul Update
    - Purpose
    - What Was Working
    - What Was Blocked
    - Package Roles Before Archival
    - Important Launch Flows
    - Control Paths That Existed
    - Repo Intentions Worth Preserving
    - Documents That Mattered
    - Key Assumptions
    - Things That Happened During the Investigation
    - Do Not Forget
- Opening content: This repository is the old `cdrone_control` stack for a Jetson Orin Nano companion computer driving a PX4 aircraft through MAVROS. It is being rehauled, so this file exists to remember what used to matter before the implementation gets replaced.

### `docs/archived/position_hover_demo_runbook.md`

- File type: Markdown documentation.
- Tracked size: 2011 bytes; 120 decoded lines.
- Purpose: Operator runbook that records a repeatable field, bench, or demo procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Props-Off Bench Test
    - Terminal 1
    - Terminal 2
    - Terminal 3
    - Terminal 4
    - Terminal 5
    - Start
    - Abort
  - Props-On Actual Test
    - Terminal 1
    - Terminal 2
    - Terminal 3
    - Terminal 4
    - Terminal 5
    - Start
    - Abort
- Opening content: ```bash source ~/.bashrc ros2 launch drone_bringup position_hover_demo.launch.py \ pose_source:=optitrack \ optitrack_server:=192.168.0.217 \ rigid_body_name:=RigidBody3 \ takeoff_altitude_m:=0.6 \ takeoff_rate_m_s:=0.3 \ takeoff_strategy:=AUTO_MODE \ hover_duration_s:=5.0 \ h...

### `docs/archived/problem2_realsense.md`

- File type: Markdown documentation.
- Tracked size: 9921 bytes; 247 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Problem 2 Journal: RealSense D455 And VSLAM Bringup
    - Current State
    - 2026-03-24
      - Goal
      - What Was Verified
      - What Changed Compared To The Earlier Failure
      - Repo Changes Made On This Date
      - VSLAM Installation Work
      - VSLAM Runtime Result
      - Evidence Captured
      - What This Means
      - What Is Still Missing
      - Next Rehaul Step
    - 2026-03-21
      - Goal
      - What Happened
      - Deeper Checks
      - Conclusion
- Opening content: This file is now a dated journal for the RealSense and VSLAM investigation so we can see what changed on each session date without reconstructing the story from memory.

### `docs/archived/recovery.md`

- File type: Markdown documentation.
- Tracked size: 10356 bytes; 336 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Recovery Guide: RealSense D455 VIO + Isaac ROS + PX4 Stack
    - Snapshot
    - What Must Be True For Recovery To Work
    - Backup Artifact
    - Copy Options
      - Option 1: Pull From Another PC Using SSH
      - Option 2: Copy To A Mounted Storage Device
      - Option 3: Serve The Export Directory Over HTTP
      - Option 4: Rebuild Instead Of Restore
    - Fast Recovery Path After A Full Wipe
      - 1. Restore The Repo
      - 2. Restore Host ROS Packages And Workspace
      - 3. Verify Docker And NVIDIA Runtime
      - 4. Confirm The D455 Firmware
      - 5. Restore Or Rebuild The Isaac Docker Image
      - 6. Start The Headless Isaac Container
      - 7. Reinstall The Isaac VSLAM Packages Into The Running Container
      - 8. Launch The RealSense-Backed VSLAM Stack
- Opening content: This guide captures the current known-good recovery path for the RealSense D455 VIO stack on this Jetson so the environment can be rebuilt after a kernel reflash or full rootfs wipe.

### `docs/archived/rehaul_vio_px4_plan.md`

- File type: Markdown documentation.
- Tracked size: 16600 bytes; 416 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Rehaul Plan: D455 VIO to PX4 Position Control
    - Objective
    - What The Current Repo Proves
    - Core Decisions
      - 1. Bootstrap through `vision_pose`, but promote to MAVROS odometry before flightworthy autonomy
      - 2. Fly `POSCTL` before `OFFBOARD`
      - 3. Use a real VIO/VSLAM engine outside this repo, but keep the PX4 bridge inside this repo
    - Compatibility Guardrails
    - Recommended End-State Architecture
      - Data flow
      - Practical architecture for this repo
      - Frame contract to standardize early
    - What To Keep, Replace, Archive, Or Delete
      - Keep
      - Replace
      - Archive
      - Delete after archive is captured
    - New Files And Packages Worth Adding
- Opening content: Rework this repo from a mixed autonomy/teleop experiment into a bench-first, no-GPS indoor flight stack that can:

### `docs/archived/rehaul_vio_px4_viability.md`

- File type: Markdown documentation.
- Tracked size: 9474 bytes; 221 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Rehaul VIO + PX4 Viability Review
    - Verdict
    - Confirmed Facts
      - 1. PX4 1.16 supports no-GPS flight with external vision
      - 2. Indoor position-hold / position-control is feasible once EV/VIO is fused
      - 3. MAVROS on ROS 2 Humble supports the needed interfaces
      - 4. PX4’s preferred VIO path is `odometry/out`, not just `vision_pose`
      - 5. D455 is a valid substitute for D435i in the Isaac ROS RealSense path
      - 6. ROS 2 Humble is still a reasonable base in March 2026
    - Important Adjustments
      - 1. Do not follow the newest Isaac ROS docs blindly
      - 2. RealSense version pinning matters more than the tutorial makes it look
      - 3. Frame handling is still an easy place to fail
      - 4. The current repo’s denylist on the MAVROS odometry plugin is incompatible with the recommended end state
    - Likely Risks, But Not Hard Blocks
      - Confirmed risks
      - Inference-based risks
    - Practical Recommendation
- Opening content: Date: 2026-03-20

### `docs/archived/takeoff_debug_summary.md`

- File type: Markdown documentation.
- Tracked size: 25749 bytes; 495 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Takeoff Debug Summary
    - Branch
    - Current Goal
    - What Already Works
    - New Demo Path
    - Current Launch Command
    - Important Code Changes Already Made
    - Takeoff Debug Timeline
      - 1. Original scripted takeoff
      - 2. Explicit local takeoff experiment
      - 3. Switched back to `AUTO_MODE`
      - 4. Latest retries
      - 5. Live capture on 2026-04-03
      - 6. Live capture on 2026-04-03 after startup hardening
      - 7. Root cause decoded from the same capture
      - 8. Live capture on 2026-04-03 after setting `COM_RC_IN_MODE` to no stick input
      - 9. Frame alignment fix (2026-04-06) — RESOLVED
      - 10. Live Motive realignment on 2026-04-07 — yaw fixed, height still confusing
- Opening content: - Active branch: `unstable_l2_realsense`

### `docs/archived/test.md`

- File type: Markdown documentation.
- Tracked size: 330 bytes; 13 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Opening content: source ~/.bashrc ros2 launch drone_bringup external_pose_px4_bridge.launch.py \ pose_source:=optitrack \ optitrack_server:=192.168.0.217 \ rigid_body_name:=RigidBody3

### `docs/archived/vslam_uav.md`

- File type: Markdown documentation.
- Tracked size: 9674 bytes; 275 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - VSLAM-UAV Summary
    - Overview
    - Stated Requirements
    - High-Level Purpose
    - Repository Layout
      - What These Appear To Mean
    - Main VSLAM Directory
    - Key Launch File: `isaac_ros_vslam_realsense.py`
      - What It Launches
      - RealSense Camera Configuration
      - Visual SLAM Configuration
      - Topic Remappings
    - MAVROS / Offboard Bridge: `mavrospy.launch.py`
      - What It Does
      - Important Behavior
      - Default FCU Assumption
    - `vslam_launch.sh`
    - RViz Configuration
- Opening content: Source: - GitHub repository: https://github.com/bandofpv/VSLAM-UAV - Linked tutorial site from the repo: https://bandofpv.github.io/docs/tutorials/robots/vslam

### `docs/generated/drone_studio_perimeter_2d.svg`

- File type: SVG vector image.
- Tracked size: 95215 bytes; 2620 decoded lines.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- SVG structure: 132 path elements, 0 text elements.

### `docs/generated/milestone4_prediction_path.svg`

- File type: SVG vector image.
- Tracked size: 62171 bytes; 1825 decoded lines.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- SVG structure: 85 path elements, 0 text elements.

### `docs/generated/milestone4_target_memory_architecture.svg`

- File type: SVG vector image.
- Tracked size: 54030 bytes; 1482 decoded lines.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- SVG structure: 65 path elements, 0 text elements.

### `docs/guides/setup/d455_firmware_downgrade.md`

- File type: Markdown documentation.
- Tracked size: 7059 bytes; 241 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - D455 Firmware Downgrade Handoff
    - Why This Downgrade Is Being Requested
    - Target Version
    - Official Resources
    - Recommended Host Machine
    - Preferred Method: `rs-fw-update`
      - 1. Download the correct firmware package
      - 2. Make sure no other app is using the camera
      - 3. Connect only the D455 you want to flash
      - 4. List devices
      - 5. Flash the firmware
      - 6. Wait for the tool to finish
      - 7. Re-list the device
    - If The Camera Enters Recovery Mode
    - Acceptable Alternative: RealSense Viewer
    - What To Hand Back After The Downgrade
    - What Happens After You Return The Camera
    - Repo Contact Notes
- Opening content: This document is for the person who will physically take the Intel RealSense D455 and downgrade its firmware for this project.

### `docs/guides/setup/host_setup.md`

- File type: Markdown documentation.
- Tracked size: 1538 bytes; 53 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Host Setup Notes
    - What `setup_jetson.sh` Installs
    - Required Privileges
    - Commands
    - Current Known Blockers On This Machine
- Opening content: This repo now assumes a VIO-first host setup on the Jetson.

### `docs/guides/setup/px4flow_topic_check.md`

- File type: Markdown documentation.
- Tracked size: 2135 bytes; 69 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - PX4Flow Topic Check
    - Commands
    - Expected Success
    - Troubleshooting
- Opening content: Use this props-off bench check after adding an optical flow sensor through the PX4/ARK/MAVLink path. It only observes MAVROS topics and optionally requests a MAVLink message stream; it does not arm, change modes, or tune estimator params.

### `docs/guides/vio/isaac_vslam_d455.md`

- File type: Markdown documentation.
- Tracked size: 4409 bytes; 147 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Isaac VSLAM D455 Bringup
    - Current Status
    - Repo Scripts
    - What The Launch Script Does
    - Expected Topics
    - Observed Runtime Result On 2026-03-24
    - Evidence Folder
    - What This Means For The Rehaul
    - Remaining Gaps
- Opening content: This document records the active Isaac ROS Visual SLAM path for the repo's D455 on the Jetson Orin Nano.

### `docs/milestone3_state_led.md`

- File type: Markdown documentation.
- Tracked size: 2757 bytes; 119 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 3 State Topic and blink(1) LED
    - State Names
    - LED Script
    - blink(1) Permission Fix
- Opening content: The Milestone 3 demo state topic is:

### `docs/milestones/milestone2.md`

- File type: Markdown documentation.
- Tracked size: 8051 bytes; 189 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 2
    - Current Demo Definition
    - Status
    - Feasible Setup
    - Launch Surface
    - Services And Topics
    - Build And Validation
    - Bench Gates Before Flight
    - Flight Gates And Video Acceptance
    - Current Limits
- Opening content: Commands in this note assume your shell has already loaded ROS 2 and the local workspace from `~/.bashrc`.

### `docs/milestones/milestone3.md`

- File type: Markdown documentation.
- Tracked size: 6656 bytes; 209 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 3
    - What Milestone 3 Adds
    - Indoor Speed Profile Fix
    - PX4 Speed Profiles
    - Launch Surface
    - Services And Topics
    - Validation Used For This Revision
    - Current Limits
- Opening content: Commands in this note assume your shell has already loaded ROS 2 and the local workspace from `~/.bashrc`.

### `docs/milestones/milestone4.md`

- File type: Markdown documentation.
- Tracked size: 6448 bytes; 175 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 4
    - What Milestone 4 Adds
    - Interfaces
    - Prediction And Control Defaults
    - Launch
    - Validation
- Opening content: Milestone 4 adds a target-memory layer between RealSense perception and the existing follow/demo controller.

### `docs/notes/offboard_indoor_blocker.md`

- File type: Markdown documentation.
- Tracked size: 5764 bytes; 104 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Current Problem: OFFBOARD Mode Blocked Indoors
    - Status
    - Root Cause
    - What Was Tried
    - System Info
    - Options to Resolve (Not Yet Tried)
      - Option A — Take outside for GPS fix (easiest)
      - Option B — Disable GPS fusion only
      - Option C — Bench-only dummy external vision pose
      - Option D — Real stereo-derived VIO
      - Option E — Switch to attitude setpoints (code change)
    - Recommended Next Session Plan
    - Resume Commands
  - Temporary indoor teleop path
  - In another terminal
  - Check manual-control messages flowing:
  - Keyboard sequence:
  - 1. Press 2 -> ALTCTL
- Opening content: - external vision / VIO - optical flow + rangefinder - attitude/body-rate control instead of velocity OFFBOARD

### `docs/plans/milestone_planner.md`

- File type: Markdown documentation.
- Tracked size: 9528 bytes; 301 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone Planner
    - Active Direction
    - Scope For This Pass
    - Do Not Forget
    - Next Demo Milestone
    - Current Control / Pose Picture
      - Ownship pose today
      - Target coordinates today
    - Transform Model We Need
      - Transform 1: Motive / VRPN world -> repo external pose input
      - Transform 2: external pose input -> PX4 trusted local pose
      - Transform 3: target body coordinates -> body-frame control commands
      - Transform 4: future body target -> map-frame target
    - Minimal Active Follow Design
      - New active node
      - Initial target-selection rule
      - Initial control rule
    - Safety Gates To Add
- Opening content: The repo focus is shifting to:

### `docs/protocols/tracking_experiment_protocol.md`

- File type: Markdown documentation.
- Tracked size: 2543 bytes; 94 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Tracking Experiment Protocol
    - Launch Pattern
    - Report Generation
    - Benchmark Scenarios
    - What To Check
- Opening content: This workflow uses the run-scoped world-track compare bundle produced by `realsense_tracker_node` and the offline `world_track_compare_report` command.

### `docs/reference/d455_vio_contract.md`

- File type: Markdown documentation.
- Tracked size: 2495 bytes; 99 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - D455 VIO Contract
    - Goal
    - Launch Entry Point
    - Default Sensor Profile
    - Expected Topics
    - Expected Rates
    - Frame Expectations
    - Validation
    - USB Link Requirement
- Opening content: This is the active D455 sensor contract for the rehauled repo.

### `docs/reference/isaac_ros_release32.md`

- File type: Markdown documentation.
- Tracked size: 7173 bytes; 176 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Isaac ROS Release 3.2 Path
    - Why This Exists
    - Workspace Location
    - Bootstrap
    - Start The RealSense Dev Container
    - Important Version Guardrail
    - Current Machine Result
    - Post-Reboot Retest
    - Firmware-Downgrade Retest
- Opening content: This repo now carries a pinned Isaac ROS bootstrap path for the D455 + VSLAM fallback.

### `docs/reference/repo_tree.md`

- File type: Markdown documentation.
- Tracked size: 1946 bytes; 76 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Repo Tree
- Opening content: This file documents the post-cleanup repo layout.

### `docs/reports/engine_detection/2026-04-16/writeup.md`

- File type: Markdown documentation.
- Tracked size: 9345 bytes; 162 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - RealSense Static-Depth Evaluation of `best_large_640_100e_77k_fp16.engine`
    - Purpose
    - Test Setup
    - Main Finding
    - Static Depth Ladder Summary
    - What The Error Trend Says
    - Why This Looks Like A Model/Engine Limitation
    - Representative Frames
    - Per-Run Reports
    - Short Version To Send With This
    - Suggested Follow-Up For The Model/Engine Owner
- Opening content: This note is meant to give a concrete, data-backed summary of how the current TensorRT engine behaves in the RealSense tracking pipeline. The goal is to show where tracking quality degrades, where it fully fails, and why the current evidence points much more strongly to model/...

### `docs/reports/engine_detection/2026-04-17/big_drone_email_report.docx`

- File type: Word document report asset.
- Tracked size: 1098933 bytes.
- Purpose: Generated or shared experiment report in Word format retained as documentation evidence.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 1098933 bytes.
- SHA-256 prefix: `eb8bbd30c42272a1` for identity checks.
- Runtime role: packaged Word report; inspect with office tooling if the editable report body is needed.

### `docs/reports/engine_detection/2026-04-17/email_report.docx`

- File type: Word document report asset.
- Tracked size: 1172257 bytes.
- Purpose: Generated or shared experiment report in Word format retained as documentation evidence.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 1172257 bytes.
- SHA-256 prefix: `c3217e1ac926b5dd` for identity checks.
- Runtime role: packaged Word report; inspect with office tooling if the editable report body is needed.

### `docs/reports/engine_detection/2026-04-20/email_frames/01_1p2m_detected.jpg`

- File type: Raster image asset.
- Tracked size: 153252 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 153252 bytes.
- SHA-256 prefix: `8fc21197c6f35ad0` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_frames/02_2p4m_detected.jpg`

- File type: Raster image asset.
- Tracked size: 151397 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 151397 bytes.
- SHA-256 prefix: `a4fec432b7b019f1` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_frames/03_3p6m_detected_not_follow_ready.jpg`

- File type: Raster image asset.
- Tracked size: 151209 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 151209 bytes.
- SHA-256 prefix: `a9d5e5f1fc9763a2` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_frames/04_4p8m_sparse_tracks.jpg`

- File type: Raster image asset.
- Tracked size: 146619 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 146619 bytes.
- SHA-256 prefix: `1db89bfac664f9d8` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_frames/05_6p0m_partial_tracks.jpg`

- File type: Raster image asset.
- Tracked size: 150563 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 150563 bytes.
- SHA-256 prefix: `bb6de9b8a06e7c0d` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_frames/06_7p2m_sparse_tracks.jpg`

- File type: Raster image asset.
- Tracked size: 150103 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 150103 bytes.
- SHA-256 prefix: `5a79c067f0e0ff82` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_frames/07_range_vs_tracking_fraction.png`

- File type: Raster image asset.
- Tracked size: 75283 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 75283 bytes.
- SHA-256 prefix: `076ca29b7ca80b3d` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_frames/08_range_vs_error_p90.png`

- File type: Raster image asset.
- Tracked size: 74676 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 74676 bytes.
- SHA-256 prefix: `d5f15641cd8d1d6e` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/email_report.docx`

- File type: Word document report asset.
- Tracked size: 1060900 bytes.
- Purpose: Generated or shared experiment report in Word format retained as documentation evidence.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 1060900 bytes.
- SHA-256 prefix: `bbc2564aa50876e0` for identity checks.
- Runtime role: packaged Word report; inspect with office tooling if the editable report body is needed.

### `docs/reports/engine_detection/2026-04-20/email_report.html`

- File type: HTML report/document.
- Tracked size: 5898 bytes; 38 decoded lines.
- Purpose: <!DOCTYPE html> <html><head><meta charset="utf-8"> <title>RealSense Bench Check of 16_k_and_drone_studio_realsense_images_model.engine</title> <style> body { font-family: Arial, sans-serif; margin: 0.75in; color: #111; line-height: 1.35; } h1 { font-size: 2...
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- HTML title: `RealSense Bench Check of 16_k_and_drone_studio_realsense_images_model.engine`.

### `docs/reports/engine_detection/2026-04-20/model_report_assets/generated_frames/4p8m_bbox_overlay.jpg`

- File type: Raster image asset.
- Tracked size: 146619 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 146619 bytes.
- SHA-256 prefix: `1db89bfac664f9d8` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/model_report_assets/range_vs_error_p90.png`

- File type: Raster image asset.
- Tracked size: 74676 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 74676 bytes.
- SHA-256 prefix: `d5f15641cd8d1d6e` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/model_report_assets/range_vs_tracking_fraction.png`

- File type: Raster image asset.
- Tracked size: 75283 bytes.
- Purpose: Documentation or report visual asset referenced by repo docs/reports.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- Binary size: 75283 bytes.
- SHA-256 prefix: `076ca29b7ca80b3d` for identity checks.
- Runtime role: report/documentation image; visual meaning comes from the surrounding report and filename.

### `docs/reports/engine_detection/2026-04-20/model_report_assets/static_depth_ladder_summary.csv`

- File type: CSV data table.
- Tracked size: 2096 bytes; 7 decoded lines.
- Purpose: label,run_id,note,frame_rows_total,track_rows_total,frame_window_rows,track_window_rows_matched,truth_range_mean_m,truth_depth_mean_m,detection_fraction_full_run,world_track_fraction_full_run,controller_valid_fraction_full_run,estimated_distance_mean_m,esti...
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- CSV rows: 6 data rows.
- Header: `label,run_id,note,frame_rows_total,track_rows_total,frame_window_rows,track_window_rows_matched,truth_range_mean_m,truth_depth_mean_m,detection_fraction_full_run,world_track_fraction_full_run,controller_valid_fraction_full_run,estimated_distance_mean_m,estimated_depth_mean_m,error_p90_m_window,er...`.

### `docs/reports/engine_detection/2026-04-20/tmp_docx_test.html`

- File type: HTML report/document.
- Tracked size: 183 bytes; 1 decoded lines.
- Purpose: <html><body><h1>Test</h1><p>Hello</p><img src="file:///home/jetson/output_dump/20260416_230929_static_depth_4m/world_track_compare_1776406245_065471845.jpg" width="600"></body></html>
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Documentation/report readers and any scripts that regenerate or compare reports.
- When to touch it: Update when regenerating documentation or report artifacts.
- HTML report without a title tag detected by this generator.

### `docs/reports/engine_detection/2026-04-20/writeup.md`

- File type: Markdown documentation.
- Tracked size: 2983 bytes; 36 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - RealSense Bench Check of 16_k_and_drone_studio_realsense_images_model.engine
    - Purpose
    - Static-depth ladder summary
- Opening content: Ground truth came from mocap. In the April 20, 2026 static-depth ladder, this new engine tracked cleanly through 2.4 m, lost follow readiness by 3.6 m, collapsed sharply at 4.8 m, and stayed unreliable at 6.0 m and 7.2 m. The 6.0 m rerun did recover intermittent detections, bu...

## Model and runtime assets

### `models/16_k_and_drone_studio_realsense_images_model.engine`

- File type: TensorRT model engine binary.
- Tracked size: 53108825 bytes.
- Purpose: Serialized TensorRT detector model used by the RealSense tracking pipeline.
- Main responsibilities: Supplies the compiled detector model used for target tracking inference.
- Important dependencies or consumers: RealSense tracker model-loading path and Jetson TensorRT runtime.
- When to touch it: Replace only when intentionally updating the detector model; verify runtime compatibility on the target Jetson.
- Binary size: 53108825 bytes.
- SHA-256 prefix: `0eac19181bc41b0a` for identity checks.
- Runtime role: serialized TensorRT inference engine loaded by the tracking pipeline on Jetson-class hardware.

## Root files and operator runbooks

### `.clang-format`

- File type: Text file.
- Tracked size: 397 bytes; 19 decoded lines.
- Purpose: Formatting policy for C/C++ style files if they are added or touched.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 19.

### `.codex`

- File type: Text file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Empty marker/config placeholder used by local Codex tooling.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 0.

### `.gitignore`

- File type: Text file.
- Tracked size: 559 bytes; 50 decoded lines.
- Purpose: Repository ignore policy for generated workspaces, caches, logs, and local outputs.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 50.

### `194_paramters.params`

- File type: PX4/ArduPilot parameter file.
- Tracked size: 35849 bytes; 1082 decoded lines.
- Purpose: # Onboard parameters for Vehicle 1 # # Stack: PX4 Pro # Vehicle: Quadrotor # Version: 1.16.0 # Git Revision: 6ea3539157000000 # # Vehicle-Id Component-Id Name Value Type 1 1 ASPD_SCALE_1 1.000000000000000000 9 1 1 BAT1_CAPACITY -1.000000000000000000 9 1 1 B...
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Parameter entries: 1074 active lines.
- Example parameters: `1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1`.

### `DATA_CAPTURE.md`

- File type: Markdown documentation.
- Tracked size: 3290 bytes; 48 decoded lines.
- Purpose: High-level data capture guide for collecting RealSense and related experiment data.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Data Capture Spec — Drone Detector v2
    - Capture priorities (in order)
      - 1. FAR / SMALL drones — top priority
      - 2. Different / non-LED drones
      - 3. Varied environment / lighting
      - 4. Hard negatives (kills false-positives)
    - Capture settings — changes from v1 (important)
    - Feeding new data back into the pipeline
    - What NOT to collect
    - Success criteria for v2
- Opening content: **Why this exists:** v1 recall is capped at **~0.74**, and the failure analysis is unambiguous — **far/small drones are missed ~50% of the time**, and there are only **6 such examples in the entire 4,900-frame v1 set**. No model/architecture change moved this (we tried standar...

### `DEPLOYMENT.md`

- File type: Markdown documentation.
- Tracked size: 5853 bytes; 94 decoded lines.
- Purpose: Deployment checklist and host/drone setup reference for moving the stack onto the Jetson/flight environment.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Drone Detector — Jetson Orin Nano Deployment Guide
    - 1. Model card (what you're deploying)
    - 2. Artifacts (this `deploy/` folder)
    - 3. Jetson prerequisites
    - 4. Build the INT8 engine — Path A (recommended)
  - → drone_n_v3.engine (INT8, calibrated on calib_images/)
    - 5. Build the INT8 engine — Path B (advanced, trtexec)
    - 6. Run + sanity check
  - or: python scripts/infer_image.py drone_n_v3.engine calib_images/run5__frame_000313.jpg
    - 7. Benchmark
  - → mean/p90 latency, FPS
    - 8. Accuracy check (INT8 vs FP32)
    - 9. If INT8 PTQ accuracy is unacceptable → QAT
    - 10. Integration notes
    - 11. Troubleshooting
    - 12. Camera change (D455 → Zed)
- Opening content: **Audience:** engineers deploying the `small_drone` detector onboard. **Goal:** run the model in **INT8** on an **8 GB Jetson Orin Nano** in real time.

### `LICENSE`

- File type: Text file.
- Tracked size: 34523 bytes; 661 decoded lines.
- Purpose: Project license text that governs redistribution and reuse.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Opening content: GNU AFFERO GENERAL PUBLIC LICENSE Version 3, 19 November 2007

### `README.md`

- File type: Markdown documentation.
- Tracked size: 5053 bytes; 100 decoded lines.
- Purpose: Primary overview for the current cdrone3 branch, identity defaults, active features, runbooks, and repo layout.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - cdrone_control
    - Source Of Truth
    - Current Features
    - Current Priorities
    - Runbooks
    - Documentation
    - Repo Layout
    - Future Work
- Opening content: ROS 2 and PX4 workspace for indoor `cdrone` experiments on a Jetson Orin Nano. This branch is the `cdrone3` clone, and the active stack is organized around a single per-drone identity file instead of scattered literals.

### `ardupilot_sitl_ArduCopter_params.parm`

- File type: PX4/ArduPilot parameter file.
- Tracked size: 31162 bytes; 1392 decoded lines.
- Purpose: ACRO_BAL_PITCH 1.000000 ACRO_BAL_ROLL 1.000000 ACRO_OPTIONS 0 ACRO_RP_EXPO 0.300000 ACRO_RP_RATE 360.000000 ACRO_RP_RATE_TC 0.000000 ACRO_THR_MID 0.000000 ACRO_TRAINER 2 ACRO_Y_EXPO 0.000000 ACRO_Y_RATE 202.500000 ACRO_Y_RATE_TC 0.000000 ADSB_TYPE 0 AHRS_CO...
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Parameter entries: 1392 active lines.
- Example parameters: `ACRO_BAL_PITCH, ACRO_BAL_ROLL, ACRO_OPTIONS, ACRO_RP_EXPO, ACRO_RP_RATE, ACRO_RP_RATE_TC, ACRO_THR_MID, ACRO_TRAINER, ACRO_Y_EXPO, ACRO_Y_RATE, ACRO_Y_RATE_TC, ADSB_TYPE, AHRS_COMP_BETA, AHRS_EKF_TYPE, AHRS_GPS_GAIN, AHRS_GPS_MINSATS, AHRS_GPS_USE, AHRS_OPTIONS, AHRS_ORIENTATION, AHRS_RP_P`.

### `build_teleop.sh`

- File type: Shell script.
- Tracked size: 1806 bytes; 59 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Shell safety flags: `set -e`.
- Command map: `echo(44), set(1), cd(1), colcon(1), if(1), else(1), exit(1), fi(1)`.

### `changelog_2026-05-04.md`

- File type: Markdown documentation.
- Tracked size: 1748 bytes; 40 decoded lines.
- Purpose: Narrative documentation, design note, historical note, or operational procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Changelog 2026-05-04
    - Milestone 3 Flight Defaults
    - Why
    - Files Changed
    - cdrone4 Implementation Notes
- Opening content: - Reduced `takeoff_altitude_m` from `3.0` to `2.6` meters for the milestone3 demo. - Raised `external_pose_timeout_s` from `0.25` to `0.75` seconds in `milestone3_demo.launch.py`. - Kept the milestone3 node fallback altitude aligned at `2.6` meters for direct node runs.

### `error_realsense_runbook.md`

- File type: Markdown documentation.
- Tracked size: 11673 bytes; 488 decoded lines.
- Purpose: Operator runbook that records a repeatable field, bench, or demo procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Error RealSense Runbook
    - What This Test Produces
    - Preflight
      - 1. Build the ROS packages
      - 2. Confirm the D455 is visible on the host
      - 3. Confirm OptiTrack topics are live
      - 4. Confirm the report command is installed
    - Tracker Launch
    - Finding the Active Run Directory
    - Standard Test Order
      - Scenario 1: Static Depth Ladder
      - Scenario 2: Axial Motion
      - Scenario 3: Lateral Sweep
      - Scenario 4: Reacquisition
    - Per-Run Execution Checklist
    - Report Generation
    - What To Inspect After Each Run
    - Pass / Fail Gates For Follow Readiness
- Opening content: This runbook is for bench-testing the RealSense tracker error pipeline and generating the depth-vs-error report bundle reliably.

### `librealsense`

- File type: Git submodule pointer / gitlink at commit 38a414419713.
- Git mode: `160000`; object: `38a41441971387197193ad3aeae3cefe6a11f2cb`.
- Purpose: Pins the external librealsense dependency to a specific upstream commit without vendoring its contents.
- Main responsibilities: Pins an external source dependency by commit.
- Important dependencies or consumers: Git submodule tooling and setup scripts that expect librealsense sources.
- When to touch it: Update only when intentionally moving the pinned external dependency commit.
- Gitlink target commit: `38a41441971387197193ad3aeae3cefe6a11f2cb`.
- Contents are not stored in this repository snapshot; clone/update submodules to inspect the external source tree.

### `milestone1_runbook.md`

- File type: Markdown documentation.
- Tracked size: 8187 bytes; 349 decoded lines.
- Purpose: Operator runbook that records a repeatable field, bench, or demo procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 1 Runbook
    - Assumptions
    - One-Time Build
    - Terminal Setup
    - Preflight Checks
    - Phase 1: Hover Demo
    - Phase 2: Short Goto South Of The Pillar
    - Phase 3: Longer Goto To The Circle Staging Point
    - Phase 4: Circle Demo
    - Optional Landing Note
    - Simple Stop Rule
    - Retired Goal
- Opening content: This runbook is for the first flight-test pass of:

### `milestone2_runbook.md`

- File type: Markdown documentation.
- Tracked size: 1042 bytes; 39 decoded lines.
- Purpose: Operator runbook that records a repeatable field, bench, or demo procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 2 Runbook
    - Build
    - Launch
    - Start
    - Abort
- Opening content: Commands below assume your shell already sourced ROS 2 and the workspace from `~/.bashrc`. Detailed notes, status, and validation context live in `docs/milestones/milestone2.md`.

### `milestone3_runbook.md`

- File type: Markdown documentation.
- Tracked size: 3613 bytes; 117 decoded lines.
- Purpose: Operator runbook that records a repeatable field, bench, or demo procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 3 Runbook
    - Build
    - Launch
    - Start
    - Abort
    - Tracking Metrics
    - Speed Tuning
- Opening content: Commands below assume your shell already loaded ROS 2 and the local workspace. Detailed notes live in `docs/milestones/milestone3.md`.

### `milestone4_runbook.md`

- File type: Markdown documentation.
- Tracked size: 7003 bytes; 202 decoded lines.
- Purpose: Operator runbook that records a repeatable field, bench, or demo procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Milestone 4 Runbook
    - Build
    - Generate Visualizations
    - Launch
    - Start
    - Abort
    - Live Checks
    - Rerun Cleaned-Track Postprocess
    - Compare With Milestone 3
    - RViz Markers
    - VSLAM Pose Alternative
- Opening content: Commands below assume your shell already loaded ROS 2 and the local workspace. Detailed notes live in `docs/milestones/milestone4.md`.

### `minjerk_of_compare_runbook.md`

- File type: Markdown documentation.
- Tracked size: 6674 bytes; 227 decoded lines.
- Purpose: Operator runbook that records a repeatable field, bench, or demo procedure.
- Main responsibilities: Records operational, architectural, or historical knowledge for humans.
- Important dependencies or consumers: Developers/operators reading the repo documentation.
- When to touch it: Touch this when procedures, architecture, assumptions, or experimental findings change.
- Heading outline:
  - Minimum-Jerk Mocap vs Optical-Flow Runbook
    - Build
    - Edit Waypoints
    - Props-Off Frame Sanity Check
    - Launch
    - Start
    - Abort
    - Outputs
    - Troubleshooting
- Opening content: This runbook launches an additive test path:

### `requirements-dev.txt`

- File type: Plain text file.
- Tracked size: 28 bytes; 4 decoded lines.
- Purpose: pytest numpy<2 scipy PyYAML
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 4 total, 4 non-empty.
  - Example: `pytest`
  - Example: `numpy<2`
  - Example: `scipy`

### `requirements-jetson.txt`

- File type: Plain text file.
- Tracked size: 111 bytes; 10 decoded lines.
- Purpose: numpy<2 scipy PyYAML norfair opencv-python==4.13.0.92 polars psutil requests pyrealsense2==2.57.7.10387 blink1
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Text lines: 10 total, 10 non-empty.
  - Example: `numpy<2`
  - Example: `scipy`
  - Example: `PyYAML`

### `ros-humble-vrpn-mocap_1.1.0-1jammy.debian.tar.xz`

- File type: Source/package archive.
- Tracked size: 2524 bytes.
- Purpose: Pinned packaging artifact for dependencies used by the flight stack.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Binary size: 2524 bytes.
- SHA-256 prefix: `b6532907b1c1d417` for identity checks.
- Runtime role: pinned package/source archive for reproducible dependency setup.

### `ros-humble-vrpn-mocap_1.1.0-1jammy.dsc`

- File type: Debian source-control metadata.
- Tracked size: 1192 bytes; 19 decoded lines.
- Purpose: Pinned packaging artifact for dependencies used by the flight stack.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Debian source fields:
  - `Format: 3.0 (quilt)`
  - `Source: ros-humble-vrpn-mocap`
  - `Binary: ros-humble-vrpn-mocap`
  - `Architecture: any`
  - `Version: 1.1.0-1jammy`
  - `Maintainer: Alvin Sun <alvinsunyixiao@gmail.com>`
  - `Standards-Version: 3.9.2`
  - `Build-Depends: debhelper (>= 9.0.0), libeigen3-dev, ros-humble-ament-cmake, ros-humble-ament-lint-auto <!nocheck>, ros-humble-ament-lint-common <!nocheck>, ros-humble-eigen3-cmake-module, ros-humbl...`

### `ros-humble-vrpn-mocap_1.1.0.orig.tar.gz`

- File type: Source/package archive.
- Tracked size: 8251 bytes.
- Purpose: Pinned packaging artifact for dependencies used by the flight stack.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- Binary size: 8251 bytes.
- SHA-256 prefix: `b0e8f5458ddb2b61` for identity checks.
- Runtime role: pinned package/source archive for reproducible dependency setup.

### `setup_jetson.sh`

- File type: Shell script.
- Tracked size: 9971 bytes; 366 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Shell safety flags: `set -euo pipefail; set +u; set -u; set +u; set -u`.
- Defined shell functions: `require_sudo, ensure_ros_apt_source, install_system_packages, install_geographiclib, install_realsense_rules, init_rosdep, install_python_requirements, install_ultralytics_runtime, build_workspace, verify_workspace, verify_tracker_runtime, main`.
- Command map: `if(30), fi(21), sudo(13), import(13), echo(11), local(10), raise(7), source(6), exit(6), set(5), python3(5), PY(5), cat(4), EOF(4), return(4), then(4), for(4), else(2), This(2), -o(2)`.

### `test_mavlink_router_connection.sh`

- File type: Shell script.
- Tracked size: 3137 bytes; 90 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Command map: `echo(46), if(5), else(5), fi(5), grep(5), exit(2), ps(1), ss(1), ls(1), date(1)`.

## Tracked runtime logs

### `runtime_logs/manual_milestone3_cdrone3_20260424_210329.log`

- File type: Text file.
- Tracked size: 181 bytes; 1 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 1 total, 1 non-empty.
  - Example: `file 'milestone3_demo.launch.py' was not found in the share directory of package 'drone_bringup' which is at '/home/jetson/cdrone_control/install/drone_bringup/share/drone_bringup'`

### `runtime_logs/manual_milestone3_cdrone3_20260424_210543.log`

- File type: Text file.
- Tracked size: 52313 bytes; 385 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 385 total, 385 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-24-21-05-43-567227-jetson-7200`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [7201]`

### `runtime_logs/manual_milestone3_cdrone3_20260424_211518.log`

- File type: Text file.
- Tracked size: 28999 bytes; 237 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 237 total, 237 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-24-21-15-18-999154-jetson-8837`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [8838]`

### `runtime_logs/manual_milestone3_cdrone3_20260424_211655.log`

- File type: Text file.
- Tracked size: 29514 bytes; 241 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 241 total, 241 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-24-21-16-55-714895-jetson-9360`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [9361]`

### `runtime_logs/manual_milestone3_cdrone3_20260424_212259.log`

- File type: Text file.
- Tracked size: 57571 bytes; 415 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 415 total, 415 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-24-21-22-59-753016-jetson-10432`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [10433]`

### `runtime_logs/milestone3_cdrone3_20260424_192804.log`

- File type: Text file.
- Tracked size: 0 bytes; 0 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 0 total, 0 non-empty.

### `runtime_logs/milestone3_dual_jetson_20260427_154819.log`

- File type: Text file.
- Tracked size: 181 bytes; 1 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 1 total, 1 non-empty.
  - Example: `file 'milestone3_demo.launch.py' was not found in the share directory of package 'drone_bringup' which is at '/home/jetson/cdrone_control/install/drone_bringup/share/drone_bringup'`

### `runtime_logs/milestone3_dual_jetson_20260427_155928.log`

- File type: Text file.
- Tracked size: 49516 bytes; 366 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 366 total, 366 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-27-15-59-30-183831-jetson-14578`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [14588]`

### `runtime_logs/milestone3_dual_jetson_20260427_162128.log`

- File type: Text file.
- Tracked size: 61322 bytes; 436 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 436 total, 436 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-27-16-21-30-857134-jetson-27177`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [27187]`

### `runtime_logs/milestone3_dual_jetson_20260427_170941.log`

- File type: Text file.
- Tracked size: 62889 bytes; 458 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 458 total, 458 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-27-17-09-43-976695-jetson-4148`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [4150]`

### `runtime_logs/milestone3_dual_jetson_20260427_175009.log`

- File type: Text file.
- Tracked size: 52480 bytes; 384 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 384 total, 384 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-27-17-50-11-346476-jetson-6743`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [6745]`

### `runtime_logs/milestone3_dual_jetson_20260427_184421.log`

- File type: Text file.
- Tracked size: 108832 bytes; 737 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 737 total, 737 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-27-18-44-22-953777-jetson-3780`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [3784]`

### `runtime_logs/milestone3_flighttest_cdrone3_20260424_203946.log`

- File type: Text file.
- Tracked size: 50305 bytes; 372 decoded lines.
- Purpose: Tracked historical runtime log kept as evidence for milestone/debug context.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Humans debugging historical flight/demo behavior.
- When to touch it: Usually do not edit; keep as immutable historical evidence unless curating tracked artifacts.
- Text lines: 372 total, 372 non-empty.
  - Example: `[INFO] [launch]: All log files can be found below /home/jetson/.ros/log/2026-04-24-20-39-47-736906-jetson-4767`
  - Example: `[INFO] [launch]: Default logging verbosity is set to INFO`
  - Example: `[INFO] [mavros_node-1]: process started with pid [4793]`

## Utility scripts

### `scripts/apply_px4_speed_profile.py`

- File type: Python source file.
- Tracked size: 13564 bytes; 443 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines classes Profile; defines functions resolve_profile_path, load_profile, build_commands, encode_param_id, decode_param_id, candidate_ports, prime_router_route, wait_for_param_value, request_param_value, set_param_value and more; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import os; import sys; import time; from dataclasses import dataclass; from pathlib import Path; from pymavlink import mavutil`
  - third-party: `import yaml`
- Top-level constants/state: `REPO_ROOT, CONFIG_DIR, DEFAULT_PX4_PORT, DEFAULT_READ_TIMEOUT_S, DEFAULT_RETRIES, DEFAULT_TARGET_SYSTEM, DEFAULT_TARGET_COMPONENT, DEFAULT_SOURCE_SYSTEM, DEFAULT_SOURCE_COMPONENT, DEFAULT_TOLERANCE, PROFILE_ALIASES`.
- Classes:
  - `Profile` (lines 41-45, bases: `object`): No class docstring; role is inferred from methods and base classes.
- Top-level functions:
  - `resolve_profile_path(profile: str) -> Path` (lines 48-53): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points.
    Notable calls: PROFILE_ALIASES.get, Path.expanduser, profile.strip, path.is_absolute, Path.
  - `load_profile(path: Path) -> Profile` (lines 56-82): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 1 for loops; 1 return points; 4 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: loaded.get, raw_parameters.items, str.strip, Profile, path.is_file, FileNotFoundError, yaml.safe_load, isinstance, ValueError, path.read_text.
  - `build_commands(parameters: dict[str, float]) -> str` (lines 85-89): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: lines.append, lines.extend, '\n'.join, parameters.items.
  - `encode_param_id(name: str) -> bytes` (lines 92-93): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: str.encode.ljust, str.encode.
  - `decode_param_id(raw: object) -> str` (lines 96-99): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: isinstance, str.rstrip, bytes.decode.rstrip, bytes.decode, bytes.
  - `candidate_ports(explicit_port: str) -> list[str]` (lines 102-122): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 1 for loops; 1 return points.
    Notable calls: str.strip, add, candidates.append, candidate.startswith, os.environ.get.
  - `prime_router_route(connection: object) -> None` (lines 125-132): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Notable calls: connection.mav.heartbeat_send.
  - `wait_for_param_value(connection: object, param_name: str, *, timeout_s: float) -> object | None` (lines 135-154): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 while loops; 2 return points.
    Notable calls: time.monotonic, connection.recv_match, decode_param_id.
  - `request_param_value(connection: object, param_name: str, *, target_system: int, target_component: int, timeout_s: float, retries: int) -> float` (lines 157-178): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points; 1 raise statements.
    Notable calls: encode_param_id, TimeoutError, prime_router_route, connection.mav.param_request_read_send, wait_for_param_value.
  - `set_param_value(connection: object, param_name: str, desired_value: float, *, target_system: int, target_component: int, timeout_s: float, retries: int, tolerance: float) -> float` (lines 181-210): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 1 return points; 1 raise statements.
    Notable calls: encode_param_id, TimeoutError, prime_router_route, connection.mav.param_set_send, wait_for_param_value.
  - `connect_and_probe(ports: list[str], probe_param_name: str, *, source_system: int, source_component: int, target_system: int, target_component: int, timeout_s: float, retries: int) -> tuple[object, str]` (lines 213-256): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 2 try/except blocks; 1 return points; 1 raise statements.
    Notable calls: RuntimeError, '\n'.join, mavutil.mavlink_connection, request_param_value, errors.append, connection.close.
  - `format_value(value: float) -> str` (lines 259-260): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `parse_args() -> argparse.Namespace` (lines 263-333): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args.
  - `main() -> int` (lines 336-439): No docstring; behavior is described from its body and call sites.
    Code map: 6 conditional branches; 4 for loops; 2 try/except blocks; 4 return points.
    Notable calls: parse_args, resolve_profile_path, load_profile, build_commands, candidate_ports, profile.parameters.items, next, connect_and_probe, iter, cached_values.get, set_param_value, request_param_value, connection.close, format_value.
- Whole-file runtime themes: loads or writes YAML, defines a CLI.

### `scripts/blink1/99-blink1.rules`

- File type: udev rules file.
- Tracked size: 297 bytes; 3 decoded lines.
- Purpose: # ThingM blink(1) mk3 USB HID access for the blink1 Python library.
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- udev rules: 2 active rule lines.
  - `SUBSYSTEM=="hidraw", ATTRS{idVendor}=="27b8", ATTRS{idProduct}=="01ed", MODE="0666", GROUP="plugdev", TAG+="uaccess"`
  - `SUBSYSTEM=="usb", ATTR{idVendor}=="27b8", ATTR{idProduct}=="01ed", MODE="0666", GROUP="plugdev", TAG+="uaccess"`

### `scripts/calibrate_perimeter.py`

- File type: Python source file.
- Tracked size: 12695 bytes; 388 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines classes PoseSample, PoseCaptureNode; defines functions load_drone_config, join_topic, quat_to_yaw_rad, round_float, wait_for_pose, average_pose, capture_named_sample, parse_args, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Interactive studio perimeter calibration helper.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import math; import sys; import time; from dataclasses import dataclass; from datetime import datetime, timezone; from pathlib import Path; from typing import Optional`
  - third-party: `import numpy as np; import yaml`
  - ROS/runtime: `import rclpy; from geometry_msgs.msg import PoseStamped; from rclpy.node import Node; from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy`
- Top-level constants/state: `DRONE_DEFAULTS, DEFAULT_DRONE_ID, DEFAULT_MAVROS_NAMESPACE, DEFAULT_POSE_TOPIC`.
- Classes:
  - `PoseSample` (lines 91-106, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `label, x_m, y_m, z_m, yaw_rad`.
    - `to_yaml_dict(self) -> dict[str, float | str]` (lines 99-106): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
      Runtime interactions: loads or writes YAML.
      Notable calls: round_float.
  - `PoseCaptureNode` (lines 109-125, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `latest_pose, create_subscription, pose_callback`.
    - `__init__(self, topic: str, best_effort: bool) -> None` (lines 110-122): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: creates subscriptions, handles pose messages.
      Notable calls: super.__init__, QoSProfile, self.create_subscription.
    - `pose_callback(self, msg: PoseStamped) -> None` (lines 124-125): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: handles pose messages.
- Top-level functions:
  - `load_drone_config() -> dict[str, object]` (lines 44-56): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 1 return points; 1 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: yaml.safe_load, isinstance, ValueError, loaded.get, Path.resolve, config_path.read_text, merged.update, Path.
  - `join_topic(namespace: str, leaf: str) -> str` (lines 59-65): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: str.strip.rstrip, namespace.startswith, str.lstrip, str.strip.
  - `quat_to_yaw_rad(msg: PoseStamped) -> float` (lines 79-83): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: handles pose messages.
    Notable calls: math.atan2.
  - `round_float(value: float) -> float` (lines 86-87): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `wait_for_pose(node: PoseCaptureNode, timeout_s: float) -> PoseStamped` (lines 128-134): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 while loops; 1 return points; 1 raise statements.
    Runtime interactions: handles pose messages.
    Notable calls: TimeoutError, time.monotonic, rclpy.spin_once.
  - `average_pose(node: PoseCaptureNode, sample_count: int, sample_interval_s: float) -> PoseSample` (lines 137-170): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 while loops; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: math.atan2, PoseSample, rclpy.spin_once, xs.append, ys.append, zs.append, yaws.append, time.sleep, np.mean, quat_to_yaw_rad, np.cos, np.sin, np.asarray.
  - `capture_named_sample(node: PoseCaptureNode, label: str, sample_count: int, sample_interval_s: float) -> Optional[PoseSample]` (lines 173-202): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 while loops; 2 return points.
    Notable calls: input.strip.lower, average_pose, input.strip, math.degrees, input.
  - `parse_args() -> argparse.Namespace` (lines 205-274): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: loads or writes YAML, defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args.
  - `main() -> int` (lines 277-384): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 2 for loops; 1 try/except blocks; 1 context managers; 3 return points; 1 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: parse_args, rclpy.init, PoseCaptureNode, wait_for_pose, Path, output_path.parent.mkdir, node.destroy_node, rclpy.ok, args.frame_id.strip, capture_named_sample, RuntimeError, datetime.now.isoformat, output.append, output_path.open, yaml.safe_dump, rclpy.shutdown, boundary_samples.append, pillar_samples.append....
- Whole-file runtime themes: creates subscriptions, handles pose messages, uses NumPy arrays/math, loads or writes YAML, defines a CLI.

### `scripts/capture_isaac_vslam_outputs.sh`

- File type: Shell script.
- Tracked size: 3055 bytes; 82 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `docker(12), -(8), echo(3), set(1), if(1), exit(1), fi(1), mkdir(1), cat(1), Isaac(1), Key(1), EOF(1)`.

### `scripts/capture_position_hover_debug.sh`

- File type: Shell script.
- Tracked size: 18465 bytes; 623 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail; set +u; set -u; set +e; set +e; set +e`.
- Defined shell functions: `read_drone_config_value, usage, normalize_ns, source_ros_env, stop_background_jobs, signal_process_tree, generate_summary, handle_exit, start_field_trace, have_tailed_log_key, register_demo_logs, monitor_demo_until_terminal_state`.
- Command map: `if(47), fi(24), echo(21), local(18), for(17), return(14), done(10), or(10), continue(8), set(6), exit(6), printf(6), ros2(6), shift(5), import(4), declare(4), scripts/capture_position_hover_debug.sh(4), start_field_trace(4), source(3), while(3)`.

### `scripts/check_d455_topics.sh`

- File type: Shell script.
- Tracked size: 918 bytes; 43 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(8), if(3), fi(3), for(2), done(2), set(1), exit(1), else(1), continue(1), timeout(1)`.

### `scripts/check_px4flow_topics.sh`

- File type: Shell script.
- Tracked size: 10841 bytes; 393 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail; set +u; set -u`.
- Defined shell functions: `usage, log, warn, fail, read_drone_config_value, normalize_ns, source_ros_env, ensure_ros_log_dir, ros2_list_visible_names, visible_count, print_graph_hint, topic_exists, service_exists, validate_positive_integer, validate_positive_number, request_stream, check_mavros_connection, inspect_topic`.
- Command map: `if(31), local(28), fi(28), printf(11), return(11), warn(10), echo(9), log(8), scripts/check_px4flow_topics.sh(5), shift(4), set(3), exit(3), import(3), mkdir(3), fail(3), grep(3), else(3), python3(2), for(2), value(2)`.

### `scripts/check_realsense_host_access.sh`

- File type: Shell script.
- Tracked size: 516 bytes; 19 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(5), lsusb(2), set(1), rs-enumerate-devices(1), while(1), done(1)`.

### `scripts/check_vio_host_status.sh`

- File type: Shell script.
- Tracked size: 1217 bytes; 60 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/bin/bash`.
- Shell safety flags: `set -euo pipefail`.
- Defined shell functions: `check_pkg`.
- Command map: `echo(21), check_pkg(7), if(6), else(6), fi(6), set(1), local(1)`.

### `scripts/diagnose_mocap_frame.py`

- File type: Python source file.
- Tracked size: 10092 bytes; 290 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines classes FrameDiagNode; defines functions load_drone_config, join_topic, quat_to_euler, fmt_pose, sample, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Diagnose mocap frame alignment.
- Imports by role:
  - standard library: `import sys; import time; import threading; from pathlib import Path; import math`
  - third-party: `import yaml`
  - ROS/runtime: `import rclpy; from rclpy.node import Node; from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy; from geometry_msgs.msg import PoseStamped`
- Top-level constants/state: `DRONE_DEFAULTS, DRONE_ID, OWNERSHIP_RIGID_BODY, VRPN_POSE_TOPIC, MAVROS_NAMESPACE, VISION_POSE_TOPIC`.
- Classes:
  - `FrameDiagNode` (lines 82-120, bases: `Node`): No class docstring; role is inferred from methods and base classes.
    State touched: `vrpn_pose, mavros_pose, lock, create_subscription, vrpn_cb, mavros_cb`.
    - `__init__(self)` (lines 83-108): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Runtime interactions: creates subscriptions, handles pose messages.
      Notable calls: super.__init__, threading.Lock, QoSProfile, self.create_subscription.
    - `vrpn_cb(self, msg)` (lines 110-112): No docstring; behavior is described from its body and call sites.
      Code map: 1 context managers.
    - `mavros_cb(self, msg)` (lines 114-116): No docstring; behavior is described from its body and call sites.
      Code map: 1 context managers.
    - `snapshot(self)` (lines 118-120): No docstring; behavior is described from its body and call sites.
      Code map: 1 context managers; 1 return points.
- Top-level functions:
  - `load_drone_config() -> dict` (lines 29-41): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 for loops; 1 return points; 1 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: yaml.safe_load, isinstance, ValueError, loaded.get, Path.resolve, config_path.read_text, merged.update, Path.
  - `join_topic(namespace: str, leaf: str) -> str` (lines 44-50): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: str.strip.rstrip, namespace.startswith, str.lstrip, str.strip.
  - `quat_to_euler(q)` (lines 65-79): Convert quaternion (x,y,z,w) to roll,pitch,yaw in degrees.
    Code map: 1 return points.
    Notable calls: math.atan2, math.asin, math.degrees.
  - `fmt_pose(msg)` (lines 123-132): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: quat_to_euler.
  - `sample(node, label, n = 20, rate = 0.05)` (lines 135-168): Collect n samples and return the average VRPN and MAVROS poses.
    Code map: 5 conditional branches; 1 for loops; 2 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.mean, rclpy.spin_once, node.snapshot, time.sleep, vrpn_samples.append, mavros_samples.append, quat_to_euler.
  - `main()` (lines 171-286): No docstring; behavior is described from its body and call sites.
    Code map: 7 conditional branches; 1 for loops; 2 while loops; 1 return points.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: rclpy.init, FrameDiagNode, input, sample, node.destroy_node, rclpy.shutdown, rclpy.spin_once, node.snapshot, np.array, np.argmax, quat_to_euler, np.abs.
- Whole-file runtime themes: creates subscriptions, handles pose messages, uses NumPy arrays/math, loads or writes YAML.

### `scripts/ensure_isaac_vslam_deps_in_container.sh`

- File type: Shell script.
- Tracked size: 1190 bytes; 41 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(5), if(3), fi(3), exit(2), set(1), ros-humble-isaac-ros-visual-slam(1), ros-humble-isaac-ros-examples(1), ros-humble-isaac-ros-realsense(1), curl(1), jq(1), tar(1), for(1), done(1), printf(1), docker(1)`.

### `scripts/generate_static_depth_email_report.py`

- File type: Python source file.
- Tracked size: 24884 bytes; 757 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines classes RunSpec, RunMetrics; defines functions parse_args, parse_run_spec, read_csv_rows, optional_float, optional_bool, safe_mean, percentile, extract_series, pick_representative_image, load_ready_for_follow and more; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Generate an email-style static-depth ladder report from run bundles.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import csv; import html; import json; import math; import shutil; import subprocess; from dataclasses import dataclass; from pathlib import Path; from docx import Document; from docx.enum.text import WD_ALIGN_PARAGRAPH; from docx.shared import Inches, Pt`
  - third-party: `import matplotlib; from matplotlib import pyplot as plt`
- Classes:
  - `RunSpec` (lines 27-33, bases: `object`): No class docstring; role is inferred from methods and base classes.
  - `RunMetrics` (lines 37-54, bases: `object`): No class docstring; role is inferred from methods and base classes.
- Top-level functions:
  - `parse_args() -> argparse.Namespace` (lines 57-82): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args.
  - `parse_run_spec(value: str) -> RunSpec` (lines 85-101): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Notable calls: RunSpec, part.strip, ValueError, Path, value.split.
  - `read_csv_rows(path: Path) -> list[dict[str, str]]` (lines 104-106): No docstring; behavior is described from its body and call sites.
    Code map: 1 context managers; 1 return points.
    Notable calls: path.open, csv.DictReader.
  - `optional_float(value: str | None) -> float | None` (lines 109-121): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 try/except blocks; 5 return points.
    Notable calls: str.strip, math.isfinite.
  - `optional_bool(value: str | None) -> bool | None` (lines 124-134): No docstring; behavior is described from its body and call sites.
    Code map: 4 conditional branches; 5 return points.
    Notable calls: str.strip.lower, str.strip.
  - `safe_mean(values: list[float]) -> float | None` (lines 137-140): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `percentile(values: list[float], q: float) -> float | None` (lines 143-152): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 3 return points.
    Notable calls: sorted, math.floor, math.ceil.
  - `extract_series(rows: list[dict[str, str]], key: str) -> list[float]` (lines 155-161): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points.
    Notable calls: optional_float, row.get, values.append.
  - `pick_representative_image(run_dir: Path) -> Path | None` (lines 164-168): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
    Notable calls: sorted, run_dir.glob.
  - `load_ready_for_follow(run_dir: Path) -> bool | None` (lines 171-182): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 4 return points.
    Notable calls: json.loads, data.get, follow_readiness.get, isinstance, summary_path.exists, summary_path.read_text.
  - `summarize_run(spec: RunSpec, window_size_frames: int) -> RunMetrics` (lines 185-253): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 1 for loops; 1 return points.
    Notable calls: read_csv_rows, extract_series, RunMetrics, optional_float, frame_window.get, safe_mean, percentile, load_ready_for_follow, row.get, track_window.append, pick_representative_image, optional_bool.
  - `ensure_clean_dir(path: Path) -> None` (lines 256-259): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches.
    Notable calls: path.exists, path.mkdir, shutil.rmtree.
  - `format_float(value: float | None, digits: int = 3) -> str` (lines 262-265): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `format_percent(value: float | None) -> str` (lines 268-271): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 2 return points.
  - `write_summary_csv(metrics: list[RunMetrics], summary_csv_path: Path) -> None` (lines 274-320): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 context managers.
    Notable calls: summary_csv_path.parent.mkdir, summary_csv_path.open, csv.writer, writer.writerow.
  - `make_line_plot(metrics: list[RunMetrics], *, y_getter, ylabel: str, title: str, output_path: Path, color: str) -> None` (lines 323-365): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 2 for loops.
    Notable calls: plt.figure, plt.xlabel, plt.ylabel, plt.title, plt.grid, plt.tight_layout, output_path.parent.mkdir, plt.savefig, plt.close, y_getter, xs.append, ys.append, labels.append, plt.plot, ylabel.lower, plt.ylim, plt.annotate.
  - `copy_report_assets(metrics: list[RunMetrics], *, email_frames_dir: Path, coverage_plot_src: Path, error_plot_src: Path) -> tuple[Path, Path]` (lines 368-388): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 1 return points.
    Notable calls: ensure_clean_dir, shutil.copy2.
  - `build_run_caption(item: RunMetrics) -> str` (lines 391-417): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 2 return points.
    Notable calls: item.spec.note.strip, prefix.endswith, format_float, format_percent.
  - `write_html_report(*, title: str, topnote: str, coverage_title: str, coverage_image: Path, coverage_caption: str, error_title: str, error_image: Path, error_caption: str, metrics: list[RunMetrics], html_path: Path) -> None` (lines 420-490): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops.
    Notable calls: parts.append, html_path.write_text, build_run_caption, parts.extend, '\n'.join, html.escape.
  - `add_picture_paragraph(document: Document, image_path: Path) -> None` (lines 493-497): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Notable calls: document.add_paragraph, paragraph.add_run, run.add_picture, Inches.
  - `add_caption_paragraph(document: Document, text: str) -> None` (lines 500-503): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Notable calls: document.add_paragraph, Pt, paragraph.add_run.
  - `write_docx_report(*, title: str, topnote: str, coverage_title: str, coverage_image: Path, coverage_caption: str, error_title: str, error_image: Path, error_caption: str, metrics: list[RunMetrics], docx_path: Path, email_frames_dir: Path) -> None` (lines 506-565): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops.
    Notable calls: Document, Inches, document.add_paragraph, title_para.add_run, Pt, topnote_para.add_run, add_picture_paragraph, add_caption_paragraph, source_para.add_run, document.save, overview_heading.add_run, error_heading.add_run, document.add_page_break, heading.add_run, build_run_caption.
  - `convert_docx_to_odt(docx_path: Path, odt_path: Path) -> None` (lines 568-590): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches.
    Notable calls: odt_path.parent.mkdir, subprocess.run, result.stdout.strip, result.stderr.strip, shutil.move.
  - `write_markdown_writeup(*, title: str, summary_heading: str, topnote: str, metrics: list[RunMetrics], writeup_path: Path, summary_csv_path: Path, coverage_plot_path: Path, error_plot_path: Path) -> None` (lines 593-662): No docstring; behavior is described from its body and call sites.
    Code map: 2 for loops.
    Notable calls: lines.extend, writeup_path.write_text, lines.append, '\n'.join, ' | '.join, format_float.
  - `main() -> None` (lines 665-753): No docstring; behavior is described from its body and call sites.
    Code map: 2 conditional branches; 1 raise statements.
    Notable calls: parse_args, Path, assets_dir.mkdir, write_summary_csv, make_line_plot, copy_report_assets, write_html_report, write_docx_report, write_markdown_writeup, parse_run_spec, SystemExit, summarize_run, convert_docx_to_odt.
- Whole-file runtime themes: defines a CLI.

### `scripts/install_blink1_host_access.sh`

- File type: Shell script.
- Tracked size: 610 bytes; 24 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(5), if(2), exit(2), fi(2), udevadm(2), set(1), install(1)`.

### `scripts/install_realsense_host_access.sh`

- File type: Shell script.
- Tracked size: 629 bytes; 24 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(5), if(2), exit(2), fi(2), udevadm(2), set(1), install(1)`.

### `scripts/launch_isaac_realsense_d455_in_container.sh`

- File type: Shell script.
- Tracked size: 3142 bytes; 74 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(9), if(3), exit(3), docker(3), sleep(2), set(1), fi(1), pkill(1), ros2(1)`.

### `scripts/launch_isaac_vslam_d455_in_container.sh`

- File type: Shell script.
- Tracked size: 2547 bytes; 54 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(8), if(3), exit(3), docker(3), pkill(2), sleep(2), set(1), fi(1), ros2(1)`.

### `scripts/led_control.py`

- File type: Python source file.
- Tracked size: 9678 bytes; 331 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines classes Color, DryRunLight, Blink1Light; defines functions repo_root, default_state_topic, color_for_engagement_state, make_light, blink1_error_help, parse_args, run_test_color, run_ros, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import os; import sys; from dataclasses import dataclass; from pathlib import Path; from typing import Any`
- Top-level constants/state: `DEFAULT_ENGAGEMENT_STATE_TOPIC, COLORS, TEST_COLORS, STARTUP_STATES, RETURN_STATES, LANDING_STATES`.
- Classes:
  - `Color` (lines 21-29, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `r, g, b`.
    - `rgb(self) -> tuple[int, int, int]` (lines 28-29): No docstring; behavior is described from its body and call sites.
      Code map: 1 return points.
  - `DryRunLight` (lines 153-159, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `set_color`.
    - `set_color(self, color: Color) -> None` (lines 154-155): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
    - `close(self, *, turn_off: bool = True) -> None` (lines 157-159): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self.set_color.
  - `Blink1Light` (lines 162-175, bases: `object`): No class docstring; role is inferred from methods and base classes.
    State touched: `_blink, _fade_ms, set_color`.
    - `__init__(self, fade_ms: int) -> None` (lines 163-167): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: Blink1.
    - `set_color(self, color: Color) -> None` (lines 169-170): No docstring; behavior is described from its body and call sites.
      Code map: straight-line helper logic.
      Notable calls: self._blink.fade_to_rgb.
    - `close(self, *, turn_off: bool = True) -> None` (lines 172-175): No docstring; behavior is described from its body and call sites.
      Code map: 1 conditional branches.
      Notable calls: self._blink.close, self.set_color.
- Top-level functions:
  - `repo_root() -> Path` (lines 88-89): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Path.resolve, Path.
  - `default_state_topic() -> str` (lines 92-120): No docstring; behavior is described from its body and call sites.
    Code map: 5 conditional branches; 1 try/except blocks; 6 return points.
    Runtime interactions: loads or writes YAML.
    Notable calls: config.get, str.strip, os.environ.get.strip, repo_root, isinstance, config_path.is_file, yaml.safe_load, os.environ.get, config_path.read_text, identity.get.
  - `color_for_engagement_state(*, state: str, blocked_reason: str = '', estop_latched: bool = False) -> Color` (lines 123-150): No docstring; behavior is described from its body and call sites.
    Code map: 10 conditional branches; 11 return points.
    Notable calls: str.strip.upper, str.strip.
  - `make_light(*, dry_run: bool, fade_ms: int) -> DryRunLight | Blink1Light` (lines 178-184): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks; 2 return points; 1 raise statements.
    Notable calls: DryRunLight, Blink1Light, RuntimeError, blink1_error_help.
  - `blink1_error_help(exc: Exception) -> str` (lines 187-195): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: repo_root.
  - `parse_args() -> argparse.Namespace` (lines 198-225): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, default_state_topic, sorted.
  - `run_test_color(args: argparse.Namespace) -> int` (lines 228-234): No docstring; behavior is described from its body and call sites.
    Code map: 1 try/except blocks; 1 return points.
    Runtime interactions: defines a CLI.
    Notable calls: make_light, light.set_color, light.close.
  - `run_ros(args: argparse.Namespace) -> int` (lines 237-316): No docstring; behavior is described from its body and call sites.
    Code map: 3 conditional branches; 2 try/except blocks; 2 return points.
    Runtime interactions: creates subscriptions, defines a CLI.
    Notable calls: make_light, rclpy.init, Milestone3StateLedNode, rclpy.spin, node.destroy_node, light.close, rclpy.ok, super.__init__, self.create_subscription, self._apply_color, str.strip.upper, str.strip, color_for_engagement_state, rclpy.shutdown, light.set_color.
  - `main() -> int` (lines 319-327): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 try/except blocks; 3 return points.
    Notable calls: parse_args, run_ros, run_test_color.
- Whole-file runtime themes: creates subscriptions, loads or writes YAML, defines a CLI.

### `scripts/of_compare_stats.py`

- File type: Python source file.
- Tracked size: 399 bytes; 17 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from pathlib import Path; import sys`
  - local/project: `from drone_control_pkg.of_compare_stats import main`
- Top-level constants/state: `REPO_ROOT, PKG_SRC`.

### `scripts/plot_milestone4_prediction_demo.py`

- File type: Python source file.
- Tracked size: 6264 bytes; 236 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines functions draw_box, draw_arrow, write_architecture_svg, write_prediction_path_svg, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; from pathlib import Path`
  - third-party: `import matplotlib.pyplot as plt; from matplotlib.patches import Circle, FancyArrowPatch, Rectangle; import numpy as np`
- Top-level constants/state: `REPO_ROOT, OUTPUT_DIR`.
- Top-level functions:
  - `draw_box(ax, xy, width, height, label, *, facecolor, edgecolor = '#253238')` (lines 16-36): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: Rectangle, ax.add_patch, ax.text.
  - `draw_arrow(ax, start, end, label, *, color = '#334e68', rad = 0.0)` (lines 39-61): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Notable calls: FancyArrowPatch, ax.add_patch, ax.text.
  - `write_architecture_svg(output_path: Path) -> None` (lines 64-147): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Notable calls: plt.subplots, ax.set_xlim, ax.set_ylim, ax.axis, draw_box, draw_arrow, ax.text, fig.tight_layout, fig.savefig, plt.close.
  - `write_prediction_path_svg(output_path: Path) -> None` (lines 150-222): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops.
    Runtime interactions: uses NumPy arrays/math.
    Notable calls: np.array, plt.subplots, ax.set_aspect, ax.set_xlim, ax.set_ylim, ax.grid, ax.set_xlabel, ax.set_ylabel, ax.set_title, ax.plot, ax.scatter, ax.annotate, ax.legend, fig.tight_layout, fig.savefig, plt.close, Circle, ax.add_patch....
  - `main() -> None` (lines 225-232): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Notable calls: OUTPUT_DIR.mkdir, write_architecture_svg, write_prediction_path_svg.
- Whole-file runtime themes: uses NumPy arrays/math.

### `scripts/plot_perimeter_2d.py`

- File type: Python source file.
- Tracked size: 6913 bytes; 231 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines functions load_config, closed_xy, annotate_vertices, polygon_centroid, max_radius_from_center, main; provides a CLI/ROS entry point.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: No module docstring; purpose is inferred from file path, imports, and symbols.
- Imports by role:
  - standard library: `from __future__ import annotations; import argparse; import math; from pathlib import Path`
  - third-party: `import matplotlib.pyplot as plt; import yaml`
- Top-level functions:
  - `load_config(path: Path) -> dict` (lines 13-17): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 return points; 1 raise statements.
    Runtime interactions: loads or writes YAML.
    Notable calls: yaml.safe_load, isinstance, ValueError, path.read_text.
  - `closed_xy(vertices: list[tuple[float, float]]) -> tuple[list[float], list[float]]` (lines 20-24): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
  - `annotate_vertices(ax, vertices, *, color: str, prefix: str) -> None` (lines 27-37): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops.
    Notable calls: ax.scatter, ax.annotate.
  - `polygon_centroid(vertices: list[tuple[float, float]]) -> tuple[float, float]` (lines 40-59): No docstring; behavior is described from its body and call sites.
    Code map: 1 conditional branches; 1 for loops; 2 return points.
    Notable calls: math.isclose.
  - `max_radius_from_center(center_xy: tuple[float, float], vertices: list[tuple[float, float]]) -> float` (lines 62-83): No docstring; behavior is described from its body and call sites.
    Code map: 2 for loops; 1 return points.
    Notable calls: math.hypot.
  - `main() -> None` (lines 86-227): No docstring; behavior is described from its body and call sites.
    Code map: straight-line helper logic.
    Runtime interactions: loads or writes YAML, defines a CLI.
    Notable calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, Path.expanduser.resolve, output_path.parent.mkdir, load_config, polygon_centroid, plt.subplots, closed_xy, ax.fill, plt.Circle, ax.add_patch, annotate_vertices, ax.scatter, ax.annotate, ax.axhline, ax.axvline, ax.set_aspect....
- Whole-file runtime themes: loads or writes YAML, defines a CLI.

### `scripts/realsense/99-realsense-libusb.rules`

- File type: udev rules file.
- Tracked size: 11171 bytes; 96 decoded lines.
- Purpose: ##Version=1.1## # Device rules for Intel RealSense devices (R200, F200, SR300 LR200, ZR300, D400, L500, T200) SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0a80", MODE:="0666", GROUP:="plugdev", RUN+="/usr/local/bin/usb-R200-in_udev" SUBSYS...
- Main responsibilities: Provides a tracked asset, configuration, report artifact, or dependency input.
- Important dependencies or consumers: Repo maintainers and adjacent tooling.
- When to touch it: Touch only when the underlying tracked asset or dependency changes.
- udev rules: 87 active rule lines.
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0a80", MODE:="0666", GROUP:="plugdev", RUN+="/usr/local/bin/usb-R200-in_udev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0a66", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa3", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa2", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa5", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0abf", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0acb", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad0", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="04b4", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad6", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af2", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af6", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afe", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aff", MODE:="0666", GROUP:="plugdev"`
  - `SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b00", MODE:="0666", GROUP:="plugdev"`

### `scripts/restart_fcu.sh`

- File type: Shell script.
- Tracked size: 5941 bytes; 210 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail; set +u; set -u`.
- Defined shell functions: `usage, log, fail, cleanup, have_command, require_file, source_if_exists, service_exists, wait_for_service, wait_for_fcu_connection, start_mavros_if_needed`.
- Command map: `if(15), fi(15), local(7), fail(7), log(5), return(5), printf(4), set(3), exit(3), while(3), done(3), shift(3), sleep(2), have_command(2), source_if_exists(2), cat(1), --dry-run(1), --no-autostart(1), --wait-seconds(1), MAVROS_NAMESPACE(1)`.

### `scripts/restore_hover_baseline_params.sh`

- File type: Shell script.
- Tracked size: 318 bytes; 9 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `set(1), exec(1)`.

### `scripts/run_isaac_realsense_dev.sh`

- File type: Shell script.
- Tracked size: 844 bytes; 25 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(3), if(2), exit(2), fi(2), set(1), export(1), cd(1), ./scripts/run_dev.sh(1)`.

### `scripts/run_minjerk_of_compare.sh`

- File type: Shell script.
- Tracked size: 3362 bytes; 136 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail; set +u; set -u`.
- Defined shell functions: `usage, fail, warn, source_ros_env, ensure_ros_log_dir`.
- Command map: `if(8), fi(8), printf(5), shift(5), mkdir(4), set(3), exit(2), local(2), source(2), return(2), cat(1), scripts/run_minjerk_of_compare.sh(1), Launch(1), --waypoints-config(1), --output-dir(1), --run-id(1), --log-level(1), Anything(1), EOF(1), command(1)`.

### `scripts/set_param.py`

- File type: Python source file.
- Tracked size: 1300 bytes; 41 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: Provides Python module behavior for the package.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Send PARAM_SET directly to PX4 via mavlink_router UDP.
- Imports by role:
  - standard library: `import sys; import time; from pymavlink import mavutil`
- Top-level constants/state: `param_name, param_val, m, msg`.

### `scripts/set_param_raw.py`

- File type: Python source file.
- Tracked size: 1835 bytes; 50 decoded lines.
- Purpose: Standalone helper script for setup, calibration, reporting, diagnostics, or operator automation.
- Main responsibilities: defines functions crc_x25, make_param_set.
- Important dependencies or consumers: Python runtime, ROS 2 console entry points, pytest, or helper shell scripts depending on its package role.
- When to touch it: Touch this when changing node logic, helper algorithms, command-line behavior, or package APIs.
- Module summary: Minimal PARAM_SET via raw UDP MAVLink (no pymavlink needed).
- Imports by role:
  - standard library: `import socket; import struct; import sys; import time`
- Top-level constants/state: `SYSID, COMPID, TGT_SYS, TGT_COMP, sock, param, val, pkt`.
- Top-level functions:
  - `crc_x25(buf)` (lines 13-19): No docstring; behavior is described from its body and call sites.
    Code map: 1 for loops; 1 return points.
  - `make_param_set(param_id: str, value: float)` (lines 24-37): No docstring; behavior is described from its body and call sites.
    Code map: 1 return points.
    Notable calls: param_id.encode.ljust, struct.pack, bytes, crc_x25, param_id.encode.

### `scripts/setup_isaac_ros_release32.sh`

- File type: Shell script.
- Tracked size: 1058 bytes; 35 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `echo(10), git(4), set(1), mkdir(1), if(1), else(1), fi(1), cat(1), EOF(1)`.

### `scripts/start_isaac_realsense_container.sh`

- File type: Shell script.
- Tracked size: 2200 bytes; 62 decoded lines.
- Purpose: Shell automation used by operators or setup flows.
- Main responsibilities: Automates repeatable shell actions and setup/debug sequences.
- Important dependencies or consumers: Human operators, setup automation, or runbook commands.
- When to touch it: Touch this when setup commands, host paths, environment variables, or operator workflow changes.
- Interpreter: `#!/usr/bin/env bash`.
- Shell safety flags: `set -euo pipefail`.
- Command map: `-v(11), -e(8), echo(7), if(3), exit(3), fi(3), set(1), docker(1), --privileged(1), --network(1), --name(1), --runtime(1), --entrypoint(1), --workdir(1), /bin/bash(1)`.

## Maintenance Suggestions

- Link this guide from `README.md` once the team confirms the format is useful.
- Split the file later into generated package guides if Markdown rendering becomes slow.
- Add generated API documentation for Python modules with pydoc or Sphinx so function signatures stay automatically current.
- Keep diagrams near architecture sections and regenerate them when launch topology or topic flow changes.
- Treat tracked runtime logs and report assets as historical evidence; prefer adding new dated reports instead of editing old evidence.
- Consider adding a small documentation-generation script if this guide will be refreshed often.

