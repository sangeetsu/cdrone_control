# Level 2 Autonomy, Monitoring Dashboard, and Weekly Roadmap

## Summary
- Week of March 9, 2026: Level 1 is complete. Manual `ALTCTL` teleop is proven.
- Week of March 16, 2026: Level 2 should mean a guarded single-target autonomous follow/approach flight with real ego-pose aiding into PX4 through `/mavros/vision_pose/pose`, a read-only remote terminal dashboard, and instant manual takeover.
- Week of March 23, 2026: Level 4 is the target only if Level 2 exits cleanly by Friday, March 20, 2026. Otherwise that week becomes Level 3 hardening.
- The dashboard itself should not be the publisher. The better split is a telemetry aggregator node that publishes normalized status, while the dashboard only subscribes.

## Problems To Solve Before Level 2
- PX4 still has no flightworthy indoor aiding source in this repo. The current `bench_vision_pose_node` is bench-only.
- Neither repo currently contains true ego-pose VIO. Recent `cdrone_yolo` commits on March 13-16, 2026 add calibration and depth-map work, not VIO.
- `cdrone_yolo` target tracking and `cdrone_control` autonomy are coupled only by file paths. There is no formal runtime contract, health gate, or deployment config.
- Camera truth is inconsistent across notes and scripts: sensor IDs differ, and the nominal 60 mm baseline conflicts with calibration notes showing about 64.4 mm. The calibration artifact and measured mount transform must become authoritative.
- The autonomy stack has no obstacle model, no friendly/unknown discrimination, no peer identity, and no multi-drone namespacing or host/IP config.
- There is no consolidated telemetry view or alert channel, so operator awareness is fragmented across raw topics.

## Key Changes
- Add a dedicated ego-localization lane for Level 2.
  - Integrate an existing stereo VIO/SLAM package as a separate dependency and bridge its pose into `/mavros/vision_pose/pose`.
  - Keep `vision_pose` as the Level 2 interface and defer `/mavros/odometry/out` to Level 3+.
  - Add a hard flight gate: no autonomous flight unless VIO is fresh, stable, frame-correct, and accepted by PX4.
- Treat `cdrone_yolo` as target-tracking only.
  - Reuse its calibration, TensorRT engine, and triangulation direction for ROS 2 target tracking.
  - Do not gate Level 2 on `depth_map.py`; that is a depth-debug path, not an ego-pose solution.
  - Make camera sensor IDs, calibration path, model path, and camera-to-body transform explicit parameters.
- Replace the current default autonomy behavior with follow logic.
  - New state machine: `SEARCH -> ALIGN -> APPROACH -> FOLLOW_STANDOFF -> LOST_TARGET_HOLD -> FAILSAFE_HOLD`.
  - Far target handling: require stable detection first, yaw-align before closing, and reduce forward speed as range uncertainty grows.
  - Near target handling: maintain standoff distance and never command positive closing velocity inside `min_safe_distance_m`.
  - Level 2 obstacle handling: stop-on-occupancy only from stereo forward-corridor depth. No autonomous rerouting yet.
- Strengthen safety and operator authority.
  - Extend health monitoring to include VIO freshness, tracker freshness, MAVROS state, estop, and manual override readiness.
  - Add an explicit autonomy-enable latch so the controller cannot take over just because topics are alive.
  - Keep `ALTCTL` manual takeover as the primary recovery path.
- Add a monitoring package for normalized telemetry and dashboarding.
  - Create `drone_monitor_pkg` with `telemetry_aggregator_node` and `dashboard_tui_node`.
  - `telemetry_aggregator_node` subscribes to MAVROS, VIO, behavior, and perception topics and publishes a normalized flight-status summary plus latched alerts.
  - `dashboard_tui_node` runs on the remote operator laptop and subscribes only. Implement it as a terminal UI using `curses`, not a web stack.
  - TUI layout:
    - Flight pane: connection, armed, mode, altitude, pose, velocity, yaw, battery.
    - Autonomy pane: autonomy state, active target ID, target range/bearing, commanded velocity, obstacle blocked, estop.
    - Health pane: VIO age, tracker age, camera FPS, inference latency, MAVROS freshness, watchdog state.
  - Alert rules:
    - stale VIO
    - stale tracks
    - `OFFBOARD` rejected
    - autonomy blocked
    - min-distance gate active
    - target lost
    - battery low
    - estop asserted
- Make the repo multi-drone ready without full swarm behavior this week.
  - Namespace all nodes and topics by `drone_id`.
  - Add deployment config keys: `drone_id`, `hostname`, `self_ip`, `peer_list`, `camera_left_sensor_id`, `camera_right_sensor_id`, `calibration_path`, `model_path`, `dashboard_rate_hz`.
  - Add a peer-state interface for later swarm work: peer id, timestamp, pose, velocity, health, host/ip metadata.
  - Define friendly vs unknown now: friendly = visual track matched to peer-state; unknown = valid track with no peer match. This filter becomes active in Level 4.

## Public Interfaces
- Keep the current perception/control topics, but change the default autonomy scenario from illumination to follow.
- Add a `vio_bridge` contract: selected stereo VIO input -> `/mavros/vision_pose/pose` output.
- Add monitor topics:
  - `/cdrone/monitor/flight_status`
  - `/cdrone/monitor/alerts`
- Add monitor message types:
  - `FlightStatus`: summarized pose, altitude, velocity, mode, battery, autonomy state, target state, health freshness.
  - `SystemAlert`: severity, code, message, timestamp, latched flag.
- Add per-drone launch/config inputs for identity, network, camera mapping, calibration, model assets, and dashboard refresh rate.

## Weekly Roadmap
- Week of March 16, 2026 -> Level 2
  - Solve PX4 flight prerequisites: arming/health diagnosis, real VIO bridge, stable `OFFBOARD`, manual handoff.
  - Port stereo target tracking into ROS 2 with calibration-backed triangulation and body-frame transform.
  - Add `telemetry_aggregator_node` and the remote terminal dashboard before autonomous flight.
  - Replace `intercept_illuminate_v1` with `track_follow_v1` and perform one guarded low-altitude target-follow flight in clear airspace with no friendlies nearby.
  - Exit criteria: PX4 accepts VIO for at least 2 minutes, the drone holds standoff on one target, the dashboard shows live pose/altitude/health/alerts, and manual takeover plus watchdog/estop are validated live.
- Week of March 23, 2026 -> Level 4 target
  - Start only if Level 2 exits by March 20, 2026. Otherwise finish Level 3 robustness first.
  - Add per-drone namespace/deployment config, peer-state heartbeat, friendly-track filtering, nearest-unknown selection, and multi-drone dashboard summary rows.
  - Add dynamic keepout around matched friendlies and stereo depth stop zones; if clearance is blocked, hold instead of closing.
  - Exit criteria: two drones launch from config only, with unique identities and no topic/IP collisions, and one drone ignores friendlies while selecting the nearest unknown track.
- Level 5 after that
  - Decentralized swarm assignment, cooperative track handoff, obstacle-aware local planning, and fully autonomous pursuit without operator target selection.

## Test Plan
- Bench
  - Verify stereo VIO publishes stable ego pose at the required rate and PX4 accepts it for `OFFBOARD`.
  - Verify target tracks publish stable 3D body-frame positions and persistent IDs with the current calibration file.
  - Verify `FlightStatus` freshness, stale-topic alerts, estop display, autonomy-blocked display, and TUI resilience to node restarts.
- Flight
  - Run low-altitude single-target follow in a clear test box with one operator and one spotter.
  - Intentionally trigger stale VIO, target loss, min-distance gate, and estop once each; confirm immediate hold/abort behavior and dashboard alerting.
  - Do not flight-test friendly avoidance or autonomous obstacle bypass until peer-state and occupancy-stop logic are live.
- Deployment
  - Start one drone plus remote TUI in Level 2.
  - Start two drone instances plus remote TUI in Level 4 and confirm no topic/IP collisions.

## Assumptions and Defaults
- Level 2 uses an external stereo VIO/SLAM source for ego pose; `cdrone_yolo` does not currently provide VIO.
- `/mavros/vision_pose/pose` is the first PX4 aiding interface; `/mavros/odometry/out` is deferred.
- The stereo calibration artifact, not the nominal 60 mm measurement, is the baseline/extrinsics source of truth.
- Level 2 obstacle handling is hold/stop only. Autonomous bypass is deferred.
- The dashboard is read-only in v1. Command paths, including estop, stay outside it.
