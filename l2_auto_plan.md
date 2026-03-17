# Level 2 Autonomy Expansion and Weekly Roadmap

## Summary
- Week of March 9, 2026: Level 1 is complete. Manual `ALTCTL` teleop is proven.
- Week of March 16, 2026: Level 2 should mean a guarded single-target autonomous follow/approach flight with operator supervision, real ego-pose aiding into PX4 through `/mavros/vision_pose/pose`, and instant manual takeover.
- Week of March 23, 2026: Level 4 is the target only if Level 2 exits cleanly by Friday, March 20, 2026. Otherwise that week becomes Level 3 hardening.
- Full autonomy later means decentralized swarm selection, friendly/unknown separation, obstacle-aware pursuit, and multi-drone cooperation.

## Problems To Solve Before Level 2
- PX4 still has no flightworthy indoor aiding source in this repo. The current `bench_vision_pose_node` is bench-only.
- Neither repo currently contains true ego-pose VIO. Recent `cdrone_yolo` commits on March 13-16, 2026 add calibration and depth-map work, not VIO.
- `cdrone_yolo` target tracking and `cdrone_control` autonomy are coupled only by file paths. There is no formal runtime contract, health gate, or deployment config.
- Camera truth is inconsistent across notes and scripts: sensor IDs differ, and the nominal 60 mm baseline conflicts with calibration notes showing about 64.4 mm. The calibration artifact and measured mount transform must become authoritative.
- The autonomy stack has no obstacle model, no friendly/unknown discrimination, no peer identity, and no multi-drone namespacing or host/IP config.
- Safety gating is not flight-ready yet. It does not block on VIO freshness, FCU arming/mode health, camera status, or explicit autonomy enable.

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
  - Lost target handling: zero forward velocity immediately, hover through a short reacquire window, then fail back to hold/manual.
  - Level 2 obstacle handling: stop-on-occupancy only from stereo forward-corridor depth. No autonomous rerouting yet.
- Strengthen safety and operator authority.
  - Extend health monitoring to include VIO freshness, tracker freshness, MAVROS state, estop, and manual override readiness.
  - Add an explicit autonomy-enable latch so the controller cannot take over just because topics are alive.
  - Keep `ALTCTL` manual takeover as the primary recovery path.
- Make the repo multi-drone ready without full swarm behavior this week.
  - Namespace all nodes and topics by `drone_id`.
  - Add deployment config keys: `drone_id`, `hostname`, `self_ip`, `peer_list`, `camera_left_sensor_id`, `camera_right_sensor_id`, `calibration_path`, `model_path`.
  - Add a peer-state interface for later swarm work: peer id, timestamp, pose, velocity, health, host/ip metadata.
  - Define friendly vs unknown now: friendly = visual track matched to peer-state; unknown = valid track with no peer match. This filter becomes active in Level 4.

## Public Interfaces
- Keep the current perception/control topics, but change the default autonomy scenario from illumination to follow.
- Add a `vio_bridge` contract: selected stereo VIO input -> `/mavros/vision_pose/pose` output.
- Add per-drone launch/config inputs for identity, network, camera mapping, calibration, and model assets.
- Add a swarm peer-state topic contract for later friendly filtering and decentralized coordination.

## Weekly Roadmap
- Week of March 16, 2026 -> Level 2
  - Solve PX4 flight prerequisites: arming/health diagnosis, real VIO bridge, stable `OFFBOARD`, manual handoff.
  - Port stereo target tracking into ROS 2 with calibration-backed triangulation and body-frame transform.
  - Replace `intercept_illuminate_v1` with `track_follow_v1` and perform one guarded low-altitude target-follow flight in clear airspace with no friendlies nearby.
  - Exit criteria: PX4 accepts VIO for at least 2 minutes, the drone holds standoff on one target, manual takeover works immediately, and watchdog/estop are validated live.
- Week of March 23, 2026 -> Level 4 target
  - Start only if Level 2 exits by March 20, 2026. Otherwise finish Level 3 robustness first.
  - Add per-drone namespace/deployment config, peer-state heartbeat, friendly-track filtering, and nearest-unknown target selection.
  - Add dynamic keepout around matched friendlies and stereo depth stop zones; if clearance is blocked, hold instead of closing.
  - Exit criteria: two drones launch from config only, with unique identities and no topic/IP collisions, and one drone ignores friendlies while selecting the nearest unknown track.
- Level 5 after that
  - Decentralized swarm assignment, cooperative track handoff, obstacle-aware local planning, and fully autonomous pursuit without operator target selection.

## Test Plan
- Bench
  - Verify stereo VIO publishes stable ego pose at the required rate and PX4 accepts it for `OFFBOARD`.
  - Verify target tracks publish stable 3D body-frame positions and persistent IDs with the current calibration file.
  - Verify autonomy enable, estop, watchdog, lost-target hold, and manual takeover with props off.
- Flight
  - Run low-altitude single-target follow in a clear test box with one operator and one spotter.
  - Intentionally trigger stale VIO, target loss, and estop once each; confirm immediate hold/abort behavior.
  - Do not flight-test friendly avoidance or autonomous obstacle bypass until peer-state and occupancy-stop logic are live.
- Deployment
  - Start two drone instances from config only, with different `drone_id`, hostname, and IP values, and confirm the same codebase runs on both.

## Assumptions and Defaults
- Level 2 uses an external stereo VIO/SLAM source for ego pose; `cdrone_yolo` does not currently provide VIO.
- `/mavros/vision_pose/pose` is the first PX4 aiding interface; `/mavros/odometry/out` is deferred.
- The stereo calibration artifact, not the nominal 60 mm measurement, is the baseline/extrinsics source of truth.
- Level 2 obstacle handling is hold/stop only. Autonomous bypass is deferred.
- This week's swarm work is interface-ready, not coordinated-behavior-ready.
