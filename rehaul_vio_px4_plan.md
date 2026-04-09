# Rehaul Plan: D455 VIO to PX4 Position Control

## Objective

Rework this repo from a mixed autonomy/teleop experiment into a bench-first, no-GPS indoor flight stack that can:

- use the connected Intel RealSense D455 as the primary aiding source
- provide PX4 1.16.0 with trusted external vision indoors
- reach stable `POSCTL` / position-hold behavior first
- then re-enable `OFFBOARD` position or velocity control on top of the same estimator

This plan is specific to the current platform:

- Jetson Orin Nano
- ARK PAB carrier + ARKV6X
- PX4 1.16.0
- MAVROS over `mavlink-router` UDP
- ROS 2 Humble
- D455 already connected
- near-term testing is props-off because the current carrier board is still ESC/motorless

## What The Current Repo Proves

- MAVROS connection and mode switching are working.
- Manual indoor teleop in `ALTCTL` / `STABILIZED` is already usable.
- Bench `OFFBOARD` can be entered when a fake external-vision pose is published.
- The missing piece is not "GPS" in general; it is a real aiding source that PX4 will trust indoors.

That means the rehaul should center the repo around external vision, not around more mode-switching or more `cmd_vel` plumbing.

## Core Decisions

### 1. Bootstrap through `vision_pose`, but promote to MAVROS odometry before flightworthy autonomy

Use MAVROS `vision_pose` first.

Why:

- it already exists in this repo and is enabled in `ros2/src/drone_bringup/config/px4_config.yaml`
- the bench path already proved that PX4 accepts the basic external-vision route through `/mavros/vision_pose/pose`
- the current `vio_bridge_node.py` is already shaped like a `PoseStamped` relay
- it is the shortest path from "no estimator" to "PX4 trusts indoor aiding"

What to do later:

- once VIO is stable, upgrade to MAVROS odometry for covariance + velocity richness
- when that happens, correct the topic direction in the repo docs and code: the publisher should target MAVROS odometry input, not the FCU output topic
- treat odometry as the real end state for flightworthy `POSCTL` validation and later `OFFBOARD`, not as an optional cleanup task

### 2. Fly `POSCTL` before `OFFBOARD`

The first flightworthy milestone should be "PX4 can hold position indoors from D455 VIO in `POSCTL`."

Why:

- it validates the estimator and EKF fusion before autonomy adds another failure layer
- it is safer than jumping straight back to closed-loop `OFFBOARD`
- it answers the actual user goal: fly in position control mode with RealSense

### 3. Use a real VIO/VSLAM engine outside this repo, but keep the PX4 bridge inside this repo

Do not turn this repo into a vendor dump of Isaac ROS, librealsense, and large external workspaces.

Recommended split:

- this repo owns bringup, bridge nodes, launch files, MAVROS config, testing scripts, and PX4-specific docs
- the actual VIO engine lives in a separate workspace or container, launched by thin wrappers from this repo

That keeps the repo maintainable during the rehaul.

## Compatibility Guardrails

These constraints should be treated as part of the plan, not as setup trivia:

- If the estimator is based on Isaac ROS while this repo stays on ROS 2 Humble, use the pinned `release-3.2` documentation and containers rather than the newest Isaac ROS getting-started flow.
- Verify JetPack and D455 firmware before deep integration work so the camera/runtime stack is chosen deliberately instead of by accident.
- Keep RealSense versions aligned with the chosen Isaac ROS release when using the Isaac container path. As of the `release-3.2` guidance summarized in this repo, that means firmware `5.13.0.50`, `librealsense` `2.55.1`, and `realsense-ros` `4.51.1-isaac`.
- Do not mix a newer host-native RealSense stack into the estimator path unless that combination has been bench-validated on this Jetson. Use the host only for basic camera sanity checks when needed.
- Write frame and timestamp validation into the implementation from day one. PX4/MAVROS integration failures will otherwise look like "VIO is unsupported" when the real bug is coordinate or latency mismatch.

## Recommended End-State Architecture

### Data flow

1. `realsense2_camera` publishes D455 stereo IR + IMU.
2. A dedicated VIO/VSLAM stack estimates pose in a world frame.
3. A new repo-owned bridge node converts frames, timestamps, and covariance into PX4-friendly ROS messages.
4. MAVROS forwards external vision into PX4.
5. PX4 EKF fuses external vision and becomes willing to enter indoor `POSCTL` and later `OFFBOARD`.
6. A small command layer publishes position/velocity setpoints only after the estimator is healthy.

### Practical architecture for this repo

- `drone_bringup`
  - owns MAVROS launch/config
  - owns D455 launch
  - owns combined bench and indoor-flight launch files
- new `drone_vio_pkg`
  - owns estimator interface contracts
  - owns frame conversion
  - owns pose/odometry relay to MAVROS
  - owns estimator health publication
- slimmed `drone_control_pkg`
  - keep manual teleop
  - keep minimal setpoint publishers
  - remove unrelated behavior logic from the critical indoor-flight path

### Frame contract to standardize early

Define this before coding:

- world frame: `map`
- vehicle body frame in ROS: `base_link`
- camera optical and IMU frames from RealSense remain sensor-native
- one bridge node is responsible for camera-to-body extrinsics and ENU/NED consistency

Do not let every node "kind of" handle frame conversion differently.

## What To Keep, Replace, Archive, Or Delete

### Keep

- `ros2/src/drone_bringup/launch/drone.launch.py`
- `ros2/src/drone_bringup/launch/realsense_d455.launch.py`
- `ros2/src/drone_bringup/config/px4_config.yaml`
- `ros2/src/drone_control_pkg/drone_control_pkg/keyboard_teleop_node.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/mavros_velocity_node.py` as a temporary command path
- the `mavlink-router` UDP topology

### Replace

- `ros2/src/drone_control_pkg/drone_control_pkg/vio_bridge_node.py`
  - replace the current simple `PoseStamped` relay with a real frame-aware bridge
- `archive/legacy_stack/ros2/src/drone_bringup/launch/autonomy_stack.launch.py`
  - replace the current perception/behavior stack with a VIO-first indoor-flight launch
- `archive/legacy_stack/ros2/src/drone_bringup/config/autonomy_params.yaml`
  - replace with a smaller VIO/PX4 parameter file

### Archive

Archive these into a `legacy/` or `archive/` area if the rehaul is truly focused on flying indoors with VIO:

- `drone_behavior_pkg`
- `drone_light_pkg`
- `drone_monitor_pkg`
- the current target-tracking-centered parts of `drone_vision_pkg`
- root docs that are tied to the old autonomy experiments rather than the new VIO-first stack

Rationale:

- they are not needed to solve the present blocker
- they make the repo feel larger than the actual indoor-flight problem
- they can come back later once the base flight stack is trustworthy

### Delete after archive is captured

- bench-only dummy vision as an active default path
- stale docs that still frame the project as "velocity OFFBOARD first"
- config names that still imply ArduPilot if they are no longer useful to the team

## New Files And Packages Worth Adding

### New package

Create `ros2/src/drone_vio_pkg` with:

- `vio_interface_node`
  - subscribes to estimator output
  - validates freshness
  - applies frame transforms
  - republishes to MAVROS external-vision topics
- `vio_health_node`
  - publishes estimator health, tracking quality, and timeout state
- `extrinsics.yaml`
  - D455 to vehicle-body transform
- `vio_params.yaml`
  - topic names, rates, frame ids, covariance defaults, timeout thresholds

### New launch files

- `ros2/src/drone_bringup/launch/vio_bringup.launch.py`
  - D455 + VIO engine + bridge only
- `ros2/src/drone_bringup/launch/indoor_position_bench.launch.py`
  - MAVROS + VIO bridge + diagnostics for props-off testing
- `ros2/src/drone_bringup/launch/indoor_position_flight.launch.py`
  - same pipeline with flight-ready settings

### New docs/scripts

- `docs/rehaul/px4_ev_params.md`
- `docs/rehaul/test_matrix.md`
- `scripts/check_vio_topics.sh`
- `scripts/record_vio_bag.sh`
- `scripts/compare_vio_to_mavros_local_position.sh`

## Exact Repo Touch Points

These are the files the rehaul will almost certainly touch:

- `problem.md`
- `README.md`
- `archive/legacy_stack/docs/vio_todo.md`
- `ros2/src/drone_bringup/launch/realsense_d455.launch.py`
- `ros2/src/drone_bringup/launch/bench_offboard.launch.py`
- `archive/legacy_stack/ros2/src/drone_bringup/launch/autonomy_stack.launch.py`
- `ros2/src/drone_bringup/config/px4_pluginlists.yaml`
- `ros2/src/drone_bringup/config/px4_config.yaml`
- `archive/legacy_stack/ros2/src/drone_bringup/config/autonomy_params.yaml`
- `ros2/src/drone_control_pkg/drone_control_pkg/vio_bridge_node.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/bench_vision_pose_node.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/mavros_velocity_node.py`

## Phased Rehaul Plan

## Phase 0: Freeze the old stack and narrow scope

Goal:

- stop treating this repo as "everything for the drone"
- make VIO-to-PX4 the primary product

Actions:

- create a memory/archive document before deleting anything
- move nonessential autonomy packages and docs into `archive/legacy_stack/`
- rename or remove launch files that imply the current target-tracking autonomy stack is still the main path
- define the new top-level README around: `ALTCTL`, D455 VIO, `POSCTL`, then `OFFBOARD`

Acceptance criteria:

- a new contributor can tell in under five minutes that the repo's main purpose is indoor VIO flight
- the old target-tracking stack is preserved but no longer dominates the repo structure

## Phase 1: Make D455 bringup deterministic

Goal:

- turn the connected D455 into a stable sensor source for VIO

Actions:

- rework `realsense_d455.launch.py` so VIO-friendly defaults are explicit
- prefer stereo IR + IMU as the primary path
- keep color/depth optional for debugging, not required for estimator operation
- pin topic names, frame ids, IMU unification, and expected rates in one config file
- add a quick sensor validation script that checks:
  - IR topics present
  - IMU topics present
  - topic rates sane
  - timestamps moving forward

Acceptance criteria:

- a single launch reliably produces D455 IR + IMU topics every time after reboot
- bags recorded on the bench are usable for offline estimator testing

## Phase 2: Stand up a real VIO estimator offboard from the current repo logic

Goal:

- replace the dummy bench pose with a real estimator without overstuffing this repo

Recommended path:

- follow the Hackster/VSLAM-UAV idea at a high level only:
  - D455 stereo IR + IMU
  - Isaac ROS Visual SLAM or comparable Jetson-suitable VIO
  - MAVROS bridge back into PX4

Repo-specific adjustment:

- do not copy the entire Hackster repo into this repo
- launch the estimator from a separate workspace or container
- expose a stable ROS contract into this repo:
  - `/vio/pose`
  - `/vio/odom`
  - `/vio/status`

Acceptance criteria:

- with the aircraft stationary, the estimator holds pose without repeated resets
- when the aircraft is hand-moved on the bench, the estimator tracks motion smoothly
- estimator output remains live for at least several minutes without dropping to standby

## Phase 3: Replace the dummy bridge with a real PX4 external-vision bridge

Goal:

- make PX4 trust the real estimator

Actions:

- replace `vio_bridge_node.py` with a bridge that:
  - consumes estimator pose and optionally odometry
  - applies the D455-to-body extrinsic transform
  - enforces frame naming and orientation conventions
  - republishes to `/mavros/vision_pose/pose` first
  - publishes companion-process health
  - exposes timeout and stale-data diagnostics
- keep `bench_vision_pose_node.py` only as a legacy fallback under `archive/` or a clearly unsafe launch

Acceptance criteria:

- with real VIO active, `/mavros/vision_pose/pose` updates continuously
- PX4 local position changes when the aircraft is moved by hand
- `bench_offboard.launch.py` is no longer needed to fake estimator health

## Phase 4: Tune PX4 for indoor external-vision fusion

Goal:

- move PX4 from "MAVROS sees pose" to "PX4 actually trusts pose"

Parameter areas to revisit in QGC:

- GPS fusion control for indoor use
- external-vision fusion enablement
- external-vision delay and measurement alignment
- external-vision position offset / lever arm relative to IMU/body
- height reference choice for indoor testing
- arming checks related to no-GPS operation
- RC input handling during companion-led tests
- offboard-loss and failsafe behavior

Important note:

- do not mix "temporary arming hacks" with real estimator bringup unless they are documented and reversible
- freeze a known-good indoor parameter set once PX4 starts accepting the estimator

Acceptance criteria:

- with D455 VIO running, PX4 enters indoor position-capable modes without requiring a fake pose
- local position in MAVROS remains consistent while the aircraft is carried around the room

## Phase 5: Prove `POSCTL` before autonomy

Goal:

- establish that the estimator is good enough for human-supervised indoor position control

Actions:

- first keep keyboard teleop for arming and mode control
- add a test checklist for:
  - estimator healthy
  - MAVROS receiving external vision
  - PX4 local position stable
  - mode transitions clean
- when motors/ESCs are present, test `POSCTL` before any `OFFBOARD` control

Acceptance criteria:

- operator can arm and enter indoor position control with D455 active
- hover/hold behavior is stable enough that `POSCTL` is the normal indoor validation mode

## Phase 6: Promote to MAVROS odometry and restore `OFFBOARD`

Goal:

- upgrade from the fast path to a better long-term interface

Actions:

- remove the `odometry` denylist entry from `ros2/src/drone_bringup/config/px4_pluginlists.yaml`
- configure MAVROS odometry intentionally instead of leaving it half-disabled
- extend the bridge to publish odometry with covariance and twist
- only after that, adapt the control layer from "velocity OFFBOARD experiment" to "position-control-capable autonomy"

Acceptance criteria:

- odometry-based external vision is active and documented
- the control layer can issue position or velocity setpoints on top of a trusted estimator
- `OFFBOARD` depends on the same validated VIO path already proven in `POSCTL`
- the repo no longer treats `vision_pose` as the primary long-term interface for real flight

## Test Progression

Use this order and do not skip steps:

1. D455 sensor-only bringup on the Jetson
2. VIO estimation with no PX4 dependency
3. VIO bridge into MAVROS while props-off
4. PX4 local-position tracking while hand-moving the airframe
5. Indoor `POSCTL` with motors/ESCs installed and prop-safe procedure
6. Indoor `OFFBOARD` with conservative limits
7. Only then reintroduce any larger autonomy stack

## Risks To Design Around

- D455 is not the exact camera used in the Hackster write-up, so treat camera config and IMU tuning as real work, not a copy-paste
- the current repo still frames autonomy around target tracking, which can distract from the base-flight problem
- MAVROS topic naming around odometry is easy to misunderstand; document the bridge interface explicitly during the rehaul
- frame mistakes will look like "PX4 does not trust VIO" even when the estimator itself is good
- stale or jittery timestamps can block progress just as effectively as bad poses

## Recommended First Week Of Work

1. Archive the old autonomy-first pieces and rewrite the README around VIO-first indoor flight.
2. Harden `realsense_d455.launch.py` for stereo IR + IMU defaults.
3. Create `drone_vio_pkg` and replace the current `vio_bridge_node` design with a real interface contract.
4. Bring up a standalone D455 VIO pipeline and standardize its output topics.
5. Bridge real pose into MAVROS `vision_pose`.
6. Tune PX4 until indoor `POSCTL` becomes possible without the dummy node.

## Bottom Line

The fastest viable rehaul is:

- keep MAVROS and the current UDP transport
- keep manual `ALTCTL` teleop as the safety fallback
- replace the fake vision publisher with a real D455 VIO pipeline
- feed PX4 through `vision_pose` first as a bootstrap step
- validate indoor `POSCTL`
- then upgrade to odometry-based external vision as the flight-ready interface and restore `OFFBOARD`

That path directly attacks the blocker in `problem.md` without dragging the whole legacy autonomy stack forward into the new design.
