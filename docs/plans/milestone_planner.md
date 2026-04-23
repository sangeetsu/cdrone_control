# Milestone Planner

## Active Direction

The repo focus is shifting to:

1. keep the current OptiTrack-backed hover / external-pose path healthy
2. clean up the repo root and stale launch/config drift
3. restore a minimal active target-follow path in the active workspace
4. keep VIO / RealSense-ownship localization on the backburner for later

VIO is now a later replacement for the current ownship pose source, not the immediate blocker for target-follow experiments.

## Scope For This Pass

This pass should leave the repo with:

- a clean repo root with older markdown files moved under `docs/archived/`
- the current MAVROS launch path using the `px4_*` config files consistently
- the hover demo still launchable after the config cleanup
- a second demo that can take off, switch to OFFBOARD, fly to a specified local `map`-frame coordinate, hold, and land
- a minimal active target-follow controller in the active workspace
- a dedicated safety gating config file that can be tuned during flight testing
- a launch flow that wires current ownship pose + target-follow controller + velocity bridge together

## Do Not Forget

- Do not drag the whole archived autonomy stack back into `ros2/src/`
- Reuse only the smallest useful pieces from archived follow logic
- Keep the control contract simple and explicit
- Verify the hover demo after touching MAVROS launch/config
- Treat "world frame" as the PX4 local `map` frame used by `/cdrone/cdrone4/mavros/local_position/pose`, not GPS latitude/longitude
- Prefer body-frame follow first; postpone map-frame target pursuit until needed

## Next Demo Milestone

The next demo after hover should be:

- take off using the same external-pose / PX4 setup as the hover demo
- switch to OFFBOARD with position setpoint warmup
- fly to a specified coordinate in the local world frame
- hold briefly at that coordinate
- land and restore the takeoff parameter

For this repo, "world frame" means the local `map` frame already established by:

- `external_pose_adapter_node`
- `external_pose_bridge_node`
- MAVROS / PX4 local position

It does not mean a geodetic GPS waypoint.

## Current Control / Pose Picture

### Ownship pose today

The current hover demo uses:

1. `vrpn_mocap` or another `PoseStamped` source
2. `external_pose_adapter_node`
3. `external_pose_bridge_node`
4. MAVROS `vision_pose/pose`
5. PX4 fused local pose at `/cdrone/cdrone4/mavros/local_position/pose`

The relevant active files are:

- `ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/external_pose_adapter_node.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/external_pose_bridge_node.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/position_hover_demo_sequence_node.py`

### Target coordinates today

The archived YOLO / Norfair path already outputs `drone_msgs/TargetTrack` with:

- `x_b_m`
- `y_b_m`
- `z_b_m`
- `vx_b_mps`
- `vy_b_mps`
- `vz_b_mps`

Those are body-relative target coordinates, not map-frame goals.

That means the shortest safe active follow path is:

`TargetTrackArray` -> body-frame follow controller -> `/cdrone/<id>/control/cmd_vel_body` -> `mavros_velocity_node` -> MAVROS velocity setpoint -> PX4 OFFBOARD

This avoids building a map-frame target transformer before it is actually needed.

## Transform Model We Need

### Transform 1: Motive / VRPN world -> repo external pose input

Handled by `external_pose_adapter_node`:

- optional position offset
- optional roll/pitch/yaw offset
- output frame normalized to `map`

This is where rigid-body orientation fixes and sensor-to-body offsets belong for the current hover path.

### Transform 2: external pose input -> PX4 trusted local pose

Handled by:

- `external_pose_bridge_node`
- MAVROS `vision_pose/pose`
- PX4 external-vision fusion

The hover demo then reads `/cdrone/cdrone4/mavros/local_position/pose`, plus home/global-origin topics, and treats that as the trusted local frame.

### Transform 3: target body coordinates -> body-frame control commands

For the minimal active follow controller, we do not need a map transform.

We only need a consistent body-frame contract:

- target forward error from `x_b_m`
- target lateral error from `y_b_m`
- target vertical error from `z_b_m`
- target yaw error from `atan2(y_b_m, x_b_m)`

The controller can publish body-frame velocity commands directly.

### Transform 4: future body target -> map-frame target

This is not needed for the first active follow pass or for a static world-frame goto demo, but the formula is:

`p_target_map = p_drone_map + R_map_body * p_target_body`

Inputs for that later step would be:

- `p_drone_map` from `/cdrone/cdrone4/mavros/local_position/pose`
- `R_map_body` from the ownship orientation in that same pose
- `p_target_body` from `TargetTrack`

That later feature becomes useful only if we want:

- explicit moving world-coordinate pursuit
- map-frame safety bubbles
- path planning instead of direct body-frame following

## Minimal Active Follow Design

### New active node

Add a new node in `drone_control_pkg` that:

- subscribes to `TargetTrackArray`
- subscribes to `/cdrone/cdrone4/mavros/state`
- subscribes to `/cdrone/cdrone4/mavros/local_position/pose`
- subscribes to `/cdrone/cdrone4/mavros/companion_process/status`
- selects one valid target
- applies safety gates before commanding motion
- publishes `TwistStamped` on `/cdrone/<id>/control/cmd_vel_body`
- publishes a simple status string for debugging

### Initial target-selection rule

Keep it minimal:

- prefer the current active track if it is still fresh and valid
- otherwise choose the best fresh valid track by score

Use archived logic as reference, not as a wholesale import.

### Initial control rule

Use proportional body-frame follow:

- forward control toward `follow_distance_m`
- lateral control toward `0`
- vertical control toward `0`
- yaw-rate control toward target bearing

Keep deadbands and hard clamps in the first version.

## Safety Gates To Add

Create a dedicated safety config file for later flight tuning.

The first version should gate on:

- `follow_enabled_on_startup`
- `require_mavros_connected`
- `require_armed`
- `require_offboard`
- `require_companion_active`
- `state_timeout_s`
- `local_pose_timeout_s`
- `companion_status_timeout_s`
- `track_timeout_s`
- `min_track_confidence`
- `max_target_distance_m`
- `require_target_in_front`
- `max_abs_target_y_m`
- `max_abs_target_z_m`
- `min_safe_distance_m`
- `publish_zero_on_block`
- `publish_zero_on_lost_target`

The first version should fail safe by publishing zero velocity, not by trying to guess a recovery trajectory.

## Launch / Config Work

### Cleanup 1: MAVROS config drift

Fix `ros2/src/drone_bringup/launch/drone.launch.py` so the active source tree uses:

- `px4_params.yaml`
- `px4_config.yaml`
- `px4_pluginlists.yaml`

### Cleanup 2: new target-follow launch

Add an active launch file that composes:

- `drone.launch.py`
- `external_pose_px4_bridge.launch.py` with OptiTrack defaults
- `mavros_velocity_node`
- new target-follow controller node

This launch should be perception-agnostic so a YOLO / Norfair publisher can be attached later without rewriting the flight-control side.

### Cleanup 3: root markdown archive

Move older root markdown files into `docs/archived/` and update surviving references.

Likely keep in the root:

- `README.md`
- `docs/notes/offboard_indoor_blocker.md`
- `docs/plans/milestone_planner.md`

## Files Likely To Change

### New files

- `docs/plans/milestone_planner.md`
- `docs/archived/README.md`
- `ros2/src/drone_control_pkg/drone_control_pkg/target_follow_controller_node.py`
- `ros2/src/drone_bringup/config/target_follow_safety.yaml`
- `ros2/src/drone_bringup/launch/target_follow.launch.py`

### Existing files

- `README.md`
- `ros2/src/drone_bringup/launch/drone.launch.py`
- `ros2/src/drone_control_pkg/setup.py`
- `ros2/src/drone_control_pkg/package.xml`
- older root markdown files moved into `docs/archived/`

## Verification Checklist

### Docs and repo hygiene

- root markdown clutter reduced
- surviving root docs point to the new archive paths where needed

### Hover demo integrity

- `position_hover_demo.launch.py` still resolves `drone.launch.py`
- MAVROS launch now reads `px4_*` configs cleanly

### Goto demo integrity

- `position_goto_demo.launch.py` resolves cleanly
- the goto demo uses the same external-pose dependency chain as the hover demo
- the goto demo publishes local position setpoints in the `map` frame

### Build integrity

- package imports succeed
- new node is registered in `setup.py`
- `colcon build` succeeds for touched packages

### Target-follow control integrity

- new controller starts without tracks
- blocked state publishes zero velocity
- valid tracks produce bounded body-frame velocity commands
- stale pose / stale companion / stale track each force zero command

## Out Of Scope For This Pass

- replacing OptiTrack with D455 VIO for ownship pose
- reactivating the full archived vision/autonomy stack
- building a map-frame path planner
- solving all perception-side YOLO / Norfair integration details
- path planning around obstacles

## Success Definition

This pass is successful if we end with:

- a cleaner repo structure
- a consistent active MAVROS launch/config path
- the hover demo still intact
- an active minimal target-follow controller that can safely consume `TargetTrackArray`
- a single config file for tuning follow safety gates during future flight tests
