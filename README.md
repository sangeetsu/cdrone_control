# cdrone_control

External-pose-backed ROS 2 stack for indoor PX4 flight experiments on a Jetson Orin Nano, with the current emphasis on OptiTrack-backed hover validation and a minimal active target-follow path.

## Active Goal

The active goal is now:

1. keep the current external-pose pipeline healthy enough for hover and recovery testing
2. restore a minimal active `TargetTrackArray` -> follow-controller -> body-velocity control path
3. validate target-follow with the current OptiTrack-backed ownship pose source
4. keep D455 / VIO work on the backburner until the follow-control loop is stable

The root problem in `problem.md` still matters for `OFFBOARD`, but VIO is no longer the immediate workstream.

## Phase Status

The repo already has the base pieces needed for this next pass:

- the old autonomy-heavy workspace has been moved to `archive/legacy_stack/`
- the active ROS workspace already has the generic external-pose adapter / bridge used by the hover demo
- the hover demo can arm, lift, hover briefly, and land with the current external-pose chain
- a second demo can now take off, switch to OFFBOARD, fly to a specified local `map`-frame coordinate, hold, and land
- the MAVROS velocity bridge still exists and can be reused for body-frame follow control
- archived YOLO / Norfair follow logic still exists as reference material under `archive/legacy_stack/`

What is still not done:

- the perception-side YOLO / Norfair publisher is still archived and not yet restored to the active workspace
- the new target-follow controller still needs real flight tuning and validation
- moving map-frame target pursuit is still future work; the current active follow controller is body-frame follow only
- VIO is intentionally deferred for now

## Active Workspace

Active ROS packages under `ros2/src`:

- `drone_bringup`: MAVROS launch/config, external-pose launch flows, and D455 launch
- `drone_control_pkg`: keyboard teleop, demo sequences, external-pose adapters/bridges, and active control nodes
- `ros2_poselib`: leftover utility package kept for compatibility during the rehaul

Archived legacy packages:

- `archive/legacy_stack/ros2/src/drone_behavior_pkg`
- `archive/legacy_stack/ros2/src/drone_vision_pkg`
- `archive/legacy_stack/ros2/src/drone_light_pkg`
- `archive/legacy_stack/ros2/src/drone_monitor_pkg`

## Key Files

- `problem.md`: current PX4 / OFFBOARD blocker context
- `milestone_planner.md`: current implementation checklist and transform plan
- `docs/archived/`: older hover, VIO, recovery, and reference notes moved out of the repo root
- `docs/rehaul/d455_vio_contract.md`: retained D455 contract notes for later VIO work
- `docs/rehaul/host_setup.md`: host dependency setup notes
- `docs/rehaul/isaac_ros_release32.md`: pinned Isaac fallback notes
- `docs/rehaul/isaac_vslam_d455.md`: retained Visual SLAM notes for later

## Active Launch Flows

Launch MAVROS only:

```bash
ros2 launch drone_bringup drone.launch.py
```

Launch the D455 with VIO-oriented defaults:

```bash
ros2 launch drone_bringup realsense_d455.launch.py
```

Run keyboard teleop for manual fallback:

```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

Use the legacy bench-only external-vision path if needed for plumbing checks:

```bash
ros2 launch drone_bringup bench_offboard.launch.py
```

Launch the current hover demo:

```bash
ros2 launch drone_bringup position_hover_demo.launch.py
```

Launch the current target-follow control backend:

```bash
ros2 launch drone_bringup target_follow.launch.py
```

Enable target follow when you are ready to let the controller command body-frame motion:

```bash
ros2 service call /cdrone/drone01/control/target_follow_enable std_srvs/srv/SetBool "{data: true}"
```

Disable it again:

```bash
ros2 service call /cdrone/drone01/control/target_follow_enable std_srvs/srv/SetBool "{data: false}"
```

Launch Isaac ROS Visual SLAM on the D455 path:

```bash
./scripts/start_isaac_realsense_container.sh
./scripts/launch_isaac_vslam_d455_in_container.sh
```

Bridge a generic external-pose source into MAVROS `vision_pose/pose`:

```bash
ros2 launch drone_bringup external_pose_px4_bridge.launch.py \
  pose_source:=optitrack \
  optitrack_server:=<motive-host>
```

Launch the second demo that goes one step past hover and flies to a specified local `map`-frame coordinate:

```bash
ros2 launch drone_bringup position_goto_demo.launch.py \
  goal_x_m:=0.5 \
  goal_y_m:=0.0 \
  goal_z_m:=0.7
```

Here, "world frame" means the local PX4 / MAVROS `map` frame from `/mavros/local_position/pose`, not GPS latitude/longitude.

The goto demo now loads `ros2/src/drone_bringup/config/drone_studio_perimeter.yaml` by default and will:

- reject goals outside the studio boundary
- reject goals inside the pillar keep-out polygon
- reject goals above the configured ceiling
- reject straight-line goal paths that would cross the boundary or the pillar keep-out
- abort into land if the vehicle drifts outside the perimeter during the demo

Launch the circle variant to take off, move to a safe orbit entry point, fly a `2.0 m` altitude circle inside the studio perimeter, hold briefly, and land:

```bash
ros2 launch drone_bringup position_circle_demo.launch.py
```

The default circle demo now derives a perimeter-checked orbit around the `studio_pillar` keep-out polygon, with `1.2 m` pillar clearance, `2.0 m` altitude, and one loop. It uses the same perimeter guard as the goto demo, so the orbit is rejected if the planned path would leave the studio boundary, enter the pillar keep-out polygon, or exceed the configured ceiling.

Start the goto sequence explicitly:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_start std_srvs/srv/Trigger "{}"
```

Abort it if needed:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_abort std_srvs/srv/Trigger "{}"
```

## Studio Safety Geometry

For studio safety, the best fit for this repo is a companion-side local-frame fence:

- define the allowed studio area as a polygon in the same local `map` frame used by `/mavros/local_position/pose`
- define the pillar as a 4-point keep-out polygon in that same frame
- reject static goals outside the polygon or inside the pillar bubble
- block target-follow motion when the next commanded motion would cross either boundary

Do not calibrate against the physical wall line unless you really mean to fly that close. Capture the safe inner boundary you actually want to enforce.

Template config:

- `ros2/src/drone_bringup/config/perimeter.yaml`
- `ros2/src/drone_bringup/config/drone_studio_perimeter.yaml`: finalized studio geometry from the latest calibration pass

Interactive calibration helper:

```bash
python3 scripts/calibrate_perimeter.py \
  --topic /mavros/local_position/pose \
  --output ros2/src/drone_bringup/config/perimeter.yaml \
  --boundary-labels north_west north_east south_east south_west \
  --pillar-labels pillar_nw pillar_ne pillar_se pillar_sw
```

If MAVROS local pose is not live yet, the same helper can sample the adapter output instead:

```bash
python3 scripts/calibrate_perimeter.py \
  --topic /cdrone/drone01/external_pose/input_pose \
  --best-effort
```

Bridge the live RealSense VSLAM pose through the same contract:

```bash
ros2 launch drone_bringup vslam_px4_bridge.launch.py
```

Capture advisor-ready VSLAM evidence:

```bash
./scripts/capture_isaac_vslam_outputs.sh
```

## D455 Defaults

The active D455 launch is tuned for VIO-style input rather than RGB/depth demos:

- color disabled by default
- depth disabled by default
- infra1 and infra2 enabled
- gyro and accel enabled
- IMU unification enabled
- IR profile pinned to `640x360x90`
- gyro and accel pinned to `200 Hz`
- emitter disabled by default

The default profile lives in:

- `ros2/src/drone_bringup/config/realsense_d455_vio.yaml`

Validation script:

```bash
./scripts/check_d455_topics.sh
```

## Host Setup

The host bootstrap script now targets the active external-pose-first stack:

```bash
./setup_jetson.sh
```

It is designed to install:

- ROS 2 Humble
- MAVROS + mavros extras
- RealSense ROS driver
- image/TF/IMU support packages
- GeographicLib datasets
- minimal Python dependencies for the active repo

Current environment note:

- this Jetson is on Ubuntu `22.04.5`
- Jetson Linux is `R36.4.4`
- ROS 2 Humble is installed
- Docker access now works in this session
- the RealSense blocker is no longer the main issue
- the next blocker is validating the new VSLAM-to-MAVROS bridge with the flight controller online

Quick audit:

```bash
./scripts/check_vio_host_status.sh
```

## Archive

Legacy docs, launch files, and the old autonomy packages are preserved under:

```text
archive/legacy_stack/
```

Use that archive when you need to recover old behavior, test notes, or implementation details without letting the active repo drift back toward the old autonomy-first shape.
