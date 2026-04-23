# Repo Memory

This repository is the old `cdrone_control` stack for a Jetson Orin Nano companion computer driving a PX4 aircraft through MAVROS. It is being rehauled, so this file exists to remember what used to matter before the implementation gets replaced.

## Rehaul Update

As of 2026-03-20, Phase 0 and the repo-side part of Phase 1 have been applied:

- the autonomy-heavy packages were moved to `archive/legacy_stack/ros2/src/`
- the old autonomy launch files were moved to `archive/legacy_stack/ros2/src/drone_bringup/launch/`
- the old plans, setup notes, and test procedures were moved to `archive/legacy_stack/docs/`
- the active README is now VIO-first instead of autonomy-first
- `realsense_d455.launch.py` now loads a dedicated VIO-oriented D455 profile from `ros2/src/drone_bringup/config/realsense_d455_vio.yaml`
- host bootstrap and validation scripts were rewritten around ROS 2 Humble + MAVROS + RealSense bringup
- a pinned Isaac ROS `release-3.2` fallback path was added under `external/isaac_ros_release32`
- repo-local Isaac helper scripts were added for bootstrap, interactive dev-shell launch, headless container start, and D455 launch inside the container

Machine-state note from the same session:

- the Jetson is on Ubuntu `22.04.5 LTS`
- Jetson Linux reports `R36.4.4`
- ROS 2 Humble and the active MAVROS / RealSense host packages were installed later in the session
- Docker access was enabled later in the session
- the first native host RealSense path still failed with `bad optional access` even after the D455 was back on USB 3
- the Isaac ROS `release-3.2` image build later succeeded on this same Jetson
- `rs-enumerate-devices` inside the Isaac container saw the D455 cleanly and reported firmware `5.16.0.1`
- the Isaac-container `realsense2_camera` node could start and advertise the expected D455 topics, but it still spammed XU / `control_transfer` errors and no live IR or IMU samples were observed from subscriber checks
- a clean reboot on 2026-03-21 did not change that behavior
- the post-reboot host kernel showed repeated `uvcvideo ... Unknown video format ...` lines and `Failed to query ... UVC control ...: -71` errors for the D455
- a raw `v4l2-ctl` stream test reached `VIDIOC_STREAMON` on `/dev/video5` but still produced a zero-byte file, which pushed the active diagnosis below ROS and below Docker
- the repo's Isaac launch helper originally killed itself because its `pkill -f` cleanup matched its own shell command; that helper was fixed the same day to use anchored process-name matching instead
- on 2026-03-24 the camera came back with firmware `5.13.0.50`, and the active Isaac container path then matched the intended version set: D455 firmware `5.13.0.50`, `realsense2_camera 4.51.1`, `librealsense 2.55.1`, and `release-3.2` with `ros2_humble.realsense`
- after that downgrade, live gyro, unified IMU, and IR frames finally flowed on this Jetson
- the remaining launch quirk was NVIDIA's documented D455 issue where IR can start near `15 Hz`; re-applying `depth_module.enable_auto_exposure=true` at runtime restored the measured IR rate to about `90 Hz`, and the repo helper was updated to do that automatically
- later on 2026-03-24, the repo advanced into Isaac ROS Visual SLAM: the container was updated with `isaac_ros_visual_slam`, `isaac_ros_examples`, and `isaac_ros_realsense`, and the live D455 VSLAM launch produced `/visual_slam/tracking/odometry` at about `90 Hz`
- the repo now has helper scripts for ensuring VSLAM dependencies in the container, launching the RealSense-backed VSLAM stack, and capturing evidence into `temp_outputs/`
- the repo now also has a bootstrap host launch, `vslam_px4_bridge.launch.py`, that relays `/visual_slam/tracking/vo_pose` into MAVROS `vision_pose/pose`
- the new blocker after that milestone is no longer "does the camera work?" but "how do we validate and harden the VSLAM-to-PX4 bridge safely?"

That matters because the current blocker to Phase 2 is partly repo work and partly machine setup.

## Purpose

The repo was built around two control stories:

1. Indoor/manual flying from the Jetson keyboard through MAVROS `ManualControl`.
2. Autonomy through velocity `OFFBOARD`, with the long-term goal of replacing GPS with visual-inertial localization.

The current blocker was not "MAVROS does not connect." MAVROS connected fine. The blocker was that PX4 1.16.0 on the ARKV6X would not accept a real indoor `OFFBOARD` workflow without a trusted aiding source. A dummy bench pose was enough to prove the plumbing, but not enough to make the stack flightworthy.

## What Was Working

- MAVROS connected to PX4 over `mavlink-router` UDP.
- Arming worked in `STABILIZED` during manual testing when throttle was held low.
- `keyboard_teleop_node` could publish centered `ManualControl` sticks and switch modes.
- Keyboard key `2` could switch PX4 into `ALTCTL`.
- `mavros_velocity_node` could stream velocity setpoints at 20 Hz.
- A bench-only external-vision publisher could make PX4 reach `OFFBOARD` in a controlled test.
- `/mavros/local_position/pose` and `/mavros/local_position/velocity_body` were alive enough to show the system was not completely dead.

## What Was Blocked

- Real `OFFBOARD` flight indoors was blocked by missing external vision or VIO.
- PX4 could acknowledge `OFFBOARD` requests and still remain in `AUTO.LOITER`.
- Normal arming was still rejected during the later bench tests, even after recalibration.
- The MAVROS `odometry` plugin was denylisted, so a proper `/mavros/odometry/out` path was not in use.
- There was no real estimator in the repo yet, only scaffolding and placeholders.

## Package Roles Before Archival

- `drone_bringup`: launch files and MAVROS configuration.
- `drone_control_pkg`: keyboard teleop, MAVROS bridge nodes, bench pose publisher, and the VIO bridge stub.
- `drone_behavior_pkg`: autonomy state machine, health monitoring, and behavior logic.
- `drone_vision_pkg`: stereo perception and tracking.
- `drone_light_pkg`: lighting control.
- `drone_monitor_pkg`: dashboard and telemetry aggregation.
- `drone_msgs`: shared ROS message definitions.
- `ros2_poselib`: pose helper library.

The first four archived packages now live under `archive/legacy_stack/ros2/src/`.

## Important Launch Flows

- `ros2 launch drone_bringup drone.launch.py` brought up MAVROS only.
- `ros2 run drone_control_pkg keyboard_teleop_node` handled manual flight from the Jetson keyboard.
- `ros2 launch drone_bringup bench_offboard.launch.py` brought up the bench-only `OFFBOARD` test path.
- `ros2 launch drone_bringup autonomy_stack.launch.py` launched the full autonomy stack. It is now archived under `archive/legacy_stack/ros2/src/drone_bringup/launch/`.
- `ros2 launch drone_bringup realsense_d455.launch.py` was added as the D455 bringup entry point and is now part of the active path.

## Control Paths That Existed

- Manual backend: keyboard teleop -> MAVROS `ManualControl` -> PX4 `ALTCTL` or `STABILIZED`.
- Velocity backend: `cmd_vel_body` -> `mavros_velocity_node` -> `/mavros/setpoint_velocity/cmd_vel` -> PX4 `OFFBOARD`.
- Bench VIO path: fixed pose publisher -> `/mavros/vision_pose/pose` -> PX4 external-vision plumbing.
- Future VIO path: `vio_bridge_node` was intended to republish real pose data into MAVROS, but it was only a bridge stub.

## Repo Intentions Worth Preserving

- Keep the distinction between manual indoor control and autonomy control.
- Keep the bench-only dummy pose idea as a non-flight test harness.
- Keep the idea of a dedicated VIO bridge, even if the estimator underneath changes.
- Keep the launch-time separation between bringup, autonomy, and testing.
- Keep the focus on PX4 external-vision fusion rather than assuming GPS can just be ignored.

## Documents That Mattered

- `docs/notes/offboard_indoor_blocker.md` captured the live failure mode and what had been tried.
- the old `README.md` summarized the previous control paths.
- `archive/legacy_stack/docs/vio_todo.md` outlined the intended VIO integration direction.
- `hackster.md` summarized the GPS-denied Jetson/PX4/RealSense reference workflow.
- `vslam_uav.md` summarized the VSLAM-UAV reference repo and its version constraints.
- `archive/legacy_stack/docs/props_off_test.md` and `archive/legacy_stack/docs/props_on_test.md` captured the old test procedures.
- `archive/legacy_stack/docs/JETSON_PX4_SETUP.md` captured the old MAVROS-based setup assumptions.

## Key Assumptions

- Hardware was a Jetson Orin Nano + ARK PAB carrier + ARKV6X + PX4 1.16.0.
- The system communicated through `mavlink-router`, not a direct serial-only setup.
- Indoor testing was bench-first and props-off first.
- The D455 was already connected and was intended to replace the D435i-style reference workflow from the notes.
- PX4 external vision needed a real, trusted estimate source before `OFFBOARD` could be treated as usable.

## Things That Happened During the Investigation

- `CBRK_VELPOSERR` turned out not to exist in PX4 1.16.0.
- `COM_RC_IN_MODE=4` fixed a separate RC/manual-control issue.
- GPS removal alone did not solve the `OFFBOARD` problem.
- Bench force-arm attempts still failed once the system-health issue remained.
- The repo already had a partial VIO bridge and D455 bringup, which meant the rehaul should extend the existing experiment trail rather than pretend it started from zero.
- The first rehaul pass confirmed the next real blocker after repo cleanup is host setup: ROS 2 Humble + ROS packages + RealSense access still need privileged install steps on this Jetson.
- the later Isaac ROS pass confirmed that version pinning helps, but the remaining D455 blocker on this Jetson is now below "wrong ROS package version" and looks more like a low-level device-control / stream-delivery problem.
- the post-reboot retest strengthened that conclusion: the failure reproduces even when the camera is on USB 3 and even when the ROS launch helper is bypassed.

## Do Not Forget

- The real problem was missing aiding, not missing MAVROS.
- Bench success with a fake pose was only a plumbing check.
- `ALTCTL` manual teleop was the safe indoor fallback.
- The old repo was already pointing toward VIO, but it never had a finished estimator wired into PX4.
- The old autonomy stack now lives in `archive/legacy_stack/` instead of the active workspace.
- Host access was needed to finish Phase 1, and that install work has now been completed on this Jetson.
- The Isaac ROS fallback is now the active path: with the D455 on firmware `5.13.0.50`, the repo can bring up live RealSense-backed Visual SLAM and publish odometry at about `90 Hz`.
