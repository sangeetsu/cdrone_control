# Milestone 4 Runbook

Commands below assume your shell already loaded ROS 2 and the local workspace.
For paired operation, use these settings in **every command terminal on both
Jetsons** before starting ROS processes:

```bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=1
```

If a ROS CLI daemon was started with different settings, run `ros2 daemon stop`
once after exporting them. Each Jetson discovers only its own stack. SSH is
used for launch and explicit start/check/abort requests; there is no exchange
of target tracks, poses, mission state, or heartbeats between the drones.
Detailed notes live in `docs/milestones/milestone4.md`. For SDK, topic,
camera, detector, and reboot diagnosis, use
[Milestone 4 troubleshooting](milestone4_troubleshooting.md).

Milestone 4 defaults to ZED 2i vision pose fused by PX4, with no mocap or GPS.
One ZED process sends pose to MAVROS and hands color/depth arrays directly to
the detector in memory; the detector and target map
follow moving drones through short detection gaps. PX4's local `map` frame is
launch-relative odometry, not a surveyed room map.

The old OptiTrack launch remains available with `pose_source:=optitrack` for
legacy comparisons. Its studio perimeter and 180-degree adapter transform do
not apply to the ZED local route.

The launch defaults to `enable_flight:=false`. The start service refuses to arm
in that mode. Reconnect flight hardware and use the supervised flight command
only after the bench checks and a clear area are confirmed.
The ZED-local mission uses `models/best.engine` on CUDA for drone OBB detection.
It will not fall back to `.pt` or `.onnx`. A TensorRT engine is not necessarily
INT8 quantized; this engine has no embedded precision or task metadata, so its
output and real-world speed must be verified on this Jetson.

## Build

After pushing the changes, pull the same `OF_localise` revision and run this
build on **both** Jetsons. Each needs its own ZED SDK, compatible detector
engine, and measured mount file. Do not copy cdrone3's calibration onto cdrone4.

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Paired launch setup

The launching Jetson is `cdrone3`; the peer is `jetson@192.168.0.121`
(`cdrone4`). Both use ZED 2i and `/home/jetson/cdrone_control`. The launch passes
explicit identities and namespaces, so pulling the tracked cdrone3 defaults
on the peer does not cause a namespace collision in this paired workflow.

Set up SSH key authentication from cdrone3, then verify noninteractive access:

```bash
ssh-copy-id jetson@192.168.0.121
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes jetson@192.168.0.121 \
  'test -f /home/jetson/cdrone_control/ros2/install/setup.bash && test -f /home/jetson/zed_hover_mount_run2.yaml'
```

The peer needs its **own** calibration at that path. Override
`peer_zed_mount_config` if it uses another filename. The other optional
arguments are `peer_user`, `peer_workspace`, and `peer_drone_id`.
Initial inspection found SSH authentication failing; complete this setup before
trying paired launch. A failed peer launch prevents the shared start service
from starting either drone.

The peer runs in a detached process session, with logs in
`~/.ros/milestone4/cdrone4_<UTC timestamp>.log` and its PID/log path in
`~/.ros/milestone4/session.json`. Closing SSH does not stop it. Each host allows
only one Milestone 4 launch at a time, including a single-drone launch.

## One-time setup without RC or QGC

`companion_only:=true` enables **read-only** checks of the additional PX4
settings below. It does not change flight-controller parameters. The existing
ZED vision profile still applies. The added profile is
[`milestone4_companion_only_px4_1_16.yaml`](ros2/src/drone_bringup/config/milestone4_companion_only_px4_1_16.yaml).

| PX4 parameter | Required value | Purpose |
| --- | --- | --- |
| `COM_RC_IN_MODE` | `4` | Disable RC/joystick stick input |
| `NAV_DLL_ACT` | `0` | Do not require a QGC/GCS connection |
| `COM_OBL_RC_ACT` | `4` | Land if OFFBOARD control is lost |
| `COM_OF_LOSS_T` | `1.0` | Retain the existing OFFBOARD-loss timeout |

These are for PX4 1.16; see the [PX4 OFFBOARD documentation](https://docs.px4.io/v1.16/en/flight_modes/offboard).
With stick input disabled, RC/joystick takeover is unavailable. Use each
drone's abort service to request landing; loss of its onboard OFFBOARD stream
still invokes PX4's landing failsafe.

Start a flight-disabled stack first. For both drones together, use the paired
flight command below with `enable_flight:=false`. On **each Jetson's own
terminal**, source ROS and the workspace, export the localhost settings above,
and set `M4_DRONE_ID=cdrone3` locally or `M4_DRONE_ID=cdrone4` on the peer.
With motors disconnected, confirm the state reports `armed: false`, then back
up the parameter mirror before making the four explicit changes:

```bash
M4_DRONE_ID=cdrone3  # Use cdrone4 on 192.168.0.121.
ros2 topic echo --once /cdrone/$M4_DRONE_ID/mavros/state
ros2 service call /cdrone/$M4_DRONE_ID/mavros/param/pull \
  mavros_msgs/srv/ParamPull '{force_pull: true}'
M4_BACKUP="$HOME/milestone4_params_${M4_DRONE_ID}_$(date -u +%Y%m%dT%H%M%SZ).yaml"
ros2 param dump /cdrone/$M4_DRONE_ID/mavros/param > "$M4_BACKUP"

ros2 param set /cdrone/$M4_DRONE_ID/mavros/param COM_RC_IN_MODE 4
ros2 param set /cdrone/$M4_DRONE_ID/mavros/param NAV_DLL_ACT 0
ros2 param set /cdrone/$M4_DRONE_ID/mavros/param COM_OBL_RC_ACT 4
ros2 param set /cdrone/$M4_DRONE_ID/mavros/param COM_OF_LOSS_T 1.0

ros2 param get /cdrone/$M4_DRONE_ID/mavros/param COM_RC_IN_MODE
ros2 param get /cdrone/$M4_DRONE_ID/mavros/param NAV_DLL_ACT
ros2 param get /cdrone/$M4_DRONE_ID/mavros/param COM_OBL_RC_ACT
ros2 param get /cdrone/$M4_DRONE_ID/mavros/param COM_OF_LOSS_T
```

Require a successful parameter pull, a populated backup, successful sets, and
the four matching readbacks. Stop the stacks while disarmed, reboot the FCUs,
and repeat readback/readiness checks to verify persistence. Retain the backup
to restore the original values explicitly if returning to RC/joystick use.
Do not apply these changes while airborne.

Telemetry stream configuration and rates are unchanged: there is **no new
30 Hz request or minimum-rate requirement**. The existing 5 Hz estimator-status
request remains. Readiness additionally checks the velocity data already
needed by stable hover: finite values received within the existing 0.5 s
freshness window. A missing stream is reported rather than having its rate
changed. Verify readiness with RC and QGC disconnected.

## ZED 2i Preflight

Confirm the ZED 2i is attached over USB and the SDK imports:

```bash
lsusb | rg -i 'stereolabs|zed'
python3 -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
uname -m
lsb_release -a
cat /etc/nv_tegra_release
```

Expected on the current Jetson:

- `lsusb` shows `STEREOLABS ZED 2i`
- Python prints a ZED SDK version, currently validated with `5.4.0`
- architecture is `aarch64`
- Ubuntu is `22.04`
- L4T/JetPack matches the Stereolabs Jetson installer you are using

## ZED 2i Image Smoke Test

Do this before launching the tracker or any flight behavior:

```bash
cd /home/jetson/cdrone_control
python3 scripts/capture_zed2i_smoke.py \
  --rotate-180 \
  --output-dir /home/jetson/output_dump/zed2i_smoke
```

Expected outputs:

- `/home/jetson/output_dump/zed2i_smoke/rgb.png`
- `/home/jetson/output_dump/zed2i_smoke/depth.png`
- `/home/jetson/output_dump/zed2i_smoke/metadata.json`

The ZED 2i is currently mounted upside down on cdrone3. Milestone 4 uses
`zed_rotate_180: true` in `zed2i_tracking.yaml`, which rotates both RGB and
depth frames before detection/depth lookup and rotates the camera intrinsics
principal point before body/world projection. Keep the smoke test's
`--rotate-180` option aligned with that runtime config.

## ZED 2i Floor-Mark Depth Ladder

Use this non-flight check to measure ZED depth for Milestone 4. It uses
floor marks as truth, not OptiTrack, and records both raw center depth and
detector-bbox depth with `models/best.engine`.
Keep the target drone stationary at the requested mark. Capture the first range
at `1.2 m`:

```bash
cd /home/jetson/cdrone_control
python3 scripts/capture_zed2i_depth_ladder.py \
  --range-m 1.2 \
  --output-dir /home/jetson/output_dump/zed2i_depth_ladder \
  --rotate-180 \
  --model-path /home/jetson/cdrone_control/models/best.engine
```

Repeat for `2.4`, `3.6`, `4.8`, `6.0`, and `7.2` after moving the target drone
to each marked range. Continue at marked ranges through `20 m`, using the same
target and floor-mark truth. Stop after two consecutive unusable ranges and
treat the 20 m gate as failed. The tool accepts OBB outputs from `best.engine`.

After one or more captures, generate the floor-mark summary:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
zed_depth_ladder_report /home/jetson/output_dump/zed2i_depth_ladder
```

Outputs:

- `/home/jetson/output_dump/zed2i_depth_ladder/<range>/depth_ladder_frames.csv`
- `/home/jetson/output_dump/zed2i_depth_ladder/<range>/summary.json`
- `/home/jetson/output_dump/zed2i_depth_ladder/zed_depth_ladder_summary.csv`
- `/home/jetson/output_dump/zed2i_depth_ladder/zed_depth_ladder_summary.md`

## Generate Visualizations

```bash
cd /home/jetson/cdrone_control
python3 scripts/plot_milestone4_prediction_demo.py
```

Outputs:

- `docs/generated/milestone4_target_memory_architecture.svg`
- `docs/generated/milestone4_prediction_path.svg`

The performance profile leaves video writing off. The supervised flight
command below saves tracking CSVs, not MP4s. To record MP4 visualizations
for a separate supervised run, append
`tracker_record_mission_video:=true tracking_metrics_write_video:=true` to the
full launch. These options add camera, GPU, and disk load. With video enabled,
the outputs include:

- `mission_recordings/mission_<drone>_<timestamp>_<state>_combined.mp4`
- `mission_recordings/mission_<drone>_<timestamp>_<state>_yolo.mp4`
- `mission_recordings/mission_<drone>_<timestamp>_<state>_target_map.mp4`

Tracking metrics continue to save CSVs when the flight/demo exits the active
state. A cleaned-path MP4 is produced only when video is enabled:

- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/tracking_samples.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/tracking_frames.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/cleaned_target_path.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/tracking_summary.csv`
- `mission_recordings/tracking_metrics/tracking_<drone>_<timestamp>_milestone4/cleaned_target_path.mp4`

The cleaned-track postprocess runs by default. With optional video enabled, it
waits up to `5 s` for the nearest matching `mission_<drone>_*_combined.mp4`,
then renders `cleaned_target_path.mp4` with the same FPS, frame count, and
duration as that combined recording. `cleaned_target_path.csv` stays point-based, but its
`timestamp_s` column is relative and starts from `0.0` instead of using absolute
ROS time.

For Milestone 4, the combined video replaces the old depth-map quadrant with a
normal detector-only YOLO quadrant. The YOLO and combined videos also include a
second annotated tile with target-map burn-in overlays when
`/cdrone/cdrone3/target_map/world_tracks` is live. The target-map MP4 is a
top-down recording of the defense drone path and target-map tracks. The separate
depth MP4 is disabled by default for Milestone 4.

## No-mocap full mission launch, flight disabled

Stop any Milestone 1 hover or square launch first. Both missions need exclusive
control of MAVROS and the ZED camera. Use the measured mount from the reported
hover; if the mount has moved, recalibrate before following targets.

The full mission remains one launch. Its ZED-local default uses
`models/best.engine` on CUDA with OBB postprocessing and the one-class drone
label map. It starts MAVROS, a combined ZED VIO/detector process, target map,
controller, and CSV metrics. With `enable_flight:=false`, it does not arm or
command flight:

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  pose_source:=zed_local enable_flight:=false \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml \
  max_target_distance_m:=20.0 max_local_excursion_m:=20.0
```

The tracker startup log should show `best.engine`, `model_task=obb`, and CUDA
device `0`. The readiness check needs a running detector and fresh perception
status, but no target in view. Use a visible target during the separate 3D
tracking and range tests below.

For this launch, MAVROS starts first and the combined ZED/detector process
starts 15 s later. The ZED camera remains configured for HD720 at 30 FPS, but
SDK depth and pose computation is capped at 20 FPS by
`zed_grab_compute_capping_fps:=20.0`; detector frames are sampled at up to
`zed_image_rate_hz:=15.0`.
The SDK's [frame-rate guide](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/frame-rate-and-latency)
explains that this cap limits depth and tracking computation while the sensor
continues capturing at 30 FPS.

The launch uses MAVROS with `zed_mavros.yaml` and one SDK camera owner inside
`zed_local_combined_node`. It sends ZED pose to MAVROS and passes 180-degree
rotated color, metric depth, capture stamp, and intrinsics directly to the
detector.

For a standalone ZED plus `best.engine` ROS bench check without MAVROS or
the mission controller, first stop the full Milestone 4 launch. Follow the
[GPU readiness procedure](milestone4_troubleshooting.md#gpu-inference-and-power)
before running:

```bash
ros2 launch drone_bringup tracking_only.launch.py \
  embedded_zed:=true source_mode:=embedded_zed \
  tracker_config_file:=/home/jetson/cdrone_control/ros2/src/drone_vision_pkg/config/zed2i_tracking.yaml \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml \
  zed_depth_mode:=PERFORMANCE zed_image_rate_hz:=15.0 \
  zed_grab_compute_capping_fps:=20.0 zed_ready_before_inference_s:=5.0 \
  model_path:=/home/jetson/cdrone_control/models/best.engine \
  model_task:=obb \
  model_data_config:=/home/jetson/cdrone_control/ros2/src/drone_vision_pkg/config/drone_detector.yaml \
  inference_device:=0 record_mission_video:=false \
  publish_world_track_compare:=false
```

This uses one ZED SDK camera and the same input-power guard as the full
mission. It still runs ZED VIO inside the combined process so the detector
gets synchronized color, depth, and camera pose. It does not command flight.
Check `/cdrone/cdrone3/zed/tracking_status`,
`/cdrone/cdrone3/perception/status`, and
`/cdrone/cdrone3/perception/tracks`. Without MAVROS local pose, this bench
check does not validate PX4 fusion or world-frame target tracks.

Require sustained `tracking_status: OK`, fresh ZED pose and perception
updates, and a visible detected 3D target. The detailed timing checks and
failure interpretations are in [Milestone 4 troubleshooting](milestone4_troubleshooting.md).

With motors disconnected, verify the PX4 1.16 parameters match the same
[ZED profile](ros2/src/drone_bringup/config/zed_hover_px4_1_16.yaml) used
for Milestone 1:

```bash
python3 scripts/inspect_onboard_localization.py \
  --profile ros2/src/drone_bringup/config/zed_hover_px4_1_16.yaml \
  --port udpin:127.0.0.1:14569
ros2 topic echo --once /cdrone/cdrone3/zed/tracking_status
ros2 topic hz /cdrone/cdrone3/zed/pose_cov
ros2 topic hz /cdrone/cdrone3/perception/status
ros2 topic echo --once /cdrone/cdrone3/mavros/estimator_status
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/pose
ros2 service call /cdrone/cdrone3/demo/milestone4_check std_srvs/srv/Trigger "{}"
```

Require `tracking_status: OK`, fresh pose and detector status, valid PX4 horizontal
position and velocity, a recent `/cdrone/cdrone3/perception/status` message,
and `milestone4_check` success even with no target visible. The check reports
`enable_flight=false` separately. To require a visible detected 3D target
before starting, opt in with `require_detector_target_on_start:=true`.
During setup, the PX4 console (QGroundControl's MAVLink Console is an optional
way to access it)
`listener estimator_status_flags -n 1` must show sustained `cs_ev_pos`,
`cs_ev_hgt`, and `cs_ev_yaw`; ROS estimator validity alone does not identify
which sensor PX4 is fusing. Move the disconnected airframe by known X/Y/Z
distances and yaw as in the Milestone 1 runbook. Confirm local pose direction,
depth, target body coordinates, and camera extrinsics before flight.

Move a target drone by hand while the pursuing drone remains stationary, then
move the pursuing drone while the target remains stationary. Check that
`perception/world_tracks` and `target_map/world_tracks` remain in the same
local frame, that target velocity reflects target motion, and that map IDs
survive brief detector dropouts. A missing camera frame produces no new
projection; short predictions may remain. Stale ownship pose or ZED tracking
loss clears the target map and blocks pursuit.

With motors disconnected and stable power, record repeated bench observations
at floor marks through 20 m. For the planned moving-target flight, require at
least **10 Hz sustained detected 3D target updates**; the camera's 20 Hz
capture rate and the tracker's nominal 15 Hz timer do not establish this.
Require no detected 3D update gap above 0.5 s, valid target range on at least 90% of
frames at 20 m, and measured range error no greater than 2 m there. Test a
stationary target while moving the ownship by hand, then a target moving up to
1 m/s while the ownship moves. Check that projected world positions and
velocities remain consistent. Do not fly if any gate fails. Neither the
engine's `.engine` extension nor a successful readiness call proves those
range and motion gates.

## Supervised flight

Use a physically clear area that accommodates a 20 m radius, the airframe,
and stopping margin. The local limit measures odometric excursion from
takeoff; it does not detect obstacles or establish a surveyed boundary.
Recheck each mount, PX4 profile, vision fusion, and independent abort path.
QGroundControl and RC are not required with the companion-only profile.
Stop both bench stacks before reconnecting flight hardware; stopping the local
launch does not stop the detached peer (see the stop commands below).

```bash
ros2 launch drone_bringup milestone4_demo.launch.py \
  pose_source:=zed_local enable_flight:=true \
  companion_only:=true launch_peer:=true \
  peer_host:=192.168.0.121 peer_drone_id:=cdrone4 \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml \
  peer_zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml \
  takeoff_altitude_m:=2.0 altitude_tolerance_m:=0.10 \
  required_completion_count:=1 max_target_distance_m:=20.0 \
  max_local_excursion_m:=20.0 max_vel_xy_mps:=0.6
```

Use this command only after the full flight-disabled launch stays stable,
`best.engine` passes the bench rate and 20 m range gates, PX4 fuses ZED vision,
and the shared power rails remain stable under load.

Run the local check, and optionally the peer check below. Then call the
**unchanged start service**: in paired mode it checks both stacks, including
`enable_flight`, before sending either start request. It sends the requests
concurrently and reports each result; acceptance does not mean both vehicles
are already airborne or guarantee an identical takeoff instant. A failed
readiness check sends neither start. Partial success is reported explicitly:
an accepted mission continues independently, with no automatic abort or retry.
An uncertain start response blocks another paired attempt until the launcher
is restarted after inspecting both drones.

Each controller warms OFFBOARD, arms, climbs relative to its own takeoff
origin, and hands control to its velocity stream. During handoff and staged
hover it corrects altitude and holds takeoff XY/yaw. With
`altitude_tolerance_m:=0.10`, the accepted climb band is **1.90–2.10 m** for a
2.0 m target, including the boundaries. Exact 2.0 m attainment is unnecessary.
The controller waits for **2 continuous seconds** in that band, with measured
horizontal speed at most 0.25 m/s and vertical speed at most 0.15 m/s. The
altitude-control deadband is half the acceptance tolerance. The existing 30 s
stage timeout requests landing if hover never stabilizes. Targets are ignored
until the state changes to `SEARCH`.

```bash
ros2 service call /cdrone/cdrone3/demo/milestone4_check std_srvs/srv/Trigger "{}"
ros2 service call /cdrone/cdrone3/demo/milestone3_start std_srvs/srv/Trigger "{}"
ros2 topic echo /cdrone/cdrone3/engagement/state
```

Optional peer readiness check from cdrone3 (this exits after one request):

```bash
ssh jetson@192.168.0.121 'source /opt/ros/humble/setup.bash && source /home/jetson/cdrone_control/ros2/install/setup.bash && export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=1 && ros2 run drone_control_pkg milestone4_session check --drone-id cdrone4'
```

The local mission controller's private start endpoint is
`/cdrone/cdrone3/demo/milestone3_start_local` in paired mode; use the public
service above for both drones. On the peer, the normal
`/cdrone/cdrone4/demo/milestone3_start` remains local to that Jetson.
`required_completion_count` defaults to `1`, must be an integer of at least
`1`, and is applied separately to each drone. A completed target-map track ID
counts once per mission. Each drone returns and lands on reaching its own
count, regardless of the other drone's progress.

For a single drone, set `launch_peer:=false` (the default). Its original start
service then directly starts only that drone. A single-drone launch on cdrone4
must explicitly set `drone_id:=cdrone4 drone_namespace:=/cdrone/cdrone4
mavros_namespace:=/cdrone/cdrone4/mavros` if the installed defaults still name
cdrone3.

The latest saved failed run reached `LOCAL_HANDOFF` and then spent 30 s in
`STAGE_HOVER`; it had already passed the takeoff-altitude threshold. Older logs
did not record which hover condition failed. New `Waiting for SEARCH` logs and
the engagement message's `blocked_reason` include altitude error, velocity
age, horizontal/vertical speed, and dwell progress. The timeout includes this
detail. Check velocity without changing its configured rate:

```bash
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/velocity_local
ros2 topic echo /cdrone/cdrone3/engagement/state
```

In `SEARCH`, the mission sends zero velocity and waits indefinitely for a
confirmed target. If none appears, use that drone's abort service below to
land; PX4 failsafes remain available. A new target needs three coherent
3D observations spanning at least 0.2 s before pursuit starts. First follow a
stationary target at the reduced 0.6 m/s cap. After takeoff,
handoff, follow, return, landing, and disarm all complete cleanly, repeat
with a slowly moving target. Then increase `max_vel_xy_mps` to `1.2` and
test targets up to 1 m/s in stages. The 1.5 m standoff and three-second
completion dwell remain. A target flying directly away at 1 m/s may not be
caught quickly from a long starting range.

At the 20 m limit, the controller holds horizontal velocity and searches;
it may follow again if the target moves into an allowed path. One rejected ZED
pose jump commands zero velocity, clears target history, and allows at most
0.5 s for two fresh accepted vision poses with valid PX4 odometry. Recovery
returns to `SEARCH`, or restarts the stable-hover dwell if the jump occurred
there. A persistent jump requests landing in place. A PX4 local
pose jump, estimator failure, or sustained ZED pose loss requests landing
immediately. Diagnose jumps with
[Milestone 4 troubleshooting](milestone4_troubleshooting.md).
An abort requested by the operator uses the same landing path:

```bash
ros2 service call /cdrone/cdrone3/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```

This abort affects **cdrone3 only**. To abort cdrone4 from cdrone3:

```bash
ssh jetson@192.168.0.121 'source /opt/ros/humble/setup.bash && source /home/jetson/cdrone_control/ros2/install/setup.bash && export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=1 && ros2 run drone_control_pkg milestone4_session abort --drone-id cdrone4'
```

After both vehicles have landed and disarmed, stop the local launch with
Ctrl-C. Stop the detached peer separately:

```bash
ssh jetson@192.168.0.121 'source /opt/ros/humble/setup.bash && source /home/jetson/cdrone_control/ros2/install/setup.bash && export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=1 && ros2 run drone_control_pkg milestone4_session stop'
```

The peer stop helper requires fresh, connected MAVROS state reporting
`armed: false`, then signals only the recorded launch process group. It
refuses to stop an armed vehicle or signal a reused PID. To inspect peer logs:

```bash
ssh jetson@192.168.0.121 'cat ~/.ros/milestone4/session.json'
```

Read the log path printed there on that Jetson. No peer status is polled during
flight. Loss of SSH, the startup dispatcher, or the other drone does not stop
a mission already running; each onboard controller still handles its own
localization faults, OFFBOARD loss, completion, and landing.

## Paired bench acceptance

Before flight, use `enable_flight:=false` and verify two separate local stacks,
correct per-drone topics/LEDs, and separate recordings. The shared start must
report flight disabled and send no starts. Verify single-drone operation too.
Complete profile readback with RC/QGC disconnected and check that velocity
remains fresh at its existing configured rate. A failed peer SSH connection,
missing calibration, duplicate launch, or failed readiness must be visible.
On a flight-disabled launch, disconnect the SSH session and confirm the remote
stack stays running; stop it using its own stop command afterward.

## Live checks

```bash
ros2 topic echo /cdrone/cdrone3/perception/status
ros2 topic echo /cdrone/cdrone3/perception/world_tracks
ros2 topic echo /cdrone/cdrone3/target_map/world_tracks
ros2 topic echo /cdrone/cdrone3/target_map/tracks
ros2 topic echo /cdrone/cdrone3/engagement/state
```

Detected and predicted targets retain stable map IDs during brief gaps.
Predictions show increasing age and uncertainty and remain speed-limited.
After the hold or uncertainty limit, pursuit stops. Saved recordings and
metrics use PX4 local pose; they cannot be overlaid on the old surveyed
OptiTrack studio map without alignment.

## Rerun Cleaned-Track Postprocess

After a recording is done, you usually do not need to run anything manually.
Use this only to backfill old metrics runs or force a specific combined video:

```bash
ros2 run drone_vision_pkg tracking_metrics_postprocess \
  --run-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/<tracking_run_dir> \
  --reference-video /home/jetson/cdrone_control/mission_recordings/<combined_video>.mp4
```

If `--reference-video` is omitted, the script auto-discovers the nearest
`mission_<drone>_*_combined.mp4` from `/home/jetson/cdrone_control/mission_recordings`.

## Compare With Milestone 3

Run one Milestone 3 flight with `enable_tracking_metrics:=true`, then one
Milestone 4 flight with the default metrics logger enabled. After both runs:

```bash
ros2 run drone_vision_pkg milestone_tracking_compare_report \
  --milestone3-run-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/<milestone3_run_dir> \
  --milestone4-run-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/<milestone4_run_dir> \
  --output-dir /home/jetson/cdrone_control/mission_recordings/tracking_metrics/milestone3_vs_milestone4_report
```

The comparison report writes:

- `comparison_summary.csv`
- `comparison_summary.json`
- `comparison.md`

## RViz Markers

```bash
rviz2
```

Add a `MarkerArray` display and set the topic to:

```text
/cdrone/cdrone3/target_map/markers
```

Color meaning:

- green: detected
- yellow: tracker-held
- orange: target-map predicted

## ZED 2i Pose Note

The default run fuses ZED VIO in PX4 and uses MAVROS local position for target
projection and return. The frame can drift or reset. Treat recordings as local
mission measurements, and stop the flight if ZED tracking or PX4 vision fusion
fails. `pose_source:=optitrack` selects the old studio setup explicitly.
