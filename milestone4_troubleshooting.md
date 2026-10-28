# Milestone 4 Troubleshooting

This document holds Milestone 4 installation fixes, topic and camera diagnosis,
GPU isolation tests, and the Jetson reboot investigation. The operational
sequence and flight gates remain in [milestone4_runbook.md](milestone4_runbook.md).
Do not run GPU detector tests or flight until the Jetson completes inference
without resets and the power rail stays stable under load.

## SDK installation and smoke-test failures

If `import pyzed.sl` fails, install the ZED SDK before launching Milestone 4:

```bash
sudo apt install zstd
```

Download the Jetson/Linux ZED SDK installer that matches this Jetson's
Ubuntu/L4T/JetPack from:

- https://www.stereolabs.com/developers/release
- https://docs.stereolabs.com/docs/development/zed-sdk/linux

Run the downloaded installer, then rerun:

```bash
python3 -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
```

Keep the Python runtime compatible with ROS, PyTorch, TensorRT, Norfair, and
ZED. The ZED 5.x wheel declares `numpy>=2`, but this Jetson's ROS/PyTorch stack
needs NumPy 1.x. Use the no-dependency ZED install path that has been bench
tested with the ZED 2i:

```bash
python3 -m pip install --user --force-reinstall numpy==1.26.4 sympy==1.13.1
python3 -m pip install --user --force-reinstall --no-deps \
  /home/jetson/Downloads/zed_sdk/wheels/pyzed-5.4-cp310-cp310-linux_aarch64.whl
```

If you need Python virtual environments for future isolated probes:

```bash
sudo apt update
sudo apt install -y python3.10-venv
```

If this fails with `ZED SDK Python import failed`, fix the SDK install before
continuing. Milestone 4 should not be flight-tested until the smoke test saves
a valid RGB image, depth visualization, and metadata file.

## Depth-ladder and range errors

Defer this TensorRT test until the current full-launch reset is resolved.

If the overlay shows the yellow ROI marker is not on the drone body, add
`--roi-center-x-px <x> --roi-center-y-px <y>` and retake the range. If
`models/best.engine` reports no accepted boxes with a visible drone, stop:
the selected detector is not ready for flight.

Interpretation:

- If center and bbox median depth are both biased at all ranges, compare ZED
  depth modes before changing tracker math:
  `--depth-mode NEURAL`, `--depth-mode NEURAL_PLUS`, and `--depth-mode QUALITY`.
- If center depth is good but bbox/tracker depth is wrong, tune the detector ROI
  sampling before changing ZED SDK settings.
- If depth is good but body/world range is wrong in tracker-only validation,
  inspect projection, `zed_rotate_180`, and camera-to-body extrinsics.

After the [GPU and power investigation](#gpu-inference-and-power) is resolved, run the
one-terminal diagnostics below before calling the start service. It opens the
ZED and engine but does not arm or command the vehicle. Do not run it while
the present hard-reset problem persists:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
timeout 30s ros2 launch drone_bringup milestone4_diagnostics.launch.py \
  stage:=all zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml
```

Do not run the start service unless this launch stays alive until the timeout
and logs regular ZED poses and OBB tracks without TensorRT or ZED exceptions.

## Launch and readiness failures

### Local odometry jump or invalid PX4 estimator

An elevated table does not by itself invalidate PX4 horizontal position or
velocity. Milestone 4 records the local pose at takeoff and commands a climb
relative to that pose. A change in PX4's local reference while vision fusion
initializes can produce a sudden pose step on the bench, including in height.
Check whether the ZED and PX4 poses settle while the airframe stays still;
do not interpret a single local `z` value as height above the floor.

The October 23 bench launch exposed two differences from the newer Milestone 1
ZED hover path. MAVROS `estimator_status` arrived at **1 Hz**, while the old
Milestone 4 health check required a message within **0.5 s**; readiness could
therefore fail between otherwise valid updates. Milestone 4 now requests the
same 5 Hz estimator stream as Milestone 1 and allows 2.5 s for estimator
telemetry. A preflight pose step now resets a 2 s continuous readiness period
and can recover once the pose and estimator are healthy. A pose step after the
mission starts still aborts and requires a restart if it is a **PX4 local pose**
step. One rejected ZED vision pose during flight now enters a 0.5 s zero-command
hold; persistent ZED jumps still request landing.

Rebuild `drone_control_pkg`, stop the old launch, and source the workspace before
retrying. A process already running keeps the old checks:

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_control_pkg
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

With flight disabled, leave the airframe still and watch the status for several
seconds. The service should first report `waiting for 2 s continuous local
readiness`, then move to the next gate or return `success=True` when the
detector is running, even with no target in view. If it reports
`PX4 estimator: horizontal position invalid`
or `horizontal velocity invalid`, that is a real estimator flag failure, not a
table-height check. A `const_pos_mode_status_flag` by itself can be set on a
resting PX4 vehicle; check the horizontal validity flags and confirm vision
fusion in QGroundControl:

```bash
ros2 topic hz /cdrone/cdrone3/mavros/estimator_status
ros2 topic echo --once /cdrone/cdrone3/mavros/estimator_status
ros2 topic echo --once /cdrone/cdrone3/zed/tracking_status
ros2 topic hz /cdrone/cdrone3/zed/pose_cov
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/pose
ros2 service call /cdrone/cdrone3/demo/milestone4_check std_srvs/srv/Trigger "{}"
```

In QGroundControl's MAVLink Console, run `listener estimator_status_flags -n 1`
and require sustained `cs_ev_pos`, `cs_ev_hgt`, and `cs_ev_yaw`. If horizontal
validity is still false after stable ZED tracking, compare the PX4 vision
profile with Milestone 1, check mount direction and camera-to-body geometry,
and confirm PX4 receives fresh vision poses. Do not bypass the flag check.

If the drone is actually flown off a table, account for the floor being lower
than the takeoff surface and verify the landing and clearance plan. The 2.0 m
mission climb is relative to the starting local pose, not a surveyed floor
height. A detected target is optional for readiness by default. Set
`require_detector_target_on_start:=true` only when a visible target is part of
the planned start sequence. Detector rate and 20 m range gates must still be
checked separately before flight.

### October 23 follow abort: ZED pose jump

The recorded controller transition was `FOLLOW -> ABORT: ZED tracking
unavailable: ZED pose jump`. Its ZED log showed an accepted pose shortly before
the first rejection, then repeated rejected poses and accepted pose rate falling
to zero. The old log does **not** record the jump distance, yaw change, or
physical cause. Target ranges in that run also hopped roughly 3–20 m and
detector IDs switched frequently, so target quality needs separate validation.

The ZED pose gate still rejects the same translation and rotation thresholds.
The new log line reports `translation_m`, `yaw_change_rad`, and
`frame_interval_s` for each throttled rejection. During flight, a single ZED
rejection clears mission and target-map history and commands zero velocity.
The controller resumes in `SEARCH` only after two accepted ZED poses within
0.5 s while PX4 local pose remains continuous and estimator flags stay valid.
If the rejection happened during `STAGE_HOVER`, it restarts the stable-hover
dwell instead.
Repeated rejection, PX4 pose discontinuity, or invalid estimator flags request
landing. The brief hold cannot correct a repeatable VIO frame shift.

After a supervised test, inspect the launch terminal or latest ROS logs for
`ZED pose jump`, `VISION_HOLD`, `local odometry discontinuity`, and the PX4
estimator state around the transition. Compare accepted ZED pose and PX4 local
pose timestamps, frame IDs, translation, and yaw during representative ownship
motion. Inspect the PX4 ULog for estimator resets and external-vision fusion.
Do not relax the pose gate to make a repeatable jump pass. Before another follow
flight, require stable VIO while moving the airframe and repeat the 10 Hz
detected-track and 20 m range checks in the runbook.

The target map now requires three coherent detected positions over at least
0.2 s before publishing a new target for pursuit. It limits association to
2 m/s target motion plus 0.5 m measurement allowance, even when the detector
reuses an ID. A 3–20 m depth hop therefore starts an unconfirmed candidate
instead of steering the current track. Check raw and target-map world tracks
side by side; if raw positions still jump, inspect depth validity, camera
calibration, and image/pose timestamps before flight.

The combined process runs one blank-frame inference with `best.engine` before
opening the ZED. This allocates the TensorRT execution context while camera
depth and VIO are idle and rejects missing OBB output early. A blank warmup
cannot prove that a real drone is detected; use a known target afterward.
The startup terminal prints `milestone4 startup:` markers before detector
imports, model load, engine warmup, ZED open, and ROS spin. It also syncs the
markers with `OC1` and `VDD_IN` readings in
`~/.ros/log/milestone4_startup_trace.log`.

The tracker startup log must show `best.engine`, `model_task=obb`, and CUDA
device `0`. A `.pt` or `.onnx` path, a missing engine, or
`tracker_inference_device:=cpu` makes the ZED-local launch fail. If the engine
cannot deserialize or produce OBB output, the tracker exits and readiness
cannot pass. Place a target drone in view for the separate known-target
validation; default `milestone4_check` can succeed in a quiet scene because
detector liveness, rather than a detection, is its start gate. A quiet scene
is not an engine validation. If this launch resets the Jetson, stop and measure the
shared power system as described below.

If readiness reports `ZED tracking unavailable: missing`, check the launch
terminal for `[zed_local_combined_node]` and its traceback. A startup failure
means the ZED publisher never began, even though the MAVROS and readiness
service processes may still be running. The October 19 failure
`InvalidParameterTypeException: inference_device ... INTEGER ... STRING` was
caused by ROS interpreting CUDA device `0` as a number; it is fixed by the
typed launch parameter. Rebuild `drone_vision_pkg` and `drone_bringup`, stop
the old launch with Ctrl-C, source the rebuilt workspace, and start the full
mission launch again. A running launch will not pick up rebuilt code.

After the power preflight was added, a launch stopped at
`Jetson VDD_IN 4656 mV is below 4850 mV`. In that case the combined node exits
before opening the ZED: `zed/tracking_status` has zero publishers,
`zed/pose_cov` and `perception/status` have no data, and `milestone4_check`
reports ZED tracking `missing`. MAVROS and the mission readiness service can
continue running. A MAVROS `local_position/pose` message does not establish
ZED vision fusion; check the estimator's horizontal position and velocity
flags. Repair the module 5 V rail under load before relaunching. Do not lower
the startup voltage threshold to make a failing power path appear ready.

After rebuilding, stop and restart any launch that still shows separate
`zed_vio_node` and `realsense_tracker_node` processes. A running Python process
does not load the new combined code. In the combined launch, a missing
`zed/color/image_rect` publisher is expected; check `zed/tracking_status`,
`zed/pose_cov`, `perception/status`, and target tracks instead.

The launch still passes `color_topic`, `depth_topic`, and `camera_info_topic`
parameters for the separate ROS-image diagnostic mode. In the full ZED-local
mission, `source_mode=embedded_zed` ignores those image topics and receives
color and depth through an in-process callback. An absent ROS image publisher
cannot by itself explain a tracker wait in this mode.

## If the tracker waits for camera frames

The combined tracker does subscribe to
`/cdrone/cdrone3/zed/tracking_status`. It waits for five continuous seconds of
fresh `OK` messages before inference. The ZED node sends an in-process image
and depth frame only after its pose gate accepts a fresh ZED pose. Check the
actual log message and health topics while the launch is still running:

```bash
pgrep -af zed_local_combined_node
ros2 topic info -v /cdrone/cdrone3/zed/tracking_status
ros2 topic echo --once /cdrone/cdrone3/zed/tracking_status
ros2 topic hz /cdrone/cdrone3/zed/pose_cov
ros2 topic hz /cdrone/cdrone3/perception/status
```

- `Waiting for stable ZED tracking ... status=missing`: no health message has
  reached the tracker. Check that the combined process is alive and that the
  status topic has a publisher. If the process died, inspect its traceback.
- `Waiting for stable ZED tracking ... status=<reason>`: the camera may have
  color frames, but the ZED pose has not passed the freshness/tracking gate.
  Read the reported reason and the `ZED capture timing` log; detector inference
  and in-process frame delivery are held until tracking is `OK`.
- `camera grab failed`, `camera image/depth retrieval failed`, or
  `camera frame timeout`: the SDK stopped returning usable frames. The node
  backs off failed grabs by 250 ms, rejects failed image/depth retrieval, and
  withdraws an old `OK` status after 0.75 s. Inspect USB errors and power at
  the ZED; a tracker restart cannot make an unstable USB link flight ready.
- `Waiting for the first in-process ZED frame`: the health gate passed, but no
  sampled frame has reached the tracker. Check `ZED timing` for
  `delivered_image_hz` and the capture timing log for a stalled `grab()` or
  image retrieval.
- `Waiting for the first color frame on /.../zed/color/image_rect`: this is the
  separate ROS-image tracker path, not the expected full-mission
  `embedded_zed` path. Check which launch and `source_mode` are running.

If the Jetson reboots, the ROS processes and their topics disappear. Recent
full-mission launches showed ZED USB video errors immediately before abrupt
reboots; a successful single raw or SDK color frame does not validate depth,
pose tracking, and TensorRT together. Use the staged diagnostics and shared
power checks below before another full-load run.

## Camera timing and readiness

The camera uses the Milestone 1 `PERFORMANCE` depth mode, caps SDK compute at
20 Hz, and samples detector frames at up to 15 Hz. Sensor depth may extend to 25 m so a target near the
20 m eligibility limit is not clipped by the sensor configuration. Check depth
quality at the intended range before flight; the 20 m setting is a detection
eligibility limit, not a promise of 20 m depth accuracy.

If `milestone4_check` reports a stale, repeated, or future ZED frame, the
camera's timestamp has failed the 0.3 s pose-age gate and PX4 cannot fuse a
fresh vision pose. An earlier two-process full launch on October 17 received
color images about 1.2 s after their timestamps and no detector inference ran
because ZED never reached `OK`. Rebuild and restart to use the combined
process, which checks and publishes pose before retrieving color/depth and
passes frames in memory. Its throttled `ZED capture timing` log reports `cycle_ms`, `grab_ms`,
`image_age_after_grab_ms`, `pose_age_at_gate_ms`, `get_position_ms`, and
`frame_delivery_ms`. The periodic `ZED timing` log reports capture, accepted
pose, and delivered image rates; `Detector timing` reports inference time,
track rate, frame age, detections, and world tracks. If age is already excessive after `grab()`, investigate
SDK processing, USB, and machine load. If image retrieval dominates, reduce
that cost without relaxing the pose-age gate. A repeated timestamp means the
SDK pose timestamp is not advancing; a future timestamp points to clock
alignment. Require sustained `OK`, live `zed/pose_cov`, valid PX4 horizontal
position and velocity, and a successful readiness check before flight.

## GPU inference and power

October 17 and October 20 full launches ended with abrupt **Jetson reboots**,
without normal ROS shutdown records. An October 20 boot logged
`uvcvideo 2-1:1.1 Non-zero status (-71)` near its reset, then enumerated the
ZED again at the next boot. The later October 20 launch started at 22:29:56
UTC, scheduled the combined detector/ZED process 15 s later, and reset without
any persisted process traceback or kernel fault near the reset. Its ROS log
files are empty. The timing implicates combined-process startup, but cannot
identify whether detector import/warmup, camera open, or a power transient was
the last operation. An earlier reset followed TensorRT execution-context
allocation shortly after the first ZED color frame. The ONNX Runtime GPU
discovery warning also appears in passing tests; MAVLink command 512 is a
denied message request, not a reboot request. The launch does not send a PX4
reboot command, and `enable_flight:=false` sends no arm or flight setpoints.
The Jetson and FCU share a battery. On an October 20 Orin Nano Super boot,
`soctherm_oc/oc1_event_cnt` exceeded 85,000 and increased even without the
Milestone 4 mission running. NVIDIA defines `OC1` for this board as an
undervoltage event near
4.5 V on the module input. Idle `VDD_IN` samples were about 4.8 V; a 5 s idle
sample showed no additional `OC1` events. This is strong evidence of input
voltage dips, but it does not prove which dip caused a particular reboot.
The later pasted launch trace narrows the failure: `OC1` rose from 166,235 to
174,838, and `VDD_IN` measured 4.568–4.664 V during Python imports and model
load. The last saved action was TensorRT allocating its `best.engine` execution
context. The node never reached `detector engine warmed` or `opening ZED`:
the ZED camera was **not yet open** at this reset. The measured voltage is
below [NVIDIA's 4.75 V minimum recommended Orin Nano module input](https://forums.developer.nvidia.com/t/jetson-orin-nano-8gb-voltage-drop-on-sys-5v-under-load-on-custom-carrier-board/366661). The
combined GPU startup likely exposed an already weak 5 V rail; a kernel/driver
failure cannot be excluded without a scope trace or pstore record.

An October 23 isolation test adds stronger evidence. The standalone
[`zed_engine_probe.py`](scripts/zed_engine_probe.py) ran without ROS, MAVROS,
VIO, or recording:

- ZED 2i `HD720@30`, `PERFORMANCE` depth: 303 unique synchronized color/depth
  frames in 15.196 s (**19.94 Hz**), with a 0.061 s maximum frame gap.
  During that camera-only run, module `VDD_IN` was sampled as low as 4.728 V
  and `OC1` rose from 11,755 to 12,426.
- `best.engine` on a saved ZED frame: loaded and returned one valid OBB result
  object with zero boxes for a scene containing no drone. The Jetson then
  reset during repeated inference. The last synced event was
  `first_inference_completed` at 4.824 V, after `OC1` rose by 99.
- `best.pt` on the same saved frame using CPU: completed four repeated
  inferences without a reset, at **0.351 Hz** (2.84 s median inference). It
  returned zero boxes for the same no-drone scene. `VDD_IN` was about
  4.752–4.760 V during inference and `OC1` rose from 15,395 to 15,673.
- `best.pt` on CUDA: loaded, then the Jetson reset during its **first**
  inference. The last synced event is `first_inference_started`. No camera or
  TensorRT engine was open in this run.

Both GPU model paths can trigger the reset, so a corrupted `best.engine` is
unlikely to be the sole cause. The engine's successful first inference also
rules out a completely unreadable artifact. Basic CUDA discovery and a
1,024-element GPU arithmetic check completed without a reset or new `OC1`
events; GPU model inference is the triggering load. The installed stack
reports JetPack 6.2.1, Jetson Linux 36.4.3, PyTorch CUDA
12.6, and TensorRT 10.3. At idle, CPU/GPU thermal sensors were about 48 °C,
4.2 GiB memory was available, and no OOM or kernel panic entry was readable
from the previous journal. This does **not** prove the exact reset mechanism:
`/sys/fs/pstore` needs root access, and a software voltage sample can miss a
brief dip. The rising `OC1` count makes the shared power path the first item
to repair. NVIDIA documents `OC1` as the Orin Nano module's undervoltage
event, near 4.5 V, even when a slower `VDD_IN` sample reads higher.

Repair the power path before another GPU inference or flight run. With the
airframe secured and motors disconnected, check the Jetson input regulator,
its current and transient rating, connector, and cable voltage drop. Measure
the module 5 V rail during GPU startup with a scope or fast logger; also check
the ZED USB supply and FCU rail. A shared battery can feed separate properly
rated regulated branches. A temporary `sudo nvpmodel -m 0` selects the
available 10 W mode for *diagnosis* on this board; it reduces load but cannot
repair an unstable supply or establish the required detector rate. Restore
the intended 25 W mode with `sudo nvpmodel -m 1` only after the rail is fixed.
If a known-good supply still resets under both GPU paths, collect the root-only
`/sys/fs/pstore` record and serial-console output, then investigate the
CUDA/driver stack before replacing or rebuilding the engine.

The combined node now checks module `VDD_IN` and whether `OC1` rises before
heavy imports and again before TensorRT warmup. If the reading is below
4.85 V (a 0.10 V startup margin above the module minimum) or `OC1` increases
during the two-second sample, or either sensor is unreadable, startup stops with a
clear error and the mission readiness check remains unavailable. A passing
software sample cannot guarantee that the rail will hold during GPU load.
Measure the supply under the combined load; a GPU/USB driver fault is still
possible. A shared battery is acceptable only if its discharge path and each
regulated rail stay within the devices' input specifications under peak load.
Do not attempt flight until the reset is resolved.

The full mission selects ZED SDK `DEPTH_MODE.PERFORMANCE` and feature-based
`POSITIONAL_TRACKING_MODE.GEN_3`; it does not enable the SDK's neural depth or
object-detection module. ZED depth is computed on the Jetson, not in a neural
accelerator inside the camera (see the
[ZED SDK depth modes](https://docs.stereolabs.com/docs/development/zed-sdk/modules/depth-sensing/depth-modes/)
and [host CUDA requirement](https://support.stereolabs.com/articles/7166665108-can-i-use-the-zed-without-cuda)).
`best.engine` is a separate TensorRT detector and does use the Jetson GPU.
The tracker's `zed_depth_mode: NEURAL` YAML default
belongs to its separate `zed_sdk` camera path; the full mission uses
`source_mode=embedded_zed`, opens one camera in the VIO node, and now reports
`PERFORMANCE` in both node parameters. `PERFORMANCE` depth, VIO, and TensorRT
still draw substantial power together.

The `Demo_ASU_News` Milestone 4 branch was configured with the same default
50 MiB TensorRT engine and has tracking recordings lasting more than two
minutes. That branch opened the ZED directly for depth and detection while
OptiTrack supplied
ownship pose. The no-mocap route runs ZED positional tracking for PX4 at
the same time as depth and detector inference. The earlier two-process route
also copied full images through ROS; the combined process removes that cost.
The additional simultaneous camera/GPU work is the relevant change in load. The
old engine warning remains a compatibility concern, but it is not proof that
the engine alone caused this reboot.

First identify which device restarted:

```bash
journalctl --list-boots --no-pager | tail
journalctl -b -1 -e --no-pager
rg 'uvcvideo|usb 2-1|thermal|Out of memory' /var/log/kern.log | tail -50
lsusb -t
tail -30 ~/.ros/log/milestone4_startup_trace.log
cat /sys/devices/platform/soctherm-oc-event/hwmon/hwmon*/oc{1,2,3}_event_cnt
sudo ls -l /sys/fs/pstore
```

`OC1` is undervoltage, `OC2` is average input-power limiting, and `OC3` is
instantaneous input-power limiting on this module. Record counters immediately
before and after each diagnostic stage; compare changes, not the absolute
count. Find the live module input reading by looking for `VDD_IN` under
`/sys/class/hwmon/hwmon*/in*_label` and reading the matching `in*_input`
file. On the sampled October 20 boot it was `hwmon2/in1_input` in millivolts, but hwmon numbers
can change. A slow software sample cannot exclude a short dip. NVIDIA's
[Orin Nano power guide](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html)
documents the limits and says not to modify the INA3221 sysfs thresholds.
If `/sys/fs/pstore` contains a new `ramoops` record, save it before another
test; it can distinguish a kernel panic from a power interruption. The
current user cannot read that directory without root access.

If the Jetson boot ID changes, measure at three points during camera and GPU
startup: the battery terminals, the Jetson input connector, and the FCU power
input. Use a scope or power logger with enough time resolution to catch a brief
dip; an idle voltage reading or a slow multimeter can miss it. Check each
regulator's output and current rating, its wiring and connector voltage drop,
and the ZED USB 3 power path. Confirm the ZED and Jetson are not drawing from
an undersized FCU USB rail. The Jetson currently reports the 25 W `nvpmodel`
mode, which does not include the FCU or ZED draw. A battery can be shared
while the Jetson and FCU use separate adequately rated regulators.
On the NVIDIA developer kit, the DC jack feeds an onboard 5 V regulator for
module `VDD_IN`; a healthy battery or DC jack reading alone cannot establish
that this internal rail stays above 4.75 V during TensorRT startup. Use the
module voltage sensor and a suitable physical probe of the 5 V rail.

Check PX4's `ATTITUDE.time_boot_ms` in QGroundControl's MAVLink Inspector
across the event. If its uptime resets, the FCU itself rebooted. If it does
not, MAVROS merely lost its connection when the Jetson rebooted. The FCU
restart is not established from the MAVROS log alone.

If the full launch resets the Jetson again, use one terminal per *run* to
isolate the failing load. Each launch stage starts separate OS processes with
separate `[node-name]` log prefixes and one Ctrl-C stops them all. With motors
disconnected, first run MAVROS only:

```bash
ros2 launch drone_bringup milestone4_diagnostics.launch.py stage:=mavros
```

On October 20 this MAVROS-only stage connected to PX4 and shut down cleanly
after a 12 s bench run. That confirms the lightweight stage can start; it does
not validate the shared rails under ZED and TensorRT load.

Stop it, then run MAVROS plus ZED VIO and published diagnostic images:

```bash
ros2 launch drone_bringup milestone4_diagnostics.launch.py stage:=zed \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml
```

Once the shared power rails are stable, stop it and run all three processes:

```bash
ros2 launch drone_bringup milestone4_diagnostics.launch.py stage:=all \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml
```

The `all` stage starts MAVROS, ZED VIO, and the ROS-image tracker with
`best.engine` in one terminal. By default it starts ZED after 15 s and the
detector after 30 s, so their initialization does not overlap MAVROS startup.
The `zed` stage also waits 15 s before opening the camera. The tracker does
not open the camera. This
diagnostic path publishes full images and may be slower than the full mission's
direct in-memory handoff. It does not start the mission controller, so
`/cdrone/cdrone3/demo/milestone4_check` is unavailable here. Stop diagnostics
before the full mission launch; only the full launch creates that service.
The diagnostic ZED stage uses the same 20 Hz SDK compute cap. If `stage:=zed`
alone drops the USB link or reboots the Jetson, troubleshoot camera power,
cable, connector, and USB 3 signal integrity before adding the detector. If
only `stage:=all` fails, capture the last startup marker plus Jetson power,
temperature, and USB logs during the added TensorRT load.

For a minimal staged check without ROS, MAVROS, VIO, or flight commands, use
one process per stage. The script writes a synced `*_events.jsonl` marker
before camera open, model load, and first inference. Its `sample_frame.npz`
contains aligned color and metric depth. Run the CPU comparison only as a
diagnostic; `.pt` remains excluded from the flight mission:

```bash
python3 scripts/zed_engine_probe.py --mode camera --duration-s 15 \
  --output-dir output_dump/zed_engine_probe
python3 scripts/zed_engine_probe.py --mode engine --model models/best.pt \
  --device cpu --duration-s 10 --output-dir output_dump/zed_engine_probe
```

After the supply is stable under load, compare GPU inference on that **saved**
frame before combining it with live capture:

```bash
python3 scripts/zed_engine_probe.py --mode engine --model models/best.pt \
  --device 0 --duration-s 10 --output-dir output_dump/zed_engine_probe
python3 scripts/zed_engine_probe.py --mode engine --model models/best.engine \
  --device 0 --duration-s 10 --output-dir output_dump/zed_engine_probe
python3 scripts/zed_engine_probe.py --mode combined --model models/best.engine \
  --device 0 --duration-s 30 --known-range-m 5.0 \
  --output-dir output_dump/zed_engine_probe
```

For the combined run, place a target drone at a measured range and adjust
`--known-range-m`. Read `combined_engine_0_summary.json` and
`combined_engine_0_detections.csv`. Coordinates are in camera optical axes
and calibrated body FLU axes; local/world coordinates and PX4 vision fusion
still require the full mission. A zero-box result on an empty bench scene
does not validate target detection. Check `valid_3d_frames.hz`, its maximum
gap, `valid_depth_fraction`, and `median_range_error_m`; camera FPS alone is
not the target update rate. A hard reset with the last marker at
`first_inference_started` implicates GPU inference but does not by itself
distinguish power delivery from a GPU/driver fault.

## ROS bench-check interpretation

A one-time `tracking_status: OK` confirms one healthy frame. For readiness,
ZED status must update within 1 s, vision pose and PX4 local pose within 0.5 s,
and perception within 3 s. If `zed/pose_cov` is around 0.6 Hz or
`perception/status` around 2 Hz, inspect the periodic stage timing logs. The
October 23 isolated `best.pt` CPU diagnostic measured 2.84 s per inference
on a saved HD720 frame. That does not establish the rate
of `best.engine` or the combined mission. The selected engine must be measured
with a visible drone at the intended ranges. If the engine cannot deserialize,
return OBB detections, or sustain the [runbook flight gates](milestone4_runbook.md#no-mocap-full-mission-launch-flight-disabled), stop; do not switch
models automatically. [NVIDIA notes that TensorRT engines on JetPack are tied
to their device and runtime](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/engine-compatibility.html).
