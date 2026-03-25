# Isaac ROS Release 3.2 Path

This repo now carries a pinned Isaac ROS bootstrap path for the D455 + VSLAM fallback.

## Why This Exists

The native host `realsense2_camera` path on this Jetson still fails even after the D455 was moved back onto USB 3.

The active fallback is the NVIDIA-tested container path:

- Isaac ROS `release-3.2`
- RealSense image key `ros2_humble.realsense`
- D455 support through the Isaac-pinned RealSense stack

## Workspace Location

By default the local Isaac workspace lives at:

```text
/home/jetson/cdrone_control/external/isaac_ros_release32
```

That path is intentionally gitignored so we do not vendor a large external workspace into this repo.

## Bootstrap

```bash
cd /home/jetson/cdrone_control
./scripts/setup_isaac_ros_release32.sh
```

## Start The RealSense Dev Container

```bash
cd /home/jetson/cdrone_control
./scripts/run_isaac_realsense_dev.sh
```

That wrapper is for an interactive terminal. In a headless session, use:

```bash
cd /home/jetson/cdrone_control
./scripts/start_isaac_realsense_container.sh
```

The headless launcher intentionally stays on the classic `--runtime nvidia` path. On this Jetson, the `release-3.2` aarch64 launcher path reached:

```text
failed to inject CDI devices: unresolvable CDI devices nvidia.com/gpu=all
```

until the launch was simplified away from CDI-style visible devices.

Inside the container, the usual next checks are:

```bash
realsense-viewer
```

or for a headless check:

```bash
rs-enumerate-devices
```

To run the repo's D455 profile inside the running container:

```bash
cd /home/jetson/cdrone_control
./scripts/launch_isaac_realsense_d455_in_container.sh
```

Then install the quickstart packages if needed:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-isaac-ros-examples \
  ros-humble-isaac-ros-realsense \
  ros-humble-isaac-ros-visual-slam
```

And bring up the RealSense VSLAM quickstart:

```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
  launch_fragments:=realsense_stereo_rect,visual_slam \
  interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json \
  base_frame:=camera_link \
  camera_optical_frames:="['camera_infra1_optical_frame', 'camera_infra2_optical_frame']"
```

## Important Version Guardrail

The official `release-3.2` RealSense setup page says the expected versions are:

- firmware `5.13.0.50`
- `librealsense` `2.55.1`
- `realsense-ros` `4.51.1-isaac`

Use the pinned `release-3.2` container path instead of mixing newer host-native RealSense packages into the Isaac flow.

## Current Machine Result

As of 2026-03-20 on this Jetson:

- the image build succeeded and produced `isaac_ros_dev-aarch64:latest`
- `rs-enumerate-devices` inside the container saw the connected D455 cleanly
- the container reported firmware `5.16.0.1` and also reported `5.16.0.1` as the recommended firmware
- `realsense2_camera` in the container started, found the D455, opened stereo IR and IMU profiles, and reached `RealSense Node Is Up!`

That is already better than the native host path, which failed earlier with `bad optional access` and did not bring the node up cleanly.

But the camera is still not usable for VIO yet on this machine:

- the launch log floods with `control_transfer returned error`
- the log also reports XU-related failures such as `get_xu(ctrl=1)` and `get_xu(id=7)`
- direct ROS subscriber checks did not receive live IR or IMU samples even after the node advertised the expected topics
- disabling `depth_module.global_time_enabled`, `motion_module.global_time_enabled`, and `depth_module.thermal_compensation` did not clear the issue

So the Isaac ROS path removed the obvious version-mismatch blocker, but it did not fully clear the remaining low-level D455 control/stream issue on this Jetson.

## Post-Reboot Retest

As of 2026-03-21, a clean Jetson reboot with the D455 unplugged did not change the failure mode:

- the D455 still came back on the SuperSpeed path at `5000M`
- `rs-enumerate-devices` inside the running Isaac container still saw the camera cleanly and reported firmware `5.16.0.1`
- the Isaac launch still reached `RealSense Node Is Up!`, advertised the expected `/d455/*` IR and IMU topics, and then immediately resumed the same `control_transfer returned error` spam
- a fresh subscriber check still received no live `/d455/gyro/sample` or `/d455/infra1/image_rect_raw` messages

The reboot narrowed the problem further because the host kernel now shows matching low-level errors outside ROS:

- repeated `uvcvideo ... Unknown video format ...` lines when the D455 enumerates
- repeated `Failed to query (GET_CUR) UVC control ...: -71` lines during direct V4L2 access
- repeated `Failed to query (130) UVC probe control : -71` lines during a raw stream test
- a direct `v4l2-ctl` stream attempt on `/dev/video5` reached `VIDIOC_STREAMON` but produced a zero-byte output file

That means the current blocker is no longer just "the ROS driver inside Docker". The Jetson is failing at the lower UVC/XU control layer too.

The next high-value experiment is to align the camera firmware with the Isaac ROS `release-3.2` guardrail and retry:

- current camera firmware on this machine: `5.16.0.1`
- official `release-3.2` RealSense setup expectation: `5.13.0.50`

If the downgrade does not clear the UVC/XU failures, the next conclusion is that this Jetson/carrier/kernel path is not presently a reliable D455 host for VIO and we should verify the camera on a known-good x86 or alternate Jetson path before spending more time on PX4 integration.

## Firmware-Downgrade Retest

As of 2026-03-24, the D455 was re-checked after being downgraded to firmware `5.13.0.50`.

The important version alignment now looks good on the active Isaac path:

- D455 firmware now reports `5.13.0.50`
- the camera still negotiates on USB 3 / `5000M`
- `isaac_ros_common` is still on branch `release-3.2`
- the Isaac config still uses `CONFIG_IMAGE_KEY=ros2_humble.realsense`
- `realsense2_camera` inside the container reports version `4.51.1`
- `librealsense2.so` inside the container reports version `2.55.1`

That means the active container path is now aligned with NVIDIA's documented `release-3.2` RealSense guardrail.

The runtime result also improved materially:

- `realsense2_camera` still emits a small burst of startup `control_transfer returned error` warnings, but the earlier `get_xu(ctrl=1)` failures no longer appeared
- live `/d455/gyro/sample` messages now arrive at about `200 Hz`
- live `/d455/imu` messages now arrive
- live `/d455/infra1/image_rect_raw` frames now arrive

One remaining D455-on-Jetson quirk showed up during the retest:

- immediately after launch, `ros2 topic hz /d455/infra1/image_rect_raw` initially measured about `15 Hz` even though the node opened `640x360@90`
- NVIDIA's troubleshooting guide documents this exact D455 issue and recommends re-applying `depth_module.enable_auto_exposure true` at runtime
- after doing that on this machine, the measured IR topic rate rose to about `89.9 Hz`

The launch helper in this repo now applies that runtime fix automatically after the node starts.
