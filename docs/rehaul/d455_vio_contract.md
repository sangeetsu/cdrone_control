# D455 VIO Contract

This is the active D455 sensor contract for the rehauled repo.

## Goal

Produce a deterministic RealSense stream that is useful for a later VIO/VSLAM estimator and easy to validate before PX4 is involved.

## Launch Entry Point

```bash
ros2 launch drone_bringup realsense_d455.launch.py
```

Default profile file:

```text
ros2/src/drone_bringup/config/realsense_d455_vio.yaml
```

## Default Sensor Profile

- color disabled
- depth disabled
- infra1 enabled
- infra2 enabled
- gyro enabled
- accel enabled
- IMU unification enabled with `unite_imu_method=2`
- IR profile: `640x360x90`
- gyro: `200 Hz`
- accel: `200 Hz`
- emitter disabled
- TF publishing enabled

## Expected Topics

With the default namespace and camera name, the expected root is:

```text
/camera/d455
```

Key topics:

- `/camera/d455/infra1/image_rect_raw`
- `/camera/d455/infra2/image_rect_raw`
- `/camera/d455/gyro/sample`
- `/camera/d455/accel/sample`
- `/camera/d455/imu`

## Expected Rates

- infra1 and infra2: about `90 Hz`
- gyro: about `200 Hz`
- accel: about `200 Hz`
- unified IMU: should advance steadily and not stall

## Frame Expectations

- sensor-native RealSense frames should stay sensor-native at this layer
- the VIO bridge, not the camera driver, should own camera-to-body extrinsics
- PX4-facing ENU/NED handling should happen in one bridge node, not ad hoc across nodes

## Validation

After launch:

```bash
./scripts/check_d455_topics.sh
```

If a topic is missing or rates are unstable, fix the camera/driver side before wiring in VIO or PX4.

For the Isaac ROS container path on Jetson, there is also a D455-specific known issue where the IR stream may start around `15 Hz` even though the node opens a `90 Hz` profile.

If that happens, re-apply:

```bash
ros2 param set /d455/d455 depth_module.enable_auto_exposure true
```

On this repo's March 24, 2026 retest, that runtime parameter restored the measured IR topic rate from about `15 Hz` to about `90 Hz`.

## USB Link Requirement

The D455 needs a healthy USB 3 link for the VIO path to be useful.

If `lsusb -t` shows the camera at `480M`, it is only running at USB 2 speed. In that state you can see symptoms like:

- `No HID info provided, IMU is disabled`
- repeated `bad optional access` errors from `realsense2_camera`
- no IR or IMU topics appearing under `/camera/d455`

If that happens:

- move the camera to a USB 3 capable port
- use a known-good USB 3 cable
- replug the camera and re-check `lsusb -t`
