# Indoor VIO Bringup for PX4 + MAVROS + ROS 2

This repo now contains a first in-repo stereo VIO path for indoor PX4 flight without GPS:

1. `drone_vision_pkg/stereo_vio_node`
2. `drone_control_pkg/px4_vision_bridge_node`
3. `drone_bringup/launch/indoor_vio.launch.py`

The estimator uses:

- the two IMX219 CSI cameras for stereo depth
- the ARK V6 flight controller IMU through `/mavros/imu/data` for attitude
- MAVROS `/mavros/vision_pose/pose` as the PX4 external-vision input

## What This Implements

- A stereo feature tracker that rectifies the left/right images from the CSI cameras.
- Stereo triangulation for metric 3D points.
- Frame-to-frame pose updates from `solvePnPRansac`.
- Attitude from the FCU IMU, position from stereo vision.
- A freshness-gated bridge into `/mavros/vision_pose/pose`.

This is a practical first-pass VIO integration inside this repository. It is suitable for bench validation and iterative tuning. It is not yet a claim of flight-proven SLAM quality.

## Step 1: Put Calibration Inside This Repo

Copy your stereo calibration file to:

```bash
/home/jetson/cdrone_control/calibration/stereo_calibration.npz
```

If your calibration lives elsewhere today, either copy it into this repo or override `calibration_path` in:

- `ros2/src/drone_vision_pkg/config/stereo_vio.yaml`

## Step 2: Review Camera-to-Body Extrinsics

Edit:

- `ros2/src/drone_vision_pkg/config/stereo_vio.yaml`

Set these values correctly for the Jetson PAB + ARK V6 mount:

- `camera_body_translation_m`
- `camera_body_rpy_rad`

If the stereo rig is mounted square to the body, start with zeros and tune after bench validation.

## Step 3: Build the Workspace

From a shell where ROS 2 and `colcon` are available:

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_vision_pkg drone_control_pkg drone_behavior_pkg drone_bringup
source install/setup.bash
```

## Step 4: Launch Indoor VIO

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_bringup indoor_vio.launch.py
```

This starts:

- MAVROS
- `stereo_vio_node`
- `px4_vision_bridge_node`

## Step 5: Verify the Estimator Before Touching Flight Modes

Check that the local VIO outputs are alive:

```bash
ros2 topic hz /cdrone/vio/pose
ros2 topic echo /cdrone/vio/healthy --once
ros2 topic echo /cdrone/vio/pose --once
```

Check that PX4 is receiving the bridged pose:

```bash
ros2 topic hz /mavros/vision_pose/pose
ros2 topic echo /mavros/companion_process/status --once
```

Move the drone slowly by hand on the bench and confirm:

- `x/y/z` change smoothly
- attitude follows the FCU IMU
- the pose stream stays fresh

## Step 6: Configure PX4 for External Vision

In QGroundControl, set PX4 EKF parameters for external vision aiding. The exact parameter names can differ by PX4 release, so verify against your `PX4 1.16` setup before flight.

Typical goals are:

- enable external vision position fusion
- enable external vision yaw fusion if the IMU-attitude-aligned output is stable
- keep GPS disabled or not required for the indoor test profile

Then reboot PX4 and confirm local position is valid.

## Step 7: Bench Check Position Mode

With props off:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
```

Then try switching to `POSITION` only after:

- `/mavros/vision_pose/pose` is stable
- `/mavros/local_position/pose` is valid
- QGC shows a healthy local-position estimate

## Step 8: Only Then Move to Low-Risk Flight Testing

- First test should still be props-off.
- First powered test should be restrained or otherwise safety-controlled.
- First hover attempt should be low altitude with a clear manual takeover path.

## Files Added for This Flow

- `ros2/src/drone_vision_pkg/drone_vision_pkg/stereo_vio_node.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/px4_vision_bridge_node.py`
- `ros2/src/drone_bringup/launch/indoor_vio.launch.py`
- `ros2/src/drone_vision_pkg/config/stereo_vio.yaml`
- `ros2/src/drone_control_pkg/config/px4_vision_bridge.yaml`

## Current Limitations

- The estimator is feature-based stereo VO plus IMU attitude, not a full factor-graph SLAM backend.
- There is no loop closure or map persistence.
- Covariances are not yet estimated.
- The MAVROS `odometry` plugin is still denylisted; this first pass targets `/mavros/vision_pose/pose`.
- Final flightworthiness depends on your camera calibration, camera mounting, vibration isolation, IMU timing, and PX4 EKF tuning.
