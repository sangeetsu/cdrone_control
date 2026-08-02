# Manual ZED 2i / Mocap / IMU / Optical-Flow Data Collection Runbook

This runbook records a non-flying, hand-moved `cdrone4` / `RigidBody4`
dataset. It logs mocap, MAVROS IMU, PX4Flow, ZED 2i SDK VIO, and right-camera
images without arming the drone or starting any autonomous mission.

The drone must remain unarmed. Do not call the minjerk start service in this
run.

## What This Records

Callback-rate CSVs are written as fast as ROS receives each stream:

- `mocap_raw.csv`
- `imu_raw.csv`
- `optical_flow_raw.csv`
- `zed_vio_raw.csv`
- `state_raw.csv`
- `px4_local_pose_raw.csv`
- `px4_vision_pose_raw.csv`

The synchronized comparison file is sampled by the logger:

```text
flight_logs/of_compare/${RUN_ID}/of_mocap_compare.csv
```

Right-camera images are saved at 2 Hz:

```text
flight_logs/of_compare/${RUN_ID}/images_dataset/
flight_logs/of_compare/${RUN_ID}/images_dataset/images.csv
```

ZED VIO is log-only. It is aligned to mocap by the logger with the first
available translation + yaw pair and is never fed to PX4 or to the live
IMU+optical-flow estimator.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Preflight Checks

Start MAVLink routing if it is not already running:

```bash
/home/jetson/.local/bin/start_mavlink_router.sh
```

In another terminal:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

ros2 topic echo --once /cdrone/cdrone4/mavros/state
ros2 topic echo --once /cdrone/cdrone4/vrpn_mocap/RigidBody4/pose
timeout 10 ros2 topic hz /cdrone/cdrone4/mavros/imu/data
scripts/check_px4flow_topics.sh \
  --mavros-namespace /cdrone/cdrone4/mavros \
  --duration 10

lsusb | rg -i 'stereolabs|zed'
lsusb -t | rg '5000M|STEREOLABS|ZED|uvcvideo'
python3 -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
```

Expected:

- MAVROS is connected and `armed: false`
- mocap is live for `/cdrone/cdrone4/vrpn_mocap/RigidBody4/pose`
- IMU and PX4Flow have stable rates
- the ZED 2i is on USB3 and the SDK import prints a version

## Launch Logging Only

Do not call `/cdrone/cdrone4/demo/minjerk_waypoints_start`.

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

RUN_ID="manual_zed_vio_cdrone4_rb4_$(date -u +%Y%m%dT%H%M%SZ)"

scripts/run_minjerk_of_compare.sh \
  --output-dir /home/jetson/cdrone_control/flight_logs/of_compare_manual \
  --run-id "${RUN_ID}" \
  --request-sensor-stream-rates \
  --imu-stream-rate-hz 250 \
  --flow-stream-rate-hz 70 \
  --capture-images \
  --image-view right \
  --image-rate-hz 2 \
  --image-format jpg \
  --capture-zed-vio \
  --zed-camera-fps 60 \
  --zed-depth-mode PERFORMANCE \
  -- \
  drone_id:=cdrone4 \
  drone_namespace:=/cdrone/cdrone4 \
  mavros_namespace:=/cdrone/cdrone4/mavros \
  mocap_namespace:=/cdrone/cdrone4/vrpn_mocap \
  rigid_body_name:=RigidBody4 \
  source_pose_topic:=/cdrone/cdrone4/vrpn_mocap/RigidBody4/pose \
  mocap_pose_topic:=/cdrone/cdrone4/external_pose/input_pose \
  frame_rpy_rad:="[0.0, 0.0, 3.141592653589793]" \
  mission_state_topic:=/cdrone/cdrone4/demo/minjerk_waypoints_state \
  mavros_state_topic:=/cdrone/cdrone4/mavros/state \
  px4_local_pose_topic:=/cdrone/cdrone4/mavros/local_position/pose \
  px4_vision_pose_topic:=/cdrone/cdrone4/mavros/vision_pose/pose \
  flow_rad_topic:=/cdrone/cdrone4/mavros/px4flow/raw/optical_flow_rad \
  flow_range_topic:=/cdrone/cdrone4/mavros/px4flow/ground_distance \
  imu_topic:=/cdrone/cdrone4/mavros/imu/data \
  fused_pose_topic:=/cdrone/cdrone4/of_compare/fused_pose \
  imu_only_pose_topic:=/cdrone/cdrone4/of_compare/imu_only_pose \
  zed_vio_pose_topic:=/cdrone/cdrone4/zed_vio/pose \
  zed_vio_status_topic:=/cdrone/cdrone4/zed_vio/status \
  zed_vio_frame_id:=zed_vio_world \
  logger_publish_rate_hz:=50 \
  use_speed_profile:=false \
  restore_speed_profile_on_exit:=false \
  quality_min:=20 \
  range_min_m:=0.3 \
  range_max_m:=4.5 \
  fusion_enabled:=true \
  imu_only_enabled:=true \
  fusion_max_sensor_age_s:=0.15 \
  fusion_max_flow_gap_s:=0.25
```

After the launch is stable, hand-move the drone through the motion you want to
measure. Include pauses and sweeps that make moving-object effects visible in
the right-camera images.

## Live Monitoring

Use another terminal while the launch keeps running:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

ros2 param get /zed_image_capture_node image_view
ros2 param get /zed_image_capture_node vio_enabled
ros2 topic echo --once /cdrone/cdrone4/zed_vio/status
timeout 10 ros2 topic hz /cdrone/cdrone4/zed_vio/pose
timeout 10 ros2 topic hz /cdrone/cdrone4/mavros/imu/data
scripts/check_px4flow_topics.sh \
  --mavros-namespace /cdrone/cdrone4/mavros \
  --no-request-stream \
  --duration 10

tail -n 3 "flight_logs/of_compare/${RUN_ID}/mocap_raw.csv"
tail -n 3 "flight_logs/of_compare/${RUN_ID}/imu_raw.csv"
tail -n 3 "flight_logs/of_compare/${RUN_ID}/optical_flow_raw.csv"
tail -n 3 "flight_logs/of_compare/${RUN_ID}/zed_vio_raw.csv"
tail -n 3 "flight_logs/of_compare/${RUN_ID}/of_mocap_compare.csv"
find "flight_logs/of_compare/${RUN_ID}/images_dataset" \
  -maxdepth 1 -name '*.jpg' | wc -l
sed -n '1,5p' "flight_logs/of_compare/${RUN_ID}/images_dataset/images.csv"
```

Expected:

- `/zed_image_capture_node` reports `image_view: right`
- `/cdrone/cdrone4/zed_vio/status` reaches `OK` during normal tracking
- `zed_vio_raw.csv`, `mocap_raw.csv`, `imu_raw.csv`, and
  `optical_flow_raw.csv` keep growing
- the image count rises at about 2 images per second
- `images.csv` records `image_view` as `right`

## Stop And Generate Stats

After the manual motion sequence, stop the launch with Ctrl-C. Then generate
plots and summaries:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

scripts/of_compare_stats.py \
  "flight_logs/of_compare/${RUN_ID}/of_mocap_compare.csv" \
  --plots
```

Important outputs:

```text
flight_logs/of_compare/${RUN_ID}/report/summary.md
flight_logs/of_compare/${RUN_ID}/report/summary.json
flight_logs/of_compare/${RUN_ID}/report/zed_vio_timeseries.csv
flight_logs/of_compare/${RUN_ID}/report/plots/trajectory_xy.png
flight_logs/of_compare/${RUN_ID}/report/plots/errors_over_time.png
flight_logs/of_compare/${RUN_ID}/report/plots/zed_vio_vs_mocap_over_time.png
flight_logs/of_compare/${RUN_ID}/report/plots/flow_health_over_time.png
flight_logs/of_compare/${RUN_ID}/report/plots/px4_imu_health_over_time.png
flight_logs/of_compare/${RUN_ID}/report/plots/mocap_health_over_time.png
```

Read `summary.md` first. It reports:

- ZED VIO vs mocap XY RMSE, XY P95, Z RMSE, 3D RMSE, and drift
- IMU+OF vs mocap
- OF-only vs mocap
- IMU-only drift baseline
- raw mocap, IMU, ZED VIO, and PX4 pose rate/age diagnostics
- flow delivery coverage and flow-vs-mocap velocity correlation

## Notes On Moving Objects

Moving objects mainly affect camera-based estimation and optical flow. This run
does not remove those effects online. Instead, it records:

- synchronized right-camera images for visual review
- ZED VIO residuals against mocap
- IMU+OF residuals against mocap
- OF-only residuals against mocap
- flow quality, range, timing coverage, and correlation diagnostics

Use the VIO-vs-mocap and IMU+OF-vs-mocap plots together with the image timeline
to identify intervals where moving objects likely degraded visual estimates.

## Prompt Refinement

A tighter prompt for future runs:

```text
Create a non-flying manual data-collection runbook for cdrone4/RigidBody4 that logs mocap, MAVROS IMU, PX4Flow, ZED 2i SDK VIO, and right-camera images at 2 Hz. Keep VIO log-only, compare every estimator against mocap after the run, generate plots and summary stats, and do not arm or start any autonomous mission.
```
