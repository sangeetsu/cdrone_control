# Hackster Summary: GPS-Denied Drone With NVIDIA Jetson Orin Nano

Source:
- Hackster article: https://www.hackster.io/bandofpv/gps-denied-drone-with-nvidia-jetson-orin-nano-9f3417
- Author: Andrew Bernas
- Published: June 22, 2025

## Overview

This project describes a GPS-denied drone that uses:
- an NVIDIA Jetson Orin Nano as the companion computer
- an Intel RealSense D435i as the visual-inertial sensor
- PX4 as the flight stack
- MAVROS / MAVLink for companion-to-flight-controller communication
- NVIDIA Isaac ROS VSLAM for onboard localization and mapping

The core idea is to replace GPS with visual-inertial localization so the drone can estimate pose and fly autonomously indoors or in other GPS-denied environments.

The article states that the drone relies on:
- VIO / VSLAM from the camera and IMU
- the flight controller IMU
- EKF fusion on PX4

The motion-capture system shown in the demo is only used for ground-truth validation, not for flight.

## Hardware Used

The article lists:
- NVIDIA Jetson Orin Nano Developer Kit
- F450 quadcopter
- Intel RealSense D435i
- PX4 flight controller
- 9V 5A voltage regulator
- USB-to-UART converter

## Stated Software / Version Requirements

The article is explicit about several versions:
- PX4: `v1.15.4`
- JetPack: `6.2`
- Isaac ROS common branch: `release-3.2`
- RealSense firmware required by the Isaac ROS Docker image: `5.13.0.50`

Important note:
- The article is written around the `D435i`, not the `D455`.
- The article also warns that RealSense firmware compatibility matters for the Isaac ROS container.

## System Architecture

The flow described in the article is:

1. RealSense D435i provides image + IMU data.
2. Isaac ROS VSLAM runs on the Jetson Orin Nano.
3. VSLAM produces vision-based pose estimates.
4. MAVROS / MAVLink bridges those estimates to PX4.
5. PX4 EKF fuses external vision and controls the drone without GPS.
6. An offboard control node commands autonomous flight patterns.

## Jetson Orin Nano Setup

### Wiring

The article recommends connecting the flight controller through `TELEM2` using a USB-to-UART adapter rather than the Jetson GPIO serial pins. The reason given is that the GPIO link was unreliable or noisy during testing.

The recommended connections are:
- flight controller `TELEM2 TX` -> adapter `RXD`
- flight controller `TELEM2 RX` -> adapter `TXD`
- RealSense camera -> Jetson USB
- separate 7-20V supply to the Jetson through barrel jack or USB-C

### JetPack Verification and Performance Setup

The article verifies JetPack with:

```bash
cat /etc/nv_tegra_release
```

It expects a JetPack 6.2 release string.

Then it recommends enabling max performance:

```bash
sudo /usr/bin/jetson_clocks
sudo /usr/sbin/nvpmodel -m 2
```

It also adds the user to the Docker group:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

## SSD Setup

The article strongly recommends an NVMe SSD for:
- Docker image storage
- rosbag storage

The process described is:
- physically install the SSD
- confirm detection with `lspci`
- identify the block device with `lsblk`
- format it as `ext4`
- mount it at `/ssd`
- add it to `/etc/fstab`
- change ownership of `/ssd`

Then Docker is migrated from `/var/lib/docker` to `/ssd/docker` by:
- stopping Docker
- copying the Docker data to the SSD with `rsync`
- updating `/etc/docker/daemon.json`
- renaming `/var/lib/docker`
- restarting Docker

The article configures Docker with NVIDIA runtime support and sets:
- `"default-runtime": "nvidia"`
- `"data-root": "/ssd/docker"`

## Isaac ROS Setup

The article uses Isaac ROS as the VSLAM framework on top of ROS 2.

### Dependencies and Workspace

It installs:

```bash
sudo apt install git-lfs
git lfs install --skip-repo
sudo apt-get install -y curl jq tar
```

Then creates a workspace on the SSD:

```bash
mkdir -p /ssd/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=/ssd/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

### Cloning Isaac ROS and VSLAM-UAV

The article clones:

```bash
cd ${ISAAC_ROS_WS}/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common

cd ${ISAAC_ROS_WS}
git clone https://github.com/bandofpv/VSLAM-UAV.git
${ISAAC_ROS_WS}/VSLAM-UAV/vslam/setup/isaac_vslam_assets.sh
```

It configures the Isaac ROS container for RealSense:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
touch .isaac_ros_common-config
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config
```

It also generates the GPU CDI spec:

```bash
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
```

### Running Isaac ROS

To build and run the Isaac ROS development container:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

The article says the RealSense camera must be connected before building/running this image.

### RealSense Validation

Inside the Isaac ROS environment, the article validates the camera with:

```bash
realsense-viewer
```

If the camera is not detected, it suggests updating RealSense udev rules with:

```bash
sudo apt install v4l-utils -y
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd ~/librealsense
./scripts/setup_udev_rules.sh
```

## Camera IMU Setup

The article treats IMU use as important for VSLAM quality.

### IMU Calibration

The D435i's internal IMU is factory calibrated, but the article recommends recalibration.

It uses the Librealsense IMU calibration tool from a RealSense Docker environment:

```bash
cd ~/VSLAM-UAV/docker/realsense
./run_docker.sh

cd ~/librealsense/tools/rs-imu-calibration
python3 rs-imu-calibration.py
```

The article walks through six static orientations:
- upright facing out
- USB cable up facing out
- upside down facing out
- USB cable down
- viewing direction down
- viewing direction up

It specifically says to write the calibration results to the camera EEPROM.

### IMU Noise Parameter Estimation

The article then estimates IMU noise and bias parameters using Allan variance.

Process described:

1. Start the Isaac ROS RealSense node:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh -b
${ISAAC_ROS_WS}/VSLAM-UAV/vslam/setup/realsense_node.sh
```

2. In another attached container, record IMU data for at least 3 hours:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh
cd ~
ros2 bag record -o imu_rosbag /camera/imu
```

3. Copy the rosbag out of the container:

```bash
docker cp $(docker ps -q --filter ancestor=isaac_ros_dev-aarch64):/home/admin/imu_rosbag ~
```

4. Copy the rosbag to a desktop machine:

```bash
scp <username>@<jetson_ip>:/home/<username>/imu_rosbag ~/VSLAM-UAV/docker/analysis
```

5. On the desktop machine, build the analysis container:

```bash
cd ~/VSLAM-UAV/docker/analysis
./run_docker.sh
```

6. Configure and run Allan analysis:

```bash
~/VSLAM-UAV/vslam/setup/allan_config.sh
ros2 launch allan_ros2 allan_node.py
python3 ~/ros2_ws/src/allan_ros2/scripts/analysis.py --data deviation.csv
```

7. Copy the resulting IMU parameters into:

```bash
${ISAAC_ROS_WS}/VSLAM-UAV/vslam/isaac_ros_vslam_realsense.py
```

The article says to update:
- `gyro_noise_density`
- `gyro_random_walk`
- `accel_noise_density`
- `accel_random_walk`

## PX4 Flight Controller Setup

### PX4 Serial / Companion Link Parameters

To communicate over `TELEM2`, the article sets:

```text
MAV_1_CONFIG = TELEM2
UXRCE_DDS_CFG = 0
SER_TEL2_BAUD = 921600
```

### PX4 External Vision / No-GPS Parameters

For vision-based operation without GPS, the article sets:

```text
EKF2_HGT_REF = Vision
EKF2_EV_DELAY = 50.0ms
EKF2_GPS_CTRL = 0
EKF2_BARO_CTRL = Disabled
EKF2_RNG_CTRL = Disable range fusion
EKF2_REQ_NSATS = 5
MAV_USEHILGPS = Enabled
EKF2_MAG_TYPE = None
```

The high-level point is that PX4 is configured to trust external vision and not GPS.

## Verifying Flight Controller Communication

The article uses MAVProxy to confirm the Jetson can talk to PX4.

First:

```bash
sudo adduser ${USER} dialout
```

Then reboot, confirm the serial port exists:

```bash
ls /dev/ttyUSB0
```

Install MAVProxy and remove `modemmanager`:

```bash
sudo apt install -y python3-pip
sudo pip3 install mavproxy
sudo apt remove -y modemmanager
```

Run MAVProxy:

```bash
sudo mavproxy.py --master=/dev/ttyUSB0 --baudrate 921600
```

If successful, this confirms the Jetson can reach PX4 over the USB-UART serial link.

## Flight Demo Flow

The article's demo flight sequence is:

1. Start the Isaac ROS container in background mode:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh -b
```

2. Launch the VSLAM node:

```bash
cd ${ISAAC_ROS_WS}/VSLAM-UAV/vslam
./vslam_launch.sh
```

3. Start the `mavrospy` container:

```bash
cd ${ISAAC_ROS_WS}/VSLAM-UAV/docker/mavrospy
./run_docker.sh
```

4. Launch the `mavrospy` node:

```bash
cd ~/VSLAM-UAV/vslam
ros2 launch mavrospy.launch.py
```

5. Optionally verify vision pose publication:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
export ROS_DOMAIN_ID=1
ros2 topic echo /mavros/vision_pose/pose_cov
```

6. Test in `POSITION` mode first, then switch to `OFFBOARD`.

### Supported Flight Patterns

The article says the offboard demo supports:
- `square`
- `square_head`
- `circle`
- `circle_head`
- `figure8`
- `figure8_head`
- `spiral`
- `spiral_head`

Example:

```bash
ros2 launch mavrospy.launch.py pattern:=figure8
```

Patterns ending in `_head` command the drone to face the direction of travel.

## Visualization

For RViz visualization:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh -b
export ROS_DOMAIN_ID=1
rviz2 -d ${ISAAC_ROS_WS}/VSLAM-UAV/vslam/vslam_realsense.cfg.rviz
```

The article warns that RViz over remote desktop or X11 forwarding is usually slow and suggests recording a rosbag instead for later visualization.

## Practical Takeaways

- The project is a full VSLAM-to-PX4 offboard stack, not just a camera demo.
- It assumes a RealSense camera with IMU, specifically a `D435i`.
- It depends heavily on Isaac ROS Docker workflows on Jetson.
- It uses external vision as PX4's primary local pose source in no-GPS flight.
- It recommends a USB-UART adapter for reliable Jetson-to-flight-controller communication.
- It treats SSD storage as effectively required for a usable Jetson workflow.

## Relevance To This Repository

This article is useful to this repo because it validates a practical no-GPS architecture built from:
- Jetson companion computer
- RealSense visual-inertial sensing
- PX4
- MAVROS / MAVLink
- external-vision fusion

The biggest differences from the current local setup are:
- the article uses a `D435i`, while this repo currently discusses a `D455`
- the article uses Isaac ROS VSLAM, while this repo does not yet contain a full real VIO / VSLAM integration path
- the article's PX4 parameter set is specifically tuned for external-vision flight rather than GPS-assisted flight

## Short Bottom Line

The Hackster project shows a working pattern for GPS-denied PX4 flight on Jetson Orin Nano by using a RealSense visual-inertial camera and Isaac ROS VSLAM to publish external vision pose to PX4. The main implementation burden is not basic camera streaming, but building a reliable VSLAM + MAVROS + PX4 external-vision pipeline with the right firmware, Docker setup, IMU tuning, and PX4 EKF configuration.
