# Rehaul VIO + PX4 Viability Review

Date: 2026-03-20

## Verdict

**Viable with adjustments.**

The overall direction is sound: a Jetson Orin Nano can run a RealSense D455-based VIO/VSLAM pipeline, publish external vision through MAVROS, and give PX4 1.16 the local position estimate needed for indoor `Position` / `OFFBOARD` flight without GPS. The main risk is not the concept itself, but **version drift and interface choices**:

- PX4 1.16 is compatible with external vision / VIO.
- MAVROS on ROS 2 Humble supports both `vision_pose` and `odometry` external-vision paths.
- The D455 is officially supported by Isaac ROS RealSense workflows.
- But Isaac ROS docs have moved on since the Humble-era tutorials, so mixing "latest" Isaac ROS setup with a Humble repo is likely to create avoidable breakage.

## Confirmed Facts

### 1. PX4 1.16 supports no-GPS flight with external vision

Confirmed from PX4 v1.16 docs:

- PX4 documents both “External Position Estimation” and “Visual Inertial Odometry (VIO)” for GPS-denied navigation.
- PX4 says `OFFBOARD` requires a valid position/pose source such as GPS, optical flow, VIO, or motion capture.
- PX4’s VIO guidance recommends publishing `nav_msgs/Odometry` to `/mavros/odometry/out` plus `mavros_msgs/CompanionProcessStatus` to `/mavros/companion_process/status`.
- PX4 documents the key estimator parameters for external vision on 1.16, including `EKF2_EV_CTRL`, `EKF2_HGT_REF`, `EKF2_EV_DELAY`, and `EKF2_EV_POS_*`.

Implication:

- The blocker in `problem.md` is real and correctly diagnosed: **the missing aiding source is the core issue**, not simply missing GPS.

### 2. Indoor position-hold / position-control is feasible once EV/VIO is fused

Confirmed from PX4 docs:

- PX4’s external-position-estimation guide explicitly describes switching into position control once external pose is valid.
- PX4’s VIO test card expects the vehicle to hold position in `Position` mode without GPS when VIO is configured correctly.

Implication:

- The repo’s target of eventually flying in indoor position control using VIO is technically viable on PX4 1.16.

### 3. MAVROS on ROS 2 Humble supports the needed interfaces

Confirmed from MAVROS docs:

- MAVROS ROS 2 requires `Humble+`.
- `mavros_extras` on Humble includes:
  - `vision_pose_estimate`
  - `vision_speed_estimate`
  - `odom`
  - `companion_process_status`
- The odometry plugin is specifically described as “Send odometry to FCU from another estimator.”

Implication:

- Your current MAVROS-based architecture is valid. You do not need to abandon MAVROS to make this work.

### 4. PX4’s preferred VIO path is `odometry/out`, not just `vision_pose`

Confirmed from PX4 v1.16 VIO docs:

- The suggested ROS VIO node should publish `nav_msgs/Odometry` to `/mavros/odometry/out`.
- PX4 also expects a companion-process status heartbeat for VIO health.

Confirmed from PX4 external-position docs:

- MAVROS can still relay `PoseStamped` on `/mavros/vision_pose/pose`, and that path remains supported.

Implication:

- `vision_pose` is acceptable as a bootstrap path.
- `odometry/out` is the better long-term interface because it carries pose, twist, and covariance in a form PX4’s VIO docs explicitly target.

### 5. D455 is a valid substitute for D435i in the Isaac ROS RealSense path

Confirmed from official Isaac ROS RealSense docs:

- The RealSense compatibility table lists both `D455` and `D435i` as supported.
- Isaac ROS RealSense docs still pin a specific stack for the examples:
  - firmware `5.13.0.50`
  - librealsense `2.55.1`
  - realsense-ros `4.51.1-isaac`

Implication:

- The Hackster article’s D435i-specific recipe is not invalid for your D455, but it is **not plug-and-play**; the reusable part is the architecture, not the exact hardware write-up.

### 6. ROS 2 Humble is still a reasonable base in March 2026

Confirmed from ROS 2 docs:

- Humble remains supported through **May 2027**.

Implication:

- A Humble-based rehaul is still reasonable for this drone stack and does not require an immediate ROS distro jump.

## Important Adjustments

### 1. Do not follow the newest Isaac ROS docs blindly

Confirmed from current Isaac ROS docs:

- The current Isaac ROS documentation has moved forward beyond the old Humble-era setup.
- Isaac ROS package pages now mention newer platform support such as JetPack `7.1` and ROS 2 `Jazzy`.
- The Humble-compatible path is still documented under the pinned `release-3.2` docs.

Implication:

- If this repo stays on ROS 2 Humble, use **Isaac ROS release-3.2-era docs and containers**, not the newest top-level “getting started” path.

### 2. RealSense version pinning matters more than the tutorial makes it look

Confirmed from Isaac ROS docs:

- Deviating from the pinned RealSense versions “will break Isaac ROS examples.”

Confirmed from Intel RealSense sources:

- Newer librealsense releases now mention Jetson JP `6.1`, `6.2`, and `7.0`, so native RealSense support has moved forward.

Inference:

- A newer host-side RealSense install may still work for camera bring-up, but it can diverge from the Isaac ROS container assumptions. Mixing host-native RealSense packages with Isaac-pinned container expectations is a likely source of pain.

Recommendation:

- Treat RealSense in two layers:
  - host layer for raw camera sanity checks
  - pinned Isaac/container layer for VSLAM runtime

### 3. Frame handling is still an easy place to fail

Confirmed from PX4 docs:

- ROS data should be provided in ENU.
- MAVROS handles conversion to PX4 frames.
- For odometry, `twist` must be expressed in the `child_frame_id`.

Implication:

- The plan is viable only if the rehaul gives frame conventions first-class treatment.
- This is a likely failure mode even when the camera and SLAM both “work.”

### 4. The current repo’s denylist on the MAVROS odometry plugin is incompatible with the recommended end state

Confirmed locally plus PX4 docs:

- The repo currently deny-lists the MAVROS `odometry` plugin.
- PX4’s VIO docs recommend `/mavros/odometry/out`.

Implication:

- A viable rehaul should remove that denylist entry when switching from bench pose injection to real VIO.

## Likely Risks, But Not Hard Blocks

### Confirmed risks

- Isaac ROS + RealSense examples are tightly version-coupled.
- New Isaac ROS docs are no longer centered on Humble.
- MAVROS odometry integration is more sensitive to TF/frame correctness than the simple `vision_pose` path.

### Inference-based risks

- If the D455 is running newer firmware than `5.13.0.50`, Isaac ROS example friction is likely.
- If JetPack on the Jetson is newer than the pinned Humble/Isaac ROS 3.2 assumptions, staying entirely native instead of containerized will increase breakage odds.
- If the eventual VIO source produces only pose and no trustworthy body-frame velocity/covariance, PX4 can still work, but tuning and estimator confidence may be worse than with a proper odometry feed.

## Practical Recommendation

Use this path:

1. Keep ROS 2 Humble and PX4 1.16.
2. Re-enable MAVROS `odometry` for the real VIO path.
3. Keep `vision_pose` only as a short bootstrap / bench-validation path.
4. Pin Isaac ROS work to the `release-3.2` documentation and version set if you adopt Isaac ROS Visual SLAM.
5. Verify the D455 firmware and RealSense driver versions before deep integration.
6. Make frame validation and latency tuning part of the implementation plan, not cleanup work.

## Bottom Line

**This project is not blocked by a fundamental incompatibility.** The architecture is viable in 2026 for your hardware. The main change I would make to the rehaul plan is to center it on:

- a pinned Humble-compatible Isaac ROS / RealSense stack if using Isaac ROS,
- MAVROS `odometry/out` as the real target interface,
- strict frame/timestamp validation before flight,
- bench-first testing before any future ESC/motor-enabled carrier is brought online.

## Sources

- PX4 v1.16 external position estimation:
  - https://docs.px4.io/v1.16/en/ros/external_position_estimation
- PX4 v1.16 VIO:
  - https://docs.px4.io/v1.16/en/computer_vision/visual_inertial_odometry
- PX4 offboard mode:
  - https://docs.px4.io/main/en/flight_modes/offboard.html
- PX4 EKF2 tuning / external vision:
  - https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html
- PX4 VIO test card:
  - https://docs.px4.io/main/en/test_cards/mc_07_vio.html
- MAVROS repository README:
  - https://github.com/mavlink/mavros
- MAVROS Humble docs:
  - https://docs.ros.org/en/humble/p/mavros/__README.html
- MAVROS extras Humble docs:
  - https://docs.ros.org/en/humble/p/mavros_extras/index.html
- Isaac ROS release-3.2 getting started:
  - https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html
- Isaac ROS release-3.2 RealSense setup:
  - https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/sensors/realsense_setup.html
- Isaac ROS current Visual SLAM docs:
  - https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
- Isaac ROS RealSense tutorial:
  - https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_realsense.html
- ROS 2 distributions / Humble support window:
  - https://docs.ros.org/en/rolling/Releases.html
- Intel RealSense ROS wrapper:
  - https://github.com/IntelRealSense/realsense-ros
- Intel RealSense SDK:
  - https://github.com/IntelRealSense/librealsense
