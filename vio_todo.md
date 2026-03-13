# VIO TODO

## Goal
Replace the bench-only fixed pose publisher with a real stereo-derived VIO feed so PX4 OFFBOARD works indoors without GPS.

## Current State
- `bench_vision_pose_node` can publish a fixed pose to `/mavros/vision_pose/pose` for props-off bench testing.
- MAVROS `vision_pose` is enabled already.
- MAVROS `odometry` is still denylisted, so `/mavros/odometry/out` is not available yet.
- The stereo vision stack is the intended source for a real estimator.

## Integration Choices
- Fast path: publish `geometry_msgs/msg/PoseStamped` to `/mavros/vision_pose/pose`.
- Better long-term path: enable the MAVROS `odometry` plugin and publish `nav_msgs/msg/Odometry` to `/mavros/odometry/out`.

## TODO
1. Decide whether the first VIO integration target is `vision_pose` or `odometry/out`.
2. Define the stereo stack output contract: pose frame, timestamp source, update rate, and covariance.
3. Add frame conversion between the stereo stack output and PX4/MAVROS expectations.
4. Add a dedicated ROS2 node that republishes stereo VIO into the chosen MAVROS topic.
5. If using `odometry/out`, remove the MAVROS `odometry` denylist entry and validate TF expectations.
6. Capture the PX4 external-vision parameter set required for bench and flight testing.
7. Validate on the bench first with props off, then replace the fixed-pose publisher in the bench launch.
