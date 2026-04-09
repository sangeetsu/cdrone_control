# Motive Frame Setup For Tomorrow

## Current Confirmed Baseline

- Software yaw offset is removed.
  - `rpy_offset_rad` default is now `[0.0, 0.0, 0.0]`.
- PX4 external-vision baseline is confirmed after FCU reboot:
  - `EKF2_EV_CTRL = 11`
  - `EKF2_MAG_TYPE = 5`
  - `EKF2_HGT_REF = 3`
  - `EKF2_GPS_CTRL = 0`
  - `EKF2_BARO_CTRL = 0`
  - `EKF2_RNG_CTRL = 0`
- No-flight arm check passed:
  - arm accepted
  - disarm accepted
- Do not add software offsets in ROS unless a future test proves Motive itself cannot be corrected cleanly.

## Goal

Make Motive the source of truth for the rigid body frame so that:

- world `+Z` is up
- world `+X` is the forward direction of the flight volume you want to use
- the drone rigid body local `+X` points out the drone nose
- raw `VRPN` pose is already correct before it reaches ROS/MAVROS/PX4

## Important Principle

Do not try to make Motive output PX4 `NED`.

Use standard Motive/ROS conventions:

- Motive / VRPN / ROS: right-handed, `Z Up`
- MAVROS / PX4: MAVROS handles the ROS-to-PX4 frame conversion

That means the thing to fix in Motive is the room frame and the rigid body orientation, not a fake NED transform.

## Before You Start

1. Put the drone on a level surface.
2. Decide what physical room direction you want to call "forward" for flight testing.
3. Mark the intended flight origin on the floor.
4. Mark the intended forward direction on the floor.

## Step 1: Set The Motive World Frame

Open Motive calibration tools and set the ground plane carefully.

1. Place the OptiTrack calibration square at the exact floor point you want as the origin.
2. Rotate the square so the direction you want to be world forward matches Motive `+X`.
3. Use Motive's ground-plane / calibration workflow to set the ground plane from that square.
4. Keep the system in `Z Up`.

Notes:

- The ground plane defines the world frame used by streamed rigid-body poses.
- If the world frame is wrong here, every rigid body will look wrong later.

## Step 2: Set Streaming To Standard Convention

Open `Settings > Streaming`.

Use these settings:

- `Up Axis = Z Up`
- stream rigid bodies enabled
- VRPN streaming enabled if that is the path you are using

Do not introduce any experimental axis remaps here.

## Step 3: Re-orient The Rigid Body In Motive

This is the most important part.

You want the rigid body local axes to mean:

- local `+X`: drone nose / forward
- local `+Y`: drone left
- local `+Z`: up

Procedure:

1. Physically place the drone level on the floor.
2. Point the drone nose along the Motive world `+X` direction you chose in Step 1.
3. In Motive, select the rigid body.
4. Turn on rigid body axis visualization if it is not already visible.
5. Open the Builder pane.
6. Go to the rigid body modify tools.
7. Use the rigid body orientation reset / modify controls so the rigid body axes align with the world axes while the drone is sitting nose-along-`+X` and level.
8. If the rigid body pivot is badly placed, move the pivot near the vehicle center.
9. If needed, run refine after the rigid body edit.

What you want to see afterward:

- drone nose points along the rigid body `+X`
- rigid body `+Z` points up
- when the drone is facing room-forward, yaw is near zero

## Step 4: If The Rigid Body Is Still Awkward, Rebuild It

If the rigid body was originally created in a bad pose and the axes still look confusing after modification, rebuild it cleanly.

Procedure:

1. Delete the current rigid body definition.
2. Put the drone level.
3. Point the drone nose along Motive world `+X`.
4. Recreate the rigid body in that pose.
5. Check the displayed local axes again.

This is often the cleanest fix if the original rigid body was created with the drone pointed backward.

## Step 5: Physical Motion Sanity Check In Motive

Do this before launching ROS.

With the drone held level:

1. Move the drone upward.
   - `z` should increase.
2. Move the drone forward, nose-first.
   - `x` should increase.
3. Move the drone to the drone's right.
   - `y` should decrease if using the standard body-left `+Y` convention for the body frame, but the world-frame position should still be visually consistent in Motive.
4. Rotate the drone nose clockwise and counterclockwise.
   - yaw should change with no hidden 180 degree jump

The key check is simple:

- nose aligned with world `+X` should look like yaw near `0 deg`

## Step 6: ROS Verification After Motive Changes

After Motive looks correct, launch the normal stack and compare:

```bash
source ~/.bashrc
ros2 launch drone_bringup position_hover_demo.launch.py \
  pose_source:=optitrack \
  optitrack_server:=192.168.0.217 \
  rigid_body_name:=RigidBody3 \
  takeoff_altitude_m:=0.6 \
  takeoff_rate_m_s:=0.3 \
  takeoff_strategy:=AUTO_MODE \
  hover_duration_s:=5.0 \
  hover_mode:=HOLD \
  max_horizontal_excursion_m:=0.5
```

Then check:

```bash
source ~/.bashrc
ros2 topic echo /vrpn_mocap/RigidBody3/pose --once
ros2 topic echo /mavros/vision_pose/pose --once
ros2 topic echo /mavros/local_position/pose --once
```

Expected behavior:

- `/vrpn_mocap/RigidBody3/pose` and `/mavros/vision_pose/pose` should match closely
- `/mavros/local_position/pose` should also match closely after EKF settles
- no 180 degree yaw disagreement

## What Success Looks Like

At rest, after everything settles:

- VRPN yaw is sensible for the actual drone orientation
- `vision_pose` yaw matches VRPN yaw
- `local_position` yaw matches `vision_pose` yaw
- moving up changes Z correctly
- moving forward changes X correctly
- no software frame hacks are needed

## What Not To Do

- Do not reintroduce the old 180 degree software yaw offset.
- Do not try to make Motive stream `NED`.
- Do not test with takeoff until the static pose checks above look correct.

## Tomorrow Checklist

- Set ground plane at the real desired origin.
- Align world `+X` with the intended flight forward direction.
- Keep streaming `Z Up`.
- Fix or rebuild the rigid body so local `+X` points out the drone nose.
- Verify yaw near zero when nose points along world `+X`.
- Launch ROS stack.
- Compare `VRPN`, `vision_pose`, and `local_position`.
- Only after that, do another no-flight arm check.
