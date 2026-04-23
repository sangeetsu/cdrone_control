# Milestone 3

Commands in this note assume your shell has already loaded ROS 2 and the local
workspace from `~/.bashrc`.

## What Milestone 3 Adds

Milestone 3 is the ground-start version of the studio demo:

1. autonomous vertical takeoff
2. OFFBOARD warmup and short staging hover
3. sequential milestone 2 target servicing
4. AUTO.LAND cleanup by default

The target geometry and success condition stay the same as milestone 2:

- target loiter range: `2.2-2.4 m`
- service distance: `1.5 +/- 0.2 m`
- dwell per target: `>= 3.0 s`

## Indoor Speed Profile Fix

The indoor profile issue had two parts:

1. the active demo paths could apply the indoor PX4 profile before takeoff
2. that profile included `MPC_TKO_SPEED: 0.6`

Milestone 3 fixes this in the active ground-start path by:

- restoring a takeoff-safe baseline profile before takeoff
- applying the indoor speed profile only after OFFBOARD handoff
- restoring the captured baseline values after landing or abort cleanup

The shared indoor profile at
`ros2/src/drone_bringup/config/indoor_speed_profile.yaml` also no longer sets
`MPC_TKO_SPEED`, so a stale indoor profile no longer slows the next takeoff.

The pre-takeoff baseline used by milestone 3 lives at:

- `ros2/src/drone_bringup/config/milestone3_takeoff_baseline_profile.yaml`

The manual recovery script still exists if a run is interrupted during cleanup:

- `/home/jetson/cdrone_control/scripts/restore_hover_baseline_params.sh`

## PX4 Speed Profiles

Milestone 3 now supports three named post-OFFBOARD PX4 speed profiles:

- `indoor`: current conservative indoor profile
- `fun`: midpoint profile between indoor and the old fast baseline
- `default`: the faster baseline motion limits

Select them directly in the launch command:

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1 \
  speed_profile:=indoor
```

or:

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1 \
  speed_profile:=fun
```

or:

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1 \
  speed_profile:=default
```

If you provide both `speed_profile:=...` and an explicit
`speed_profile_config:=...` path, the explicit config path wins.

If you want to push one of the same profiles to PX4 outside the Milestone 3
launch flow, use:

```bash
python3 scripts/apply_px4_speed_profile.py indoor
python3 scripts/apply_px4_speed_profile.py fun
python3 scripts/apply_px4_speed_profile.py default
```

These PX4 values are real units, not normalized multipliers:

- `MPC_XY_CRUISE`, `MPC_XY_VEL_MAX`, `MPC_Z_V_AUTO_UP`, `MPC_Z_V_AUTO_DN`,
  `MPC_Z_VEL_MAX_DN`, `MPC_LAND_SPEED`, and `MPC_TKO_SPEED` are in `m/s`
- `MPC_ACC_HOR` is in `m/s^2`
- `MPC_JERK_AUTO` and `MPC_JERK_MAX` are in `m/s^3`

Current post-OFFBOARD profile values:

| Profile | `MPC_XY_CRUISE` | `MPC_XY_VEL_MAX` | `MPC_ACC_HOR` | `MPC_JERK_AUTO` | `MPC_JERK_MAX` | `MPC_Z_V_AUTO_UP` | `MPC_Z_V_AUTO_DN` | `MPC_Z_VEL_MAX_DN` | `MPC_LAND_SPEED` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `indoor` | `1.0` | `1.5` | `1.0` | `1.0` | `2.0` | `0.8` | `0.5` | `0.5` | `0.35` |
| `fun` | `3.0` | `6.0` | `2.0` | `2.5` | `5.0` | `1.8` | `1.0` | `1.0` | `0.5` |
| `default` | `5.0` | `12.0` | `3.0` | `4.0` | `8.0` | `3.0` | `1.5` | `1.5` | `0.7` |

Takeoff is handled separately. Before OFFBOARD handoff, Milestone 3 restores
the pre-takeoff baseline profile from
`ros2/src/drone_bringup/config/milestone3_takeoff_baseline_profile.yaml`,
including `MPC_TKO_SPEED: 1.5`, so the named post-OFFBOARD profiles do not slow
the climbout.

Important tuning note: if the drone still turns too slowly to keep a laterally
moving target in frame, the PX4 profile is only part of the story. Milestone 3
also limits turning on the controller side in
`ros2/src/drone_bringup/config/milestone3_demo.yaml`, especially:

- `kp_yaw`
- `max_yaw_rate_rps`

So a faster PX4 profile can help the vehicle translate more aggressively, but
it will not override a low yaw-rate cap in the Milestone 3 controller.

## Launch Surface

Main launch:

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1
```

Useful target-count variants:

- one-target first flight: `required_completion_count:=1`
- two-target follow-up: `required_completion_count:=2`
- three-target full sequence: `required_completion_count:=3`

The launch defaults to:

- pre-takeoff baseline profile enabled
- indoor speed profile enabled after OFFBOARD handoff
- automatic landing on complete
- automatic landing on abort

## Services And Topics

Start the full ground-to-target sequence:

```bash
ros2 service call /cdrone/cdrone4/demo/milestone3_start std_srvs/srv/Trigger "{}"
```

Abort the sequence:

```bash
ros2 service call /cdrone/cdrone4/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```

Useful live topics:

```bash
ros2 topic echo /cdrone/cdrone4/engagement/state
ros2 topic echo /cdrone/cdrone4/perception/tracks
ros2 topic echo /cdrone/cdrone4/mavros/state
```

## Validation Used For This Revision

Targeted checks for this milestone:

```bash
cd /home/jetson/cdrone_control/ros2
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
python3 -m pytest src/drone_control_pkg/test/test_milestone2_demo_logic.py -q
python3 -m pytest src/drone_control_pkg/test/test_speed_profile_configs.py -q
ros2 launch drone_bringup milestone3_demo.launch.py --show-args
```

## Current Limits

Milestone 3 now has the code path for:

- autonomous takeoff to staging hover
- delayed indoor speed-profile activation
- sequential short-range target servicing
- automatic cleanup landing

Still pending flight validation:

- first end-to-end one-target ground-start run
- proof that the OFFBOARD handoff is clean in the studio on the current FCU
- full three-target continuous run indoors
