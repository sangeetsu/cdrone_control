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
ros2 service call /cdrone/drone01/demo/milestone3_start std_srvs/srv/Trigger "{}"
```

Abort the sequence:

```bash
ros2 service call /cdrone/drone01/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```

Useful live topics:

```bash
ros2 topic echo /cdrone/drone01/engagement/state
ros2 topic echo /cdrone/drone01/perception/tracks
ros2 topic echo /mavros/state
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
