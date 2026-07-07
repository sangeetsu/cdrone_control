# Changelog 2026-05-04

## Milestone 3 Flight Defaults

- Reduced `takeoff_altitude_m` from `3.0` to `2.6` meters for the milestone3 demo.
- Raised `external_pose_timeout_s` from `0.25` to `0.75` seconds in `milestone3_demo.launch.py`.
- Kept the milestone3 node fallback altitude aligned at `2.6` meters for direct node runs.

## Why

- Recent flights showed short OptiTrack/VRPN pose gaps around `0.30` seconds.
- The external pose bridge was marking companion pose inactive after `0.25` seconds, so the demo aborted and commanded `AUTO.LAND` before the demo's `local_pose_timeout_s=0.5` window mattered.
- A `0.75` second external pose timeout gives brief mocap gaps room to recover while still treating sustained pose loss as unsafe.
- Lowering the takeoff altitude to `2.6` meters should reduce marker visibility stress compared with the recent `3.0` meter tests.

## Files Changed

- `ros2/src/drone_bringup/config/milestone3_demo.yaml`
- `ros2/src/drone_bringup/launch/milestone3_demo.launch.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/milestone3_demo_sequence_node.py`
- Removed stale milestone3 `.bak` config/code backups that still carried older altitude defaults.

## cdrone4 Implementation Notes

Apply the same values on cdrone4:

```bash
external_pose_timeout_s:=0.75
takeoff_altitude_m:=2.6
```

If cdrone4 has its own milestone3 config or launch override, update both places so the launch argument and YAML config agree.

Before flying, confirm the active launch prints or parameters show:

```bash
ros2 param get /cdrone/cdrone4/milestone3_demo_sequence_node takeoff_altitude_m
ros2 param get /cdrone/cdrone4/external_pose_bridge_node input_timeout_s
ros2 param get /cdrone/cdrone4/external_pose_adapter_node timeout_s
```
