# Props-Off Bench Test

## Terminal 1

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

## Terminal 2

```bash
source ~/.bashrc
ros2 topic echo /mavros/state
```

## Terminal 3

```bash
source ~/.bashrc
ros2 topic echo /mavros/local_position/pose
```

## Terminal 4

```bash
source ~/.bashrc
ros2 topic echo /cdrone/drone01/demo/position_hover_state
```

## Terminal 5

```bash
source ~/.bashrc
ros2 topic echo /mavros/statustext/recv
```

## Start

```bash
source ~/.bashrc
ros2 service call /cdrone/drone01/demo/position_hover_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
source ~/.bashrc
ros2 service call /cdrone/drone01/demo/position_hover_abort std_srvs/srv/Trigger "{}"
```

# Props-On Actual Test

## Terminal 1

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

## Terminal 2

```bash
source ~/.bashrc
ros2 topic echo /mavros/state
```

## Terminal 3

```bash
source ~/.bashrc
ros2 topic echo /mavros/local_position/pose
```

## Terminal 4

```bash
source ~/.bashrc
ros2 topic echo /cdrone/drone01/demo/position_hover_state
```

## Terminal 5

```bash
source ~/.bashrc
ros2 topic echo /mavros/statustext/recv
```

## Start

```bash
source ~/.bashrc
ros2 service call /cdrone/drone01/demo/position_hover_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
source ~/.bashrc
ros2 service call /cdrone/drone01/demo/position_hover_abort std_srvs/srv/Trigger "{}"
```
