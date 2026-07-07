# PX4Flow Topic Check

Use this props-off bench check after adding an optical flow sensor through the
PX4/ARK/MAVLink path. It only observes MAVROS topics and optionally requests a
MAVLink message stream; it does not arm, change modes, or tune estimator params.

## Commands

Start MAVLink routing if it is not already running:

```bash
/home/jetson/.local/bin/start_mavlink_router.sh
```

Start MAVROS with the repo defaults:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
ros2 launch drone_bringup drone.launch.py
```

In another terminal, run the check:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
scripts/check_px4flow_topics.sh
```

If the topics exist but no samples arrive, ask PX4/MAVROS to stream optical
flow messages during the check:

```bash
scripts/check_px4flow_topics.sh --request-stream-rate 10
```

## Expected Success

The script should report `PASS` after at least one checked topic produces both
a live sample and a rate estimate. The primary flow topic is:

```text
/cdrone/cdrone3/mavros/px4flow/raw/optical_flow_rad
```

Useful fields include `quality`, `distance`, `integrated_x`, and
`integrated_y`. PX4Flow range and fallback MAVROS optical-flow plugin topics
are also checked:

```text
/cdrone/cdrone3/mavros/px4flow/ground_distance
/cdrone/cdrone3/mavros/optical_flow/raw/optical_flow
/cdrone/cdrone3/mavros/optical_flow/ground_distance
```

## Troubleshooting

- `MAVROS state topic is missing`: start `mavlink-router`, then launch
  `ros2 launch drone_bringup drone.launch.py`.
- `connected=true was not observed`: MAVROS is running, but the FCU link is not
  healthy. Check FCU power, USB, and the mavlink-router endpoint.
- `topic exists but no sample`: the plugin loaded, but PX4 is not sending the
  message on this MAVLink link. Retry with `--request-stream-rate 10`.
- `no checked topic produced both a sample and rate estimate`: confirm the
  optical flow sensor is connected to PX4, visible in QGC/PX4, and configured
  to publish MAVLink `OPTICAL_FLOW_RAD` or `OPTICAL_FLOW`.
