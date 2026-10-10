"""MAVROS transport for local OFFBOARD hover with ARK Flow or ZED aiding."""

from dataclasses import fields
import math
import signal
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, ExtendedState, OpticalFlowRad, PositionTarget, State
from mavros_msgs.srv import CommandBool, MessageInterval, SetMode
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace
from drone_control_pkg.onboard_hover_logic import HoverController, HoverSettings
from drone_control_pkg.topic_utils import cdrone_topic, join_topic


class OnboardHoverDemoSequenceNode(Node):
    ZED_PROFILE = {
        "EKF2_EV_CTRL": 11, "EKF2_HGT_REF": 3, "EKF2_OF_CTRL": 0,
        "EKF2_RNG_CTRL": 0, "EKF2_BARO_CTRL": 1, "EKF2_GPS_CTRL": 0,
        "EKF2_MAG_TYPE": 5, "EKF2_EV_DELAY": 0.0, "EKF2_EV_NOISE_MD": 0,
        "EKF2_EV_POS_X": 0.0, "EKF2_EV_POS_Y": 0.0, "EKF2_EV_POS_Z": 0.0,
        "COM_OBL_RC_ACT": 4, "COM_OF_LOSS_T": 1.0,
    }

    def __init__(self, **kwargs):
        super().__init__("onboard_hover_demo_sequence_node", **kwargs)
        descriptor = ParameterDescriptor(read_only=True)

        def parameter(name, default):
            return self.declare_parameter(name, default, descriptor).value

        defaults = HoverSettings()
        settings = HoverSettings(**{
            f.name: parameter(f.name, getattr(defaults, f.name)) for f in fields(defaults)
        })
        self.controller = HoverController(settings)
        self.drone_id = parameter("drone_id", configured_drone_id())
        namespace = parameter("mavros_namespace", configured_mavros_namespace())
        self.request_streams = parameter("request_telemetry_streams", True)
        self.flow_topic = parameter("flow_topic", "") or join_topic(
            namespace, "px4flow/raw/optical_flow_rad")
        self.range_topic = parameter("range_topic", "") or join_topic(
            namespace, "px4flow/ground_distance")
        self.vision_topic = parameter("vision_topic", "") or cdrone_topic(
            self.drone_id, "zed/pose_cov")
        self.estimator_note = ""
        self.source_stamps = {}
        self.future = None
        self.interval_future = None
        self.interval_started = 0.0
        self.interval_attempts = 0
        self.next_interval_time = 0.0
        # These only adjust outgoing telemetry rates, never sensor/EKF parameters.
        self.intervals = [(32, 30.0), (230, 5.0), (245, 5.0)]
        if settings.localization_source != "zed":
            self.intervals[1:1] = [(100, 20.0), (132, 20.0)]
        if settings.initial_climb_enabled:
            self.intervals.insert(1, (85, 10.0))  # FCU position target echo for handover
        self.next_interval = 0
        self.profile_client = (self.create_client(GetParameters, join_topic(namespace, "param/get_parameters"))
                               if settings.localization_source == "zed" else None)
        self.profile_future = None
        self.profile_started = 0.0
        self.profile_retry_after = 0.0
        self.last_status_time = -math.inf
        self.last_diagnostics = ""

        for message_type, topic, callback in (
            (State, join_topic(namespace, "state"), self.state_callback),
            (ExtendedState, join_topic(namespace, "extended_state"), self.extended_callback),
            (EstimatorStatus, join_topic(namespace, "estimator_status"), self.estimator_callback),
            (PoseStamped, join_topic(namespace, "local_position/pose"), self.pose_callback),
        ):
            self.create_subscription(message_type, topic, callback, qos_profile_sensor_data)
        if settings.localization_source == "zed":
            self.create_subscription(PoseWithCovarianceStamped, self.vision_topic,
                                     self.vision_pose_callback, qos_profile_sensor_data)
        else:
            self.create_subscription(OpticalFlowRad, self.flow_topic,
                                     self.flow_callback, qos_profile_sensor_data)
            self.create_subscription(Range, self.range_topic,
                                     self.range_callback, qos_profile_sensor_data)
        if settings.initial_climb_enabled:
            self.create_subscription(Imu, join_topic(namespace, "imu/data"),
                                     self.imu_callback, qos_profile_sensor_data)
            self.create_subscription(PositionTarget, join_topic(namespace, "setpoint_raw/target_local"),
                                     self.position_target_callback, qos_profile_sensor_data)

        self.position_pub = self.create_publisher(
            PoseStamped, join_topic(namespace, "setpoint_position/local"), 10)
        self.attitude_pub = self.create_publisher(
            AttitudeTarget, join_topic(namespace, "setpoint_raw/attitude"), 10)
        self.status_pub = self.create_publisher(
            String, cdrone_topic(self.drone_id, "demo/position_hover_state"), 10)
        self.diagnostics_pub = self.create_publisher(
            String, cdrone_topic(self.drone_id, "demo/position_hover_diagnostics"), 10)
        self.mode_client = self.create_client(SetMode, join_topic(namespace, "set_mode"))
        self.arm_client = self.create_client(CommandBool, join_topic(namespace, "cmd/arming"))
        self.interval_client = self.create_client(
            MessageInterval, join_topic(namespace, "set_message_interval"))
        for suffix, callback in (
            ("start", self.start_callback), ("abort", self.abort_callback),
            ("check", self.check_callback),
        ):
            self.create_service(Trigger, cdrone_topic(
                self.drone_id, f"demo/position_hover_{suffix}"), callback)
        self.create_timer(1.0 / settings.publish_rate_hz, self.tick)
        self.get_logger().info(
            f"Local hover: source={settings.localization_source}; enable_flight={settings.enable_flight}; "
            f"local ENU pose; vision={self.vision_topic}" if settings.localization_source == "zed" else
            f"Onboard hover: enable_flight={settings.enable_flight}; local ENU pose; "
            f"flow={self.flow_topic}; downward range={self.range_topic}")

    def now_s(self):
        # Safety timeouts must advance even if ROS simulation time stops or jumps.
        return time.monotonic()

    def accept_stamp(self, name, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        settings = self.controller.settings
        timeout = (settings.state_timeout_s if name in {"state", "extended", "estimator"}
                   else settings.sensor_timeout_s)
        # Do not refresh receipt time for duplicates, out-of-order, or delayed samples.
        if (stamp <= self.source_stamps.get(name, -math.inf)
                or not -0.1 <= ros_now - stamp <= timeout):
            return False
        self.source_stamps[name] = stamp
        self.controller.telemetry.received[name] = self.now_s()
        return True

    def state_callback(self, msg):
        t = self.controller.telemetry
        if msg.connected != t.connected:
            self.controller.connection_reset(self.now_s())
            self.source_stamps.clear()
            self.next_interval = 0
            self.interval_attempts = 0
            self.next_interval_time = self.now_s() + 2.0
            if self.profile_client is not None:
                if self.profile_future is not None:
                    self.profile_future.cancel()
                self.profile_future = None
                t.vision_profile_ok = False
                t.vision_profile_reason = "PX4 vision parameters not verified after reconnection"
            if self.interval_future is not None:
                self.interval_future.cancel()
                self.interval_future = None
        if self.accept_stamp("state", msg):
            t.connected, t.armed, t.mode = msg.connected, msg.armed, msg.mode.upper()

    def extended_callback(self, msg):
        if self.accept_stamp("extended", msg):
            self.controller.telemetry.landed = msg.landed_state

    def estimator_callback(self, msg):
        if not self.accept_stamp("estimator", msg):
            return
        aiding_source = ("vision" if self.controller.settings.localization_source == "zed"
                         else "optical-flow")
        t = self.controller.telemetry
        t.update_relative_position_prediction(
            msg.pred_pos_horiz_rel_status_flag, self.now_s())
        checks = {
            "attitude invalid": msg.attitude_status_flag,
            "horizontal velocity invalid": msg.velocity_horiz_status_flag,
            "vertical velocity invalid": msg.velocity_vert_status_flag,
            "horizontal position invalid": (
                msg.pos_horiz_rel_status_flag or msg.pos_horiz_abs_status_flag),
            "vertical position invalid": msg.pos_vert_abs_status_flag,
            "accelerometer error": not msg.accel_error_status_flag,
        }
        if self.controller.settings.localization_source != "zed":
            checks[f"horizontal aiding inactive (check PX4 {aiding_source} fusion)"] = (
                msg.pred_pos_horiz_rel_status_flag)
        # PX4 1.16 sets CONST_POS_MODE for vehicle_at_rest as well as fake_pos.
        # The bit alone cannot distinguish a healthy resting vehicle from fallback.
        self.estimator_note = (
            "PX4 constant-position/rest flag set; checked aiding separately"
            if msg.const_pos_mode_status_flag else "")
        vertical_checks = {name: checks[name] for name in (
            "attitude invalid", "vertical velocity invalid", "vertical position invalid", "accelerometer error")}
        t.vertical_estimator_ok = all(vertical_checks.values())
        t.vertical_estimator_reason = ", ".join(
            name for name, good in vertical_checks.items() if not good) or "valid"
        t.estimator_ok = all(checks.values())
        t.estimator_reason = ", ".join(name for name, good in checks.items() if not good) or "valid"

    def imu_callback(self, msg):
        if not self.accept_stamp("imu", msg):
            return
        q = msg.orientation
        values = (q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v*v for v in values))
        if (msg.orientation_covariance[0] < 0 or not all(math.isfinite(v) for v in values)
                or not 0.9 <= norm <= 1.1):
            self.controller.telemetry.attitude = None
            return
        x, y, z, w = (v / norm for v in values)
        self.controller.telemetry.attitude = (
            math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

    def position_target_callback(self, msg):
        if not self.accept_stamp("position_target", msg):
            return
        ignored = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                   PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_YAW)
        p = msg.position
        values = (p.x, p.y, p.z, msg.yaw)
        self.controller.telemetry.position_target = (
            values if (msg.coordinate_frame == PositionTarget.FRAME_LOCAL_NED
                       and not msg.type_mask & ignored
                       and all(math.isfinite(v) for v in values)) else None)

    def pose_callback(self, msg):
        if not self.accept_stamp("pose", msg):
            return
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not all(math.isfinite(v) for v in values) or not 0.9 <= norm <= 1.1:
            self.controller.telemetry.pose = None
            return
        qx, qy, qz, qw = (v / norm for v in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.controller.update_pose((p.x, p.y, p.z, yaw), msg.header.frame_id, self.now_s())

    def vision_pose_callback(self, msg):
        if not self.accept_stamp("vision", msg):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        covariance = msg.pose.covariance
        if (not all(math.isfinite(v) for v in (*values, *covariance))
                or any(covariance[7*i] < 0 for i in range(6))
                or not 0.9 <= norm <= 1.1):
            self.controller.telemetry.vision_pose = None
            return
        qx, qy, qz, qw = (v / norm for v in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.controller.update_vision_pose((p.x, p.y, p.z, yaw), msg.header.frame_id,
                                           self.now_s())

    def flow_callback(self, msg):
        if self.accept_stamp("flow", msg):
            reasons = []
            if msg.integration_time_us <= 0:
                reasons.append(f"integration_time_us={msg.integration_time_us} must be positive")
            minimum = self.controller.settings.flow_min_quality
            if msg.quality < minimum:
                reasons.append(f"quality={msg.quality} below minimum={minimum}")
            if not all(math.isfinite(v) for v in (msg.integrated_x, msg.integrated_y)):
                reasons.append("non-finite integrated pixel flow")
            # Optional gyro integrals can be NaN (including after MAVROS frame
            # conversion). This node uses PX4's fused estimate, not these gyros.
            t = self.controller.telemetry
            t.flow_ok = not reasons
            t.flow_reason = ", ".join(reasons) or "valid"

    def range_callback(self, msg):
        if self.accept_stamp("range", msg):
            s = self.controller.settings
            minimum = max(s.range_min_m, msg.min_range)
            maximum = min(s.range_max_m, msg.max_range)
            if (not all(math.isfinite(v) for v in (msg.min_range, msg.max_range))
                    or not 0 <= msg.min_range < msg.max_range or minimum >= maximum):
                reason = f"invalid message limits [{msg.min_range:g}, {msg.max_range:g}] m"
            elif not math.isfinite(msg.range):
                reason = f"non-finite distance={msg.range}"
            elif not minimum - 1e-6 <= msg.range <= maximum:
                reason = (f"distance={msg.range:.3f} m outside usable "
                          f"[{minimum:.3f}, {maximum:.3f}] m")
            else:
                reason = ""
            t = self.controller.telemetry
            t.range_ok = not reason
            t.range_reason = reason or "valid"
            t.ground_distance = msg.range if not reason else None

    def check_callback(self, request, response):
        del request
        reasons = self.controller.readiness(self.now_s())
        if self.controller.state != "IDLE":
            if self.controller.state in self.controller.TERMINAL:
                reasons.insert(0, f"restart node after {self.controller.state.lower()} run")
            else:
                reasons.insert(0, "sequence already active")
        response.success = not reasons
        response.message = ("ready" if not reasons else "; ".join(reasons)) + (
            f"; enable_flight={self.controller.settings.enable_flight}"
            f"; state={self.controller.state}; source={self.controller.settings.localization_source}; "
            + (f"vision_topic={self.vision_topic}" if self.controller.settings.localization_source == "zed"
               else f"range_topic={self.range_topic}"))
        if self.estimator_note and self.controller.telemetry.fresh(
                "estimator", self.now_s(), self.controller.settings.state_timeout_s):
            response.message += "; note: " + self.estimator_note
        if self.controller.uses_initial_climb():
            response.message += "; initial climb: horizontal position hold pending"
            climb = self.controller.climb
            if climb is not None:
                response.message += (f"; measured_climb={climb.height:.3f} m"
                                     f"; climb_speed={climb.speed:.3f} m/s; thrust={climb.thrust:.3f}")
        elif self.controller.settings.initial_climb_enabled and self.controller.state in self.controller.TERMINAL:
            response.message += "; initial climb enabled; horizontal aiding checked before position handover"
        return response

    def start_callback(self, request, response):
        del request
        if self.controller.settings.enable_flight and (
                not self.mode_client.service_is_ready() or not self.arm_client.service_is_ready()):
            response.success, response.message = False, "MAVROS command services unavailable"
        else:
            response.success, response.message = self.controller.start(self.now_s())
        return response

    def abort_callback(self, request, response):
        del request
        response.success = self.controller.state not in self.controller.TERMINAL
        if response.success:
            self.controller.abort(self.now_s(), "operator abort")
        response.message = "abort requested" if response.success else "sequence is not active"
        return response

    def service_telemetry(self, now):
        if not self.request_streams or not self.controller.telemetry.connected:
            return
        if self.interval_future is not None:
            if self.interval_future.done():
                try:
                    success = self.interval_future.result().success
                except Exception:
                    success = False
                self.interval_future = None
                self.next_interval_time = now + 1.0
                if success or self.interval_attempts >= 3:
                    if not success:
                        self.get_logger().warn(
                            f"Telemetry stream {self.intervals[self.next_interval][0]} "
                            "not accepted after 3 attempts; check FCU sensors/streams")
                    self.next_interval += 1
                    self.interval_attempts = 0
            elif now - self.interval_started > 2.0:
                self.interval_future.cancel()
                self.interval_future = None
                self.next_interval_time = now + 1.0
                if self.interval_attempts >= 3:
                    self.get_logger().warn(
                        f"Telemetry stream {self.intervals[self.next_interval][0]} timed out")
                    self.next_interval += 1
                    self.interval_attempts = 0
            return
        if (now < self.next_interval_time or self.next_interval >= len(self.intervals)
                or not self.interval_client.service_is_ready()):
            return
        request = MessageInterval.Request()
        request.message_id, request.message_rate = self.intervals[self.next_interval]
        self.interval_future = self.interval_client.call_async(request)
        self.interval_started = now
        self.interval_attempts += 1

    def service_vision_profile(self, now):
        telemetry = self.controller.telemetry
        if self.profile_client is None or not telemetry.connected or telemetry.vision_profile_ok:
            return
        if self.profile_future is not None:
            if not self.profile_future.done():
                if now - self.profile_started > 2.0:
                    self.profile_future.cancel()
                    self.profile_future = None
                    self.profile_retry_after = now + 2.0
                    telemetry.vision_profile_reason = "PX4 parameter read timed out"
                return
            try:
                result = self.profile_future.result()
                if len(result.values) != len(self.ZED_PROFILE):
                    raise ValueError("incomplete PX4 parameter response")
                for (name, expected), value in zip(self.ZED_PROFILE.items(), result.values):
                    floating = isinstance(expected, float)
                    required_type = (ParameterType.PARAMETER_DOUBLE if floating
                                     else ParameterType.PARAMETER_INTEGER)
                    if value.type != required_type:
                        raise ValueError(f"PX4 {name}: parameter unavailable or wrong type")
                    actual = value.double_value if floating else value.integer_value
                    if not math.isclose(actual, expected, abs_tol=1e-5):
                        raise ValueError(f"PX4 {name}: current={actual:g}, expected={expected:g}")
            except Exception as exc:
                telemetry.vision_profile_ok = False
                telemetry.vision_profile_reason = str(exc)
                self.profile_retry_after = now + 2.0
            else:
                telemetry.vision_profile_ok = True
                telemetry.vision_profile_reason = "valid"
            self.profile_future = None
            return
        if now < self.profile_retry_after or not self.profile_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = list(self.ZED_PROFILE)
        self.profile_future = self.profile_client.call_async(request)
        self.profile_started = now

    def send_command(self, command):
        # Keep the transport gate even though the state machine also enforces it.
        if not self.controller.settings.enable_flight:
            return
        kind, value = command
        if kind == "mode":
            request = SetMode.Request()
            request.custom_mode = value
            client = self.mode_client
        else:
            request = CommandBool.Request()
            request.value = value
            client = self.arm_client
        if not client.service_is_ready():
            self.controller.command_result(False, self.now_s(), "service unavailable")
            return
        try:
            self.future = client.call_async(request)
        except Exception as exc:
            self.controller.command_result(False, self.now_s(), str(exc))

    def tick(self):
        now = self.now_s()
        self.service_vision_profile(now)
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                accepted = getattr(result, "success", getattr(result, "mode_sent", False))
                self.controller.command_result(accepted, now, str(result))
            except Exception as exc:
                self.controller.command_result(False, now, str(exc))
            self.future = None
        output = self.controller.tick(now)
        if output.attitude_setpoint is not None and self.controller.settings.enable_flight:
            yaw, thrust = output.attitude_setpoint
            msg = AttitudeTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            # MAVROS converts ENU/FLU orientation to NED/FRD. PX4 controls attitude.
            msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE |
                             AttitudeTarget.IGNORE_YAW_RATE)
            msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.thrust = float(thrust)
            self.attitude_pub.publish(msg)
        if output.setpoint is not None and self.controller.settings.enable_flight:
            x, y, z, yaw = output.setpoint
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.controller.origin_frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.position_pub.publish(msg)
        if output.command is not None:
            self.send_command(output.command)
        self.service_telemetry(now)
        if now - self.last_status_time >= 0.5:
            self.last_status_time = now
            status = String()
            status.data = self.controller.state
            self.status_pub.publish(status)
            status.data = self.check_callback(None, Trigger.Response()).message
            if self.controller.reason:
                status.data += "; " + self.controller.reason
            self.diagnostics_pub.publish(status)
            if status.data != self.last_diagnostics:
                self.get_logger().info(status.data)
                self.last_diagnostics = status.data


def main(args=None):
    rclpy.init(args=args)
    node = OnboardHoverDemoSequenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # No shutdown disarm: loss of setpoints invokes the configured PX4 failsafe.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
