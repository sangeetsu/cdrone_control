from __future__ import annotations

import math
import socket
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from drone_monitor_pkg.topic_utils import cdrone_ns, cdrone_topic, join_topic
from drone_msgs.msg import (
    EngagementState,
    FlightStatus,
    PeerState,
    PerceptionStatus,
    SystemAlert,
    TargetTrackArray,
)


@dataclass
class AlertRecord:
    severity: int
    message: str
    active: bool = False


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class TelemetryAggregatorNode(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_aggregator_node")

        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("hostname", socket.gethostname())
        self.declare_parameter("self_ip", "127.0.0.1")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("dashboard_rate_hz", 5.0)
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("battery_low_threshold_pct", 0.20)
        self.declare_parameter("vio_timeout_s", 0.5)
        self.declare_parameter("tracks_timeout_s", 1.0)
        self.declare_parameter("mavros_timeout_s", 1.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("autonomy_enable_topic", "")
        self.declare_parameter("estop_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("perception_status_topic", "")
        self.declare_parameter("engagement_state_topic", "")
        self.declare_parameter("obstacle_blocked_topic", "")
        self.declare_parameter("flight_status_topic", "")
        self.declare_parameter("alerts_topic", "")
        self.declare_parameter("peer_state_topic", "")

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.hostname = str(self.get_parameter("hostname").value)
        self.self_ip = str(self.get_parameter("self_ip").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.mavros_ns = str(self.get_parameter("mavros_namespace").value)
        self.battery_low_threshold_pct = float(
            self.get_parameter("battery_low_threshold_pct").value
        )
        self.vio_timeout_s = float(self.get_parameter("vio_timeout_s").value)
        self.tracks_timeout_s = float(self.get_parameter("tracks_timeout_s").value)
        self.mavros_timeout_s = float(self.get_parameter("mavros_timeout_s").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)

        self.autonomy_enable_topic = self._topic_param(
            "autonomy_enable_topic", cdrone_topic(self.drone_id, "control/autonomy_enable")
        )
        self.estop_topic = self._topic_param(
            "estop_topic", cdrone_topic(self.drone_id, "safety/estop")
        )
        self.cmd_vel_topic = self._topic_param(
            "cmd_vel_topic", cdrone_topic(self.drone_id, "control/cmd_vel_body")
        )
        self.tracks_topic = self._topic_param(
            "tracks_topic", cdrone_topic(self.drone_id, "perception/tracks")
        )
        self.perception_status_topic = self._topic_param(
            "perception_status_topic", cdrone_topic(self.drone_id, "perception/status")
        )
        self.engagement_state_topic = self._topic_param(
            "engagement_state_topic", cdrone_topic(self.drone_id, "engagement/state")
        )
        self.obstacle_blocked_topic = self._topic_param(
            "obstacle_blocked_topic",
            cdrone_topic(self.drone_id, "perception/obstacle_blocked"),
        )
        self.flight_status_topic = self._topic_param(
            "flight_status_topic", cdrone_topic(self.drone_id, "monitor/flight_status")
        )
        self.alerts_topic = self._topic_param(
            "alerts_topic", cdrone_topic(self.drone_id, "monitor/alerts")
        )
        self.peer_state_topic = self._topic_param(
            "peer_state_topic", cdrone_topic(self.drone_id, "swarm/self_state")
        )

        self.latest_state = State()
        self.latest_pose = PoseStamped()
        self.latest_velocity = TwistStamped()
        self.latest_cmd = TwistStamped()
        self.latest_battery = BatteryState()
        self.latest_engagement = EngagementState()
        self.latest_perception_status = PerceptionStatus()
        self.latest_tracks = TargetTrackArray()
        self.autonomy_enabled = False
        self.estop = False
        self.obstacle_blocked = False

        self.last_state_s: Optional[float] = None
        self.last_pose_s: Optional[float] = None
        self.last_velocity_s: Optional[float] = None
        self.last_cmd_s: Optional[float] = None
        self.last_battery_s: Optional[float] = None
        self.last_engagement_s: Optional[float] = None
        self.last_tracks_s: Optional[float] = None
        self.last_perception_status_s: Optional[float] = None
        self.last_vio_s: Optional[float] = None
        self.alerts: Dict[str, AlertRecord] = {}

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(State, self._mavros("state"), self.state_callback, 10)
        self.create_subscription(
            PoseStamped,
            self._mavros("local_position/pose"),
            self.pose_callback,
            best_effort,
        )
        self.create_subscription(
            TwistStamped,
            self._mavros("local_position/velocity_body"),
            self.velocity_callback,
            best_effort,
        )
        self.create_subscription(
            PoseStamped,
            self._mavros("vision_pose/pose"),
            self.vio_pose_callback,
            10,
        )
        self.create_subscription(
            BatteryState,
            self._mavros("battery"),
            self.battery_callback,
            best_effort,
        )
        self.create_subscription(Bool, self.autonomy_enable_topic, self.autonomy_enable_callback, 10)
        self.create_subscription(Bool, self.estop_topic, self.estop_callback, 10)
        self.create_subscription(Bool, self.obstacle_blocked_topic, self.obstacle_callback, 10)
        self.create_subscription(TwistStamped, self.cmd_vel_topic, self.cmd_callback, 10)
        self.create_subscription(TargetTrackArray, self.tracks_topic, self.tracks_callback, 10)
        self.create_subscription(
            PerceptionStatus,
            self.perception_status_topic,
            self.perception_status_callback,
            10,
        )
        self.create_subscription(
            EngagementState,
            self.engagement_state_topic,
            self.engagement_state_callback,
            10,
        )

        self.flight_status_pub = self.create_publisher(FlightStatus, self.flight_status_topic, 10)
        self.alert_pub = self.create_publisher(SystemAlert, self.alerts_topic, 20)
        self.peer_state_pub = self.create_publisher(PeerState, self.peer_state_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.publish_loop)

        self.get_logger().info(
            f"Telemetry aggregator started for {cdrone_ns(self.drone_id)} on {self.hostname}."
        )

    def _topic_param(self, param_name: str, default: str) -> str:
        value = str(self.get_parameter(param_name).value).strip()
        return value or default

    def _mavros(self, leaf: str) -> str:
        return join_topic(self.mavros_ns, leaf)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def state_callback(self, msg: State) -> None:
        self.latest_state = msg
        self.last_state_s = self.now_s()

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.last_pose_s = self.now_s()

    def velocity_callback(self, msg: TwistStamped) -> None:
        self.latest_velocity = msg
        self.last_velocity_s = self.now_s()

    def vio_pose_callback(self, _msg: PoseStamped) -> None:
        self.last_vio_s = self.now_s()

    def battery_callback(self, msg: BatteryState) -> None:
        self.latest_battery = msg
        self.last_battery_s = self.now_s()

    def autonomy_enable_callback(self, msg: Bool) -> None:
        self.autonomy_enabled = bool(msg.data)

    def estop_callback(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def obstacle_callback(self, msg: Bool) -> None:
        self.obstacle_blocked = bool(msg.data)

    def cmd_callback(self, msg: TwistStamped) -> None:
        self.latest_cmd = msg
        self.last_cmd_s = self.now_s()

    def tracks_callback(self, msg: TargetTrackArray) -> None:
        self.latest_tracks = msg
        self.last_tracks_s = self.now_s()

    def perception_status_callback(self, msg: PerceptionStatus) -> None:
        self.latest_perception_status = msg
        self.last_perception_status_s = self.now_s()

    def engagement_state_callback(self, msg: EngagementState) -> None:
        self.latest_engagement = msg
        self.last_engagement_s = self.now_s()

    def _age(self, value: Optional[float], fallback: float = 9999.0) -> float:
        if value is None:
            return fallback
        return max(0.0, self.now_s() - value)

    def _update_alert(self, code: str, active: bool, severity: int, message: str) -> None:
        record = self.alerts.get(code)
        changed = record is None or record.active != active or record.message != message
        if record is None:
            record = AlertRecord(severity=severity, message=message, active=active)
            self.alerts[code] = record
        else:
            record.severity = severity
            record.message = message
            record.active = active

        if not changed and not active:
            return

        alert = SystemAlert()
        alert.stamp = self.get_clock().now().to_msg()
        alert.drone_id = self.drone_id
        alert.severity = severity
        alert.code = code
        alert.message = message
        alert.latched = bool(active)
        self.alert_pub.publish(alert)

    def publish_loop(self) -> None:
        status = FlightStatus()
        status.stamp = self.get_clock().now().to_msg()
        status.drone_id = self.drone_id
        status.hostname = self.hostname
        status.connected = bool(self.latest_state.connected)
        status.armed = bool(self.latest_state.armed)
        status.guided = bool(self.latest_state.guided)
        status.mode = self.latest_state.mode
        status.autonomy_enabled = bool(self.autonomy_enabled)
        status.estop = bool(self.estop)
        status.obstacle_blocked = bool(
            self.obstacle_blocked
            or getattr(self.latest_engagement, "obstacle_blocked", False)
        )
        status.min_distance_gate_active = bool(
            getattr(self.latest_engagement, "min_distance_gate_active", False)
        )
        status.target_visible = bool(getattr(self.latest_engagement, "target_visible", False))
        status.watchdog_healthy = self._age(self.last_cmd_s) <= self.cmd_timeout_s

        pos = self.latest_pose.pose.position
        ori = self.latest_pose.pose.orientation
        vel = self.latest_velocity.twist.linear
        cmd = self.latest_cmd.twist
        roll, pitch, yaw = quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)

        status.altitude_m = float(pos.z)
        status.x_m = float(pos.x)
        status.y_m = float(pos.y)
        status.z_m = float(pos.z)
        status.roll_rad = roll
        status.pitch_rad = pitch
        status.yaw_rad = yaw
        status.vx_mps = float(vel.x)
        status.vy_mps = float(vel.y)
        status.vz_mps = float(vel.z)
        status.cmd_vx_mps = float(cmd.linear.x)
        status.cmd_vy_mps = float(cmd.linear.y)
        status.cmd_vz_mps = float(cmd.linear.z)
        status.cmd_yaw_rate_rps = float(cmd.angular.z)
        status.behavior_state = self.latest_engagement.state or "UNKNOWN"
        status.active_track_id = (
            int(self.latest_engagement.active_track_id)
            if self.last_engagement_s is not None
            else -1
        )
        status.target_distance_m = (
            float(self.latest_engagement.active_distance_m)
            if self.last_engagement_s is not None
            else -1.0
        )
        status.target_bearing_rad = float(self.latest_engagement.active_bearing_rad)
        status.battery_voltage_v = float(self.latest_battery.voltage)
        status.battery_remaining_pct = float(self.latest_battery.percentage)
        status.vio_age_s = float(self._age(self.last_vio_s))
        status.tracks_age_s = float(self._age(self.last_tracks_s))
        status.mavros_age_s = float(min(self._age(self.last_state_s), self._age(self.last_pose_s)))
        status.cmd_age_s = float(self._age(self.last_cmd_s))
        status.perception_fps = float(self.latest_perception_status.tracker_fps)
        status.inference_latency_ms = float(self.latest_perception_status.inference_latency_ms)

        blocked_reason = str(getattr(self.latest_engagement, "blocked_reason", ""))
        autonomy_ready = bool(getattr(self.latest_engagement, "autonomy_ready", False))
        if status.estop:
            blocked_reason = blocked_reason or "estop"
        elif status.vio_age_s > self.vio_timeout_s:
            blocked_reason = blocked_reason or "stale_vio"
        elif status.mavros_age_s > self.mavros_timeout_s or not status.connected:
            blocked_reason = blocked_reason or "mavros_unavailable"
        elif status.autonomy_enabled and status.mode != "OFFBOARD":
            blocked_reason = blocked_reason or "offboard_rejected"

        status.autonomy_ready = bool(autonomy_ready and not blocked_reason)
        status.autonomy_blocked = bool(blocked_reason)
        status.autonomy_blocked_reason = blocked_reason
        self.flight_status_pub.publish(status)

        peer = PeerState()
        peer.stamp = status.stamp
        peer.drone_id = self.drone_id
        peer.hostname = self.hostname
        peer.self_ip = self.self_ip
        peer.connected = status.connected
        peer.armed = status.armed
        peer.autonomy_enabled = status.autonomy_enabled
        peer.mode = status.mode
        peer.x_m = status.x_m
        peer.y_m = status.y_m
        peer.z_m = status.z_m
        peer.vx_mps = status.vx_mps
        peer.vy_mps = status.vy_mps
        peer.vz_mps = status.vz_mps
        self.peer_state_pub.publish(peer)

        self._update_alert(
            "stale_vio",
            status.vio_age_s > self.vio_timeout_s,
            SystemAlert.SEVERITY_ERROR,
            f"VIO stale: {status.vio_age_s:.2f}s",
        )
        self._update_alert(
            "stale_tracks",
            status.tracks_age_s > self.tracks_timeout_s,
            SystemAlert.SEVERITY_WARN,
            f"Tracks stale: {status.tracks_age_s:.2f}s",
        )
        self._update_alert(
            "offboard_rejected",
            status.autonomy_enabled and status.mode != "OFFBOARD",
            SystemAlert.SEVERITY_ERROR,
            f"Autonomy enabled but mode is {status.mode or 'UNKNOWN'}",
        )
        self._update_alert(
            "autonomy_blocked",
            status.autonomy_blocked,
            SystemAlert.SEVERITY_ERROR,
            f"Autonomy blocked: {status.autonomy_blocked_reason or 'unknown'}",
        )
        self._update_alert(
            "min_distance_gate",
            status.min_distance_gate_active,
            SystemAlert.SEVERITY_WARN,
            "Minimum distance gate active",
        )
        self._update_alert(
            "target_lost",
            status.behavior_state == "LOST_TARGET_HOLD",
            SystemAlert.SEVERITY_WARN,
            "Target lost, holding position",
        )
        low_battery = (
            self.last_battery_s is not None
            and status.battery_remaining_pct >= 0.0
            and status.battery_remaining_pct <= self.battery_low_threshold_pct
        )
        self._update_alert(
            "battery_low",
            low_battery,
            SystemAlert.SEVERITY_WARN,
            f"Battery low: {status.battery_remaining_pct * 100.0:.0f}%",
        )
        self._update_alert(
            "estop",
            status.estop,
            SystemAlert.SEVERITY_FATAL,
            "Emergency stop asserted",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
