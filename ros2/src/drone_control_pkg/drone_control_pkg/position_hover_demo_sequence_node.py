from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CompanionProcessStatus, HomePosition, ManualControl, State
from mavros_msgs.srv import CommandBool, CommandTOLLocal, ParamPull, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_control_pkg.topic_utils import cdrone_topic, join_topic


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min(value, max_v), min_v)


def normalize_hover_mode(value: str) -> str:
    normalized = str(value or "").strip().upper()
    aliases = {
        "HOLD": "AUTO.LOITER",
        "AUTO.LOITER": "AUTO.LOITER",
        "LOITER": "AUTO.LOITER",
        "POSCTL": "POSCTL",
    }
    if normalized not in aliases:
        raise ValueError(
            "hover_mode must be one of: POSCTL, HOLD, AUTO.LOITER, LOITER"
        )
    return aliases[normalized]


def normalize_takeoff_strategy(value: str) -> str:
    normalized = str(value or "").strip().upper()
    aliases = {
        "AUTO": "AUTO_MODE",
        "AUTO_MODE": "AUTO_MODE",
        "AUTO_TAKEOFF": "AUTO_MODE",
        "LOCAL": "LOCAL_COMMAND",
        "LOCAL_COMMAND": "LOCAL_COMMAND",
        "TAKEOFF_LOCAL": "LOCAL_COMMAND",
    }
    if normalized not in aliases:
        raise ValueError(
            "takeoff_strategy must be one of: AUTO_MODE, AUTO, AUTO_TAKEOFF, "
            "LOCAL_COMMAND, LOCAL, TAKEOFF_LOCAL"
        )
    return aliases[normalized]


def parameter_value_to_float(value: ParameterValue) -> float:
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    if value.double_value != 0.0:
        return float(value.double_value)
    if value.integer_value != 0:
        return float(value.integer_value)
    return 0.0


def parameter_value_is_declared_numeric(value: ParameterValue) -> bool:
    return value.type in {
        ParameterType.PARAMETER_DOUBLE,
        ParameterType.PARAMETER_INTEGER,
    }


class PositionHoverDemoSequenceNode(Node):
    TERMINAL_STATES = {"IDLE", "COMPLETE", "ABORT"}

    def __init__(self) -> None:
        super().__init__("position_hover_demo_sequence_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("takeoff_altitude_m", 0.7)
        self.declare_parameter("takeoff_rate_m_s", 0.5)
        self.declare_parameter("takeoff_strategy", "AUTO_MODE")
        self.declare_parameter("hover_duration_s", 5.0)
        self.declare_parameter("hover_mode", "HOLD")
        self.declare_parameter("altitude_tolerance_m", 0.10)
        self.declare_parameter("start_altitude_limit_m", 0.20)
        self.declare_parameter("touchdown_altitude_m", 0.15)
        self.declare_parameter("touchdown_dwell_s", 1.0)
        self.declare_parameter("stage_timeout_s", 30.0)
        self.declare_parameter("arm_zero_throttle_hold_s", 1.5)
        self.declare_parameter("manual_hover_throttle_center", 500.0)
        self.declare_parameter("local_pose_timeout_s", 0.5)
        self.declare_parameter("local_pose_timeout_during_param_sync_s", 1.0)
        self.declare_parameter("state_timeout_s", 2.0)
        self.declare_parameter("state_timeout_during_param_sync_s", 8.0)
        self.declare_parameter("connection_loss_timeout_s", 0.5)
        self.declare_parameter(
            "connection_loss_timeout_during_param_sync_s", 3.0
        )
        self.declare_parameter("mode_request_retry_interval_s", 0.5)
        self.declare_parameter("companion_status_timeout_s", 0.5)
        self.declare_parameter("require_companion_active", True)
        self.declare_parameter("max_horizontal_excursion_m", 0.75)
        self.declare_parameter("restore_takeoff_alt_on_exit", True)
        self.declare_parameter("takeoff_param_id", "MIS_TAKEOFF_ALT")
        self.declare_parameter("param_pull_force", True)
        self.declare_parameter("param_pull_retry_delay_s", 1.0)
        self.declare_parameter("param_sync_timeout_s", 60.0)
        self.declare_parameter("manual_control_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("local_pose_topic", "")
        self.declare_parameter("home_position_topic", "")
        self.declare_parameter("global_origin_topic", "")
        self.declare_parameter("companion_status_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("start_service", "")
        self.declare_parameter("abort_service", "")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.takeoff_altitude_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.takeoff_rate_m_s = float(self.get_parameter("takeoff_rate_m_s").value)
        self.takeoff_strategy = normalize_takeoff_strategy(
            str(self.get_parameter("takeoff_strategy").value)
        )
        self.hover_duration_s = float(self.get_parameter("hover_duration_s").value)
        self.hover_mode = normalize_hover_mode(
            str(self.get_parameter("hover_mode").value)
        )
        self.altitude_tolerance_m = float(
            self.get_parameter("altitude_tolerance_m").value
        )
        self.start_altitude_limit_m = float(
            self.get_parameter("start_altitude_limit_m").value
        )
        self.touchdown_altitude_m = float(
            self.get_parameter("touchdown_altitude_m").value
        )
        self.touchdown_dwell_s = float(self.get_parameter("touchdown_dwell_s").value)
        self.stage_timeout_s = float(self.get_parameter("stage_timeout_s").value)
        self.arm_zero_throttle_hold_s = float(
            self.get_parameter("arm_zero_throttle_hold_s").value
        )
        self.manual_hover_throttle_center = float(
            self.get_parameter("manual_hover_throttle_center").value
        )
        self.local_pose_timeout_s = float(
            self.get_parameter("local_pose_timeout_s").value
        )
        self.local_pose_timeout_during_param_sync_s = float(
            self.get_parameter("local_pose_timeout_during_param_sync_s").value
        )
        self.state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self.state_timeout_during_param_sync_s = float(
            self.get_parameter("state_timeout_during_param_sync_s").value
        )
        self.connection_loss_timeout_s = float(
            self.get_parameter("connection_loss_timeout_s").value
        )
        self.connection_loss_timeout_during_param_sync_s = float(
            self.get_parameter(
                "connection_loss_timeout_during_param_sync_s"
            ).value
        )
        self.mode_request_retry_interval_s = float(
            self.get_parameter("mode_request_retry_interval_s").value
        )
        self.companion_status_timeout_s = float(
            self.get_parameter("companion_status_timeout_s").value
        )
        self.require_companion_active = bool(
            self.get_parameter("require_companion_active").value
        )
        self.max_horizontal_excursion_m = float(
            self.get_parameter("max_horizontal_excursion_m").value
        )
        self.restore_takeoff_alt_on_exit = bool(
            self.get_parameter("restore_takeoff_alt_on_exit").value
        )
        self.takeoff_param_id = str(self.get_parameter("takeoff_param_id").value)
        self.param_pull_force = bool(self.get_parameter("param_pull_force").value)
        self.param_pull_retry_delay_s = float(
            self.get_parameter("param_pull_retry_delay_s").value
        )
        self.param_sync_timeout_s = float(
            self.get_parameter("param_sync_timeout_s").value
        )

        self.manual_control_topic = (
            str(self.get_parameter("manual_control_topic").value).strip()
            or join_topic(self.mavros_namespace, "manual_control/send")
        )
        self.state_topic = (
            str(self.get_parameter("state_topic").value).strip()
            or join_topic(self.mavros_namespace, "state")
        )
        self.local_pose_topic = (
            str(self.get_parameter("local_pose_topic").value).strip()
            or join_topic(self.mavros_namespace, "local_position/pose")
        )
        self.home_position_topic = (
            str(self.get_parameter("home_position_topic").value).strip()
            or join_topic(self.mavros_namespace, "home_position/home")
        )
        self.global_origin_topic = (
            str(self.get_parameter("global_origin_topic").value).strip()
            or join_topic(self.mavros_namespace, "global_position/gp_origin")
        )
        self.companion_status_topic = (
            str(self.get_parameter("companion_status_topic").value).strip()
            or join_topic(self.mavros_namespace, "companion_process/status")
        )
        self.status_topic = (
            str(self.get_parameter("status_topic").value).strip()
            or cdrone_topic(self.drone_id, "demo/position_hover_state")
        )
        self.start_service_name = (
            str(self.get_parameter("start_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/position_hover_start")
        )
        self.abort_service_name = (
            str(self.get_parameter("abort_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/position_hover_abort")
        )

        self.takeoff_mode = "AUTO.TAKEOFF"
        self.land_mode = "AUTO.LAND"

        self.latest_state = State()
        self.latest_pose = PoseStamped()
        self.latest_home_position: Optional[HomePosition] = None
        self.latest_global_origin: Optional[GeoPointStamped] = None
        self.last_state_time_s = 0.0
        self.last_pose_time_s = 0.0
        self.last_fcu_connect_time_s = 0.0
        self.last_home_position_time_s = 0.0
        self.last_global_origin_time_s = 0.0
        self.last_companion_time_s = 0.0
        self.companion_active = False

        self.demo_state = "IDLE"
        self.stage_started_s = self.now_s()
        self.abort_reason = ""
        self.takeoff_origin_xy: Optional[Tuple[float, float]] = None
        self.takeoff_origin_altitude_m: Optional[float] = None
        self.touchdown_started_s: Optional[float] = None

        self.mode_future = None
        self.pending_mode_name: Optional[str] = None
        self.arm_future = None
        self.pending_arm_value: Optional[bool] = None
        self.takeoff_future = None
        self.takeoff_request_sent = False
        self.takeoff_request_accepted = False
        self.pending_takeoff_target: Optional[Tuple[float, float, float]] = None
        self.last_takeoff_progress_log_s = 0.0
        self.param_pull_future = None
        self.param_future = None
        self.param_mirror_ready = False
        self.allow_param_lookup_without_full_mirror = True
        self.param_future_kind: Optional[str] = None
        self.pending_param_value: Optional[float] = None

        self.original_takeoff_alt_m: Optional[float] = None
        self.takeoff_param_ready = False
        self.takeoff_param_changed = False
        self.restore_complete = False
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.connection_lost_since_s: Optional[float] = None
        self.connection_loss_warned = False
        self.last_mode_request_time_s = 0.0
        self.next_param_pull_attempt_s = 0.0
        self.param_pull_attempt_count = 0

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.manual_pub = self.create_publisher(
            ManualControl, self.manual_control_topic, 10
        )

        state_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            state_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.local_pose_topic,
            self.pose_callback,
            best_effort_qos,
        )
        self.create_subscription(
            HomePosition,
            self.home_position_topic,
            self.home_position_callback,
            state_qos,
        )
        self.create_subscription(
            GeoPointStamped,
            self.global_origin_topic,
            self.global_origin_callback,
            state_qos,
        )
        self.create_subscription(
            CompanionProcessStatus,
            self.companion_status_topic,
            self.companion_status_callback,
            10,
        )

        self.mode_client = self.create_client(
            SetMode, join_topic(self.mavros_namespace, "set_mode")
        )
        self.arm_client = self.create_client(
            CommandBool, join_topic(self.mavros_namespace, "cmd/arming")
        )
        self.takeoff_client = self.create_client(
            CommandTOLLocal, join_topic(self.mavros_namespace, "cmd/takeoff_local")
        )
        self.param_get_client = self.create_client(
            GetParameters, join_topic(self.mavros_namespace, "param/get_parameters")
        )
        self.param_pull_client = self.create_client(
            ParamPull, join_topic(self.mavros_namespace, "param/pull")
        )
        self.param_set_client = self.create_client(
            SetParameters, join_topic(self.mavros_namespace, "param/set_parameters")
        )

        self.create_service(
            Trigger, self.start_service_name, self.handle_start_request
        )
        self.create_service(
            Trigger, self.abort_service_name, self.handle_abort_request
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback
        )
        self.get_logger().info(
            "Position hover demo sequence node started. "
            f"Waiting for explicit start request. "
            f"takeoff_strategy={self.takeoff_strategy}"
        )
        self.publish_status()

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def state_callback(self, msg: State) -> None:
        was_connected = bool(self.latest_state.connected)
        now_s = self.now_s()
        self.latest_state = msg
        self.last_state_time_s = now_s
        if msg.connected and not was_connected:
            self.last_fcu_connect_time_s = now_s
            self.last_home_position_time_s = 0.0
            self.last_global_origin_time_s = 0.0
        elif was_connected and not msg.connected:
            self.last_home_position_time_s = 0.0
            self.last_global_origin_time_s = 0.0

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.last_pose_time_s = self.now_s()

    def home_position_callback(self, msg: HomePosition) -> None:
        self.latest_home_position = msg
        self.last_home_position_time_s = self.now_s()

    def global_origin_callback(self, msg: GeoPointStamped) -> None:
        self.latest_global_origin = msg
        self.last_global_origin_time_s = self.now_s()

    def companion_status_callback(self, msg: CompanionProcessStatus) -> None:
        if (
            msg.component
            != CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        ):
            return
        self.last_companion_time_s = self.now_s()
        self.companion_active = (
            msg.state == CompanionProcessStatus.MAV_STATE_ACTIVE
        )

    def current_altitude_m(self) -> float:
        return float(self.latest_pose.pose.position.z)

    def current_xy(self) -> Tuple[float, float]:
        return (
            float(self.latest_pose.pose.position.x),
            float(self.latest_pose.pose.position.y),
        )

    def horizontal_excursion_m(self) -> float:
        if self.takeoff_origin_xy is None:
            return 0.0
        current_xy = self.current_xy()
        return math.hypot(
            current_xy[0] - self.takeoff_origin_xy[0],
            current_xy[1] - self.takeoff_origin_xy[1],
        )

    def takeoff_target_altitude_m(self) -> float:
        if self.takeoff_origin_altitude_m is None:
            return self.takeoff_altitude_m
        return self.takeoff_origin_altitude_m + self.takeoff_altitude_m

    def touchdown_threshold_altitude_m(self) -> float:
        if self.takeoff_origin_altitude_m is None:
            return self.touchdown_altitude_m
        return self.takeoff_origin_altitude_m + self.touchdown_altitude_m

    def current_altitude_above_home_m(self) -> Optional[float]:
        if self.latest_home_position is None:
            return None
        return self.current_altitude_m() - float(self.latest_home_position.position.z)

    def takeoff_origin_above_home_m(self) -> Optional[float]:
        if self.latest_home_position is None or self.takeoff_origin_altitude_m is None:
            return None
        return self.takeoff_origin_altitude_m - float(self.latest_home_position.position.z)

    def current_yaw_rad(self) -> float:
        orientation = self.latest_pose.pose.orientation
        siny_cosp = 2.0 * (
            (orientation.w * orientation.z) + (orientation.x * orientation.y)
        )
        cosy_cosp = 1.0 - 2.0 * (
            (orientation.y * orientation.y) + (orientation.z * orientation.z)
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_fresh(self) -> bool:
        timeout_s = self.local_pose_timeout_s
        if self.demo_state == "SYNC_TAKEOFF_PARAM":
            timeout_s = max(timeout_s, self.local_pose_timeout_during_param_sync_s)
        return (self.now_s() - self.last_pose_time_s) <= timeout_s

    def active_state_timeout_s(self) -> float:
        timeout_s = self.state_timeout_s
        if self.demo_state in {
            "SYNC_TAKEOFF_PARAM",
            "SET_TAKEOFF_MODE",
            "ARMING",
            "TAKEOFF",
        }:
            timeout_s = max(timeout_s, self.state_timeout_during_param_sync_s)
        return max(timeout_s, 0.0)

    def state_fresh(self) -> bool:
        return (self.now_s() - self.last_state_time_s) <= self.active_state_timeout_s()

    def companion_status_fresh(self) -> bool:
        return (
            self.now_s() - self.last_companion_time_s
        ) <= self.companion_status_timeout_s

    def home_position_ready(self) -> bool:
        return self.last_home_position_time_s >= self.last_fcu_connect_time_s > 0.0

    def global_origin_ready(self) -> bool:
        return self.last_global_origin_time_s >= self.last_fcu_connect_time_s > 0.0

    def active_connection_loss_timeout_s(self) -> float:
        timeout_s = self.connection_loss_timeout_s
        if self.demo_state in {
            "SYNC_TAKEOFF_PARAM",
            "SET_TAKEOFF_MODE",
            "ARMING",
            "TAKEOFF",
        }:
            timeout_s = max(
                timeout_s,
                self.connection_loss_timeout_during_param_sync_s,
            )
        return max(timeout_s, 0.0)

    def mode_matches(self, mode_name: str) -> bool:
        return str(self.latest_state.mode).upper() == str(mode_name).upper()

    def mode_request_retry_ready(self) -> bool:
        if self.last_mode_request_time_s <= 0.0:
            return True
        return (
            self.now_s() - self.last_mode_request_time_s
        ) >= self.mode_request_retry_interval_s

    def is_land_mode(self) -> bool:
        return "LAND" in str(self.latest_state.mode).upper()

    def is_takeoff_mode(self) -> bool:
        return "TAKEOFF" in str(self.latest_state.mode).upper()

    def is_takeoff_handoff_mode(self) -> bool:
        return self.mode_matches("AUTO.LOITER") or self.mode_matches(self.hover_mode)

    def publish_status(self) -> None:
        msg = String()
        msg.data = self.demo_state
        self.status_pub.publish(msg)

    def publish_manual(self, throttle_cmd: float) -> None:
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = float(clamp(throttle_cmd, 0.0, 1000.0))
        msg.r = 0.0
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def startable(self) -> Tuple[bool, str]:
        if self.demo_state not in {"IDLE", "COMPLETE", "ABORT"}:
            return False, f"demo already active in state {self.demo_state}"
        if (
            self.mode_future is not None
            or self.arm_future is not None
            or self.takeoff_future is not None
            or self.param_pull_future is not None
            or self.param_future is not None
        ):
            return False, "a prior service request is still in flight"
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return False, "mode service is unavailable"
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            return False, "arming service is unavailable"
        if (
            self.takeoff_strategy == "LOCAL_COMMAND"
            and not self.takeoff_client.wait_for_service(timeout_sec=1.0)
        ):
            return False, "local takeoff service is unavailable"
        if not self.param_get_client.wait_for_service(timeout_sec=1.0):
            return False, "param get service is unavailable"
        if not self.param_pull_client.wait_for_service(timeout_sec=1.0):
            return False, "param pull service is unavailable"
        if not self.param_set_client.wait_for_service(timeout_sec=1.0):
            return False, "param set service is unavailable"
        if not self.state_fresh():
            return False, "MAVROS state is stale or missing"
        if not self.latest_state.connected:
            return False, "FCU is not connected"
        if not self.pose_fresh():
            return False, "local pose is stale or missing"
        if not self.global_origin_ready():
            return False, "global origin is missing"
        if not self.home_position_ready():
            return False, "home position is missing"
        if self.require_companion_active:
            if not self.companion_status_fresh():
                return False, "external pose status is stale or missing"
            if not self.companion_active:
                return False, "external pose source is not active"
        if self.latest_state.armed:
            return False, "refusing to start because the vehicle is already armed"
        return True, "ready"

    def handle_start_request(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        ready, reason = self.startable()
        if not ready:
            response.success = False
            response.message = reason
            return response

        self.abort_reason = ""
        self.takeoff_origin_xy = self.current_xy()
        self.takeoff_origin_altitude_m = self.current_altitude_m()
        self.touchdown_started_s = None
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.takeoff_request_sent = False
        self.takeoff_request_accepted = False
        self.pending_takeoff_target = None
        self.last_takeoff_progress_log_s = 0.0
        self.pending_mode_name = None
        self.pending_arm_value = None
        self.pending_param_value = None
        self.param_mirror_ready = False
        self.allow_param_lookup_without_full_mirror = True
        self.original_takeoff_alt_m = None
        self.takeoff_param_ready = False
        self.takeoff_param_changed = False
        self.restore_complete = False
        self.connection_lost_since_s = None
        self.connection_loss_warned = False
        self.last_mode_request_time_s = 0.0
        self.next_param_pull_attempt_s = 0.0
        self.param_pull_attempt_count = 0

        start_above_home_m = self.takeoff_origin_above_home_m()
        if start_above_home_m is not None:
            self.get_logger().info(
                "Takeoff reference: "
                f"start_z={self.takeoff_origin_altitude_m:.2f} m "
                f"home_z={self.latest_home_position.position.z:.2f} m "
                f"start_minus_home={start_above_home_m:.2f} m"
            )

        self.transition_to("SYNC_TAKEOFF_PARAM", "start requested")
        response.success = True
        response.message = "position hover demo sequence started"
        return response

    def handle_abort_request(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self.demo_state in {"IDLE", "COMPLETE"}:
            response.success = False
            response.message = "demo is not active"
            return response

        self.enter_abort("operator abort")
        response.success = True
        response.message = "abort accepted"
        return response

    def transition_to(self, new_state: str, reason: str = "") -> None:
        old_state = self.demo_state
        self.demo_state = new_state
        self.stage_started_s = self.now_s()
        if new_state == "TAKEOFF":
            self.last_takeoff_progress_log_s = 0.0
        if new_state != "WAIT_TOUCHDOWN":
            self.touchdown_started_s = None
        if reason:
            self.get_logger().info(f"State {old_state} -> {new_state}: {reason}")
        else:
            self.get_logger().info(f"State {old_state} -> {new_state}")
        self.publish_status()

    def enter_abort(self, reason: str) -> None:
        if self.demo_state == "ABORT":
            return
        self.abort_reason = reason
        self.get_logger().error(f"Aborting demo: {reason}")
        self.transition_to("ABORT", reason)

    def stage_elapsed_s(self) -> float:
        return self.now_s() - self.stage_started_s

    def stage_timeout_limit_s(self) -> Optional[float]:
        if self.demo_state in self.TERMINAL_STATES:
            return None
        if self.demo_state == "HOVER":
            return max(self.stage_timeout_s, self.hover_duration_s + 2.0)
        if self.demo_state == "SYNC_TAKEOFF_PARAM":
            return max(self.stage_timeout_s, self.param_sync_timeout_s)
        if self.demo_state in {"WAIT_TOUCHDOWN", "DISARMING"}:
            return max(self.stage_timeout_s, 25.0)
        return self.stage_timeout_s

    def request_mode(self, mode_name: str) -> None:
        if self.mode_future is not None:
            return
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort(f"mode service unavailable for {mode_name}")
            return
        req = SetMode.Request()
        req.custom_mode = mode_name
        self.pending_mode_name = mode_name
        self.mode_future = self.mode_client.call_async(req)
        self.last_mode_request_time_s = self.now_s()
        self.get_logger().info(f"Requested mode {mode_name}")

    def request_arm(self, arm_value: bool) -> None:
        if self.arm_future is not None:
            return
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort("arming service unavailable")
            return
        req = CommandBool.Request()
        req.value = arm_value
        self.pending_arm_value = arm_value
        self.arm_future = self.arm_client.call_async(req)
        action = "arm" if arm_value else "disarm"
        self.get_logger().info(f"Requested {action}")

    def request_takeoff(self) -> None:
        if self.takeoff_future is not None:
            return
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort("local takeoff service unavailable")
            return

        current_xy = self.current_xy()
        target_altitude_m = self.takeoff_target_altitude_m()
        req = CommandTOLLocal.Request()
        req.min_pitch = 0.0
        req.offset = 0.0
        req.rate = float(max(self.takeoff_rate_m_s, 0.05))
        req.yaw = self.current_yaw_rad()
        req.position.x = float(current_xy[0])
        req.position.y = float(current_xy[1])
        req.position.z = float(target_altitude_m)

        self.pending_takeoff_target = (
            float(current_xy[0]),
            float(current_xy[1]),
            float(target_altitude_m),
        )
        self.takeoff_future = self.takeoff_client.call_async(req)
        self.get_logger().info(
            "Requested local takeoff to "
            f"x={req.position.x:.2f} y={req.position.y:.2f} z={req.position.z:.2f} "
            f"at {req.rate:.2f} m/s"
        )

    def request_param_get(self) -> None:
        if self.param_future is not None:
            return
        if not self.param_get_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort(f"param get service unavailable for {self.takeoff_param_id}")
            return
        self.param_future_kind = "GET_ORIGINAL"
        req = GetParameters.Request()
        req.names = [self.takeoff_param_id]
        self.param_future = self.param_get_client.call_async(req)
        self.get_logger().info(f"Requested current {self.takeoff_param_id}")

    def request_param_pull(self) -> None:
        if self.param_pull_future is not None:
            return
        if not self.param_pull_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort(f"param pull service unavailable for {self.takeoff_param_id}")
            return
        req = ParamPull.Request()
        req.force_pull = self.param_pull_force
        self.param_pull_future = self.param_pull_client.call_async(req)
        self.param_pull_attempt_count += 1
        self.get_logger().info(
            "Requested FCU parameter pull "
            f"(attempt {self.param_pull_attempt_count})"
        )

    def schedule_param_pull_retry(self, reason: str) -> None:
        delay_s = max(self.param_pull_retry_delay_s, 0.0)
        self.param_mirror_ready = False
        self.allow_param_lookup_without_full_mirror = False
        self.next_param_pull_attempt_s = self.now_s() + delay_s
        if delay_s > 0.0:
            self.get_logger().warn(f"{reason}; retrying param pull in {delay_s:.1f}s")
        else:
            self.get_logger().warn(f"{reason}; retrying param pull immediately")

    def request_param_set(self, value: float, *, kind: str) -> None:
        if self.param_future is not None:
            return
        if not self.param_set_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort(f"param set service unavailable for {self.takeoff_param_id}")
            return
        self.param_future_kind = kind
        self.pending_param_value = float(value)
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name=self.takeoff_param_id,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(value),
                ),
            )
        ]
        self.param_future = self.param_set_client.call_async(req)
        self.get_logger().info(
            f"Requested {self.takeoff_param_id}={float(value):.2f}"
        )

    def poll_service_futures(self) -> None:
        if self.mode_future is not None and self.mode_future.done():
            mode_name = self.pending_mode_name or "unknown"
            try:
                response = self.mode_future.result()
                if not response.mode_sent:
                    self.enter_abort(f"mode change to {mode_name} was rejected")
                else:
                    self.get_logger().info(f"Mode request accepted for {mode_name}")
            except Exception as exc:
                self.enter_abort(f"mode change to {mode_name} failed: {exc}")
            finally:
                self.mode_future = None
                self.pending_mode_name = None

        if self.arm_future is not None and self.arm_future.done():
            arm_value = bool(self.pending_arm_value)
            action = "arm" if arm_value else "disarm"
            try:
                response = self.arm_future.result()
                if not response.success:
                    self.enter_abort(
                        f"{action} request was rejected "
                        f"(result {response.result}, mode={self.latest_state.mode})"
                    )
                else:
                    self.get_logger().info(f"{action.capitalize()} request accepted")
            except Exception as exc:
                self.enter_abort(f"{action} request failed: {exc}")
            finally:
                self.arm_future = None
                self.pending_arm_value = None

        if self.takeoff_future is not None and self.takeoff_future.done():
            try:
                response = self.takeoff_future.result()
                if not response.success:
                    self.enter_abort(
                        "local takeoff request was rejected "
                        f"(result {response.result})"
                    )
                else:
                    self.takeoff_request_accepted = True
                    target = self.pending_takeoff_target
                    if target is None:
                        self.get_logger().info("Local takeoff request accepted")
                    else:
                        self.get_logger().info(
                            "Local takeoff request accepted for "
                            f"x={target[0]:.2f} y={target[1]:.2f} z={target[2]:.2f}"
                        )
            except Exception as exc:
                self.enter_abort(f"local takeoff request failed: {exc}")
            finally:
                self.takeoff_future = None

        if self.param_pull_future is not None and self.param_pull_future.done():
            try:
                response = self.param_pull_future.result()
                if not response.success or response.param_received <= 0:
                    failure_reason = (
                        "reported failure"
                        if not response.success
                        else "returned no parameters"
                    )
                    self.schedule_param_pull_retry(
                        f"FCU parameter pull failed for {self.takeoff_param_id} "
                        f"({failure_reason})"
                    )
                else:
                    self.get_logger().info(
                        f"FCU parameter pull complete: {response.param_received} params"
                    )
                    self.param_mirror_ready = True
                    self.next_param_pull_attempt_s = 0.0
                self.allow_param_lookup_without_full_mirror = True
            except Exception as exc:
                self.schedule_param_pull_retry(
                    f"FCU parameter pull raised for {self.takeoff_param_id}: {exc}"
                )
            finally:
                self.param_pull_future = None

        if self.param_future is not None and self.param_future.done():
            future_kind = self.param_future_kind or "unknown"
            try:
                response = self.param_future.result()
                if future_kind == "GET_ORIGINAL":
                    if not response.values:
                        self.original_takeoff_alt_m = None
                        self.schedule_param_pull_retry(
                            f"{self.takeoff_param_id} not returned yet"
                        )
                        return
                    if not parameter_value_is_declared_numeric(response.values[0]):
                        self.original_takeoff_alt_m = None
                        self.schedule_param_pull_retry(
                            f"{self.takeoff_param_id} is not declared on MAVROS yet"
                        )
                        return
                    self.original_takeoff_alt_m = parameter_value_to_float(
                        response.values[0]
                    )
                    self.get_logger().info(
                        f"{self.takeoff_param_id} currently "
                        f"{self.original_takeoff_alt_m:.2f}"
                    )
                    if math.isclose(
                        self.original_takeoff_alt_m,
                        self.takeoff_altitude_m,
                        abs_tol=1e-3,
                    ):
                        self.takeoff_param_ready = True
                elif future_kind == "SET_TARGET":
                    if not response.results or not response.results[0].successful:
                        reason = (
                            response.results[0].reason
                            if response.results
                            else "no response"
                        )
                        if "undeclared" in str(reason).lower():
                            self.original_takeoff_alt_m = None
                            self.schedule_param_pull_retry(
                                f"{self.takeoff_param_id} not declared yet during set"
                            )
                            self.takeoff_param_ready = False
                            self.takeoff_param_changed = False
                            return
                        self.enter_abort(
                            f"failed to set {self.takeoff_param_id}: {reason}"
                        )
                        return
                    self.takeoff_param_ready = True
                    self.takeoff_param_changed = True
                    self.get_logger().info(
                        f"Set {self.takeoff_param_id} to "
                        f"{self.pending_param_value:.2f}"
                    )
                elif future_kind == "RESTORE_ORIGINAL":
                    if not response.results or not response.results[0].successful:
                        reason = (
                            response.results[0].reason
                            if response.results
                            else "no response"
                        )
                        if "undeclared" in str(reason).lower():
                            self.schedule_param_pull_retry(
                                f"{self.takeoff_param_id} not declared yet during restore"
                            )
                            self.restore_complete = False
                            return
                        self.enter_abort(
                            f"failed to restore {self.takeoff_param_id}: {reason}"
                        )
                        return
                    self.restore_complete = True
                    self.takeoff_param_changed = False
                    self.get_logger().info(
                        f"Restored {self.takeoff_param_id} to "
                        f"{self.pending_param_value:.2f}"
                    )
            except Exception as exc:
                self.enter_abort(
                    f"{future_kind} request raised for {self.takeoff_param_id}: {exc}"
                )
            finally:
                if self.param_future is not None and self.param_future.done():
                    self.param_future = None
                    self.param_future_kind = None
                    self.pending_param_value = None

    def run_safety_checks(self) -> None:
        if self.demo_state in self.TERMINAL_STATES:
            return

        if not self.state_fresh():
            self.enter_abort("lost MAVROS state updates")
            return
        if self.latest_state.connected:
            if self.connection_lost_since_s is not None and self.connection_loss_warned:
                self.get_logger().info(
                    "FCU connection recovered after "
                    f"{self.now_s() - self.connection_lost_since_s:.2f}s"
                )
            self.connection_lost_since_s = None
            self.connection_loss_warned = False
        else:
            now_s = self.now_s()
            if self.connection_lost_since_s is None:
                self.connection_lost_since_s = now_s
            disconnect_elapsed_s = now_s - self.connection_lost_since_s
            disconnect_timeout_s = self.active_connection_loss_timeout_s()
            if not self.connection_loss_warned:
                self.get_logger().warn(
                    "MAVROS reported FCU disconnected; allowing up to "
                    f"{disconnect_timeout_s:.1f}s for recovery "
                    f"while in {self.demo_state}"
                )
                self.connection_loss_warned = True
            if disconnect_elapsed_s > disconnect_timeout_s:
                self.enter_abort(
                    f"FCU disconnected for {disconnect_elapsed_s:.2f}s"
                )
            return
        if not self.pose_fresh():
            pose_age_s = self.now_s() - self.last_pose_time_s
            self.enter_abort(
                f"lost local pose updates (age {pose_age_s:.2f}s)"
            )
            return
        if self.require_companion_active:
            if not self.companion_status_fresh():
                self.enter_abort("external pose companion status went stale")
                return
            if not self.companion_active:
                self.enter_abort("external pose companion status is not active")
                return
        if (
            self.takeoff_origin_xy is not None
            and self.latest_state.armed
            and self.max_horizontal_excursion_m > 0.0
            and self.horizontal_excursion_m() > self.max_horizontal_excursion_m
        ):
            self.enter_abort(
                f"horizontal excursion exceeded "
                f"{self.max_horizontal_excursion_m:.2f} m"
            )
            return
        if (
            self.demo_state in {"SET_HOVER_MODE", "HOVER"}
            and self.hover_mode == "POSCTL"
            and self.mode_future is None
            and not self.mode_matches("POSCTL")
        ):
            self.enter_abort(
                f"mode mismatch during {self.demo_state}: {self.latest_state.mode}"
            )
            return
        if (
            self.demo_state == "TAKEOFF"
            and not self.latest_state.armed
        ):
            if self.arm_request_sent and self.mode_future is None:
                self.enter_abort(
                    f"vehicle disarmed during takeoff while mode={self.latest_state.mode}"
                )
            else:
                self.enter_abort(
                    f"vehicle is not armed in TAKEOFF while mode={self.latest_state.mode}"
                )
            return
        if (
            self.demo_state == "TAKEOFF"
            and self.latest_state.armed
            and self.mode_future is None
            and not self.is_takeoff_mode()
        ):
            if not self.is_takeoff_handoff_mode():
                self.enter_abort(
                    f"lost takeoff mode during climb: {self.latest_state.mode}"
                )
                return

        timeout_limit = self.stage_timeout_limit_s()
        if timeout_limit is not None and self.stage_elapsed_s() > timeout_limit:
            self.enter_abort(f"stage timeout in {self.demo_state}")

    def step_state_machine(self) -> None:
        if self.demo_state == "IDLE":
            return

        if self.demo_state == "SYNC_TAKEOFF_PARAM":
            if self.takeoff_param_ready:
                self.transition_to("SET_TAKEOFF_MODE", "takeoff altitude configured")
            elif (
                self.param_future is None
                and (
                    self.param_mirror_ready
                    or self.allow_param_lookup_without_full_mirror
                )
            ):
                if self.original_takeoff_alt_m is None:
                    self.request_param_get()
                else:
                    self.request_param_set(
                        self.takeoff_altitude_m,
                        kind="SET_TARGET",
                    )
            elif (
                self.param_pull_future is None
                and not self.param_mirror_ready
                and self.now_s() >= self.next_param_pull_attempt_s
            ):
                self.request_param_pull()
            return

        if self.demo_state == "SET_TAKEOFF_MODE":
            if self.is_takeoff_mode():
                self.transition_to("ARMING", "AUTO.TAKEOFF confirmed")
            elif self.mode_future is None and self.mode_request_retry_ready():
                self.request_mode(self.takeoff_mode)
            return

        if self.demo_state == "ARMING":
            if self.latest_state.armed:
                if self.takeoff_strategy == "LOCAL_COMMAND":
                    self.transition_to("REQUEST_TAKEOFF", "vehicle armed")
                else:
                    self.transition_to(
                        "TAKEOFF",
                        "vehicle armed; waiting for PX4 AUTO.TAKEOFF climb",
                    )
                return
            if (
                not self.is_takeoff_mode()
                and self.mode_future is None
                and self.mode_request_retry_ready()
            ):
                self.request_mode(self.takeoff_mode)
                return
            if (
                self.stage_elapsed_s() >= self.arm_zero_throttle_hold_s
                and not self.arm_request_sent
            ):
                self.arm_request_sent = True
                self.request_arm(True)
            return

        if self.demo_state == "REQUEST_TAKEOFF":
            if not self.is_takeoff_mode():
                if self.mode_future is None:
                    self.request_mode(self.takeoff_mode)
                return
            if self.takeoff_request_accepted:
                self.transition_to("TAKEOFF", "local takeoff command accepted")
                return
            if not self.takeoff_request_sent and self.takeoff_future is None:
                self.takeoff_request_sent = True
                self.request_takeoff()
            return

        if self.demo_state == "TAKEOFF":
            current_altitude_m = self.current_altitude_m()
            target_altitude_m = self.takeoff_target_altitude_m()
            altitude_delta_m = current_altitude_m - (
                self.takeoff_origin_altitude_m
                if self.takeoff_origin_altitude_m is not None
                else 0.0
            )
            current_above_home_m = self.current_altitude_above_home_m()
            if (
                self.last_takeoff_progress_log_s == 0.0
                or (self.now_s() - self.last_takeoff_progress_log_s) >= 1.0
            ):
                progress_message = (
                    "Takeoff progress: "
                    f"delta_z={altitude_delta_m:.2f} m "
                    f"target_delta_z={self.takeoff_altitude_m:.2f} m "
                    f"mode={self.latest_state.mode} "
                    f"armed={self.latest_state.armed}"
                )
                if current_above_home_m is not None:
                    progress_message += (
                        f" local_minus_home={current_above_home_m:.2f} m"
                    )
                self.get_logger().info(progress_message)
                self.last_takeoff_progress_log_s = self.now_s()
            if current_altitude_m >= (target_altitude_m - self.altitude_tolerance_m):
                if self.mode_matches(self.hover_mode):
                    self.transition_to("HOVER", "target altitude reached")
                else:
                    self.transition_to("SET_HOVER_MODE", "target altitude reached")
                return
            if self.is_takeoff_handoff_mode():
                shortfall_m = max(target_altitude_m - current_altitude_m, 0.0)
                start_above_home_m = self.takeoff_origin_above_home_m()
                if shortfall_m > self.altitude_tolerance_m:
                    warn_message = (
                        "PX4 exited AUTO.TAKEOFF before the demo observed the "
                        f"target altitude; shortfall={shortfall_m:.2f} m "
                        f"mode={self.latest_state.mode}"
                    )
                    if current_above_home_m is not None and start_above_home_m is not None:
                        warn_message += (
                            f" local_minus_home={current_above_home_m:.2f} m"
                            f" start_minus_home={start_above_home_m:.2f} m"
                        )
                    warn_message += ". Accepting hover handoff."
                    self.get_logger().warn(warn_message)
                if self.mode_matches(self.hover_mode):
                    self.transition_to(
                        "HOVER",
                        f"PX4 handed off to {self.latest_state.mode} during takeoff",
                    )
                else:
                    self.transition_to(
                        "SET_HOVER_MODE",
                        f"PX4 handed off to {self.latest_state.mode} during takeoff",
                    )
            return

        if self.demo_state == "SET_HOVER_MODE":
            if self.mode_matches(self.hover_mode):
                self.transition_to("HOVER", "hover mode confirmed")
            elif self.mode_future is None and self.mode_request_retry_ready():
                self.request_mode(self.hover_mode)
            return

        if self.demo_state == "HOVER":
            if self.stage_elapsed_s() >= self.hover_duration_s:
                self.transition_to("SET_LAND_MODE", "hover complete")
            return

        if self.demo_state == "SET_LAND_MODE":
            if self.is_land_mode():
                self.transition_to("WAIT_TOUCHDOWN", "AUTO.LAND confirmed")
            elif self.mode_future is None and self.mode_request_retry_ready():
                self.request_mode(self.land_mode)
            return

        if self.demo_state == "WAIT_TOUCHDOWN":
            if not self.latest_state.armed:
                if self.takeoff_param_changed and self.restore_takeoff_alt_on_exit:
                    self.transition_to(
                        "RESTORE_TAKEOFF_PARAM",
                        "vehicle auto-disarmed after landing",
                    )
                else:
                    self.transition_to("COMPLETE", "vehicle auto-disarmed after landing")
                return
            if self.current_altitude_m() <= self.touchdown_threshold_altitude_m():
                if self.touchdown_started_s is None:
                    self.touchdown_started_s = self.now_s()
                elif (
                    self.now_s() - self.touchdown_started_s
                ) >= self.touchdown_dwell_s:
                    self.transition_to("DISARMING", "touchdown detected")
            else:
                self.touchdown_started_s = None
            return

        if self.demo_state == "DISARMING":
            if not self.latest_state.armed:
                if self.takeoff_param_changed and self.restore_takeoff_alt_on_exit:
                    self.transition_to("RESTORE_TAKEOFF_PARAM", "vehicle disarmed")
                else:
                    self.transition_to("COMPLETE", "vehicle disarmed")
                return
            if not self.disarm_request_sent:
                self.disarm_request_sent = True
                self.request_arm(False)
            return

        if self.demo_state == "RESTORE_TAKEOFF_PARAM":
            if not self.restore_takeoff_alt_on_exit:
                next_state = "ABORT" if self.abort_reason else "COMPLETE"
                self.transition_to(next_state, "skipping takeoff-altitude restore")
                return
            if not self.takeoff_param_changed or self.original_takeoff_alt_m is None:
                next_state = "ABORT" if self.abort_reason else "COMPLETE"
                self.transition_to(next_state, "takeoff altitude already restored")
                return
            if self.restore_complete:
                next_state = "ABORT" if self.abort_reason else "COMPLETE"
                reason = "abort cleanup complete" if self.abort_reason else "demo complete"
                self.transition_to(next_state, reason)
                return
            if (
                self.param_pull_future is None
                and not self.param_mirror_ready
                and self.now_s() >= self.next_param_pull_attempt_s
            ):
                self.request_param_pull()
                return
            if self.param_pull_future is not None:
                return
            if self.param_future is None:
                self.request_param_set(
                    self.original_takeoff_alt_m,
                    kind="RESTORE_ORIGINAL",
                )
            return

        if self.demo_state == "ABORT":
            if self.latest_state.connected and self.latest_state.armed:
                if (
                    not self.is_land_mode()
                    and self.mode_future is None
                    and self.mode_request_retry_ready()
                ):
                    self.request_mode(self.land_mode)
                return
            if self.takeoff_param_changed and self.restore_takeoff_alt_on_exit:
                if self.restore_complete:
                    return
                if (
                    self.param_pull_future is None
                    and not self.param_mirror_ready
                    and self.now_s() >= self.next_param_pull_attempt_s
                ):
                    self.request_param_pull()
                    return
                if self.param_pull_future is not None:
                    return
                if self.param_future is None and self.original_takeoff_alt_m is not None:
                    self.request_param_set(
                        self.original_takeoff_alt_m,
                        kind="RESTORE_ORIGINAL",
                    )
            return

    def publish_manual_for_state(self) -> None:
        if self.demo_state == "ARMING":
            self.publish_manual(throttle_cmd=0.0)
            return
        if self.hover_mode != "POSCTL":
            return
        if self.demo_state in {"SET_HOVER_MODE", "HOVER"}:
            self.publish_manual(self.manual_hover_throttle_center)

    def timer_callback(self) -> None:
        self.poll_service_futures()
        self.run_safety_checks()
        self.step_state_machine()
        self.publish_manual_for_state()
        self.publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PositionHoverDemoSequenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
