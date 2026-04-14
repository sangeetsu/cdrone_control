from __future__ import annotations

import math
import os
from typing import Optional, Tuple

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CompanionProcessStatus, HomePosition, State
from mavros_msgs.srv import CommandBool, CommandTOLLocal, ParamPull, SetMode
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_control_pkg.perimeter_utils import PerimeterGuard
from drone_control_pkg.px4_param_profile import Px4ParamProfile
from drone_control_pkg.topic_utils import cdrone_topic, join_topic


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min(value, max_v), min_v)


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


def normalize_demo_mode(value: str) -> str:
    normalized = str(value or "").strip().lower()
    aliases = {
        "goto": "goto",
        "position_goto": "goto",
        "circle": "circle",
        "orbit": "circle",
        "position_circle": "circle",
    }
    if normalized not in aliases:
        raise ValueError("demo_mode must be one of: goto, circle, orbit")
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


def quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def polygon_centroid_xy(vertices_xy: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    area_twice = 0.0
    centroid_x = 0.0
    centroid_y = 0.0
    for idx in range(len(vertices_xy)):
        x1, y1 = vertices_xy[idx]
        x2, y2 = vertices_xy[(idx + 1) % len(vertices_xy)]
        cross = (x1 * y2) - (x2 * y1)
        area_twice += cross
        centroid_x += (x1 + x2) * cross
        centroid_y += (y1 + y2) * cross
    if math.isclose(area_twice, 0.0, abs_tol=1e-9):
        mean_x = sum(vertex[0] for vertex in vertices_xy) / len(vertices_xy)
        mean_y = sum(vertex[1] for vertex in vertices_xy) / len(vertices_xy)
        return (mean_x, mean_y)
    return (
        centroid_x / (3.0 * area_twice),
        centroid_y / (3.0 * area_twice),
    )


def polygon_max_radius_from_center(
    center_xy: tuple[float, float],
    vertices_xy: tuple[tuple[float, float], ...],
) -> float:
    max_radius_m = 0.0
    for idx in range(len(vertices_xy)):
        start_xy = vertices_xy[idx]
        end_xy = vertices_xy[(idx + 1) % len(vertices_xy)]
        for sample_idx in range(51):
            t = sample_idx / 50.0
            sample_xy = (
                start_xy[0] + ((end_xy[0] - start_xy[0]) * t),
                start_xy[1] + ((end_xy[1] - start_xy[1]) * t),
            )
            max_radius_m = max(
                max_radius_m,
                math.hypot(
                    sample_xy[0] - center_xy[0],
                    sample_xy[1] - center_xy[1],
                ),
            )
    return max_radius_m


class PositionGotoDemoSequenceNode(Node):
    TERMINAL_STATES = {"IDLE", "COMPLETE", "ABORT"}
    OFFBOARD_PUBLISH_STATES = {
        "WARMUP_OFFBOARD",
        "SET_OFFBOARD_MODE",
        "GOTO",
        "GOAL_HOLD",
        "MOVE_TO_CIRCLE_ENTRY",
        "ORBIT",
        "ORBIT_HOLD",
    }

    def __init__(self) -> None:
        super().__init__("position_goto_demo_sequence_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("demo_mode", "goto")
        self.declare_parameter("takeoff_altitude_m", 0.7)
        self.declare_parameter("takeoff_rate_m_s", 0.5)
        self.declare_parameter("takeoff_strategy", "AUTO_MODE")
        self.declare_parameter("altitude_tolerance_m", 0.10)
        self.declare_parameter("touchdown_altitude_m", 0.15)
        self.declare_parameter("touchdown_dwell_s", 1.0)
        self.declare_parameter("stage_timeout_s", 30.0)
        self.declare_parameter("arm_zero_throttle_hold_s", 1.5)
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
        self.declare_parameter("restore_takeoff_alt_on_exit", True)
        self.declare_parameter("takeoff_param_id", "MIS_TAKEOFF_ALT")
        self.declare_parameter("param_pull_force", True)
        self.declare_parameter("param_pull_retry_delay_s", 1.0)
        self.declare_parameter("param_sync_timeout_s", 60.0)
        self.declare_parameter("use_speed_profile", False)
        self.declare_parameter("speed_profile_config", "")
        self.declare_parameter("restore_speed_profile_on_exit", True)
        self.declare_parameter("offboard_setpoint_warmup_s", 1.5)
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("goal_x_m", 0.5)
        self.declare_parameter("goal_y_m", 0.0)
        self.declare_parameter("goal_z_m", 0.7)
        self.declare_parameter("use_current_yaw_for_goal", True)
        self.declare_parameter("goal_yaw_rad", 0.0)
        self.declare_parameter("goal_position_tolerance_m", 0.15)
        self.declare_parameter("goal_hold_duration_s", 2.0)
        self.declare_parameter("max_goal_distance_from_start_m", 2.0)
        self.declare_parameter("circle_center_x_m", -0.1357)
        self.declare_parameter("circle_center_y_m", 0.1878)
        self.declare_parameter("circle_radius_m", 3.0)
        self.declare_parameter("circle_altitude_m", 2.0)
        self.declare_parameter("circle_speed_mps", 0.8)
        self.declare_parameter("circle_loops", 1.0)
        self.declare_parameter("circle_clockwise", False)
        self.declare_parameter("circle_use_keep_out_orbit", False)
        self.declare_parameter("circle_keep_out_name", "studio_pillar")
        self.declare_parameter("circle_keep_out_clearance_m", 1.2)
        self.declare_parameter("circle_sample_count", 180)
        self.declare_parameter("circle_entry_candidate_count", 72)
        self.declare_parameter("max_circle_entry_distance_from_start_m", 0.0)
        self.declare_parameter("enable_perimeter_guard", True)
        self.declare_parameter("perimeter_config", "")
        self.declare_parameter("perimeter_segment_sample_step_m", 0.10)
        self.declare_parameter("perimeter_boundary_tolerance_m", 0.05)
        self.declare_parameter("perimeter_boundary_margin_m", 1.0)
        self.declare_parameter("perimeter_keep_out_margin_m", 0.0)
        self.declare_parameter("perimeter_ceiling_tolerance_m", 0.05)
        self.declare_parameter("state_topic", "")
        self.declare_parameter("local_pose_topic", "")
        self.declare_parameter("home_position_topic", "")
        self.declare_parameter("global_origin_topic", "")
        self.declare_parameter("companion_status_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("start_service", "")
        self.declare_parameter("abort_service", "")
        self.declare_parameter("position_setpoint_topic", "")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.demo_mode = normalize_demo_mode(
            str(self.get_parameter("demo_mode").value)
        )
        self.takeoff_altitude_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.takeoff_rate_m_s = float(self.get_parameter("takeoff_rate_m_s").value)
        self.takeoff_strategy = normalize_takeoff_strategy(
            str(self.get_parameter("takeoff_strategy").value)
        )
        self.altitude_tolerance_m = float(
            self.get_parameter("altitude_tolerance_m").value
        )
        self.touchdown_altitude_m = float(
            self.get_parameter("touchdown_altitude_m").value
        )
        self.touchdown_dwell_s = float(self.get_parameter("touchdown_dwell_s").value)
        self.stage_timeout_s = float(self.get_parameter("stage_timeout_s").value)
        self.arm_zero_throttle_hold_s = float(
            self.get_parameter("arm_zero_throttle_hold_s").value
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
        self.use_speed_profile = bool(
            self.get_parameter("use_speed_profile").value
        )
        self.speed_profile_config = str(
            self.get_parameter("speed_profile_config").value
        ).strip()
        self.restore_speed_profile_on_exit = bool(
            self.get_parameter("restore_speed_profile_on_exit").value
        )
        self.offboard_setpoint_warmup_s = float(
            self.get_parameter("offboard_setpoint_warmup_s").value
        )
        self.goal_frame_id = str(self.get_parameter("goal_frame_id").value).strip()
        self.goal_x_m = float(self.get_parameter("goal_x_m").value)
        self.goal_y_m = float(self.get_parameter("goal_y_m").value)
        self.goal_z_m = float(self.get_parameter("goal_z_m").value)
        self.use_current_yaw_for_goal = bool(
            self.get_parameter("use_current_yaw_for_goal").value
        )
        self.goal_yaw_rad = float(self.get_parameter("goal_yaw_rad").value)
        self.goal_position_tolerance_m = float(
            self.get_parameter("goal_position_tolerance_m").value
        )
        self.goal_hold_duration_s = float(
            self.get_parameter("goal_hold_duration_s").value
        )
        self.max_goal_distance_from_start_m = float(
            self.get_parameter("max_goal_distance_from_start_m").value
        )
        self.circle_center_x_m = float(
            self.get_parameter("circle_center_x_m").value
        )
        self.circle_center_y_m = float(
            self.get_parameter("circle_center_y_m").value
        )
        self.circle_radius_m = float(self.get_parameter("circle_radius_m").value)
        self.circle_altitude_m = float(
            self.get_parameter("circle_altitude_m").value
        )
        self.circle_speed_mps = float(self.get_parameter("circle_speed_mps").value)
        self.circle_loops = float(self.get_parameter("circle_loops").value)
        self.circle_clockwise = bool(
            self.get_parameter("circle_clockwise").value
        )
        self.circle_use_keep_out_orbit = bool(
            self.get_parameter("circle_use_keep_out_orbit").value
        )
        self.circle_keep_out_name = str(
            self.get_parameter("circle_keep_out_name").value
        ).strip()
        self.circle_keep_out_clearance_m = float(
            self.get_parameter("circle_keep_out_clearance_m").value
        )
        self.circle_sample_count = int(
            self.get_parameter("circle_sample_count").value
        )
        self.circle_entry_candidate_count = int(
            self.get_parameter("circle_entry_candidate_count").value
        )
        self.max_circle_entry_distance_from_start_m = float(
            self.get_parameter("max_circle_entry_distance_from_start_m").value
        )
        self.enable_perimeter_guard = bool(
            self.get_parameter("enable_perimeter_guard").value
        )
        self.perimeter_config = str(
            self.get_parameter("perimeter_config").value
        ).strip()
        self.perimeter_segment_sample_step_m = float(
            self.get_parameter("perimeter_segment_sample_step_m").value
        )
        self.perimeter_boundary_tolerance_m = float(
            self.get_parameter("perimeter_boundary_tolerance_m").value
        )
        self.perimeter_boundary_margin_m = float(
            self.get_parameter("perimeter_boundary_margin_m").value
        )
        self.perimeter_keep_out_margin_m = float(
            self.get_parameter("perimeter_keep_out_margin_m").value
        )
        self.perimeter_ceiling_tolerance_m = float(
            self.get_parameter("perimeter_ceiling_tolerance_m").value
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
            or cdrone_topic(self.drone_id, "demo/position_goto_state")
        )
        self.start_service_name = (
            str(self.get_parameter("start_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/position_goto_start")
        )
        self.abort_service_name = (
            str(self.get_parameter("abort_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/position_goto_abort")
        )
        self.position_setpoint_topic = (
            str(self.get_parameter("position_setpoint_topic").value).strip()
            or join_topic(self.mavros_namespace, "setpoint_position/local")
        )

        self.takeoff_mode = "AUTO.TAKEOFF"
        self.offboard_mode = "OFFBOARD"
        self.land_mode = "AUTO.LAND"
        self.perimeter_guard: Optional[PerimeterGuard] = None
        self.perimeter_guard_error = ""

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
        self.takeoff_origin_altitude_m: Optional[float] = None
        self.takeoff_origin_xy: Optional[Tuple[float, float]] = None
        self.touchdown_started_s: Optional[float] = None
        self.goal_reached_started_s: Optional[float] = None

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
        self.pending_param_name: Optional[str] = None
        self.original_takeoff_alt_m: Optional[float] = None
        self.takeoff_param_ready = False
        self.takeoff_param_changed = False
        self.restore_complete = False
        self.speed_profile: Optional[Px4ParamProfile] = None
        self.speed_profile_error = ""
        self.speed_profile_param_names: list[str] = []
        self.speed_profile_original_values: dict[str, float] = {}
        self.speed_profile_applied_names: set[str] = set()
        self.speed_profile_changed_names: set[str] = set()
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.connection_lost_since_s: Optional[float] = None
        self.connection_loss_warned = False
        self.last_mode_request_time_s = 0.0
        self.next_param_pull_attempt_s = 0.0
        self.param_pull_attempt_count = 0
        self.offboard_setpoint: Optional[PoseStamped] = None
        self.circle_entry_angle_rad: Optional[float] = None
        self.circle_started_s: Optional[float] = None
        self.circle_fixed_yaw_rad: Optional[float] = None
        self.circle_keep_out_orbit_applied = False

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.position_setpoint_pub = self.create_publisher(
            PoseStamped, self.position_setpoint_topic, 10
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

        self.create_subscription(State, self.state_topic, self.state_callback, state_qos)
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
        self.load_perimeter_guard()
        self.load_speed_profile()
        self.get_logger().info(
            "Position goto demo sequence node started. "
            f"Waiting for explicit start request. demo_mode={self.demo_mode} "
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
            "SYNC_SPEED_PROFILE",
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
        mode_name = str(self.latest_state.mode).upper()
        return mode_name in {"AUTO.LOITER", "POSCTL"}

    def takeoff_target_altitude_m(self) -> float:
        if self.takeoff_origin_altitude_m is None:
            return self.takeoff_altitude_m
        return self.takeoff_origin_altitude_m + self.takeoff_altitude_m

    def touchdown_threshold_altitude_m(self) -> float:
        if self.takeoff_origin_altitude_m is None:
            return self.touchdown_altitude_m
        return self.takeoff_origin_altitude_m + self.touchdown_altitude_m

    def current_frame_id(self) -> str:
        return str(self.latest_pose.header.frame_id or self.goal_frame_id or "map").strip()

    def goal_xy(self) -> Tuple[float, float]:
        return (self.goal_x_m, self.goal_y_m)

    def circle_center_xy(self) -> Tuple[float, float]:
        return (self.circle_center_x_m, self.circle_center_y_m)

    def circle_direction_sign(self) -> float:
        return -1.0 if self.circle_clockwise else 1.0

    def circle_total_angle_rad(self) -> float:
        return 2.0 * math.pi * max(self.circle_loops, 0.0)

    def circle_total_duration_s(self) -> float:
        speed_mps = max(self.circle_speed_mps, 1e-3)
        circumference_m = self.circle_total_angle_rad() * max(self.circle_radius_m, 0.0)
        return circumference_m / speed_mps

    def circle_point_xy(self, angle_rad: float) -> Tuple[float, float]:
        return (
            self.circle_center_x_m + (self.circle_radius_m * math.cos(angle_rad)),
            self.circle_center_y_m + (self.circle_radius_m * math.sin(angle_rad)),
        )

    def circle_entry_pose(self) -> Optional[PoseStamped]:
        if self.circle_entry_angle_rad is None:
            return None
        x_m, y_m = self.circle_point_xy(self.circle_entry_angle_rad)
        yaw_rad = (
            self.circle_fixed_yaw_rad
            if self.circle_fixed_yaw_rad is not None
            else self.current_yaw_rad()
        )
        return self.make_pose_setpoint(x_m, y_m, self.circle_altitude_m, yaw_rad)

    def configure_circle_keep_out_orbit(self) -> Optional[str]:
        if not self.circle_use_keep_out_orbit:
            return None
        if self.perimeter_guard is None:
            return self.perimeter_guard_error or "perimeter guard is unavailable"
        keep_out_name = self.circle_keep_out_name or "studio_pillar"
        keep_out = next(
            (
                region
                for region in self.perimeter_guard.keep_outs
                if region.name == keep_out_name
            ),
            None,
        )
        if keep_out is None:
            return f"keep-out polygon '{keep_out_name}' was not found"
        center_xy = polygon_centroid_xy(keep_out.vertices_xy_m)
        radius_m = polygon_max_radius_from_center(
            center_xy,
            keep_out.vertices_xy_m,
        ) + max(self.circle_keep_out_clearance_m, 0.0)
        self.circle_center_x_m = center_xy[0]
        self.circle_center_y_m = center_xy[1]
        self.circle_radius_m = radius_m
        if not self.circle_keep_out_orbit_applied:
            self.get_logger().info(
                "Derived circle orbit from keep-out "
                f"'{keep_out_name}': center=({self.circle_center_x_m:.3f}, "
                f"{self.circle_center_y_m:.3f}) radius={self.circle_radius_m:.3f} m"
            )
            self.circle_keep_out_orbit_applied = True
        return None

    def choose_circle_entry_angle(
        self,
        start_xy: Tuple[float, float],
    ) -> Tuple[Optional[float], Optional[float]]:
        if self.perimeter_guard is None:
            return 0.0, math.hypot(
                start_xy[0] - (self.circle_center_x_m + self.circle_radius_m),
                start_xy[1] - self.circle_center_y_m,
            )

        candidate_count = max(self.circle_entry_candidate_count, 8)
        best_angle = None
        best_distance = None
        for idx in range(candidate_count):
            angle_rad = (2.0 * math.pi * idx) / candidate_count
            point_xy = self.circle_point_xy(angle_rad)
            violation = self.perimeter_guard.segment_violation_reason(
                start_xy,
                point_xy,
                sample_step_m=self.perimeter_segment_sample_step_m,
                boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
                boundary_margin_m=self.perimeter_boundary_margin_m,
                keep_out_margin_m=self.perimeter_keep_out_margin_m,
            )
            if violation is not None:
                continue
            distance_m = math.hypot(
                point_xy[0] - start_xy[0],
                point_xy[1] - start_xy[1],
            )
            if best_distance is None or distance_m < best_distance:
                best_angle = angle_rad
                best_distance = distance_m
        return best_angle, best_distance

    def load_perimeter_guard(self) -> None:
        self.perimeter_guard = None
        self.perimeter_guard_error = ""
        if not self.enable_perimeter_guard:
            self.get_logger().info("Perimeter guard disabled for goto demo.")
            return
        if not self.perimeter_config:
            self.perimeter_guard_error = "perimeter_config is empty"
            self.get_logger().error(
                "Perimeter guard enabled but no perimeter_config was provided."
            )
            return
        try:
            self.perimeter_guard = PerimeterGuard.load_from_yaml(self.perimeter_config)
        except Exception as exc:  # noqa: BLE001
            self.perimeter_guard_error = str(exc)
            self.get_logger().error(
                f"Failed to load perimeter guard from {self.perimeter_config}: {exc}"
            )
            return

        self.get_logger().info(
            "Loaded perimeter guard: "
            f"{os.path.basename(self.perimeter_guard.source_path)} "
            f"frame={self.perimeter_guard.frame_id} "
            f"keep_outs={len(self.perimeter_guard.keep_outs)}"
        )

    def load_speed_profile(self) -> None:
        self.speed_profile = None
        self.speed_profile_error = ""
        self.speed_profile_param_names = []
        if not self.use_speed_profile:
            return
        if not self.speed_profile_config:
            self.speed_profile_error = "speed_profile_config is empty"
            self.get_logger().error(
                "Speed profile enabled but no speed_profile_config was provided."
            )
            return
        try:
            self.speed_profile = Px4ParamProfile.load_from_yaml(self.speed_profile_config)
        except Exception as exc:  # noqa: BLE001
            self.speed_profile_error = str(exc)
            self.get_logger().error(
                f"Failed to load speed profile from {self.speed_profile_config}: {exc}"
            )
            return
        self.speed_profile_param_names = list(self.speed_profile.parameters.keys())
        self.get_logger().info(
            "Loaded speed profile: "
            f"{self.speed_profile.name} params={len(self.speed_profile_param_names)}"
        )

    def should_restore_speed_profile(self) -> bool:
        return (
            self.use_speed_profile
            and self.restore_speed_profile_on_exit
            and bool(self.speed_profile_changed_names)
        )

    def speed_profile_apply_complete(self) -> bool:
        if not self.use_speed_profile or self.speed_profile is None:
            return True
        return all(
            name in self.speed_profile_applied_names
            for name in self.speed_profile_param_names
        )

    def next_speed_profile_param_to_apply(self) -> Optional[str]:
        for param_name in self.speed_profile_param_names:
            if param_name not in self.speed_profile_applied_names:
                return param_name
        return None

    def next_speed_profile_param_to_restore(self) -> Optional[str]:
        for param_name in self.speed_profile_param_names:
            if param_name in self.speed_profile_changed_names:
                return param_name
        return None

    def perimeter_start_violation_reason(
        self,
        start_xy: Tuple[float, float],
    ) -> Optional[str]:
        if not self.enable_perimeter_guard:
            return None
        if self.perimeter_guard is None:
            return self.perimeter_guard_error or "perimeter guard is unavailable"
        if not self.perimeter_guard.frame_matches(self.goal_frame_id):
            return (
                f"goal frame '{self.goal_frame_id}' does not match perimeter frame "
                f"'{self.perimeter_guard.frame_id}'"
            )

        current_frame = self.current_frame_id()
        if current_frame and not self.perimeter_guard.frame_matches(current_frame):
            return (
                f"local pose frame '{current_frame}' does not match perimeter frame "
                f"'{self.perimeter_guard.frame_id}'"
            )

        start_reason = self.perimeter_guard.xy_violation_reason(
            start_xy[0],
            start_xy[1],
            boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
            boundary_margin_m=self.perimeter_boundary_margin_m,
            keep_out_margin_m=self.perimeter_keep_out_margin_m,
        )
        if start_reason is not None:
            return f"start position is unsafe: {start_reason}"

        if self.demo_mode == "circle":
            keep_out_orbit_error = self.configure_circle_keep_out_orbit()
            if keep_out_orbit_error is not None:
                return keep_out_orbit_error
            if self.circle_radius_m <= 0.0:
                return "circle_radius_m must be > 0"
            if self.circle_speed_mps <= 0.0:
                return "circle_speed_mps must be > 0"
            if self.circle_loops <= 0.0:
                return "circle_loops must be > 0"

            altitude_reason = self.perimeter_guard.goal_altitude_violation_reason(
                self.circle_altitude_m
            )
            if altitude_reason is not None:
                return altitude_reason

            sample_count = max(self.circle_sample_count, 24)
            for idx in range(sample_count):
                angle_rad = (2.0 * math.pi * idx) / sample_count
                point_xy = self.circle_point_xy(angle_rad)
                circle_reason = self.perimeter_guard.xy_violation_reason(
                    point_xy[0],
                    point_xy[1],
                    boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
                    boundary_margin_m=self.perimeter_boundary_margin_m,
                    keep_out_margin_m=self.perimeter_keep_out_margin_m,
                )
                if circle_reason is not None:
                    return f"circle orbit is unsafe at angle {angle_rad:.2f} rad: {circle_reason}"

            entry_angle_rad, entry_distance_m = self.choose_circle_entry_angle(start_xy)
            if entry_angle_rad is None:
                return "no safe straight-line entry path to the circle was found"
            if (
                self.max_circle_entry_distance_from_start_m > 0.0
                and entry_distance_m is not None
                and entry_distance_m > self.max_circle_entry_distance_from_start_m
            ):
                return (
                    f"circle entry is {entry_distance_m:.2f} m from start, exceeds "
                    f"{self.max_circle_entry_distance_from_start_m:.2f} m limit"
                )
            self.circle_entry_angle_rad = entry_angle_rad
            return None

        goal_reason = self.perimeter_guard.xy_violation_reason(
            self.goal_x_m,
            self.goal_y_m,
            boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
            boundary_margin_m=self.perimeter_boundary_margin_m,
            keep_out_margin_m=self.perimeter_keep_out_margin_m,
        )
        if goal_reason is not None:
            return f"goal position is unsafe: {goal_reason}"

        altitude_reason = self.perimeter_guard.goal_altitude_violation_reason(
            self.goal_z_m
        )
        if altitude_reason is not None:
            return altitude_reason

        path_reason = self.perimeter_guard.segment_violation_reason(
            start_xy,
            self.goal_xy(),
            sample_step_m=self.perimeter_segment_sample_step_m,
            boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
            boundary_margin_m=self.perimeter_boundary_margin_m,
            keep_out_margin_m=self.perimeter_keep_out_margin_m,
        )
        if path_reason is not None:
            return f"straight-line path to goal is unsafe: {path_reason}"
        return None

    def perimeter_runtime_violation_reason(self) -> Optional[str]:
        if not self.enable_perimeter_guard or self.perimeter_guard is None:
            return None

        current_xy = self.current_xy()
        xy_reason = self.perimeter_guard.xy_violation_reason(
            current_xy[0],
            current_xy[1],
            boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
            boundary_margin_m=self.perimeter_boundary_margin_m,
            keep_out_margin_m=self.perimeter_keep_out_margin_m,
        )
        if xy_reason is not None:
            return xy_reason

        return self.perimeter_guard.runtime_ceiling_violation_reason(
            self.current_altitude_m(),
            ceiling_tolerance_m=self.perimeter_ceiling_tolerance_m,
        )

    def publish_status(self) -> None:
        msg = String()
        msg.data = self.demo_state
        self.status_pub.publish(msg)

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
        if self.use_speed_profile and self.speed_profile is None:
            return False, self.speed_profile_error or "speed profile is unavailable"
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
        start_xy = self.current_xy()
        if self.demo_mode == "goto":
            goal_offset_m = math.hypot(
                self.goal_x_m - start_xy[0],
                self.goal_y_m - start_xy[1],
            )
            if (
                self.max_goal_distance_from_start_m > 0.0
                and goal_offset_m > self.max_goal_distance_from_start_m
            ):
                return (
                    False,
                    f"goal is {goal_offset_m:.2f} m from start, exceeds "
                    f"{self.max_goal_distance_from_start_m:.2f} m limit",
                )
        else:
            if self.circle_use_keep_out_orbit:
                keep_out_orbit_error = self.configure_circle_keep_out_orbit()
                if keep_out_orbit_error is not None:
                    return False, keep_out_orbit_error
            if self.circle_radius_m <= 0.0:
                return False, "circle_radius_m must be > 0"
            if self.circle_speed_mps <= 0.0:
                return False, "circle_speed_mps must be > 0"
            if self.circle_loops <= 0.0:
                return False, "circle_loops must be > 0"
            if not self.enable_perimeter_guard:
                entry_angle_rad, entry_distance_m = self.choose_circle_entry_angle(start_xy)
                self.circle_entry_angle_rad = entry_angle_rad
                if (
                    self.max_circle_entry_distance_from_start_m > 0.0
                    and entry_distance_m is not None
                    and entry_distance_m > self.max_circle_entry_distance_from_start_m
                ):
                    return (
                        False,
                        f"circle entry is {entry_distance_m:.2f} m from start, exceeds "
                        f"{self.max_circle_entry_distance_from_start_m:.2f} m limit",
                    )
        perimeter_reason = self.perimeter_start_violation_reason(start_xy)
        if perimeter_reason is not None:
            return False, perimeter_reason
        if self.demo_mode == "circle" and self.circle_entry_angle_rad is None:
            entry_angle_rad, _ = self.choose_circle_entry_angle(start_xy)
            self.circle_entry_angle_rad = entry_angle_rad
            if self.circle_entry_angle_rad is None:
                return False, "failed to choose a circle entry point"
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
        self.goal_reached_started_s = None
        self.takeoff_request_sent = False
        self.takeoff_request_accepted = False
        self.pending_takeoff_target = None
        self.last_takeoff_progress_log_s = 0.0
        self.pending_mode_name = None
        self.pending_arm_value = None
        self.pending_param_value = None
        self.pending_param_name = None
        self.param_mirror_ready = False
        self.allow_param_lookup_without_full_mirror = True
        self.original_takeoff_alt_m = None
        self.takeoff_param_ready = False
        self.takeoff_param_changed = False
        self.restore_complete = False
        self.speed_profile_original_values = {}
        self.speed_profile_applied_names = set()
        self.speed_profile_changed_names = set()
        self.connection_lost_since_s = None
        self.connection_loss_warned = False
        self.last_mode_request_time_s = 0.0
        self.next_param_pull_attempt_s = 0.0
        self.param_pull_attempt_count = 0
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.offboard_setpoint = None
        self.circle_started_s = None
        self.circle_fixed_yaw_rad = None

        self.transition_to("SYNC_TAKEOFF_PARAM", "start requested")
        response.success = True
        response.message = "position goto demo sequence started"
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
        if new_state == "ORBIT":
            self.circle_started_s = self.stage_started_s
        elif new_state != "MOVE_TO_CIRCLE_ENTRY":
            self.circle_started_s = None
        if new_state != "WAIT_TOUCHDOWN":
            self.touchdown_started_s = None
        if new_state not in {"GOAL_HOLD", "ORBIT_HOLD"}:
            self.goal_reached_started_s = None
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
        if self.demo_state == "SYNC_TAKEOFF_PARAM":
            return max(self.stage_timeout_s, self.param_sync_timeout_s)
        if self.demo_state == "SYNC_SPEED_PROFILE":
            return max(self.stage_timeout_s, self.param_sync_timeout_s)
        if self.demo_state in {"WAIT_TOUCHDOWN", "DISARMING"}:
            return max(self.stage_timeout_s, 25.0)
        if self.demo_state == "RESTORE_SPEED_PROFILE":
            return max(self.stage_timeout_s, self.param_sync_timeout_s)
        if self.demo_state in {"GOAL_HOLD", "ORBIT_HOLD"}:
            return max(self.stage_timeout_s, self.goal_hold_duration_s + 2.0)
        if self.demo_state == "ORBIT":
            return max(self.stage_timeout_s, self.circle_total_duration_s() + 5.0)
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

    def request_param_get(
        self,
        *,
        param_name: Optional[str] = None,
        kind: str = "GET_ORIGINAL",
    ) -> None:
        if self.param_future is not None:
            return
        resolved_param_name = str(param_name or self.takeoff_param_id).strip()
        if not resolved_param_name:
            self.enter_abort("param get requested with an empty parameter name")
            return
        if not self.param_get_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort(f"param get service unavailable for {resolved_param_name}")
            return
        self.param_future_kind = kind
        self.pending_param_name = resolved_param_name
        req = GetParameters.Request()
        req.names = [resolved_param_name]
        self.param_future = self.param_get_client.call_async(req)
        self.get_logger().info(f"Requested current {resolved_param_name}")

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

    def request_param_set(
        self,
        value: float,
        *,
        kind: str,
        param_name: Optional[str] = None,
    ) -> None:
        if self.param_future is not None:
            return
        resolved_param_name = str(param_name or self.takeoff_param_id).strip()
        if not resolved_param_name:
            self.enter_abort("param set requested with an empty parameter name")
            return
        if not self.param_set_client.wait_for_service(timeout_sec=1.0):
            self.enter_abort(f"param set service unavailable for {resolved_param_name}")
            return
        self.param_future_kind = kind
        self.pending_param_value = float(value)
        self.pending_param_name = resolved_param_name
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name=resolved_param_name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(value),
                ),
            )
        ]
        self.param_future = self.param_set_client.call_async(req)
        self.get_logger().info(
            f"Requested {resolved_param_name}={float(value):.2f}"
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
            param_name = self.pending_param_name or self.takeoff_param_id
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
                elif future_kind == "PROFILE_GET_ORIGINAL":
                    if not response.values:
                        self.schedule_param_pull_retry(
                            f"{param_name} not returned yet"
                        )
                        return
                    if not parameter_value_is_declared_numeric(response.values[0]):
                        self.schedule_param_pull_retry(
                            f"{param_name} is not declared on MAVROS yet"
                        )
                        return
                    original_value = parameter_value_to_float(response.values[0])
                    self.speed_profile_original_values[param_name] = original_value
                    if (
                        self.speed_profile is not None
                        and math.isclose(
                            original_value,
                            self.speed_profile.parameters[param_name],
                            abs_tol=1e-3,
                        )
                    ):
                        self.speed_profile_applied_names.add(param_name)
                    self.get_logger().info(
                        f"{param_name} currently {original_value:.2f}"
                    )
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
                elif future_kind == "PROFILE_SET_TARGET":
                    if not response.results or not response.results[0].successful:
                        reason = (
                            response.results[0].reason
                            if response.results
                            else "no response"
                        )
                        if "undeclared" in str(reason).lower():
                            self.schedule_param_pull_retry(
                                f"{param_name} not declared yet during set"
                            )
                            return
                        self.enter_abort(f"failed to set {param_name}: {reason}")
                        return
                    self.speed_profile_applied_names.add(param_name)
                    self.speed_profile_changed_names.add(param_name)
                    self.get_logger().info(
                        f"Set {param_name} to {self.pending_param_value:.2f}"
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
                elif future_kind == "PROFILE_RESTORE_ORIGINAL":
                    if not response.results or not response.results[0].successful:
                        reason = (
                            response.results[0].reason
                            if response.results
                            else "no response"
                        )
                        if "undeclared" in str(reason).lower():
                            self.schedule_param_pull_retry(
                                f"{param_name} not declared yet during restore"
                            )
                            return
                        self.enter_abort(f"failed to restore {param_name}: {reason}")
                        return
                    self.speed_profile_changed_names.discard(param_name)
                    self.speed_profile_applied_names.discard(param_name)
                    self.get_logger().info(
                        f"Restored {param_name} to {self.pending_param_value:.2f}"
                    )
            except Exception as exc:
                self.enter_abort(
                    f"{future_kind} request raised for {param_name}: {exc}"
                )
            finally:
                if self.param_future is not None and self.param_future.done():
                    self.param_future = None
                    self.param_future_kind = None
                    self.pending_param_value = None
                    self.pending_param_name = None

    def make_pose_setpoint(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
    ) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame_id or "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x_m)
        msg.pose.position.y = float(y_m)
        msg.pose.position.z = float(z_m)
        qx, qy, qz, qw = quaternion_from_yaw(yaw_rad)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def current_hold_pose(self) -> PoseStamped:
        pose = self.latest_pose
        msg = PoseStamped()
        msg.header.frame_id = pose.header.frame_id or self.goal_frame_id or "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose.pose
        return msg

    def goal_pose(self) -> PoseStamped:
        yaw_rad = self.current_yaw_rad() if self.use_current_yaw_for_goal else self.goal_yaw_rad
        return self.make_pose_setpoint(
            self.goal_x_m,
            self.goal_y_m,
            self.goal_z_m,
            yaw_rad,
        )

    def circle_pose(self, angle_rad: float) -> PoseStamped:
        x_m, y_m = self.circle_point_xy(angle_rad)
        yaw_rad = (
            self.circle_fixed_yaw_rad
            if self.circle_fixed_yaw_rad is not None
            else self.current_yaw_rad()
        )
        return self.make_pose_setpoint(x_m, y_m, self.circle_altitude_m, yaw_rad)

    def publish_setpoint_for_state(self) -> None:
        if self.demo_state not in self.OFFBOARD_PUBLISH_STATES:
            return
        if self.offboard_setpoint is None:
            return
        msg = PoseStamped()
        msg.header = self.offboard_setpoint.header
        msg.pose = self.offboard_setpoint.pose
        msg.header.stamp = self.get_clock().now().to_msg()
        self.position_setpoint_pub.publish(msg)

    def goal_position_error_m(self) -> Optional[float]:
        if self.offboard_setpoint is None:
            return None
        dx = float(self.latest_pose.pose.position.x - self.offboard_setpoint.pose.position.x)
        dy = float(self.latest_pose.pose.position.y - self.offboard_setpoint.pose.position.y)
        dz = float(self.latest_pose.pose.position.z - self.offboard_setpoint.pose.position.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

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
            self.enter_abort(f"lost local pose updates (age {pose_age_s:.2f}s)")
            return
        perimeter_reason = self.perimeter_runtime_violation_reason()
        if perimeter_reason is not None:
            self.enter_abort(f"perimeter violation: {perimeter_reason}")
            return
        if self.require_companion_active:
            if not self.companion_status_fresh():
                self.enter_abort("external pose companion status went stale")
                return
            if not self.companion_active:
                self.enter_abort("external pose companion status is not active")
                return
        if self.demo_state == "TAKEOFF" and not self.latest_state.armed:
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
        if (
            self.demo_state in {"GOTO", "GOAL_HOLD", "MOVE_TO_CIRCLE_ENTRY", "ORBIT", "ORBIT_HOLD"}
            and self.mode_future is None
            and not self.mode_matches(self.offboard_mode)
        ):
            self.enter_abort(
                f"mode mismatch during {self.demo_state}: {self.latest_state.mode}"
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
                if self.use_speed_profile:
                    self.transition_to(
                        "SYNC_SPEED_PROFILE",
                        "takeoff altitude configured",
                    )
                else:
                    self.transition_to(
                        "SET_TAKEOFF_MODE",
                        "takeoff altitude configured",
                    )
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

        if self.demo_state == "SYNC_SPEED_PROFILE":
            if not self.use_speed_profile or self.speed_profile is None:
                self.transition_to("SET_TAKEOFF_MODE", "speed profile disabled")
                return
            if self.speed_profile_apply_complete():
                self.transition_to("SET_TAKEOFF_MODE", "speed profile configured")
                return
            if self.param_future is not None or self.param_pull_future is not None:
                return
            if (
                not self.param_mirror_ready
                and not self.allow_param_lookup_without_full_mirror
                and self.now_s() >= self.next_param_pull_attempt_s
            ):
                self.request_param_pull()
                return
            param_name = self.next_speed_profile_param_to_apply()
            if param_name is None:
                self.transition_to("SET_TAKEOFF_MODE", "speed profile configured")
                return
            if param_name not in self.speed_profile_original_values:
                self.request_param_get(
                    param_name=param_name,
                    kind="PROFILE_GET_ORIGINAL",
                )
                return
            original_value = self.speed_profile_original_values[param_name]
            target_value = self.speed_profile.parameters[param_name]
            if math.isclose(original_value, target_value, abs_tol=1e-3):
                self.speed_profile_applied_names.add(param_name)
                return
            self.request_param_set(
                target_value,
                kind="PROFILE_SET_TARGET",
                param_name=param_name,
            )
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
            if (
                self.last_takeoff_progress_log_s == 0.0
                or (self.now_s() - self.last_takeoff_progress_log_s) >= 1.0
            ):
                self.get_logger().info(
                    "Takeoff progress: "
                    f"delta_z={altitude_delta_m:.2f} m "
                    f"target_delta_z={self.takeoff_altitude_m:.2f} m "
                    f"mode={self.latest_state.mode} "
                    f"armed={self.latest_state.armed}"
                )
                self.last_takeoff_progress_log_s = self.now_s()
            if current_altitude_m >= (target_altitude_m - self.altitude_tolerance_m):
                self.offboard_setpoint = self.current_hold_pose()
                self.transition_to("WARMUP_OFFBOARD", "target altitude reached")
                return
            if self.is_takeoff_handoff_mode():
                shortfall_m = max(target_altitude_m - current_altitude_m, 0.0)
                if shortfall_m > self.altitude_tolerance_m:
                    self.get_logger().warn(
                        "PX4 exited AUTO.TAKEOFF before the demo observed the "
                        f"target altitude; shortfall={shortfall_m:.2f} m "
                        f"mode={self.latest_state.mode}. Accepting handoff."
                    )
                self.offboard_setpoint = self.current_hold_pose()
                self.transition_to(
                    "WARMUP_OFFBOARD",
                    f"PX4 handed off to {self.latest_state.mode} during takeoff",
                )
            return

        if self.demo_state == "WARMUP_OFFBOARD":
            if self.offboard_setpoint is None:
                self.offboard_setpoint = self.current_hold_pose()
            if self.stage_elapsed_s() >= self.offboard_setpoint_warmup_s:
                self.transition_to("SET_OFFBOARD_MODE", "offboard setpoint warmup complete")
            return

        if self.demo_state == "SET_OFFBOARD_MODE":
            if self.mode_matches(self.offboard_mode):
                if self.demo_mode == "circle":
                    self.circle_fixed_yaw_rad = self.current_yaw_rad()
                    self.offboard_setpoint = self.circle_entry_pose()
                    if self.offboard_setpoint is None:
                        self.enter_abort("circle entry pose is unavailable")
                    else:
                        self.transition_to(
                            "MOVE_TO_CIRCLE_ENTRY",
                            "OFFBOARD confirmed for circle demo",
                        )
                else:
                    self.offboard_setpoint = self.goal_pose()
                    self.transition_to("GOTO", "OFFBOARD confirmed")
            elif self.mode_future is None and self.mode_request_retry_ready():
                self.request_mode(self.offboard_mode)
            return

        if self.demo_state == "MOVE_TO_CIRCLE_ENTRY":
            error_m = self.goal_position_error_m()
            if error_m is not None and error_m <= self.goal_position_tolerance_m:
                if self.circle_entry_angle_rad is None:
                    self.enter_abort("circle entry angle is unavailable")
                    return
                self.offboard_setpoint = self.circle_pose(self.circle_entry_angle_rad)
                self.transition_to("ORBIT", "circle entry reached")
            return

        if self.demo_state == "GOTO":
            error_m = self.goal_position_error_m()
            if error_m is not None and error_m <= self.goal_position_tolerance_m:
                self.transition_to("GOAL_HOLD", "goal position reached")
            return

        if self.demo_state == "GOAL_HOLD":
            error_m = self.goal_position_error_m()
            if error_m is None:
                return
            if error_m > self.goal_position_tolerance_m:
                self.transition_to("GOTO", "goal hold broken")
                return
            if self.stage_elapsed_s() >= self.goal_hold_duration_s:
                self.transition_to("SET_LAND_MODE", "goal hold complete")
            return

        if self.demo_state == "ORBIT":
            if self.circle_entry_angle_rad is None:
                self.enter_abort("circle entry angle is unavailable")
                return
            total_duration_s = self.circle_total_duration_s()
            if total_duration_s <= 0.0:
                self.enter_abort("circle duration is invalid")
                return
            progress = min(max(self.stage_elapsed_s() / total_duration_s, 0.0), 1.0)
            angle_rad = self.circle_entry_angle_rad + (
                self.circle_direction_sign() * self.circle_total_angle_rad() * progress
            )
            self.offboard_setpoint = self.circle_pose(angle_rad)
            if self.stage_elapsed_s() >= total_duration_s:
                self.transition_to("ORBIT_HOLD", "circle complete")
            return

        if self.demo_state == "ORBIT_HOLD":
            if self.circle_entry_angle_rad is None:
                self.enter_abort("circle entry angle is unavailable")
                return
            final_angle_rad = self.circle_entry_angle_rad + (
                self.circle_direction_sign() * self.circle_total_angle_rad()
            )
            self.offboard_setpoint = self.circle_pose(final_angle_rad)
            if self.stage_elapsed_s() >= self.goal_hold_duration_s:
                self.transition_to("SET_LAND_MODE", "orbit hold complete")
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
                elif self.should_restore_speed_profile():
                    self.transition_to(
                        "RESTORE_SPEED_PROFILE",
                        "vehicle auto-disarmed after landing",
                    )
                else:
                    self.transition_to(
                        "COMPLETE", "vehicle auto-disarmed after landing"
                    )
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
                elif self.should_restore_speed_profile():
                    self.transition_to("RESTORE_SPEED_PROFILE", "vehicle disarmed")
                else:
                    self.transition_to("COMPLETE", "vehicle disarmed")
                return
            if not self.disarm_request_sent:
                self.disarm_request_sent = True
                self.request_arm(False)
            return

        if self.demo_state == "RESTORE_TAKEOFF_PARAM":
            if not self.restore_takeoff_alt_on_exit:
                if self.should_restore_speed_profile():
                    self.transition_to(
                        "RESTORE_SPEED_PROFILE",
                        "skipping takeoff-altitude restore",
                    )
                else:
                    next_state = "ABORT" if self.abort_reason else "COMPLETE"
                    self.transition_to(next_state, "skipping takeoff-altitude restore")
                return
            if not self.takeoff_param_changed or self.original_takeoff_alt_m is None:
                if self.should_restore_speed_profile():
                    self.transition_to(
                        "RESTORE_SPEED_PROFILE",
                        "takeoff altitude already restored",
                    )
                else:
                    next_state = "ABORT" if self.abort_reason else "COMPLETE"
                    self.transition_to(next_state, "takeoff altitude already restored")
                return
            if self.restore_complete:
                if self.should_restore_speed_profile():
                    self.transition_to(
                        "RESTORE_SPEED_PROFILE",
                        "takeoff altitude restored",
                    )
                else:
                    next_state = "ABORT" if self.abort_reason else "COMPLETE"
                    reason = (
                        "abort cleanup complete"
                        if self.abort_reason
                        else "demo complete"
                    )
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

        if self.demo_state == "RESTORE_SPEED_PROFILE":
            if not self.should_restore_speed_profile():
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
            if self.param_future is not None:
                return
            param_name = self.next_speed_profile_param_to_restore()
            if param_name is None:
                next_state = "ABORT" if self.abort_reason else "COMPLETE"
                reason = "abort cleanup complete" if self.abort_reason else "demo complete"
                self.transition_to(next_state, reason)
                return
            original_value = self.speed_profile_original_values.get(param_name)
            if original_value is None:
                self.speed_profile_changed_names.discard(param_name)
                return
            self.request_param_set(
                original_value,
                kind="PROFILE_RESTORE_ORIGINAL",
                param_name=param_name,
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
            if self.should_restore_speed_profile():
                if (
                    self.param_pull_future is None
                    and not self.param_mirror_ready
                    and self.now_s() >= self.next_param_pull_attempt_s
                ):
                    self.request_param_pull()
                    return
                if self.param_pull_future is not None:
                    return
                if self.param_future is not None:
                    return
                param_name = self.next_speed_profile_param_to_restore()
                if param_name is None:
                    return
                original_value = self.speed_profile_original_values.get(param_name)
                if original_value is None:
                    self.speed_profile_changed_names.discard(param_name)
                    return
                self.request_param_set(
                    original_value,
                    kind="PROFILE_RESTORE_ORIGINAL",
                    param_name=param_name,
                )
            return

    def timer_callback(self) -> None:
        self.poll_service_futures()
        self.run_safety_checks()
        self.step_state_machine()
        self.publish_setpoint_for_state()
        self.publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PositionGotoDemoSequenceNode()

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
