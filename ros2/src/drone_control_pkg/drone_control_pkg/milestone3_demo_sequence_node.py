from __future__ import annotations

import math
import os
from typing import Dict, Optional, Tuple

import rclpy
from drone_msgs.msg import EngagementState, TargetTrackArray
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import CompanionProcessStatus, HomePosition, State
from mavros_msgs.srv import CommandBool, CommandTOLLocal, ParamPull, SetMode
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from drone_control_pkg.deployment_config import (
    configured_drone_id,
    configured_mavros_namespace,
)
from drone_control_pkg.follow_utils import (
    FollowCommand,
    TRACK_SOURCE_PREDICTED,
    TrackSnapshot,
    clamp_follow_command_altitude,
    clamp_follow_command_velocity,
    compute_follow_command,
    compute_return_to_point_command,
    distance_in_standoff_window,
    target_bearing_rad,
    wrap_angle_rad,
)
from drone_control_pkg.milestone2_demo_logic import (
    project_body_velocity_to_world_xy,
    select_sequential_target,
    update_dwell_progress,
)
from drone_control_pkg.milestone3_state_logic import (
    completion_next_state,
    post_return_next_state,
    requires_runtime_tracking_guards,
)
from drone_control_pkg.perimeter_utils import PerimeterGuard
from drone_control_pkg.px4_param_profile import Px4ParamProfile
from drone_control_pkg.topic_utils import cdrone_topic, join_topic


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


class Milestone3DemoSequenceNode(Node):
    TERMINAL_STATES = {"IDLE", "COMPLETE", "ABORT"}
    ACTIVE_ENGAGEMENT_STATES = {"SEARCH", "FOLLOW", "DWELL"}
    RETURN_STATES = {"RETURN_TO_START", "RETURN_HOME_HOLD"}
    OFFBOARD_HOLD_STATES = {
        "WARMUP_OFFBOARD",
        "SET_OFFBOARD_MODE",
        "SYNC_SPEED_PROFILE",
        "CLIMB_TO_TAKEOFF_ALTITUDE",
        "STAGE_HOVER",
        "RETURN_HOME_HOLD",
    }

    def __init__(self) -> None:
        super().__init__("milestone3_demo_sequence_node")

        self.declare_parameter("scenario_id", "milestone3_demo_v1")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("takeoff_altitude_m", 2.6)
        self.declare_parameter("takeoff_rate_m_s", 0.5)
        self.declare_parameter("takeoff_strategy", "AUTO_MODE")
        self.declare_parameter("altitude_tolerance_m", 0.10)
        self.declare_parameter("touchdown_altitude_m", 0.15)
        self.declare_parameter("touchdown_dwell_s", 1.0)
        self.declare_parameter("stage_timeout_s", 30.0)
        self.declare_parameter("arm_zero_throttle_hold_s", 1.5)
        self.declare_parameter("offboard_warmup_s", 1.5)
        self.declare_parameter("stage_hover_duration_s", 2.0)
        self.declare_parameter("climb_handoff_timeout_s", 8.0)
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
        self.declare_parameter("require_mavros_connected", True)
        self.declare_parameter("require_companion_active", True)
        self.declare_parameter("restore_takeoff_alt_on_exit", True)
        self.declare_parameter("takeoff_param_id", "MIS_TAKEOFF_ALT")
        self.declare_parameter("param_pull_force", True)
        self.declare_parameter("param_pull_retry_delay_s", 1.0)
        self.declare_parameter("param_sync_timeout_s", 60.0)
        self.declare_parameter("pre_takeoff_profile_config", "")
        self.declare_parameter("use_speed_profile", True)
        self.declare_parameter("speed_profile_config", "")
        self.declare_parameter("restore_speed_profile_on_exit", True)
        self.declare_parameter("land_on_complete", True)
        self.declare_parameter("land_on_abort", True)
        self.declare_parameter("return_to_takeoff_on_complete", True)
        self.declare_parameter("return_position_tolerance_m", 0.20)
        self.declare_parameter("return_yaw_tolerance_rad", 0.10)
        self.declare_parameter("return_hold_duration_s", 1.5)
        self.declare_parameter("required_completion_count", 3)
        self.declare_parameter("dwell_time_s", 3.0)
        self.declare_parameter("follow_distance_m", 1.5)
        self.declare_parameter("follow_distance_tolerance_m", 0.2)
        self.declare_parameter("lateral_deadband_m", 0.15)
        self.declare_parameter("vertical_deadband_m", 0.15)
        self.declare_parameter("yaw_deadband_rad", 0.08)
        self.declare_parameter("track_timeout_s", 0.5)
        self.declare_parameter("track_gc_s", 2.5)
        self.declare_parameter("min_track_confidence", 0.35)
        self.declare_parameter("min_target_distance_m", 0.0)
        self.declare_parameter("max_target_distance_m", 2.6)
        self.declare_parameter("require_target_in_front", True)
        self.declare_parameter("max_abs_target_y_m", 4.0)
        self.declare_parameter("max_abs_target_z_m", 2.5)
        self.declare_parameter("enable_predicted_track_control", False)
        self.declare_parameter("predicted_track_hold_s", 1.5)
        self.declare_parameter("max_predicted_position_uncertainty_m", 0.75)
        self.declare_parameter("predicted_max_vel_xy_mps", 0.25)
        self.declare_parameter("predicted_max_vel_z_mps", 0.15)
        self.declare_parameter("predicted_max_yaw_rate_rps", 0.25)
        self.declare_parameter("min_safe_distance_m", 1.0)
        self.declare_parameter("publish_zero_on_block", True)
        self.declare_parameter("block_target_on_perimeter_violation", True)
        self.declare_parameter("projected_path_horizon_s", 0.75)
        self.declare_parameter("kp_xy", 0.45)
        self.declare_parameter("kp_z", 0.30)
        self.declare_parameter("kp_yaw", 0.80)
        self.declare_parameter("max_vel_xy_mps", 0.6)
        self.declare_parameter("max_vel_z_mps", 0.3)
        self.declare_parameter("max_yaw_rate_rps", 0.4)
        self.declare_parameter("enable_perimeter_guard", True)
        self.declare_parameter("perimeter_config", "")
        self.declare_parameter("perimeter_segment_sample_step_m", 0.10)
        self.declare_parameter("perimeter_boundary_tolerance_m", 0.05)
        self.declare_parameter("perimeter_boundary_margin_m", 1.0)
        self.declare_parameter("perimeter_keep_out_margin_m", 0.3)
        self.declare_parameter("perimeter_ceiling_tolerance_m", 0.05)
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("estop_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("local_pose_topic", "")
        self.declare_parameter("home_position_topic", "")
        self.declare_parameter("global_origin_topic", "")
        self.declare_parameter("companion_status_topic", "")
        self.declare_parameter("engagement_state_topic", "")
        self.declare_parameter("start_service", "")
        self.declare_parameter("abort_service", "")

        self.scenario_id = str(self.get_parameter("scenario_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
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
        self.offboard_warmup_s = float(self.get_parameter("offboard_warmup_s").value)
        self.stage_hover_duration_s = float(
            self.get_parameter("stage_hover_duration_s").value
        )
        self.climb_handoff_timeout_s = float(
            self.get_parameter("climb_handoff_timeout_s").value
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
        self.require_mavros_connected = bool(
            self.get_parameter("require_mavros_connected").value
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
        self.pre_takeoff_profile_config = str(
            self.get_parameter("pre_takeoff_profile_config").value
        ).strip()
        self.use_speed_profile = bool(
            self.get_parameter("use_speed_profile").value
        )
        self.speed_profile_config = str(
            self.get_parameter("speed_profile_config").value
        ).strip()
        self.restore_speed_profile_on_exit = bool(
            self.get_parameter("restore_speed_profile_on_exit").value
        )
        self.land_on_complete = bool(self.get_parameter("land_on_complete").value)
        self.land_on_abort = bool(self.get_parameter("land_on_abort").value)
        self.return_to_takeoff_on_complete = bool(
            self.get_parameter("return_to_takeoff_on_complete").value
        )
        self.return_position_tolerance_m = float(
            self.get_parameter("return_position_tolerance_m").value
        )
        self.return_yaw_tolerance_rad = float(
            self.get_parameter("return_yaw_tolerance_rad").value
        )
        self.return_hold_duration_s = float(
            self.get_parameter("return_hold_duration_s").value
        )
        self.required_completion_count = max(
            1, int(self.get_parameter("required_completion_count").value)
        )
        self.dwell_time_s = float(self.get_parameter("dwell_time_s").value)
        self.follow_distance_m = float(self.get_parameter("follow_distance_m").value)
        self.follow_distance_tolerance_m = float(
            self.get_parameter("follow_distance_tolerance_m").value
        )
        self.lateral_deadband_m = float(
            self.get_parameter("lateral_deadband_m").value
        )
        self.vertical_deadband_m = float(
            self.get_parameter("vertical_deadband_m").value
        )
        self.yaw_deadband_rad = float(self.get_parameter("yaw_deadband_rad").value)
        self.track_timeout_s = float(self.get_parameter("track_timeout_s").value)
        self.track_gc_s = float(self.get_parameter("track_gc_s").value)
        self.min_track_confidence = float(
            self.get_parameter("min_track_confidence").value
        )
        self.min_target_distance_m = float(
            self.get_parameter("min_target_distance_m").value
        )
        self.max_target_distance_m = float(
            self.get_parameter("max_target_distance_m").value
        )
        self.require_target_in_front = bool(
            self.get_parameter("require_target_in_front").value
        )
        self.max_abs_target_y_m = float(
            self.get_parameter("max_abs_target_y_m").value
        )
        self.max_abs_target_z_m = float(
            self.get_parameter("max_abs_target_z_m").value
        )
        self.enable_predicted_track_control = bool(
            self.get_parameter("enable_predicted_track_control").value
        )
        self.predicted_track_hold_s = float(
            self.get_parameter("predicted_track_hold_s").value
        )
        self.max_predicted_position_uncertainty_m = float(
            self.get_parameter("max_predicted_position_uncertainty_m").value
        )
        self.predicted_max_vel_xy_mps = float(
            self.get_parameter("predicted_max_vel_xy_mps").value
        )
        self.predicted_max_vel_z_mps = float(
            self.get_parameter("predicted_max_vel_z_mps").value
        )
        self.predicted_max_yaw_rate_rps = float(
            self.get_parameter("predicted_max_yaw_rate_rps").value
        )
        self.min_safe_distance_m = float(
            self.get_parameter("min_safe_distance_m").value
        )
        self.publish_zero_on_block = bool(
            self.get_parameter("publish_zero_on_block").value
        )
        self.block_target_on_perimeter_violation = bool(
            self.get_parameter("block_target_on_perimeter_violation").value
        )
        self.projected_path_horizon_s = float(
            self.get_parameter("projected_path_horizon_s").value
        )
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_vel_xy_mps = float(self.get_parameter("max_vel_xy_mps").value)
        self.max_vel_z_mps = float(self.get_parameter("max_vel_z_mps").value)
        self.max_yaw_rate_rps = float(self.get_parameter("max_yaw_rate_rps").value)
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

        self.tracks_topic = (
            str(self.get_parameter("tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/tracks")
        )
        self.cmd_vel_topic = (
            str(self.get_parameter("cmd_vel_topic").value).strip()
            or cdrone_topic(self.drone_id, "control/cmd_vel_body")
        )
        self.estop_topic = (
            str(self.get_parameter("estop_topic").value).strip()
            or cdrone_topic(self.drone_id, "safety/estop")
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
        self.engagement_state_topic = (
            str(self.get_parameter("engagement_state_topic").value).strip()
            or cdrone_topic(self.drone_id, "engagement/state")
        )
        self.start_service_name = (
            str(self.get_parameter("start_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/milestone3_start")
        )
        self.abort_service_name = (
            str(self.get_parameter("abort_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/milestone3_abort")
        )

        self.takeoff_mode = "AUTO.TAKEOFF"
        self.offboard_mode = "OFFBOARD"
        self.land_mode = "AUTO.LAND"
        self.demo_state = "IDLE"
        self.stage_started_s = self.now_s()
        self.abort_reason = ""
        self.last_blocked_reason = ""
        self.dwell_started_s: Optional[float] = None
        self.dwell_elapsed_s = 0.0
        self.dwell_remaining_s = self.dwell_time_s
        self.min_distance_gate_active = False
        self.estop_latched = False

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
        self.connection_lost_since_s: Optional[float] = None
        self.connection_loss_warned = False

        self.takeoff_origin_altitude_m: Optional[float] = None
        self.takeoff_origin_xy: Optional[Tuple[float, float]] = None
        self.takeoff_origin_yaw_rad: Optional[float] = None
        self.climb_hold_xy: Optional[Tuple[float, float]] = None
        self.climb_hold_yaw_rad: Optional[float] = None
        self.touchdown_started_s: Optional[float] = None
        self.last_takeoff_progress_log_s = 0.0

        self.mode_future = None
        self.pending_mode_name: Optional[str] = None
        self.arm_future = None
        self.pending_arm_value: Optional[bool] = None
        self.takeoff_future = None
        self.takeoff_request_sent = False
        self.takeoff_request_accepted = False
        self.pending_takeoff_target: Optional[Tuple[float, float, float]] = None
        self.param_pull_future = None
        self.param_future = None
        self.param_future_kind: Optional[str] = None
        self.pending_param_value: Optional[float] = None
        self.pending_param_name: Optional[str] = None
        self.param_mirror_ready = False
        self.allow_param_lookup_without_full_mirror = True
        self.next_param_pull_attempt_s = 0.0
        self.param_pull_attempt_count = 0
        self.last_mode_request_time_s = 0.0
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.restore_complete = False
        self.original_takeoff_alt_m: Optional[float] = None
        self.takeoff_param_ready = False
        self.takeoff_param_changed = False

        self.pre_takeoff_profile: Optional[Px4ParamProfile] = None
        self.pre_takeoff_profile_error = ""
        self.pre_takeoff_profile_param_names: list[str] = []
        self.pre_takeoff_profile_applied_names: set[str] = set()
        self.pre_takeoff_profile_skipped_names: set[str] = set()

        self.speed_profile: Optional[Px4ParamProfile] = None
        self.speed_profile_error = ""
        self.speed_profile_param_names: list[str] = []
        self.speed_profile_original_values: dict[str, float] = {}
        self.speed_profile_applied_names: set[str] = set()
        self.speed_profile_changed_names: set[str] = set()
        self.speed_profile_skipped_names: set[str] = set()

        self.tracks: Dict[int, TrackSnapshot] = {}
        self.active_track_id: Optional[int] = None
        self.completed_track_ids: set[int] = set()
        self.blocked_track_ids: set[int] = set()

        self.perimeter_guard: Optional[PerimeterGuard] = None
        self.perimeter_guard_error = ""
        self.last_altitude_clamp_log_s = 0.0

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(
            EngagementState,
            self.engagement_state_topic,
            10,
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

        self.create_subscription(TargetTrackArray, self.tracks_topic, self.tracks_callback, 10)
        self.create_subscription(Bool, self.estop_topic, self.estop_callback, 10)
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
            Trigger,
            self.start_service_name,
            self.handle_start_request,
        )
        self.create_service(
            Trigger,
            self.abort_service_name,
            self.handle_abort_request,
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self.timer_callback,
        )

        self.load_perimeter_guard()
        self.load_pre_takeoff_profile()
        self.load_speed_profile()

        self.get_logger().info(
            "Milestone 3 demo sequence node started. "
            f"tracks={self.tracks_topic} cmd_vel={self.cmd_vel_topic} "
            f"state_topic={self.engagement_state_topic} "
            f"target_gates=conf>={self.min_track_confidence:.2f}, "
            f"dist={self.min_target_distance_m:.2f}-"
            f"{self.max_target_distance_m:.2f}m, "
            f"|y|<={self.max_abs_target_y_m:.2f}m, "
            f"|z|<={self.max_abs_target_z_m:.2f}m"
        )
        self.publish_engagement_state(now_s=self.now_s(), target=None)

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

    def estop_callback(self, msg: Bool) -> None:
        self.estop_latched = bool(msg.data)

    def tracks_callback(self, msg: TargetTrackArray) -> None:
        now_s = self.now_s()
        seen_ids = set()
        for track in msg.tracks:
            snapshot = TrackSnapshot(
                track_id=int(track.track_id),
                x_b_m=float(track.x_b_m),
                y_b_m=float(track.y_b_m),
                z_b_m=float(track.z_b_m),
                vx_b_mps=float(track.vx_b_mps),
                vy_b_mps=float(track.vy_b_mps),
                vz_b_mps=float(track.vz_b_mps),
                distance_m=float(track.distance_m),
                confidence=float(track.confidence),
                bbox_area_px=float(track.bbox_area_px),
                inbound=bool(track.inbound),
                last_seen_s=now_s,
                detector_track_id=int(
                    getattr(track, "detector_track_id", int(track.track_id))
                ),
                source=int(getattr(track, "source", 0)),
                last_observed_age_s=float(
                    getattr(track, "last_observed_age_s", 0.0)
                ),
                prediction_horizon_s=float(
                    getattr(track, "prediction_horizon_s", 0.0)
                ),
                position_uncertainty_m=float(
                    getattr(track, "position_uncertainty_m", 0.0)
                ),
                velocity_uncertainty_mps=float(
                    getattr(track, "velocity_uncertainty_mps", 0.0)
                ),
            )
            self.tracks[snapshot.track_id] = snapshot
            seen_ids.add(snapshot.track_id)

        gc_window_s = max(self.track_gc_s, self.track_timeout_s)
        for track_id in list(self.tracks.keys()):
            track = self.tracks[track_id]
            if track_id in seen_ids:
                continue
            if now_s - track.last_seen_s > gc_window_s:
                self.tracks.pop(track_id, None)
                if self.active_track_id == track_id:
                    self.active_track_id = None

    def load_perimeter_guard(self) -> None:
        self.perimeter_guard = None
        self.perimeter_guard_error = ""
        if not self.enable_perimeter_guard:
            self.get_logger().info("Perimeter guard disabled for milestone 3 demo.")
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

    def load_pre_takeoff_profile(self) -> None:
        self.pre_takeoff_profile = None
        self.pre_takeoff_profile_error = ""
        self.pre_takeoff_profile_param_names = []
        if not self.pre_takeoff_profile_config:
            return
        try:
            self.pre_takeoff_profile = Px4ParamProfile.load_from_yaml(
                self.pre_takeoff_profile_config
            )
        except Exception as exc:  # noqa: BLE001
            self.pre_takeoff_profile_error = str(exc)
            self.get_logger().error(
                "Failed to load pre-takeoff profile from "
                f"{self.pre_takeoff_profile_config}: {exc}"
            )
            return
        self.pre_takeoff_profile_param_names = list(
            self.pre_takeoff_profile.parameters.keys()
        )
        self.get_logger().info(
            "Loaded pre-takeoff profile: "
            f"{self.pre_takeoff_profile.name} params="
            f"{len(self.pre_takeoff_profile_param_names)}"
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

    def has_takeoff_return_target(self) -> bool:
        return (
            self.takeoff_origin_xy is not None
            and self.takeoff_origin_altitude_m is not None
            and self.takeoff_origin_yaw_rad is not None
        )

    def return_target_reached(self) -> bool:
        if not self.has_takeoff_return_target():
            return False

        current_xy = self.current_xy()
        target_xy = self.takeoff_origin_xy
        target_yaw_rad = self.takeoff_origin_yaw_rad
        assert target_xy is not None
        assert target_yaw_rad is not None

        horizontal_error_m = math.hypot(
            current_xy[0] - target_xy[0],
            current_xy[1] - target_xy[1],
        )
        altitude_error_m = abs(
            self.current_altitude_m() - self.takeoff_target_altitude_m()
        )
        yaw_error_rad = abs(
            wrap_angle_rad(target_yaw_rad - self.current_yaw_rad())
        )
        return (
            horizontal_error_m <= self.return_position_tolerance_m
            and altitude_error_m <= self.altitude_tolerance_m
            and yaw_error_rad <= self.return_yaw_tolerance_rad
        )

    def current_frame_id(self) -> str:
        return str(self.latest_pose.header.frame_id or "map").strip()

    def pose_fresh(self) -> bool:
        timeout_s = self.local_pose_timeout_s
        if self.demo_state in {
            "SYNC_TAKEOFF_PARAM",
            "SYNC_PRE_TAKEOFF_PROFILE",
            "SYNC_SPEED_PROFILE",
            "RESTORE_TAKEOFF_PARAM",
            "RESTORE_SPEED_PROFILE",
        }:
            timeout_s = max(timeout_s, self.local_pose_timeout_during_param_sync_s)
        return (self.now_s() - self.last_pose_time_s) <= timeout_s

    def active_state_timeout_s(self) -> float:
        timeout_s = self.state_timeout_s
        if self.demo_state in {
            "SYNC_TAKEOFF_PARAM",
            "SYNC_PRE_TAKEOFF_PROFILE",
            "SET_TAKEOFF_MODE",
            "ARMING",
            "TAKEOFF",
            "SYNC_SPEED_PROFILE",
            "RESTORE_TAKEOFF_PARAM",
            "RESTORE_SPEED_PROFILE",
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
            "SYNC_PRE_TAKEOFF_PROFILE",
            "SET_TAKEOFF_MODE",
            "ARMING",
            "TAKEOFF",
            "SYNC_SPEED_PROFILE",
            "RESTORE_TAKEOFF_PARAM",
            "RESTORE_SPEED_PROFILE",
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
        return mode_name in {"AUTO.LOITER", "POSCTL", "OFFBOARD"}

    def takeoff_target_altitude_m(self) -> float:
        if self.takeoff_origin_altitude_m is None:
            return self.takeoff_altitude_m
        return self.takeoff_origin_altitude_m + self.takeoff_altitude_m

    def takeoff_altitude_reached(self) -> bool:
        return self.current_altitude_m() >= (
            self.takeoff_target_altitude_m() - self.altitude_tolerance_m
        )

    def touchdown_threshold_altitude_m(self) -> float:
        if self.takeoff_origin_altitude_m is None:
            return self.touchdown_altitude_m
        return self.takeoff_origin_altitude_m + self.touchdown_altitude_m

    def zero_cmd(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def publish_zero_command(self) -> None:
        self.cmd_pub.publish(self.zero_cmd())

    def publish_follow_command(self, command: FollowCommand) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(command.vx)
        msg.twist.linear.y = float(command.vy)
        msg.twist.linear.z = float(command.vz)
        msg.twist.angular.z = float(command.yaw_rate)
        self.cmd_pub.publish(msg)

    def publish_takeoff_hold_command(self) -> None:
        target_xy = self.climb_hold_xy or self.current_xy()
        current_altitude_m = self.current_altitude_m()
        target_yaw_rad = self.climb_hold_yaw_rad
        if target_yaw_rad is None:
            target_yaw_rad = self.current_yaw_rad()
        current_yaw_rad = self.current_yaw_rad()
        command = compute_return_to_point_command(
            current_xy=self.current_xy(),
            current_altitude_m=current_altitude_m,
            current_yaw_rad=current_yaw_rad,
            target_xy=target_xy,
            target_altitude_m=self.takeoff_target_altitude_m(),
            target_yaw_rad=target_yaw_rad,
            xy_deadband_m=self.return_position_tolerance_m,
            z_deadband_m=self.altitude_tolerance_m,
            yaw_deadband_rad=self.yaw_deadband_rad,
            kp_xy=self.kp_xy,
            kp_z=self.kp_z,
            kp_yaw=self.kp_yaw,
            max_vel_xy_mps=self.max_vel_xy_mps,
            max_vel_z_mps=self.max_vel_z_mps,
            max_yaw_rate_rps=self.max_yaw_rate_rps,
        )
        command = self.clamp_command_to_perimeter_altitude(command)
        projection_reason = self.command_projection_violation_reason(command)
        if projection_reason is not None:
            self.last_blocked_reason = projection_reason
            self.publish_zero_command()
            return

        self.min_distance_gate_active = False
        self.publish_follow_command(command)

    def clear_active_target(self) -> None:
        self.active_track_id = None
        self.dwell_started_s = None
        self.dwell_elapsed_s = 0.0
        self.dwell_remaining_s = self.dwell_time_s
        self.min_distance_gate_active = False

    def set_demo_state(self, new_state: str, reason: str = "") -> None:
        old_state = self.demo_state
        self.demo_state = new_state
        self.stage_started_s = self.now_s()
        if new_state == "TAKEOFF":
            self.last_takeoff_progress_log_s = 0.0
        if new_state == "CLIMB_TO_TAKEOFF_ALTITUDE":
            self.climb_hold_xy = self.current_xy()
            self.climb_hold_yaw_rad = self.current_yaw_rad()
        elif old_state == "CLIMB_TO_TAKEOFF_ALTITUDE":
            self.climb_hold_xy = None
            self.climb_hold_yaw_rad = None
        elif new_state in {"IDLE", "ABORT", "COMPLETE"}:
            self.climb_hold_xy = None
            self.climb_hold_yaw_rad = None
        if new_state != "WAIT_TOUCHDOWN":
            self.touchdown_started_s = None
        if reason:
            self.get_logger().info(f"State {old_state} -> {new_state}: {reason}")
        elif old_state != new_state:
            self.get_logger().info(f"State {old_state} -> {new_state}")

    def stage_elapsed_s(self) -> float:
        return self.now_s() - self.stage_started_s

    def stage_timeout_limit_s(self) -> Optional[float]:
        if self.demo_state in self.TERMINAL_STATES:
            return None
        if self.demo_state in {
            "SYNC_TAKEOFF_PARAM",
            "SYNC_PRE_TAKEOFF_PROFILE",
            "SYNC_SPEED_PROFILE",
            "RESTORE_TAKEOFF_PARAM",
            "RESTORE_SPEED_PROFILE",
        }:
            return max(self.stage_timeout_s, self.param_sync_timeout_s)
        if self.demo_state in {"SEARCH", "FOLLOW", "DWELL"}:
            return None
        if self.demo_state in {"WAIT_TOUCHDOWN", "DISARMING"}:
            return max(self.stage_timeout_s, 25.0)
        return self.stage_timeout_s

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
            or name in self.speed_profile_skipped_names
            for name in self.speed_profile_param_names
        )

    def next_speed_profile_param_to_apply(self) -> Optional[str]:
        for param_name in self.speed_profile_param_names:
            if (
                param_name not in self.speed_profile_applied_names
                and param_name not in self.speed_profile_skipped_names
            ):
                return param_name
        return None

    def next_speed_profile_param_to_restore(self) -> Optional[str]:
        for param_name in self.speed_profile_param_names:
            if param_name in self.speed_profile_changed_names:
                return param_name
        return None

    def pre_takeoff_profile_apply_complete(self) -> bool:
        if self.pre_takeoff_profile is None:
            return True
        return all(
            name in self.pre_takeoff_profile_applied_names
            or name in self.pre_takeoff_profile_skipped_names
            for name in self.pre_takeoff_profile_param_names
        )

    def next_pre_takeoff_param_to_apply(self) -> Optional[str]:
        for param_name in self.pre_takeoff_profile_param_names:
            if (
                param_name not in self.pre_takeoff_profile_applied_names
                and param_name not in self.pre_takeoff_profile_skipped_names
            ):
                return param_name
        return None

    def skip_pre_takeoff_profile_param(self, param_name: str, reason: str) -> None:
        if param_name in self.pre_takeoff_profile_skipped_names:
            return
        self.pre_takeoff_profile_skipped_names.add(param_name)
        self.pre_takeoff_profile_applied_names.discard(param_name)
        self.get_logger().warn(
            f"Skipping pre-takeoff profile parameter {param_name}: {reason}"
        )

    def skip_speed_profile_param(self, param_name: str, reason: str) -> None:
        if param_name in self.speed_profile_skipped_names:
            return
        self.speed_profile_skipped_names.add(param_name)
        self.speed_profile_original_values.pop(param_name, None)
        self.speed_profile_changed_names.discard(param_name)
        self.speed_profile_applied_names.discard(param_name)
        self.get_logger().warn(
            f"Skipping speed profile parameter {param_name}: {reason}"
        )

    @staticmethod
    def is_missing_param_reason(reason: str) -> bool:
        reason_text = str(reason).lower()
        return "undeclared" in reason_text or "unknown parameter" in reason_text

    def skip_takeoff_param_restore(self, reason: str) -> None:
        self.restore_complete = True
        self.takeoff_param_changed = False
        self.original_takeoff_alt_m = None
        self.get_logger().warn(
            f"Skipping takeoff-parameter restore for {self.takeoff_param_id}: {reason}"
        )

    def perimeter_start_violation_reason(self) -> Optional[str]:
        if not self.enable_perimeter_guard:
            return None
        if self.perimeter_guard is None:
            return self.perimeter_guard_error or "perimeter guard is unavailable"
        current_frame = self.current_frame_id()
        if current_frame and not self.perimeter_guard.frame_matches(current_frame):
            return (
                f"local pose frame '{current_frame}' does not match perimeter frame "
                f"'{self.perimeter_guard.frame_id}'"
            )
        current_xy = self.current_xy()
        start_reason = self.perimeter_guard.xy_violation_reason(
            current_xy[0],
            current_xy[1],
            boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
            boundary_margin_m=self.perimeter_boundary_margin_m,
            keep_out_margin_m=self.perimeter_keep_out_margin_m,
        )
        if start_reason is not None:
            return f"start position is unsafe: {start_reason}"
        return self.perimeter_guard.goal_altitude_violation_reason(
            self.takeoff_target_altitude_m()
        )

    def perimeter_runtime_violation_reason(self) -> Optional[str]:
        if not self.enable_perimeter_guard:
            return None
        if self.perimeter_guard is None:
            return self.perimeter_guard_error or "perimeter guard is unavailable"

        current_frame = self.current_frame_id()
        if current_frame and not self.perimeter_guard.frame_matches(current_frame):
            return (
                f"local pose frame '{current_frame}' does not match perimeter frame "
                f"'{self.perimeter_guard.frame_id}'"
            )
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

    def command_projection_violation_reason(
        self,
        command: FollowCommand,
    ) -> Optional[str]:
        if not self.enable_perimeter_guard or self.perimeter_guard is None:
            return None
        current_xy = self.current_xy()
        predicted_xy = project_body_velocity_to_world_xy(
            current_xy=current_xy,
            yaw_rad=self.current_yaw_rad(),
            vx_mps=command.vx,
            vy_mps=command.vy,
            horizon_s=self.projected_path_horizon_s,
        )
        path_reason = self.perimeter_guard.segment_violation_reason(
            current_xy,
            predicted_xy,
            sample_step_m=self.perimeter_segment_sample_step_m,
            boundary_tolerance_m=self.perimeter_boundary_tolerance_m,
            boundary_margin_m=self.perimeter_boundary_margin_m,
            keep_out_margin_m=self.perimeter_keep_out_margin_m,
        )
        if path_reason is not None:
            return path_reason
        predicted_z = self.current_altitude_m() + (
            command.vz * max(self.projected_path_horizon_s, 0.0)
        )
        return self.perimeter_guard.goal_altitude_violation_reason(predicted_z)

    def clamp_command_to_perimeter_altitude(
        self,
        command: FollowCommand,
    ) -> FollowCommand:
        if not self.enable_perimeter_guard or self.perimeter_guard is None:
            return command

        altitude_limits = self.perimeter_guard.altitude_limits
        clamped_command = clamp_follow_command_altitude(
            command,
            current_altitude_m=self.current_altitude_m(),
            projected_horizon_s=self.projected_path_horizon_s,
            min_z_m=altitude_limits.min_z_m,
            max_z_m=altitude_limits.max_z_m,
        )
        if clamped_command.vz == command.vz:
            return command

        now_s = self.now_s()
        if (now_s - self.last_altitude_clamp_log_s) >= 1.0:
            active_track_id = self.active_track_id
            self.get_logger().info(
                "Clamped follow vertical command to respect perimeter altitude "
                f"limits: id={active_track_id} vz {command.vz:.2f} -> "
                f"{clamped_command.vz:.2f}"
            )
            self.last_altitude_clamp_log_s = now_s
        return clamped_command

    def control_block_reason(self) -> str:
        if self.estop_latched:
            return "ESTOP"
        if not self.state_fresh():
            return "STALE_STATE"
        if self.require_mavros_connected and not self.latest_state.connected:
            return "MAVROS_DISCONNECTED"
        if not self.latest_state.armed:
            return "NOT_ARMED"
        if str(self.latest_state.mode).upper() != self.offboard_mode:
            return f"MODE_{str(self.latest_state.mode or 'UNKNOWN').upper()}"
        if self.local_pose_timeout_s > 0.0 and not self.pose_fresh():
            return "STALE_LOCAL_POSE"
        if self.require_companion_active:
            if not self.companion_status_fresh():
                return "STALE_COMPANION_STATUS"
            if not self.companion_active:
                return "COMPANION_INACTIVE"
        return ""

    def select_target(self, now_s: float) -> Optional[TrackSnapshot]:
        excluded_track_ids = self.completed_track_ids | self.blocked_track_ids
        chosen = select_sequential_target(
            self.tracks.values(),
            active_track_id=self.active_track_id,
            excluded_track_ids=excluded_track_ids,
            now_s=now_s,
            track_timeout_s=self.track_timeout_s,
            min_track_confidence=self.min_track_confidence,
            max_target_distance_m=self.max_target_distance_m,
            min_target_distance_m=self.min_target_distance_m,
            require_target_in_front=self.require_target_in_front,
            max_abs_target_y_m=self.max_abs_target_y_m,
            max_abs_target_z_m=self.max_abs_target_z_m,
            allow_predicted_tracks=self.enable_predicted_track_control,
            max_predicted_track_age_s=self.predicted_track_hold_s,
            max_predicted_position_uncertainty_m=(
                self.max_predicted_position_uncertainty_m
            ),
        )
        if chosen is None:
            if self.active_track_id is not None:
                self.get_logger().info(
                    f"Clearing target lock: id={self.active_track_id}"
                )
            self.clear_active_target()
            return None
        if self.active_track_id != chosen.track_id:
            self.active_track_id = chosen.track_id
            self.dwell_started_s = None
            self.dwell_elapsed_s = 0.0
            self.dwell_remaining_s = self.dwell_time_s
            self.min_distance_gate_active = False
            self.get_logger().info(
                "Target lock: "
                f"id={chosen.track_id} dist={chosen.distance_m:.2f} "
                f"x={chosen.x_b_m:.2f} y={chosen.y_b_m:.2f} z={chosen.z_b_m:.2f}"
            )
        return chosen

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
        if self.pre_takeoff_profile_config and self.pre_takeoff_profile is None:
            return (
                False,
                self.pre_takeoff_profile_error or "pre-takeoff profile is unavailable",
            )
        if self.use_speed_profile and self.speed_profile is None:
            return False, self.speed_profile_error or "speed profile is unavailable"
        if not self.state_fresh():
            return False, "MAVROS state is stale or missing"
        if self.require_mavros_connected and not self.latest_state.connected:
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
        perimeter_reason = self.perimeter_start_violation_reason()
        if perimeter_reason is not None:
            return False, perimeter_reason
        return True, "ready"

    def handle_start_request(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        ready, reason = self.startable()
        if not ready:
            response.success = False
            response.message = reason
            return response

        self.abort_reason = ""
        self.last_blocked_reason = ""
        self.takeoff_origin_xy = self.current_xy()
        self.takeoff_origin_altitude_m = self.current_altitude_m()
        self.takeoff_origin_yaw_rad = self.current_yaw_rad()
        self.climb_hold_xy = None
        self.climb_hold_yaw_rad = None
        self.touchdown_started_s = None
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
        self.next_param_pull_attempt_s = 0.0
        self.param_pull_attempt_count = 0
        self.connection_lost_since_s = None
        self.connection_loss_warned = False
        self.last_mode_request_time_s = 0.0
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.restore_complete = False
        self.original_takeoff_alt_m = None
        self.takeoff_param_ready = False
        self.takeoff_param_changed = False
        self.pre_takeoff_profile_applied_names = set()
        self.pre_takeoff_profile_skipped_names = set()
        self.speed_profile_original_values = {}
        self.speed_profile_applied_names = set()
        self.speed_profile_changed_names = set()
        self.speed_profile_skipped_names = set()
        self.completed_track_ids.clear()
        self.blocked_track_ids.clear()
        self.clear_active_target()
        self.set_demo_state("SYNC_TAKEOFF_PARAM", "start requested")
        response.success = True
        response.message = "milestone3 demo sequence started"
        return response

    def handle_abort_request(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if self.demo_state in {"IDLE", "COMPLETE"}:
            response.success = False
            response.message = "milestone3 demo is not active"
            return response
        self.enter_abort("operator abort")
        response.success = True
        response.message = "abort accepted"
        return response

    def enter_abort(self, reason: str) -> None:
        if self.demo_state == "ABORT" and self.abort_reason:
            return
        self.abort_reason = reason
        self.last_blocked_reason = ""
        self.clear_active_target()
        self.publish_zero_command()
        self.set_demo_state("ABORT", reason)

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
            allow_landing_disarm_handoff = (
                not arm_value
                and self.demo_state in {"WAIT_TOUCHDOWN", "DISARMING"}
                and (
                    self.is_land_mode()
                    or self.current_altitude_m()
                    <= self.touchdown_threshold_altitude_m()
                )
            )
            try:
                response = self.arm_future.result()
                if not response.success:
                    if self.latest_state.armed == arm_value:
                        self.get_logger().warn(
                            f"{action.capitalize()} request reported failure "
                            f"(result {response.result}) but the vehicle is already "
                            f"{'armed' if arm_value else 'disarmed'}; continuing."
                        )
                    elif allow_landing_disarm_handoff:
                        self.get_logger().warn(
                            "Disarm request reported failure "
                            f"(result {response.result}) while PX4 is landing; "
                            "continuing to wait for auto-disarm."
                        )
                    else:
                        self.enter_abort(
                            f"{action} request was rejected "
                            f"(result {response.result}, mode={self.latest_state.mode})"
                        )
                else:
                    self.get_logger().info(f"{action.capitalize()} request accepted")
            except Exception as exc:
                if self.latest_state.armed == arm_value:
                    self.get_logger().warn(
                        f"{action.capitalize()} request raised after the vehicle was "
                        f"already {'armed' if arm_value else 'disarmed'}: {exc}"
                    )
                elif allow_landing_disarm_handoff:
                    self.get_logger().warn(
                        "Disarm request raised while PX4 is landing; continuing to "
                        f"wait for auto-disarm: {exc}"
                    )
                else:
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
                elif future_kind == "PRE_TAKEOFF_PROFILE_SET_TARGET":
                    if not response.results or not response.results[0].successful:
                        reason = (
                            response.results[0].reason
                            if response.results
                            else "no response"
                        )
                        if "undeclared" in str(reason).lower():
                            self.skip_pre_takeoff_profile_param(param_name, reason)
                            return
                        self.enter_abort(f"failed to set {param_name}: {reason}")
                        return
                    self.pre_takeoff_profile_applied_names.add(param_name)
                    self.get_logger().info(
                        f"Set pre-takeoff baseline {param_name} "
                        f"to {self.pending_param_value:.2f}"
                    )
                elif future_kind == "PROFILE_GET_ORIGINAL":
                    if not response.values:
                        self.skip_speed_profile_param(
                            param_name,
                            "it was not returned by MAVROS",
                        )
                        return
                    if not parameter_value_is_declared_numeric(response.values[0]):
                        self.skip_speed_profile_param(
                            param_name,
                            "it is not declared on MAVROS",
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
                        if self.is_missing_param_reason(reason):
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
                        if self.is_missing_param_reason(reason):
                            self.skip_speed_profile_param(
                                param_name,
                                f"set failed because it is not declared: {reason}",
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
                        if self.is_missing_param_reason(reason):
                            self.skip_takeoff_param_restore(
                                f"restore failed because it is not declared: {reason}"
                            )
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
                        if self.is_missing_param_reason(reason):
                            self.skip_speed_profile_param(
                                param_name,
                                f"restore failed because it is not declared: {reason}",
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

        if not requires_runtime_tracking_guards(
            self.demo_state,
            armed=bool(self.latest_state.armed),
        ):
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
            self.demo_state
            in {"CLIMB_TO_TAKEOFF_ALTITUDE", "STAGE_HOVER"}
            | self.ACTIVE_ENGAGEMENT_STATES
            | self.RETURN_STATES
            and not self.latest_state.armed
        ):
            self.enter_abort(
                f"vehicle is not armed in {self.demo_state} while mode={self.latest_state.mode}"
            )
            return

        if (
            self.demo_state
            in {"CLIMB_TO_TAKEOFF_ALTITUDE", "STAGE_HOVER"}
            | self.ACTIVE_ENGAGEMENT_STATES
            | self.RETURN_STATES
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

    def publish_engagement_state(
        self,
        *,
        now_s: float,
        target: Optional[TrackSnapshot],
    ) -> None:
        msg = EngagementState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.scenario_id = self.scenario_id
        msg.state = self.demo_state
        msg.autonomy_enabled = self.demo_state not in self.TERMINAL_STATES

        blocked_reason = ""
        if self.demo_state == "ABORT":
            blocked_reason = self.abort_reason
        elif self.demo_state in self.ACTIVE_ENGAGEMENT_STATES | self.RETURN_STATES:
            blocked_reason = self.last_blocked_reason or self.control_block_reason()
        elif self.demo_state == "IDLE":
            blocked_reason = ""
        elif self.estop_latched:
            blocked_reason = "ESTOP"

        msg.autonomy_ready = (
            self.demo_state
            in {"STAGE_HOVER"} | self.ACTIVE_ENGAGEMENT_STATES | self.RETURN_STATES
            and not blocked_reason
        )
        msg.blocked_reason = blocked_reason
        msg.estop_latched = bool(self.estop_latched)
        msg.obstacle_blocked = False
        msg.min_distance_gate_active = bool(self.min_distance_gate_active)
        msg.active_track_id = (
            int(self.active_track_id) if self.active_track_id is not None else -1
        )
        msg.active_distance_m = float(target.distance_m) if target is not None else 0.0
        msg.active_bearing_rad = (
            float(target_bearing_rad(target)) if target is not None else 0.0
        )
        msg.target_visible = bool(
            target is not None and target.source != TRACK_SOURCE_PREDICTED
        )
        msg.dwell_remaining_s = float(self.dwell_remaining_s)
        msg.dwell_elapsed_s = float(self.dwell_elapsed_s)
        msg.completed_targets_count = int(len(self.completed_track_ids))
        msg.required_targets_count = int(self.required_completion_count)
        self.state_pub.publish(msg)

    def step_state_machine(self) -> Optional[TrackSnapshot]:
        now_s = self.now_s()

        if self.demo_state == "IDLE":
            return None

        if self.demo_state in {
            "SYNC_TAKEOFF_PARAM",
            "SYNC_PRE_TAKEOFF_PROFILE",
            "SET_TAKEOFF_MODE",
            "ARMING",
            "REQUEST_TAKEOFF",
            "TAKEOFF",
            "STAGE_HOVER",
            "RETURN_HOME_HOLD",
        }:
            self.publish_zero_command()

        if self.demo_state in {
            "WARMUP_OFFBOARD",
            "SET_OFFBOARD_MODE",
            "SYNC_SPEED_PROFILE",
            "CLIMB_TO_TAKEOFF_ALTITUDE",
        }:
            if self.takeoff_altitude_reached():
                self.publish_zero_command()
            else:
                self.publish_takeoff_hold_command()

        if self.demo_state == "SYNC_TAKEOFF_PARAM":
            if self.takeoff_param_ready:
                if self.pre_takeoff_profile is not None:
                    self.set_demo_state(
                        "SYNC_PRE_TAKEOFF_PROFILE",
                        "takeoff altitude configured",
                    )
                else:
                    self.set_demo_state(
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
            return None

        if self.demo_state == "SYNC_PRE_TAKEOFF_PROFILE":
            if self.pre_takeoff_profile is None:
                self.set_demo_state("SET_TAKEOFF_MODE", "pre-takeoff profile disabled")
                return None
            if self.pre_takeoff_profile_apply_complete():
                self.set_demo_state("SET_TAKEOFF_MODE", "pre-takeoff profile configured")
                return None
            if self.param_future is not None:
                return None
            param_name = self.next_pre_takeoff_param_to_apply()
            if param_name is None:
                self.set_demo_state("SET_TAKEOFF_MODE", "pre-takeoff profile configured")
                return None
            target_value = self.pre_takeoff_profile.parameters[param_name]
            self.request_param_set(
                target_value,
                kind="PRE_TAKEOFF_PROFILE_SET_TARGET",
                param_name=param_name,
            )
            return None

        if self.demo_state == "SET_TAKEOFF_MODE":
            if self.is_takeoff_mode():
                self.set_demo_state("ARMING", "AUTO.TAKEOFF confirmed")
            elif self.mode_future is None and self.mode_request_retry_ready():
                self.request_mode(self.takeoff_mode)
            return None

        if self.demo_state == "ARMING":
            if self.latest_state.armed:
                if self.takeoff_strategy == "LOCAL_COMMAND":
                    self.set_demo_state("REQUEST_TAKEOFF", "vehicle armed")
                else:
                    self.set_demo_state(
                        "TAKEOFF",
                        "vehicle armed; waiting for PX4 AUTO.TAKEOFF climb",
                    )
                return None
            if (
                not self.is_takeoff_mode()
                and self.mode_future is None
                and self.mode_request_retry_ready()
            ):
                self.request_mode(self.takeoff_mode)
                return None
            if (
                self.stage_elapsed_s() >= self.arm_zero_throttle_hold_s
                and not self.arm_request_sent
            ):
                self.arm_request_sent = True
                self.request_arm(True)
            return None

        if self.demo_state == "REQUEST_TAKEOFF":
            if not self.is_takeoff_mode():
                if self.mode_future is None:
                    self.request_mode(self.takeoff_mode)
                return None
            if self.takeoff_request_accepted:
                self.set_demo_state("TAKEOFF", "local takeoff command accepted")
                return None
            if not self.takeoff_request_sent and self.takeoff_future is None:
                self.takeoff_request_sent = True
                self.request_takeoff()
            return None

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
            if self.takeoff_altitude_reached():
                self.set_demo_state("WARMUP_OFFBOARD", "target altitude reached")
                return None
            if self.is_takeoff_handoff_mode():
                shortfall_m = max(target_altitude_m - current_altitude_m, 0.0)
                if shortfall_m > self.altitude_tolerance_m:
                    self.get_logger().warn(
                        "PX4 exited AUTO.TAKEOFF before the demo observed the "
                        f"target altitude; shortfall={shortfall_m:.2f} m "
                        f"mode={self.latest_state.mode}. Continuing climb in OFFBOARD."
                    )
                self.set_demo_state(
                    "WARMUP_OFFBOARD",
                    f"PX4 handed off to {self.latest_state.mode} during takeoff",
                )
            return None

        if self.demo_state == "WARMUP_OFFBOARD":
            if self.stage_elapsed_s() >= self.offboard_warmup_s:
                self.set_demo_state("SET_OFFBOARD_MODE", "offboard warmup complete")
            return None

        if self.demo_state == "SET_OFFBOARD_MODE":
            if self.mode_matches(self.offboard_mode):
                if self.use_speed_profile:
                    self.set_demo_state("SYNC_SPEED_PROFILE", "OFFBOARD confirmed")
                elif self.takeoff_altitude_reached():
                    self.set_demo_state("STAGE_HOVER", "OFFBOARD confirmed")
                else:
                    self.set_demo_state(
                        "CLIMB_TO_TAKEOFF_ALTITUDE",
                        "OFFBOARD confirmed below takeoff altitude",
                    )
            elif self.mode_future is None and self.mode_request_retry_ready():
                self.request_mode(self.offboard_mode)
            return None

        if self.demo_state == "SYNC_SPEED_PROFILE":
            if not self.use_speed_profile or self.speed_profile is None:
                if self.takeoff_altitude_reached():
                    self.set_demo_state("STAGE_HOVER", "speed profile disabled")
                else:
                    self.set_demo_state(
                        "CLIMB_TO_TAKEOFF_ALTITUDE",
                        "speed profile disabled below takeoff altitude",
                    )
                return None
            if self.speed_profile_apply_complete():
                if self.takeoff_altitude_reached():
                    self.set_demo_state("STAGE_HOVER", "speed profile configured")
                else:
                    self.set_demo_state(
                        "CLIMB_TO_TAKEOFF_ALTITUDE",
                        "speed profile configured below takeoff altitude",
                    )
                return None
            if self.param_future is not None or self.param_pull_future is not None:
                return None
            if (
                not self.param_mirror_ready
                and not self.allow_param_lookup_without_full_mirror
                and self.now_s() >= self.next_param_pull_attempt_s
            ):
                self.request_param_pull()
                return None
            param_name = self.next_speed_profile_param_to_apply()
            if param_name is None:
                if self.takeoff_altitude_reached():
                    self.set_demo_state("STAGE_HOVER", "speed profile configured")
                else:
                    self.set_demo_state(
                        "CLIMB_TO_TAKEOFF_ALTITUDE",
                        "speed profile configured below takeoff altitude",
                    )
                return None
            if param_name not in self.speed_profile_original_values:
                self.request_param_get(
                    param_name=param_name,
                    kind="PROFILE_GET_ORIGINAL",
                )
                return None
            original_value = self.speed_profile_original_values[param_name]
            target_value = self.speed_profile.parameters[param_name]
            if math.isclose(original_value, target_value, abs_tol=1e-3):
                self.speed_profile_applied_names.add(param_name)
                return None
            self.request_param_set(
                target_value,
                kind="PROFILE_SET_TARGET",
                param_name=param_name,
            )
            return None

        if self.demo_state == "CLIMB_TO_TAKEOFF_ALTITUDE":
            if self.takeoff_altitude_reached():
                self.set_demo_state(
                    "STAGE_HOVER",
                    "takeoff altitude reached in OFFBOARD",
                )
            elif self.stage_elapsed_s() >= self.climb_handoff_timeout_s:
                self.set_demo_state(
                    "STAGE_HOVER",
                    "climb handoff timeout reached",
                )
            return None

        if self.demo_state == "STAGE_HOVER":
            if self.stage_elapsed_s() >= self.stage_hover_duration_s:
                self.set_demo_state("SEARCH", "stage hover complete")
            return None

        if self.demo_state in self.ACTIVE_ENGAGEMENT_STATES:
            control_block = self.control_block_reason()
            if control_block:
                self.last_blocked_reason = control_block
                self.clear_active_target()
                if self.publish_zero_on_block:
                    self.publish_zero_command()
                self.set_demo_state("SEARCH")
                return None

            target = self.select_target(now_s)
            if target is None:
                self.last_blocked_reason = ""
                self.publish_zero_command()
                self.set_demo_state("SEARCH")
                return None

            command = compute_follow_command(
                target,
                follow_distance_m=self.follow_distance_m,
                follow_distance_tolerance_m=self.follow_distance_tolerance_m,
                lateral_deadband_m=self.lateral_deadband_m,
                vertical_deadband_m=self.vertical_deadband_m,
                yaw_deadband_rad=self.yaw_deadband_rad,
                kp_xy=self.kp_xy,
                kp_z=self.kp_z,
                kp_yaw=self.kp_yaw,
                max_vel_xy_mps=self.max_vel_xy_mps,
                max_vel_z_mps=self.max_vel_z_mps,
                max_yaw_rate_rps=self.max_yaw_rate_rps,
                min_safe_distance_m=self.min_safe_distance_m,
            )
            if target.source == TRACK_SOURCE_PREDICTED:
                command = clamp_follow_command_velocity(
                    command,
                    max_vel_xy_mps=self.predicted_max_vel_xy_mps,
                    max_vel_z_mps=self.predicted_max_vel_z_mps,
                    max_yaw_rate_rps=self.predicted_max_yaw_rate_rps,
                )
            command = self.clamp_command_to_perimeter_altitude(command)
            self.min_distance_gate_active = command.min_distance_gate_active

            projection_reason = self.command_projection_violation_reason(command)
            if projection_reason is not None:
                active_track_id = self.active_track_id
                if (
                    active_track_id is not None
                    and self.block_target_on_perimeter_violation
                ):
                    self.blocked_track_ids.add(active_track_id)
                    self.get_logger().warn(
                        "Blocking target after projected perimeter conflict: "
                        f"id={active_track_id} reason={projection_reason}"
                    )
                self.last_blocked_reason = projection_reason
                self.clear_active_target()
                self.publish_zero_command()
                self.set_demo_state("SEARCH", projection_reason)
                return None

            in_window = distance_in_standoff_window(
                target.distance_m,
                self.follow_distance_m,
                self.follow_distance_tolerance_m,
            )
            (
                self.dwell_started_s,
                self.dwell_elapsed_s,
                self.dwell_remaining_s,
                dwell_complete,
            ) = update_dwell_progress(
                self.dwell_started_s,
                in_standoff_window=in_window,
                now_s=now_s,
                dwell_time_s=self.dwell_time_s,
            )
            self.last_blocked_reason = ""
            self.publish_follow_command(command)

            if in_window:
                self.set_demo_state("DWELL")
            else:
                self.set_demo_state("FOLLOW")

            if dwell_complete and target.track_id is not None:
                completed_track_id = int(target.track_id)
                self.completed_track_ids.add(completed_track_id)
                self.get_logger().info(
                    "Target complete: "
                    f"id={completed_track_id} completed="
                    f"{len(self.completed_track_ids)}/"
                    f"{self.required_completion_count}"
                )
                self.clear_active_target()
                if len(self.completed_track_ids) >= self.required_completion_count:
                    next_state = completion_next_state(
                        return_to_takeoff_on_complete=(
                            self.return_to_takeoff_on_complete
                        ),
                        has_takeoff_return_target=self.has_takeoff_return_target(),
                        land_on_complete=self.land_on_complete,
                    )
                    if next_state == "RETURN_TO_START":
                        self.set_demo_state(
                            next_state,
                            "all required targets completed; returning to takeoff origin",
                        )
                    else:
                        self.set_demo_state(
                            next_state,
                            "all required targets completed",
                        )
                else:
                    self.set_demo_state(
                        "SEARCH",
                        f"completed track {completed_track_id}",
                    )
            return target

        if self.demo_state == "RETURN_TO_START":
            control_block = self.control_block_reason()
            if control_block:
                self.last_blocked_reason = control_block
                self.publish_zero_command()
                return None

            if not self.has_takeoff_return_target():
                self.last_blocked_reason = ""
                next_state = post_return_next_state(
                    land_on_complete=self.land_on_complete
                )
                self.set_demo_state(
                    next_state,
                    "takeoff origin unavailable; skipping return",
                )
                return None

            if self.return_target_reached():
                self.last_blocked_reason = ""
                self.publish_zero_command()
                self.set_demo_state("RETURN_HOME_HOLD", "takeoff origin reached")
                return None

            target_xy = self.takeoff_origin_xy
            target_yaw_rad = self.takeoff_origin_yaw_rad
            assert target_xy is not None
            assert target_yaw_rad is not None
            command = compute_return_to_point_command(
                current_xy=self.current_xy(),
                current_altitude_m=self.current_altitude_m(),
                current_yaw_rad=self.current_yaw_rad(),
                target_xy=target_xy,
                target_altitude_m=self.takeoff_target_altitude_m(),
                target_yaw_rad=target_yaw_rad,
                xy_deadband_m=self.return_position_tolerance_m,
                z_deadband_m=self.altitude_tolerance_m,
                yaw_deadband_rad=self.yaw_deadband_rad,
                kp_xy=self.kp_xy,
                kp_z=self.kp_z,
                kp_yaw=self.kp_yaw,
                max_vel_xy_mps=self.max_vel_xy_mps,
                max_vel_z_mps=self.max_vel_z_mps,
                max_yaw_rate_rps=self.max_yaw_rate_rps,
            )
            command = self.clamp_command_to_perimeter_altitude(command)
            projection_reason = self.command_projection_violation_reason(command)
            if projection_reason is not None:
                self.last_blocked_reason = projection_reason
                self.publish_zero_command()
                self.enter_abort(f"return-to-start blocked: {projection_reason}")
                return None

            self.last_blocked_reason = ""
            self.min_distance_gate_active = False
            self.publish_follow_command(command)
            return None

        if self.demo_state == "RETURN_HOME_HOLD":
            control_block = self.control_block_reason()
            if control_block:
                self.last_blocked_reason = control_block
            else:
                self.last_blocked_reason = ""
            self.min_distance_gate_active = False
            if not control_block and self.stage_elapsed_s() >= self.return_hold_duration_s:
                next_state = post_return_next_state(
                    land_on_complete=self.land_on_complete
                )
                reason = (
                    "return hold complete; requesting landing"
                    if next_state == "SET_LAND_MODE"
                    else "return hold complete"
                )
                self.set_demo_state(next_state, reason)
            return None

        if self.demo_state == "SET_LAND_MODE":
            if self.is_land_mode():
                self.set_demo_state("WAIT_TOUCHDOWN", "AUTO.LAND confirmed")
            elif self.mode_future is None and self.mode_request_retry_ready():
                self.request_mode(self.land_mode)
            return None

        if self.demo_state == "WAIT_TOUCHDOWN":
            if not self.latest_state.armed:
                if self.takeoff_param_changed and self.restore_takeoff_alt_on_exit:
                    self.set_demo_state(
                        "RESTORE_TAKEOFF_PARAM",
                        "vehicle auto-disarmed after landing",
                    )
                elif self.should_restore_speed_profile():
                    self.set_demo_state(
                        "RESTORE_SPEED_PROFILE",
                        "vehicle auto-disarmed after landing",
                    )
                else:
                    self.set_demo_state("COMPLETE", "vehicle auto-disarmed after landing")
                return None
            if self.current_altitude_m() <= self.touchdown_threshold_altitude_m():
                if self.touchdown_started_s is None:
                    self.touchdown_started_s = self.now_s()
                elif (
                    self.now_s() - self.touchdown_started_s
                ) >= self.touchdown_dwell_s:
                    self.set_demo_state("DISARMING", "touchdown detected")
            else:
                self.touchdown_started_s = None
            return None

        if self.demo_state == "DISARMING":
            if not self.latest_state.armed:
                if self.takeoff_param_changed and self.restore_takeoff_alt_on_exit:
                    self.set_demo_state("RESTORE_TAKEOFF_PARAM", "vehicle disarmed")
                elif self.should_restore_speed_profile():
                    self.set_demo_state("RESTORE_SPEED_PROFILE", "vehicle disarmed")
                else:
                    self.set_demo_state("COMPLETE", "vehicle disarmed")
                return None
            if not self.disarm_request_sent:
                self.disarm_request_sent = True
                self.request_arm(False)
            return None

        if self.demo_state == "RESTORE_TAKEOFF_PARAM":
            if not self.restore_takeoff_alt_on_exit:
                if self.should_restore_speed_profile():
                    self.set_demo_state(
                        "RESTORE_SPEED_PROFILE",
                        "skipping takeoff-altitude restore",
                    )
                else:
                    next_state = "ABORT" if self.abort_reason else "COMPLETE"
                    self.set_demo_state(next_state, "skipping takeoff-altitude restore")
                return None
            if not self.takeoff_param_changed or self.original_takeoff_alt_m is None:
                if self.should_restore_speed_profile():
                    self.set_demo_state(
                        "RESTORE_SPEED_PROFILE",
                        "takeoff altitude already restored",
                    )
                else:
                    next_state = "ABORT" if self.abort_reason else "COMPLETE"
                    self.set_demo_state(next_state, "takeoff altitude already restored")
                return None
            if self.restore_complete:
                if self.should_restore_speed_profile():
                    self.set_demo_state(
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
                    self.set_demo_state(next_state, reason)
                return None
            if (
                self.param_pull_future is None
                and not self.param_mirror_ready
                and self.now_s() >= self.next_param_pull_attempt_s
            ):
                self.request_param_pull()
                return None
            if self.param_pull_future is not None:
                return None
            if self.param_future is None:
                self.request_param_set(
                    self.original_takeoff_alt_m,
                    kind="RESTORE_ORIGINAL",
                )
            return None

        if self.demo_state == "RESTORE_SPEED_PROFILE":
            if not self.should_restore_speed_profile():
                next_state = "ABORT" if self.abort_reason else "COMPLETE"
                reason = "abort cleanup complete" if self.abort_reason else "demo complete"
                self.set_demo_state(next_state, reason)
                return None
            if (
                self.param_pull_future is None
                and not self.param_mirror_ready
                and self.now_s() >= self.next_param_pull_attempt_s
            ):
                self.request_param_pull()
                return None
            if self.param_pull_future is not None:
                return None
            if self.param_future is not None:
                return None
            param_name = self.next_speed_profile_param_to_restore()
            if param_name is None:
                next_state = "ABORT" if self.abort_reason else "COMPLETE"
                reason = "abort cleanup complete" if self.abort_reason else "demo complete"
                self.set_demo_state(next_state, reason)
                return None
            original_value = self.speed_profile_original_values.get(param_name)
            if original_value is None:
                self.speed_profile_changed_names.discard(param_name)
                return None
            self.request_param_set(
                original_value,
                kind="PROFILE_RESTORE_ORIGINAL",
                param_name=param_name,
            )
            return None

        if self.demo_state == "ABORT":
            if self.latest_state.connected and self.latest_state.armed and self.land_on_abort:
                if (
                    not self.is_land_mode()
                    and self.mode_future is None
                    and self.mode_request_retry_ready()
                ):
                    self.request_mode(self.land_mode)
                return None
            if self.takeoff_param_changed and self.restore_takeoff_alt_on_exit:
                if self.restore_complete:
                    return None
                if (
                    self.param_pull_future is None
                    and not self.param_mirror_ready
                    and self.now_s() >= self.next_param_pull_attempt_s
                ):
                    self.request_param_pull()
                    return None
                if self.param_pull_future is not None:
                    return None
                if self.param_future is None and self.original_takeoff_alt_m is not None:
                    self.request_param_set(
                        self.original_takeoff_alt_m,
                        kind="RESTORE_ORIGINAL",
                    )
                    return None
            if self.should_restore_speed_profile():
                if (
                    self.param_pull_future is None
                    and not self.param_mirror_ready
                    and self.now_s() >= self.next_param_pull_attempt_s
                ):
                    self.request_param_pull()
                    return None
                if self.param_pull_future is not None:
                    return None
                if self.param_future is not None:
                    return None
                param_name = self.next_speed_profile_param_to_restore()
                if param_name is None:
                    return None
                original_value = self.speed_profile_original_values.get(param_name)
                if original_value is None:
                    self.speed_profile_changed_names.discard(param_name)
                    return None
                self.request_param_set(
                    original_value,
                    kind="PROFILE_RESTORE_ORIGINAL",
                    param_name=param_name,
                )
            return None

        return None

    def timer_callback(self) -> None:
        self.poll_service_futures()
        self.run_safety_checks()
        target = self.step_state_machine()
        if target is None and self.active_track_id is not None:
            target = self.tracks.get(self.active_track_id)
        self.publish_engagement_state(now_s=self.now_s(), target=target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Milestone3DemoSequenceNode()
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
