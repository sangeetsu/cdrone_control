from __future__ import annotations

import math
import os
from typing import Dict, Optional

import rclpy
from drone_msgs.msg import EngagementState, TargetTrackArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import CompanionProcessStatus, State
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
    TrackSnapshot,
    compute_follow_command,
    distance_in_standoff_window,
    target_bearing_rad,
)
from drone_control_pkg.milestone2_demo_logic import (
    project_body_velocity_to_world_xy,
    select_sequential_target,
    update_dwell_progress,
)
from drone_control_pkg.perimeter_utils import PerimeterGuard
from drone_control_pkg.topic_utils import cdrone_topic, join_topic


class Milestone2DemoSequenceNode(Node):
    ACTIVE_STATES = {"SEARCH", "FOLLOW", "DWELL"}

    def __init__(self) -> None:
        super().__init__("milestone2_demo_sequence_node")

        self.declare_parameter("scenario_id", "milestone2_demo_v1")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("required_completion_count", 3)
        self.declare_parameter("dwell_time_s", 3.0)
        self.declare_parameter("follow_distance_m", 1.5)
        self.declare_parameter("follow_distance_tolerance_m", 0.2)
        self.declare_parameter("lateral_deadband_m", 0.15)
        self.declare_parameter("vertical_deadband_m", 0.15)
        self.declare_parameter("yaw_deadband_rad", 0.08)
        self.declare_parameter("track_timeout_s", 0.5)
        self.declare_parameter("track_gc_s", 2.5)
        self.declare_parameter("state_timeout_s", 1.0)
        self.declare_parameter("local_pose_timeout_s", 0.5)
        self.declare_parameter("companion_status_timeout_s", 0.5)
        self.declare_parameter("min_track_confidence", 0.35)
        self.declare_parameter("min_target_distance_m", 0.0)
        self.declare_parameter("max_target_distance_m", 2.6)
        self.declare_parameter("require_target_in_front", True)
        self.declare_parameter("max_abs_target_y_m", 4.0)
        self.declare_parameter("max_abs_target_z_m", 2.5)
        self.declare_parameter("min_safe_distance_m", 1.0)
        self.declare_parameter("require_mavros_connected", True)
        self.declare_parameter("require_armed", True)
        self.declare_parameter("require_offboard", True)
        self.declare_parameter("require_companion_active", True)
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
        self.declare_parameter("companion_status_topic", "")
        self.declare_parameter("engagement_state_topic", "")
        self.declare_parameter("start_service", "")
        self.declare_parameter("abort_service", "")

        self.scenario_id = str(self.get_parameter("scenario_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.required_completion_count = int(
            self.get_parameter("required_completion_count").value
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
        self.state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self.local_pose_timeout_s = float(
            self.get_parameter("local_pose_timeout_s").value
        )
        self.companion_status_timeout_s = float(
            self.get_parameter("companion_status_timeout_s").value
        )
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
        self.min_safe_distance_m = float(
            self.get_parameter("min_safe_distance_m").value
        )
        self.require_mavros_connected = bool(
            self.get_parameter("require_mavros_connected").value
        )
        self.require_armed = bool(self.get_parameter("require_armed").value)
        self.require_offboard = bool(self.get_parameter("require_offboard").value)
        self.require_companion_active = bool(
            self.get_parameter("require_companion_active").value
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
        self.perimeter_config = str(self.get_parameter("perimeter_config").value)
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
            or cdrone_topic(self.drone_id, "demo/milestone2_start")
        )
        self.abort_service_name = (
            str(self.get_parameter("abort_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/milestone2_abort")
        )

        self.latest_state = State()
        self.latest_pose = PoseStamped()
        self.last_state_time_s = 0.0
        self.last_pose_time_s = 0.0
        self.last_companion_time_s = 0.0
        self.companion_active = False
        self.estop_latched = False
        self.tracks: Dict[int, TrackSnapshot] = {}
        self.active_track_id: Optional[int] = None
        self.completed_track_ids: set[int] = set()
        self.blocked_track_ids: set[int] = set()
        self.dwell_started_s: Optional[float] = None
        self.dwell_elapsed_s = 0.0
        self.dwell_remaining_s = self.dwell_time_s
        self.min_distance_gate_active = False
        self.last_blocked_reason = ""
        self.abort_reason = ""
        self.demo_state = "IDLE"

        self.perimeter_guard: Optional[PerimeterGuard] = None
        self.perimeter_guard_error = ""
        self.load_perimeter_guard()

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
            self.local_pose_callback,
            best_effort_qos,
        )
        self.create_subscription(
            CompanionProcessStatus,
            self.companion_status_topic,
            self.companion_status_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(
            EngagementState,
            self.engagement_state_topic,
            10,
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

        self.get_logger().info(
            "Milestone 2 demo sequence node started. "
            f"tracks={self.tracks_topic} cmd_vel={self.cmd_vel_topic} "
            f"state_topic={self.engagement_state_topic}"
        )
        self.publish_engagement_state(now_s=self.now_s(), target=None)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def load_perimeter_guard(self) -> None:
        self.perimeter_guard = None
        self.perimeter_guard_error = ""
        if not self.enable_perimeter_guard:
            self.get_logger().info("Perimeter guard disabled for milestone 2 demo.")
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

    def state_callback(self, msg: State) -> None:
        self.latest_state = msg
        self.last_state_time_s = self.now_s()

    def local_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.last_pose_time_s = self.now_s()

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

    def current_frame_id(self) -> str:
        return str(self.latest_pose.header.frame_id or "map").strip()

    def current_xy(self) -> tuple[float, float]:
        return (
            float(self.latest_pose.pose.position.x),
            float(self.latest_pose.pose.position.y),
        )

    def current_altitude_m(self) -> float:
        return float(self.latest_pose.pose.position.z)

    def current_yaw_rad(self) -> float:
        orientation = self.latest_pose.pose.orientation
        siny_cosp = 2.0 * (
            (orientation.w * orientation.z) + (orientation.x * orientation.y)
        )
        cosy_cosp = 1.0 - 2.0 * (
            (orientation.y * orientation.y) + (orientation.z * orientation.z)
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def state_fresh(self) -> bool:
        return (self.now_s() - self.last_state_time_s) <= self.state_timeout_s

    def pose_fresh(self) -> bool:
        return (self.now_s() - self.last_pose_time_s) <= self.local_pose_timeout_s

    def companion_status_fresh(self) -> bool:
        return (
            self.now_s() - self.last_companion_time_s
        ) <= self.companion_status_timeout_s

    def zero_cmd(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def publish_zero_command(self) -> None:
        self.cmd_pub.publish(self.zero_cmd())

    def publish_follow_command(self, command: FollowCommand) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = command.vx
        msg.twist.linear.y = command.vy
        msg.twist.linear.z = command.vz
        msg.twist.angular.z = command.yaw_rate
        self.cmd_pub.publish(msg)

    def clear_active_target(self) -> None:
        self.active_track_id = None
        self.dwell_started_s = None
        self.dwell_elapsed_s = 0.0
        self.dwell_remaining_s = self.dwell_time_s

    def set_demo_state(self, new_state: str, reason: str = "") -> None:
        if new_state == self.demo_state and not reason:
            return
        old_state = self.demo_state
        self.demo_state = new_state
        if reason:
            self.get_logger().info(f"State {old_state} -> {new_state}: {reason}")
        elif old_state != new_state:
            self.get_logger().info(f"State {old_state} -> {new_state}")

    def control_block_reason(self) -> str:
        if self.estop_latched:
            return "ESTOP"
        if not self.state_fresh():
            return "STALE_STATE"
        if self.require_mavros_connected and not self.latest_state.connected:
            return "MAVROS_DISCONNECTED"
        if self.require_armed and not self.latest_state.armed:
            return "NOT_ARMED"
        if self.require_offboard and str(self.latest_state.mode).upper() != "OFFBOARD":
            return f"MODE_{str(self.latest_state.mode or 'UNKNOWN').upper()}"
        if self.local_pose_timeout_s > 0.0 and not self.pose_fresh():
            return "STALE_LOCAL_POSE"
        if self.require_companion_active:
            if not self.companion_status_fresh():
                return "STALE_COMPANION_STATUS"
            if not self.companion_active:
                return "COMPANION_INACTIVE"
        return ""

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
        )
        if chosen is None:
            if self.active_track_id is not None:
                self.get_logger().info(
                    f"Clearing target lock: id={self.active_track_id}"
                )
            self.active_track_id = None
            return None

        if self.active_track_id != chosen.track_id:
            self.active_track_id = chosen.track_id
            self.dwell_started_s = None
            self.dwell_elapsed_s = 0.0
            self.dwell_remaining_s = self.dwell_time_s
            self.get_logger().info(
                "Target lock: "
                f"id={chosen.track_id} dist={chosen.distance_m:.2f} "
                f"x={chosen.x_b_m:.2f} y={chosen.y_b_m:.2f} z={chosen.z_b_m:.2f}"
            )
        return chosen

    def startable(self) -> tuple[bool, str]:
        if self.demo_state in self.ACTIVE_STATES:
            return False, f"demo already active in state {self.demo_state}"
        if self.enable_perimeter_guard and self.perimeter_guard is None:
            return False, self.perimeter_guard_error or "perimeter guard is unavailable"
        block_reason = self.control_block_reason()
        if block_reason:
            return False, block_reason
        runtime_reason = self.perimeter_runtime_violation_reason()
        if runtime_reason is not None:
            return False, runtime_reason
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

        self.active_track_id = None
        self.completed_track_ids.clear()
        self.blocked_track_ids.clear()
        self.dwell_started_s = None
        self.dwell_elapsed_s = 0.0
        self.dwell_remaining_s = self.dwell_time_s
        self.min_distance_gate_active = False
        self.last_blocked_reason = ""
        self.abort_reason = ""
        self.set_demo_state("SEARCH", "start requested")
        response.success = True
        response.message = "milestone2 demo sequence started"
        return response

    def handle_abort_request(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if self.demo_state not in self.ACTIVE_STATES:
            response.success = False
            response.message = "milestone2 demo is not active"
            return response

        self.enter_abort("operator abort")
        response.success = True
        response.message = "abort accepted"
        return response

    def enter_abort(self, reason: str) -> None:
        self.abort_reason = reason
        self.last_blocked_reason = ""
        self.clear_active_target()
        self.publish_zero_command()
        self.set_demo_state("ABORT", reason)

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
        msg.autonomy_enabled = self.demo_state in self.ACTIVE_STATES
        msg.autonomy_ready = bool(
            self.demo_state in self.ACTIVE_STATES and not self.last_blocked_reason
        )
        msg.blocked_reason = self.abort_reason or self.last_blocked_reason
        msg.estop_latched = bool(self.estop_latched)
        msg.obstacle_blocked = False
        msg.min_distance_gate_active = bool(self.min_distance_gate_active)
        msg.active_track_id = int(self.active_track_id) if self.active_track_id is not None else -1
        msg.active_distance_m = float(target.distance_m) if target is not None else -1.0
        msg.active_bearing_rad = (
            float(target_bearing_rad(target)) if target is not None else 0.0
        )
        msg.target_visible = bool(target is not None)
        msg.dwell_remaining_s = float(self.dwell_remaining_s)
        msg.dwell_elapsed_s = float(self.dwell_elapsed_s)
        msg.completed_targets_count = int(len(self.completed_track_ids))
        msg.required_targets_count = int(self.required_completion_count)
        self.state_pub.publish(msg)

    def timer_callback(self) -> None:
        now_s = self.now_s()
        target: Optional[TrackSnapshot] = None

        if self.demo_state == "IDLE":
            self.publish_engagement_state(now_s=now_s, target=None)
            return

        if self.demo_state in {"ABORT", "COMPLETE"}:
            self.publish_zero_command()
            self.publish_engagement_state(now_s=now_s, target=None)
            return

        runtime_reason = self.perimeter_runtime_violation_reason()
        if runtime_reason is not None:
            self.enter_abort(f"perimeter violation: {runtime_reason}")
            self.publish_engagement_state(now_s=now_s, target=None)
            return

        self.last_blocked_reason = self.control_block_reason()
        if self.last_blocked_reason:
            self.min_distance_gate_active = False
            self.dwell_started_s = None
            self.dwell_elapsed_s = 0.0
            self.dwell_remaining_s = self.dwell_time_s
            if self.publish_zero_on_block:
                self.publish_zero_command()
            self.set_demo_state("SEARCH")
            self.publish_engagement_state(now_s=now_s, target=None)
            return

        target = self.select_target(now_s)
        if target is None:
            self.min_distance_gate_active = False
            self.dwell_started_s = None
            self.dwell_elapsed_s = 0.0
            self.dwell_remaining_s = self.dwell_time_s
            self.publish_zero_command()
            self.set_demo_state("SEARCH")
            self.publish_engagement_state(now_s=now_s, target=None)
            return

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

        projection_reason = self.command_projection_violation_reason(command)
        if projection_reason is not None:
            active_track_id = self.active_track_id
            if (
                active_track_id is not None
                and self.block_target_on_perimeter_violation
            ):
                self.blocked_track_ids.add(active_track_id)
                self.get_logger().warn(
                    "Blocking target due to predicted perimeter conflict: "
                    f"id={active_track_id} reason={projection_reason}"
                )
            self.clear_active_target()
            self.last_blocked_reason = f"TARGET_PATH_BLOCKED: {projection_reason}"
            self.min_distance_gate_active = False
            self.publish_zero_command()
            self.set_demo_state("SEARCH")
            self.publish_engagement_state(now_s=now_s, target=None)
            return

        self.min_distance_gate_active = bool(command.min_distance_gate_active)
        self.publish_follow_command(command)

        in_standoff_window = distance_in_standoff_window(
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
            in_standoff_window=in_standoff_window,
            now_s=now_s,
            dwell_time_s=self.dwell_time_s,
        )

        if in_standoff_window:
            self.set_demo_state("DWELL")
        else:
            self.set_demo_state("FOLLOW")

        if dwell_complete:
            completed_track_id = target.track_id
            self.completed_track_ids.add(completed_track_id)
            self.get_logger().info(
                "Target complete: "
                f"id={completed_track_id} dwell={self.dwell_time_s:.1f}s "
                f"distance={target.distance_m:.2f}"
            )
            self.clear_active_target()
            self.min_distance_gate_active = False
            self.publish_zero_command()
            if len(self.completed_track_ids) >= self.required_completion_count:
                self.set_demo_state(
                    "COMPLETE",
                    f"completed {len(self.completed_track_ids)} targets",
                )
            else:
                self.set_demo_state("SEARCH", f"completed track {completed_track_id}")
            self.publish_engagement_state(now_s=now_s, target=None)
            return

        self.publish_engagement_state(now_s=now_s, target=target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Milestone2DemoSequenceNode()
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
