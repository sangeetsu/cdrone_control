from __future__ import annotations

from typing import Dict, Optional

import rclpy
from drone_msgs.msg import TargetTrackArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import CompanionProcessStatus, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from drone_control_pkg.follow_utils import (
    TrackSnapshot,
    compute_follow_command,
    score_track,
    track_is_valid,
)
from drone_control_pkg.topic_utils import cdrone_topic, join_topic

class TargetFollowControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("target_follow_controller_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("follow_enabled_on_startup", False)
        self.declare_parameter("follow_distance_m", 3.0)
        self.declare_parameter("follow_distance_tolerance_m", 0.25)
        self.declare_parameter("lateral_deadband_m", 0.15)
        self.declare_parameter("vertical_deadband_m", 0.15)
        self.declare_parameter("yaw_deadband_rad", 0.08)
        self.declare_parameter("track_timeout_s", 0.5)
        self.declare_parameter("track_gc_s", 2.5)
        self.declare_parameter("state_timeout_s", 1.0)
        self.declare_parameter("local_pose_timeout_s", 0.5)
        self.declare_parameter("companion_status_timeout_s", 0.5)
        self.declare_parameter("min_track_confidence", 0.35)
        self.declare_parameter("max_target_distance_m", 8.0)
        self.declare_parameter("require_target_in_front", True)
        self.declare_parameter("max_abs_target_y_m", 4.0)
        self.declare_parameter("max_abs_target_z_m", 2.5)
        self.declare_parameter("min_safe_distance_m", 1.0)
        self.declare_parameter("require_mavros_connected", True)
        self.declare_parameter("require_armed", True)
        self.declare_parameter("require_offboard", True)
        self.declare_parameter("require_companion_active", True)
        self.declare_parameter("publish_zero_on_block", True)
        self.declare_parameter("publish_zero_on_lost_target", True)
        self.declare_parameter("kp_xy", 0.45)
        self.declare_parameter("kp_z", 0.30)
        self.declare_parameter("kp_yaw", 0.80)
        self.declare_parameter("max_vel_xy_mps", 1.0)
        self.declare_parameter("max_vel_z_mps", 0.5)
        self.declare_parameter("max_yaw_rate_rps", 0.6)
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("estop_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("local_pose_topic", "")
        self.declare_parameter("companion_status_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("enable_service", "")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.follow_enabled = bool(
            self.get_parameter("follow_enabled_on_startup").value
        )
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
        self.publish_zero_on_lost_target = bool(
            self.get_parameter("publish_zero_on_lost_target").value
        )
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_vel_xy_mps = float(self.get_parameter("max_vel_xy_mps").value)
        self.max_vel_z_mps = float(self.get_parameter("max_vel_z_mps").value)
        self.max_yaw_rate_rps = float(self.get_parameter("max_yaw_rate_rps").value)

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
        self.status_topic = (
            str(self.get_parameter("status_topic").value).strip()
            or cdrone_topic(self.drone_id, "control/target_follow_status")
        )
        self.enable_service_name = (
            str(self.get_parameter("enable_service").value).strip()
            or cdrone_topic(self.drone_id, "control/target_follow_enable")
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
        self.last_status = ""

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
            TargetTrackArray,
            self.tracks_topic,
            self.tracks_callback,
            10,
        )
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
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_service(
            SetBool,
            self.enable_service_name,
            self.handle_enable_request,
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self.control_loop,
        )

        self.get_logger().info(
            "Target follow controller started. "
            f"tracks={self.tracks_topic} cmd_vel={self.cmd_vel_topic} "
            f"enabled={self.follow_enabled}"
        )
        self.publish_status("FOLLOW_DISABLED" if not self.follow_enabled else "READY")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

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

    def handle_enable_request(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        self.follow_enabled = bool(request.data)
        if not self.follow_enabled and self.publish_zero_on_block:
            self.cmd_pub.publish(self.zero_cmd())
        state = "enabled" if self.follow_enabled else "disabled"
        self.get_logger().info(f"Target follow {state} by service request.")
        response.success = True
        response.message = f"target follow {state}"
        return response

    def state_fresh(self) -> bool:
        return (self.now_s() - self.last_state_time_s) <= self.state_timeout_s

    def pose_fresh(self) -> bool:
        return (self.now_s() - self.last_pose_time_s) <= self.local_pose_timeout_s

    def companion_status_fresh(self) -> bool:
        return (
            self.now_s() - self.last_companion_time_s
        ) <= self.companion_status_timeout_s

    def track_is_valid(self, track: TrackSnapshot, now_s: float) -> bool:
        if now_s - track.last_seen_s > self.track_timeout_s:
            return False
        return track_is_valid(
            track,
            min_track_confidence=self.min_track_confidence,
            max_target_distance_m=self.max_target_distance_m,
            require_target_in_front=self.require_target_in_front,
            max_abs_target_y_m=self.max_abs_target_y_m,
            max_abs_target_z_m=self.max_abs_target_z_m,
        )

    def select_target(self, now_s: float) -> Optional[TrackSnapshot]:
        if self.active_track_id is not None:
            current = self.tracks.get(self.active_track_id)
            if current is not None and self.track_is_valid(current, now_s):
                return current

        candidates = [
            track
            for track in self.tracks.values()
            if self.track_is_valid(track, now_s)
        ]
        if not candidates:
            self.active_track_id = None
            return None

        candidates.sort(
            key=lambda track: (
                -score_track(track),
                track.distance_m,
                -track.confidence,
            )
        )
        chosen = candidates[0]
        self.active_track_id = chosen.track_id
        return chosen

    def control_block_reason(self, now_s: float) -> str:
        del now_s
        if not self.follow_enabled:
            return "FOLLOW_DISABLED"
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

    def zero_cmd(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        if status != self.last_status:
            self.get_logger().info(f"Target follow status: {status}")
            self.last_status = status

    def publish_follow_command(self, track: TrackSnapshot) -> None:
        command = compute_follow_command(
            track,
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

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = command.vx
        cmd.twist.linear.y = command.vy
        cmd.twist.linear.z = command.vz
        cmd.twist.angular.z = command.yaw_rate
        self.cmd_pub.publish(cmd)

        self.publish_status(
            "FOLLOWING "
            f"id={track.track_id} dist={track.distance_m:.2f} "
            f"x={track.x_b_m:.2f} y={track.y_b_m:.2f} z={track.z_b_m:.2f}"
        )

    def control_loop(self) -> None:
        now_s = self.now_s()
        block_reason = self.control_block_reason(now_s)
        if block_reason:
            if self.publish_zero_on_block:
                self.cmd_pub.publish(self.zero_cmd())
            self.publish_status(block_reason)
            return

        target = self.select_target(now_s)
        if target is None:
            if self.publish_zero_on_lost_target:
                self.cmd_pub.publish(self.zero_cmd())
            self.publish_status("NO_VALID_TARGET")
            return

        self.publish_follow_command(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetFollowControllerNode()
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
