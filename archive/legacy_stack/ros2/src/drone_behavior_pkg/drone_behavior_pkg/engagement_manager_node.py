from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_behavior_pkg.behavior_registry import build_behavior_registry
from drone_behavior_pkg.math_utils import (
    TrackSnapshot,
    clamp,
    compute_velocity_command_details,
    score_track,
    target_bearing_rad,
)
from drone_behavior_pkg.topic_utils import cdrone_topic, join_topic
from drone_msgs.msg import EngagementState, LightCommand, TargetTrack, TargetTrackArray

class EngagementManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("engagement_manager_node")

        default_scenario = (
            Path(get_package_share_directory("drone_behavior_pkg"))
            / "config"
            / "scenarios"
            / "track_follow_v1.yaml"
        )

        self.declare_parameter("scenario_file", str(default_scenario))
        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("follow_distance_m", 3.0)
        self.declare_parameter("follow_distance_tolerance_m", 0.4)
        self.declare_parameter("align_yaw_tolerance_rad", 0.12)
        self.declare_parameter("target_cooldown_s", 15.0)
        self.declare_parameter("min_safe_distance_m", 0.8)
        self.declare_parameter("lost_target_timeout_s", 0.75)
        self.declare_parameter("reacquire_timeout_s", 2.0)
        self.declare_parameter("vio_timeout_s", 0.5)
        self.declare_parameter("track_gc_s", 2.5)
        self.declare_parameter("kp_xy", 0.45)
        self.declare_parameter("kp_z", 0.3)
        self.declare_parameter("kp_yaw", 0.8)
        self.declare_parameter("max_vel_xy_mps", 1.5)
        self.declare_parameter("max_vel_z_mps", 0.8)
        self.declare_parameter("max_yaw_rate_rps", 0.6)
        self.declare_parameter("obstacle_forward_distance_m", 1.5)
        self.declare_parameter("obstacle_lateral_band_m", 0.8)
        self.declare_parameter("obstacle_vertical_band_m", 0.8)
        self.declare_parameter("light_default_intensity", 0.9)
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("engagement_state_topic", "")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("light_cmd_topic", "")
        self.declare_parameter("estop_topic", "")
        self.declare_parameter("estop_reset_topic", "")
        self.declare_parameter("autonomy_enable_topic", "")

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.follow_distance_m = float(self.get_parameter("follow_distance_m").value)
        self.follow_distance_tolerance_m = float(
            self.get_parameter("follow_distance_tolerance_m").value
        )
        self.align_yaw_tolerance_rad = float(
            self.get_parameter("align_yaw_tolerance_rad").value
        )
        self.target_cooldown_s = float(self.get_parameter("target_cooldown_s").value)
        self.min_safe_distance_m = float(self.get_parameter("min_safe_distance_m").value)
        self.lost_target_timeout_s = float(self.get_parameter("lost_target_timeout_s").value)
        self.reacquire_timeout_s = float(self.get_parameter("reacquire_timeout_s").value)
        self.vio_timeout_s = float(self.get_parameter("vio_timeout_s").value)
        self.track_gc_s = float(self.get_parameter("track_gc_s").value)
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_vel_xy_mps = float(self.get_parameter("max_vel_xy_mps").value)
        self.max_vel_z_mps = float(self.get_parameter("max_vel_z_mps").value)
        self.max_yaw_rate_rps = float(self.get_parameter("max_yaw_rate_rps").value)
        self.obstacle_forward_distance_m = float(
            self.get_parameter("obstacle_forward_distance_m").value
        )
        self.obstacle_lateral_band_m = float(
            self.get_parameter("obstacle_lateral_band_m").value
        )
        self.obstacle_vertical_band_m = float(
            self.get_parameter("obstacle_vertical_band_m").value
        )
        self.light_default_intensity = float(
            self.get_parameter("light_default_intensity").value
        )
        self.tracks_topic = (
            str(self.get_parameter("tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/tracks")
        )
        self.engagement_state_topic = (
            str(self.get_parameter("engagement_state_topic").value).strip()
            or cdrone_topic(self.drone_id, "engagement/state")
        )
        self.cmd_vel_topic = (
            str(self.get_parameter("cmd_vel_topic").value).strip()
            or cdrone_topic(self.drone_id, "control/cmd_vel_body")
        )
        self.light_cmd_topic = (
            str(self.get_parameter("light_cmd_topic").value).strip()
            or cdrone_topic(self.drone_id, "light/cmd")
        )
        self.estop_topic = (
            str(self.get_parameter("estop_topic").value).strip()
            or cdrone_topic(self.drone_id, "safety/estop")
        )
        self.estop_reset_topic = (
            str(self.get_parameter("estop_reset_topic").value).strip()
            or cdrone_topic(self.drone_id, "safety/estop_reset")
        )
        self.autonomy_enable_topic = (
            str(self.get_parameter("autonomy_enable_topic").value).strip()
            or cdrone_topic(self.drone_id, "control/autonomy_enable")
        )
        self.local_pose_topic = join_topic(self.mavros_namespace, "local_position/pose")
        self.vio_pose_topic = join_topic(self.mavros_namespace, "vision_pose/pose")
        self.mavros_state_topic = join_topic(self.mavros_namespace, "state")

        scenario_file = Path(str(self.get_parameter("scenario_file").value))
        self.scenario = self._load_scenario(scenario_file)
        self.scenario_id = str(self.scenario.get("scenario_id", "track_follow_v1"))
        states = list(self.scenario.get("states", []))
        self.state = states[0] if states else "SEARCH"

        self.behaviors = build_behavior_registry()
        if self.state not in self.behaviors:
            self.state = "SEARCH"

        self.tracks: Dict[int, TrackSnapshot] = {}
        self.cooldown_until: Dict[int, float] = {}
        self.active_track_id: Optional[int] = None
        self.active_target_lost_start_s: Optional[float] = None
        self.failsafe_enter_time_s: Optional[float] = None

        self.autonomy_enabled = False
        self.estop_signal = False
        self.estop_latched = False
        self.last_local_pose = PoseStamped()
        self.last_vio_pose_time_s: Optional[float] = None
        self.mavros_state = State()
        self.last_blocked_reason = "autonomy_disabled"
        self.min_distance_gate_active = False
        self.obstacle_blocked = False

        self.tracks_sub = self.create_subscription(
            TargetTrackArray, self.tracks_topic, self.tracks_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, self.estop_topic, self.estop_callback, 10
        )
        self.estop_reset_sub = self.create_subscription(
            Bool, self.estop_reset_topic, self.estop_reset_callback, 10
        )
        self.autonomy_enable_sub = self.create_subscription(
            Bool, self.autonomy_enable_topic, self.autonomy_enable_callback, 10
        )
        self.local_pose_sub = self.create_subscription(
            PoseStamped, self.local_pose_topic, self.local_pose_callback, 10
        )
        self.vio_pose_sub = self.create_subscription(
            PoseStamped, self.vio_pose_topic, self.vio_pose_callback, 10
        )
        self.mavros_state_sub = self.create_subscription(
            State, self.mavros_state_topic, self.mavros_state_callback, 10
        )

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.light_pub = self.create_publisher(LightCommand, self.light_cmd_topic, 10)
        self.state_pub = self.create_publisher(EngagementState, self.engagement_state_topic, 10)

        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0), self.control_loop
        )

        self.get_logger().info(
            f"Engagement manager started. scenario={self.scenario_id}, state={self.state}"
        )

    def _load_scenario(self, path: Path) -> dict:
        if not path.exists():
            self.get_logger().warning(
                f"Scenario file not found: {path}. Falling back to default state list."
            )
            return {
                "scenario_id": "default",
                "states": [
                    "SEARCH",
                    "ALIGN",
                    "APPROACH",
                    "FOLLOW_STANDOFF",
                    "LOST_TARGET_HOLD",
                    "FAILSAFE_HOLD",
                ],
            }
        with path.open("r", encoding="utf-8") as fp:
            return yaml.safe_load(fp) or {}

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def tracks_callback(self, msg: TargetTrackArray) -> None:
        now_s = self.now_s()
        seen_ids = set()
        for t in msg.tracks:
            snap = TrackSnapshot(
                track_id=int(t.track_id),
                x_b_m=float(t.x_b_m),
                y_b_m=float(t.y_b_m),
                z_b_m=float(t.z_b_m),
                vx_b_mps=float(t.vx_b_mps),
                vy_b_mps=float(t.vy_b_mps),
                vz_b_mps=float(t.vz_b_mps),
                distance_m=float(t.distance_m),
                confidence=float(t.confidence),
                bbox_area_px=float(t.bbox_area_px),
                inbound=bool(t.inbound),
                last_seen_s=now_s,
            )
            self.tracks[snap.track_id] = snap
            seen_ids.add(snap.track_id)

        for track_id in list(self.tracks.keys()):
            if track_id in seen_ids:
                continue
            if now_s - self.tracks[track_id].last_seen_s > self.track_gc_s:
                self.tracks.pop(track_id, None)

    def estop_callback(self, msg: Bool) -> None:
        self.estop_signal = bool(msg.data)
        if self.estop_signal:
            self.estop_latched = True

    def estop_reset_callback(self, msg: Bool) -> None:
        if bool(msg.data) and not self.estop_signal:
            self.estop_latched = False
            if self.state == "FAILSAFE_HOLD":
                self.state = "SEARCH"
                self.failsafe_enter_time_s = None
                self.get_logger().warn("E-stop reset received, returning to SEARCH.")

    def autonomy_enable_callback(self, msg: Bool) -> None:
        self.autonomy_enabled = bool(msg.data)

    def local_pose_callback(self, msg: PoseStamped) -> None:
        self.last_local_pose = msg

    def vio_pose_callback(self, _msg: PoseStamped) -> None:
        self.last_vio_pose_time_s = self.now_s()

    def mavros_state_callback(self, msg: State) -> None:
        self.mavros_state = msg

    def select_target(self, now_s: float) -> Optional[TrackSnapshot]:
        candidates = []
        for track in self.tracks.values():
            if now_s - track.last_seen_s > self.lost_target_timeout_s:
                continue
            cooldown_expire = self.cooldown_until.get(track.track_id, 0.0)
            if cooldown_expire > now_s:
                continue
            candidates.append(track)

        if not candidates:
            return None

        candidates.sort(key=lambda t: (-score_track(t), t.distance_m))
        return candidates[0]

    def set_active_target(self, track_id: int, now_s: float) -> None:
        self.active_track_id = int(track_id)
        self.active_target_lost_start_s = None
        self.failsafe_enter_time_s = None
        self.get_logger().info(f"Target lock: id={track_id}")

    def clear_active_target(self) -> None:
        self.active_track_id = None
        self.active_target_lost_start_s = None

    def mark_active_target_completed(self, now_s: float) -> None:
        if self.active_track_id is None:
            return
        self.cooldown_until[self.active_track_id] = now_s + self.target_cooldown_s
        self.get_logger().info(
            f"Target complete: id={self.active_track_id}, cooldown={self.target_cooldown_s:.1f}s"
        )

    def get_active_target(self, now_s: float) -> Optional[TrackSnapshot]:
        if self.active_track_id is None:
            return None
        target = self.tracks.get(self.active_track_id)
        if target is not None and now_s - target.last_seen_s <= self.lost_target_timeout_s:
            self.active_target_lost_start_s = None
            return target
        if self.active_target_lost_start_s is None:
            self.active_target_lost_start_s = now_s
        return None

    def active_target_lost_for(self, now_s: float) -> float:
        if self.active_target_lost_start_s is None:
            return 0.0
        return now_s - self.active_target_lost_start_s

    def publish_zero_velocity(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)

    def publish_light(self, enabled: bool, intensity: float, strobe_hz: float) -> None:
        msg = LightCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.enabled = bool(enabled)
        msg.intensity_0_to_1 = float(clamp(intensity, 0.0, 1.0))
        msg.strobe_hz = float(max(strobe_hz, 0.0))
        self.light_pub.publish(msg)

    def publish_velocity_for_target(
        self, target: TrackSnapshot, desired_distance_m: float
    ) -> None:
        command = compute_velocity_command_details(
            track=target,
            desired_distance_m=desired_distance_m,
            kp_xy=self.kp_xy,
            kp_z=self.kp_z,
            kp_yaw=self.kp_yaw,
            max_vel_xy_mps=self.max_vel_xy_mps,
            max_vel_z_mps=self.max_vel_z_mps,
            max_yaw_rate_rps=self.max_yaw_rate_rps,
            min_safe_distance_m=self.min_safe_distance_m,
        )
        self.min_distance_gate_active = command.min_distance_gate_active

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = command.vx
        msg.twist.linear.y = command.vy
        msg.twist.linear.z = command.vz
        msg.twist.angular.z = command.yaw_rate
        self.cmd_pub.publish(msg)

    def publish_align_for_target(self, target: TrackSnapshot) -> float:
        yaw_error = target_bearing_rad(target)
        yaw_rate = clamp(
            self.kp_yaw * yaw_error, -self.max_yaw_rate_rps, self.max_yaw_rate_rps
        )
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = yaw_rate
        self.min_distance_gate_active = False
        self.cmd_pub.publish(msg)
        return yaw_error

    def compute_obstacle_blocked(self, active_target: Optional[TrackSnapshot]) -> bool:
        for track in self.tracks.values():
            if self.active_track_id is not None and track.track_id == self.active_track_id:
                continue
            if self.now_s() - track.last_seen_s > self.lost_target_timeout_s:
                continue
            if track.x_b_m <= 0.0 or track.x_b_m > self.obstacle_forward_distance_m:
                continue
            if abs(track.y_b_m) > self.obstacle_lateral_band_m:
                continue
            if abs(track.z_b_m) > self.obstacle_vertical_band_m:
                continue
            return True
        return False

    def autonomy_block_reason(self, now_s: float) -> str:
        if self.estop_latched:
            return "estop"
        if not self.autonomy_enabled:
            return "autonomy_disabled"
        if self.last_vio_pose_time_s is None or now_s - self.last_vio_pose_time_s > self.vio_timeout_s:
            return "stale_vio"
        if not self.mavros_state.connected:
            return "mavros_disconnected"
        if not self.mavros_state.armed:
            return "not_armed"
        if self.mavros_state.mode != "OFFBOARD":
            return f"mode_{self.mavros_state.mode or 'unknown'}"
        return ""

    def _publish_engagement_state(self, now_s: float) -> None:
        msg = EngagementState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.scenario_id = self.scenario_id
        msg.state = self.state
        msg.autonomy_enabled = bool(self.autonomy_enabled)
        msg.autonomy_ready = bool(not self.last_blocked_reason)
        msg.blocked_reason = self.last_blocked_reason
        msg.estop_latched = bool(self.estop_latched)
        msg.obstacle_blocked = bool(self.obstacle_blocked)
        msg.min_distance_gate_active = bool(self.min_distance_gate_active)
        msg.active_track_id = int(self.active_track_id) if self.active_track_id is not None else -1
        target = self.get_active_target(now_s)
        msg.active_distance_m = float(target.distance_m) if target is not None else -1.0
        msg.active_bearing_rad = (
            float(target_bearing_rad(target)) if target is not None else 0.0
        )
        msg.target_visible = bool(target is not None)
        msg.dwell_remaining_s = 0.0
        self.state_pub.publish(msg)

    def control_loop(self) -> None:
        now_s = self.now_s()

        for track_id in list(self.cooldown_until.keys()):
            if self.cooldown_until[track_id] <= now_s:
                self.cooldown_until.pop(track_id, None)

        self.obstacle_blocked = self.compute_obstacle_blocked(self.get_active_target(now_s))
        self.min_distance_gate_active = False
        blocked_reason = self.autonomy_block_reason(now_s)
        self.last_blocked_reason = blocked_reason

        if self.estop_latched:
            next_state = "FAILSAFE_HOLD"
        elif blocked_reason:
            self.publish_zero_velocity()
            self.publish_light(False, 0.0, 0.0)
            next_state = "SEARCH"
        else:
            behavior = self.behaviors.get(self.state, self.behaviors["SEARCH"])
            next_state = behavior.step(self, now_s)
            if next_state not in self.behaviors:
                next_state = "SEARCH"

        if self.state != next_state:
            self.get_logger().info(f"State transition: {self.state} -> {next_state}")
            self.state = next_state

        if self.state == "FAILSAFE_HOLD":
            self.behaviors["FAILSAFE_HOLD"].step(self, now_s)

        self._publish_engagement_state(now_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EngagementManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
