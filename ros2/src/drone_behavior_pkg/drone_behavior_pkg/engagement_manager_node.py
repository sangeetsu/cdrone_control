from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_behavior_pkg.behavior_registry import build_behavior_registry
from drone_behavior_pkg.math_utils import (
    TrackSnapshot,
    clamp,
    compute_velocity_command,
    score_track,
)
from drone_msgs.msg import EngagementState, LightCommand, TargetTrack, TargetTrackArray

class EngagementManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("engagement_manager_node")

        default_scenario = (
            Path(get_package_share_directory("drone_behavior_pkg"))
            / "config"
            / "scenarios"
            / "intercept_illuminate_v1.yaml"
        )

        self.declare_parameter("scenario_file", str(default_scenario))
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("follow_distance_m", 3.0)
        self.declare_parameter("engage_distance_m", 1.2)
        self.declare_parameter("dwell_time_s", 2.0)
        self.declare_parameter("stability_time_s", 0.5)
        self.declare_parameter("target_cooldown_s", 15.0)
        self.declare_parameter("min_safe_distance_m", 0.8)
        self.declare_parameter("lost_target_timeout_s", 0.75)
        self.declare_parameter("reacquire_timeout_s", 2.0)
        self.declare_parameter("track_gc_s", 2.5)
        self.declare_parameter("kp_xy", 0.45)
        self.declare_parameter("kp_z", 0.3)
        self.declare_parameter("kp_yaw", 0.8)
        self.declare_parameter("max_vel_xy_mps", 1.5)
        self.declare_parameter("max_vel_z_mps", 0.8)
        self.declare_parameter("max_yaw_rate_rps", 0.6)
        self.declare_parameter("light_default_intensity", 0.9)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.follow_distance_m = float(self.get_parameter("follow_distance_m").value)
        self.engage_distance_m = float(self.get_parameter("engage_distance_m").value)
        self.dwell_time_s = float(self.get_parameter("dwell_time_s").value)
        self.stability_time_s = float(self.get_parameter("stability_time_s").value)
        self.target_cooldown_s = float(self.get_parameter("target_cooldown_s").value)
        self.min_safe_distance_m = float(self.get_parameter("min_safe_distance_m").value)
        self.lost_target_timeout_s = float(self.get_parameter("lost_target_timeout_s").value)
        self.reacquire_timeout_s = float(self.get_parameter("reacquire_timeout_s").value)
        self.track_gc_s = float(self.get_parameter("track_gc_s").value)
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_z = float(self.get_parameter("kp_z").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_vel_xy_mps = float(self.get_parameter("max_vel_xy_mps").value)
        self.max_vel_z_mps = float(self.get_parameter("max_vel_z_mps").value)
        self.max_yaw_rate_rps = float(self.get_parameter("max_yaw_rate_rps").value)
        self.light_default_intensity = float(
            self.get_parameter("light_default_intensity").value
        )

        scenario_file = Path(str(self.get_parameter("scenario_file").value))
        self.scenario = self._load_scenario(scenario_file)
        self.scenario_id = str(self.scenario.get("scenario_id", "intercept_illuminate_v1"))
        states = list(self.scenario.get("states", []))
        self.state = states[0] if states else "SEARCH"

        self.behaviors = build_behavior_registry()
        if self.state not in self.behaviors:
            self.state = "SEARCH"

        self.tracks: Dict[int, TrackSnapshot] = {}
        self.cooldown_until: Dict[int, float] = {}
        self.active_track_id: Optional[int] = None
        self.active_target_lost_start_s: Optional[float] = None
        self.approach_stable_start_s: Optional[float] = None
        self.illumination_start_time_s: Optional[float] = None
        self.failsafe_enter_time_s: Optional[float] = None

        self.estop_signal = False
        self.estop_latched = False
        self.last_local_pose = PoseStamped()

        self.tracks_sub = self.create_subscription(
            TargetTrackArray, "/cdrone/perception/tracks", self.tracks_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, "/cdrone/safety/estop", self.estop_callback, 10
        )
        self.estop_reset_sub = self.create_subscription(
            Bool, "/cdrone/safety/estop_reset", self.estop_reset_callback, 10
        )
        self.local_pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.local_pose_callback, 10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, "/cdrone/control/cmd_vel_body", 10
        )
        self.light_pub = self.create_publisher(LightCommand, "/cdrone/light/cmd", 10)
        self.state_pub = self.create_publisher(
            EngagementState, "/cdrone/engagement/state", 10
        )

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
            return {"scenario_id": "default", "states": ["SEARCH", "APPROACH", "ILLUMINATE", "ADVANCE_QUEUE", "FAILSAFE_HOLD"]}
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

    def local_pose_callback(self, msg: PoseStamped) -> None:
        self.last_local_pose = msg

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
        self.approach_stable_start_s = None
        self.illumination_start_time_s = None
        self.failsafe_enter_time_s = None
        self.get_logger().info(f"Target lock: id={track_id}")

    def clear_active_target(self) -> None:
        self.active_track_id = None
        self.active_target_lost_start_s = None
        self.approach_stable_start_s = None
        self.illumination_start_time_s = None

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
        vx, vy, vz, yaw_rate = compute_velocity_command(
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

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        self.cmd_pub.publish(msg)

    def _publish_engagement_state(self, now_s: float) -> None:
        msg = EngagementState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.scenario_id = self.scenario_id
        msg.state = self.state
        msg.active_track_id = int(self.active_track_id) if self.active_track_id is not None else -1
        target = self.get_active_target(now_s)
        msg.active_distance_m = float(target.distance_m) if target is not None else -1.0
        if self.state == "ILLUMINATE" and self.illumination_start_time_s is not None:
            elapsed = now_s - self.illumination_start_time_s
            msg.dwell_remaining_s = float(max(0.0, self.dwell_time_s - elapsed))
        else:
            msg.dwell_remaining_s = 0.0
        self.state_pub.publish(msg)

    def control_loop(self) -> None:
        now_s = self.now_s()

        for track_id in list(self.cooldown_until.keys()):
            if self.cooldown_until[track_id] <= now_s:
                self.cooldown_until.pop(track_id, None)

        if self.estop_latched:
            next_state = "FAILSAFE_HOLD"
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
