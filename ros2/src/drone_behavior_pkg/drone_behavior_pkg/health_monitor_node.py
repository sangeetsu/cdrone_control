from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from drone_behavior_pkg.topic_utils import cdrone_topic, join_topic
from drone_msgs.msg import EngagementState, TargetTrackArray


class HealthMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("health_monitor_node")

        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("check_rate_hz", 5.0)
        self.declare_parameter("tracks_timeout_s", 1.0)
        self.declare_parameter("engagement_timeout_s", 2.0)
        self.declare_parameter("vio_timeout_s", 0.5)
        self.declare_parameter("startup_grace_s", 5.0)
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("engagement_state_topic", "")
        self.declare_parameter("estop_topic", "")
        self.declare_parameter("autonomy_enable_topic", "")

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.check_rate_hz = float(self.get_parameter("check_rate_hz").value)
        self.tracks_timeout_s = float(self.get_parameter("tracks_timeout_s").value)
        self.engagement_timeout_s = float(self.get_parameter("engagement_timeout_s").value)
        self.vio_timeout_s = float(self.get_parameter("vio_timeout_s").value)
        self.startup_grace_s = float(self.get_parameter("startup_grace_s").value)
        self.tracks_topic = (
            str(self.get_parameter("tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/tracks")
        )
        self.engagement_state_topic = (
            str(self.get_parameter("engagement_state_topic").value).strip()
            or cdrone_topic(self.drone_id, "engagement/state")
        )
        self.estop_topic = (
            str(self.get_parameter("estop_topic").value).strip()
            or cdrone_topic(self.drone_id, "safety/estop")
        )
        self.autonomy_enable_topic = (
            str(self.get_parameter("autonomy_enable_topic").value).strip()
            or cdrone_topic(self.drone_id, "control/autonomy_enable")
        )
        self.vio_pose_topic = join_topic(self.mavros_namespace, "vision_pose/pose")

        self.last_tracks_time_s: Optional[float] = None
        self.last_engagement_time_s: Optional[float] = None
        self.last_vio_time_s: Optional[float] = None
        self.last_estop_state = False
        self.start_time_s = self.now_s()
        self.autonomy_enabled = False

        self.tracks_sub = self.create_subscription(
            TargetTrackArray, self.tracks_topic, self.tracks_callback, 10
        )
        self.engagement_sub = self.create_subscription(
            EngagementState, self.engagement_state_topic, self.engagement_callback, 10
        )
        self.vio_sub = self.create_subscription(
            PoseStamped, self.vio_pose_topic, self.vio_callback, 10
        )
        self.autonomy_enable_sub = self.create_subscription(
            Bool, self.autonomy_enable_topic, self.autonomy_enable_callback, 10
        )
        self.estop_pub = self.create_publisher(Bool, self.estop_topic, 10)

        self.timer = self.create_timer(
            1.0 / max(self.check_rate_hz, 1.0), self.check_health
        )
        self.get_logger().info("Health monitor started.")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def tracks_callback(self, _msg: TargetTrackArray) -> None:
        self.last_tracks_time_s = self.now_s()

    def engagement_callback(self, _msg: EngagementState) -> None:
        self.last_engagement_time_s = self.now_s()

    def vio_callback(self, _msg: PoseStamped) -> None:
        self.last_vio_time_s = self.now_s()

    def autonomy_enable_callback(self, msg: Bool) -> None:
        self.autonomy_enabled = bool(msg.data)

    def check_health(self) -> None:
        now_s = self.now_s()
        if now_s - self.start_time_s < self.startup_grace_s:
            return
        if not self.autonomy_enabled:
            if self.last_estop_state:
                self.last_estop_state = False
                msg = Bool()
                msg.data = False
                self.estop_pub.publish(msg)
            return
        track_stale = (
            self.last_tracks_time_s is None
            or now_s - self.last_tracks_time_s > self.tracks_timeout_s
        )
        engagement_stale = (
            self.last_engagement_time_s is None
            or now_s - self.last_engagement_time_s > self.engagement_timeout_s
        )
        vio_stale = (
            self.last_vio_time_s is None or now_s - self.last_vio_time_s > self.vio_timeout_s
        )
        estop = bool(track_stale or engagement_stale or vio_stale)
        if estop == self.last_estop_state:
            return

        self.last_estop_state = estop
        msg = Bool()
        msg.data = estop
        self.estop_pub.publish(msg)

        if estop:
            self.get_logger().error(
                "Health timeout detected, asserting /cdrone/safety/estop."
            )
        else:
            self.get_logger().warn("Health recovered, clearing /cdrone/safety/estop.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
