from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CompanionProcessStatus
from rclpy.node import Node
from std_msgs.msg import Bool


class Px4VisionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("px4_vision_bridge_node")

        self.declare_parameter("input_pose_topic", "/cdrone/vio/pose")
        self.declare_parameter("input_health_topic", "/cdrone/vio/healthy")
        self.declare_parameter("output_pose_topic", "/mavros/vision_pose/pose")
        self.declare_parameter(
            "companion_status_topic", "/mavros/companion_process/status"
        )
        self.declare_parameter("publish_companion_status", True)
        self.declare_parameter("freshness_timeout_s", 0.25)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("frame_id", "map")

        self.input_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self.input_health_topic = str(self.get_parameter("input_health_topic").value)
        self.output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        self.companion_status_topic = str(
            self.get_parameter("companion_status_topic").value
        )
        self.publish_companion_status = bool(
            self.get_parameter("publish_companion_status").value
        )
        self.freshness_timeout_s = float(
            self.get_parameter("freshness_timeout_s").value
        )
        self.status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.last_pose_msg: PoseStamped | None = None
        self.last_pose_time_s = 0.0
        self.latest_health = False

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self.pose_callback,
            10,
        )
        self.health_sub = self.create_subscription(
            Bool,
            self.input_health_topic,
            self.health_callback,
            10,
        )
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.status_pub = self.create_publisher(
            CompanionProcessStatus,
            self.companion_status_topic,
            10,
        )
        self.status_timer = self.create_timer(
            1.0 / max(self.status_rate_hz, 1.0), self.publish_status
        )

        self.get_logger().info(
            "PX4 vision bridge started. "
            f"{self.input_pose_topic} -> {self.output_pose_topic}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def health_callback(self, msg: Bool) -> None:
        self.latest_health = bool(msg.data)

    def pose_callback(self, msg: PoseStamped) -> None:
        bridged = deepcopy(msg)
        bridged.header.frame_id = self.frame_id
        self.last_pose_msg = bridged
        self.last_pose_time_s = self.now_s()
        if self.latest_health:
            self.pose_pub.publish(bridged)

    def _is_fresh(self) -> bool:
        if self.last_pose_msg is None or not self.latest_health:
            return False
        return (self.now_s() - self.last_pose_time_s) <= self.freshness_timeout_s

    def publish_status(self) -> None:
        if not self.publish_companion_status:
            return

        status_msg = CompanionProcessStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.component = (
            CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        )
        if self._is_fresh():
            status_msg.state = CompanionProcessStatus.MAV_STATE_ACTIVE
        else:
            status_msg.state = CompanionProcessStatus.MAV_STATE_CRITICAL
        self.status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4VisionBridgeNode()
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
