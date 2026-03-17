from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CompanionProcessStatus
from rclpy.node import Node

from drone_control_pkg.topic_utils import cdrone_topic, join_topic


class VioBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vio_bridge_node")

        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("publish_companion_status", True)
        self.declare_parameter("input_pose_topic", "")
        self.declare_parameter("output_pose_topic", "")

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.publish_companion_status = bool(
            self.get_parameter("publish_companion_status").value
        )
        self.input_pose_topic = (
            str(self.get_parameter("input_pose_topic").value).strip()
            or cdrone_topic(self.drone_id, "vio/input_pose")
        )
        self.output_pose_topic = (
            str(self.get_parameter("output_pose_topic").value).strip()
            or join_topic(self.mavros_namespace, "vision_pose/pose")
        )
        self.output_status_topic = join_topic(
            self.mavros_namespace, "companion_process/status"
        )

        self.latest_pose = PoseStamped()
        self.last_pose_s = 0.0

        self.create_subscription(PoseStamped, self.input_pose_topic, self.pose_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.status_pub = self.create_publisher(
            CompanionProcessStatus, self.output_status_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0), self.publish_loop
        )

        self.get_logger().info(
            f"VIO bridge started: {self.input_pose_topic} -> {self.output_pose_topic}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.last_pose_s = self.now_s()

    def publish_loop(self) -> None:
        now_s = self.now_s()
        if self.last_pose_s <= 0.0 or now_s - self.last_pose_s > self.input_timeout_s:
            if self.publish_companion_status:
                self._publish_status(active=False)
            return

        pose_msg = deepcopy(self.latest_pose)
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(pose_msg)
        if self.publish_companion_status:
            self._publish_status(active=True)

    def _publish_status(self, active: bool) -> None:
        status_msg = CompanionProcessStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.state = (
            CompanionProcessStatus.MAV_STATE_ACTIVE
            if active
            else CompanionProcessStatus.MAV_STATE_STANDBY
        )
        status_msg.component = (
            CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        )
        self.status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VioBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
