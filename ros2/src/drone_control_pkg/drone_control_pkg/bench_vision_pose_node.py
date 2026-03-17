from math import cos, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CompanionProcessStatus
from rclpy.node import Node

from drone_control_pkg.topic_utils import join_topic


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = cos(half_roll)
    sr = sin(half_roll)
    cp = cos(half_pitch)
    sp = sin(half_pitch)
    cy = cos(half_yaw)
    sy = sin(half_yaw)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class BenchVisionPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("bench_vision_pose_node")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x_m", 0.0)
        self.declare_parameter("y_m", 0.0)
        self.declare_parameter("z_m", 0.0)
        self.declare_parameter("roll_rad", 0.0)
        self.declare_parameter("pitch_rad", 0.0)
        self.declare_parameter("yaw_rad", 0.0)
        self.declare_parameter("publish_companion_status", True)
        self.declare_parameter("mavros_namespace", "/mavros")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.x_m = float(self.get_parameter("x_m").value)
        self.y_m = float(self.get_parameter("y_m").value)
        self.z_m = float(self.get_parameter("z_m").value)
        self.roll_rad = float(self.get_parameter("roll_rad").value)
        self.pitch_rad = float(self.get_parameter("pitch_rad").value)
        self.yaw_rad = float(self.get_parameter("yaw_rad").value)
        self.publish_companion_status = bool(
            self.get_parameter("publish_companion_status").value
        )
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)

        self.pose_pub = self.create_publisher(
            PoseStamped, join_topic(self.mavros_namespace, "vision_pose/pose"), 10
        )
        self.status_pub = self.create_publisher(
            CompanionProcessStatus,
            join_topic(self.mavros_namespace, "companion_process/status"),
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0), self.publish_loop
        )

        self.get_logger().warn(
            "Publishing fixed bench-only external vision pose. "
            "Do not fly with this node."
        )
        self.get_logger().info(
            f"Pose: frame={self.frame_id}, x={self.x_m:.2f}, y={self.y_m:.2f}, "
            f"z={self.z_m:.2f}, roll={self.roll_rad:.2f}, "
            f"pitch={self.pitch_rad:.2f}, yaw={self.yaw_rad:.2f}"
        )

    def publish_loop(self) -> None:
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(
            self.roll_rad, self.pitch_rad, self.yaw_rad
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = self.x_m
        pose_msg.pose.position.y = self.y_m
        pose_msg.pose.position.z = self.z_m
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        if not self.publish_companion_status:
            return

        status_msg = CompanionProcessStatus()
        status_msg.header.stamp = now
        status_msg.state = CompanionProcessStatus.MAV_STATE_ACTIVE
        status_msg.component = (
            CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        )
        self.status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BenchVisionPoseNode()
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
