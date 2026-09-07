from __future__ import annotations

import ast
from copy import deepcopy
from math import cos, sin, sqrt
from typing import Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_control_pkg.deployment_config import configured_drone_id
from drone_control_pkg.topic_utils import external_pose_input_topic


def _normalize_quaternion(
    x: float, y: float, z: float, w: float
) -> tuple[float, float, float, float]:
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def _quaternion_from_euler(
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

    return _normalize_quaternion(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_multiply(
    q0: tuple[float, float, float, float],
    q1: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return _normalize_quaternion(
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    )


def _rotate_vector(
    q: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = _normalize_quaternion(*q)
    vx, vy, vz = vector

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz,
    )


def _stamp_is_zero(msg: PoseStamped) -> bool:
    return msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0


def _parse_vector(
    value: object,
    *,
    expected_len: int,
    name: str,
) -> tuple[float, ...]:
    parsed: object = value
    if isinstance(value, str):
        parsed = ast.literal_eval(value.strip())

    if not isinstance(parsed, Iterable):
        raise ValueError(f"{name} must be an iterable of length {expected_len}")

    items = tuple(float(item) for item in parsed)
    if len(items) != expected_len:
        raise ValueError(f"{name} must contain exactly {expected_len} values")
    return items


def transform_pose_components(
    position_xyz: tuple[float, float, float],
    source_q: tuple[float, float, float, float],
    *,
    frame_q: tuple[float, float, float, float],
    position_offset_m: tuple[float, float, float],
    orientation_offset_q: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    source_q = _normalize_quaternion(*source_q)
    frame_q = _normalize_quaternion(*frame_q)
    body_offset_xyz = _rotate_vector(source_q, position_offset_m)
    source_position_xyz = (
        float(position_xyz[0]) + body_offset_xyz[0],
        float(position_xyz[1]) + body_offset_xyz[1],
        float(position_xyz[2]) + body_offset_xyz[2],
    )
    output_position_xyz = _rotate_vector(frame_q, source_position_xyz)
    output_q = _quaternion_multiply(
        frame_q,
        _quaternion_multiply(source_q, orientation_offset_q),
    )
    return output_position_xyz, output_q


class ExternalPoseAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("external_pose_adapter_node")

        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("source_pose_topic", "/visual_slam/tracking/vo_pose")
        self.declare_parameter("output_pose_topic", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("frame_rpy_rad", [0.0, 0.0, 0.0])
        self.declare_parameter("position_offset_m", [0.0, 0.0, 0.0])
        self.declare_parameter("rpy_offset_rad", [0.0, 0.0, 0.0])
        self.declare_parameter("timeout_s", 0.25)
        self.declare_parameter("source_best_effort", True)

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.source_pose_topic = str(self.get_parameter("source_pose_topic").value).strip()
        self.output_pose_topic = (
            str(self.get_parameter("output_pose_topic").value).strip()
            or external_pose_input_topic(self.drone_id)
        )
        self.map_frame = str(self.get_parameter("map_frame").value).strip()
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.source_best_effort = bool(
            self.get_parameter("source_best_effort").value
        )
        self.position_offset_m = _parse_vector(
            self.get_parameter("position_offset_m").value,
            expected_len=3,
            name="position_offset_m",
        )
        self.frame_rpy_rad = _parse_vector(
            self.get_parameter("frame_rpy_rad").value,
            expected_len=3,
            name="frame_rpy_rad",
        )
        self.rpy_offset_rad = _parse_vector(
            self.get_parameter("rpy_offset_rad").value,
            expected_len=3,
            name="rpy_offset_rad",
        )
        self.frame_q = _quaternion_from_euler(*self.frame_rpy_rad)
        self.orientation_offset_q = _quaternion_from_euler(*self.rpy_offset_rad)

        self.last_source_s = 0.0
        self.last_timeout_warn_s = 0.0

        source_qos = QoSProfile(
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if self.source_best_effort
                else ReliabilityPolicy.RELIABLE
            ),
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseStamped,
            self.source_pose_topic,
            self.pose_callback,
            source_qos,
        )
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.timeout_timer = self.create_timer(0.1, self.check_source_timeout)

        self.get_logger().info(
            "External pose adapter started: "
            f"{self.source_pose_topic} -> {self.output_pose_topic} "
            f"(source_qos={'best_effort' if self.source_best_effort else 'reliable'}, "
            f"frame_rpy_rad={self.frame_rpy_rad})"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg: PoseStamped) -> None:
        self.last_source_s = self.now_s()

        pose_msg = deepcopy(msg)
        pose_msg.header.frame_id = self.map_frame or msg.header.frame_id or "map"
        if _stamp_is_zero(pose_msg):
            pose_msg.header.stamp = self.get_clock().now().to_msg()

        source_q = _normalize_quaternion(
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        )
        position_xyz, out_q = transform_pose_components(
            (
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ),
            source_q,
            frame_q=self.frame_q,
            position_offset_m=self.position_offset_m,
            orientation_offset_q=self.orientation_offset_q,
        )
        pose_msg.pose.position.x = position_xyz[0]
        pose_msg.pose.position.y = position_xyz[1]
        pose_msg.pose.position.z = position_xyz[2]
        pose_msg.pose.orientation.x = out_q[0]
        pose_msg.pose.orientation.y = out_q[1]
        pose_msg.pose.orientation.z = out_q[2]
        pose_msg.pose.orientation.w = out_q[3]
        self.pose_pub.publish(pose_msg)

    def check_source_timeout(self) -> None:
        if self.timeout_s <= 0.0 or self.last_source_s <= 0.0:
            return

        now_s = self.now_s()
        if now_s - self.last_source_s <= self.timeout_s:
            return
        if now_s - self.last_timeout_warn_s <= max(self.timeout_s, 1.0):
            return

        self.last_timeout_warn_s = now_s
        self.get_logger().warn(
            f"External pose source timed out on {self.source_pose_topic} "
            f"for {now_s - self.last_source_s:.2f}s"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExternalPoseAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
