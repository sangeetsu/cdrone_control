from __future__ import annotations

from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool

from drone_control_pkg.deployment_config import (
    configured_drone_id,
    configured_mavros_namespace,
)
from drone_control_pkg.topic_utils import cdrone_topic, join_topic


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min(value, max_v), min_v)


class MavrosVelocityNode(Node):
    def __init__(self) -> None:
        super().__init__("mavros_velocity_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("watchdog_timeout_s", 0.5)
        self.declare_parameter("max_vel_xy_mps", 1.5)
        self.declare_parameter("max_vel_z_mps", 0.8)
        self.declare_parameter("max_yaw_rate_rps", 0.6)
        self.declare_parameter("require_guided_mode", True)
        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("estop_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("local_pose_topic", "")
        self.declare_parameter("output_topic", "")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.watchdog_timeout_s = float(self.get_parameter("watchdog_timeout_s").value)
        self.max_vel_xy_mps = float(self.get_parameter("max_vel_xy_mps").value)
        self.max_vel_z_mps = float(self.get_parameter("max_vel_z_mps").value)
        self.max_yaw_rate_rps = float(self.get_parameter("max_yaw_rate_rps").value)
        self.require_guided_mode = bool(self.get_parameter("require_guided_mode").value)
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
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
        self.output_topic = (
            str(self.get_parameter("output_topic").value).strip()
            or join_topic(self.mavros_namespace, "setpoint_velocity/cmd_vel")
        )

        self.latest_cmd = TwistStamped()
        self.latest_cmd_time_s = 0.0
        self.estop = False
        self.state = State()
        self.local_pose = PoseStamped()

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )
        self.estop_sub = self.create_subscription(
            Bool, self.estop_topic, self.estop_callback, 10
        )
        self.state_sub = self.create_subscription(
            State, self.state_topic, self.state_callback, 10
        )
        # MAVROS publishes local_position/pose with BEST_EFFORT reliability
        _best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.local_pose_sub = self.create_subscription(
            PoseStamped, self.local_pose_topic, self.local_pose_callback,
            _best_effort_qos,
        )
        self.velocity_pub = self.create_publisher(
            TwistStamped, self.output_topic, 10
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0), self.publish_loop
        )
        self.get_logger().info("MAVROS velocity node started.")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def latest_cmd_is_effectively_zero(self) -> bool:
        cmd = self.latest_cmd.twist
        return (
            abs(float(cmd.linear.x)) < 1e-4
            and abs(float(cmd.linear.y)) < 1e-4
            and abs(float(cmd.linear.z)) < 1e-4
            and abs(float(cmd.angular.z)) < 1e-4
        )

    def cmd_callback(self, msg: TwistStamped) -> None:
        cmd = deepcopy(msg)
        cmd.twist.linear.x = clamp(
            cmd.twist.linear.x, -self.max_vel_xy_mps, self.max_vel_xy_mps
        )
        cmd.twist.linear.y = clamp(
            cmd.twist.linear.y, -self.max_vel_xy_mps, self.max_vel_xy_mps
        )
        cmd.twist.linear.z = clamp(
            cmd.twist.linear.z, -self.max_vel_z_mps, self.max_vel_z_mps
        )
        cmd.twist.angular.z = clamp(
            cmd.twist.angular.z, -self.max_yaw_rate_rps, self.max_yaw_rate_rps
        )
        self.latest_cmd = cmd
        self.latest_cmd_time_s = self.now_s()

    def estop_callback(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def state_callback(self, msg: State) -> None:
        old_mode = self.state.mode
        old_armed = self.state.armed
        self.state = msg
        # Log meaningful state changes
        if msg.mode != old_mode:
            self.get_logger().info(f'Mode changed: {old_mode} → {msg.mode}')
        if msg.armed != old_armed:
            self.get_logger().info(f'Armed: {msg.armed}')

    def local_pose_callback(self, msg: PoseStamped) -> None:
        self.local_pose = msg

    def _guided_gate_open(self) -> bool:
        if not self.require_guided_mode:
            return True
        return bool(
            self.state.armed and (self.state.mode == "OFFBOARD" or self.state.guided)
        )

    def _zero_cmd(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def publish_loop(self) -> None:
        now_s = self.now_s()
        stale = now_s - self.latest_cmd_time_s > self.watchdog_timeout_s

        if self.estop:
            self.velocity_pub.publish(self._zero_cmd())
            return

        if not self._guided_gate_open():
            self.velocity_pub.publish(self._zero_cmd())
            # Warn periodically if non-zero motion commands are being blocked.
            # Milestone/demo nodes often publish zero hold setpoints before
            # OFFBOARD handoff, and those should not look like a teleop problem.
            if not stale and not hasattr(self, '_last_gate_warn'):
                self._last_gate_warn = 0.0
            if (
                not stale
                and not self.latest_cmd_is_effectively_zero()
                and (now_s - getattr(self, '_last_gate_warn', 0.0)) > 5.0
            ):
                self.get_logger().warn(
                    f'Commands blocked: armed={self.state.armed}, '
                    f'mode={self.state.mode}. Velocity commands are held until '
                    'the vehicle is armed and in OFFBOARD.'
                )
                self._last_gate_warn = now_s
            return

        if stale:
            self.velocity_pub.publish(self._zero_cmd())
            return

        cmd = deepcopy(self.latest_cmd)
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.velocity_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MavrosVelocityNode()
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
