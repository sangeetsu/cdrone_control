from __future__ import annotations

import rclpy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import HomePosition, State
from rclpy import qos
from rclpy.node import Node

from drone_control_pkg.deployment_config import configured_mavros_namespace
from drone_control_pkg.topic_utils import join_topic

STATE_QOS = qos.QoSProfile(
    depth=10,
    durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=qos.QoSHistoryPolicy.KEEP_LAST,
    reliability=qos.QoSReliabilityPolicy.RELIABLE,
)

PUB_QOS = qos.QoSProfile(
    depth=10,
    durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=qos.QoSReliabilityPolicy.RELIABLE,
)


class DroneSetup(Node):
    def __init__(self) -> None:
        super().__init__("drone_setup")

        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("global_origin_latitude_deg", 0.0)
        self.declare_parameter("global_origin_longitude_deg", 0.0)
        self.declare_parameter("global_origin_altitude_m", 0.0)
        self.declare_parameter("home_position_x_m", 0.0)
        self.declare_parameter("home_position_y_m", 0.0)
        self.declare_parameter("home_position_z_m", 0.0)
        self.declare_parameter("home_approach_z_m", 1.0)
        self.declare_parameter("retry_period_s", 1.0)
        self.declare_parameter("assume_global_origin_after_publish", False)
        self.declare_parameter("assume_home_position_after_publish", False)

        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.global_origin_latitude_deg = float(
            self.get_parameter("global_origin_latitude_deg").value
        )
        self.global_origin_longitude_deg = float(
            self.get_parameter("global_origin_longitude_deg").value
        )
        self.global_origin_altitude_m = float(
            self.get_parameter("global_origin_altitude_m").value
        )
        self.home_position_x_m = float(self.get_parameter("home_position_x_m").value)
        self.home_position_y_m = float(self.get_parameter("home_position_y_m").value)
        self.home_position_z_m = float(self.get_parameter("home_position_z_m").value)
        self.home_approach_z_m = float(self.get_parameter("home_approach_z_m").value)
        self.retry_period_s = max(
            float(self.get_parameter("retry_period_s").value),
            0.1,
        )
        self.assume_home_position_after_publish = bool(
            self.get_parameter("assume_home_position_after_publish").value
        )
        self.assume_global_origin_after_publish = bool(
            self.get_parameter("assume_global_origin_after_publish").value
        )

        self.state_topic = join_topic(self.mavros_namespace, "state")
        self.home_position_set_topic = join_topic(
            self.mavros_namespace, "home_position/set"
        )
        self.home_position_topic = join_topic(
            self.mavros_namespace, "home_position/home"
        )
        self.global_origin_set_topic = join_topic(
            self.mavros_namespace, "global_position/set_gp_origin"
        )
        self.global_origin_topic = join_topic(
            self.mavros_namespace, "global_position/gp_origin"
        )

        self.connected = False
        self.last_connect_time_s = 0.0
        self.last_home_position_time_s = 0.0
        self.last_global_origin_time_s = 0.0
        self.publish_attempt_count = 0
        self.setup_complete_logged = False

        self.create_subscription(State, self.state_topic, self.state_callback, STATE_QOS)
        self.create_subscription(
            HomePosition,
            self.home_position_topic,
            self.home_position_callback,
            STATE_QOS,
        )
        self.create_subscription(
            GeoPointStamped,
            self.global_origin_topic,
            self.global_origin_callback,
            STATE_QOS,
        )

        self.set_home_pub = self.create_publisher(
            HomePosition, self.home_position_set_topic, PUB_QOS
        )
        self.set_gp_pub = self.create_publisher(
            GeoPointStamped, self.global_origin_set_topic, PUB_QOS
        )

        self.timer = self.create_timer(self.retry_period_s, self.timer_callback)
        self.get_logger().info(
            "Drone setup node started: "
            f"{self.global_origin_set_topic}, {self.home_position_set_topic}; "
            f"origin=({self.global_origin_latitude_deg:.7f}, "
            f"{self.global_origin_longitude_deg:.7f}, "
            f"{self.global_origin_altitude_m:.4f}m) "
            f"home=({self.home_position_x_m:.3f}, {self.home_position_y_m:.3f}, "
            f"{self.home_position_z_m:.3f})"
        )
        if (
            abs(self.global_origin_latitude_deg) < 1e-9
            and abs(self.global_origin_longitude_deg) < 1e-9
        ):
            self.get_logger().warn(
                "Using a synthetic indoor global origin at lat/lon 0,0. "
                "This is acceptable for bench indoor EV use, but replace it "
                "with the lab's real lat/lon/ellipsoid altitude for production."
            )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def state_callback(self, msg: State) -> None:
        was_connected = self.connected
        self.connected = bool(msg.connected)

        if self.connected and not was_connected:
            self.last_connect_time_s = self.now_s()
            self.last_home_position_time_s = 0.0
            self.last_global_origin_time_s = 0.0
            self.publish_attempt_count = 0
            self.setup_complete_logged = False
            self.get_logger().info(
                "FCU connected; publishing indoor global origin and home position"
            )
        elif was_connected and not self.connected:
            self.last_home_position_time_s = 0.0
            self.last_global_origin_time_s = 0.0
            self.setup_complete_logged = False
            self.get_logger().warn("FCU disconnected; waiting to republish setup")

    def home_position_callback(self, msg: HomePosition) -> None:
        del msg
        self.last_home_position_time_s = self.now_s()

    def global_origin_callback(self, msg: GeoPointStamped) -> None:
        del msg
        self.last_global_origin_time_s = self.now_s()

    def home_position_ready(self) -> bool:
        return self.last_home_position_time_s >= self.last_connect_time_s > 0.0

    def global_origin_ready(self) -> bool:
        return self.last_global_origin_time_s >= self.last_connect_time_s > 0.0

    def make_home_position_msg(self) -> HomePosition:
        msg = HomePosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.geo.latitude = float(self.global_origin_latitude_deg)
        msg.geo.longitude = float(self.global_origin_longitude_deg)
        msg.geo.altitude = float(self.global_origin_altitude_m)
        msg.position.x = float(self.home_position_x_m)
        msg.position.y = float(self.home_position_y_m)
        msg.position.z = float(self.home_position_z_m)
        msg.orientation.w = 1.0
        msg.approach.z = float(self.home_approach_z_m)
        return msg

    def make_global_origin_msg(self) -> GeoPointStamped:
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.position.latitude = float(self.global_origin_latitude_deg)
        msg.position.longitude = float(self.global_origin_longitude_deg)
        msg.position.altitude = float(self.global_origin_altitude_m)
        return msg

    def timer_callback(self) -> None:
        if not self.connected:
            return

        if self.home_position_ready() and self.global_origin_ready():
            if not self.setup_complete_logged:
                self.setup_complete_logged = True
                self.get_logger().info(
                    "Indoor global origin and home position are confirmed by MAVROS"
                )
            return

        self.publish_attempt_count += 1
        if not self.global_origin_ready():
            self.set_gp_pub.publish(self.make_global_origin_msg())
            if self.assume_global_origin_after_publish:
                self.last_global_origin_time_s = self.now_s()
        if not self.home_position_ready():
            self.set_home_pub.publish(self.make_home_position_msg())
            if self.assume_home_position_after_publish:
                self.last_home_position_time_s = self.now_s()

        self.get_logger().info(
            "Publishing indoor reference setup "
            f"(attempt {self.publish_attempt_count}): "
            f"global_origin_ready={self.global_origin_ready()} "
            f"home_position_ready={self.home_position_ready()}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneSetup()
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
