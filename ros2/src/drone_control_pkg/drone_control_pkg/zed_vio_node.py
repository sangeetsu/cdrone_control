"""Publish measured ZED 2i ownship pose to MAVROS for local hover."""

from pathlib import Path
import time

import numpy as np
import pyzed.sl as sl
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from drone_control_pkg.deployment_config import configured_drone_id, configured_mavros_namespace
from drone_control_pkg.topic_utils import cdrone_topic, join_topic
from drone_control_pkg.zed_vio import PoseGate, load_mount, transform_camera_to_body


class ZedVioNode(Node):
    def __init__(self):
        super().__init__("zed_vio_node")
        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        default_config = (Path(get_package_share_directory("drone_control_pkg"))
                          / "config/zed_hover_mount.yaml")
        self.declare_parameter("mount_config", str(default_config))
        drone_id = str(self.get_parameter("drone_id").value)
        namespace = str(self.get_parameter("mavros_namespace").value)
        self.mount = load_mount(str(self.get_parameter("mount_config").value))
        self.gate = PoseGate()
        self.pose = sl.Pose()
        self.camera = None
        self.status = "mount uncalibrated; enter measured body-to-camera transform"
        self.last_status = ""
        self.source_pub = self.create_publisher(
            PoseWithCovarianceStamped, cdrone_topic(drone_id, "zed/pose_cov"), 10)
        self.mavros_pub = self.create_publisher(
            PoseWithCovarianceStamped, join_topic(namespace, "vision_pose/pose_cov"), 10)
        self.status_pub = self.create_publisher(
            String, cdrone_topic(drone_id, "zed/tracking_status"), 10)
        self.create_timer(0.5, self.publish_status)
        if self.mount is None:
            self.get_logger().error(self.status)
            return
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 30
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.camera = sl.Camera()
        result = self.camera.open(init)
        if result != sl.ERROR_CODE.SUCCESS:
            self.status = f"camera open failed: {result}"
            self.get_logger().error(self.status)
            self.camera = None
            return
        tracking = sl.PositionalTrackingParameters()
        tracking.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        tracking.enable_imu_fusion = True
        result = self.camera.enable_positional_tracking(tracking)
        if result != sl.ERROR_CODE.SUCCESS:
            self.status = f"position tracking failed: {result}"
            self.get_logger().error(self.status)
            self.camera.close()
            self.camera = None
            return
        self.status = "waiting for healthy camera frames"
        self.create_timer(0.001, self.capture)

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)
        if self.status != self.last_status:
            self.get_logger().info(self.status)
            self.last_status = self.status

    def capture(self):
        if self.camera.grab() != sl.ERROR_CODE.SUCCESS:
            self.status = "camera grab failed"
            return
        state = self.camera.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        stamp_ns = int(self.pose.timestamp.get_nanoseconds())
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            self.status = f"tracking state {state}"
            return
        try:
            position, orientation, covariance = transform_camera_to_body(
                self.pose.get_translation().get(), self.pose.get_orientation().get(),
                self.pose.pose_covariance, self.mount)
        except ValueError as exc:
            self.status = str(exc)
            return
        accepted, reason = self.gate.accept(str(state), stamp_ns, time.time_ns(),
                                            position, orientation)
        if not accepted:
            self.status = reason
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = "zed_odom"
        (msg.pose.pose.position.x, msg.pose.pose.position.y,
         msg.pose.pose.position.z) = map(float, position)
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w) = map(float, orientation)
        msg.pose.covariance = list(map(float, np.ravel(covariance)))
        self.source_pub.publish(msg)
        self.mavros_pub.publish(msg)
        self.status = "OK"

    def destroy_node(self):
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedVioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
