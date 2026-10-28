"""Publish measured ZED 2i ownship pose to MAVROS for local hover."""

from pathlib import Path
import math
import time

import numpy as np
import pyzed.sl as sl
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
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
        self.declare_parameter("publish_images", False)
        self.declare_parameter("capture_frames", False)
        self.declare_parameter("image_rate_hz", 10.0)
        self.declare_parameter("rotate_180", True)
        self.declare_parameter("depth_mode", "PERFORMANCE")
        self.declare_parameter("depth_maximum_distance_m", 20.0)
        self.declare_parameter("grab_compute_capping_fps", 0.0)
        drone_id = str(self.get_parameter("drone_id").value)
        namespace = str(self.get_parameter("mavros_namespace").value)
        self.mount = load_mount(str(self.get_parameter("mount_config").value))
        self.publish_images = bool(self.get_parameter("publish_images").value)
        self.capture_frames = self.publish_images or bool(self.get_parameter("capture_frames").value)
        self.frame_sink = None
        self.image_period_ns = int(1e9 / max(float(self.get_parameter("image_rate_hz").value), 1.0))
        self.rotate_180 = bool(self.get_parameter("rotate_180").value)
        self.last_image_stamp_ns = 0
        self.last_timing_warning_ns = 0
        self.last_jump_warning_ns = 0
        self.last_accepted_pose_monotonic_ns = 0
        self.next_grab_attempt_ns = 0
        self.last_rate_log_ns = time.time_ns()
        self.capture_cycles = 0
        self.accepted_poses = 0
        self.delivered_frames = 0
        self.grab_time_ms_sum = 0.0
        self.delivery_time_ms_sum = 0.0
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
        if self.publish_images:
            self.color_pub = self.create_publisher(
                Image, cdrone_topic(drone_id, "zed/color/image_rect"), qos_profile_sensor_data)
            self.depth_pub = self.create_publisher(
                Image, cdrone_topic(drone_id, "zed/depth/image_rect"), qos_profile_sensor_data)
            self.info_pub = self.create_publisher(
                CameraInfo, cdrone_topic(drone_id, "zed/color/camera_info"), qos_profile_sensor_data)
        # Keep health publishing responsive if the SDK blocks inside grab().
        self.status_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.5, self.publish_status,
                          callback_group=self.status_callback_group)
        if self.mount is None:
            self.get_logger().error(self.status)
            return
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 30
        cap_fps = float(self.get_parameter("grab_compute_capping_fps").value)
        if not math.isfinite(cap_fps) or cap_fps < 0.0 or cap_fps >= init.camera_fps:
            raise ValueError("grab_compute_capping_fps must be 0 or below camera_fps")
        init.grab_compute_capping_fps = cap_fps
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        depth_mode = str(self.get_parameter("depth_mode").value).strip().upper()
        if depth_mode not in {"PERFORMANCE", "QUALITY", "NEURAL", "NEURAL_PLUS"}:
            raise ValueError(f"unsupported ZED depth mode: {depth_mode}")
        init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
        if self.capture_frames:
            init.depth_minimum_distance = 0.4
            init.depth_maximum_distance = float(
                self.get_parameter("depth_maximum_distance_m").value)
        self.get_logger().info(
            f"Opening ZED HD720@30, depth={depth_mode}, "
            f"compute_cap={cap_fps:.1f} Hz, image_sample={1e9 / self.image_period_ns:.1f} Hz")
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
        if self.capture_frames:
            calibration = self.camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
            configuration = self.camera.get_camera_information().camera_configuration
            self.image_width = int(configuration.resolution.width)
            self.image_height = int(configuration.resolution.height)
            self.intrinsics = (float(calibration.fx), float(calibration.fy),
                               float(calibration.cx), float(calibration.cy))
            self.color_mat = sl.Mat()
            self.depth_mat = sl.Mat()
        self.create_timer(0.001, self.capture)

    def publish_status(self):
        if (self.status == "OK" and self.last_accepted_pose_monotonic_ns and
                time.monotonic_ns() - self.last_accepted_pose_monotonic_ns > 750_000_000):
            self.status = "camera frame timeout"
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)
        if self.status != self.last_status:
            self.get_logger().info(self.status)
            self.last_status = self.status

    def capture(self):
        if time.monotonic_ns() < self.next_grab_attempt_ns:
            return
        grab_start_ns = time.time_ns()
        if self.camera.grab() != sl.ERROR_CODE.SUCCESS:
            self.status = "camera grab failed"
            # A disconnected USB camera can fail immediately. Avoid a 1 kHz
            # retry loop that monopolizes a CPU while the link recovers.
            self.next_grab_attempt_ns = time.monotonic_ns() + 250_000_000
            return
        after_grab_ns = time.time_ns()
        frame_stamp_ns = int(self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        state = self.camera.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        stamp_ns = int(self.pose.timestamp.get_nanoseconds())
        after_position_ns = time.time_ns()
        pose_check_ns = after_position_ns
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            self.status = f"tracking state {state}"
        else:
            try:
                position, orientation, covariance = transform_camera_to_body(
                    self.pose.get_translation().get(), self.pose.get_orientation().get(),
                    self.pose.pose_covariance, self.mount)
            except ValueError as exc:
                self.status = str(exc)
            else:
                pose_check_ns = time.time_ns()
                accepted, reason = self.gate.accept(str(state), stamp_ns, pose_check_ns,
                                                    position, orientation)
                if not accepted:
                    self.status = reason
                    if (reason == "ZED pose jump" and self.gate.last_jump is not None and
                            pose_check_ns - self.last_jump_warning_ns >= 2_000_000_000):
                        jump = self.gate.last_jump
                        self.get_logger().warn(
                            "ZED pose jump rejected: "
                            f"translation_m={jump['translation_m']:.3f}, "
                            f"rotation_rad={jump['rotation_rad']:.3f}, "
                            f"yaw_change_rad={jump['yaw_change_rad']:.3f}, "
                            f"frame_interval_s={jump['frame_interval_s']:.3f}")
                        self.last_jump_warning_ns = pose_check_ns
                else:
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
                    self.accepted_poses += 1
                    self.status = "OK"
                    self.last_accepted_pose_monotonic_ns = time.monotonic_ns()

        frame_delivery_ms = 0.0
        if (self.capture_frames and (self.publish_images or
                                     (self.frame_sink is not None and self.status == "OK")) and
                frame_stamp_ns - self.last_image_stamp_ns >= self.image_period_ns):
            image_start_ns = time.time_ns()
            if self.deliver_camera_frame(frame_stamp_ns):
                self.last_image_stamp_ns = frame_stamp_ns
                self.delivered_frames += 1
            frame_delivery_ms = (time.time_ns() - image_start_ns) / 1e6
        cycle_end_ns = time.time_ns()
        self.capture_cycles += 1
        self.grab_time_ms_sum += (after_grab_ns - grab_start_ns) / 1e6
        self.delivery_time_ms_sum += frame_delivery_ms
        elapsed_s = (cycle_end_ns - self.last_rate_log_ns) / 1e9
        if elapsed_s >= 5.0:
            count = max(self.capture_cycles, 1)
            self.get_logger().info(
                f"ZED timing: capture_hz={self.capture_cycles / elapsed_s:.2f}, "
                f"accepted_pose_hz={self.accepted_poses / elapsed_s:.2f}, "
                f"delivered_image_hz={self.delivered_frames / elapsed_s:.2f}, "
                f"mean_grab_ms={self.grab_time_ms_sum / count:.1f}, "
                f"mean_delivery_ms={self.delivery_time_ms_sum / count:.1f}")
            self.last_rate_log_ns = cycle_end_ns
            self.capture_cycles = 0
            self.accepted_poses = 0
            self.delivered_frames = 0
            self.grab_time_ms_sum = 0.0
            self.delivery_time_ms_sum = 0.0
        cycle_ms = (cycle_end_ns - grab_start_ns) / 1e6
        if ((self.status != "OK" or cycle_ms >= 500.0) and
                cycle_end_ns - self.last_timing_warning_ns >= 5_000_000_000):
            self.last_timing_warning_ns = cycle_end_ns
            self.get_logger().warn(
                f"ZED capture timing: status={self.status}, cycle_ms={cycle_ms:.1f}, "
                f"grab_ms={(after_grab_ns - grab_start_ns) / 1e6:.1f}, "
                f"image_age_after_grab_ms={(after_grab_ns - frame_stamp_ns) / 1e6:.1f}, "
                f"pose_age_at_gate_ms={(pose_check_ns - stamp_ns) / 1e6:.1f}, "
                f"get_position_ms={(after_position_ns - after_grab_ns) / 1e6:.1f}, "
                f"frame_delivery_ms={frame_delivery_ms:.1f}")

    def deliver_camera_frame(self, stamp_ns):
        color_result = self.camera.retrieve_image(self.color_mat, sl.VIEW.LEFT_BGR)
        depth_result = self.camera.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)
        if color_result != sl.ERROR_CODE.SUCCESS or depth_result != sl.ERROR_CODE.SUCCESS:
            self.status = f"camera image/depth retrieval failed: {color_result}, {depth_result}"
            return False
        color = self.color_mat.get_data()
        depth = self.depth_mat.get_data()
        if color is None or depth is None:
            self.status = "camera image/depth data missing"
            return False
        color = color[:, :, :3]
        depth = (np.asarray(depth, dtype=np.float32)[:, :, 0]
                 if np.asarray(depth).ndim == 3 else np.asarray(depth, dtype=np.float32))
        if self.rotate_180:
            color = np.ascontiguousarray(np.rot90(color, 2))
            depth = np.ascontiguousarray(np.rot90(depth, 2))
        else:
            # SDK Mat storage is reused by the next grab; inference may still
            # be reading this frame in the other executor thread.
            color = np.ascontiguousarray(color).copy()
            depth = np.ascontiguousarray(depth).copy()
        fx, fy, cx, cy = self.intrinsics
        if self.rotate_180:
            cx, cy = color.shape[1] - 1 - cx, color.shape[0] - 1 - cy
        if self.frame_sink is not None and self.status == "OK":
            self.frame_sink(color, depth, stamp_ns, (fx, fy, cx, cy))
        if not self.publish_images:
            return True
        stamp = self.get_clock().now().to_msg()
        stamp.sec, stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        frame_id = "zed_left_optical"
        color_msg = Image()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = frame_id
        color_msg.height, color_msg.width = color.shape[:2]
        color_msg.encoding = "bgr8"
        color_msg.step = color_msg.width * 3
        color_msg.data = color.tobytes()
        depth_msg = Image()
        depth_msg.header = color_msg.header
        depth_msg.height, depth_msg.width = depth.shape
        depth_msg.encoding = "32FC1"
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = depth.tobytes()
        info = CameraInfo()
        info.header = color_msg.header
        info.width, info.height = color_msg.width, color_msg.height
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)
        return True

    def destroy_node(self):
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedVioNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
