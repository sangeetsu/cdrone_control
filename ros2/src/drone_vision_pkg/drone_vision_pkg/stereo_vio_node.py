from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

try:
    import cv2
except Exception:  # pragma: no cover - platform specific
    cv2 = None

from drone_vision_pkg.stereo_utils import (
    build_csi_pipeline,
    normalize_quaternion,
    quaternion_to_rotation_matrix,
    rpy_to_rotation_matrix,
    triangulate_point,
)


@dataclass
class StereoFeatureState:
    gray_left: np.ndarray
    points_left: np.ndarray
    points_3d_cam: np.ndarray


class StereoVioNode(Node):
    def __init__(self) -> None:
        super().__init__("stereo_vio_node")

        if cv2 is None:
            raise RuntimeError("OpenCV is required for stereo_vio_node.")

        self._declare_parameters()
        self._load_parameters()
        self._load_calibration(self.calibration_path)

        self.left_cap = self._open_capture(self.left_sensor_id, self.left_url)
        self.right_cap = self._open_capture(self.right_sensor_id, self.right_url)

        self.latest_imu_quat_xyzw: Optional[np.ndarray] = None
        self.latest_imu_time_s: float = 0.0
        self.last_pose_time_s: Optional[float] = None
        self.last_body_rotation_world: Optional[np.ndarray] = None
        self.position_world = np.zeros(3, dtype=float)
        self.velocity_world = np.zeros(3, dtype=float)
        self.prev_state: Optional[StereoFeatureState] = None
        self.frame_counter = 0

        self.pose_pub = self.create_publisher(PoseStamped, "/cdrone/vio/pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/cdrone/vio/odometry", 10)
        self.healthy_pub = self.create_publisher(Bool, "/cdrone/vio/healthy", 10)
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            20,
        )

        self.timer = self.create_timer(
            1.0 / max(self.vio_rate_hz, 1.0), self.process_frame
        )
        self.get_logger().info(
            "Stereo VIO started. "
            f"source_mode={self.source_mode}, imu_topic={self.imu_topic}, "
            f"calibration={self.calibration_path}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("source_mode", "csi")
        self.declare_parameter("left_sensor_id", 0)
        self.declare_parameter("right_sensor_id", 1)
        self.declare_parameter("left_url", "rtsp://127.0.0.1:5600/camera1")
        self.declare_parameter("right_url", "rtsp://127.0.0.1:5600/camera2")
        self.declare_parameter("capture_width", 1920)
        self.declare_parameter("capture_height", 1080)
        self.declare_parameter("display_width", 640)
        self.declare_parameter("display_height", 640)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("flip_method", 0)
        self.declare_parameter("calibration_path", "stereo_calibration.npz")
        self.declare_parameter("camera_body_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_body_rpy_rad", [0.0, 0.0, 0.0])
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        self.declare_parameter("imu_timeout_s", 0.2)
        self.declare_parameter("vio_rate_hz", 15.0)
        self.declare_parameter("max_corners", 300)
        self.declare_parameter("quality_level", 0.01)
        self.declare_parameter("min_distance_px", 12.0)
        self.declare_parameter("stereo_epipolar_tolerance_px", 2.5)
        self.declare_parameter("min_depth_m", 0.25)
        self.declare_parameter("max_depth_m", 20.0)
        self.declare_parameter("min_pnp_inliers", 20)
        self.declare_parameter("min_tracked_points", 30)
        self.declare_parameter("min_translation_norm_m", 0.0005)
        self.declare_parameter("world_frame_id", "map")
        self.declare_parameter("body_frame_id", "base_link")
        self.declare_parameter("rtsp_transport", "udp")

    def _load_parameters(self) -> None:
        self.source_mode = str(self.get_parameter("source_mode").value).strip().lower()
        self.left_sensor_id = int(self.get_parameter("left_sensor_id").value)
        self.right_sensor_id = int(self.get_parameter("right_sensor_id").value)
        self.left_url = str(self.get_parameter("left_url").value)
        self.right_url = str(self.get_parameter("right_url").value)
        self.capture_width = int(self.get_parameter("capture_width").value)
        self.capture_height = int(self.get_parameter("capture_height").value)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)
        self.framerate = int(self.get_parameter("framerate").value)
        self.flip_method = int(self.get_parameter("flip_method").value)
        self.calibration_path = str(self.get_parameter("calibration_path").value)
        self.rotation_body_from_camera = rpy_to_rotation_matrix(
            np.array(self.get_parameter("camera_body_rpy_rad").value, dtype=float)
        )
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.imu_timeout_s = float(self.get_parameter("imu_timeout_s").value)
        self.vio_rate_hz = float(self.get_parameter("vio_rate_hz").value)
        self.max_corners = int(self.get_parameter("max_corners").value)
        self.quality_level = float(self.get_parameter("quality_level").value)
        self.min_distance_px = float(self.get_parameter("min_distance_px").value)
        self.stereo_epipolar_tolerance_px = float(
            self.get_parameter("stereo_epipolar_tolerance_px").value
        )
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_pnp_inliers = int(self.get_parameter("min_pnp_inliers").value)
        self.min_tracked_points = int(self.get_parameter("min_tracked_points").value)
        self.min_translation_norm_m = float(
            self.get_parameter("min_translation_norm_m").value
        )
        self.world_frame_id = str(self.get_parameter("world_frame_id").value)
        self.body_frame_id = str(self.get_parameter("body_frame_id").value)

        if self.source_mode not in ("csi", "rtsp"):
            self.get_logger().warn(
                f"Unsupported source_mode='{self.source_mode}', defaulting to csi."
            )
            self.source_mode = "csi"

        if self.source_mode == "rtsp":
            transport = str(self.get_parameter("rtsp_transport").value).strip().lower()
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{transport}"

    def _load_calibration(self, calibration_path: str) -> None:
        path = Path(calibration_path)
        if not path.exists():
            raise FileNotFoundError(f"Calibration file not found: {path}")

        data = np.load(str(path))
        self.p1 = data["P1"]
        self.p2 = data["P2"]
        self.map1_x = data["map1_x"]
        self.map1_y = data["map1_y"]
        self.map2_x = data["map2_x"]
        self.map2_y = data["map2_y"]
        self.camera_matrix = self.p1[:, :3].copy()

    def _open_capture(self, sensor_id: int, url: str):
        if self.source_mode == "csi":
            pipeline = build_csi_pipeline(
                sensor_id=sensor_id,
                capture_width=self.capture_width,
                capture_height=self.capture_height,
                display_width=self.display_width,
                display_height=self.display_height,
                framerate=self.framerate,
                flip_method=self.flip_method,
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            cap = cv2.VideoCapture(url)

        if not cap.isOpened():
            source = f"sensor-id {sensor_id}" if self.source_mode == "csi" else url
            raise RuntimeError(f"Failed to open camera source: {source}")
        return cap

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def imu_callback(self, msg: Imu) -> None:
        quat = np.array(
            [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
            dtype=float,
        )
        self.latest_imu_quat_xyzw = normalize_quaternion(quat)
        self.latest_imu_time_s = self.now_s()

    def _read_rectified_pair(self) -> tuple[np.ndarray, np.ndarray]:
        ok_left, left = self.left_cap.read()
        ok_right, right = self.right_cap.read()
        if not ok_left or not ok_right:
            raise RuntimeError("Stereo frame read failed.")

        rect_left = cv2.remap(left, self.map1_x, self.map1_y, cv2.INTER_LINEAR)
        rect_right = cv2.remap(right, self.map2_x, self.map2_y, cv2.INTER_LINEAR)
        return rect_left, rect_right

    def _triangulate_points(
        self,
        left_points: np.ndarray,
        right_points: np.ndarray,
    ) -> np.ndarray:
        points_3d = []
        for left_pt, right_pt in zip(left_points, right_points):
            point_3d = triangulate_point(
                self.p1,
                self.p2,
                (float(left_pt[0]), float(left_pt[1])),
                (float(right_pt[0]), float(right_pt[1])),
            )
            points_3d.append(point_3d)
        if not points_3d:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(points_3d, dtype=np.float32)

    def _filter_stereo_matches(
        self,
        left_points: np.ndarray,
        right_points: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        if left_points.size == 0 or right_points.size == 0:
            return (
                np.empty((0, 2), dtype=np.float32),
                np.empty((0, 2), dtype=np.float32),
            )

        disparities = left_points[:, 0] - right_points[:, 0]
        y_error = np.abs(left_points[:, 1] - right_points[:, 1])
        mask = (
            np.isfinite(left_points).all(axis=1)
            & np.isfinite(right_points).all(axis=1)
            & (disparities > 0.5)
            & (y_error <= self.stereo_epipolar_tolerance_px)
        )
        return left_points[mask], right_points[mask]

    def _build_feature_state(
        self,
        gray_left: np.ndarray,
        gray_right: np.ndarray,
    ) -> Optional[StereoFeatureState]:
        corners = cv2.goodFeaturesToTrack(
            gray_left,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance_px,
            blockSize=7,
        )
        if corners is None:
            return None

        left_points = corners.reshape(-1, 2).astype(np.float32)
        right_points, status, _ = cv2.calcOpticalFlowPyrLK(
            gray_left,
            gray_right,
            left_points.reshape(-1, 1, 2),
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30,
                0.01,
            ),
        )
        if right_points is None or status is None:
            return None

        status_mask = status.reshape(-1).astype(bool)
        left_points = left_points[status_mask]
        right_points = right_points.reshape(-1, 2)[status_mask]
        left_points, right_points = self._filter_stereo_matches(
            left_points,
            right_points,
        )
        if len(left_points) < self.min_tracked_points:
            return None

        points_3d = self._triangulate_points(left_points, right_points)
        valid_depth = (
            np.isfinite(points_3d).all(axis=1)
            & (points_3d[:, 2] >= self.min_depth_m)
            & (points_3d[:, 2] <= self.max_depth_m)
        )
        left_points = left_points[valid_depth]
        points_3d = points_3d[valid_depth]
        if len(left_points) < self.min_tracked_points:
            return None

        return StereoFeatureState(
            gray_left=gray_left,
            points_left=left_points.astype(np.float32),
            points_3d_cam=points_3d.astype(np.float32),
        )

    def _publish_health(self, healthy: bool) -> None:
        msg = Bool()
        msg.data = healthy
        self.healthy_pub.publish(msg)

    def _publish_pose(self, now_msg, body_quat_xyzw: np.ndarray) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = self.world_frame_id
        pose_msg.pose.position.x = float(self.position_world[0])
        pose_msg.pose.position.y = float(self.position_world[1])
        pose_msg.pose.position.z = float(self.position_world[2])
        pose_msg.pose.orientation.x = float(body_quat_xyzw[0])
        pose_msg.pose.orientation.y = float(body_quat_xyzw[1])
        pose_msg.pose.orientation.z = float(body_quat_xyzw[2])
        pose_msg.pose.orientation.w = float(body_quat_xyzw[3])
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.body_frame_id
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = float(self.velocity_world[0])
        odom_msg.twist.twist.linear.y = float(self.velocity_world[1])
        odom_msg.twist.twist.linear.z = float(self.velocity_world[2])
        self.odom_pub.publish(odom_msg)

    def _reopen_captures(self) -> None:
        try:
            self.left_cap.release()
            self.right_cap.release()
        except Exception:
            pass
        self.left_cap = self._open_capture(self.left_sensor_id, self.left_url)
        self.right_cap = self._open_capture(self.right_sensor_id, self.right_url)

    def process_frame(self) -> None:
        try:
            left_frame, right_frame = self._read_rectified_pair()
        except Exception as exc:
            self.get_logger().warn(f"{exc} Reopening stereo cameras.")
            self.prev_state = None
            self._publish_health(False)
            self._reopen_captures()
            return

        gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

        if self.latest_imu_quat_xyzw is None:
            if self.frame_counter % 30 == 0:
                self.get_logger().warn("Waiting for IMU orientation on /mavros/imu/data.")
            self.prev_state = self._build_feature_state(gray_left, gray_right)
            self._publish_health(False)
            self.frame_counter += 1
            return

        if self.now_s() - self.latest_imu_time_s > self.imu_timeout_s:
            self.get_logger().warn("IMU data is stale; holding VIO publish.")
            self.prev_state = self._build_feature_state(gray_left, gray_right)
            self._publish_health(False)
            self.frame_counter += 1
            return

        if self.prev_state is None:
            self.prev_state = self._build_feature_state(gray_left, gray_right)
            self.last_body_rotation_world = quaternion_to_rotation_matrix(
                self.latest_imu_quat_xyzw
            )
            self._publish_health(False)
            self.frame_counter += 1
            return

        tracked_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_state.gray_left,
            gray_left,
            self.prev_state.points_left.reshape(-1, 1, 2),
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30,
                0.01,
            ),
        )
        if tracked_points is None or status is None:
            self.prev_state = self._build_feature_state(gray_left, gray_right)
            self._publish_health(False)
            self.frame_counter += 1
            return

        status_mask = status.reshape(-1).astype(bool)
        image_points = tracked_points.reshape(-1, 2)[status_mask]
        object_points = self.prev_state.points_3d_cam[status_mask]

        finite_mask = np.isfinite(image_points).all(axis=1) & np.isfinite(
            object_points
        ).all(axis=1)
        image_points = image_points[finite_mask]
        object_points = object_points[finite_mask]

        if len(image_points) < self.min_tracked_points:
            self.prev_state = self._build_feature_state(gray_left, gray_right)
            self._publish_health(False)
            self.frame_counter += 1
            return

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            self.camera_matrix,
            None,
            flags=cv2.SOLVEPNP_EPNP,
            reprojectionError=3.0,
            iterationsCount=100,
            confidence=0.99,
        )
        if (
            not success
            or inliers is None
            or int(len(inliers)) < self.min_pnp_inliers
        ):
            self.prev_state = self._build_feature_state(gray_left, gray_right)
            self._publish_health(False)
            self.frame_counter += 1
            return

        rotation_current_from_prev, _ = cv2.Rodrigues(rvec)
        translation_current_in_current = tvec.reshape(3)
        delta_camera_prev = -rotation_current_from_prev.T @ translation_current_in_current
        if float(np.linalg.norm(delta_camera_prev)) < self.min_translation_norm_m:
            delta_camera_prev = np.zeros(3, dtype=float)

        current_body_rotation_world = quaternion_to_rotation_matrix(
            self.latest_imu_quat_xyzw
        )
        previous_body_rotation_world = self.last_body_rotation_world
        if previous_body_rotation_world is None:
            previous_body_rotation_world = current_body_rotation_world

        delta_body_prev = self.rotation_body_from_camera @ delta_camera_prev
        delta_world = previous_body_rotation_world @ delta_body_prev
        self.position_world = self.position_world + delta_world

        now_s = self.now_s()
        if self.last_pose_time_s is not None and now_s > self.last_pose_time_s:
            dt = now_s - self.last_pose_time_s
            self.velocity_world = delta_world / dt
        else:
            self.velocity_world = np.zeros(3, dtype=float)
        self.last_pose_time_s = now_s
        self.last_body_rotation_world = current_body_rotation_world

        next_state = self._build_feature_state(gray_left, gray_right)
        self.prev_state = next_state

        self._publish_pose(self.get_clock().now().to_msg(), self.latest_imu_quat_xyzw)
        self._publish_health(next_state is not None)
        self.frame_counter += 1

    def destroy_node(self) -> bool:
        try:
            self.left_cap.release()
            self.right_cap.release()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoVioNode()
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
