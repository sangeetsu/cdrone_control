from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np
import rclpy
from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic
from drone_msgs.msg import (
    PerceptionStatus,
    TargetTrack,
    TargetTrackArray,
    WorldTargetTrack,
    WorldTargetTrackArray,
)
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image

from drone_vision_pkg.model_paths import resolve_model_path
from drone_vision_pkg.projection import RealsenseProjection

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - platform specific
    CvBridge = None

try:
    import cv2
except Exception:  # pragma: no cover - platform specific
    cv2 = None

try:
    import pyrealsense2 as rs
except Exception:  # pragma: no cover - platform specific
    rs = None

try:
    from norfair import Detection, Tracker
except Exception:  # pragma: no cover - platform specific
    Detection = None
    Tracker = None

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - platform specific
    YOLO = None


@dataclass(frozen=True)
class TrackVelocity:
    vector_mps: np.ndarray
    inbound: bool


def get_centroid(points: np.ndarray) -> np.ndarray:
    array = np.asarray(points, dtype=np.float32)
    if array.ndim == 1 and array.shape[0] == 2:
        return array
    if array.ndim == 2 and array.shape == (2, 2):
        return np.array(
            [
                (array[0, 0] + array[1, 0]) / 2.0,
                (array[0, 1] + array[1, 1]) / 2.0,
            ],
            dtype=np.float32,
        )
    return np.mean(array, axis=0, dtype=np.float32)


def map_point_between_frames(
    point: np.ndarray,
    src_shape: tuple[int, ...] | None,
    dst_shape: tuple[int, ...] | None,
) -> np.ndarray:
    if src_shape is None or dst_shape is None:
        return np.asarray(point, dtype=np.float32)
    src_h, src_w = src_shape[:2]
    dst_h, dst_w = dst_shape[:2]
    if src_h <= 0 or src_w <= 0 or dst_h <= 0 or dst_w <= 0:
        return np.asarray(point, dtype=np.float32)
    scale_x = float(dst_w) / float(src_w)
    scale_y = float(dst_h) / float(src_h)
    return np.array([point[0] * scale_x, point[1] * scale_y], dtype=np.float32)


def compute_mean_depth(
    depth_frame: np.ndarray | None,
    center: np.ndarray,
    radius_px: int,
    max_valid_depth_m: float,
) -> float:
    if depth_frame is None:
        return 0.0
    h, w = depth_frame.shape[:2]
    x = int(round(float(center[0])))
    y = int(round(float(center[1])))
    radius_px = max(int(radius_px), 0)
    x0 = max(0, x - radius_px)
    x1 = min(w - 1, x + radius_px)
    y0 = max(0, y - radius_px)
    y1 = min(h - 1, y + radius_px)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    crop = depth_frame[y0 : y1 + 1, x0 : x1 + 1].astype(np.float32)
    if crop.size == 0:
        return 0.0
    if float(np.nanmax(crop)) > 100.0:
        crop *= 0.001
    mask = np.isfinite(crop) & (crop > 0.0) & (crop <= max_valid_depth_m)
    if not np.any(mask):
        return 0.0
    return float(np.median(crop[mask]))


def yolo_boxes_to_detections(boxes) -> list[Detection]:
    if boxes is None or boxes.xyxy is None:
        return []
    xyxy = boxes.xyxy.cpu().numpy()
    if len(xyxy) == 0:
        return []
    conf = boxes.conf.cpu().numpy() if boxes.conf is not None else np.ones(len(xyxy))
    detections: list[Detection] = []
    for idx, bbox in enumerate(xyxy):
        x1, y1, x2, y2 = [float(value) for value in bbox[:4]]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        bbox_area_px = max(0.0, (x2 - x1) * (y2 - y1))
        detections.append(
            Detection(
                points=np.array([cx, cy], dtype=np.float32),
                scores=np.array([float(conf[idx])], dtype=np.float32),
                data={
                    "bbox": [x1, y1, x2, y2],
                    "bbox_area_px": bbox_area_px,
                    "confidence": float(conf[idx]),
                },
            )
        )
    return detections


class RealsenseTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("realsense_tracker_node")

        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("source_mode", "direct")
        self.declare_parameter("tracker_rate_hz", 10.0)
        self.declare_parameter("device_id", 0)
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_input_size", 640)
        self.declare_parameter("detection_conf_threshold", 0.4)
        self.declare_parameter("norfair_distance_threshold", 80.0)
        self.declare_parameter("track_hit_counter_max", 12)
        self.declare_parameter("max_valid_depth_m", 20.0)
        self.declare_parameter("depth_window_px", 5)
        self.declare_parameter("publish_world_tracks", True)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("camera_offset_body_m", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_rpy_body_rad", [0.0, 0.0, 0.0])
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("world_tracks_topic", "")
        self.declare_parameter("perception_status_topic", "")
        self.declare_parameter("align_depth_to_color", True)
        self.declare_parameter("direct_color_width", 848)
        self.declare_parameter("direct_color_height", 480)
        self.declare_parameter("direct_color_fps", 15)
        self.declare_parameter("direct_depth_width", 848)
        self.declare_parameter("direct_depth_height", 480)
        self.declare_parameter("direct_depth_fps", 15)

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.source_mode = str(self.get_parameter("source_mode").value).strip().lower()
        self.tracker_rate_hz = float(self.get_parameter("tracker_rate_hz").value)
        self.device_id = int(self.get_parameter("device_id").value)
        self.model_input_size = int(self.get_parameter("model_input_size").value)
        self.conf_threshold = float(
            self.get_parameter("detection_conf_threshold").value
        )
        self.norfair_distance_threshold = float(
            self.get_parameter("norfair_distance_threshold").value
        )
        self.track_hit_counter_max = int(
            self.get_parameter("track_hit_counter_max").value
        )
        self.max_valid_depth_m = float(self.get_parameter("max_valid_depth_m").value)
        self.depth_window_px = int(self.get_parameter("depth_window_px").value)
        self.publish_world_tracks = bool(
            self.get_parameter("publish_world_tracks").value
        )
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.align_depth_to_color = bool(
            self.get_parameter("align_depth_to_color").value
        )
        self.color_topic = str(self.get_parameter("color_topic").value).strip()
        self.depth_topic = str(self.get_parameter("depth_topic").value).strip()
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        ).strip()
        self.direct_color_width = int(self.get_parameter("direct_color_width").value)
        self.direct_color_height = int(self.get_parameter("direct_color_height").value)
        self.direct_color_fps = int(self.get_parameter("direct_color_fps").value)
        self.direct_depth_width = int(self.get_parameter("direct_depth_width").value)
        self.direct_depth_height = int(self.get_parameter("direct_depth_height").value)
        self.direct_depth_fps = int(self.get_parameter("direct_depth_fps").value)
        pose_topic = str(self.get_parameter("pose_topic").value).strip()
        self.pose_topic = pose_topic or external_pose_input_topic(self.drone_id)
        self.tracks_topic = (
            str(self.get_parameter("tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/tracks")
        )
        self.world_tracks_topic = (
            str(self.get_parameter("world_tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/world_tracks")
        )
        self.perception_status_topic = (
            str(self.get_parameter("perception_status_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/status")
        )

        camera_offset_body_m = [
            float(value)
            for value in self.get_parameter("camera_offset_body_m").value
        ]
        camera_rpy_body_rad = [
            float(value)
            for value in self.get_parameter("camera_rpy_body_rad").value
        ]

        if self.source_mode not in {"direct", "ros"}:
            raise RuntimeError(
                "Unsupported source_mode. Expected 'direct' or 'ros', got "
                f"'{self.source_mode}'."
            )
        if cv2 is None or YOLO is None or Detection is None or Tracker is None:
            raise RuntimeError(
                "Missing dependencies. Ensure OpenCV, ultralytics, and norfair are "
                "installed."
            )
        if self.source_mode == "ros" and CvBridge is None:
            raise RuntimeError(
                "ROS image source_mode requires cv_bridge to be installed."
            )
        if self.source_mode == "direct" and rs is None:
            raise RuntimeError(
                "Direct source_mode requires pyrealsense2 to be installed."
            )

        self.model_path = resolve_model_path(
            str(self.get_parameter("model_path").value)
        )
        self.bridge = CvBridge() if self.source_mode == "ros" and CvBridge else None
        self.projector = RealsenseProjection(
            camera_offset_body_m=camera_offset_body_m,
            camera_rpy_body_rad=camera_rpy_body_rad,
            world_frame=self.world_frame,
        )
        self.model = YOLO(self.model_path, task="detect")
        self.tracker = Tracker(
            distance_function="euclidean",
            distance_threshold=self.norfair_distance_threshold,
            initialization_delay=1,
            hit_counter_max=self.track_hit_counter_max,
        )

        self.latest_color_msg: Image | None = None
        self.latest_depth_msg: Image | None = None
        self.last_color_stamp_key: tuple[int, int] = (-1, -1)
        self.last_process_s = 0.0
        self.last_inference_ms = 0.0
        self.last_wait_warn_s = 0.0
        self.last_pose_warn_s = 0.0
        self.last_intrinsics_warn_s = 0.0
        self.prev_body_positions: dict[int, tuple[np.ndarray, float]] = {}
        self.prev_world_positions: dict[int, tuple[np.ndarray, float]] = {}
        self.rs_pipeline = None
        self.rs_align = None
        self.depth_scale_m = 0.001

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        if self.source_mode == "ros":
            self.create_subscription(
                Image,
                self.color_topic,
                self.color_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                self.depth_topic,
                self.depth_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                qos_profile_sensor_data,
            )
        else:
            self._start_direct_pipeline()

        if self.publish_world_tracks:
            self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                best_effort_qos,
            )

        self.tracks_pub = self.create_publisher(TargetTrackArray, self.tracks_topic, 10)
        self.world_tracks_pub = self.create_publisher(
            WorldTargetTrackArray, self.world_tracks_topic, 10
        )
        self.status_pub = self.create_publisher(
            PerceptionStatus, self.perception_status_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / max(self.tracker_rate_hz, 1.0), self.process_latest_frame
        )

        self.get_logger().info(
            "RealSense tracker started: "
            f"source_mode={self.source_mode}, model={self.model_path}, "
            f"tracks={self.tracks_topic}, "
            f"world_tracks={self.world_tracks_topic}, "
            f"pose={self.pose_topic if self.publish_world_tracks else 'disabled'}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self) -> bool:
        if self.rs_pipeline is not None:
            try:
                self.rs_pipeline.stop()
            except Exception as exc:  # pragma: no cover - device specific cleanup
                self.get_logger().warn(
                    f"Stopping direct RealSense pipeline raised: {exc}"
                )
            finally:
                self.rs_pipeline = None
                self.rs_align = None
        return super().destroy_node()

    def color_callback(self, msg: Image) -> None:
        self.latest_color_msg = msg

    def depth_callback(self, msg: Image) -> None:
        self.latest_depth_msg = msg

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.projector.update_camera_info(msg)

    def pose_callback(self, msg: PoseStamped) -> None:
        self.projector.update_pose(msg)

    def process_latest_frame(self) -> None:
        frame: np.ndarray | None = None
        depth_frame: np.ndarray | None = None

        if self.source_mode == "direct":
            direct_frame = self._read_direct_frame()
            if direct_frame is None:
                return
            frame, depth_frame, stamp_key = direct_frame
        else:
            color_msg = self.latest_color_msg
            if color_msg is None:
                self._warn_throttled(
                    "Waiting for the first color frame on "
                    f"{self.color_topic}.",
                    attr_name="last_wait_warn_s",
                )
                return

            stamp_key = (
                int(color_msg.header.stamp.sec),
                int(color_msg.header.stamp.nanosec),
            )
            if stamp_key == self.last_color_stamp_key:
                return

            frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            if self.latest_depth_msg is not None:
                depth_frame = self.bridge.imgmsg_to_cv2(
                    self.latest_depth_msg,
                    desired_encoding="passthrough",
                )

        if stamp_key == self.last_color_stamp_key:
            return
        self.last_color_stamp_key = stamp_key

        infer_start_s = time.perf_counter()
        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            imgsz=self.model_input_size,
            verbose=False,
            device=self.device_id,
        )
        self.last_inference_ms = (time.perf_counter() - infer_start_s) * 1000.0

        detections = yolo_boxes_to_detections(results[0].boxes if results else None)
        tracked_objects = self.tracker.update(detections)
        now_msg = self.get_clock().now().to_msg()
        process_time_s = self.now_s()

        body_tracks = self._build_body_tracks(
            tracked_objects=tracked_objects,
            frame=frame,
            depth_frame=depth_frame,
            stamp_msg=now_msg,
            now_s=process_time_s,
        )
        world_tracks = self._build_world_tracks(
            body_tracks=body_tracks,
            stamp_msg=now_msg,
            now_s=process_time_s,
        )

        body_array = TargetTrackArray()
        body_array.stamp = now_msg
        body_array.tracks = body_tracks
        self.tracks_pub.publish(body_array)

        world_array = WorldTargetTrackArray()
        world_array.stamp = now_msg
        world_array.frame_id = self.projector.frame_id
        world_array.tracks = world_tracks
        self.world_tracks_pub.publish(world_array)

        tracker_fps = 0.0
        if self.last_process_s > 0.0:
            tracker_fps = 1.0 / max(process_time_s - self.last_process_s, 1e-3)
        self.last_process_s = process_time_s

        status = PerceptionStatus()
        status.stamp = now_msg
        status.drone_id = self.drone_id
        status.tracker_fps = float(tracker_fps)
        status.inference_latency_ms = float(self.last_inference_ms)
        status.left_detections = int(len(detections))
        status.right_detections = 0
        status.paired_detections = 0
        status.active_tracks = int(len(tracked_objects))
        self.status_pub.publish(status)

    def _start_direct_pipeline(self) -> None:
        config = rs.config()
        config.enable_stream(
            rs.stream.color,
            self.direct_color_width,
            self.direct_color_height,
            rs.format.bgr8,
            self.direct_color_fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self.direct_depth_width,
            self.direct_depth_height,
            rs.format.z16,
            self.direct_depth_fps,
        )

        self.rs_pipeline = rs.pipeline()
        profile = self.rs_pipeline.start(config)
        self.rs_align = rs.align(rs.stream.color) if self.align_depth_to_color else None

        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale_m = float(depth_sensor.get_depth_scale())
        except Exception:
            self.depth_scale_m = 0.001

        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
        self.projector.set_intrinsics(
            fx=float(intrinsics.fx),
            fy=float(intrinsics.fy),
            cx=float(intrinsics.ppx),
            cy=float(intrinsics.ppy),
        )

        self.get_logger().info(
            "Started direct RealSense pipeline: "
            f"color={self.direct_color_width}x{self.direct_color_height}@"
            f"{self.direct_color_fps}, depth={self.direct_depth_width}x"
            f"{self.direct_depth_height}@{self.direct_depth_fps}, "
            f"align_depth_to_color={self.align_depth_to_color}, "
            f"depth_scale_m={self.depth_scale_m:.6f}"
        )

    def _read_direct_frame(
        self,
    ) -> tuple[np.ndarray, np.ndarray | None, tuple[int, int]] | None:
        if self.rs_pipeline is None:
            return None
        try:
            frames = self.rs_pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as exc:
            self._warn_throttled(
                f"Waiting for direct RealSense frames failed: {exc}",
                attr_name="last_wait_warn_s",
            )
            return None

        if self.rs_align is not None:
            frames = self.rs_align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            self._warn_throttled(
                "Waiting for the first color frame from the direct RealSense "
                "pipeline.",
                attr_name="last_wait_warn_s",
            )
            return None

        depth_frame = frames.get_depth_frame()
        color = np.asanyarray(color_frame.get_data())
        depth = None
        if depth_frame:
            depth = (
                np.asanyarray(depth_frame.get_data()).astype(np.float32)
                * self.depth_scale_m
            )

        frame_number = int(color_frame.get_frame_number())
        return color, depth, (frame_number, 0)

    def _build_body_tracks(
        self,
        tracked_objects: list,
        frame: np.ndarray,
        depth_frame: np.ndarray | None,
        stamp_msg,
        now_s: float,
    ) -> list[TargetTrack]:
        if not self.projector.has_intrinsics():
            self._warn_throttled(
                "Camera intrinsics have not arrived yet. Body/world tracks are "
                f"waiting on {self.camera_info_topic}.",
                attr_name="last_intrinsics_warn_s",
            )
            return []

        track_messages: list[TargetTrack] = []
        active_track_ids: set[int] = set()

        for tracked_object in tracked_objects:
            detection = tracked_object.last_detection
            if detection is None:
                continue
            track_id = int(tracked_object.id)
            centroid = get_centroid(
                tracked_object.estimate
                if getattr(tracked_object, "estimate", None) is not None
                else detection.points
            )
            depth_center = map_point_between_frames(
                centroid,
                frame.shape,
                None if depth_frame is None else depth_frame.shape,
            )
            depth_m = compute_mean_depth(
                depth_frame,
                depth_center,
                radius_px=self.depth_window_px,
                max_valid_depth_m=self.max_valid_depth_m,
            )
            body_point_m = self.projector.pixel_depth_to_body(
                float(centroid[0]),
                float(centroid[1]),
                depth_m,
            )
            if body_point_m is None:
                continue

            velocity = self._estimate_velocity(
                track_id=track_id,
                point_m=body_point_m,
                now_s=now_s,
                history=self.prev_body_positions,
            )
            distance_m = float(np.linalg.norm(body_point_m))
            bbox_area_px = float(detection.data.get("bbox_area_px", 0.0))
            confidence = float(detection.data.get("confidence", 0.0))

            msg = TargetTrack()
            msg.stamp = stamp_msg
            msg.track_id = track_id
            msg.x_b_m = float(body_point_m[0])
            msg.y_b_m = float(body_point_m[1])
            msg.z_b_m = float(body_point_m[2])
            msg.vx_b_mps = float(velocity.vector_mps[0])
            msg.vy_b_mps = float(velocity.vector_mps[1])
            msg.vz_b_mps = float(velocity.vector_mps[2])
            msg.distance_m = distance_m
            msg.confidence = confidence
            msg.bbox_area_px = bbox_area_px
            msg.inbound = velocity.inbound
            track_messages.append(msg)
            active_track_ids.add(track_id)

        self._garbage_collect_history(self.prev_body_positions, active_track_ids)
        return track_messages

    def _build_world_tracks(
        self,
        body_tracks: list[TargetTrack],
        stamp_msg,
        now_s: float,
    ) -> list[WorldTargetTrack]:
        if not self.publish_world_tracks:
            return []
        if not self.projector.has_pose():
            if body_tracks:
                self._warn_throttled(
                    "Ownship pose is missing, so world tracks are not being "
                    f"published yet on frame '{self.projector.frame_id}'. "
                    f"Expected pose topic: {self.pose_topic}",
                    attr_name="last_pose_warn_s",
                )
            return []

        world_tracks: list[WorldTargetTrack] = []
        active_track_ids: set[int] = set()
        for body_track in body_tracks:
            body_point_m = np.array(
                [body_track.x_b_m, body_track.y_b_m, body_track.z_b_m],
                dtype=np.float64,
            )
            world_point_m = self.projector.body_to_world(body_point_m)
            if world_point_m is None:
                continue

            velocity = self._estimate_velocity(
                track_id=int(body_track.track_id),
                point_m=world_point_m,
                now_s=now_s,
                history=self.prev_world_positions,
            )

            msg = WorldTargetTrack()
            msg.stamp = stamp_msg
            msg.track_id = int(body_track.track_id)
            msg.x_m = float(world_point_m[0])
            msg.y_m = float(world_point_m[1])
            msg.z_m = float(world_point_m[2])
            msg.vx_mps = float(velocity.vector_mps[0])
            msg.vy_mps = float(velocity.vector_mps[1])
            msg.vz_mps = float(velocity.vector_mps[2])
            msg.distance_m = float(body_track.distance_m)
            msg.confidence = float(body_track.confidence)
            msg.bbox_area_px = float(body_track.bbox_area_px)
            msg.inbound = bool(body_track.inbound)
            world_tracks.append(msg)
            active_track_ids.add(int(body_track.track_id))

        self._garbage_collect_history(self.prev_world_positions, active_track_ids)
        return world_tracks

    def _estimate_velocity(
        self,
        track_id: int,
        point_m: np.ndarray,
        now_s: float,
        history: dict[int, tuple[np.ndarray, float]],
    ) -> TrackVelocity:
        previous = history.get(track_id)
        velocity = np.zeros(3, dtype=np.float64)
        if previous is not None:
            previous_point_m, previous_time_s = previous
            dt_s = now_s - previous_time_s
            if dt_s > 1e-3:
                velocity = (point_m - previous_point_m) / dt_s
        history[track_id] = (point_m.copy(), now_s)

        distance_m = float(np.linalg.norm(point_m))
        radial_velocity = -float(np.dot(point_m, velocity)) / max(distance_m, 1e-3)
        return TrackVelocity(vector_mps=velocity, inbound=radial_velocity > 0.05)

    def _garbage_collect_history(
        self,
        history: dict[int, tuple[np.ndarray, float]],
        active_track_ids: set[int],
    ) -> None:
        stale_ids = [
            track_id for track_id in history if track_id not in active_track_ids
        ]
        for stale_id in stale_ids:
            history.pop(stale_id, None)

    def _warn_throttled(self, message: str, attr_name: str) -> None:
        now_s = self.now_s()
        last_warn_s = float(getattr(self, attr_name, 0.0))
        if now_s - last_warn_s < 1.0:
            return
        setattr(self, attr_name, now_s)
        self.get_logger().warn(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealsenseTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
