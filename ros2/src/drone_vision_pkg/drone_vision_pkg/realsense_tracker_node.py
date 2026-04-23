from __future__ import annotations

import csv
from dataclasses import dataclass
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from drone_control_pkg.deployment_config import (
    configured_compare_pose_topic,
    configured_drone_id,
)
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
from std_msgs.msg import String

from drone_vision_pkg.model_paths import resolve_model_path
from drone_vision_pkg.projection import RealsenseProjection
from drone_vision_pkg.tracker_resilience import (
    BodyTrackState,
    compute_lab_histogram_embedding,
    select_held_track_states,
    tracked_object_reid_distance,
)
from drone_vision_pkg.world_track_compare_common import (
    FRAMES_CSV_FIELDNAMES,
    FRAMES_CSV_FILENAME,
    RUN_MANIFEST_FILENAME,
    TRACKS_CSV_FIELDNAMES,
    TRACKS_CSV_FILENAME_DEFAULT,
    ControllerTrackSample,
    csv_value,
    make_run_id,
    sanitize_experiment_tag,
    select_best_controller_track,
)

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
    from norfair.camera_motion import MotionEstimator
except Exception:  # pragma: no cover - platform specific
    MotionEstimator = None

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


def bbox_to_dict(
    bbox: list[float] | tuple[float, float, float, float] | None,
) -> dict[str, float] | None:
    if bbox is None or len(bbox) != 4:
        return None
    return {
        "x1": float(bbox[0]),
        "y1": float(bbox[1]),
        "x2": float(bbox[2]),
        "y2": float(bbox[3]),
    }


def point_to_dict(
    point: np.ndarray | list[float] | tuple[float, float] | None,
) -> dict[str, float] | None:
    if point is None or len(point) != 2:
        return None
    return {"x": float(point[0]), "y": float(point[1])}


def point3_to_dict(
    point: np.ndarray | list[float] | tuple[float, float, float] | None,
) -> dict[str, float] | None:
    if point is None or len(point) != 3:
        return None
    return {
        "x": float(point[0]),
        "y": float(point[1]),
        "z": float(point[2]),
    }


def isoformat_wall_time(timestamp_s: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(timestamp_s))


def dict_or_empty(value: object) -> dict[str, object]:
    if isinstance(value, dict):
        return value
    return {}


def yolo_boxes_to_detections(
    boxes,
    *,
    frame: np.ndarray | None = None,
    enable_reid: bool = False,
    reid_histogram_bins: int = 32,
) -> list[Detection]:
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
        if enable_reid and frame is not None:
            detections[-1].embedding = compute_lab_histogram_embedding(
                frame,
                [x1, y1, x2, y2],
                bins_per_channel=reid_histogram_bins,
            )
    return detections


class RealsenseTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("realsense_tracker_node")

        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("source_mode", "direct")
        self.declare_parameter("tracker_rate_hz", 10.0)
        self.declare_parameter("device_id", 0)
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_input_size", 640)
        self.declare_parameter("detection_conf_threshold", 0.4)
        self.declare_parameter("norfair_distance_threshold", 80.0)
        self.declare_parameter("track_hit_counter_max", 12)
        self.declare_parameter("enable_reid", False)
        self.declare_parameter("reid_histogram_bins", 32)
        self.declare_parameter("reid_distance_threshold", 0.20)
        self.declare_parameter("reid_hit_counter_max", 30)
        self.declare_parameter("enable_motion_estimator", False)
        self.declare_parameter("publish_track_hold_s", 0.0)
        self.declare_parameter("publish_track_hold_max_extrapolation_m", 0.25)
        self.declare_parameter("max_valid_depth_m", 20.0)
        self.declare_parameter("depth_window_px", 5)
        self.declare_parameter("publish_world_tracks", True)
        self.declare_parameter("publish_world_track_compare", False)
        self.declare_parameter("save_world_track_compare_frames", True)
        self.declare_parameter("save_world_track_compare_csv", True)
        self.declare_parameter("save_world_track_compare_frame_csv", True)
        self.declare_parameter("world_track_compare_frame_dump_interval_s", 3.0)
        self.declare_parameter("world_track_compare_csv_dump_interval_s", 3.0)
        self.declare_parameter("world_track_compare_frame_dump_dir", "output_dump")
        self.declare_parameter(
            "world_track_compare_csv_filename",
            TRACKS_CSV_FILENAME_DEFAULT,
        )
        self.declare_parameter("experiment_tag", "")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("camera_offset_body_m", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_rpy_body_rad", [0.0, 0.0, 0.0])
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("compare_pose_topic", configured_compare_pose_topic())
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("world_tracks_topic", "")
        self.declare_parameter("world_track_compare_topic", "")
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
        self.enable_reid = bool(self.get_parameter("enable_reid").value)
        self.reid_histogram_bins = int(
            self.get_parameter("reid_histogram_bins").value
        )
        self.reid_distance_threshold = float(
            self.get_parameter("reid_distance_threshold").value
        )
        self.reid_hit_counter_max = int(
            self.get_parameter("reid_hit_counter_max").value
        )
        self.enable_motion_estimator = bool(
            self.get_parameter("enable_motion_estimator").value
        )
        self.publish_track_hold_s = float(
            self.get_parameter("publish_track_hold_s").value
        )
        self.publish_track_hold_max_extrapolation_m = float(
            self.get_parameter("publish_track_hold_max_extrapolation_m").value
        )
        self.max_valid_depth_m = float(self.get_parameter("max_valid_depth_m").value)
        self.depth_window_px = int(self.get_parameter("depth_window_px").value)
        self.publish_world_tracks = bool(
            self.get_parameter("publish_world_tracks").value
        )
        self.publish_world_track_compare = bool(
            self.get_parameter("publish_world_track_compare").value
        )
        self.save_world_track_compare_frames = bool(
            self.get_parameter("save_world_track_compare_frames").value
        )
        self.save_world_track_compare_csv = bool(
            self.get_parameter("save_world_track_compare_csv").value
        )
        self.save_world_track_compare_frame_csv = bool(
            self.get_parameter("save_world_track_compare_frame_csv").value
        )
        self.world_track_compare_frame_dump_interval_s = float(
            self.get_parameter("world_track_compare_frame_dump_interval_s").value
        )
        self.world_track_compare_csv_dump_interval_s = float(
            self.get_parameter("world_track_compare_csv_dump_interval_s").value
        )
        self.world_track_compare_root_dir = Path(
            str(
                self.get_parameter("world_track_compare_frame_dump_dir").value
            ).strip()
            or "output_dump"
        )
        self.world_track_compare_tracks_csv_filename = (
            str(self.get_parameter("world_track_compare_csv_filename").value).strip()
            or TRACKS_CSV_FILENAME_DEFAULT
        )
        self.experiment_tag = sanitize_experiment_tag(
            str(self.get_parameter("experiment_tag").value)
        )
        self.run_started_wall_time_s = time.time()
        self.world_track_compare_run_id = make_run_id(
            started_wall_time_s=self.run_started_wall_time_s,
            experiment_tag=self.experiment_tag,
        )
        self.world_track_compare_run_dir = (
            self.world_track_compare_root_dir / self.world_track_compare_run_id
        )
        self.world_track_compare_tracks_csv_path = (
            self.world_track_compare_run_dir
            / str(
                self.world_track_compare_tracks_csv_filename
            )
        )
        self.world_track_compare_frames_csv_path = (
            self.world_track_compare_run_dir / FRAMES_CSV_FILENAME
        )
        self.world_track_compare_manifest_path = (
            self.world_track_compare_run_dir / RUN_MANIFEST_FILENAME
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
        self.compare_pose_topic = (
            str(self.get_parameter("compare_pose_topic").value).strip()
            or configured_compare_pose_topic()
        )
        self.tracks_topic = (
            str(self.get_parameter("tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/tracks")
        )
        self.world_tracks_topic = (
            str(self.get_parameter("world_tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/world_tracks")
        )
        self.world_track_compare_topic = (
            str(self.get_parameter("world_track_compare_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/world_track_compare")
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
        self.tracker = self._build_tracker()
        self.motion_estimator = (
            MotionEstimator()
            if self.enable_motion_estimator and MotionEstimator is not None
            else None
        )

        self.latest_color_msg: Image | None = None
        self.latest_depth_msg: Image | None = None
        self.last_color_stamp_key: tuple[int, int] = (-1, -1)
        self.last_process_s = 0.0
        self.last_inference_ms = 0.0
        self.last_wait_warn_s = 0.0
        self.last_pose_warn_s = 0.0
        self.last_compare_pose_warn_s = 0.0
        self.last_intrinsics_warn_s = 0.0
        self.last_compare_frame_warn_s = 0.0
        self.last_frame_dump_s = 0.0
        self.last_csv_dump_s = 0.0
        self.csv_buffer_started_s = 0.0
        self.world_track_compare_tracks_csv_ready = False
        self.world_track_compare_frames_csv_ready = False
        self.pending_world_track_compare_track_rows: list[list[object]] = []
        self.pending_world_track_compare_frame_rows: list[list[object]] = []
        self.world_track_compare_frame_count = 0
        self.world_track_compare_track_count = 0
        self.prev_body_positions: dict[int, tuple[np.ndarray, float]] = {}
        self.prev_world_positions: dict[int, tuple[np.ndarray, float]] = {}
        self.cached_body_tracks: dict[int, BodyTrackState] = {}
        self.cached_body_track_debug: dict[int, dict[str, object]] = {}
        self.rs_pipeline = None
        self.rs_align = None
        self.depth_scale_m = 0.001
        self.latest_compare_pose: PoseStamped | None = None

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
        if self.publish_world_track_compare:
            self.create_subscription(
                PoseStamped,
                self.compare_pose_topic,
                self.compare_pose_callback,
                best_effort_qos,
            )
            if (
                self.save_world_track_compare_frames
                or self.save_world_track_compare_csv
                or self.save_world_track_compare_frame_csv
            ):
                self._ensure_output_dump_dir()
            if self.save_world_track_compare_csv:
                self._ensure_world_track_compare_tracks_csv_ready()
            if self.save_world_track_compare_frame_csv:
                self._ensure_world_track_compare_frames_csv_ready()
            self._write_world_track_compare_manifest()

        self.tracks_pub = self.create_publisher(TargetTrackArray, self.tracks_topic, 10)
        self.world_tracks_pub = self.create_publisher(
            WorldTargetTrackArray, self.world_tracks_topic, 10
        )
        self.world_track_compare_pub = self.create_publisher(
            String, self.world_track_compare_topic, 10
        )
        self.status_pub = self.create_publisher(
            PerceptionStatus, self.perception_status_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / max(self.tracker_rate_hz, 1.0), self.process_latest_frame
        )
        world_track_compare_enabled = (
            self.publish_world_track_compare
            and (
                self.save_world_track_compare_frames
                or self.save_world_track_compare_csv
                or self.save_world_track_compare_frame_csv
            )
        )
        tracks_csv_log_path = (
            self.world_track_compare_tracks_csv_path
            if self.publish_world_track_compare and self.save_world_track_compare_csv
            else "disabled"
        )
        world_track_compare_topic = (
            self.world_track_compare_topic
            if self.publish_world_track_compare
            else "disabled"
        )
        frames_csv_log_path = (
            self.world_track_compare_frames_csv_path
            if (
                self.publish_world_track_compare
                and self.save_world_track_compare_frame_csv
            )
            else "disabled"
        )

        self.get_logger().info(
            "RealSense tracker started: "
            f"source_mode={self.source_mode}, model={self.model_path}, "
            f"tracks={self.tracks_topic}, "
            f"world_tracks={self.world_tracks_topic}, "
            f"pose={self.pose_topic if self.publish_world_tracks else 'disabled'}, "
            f"reid={'on' if self.enable_reid else 'off'}, "
            f"motion_estimator={'on' if self.motion_estimator is not None else 'off'}, "
            f"track_hold_s={max(self.publish_track_hold_s, 0.0):.2f}, "
            "world_track_compare="
            f"{world_track_compare_topic}, "
            "run_dir="
            f"{self.world_track_compare_run_dir if world_track_compare_enabled else 'disabled'}, "
            "tracks_csv="
            f"{tracks_csv_log_path}, "
            "frames_csv="
            f"{frames_csv_log_path}"
        )
        if self.enable_motion_estimator and self.motion_estimator is None:
            self.get_logger().warn(
                "Motion estimator was enabled, but norfair.camera_motion.MotionEstimator "
                "is unavailable. Continuing without camera motion compensation."
            )

    def _build_tracker(self) -> Tracker:
        tracker_kwargs = {
            "distance_function": "euclidean",
            "distance_threshold": self.norfair_distance_threshold,
            "initialization_delay": 1,
            "hit_counter_max": self.track_hit_counter_max,
        }
        if self.enable_reid:
            tracker_kwargs.update(
                {
                    "reid_distance_function": tracked_object_reid_distance,
                    "reid_distance_threshold": self.reid_distance_threshold,
                    "reid_hit_counter_max": self.reid_hit_counter_max,
                }
            )
        try:
            return Tracker(**tracker_kwargs)
        except TypeError:
            tracker_kwargs.pop("reid_distance_function", None)
            tracker_kwargs.pop("reid_distance_threshold", None)
            tracker_kwargs.pop("reid_hit_counter_max", None)
            if self.enable_reid:
                self.get_logger().warn(
                    "Installed Norfair build does not support Re-ID arguments. "
                    "Continuing with the base tracker only."
                )
            return Tracker(**tracker_kwargs)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self) -> bool:
        self._flush_world_track_compare_csv(force=True)
        self._write_world_track_compare_manifest(completed=True)
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

    def compare_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_compare_pose = msg

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

        detections = yolo_boxes_to_detections(
            results[0].boxes if results else None,
            frame=frame,
            enable_reid=self.enable_reid,
            reid_histogram_bins=self.reid_histogram_bins,
        )
        coord_transformations = None
        if self.motion_estimator is not None:
            coord_transformations = self.motion_estimator.update(frame)
        if coord_transformations is not None:
            tracked_objects = self.tracker.update(
                detections,
                coord_transformations=coord_transformations,
            )
        else:
            tracked_objects = self.tracker.update(detections)
        now_msg = self.get_clock().now().to_msg()
        process_time_s = self.now_s()
        tracker_fps = 0.0
        if self.last_process_s > 0.0:
            tracker_fps = 1.0 / max(process_time_s - self.last_process_s, 1e-3)
        self.last_process_s = process_time_s

        body_tracks, body_track_debug = self._build_body_tracks(
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
        compare_payload = self._publish_world_track_compare(
            world_tracks=world_tracks,
            body_tracks=body_tracks,
            stamp_msg=now_msg,
            body_track_debug=body_track_debug,
            detections_count=len(detections),
            active_tracks_count=len(tracked_objects),
            tracker_fps=tracker_fps,
        )
        self._maybe_dump_world_track_compare_frame(
            frame=frame,
            compare_payload=compare_payload,
            now_s=process_time_s,
        )
        self._maybe_append_world_track_compare_tracks_csv(compare_payload)
        self._maybe_append_world_track_compare_frames_csv(compare_payload)

        status = PerceptionStatus()
        status.stamp = now_msg
        status.drone_id = self.drone_id
        status.tracker_fps = float(tracker_fps)
        status.inference_latency_ms = float(self.last_inference_ms)
        status.left_detections = int(len(detections))
        status.right_detections = 0
        status.paired_detections = 0
        status.active_tracks = int(len(body_tracks))
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
    ) -> tuple[list[TargetTrack], dict[int, dict[str, object]]]:
        if not self.projector.has_intrinsics():
            self._warn_throttled(
                "Camera intrinsics have not arrived yet. Body/world tracks are "
                f"waiting on {self.camera_info_topic}.",
                attr_name="last_intrinsics_warn_s",
            )
            return [], {}

        track_messages: list[TargetTrack] = []
        track_debug: dict[int, dict[str, object]] = {}
        norfair_active_track_ids: set[int] = set()

        for tracked_object in tracked_objects:
            raw_track_id = getattr(tracked_object, "id", None)
            if raw_track_id is None:
                continue
            track_id = int(raw_track_id)
            norfair_active_track_ids.add(track_id)
            detection = tracked_object.last_detection
            if detection is None:
                continue
            yolo_centroid = np.asarray(detection.points, dtype=np.float32)
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
            yolo_bbox_raw = detection.data.get("bbox")
            yolo_bbox = (
                [float(value) for value in yolo_bbox_raw]
                if isinstance(yolo_bbox_raw, list) and len(yolo_bbox_raw) == 4
                else None
            )
            norfair_bbox = None
            if yolo_bbox is not None:
                bbox_width_px = yolo_bbox[2] - yolo_bbox[0]
                bbox_height_px = yolo_bbox[3] - yolo_bbox[1]
                norfair_bbox = [
                    float(centroid[0]) - bbox_width_px / 2.0,
                    float(centroid[1]) - bbox_height_px / 2.0,
                    float(centroid[0]) + bbox_width_px / 2.0,
                    float(centroid[1]) + bbox_height_px / 2.0,
                ]

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
            track_debug[track_id] = {
                "body_position_m": point3_to_dict(body_point_m),
                "body_velocity_mps": point3_to_dict(velocity.vector_mps),
                "yolo_bbox_px": bbox_to_dict(yolo_bbox),
                "norfair_bbox_px": bbox_to_dict(norfair_bbox),
                "yolo_centroid_px": point_to_dict(yolo_centroid),
                "norfair_centroid_px": point_to_dict(centroid),
                "depth_z_m": float(depth_m),
                "held_track": False,
                "held_track_age_s": 0.0,
            }

        self._cache_fresh_body_tracks(track_messages, track_debug, now_s=now_s)
        track_messages, track_debug = self._append_held_body_tracks(
            body_tracks=track_messages,
            body_track_debug=track_debug,
            active_track_ids=norfair_active_track_ids,
            stamp_msg=stamp_msg,
            now_s=now_s,
        )
        self._garbage_collect_history(self.prev_body_positions, norfair_active_track_ids)
        return track_messages, track_debug

    def _message_to_body_track_state(
        self,
        track: TargetTrack,
        *,
        now_s: float,
    ) -> BodyTrackState:
        return BodyTrackState(
            track_id=int(track.track_id),
            x_b_m=float(track.x_b_m),
            y_b_m=float(track.y_b_m),
            z_b_m=float(track.z_b_m),
            vx_b_mps=float(track.vx_b_mps),
            vy_b_mps=float(track.vy_b_mps),
            vz_b_mps=float(track.vz_b_mps),
            confidence=float(track.confidence),
            bbox_area_px=float(track.bbox_area_px),
            inbound=bool(track.inbound),
            last_seen_s=float(now_s),
        )

    def _body_track_state_to_msg(self, state: BodyTrackState, stamp_msg) -> TargetTrack:
        msg = TargetTrack()
        msg.stamp = stamp_msg
        msg.track_id = int(state.track_id)
        msg.x_b_m = float(state.x_b_m)
        msg.y_b_m = float(state.y_b_m)
        msg.z_b_m = float(state.z_b_m)
        msg.vx_b_mps = float(state.vx_b_mps)
        msg.vy_b_mps = float(state.vy_b_mps)
        msg.vz_b_mps = float(state.vz_b_mps)
        msg.distance_m = float(
            np.linalg.norm(np.array([state.x_b_m, state.y_b_m, state.z_b_m]))
        )
        msg.confidence = float(state.confidence)
        msg.bbox_area_px = float(state.bbox_area_px)
        msg.inbound = bool(state.inbound)
        return msg

    def _cache_fresh_body_tracks(
        self,
        body_tracks: list[TargetTrack],
        body_track_debug: dict[int, dict[str, object]],
        *,
        now_s: float,
    ) -> None:
        fresh_track_ids: set[int] = set()
        for body_track in body_tracks:
            track_id = int(body_track.track_id)
            fresh_track_ids.add(track_id)
            self.cached_body_tracks[track_id] = self._message_to_body_track_state(
                body_track,
                now_s=now_s,
            )
            self.cached_body_track_debug[track_id] = dict(
                body_track_debug.get(track_id, {})
            )

        cache_keep_window_s = max(self.publish_track_hold_s, 0.0) + 1.0
        for track_id in list(self.cached_body_tracks.keys()):
            if track_id in fresh_track_ids:
                continue
            track_age_s = max(
                float(now_s) - float(self.cached_body_tracks[track_id].last_seen_s),
                0.0,
            )
            if track_age_s > cache_keep_window_s:
                self.cached_body_tracks.pop(track_id, None)
                self.cached_body_track_debug.pop(track_id, None)

    def _append_held_body_tracks(
        self,
        *,
        body_tracks: list[TargetTrack],
        body_track_debug: dict[int, dict[str, object]],
        active_track_ids: set[int],
        stamp_msg,
        now_s: float,
    ) -> tuple[list[TargetTrack], dict[int, dict[str, object]]]:
        held_tracks = select_held_track_states(
            active_track_ids=active_track_ids,
            fresh_track_ids={int(track.track_id) for track in body_tracks},
            cached_tracks=self.cached_body_tracks,
            now_s=now_s,
            hold_window_s=self.publish_track_hold_s,
            max_extrapolation_m=self.publish_track_hold_max_extrapolation_m,
        )
        for track_id, held_state in held_tracks.items():
            body_tracks.append(self._body_track_state_to_msg(held_state, stamp_msg))
            held_age_s = max(
                float(now_s) - float(self.cached_body_tracks[track_id].last_seen_s),
                0.0,
            )
            held_debug = dict(self.cached_body_track_debug.get(track_id, {}))
            held_debug["held_track"] = True
            held_debug["held_track_age_s"] = float(held_age_s)
            body_track_debug[track_id] = held_debug
        return body_tracks, body_track_debug

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

    def _ensure_output_dump_dir(self) -> None:
        try:
            self.world_track_compare_root_dir.mkdir(parents=True, exist_ok=True)
            self.world_track_compare_run_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.save_world_track_compare_frames = False
            self.save_world_track_compare_csv = False
            self.save_world_track_compare_frame_csv = False
            self.get_logger().warn(
                "Disabling world-track compare outputs because the output "
                f"directory could not be created: {exc}"
            )

    def _ensure_world_track_compare_csv_ready(
        self,
        *,
        path: Path,
        header: list[str],
        ready_attr: str,
        enabled_attr: str,
        log_label: str,
    ) -> bool:
        if not bool(getattr(self, enabled_attr, False)):
            return False
        if bool(getattr(self, ready_attr, False)):
            return True
        try:
            file_exists = path.exists()
            with path.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                if not file_exists or path.stat().st_size == 0:
                    writer.writerow(header)
            setattr(self, ready_attr, True)
            return True
        except OSError as exc:
            setattr(self, enabled_attr, False)
            self.get_logger().warn(
                f"Disabling {log_label} because the file "
                f"could not be prepared: {exc}"
            )
            return False

    def _ensure_world_track_compare_tracks_csv_ready(self) -> bool:
        return self._ensure_world_track_compare_csv_ready(
            path=self.world_track_compare_tracks_csv_path,
            header=TRACKS_CSV_FIELDNAMES,
            ready_attr="world_track_compare_tracks_csv_ready",
            enabled_attr="save_world_track_compare_csv",
            log_label="world-track compare track CSV logging",
        )

    def _ensure_world_track_compare_frames_csv_ready(self) -> bool:
        return self._ensure_world_track_compare_csv_ready(
            path=self.world_track_compare_frames_csv_path,
            header=FRAMES_CSV_FIELDNAMES,
            ready_attr="world_track_compare_frames_csv_ready",
            enabled_attr="save_world_track_compare_frame_csv",
            log_label="world-track compare frame CSV logging",
        )

    def _write_world_track_compare_manifest(self, completed: bool = False) -> None:
        if not self.publish_world_track_compare:
            return
        if (
            not self.save_world_track_compare_frames
            and not self.save_world_track_compare_csv
            and not self.save_world_track_compare_frame_csv
        ):
            return
        if not self.world_track_compare_run_dir.exists():
            return

        completed_at = time.time() if completed else None
        manifest = {
            "run_id": self.world_track_compare_run_id,
            "experiment_tag": self.experiment_tag,
            "started_at_wall": isoformat_wall_time(self.run_started_wall_time_s),
            "completed_at_wall": (
                isoformat_wall_time(completed_at)
                if completed_at is not None
                else None
            ),
            "output_root_dir": str(self.world_track_compare_root_dir),
            "run_dir": str(self.world_track_compare_run_dir),
            "files": {
                "manifest": str(self.world_track_compare_manifest_path),
                "tracks_csv": (
                    str(self.world_track_compare_tracks_csv_path)
                    if self.save_world_track_compare_csv
                    else None
                ),
                "frames_csv": (
                    str(self.world_track_compare_frames_csv_path)
                    if self.save_world_track_compare_frame_csv
                    else None
                ),
                "frame_glob": (
                    str(self.world_track_compare_run_dir / "world_track_compare_*.jpg")
                    if self.save_world_track_compare_frames
                    else None
                ),
            },
            "tracker": {
                "source_mode": self.source_mode,
                "model_path": str(self.model_path),
                "tracker_rate_hz": float(self.tracker_rate_hz),
                "model_input_size": int(self.model_input_size),
                "detection_conf_threshold": float(self.conf_threshold),
                "norfair_distance_threshold": float(
                    self.norfair_distance_threshold
                ),
                "track_hit_counter_max": int(self.track_hit_counter_max),
                "enable_reid": bool(self.enable_reid),
                "reid_histogram_bins": int(self.reid_histogram_bins),
                "reid_distance_threshold": float(self.reid_distance_threshold),
                "reid_hit_counter_max": int(self.reid_hit_counter_max),
                "enable_motion_estimator": bool(self.enable_motion_estimator),
                "publish_track_hold_s": float(self.publish_track_hold_s),
                "publish_track_hold_max_extrapolation_m": float(
                    self.publish_track_hold_max_extrapolation_m
                ),
                "max_valid_depth_m": float(self.max_valid_depth_m),
                "depth_window_px": int(self.depth_window_px),
                "world_frame": self.projector.frame_id,
                "color_topic": self.color_topic,
                "depth_topic": self.depth_topic,
                "camera_info_topic": self.camera_info_topic,
                "pose_topic": self.pose_topic,
                "compare_pose_topic": self.compare_pose_topic,
                "tracks_topic": self.tracks_topic,
                "world_tracks_topic": self.world_tracks_topic,
                "world_track_compare_topic": self.world_track_compare_topic,
                "perception_status_topic": self.perception_status_topic,
            },
            "camera": {
                "align_depth_to_color": bool(self.align_depth_to_color),
                "direct_color_width": int(self.direct_color_width),
                "direct_color_height": int(self.direct_color_height),
                "direct_color_fps": int(self.direct_color_fps),
                "direct_depth_width": int(self.direct_depth_width),
                "direct_depth_height": int(self.direct_depth_height),
                "direct_depth_fps": int(self.direct_depth_fps),
                "depth_scale_m": float(self.depth_scale_m),
            },
            "counts": {
                "frame_rows": int(self.world_track_compare_frame_count),
                "track_rows": int(self.world_track_compare_track_count),
            },
        }
        try:
            with self.world_track_compare_manifest_path.open(
                "w",
                encoding="utf-8",
            ) as handle:
                json.dump(manifest, handle, indent=2, sort_keys=True)
        except OSError as exc:
            self.get_logger().warn(
                f"Failed to write world-track compare manifest: {exc}"
            )

    def _controller_track_sample(
        self, track: TargetTrack
    ) -> ControllerTrackSample:
        return ControllerTrackSample(
            track_id=int(track.track_id),
            x_b_m=float(track.x_b_m),
            y_b_m=float(track.y_b_m),
            z_b_m=float(track.z_b_m),
            vx_b_mps=float(track.vx_b_mps),
            vy_b_mps=float(track.vy_b_mps),
            vz_b_mps=float(track.vz_b_mps),
            distance_m=float(track.distance_m),
            confidence=float(track.confidence),
            bbox_area_px=float(track.bbox_area_px),
            inbound=bool(track.inbound),
        )

    def _publish_world_track_compare(
        self,
        world_tracks: list[WorldTargetTrack],
        body_tracks: list[TargetTrack],
        stamp_msg,
        body_track_debug: dict[int, dict[str, object]],
        detections_count: int,
        active_tracks_count: int,
        tracker_fps: float,
    ) -> dict[str, object] | None:
        if not self.publish_world_track_compare:
            return None

        compare_pose = self.latest_compare_pose
        ownship_pose_available = self.projector.has_pose()
        best_track, best_track_score = select_best_controller_track(
            [self._controller_track_sample(track) for track in body_tracks]
        )
        payload: dict[str, object] = {
            "run_id": self.world_track_compare_run_id,
            "experiment_tag": self.experiment_tag,
            "stamp": {
                "sec": int(stamp_msg.sec),
                "nanosec": int(stamp_msg.nanosec),
            },
            "world_frame_id": self.projector.frame_id,
            "reference_pose_topic": self.compare_pose_topic,
            "reference_pose_available": compare_pose is not None,
            "ownship_pose_available": ownship_pose_available,
            "frame_match": None,
            "truth_range_m": None,
            "truth_depth_z_m": None,
            "detections_count": int(detections_count),
            "active_tracks_count": int(active_tracks_count),
            "world_tracks_count": int(len(world_tracks)),
            "tracker_fps": float(tracker_fps),
            "inference_latency_ms": float(self.last_inference_ms),
            "has_controller_valid_track": best_track is not None,
            "best_track_id": None if best_track is None else int(best_track.track_id),
            "best_track_score": (
                None if best_track_score is None else float(best_track_score)
            ),
            "best_track_confidence": (
                None if best_track is None else float(best_track.confidence)
            ),
            "best_track_distance_m": (
                None if best_track is None else float(best_track.distance_m)
            ),
            "comparisons": [],
        }

        if compare_pose is None:
            if world_tracks:
                self._warn_throttled(
                    "World-track comparison is enabled but no reference pose has "
                    f"arrived yet on {self.compare_pose_topic}.",
                    attr_name="last_compare_pose_warn_s",
                )
            msg = String()
            msg.data = json.dumps(payload)
            self.world_track_compare_pub.publish(msg)
            return payload

        reference_frame_id = (
            str(compare_pose.header.frame_id).strip() or self.projector.frame_id
        )
        reference_position = np.array(
            [
                float(compare_pose.pose.position.x),
                float(compare_pose.pose.position.y),
                float(compare_pose.pose.position.z),
            ],
            dtype=np.float64,
        )
        payload["reference_frame_id"] = reference_frame_id
        payload["reference_position_m"] = {
            "x": float(reference_position[0]),
            "y": float(reference_position[1]),
            "z": float(reference_position[2]),
        }
        payload["frame_match"] = reference_frame_id == self.projector.frame_id
        reference_body_position = None
        reference_camera_position = None
        if payload["frame_match"] and ownship_pose_available:
            reference_body_position = self.projector.world_to_body(reference_position)
            reference_camera_position = self.projector.world_to_camera(
                reference_position
            )
        payload["truth_body_position_m"] = point3_to_dict(reference_body_position)
        if reference_camera_position is not None:
            payload["truth_range_m"] = float(
                np.linalg.norm(reference_camera_position)
            )
            payload["truth_depth_z_m"] = float(reference_camera_position[2])

        if reference_frame_id != self.projector.frame_id and world_tracks:
            self._warn_throttled(
                "World-track comparison is using different frame_ids: "
                f"world_tracks='{self.projector.frame_id}' vs "
                f"reference='{reference_frame_id}'.",
                attr_name="last_compare_frame_warn_s",
            )

        comparisons: list[dict[str, object]] = []
        for world_track in world_tracks:
            debug_info = body_track_debug.get(int(world_track.track_id), {})
            body_position = debug_info.get("body_position_m")
            delta = np.array(
                [
                    float(world_track.x_m) - reference_position[0],
                    float(world_track.y_m) - reference_position[1],
                    float(world_track.z_m) - reference_position[2],
                ],
                dtype=np.float64,
            )
            body_delta = None
            if (
                isinstance(body_position, dict)
                and reference_body_position is not None
            ):
                body_delta = {
                    "x": float(body_position.get("x", 0.0))
                    - float(reference_body_position[0]),
                    "y": float(body_position.get("y", 0.0))
                    - float(reference_body_position[1]),
                    "z": float(body_position.get("z", 0.0))
                    - float(reference_body_position[2]),
                }
            comparisons.append(
                {
                    "track_id": int(world_track.track_id),
                    "track_position_m": {
                        "x": float(world_track.x_m),
                        "y": float(world_track.y_m),
                        "z": float(world_track.z_m),
                    },
                    "delta_m": {
                        "x": float(delta[0]),
                        "y": float(delta[1]),
                        "z": float(delta[2]),
                    },
                    "delta_norm_m": float(np.linalg.norm(delta)),
                    "body_position_m": body_position,
                    "truth_body_position_m": point3_to_dict(reference_body_position),
                    "body_delta_m": body_delta,
                    "truth_range_m": payload.get("truth_range_m"),
                    "truth_depth_z_m": payload.get("truth_depth_z_m"),
                    "confidence": float(world_track.confidence),
                    "distance_m": float(world_track.distance_m),
                    "bbox_area_px": float(world_track.bbox_area_px),
                    "yolo_bbox_px": debug_info.get("yolo_bbox_px"),
                    "norfair_bbox_px": debug_info.get("norfair_bbox_px"),
                    "yolo_centroid_px": debug_info.get("yolo_centroid_px"),
                    "norfair_centroid_px": debug_info.get("norfair_centroid_px"),
                    "depth_z_m": float(debug_info.get("depth_z_m", 0.0)),
                }
            )
        payload["comparisons"] = comparisons

        msg = String()
        msg.data = json.dumps(payload)
        self.world_track_compare_pub.publish(msg)
        return payload

    def _maybe_dump_world_track_compare_frame(
        self,
        frame: np.ndarray,
        compare_payload: dict[str, object] | None,
        now_s: float,
    ) -> None:
        if (
            not self.publish_world_track_compare
            or not self.save_world_track_compare_frames
            or compare_payload is None
        ):
            return
        if (
            now_s - self.last_frame_dump_s
            < self.world_track_compare_frame_dump_interval_s
        ):
            return

        annotated = frame.copy()
        comparisons = compare_payload.get("comparisons", [])
        if isinstance(comparisons, list):
            for comparison in comparisons:
                if not isinstance(comparison, dict):
                    continue
                track_id = int(comparison.get("track_id", -1))
                yolo_bbox = comparison.get("yolo_bbox_px")
                if isinstance(yolo_bbox, dict):
                    x1 = int(round(float(yolo_bbox.get("x1", 0.0))))
                    y1 = int(round(float(yolo_bbox.get("y1", 0.0))))
                    x2 = int(round(float(yolo_bbox.get("x2", 0.0))))
                    y2 = int(round(float(yolo_bbox.get("y2", 0.0))))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        annotated,
                        f"YOLO {track_id}",
                        (x1, max(18, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

                norfair_bbox = comparison.get("norfair_bbox_px")
                if isinstance(norfair_bbox, dict):
                    x1 = int(round(float(norfair_bbox.get("x1", 0.0))))
                    y1 = int(round(float(norfair_bbox.get("y1", 0.0))))
                    x2 = int(round(float(norfair_bbox.get("x2", 0.0))))
                    y2 = int(round(float(norfair_bbox.get("y2", 0.0))))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(
                        annotated,
                        f"Norfair {track_id}",
                        (x1, min(annotated.shape[0] - 10, y2 + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                for point_key, color in (
                    ("yolo_centroid_px", (0, 255, 0)),
                    ("norfair_centroid_px", (0, 255, 255)),
                ):
                    point = comparison.get(point_key)
                    if not isinstance(point, dict):
                        continue
                    center = (
                        int(round(float(point.get("x", 0.0)))),
                        int(round(float(point.get("y", 0.0)))),
                    )
                    cv2.circle(annotated, center, 4, color, thickness=-1)

        overlay_lines = self._world_track_compare_overlay_lines(compare_payload)
        line_height_px = 26
        margin_px = 12
        overlay_height_px = margin_px * 2 + max(1, len(overlay_lines)) * line_height_px
        overlay_width_px = min(annotated.shape[1] - margin_px * 2, 720)
        cv2.rectangle(
            annotated,
            (margin_px, margin_px),
            (margin_px + overlay_width_px, margin_px + overlay_height_px),
            (0, 0, 0),
            thickness=-1,
        )
        cv2.rectangle(
            annotated,
            (margin_px, margin_px),
            (margin_px + overlay_width_px, margin_px + overlay_height_px),
            (0, 255, 255),
            thickness=2,
        )
        y_px = margin_px + 22
        for line in overlay_lines:
            cv2.putText(
                annotated,
                line,
                (margin_px + 10, y_px),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            y_px += line_height_px

        stamp = compare_payload.get("stamp", {})
        sec = int(stamp.get("sec", 0)) if isinstance(stamp, dict) else 0
        nanosec = int(stamp.get("nanosec", 0)) if isinstance(stamp, dict) else 0
        filename = f"world_track_compare_{sec}_{nanosec:09d}.jpg"
        output_path = self.world_track_compare_run_dir / filename
        if cv2.imwrite(str(output_path), annotated):
            self.last_frame_dump_s = now_s
        else:
            self.get_logger().warn(
                f"Failed to save world-track comparison frame to {output_path}"
            )

    def _track_compare_rows_from_payload(
        self, compare_payload: dict[str, object] | None
    ) -> list[list[object]]:
        if compare_payload is None:
            return []

        stamp = compare_payload.get("stamp", {})
        if not isinstance(stamp, dict):
            return []
        stamp_sec = int(stamp.get("sec", 0))
        stamp_nanosec = int(stamp.get("nanosec", 0))
        timestamp_s = float(stamp_sec) + float(stamp_nanosec) / 1e9

        run_id = str(compare_payload.get("run_id", ""))
        experiment_tag = str(compare_payload.get("experiment_tag", ""))
        world_frame_id = str(compare_payload.get("world_frame_id", ""))
        reference_frame_id = str(compare_payload.get("reference_frame_id", ""))
        reference_position = compare_payload.get("reference_position_m", {})
        reference_body_position = compare_payload.get("truth_body_position_m", {})
        reference_available = bool(
            compare_payload.get("reference_pose_available", False)
        )
        ownship_pose_available = bool(
            compare_payload.get("ownship_pose_available", False)
        )
        frame_match = compare_payload.get("frame_match")
        world_tracks_count = int(compare_payload.get("world_tracks_count", 0))
        tracker_fps = float(compare_payload.get("tracker_fps", 0.0))
        inference_latency_ms = float(
            compare_payload.get("inference_latency_ms", 0.0)
        )
        truth_range_m = compare_payload.get("truth_range_m")
        truth_depth_z_m = compare_payload.get("truth_depth_z_m")

        comparisons = compare_payload.get("comparisons", [])
        if not isinstance(comparisons, list) or not comparisons:
            return []

        rows: list[list[object]] = []
        for comparison in comparisons:
            if not isinstance(comparison, dict):
                continue
            track_position = dict_or_empty(comparison.get("track_position_m"))
            delta = dict_or_empty(comparison.get("delta_m"))
            yolo_bbox = dict_or_empty(comparison.get("yolo_bbox_px"))
            norfair_bbox = dict_or_empty(comparison.get("norfair_bbox_px"))
            yolo_centroid = dict_or_empty(comparison.get("yolo_centroid_px"))
            norfair_centroid = dict_or_empty(
                comparison.get("norfair_centroid_px")
            )
            body_position = dict_or_empty(comparison.get("body_position_m"))
            body_delta = dict_or_empty(comparison.get("body_delta_m"))
            rows.append(
                [
                    run_id,
                    experiment_tag,
                    stamp_sec,
                    stamp_nanosec,
                    f"{timestamp_s:.9f}",
                    int(comparison.get("track_id", -1)),
                    world_frame_id,
                    reference_frame_id,
                    int(reference_available),
                    int(ownship_pose_available),
                    csv_value(None if frame_match is None else int(bool(frame_match))),
                    world_tracks_count,
                    tracker_fps,
                    inference_latency_ms,
                    float(track_position.get("x", 0.0)),
                    float(track_position.get("y", 0.0)),
                    float(track_position.get("z", 0.0)),
                    float(reference_position.get("x", 0.0))
                    if isinstance(reference_position, dict)
                    else "",
                    float(reference_position.get("y", 0.0))
                    if isinstance(reference_position, dict)
                    else "",
                    float(reference_position.get("z", 0.0))
                    if isinstance(reference_position, dict)
                    else "",
                    float(delta.get("x", 0.0)),
                    float(delta.get("y", 0.0)),
                    float(delta.get("z", 0.0)),
                    float(comparison.get("delta_norm_m", 0.0)),
                    float(body_position.get("x", 0.0)),
                    float(body_position.get("y", 0.0)),
                    float(body_position.get("z", 0.0)),
                    float(reference_body_position.get("x", 0.0))
                    if isinstance(reference_body_position, dict)
                    else "",
                    float(reference_body_position.get("y", 0.0))
                    if isinstance(reference_body_position, dict)
                    else "",
                    float(reference_body_position.get("z", 0.0))
                    if isinstance(reference_body_position, dict)
                    else "",
                    csv_value(
                        float(body_delta.get("x", 0.0))
                        if isinstance(body_delta, dict)
                        else None
                    ),
                    csv_value(
                        float(body_delta.get("y", 0.0))
                        if isinstance(body_delta, dict)
                        else None
                    ),
                    csv_value(
                        float(body_delta.get("z", 0.0))
                        if isinstance(body_delta, dict)
                        else None
                    ),
                    csv_value(
                        float(truth_range_m) if truth_range_m is not None else None
                    ),
                    csv_value(
                        float(truth_depth_z_m)
                        if truth_depth_z_m is not None
                        else None
                    ),
                    float(comparison.get("confidence", 0.0)),
                    float(comparison.get("distance_m", 0.0)),
                    float(comparison.get("bbox_area_px", 0.0)),
                    float(yolo_bbox.get("x1", 0.0)),
                    float(yolo_bbox.get("y1", 0.0)),
                    float(yolo_bbox.get("x2", 0.0)),
                    float(yolo_bbox.get("y2", 0.0)),
                    float(norfair_bbox.get("x1", 0.0)),
                    float(norfair_bbox.get("y1", 0.0)),
                    float(norfair_bbox.get("x2", 0.0)),
                    float(norfair_bbox.get("y2", 0.0)),
                    float(yolo_centroid.get("x", 0.0)),
                    float(yolo_centroid.get("y", 0.0)),
                    float(norfair_centroid.get("x", 0.0)),
                    float(norfair_centroid.get("y", 0.0)),
                    float(comparison.get("depth_z_m", 0.0)),
                ]
            )
        return rows

    def _frame_compare_row_from_payload(
        self, compare_payload: dict[str, object] | None
    ) -> list[object] | None:
        if compare_payload is None:
            return None

        stamp = compare_payload.get("stamp", {})
        if not isinstance(stamp, dict):
            return None
        stamp_sec = int(stamp.get("sec", 0))
        stamp_nanosec = int(stamp.get("nanosec", 0))
        timestamp_s = float(stamp_sec) + float(stamp_nanosec) / 1e9
        frame_match = compare_payload.get("frame_match")
        return [
            str(compare_payload.get("run_id", "")),
            str(compare_payload.get("experiment_tag", "")),
            stamp_sec,
            stamp_nanosec,
            f"{timestamp_s:.9f}",
            str(compare_payload.get("world_frame_id", "")),
            str(compare_payload.get("reference_frame_id", "")),
            int(bool(compare_payload.get("reference_pose_available", False))),
            int(bool(compare_payload.get("ownship_pose_available", False))),
            csv_value(None if frame_match is None else int(bool(frame_match))),
            csv_value(compare_payload.get("truth_range_m")),
            csv_value(compare_payload.get("truth_depth_z_m")),
            int(compare_payload.get("detections_count", 0)),
            int(compare_payload.get("active_tracks_count", 0)),
            int(compare_payload.get("world_tracks_count", 0)),
            int(bool(compare_payload.get("has_controller_valid_track", False))),
            csv_value(compare_payload.get("best_track_id")),
            csv_value(compare_payload.get("best_track_score")),
            csv_value(compare_payload.get("best_track_confidence")),
            csv_value(compare_payload.get("best_track_distance_m")),
            float(compare_payload.get("tracker_fps", 0.0)),
            float(compare_payload.get("inference_latency_ms", 0.0)),
        ]

    def _maybe_append_world_track_compare_tracks_csv(
        self, compare_payload: dict[str, object] | None
    ) -> None:
        if not self.save_world_track_compare_csv or compare_payload is None:
            return
        rows = self._track_compare_rows_from_payload(compare_payload)
        if not rows:
            return
        self.pending_world_track_compare_track_rows.extend(rows)
        self.world_track_compare_track_count += len(rows)
        now_s = self.now_s()
        if self.csv_buffer_started_s <= 0.0:
            self.csv_buffer_started_s = now_s
        self._flush_world_track_compare_csv(now_s=now_s)

    def _maybe_append_world_track_compare_frames_csv(
        self, compare_payload: dict[str, object] | None
    ) -> None:
        if not self.save_world_track_compare_frame_csv or compare_payload is None:
            return
        row = self._frame_compare_row_from_payload(compare_payload)
        if row is None:
            return
        self.pending_world_track_compare_frame_rows.append(row)
        self.world_track_compare_frame_count += 1
        now_s = self.now_s()
        if self.csv_buffer_started_s <= 0.0:
            self.csv_buffer_started_s = now_s
        self._flush_world_track_compare_csv(now_s=now_s)

    def _flush_world_track_compare_csv(
        self,
        *,
        now_s: float | None = None,
        force: bool = False,
    ) -> None:
        if not self.save_world_track_compare_csv:
            self.pending_world_track_compare_track_rows.clear()
        if not self.save_world_track_compare_frame_csv:
            self.pending_world_track_compare_frame_rows.clear()
        if (
            not self.pending_world_track_compare_track_rows
            and not self.pending_world_track_compare_frame_rows
        ):
            if force:
                self.csv_buffer_started_s = 0.0
            return

        if not force:
            current_time_s = self.now_s() if now_s is None else now_s
            if self.csv_buffer_started_s <= 0.0:
                self.csv_buffer_started_s = current_time_s
                return
            if (
                    current_time_s - self.csv_buffer_started_s
                < self.world_track_compare_csv_dump_interval_s
            ):
                return
        else:
            current_time_s = self.now_s() if now_s is None else now_s

        wrote_rows = False
        if (
            self.pending_world_track_compare_track_rows
            and self._ensure_world_track_compare_tracks_csv_ready()
        ):
            try:
                with self.world_track_compare_tracks_csv_path.open(
                    "a",
                    newline="",
                    encoding="utf-8",
                ) as handle:
                    writer = csv.writer(handle)
                    writer.writerows(self.pending_world_track_compare_track_rows)
                self.pending_world_track_compare_track_rows.clear()
                wrote_rows = True
            except OSError as exc:
                self.save_world_track_compare_csv = False
                self.get_logger().warn(
                    "Disabling world-track compare track CSV logging because "
                    f"appending failed: {exc}"
                )
        if (
            self.pending_world_track_compare_frame_rows
            and self._ensure_world_track_compare_frames_csv_ready()
        ):
            try:
                with self.world_track_compare_frames_csv_path.open(
                    "a",
                    newline="",
                    encoding="utf-8",
                ) as handle:
                    writer = csv.writer(handle)
                    writer.writerows(self.pending_world_track_compare_frame_rows)
                self.pending_world_track_compare_frame_rows.clear()
                wrote_rows = True
            except OSError as exc:
                self.save_world_track_compare_frame_csv = False
                self.get_logger().warn(
                    "Disabling world-track compare frame CSV logging because "
                    f"appending failed: {exc}"
                )
        if wrote_rows:
            self.last_csv_dump_s = current_time_s
        if (
            not self.pending_world_track_compare_track_rows
            and not self.pending_world_track_compare_frame_rows
        ):
            self.csv_buffer_started_s = 0.0

    def _world_track_compare_overlay_lines(
        self, compare_payload: dict[str, object]
    ) -> list[str]:
        lines = [
            "world_track_compare  "
            f"run={compare_payload.get('run_id', 'unknown')}  "
            f"frame={compare_payload.get('world_frame_id', 'map')}",
        ]
        reference_available = bool(
            compare_payload.get("reference_pose_available", False)
        )
        if not reference_available:
            lines.append("reference pose unavailable")
            return lines

        reference_position = compare_payload.get("reference_position_m", {})
        if isinstance(reference_position, dict):
            lines.append(
                "RigidBody2 ref "
                f"x={float(reference_position.get('x', 0.0)):.3f} "
                f"y={float(reference_position.get('y', 0.0)):.3f} "
                f"z={float(reference_position.get('z', 0.0)):.3f}"
            )
        if compare_payload.get("truth_range_m") is not None:
            lines.append(
                "truth "
                f"range={float(compare_payload.get('truth_range_m', 0.0)):.2f}m  "
                f"depth_z={float(compare_payload.get('truth_depth_z_m', 0.0)):.2f}m"
            )
        lines.append(
            "counts "
            f"det={int(compare_payload.get('detections_count', 0))}  "
            f"trk={int(compare_payload.get('active_tracks_count', 0))}  "
            f"world={int(compare_payload.get('world_tracks_count', 0))}  "
            "ctrl_valid="
            f"{int(bool(compare_payload.get('has_controller_valid_track', False)))}"
        )

        comparisons = compare_payload.get("comparisons", [])
        if not isinstance(comparisons, list) or not comparisons:
            lines.append("no world tracks available")
            return lines

        for comparison in comparisons:
            if not isinstance(comparison, dict):
                continue
            delta = comparison.get("delta_m", {})
            track_position = comparison.get("track_position_m", {})
            yolo_centroid = comparison.get("yolo_centroid_px", {})
            norfair_centroid = comparison.get("norfair_centroid_px", {})
            if (
                not isinstance(delta, dict)
                or not isinstance(track_position, dict)
                or not isinstance(yolo_centroid, dict)
                or not isinstance(norfair_centroid, dict)
            ):
                continue
            lines.append(
                f"track {int(comparison.get('track_id', -1))}  "
                f"err={float(comparison.get('delta_norm_m', 0.0)):.3f}m  "
                f"gt=({float(reference_position.get('x', 0.0)):.2f},"
                f"{float(reference_position.get('y', 0.0)):.2f},"
                f"{float(reference_position.get('z', 0.0)):.2f})"
            )
            lines.append(
                "calc="
                f"({float(track_position.get('x', 0.0)):.2f},"
                f"{float(track_position.get('y', 0.0)):.2f},"
                f"{float(track_position.get('z', 0.0)):.2f})  "
                f"d=({float(delta.get('x', 0.0)):+.2f},"
                f"{float(delta.get('y', 0.0)):+.2f},"
                f"{float(delta.get('z', 0.0)):+.2f})"
            )
            lines.append(
                "yolo_xy="
                f"({float(yolo_centroid.get('x', 0.0)):.1f},"
                f"{float(yolo_centroid.get('y', 0.0)):.1f})  "
                "norfair_xy="
                f"({float(norfair_centroid.get('x', 0.0)):.1f},"
                f"{float(norfair_centroid.get('y', 0.0)):.1f})  "
                f"depth_z={float(comparison.get('depth_z_m', 0.0)):.2f}m"
            )
        return lines


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
