from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from drone_control_pkg.deployment_config import (
    configured_compare_pose_topic,
    configured_drone_id,
    configured_mavros_namespace,
)
from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic, join_topic
from drone_msgs.msg import (
    EngagementState,
    PerceptionStatus,
    TargetTrack,
    TargetTrackArray,
    WorldTargetTrack,
    WorldTargetTrackArray,
)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, Range
from std_msgs.msg import String

from drone_vision_pkg.model_paths import resolve_model_path
from drone_vision_pkg.projection import RealsenseProjection
from drone_vision_pkg.tracker_resilience import (
    BodyTrackState,
    TRACK_SOURCE_DETECTED,
    TRACK_SOURCE_HELD,
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
    import pyzed.sl as sl
except Exception:  # pragma: no cover - platform specific
    sl = None

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


STARTUP_LED_STATES = {
    "SYNC_TAKEOFF_PARAM",
    "SYNC_PRE_TAKEOFF_PROFILE",
    "SET_TAKEOFF_MODE",
    "ARMING",
    "REQUEST_TAKEOFF",
    "TAKEOFF",
    "WARMUP_OFFBOARD",
    "SET_OFFBOARD_MODE",
    "SYNC_SPEED_PROFILE",
    "CLIMB_TO_TAKEOFF_ALTITUDE",
    "STAGE_HOVER",
}
RETURN_LED_STATES = {"RETURN_TO_START", "RETURN_HOME_HOLD"}
LANDING_LED_STATES = {
    "SET_LAND_MODE",
    "WAIT_TOUCHDOWN",
    "DISARMING",
    "RESTORE_TAKEOFF_PARAM",
    "RESTORE_SPEED_PROFILE",
}
MISSION_TERMINAL_STATES = {"IDLE", "COMPLETE", "ABORT"}
LED_RGB_BY_NAME: dict[str, tuple[int, int, int]] = {
    "off": (0, 0, 0),
    "startup": (0, 80, 255),
    "search": (255, 220, 0),
    "follow": (0, 255, 0),
    "dwell": (0, 255, 255),
    "return": (180, 0, 255),
    "landing": (255, 120, 0),
    "complete": (255, 255, 255),
    "blocked": (255, 60, 0),
    "abort": (255, 0, 0),
    "unknown": (120, 120, 120),
}
TARGET_TRACK_SOURCE_DETECTED = 0
TARGET_TRACK_SOURCE_HELD = 1
TARGET_TRACK_SOURCE_PREDICTED = 2
TARGET_MAP_SOURCE_LABELS = {
    TARGET_TRACK_SOURCE_DETECTED: "det",
    TARGET_TRACK_SOURCE_HELD: "held",
    TARGET_TRACK_SOURCE_PREDICTED: "pred",
}
TARGET_MAP_SOURCE_BGR = {
    TARGET_TRACK_SOURCE_DETECTED: (40, 220, 70),
    TARGET_TRACK_SOURCE_HELD: (0, 220, 255),
    TARGET_TRACK_SOURCE_PREDICTED: (0, 130, 255),
}


def led_color_name_for_engagement_state(
    *, state: str, blocked_reason: str = "", estop_latched: bool = False
) -> str:
    state_upper = str(state or "").strip().upper()
    blocked = bool(str(blocked_reason or "").strip())

    if estop_latched or state_upper == "ABORT":
        return "abort"
    if blocked and state_upper not in {"IDLE", "COMPLETE"}:
        return "blocked"
    if state_upper == "IDLE":
        return "off"
    if state_upper == "COMPLETE":
        return "complete"
    if state_upper in STARTUP_LED_STATES:
        return "startup"
    if state_upper == "SEARCH":
        return "search"
    if state_upper == "FOLLOW":
        return "follow"
    if state_upper == "DWELL":
        return "dwell"
    if state_upper in RETURN_LED_STATES:
        return "return"
    if state_upper in LANDING_LED_STATES:
        return "landing"
    return "unknown"


def led_bgr_for_color_name(color_name: str) -> tuple[int, int, int]:
    rgb = LED_RGB_BY_NAME.get(str(color_name), LED_RGB_BY_NAME["unknown"])
    return (int(rgb[2]), int(rgb[1]), int(rgb[0]))


def is_active_mission_state(state: str) -> bool:
    state_upper = str(state or "").strip().upper()
    return bool(state_upper) and state_upper not in MISSION_TERMINAL_STATES


def make_depth_colormap(
    depth_frame: np.ndarray | None,
    *,
    max_depth_m: float,
    output_size: tuple[int, int],
) -> np.ndarray:
    width_px, height_px = output_size
    if cv2 is None or depth_frame is None:
        return np.zeros((height_px, width_px, 3), dtype=np.uint8)

    depth = np.asarray(depth_frame, dtype=np.float32)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    max_depth_m = max(float(max_depth_m), 0.1)
    finite_depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    clipped = np.clip(finite_depth, 0.0, max_depth_m)
    normalized = (255.0 * (clipped / max_depth_m)).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    invalid_mask = finite_depth <= 0.0
    if np.any(invalid_mask):
        colored[invalid_mask] = (0, 0, 0)
    return cv2.resize(colored, output_size, interpolation=cv2.INTER_AREA)


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


def normalize_zed_enum_name(
    value: object,
    *,
    parameter_name: str,
    allowed: set[str],
) -> str:
    normalized = str(value or "").strip().upper()
    if normalized in allowed:
        return normalized
    options = ", ".join(sorted(name.lower() for name in allowed))
    raise ValueError(f"{parameter_name} must be one of: {options}")


def normalize_zed_flip_mode(value: object) -> str:
    if isinstance(value, bool):
        return "ON" if value else "OFF"
    normalized = str(value or "").strip().upper()
    aliases = {
        "TRUE": "ON",
        "YES": "ON",
        "1": "ON",
        "FALSE": "OFF",
        "NO": "OFF",
        "0": "OFF",
    }
    normalized = aliases.get(normalized, normalized)
    if normalized in {"ON", "OFF", "AUTO"}:
        return normalized
    raise ValueError("zed_flip_mode must be one of: ON, OFF, AUTO")


def normalize_zed_image_view(value: object) -> str:
    normalized = str(value or "").strip().upper()
    aliases = {
        "L": "LEFT_BGR",
        "LEFT": "LEFT_BGR",
        "LEFT_COLOR": "LEFT_BGR",
        "LEFT_BGR": "LEFT_BGR",
        "R": "RIGHT_BGR",
        "RIGHT": "RIGHT_BGR",
        "RIGHT_COLOR": "RIGHT_BGR",
        "RIGHT_BGR": "RIGHT_BGR",
    }
    result = aliases.get(normalized)
    if result is not None:
        return result
    raise ValueError("zed_image_view must be one of: left, left_bgr, right, right_bgr")


def zed_timestamp_key(timestamp_ns: int) -> tuple[int, int]:
    timestamp_ns = int(timestamp_ns)
    return (timestamp_ns // 1_000_000_000, timestamp_ns % 1_000_000_000)


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
        self.declare_parameter("zed_camera_resolution", "HD720")
        self.declare_parameter("zed_camera_fps", 30)
        self.declare_parameter("zed_depth_mode", "PERFORMANCE")
        self.declare_parameter("zed_coordinate_system", "RIGHT_HANDED_Z_UP_X_FWD")
        self.declare_parameter("zed_coordinate_units", "METER")
        self.declare_parameter("zed_flip_mode", "ON")
        self.declare_parameter("zed_depth_minimum_distance_m", 0.3)
        self.declare_parameter("zed_depth_maximum_distance_m", 20.0)
        self.declare_parameter("zed_image_view", "left")
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("record_mission_video", True)
        self.declare_parameter("recording_output_dir", "mission_recordings")
        self.declare_parameter("recording_fps", 10.0)
        self.declare_parameter("recording_width", 1280)
        self.declare_parameter("recording_height", 720)
        self.declare_parameter("recording_fourcc", "mp4v")
        self.declare_parameter("recording_depth_max_m", 6.0)
        self.declare_parameter("recording_save_depth_video", True)
        self.declare_parameter("recording_replace_depth_tile_with_yolo", False)
        self.declare_parameter("recording_state_topic", "")
        self.declare_parameter("recording_mavros_state_topic", "")
        self.declare_parameter("recording_local_pose_topic", "")
        self.declare_parameter("recording_flow_range_topic", "")
        self.declare_parameter("recording_target_map_world_topic", "")
        self.declare_parameter("recording_target_map_history_s", 3.0)

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
        self.zed_camera_resolution = str(
            self.get_parameter("zed_camera_resolution").value
        ).strip()
        self.zed_camera_fps = int(self.get_parameter("zed_camera_fps").value)
        self.zed_depth_mode = str(self.get_parameter("zed_depth_mode").value).strip()
        self.zed_coordinate_system = str(
            self.get_parameter("zed_coordinate_system").value
        ).strip()
        self.zed_coordinate_units = str(
            self.get_parameter("zed_coordinate_units").value
        ).strip()
        self.zed_flip_mode = self.get_parameter("zed_flip_mode").value
        self.zed_depth_minimum_distance_m = float(
            self.get_parameter("zed_depth_minimum_distance_m").value
        )
        self.zed_depth_maximum_distance_m = float(
            self.get_parameter("zed_depth_maximum_distance_m").value
        )
        self.zed_image_view_name = str(
            self.get_parameter("zed_image_view").value
        ).strip()
        self.mavros_namespace = (
            str(self.get_parameter("mavros_namespace").value).strip()
            or configured_mavros_namespace()
        )
        self.record_mission_video = bool(
            self.get_parameter("record_mission_video").value
        )
        self.recording_output_dir = Path(
            str(self.get_parameter("recording_output_dir").value).strip()
            or "mission_recordings"
        )
        self.recording_fps = float(self.get_parameter("recording_fps").value)
        self.recording_width = int(self.get_parameter("recording_width").value)
        self.recording_height = int(self.get_parameter("recording_height").value)
        self.recording_fourcc = (
            str(self.get_parameter("recording_fourcc").value).strip() or "mp4v"
        )
        self.recording_depth_max_m = float(
            self.get_parameter("recording_depth_max_m").value
        )
        self.recording_save_depth_video = bool(
            self.get_parameter("recording_save_depth_video").value
        )
        self.recording_replace_depth_tile_with_yolo = bool(
            self.get_parameter("recording_replace_depth_tile_with_yolo").value
        )
        self.recording_state_topic = (
            str(self.get_parameter("recording_state_topic").value).strip()
            or cdrone_topic(self.drone_id, "engagement/state")
        )
        self.recording_mavros_state_topic = (
            str(self.get_parameter("recording_mavros_state_topic").value).strip()
            or join_topic(self.mavros_namespace, "state")
        )
        self.recording_local_pose_topic = (
            str(self.get_parameter("recording_local_pose_topic").value).strip()
            or join_topic(self.mavros_namespace, "local_position/pose")
        )
        self.recording_flow_range_topic = (
            str(self.get_parameter("recording_flow_range_topic").value).strip()
            or join_topic(self.mavros_namespace, "px4flow/ground_distance")
        )
        self.recording_target_map_world_topic = (
            str(self.get_parameter("recording_target_map_world_topic").value).strip()
            or cdrone_topic(self.drone_id, "target_map/world_tracks")
        )
        self.recording_target_map_history_s = float(
            self.get_parameter("recording_target_map_history_s").value
        )
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

        if self.source_mode not in {"direct", "ros", "zed_sdk"}:
            raise RuntimeError(
                "Unsupported source_mode. Expected 'direct', 'ros', or "
                f"'zed_sdk', got '{self.source_mode}'."
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
        if self.source_mode == "zed_sdk" and sl is None:
            raise RuntimeError(
                "ZED SDK source_mode requires pyzed.sl to be installed."
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
        self.zed_camera = None
        self.zed_runtime = None
        self.zed_color_mat = None
        self.zed_depth_mat = None
        self.zed_image_view = None
        self.depth_scale_m = 0.001
        self.latest_compare_pose: PoseStamped | None = None
        self.latest_engagement_state = EngagementState()
        self.latest_engagement_state.state = "IDLE"
        self.latest_mavros_state = State()
        self.latest_local_pose: PoseStamped | None = None
        self.latest_flow_range: Range | None = None
        self.latest_target_map_world_tracks: list[WorldTargetTrack] = []
        self.last_engagement_state_s = 0.0
        self.last_mavros_state_s = 0.0
        self.last_local_pose_s = 0.0
        self.last_flow_range_s = 0.0
        self.last_target_map_world_s = 0.0
        self.target_map_world_history: dict[int, list[tuple[float, np.ndarray]]] = {}
        self.mission_ownship_history: list[tuple[float, np.ndarray]] = []
        self.mission_target_map_world_history: dict[
            int, list[tuple[float, np.ndarray]]
        ] = {}
        self.mission_recording_writers: dict[str, object] = {}
        self.mission_recording_paths: dict[str, Path] = {}
        self.mission_recording_started_wall_time_s = 0.0
        self.mission_recording_last_write_s = 0.0
        self.mission_recording_frame_count = 0
        self.mission_recording_warned_unavailable = False

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
        elif self.source_mode == "direct":
            self._start_direct_pipeline()
        else:
            self._start_zed_sdk_pipeline()

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

        if self.record_mission_video:
            self.create_subscription(
                EngagementState,
                self.recording_state_topic,
                self.engagement_state_callback,
                10,
            )
            self.create_subscription(
                State,
                self.recording_mavros_state_topic,
                self.mavros_state_callback,
                10,
            )
            self.create_subscription(
                PoseStamped,
                self.recording_local_pose_topic,
                self.local_pose_callback,
                best_effort_qos,
            )
            self.create_subscription(
                Range,
                self.recording_flow_range_topic,
                self.flow_range_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                WorldTargetTrackArray,
                self.recording_target_map_world_topic,
                self.target_map_world_callback,
                10,
            )

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
            f"{frames_csv_log_path}, "
            "mission_recording="
            f"{self.recording_output_dir if self.record_mission_video else 'disabled'}"
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
        self._stop_mission_recording("node shutdown")
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
        if self.zed_camera is not None:
            try:
                self.zed_camera.close()
            except Exception as exc:  # pragma: no cover - device specific cleanup
                self.get_logger().warn(
                    f"Closing direct ZED SDK camera raised: {exc}"
                )
            finally:
                self.zed_camera = None
                self.zed_runtime = None
                self.zed_color_mat = None
                self.zed_depth_mat = None
                self.zed_image_view = None
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

    def engagement_state_callback(self, msg: EngagementState) -> None:
        self.latest_engagement_state = msg
        self.last_engagement_state_s = self.now_s()
        if is_active_mission_state(msg.state):
            self._start_mission_recording(str(msg.state))
        else:
            self._stop_mission_recording(str(msg.state or "terminal"))

    def mavros_state_callback(self, msg: State) -> None:
        self.latest_mavros_state = msg
        self.last_mavros_state_s = self.now_s()

    def local_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_local_pose = msg
        now_s = self.now_s()
        self.last_local_pose_s = now_s
        if self.mission_recording_writers:
            self.mission_ownship_history.append(
                (
                    now_s,
                    np.array(
                        [
                            msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z,
                        ],
                        dtype=np.float64,
                    ),
                )
            )
            self.mission_ownship_history = self.mission_ownship_history[-6000:]

    def flow_range_callback(self, msg: Range) -> None:
        self.latest_flow_range = msg
        self.last_flow_range_s = self.now_s()

    def target_map_world_callback(self, msg: WorldTargetTrackArray) -> None:
        now_s = self.now_s()
        self.latest_target_map_world_tracks = list(msg.tracks)
        self.last_target_map_world_s = now_s
        seen_ids: set[int] = set()
        for track in msg.tracks:
            track_id = int(track.track_id)
            seen_ids.add(track_id)
            history = self.target_map_world_history.setdefault(track_id, [])
            history.append(
                (
                    now_s,
                    np.array([track.x_m, track.y_m, track.z_m], dtype=np.float64),
                )
            )
            keep_after_s = now_s - max(self.recording_target_map_history_s, 0.0)
            self.target_map_world_history[track_id] = [
                item for item in history if item[0] >= keep_after_s
            ][-120:]
            if self.mission_recording_writers:
                mission_history = self.mission_target_map_world_history.setdefault(
                    track_id,
                    [],
                )
                mission_history.append(
                    (
                        now_s,
                        np.array([track.x_m, track.y_m, track.z_m], dtype=np.float64),
                    )
                )
                self.mission_target_map_world_history[track_id] = mission_history[-6000:]

        keep_after_s = now_s - max(self.recording_target_map_history_s, 0.0)
        for track_id in list(self.target_map_world_history.keys()):
            history = [
                item
                for item in self.target_map_world_history[track_id]
                if item[0] >= keep_after_s
            ]
            if history:
                self.target_map_world_history[track_id] = history
            elif track_id not in seen_ids:
                self.target_map_world_history.pop(track_id, None)

    def _mission_recording_output_size(self) -> tuple[int, int]:
        width_px = max(int(self.recording_width), 640)
        height_px = max(int(self.recording_height), 480)
        if width_px % 2:
            width_px += 1
        if height_px % 2:
            height_px += 1
        return width_px, height_px

    def _start_mission_recording(self, state: str) -> None:
        if not self.record_mission_video or cv2 is None:
            return
        if self.mission_recording_writers:
            return

        width_px, height_px = self._mission_recording_output_size()
        tile_size = (width_px // 2, height_px // 2)
        output_dir = self.recording_output_dir
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            if not self.mission_recording_warned_unavailable:
                self.get_logger().warn(
                    "Mission recording disabled because the output directory "
                    f"could not be created: {exc}"
                )
                self.mission_recording_warned_unavailable = True
            return

        started_wall_time_s = time.time()
        timestamp = time.strftime(
            "%Y%m%d_%H%M%S",
            time.localtime(started_wall_time_s),
        )
        timestamp = f"{timestamp}_{int((started_wall_time_s % 1.0) * 1000):03d}"
        safe_state = "".join(
            ch.lower() if ch.isalnum() else "_" for ch in str(state or "mission")
        ).strip("_")
        output_stem = f"mission_{self.drone_id}_{timestamp}_{safe_state or 'active'}"
        fourcc_text = (self.recording_fourcc or "mp4v")[:4].ljust(4)
        fourcc = cv2.VideoWriter_fourcc(*fourcc_text)
        fps = max(float(self.recording_fps), 1.0)
        stream_specs = {
            "combined": (
                (width_px, height_px),
                output_dir / f"{output_stem}_combined.mp4",
            ),
            "rgb": (tile_size, output_dir / f"{output_stem}_rgb.mp4"),
            "yolo": (tile_size, output_dir / f"{output_stem}_yolo.mp4"),
            "stats": (tile_size, output_dir / f"{output_stem}_stats.mp4"),
            "target_map": (
                (width_px, height_px),
                output_dir / f"{output_stem}_target_map.mp4",
            ),
        }
        if self.recording_save_depth_video:
            stream_specs["depth"] = (
                tile_size,
                output_dir / f"{output_stem}_depth.mp4",
            )

        writers = {}
        paths = {}
        for stream_name, (stream_size, output_path) in stream_specs.items():
            writer = cv2.VideoWriter(
                str(output_path),
                fourcc,
                fps,
                stream_size,
            )
            if not writer.isOpened():
                writer.release()
                for opened_writer in writers.values():
                    opened_writer.release()
                self.get_logger().warn(
                    "Mission recording could not open MP4 writer "
                    f"for {stream_name} at {output_path}"
                )
                return
            writers[stream_name] = writer
            paths[stream_name] = output_path

        self.mission_recording_writers = writers
        self.mission_recording_paths = paths
        self.mission_recording_started_wall_time_s = started_wall_time_s
        self.mission_recording_last_write_s = 0.0
        self.mission_recording_frame_count = 0
        self.mission_ownship_history = []
        self.mission_target_map_world_history = {}
        self.get_logger().info(
            "Mission recording started: "
            + ", ".join(f"{name}={path}" for name, path in paths.items())
        )

    def _stop_mission_recording(self, reason: str) -> None:
        writers = self.mission_recording_writers
        if not writers:
            return
        for writer in writers.values():
            writer.release()
        output_paths = dict(self.mission_recording_paths)
        frame_count = self.mission_recording_frame_count
        self.mission_recording_writers = {}
        self.mission_recording_paths = {}
        self.mission_recording_started_wall_time_s = 0.0
        self.mission_recording_last_write_s = 0.0
        self.mission_recording_frame_count = 0
        self.get_logger().info(
            "Mission recording stopped "
            f"({reason}): "
            + ", ".join(f"{name}={path}" for name, path in output_paths.items())
            + f" frames={frame_count}"
        )

    def _maybe_record_mission_frame(
        self,
        *,
        frame: np.ndarray,
        depth_frame: np.ndarray | None,
        body_tracks: list[TargetTrack],
        body_track_debug: dict[int, dict[str, object]],
        detections_count: int,
        active_tracks_count: int,
        tracker_fps: float,
        now_s: float,
    ) -> None:
        if not self.record_mission_video:
            return
        if is_active_mission_state(self.latest_engagement_state.state):
            self._start_mission_recording(self.latest_engagement_state.state)
        writers = self.mission_recording_writers
        if not writers:
            return

        min_period_s = 1.0 / max(float(self.recording_fps), 1.0)
        if (
            self.mission_recording_last_write_s > 0.0
            and now_s - self.mission_recording_last_write_s < min_period_s
        ):
            return

        output_frames = self._compose_mission_recording_frames(
            frame=frame,
            depth_frame=depth_frame,
            body_tracks=body_tracks,
            body_track_debug=body_track_debug,
            detections_count=detections_count,
            active_tracks_count=active_tracks_count,
            tracker_fps=tracker_fps,
        )
        for stream_name, output_frame in output_frames.items():
            writer = writers.get(stream_name)
            if writer is not None:
                writer.write(output_frame)
        self.mission_recording_last_write_s = now_s
        self.mission_recording_frame_count += 1

    def _compose_mission_recording_frames(
        self,
        *,
        frame: np.ndarray,
        depth_frame: np.ndarray | None,
        body_tracks: list[TargetTrack],
        body_track_debug: dict[int, dict[str, object]],
        detections_count: int,
        active_tracks_count: int,
        tracker_fps: float,
    ) -> dict[str, np.ndarray]:
        output_w, output_h = self._mission_recording_output_size()
        tile_w = output_w // 2
        tile_h = output_h // 2

        rgb_tile = cv2.resize(frame, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
        depth_tile = make_depth_colormap(
            depth_frame,
            max_depth_m=self.recording_depth_max_m,
            output_size=(tile_w, tile_h),
        )
        yolo_detector_tile = cv2.resize(
            self._make_yolo_overlay(
                frame,
                body_track_debug,
                include_target_map=False,
            ),
            (tile_w, tile_h),
            interpolation=cv2.INTER_AREA,
        )
        annotated_tile = cv2.resize(
            self._make_yolo_overlay(
                frame,
                body_track_debug,
                include_target_map=True,
            ),
            (tile_w, tile_h),
            interpolation=cv2.INTER_AREA,
        )
        stats_tile = self._make_stats_panel(
            size=(tile_w, tile_h),
            body_tracks=body_tracks,
            detections_count=detections_count,
            active_tracks_count=active_tracks_count,
            tracker_fps=tracker_fps,
        )
        target_map_frame = self._make_target_map_recording_frame(
            size=(output_w, output_h)
        )

        first_tile = (
            yolo_detector_tile
            if self.recording_replace_depth_tile_with_yolo
            else depth_tile
        )
        first_tile_label = (
            "RGB + YOLO"
            if self.recording_replace_depth_tile_with_yolo
            else "Depth Map"
        )
        self._draw_tile_label(first_tile, first_tile_label)
        self._draw_tile_label(rgb_tile, "RGB")
        annotated_label = (
            "YOLO + Target Map"
            if self.latest_target_map_world_tracks
            else "RGB + YOLO"
        )
        self._draw_tile_label(annotated_tile, annotated_label)
        mosaic = np.vstack(
            (
                np.hstack((first_tile, rgb_tile)),
                np.hstack((annotated_tile, stats_tile)),
            )
        )
        output_frames = {
            "combined": mosaic,
            "rgb": rgb_tile,
            "yolo": annotated_tile,
            "stats": stats_tile,
            "target_map": target_map_frame,
        }
        if self.recording_save_depth_video:
            output_frames["depth"] = depth_tile
        return output_frames

    def _make_yolo_overlay(
        self,
        frame: np.ndarray,
        body_track_debug: dict[int, dict[str, object]],
        *,
        include_target_map: bool,
    ) -> np.ndarray:
        annotated = frame.copy()
        for track_id, debug_info in body_track_debug.items():
            yolo_bbox = debug_info.get("yolo_bbox_px")
            if not isinstance(yolo_bbox, dict):
                continue
            x1 = int(round(float(yolo_bbox.get("x1", 0.0))))
            y1 = int(round(float(yolo_bbox.get("y1", 0.0))))
            x2 = int(round(float(yolo_bbox.get("x2", 0.0))))
            y2 = int(round(float(yolo_bbox.get("y2", 0.0))))
            x1 = max(0, min(annotated.shape[1] - 1, x1))
            x2 = max(0, min(annotated.shape[1] - 1, x2))
            y1 = max(0, min(annotated.shape[0] - 1, y1))
            y2 = max(0, min(annotated.shape[0] - 1, y2))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label_parts = [f"id {track_id}"]
            depth_m = debug_info.get("depth_z_m")
            if depth_m is not None:
                label_parts.append(f"z {float(depth_m):.2f}m")
            cv2.putText(
                annotated,
                " ".join(label_parts),
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        if include_target_map:
            self._draw_target_map_video_overlay(annotated)
        return annotated

    def _project_world_to_pixel(
        self,
        world_point_m: np.ndarray,
        *,
        image_shape: tuple[int, ...],
    ) -> tuple[int, int, float] | None:
        if not self.projector.has_pose() or not self.projector.has_intrinsics():
            return None
        camera_point_m = self.projector.world_to_camera(world_point_m)
        if camera_point_m is None:
            return None
        z_m = float(camera_point_m[2])
        if z_m <= 0.05:
            return None
        intrinsics = self.projector.intrinsics
        if intrinsics is None:
            return None
        x_px = (float(camera_point_m[0]) * intrinsics.fx / z_m) + intrinsics.cx
        y_px = (float(camera_point_m[1]) * intrinsics.fy / z_m) + intrinsics.cy
        if not np.isfinite(x_px) or not np.isfinite(y_px):
            return None
        height_px, width_px = image_shape[:2]
        margin_px = max(width_px, height_px) * 0.25
        if (
            x_px < -margin_px
            or x_px > width_px + margin_px
            or y_px < -margin_px
            or y_px > height_px + margin_px
        ):
            return None
        return int(round(x_px)), int(round(y_px)), z_m

    def _draw_target_map_video_overlay(self, image: np.ndarray) -> None:
        if cv2 is None or not self.latest_target_map_world_tracks:
            return
        now_s = self.now_s()
        topic_age_s = now_s - self.last_target_map_world_s
        if topic_age_s > 2.0:
            return

        height_px, width_px = image.shape[:2]
        legend_lines = [
            f"Target map overlay age={topic_age_s:.1f}s",
            "green=detected yellow=held orange=predicted",
        ]
        overlay_w = min(width_px - 20, 620)
        overlay_h = 16 + (len(legend_lines) * 22)
        cv2.rectangle(image, (10, height_px - overlay_h - 10), (10 + overlay_w, height_px - 10), (0, 0, 0), -1)
        cv2.rectangle(image, (10, height_px - overlay_h - 10), (10 + overlay_w, height_px - 10), (0, 130, 255), 2)
        for idx, line in enumerate(legend_lines):
            cv2.putText(
                image,
                line,
                (22, height_px - overlay_h + 14 + (idx * 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (245, 245, 245),
                1,
                cv2.LINE_AA,
            )

        for track in self.latest_target_map_world_tracks:
            source = int(getattr(track, "source", TARGET_TRACK_SOURCE_DETECTED))
            color = TARGET_MAP_SOURCE_BGR.get(source, (220, 220, 220))
            world_position_m = np.array(
                [track.x_m, track.y_m, track.z_m],
                dtype=np.float64,
            )
            projected = self._project_world_to_pixel(
                world_position_m,
                image_shape=image.shape,
            )
            if projected is None:
                continue
            x_px, y_px, depth_z_m = projected
            x_px = max(0, min(width_px - 1, x_px))
            y_px = max(0, min(height_px - 1, y_px))

            self._draw_target_map_history(
                image,
                track_id=int(track.track_id),
                color=color,
            )
            uncertainty_m = max(float(getattr(track, "position_uncertainty_m", 0.0)), 0.0)
            uncertainty_px = self._uncertainty_radius_px(
                uncertainty_m,
                depth_z_m=depth_z_m,
            )
            if uncertainty_px > 2:
                cv2.circle(image, (x_px, y_px), uncertainty_px, color, 1, cv2.LINE_AA)

            marker_radius_px = 7 if source == TARGET_TRACK_SOURCE_PREDICTED else 5
            cv2.circle(image, (x_px, y_px), marker_radius_px, color, thickness=-1)
            cv2.circle(image, (x_px, y_px), marker_radius_px + 2, (0, 0, 0), thickness=1)
            velocity_endpoint = self._project_world_to_pixel(
                world_position_m
                + (np.array([track.vx_mps, track.vy_mps, track.vz_mps], dtype=np.float64) * 0.5),
                image_shape=image.shape,
            )
            if velocity_endpoint is not None:
                end_x = max(0, min(width_px - 1, velocity_endpoint[0]))
                end_y = max(0, min(height_px - 1, velocity_endpoint[1]))
                cv2.arrowedLine(
                    image,
                    (x_px, y_px),
                    (end_x, end_y),
                    color,
                    2,
                    cv2.LINE_AA,
                    tipLength=0.25,
                )

            source_label = TARGET_MAP_SOURCE_LABELS.get(source, "unk")
            label = (
                f"M{int(track.track_id)} {source_label} "
                f"age={float(getattr(track, 'last_observed_age_s', 0.0)):.1f}s "
                f"unc={uncertainty_m:.2f}m"
            )
            cv2.putText(
                image,
                label,
                (min(width_px - 260, x_px + 10), max(22, y_px - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                label,
                (min(width_px - 260, x_px + 10), max(22, y_px - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                1,
                cv2.LINE_AA,
            )

    def _uncertainty_radius_px(self, uncertainty_m: float, *, depth_z_m: float) -> int:
        if uncertainty_m <= 0.0 or depth_z_m <= 0.05:
            return 0
        intrinsics = self.projector.intrinsics
        if intrinsics is None:
            return 0
        focal_px = (float(intrinsics.fx) + float(intrinsics.fy)) / 2.0
        return int(round(min(120.0, max(0.0, focal_px * uncertainty_m / depth_z_m))))

    def _draw_target_map_history(
        self,
        image: np.ndarray,
        *,
        track_id: int,
        color: tuple[int, int, int],
    ) -> None:
        history = self.target_map_world_history.get(int(track_id), [])
        if len(history) < 2:
            return
        points: list[tuple[int, int]] = []
        for _, world_position_m in history:
            projected = self._project_world_to_pixel(
                world_position_m,
                image_shape=image.shape,
            )
            if projected is None:
                points = []
                continue
            x_px = max(0, min(image.shape[1] - 1, projected[0]))
            y_px = max(0, min(image.shape[0] - 1, projected[1]))
            points.append((x_px, y_px))
        for start, end in zip(points[:-1], points[1:]):
            cv2.line(image, start, end, color, 2, cv2.LINE_AA)

    def _make_target_map_recording_frame(
        self,
        *,
        size: tuple[int, int],
    ) -> np.ndarray:
        width_px, height_px = size
        frame = np.full((height_px, width_px, 3), (22, 24, 28), dtype=np.uint8)
        self._draw_tile_label(frame, "Target Map Recording", accent_bgr=(0, 130, 255))

        map_rect = (70, 70, width_px - 40, height_px - 80)
        map_points = self._recording_map_points()
        if not map_points:
            cv2.putText(
                frame,
                "Waiting for ownship pose or target-map tracks",
                (90, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (230, 230, 230),
                2,
                cv2.LINE_AA,
            )
            return frame

        bounds = self._recording_map_bounds(map_points)
        self._draw_recording_map_grid(frame, map_rect=map_rect, bounds=bounds)
        self._draw_recording_map_ownship(frame, map_rect=map_rect, bounds=bounds)
        self._draw_recording_map_targets(frame, map_rect=map_rect, bounds=bounds)
        self._draw_recording_map_legend(frame, map_rect=map_rect)
        return frame

    def _recording_map_points(self) -> list[np.ndarray]:
        points: list[np.ndarray] = []
        points.extend(position for _, position in self.mission_ownship_history)
        for history in self.mission_target_map_world_history.values():
            points.extend(position for _, position in history)
        if self.latest_local_pose is not None:
            points.append(
                np.array(
                    [
                        self.latest_local_pose.pose.position.x,
                        self.latest_local_pose.pose.position.y,
                        self.latest_local_pose.pose.position.z,
                    ],
                    dtype=np.float64,
                )
            )
        for track in self.latest_target_map_world_tracks:
            points.append(
                np.array([track.x_m, track.y_m, track.z_m], dtype=np.float64)
            )
        return [point for point in points if np.all(np.isfinite(point[:2]))]

    def _recording_map_bounds(
        self,
        points: list[np.ndarray],
    ) -> tuple[float, float, float, float]:
        xs = [float(point[0]) for point in points]
        ys = [float(point[1]) for point in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        margin = max(0.75, 0.18 * max(span_x, span_y))
        return min_x - margin, max_x + margin, min_y - margin, max_y + margin

    def _map_world_to_pixel(
        self,
        point_m: np.ndarray,
        *,
        map_rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
    ) -> tuple[int, int]:
        left, top, right, bottom = map_rect
        min_x, max_x, min_y, max_y = bounds
        usable_w = max(right - left, 1)
        usable_h = max(bottom - top, 1)
        x_ratio = (float(point_m[0]) - min_x) / max(max_x - min_x, 1e-6)
        y_ratio = (float(point_m[1]) - min_y) / max(max_y - min_y, 1e-6)
        x_px = left + int(round(x_ratio * usable_w))
        y_px = bottom - int(round(y_ratio * usable_h))
        return (
            max(left, min(right, x_px)),
            max(top, min(bottom, y_px)),
        )

    def _draw_recording_map_grid(
        self,
        image: np.ndarray,
        *,
        map_rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
    ) -> None:
        left, top, right, bottom = map_rect
        cv2.rectangle(image, (left, top), (right, bottom), (12, 14, 18), -1)
        cv2.rectangle(image, (left, top), (right, bottom), (120, 120, 120), 1)
        min_x, max_x, min_y, max_y = bounds
        span = max(max_x - min_x, max_y - min_y)
        grid_step = 0.5 if span <= 5.0 else 1.0 if span <= 12.0 else 2.0
        x_start = math.floor(min_x / grid_step) * grid_step
        y_start = math.floor(min_y / grid_step) * grid_step

        x_value = x_start
        while x_value <= max_x + 1e-6:
            point_a = self._map_world_to_pixel(
                np.array([x_value, min_y, 0.0], dtype=np.float64),
                map_rect=map_rect,
                bounds=bounds,
            )
            point_b = self._map_world_to_pixel(
                np.array([x_value, max_y, 0.0], dtype=np.float64),
                map_rect=map_rect,
                bounds=bounds,
            )
            color = (80, 80, 80) if abs(x_value) > 1e-6 else (120, 120, 120)
            cv2.line(image, point_a, point_b, color, 1, cv2.LINE_AA)
            x_value += grid_step

        y_value = y_start
        while y_value <= max_y + 1e-6:
            point_a = self._map_world_to_pixel(
                np.array([min_x, y_value, 0.0], dtype=np.float64),
                map_rect=map_rect,
                bounds=bounds,
            )
            point_b = self._map_world_to_pixel(
                np.array([max_x, y_value, 0.0], dtype=np.float64),
                map_rect=map_rect,
                bounds=bounds,
            )
            color = (80, 80, 80) if abs(y_value) > 1e-6 else (120, 120, 120)
            cv2.line(image, point_a, point_b, color, 1, cv2.LINE_AA)
            y_value += grid_step

        cv2.putText(
            image,
            f"frame={self.projector.frame_id} x[{min_x:.1f},{max_x:.1f}] y[{min_y:.1f},{max_y:.1f}]",
            (left, bottom + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (210, 210, 210),
            1,
            cv2.LINE_AA,
        )

    def _draw_recording_map_ownship(
        self,
        image: np.ndarray,
        *,
        map_rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
    ) -> None:
        if len(self.mission_ownship_history) >= 2:
            points = [
                self._map_world_to_pixel(
                    position,
                    map_rect=map_rect,
                    bounds=bounds,
                )
                for _, position in self.mission_ownship_history
            ]
            for start, end in zip(points[:-1], points[1:]):
                cv2.line(image, start, end, (255, 160, 40), 2, cv2.LINE_AA)

        if self.latest_local_pose is None:
            return
        current = np.array(
            [
                self.latest_local_pose.pose.position.x,
                self.latest_local_pose.pose.position.y,
                self.latest_local_pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        current_px = self._map_world_to_pixel(
            current,
            map_rect=map_rect,
            bounds=bounds,
        )
        cv2.circle(image, current_px, 8, (255, 160, 40), -1, cv2.LINE_AA)
        cv2.circle(image, current_px, 10, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(
            image,
            "defense",
            (current_px[0] + 12, current_px[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 190, 80),
            2,
            cv2.LINE_AA,
        )

    def _draw_recording_map_targets(
        self,
        image: np.ndarray,
        *,
        map_rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
    ) -> None:
        source_by_track_id = {
            int(track.track_id): int(
                getattr(track, "source", TARGET_TRACK_SOURCE_DETECTED)
            )
            for track in self.latest_target_map_world_tracks
        }
        for track_id, history in sorted(self.mission_target_map_world_history.items()):
            source = source_by_track_id.get(track_id, TARGET_TRACK_SOURCE_PREDICTED)
            color = TARGET_MAP_SOURCE_BGR.get(source, (220, 220, 220))
            points = [
                self._map_world_to_pixel(
                    position,
                    map_rect=map_rect,
                    bounds=bounds,
                )
                for _, position in history
            ]
            for start, end in zip(points[:-1], points[1:]):
                cv2.line(image, start, end, color, 2, cv2.LINE_AA)

        for track in self.latest_target_map_world_tracks:
            source = int(getattr(track, "source", TARGET_TRACK_SOURCE_DETECTED))
            color = TARGET_MAP_SOURCE_BGR.get(source, (220, 220, 220))
            position = np.array([track.x_m, track.y_m, track.z_m], dtype=np.float64)
            point_px = self._map_world_to_pixel(
                position,
                map_rect=map_rect,
                bounds=bounds,
            )
            uncertainty_m = max(
                float(getattr(track, "position_uncertainty_m", 0.0)),
                0.0,
            )
            if uncertainty_m > 0.0:
                radius_px = self._map_uncertainty_radius_px(
                    uncertainty_m,
                    map_rect=map_rect,
                    bounds=bounds,
                )
                if radius_px > 1:
                    cv2.circle(image, point_px, radius_px, color, 1, cv2.LINE_AA)
            cv2.circle(image, point_px, 7, color, -1, cv2.LINE_AA)
            cv2.circle(image, point_px, 9, (0, 0, 0), 1, cv2.LINE_AA)
            label = (
                f"M{int(track.track_id)} "
                f"{TARGET_MAP_SOURCE_LABELS.get(source, 'unk')} "
                f"{float(getattr(track, 'last_observed_age_s', 0.0)):.1f}s"
            )
            cv2.putText(
                image,
                label,
                (point_px[0] + 12, point_px[1] + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                color,
                2,
                cv2.LINE_AA,
            )

    def _map_uncertainty_radius_px(
        self,
        uncertainty_m: float,
        *,
        map_rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
    ) -> int:
        left, top, right, bottom = map_rect
        min_x, max_x, min_y, max_y = bounds
        px_per_m_x = (right - left) / max(max_x - min_x, 1e-6)
        px_per_m_y = (bottom - top) / max(max_y - min_y, 1e-6)
        px_per_m = min(px_per_m_x, px_per_m_y)
        return int(round(min(160.0, max(0.0, uncertainty_m * px_per_m))))

    def _draw_recording_map_legend(
        self,
        image: np.ndarray,
        *,
        map_rect: tuple[int, int, int, int],
    ) -> None:
        left, top, right, _ = map_rect
        legend_x = max(left + 12, right - 390)
        legend_y = top + 14
        rows = [
            ((255, 160, 40), "defense drone path"),
            (TARGET_MAP_SOURCE_BGR[TARGET_TRACK_SOURCE_DETECTED], "target detected"),
            (TARGET_MAP_SOURCE_BGR[TARGET_TRACK_SOURCE_HELD], "target held"),
            (TARGET_MAP_SOURCE_BGR[TARGET_TRACK_SOURCE_PREDICTED], "target predicted"),
        ]
        cv2.rectangle(
            image,
            (legend_x - 10, legend_y - 6),
            (legend_x + 360, legend_y + (len(rows) * 24) + 8),
            (0, 0, 0),
            -1,
        )
        cv2.rectangle(
            image,
            (legend_x - 10, legend_y - 6),
            (legend_x + 360, legend_y + (len(rows) * 24) + 8),
            (80, 80, 80),
            1,
        )
        for idx, (color, label) in enumerate(rows):
            y_px = legend_y + (idx * 24) + 14
            cv2.circle(image, (legend_x + 8, y_px - 5), 5, color, -1, cv2.LINE_AA)
            cv2.putText(
                image,
                label,
                (legend_x + 24, y_px),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (230, 230, 230),
                1,
                cv2.LINE_AA,
            )

    def _draw_tile_label(
        self,
        image: np.ndarray,
        label: str,
        *,
        accent_bgr: tuple[int, int, int] | None = None,
    ) -> None:
        accent = (255, 255, 255) if accent_bgr is None else accent_bgr
        cv2.rectangle(image, (0, 0), (image.shape[1], 34), (0, 0, 0), -1)
        cv2.rectangle(image, (0, 0), (image.shape[1], 34), accent, 2)
        cv2.putText(
            image,
            label,
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _make_stats_panel(
        self,
        *,
        size: tuple[int, int],
        body_tracks: list[TargetTrack],
        detections_count: int,
        active_tracks_count: int,
        tracker_fps: float,
    ) -> np.ndarray:
        width_px, height_px = size
        panel = np.full((height_px, width_px, 3), (24, 24, 24), dtype=np.uint8)
        engagement = self.latest_engagement_state
        color_name = led_color_name_for_engagement_state(
            state=engagement.state,
            blocked_reason=engagement.blocked_reason,
            estop_latched=engagement.estop_latched,
        )
        led_bgr = led_bgr_for_color_name(color_name)
        self._draw_tile_label(panel, "Stats", accent_bgr=led_bgr)
        cv2.circle(panel, (width_px - 32, 17), 10, led_bgr, thickness=-1)
        cv2.circle(panel, (width_px - 32, 17), 11, (255, 255, 255), thickness=1)

        lines = self._mission_stats_lines(
            body_tracks=body_tracks,
            detections_count=detections_count,
            active_tracks_count=active_tracks_count,
            tracker_fps=tracker_fps,
            led_color_name=color_name,
        )
        y_px = 62
        line_height_px = 25
        for label, value, color in lines:
            if y_px > height_px - 12:
                break
            cv2.putText(
                panel,
                label,
                (16, y_px),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (170, 170, 170),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                panel,
                value,
                (190, y_px),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color,
                2,
                cv2.LINE_AA,
            )
            y_px += line_height_px
        return panel

    def _mission_stats_lines(
        self,
        *,
        body_tracks: list[TargetTrack],
        detections_count: int,
        active_tracks_count: int,
        tracker_fps: float,
        led_color_name: str,
    ) -> list[tuple[str, str, tuple[int, int, int]]]:
        now_s = self.now_s()
        engagement = self.latest_engagement_state
        mavros_state = self.latest_mavros_state
        active_track = self._select_stats_track(body_tracks)
        pose = self.latest_local_pose
        flow_range = self.latest_flow_range
        white = (245, 245, 245)
        muted = (180, 180, 180)
        warn = (0, 180, 255)
        bad = (40, 40, 255)
        good = (80, 255, 80)

        lines: list[tuple[str, str, tuple[int, int, int]]] = [
            (
                "Mission state",
                str(engagement.state or "UNKNOWN"),
                led_bgr_for_color_name(led_color_name),
            ),
            ("LED color", led_color_name, led_bgr_for_color_name(led_color_name)),
        ]
        if engagement.blocked_reason:
            lines.append(("Blocked", str(engagement.blocked_reason), warn))
        if engagement.estop_latched:
            lines.append(("ESTOP", "LATCHED", bad))

        state_age_s = now_s - self.last_mavros_state_s
        state_color = good if state_age_s <= 1.0 and mavros_state.connected else warn
        lines.extend(
            [
                ("PX4 mode", str(mavros_state.mode or "unknown"), state_color),
                (
                    "MAVROS",
                    f"conn={int(mavros_state.connected)} armed={int(mavros_state.armed)}",
                    state_color,
                ),
                (
                    "Target",
                    self._format_target_stats(active_track, engagement),
                    good if engagement.target_visible else muted,
                ),
                (
                    "Dwell",
                    f"{engagement.dwell_elapsed_s:.1f}s / rem {engagement.dwell_remaining_s:.1f}s",
                    white,
                ),
                (
                    "Completed",
                    f"{engagement.completed_targets_count}/{engagement.required_targets_count}",
                    white,
                ),
                (
                    "Perception",
                    f"det={detections_count} tracks={active_tracks_count} fps={tracker_fps:.1f}",
                    white,
                ),
                (
                    "Target map",
                    self._format_target_map_recording_stats(now_s=now_s),
                    good if now_s - self.last_target_map_world_s <= 1.0 else muted,
                ),
                ("Inference", f"{self.last_inference_ms:.1f} ms", white),
                (
                    "Local pose",
                    self._format_pose_stats(pose, now_s=now_s),
                    good if now_s - self.last_local_pose_s <= 1.0 else warn,
                ),
                (
                    "Flow range",
                    self._format_flow_range_stats(flow_range, now_s=now_s),
                    good if now_s - self.last_flow_range_s <= 1.0 else warn,
                ),
                (
                    "Recording",
                    f"{self.mission_recording_frame_count} frames",
                    white,
                ),
            ]
        )
        return lines

    def _format_target_map_recording_stats(self, *, now_s: float) -> str:
        if not self.latest_target_map_world_tracks:
            return "waiting"
        predicted_count = sum(
            1
            for track in self.latest_target_map_world_tracks
            if int(getattr(track, "source", TARGET_TRACK_SOURCE_DETECTED))
            == TARGET_TRACK_SOURCE_PREDICTED
        )
        max_age_s = max(
            float(getattr(track, "last_observed_age_s", 0.0))
            for track in self.latest_target_map_world_tracks
        )
        topic_age_s = max(now_s - self.last_target_map_world_s, 0.0)
        return (
            f"tracks={len(self.latest_target_map_world_tracks)} "
            f"pred={predicted_count} max_age={max_age_s:.1f}s topic={topic_age_s:.1f}s"
        )

    def _select_stats_track(self, body_tracks: list[TargetTrack]) -> TargetTrack | None:
        active_id = int(self.latest_engagement_state.active_track_id)
        if active_id >= 0:
            for track in body_tracks:
                if int(track.track_id) == active_id:
                    return track
        if not body_tracks:
            return None
        return max(body_tracks, key=lambda track: float(track.confidence))

    def _format_target_stats(
        self,
        track: TargetTrack | None,
        engagement: EngagementState,
    ) -> str:
        if track is None:
            if engagement.target_visible:
                return (
                    f"id={engagement.active_track_id} "
                    f"dist={engagement.active_distance_m:.2f}m"
                )
            return "none"
        return (
            f"id={track.track_id} dist={track.distance_m:.2f}m "
            f"conf={track.confidence:.2f}"
        )

    def _format_pose_stats(self, pose: PoseStamped | None, *, now_s: float) -> str:
        if pose is None:
            return "missing"
        age_s = now_s - self.last_local_pose_s
        position = pose.pose.position
        return (
            f"x={position.x:.2f} y={position.y:.2f} "
            f"z={position.z:.2f} age={age_s:.1f}s"
        )

    def _format_flow_range_stats(self, flow_range: Range | None, *, now_s: float) -> str:
        if flow_range is None:
            return "missing"
        age_s = now_s - self.last_flow_range_s
        return f"{float(flow_range.range):.2f}m age={age_s:.1f}s"

    def process_latest_frame(self) -> None:
        frame: np.ndarray | None = None
        depth_frame: np.ndarray | None = None

        if self.source_mode == "direct":
            direct_frame = self._read_direct_frame()
            if direct_frame is None:
                return
            frame, depth_frame, stamp_key = direct_frame
        elif self.source_mode == "zed_sdk":
            zed_frame = self._read_zed_sdk_frame()
            if zed_frame is None:
                return
            frame, depth_frame, stamp_key = zed_frame
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
        self._maybe_record_mission_frame(
            frame=frame,
            depth_frame=depth_frame,
            body_tracks=body_tracks,
            body_track_debug=body_track_debug,
            detections_count=len(detections),
            active_tracks_count=len(tracked_objects),
            tracker_fps=tracker_fps,
            now_s=process_time_s,
        )

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

    def _start_zed_sdk_pipeline(self) -> None:
        assert sl is not None

        resolution_name = normalize_zed_enum_name(
            self.zed_camera_resolution,
            parameter_name="zed_camera_resolution",
            allowed={name for name in dir(sl.RESOLUTION) if name.isupper()},
        )
        depth_mode_name = normalize_zed_enum_name(
            self.zed_depth_mode,
            parameter_name="zed_depth_mode",
            allowed={name for name in dir(sl.DEPTH_MODE) if name.isupper()},
        )
        coordinate_system_name = normalize_zed_enum_name(
            self.zed_coordinate_system,
            parameter_name="zed_coordinate_system",
            allowed={name for name in dir(sl.COORDINATE_SYSTEM) if name.isupper()},
        )
        coordinate_units_name = normalize_zed_enum_name(
            self.zed_coordinate_units,
            parameter_name="zed_coordinate_units",
            allowed={name for name in dir(sl.UNIT) if name.isupper()},
        )
        flip_name = normalize_zed_flip_mode(self.zed_flip_mode)
        image_view_name = normalize_zed_image_view(self.zed_image_view_name)

        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, resolution_name)
        init.camera_fps = max(int(self.zed_camera_fps), 1)
        init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode_name)
        init.coordinate_system = getattr(sl.COORDINATE_SYSTEM, coordinate_system_name)
        init.coordinate_units = getattr(sl.UNIT, coordinate_units_name)
        init.camera_image_flip = getattr(sl.FLIP_MODE, flip_name)
        init.depth_minimum_distance = float(self.zed_depth_minimum_distance_m)
        init.depth_maximum_distance = float(self.zed_depth_maximum_distance_m)

        self.zed_camera = sl.Camera()
        status = self.zed_camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.zed_camera = None
            raise RuntimeError(f"Could not open ZED SDK camera: {status}")

        calibration = (
            self.zed_camera.get_camera_information()
            .camera_configuration
            .calibration_parameters
        )
        left_cam = calibration.left_cam
        self.projector.set_intrinsics(
            fx=float(left_cam.fx),
            fy=float(left_cam.fy),
            cx=float(left_cam.cx),
            cy=float(left_cam.cy),
        )

        self.zed_runtime = sl.RuntimeParameters()
        self.zed_runtime.enable_depth = True
        self.zed_color_mat = sl.Mat()
        self.zed_depth_mat = sl.Mat()
        self.zed_image_view = getattr(sl.VIEW, image_view_name)
        self.depth_scale_m = 1.0

        self.get_logger().info(
            "Started direct ZED SDK pipeline: "
            f"resolution={resolution_name}, fps={init.camera_fps}, "
            f"depth_mode={depth_mode_name}, image_view={image_view_name}, "
            f"depth_range={init.depth_minimum_distance:.2f}-"
            f"{init.depth_maximum_distance:.2f}m, "
            f"intrinsics=fx:{float(left_cam.fx):.1f} fy:{float(left_cam.fy):.1f} "
            f"cx:{float(left_cam.cx):.1f} cy:{float(left_cam.cy):.1f}"
        )

    def _read_zed_sdk_frame(
        self,
    ) -> tuple[np.ndarray, np.ndarray | None, tuple[int, int]] | None:
        if (
            self.zed_camera is None
            or self.zed_runtime is None
            or self.zed_color_mat is None
            or self.zed_depth_mat is None
            or self.zed_image_view is None
        ):
            return None
        assert sl is not None

        status = self.zed_camera.grab(self.zed_runtime)
        if status != sl.ERROR_CODE.SUCCESS:
            self._warn_throttled(
                f"Waiting for direct ZED SDK frames failed: {status}",
                attr_name="last_wait_warn_s",
            )
            return None

        self.zed_camera.retrieve_image(self.zed_color_mat, self.zed_image_view)
        self.zed_camera.retrieve_measure(self.zed_depth_mat, sl.MEASURE.DEPTH)
        color = self.zed_color_mat.get_data()
        if color is None or color.ndim < 3 or color.shape[2] < 3:
            self._warn_throttled(
                "Waiting for the first color frame from the direct ZED SDK "
                "pipeline.",
                attr_name="last_wait_warn_s",
            )
            return None
        color_bgr = np.ascontiguousarray(color[:, :, :3])

        depth = self.zed_depth_mat.get_data()
        depth_m = None
        if depth is not None:
            depth_m = np.asarray(depth, dtype=np.float32)
            if depth_m.ndim == 3:
                depth_m = depth_m[:, :, 0]
            depth_m = np.ascontiguousarray(depth_m)

        timestamp_ns = int(
            self.zed_camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
        )
        return color_bgr, depth_m, zed_timestamp_key(timestamp_ns)

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
            msg.detector_track_id = track_id
            msg.source = TRACK_SOURCE_DETECTED
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
            msg.last_observed_age_s = 0.0
            msg.prediction_horizon_s = 0.0
            msg.position_uncertainty_m = 0.0
            msg.velocity_uncertainty_mps = 0.0
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
            detector_track_id=int(
                getattr(track, "detector_track_id", int(track.track_id))
            ),
            source=int(getattr(track, "source", TRACK_SOURCE_DETECTED)),
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
            last_observed_age_s=float(getattr(track, "last_observed_age_s", 0.0)),
            prediction_horizon_s=float(getattr(track, "prediction_horizon_s", 0.0)),
            position_uncertainty_m=float(
                getattr(track, "position_uncertainty_m", 0.0)
            ),
            velocity_uncertainty_mps=float(
                getattr(track, "velocity_uncertainty_mps", 0.0)
            ),
        )

    def _body_track_state_to_msg(self, state: BodyTrackState, stamp_msg) -> TargetTrack:
        msg = TargetTrack()
        msg.stamp = stamp_msg
        msg.track_id = int(state.track_id)
        msg.detector_track_id = int(state.detector_track_id)
        msg.source = int(state.source)
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
        msg.last_observed_age_s = float(state.last_observed_age_s)
        msg.prediction_horizon_s = float(state.prediction_horizon_s)
        msg.position_uncertainty_m = float(state.position_uncertainty_m)
        msg.velocity_uncertainty_mps = float(state.velocity_uncertainty_mps)
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
            held_debug["source"] = TRACK_SOURCE_HELD
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
            msg.detector_track_id = int(
                getattr(body_track, "detector_track_id", int(body_track.track_id))
            )
            msg.source = int(getattr(body_track, "source", TRACK_SOURCE_DETECTED))
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
            msg.last_observed_age_s = float(
                getattr(body_track, "last_observed_age_s", 0.0)
            )
            msg.prediction_horizon_s = float(
                getattr(body_track, "prediction_horizon_s", 0.0)
            )
            msg.position_uncertainty_m = float(
                getattr(body_track, "position_uncertainty_m", 0.0)
            )
            msg.velocity_uncertainty_mps = float(
                getattr(body_track, "velocity_uncertainty_mps", 0.0)
            )
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
                "zed_camera_resolution": self.zed_camera_resolution,
                "zed_camera_fps": int(self.zed_camera_fps),
                "zed_depth_mode": self.zed_depth_mode,
                "zed_coordinate_system": self.zed_coordinate_system,
                "zed_coordinate_units": self.zed_coordinate_units,
                "zed_depth_minimum_distance_m": float(
                    self.zed_depth_minimum_distance_m
                ),
                "zed_depth_maximum_distance_m": float(
                    self.zed_depth_maximum_distance_m
                ),
                "zed_image_view": self.zed_image_view_name,
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
