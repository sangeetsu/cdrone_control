from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

try:
    import cv2
except Exception:  # pragma: no cover - platform specific
    cv2 = None

try:
    from norfair import Detection, Tracker
except Exception:  # pragma: no cover - platform specific
    Detection = None
    Tracker = None

try: 
    from norfair.utils import get_cutout
except Exception:  # pragma: no cover - platform specific
    get_cutout = None

try: 
    from norfair.camera_motion import MotionEstimator
except Exception:  # pragma: no cover - platform specific
    MotionEstimator = None


try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - platform specific
    YOLO = None

from drone_vision_pkg.stereo_utils import (
    StereoDetection,
    build_csi_pipeline,
    camera_to_body,
    match_detections_epipolar,
    rpy_to_rotation_matrix,
    triangulate_point,
)
from drone_msgs.msg import PerceptionStatus, TargetTrack, TargetTrackArray


def cdrone_topic(drone_id: str, leaf: str) -> str:
    drone_id = str(drone_id or "").strip().strip("/")
    base = "/cdrone" if not drone_id else f"/cdrone/{drone_id}"
    return f"{base}/{leaf.lstrip('/')}"


class StereoTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("stereo_tracker_node")

        if cv2 is None or YOLO is None or Tracker is None or Detection is None:
            raise RuntimeError(
                "Missing dependencies. Ensure opencv, ultralytics and norfair are installed."
            )

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
        self.declare_parameter("model_path", "non_xai_best.engine")
        self.declare_parameter("device_id", 0)
        self.declare_parameter("model_input_size", 640)
        self.declare_parameter("detection_conf_threshold", 0.35)
        self.declare_parameter("epipolar_tolerance_px", 10.0)
        self.declare_parameter("tracker_rate_hz", 20.0)
        self.declare_parameter("calibration_path", "stereo_calibration.npz")
        self.declare_parameter("camera_body_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_body_rpy_rad", [0.0, 0.0, 0.0])
        self.declare_parameter("norfair_distance_threshold", 1.25)
        self.declare_parameter("rtsp_transport", "udp")
        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("tracks_topic", "")
        self.declare_parameter("perception_status_topic", "")

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
        self.model_path = str(self.get_parameter("model_path").value)
        self.device_id = int(self.get_parameter("device_id").value)
        self.model_input_size = int(self.get_parameter("model_input_size").value)
        self.conf_thresh = float(self.get_parameter("detection_conf_threshold").value)
        self.epipolar_tol = float(self.get_parameter("epipolar_tolerance_px").value)
        self.tracker_rate_hz = float(self.get_parameter("tracker_rate_hz").value)
        self.calibration_path = str(self.get_parameter("calibration_path").value)
        self.translation = np.array(
            self.get_parameter("camera_body_translation_m").value, dtype=float
        )
        self.rotation = rpy_to_rotation_matrix(
            np.array(self.get_parameter("camera_body_rpy_rad").value, dtype=float)
        )
        self.norfair_distance_threshold = float(
            self.get_parameter("norfair_distance_threshold").value
        )
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.tracks_topic = (
            str(self.get_parameter("tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/tracks")
        )
        self.perception_status_topic = (
            str(self.get_parameter("perception_status_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/status")
        )

        if self.source_mode not in ("csi", "rtsp"):
            self.get_logger().warn(
                f"Unsupported source_mode='{self.source_mode}', defaulting to csi."
            )
            self.source_mode = "csi"

        if self.source_mode == "rtsp":
            transport = str(self.get_parameter("rtsp_transport").value).strip().lower()
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{transport}"

        self._load_calibration(self.calibration_path)
        self.model = YOLO(self.model_path)
        self.tracker = self._build_tracker()
        self.motion_estimator = MotionEstimator() if MotionEstimator is not None else None

        self.left_cap = self._open_capture(self.left_sensor_id, self.left_url)
        self.right_cap = self._open_capture(self.right_sensor_id, self.right_url)

        self.prev_positions: Dict[int, Tuple[np.ndarray, float]] = {}
        self.tracks_pub = self.create_publisher(TargetTrackArray, self.tracks_topic, 10)
        self.perception_status_pub = self.create_publisher(
            PerceptionStatus, self.perception_status_topic, 10
        )
        self.last_process_time_s: float | None = None
        self.timer = self.create_timer(
            1.0 / max(self.tracker_rate_hz, 1.0), self.process_frame
        )
        self.get_logger().info(
            f"Stereo tracker started. source_mode={self.source_mode}, model={self.model_path}"
        )

    def _build_tracker(self):
        try:
            return Tracker(
                distance_function="euclidean",
                distance_threshold=self.norfair_distance_threshold,
                initialization_delay=1,
                hit_counter_max=15,

                reid_distance_function=self.embedding_distance,
                reid_distance_threshold=20,
                reid_hit_counter_max=30
            )
        except (TypeError, NameError):
            return Tracker(
                distance_function="euclidean",
                distance_threshold=self.norfair_distance_threshold,
                initialization_delay=1,
                hit_counter_max=15,
            )

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
        self.get_logger().info(f"Loaded calibration from {path}")

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

    def _extract_detections(self, image: np.ndarray) -> List[StereoDetection]:
        results = self.model.predict(
            source=image,
            conf=self.conf_thresh,
            imgsz=self.model_input_size,
            verbose=False,
            device=self.device_id,
        )
        if not results:
            return []

        boxes = results[0].boxes
        if boxes is None or boxes.xyxy is None:
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy() if boxes.conf is not None else np.ones(len(xyxy))

        detections: List[StereoDetection] = []
        for idx, box in enumerate(xyxy):
            x1, y1, x2, y2 = [float(v) for v in box[:4]]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            area = max(0.0, (x2 - x1) * (y2 - y1))
            detections.append(
                StereoDetection(
                    bbox_xyxy=(x1, y1, x2, y2),
                    center_x=cx,
                    center_y=cy,
                    confidence=float(conf[idx]),
                    bbox_area_px=float(area),
                )
            )
        return detections

    def _publish_tracks(self, tracked_objects) -> None:
        now = self.get_clock().now().to_msg()
        now_s = self.get_clock().now().nanoseconds / 1e9

        msg = TargetTrackArray()
        msg.stamp = now
        msg.tracks = []

        for obj in tracked_objects:
            estimate = np.array(obj.estimate, dtype=float).reshape(-1)
            if estimate.size < 3:
                continue
            track_id = int(obj.id) if obj.id is not None else -1
            pos = estimate[:3]

            if track_id in self.prev_positions:
                prev_pos, prev_t = self.prev_positions[track_id]
                dt = max(1e-3, now_s - prev_t)
                vel = (pos - prev_pos) / dt
            else:
                vel = np.zeros(3, dtype=float)
            self.prev_positions[track_id] = (pos.copy(), now_s)

            distance = float(np.linalg.norm(pos))
            radial_v = -float(np.dot(pos, vel)) / (distance + 1e-3)
            inbound = radial_v > 0.0

            confidence = 0.0
            bbox_area_px = 0.0
            if getattr(obj, "last_detection", None) is not None:
                data = getattr(obj.last_detection, "data", None)
                if isinstance(data, dict):
                    confidence = float(data.get("confidence", 0.0))
                    bbox_area_px = float(data.get("bbox_area_px", 0.0))

            track_msg = TargetTrack()
            track_msg.stamp = now
            track_msg.track_id = track_id
            track_msg.x_b_m = float(pos[0])
            track_msg.y_b_m = float(pos[1])
            track_msg.z_b_m = float(pos[2])
            track_msg.vx_b_mps = float(vel[0])
            track_msg.vy_b_mps = float(vel[1])
            track_msg.vz_b_mps = float(vel[2])
            track_msg.distance_m = distance
            track_msg.confidence = confidence
            track_msg.bbox_area_px = bbox_area_px
            track_msg.inbound = inbound
            msg.tracks.append(track_msg)

        self.tracks_pub.publish(msg)

    def _publish_perception_status(
        self,
        left_count: int,
        right_count: int,
        paired_count: int,
        active_tracks: int,
        inference_latency_ms: float,
    ) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.last_process_time_s is None:
            fps = 0.0
        else:
            fps = 1.0 / max(1e-3, now_s - self.last_process_time_s)
        self.last_process_time_s = now_s

        msg = PerceptionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.drone_id = self.drone_id
        msg.tracker_fps = float(fps)
        msg.inference_latency_ms = float(inference_latency_ms)
        msg.left_detections = int(left_count)
        msg.right_detections = int(right_count)
        msg.paired_detections = int(paired_count)
        msg.active_tracks = int(active_tracks)
        self.perception_status_pub.publish(msg)

    def get_hist(self, image):
        hist = cv2.calcHist(
            [cv2.cvtColor(image, cv2.COLOR_BGR2Lab)],
            [0, 1],
            None,
            [128, 128],
            [0, 256, 0, 256],
        )
        return cv2.normalize(hist, hist).flatten()

    def embedding_distance(self, matched_not_init_trackers, unmatched_trackers):
        snd_embedding = unmatched_trackers.last_detection.embedding

        if snd_embedding is None:
            for detection in reversed(unmatched_trackers.past_detections):
                if detection.embedding is not None:
                    snd_embedding = detection.embedding
                    break
            else:
                return 1

        for detection_fst in matched_not_init_trackers.past_detections:
            if detection_fst.embedding is None:
                continue

            distance = 1 - cv2.compareHist(
                snd_embedding, detection_fst.embedding, cv2.HISTCMP_CORREL
            )
            if distance < 0.5:
                return distance
        return 1
        
    def process_frame(self) -> None:
        start_t = time.perf_counter()
        ok_left, left = self.left_cap.read()
        ok_right, right = self.right_cap.read()
        if not ok_left or not ok_right:
            self.get_logger().warn("Stereo frame read failed, attempting reopen.")
            try:
                self.left_cap.release()
                self.right_cap.release()
            except Exception:
                pass
            self.left_cap = self._open_capture(self.left_sensor_id, self.left_url)
            self.right_cap = self._open_capture(self.right_sensor_id, self.right_url)
            return

        rect_left = cv2.remap(left, self.map1_x, self.map1_y, cv2.INTER_LINEAR)
        rect_right = cv2.remap(right, self.map2_x, self.map2_y, cv2.INTER_LINEAR)

        left_dets = self._extract_detections(rect_left)
        right_dets = self._extract_detections(rect_right)
        pairs = match_detections_epipolar(left_dets, right_dets, self.epipolar_tol)

        detections_3d = []
        for left_idx, right_idx in pairs:
            l_det = left_dets[left_idx]
            r_det = right_dets[right_idx]
            try:
                cam_point = triangulate_point(
                    self.p1,
                    self.p2,
                    (l_det.center_x, l_det.center_y),
                    (r_det.center_x, r_det.center_y),
                )
            except Exception:
                continue
            if not np.isfinite(cam_point).all() or cam_point[2] <= 0.0:
                continue

            body_point = camera_to_body(cam_point, self.rotation, self.translation)
            conf = min(l_det.confidence, r_det.confidence)
            area = max(l_det.bbox_area_px, r_det.bbox_area_px)
            detections_3d.append(
                Detection(
                    points=np.array(body_point, dtype=float),
                    scores=np.array([conf], dtype=float),
                    data={"confidence": conf, "bbox_area_px": area},
                )
            )

        for detection in detections_3d:
            cut = get_cutout(points=detection.points, image=rect_left)
            if cut.shape[0] > 0 and cut.shape[1] > 0:
                detection.embedding = self.get_hist(cut)
            else:
                detection.embedding = None

        coord_transform = None
        if self.motion_estimator is not None:
            coord_transform = self.motion_estimator.update(rect_left)
    
        if coord_transform is not None:
            tracked = self.tracker.update(detections_3d, coord_transformations=coord_transform)
        else:
            tracked = self.tracker.update(detections_3d)
        self._publish_tracks(tracked)
        self._publish_perception_status(
            left_count=len(left_dets),
            right_count=len(right_dets),
            paired_count=len(pairs),
            active_tracks=len(tracked),
            inference_latency_ms=(time.perf_counter() - start_t) * 1000.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.left_cap.release()
            node.right_cap.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
