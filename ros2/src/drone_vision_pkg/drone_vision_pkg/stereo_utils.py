from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import math
import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - platform specific
    cv2 = None

try:
    from scipy.optimize import linear_sum_assignment
except Exception:  # pragma: no cover - platform specific
    linear_sum_assignment = None


@dataclass
class StereoDetection:
    bbox_xyxy: Tuple[float, float, float, float]
    center_x: float
    center_y: float
    confidence: float
    bbox_area_px: float


def build_csi_pipeline(
    sensor_id: int,
    capture_width: int,
    capture_height: int,
    display_width: int,
    display_height: int,
    framerate: int,
    flip_method: int,
) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
    )


def rpy_to_rotation_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def match_detections_epipolar(
    left: Sequence[StereoDetection],
    right: Sequence[StereoDetection],
    y_tolerance_px: float,
) -> List[Tuple[int, int]]:
    if not left or not right:
        return []

    max_cost = 1e6
    cost = np.full((len(left), len(right)), max_cost, dtype=float)

    for i, l_det in enumerate(left):
        for j, r_det in enumerate(right):
            y_diff = abs(l_det.center_y - r_det.center_y)
            disparity = l_det.center_x - r_det.center_x
            if y_diff <= y_tolerance_px and disparity > 0.0:
                cost[i, j] = abs(disparity)

    if linear_sum_assignment is None:
        pairs: List[Tuple[int, int]] = []
        used_right = set()
        for i in range(len(left)):
            j = int(np.argmin(cost[i]))
            if cost[i, j] >= max_cost or j in used_right:
                continue
            pairs.append((i, j))
            used_right.add(j)
        return pairs

    row_idx, col_idx = linear_sum_assignment(cost)
    pairs = []
    for i, j in zip(row_idx, col_idx):
        if cost[i, j] >= max_cost:
            continue
        pairs.append((int(i), int(j)))
    return pairs


def triangulate_point(
    p1: np.ndarray, p2: np.ndarray, left_uv: Tuple[float, float], right_uv: Tuple[float, float]
) -> np.ndarray:
    if cv2 is None:
        raise RuntimeError("OpenCV is required for triangulation.")
    left_pt = np.array(left_uv, dtype=np.float64).reshape(2, 1)
    right_pt = np.array(right_uv, dtype=np.float64).reshape(2, 1)
    point_h = cv2.triangulatePoints(p1, p2, left_pt, right_pt)
    point_h /= point_h[3]
    return point_h[:3].reshape(3)


def camera_to_body(point_cam: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return rotation @ point_cam + translation


def normalize_quaternion(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    quat = np.array(quaternion_xyzw, dtype=float).reshape(4)
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def quaternion_to_rotation_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(quaternion_xyzw)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (yy + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    r = np.array(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(r))

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (r[2, 1] - r[1, 2]) / scale
        y = (r[0, 2] - r[2, 0]) / scale
        z = (r[1, 0] - r[0, 1]) / scale
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        scale = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / scale
        x = 0.25 * scale
        y = (r[0, 1] + r[1, 0]) / scale
        z = (r[0, 2] + r[2, 0]) / scale
    elif r[1, 1] > r[2, 2]:
        scale = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / scale
        x = (r[0, 1] + r[1, 0]) / scale
        y = 0.25 * scale
        z = (r[1, 2] + r[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / scale
        x = (r[0, 2] + r[2, 0]) / scale
        y = (r[1, 2] + r[2, 1]) / scale
        z = 0.25 * scale

    return normalize_quaternion([x, y, z, w])
