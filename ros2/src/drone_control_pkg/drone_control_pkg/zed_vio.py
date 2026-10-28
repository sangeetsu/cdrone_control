"""ZED camera-to-airframe pose conversion and fresh-frame gating."""

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


@dataclass(frozen=True)
class Mount:
    # Camera origin and orientation relative to base_link (X forward, Y left, Z up).
    body_to_camera_translation_m: tuple[float, float, float]
    body_to_camera_rpy_rad: tuple[float, float, float]


def load_mount(path: str | Path) -> Mount | None:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if data.get("calibrated") is not True:
        return None
    values = []
    for name in ("body_to_camera_translation_m", "body_to_camera_rpy_rad"):
        entry = data.get(name)
        if not isinstance(entry, (list, tuple)) or len(entry) != 3:
            raise ValueError(f"{name} must contain three measured values")
        parsed = tuple(float(value) for value in entry)
        if not all(math.isfinite(value) for value in parsed):
            raise ValueError(f"{name} must contain finite values")
        values.append(parsed)
    return Mount(*values)


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def transform_camera_to_body(position, quaternion_xyzw, covariance, mount: Mount):
    """Return airframe-center pose/covariance in the ZED world frame.

    ZED covariance is treated as a world-frame small-angle pose covariance.
    The SE(3) Jacobian includes the camera lever arm.
    """
    p_wc = np.asarray(position, dtype=float)
    q_wc = np.asarray(quaternion_xyzw, dtype=float)
    cov = np.asarray(covariance, dtype=float).reshape(6, 6)
    if (p_wc.shape != (3,) or q_wc.shape != (4,) or not np.isfinite(p_wc).all()
            or not np.isfinite(q_wc).all() or abs(np.linalg.norm(q_wc) - 1) > 0.1
            or not np.isfinite(cov).all() or np.any(np.diag(cov) < 0)):
        raise ValueError("invalid ZED pose or covariance")
    camera_in_body = Rotation.from_euler("xyz", mount.body_to_camera_rpy_rad)
    body_in_world = Rotation.from_quat(q_wc / np.linalg.norm(q_wc)) * camera_in_body.inv()
    lever_world = body_in_world.apply(mount.body_to_camera_translation_m)
    p_wb = p_wc - lever_world
    jacobian = np.eye(6)
    jacobian[:3, 3:] = _skew(lever_world)
    cov_body = jacobian @ ((cov + cov.T) / 2) @ jacobian.T
    return p_wb, body_in_world.as_quat(), cov_body


class PoseGate:
    def __init__(self, max_age_s=0.3, max_jump_m=0.5, max_yaw_jump_rad=0.52):
        self.max_age_ns = int(max_age_s * 1e9)
        self.max_jump_m = max_jump_m
        self.max_yaw_jump_rad = max_yaw_jump_rad
        self.last_stamp_ns = 0
        self.last_position = None
        self.last_quaternion = None
        self.last_jump = None

    def accept(self, tracking_state, stamp_ns, now_ns, position, quaternion):
        self.last_jump = None
        if tracking_state != "OK":
            return False, f"tracking state {tracking_state}"
        if stamp_ns <= 0:
            return False, "invalid camera frame timestamp"
        if stamp_ns <= self.last_stamp_ns:
            return False, "repeated camera frame"
        age_ns = now_ns - stamp_ns
        if age_ns < -100_000_000:
            return False, "future camera frame"
        if age_ns > self.max_age_ns:
            return False, "stale camera frame"
        if self.last_position is not None:
            distance = float(np.linalg.norm(position - self.last_position))
            rotation = (Rotation.from_quat(self.last_quaternion).inv()
                        * Rotation.from_quat(quaternion))
            yaw_change_rad = float(rotation.magnitude())
            if distance > self.max_jump_m or yaw_change_rad > self.max_yaw_jump_rad:
                previous_yaw = Rotation.from_quat(self.last_quaternion).as_euler("xyz")[2]
                current_yaw = Rotation.from_quat(quaternion).as_euler("xyz")[2]
                heading_step = math.atan2(math.sin(current_yaw - previous_yaw),
                                          math.cos(current_yaw - previous_yaw))
                self.last_jump = {
                    "translation_m": distance,
                    "rotation_rad": yaw_change_rad,
                    "yaw_change_rad": abs(heading_step),
                    "frame_interval_s": (stamp_ns - self.last_stamp_ns) / 1e9,
                }
                return False, "ZED pose jump"
        self.last_stamp_ns = stamp_ns
        self.last_position = np.array(position, copy=True)
        self.last_quaternion = np.array(quaternion, copy=True)
        return True, "OK"
