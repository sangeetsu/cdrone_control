from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo


OPTICAL_TO_FLU = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def _normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float]:
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def _quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    qx, qy, qz, qw = _normalize_quaternion(x, y, z, w)
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
        dtype=np.float64,
    )
    ry = np.array(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
        dtype=np.float64,
    )
    rz = np.array(
        [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return rz @ ry @ rx


class RealsenseProjection:
    def __init__(
        self,
        camera_offset_body_m: list[float],
        camera_rpy_body_rad: list[float],
        world_frame: str,
    ) -> None:
        self.world_frame = str(world_frame or "").strip()
        self.pose_frame_id = self.world_frame or "map"
        self.intrinsics: CameraIntrinsics | None = None
        self.drone_position_m: np.ndarray | None = None
        self.world_from_body_rotation: np.ndarray | None = None
        self.camera_offset_body_m = np.asarray(camera_offset_body_m, dtype=np.float64)
        mount_rotation = _rotation_matrix_from_rpy(*camera_rpy_body_rad)
        self.camera_to_body_rotation = mount_rotation @ OPTICAL_TO_FLU

    @property
    def frame_id(self) -> str:
        return self.world_frame or self.pose_frame_id or "map"

    def has_intrinsics(self) -> bool:
        return self.intrinsics is not None

    def has_pose(self) -> bool:
        return (
            self.drone_position_m is not None
            and self.world_from_body_rotation is not None
        )

    def set_intrinsics(self, fx: float, fy: float, cx: float, cy: float) -> None:
        if fx <= 0.0 or fy <= 0.0:
            return
        self.intrinsics = CameraIntrinsics(
            fx=float(fx),
            fy=float(fy),
            cx=float(cx),
            cy=float(cy),
        )

    def update_camera_info(self, msg: CameraInfo) -> None:
        fx = float(msg.k[0]) if len(msg.k) >= 9 and msg.k[0] > 0.0 else float(msg.p[0])
        fy = float(msg.k[4]) if len(msg.k) >= 9 and msg.k[4] > 0.0 else float(msg.p[5])
        cx = float(msg.k[2]) if len(msg.k) >= 9 else float(msg.p[2])
        cy = float(msg.k[5]) if len(msg.k) >= 9 else float(msg.p[6])
        self.set_intrinsics(fx=fx, fy=fy, cx=cx, cy=cy)

    def update_pose(self, msg: PoseStamped) -> None:
        self.pose_frame_id = str(msg.header.frame_id).strip() or self.frame_id
        self.drone_position_m = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        self.world_from_body_rotation = _quaternion_to_rotation_matrix(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    def pixel_depth_to_camera_frame(
        self,
        x_px: float,
        y_px: float,
        depth_m: float,
    ) -> np.ndarray | None:
        if depth_m <= 0.0 or self.intrinsics is None:
            return None
        intrinsics = self.intrinsics
        x_cam = (x_px - intrinsics.cx) * depth_m / intrinsics.fx
        y_cam = (y_px - intrinsics.cy) * depth_m / intrinsics.fy
        return np.array([x_cam, y_cam, depth_m], dtype=np.float64)

    def camera_to_body_frame(self, camera_point_m: np.ndarray) -> np.ndarray:
        return self.camera_to_body_rotation @ camera_point_m + self.camera_offset_body_m

    def body_to_camera_frame(self, body_point_m: np.ndarray) -> np.ndarray:
        centered_body_point = np.asarray(body_point_m, dtype=np.float64)
        centered_body_point = centered_body_point - self.camera_offset_body_m
        return self.camera_to_body_rotation.T @ centered_body_point

    def pixel_depth_to_body(
        self,
        x_px: float,
        y_px: float,
        depth_m: float,
    ) -> np.ndarray | None:
        camera_point_m = self.pixel_depth_to_camera_frame(x_px, y_px, depth_m)
        if camera_point_m is None:
            return None
        return self.camera_to_body_frame(camera_point_m)

    def pixel_depth_to_world(
        self,
        x_px: float,
        y_px: float,
        depth_m: float,
    ) -> np.ndarray | None:
        body_point_m = self.pixel_depth_to_body(x_px, y_px, depth_m)
        if body_point_m is None or not self.has_pose():
            return None
        return self.body_to_world(body_point_m)

    def body_to_world(self, body_point_m: np.ndarray) -> np.ndarray | None:
        if not self.has_pose():
            return None
        return self.drone_position_m + self.world_from_body_rotation @ body_point_m

    def world_to_body(self, world_point_m: np.ndarray) -> np.ndarray | None:
        if not self.has_pose():
            return None
        centered_world_point = np.asarray(world_point_m, dtype=np.float64)
        centered_world_point = centered_world_point - self.drone_position_m
        return self.world_from_body_rotation.T @ centered_world_point

    def world_to_camera(self, world_point_m: np.ndarray) -> np.ndarray | None:
        body_point_m = self.world_to_body(world_point_m)
        if body_point_m is None:
            return None
        return self.body_to_camera_frame(body_point_m)
