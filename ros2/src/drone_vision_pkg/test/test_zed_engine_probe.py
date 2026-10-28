"""Hardware-free checks for the isolated camera/engine diagnostic."""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "scripts"))

from zed_engine_probe import (  # noqa: E402
    Frame,
    Intrinsics,
    decode_obb,
    load_mount_transform,
    passes_ten_hz_target_gate,
    pixel_to_optical_m,
    rate_summary,
    sample_obb_depth_m,
)


class Tensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def cpu(self):
        return self

    def numpy(self):
        return self.value


def test_obb_depth_uses_target_interior_and_rejects_invalid_background():
    depth = np.full((60, 60), 24.0, dtype=np.float32)
    depth[20:41, 20:41] = 12.0
    depth[29, 29] = np.nan
    polygon = np.array([[20, 20], [40, 20], [40, 40], [20, 40]])
    assert sample_obb_depth_m(depth, polygon, 20.0) == pytest.approx(12.0)
    depth[20:41, 20:41] = np.nan
    assert sample_obb_depth_m(depth, polygon, 20.0) is None


def test_camera_center_projects_to_forward_body_coordinate(tmp_path):
    mount_file = tmp_path / "mount.yaml"
    mount_file.write_text(
        "calibrated: true\n"
        "body_to_camera_translation_m: [0.15, -0.06, -0.03]\n"
        "body_to_camera_rpy_rad: [0, 0, 0]\n", encoding="utf-8")
    rotation, offset = load_mount_transform(mount_file, rotate_180=False)
    optical = pixel_to_optical_m(50, 50, 5.0, Intrinsics(100, 100, 50, 50))
    np.testing.assert_allclose(optical, [0, 0, 5])
    np.testing.assert_allclose(rotation @ optical + offset, [5.15, -0.06, -0.03])


def test_obb_result_yields_metric_coordinates_and_rejects_wrong_task():
    depth = np.full((100, 100), 6.0, dtype=np.float32)
    frame = Frame(np.zeros((100, 100, 3), dtype=np.uint8), depth, 123)
    obb = SimpleNamespace(
        xyxyxyxy=Tensor([[[45, 45], [55, 45], [55, 55], [45, 55]]]),
        conf=Tensor([0.9]), cls=Tensor([0]))
    result = SimpleNamespace(obb=obb, names={0: "drone"})
    detections = decode_obb(result, frame, Intrinsics(100, 100, 50, 50),
                            (np.eye(3), np.zeros(3)), 20.0)
    assert len(detections) == 1
    assert detections[0]["class_name"] == "drone"
    assert detections[0]["depth_m"] == pytest.approx(6.0)
    assert detections[0]["body_up_m"] == pytest.approx(6.0)
    with pytest.raises(RuntimeError, match="no OBB"):
        decode_obb(SimpleNamespace(obb=None), frame,
                   Intrinsics(100, 100, 50, 50), None, 20.0)


def test_ten_hz_gate_uses_valid_3d_updates_and_max_gap():
    ten_hz = rate_summary([i * 0.1 for i in range(100)], 10.0)
    assert passes_ten_hz_target_gate(ten_hz, 0.9)
    assert not passes_ten_hz_target_gate(ten_hz, 0.89)
    assert not passes_ten_hz_target_gate(rate_summary([0, 1, 2], 3.0), 1.0)
    assert not passes_ten_hz_target_gate(
        rate_summary([i * 0.05 for i in range(90)] + [5.0] +
                     [5.0 + i * 0.05 for i in range(1, 90)], 10.0), 1.0)
    assert not passes_ten_hz_target_gate(
        rate_summary([i * 0.05 for i in range(200)], 20.0), 1.0)
