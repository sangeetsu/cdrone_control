from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg.realsense_tracker_node import (
    compute_detection_depth,
    select_yolo_boxes,
    yolo_boxes_to_detections,
)


class Tensor:
    def __init__(self, value):
        self.value = np.asarray(value, dtype=np.float32)

    def cpu(self):
        return self

    def numpy(self):
        return self.value


def test_obb_output_becomes_norfair_detection_with_polygon():
    obb = SimpleNamespace(
        xyxy=Tensor([[10, 10, 30, 30]]),
        xyxyxyxy=Tensor([[[20, 10], [30, 20], [20, 30], [10, 20]]]),
        conf=Tensor([0.87]),
    )
    boxes = select_yolo_boxes(SimpleNamespace(obb=obb, boxes=None), "obb")
    detections = yolo_boxes_to_detections(boxes)
    assert len(detections) == 1
    assert detections[0].points.tolist() == [[20.0, 20.0]]
    assert detections[0].data["obb_polygon"] == [
        [20.0, 10.0], [30.0, 20.0], [20.0, 30.0], [10.0, 20.0]]


def test_wrong_engine_task_fails_closed():
    with pytest.raises(RuntimeError, match="no obb output"):
        select_yolo_boxes(SimpleNamespace(obb=None, boxes=object()), "obb")


def test_obb_depth_excludes_far_background_and_invalid_pixels():
    depth = np.full((50, 50), 24.0, dtype=np.float32)
    depth[19:32, 19:32] = 18.0
    depth[23, 23] = np.nan
    measured = compute_detection_depth(
        depth, np.array([25.0, 25.0]), frame_shape=(50, 50, 3),
        bbox=[15, 15, 35, 35],
        obb_polygon=[[25, 15], [35, 25], [25, 35], [15, 25]],
        radius_px=5, max_valid_depth_m=25.0)
    assert measured == pytest.approx(18.0)


def test_obb_depth_does_not_substitute_background_when_target_depth_is_missing():
    depth = np.full((50, 50), 24.0, dtype=np.float32)
    depth[15:36, 15:36] = np.nan
    measured = compute_detection_depth(
        depth, np.array([25.0, 25.0]), frame_shape=(50, 50, 3),
        bbox=[15, 15, 35, 35],
        obb_polygon=[[25, 15], [35, 25], [25, 35], [15, 25]],
        radius_px=5, max_valid_depth_m=25.0)
    assert measured == 0.0
