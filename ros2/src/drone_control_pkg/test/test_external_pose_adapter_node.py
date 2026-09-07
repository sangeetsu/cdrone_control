from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.external_pose_adapter_node import (
    _quaternion_from_euler,
    transform_pose_components,
)
from drone_control_pkg.perimeter_utils import PerimeterGuard


def test_transform_pose_components_preserves_zero_frame_transform() -> None:
    position, orientation = transform_pose_components(
        (1.0, 2.0, 0.5),
        (0.0, 0.0, 0.0, 1.0),
        frame_q=_quaternion_from_euler(0.0, 0.0, 0.0),
        position_offset_m=(0.1, -0.2, 0.3),
        orientation_offset_q=_quaternion_from_euler(0.0, 0.0, 0.0),
    )

    assert position == pytest.approx((1.1, 1.8, 0.8))
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_transform_pose_components_rotates_optitrack_xy_into_studio_boundary() -> None:
    position, orientation = transform_pose_components(
        (11.065, -0.535, 0.105),
        (0.0, 0.0, 0.0, 1.0),
        frame_q=_quaternion_from_euler(0.0, 0.0, math.pi),
        position_offset_m=(0.0, 0.0, 0.0),
        orientation_offset_q=_quaternion_from_euler(0.0, 0.0, 0.0),
    )
    perimeter = PerimeterGuard.load_from_yaml(
        Path(__file__).resolve().parents[2]
        / "drone_bringup"
        / "config"
        / "drone_studio_perimeter.yaml"
    )

    assert position == pytest.approx((-11.065, 0.535, 0.105), abs=1e-6)
    assert orientation[2] == pytest.approx(1.0)
    assert orientation[3] == pytest.approx(0.0, abs=1e-6)
    assert (
        perimeter.xy_violation_reason(
            position[0],
            position[1],
            boundary_tolerance_m=0.05,
            boundary_margin_m=0.8,
            keep_out_margin_m=0.3,
        )
        is None
    )


def test_live_unrotated_pose_is_outside_studio_boundary() -> None:
    perimeter = PerimeterGuard.load_from_yaml(
        Path(__file__).resolve().parents[2]
        / "drone_bringup"
        / "config"
        / "drone_studio_perimeter.yaml"
    )

    assert (
        perimeter.xy_violation_reason(
            11.065,
            -0.535,
            boundary_tolerance_m=0.05,
            boundary_margin_m=0.8,
            keep_out_margin_m=0.3,
        )
        == "outside studio boundary"
    )
