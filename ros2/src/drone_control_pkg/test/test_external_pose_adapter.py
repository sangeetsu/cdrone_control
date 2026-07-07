from __future__ import annotations

from math import pi
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.external_pose_adapter_node import (
    _quaternion_from_euler,
    _transform_pose_components,
)


def test_frame_yaw_pi_flips_x_and_y_position() -> None:
    position, orientation = _transform_pose_components(
        (11.75, 0.54, 0.08),
        _quaternion_from_euler(0.0, 0.0, 0.0),
        frame_offset_q=_quaternion_from_euler(0.0, 0.0, pi),
        position_offset_m=(0.0, 0.0, 0.0),
        orientation_offset_q=_quaternion_from_euler(0.0, 0.0, 0.0),
    )

    assert position[0] == pytest.approx(-11.75)
    assert position[1] == pytest.approx(-0.54)
    assert position[2] == pytest.approx(0.08)
    assert orientation[2] == pytest.approx(1.0)
    assert orientation[3] == pytest.approx(0.0, abs=1e-9)


def test_body_offset_is_applied_after_frame_rotation() -> None:
    position, _ = _transform_pose_components(
        (1.0, 2.0, 3.0),
        _quaternion_from_euler(0.0, 0.0, 0.0),
        frame_offset_q=_quaternion_from_euler(0.0, 0.0, pi),
        position_offset_m=(0.5, 0.0, 0.0),
        orientation_offset_q=_quaternion_from_euler(0.0, 0.0, 0.0),
    )

    assert position[0] == pytest.approx(-1.5)
    assert position[1] == pytest.approx(-2.0)
    assert position[2] == pytest.approx(3.0)
