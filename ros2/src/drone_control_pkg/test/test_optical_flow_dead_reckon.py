from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.optical_flow_dead_reckon import (
    OpticalFlowDeadReckoner,
    OpticalFlowSample,
)


def make_sample(**overrides) -> OpticalFlowSample:
    values = {
        "integrated_x": 0.0,
        "integrated_y": 0.0,
        "integrated_xgyro": 0.0,
        "integrated_ygyro": 0.0,
        "integrated_zgyro": 0.0,
        "quality": 100,
        "distance_m": 1.0,
        "integration_time_s": 0.02,
    }
    values.update(overrides)
    return OpticalFlowSample(**values)


def test_dead_reckon_maps_flow_to_body_xy() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=1.0, y_m=2.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = reckoner.update(
        make_sample(integrated_x=0.2, integrated_y=0.1),
        yaw_rad=0.0,
    )

    assert update.valid
    assert update.body_dx_m == pytest.approx(0.1)
    assert update.body_dy_m == pytest.approx(-0.2)
    assert update.pose.x_m == pytest.approx(1.1)
    assert update.pose.y_m == pytest.approx(1.8)


def test_dead_reckon_rotates_body_motion_with_mocap_yaw() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = reckoner.update(
        make_sample(integrated_y=0.2),
        yaw_rad=math.pi / 2.0,
    )

    assert update.map_dx_m == pytest.approx(0.0, abs=1e-9)
    assert update.map_dy_m == pytest.approx(0.2)
    assert update.pose.x_m == pytest.approx(0.0, abs=1e-9)
    assert update.pose.y_m == pytest.approx(0.2)


def test_dead_reckon_rejects_low_quality_without_xy_integration() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map", quality_min=50)
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = reckoner.update(
        make_sample(integrated_y=1.0, quality=10),
        yaw_rad=0.0,
    )

    assert not update.valid
    assert update.reject_reason == "quality_below_min"
    assert update.pose.x_m == pytest.approx(0.0)
    assert update.pose.y_m == pytest.approx(0.0)


def test_dead_reckon_uses_raw_flow_when_gyro_terms_are_nan() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=1.0, y_m=2.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = reckoner.update(
        make_sample(
            integrated_x=0.2,
            integrated_y=0.1,
            integrated_xgyro=float("nan"),
            integrated_ygyro=float("nan"),
            integrated_zgyro=float("nan"),
        ),
        yaw_rad=0.0,
    )

    assert update.valid
    assert update.body_dx_m == pytest.approx(0.1)
    assert update.body_dy_m == pytest.approx(-0.2)
    assert update.pose.x_m == pytest.approx(1.1)
    assert update.pose.y_m == pytest.approx(1.8)


def test_dead_reckon_rejects_nonfinite_flow_without_nan_pose() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=1.0, y_m=2.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = reckoner.update(
        make_sample(integrated_x=float("nan"), integrated_y=0.1),
        yaw_rad=0.0,
    )

    assert not update.valid
    assert update.reject_reason == "flow_x_not_finite"
    assert update.pose.x_m == pytest.approx(1.0)
    assert update.pose.y_m == pytest.approx(2.0)
