from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.optical_flow_dead_reckon import (  # noqa: E402
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


def test_dead_reckon_rejects_missing_gyro_without_imu_coverage() -> None:
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

    assert not update.valid
    assert update.reject_reason == "gyro_integral_unavailable"
    assert update.pose.x_m == pytest.approx(1.0)
    assert update.pose.y_m == pytest.approx(2.0)


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


def test_stamped_flow_integrates_velocity_over_message_interval() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    first = reckoner.update(
        make_sample(
            integrated_y=0.02,
            integration_time_s=0.02,
            source_stamp_s=1.0,
        ),
        yaw_rad=0.0,
    )
    second = reckoner.update(
        make_sample(
            integrated_y=0.02,
            integration_time_s=0.02,
            source_stamp_s=1.1,
        ),
        yaw_rad=0.0,
    )

    assert first.valid and first.map_dx_m == pytest.approx(0.0)
    assert first.map_vx_mps == pytest.approx(1.0)
    assert second.valid and second.dt_s == pytest.approx(0.1)
    assert second.map_dx_m == pytest.approx(0.1)
    assert second.pose.x_m == pytest.approx(0.1)


def test_stamped_flow_uses_trapezoid_for_changing_velocity() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    reckoner.update(
        make_sample(
            integrated_y=0.02,
            integration_time_s=0.02,
            source_stamp_s=1.0,
        ),
        yaw_rad=0.0,
    )

    update = reckoner.update(
        make_sample(
            integrated_y=0.06,
            integration_time_s=0.02,
            source_stamp_s=1.1,
        ),
        yaw_rad=0.0,
    )

    assert update.map_vx_mps == pytest.approx(3.0)
    assert update.map_dx_m == pytest.approx(0.2)


def test_stamped_flow_bounds_large_message_gap() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map", max_flow_dt_s=0.25)
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    reckoner.update(
        make_sample(
            integrated_y=0.02,
            integration_time_s=0.02,
            source_stamp_s=1.0,
        ),
        yaw_rad=0.0,
    )

    update = reckoner.update(
        make_sample(
            integrated_y=0.02,
            integration_time_s=0.02,
            source_stamp_s=2.0,
        ),
        yaw_rad=0.0,
    )

    assert update.dt_s == pytest.approx(0.25)
    assert update.map_dx_m == pytest.approx(0.25)


def test_dead_reckoner_uses_synchronized_imu_gyro_fallback() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    for stamp in (0.98, 0.99, 1.0):
        assert reckoner.update_imu_gyro(
            stamp_s=stamp,
            angular_velocity_x=0.2,
            angular_velocity_y=0.0,
            angular_velocity_z=0.0,
        )

    update = reckoner.update(
        make_sample(
            integrated_x=0.004,
            integrated_xgyro=float("nan"),
            integrated_ygyro=float("nan"),
            integrated_zgyro=float("nan"),
            source_stamp_s=1.0,
            integration_time_s=0.02,
        ),
        yaw_rad=0.0,
    )

    assert update.valid
    assert update.gyro_source == "imu_buffer"
    assert update.map_vy_mps == pytest.approx(0.0, abs=1e-10)


def test_dead_reckoner_rejects_duplicate_source_stamp() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map")
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    assert reckoner.update(
        make_sample(source_stamp_s=1.0), yaw_rad=0.0
    ).valid

    update = reckoner.update(make_sample(source_stamp_s=1.0), yaw_rad=0.0)

    assert not update.valid
    assert update.reject_reason == "flow_out_of_order"


def test_dead_reckoner_does_not_bridge_rejected_interval() -> None:
    reckoner = OpticalFlowDeadReckoner(frame_id="map", quality_min=50)
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    reckoner.update(
        make_sample(
            integrated_y=0.02,
            source_stamp_s=1.0,
            integration_time_s=0.02,
        ),
        yaw_rad=0.0,
    )
    rejected = reckoner.update(
        make_sample(quality=1, source_stamp_s=1.1),
        yaw_rad=0.0,
    )

    recovered = reckoner.update(
        make_sample(
            integrated_y=0.02,
            source_stamp_s=1.2,
            integration_time_s=0.02,
        ),
        yaw_rad=0.0,
    )

    assert not rejected.valid
    assert recovered.valid
    assert recovered.dt_s == pytest.approx(0.0)
    assert recovered.map_dx_m == pytest.approx(0.0)


def test_dead_reckoner_applies_sensor_yaw_extrinsic() -> None:
    reckoner = OpticalFlowDeadReckoner(
        frame_id="map", flow_sensor_yaw_rad=math.pi / 2.0
    )
    reckoner.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = reckoner.update(
        make_sample(integrated_y=0.02, integration_time_s=0.02),
        yaw_rad=0.0,
    )

    assert update.map_dx_m == pytest.approx(0.0, abs=1e-10)
    assert update.map_dy_m == pytest.approx(0.02)
