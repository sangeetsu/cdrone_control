from pathlib import Path
import math
import sys

import pytest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.imu_optical_flow_fusion import (  # noqa: E402
    ImuOpticalFlowFusion,
    ImuSample,
)
from drone_control_pkg.optical_flow_dead_reckon import (  # noqa: E402
    OpticalFlowSample,
)


def make_imu(**overrides) -> ImuSample:
    values = {
        "stamp_s": 1.0,
        "orientation_x": 0.0,
        "orientation_y": 0.0,
        "orientation_z": 0.0,
        "orientation_w": 1.0,
        "angular_velocity_x": 0.0,
        "angular_velocity_y": 0.0,
        "angular_velocity_z": 0.0,
        "linear_acceleration_x": 0.0,
        "linear_acceleration_y": 0.0,
        "linear_acceleration_z": 9.80665,
    }
    values.update(overrides)
    return ImuSample(**values)


def make_flow(**overrides) -> OpticalFlowSample:
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


def test_reset_uses_mocap_start_once() -> None:
    fusion = ImuOpticalFlowFusion(frame_id="map")

    pose = fusion.reset(
        x_m=1.0,
        y_m=2.0,
        z_m=1.5,
        yaw_rad=0.25,
        range_m=1.0,
    )

    assert pose.x_m == pytest.approx(1.0)
    assert pose.y_m == pytest.approx(2.0)
    assert pose.z_m == pytest.approx(1.5)

    fusion.update_flow(make_flow(integrated_y=0.02))

    assert fusion.pose is not None
    assert fusion.pose.x_m != pytest.approx(1.0)
    assert fusion.origin_pose is not None
    assert fusion.origin_pose.x_m == pytest.approx(1.0)


def test_stationary_imu_does_not_drift_after_gravity_compensation() -> None:
    fusion = ImuOpticalFlowFusion(frame_id="map")
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    fusion.update_imu(make_imu(stamp_s=1.0))
    for index in range(2, 12):
        update = fusion.update_imu(make_imu(stamp_s=1.0 + (index * 0.01)))

    assert update.valid
    assert fusion.pose is not None
    assert fusion.pose.x_m == pytest.approx(0.0, abs=1e-6)
    assert fusion.pose.y_m == pytest.approx(0.0, abs=1e-6)
    assert fusion.pose.z_m == pytest.approx(1.0, abs=1e-6)


def test_flow_corrects_horizontal_position_and_velocity() -> None:
    fusion = ImuOpticalFlowFusion(
        frame_id="map",
        flow_position_weight=1.0,
        flow_velocity_weight=1.0,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = fusion.update_flow(
        make_flow(integrated_y=0.10, integration_time_s=0.10)
    )

    assert update.valid
    assert update.body_dx_m == pytest.approx(0.10)
    assert update.map_dx_m == pytest.approx(0.10)
    assert fusion.pose is not None
    assert fusion.pose.x_m == pytest.approx(0.10)
    assert fusion.pose.vx_mps == pytest.approx(1.0)


def test_low_quality_flow_is_rejected_without_nan_pose() -> None:
    fusion = ImuOpticalFlowFusion(frame_id="map", quality_min=50)
    fusion.reset(x_m=1.0, y_m=2.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = fusion.update_flow(make_flow(integrated_y=1.0, quality=10))

    assert not update.valid
    assert update.reject_reason == "quality_below_min"
    assert fusion.pose is not None
    assert fusion.pose.x_m == pytest.approx(1.0)
    assert fusion.pose.y_m == pytest.approx(2.0)


def test_imu_yaw_rotates_flow_measurement() -> None:
    fusion = ImuOpticalFlowFusion(
        frame_id="map",
        flow_position_weight=1.0,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    half_yaw = math.pi / 4.0
    fusion.update_imu(
        make_imu(
            stamp_s=1.0,
            orientation_z=math.sin(half_yaw),
            orientation_w=math.cos(half_yaw),
        )
    )
    update = fusion.update_flow(make_flow(integrated_y=0.02))

    assert update.valid
    assert update.map_dx_m == pytest.approx(0.0, abs=1e-9)
    assert update.map_dy_m == pytest.approx(0.02)


def test_ekf_state_and_covariance_are_exposed_as_defensive_copies() -> None:
    fusion = ImuOpticalFlowFusion()
    fusion.reset(x_m=1.0, y_m=2.0, z_m=3.0, yaw_rad=0.0)

    state = fusion.state
    covariance = fusion.covariance

    assert state is not None and state.shape == (9,)
    assert covariance is not None and covariance.shape == (9, 9)
    assert np.all(np.diag(covariance) > 0.0)
    state[0] = 100.0
    covariance[0, 0] = 100.0
    assert fusion.state is not None and fusion.state[0] == pytest.approx(1.0)
    assert fusion.covariance is not None
    assert fusion.covariance[0, 0] != pytest.approx(100.0)


def test_imu_only_constant_acceleration_matches_kinematics() -> None:
    fusion = ImuOpticalFlowFusion(
        accel_lpf_tau_s=0.0,
        accel_deadband_mps2=0.0,
        velocity_decay_per_s=0.0,
        max_accel_mps2=20.0,
        max_velocity_mps=20.0,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0)
    fusion.update_imu(make_imu(stamp_s=0.0, linear_acceleration_x=1.0))
    for index in range(1, 101):
        update = fusion.update_imu(
            make_imu(stamp_s=index * 0.01, linear_acceleration_x=1.0)
        )

    assert update.valid
    assert update.health_state == "imu_only"
    assert fusion.pose is not None
    assert fusion.pose.x_m == pytest.approx(0.5, abs=1e-8)
    assert fusion.pose.vx_mps == pytest.approx(1.0, abs=1e-8)


def test_stationary_flow_updates_estimate_accelerometer_bias() -> None:
    fusion = ImuOpticalFlowFusion(
        accel_lpf_tau_s=0.0,
        accel_deadband_mps2=0.0,
        velocity_decay_per_s=0.0,
        max_velocity_mps=20.0,
        accel_noise_mps2=0.1,
        flow_velocity_noise_mps=0.03,
        flow_quality_noise_scale=0.0,
        flow_range_noise_scale=0.0,
        flow_gap_noise_scale=0.0,
        innovation_gate_nis=1e6,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    for index in range(201):
        stamp = index * 0.01
        fusion.update_imu(
            make_imu(stamp_s=stamp, linear_acceleration_x=0.2)
        )
        if index and index % 10 == 0:
            fusion.update_flow(
                make_flow(
                    quality=255,
                    source_stamp_s=stamp,
                    integration_time_s=0.02,
                )
            )

    state = fusion.state
    assert state is not None
    assert state[6] == pytest.approx(0.2, abs=0.01)
    assert fusion.pose is not None
    assert fusion.pose.vx_mps == pytest.approx(0.0, abs=0.01)


def test_flow_noise_increases_as_quality_falls_and_gap_grows() -> None:
    def variance(quality: int, second_stamp: float) -> float:
        fusion = ImuOpticalFlowFusion(
            flow_velocity_noise_mps=0.1,
            flow_velocity_weight=0.5,
            innovation_gate_nis=1e6,
        )
        fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
        fusion.update_flow(
            make_flow(
                quality=quality,
                source_stamp_s=1.0,
                integration_time_s=0.02,
            )
        )
        update = fusion.update_flow(
            make_flow(
                quality=quality,
                source_stamp_s=second_stamp,
                integration_time_s=0.02,
            )
        )
        assert update.valid
        return update.measurement_variance_x

    best = variance(255, 1.02)
    low_quality = variance(50, 1.02)
    sparse = variance(255, 1.10)

    assert low_quality > best
    assert sparse > best


def test_missing_flow_gyro_uses_synchronized_imu_exposure() -> None:
    fusion = ImuOpticalFlowFusion(
        accel_lpf_tau_s=0.0,
        innovation_gate_nis=1e6,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    for stamp in (0.98, 0.99, 1.00):
        fusion.update_imu(
            make_imu(stamp_s=stamp, angular_velocity_x=0.2)
        )

    update = fusion.update_flow(
        make_flow(
            integrated_x=0.004,
            integrated_xgyro=float("nan"),
            integrated_ygyro=float("nan"),
            integrated_zgyro=float("nan"),
            source_stamp_s=1.0,
            integration_time_s=0.02,
        )
    )

    assert update.valid
    assert update.gyro_source == "imu_buffer"
    assert update.flow_vy_mps == pytest.approx(0.0, abs=1e-10)


def test_missing_flow_gyro_without_coverage_is_rejected() -> None:
    fusion = ImuOpticalFlowFusion()
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    fusion.update_imu(make_imu(stamp_s=1.0))

    update = fusion.update_flow(
        make_flow(
            integrated_xgyro=float("nan"),
            integrated_ygyro=float("nan"),
            source_stamp_s=1.0,
            integration_time_s=0.02,
        )
    )

    assert not update.valid
    assert update.reject_reason == "gyro_integral_unavailable"


def test_duplicate_and_stale_events_are_rejected() -> None:
    fusion = ImuOpticalFlowFusion(max_sensor_age_s=0.1)
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    stale = fusion.update_imu(
        make_imu(stamp_s=1.0, source_stamp_s=1.0, receive_stamp_s=1.2)
    )
    assert not stale.valid and stale.reject_reason == "sensor_stale"

    assert fusion.update_imu(make_imu(stamp_s=1.0)).valid
    duplicate = fusion.update_imu(make_imu(stamp_s=1.0))
    assert not duplicate.valid and duplicate.reject_reason == "imu_duplicate"

    assert fusion.update_flow(make_flow(source_stamp_s=1.0)).valid
    flow_duplicate = fusion.update_flow(make_flow(source_stamp_s=1.0))
    assert not flow_duplicate.valid
    assert flow_duplicate.reject_reason == "flow_duplicate"


def test_flow_older_than_propagated_imu_state_is_rejected() -> None:
    fusion = ImuOpticalFlowFusion(reorder_tolerance_s=0.02)
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    fusion.update_imu(make_imu(stamp_s=1.10))

    update = fusion.update_flow(make_flow(source_stamp_s=1.07))

    assert not update.valid
    assert update.reject_reason == "flow_state_skew"


def test_long_flow_gap_rejects_one_update_then_recovers() -> None:
    fusion = ImuOpticalFlowFusion(max_flow_gap_s=0.2)
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    assert fusion.update_flow(make_flow(source_stamp_s=1.0)).valid
    gap = fusion.update_flow(make_flow(source_stamp_s=1.5))
    recovered = fusion.update_flow(make_flow(source_stamp_s=1.6))

    assert not gap.valid and gap.reject_reason == "flow_gap_exceeded"
    assert recovered.valid


def test_velocity_outlier_is_rejected_by_nis_gate() -> None:
    fusion = ImuOpticalFlowFusion(
        max_velocity_mps=100.0,
        flow_velocity_noise_mps=0.01,
        innovation_gate_nis=6.0,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = fusion.update_flow(
        make_flow(integrated_y=1.0, integration_time_s=0.02)
    )

    assert not update.valid
    assert update.reject_reason == "innovation_gate"
    assert update.nis > fusion.innovation_gate_nis


def test_tilt_gate_rejects_flow() -> None:
    fusion = ImuOpticalFlowFusion(max_tilt_rad=math.radians(30.0))
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    half_roll = math.radians(60.0) / 2.0
    fusion.update_imu(
        make_imu(
            stamp_s=1.0,
            orientation_x=math.sin(half_roll),
            orientation_w=math.cos(half_roll),
        )
    )

    update = fusion.update_flow(make_flow())

    assert not update.valid
    assert update.reject_reason == "tilt_above_max"


def test_sensor_extrinsic_rotates_flow_velocity() -> None:
    fusion = ImuOpticalFlowFusion(
        flow_sensor_yaw_rad=math.pi / 2.0,
        flow_velocity_weight=1.0,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = fusion.update_flow(
        make_flow(integrated_y=0.02, integration_time_s=0.02)
    )

    assert update.valid
    assert update.map_dx_m == pytest.approx(0.0, abs=1e-10)
    assert update.map_dy_m == pytest.approx(0.02)
    assert update.flow_vx_mps == pytest.approx(0.0, abs=1e-10)
    assert update.flow_vy_mps == pytest.approx(1.0)


def test_range_update_remains_available_when_flow_quality_is_bad() -> None:
    fusion = ImuOpticalFlowFusion(quality_min=200)
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)

    update = fusion.update_flow(make_flow(quality=10, distance_m=1.5))

    assert not update.valid
    assert update.reject_reason == "quality_below_min"
    assert fusion.pose is not None and fusion.pose.z_m > 1.4
    assert math.isfinite(update.measurement_variance_z)


def test_range_update_projects_slant_range_through_current_tilt() -> None:
    fusion = ImuOpticalFlowFusion(
        quality_min=200,
        max_tilt_rad=math.radians(80.0),
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    half_roll = math.radians(60.0) / 2.0
    fusion.update_imu(
        make_imu(
            stamp_s=1.0,
            orientation_x=math.sin(half_roll),
            orientation_w=math.cos(half_roll),
            linear_acceleration_y=9.80665 * math.sin(math.radians(60.0)),
            linear_acceleration_z=9.80665 * math.cos(math.radians(60.0)),
        )
    )

    fusion.update_flow(make_flow(quality=10, distance_m=2.0))

    # A 2 m body-Z slant range at 60 degrees has 1 m vertical projection,
    # exactly matching the reset range and therefore preserving map Z.
    assert fusion.pose is not None
    assert fusion.pose.z_m == pytest.approx(1.0, abs=1e-9)


def test_freeze_stops_post_landing_drift_and_zeros_velocity() -> None:
    fusion = ImuOpticalFlowFusion(
        accel_lpf_tau_s=0.0,
        accel_deadband_mps2=0.0,
        velocity_decay_per_s=0.0,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0)
    fusion.update_imu(make_imu(stamp_s=0.0, linear_acceleration_x=1.0))
    fusion.update_imu(make_imu(stamp_s=0.1, linear_acceleration_x=1.0))
    before = fusion.freeze()
    assert before is not None
    frozen_position = before.x_m

    for index in range(2, 10):
        update = fusion.update_imu(
            make_imu(stamp_s=index * 0.1, linear_acceleration_x=1.0)
        )

    assert update.health_state == "frozen"
    assert fusion.pose is not None
    assert fusion.pose.x_m == pytest.approx(frozen_position)
    assert fusion.pose.vx_mps == pytest.approx(0.0)


def test_covariance_stays_symmetric_and_positive_during_replay() -> None:
    fusion = ImuOpticalFlowFusion(
        accel_lpf_tau_s=0.0,
        innovation_gate_nis=1e6,
    )
    fusion.reset(x_m=0.0, y_m=0.0, z_m=1.0, yaw_rad=0.0, range_m=1.0)
    fusion.update_imu(make_imu(stamp_s=0.0))
    for index in range(1, 101):
        stamp = index * 0.01
        fusion.update_imu(
            make_imu(stamp_s=stamp, linear_acceleration_x=0.1)
        )
        if index % 10 == 0:
            fusion.update_flow(
                make_flow(
                    integrated_y=0.002,
                    integration_time_s=0.02,
                    source_stamp_s=stamp,
                    quality=255,
                )
            )

    covariance = fusion.covariance
    assert covariance is not None
    assert np.allclose(covariance, covariance.T, atol=1e-12)
    assert np.all(np.linalg.eigvalsh(covariance) > -1e-10)
