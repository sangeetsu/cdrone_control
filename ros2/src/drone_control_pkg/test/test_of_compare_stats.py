from pathlib import Path
import csv
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.of_compare_stats import (  # noqa: E402
    generate_stats,
    write_report,
)


def _write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _estimator_row(
    *,
    stamp: float,
    mission_state: str,
    of_error: float,
    fused_error: float,
    imu_error: float,
    sample_valid: int = 1,
    fusion_flow_valid: int = 1,
) -> dict[str, object]:
    return {
        "stamp_ros_s": stamp,
        "mission_state": mission_state,
        "frame_match": 1,
        "sample_valid": sample_valid,
        "fusion_pose_valid": 1,
        "fusion_flow_valid": fusion_flow_valid,
        "imu_only_pose_valid": 1,
        "mocap_x_m": 0.0,
        "mocap_y_m": 0.0,
        "mocap_z_m": 0.0,
        "of_x_m": of_error,
        "of_y_m": 0.0,
        "of_z_m": 0.0,
        "err_x_m": of_error,
        "err_y_m": 0.0,
        "err_z_m": 0.0,
        "err_xy_m": abs(of_error),
        "err_3d_m": abs(of_error),
        "fused_x_m": fused_error,
        "fused_y_m": 0.0,
        "fused_z_m": 0.0,
        "fused_err_x_m": fused_error,
        "fused_err_y_m": 0.0,
        "fused_err_z_m": 0.0,
        "fused_err_xy_m": abs(fused_error),
        "fused_err_3d_m": abs(fused_error),
        "imu_only_x_m": imu_error,
        "imu_only_y_m": 0.0,
        "imu_only_z_m": 0.0,
        "imu_only_vx_mps": 0.0,
        "imu_only_vy_mps": 0.0,
        "imu_only_vz_mps": 0.0,
        "imu_only_err_x_m": imu_error,
        "imu_only_err_y_m": 0.0,
        "imu_only_err_z_m": 0.0,
        "imu_only_err_xy_m": abs(imu_error),
        "imu_only_err_3d_m": abs(imu_error),
    }


def test_generate_stats_filters_invalid_rows(tmp_path: Path) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "stamp_ros_s",
                "frame_match",
                "sample_valid",
                "err_x_m",
                "err_y_m",
                "err_z_m",
                "err_xy_m",
                "err_3d_m",
                "flow_quality",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "stamp_ros_s": "1.0",
                "frame_match": "1",
                "sample_valid": "1",
                "err_x_m": "0.1",
                "err_y_m": "0.0",
                "err_z_m": "0.0",
                "err_xy_m": "0.1",
                "err_3d_m": "0.1",
                "flow_quality": "80",
            }
        )
        writer.writerow(
            {
                "stamp_ros_s": "2.0",
                "frame_match": "0",
                "sample_valid": "1",
                "err_x_m": "9.0",
                "err_y_m": "9.0",
                "err_z_m": "9.0",
                "err_xy_m": "12.7",
                "err_3d_m": "15.6",
                "flow_quality": "5",
            }
        )

    summary = generate_stats(csv_path)

    assert summary["counts"]["rows"] == 2
    assert summary["counts"]["valid_rows"] == 1
    assert summary["duration_s"] == pytest.approx(1.0)
    assert summary["errors"]["err_xy_m"]["rmse"] == pytest.approx(0.1)


def test_write_report_creates_summary_files(tmp_path: Path) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    csv_path.write_text(
        "stamp_ros_s,frame_match,sample_valid,err_3d_m,err_xy_m\n"
        "1.0,1,1,0.2,0.1\n",
        encoding="utf-8",
    )

    summary = write_report(csv_path)

    assert summary["counts"]["valid_rows"] == 1
    assert (tmp_path / "report" / "summary.json").exists()
    assert (tmp_path / "report" / "summary.md").exists()


def test_generate_stats_includes_fused_estimator(tmp_path: Path) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "stamp_ros_s",
                "frame_match",
                "sample_valid",
                "fusion_pose_valid",
                "err_x_m",
                "err_y_m",
                "err_z_m",
                "err_xy_m",
                "err_3d_m",
                "fused_err_x_m",
                "fused_err_y_m",
                "fused_err_z_m",
                "fused_err_xy_m",
                "fused_err_3d_m",
                "imu_update_valid",
                "imu_update_dt_s",
                "imu_map_accel_x_mps2",
                "imu_map_accel_y_mps2",
                "imu_map_accel_z_mps2",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "stamp_ros_s": "1.0",
                "frame_match": "1",
                "sample_valid": "1",
                "fusion_pose_valid": "1",
                "err_x_m": "0.4",
                "err_y_m": "0.0",
                "err_z_m": "0.0",
                "err_xy_m": "0.4",
                "err_3d_m": "0.4",
                "fused_err_x_m": "0.2",
                "fused_err_y_m": "0.0",
                "fused_err_z_m": "0.0",
                "fused_err_xy_m": "0.2",
                "fused_err_3d_m": "0.2",
                "imu_update_valid": "1",
                "imu_update_dt_s": "0.01",
                "imu_map_accel_x_mps2": "0.0",
                "imu_map_accel_y_mps2": "0.0",
                "imu_map_accel_z_mps2": "0.0",
            }
        )

    summary = write_report(csv_path)

    assert summary["fusion"]["counts"]["valid_3d_rows"] == 1
    assert summary["fusion"]["errors"]["fused_err_xy_m"]["rmse"] == pytest.approx(
        0.2
    )
    assert summary["fusion_improvement"]["err_xy_rmse_delta_m"] == pytest.approx(
        0.2
    )
    assert summary["imu"]["valid_update_rows"] == 1


def test_three_way_comparisons_use_identical_rows_and_endpoints(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    _write_rows(
        csv_path,
        [
            _estimator_row(
                stamp=1.0,
                mission_state="WAYPOINTS",
                of_error=1.0,
                fused_error=0.5,
                imu_error=2.0,
            ),
            _estimator_row(
                stamp=2.0,
                mission_state="WAYPOINTS",
                of_error=100.0,
                fused_error=0.5,
                imu_error=2.0,
                fusion_flow_valid=0,
            ),
            _estimator_row(
                stamp=3.0,
                mission_state="WAYPOINTS",
                of_error=3.0,
                fused_error=1.0,
                imu_error=2.0,
                sample_valid=0,
            ),
        ],
    )

    summary = generate_stats(csv_path)
    mission = summary["comparisons"]["mission_window"]
    valid = summary["comparisons"]["valid_updates"]

    assert mission["common_rows"] == 3
    assert mission["common_start_stamp_s"] == pytest.approx(1.0)
    assert mission["common_end_stamp_s"] == pytest.approx(3.0)
    assert {
        estimator["counts"]["valid_3d_rows"]
        for estimator in mission["estimators"].values()
    } == {3}
    assert valid["common_rows"] == 1
    assert valid["common_start_stamp_s"] == pytest.approx(1.0)
    assert valid["common_end_stamp_s"] == pytest.approx(1.0)
    assert {
        estimator["counts"]["valid_3d_rows"]
        for estimator in valid["estimators"].values()
    } == {1}
    assert summary["fusion_improvement"]["comparison_rows"] == 3


def test_legacy_imu_only_replay_is_labeled_and_deterministic(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    base_stamp = 1_783_643_573.0
    rows = []
    for index, imu_stamp in enumerate(
        (base_stamp, base_stamp + 0.05, base_stamp + 0.05, base_stamp + 0.10)
    ):
        rows.append(
            {
                "stamp_ros_s": base_stamp + (index * 0.05),
                "mission_state": "WAYPOINTS",
                "frame_match": 1,
                "origin_reset_mocap_x_m": 10.0,
                "origin_reset_mocap_y_m": 20.0,
                "origin_reset_mocap_z_m": 1.0,
                "mocap_x_m": 10.0,
                "mocap_y_m": 20.0,
                "mocap_z_m": 1.0,
                "imu_stamp_s": imu_stamp,
                "imu_update_valid": 1,
                "imu_map_accel_x_mps2": 2.0,
                "imu_map_accel_y_mps2": 0.0,
                "imu_map_accel_z_mps2": 0.0,
            }
        )
    _write_rows(csv_path, rows)

    summary = write_report(csv_path)
    timeseries_path = tmp_path / "report" / "imu_only_timeseries.csv"
    with timeseries_path.open(encoding="utf-8", newline="") as handle:
        replay_rows = list(csv.DictReader(handle))

    assert summary["imu_only"]["source"] == "approximate_legacy_replay"
    assert summary["imu_only"]["approximate"] is True
    assert summary["imu_only"]["unique_imu_samples_replayed"] == 3
    assert "imu_only_timeseries.csv" in summary["artifacts"]
    assert len(replay_rows) == 4
    assert replay_rows[0]["estimator_source"] == "approximate_legacy_replay"
    assert replay_rows[0]["approximate"] == "1"
    assert float(replay_rows[1]["imu_only_x_m"]) == pytest.approx(10.0025)
    assert replay_rows[2]["imu_only_x_m"] == replay_rows[1]["imu_only_x_m"]
    assert float(replay_rows[3]["imu_only_x_m"]) == pytest.approx(10.01)
    assert float(replay_rows[3]["imu_only_vx_mps"]) == pytest.approx(0.1998)


def test_native_imu_only_is_not_replayed(tmp_path: Path) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    _write_rows(
        csv_path,
        [
            _estimator_row(
                stamp=1.0,
                mission_state="WAYPOINTS",
                of_error=1.0,
                fused_error=0.5,
                imu_error=7.0,
            )
        ],
    )

    summary = write_report(csv_path)
    with (tmp_path / "report" / "imu_only_timeseries.csv").open(
        encoding="utf-8",
        newline="",
    ) as handle:
        timeseries = list(csv.DictReader(handle))

    assert summary["imu_only"]["source"] == "logged"
    assert summary["imu_only"]["approximate"] is False
    assert float(timeseries[0]["imu_only_x_m"]) == pytest.approx(7.0)
    assert timeseries[0]["approximate"] == "0"


def test_logged_legacy_replay_preserves_approximate_label(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    row = _estimator_row(
        stamp=1.0,
        mission_state="WAYPOINTS",
        of_error=1.0,
        fused_error=0.5,
        imu_error=7.0,
    )
    row["replay_approximate"] = "1"
    _write_rows(csv_path, [row])

    summary = write_report(csv_path)

    assert summary["imu_only"]["source"] == "approximate_legacy_replay"
    assert summary["imu_only"]["approximate"] is True


def test_flow_delivery_diagnostics_deduplicate_logger_rows(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    rows: list[dict[str, object]] = []
    delivery_positions = (0.0, 0.1, 0.3, 0.6)
    for sequence in range(1, 5):
        source_stamp = 10.0 + ((sequence - 1) * 0.1)
        state = "IDLE" if sequence <= 2 else "WAYPOINTS"
        for repeat in range(2):
            rows.append(
                {
                    "stamp_ros_s": source_stamp + (repeat * 0.05),
                    "mission_state": state,
                    "frame_match": 1,
                    # Event-aware logs only stamp the logger row that actually
                    # received the flow message; timer repeats stay blank.
                    "flow_sequence": sequence if repeat == 0 else "",
                    "flow_source_stamp_s": source_stamp if repeat == 0 else "",
                    "flow_integrated_x": 0.001 * sequence,
                    "flow_integrated_y": 0.002 * sequence,
                    "flow_integrated_xgyro": 0.0 if sequence in {1, 3} else "",
                    "flow_integrated_ygyro": 0.0 if sequence in {1, 3} else "",
                    "flow_integrated_zgyro": 0.0 if sequence in {1, 3} else "",
                    "flow_quality": 100,
                    "flow_distance_m": 1.0,
                    "flow_dt_s": 0.02,
                    "fusion_flow_vx_mps": float(sequence),
                    "fusion_flow_vy_mps": 0.0,
                    "mocap_x_m": delivery_positions[sequence - 1],
                    "mocap_y_m": 0.0,
                    "mocap_z_m": 1.0,
                }
            )
    _write_rows(csv_path, rows)

    diagnostics = generate_stats(csv_path)["flow_diagnostics"]
    correlation = diagnostics["flow_vs_mocap_velocity_correlation"]
    stationary = diagnostics["stationary_flow_rms"]

    assert diagnostics["identity_method"] == "flow_sequence"
    assert diagnostics["identity_approximate"] is False
    assert diagnostics["unique_deliveries"] == 4
    assert diagnostics["repeated_logger_rows"] == 4
    assert diagnostics["repeat_rate"] == pytest.approx(0.5)
    assert diagnostics["delivery_rate_hz"] == pytest.approx(10.0)
    assert diagnostics["integration_coverage_ratio"] == pytest.approx(0.08 / 0.3)
    assert diagnostics["gyro"]["xy_valid_rate"] == pytest.approx(0.5)
    assert correlation["samples"] == 2
    assert correlation["x_pearson"] == pytest.approx(1.0)
    assert stationary["speed_rms_mps"] == pytest.approx((2.5) ** 0.5)


def test_post_complete_drift_uses_common_endpoints(tmp_path: Path) -> None:
    csv_path = tmp_path / "of_mocap_compare.csv"
    first = _estimator_row(
        stamp=5.0,
        mission_state="COMPLETE",
        of_error=0.1,
        fused_error=0.2,
        imu_error=0.3,
    )
    second = _estimator_row(
        stamp=6.0,
        mission_state="COMPLETE",
        of_error=0.2,
        fused_error=0.5,
        imu_error=0.3,
    )
    _write_rows(csv_path, [first, second])

    post_complete = generate_stats(csv_path)["post_complete"]

    assert post_complete["common_rows"] == 2
    assert post_complete["common_start_stamp_s"] == pytest.approx(5.0)
    assert post_complete["common_end_stamp_s"] == pytest.approx(6.0)
    assert post_complete["estimators"]["of_only"]["drift"][
        "delta_err_xy_m"
    ] == pytest.approx(0.1)
    assert post_complete["estimators"]["imu_of_fused"]["drift"][
        "delta_err_xy_m"
    ] == pytest.approx(0.3)
    assert post_complete["estimators"]["imu_only"]["drift"][
        "delta_err_xy_m"
    ] == pytest.approx(0.0)
