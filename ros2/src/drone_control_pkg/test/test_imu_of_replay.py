import csv
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.imu_of_replay import replay_rows  # noqa: E402


def _row(stamp: float, state: str, flow_y: float) -> dict[str, str]:
    return {
        "run_id": "synthetic",
        "stamp_ros_s": str(stamp),
        "mission_state": state,
        "comparison_frame_id": "map",
        "mocap_x_m": str(max(0.0, stamp - 1.0)),
        "mocap_y_m": "0.0",
        "mocap_z_m": "1.0",
        "origin_reset_mocap_x_m": "0.0",
        "origin_reset_mocap_y_m": "0.0",
        "origin_reset_mocap_z_m": "1.0",
        "origin_reset_mocap_yaw_rad": "0.0",
        "imu_frame_id": "base_link",
        "imu_stamp_s": str(stamp),
        "imu_yaw_rad": "0.0",
        "imu_ang_vel_x_radps": "0.0",
        "imu_ang_vel_y_radps": "0.0",
        "imu_ang_vel_z_radps": "0.0",
        "imu_map_accel_x_mps2": "0.0",
        "imu_map_accel_y_mps2": "0.0",
        "imu_map_accel_z_mps2": "0.0",
        "flow_integrated_x": "0.0",
        "flow_integrated_y": str(flow_y),
        "flow_integrated_xgyro": "0.0",
        "flow_integrated_ygyro": "0.0",
        "flow_integrated_zgyro": "0.0",
        "flow_quality": "255",
        "flow_distance_m": "1.0",
        "range_m": "1.0",
        "flow_dt_s": "0.1",
    }


def test_legacy_replay_adds_three_estimator_outputs() -> None:
    rows = [
        _row(1.0, "SYNC_TAKEOFF_PARAM", 0.0),
        _row(1.1, "WAYPOINTS", 0.01),
        _row(1.2, "WAYPOINTS", 0.02),
        _row(1.3, "COMPLETE", 0.02),
    ]

    result = replay_rows(rows)

    assert result.metadata["approximate_legacy_replay"] is True
    assert result.metadata["imu_events"] == 3
    assert result.metadata["flow_events"] >= 2
    assert result.rows[1]["imu_only_pose_valid"] == "1"
    assert result.rows[1]["imu_only_x_m"] != ""
    assert result.rows[1]["of_x_m"] != ""
    assert result.rows[1]["fused_x_m"] != ""
    assert result.rows[-1]["replay_approximate"] == "1"


def test_replay_rows_can_be_written_as_csv(tmp_path: Path) -> None:
    result = replay_rows(
        [
            _row(1.0, "SYNC_TAKEOFF_PARAM", 0.0),
            _row(1.1, "WAYPOINTS", 0.01),
        ]
    )
    path = tmp_path / "replay.csv"
    fieldnames = list(result.rows[0])
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(result.rows)

    assert path.read_text(encoding="utf-8").startswith("run_id,")
