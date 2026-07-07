from pathlib import Path
import csv
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.of_compare_stats import generate_stats, write_report


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
