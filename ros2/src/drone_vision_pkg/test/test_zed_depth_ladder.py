from __future__ import annotations

import csv
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg.zed_depth_ladder import (
    FRAME_CSV,
    FRAME_FIELDNAMES,
    SESSION_FIELDNAMES,
    analyze_session,
    bbox_roi,
    center_roi,
    depth_roi_stats,
    frame_row,
    range_label,
    summarize_capture_rows,
    summarize_tracker_csv,
    write_run_summary,
    write_session_report,
)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def test_depth_roi_stats_filters_invalid_values_and_reports_spread() -> None:
    depth = np.array(
        [
            [0.0, 1.0, 1.1],
            [np.nan, 1.2, 50.0],
            [1.3, np.inf, 1.4],
        ],
        dtype=np.float32,
    )

    stats = depth_roi_stats(depth, (0, 0, 3, 3), max_valid_depth_m=5.0)

    assert stats["total_count"] == 9
    assert stats["valid_count"] == 5
    assert stats["invalid_count"] == 4
    assert stats["valid_fraction"] == pytest.approx(5.0 / 9.0)
    assert stats["median_m"] == pytest.approx(1.2)
    assert stats["p90_m"] == pytest.approx(1.36)


def test_depth_roi_stats_converts_millimeters_when_needed() -> None:
    depth_mm = np.array([[1000.0, 1100.0], [1200.0, 0.0]], dtype=np.float32)

    stats = depth_roi_stats(depth_mm, (0, 0, 2, 2), max_valid_depth_m=2.0)

    assert stats["valid_count"] == 3
    assert stats["median_m"] == pytest.approx(1.1)


def test_center_and_bbox_roi_are_clipped() -> None:
    assert center_roi(width_px=10, height_px=8, half_size_px=2) == (2, 2, 7, 7)
    assert bbox_roi(
        (-10.0, 1.2, 12.0, 9.5),
        width_px=10,
        height_px=8,
        padding_fraction=0.1,
    ) == (0, 0, 10, 8)


def test_frame_row_records_nominal_errors_for_center_and_bbox() -> None:
    row = frame_row(
        range_label_text="2p4m",
        nominal_range_m=2.4,
        frame_index=3,
        timestamp_s=10.0,
        center_stats={
            "median_m": 2.30,
            "mean_m": 2.31,
            "valid_fraction": 0.9,
            "invalid_fraction": 0.1,
            "p10_m": 2.2,
            "p90_m": 2.4,
            "std_m": 0.05,
        },
        bbox_stats={
            "median_m": 2.45,
            "mean_m": 2.46,
            "valid_fraction": 0.8,
            "invalid_fraction": 0.2,
            "p10_m": 2.3,
            "p90_m": 2.6,
            "std_m": 0.08,
        },
        detections_count=1,
        best_confidence=0.77,
        best_bbox_area_px=1200.0,
    )

    assert row["range_label"] == "2p4m"
    assert row["center_error_m"] == pytest.approx(-0.1)
    assert row["bbox_abs_error_m"] == pytest.approx(0.05)
    assert row["detections_count"] == 1


def test_summarize_capture_rows_marks_good_range_usable() -> None:
    rows = [
        {
            "detections_count": 1,
            "center_median_m": 1.21,
            "center_mean_m": 1.20,
            "center_valid_fraction": 0.95,
            "center_invalid_fraction": 0.05,
            "center_p10_m": 1.15,
            "center_p90_m": 1.25,
            "bbox_median_m": 1.19,
            "bbox_mean_m": 1.20,
            "bbox_valid_fraction": 0.90,
            "bbox_invalid_fraction": 0.10,
            "bbox_p10_m": 1.12,
            "bbox_p90_m": 1.27,
        },
        {
            "detections_count": 1,
            "center_median_m": 1.18,
            "center_mean_m": 1.19,
            "center_valid_fraction": 0.94,
            "center_invalid_fraction": 0.06,
            "center_p10_m": 1.13,
            "center_p90_m": 1.24,
            "bbox_median_m": 1.22,
            "bbox_mean_m": 1.21,
            "bbox_valid_fraction": 0.91,
            "bbox_invalid_fraction": 0.09,
            "bbox_p10_m": 1.15,
            "bbox_p90_m": 1.28,
        },
    ]

    summary = summarize_capture_rows(
        rows,
        nominal_range_m=1.2,
        range_label_text="1p2m",
        run_dir=Path("/tmp/1p2m"),
    )

    assert summary["usable"] is True
    assert summary["detection_coverage"] == pytest.approx(1.0)
    assert summary["bbox_median_error_m"] == pytest.approx(0.005)
    assert summary["center_spread_p90_p10_m"] == pytest.approx(0.105)


def test_summarize_capture_rows_marks_bad_range_unusable() -> None:
    rows = [
        {
            "detections_count": 0,
            "center_median_m": 4.0,
            "center_valid_fraction": 0.8,
            "center_invalid_fraction": 0.2,
        }
    ]

    summary = summarize_capture_rows(
        rows,
        nominal_range_m=2.4,
        range_label_text="2p4m",
        run_dir=Path("/tmp/2p4m"),
    )

    assert summary["usable"] is False
    assert "depth error high" in str(summary["notes"])


def test_summarize_tracker_csv_uses_nominal_range_as_truth(tmp_path: Path) -> None:
    tracker_csv = tmp_path / "world_track_compare_tracks.csv"
    write_csv(
        tracker_csv,
        ["depth_z_m", "distance_m"],
        [
            {"depth_z_m": "2.35", "distance_m": "2.40"},
            {"depth_z_m": "2.45", "distance_m": "2.50"},
        ],
    )

    summary = summarize_tracker_csv(tracker_csv, nominal_range_m=2.4)

    assert summary["tracker_depth_median_m"] == pytest.approx(2.4)
    assert summary["tracker_depth_median_error_m"] == pytest.approx(0.0)
    assert summary["tracker_distance_median_error_m"] == pytest.approx(0.05)


def test_analyze_session_and_write_report(tmp_path: Path) -> None:
    session_dir = tmp_path / "session"
    run_dir = session_dir / range_label(1.2)
    run_dir.mkdir(parents=True)
    rows = [
        frame_row(
            range_label_text="1p2m",
            nominal_range_m=1.2,
            frame_index=0,
            timestamp_s=1.0,
            center_stats={
                "median_m": 1.2,
                "mean_m": 1.2,
                "valid_fraction": 1.0,
                "invalid_fraction": 0.0,
                "p10_m": 1.1,
                "p90_m": 1.3,
                "std_m": 0.01,
            },
            detections_count=0,
        )
    ]
    write_csv(run_dir / FRAME_CSV, FRAME_FIELDNAMES, rows)
    summary = summarize_capture_rows(
        rows,
        nominal_range_m=1.2,
        range_label_text="1p2m",
        run_dir=run_dir,
    )
    write_run_summary(run_dir, summary)

    summaries = analyze_session(session_dir)
    outputs = write_session_report(session_dir, summaries)

    assert len(summaries) == 1
    assert summaries[0]["center_median_m"] == pytest.approx(1.2)
    assert outputs["csv"].exists()
    assert outputs["markdown"].exists()
    with outputs["csv"].open("r", encoding="utf-8", newline="") as handle:
        report_rows = list(csv.DictReader(handle))
    assert list(report_rows[0].keys()) == SESSION_FIELDNAMES
