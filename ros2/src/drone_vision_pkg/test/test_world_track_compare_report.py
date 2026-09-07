"""Tests for world-track comparison helpers and reporting."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped

from drone_vision_pkg.projection import RealsenseProjection
from drone_vision_pkg.world_track_compare_common import (
    FRAMES_CSV_FIELDNAMES,
    RUN_MANIFEST_FILENAME,
    TRACKS_CSV_FIELDNAMES,
    TRACKS_CSV_FILENAME_DEFAULT,
)
from drone_vision_pkg.world_track_compare_report import generate_report


def write_csv(path: Path, header: list[str], rows: list[list[object]]) -> None:
    """Write a CSV file for a synthetic report test."""

    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def write_manifest(run_dir: Path, run_id: str, experiment_tag: str) -> None:
    """Write a minimal manifest file for a synthetic run."""

    manifest = {
        "run_id": run_id,
        "experiment_tag": experiment_tag,
        "files": {
            "tracks_csv": str(run_dir / TRACKS_CSV_FILENAME_DEFAULT),
            "frames_csv": str(run_dir / "world_track_compare_frames.csv"),
        },
    }
    (run_dir / RUN_MANIFEST_FILENAME).write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )


def test_projection_inverse_helpers_round_trip() -> None:
    """Projection helpers should invert camera/body/world transforms."""

    projection = RealsenseProjection(
        camera_offset_body_m=[0.1, -0.2, 0.3],
        camera_rpy_body_rad=[0.0, 0.0, 0.0],
        world_frame="map",
    )

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0
    pose.pose.position.z = 3.0
    pose.pose.orientation.w = 1.0
    projection.update_pose(pose)

    camera_point = np.array([0.3, -0.4, 4.5], dtype=np.float64)
    body_point = projection.camera_to_body_frame(camera_point)
    recovered_camera_point = projection.body_to_camera_frame(body_point)
    assert np.allclose(recovered_camera_point, camera_point)

    world_point = projection.body_to_world(body_point)
    assert world_point is not None
    recovered_body_point = projection.world_to_body(world_point)
    recovered_camera_from_world = projection.world_to_camera(world_point)
    assert recovered_body_point is not None
    assert recovered_camera_from_world is not None
    assert np.allclose(recovered_body_point, body_point)
    assert np.allclose(recovered_camera_from_world, camera_point)
    assert np.isclose(np.linalg.norm(recovered_camera_from_world), 4.5276925691)
    assert np.isclose(recovered_camera_from_world[2], 4.5)


def test_projection_axis_signs_can_flip_forward_body_axis() -> None:
    """A mount-specific sign correction should invert body X only."""

    projection = RealsenseProjection(
        camera_offset_body_m=[0.0, 0.0, 0.0],
        camera_rpy_body_rad=[0.0, 0.0, 0.0],
        world_frame="map",
        camera_body_axis_signs=[-1.0, 1.0, 1.0],
    )

    projection.set_intrinsics(fx=100.0, fy=100.0, cx=50.0, cy=50.0)
    body_point = projection.pixel_depth_to_body(50.0, 50.0, 3.0)

    assert body_point is not None
    assert body_point[0] == pytest.approx(-3.0)
    assert body_point[1] == pytest.approx(0.0)
    assert body_point[2] == pytest.approx(0.0)
    assert np.allclose(
        projection.body_to_camera_frame(body_point),
        np.array([0.0, 0.0, 3.0], dtype=np.float64),
    )


def test_generate_report_for_empty_run(tmp_path: Path) -> None:
    """The report should handle runs with only frame rows."""

    run_dir = tmp_path / "empty_run"
    run_dir.mkdir()
    write_manifest(run_dir, "run_empty", "empty")
    write_csv(
        run_dir / TRACKS_CSV_FILENAME_DEFAULT,
        TRACKS_CSV_FIELDNAMES,
        [],
    )
    write_csv(
        run_dir / "world_track_compare_frames.csv",
        FRAMES_CSV_FIELDNAMES,
        [
            [
                "run_empty",
                "empty",
                1,
                0,
                "1.000000000",
                "map",
                "map",
                1,
                1,
                1,
                3.0,
                3.0,
                0,
                0,
                0,
                0,
                "",
                "",
                "",
                "",
                10.0,
                65.0,
            ],
            [
                "run_empty",
                "empty",
                2,
                0,
                "2.000000000",
                "map",
                "map",
                1,
                1,
                1,
                3.5,
                3.5,
                0,
                0,
                0,
                0,
                "",
                "",
                "",
                "",
                10.0,
                66.0,
            ],
        ],
    )

    summary = generate_report(run_dir)

    assert summary["counts"]["frame_rows"] == 2
    assert summary["counts"]["track_rows"] == 0
    assert summary["errors"]["error_norm_median"] is None
    assert (run_dir / "report" / "summary.json").exists()
    assert (run_dir / "report" / "depth_vs_error.png").exists()
    assert (run_dir / "report" / "coverage_vs_range.png").exists()


def test_generate_report_for_clean_hit_run(tmp_path: Path) -> None:
    """A clean run should yield a ready-for-follow summary."""

    run_dir = tmp_path / "clean_run"
    run_dir.mkdir()
    write_manifest(run_dir, "run_clean", "static_depth")
    write_csv(
        run_dir / TRACKS_CSV_FILENAME_DEFAULT,
        TRACKS_CSV_FIELDNAMES,
        [
            [
                "run_clean",
                "static_depth",
                1,
                0,
                "1.000000000",
                7,
                "map",
                "map",
                1,
                1,
                1,
                1,
                10.0,
                70.0,
                1.00,
                0.05,
                0.02,
                1.02,
                0.05,
                0.00,
                -0.02,
                0.02,
                0.05,
                0.02,
                3.05,
                0.03,
                0.01,
                3.00,
                0.02,
                0.03,
                0.01,
                0.05,
                0.01,
                3.08,
                3.00,
                0.92,
                3.05,
                4000.0,
                10.0,
                10.0,
                20.0,
                20.0,
                10.2,
                10.1,
                20.2,
                20.1,
                15.0,
                15.0,
                15.1,
                15.0,
                3.10,
            ],
            [
                "run_clean",
                "static_depth",
                2,
                0,
                "2.000000000",
                7,
                "map",
                "map",
                1,
                1,
                1,
                1,
                10.0,
                69.0,
                1.01,
                0.05,
                0.01,
                1.02,
                0.05,
                0.00,
                -0.01,
                0.01,
                0.05,
                0.01,
                3.04,
                0.04,
                0.01,
                3.00,
                0.01,
                0.04,
                0.01,
                0.04,
                0.01,
                3.07,
                3.00,
                0.93,
                3.04,
                4000.0,
                10.0,
                10.0,
                20.0,
                20.0,
                10.3,
                10.2,
                20.3,
                20.2,
                15.0,
                15.0,
                15.2,
                15.1,
                3.08,
            ],
        ],
    )
    write_csv(
        run_dir / "world_track_compare_frames.csv",
        FRAMES_CSV_FIELDNAMES,
        [
            [
                "run_clean",
                "static_depth",
                1,
                0,
                "1.000000000",
                "map",
                "map",
                1,
                1,
                1,
                3.08,
                3.00,
                1,
                1,
                1,
                1,
                7,
                0.8,
                0.92,
                3.05,
                10.0,
                70.0,
            ],
            [
                "run_clean",
                "static_depth",
                2,
                0,
                "2.000000000",
                "map",
                "map",
                1,
                1,
                1,
                3.07,
                3.00,
                1,
                1,
                1,
                1,
                7,
                0.82,
                0.93,
                3.04,
                10.0,
                69.0,
            ],
        ],
    )

    summary = generate_report(run_dir)

    assert summary["counts"]["track_rows"] == 2
    assert summary["coverage_by_range"]
    assert summary["follow_readiness"]["ready_for_follow"] is True


def test_generate_report_for_mixed_run(tmp_path: Path) -> None:
    """A mixed run should preserve misses, mismatches, and dropout stats."""

    run_dir = tmp_path / "mixed_run"
    run_dir.mkdir()
    write_manifest(run_dir, "run_mixed", "reacquisition")
    write_csv(
        run_dir / TRACKS_CSV_FILENAME_DEFAULT,
        TRACKS_CSV_FIELDNAMES,
        [
            [
                "run_mixed",
                "reacquisition",
                3,
                0,
                "3.000000000",
                11,
                "map",
                "odom",
                1,
                1,
                0,
                1,
                9.5,
                72.0,
                2.0,
                0.8,
                0.2,
                2.3,
                0.9,
                0.2,
                -0.3,
                -0.1,
                0.0,
                0.32,
                4.5,
                0.2,
                0.1,
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                0.40,
                4.5,
                2000.0,
                12.0,
                11.0,
                24.0,
                24.0,
                12.4,
                11.2,
                24.4,
                24.2,
                18.0,
                17.5,
                18.1,
                17.6,
                4.6,
            ],
        ],
    )
    write_csv(
        run_dir / "world_track_compare_frames.csv",
        FRAMES_CSV_FIELDNAMES,
        [
            [
                "run_mixed",
                "reacquisition",
                1,
                0,
                "1.000000000",
                "map",
                "map",
                1,
                1,
                1,
                4.0,
                4.0,
                1,
                1,
                1,
                1,
                11,
                0.7,
                0.80,
                4.0,
                9.5,
                72.0,
            ],
            [
                "run_mixed",
                "reacquisition",
                2,
                0,
                "2.000000000",
                "map",
                "map",
                1,
                1,
                1,
                4.5,
                4.5,
                0,
                0,
                0,
                0,
                "",
                "",
                "",
                "",
                9.5,
                73.0,
            ],
            [
                "run_mixed",
                "reacquisition",
                3,
                0,
                "3.000000000",
                "map",
                "odom",
                1,
                1,
                0,
                4.6,
                4.5,
                1,
                1,
                1,
                0,
                "",
                "",
                "",
                "",
                9.5,
                72.0,
            ],
        ],
    )

    summary = generate_report(run_dir)

    assert summary["rates"]["controller_valid_track_ratio"] < 1.0
    assert summary["follow_readiness"]["ready_for_follow"] is False
    assert summary["errors"]["longest_no_valid_track_gap_s"] is not None
