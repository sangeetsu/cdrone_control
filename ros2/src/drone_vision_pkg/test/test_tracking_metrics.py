from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg import tracking_metrics as metrics
from drone_vision_pkg.tracking_metrics import (
    SUMMARY_FIELDNAMES,
    TrackSample,
    TrackingMetricsRun,
    clean_target_path,
    compare_tracking_runs,
    simulated_dropout_backtest,
    write_csv,
    write_run_outputs,
)


def write_reference_video(path: Path, *, fps: float, frame_count: int) -> None:
    assert metrics.cv2 is not None
    width, height = 640, 480
    fourcc = metrics.cv2.VideoWriter_fourcc(*"mp4v")
    writer = metrics.cv2.VideoWriter(str(path), fourcc, fps, (width, height))
    assert writer.isOpened()
    try:
        for index in range(frame_count):
            frame = np.full((height, width, 3), index % 255, dtype=np.uint8)
            writer.write(frame)
    finally:
        writer.release()


def sample(
    timestamp_s: float,
    *,
    sample_kind: str,
    track_id: int,
    x_m: float,
    source: int = metrics.TRACK_SOURCE_DETECTED,
    uncertainty_m: float = 0.2,
    confidence: float = 0.9,
) -> TrackSample:
    return TrackSample(
        timestamp_s=timestamp_s,
        sample_kind=sample_kind,
        track_id=track_id,
        detector_track_id=track_id,
        source=source,
        position_m=np.array([x_m, 0.0, 1.0], dtype=np.float64),
        velocity_mps=np.zeros(3, dtype=np.float64),
        confidence=confidence,
        distance_m=max(x_m, 0.1),
        bbox_area_px=1000.0,
        inbound=False,
        last_observed_age_s=0.0,
        prediction_horizon_s=0.0,
        position_uncertainty_m=uncertainty_m,
        velocity_uncertainty_mps=0.2,
    )


def test_tracking_run_counts_prediction_bridge_and_reacquisition() -> None:
    run = TrackingMetricsRun(run_id="run_m4", experiment_tag="milestone4")

    frames = [
        (
            0.0,
            [sample(0.0, sample_kind="raw", track_id=10, x_m=0.0)],
            [sample(0.0, sample_kind="target_map", track_id=1, x_m=0.0)],
        ),
        (
            1.0,
            [sample(1.0, sample_kind="raw", track_id=10, x_m=1.0)],
            [sample(1.0, sample_kind="target_map", track_id=1, x_m=1.0)],
        ),
        (
            2.0,
            [],
            [
                sample(
                    2.0,
                    sample_kind="target_map",
                    track_id=1,
                    x_m=2.0,
                    source=metrics.TRACK_SOURCE_PREDICTED,
                    uncertainty_m=0.5,
                )
            ],
        ),
        (
            3.0,
            [],
            [
                sample(
                    3.0,
                    sample_kind="target_map",
                    track_id=1,
                    x_m=3.0,
                    source=metrics.TRACK_SOURCE_PREDICTED,
                    uncertainty_m=0.7,
                )
            ],
        ),
        (
            4.0,
            [sample(4.0, sample_kind="raw", track_id=11, x_m=3.2)],
            [sample(4.0, sample_kind="target_map", track_id=1, x_m=3.2)],
        ),
    ]

    for timestamp_s, raw_tracks, map_tracks in frames:
        run.add_samples(raw_tracks + map_tracks)
        run.sample_frame(
            timestamp_s=timestamp_s,
            state="FOLLOW",
            raw_tracks=raw_tracks,
            target_map_tracks=map_tracks,
        )

    summary = run.summary()

    assert summary["frame_count"] == 5
    assert summary["raw_visual_track_coverage_ratio"] == "0.600000"
    assert summary["target_map_coverage_ratio"] == "1.000000"
    assert summary["predicted_coverage_during_raw_dropout_ratio"] == "1.000000"
    assert summary["longest_raw_dropout_s"] == "1.000000"
    assert summary["longest_target_map_bridge_s"] == "1.000000"
    assert summary["raw_id_switch_count"] == 1
    assert summary["target_map_id_switch_count"] == 0
    assert summary["raw_reacquisition_count"] == 1
    assert summary["target_map_same_id_reacquisition_count"] == 1
    assert summary["reacquisition_residual_count"] == 1
    assert summary["uncertainty_calibration_ratio"] == "1.000000"


def test_clean_target_path_prefers_target_map_and_smooths() -> None:
    samples = [
        sample(0.0, sample_kind="raw", track_id=99, x_m=20.0),
        sample(0.0, sample_kind="target_map", track_id=1, x_m=0.0),
        sample(1.0, sample_kind="target_map", track_id=1, x_m=2.0),
        sample(2.0, sample_kind="target_map", track_id=1, x_m=4.0),
    ]

    cleaned = clean_target_path(samples, smoothing_window=3)

    assert [point.sample_kind for point in cleaned] == [
        "target_map",
        "target_map",
        "target_map",
    ]
    assert np.isclose(cleaned[1].cleaned_position_m[0], 2.0)


def test_video_history_window_filters_tracks_older_than_ten_seconds() -> None:
    samples = [
        sample(0.0, sample_kind="raw", track_id=1, x_m=0.0),
        sample(4.9, sample_kind="raw", track_id=1, x_m=1.0),
        sample(5.0, sample_kind="raw", track_id=1, x_m=2.0),
        sample(15.0, sample_kind="raw", track_id=1, x_m=3.0),
    ]
    cleaned = clean_target_path(samples, smoothing_window=1)
    ownship_history = [
        (4.0, np.array([4.0, 0.0, 0.0], dtype=np.float64)),
        (5.0, np.array([5.0, 0.0, 0.0], dtype=np.float64)),
        (15.0, np.array([15.0, 0.0, 0.0], dtype=np.float64)),
    ]

    recent_samples = metrics.samples_in_history_window(
        samples,
        frame_time_s=15.0,
    )
    recent_cleaned = metrics.cleaned_points_in_history_window(
        cleaned,
        frame_time_s=15.0,
    )
    recent_ownship = metrics.positions_in_history_window(
        ownship_history,
        frame_time_s=15.0,
    )

    assert [item.timestamp_s for item in recent_samples] == [5.0, 15.0]
    assert [item.timestamp_s for item in recent_cleaned] == [5.0, 15.0]
    assert [float(position[0]) for position in recent_ownship] == [5.0, 15.0]


def test_simulated_dropout_backtest_uses_visible_raw_motion() -> None:
    samples = [
        sample(0.0, sample_kind="raw", track_id=5, x_m=0.0),
        sample(0.5, sample_kind="raw", track_id=5, x_m=0.5),
        sample(1.0, sample_kind="raw", track_id=5, x_m=1.0),
        sample(1.5, sample_kind="raw", track_id=5, x_m=1.5),
    ]

    errors = simulated_dropout_backtest(samples, horizon_s=0.5)

    assert errors
    assert max(errors) == pytest.approx(0.0)


def test_write_run_outputs_creates_csv_bundle(tmp_path: Path) -> None:
    run = TrackingMetricsRun(run_id="run_csv", experiment_tag="milestone4")
    raw = [sample(0.0, sample_kind="raw", track_id=1, x_m=0.0)]
    target_map = [sample(0.0, sample_kind="target_map", track_id=1, x_m=0.0)]
    run.add_samples(raw + target_map)
    run.sample_frame(
        timestamp_s=0.0,
        state="FOLLOW",
        raw_tracks=raw,
        target_map_tracks=target_map,
    )

    artifacts = write_run_outputs(run, run_dir=tmp_path, write_video=False)

    assert Path(artifacts["samples_csv"]).exists()
    assert Path(artifacts["frames_csv"]).exists()
    assert Path(artifacts["cleaned_path_csv"]).exists()
    assert Path(artifacts["summary_csv"]).exists()


def test_cleaned_path_csv_timestamps_start_at_zero(tmp_path: Path) -> None:
    run = TrackingMetricsRun(run_id="run_relative_csv", experiment_tag="milestone4")
    for timestamp_s, x_m in [(100.0, 0.0), (101.25, 1.0), (102.0, 2.0)]:
        target_map = [
            sample(
                timestamp_s,
                sample_kind="target_map",
                track_id=7,
                x_m=x_m,
            )
        ]
        run.add_samples(target_map)
        run.sample_frame(
            timestamp_s=timestamp_s,
            state="FOLLOW",
            raw_tracks=[],
            target_map_tracks=target_map,
        )

    artifacts = write_run_outputs(run, run_dir=tmp_path, write_video=False)

    with Path(artifacts["cleaned_path_csv"]).open("r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]["timestamp_s"] == "0.000000000"
    assert float(rows[1]["timestamp_s"]) == pytest.approx(1.25)

    manifest = json.loads((tmp_path / metrics.MANIFEST_JSON).read_text())
    assert manifest["postprocessing"]["time_origin_s"] == pytest.approx(100.0)


def test_write_run_outputs_creates_mp4_when_opencv_available(tmp_path: Path) -> None:
    if metrics.cv2 is None:
        pytest.skip("OpenCV is not available")
    run = TrackingMetricsRun(run_id="run_video", experiment_tag="milestone4")
    for index in range(4):
        raw = [sample(float(index), sample_kind="raw", track_id=1, x_m=float(index))]
        target_map = [
            sample(float(index), sample_kind="target_map", track_id=1, x_m=float(index))
        ]
        run.add_samples(raw + target_map)
        run.add_ownship_position(
            float(index),
            np.array([0.0, float(index) * 0.1, 0.0], dtype=np.float64),
        )
        run.sample_frame(
            timestamp_s=float(index),
            state="FOLLOW",
            raw_tracks=raw,
            target_map_tracks=target_map,
        )

    artifacts = write_run_outputs(run, run_dir=tmp_path, write_video=True)

    assert Path(artifacts["cleaned_path_mp4"]).exists()
    assert Path(artifacts["cleaned_path_mp4"]).stat().st_size > 0


def test_cleaned_path_mp4_matches_reference_video_metadata(tmp_path: Path) -> None:
    if metrics.cv2 is None:
        pytest.skip("OpenCV is not available")

    reference_video = (
        tmp_path / "mission_cdrone3_20260625_154233_557_sync_takeoff_param_combined.mp4"
    )
    write_reference_video(reference_video, fps=5.0, frame_count=7)

    run_dir = tmp_path / "tracking_cdrone3_20260625_154233_192_milestone4"
    run = TrackingMetricsRun(
        run_id=run_dir.name,
        experiment_tag="milestone4",
    )
    for index in range(4):
        target_map = [
            sample(
                100.0 + float(index),
                sample_kind="target_map",
                track_id=1,
                x_m=float(index),
            )
        ]
        run.add_samples(target_map)
        run.sample_frame(
            timestamp_s=100.0 + float(index),
            state="FOLLOW",
            raw_tracks=[],
            target_map_tracks=target_map,
        )

    artifacts = write_run_outputs(
        run,
        run_dir=run_dir,
        drone_id="cdrone3",
        write_video=True,
        reference_video=reference_video,
    )

    capture = metrics.cv2.VideoCapture(artifacts["cleaned_path_mp4"])
    try:
        assert int(capture.get(metrics.cv2.CAP_PROP_FRAME_COUNT)) == 7
        assert capture.get(metrics.cv2.CAP_PROP_FPS) == pytest.approx(5.0)
    finally:
        capture.release()

    manifest = json.loads((run_dir / metrics.MANIFEST_JSON).read_text())
    assert manifest["postprocessing"]["video_synced_to_reference"] is True
    assert manifest["postprocessing"]["reference_video_frame_count"] == 7


def test_find_nearest_reference_video_uses_run_timestamp(tmp_path: Path) -> None:
    recording_dir = tmp_path / "mission_recordings"
    recording_dir.mkdir()
    far = recording_dir / (
        "mission_cdrone3_20260625_155245_348_sync_takeoff_param_combined.mp4"
    )
    near = recording_dir / (
        "mission_cdrone3_20260625_154233_557_sync_takeoff_param_combined.mp4"
    )
    other_drone = recording_dir / (
        "mission_cdrone4_20260625_154233_111_sync_takeoff_param_combined.mp4"
    )
    far.touch()
    near.touch()
    other_drone.touch()
    run_dir = (
        tmp_path
        / "tracking_metrics"
        / "tracking_cdrone3_20260625_154233_192_milestone4"
    )
    run_dir.mkdir(parents=True)

    found = metrics.find_nearest_reference_video(
        run_dir=run_dir,
        recording_dir=recording_dir,
        run_id=run_dir.name,
        drone_id="cdrone3",
    )

    assert found == near


def test_postprocess_main_generates_cleaned_outputs(tmp_path: Path) -> None:
    if metrics.cv2 is None:
        pytest.skip("OpenCV is not available")

    run_dir = tmp_path / "tracking_cdrone3_20260625_154233_192_milestone4"
    run_dir.mkdir()
    samples = [
        sample(200.0, sample_kind="target_map", track_id=2, x_m=0.0),
        sample(201.0, sample_kind="target_map", track_id=2, x_m=1.0),
        sample(202.0, sample_kind="target_map", track_id=2, x_m=2.0),
    ]
    rows = [
        metrics.sample_to_csv_row(
            item,
            run_id=run_dir.name,
            experiment_tag="milestone4",
        )
        for item in samples
    ]
    write_csv(run_dir / metrics.SAMPLES_CSV, metrics.SAMPLE_FIELDNAMES, rows)
    reference_video = (
        tmp_path / "mission_cdrone3_20260625_154233_557_sync_takeoff_param_combined.mp4"
    )
    write_reference_video(reference_video, fps=4.0, frame_count=6)

    exit_code = metrics.postprocess_main(
        [
            "--run-dir",
            str(run_dir),
            "--reference-video",
            str(reference_video),
            "--video-width",
            "640",
            "--video-height",
            "480",
        ]
    )

    assert exit_code == 0
    with (run_dir / metrics.CLEANED_PATH_CSV).open("r", encoding="utf-8") as handle:
        cleaned_rows = list(csv.DictReader(handle))
    assert cleaned_rows[0]["timestamp_s"] == "0.000000000"

    capture = metrics.cv2.VideoCapture(str(run_dir / metrics.CLEANED_PATH_MP4))
    try:
        assert int(capture.get(metrics.cv2.CAP_PROP_FRAME_COUNT)) == 6
        assert capture.get(metrics.cv2.CAP_PROP_FPS) == pytest.approx(4.0)
    finally:
        capture.release()


def test_compare_tracking_runs_writes_comparison_files(tmp_path: Path) -> None:
    m3_dir = tmp_path / "m3"
    m4_dir = tmp_path / "m4"
    out_dir = tmp_path / "report"
    m3_dir.mkdir()
    m4_dir.mkdir()
    m3_summary = {field: "" for field in SUMMARY_FIELDNAMES}
    m3_summary.update(
        {
            "run_id": "m3",
            "experiment_tag": "milestone3",
            "raw_visual_track_coverage_ratio": "0.4",
            "target_map_coverage_ratio": "",
            "longest_no_track_gap_s": "2.0",
        }
    )
    m4_summary = {field: "" for field in SUMMARY_FIELDNAMES}
    m4_summary.update(
        {
            "run_id": "m4",
            "experiment_tag": "milestone4",
            "raw_visual_track_coverage_ratio": "0.4",
            "target_map_coverage_ratio": "0.9",
            "longest_no_track_gap_s": "0.2",
        }
    )
    write_csv(m3_dir / metrics.SUMMARY_CSV, SUMMARY_FIELDNAMES, [m3_summary])
    write_csv(m4_dir / metrics.SUMMARY_CSV, SUMMARY_FIELDNAMES, [m4_summary])

    report = compare_tracking_runs(
        milestone3_run_dir=m3_dir,
        milestone4_run_dir=m4_dir,
        output_dir=out_dir,
    )

    assert Path(report["comparison_csv"]).exists()
    assert Path(report["comparison_json"]).exists()
    assert Path(report["comparison_markdown"]).exists()
