"""Offline reporting for world-track comparison experiment bundles."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

from drone_vision_pkg.world_track_compare_common import (
    FOLLOW_CONTROLLER_GATES,
    FRAMES_CSV_FILENAME,
    REPORT_DIRNAME,
    RUN_MANIFEST_FILENAME,
    TRACKS_CSV_FILENAME_DEFAULT,
)

matplotlib.use("Agg")
from matplotlib import pyplot as plt  # noqa: E402


TRACK_FLOAT_FIELDS = {
    "timestamp_s",
    "tracker_fps",
    "inference_latency_ms",
    "world_x_m",
    "world_y_m",
    "world_z_m",
    "rigidbody2_x_m",
    "rigidbody2_y_m",
    "rigidbody2_z_m",
    "error_x_m",
    "error_y_m",
    "error_z_m",
    "error_norm_m",
    "body_x_m",
    "body_y_m",
    "body_z_m",
    "truth_body_x_m",
    "truth_body_y_m",
    "truth_body_z_m",
    "body_error_x_m",
    "body_error_y_m",
    "body_error_z_m",
    "truth_range_m",
    "truth_depth_z_m",
    "confidence",
    "distance_m",
    "bbox_area_px",
    "yolo_bbox_x1_px",
    "yolo_bbox_y1_px",
    "yolo_bbox_x2_px",
    "yolo_bbox_y2_px",
    "norfair_bbox_x1_px",
    "norfair_bbox_y1_px",
    "norfair_bbox_x2_px",
    "norfair_bbox_y2_px",
    "yolo_x_px",
    "yolo_y_px",
    "norfair_x_px",
    "norfair_y_px",
    "depth_z_m",
}
TRACK_INT_FIELDS = {
    "stamp_sec",
    "stamp_nanosec",
    "track_id",
    "world_tracks_count",
}
TRACK_BOOL_FIELDS = {
    "reference_available",
    "ownship_pose_available",
    "frame_match",
}

FRAME_FLOAT_FIELDS = {
    "timestamp_s",
    "truth_range_m",
    "truth_depth_z_m",
    "best_track_score",
    "best_track_confidence",
    "best_track_distance_m",
    "tracker_fps",
    "inference_latency_ms",
}
FRAME_INT_FIELDS = {
    "stamp_sec",
    "stamp_nanosec",
    "detections_count",
    "active_tracks_count",
    "world_tracks_count",
    "best_track_id",
}
FRAME_BOOL_FIELDS = {
    "reference_available",
    "ownship_pose_available",
    "frame_match",
    "has_controller_valid_track",
}


def optional_float(value: Any) -> float | None:
    """Parse a CSV value into a float or None."""

    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    return float(text)


def optional_int(value: Any) -> int | None:
    """Parse a CSV value into an int or None."""

    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    return int(float(text))


def optional_bool(value: Any) -> bool | None:
    """Parse a CSV value into a bool or None."""

    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    if text in {"1", "true", "yes"}:
        return True
    if text in {"0", "false", "no"}:
        return False
    raise ValueError(f"Unsupported boolean value: {value!r}")


def safe_mean(values: list[float]) -> float | None:
    """Return the mean of a list or None when empty."""

    if not values:
        return None
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def percentile(values: list[float], q: float) -> float | None:
    """Return a percentile from a list of floats."""

    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def load_json(path: Path) -> dict[str, Any]:
    """Load a JSON file if it exists."""

    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data if isinstance(data, dict) else {}


def read_typed_csv(
    path: Path,
    *,
    float_fields: set[str],
    int_fields: set[str],
    bool_fields: set[str],
) -> list[dict[str, Any]]:
    """Read a CSV into typed dictionaries."""

    if not path.exists():
        return []

    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            typed_row: dict[str, Any] = {}
            for key, value in row.items():
                if key is None:
                    continue
                if key in float_fields:
                    typed_row[key] = optional_float(value)
                elif key in int_fields:
                    typed_row[key] = optional_int(value)
                elif key in bool_fields:
                    typed_row[key] = optional_bool(value)
                else:
                    typed_row[key] = value
            rows.append(typed_row)
    return rows


def first_existing_path(paths: list[Path]) -> Path | None:
    """Return the first existing path from a list."""

    for path in paths:
        if path.exists():
            return path
    return None


def resolve_artifacts(run_dir: Path) -> dict[str, Path | None]:
    """Resolve the manifest and CSV files for a run bundle."""

    manifest_path = run_dir / RUN_MANIFEST_FILENAME
    manifest = load_json(manifest_path)
    manifest_files = manifest.get("files", {}) if isinstance(manifest, dict) else {}

    tracks_candidates = [
        run_dir / TRACKS_CSV_FILENAME_DEFAULT,
        run_dir / "world_track_compare.csv",
    ]
    frames_candidates = [run_dir / FRAMES_CSV_FILENAME]
    manifest_tracks = manifest_files.get("tracks_csv")
    manifest_frames = manifest_files.get("frames_csv")
    if isinstance(manifest_tracks, str) and manifest_tracks:
        tracks_candidates.insert(0, Path(manifest_tracks))
    if isinstance(manifest_frames, str) and manifest_frames:
        frames_candidates.insert(0, Path(manifest_frames))

    return {
        "manifest": manifest_path if manifest_path.exists() else None,
        "tracks_csv": first_existing_path(tracks_candidates),
        "frames_csv": first_existing_path(frames_candidates),
    }


def finite_series(rows: list[dict[str, Any]], key: str) -> list[float]:
    """Extract a finite float series from a list of rows."""

    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if value is None:
            continue
        number = float(value)
        if np.isfinite(number):
            values.append(number)
    return values


def finite_xy(
    rows: list[dict[str, Any]],
    x_key: str,
    y_key: str,
) -> tuple[list[float], list[float]]:
    """Extract paired finite series from a list of rows."""

    xs: list[float] = []
    ys: list[float] = []
    for row in rows:
        x_value = row.get(x_key)
        y_value = row.get(y_key)
        if x_value is None or y_value is None:
            continue
        x_number = float(x_value)
        y_number = float(y_value)
        if not np.isfinite(x_number) or not np.isfinite(y_number):
            continue
        xs.append(x_number)
        ys.append(y_number)
    return xs, ys


def empty_plot(output_path: Path, title: str, message: str) -> None:
    """Write a placeholder plot when no usable data exists."""

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.axis("off")
    ax.text(0.5, 0.5, message, ha="center", va="center", fontsize=12)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def scatter_plot(
    xs: list[float],
    ys: list[float],
    *,
    output_path: Path,
    title: str,
    xlabel: str,
    ylabel: str,
) -> None:
    """Save a scatter plot or a placeholder when empty."""

    if not xs or not ys:
        empty_plot(output_path, title, "No usable rows were available for this plot.")
        return

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.scatter(xs, ys, s=18, alpha=0.55, edgecolors="none")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def bins_for(values: list[float], bin_size: float) -> np.ndarray:
    """Build inclusive bin edges for a series."""

    min_value = float(min(values))
    max_value = float(max(values))
    start = np.floor(min_value / bin_size) * bin_size
    stop = np.ceil(max_value / bin_size) * bin_size + bin_size
    if stop <= start:
        stop = start + bin_size
    return np.arange(start, stop + 0.5 * bin_size, bin_size)


def depth_binned_error_plot(
    track_rows: list[dict[str, Any]],
    *,
    depth_bin_size: float,
    output_path: Path,
) -> None:
    """Plot median and p90 error by estimated depth bin."""

    xs, ys = finite_xy(track_rows, "depth_z_m", "error_norm_m")
    if not xs or not ys:
        empty_plot(
            output_path,
            "Estimated Depth vs Error Bands",
            "No track rows contained both estimated depth and error.",
        )
        return

    edges = bins_for(xs, depth_bin_size)
    centers: list[float] = []
    medians: list[float] = []
    p90s: list[float] = []
    for left_edge, right_edge in zip(edges[:-1], edges[1:]):
        bin_values = [
            error_value
            for depth_value, error_value in zip(xs, ys)
            if left_edge <= depth_value < right_edge
        ]
        if not bin_values:
            continue
        centers.append(float((left_edge + right_edge) / 2.0))
        medians.append(float(np.median(np.asarray(bin_values, dtype=np.float64))))
        p90s.append(float(np.percentile(np.asarray(bin_values, dtype=np.float64), 90)))

    if not centers:
        empty_plot(
            output_path,
            "Estimated Depth vs Error Bands",
            "Depth bins were empty after filtering.",
        )
        return

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(centers, medians, marker="o", linewidth=2.0, label="median error")
    ax.plot(centers, p90s, marker="s", linewidth=2.0, label="p90 error")
    ax.set_title("Estimated Depth vs Error Bands")
    ax.set_xlabel("Estimated depth_z_m")
    ax.set_ylabel("Position error norm (m)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def coverage_by_range(
    frame_rows: list[dict[str, Any]],
    *,
    depth_bin_size: float,
) -> list[dict[str, Any]]:
    """Compute range-binned frame coverage metrics."""

    valid_rows = [
        row
        for row in frame_rows
        if row.get("truth_range_m") is not None
        and float(row["truth_range_m"]) >= 0.0
    ]
    if not valid_rows:
        return []

    truth_ranges = [float(row["truth_range_m"]) for row in valid_rows]
    edges = bins_for(truth_ranges, depth_bin_size)
    summaries: list[dict[str, Any]] = []
    for left_edge, right_edge in zip(edges[:-1], edges[1:]):
        bin_rows = [
            row
            for row in valid_rows
            if left_edge <= float(row["truth_range_m"]) < right_edge
        ]
        if not bin_rows:
            continue
        frame_count = len(bin_rows)
        with_world_track = sum(
            1 for row in bin_rows if int(row.get("world_tracks_count") or 0) > 0
        )
        with_valid_track = sum(
            1
            for row in bin_rows
            if bool(row.get("has_controller_valid_track", False))
        )
        summaries.append(
            {
                "bin_start_m": float(left_edge),
                "bin_end_m": float(right_edge),
                "frame_count": frame_count,
                "world_track_fraction": with_world_track / frame_count,
                "controller_valid_fraction": with_valid_track / frame_count,
            }
        )
    return summaries


def coverage_plot(
    frame_rows: list[dict[str, Any]],
    *,
    depth_bin_size: float,
    output_path: Path,
) -> list[dict[str, Any]]:
    """Plot world-track and controller-valid coverage by range."""

    coverage_rows = coverage_by_range(frame_rows, depth_bin_size=depth_bin_size)
    if not coverage_rows:
        empty_plot(
            output_path,
            "Coverage vs True Range",
            "No frame rows contained truth range data.",
        )
        return []

    xs = [
        float((row["bin_start_m"] + row["bin_end_m"]) / 2.0)
        for row in coverage_rows
    ]
    world_track_fraction = [float(row["world_track_fraction"]) for row in coverage_rows]
    controller_valid_fraction = [
        float(row["controller_valid_fraction"]) for row in coverage_rows
    ]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(xs, world_track_fraction, marker="o", linewidth=2.0, label="world track")
    ax.plot(
        xs,
        controller_valid_fraction,
        marker="s",
        linewidth=2.0,
        label="controller-valid track",
    )
    ax.set_ylim(-0.05, 1.05)
    ax.set_title("Coverage vs True Range")
    ax.set_xlabel("True camera range (m)")
    ax.set_ylabel("Frame fraction")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return coverage_rows


def longest_false_run_s(
    frame_rows: list[dict[str, Any]],
    *,
    key: str,
    only_with_truth_range: bool = False,
) -> float | None:
    """Return the longest consecutive false run for a frame-level boolean key."""

    ordered_rows = sorted(
        frame_rows,
        key=lambda row: float(row.get("timestamp_s") or 0.0),
    )
    if only_with_truth_range:
        ordered_rows = [
            row for row in ordered_rows if row.get("truth_range_m") is not None
        ]
    if len(ordered_rows) < 2:
        return None

    longest = 0.0
    false_start: float | None = None
    last_timestamp = float(ordered_rows[0].get("timestamp_s") or 0.0)
    for row in ordered_rows:
        timestamp_s = float(row.get("timestamp_s") or 0.0)
        value = bool(row.get(key, False))
        if not value and false_start is None:
            false_start = timestamp_s
        if value and false_start is not None:
            longest = max(longest, timestamp_s - false_start)
            false_start = None
        last_timestamp = timestamp_s
    if false_start is not None:
        longest = max(longest, last_timestamp - false_start)
    return float(longest)


def jitter_m(rows: list[dict[str, Any]], axis_keys: tuple[str, str, str]) -> float | None:
    """Estimate jitter as combined standard deviation across three axes."""

    valid_vectors: list[list[float]] = []
    for row in rows:
        values = [row.get(axis_key) for axis_key in axis_keys]
        if any(value is None for value in values):
            continue
        vector = [float(value) for value in values]
        if not np.all(np.isfinite(np.asarray(vector, dtype=np.float64))):
            continue
        valid_vectors.append(vector)
    if len(valid_vectors) < 2:
        return None
    array = np.asarray(valid_vectors, dtype=np.float64)
    std_vector = np.std(array, axis=0)
    return float(np.linalg.norm(std_vector))


def follow_readiness_summary(
    track_rows: list[dict[str, Any]],
    frame_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    """Evaluate follow-readiness in the controller's operating range."""

    max_range = FOLLOW_CONTROLLER_GATES["max_target_distance_m"]
    operating_frames = [
        row
        for row in frame_rows
        if row.get("truth_range_m") is not None
        and float(row["truth_range_m"]) <= max_range
    ]
    operating_tracks = [
        row
        for row in track_rows
        if row.get("truth_range_m") is not None
        and float(row["truth_range_m"]) <= max_range
    ]

    controller_valid_ratio = None
    if operating_frames:
        controller_valid_ratio = float(
            np.mean(
                np.asarray(
                    [
                        1.0
                        if bool(row.get("has_controller_valid_track", False))
                        else 0.0
                        for row in operating_frames
                    ],
                    dtype=np.float64,
                )
            )
        )

    forward_errors = [
        abs(float(row["body_error_x_m"]))
        for row in operating_tracks
        if row.get("body_error_x_m") is not None
    ]
    lateral_errors = [
        abs(float(row["body_error_y_m"]))
        for row in operating_tracks
        if row.get("body_error_y_m") is not None
    ]
    vertical_errors = [
        abs(float(row["body_error_z_m"]))
        for row in operating_tracks
        if row.get("body_error_z_m") is not None
    ]

    longest_gap_s = longest_false_run_s(
        operating_frames,
        key="has_controller_valid_track",
        only_with_truth_range=False,
    )
    p90_forward_error_m = percentile(forward_errors, 90)
    p90_lateral_error_m = percentile(lateral_errors, 90)
    p90_vertical_error_m = percentile(vertical_errors, 90)

    ready = all(
        metric is not None
        for metric in (
            controller_valid_ratio,
            longest_gap_s,
            p90_forward_error_m,
            p90_lateral_error_m,
            p90_vertical_error_m,
        )
    )
    if ready:
        ready = bool(
            controller_valid_ratio
            >= FOLLOW_CONTROLLER_GATES["controller_valid_ratio_min"]
            and longest_gap_s <= FOLLOW_CONTROLLER_GATES["longest_gap_max_s"]
            and p90_forward_error_m
            <= FOLLOW_CONTROLLER_GATES["follow_error_p90_x_m"]
            and p90_lateral_error_m
            <= FOLLOW_CONTROLLER_GATES["follow_error_p90_y_m"]
            and p90_vertical_error_m
            <= FOLLOW_CONTROLLER_GATES["follow_error_p90_z_m"]
        )

    return {
        "operating_band_max_range_m": max_range,
        "operating_band_frame_count": len(operating_frames),
        "operating_band_track_count": len(operating_tracks),
        "controller_valid_track_ratio": controller_valid_ratio,
        "longest_no_valid_track_gap_s": longest_gap_s,
        "p90_forward_error_m": p90_forward_error_m,
        "p90_lateral_error_m": p90_lateral_error_m,
        "p90_vertical_error_m": p90_vertical_error_m,
        "ready_for_follow": bool(ready),
    }


def build_summary(
    *,
    run_dir: Path,
    manifest: dict[str, Any],
    track_rows: list[dict[str, Any]],
    frame_rows: list[dict[str, Any]],
    coverage_rows: list[dict[str, Any]],
    plot_paths: dict[str, str],
) -> dict[str, Any]:
    """Build the JSON summary for a report bundle."""

    error_norm_values = finite_series(track_rows, "error_norm_m")
    tracker_fps_values = finite_series(frame_rows, "tracker_fps")
    latency_values = finite_series(frame_rows, "inference_latency_ms")

    total_frames = len(frame_rows)
    frames_with_reference = sum(
        1 for row in frame_rows if bool(row.get("reference_available", False))
    )
    frames_with_world_track = sum(
        1 for row in frame_rows if int(row.get("world_tracks_count") or 0) > 0
    )
    frames_with_valid_track = sum(
        1
        for row in frame_rows
        if bool(row.get("has_controller_valid_track", False))
    )

    summary = {
        "run_dir": str(run_dir),
        "run_id": (
            manifest.get("run_id")
            if isinstance(manifest.get("run_id"), str)
            else run_dir.name
        ),
        "experiment_tag": manifest.get("experiment_tag", ""),
        "started_at_wall": manifest.get("started_at_wall"),
        "completed_at_wall": manifest.get("completed_at_wall"),
        "plots": plot_paths,
        "counts": {
            "frame_rows": total_frames,
            "track_rows": len(track_rows),
            "frames_with_reference": frames_with_reference,
            "frames_with_world_track": frames_with_world_track,
            "frames_with_controller_valid_track": frames_with_valid_track,
        },
        "rates": {
            "reference_available_ratio": (
                frames_with_reference / total_frames if total_frames else None
            ),
            "world_track_frame_ratio": (
                frames_with_world_track / total_frames if total_frames else None
            ),
            "controller_valid_track_ratio": (
                frames_with_valid_track / total_frames if total_frames else None
            ),
        },
        "timing": {
            "tracker_fps_mean": safe_mean(tracker_fps_values),
            "inference_latency_ms_mean": safe_mean(latency_values),
        },
        "errors": {
            "error_norm_median": percentile(error_norm_values, 50),
            "error_norm_p90": percentile(error_norm_values, 90),
            "error_norm_p95": percentile(error_norm_values, 95),
            "longest_no_valid_track_gap_s": longest_false_run_s(
                frame_rows,
                key="has_controller_valid_track",
                only_with_truth_range=False,
            ),
            "body_position_jitter_m": jitter_m(
                track_rows,
                ("body_x_m", "body_y_m", "body_z_m"),
            ),
            "world_position_jitter_m": jitter_m(
                track_rows,
                ("world_x_m", "world_y_m", "world_z_m"),
            ),
        },
        "coverage_by_range": coverage_rows,
        "follow_readiness": follow_readiness_summary(track_rows, frame_rows),
    }
    return summary


def write_summary_markdown(summary: dict[str, Any], output_path: Path) -> None:
    """Write a concise markdown report next to the plots."""

    counts = summary.get("counts", {})
    rates = summary.get("rates", {})
    timing = summary.get("timing", {})
    errors = summary.get("errors", {})
    readiness = summary.get("follow_readiness", {})
    lines = [
        "# World Track Compare Report",
        "",
        f"- Run ID: `{summary.get('run_id', 'unknown')}`",
        f"- Experiment tag: `{summary.get('experiment_tag', '')}`",
        f"- Run directory: `{summary.get('run_dir', '')}`",
        "",
        "## Overview",
        "",
        f"- Frame rows: `{counts.get('frame_rows')}`",
        f"- Track rows: `{counts.get('track_rows')}`",
        f"- Reference available ratio: `{rates.get('reference_available_ratio')}`",
        f"- World-track frame ratio: `{rates.get('world_track_frame_ratio')}`",
        "- Controller-valid frame ratio: "
        f"`{rates.get('controller_valid_track_ratio')}`",
        "",
        "## Error Metrics",
        "",
        f"- Error median: `{errors.get('error_norm_median')}` m",
        f"- Error p90: `{errors.get('error_norm_p90')}` m",
        f"- Error p95: `{errors.get('error_norm_p95')}` m",
        "- Longest no-valid-track gap: "
        f"`{errors.get('longest_no_valid_track_gap_s')}` s",
        f"- Body-position jitter: `{errors.get('body_position_jitter_m')}` m",
        f"- World-position jitter: `{errors.get('world_position_jitter_m')}` m",
        "",
        "## Follow Readiness",
        "",
        "- Operating-band frame count: "
        f"`{readiness.get('operating_band_frame_count')}`",
        "- Operating-band controller-valid ratio: "
        f"`{readiness.get('controller_valid_track_ratio')}`",
        "- Operating-band longest gap: "
        f"`{readiness.get('longest_no_valid_track_gap_s')}` s",
        f"- Forward error p90: `{readiness.get('p90_forward_error_m')}` m",
        f"- Lateral error p90: `{readiness.get('p90_lateral_error_m')}` m",
        f"- Vertical error p90: `{readiness.get('p90_vertical_error_m')}` m",
        f"- Ready for follow: `{readiness.get('ready_for_follow')}`",
        "",
        "## Timing",
        "",
        f"- Mean tracker FPS: `{timing.get('tracker_fps_mean')}`",
        f"- Mean inference latency: `{timing.get('inference_latency_ms_mean')}` ms",
        "",
        "## Artifacts",
        "",
        "- `depth_vs_error.png`",
        "- `depth_binned_error.png`",
        "- `truth_range_vs_error.png`",
        "- `coverage_vs_range.png`",
        "- `summary.json`",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate_report(
    run_dir: str | Path,
    *,
    depth_bin_size: float = 0.5,
    output_dir: str | Path | None = None,
) -> dict[str, Any]:
    """Generate a full offline report for a run bundle."""

    run_dir = Path(run_dir).expanduser().resolve()
    if not run_dir.exists():
        raise FileNotFoundError(f"Run directory does not exist: {run_dir}")

    if output_dir is None:
        output_dir = run_dir / REPORT_DIRNAME
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    artifacts = resolve_artifacts(run_dir)
    manifest = load_json(artifacts["manifest"]) if artifacts["manifest"] else {}
    track_rows = (
        read_typed_csv(
            artifacts["tracks_csv"],
            float_fields=TRACK_FLOAT_FIELDS,
            int_fields=TRACK_INT_FIELDS,
            bool_fields=TRACK_BOOL_FIELDS,
        )
        if artifacts["tracks_csv"] is not None
        else []
    )
    frame_rows = (
        read_typed_csv(
            artifacts["frames_csv"],
            float_fields=FRAME_FLOAT_FIELDS,
            int_fields=FRAME_INT_FIELDS,
            bool_fields=FRAME_BOOL_FIELDS,
        )
        if artifacts["frames_csv"] is not None
        else []
    )

    depth_vs_error_path = output_dir / "depth_vs_error.png"
    depth_binned_error_path = output_dir / "depth_binned_error.png"
    truth_range_vs_error_path = output_dir / "truth_range_vs_error.png"
    coverage_vs_range_path = output_dir / "coverage_vs_range.png"
    summary_json_path = output_dir / "summary.json"
    summary_md_path = output_dir / "summary.md"

    depth_xs, depth_ys = finite_xy(track_rows, "depth_z_m", "error_norm_m")
    scatter_plot(
        depth_xs,
        depth_ys,
        output_path=depth_vs_error_path,
        title="Estimated Depth vs Position Error",
        xlabel="Estimated depth_z_m",
        ylabel="Position error norm (m)",
    )
    depth_binned_error_plot(
        track_rows,
        depth_bin_size=depth_bin_size,
        output_path=depth_binned_error_path,
    )
    truth_range_xs, truth_range_ys = finite_xy(
        track_rows,
        "truth_range_m",
        "error_norm_m",
    )
    scatter_plot(
        truth_range_xs,
        truth_range_ys,
        output_path=truth_range_vs_error_path,
        title="True Camera Range vs Position Error",
        xlabel="True camera range (m)",
        ylabel="Position error norm (m)",
    )
    coverage_rows = coverage_plot(
        frame_rows,
        depth_bin_size=depth_bin_size,
        output_path=coverage_vs_range_path,
    )

    plot_paths = {
        "depth_vs_error": str(depth_vs_error_path),
        "depth_binned_error": str(depth_binned_error_path),
        "truth_range_vs_error": str(truth_range_vs_error_path),
        "coverage_vs_range": str(coverage_vs_range_path),
    }
    summary = build_summary(
        run_dir=run_dir,
        manifest=manifest,
        track_rows=track_rows,
        frame_rows=frame_rows,
        coverage_rows=coverage_rows,
        plot_paths=plot_paths,
    )
    summary_json_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_summary_markdown(summary, summary_md_path)
    return summary


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser for the report generator."""

    parser = argparse.ArgumentParser(
        description="Generate plots and summaries for a world-track comparison run.",
    )
    parser.add_argument(
        "--run-dir",
        required=True,
        help="Run bundle directory that contains the comparison artifacts.",
    )
    parser.add_argument(
        "--depth-bin-size",
        type=float,
        default=0.5,
        help="Bin size in meters for depth/range aggregation plots.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Optional output directory. Defaults to <run-dir>/report.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the offline report CLI."""

    parser = build_arg_parser()
    args = parser.parse_args(argv)
    summary = generate_report(
        args.run_dir,
        depth_bin_size=float(args.depth_bin_size),
        output_dir=args.output_dir,
    )
    print(
        "Generated world-track compare report for "
        f"{summary.get('run_id', 'unknown')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
