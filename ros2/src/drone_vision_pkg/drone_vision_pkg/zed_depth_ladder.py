"""Floor-mark-based ZED 2i depth ladder capture analysis."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np


FRAME_CSV = "depth_ladder_frames.csv"
SUMMARY_JSON = "summary.json"
SESSION_SUMMARY_CSV = "zed_depth_ladder_summary.csv"
SESSION_SUMMARY_MD = "zed_depth_ladder_summary.md"

FRAME_FIELDNAMES = [
    "range_label",
    "nominal_range_m",
    "frame_index",
    "timestamp_s",
    "detections_count",
    "best_confidence",
    "best_bbox_area_px",
    "center_median_m",
    "center_mean_m",
    "center_error_m",
    "center_abs_error_m",
    "center_valid_fraction",
    "center_invalid_fraction",
    "center_p10_m",
    "center_p90_m",
    "center_std_m",
    "bbox_median_m",
    "bbox_mean_m",
    "bbox_error_m",
    "bbox_abs_error_m",
    "bbox_valid_fraction",
    "bbox_invalid_fraction",
    "bbox_p10_m",
    "bbox_p90_m",
    "bbox_std_m",
]

SESSION_FIELDNAMES = [
    "range_label",
    "run_dir",
    "nominal_range_m",
    "frame_count",
    "detection_coverage",
    "center_median_m",
    "center_mean_m",
    "center_median_error_m",
    "center_abs_error_median_m",
    "center_valid_fraction_mean",
    "center_invalid_fraction_mean",
    "center_spread_p90_p10_m",
    "bbox_median_m",
    "bbox_mean_m",
    "bbox_median_error_m",
    "bbox_abs_error_median_m",
    "bbox_valid_fraction_mean",
    "bbox_invalid_fraction_mean",
    "bbox_spread_p90_p10_m",
    "tracker_depth_median_m",
    "tracker_depth_median_error_m",
    "tracker_distance_median_m",
    "tracker_distance_median_error_m",
    "usable",
    "notes",
]


def range_label(range_m: float) -> str:
    """Return a filesystem-friendly range label."""

    text = f"{float(range_m):.1f}".rstrip("0").rstrip(".")
    return text.replace(".", "p") + "m"


def finite_float(value: object, default: float | None = None) -> float | None:
    """Parse a finite float, returning default for blanks and non-finite values."""

    if value is None:
        return default
    text = str(value).strip()
    if not text:
        return default
    try:
        number = float(text)
    except ValueError:
        return default
    if not math.isfinite(number):
        return default
    return number


def center_roi(
    *,
    width_px: int,
    height_px: int,
    half_size_px: int,
    center_x_px: float | None = None,
    center_y_px: float | None = None,
) -> tuple[int, int, int, int]:
    """Return an x0, y0, x1, y1 center ROI with exclusive max bounds."""

    half_size_px = max(int(half_size_px), 1)
    center_x = int(
        round(
            (int(width_px) - 1) / 2.0
            if center_x_px is None
            else float(center_x_px)
        )
    )
    center_y = int(
        round(
            (int(height_px) - 1) / 2.0
            if center_y_px is None
            else float(center_y_px)
        )
    )
    return clip_roi(
        (
            center_x - half_size_px,
            center_y - half_size_px,
            center_x + half_size_px + 1,
            center_y + half_size_px + 1,
        ),
        width_px=width_px,
        height_px=height_px,
    )


def bbox_roi(
    bbox_xyxy: Iterable[float],
    *,
    width_px: int,
    height_px: int,
    padding_fraction: float = 0.0,
) -> tuple[int, int, int, int]:
    """Return a clipped exclusive ROI for a detector bbox."""

    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    pad_x = max(0.0, x2 - x1) * max(float(padding_fraction), 0.0)
    pad_y = max(0.0, y2 - y1) * max(float(padding_fraction), 0.0)
    return clip_roi(
        (
            int(math.floor(x1 - pad_x)),
            int(math.floor(y1 - pad_y)),
            int(math.ceil(x2 + pad_x)),
            int(math.ceil(y2 + pad_y)),
        ),
        width_px=width_px,
        height_px=height_px,
    )


def clip_roi(
    roi: tuple[int, int, int, int],
    *,
    width_px: int,
    height_px: int,
) -> tuple[int, int, int, int]:
    """Clip an exclusive ROI to an image."""

    x0, y0, x1, y1 = [int(value) for value in roi]
    width_px = max(int(width_px), 0)
    height_px = max(int(height_px), 0)
    x0 = min(max(x0, 0), width_px)
    x1 = min(max(x1, 0), width_px)
    y0 = min(max(y0, 0), height_px)
    y1 = min(max(y1, 0), height_px)
    if x1 <= x0:
        x1 = min(width_px, x0 + 1)
    if y1 <= y0:
        y1 = min(height_px, y0 + 1)
    return x0, y0, x1, y1


def depth_roi_stats(
    depth_m: np.ndarray | None,
    roi: tuple[int, int, int, int],
    *,
    max_valid_depth_m: float,
) -> dict[str, float | int | None]:
    """Compute robust depth statistics for an ROI."""

    if depth_m is None:
        return empty_depth_stats()
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    if depth.size == 0:
        return empty_depth_stats()

    x0, y0, x1, y1 = clip_roi(
        roi,
        width_px=int(depth.shape[1]),
        height_px=int(depth.shape[0]),
    )
    crop = np.asarray(depth[y0:y1, x0:x1], dtype=np.float32)
    if crop.size == 0:
        return empty_depth_stats()
    finite_positive = crop[np.isfinite(crop) & (crop > 0.0)]
    if finite_positive.size > 0 and float(np.max(finite_positive)) > 100.0:
        crop = crop * 0.001

    max_valid_depth_m = max(float(max_valid_depth_m), 0.001)
    valid_mask = np.isfinite(crop) & (crop > 0.0) & (crop <= max_valid_depth_m)
    valid = crop[valid_mask].astype(np.float64)
    total_count = int(crop.size)
    valid_count = int(valid.size)
    invalid_count = total_count - valid_count
    if valid_count <= 0:
        stats = empty_depth_stats()
        stats.update(
            {
                "total_count": total_count,
                "valid_count": 0,
                "invalid_count": invalid_count,
                "valid_fraction": 0.0,
                "invalid_fraction": 1.0 if total_count > 0 else 0.0,
            }
        )
        return stats

    return {
        "total_count": total_count,
        "valid_count": valid_count,
        "invalid_count": invalid_count,
        "valid_fraction": valid_count / total_count,
        "invalid_fraction": invalid_count / total_count,
        "min_m": float(np.min(valid)),
        "max_m": float(np.max(valid)),
        "mean_m": float(np.mean(valid)),
        "median_m": float(np.median(valid)),
        "p10_m": float(np.percentile(valid, 10)),
        "p90_m": float(np.percentile(valid, 90)),
        "std_m": float(np.std(valid)),
    }


def empty_depth_stats() -> dict[str, float | int | None]:
    """Return an empty stats dictionary with stable keys."""

    return {
        "total_count": 0,
        "valid_count": 0,
        "invalid_count": 0,
        "valid_fraction": 0.0,
        "invalid_fraction": 0.0,
        "min_m": None,
        "max_m": None,
        "mean_m": None,
        "median_m": None,
        "p10_m": None,
        "p90_m": None,
        "std_m": None,
    }


def frame_row(
    *,
    range_label_text: str,
    nominal_range_m: float,
    frame_index: int,
    timestamp_s: float,
    center_stats: dict[str, Any],
    bbox_stats: dict[str, Any] | None = None,
    detections_count: int = 0,
    best_confidence: float | None = None,
    best_bbox_area_px: float | None = None,
) -> dict[str, object]:
    """Build one capture CSV row."""

    bbox_stats = bbox_stats or empty_depth_stats()
    center_median = finite_float(center_stats.get("median_m"))
    bbox_median = finite_float(bbox_stats.get("median_m"))
    return {
        "range_label": range_label_text,
        "nominal_range_m": float(nominal_range_m),
        "frame_index": int(frame_index),
        "timestamp_s": float(timestamp_s),
        "detections_count": int(detections_count),
        "best_confidence": blank_if_none(best_confidence),
        "best_bbox_area_px": blank_if_none(best_bbox_area_px),
        "center_median_m": blank_if_none(center_median),
        "center_mean_m": blank_if_none(center_stats.get("mean_m")),
        "center_error_m": blank_if_none(
            None if center_median is None else center_median - float(nominal_range_m)
        ),
        "center_abs_error_m": blank_if_none(
            None if center_median is None else abs(center_median - float(nominal_range_m))
        ),
        "center_valid_fraction": blank_if_none(center_stats.get("valid_fraction")),
        "center_invalid_fraction": blank_if_none(center_stats.get("invalid_fraction")),
        "center_p10_m": blank_if_none(center_stats.get("p10_m")),
        "center_p90_m": blank_if_none(center_stats.get("p90_m")),
        "center_std_m": blank_if_none(center_stats.get("std_m")),
        "bbox_median_m": blank_if_none(bbox_median),
        "bbox_mean_m": blank_if_none(bbox_stats.get("mean_m")),
        "bbox_error_m": blank_if_none(
            None if bbox_median is None else bbox_median - float(nominal_range_m)
        ),
        "bbox_abs_error_m": blank_if_none(
            None if bbox_median is None else abs(bbox_median - float(nominal_range_m))
        ),
        "bbox_valid_fraction": blank_if_none(bbox_stats.get("valid_fraction")),
        "bbox_invalid_fraction": blank_if_none(bbox_stats.get("invalid_fraction")),
        "bbox_p10_m": blank_if_none(bbox_stats.get("p10_m")),
        "bbox_p90_m": blank_if_none(bbox_stats.get("p90_m")),
        "bbox_std_m": blank_if_none(bbox_stats.get("std_m")),
    }


def blank_if_none(value: object) -> object:
    """Return a CSV-friendly blank for None and non-finite floats."""

    number = finite_float(value)
    if number is None:
        return ""
    return number


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    """Read CSV rows or return an empty list."""

    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def finite_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    """Extract finite numeric values from rows."""

    values: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        if value is not None:
            values.append(value)
    return values


def median(values: list[float]) -> float | None:
    """Return median or None."""

    if not values:
        return None
    return float(np.median(np.asarray(values, dtype=np.float64)))


def mean(values: list[float]) -> float | None:
    """Return mean or None."""

    if not values:
        return None
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def summarize_capture_rows(
    rows: list[dict[str, Any]],
    *,
    nominal_range_m: float,
    range_label_text: str,
    run_dir: Path,
    max_usable_abs_error_m: float = 0.5,
    min_detection_coverage: float = 0.5,
) -> dict[str, object]:
    """Summarize one nominal range capture."""

    frame_count = len(rows)
    detected_count = sum(
        1
        for row in rows
        if finite_float(row.get("best_confidence")) is not None
        or finite_float(row.get("bbox_median_m")) is not None
    )
    raw_detection_count = sum(
        1 for row in rows if int(float(row.get("detections_count") or 0)) > 0
    )
    detection_coverage = detected_count / frame_count if frame_count else 0.0
    center_medians = finite_values(rows, "center_median_m")
    center_errors = [value - float(nominal_range_m) for value in center_medians]
    center_abs_errors = [abs(value) for value in center_errors]
    center_p10 = median(finite_values(rows, "center_p10_m"))
    center_p90 = median(finite_values(rows, "center_p90_m"))
    bbox_medians = finite_values(rows, "bbox_median_m")
    bbox_errors = [value - float(nominal_range_m) for value in bbox_medians]
    bbox_abs_errors = [abs(value) for value in bbox_errors]
    bbox_p10 = median(finite_values(rows, "bbox_p10_m"))
    bbox_p90 = median(finite_values(rows, "bbox_p90_m"))

    center_abs_error_median = median(center_abs_errors)
    bbox_abs_error_median = median(bbox_abs_errors)
    primary_abs_error = (
        bbox_abs_error_median
        if bbox_abs_error_median is not None
        else center_abs_error_median
    )
    usable = bool(
        frame_count > 0
        and primary_abs_error is not None
        and primary_abs_error <= float(max_usable_abs_error_m)
        and (detection_coverage >= float(min_detection_coverage) or not bbox_medians)
    )
    notes: list[str] = []
    if frame_count <= 0:
        notes.append("no frames")
    if primary_abs_error is None:
        notes.append("no valid depth")
    elif primary_abs_error > float(max_usable_abs_error_m):
        notes.append("depth error high")
    if bbox_medians and detection_coverage < float(min_detection_coverage):
        notes.append("detection coverage low")
    if raw_detection_count > 0 and detected_count == 0:
        notes.append("no accepted detector boxes")

    return {
        "range_label": range_label_text,
        "run_dir": str(run_dir),
        "nominal_range_m": float(nominal_range_m),
        "frame_count": frame_count,
        "detection_coverage": detection_coverage,
        "center_median_m": median(center_medians),
        "center_mean_m": mean(finite_values(rows, "center_mean_m")),
        "center_median_error_m": median(center_errors),
        "center_abs_error_median_m": center_abs_error_median,
        "center_valid_fraction_mean": mean(finite_values(rows, "center_valid_fraction")),
        "center_invalid_fraction_mean": mean(finite_values(rows, "center_invalid_fraction")),
        "center_spread_p90_p10_m": (
            None if center_p10 is None or center_p90 is None else center_p90 - center_p10
        ),
        "bbox_median_m": median(bbox_medians),
        "bbox_mean_m": mean(finite_values(rows, "bbox_mean_m")),
        "bbox_median_error_m": median(bbox_errors),
        "bbox_abs_error_median_m": bbox_abs_error_median,
        "bbox_valid_fraction_mean": mean(finite_values(rows, "bbox_valid_fraction")),
        "bbox_invalid_fraction_mean": mean(finite_values(rows, "bbox_invalid_fraction")),
        "bbox_spread_p90_p10_m": (
            None if bbox_p10 is None or bbox_p90 is None else bbox_p90 - bbox_p10
        ),
        "tracker_depth_median_m": None,
        "tracker_depth_median_error_m": None,
        "tracker_distance_median_m": None,
        "tracker_distance_median_error_m": None,
        "usable": usable,
        "notes": "; ".join(notes) if notes else "ok",
    }


def summarize_tracker_csv(
    tracker_csv: Path,
    *,
    nominal_range_m: float,
) -> dict[str, float | None]:
    """Summarize a world-track-compare CSV using nominal range as truth."""

    rows = read_csv_rows(tracker_csv)
    depth_values = finite_values(rows, "depth_z_m")
    distance_values = finite_values(rows, "distance_m")
    depth_median = median(depth_values)
    distance_median = median(distance_values)
    return {
        "tracker_depth_median_m": depth_median,
        "tracker_depth_median_error_m": (
            None if depth_median is None else depth_median - float(nominal_range_m)
        ),
        "tracker_distance_median_m": distance_median,
        "tracker_distance_median_error_m": (
            None if distance_median is None else distance_median - float(nominal_range_m)
        ),
    }


def load_run_summary(run_dir: Path) -> dict[str, object]:
    """Load or compute one range summary."""

    frames = read_csv_rows(run_dir / FRAME_CSV)
    nominal = None
    label = run_dir.name
    if frames:
        nominal = finite_float(frames[0].get("nominal_range_m"))
        label = str(frames[0].get("range_label") or label)
    if frames and nominal is not None:
        return summarize_capture_rows(
            frames,
            nominal_range_m=float(nominal),
            range_label_text=label,
            run_dir=run_dir,
        )

    summary_path = run_dir / SUMMARY_JSON
    if summary_path.exists():
        with summary_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        return data if isinstance(data, dict) else {}

    if nominal is None:
        manifest_path = run_dir / "manifest.json"
        if manifest_path.exists():
            with manifest_path.open("r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            nominal = finite_float(manifest.get("nominal_range_m"))
            label = str(manifest.get("range_label") or label)
    if nominal is None:
        nominal = 0.0
    return summarize_capture_rows(
        frames,
        nominal_range_m=float(nominal),
        range_label_text=label,
        run_dir=run_dir,
    )


def write_run_summary(run_dir: Path, summary: dict[str, object]) -> Path:
    """Write one range summary JSON."""

    path = run_dir / SUMMARY_JSON
    with path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")
    return path


def analyze_session(
    session_dir: Path,
    *,
    tracker_csv_by_label: dict[str, Path] | None = None,
) -> list[dict[str, object]]:
    """Analyze all range directories in a ladder session."""

    tracker_csv_by_label = tracker_csv_by_label or {}
    summaries: list[dict[str, object]] = []
    for run_dir in sorted(path for path in session_dir.iterdir() if path.is_dir()):
        if not (run_dir / FRAME_CSV).exists() and not (run_dir / SUMMARY_JSON).exists():
            continue
        summary = load_run_summary(run_dir)
        label = str(summary.get("range_label") or run_dir.name)
        nominal = finite_float(summary.get("nominal_range_m"), 0.0) or 0.0
        tracker_csv = tracker_csv_by_label.get(label)
        if tracker_csv is not None:
            summary.update(summarize_tracker_csv(tracker_csv, nominal_range_m=nominal))
        summaries.append(summary)
    summaries.sort(key=lambda row: float(row.get("nominal_range_m") or 0.0))
    return summaries


def write_session_report(session_dir: Path, summaries: list[dict[str, object]]) -> dict[str, Path]:
    """Write session-level CSV and Markdown reports."""

    csv_path = session_dir / SESSION_SUMMARY_CSV
    md_path = session_dir / SESSION_SUMMARY_MD
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SESSION_FIELDNAMES)
        writer.writeheader()
        for summary in summaries:
            writer.writerow({key: csv_value(summary.get(key)) for key in SESSION_FIELDNAMES})

    lines = [
        "# ZED 2i Depth Ladder Summary",
        "",
        "| Range | Frames | Detect cov | Center med err | Bbox med err | Tracker depth err | Usable | Notes |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for summary in summaries:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(summary.get("range_label") or ""),
                    str(summary.get("frame_count") or 0),
                    format_optional(summary.get("detection_coverage"), "{:.3f}"),
                    format_optional(summary.get("center_median_error_m"), "{:.3f}"),
                    format_optional(summary.get("bbox_median_error_m"), "{:.3f}"),
                    format_optional(summary.get("tracker_depth_median_error_m"), "{:.3f}"),
                    "yes" if bool(summary.get("usable")) else "no",
                    str(summary.get("notes") or ""),
                ]
            )
            + " |"
        )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"csv": csv_path, "markdown": md_path}


def csv_value(value: object) -> object:
    """Format values for CSV output."""

    if value is None:
        return ""
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"
    return value


def format_optional(value: object, fmt: str) -> str:
    """Format optional floats for Markdown."""

    number = finite_float(value)
    if number is None:
        return "n/a"
    return fmt.format(number)


def parse_tracker_csv_specs(specs: list[str]) -> dict[str, Path]:
    """Parse label=path CLI tracker CSV mappings."""

    result: dict[str, Path] = {}
    for spec in specs:
        if "=" not in spec:
            raise ValueError(
                "--tracker-csv entries must use LABEL=PATH, for example 2p4m=/tmp/world_track_compare_tracks.csv"
            )
        label, path = spec.split("=", 1)
        label = label.strip()
        if not label:
            raise ValueError("--tracker-csv label cannot be blank")
        result[label] = Path(path).expanduser()
    return result


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for session analysis."""

    parser = argparse.ArgumentParser(description="Analyze ZED 2i depth ladder captures.")
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--tracker-csv",
        action="append",
        default=[],
        help="Optional LABEL=world_track_compare_tracks.csv mapping for tracker-depth summary.",
    )
    args = parser.parse_args(argv)

    tracker_csv_by_label = parse_tracker_csv_specs(args.tracker_csv)
    summaries = analyze_session(args.session_dir, tracker_csv_by_label=tracker_csv_by_label)
    outputs = write_session_report(args.session_dir, summaries)
    print(f"Wrote {outputs['csv']}")
    print(f"Wrote {outputs['markdown']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
