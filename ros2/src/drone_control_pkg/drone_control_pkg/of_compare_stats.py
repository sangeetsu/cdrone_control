from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import Counter
from pathlib import Path
from statistics import mean, median
from typing import Iterable, Optional, Sequence


ERROR_COLUMNS = ("err_x_m", "err_y_m", "err_z_m", "err_xy_m", "err_3d_m")


def generate_stats(csv_path: str | Path) -> dict[str, object]:
    path = Path(csv_path).expanduser()
    rows = _read_rows(path)
    valid_3d_rows = _eligible_rows(rows, columns=("err_3d_m",))
    valid_xyz_rows = _eligible_rows(rows, columns=ERROR_COLUMNS)
    valid_z_rows = _eligible_rows(rows, columns=("err_z_m",))
    finite_pose_rows = _eligible_rows(
        rows,
        require_sample_valid=False,
        columns=ERROR_COLUMNS,
    )

    errors: dict[str, object] = {}
    for column in ERROR_COLUMNS:
        values = _numeric_values(row.get(column) for row in valid_3d_rows)
        errors[column] = _series_summary(values)

    return {
        "csv_path": str(path),
        "counts": {
            "rows": len(rows),
            "valid_rows": len(valid_3d_rows),
            "valid_3d_rows": len(valid_3d_rows),
            "valid_xyz_rows": len(valid_xyz_rows),
            "valid_z_rows": len(valid_z_rows),
            "finite_pose_rows": len(finite_pose_rows),
            "frame_match_rows": sum(
                1 for row in rows if _truthy(row.get("frame_match"))
            ),
            "sample_valid_rows": sum(
                1 for row in rows if _truthy(row.get("sample_valid"))
            ),
        },
        "duration_s": _duration_s(rows),
        "time_range": _time_range(rows),
        "mission_states": _value_counts(row.get("mission_state") for row in rows),
        "reject_reasons": _reject_reason_counts(rows),
        "errors": errors,
        "z_errors": _series_summary(
            _numeric_values(row.get("err_z_m") for row in valid_z_rows)
        ),
        "quality": _quality_summary(rows),
        "range": _range_summary(rows),
        "drift": _drift_summary(valid_3d_rows),
        "partial_drift": _drift_summary(finite_pose_rows),
    }


def write_report(
    csv_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    plots: bool = False,
) -> dict[str, object]:
    path = Path(csv_path).expanduser()
    summary = generate_stats(path)
    report_dir = (
        Path(output_dir).expanduser()
        if output_dir is not None
        else path.parent / "report"
    )
    report_dir.mkdir(parents=True, exist_ok=True)
    summary["artifacts"] = [
        path.relative_to(report_dir).as_posix()
        for path in _write_timeseries_csv(_read_rows(path), report_dir)
    ]
    summary["plots"] = _write_plots(path, report_dir) if plots else []

    summary_path = report_dir / "summary.json"
    markdown_path = report_dir / "summary.md"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(summary), encoding="utf-8")
    return summary


def render_markdown(summary: dict[str, object]) -> str:
    counts = summary.get("counts", {})
    errors = summary.get("errors", {})
    z_errors = summary.get("z_errors", {})
    drift = summary.get("drift", {})
    partial_drift = summary.get("partial_drift", {})
    range_summary = summary.get("range", {})
    quality = summary.get("quality", {})
    plots = summary.get("plots", [])
    artifacts = summary.get("artifacts", [])

    lines = [
        "# Mocap vs Optical-Flow Summary",
        "",
        f"- CSV: `{summary.get('csv_path', '')}`",
        f"- Duration: {_fmt(summary.get('duration_s'))} s",
        f"- Rows: {counts.get('rows', 0)}",
        f"- Valid 3D rows: {counts.get('valid_3d_rows', 0)}",
        f"- Valid XYZ rows: {counts.get('valid_xyz_rows', 0)}",
        f"- Valid Z rows: {counts.get('valid_z_rows', 0)}",
        f"- Frame-match rows: {counts.get('frame_match_rows', 0)}",
        f"- Sample-valid rows: {counts.get('sample_valid_rows', 0)}",
        f"- Reject reasons: `{summary.get('reject_reasons', {})}`",
        "",
        "## 3D Error Statistics",
        "",
        "| Metric | RMSE m | Mean signed m | MAE m | Median abs m | P95 abs m | Max abs m |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for column in ERROR_COLUMNS:
        stats = errors.get(column, {})
        label = column.removeprefix("err_").removesuffix("_m")
        lines.append(
            "| "
            f"{label} | {_fmt(stats.get('rmse'))} | "
            f"{_fmt(stats.get('mean'))} | {_fmt(stats.get('mae'))} | "
            f"{_fmt(stats.get('median_abs'))} | {_fmt(stats.get('p95_abs'))} | "
            f"{_fmt(stats.get('max_abs'))} |"
        )

    lines.extend(
        [
            "",
            "## Z-Only Error Statistics",
            "",
            "| Metric | RMSE m | Mean signed m | MAE m | Median abs m | P95 abs m | Max abs m |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
            "| z_valid | "
            f"{_fmt(z_errors.get('rmse'))} | "
            f"{_fmt(z_errors.get('mean'))} | "
            f"{_fmt(z_errors.get('mae'))} | "
            f"{_fmt(z_errors.get('median_abs'))} | "
            f"{_fmt(z_errors.get('p95_abs'))} | "
            f"{_fmt(z_errors.get('max_abs'))} |",
            "",
            "## Drift",
            "",
            f"- Final horizontal drift: {_fmt(drift.get('final_err_xy_m'))} m",
            f"- Final 3D drift: {_fmt(drift.get('final_err_3d_m'))} m",
            f"- Horizontal drift rate: {_fmt(drift.get('err_xy_rate_mps'))} m/s",
            f"- 3D drift rate: {_fmt(drift.get('err_3d_rate_mps'))} m/s",
            f"- Final partial horizontal drift: "
            f"{_fmt(partial_drift.get('final_err_xy_m'))} m",
            f"- Final partial 3D drift: "
            f"{_fmt(partial_drift.get('final_err_3d_m'))} m",
            "",
            "## Flow Health",
            "",
            f"- Range median: {_fmt(range_summary.get('median'))} m",
            f"- Range p95: {_fmt(range_summary.get('p95'))} m",
            f"- Range min/max: {_fmt(range_summary.get('min'))} / "
            f"{_fmt(range_summary.get('max'))} m",
            f"- Quality median: {_fmt(quality.get('median'))}",
            f"- Quality min/max: {_fmt(quality.get('min'))} / "
            f"{_fmt(quality.get('max'))}",
            "",
        ]
    )
    lines.extend(["## Artifacts", ""])
    if artifacts:
        for artifact in artifacts:
            lines.append(f"- `{artifact}`")
    else:
        lines.append("- n/a")
    lines.extend(["", "## Plots", ""])
    if plots:
        for plot in plots:
            lines.append(f"- `{plot}`")
    else:
        lines.append("- n/a")
    lines.append("")
    return "\n".join(lines)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate mocap-vs-optical-flow comparison stats."
    )
    parser.add_argument("csv_path", help="Path to of_mocap_compare.csv")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for summary.json and summary.md. Default: CSV/report",
    )
    parser.add_argument(
        "--plots",
        action="store_true",
        help="Also generate PNG plots under the report directory.",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Deprecated no-op; plots are disabled by default.",
    )
    args = parser.parse_args(argv)

    summary = write_report(
        args.csv_path,
        output_dir=args.output_dir,
        plots=bool(args.plots and not args.no_plots),
    )
    counts = summary["counts"]
    err_xy = summary["errors"]["err_xy_m"]
    err_3d = summary["errors"]["err_3d_m"]
    print(
        f"rows={counts['rows']} "
        f"valid_3d_rows={counts['valid_3d_rows']} "
        f"valid_z_rows={counts['valid_z_rows']}"
    )
    print(
        "err_xy_rmse_m="
        f"{_fmt(err_xy.get('rmse'))} "
        "err_xy_p95_m="
        f"{_fmt(err_xy.get('p95_abs'))}"
    )
    print(
        "err_3d_rmse_m="
        f"{_fmt(err_3d.get('rmse'))} "
        "err_3d_p95_m="
        f"{_fmt(err_3d.get('p95_abs'))}"
    )
    plots = summary.get("plots", [])
    if plots:
        print("plots=" + ",".join(str(item) for item in plots))
    return 0


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _eligible_rows(
    rows: Sequence[dict[str, str]],
    *,
    columns: Sequence[str],
    require_sample_valid: bool = True,
) -> list[dict[str, str]]:
    result = []
    for row in rows:
        if not _truthy(row.get("frame_match")):
            continue
        if require_sample_valid and not _truthy(row.get("sample_valid")):
            continue
        if all(_float_or_none(row.get(column)) is not None for column in columns):
            result.append(row)
    return result


def _duration_s(rows: Sequence[dict[str, str]]) -> Optional[float]:
    stamps = _numeric_values(row.get("stamp_ros_s") for row in rows)
    if len(stamps) < 2:
        return None
    return max(stamps) - min(stamps)


def _time_range(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    if not rows:
        return {"start_wall_time_iso": "", "end_wall_time_iso": ""}
    return {
        "start_wall_time_iso": rows[0].get("wall_time_iso", ""),
        "end_wall_time_iso": rows[-1].get("wall_time_iso", ""),
    }


def _quality_summary(rows: Sequence[dict[str, str]]) -> dict[str, Optional[float]]:
    qualities = _numeric_values(row.get("flow_quality") for row in rows)
    return _series_min_median_mean(qualities)


def _range_summary(rows: Sequence[dict[str, str]]) -> dict[str, Optional[float]]:
    ranges = _numeric_values(row.get("range_m") for row in rows)
    if not ranges:
        ranges = _numeric_values(row.get("flow_distance_m") for row in rows)
    return _series_summary(ranges)


def _reject_reason_counts(rows: Sequence[dict[str, str]]) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for row in rows:
        reason = str(row.get("reject_reason", "") or "valid")
        counter[reason] += 1
    return dict(counter.most_common())


def _value_counts(values: Iterable[object]) -> dict[str, int]:
    counter: Counter[str] = Counter(str(value or "") for value in values)
    return dict(counter.most_common())


def _drift_summary(rows: Sequence[dict[str, str]]) -> dict[str, Optional[float]]:
    if not rows:
        return {
            "duration_s": None,
            "initial_err_xy_m": None,
            "final_err_xy_m": None,
            "delta_err_xy_m": None,
            "max_err_xy_m": None,
            "err_xy_rate_mps": None,
            "initial_err_3d_m": None,
            "final_err_3d_m": None,
            "delta_err_3d_m": None,
            "max_err_3d_m": None,
            "err_3d_rate_mps": None,
        }

    times = _relative_times(rows)
    err_xy = _numeric_values(row.get("err_xy_m") for row in rows)
    err_3d = _numeric_values(row.get("err_3d_m") for row in rows)
    duration_s = max(times) - min(times) if len(times) >= 2 else 0.0
    return {
        "duration_s": duration_s,
        "initial_err_xy_m": err_xy[0] if err_xy else None,
        "final_err_xy_m": err_xy[-1] if err_xy else None,
        "delta_err_xy_m": (err_xy[-1] - err_xy[0]) if len(err_xy) >= 2 else None,
        "max_err_xy_m": max(err_xy) if err_xy else None,
        "err_xy_rate_mps": _linear_slope(times, err_xy),
        "initial_err_3d_m": err_3d[0] if err_3d else None,
        "final_err_3d_m": err_3d[-1] if err_3d else None,
        "delta_err_3d_m": (err_3d[-1] - err_3d[0]) if len(err_3d) >= 2 else None,
        "max_err_3d_m": max(err_3d) if err_3d else None,
        "err_3d_rate_mps": _linear_slope(times, err_3d),
    }


def _write_plots(csv_path: Path, report_dir: Path) -> list[str]:
    rows = _read_rows(csv_path)
    plot_dir = report_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(report_dir / ".mplconfig"))
    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - depends on host install
        error_path = report_dir / "plot_error.txt"
        error_path.write_text(f"plot generation failed: {exc}\n", encoding="utf-8")
        return [error_path.relative_to(report_dir).as_posix()]

    outputs: list[Path] = []
    outputs.extend(_plot_errors(rows, plot_dir, plt))
    outputs.extend(_plot_drift(rows, plot_dir, plt))
    outputs.extend(_plot_trajectory(rows, plot_dir, plt))
    outputs.extend(_plot_flow_health(rows, plot_dir, plt))
    return [path.relative_to(report_dir).as_posix() for path in outputs]


def _plot_errors(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]:
    valid_3d_rows = _eligible_rows(rows, columns=ERROR_COLUMNS)
    valid_z_rows = _eligible_rows(rows, columns=("err_z_m",))
    base_rows = valid_3d_rows if valid_3d_rows else valid_z_rows
    if len(base_rows) < 2:
        return []

    times = _relative_times(base_rows)
    path = plot_dir / "errors_over_time.png"
    if valid_3d_rows:
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        axes[0].plot(times, _series(base_rows, "err_x_m"), label="x")
        axes[0].plot(times, _series(base_rows, "err_y_m"), label="y")
        axes[0].plot(times, _series(base_rows, "err_z_m"), label="z")
        axes[0].set_ylabel("signed error (m)")
        axes[0].legend(loc="best")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(times, _series(base_rows, "err_xy_m"), label="xy")
        axes[1].plot(times, _series(base_rows, "err_3d_m"), label="3d")
        axes[1].set_ylabel("error norm (m)")
        axes[1].set_xlabel("time since first valid sample (s)")
        axes[1].legend(loc="best")
        axes[1].grid(True, alpha=0.3)
    else:
        fig, ax = plt.subplots(1, 1, figsize=(10, 4))
        ax.plot(times, _series(base_rows, "err_z_m"), label="z")
        ax.set_ylabel("signed z error (m)")
        ax.set_xlabel("time since first valid sample (s)")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
    fig.suptitle("Mocap vs Optical-Flow Error Over Time")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return [path]


def _plot_drift(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]:
    valid_3d_rows = _eligible_rows(rows, columns=ERROR_COLUMNS)
    if len(valid_3d_rows) < 2:
        return []

    times = _relative_times(valid_3d_rows)
    err_xy = _series(valid_3d_rows, "err_xy_m")
    err_3d = _series(valid_3d_rows, "err_3d_m")
    path = plot_dir / "drift_over_time.png"
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    ax.plot(times, err_xy, label="xy drift")
    ax.plot(times, err_3d, label="3d drift")
    for values, label in ((err_xy, "xy fit"), (err_3d, "3d fit")):
        slope = _linear_slope(times, values)
        if slope is None:
            continue
        intercept = mean(values) - (slope * mean(times))
        ax.plot(times, [intercept + (slope * t) for t in times], "--", label=label)
    ax.set_title("Dead-Reckon Drift Over Time")
    ax.set_xlabel("time since first valid sample (s)")
    ax.set_ylabel("error norm (m)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return [path]


def _plot_trajectory(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]:
    rows_to_plot = _eligible_rows(rows, columns=ERROR_COLUMNS)
    title_suffix = "valid 3D rows"
    if len(rows_to_plot) < 2:
        rows_to_plot = _eligible_rows(
            rows,
            require_sample_valid=False,
            columns=ERROR_COLUMNS,
        )
        title_suffix = "finite pose rows; may include rejected samples"
    if len(rows_to_plot) < 2:
        return []

    path = plot_dir / "trajectory_xy.png"
    fig, ax = plt.subplots(1, 1, figsize=(7, 7))
    ax.plot(_series(rows_to_plot, "mocap_x_m"), _series(rows_to_plot, "mocap_y_m"), label="mocap")
    ax.plot(_series(rows_to_plot, "of_x_m"), _series(rows_to_plot, "of_y_m"), label="OF dead-reckon")
    ax.set_title(f"XY Trajectory ({title_suffix})")
    ax.set_xlabel("x map (m)")
    ax.set_ylabel("y map (m)")
    ax.axis("equal")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return [path]


def _plot_flow_health(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]:
    flow_rows = [
        row
        for row in rows
        if _float_or_none(row.get("range_m")) is not None
        or _float_or_none(row.get("flow_distance_m")) is not None
        or _float_or_none(row.get("flow_quality")) is not None
    ]
    if len(flow_rows) < 2:
        return []

    times = _relative_times(flow_rows)
    range_values = [
        _float_or_none(row.get("range_m"))
        if _float_or_none(row.get("range_m")) is not None
        else _float_or_none(row.get("flow_distance_m"))
        for row in flow_rows
    ]
    quality_values = _series(flow_rows, "flow_quality")
    path = plot_dir / "flow_health_over_time.png"
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(times, range_values, label="range")
    axes[0].axhline(0.2, color="tab:red", linestyle="--", label="default min accepted range")
    axes[0].set_ylabel("range (m)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(times, quality_values, label="quality", color="tab:green")
    axes[1].set_xlabel("time since first flow sample (s)")
    axes[1].set_ylabel("quality")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("PX4Flow Health Over Time")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return [path]


def _write_timeseries_csv(rows: Sequence[dict[str, str]], report_dir: Path) -> list[Path]:
    valid_3d_rows = _eligible_rows(rows, columns=ERROR_COLUMNS)
    valid_z_rows = _eligible_rows(rows, columns=("err_z_m",))
    rows_to_write = valid_3d_rows if valid_3d_rows else valid_z_rows
    if not rows_to_write:
        return []

    path = report_dir / "drift_timeseries.csv"
    fields = [
        "t_s",
        "stamp_ros_s",
        "mission_state",
        "err_x_m",
        "err_y_m",
        "err_z_m",
        "err_xy_m",
        "err_3d_m",
        "mocap_x_m",
        "mocap_y_m",
        "mocap_z_m",
        "of_x_m",
        "of_y_m",
        "of_z_m",
        "flow_quality",
        "range_m",
        "reject_reason",
    ]
    times = _relative_times(rows_to_write)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for time_s, row in zip(times, rows_to_write):
            output = {field: row.get(field, "") for field in fields}
            output["t_s"] = f"{time_s:.9f}"
            writer.writerow(output)
    return [path]


def _series(rows: Sequence[dict[str, str]], column: str) -> list[float]:
    return [_float_or_none(row.get(column)) or 0.0 for row in rows]


def _relative_times(rows: Sequence[dict[str, str]]) -> list[float]:
    stamps = [_float_or_none(row.get("stamp_ros_s")) for row in rows]
    finite_stamps = [stamp for stamp in stamps if stamp is not None]
    if not finite_stamps:
        return [float(index) for index, _ in enumerate(rows)]
    base = finite_stamps[0]
    times: list[float] = []
    fallback = 0.0
    for stamp in stamps:
        if stamp is None:
            times.append(fallback)
        else:
            fallback = stamp - base
            times.append(fallback)
    return times


def _series_summary(values: Sequence[float]) -> dict[str, Optional[float]]:
    finite_values = [value for value in values if math.isfinite(float(value))]
    if not finite_values:
        return {
            "count": 0,
            "mean": None,
            "mae": None,
            "median": None,
            "median_abs": None,
            "rmse": None,
            "p95": None,
            "p95_abs": None,
            "max": None,
            "max_abs": None,
            "min": None,
        }
    abs_values = [abs(value) for value in finite_values]
    return {
        "count": len(finite_values),
        "mean": mean(finite_values),
        "mae": mean(abs_values),
        "median": median(finite_values),
        "median_abs": median(abs_values),
        "rmse": math.sqrt(mean(value * value for value in finite_values)),
        "p95": _percentile(finite_values, 0.95),
        "p95_abs": _percentile(abs_values, 0.95),
        "max": max(finite_values),
        "max_abs": max(abs_values),
        "min": min(finite_values),
    }


def _series_min_median_mean(values: Sequence[float]) -> dict[str, Optional[float]]:
    finite_values = [value for value in values if math.isfinite(float(value))]
    if not finite_values:
        return {"min": None, "median": None, "mean": None, "max": None}
    return {
        "min": min(finite_values),
        "median": median(finite_values),
        "mean": mean(finite_values),
        "max": max(finite_values),
    }


def _linear_slope(times: Sequence[float], values: Sequence[float]) -> Optional[float]:
    if len(times) != len(values) or len(times) < 2:
        return None
    mean_t = mean(times)
    mean_v = mean(values)
    denominator = sum((time - mean_t) ** 2 for time in times)
    if denominator <= 0.0:
        return None
    return sum((time - mean_t) * (value - mean_v) for time, value in zip(times, values)) / denominator


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * min(max(float(fraction), 0.0), 1.0)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    blend = position - lower
    return ordered[lower] + ((ordered[upper] - ordered[lower]) * blend)


def _numeric_values(values: Iterable[object]) -> list[float]:
    result = []
    for value in values:
        parsed = _float_or_none(value)
        if parsed is not None:
            result.append(parsed)
    return result


def _float_or_none(value: object) -> Optional[float]:
    try:
        if value in ("", None):
            return None
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _fmt(value: object) -> str:
    parsed = _float_or_none(value)
    if parsed is None:
        return "n/a"
    return f"{parsed:.4f}"
