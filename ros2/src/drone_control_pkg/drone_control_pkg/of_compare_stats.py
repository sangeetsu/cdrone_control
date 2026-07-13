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
FUSED_ERROR_COLUMNS = (
    "fused_err_x_m",
    "fused_err_y_m",
    "fused_err_z_m",
    "fused_err_xy_m",
    "fused_err_3d_m",
)
IMU_ONLY_ERROR_COLUMNS = (
    "imu_only_err_x_m",
    "imu_only_err_y_m",
    "imu_only_err_z_m",
    "imu_only_err_xy_m",
    "imu_only_err_3d_m",
)

ESTIMATORS = {
    "of_only": {
        "label": "OF-only",
        "error_columns": ERROR_COLUMNS,
        "pose_columns": ("of_x_m", "of_y_m", "of_z_m"),
        "pose_valid_column": "sample_valid",
    },
    "imu_of_fused": {
        "label": "IMU+OF",
        "error_columns": FUSED_ERROR_COLUMNS,
        "pose_columns": ("fused_x_m", "fused_y_m", "fused_z_m"),
        "pose_valid_column": "fusion_pose_valid",
    },
    "imu_only": {
        "label": "IMU-only",
        "error_columns": IMU_ONLY_ERROR_COLUMNS,
        "pose_columns": ("imu_only_x_m", "imu_only_y_m", "imu_only_z_m"),
        "pose_valid_column": "imu_only_pose_valid",
    },
}

LEGACY_IMU_REPLAY_DEFAULTS = {
    "max_accel_mps2": 6.0,
    "max_velocity_mps": 4.0,
    "max_imu_dt_s": 0.10,
    "velocity_decay_per_s": 0.04,
}


def generate_stats(csv_path: str | Path) -> dict[str, object]:
    path = Path(csv_path).expanduser()
    rows, imu_only_metadata = _prepare_imu_only_rows(_read_rows(path))
    return _generate_stats_from_rows(path, rows, imu_only_metadata)


def _generate_stats_from_rows(
    path: Path,
    rows: Sequence[dict[str, str]],
    imu_only_metadata: dict[str, object],
) -> dict[str, object]:
    of_summary = _estimator_summary(
        rows,
        error_columns=ERROR_COLUMNS,
        sample_valid_column="sample_valid",
    )
    fused_summary = _estimator_summary(
        rows,
        error_columns=FUSED_ERROR_COLUMNS,
        sample_valid_column="fusion_pose_valid",
    )
    imu_only_summary = _estimator_summary(
        rows,
        error_columns=IMU_ONLY_ERROR_COLUMNS,
        sample_valid_column="imu_only_pose_valid",
    )
    imu_only_summary.update(imu_only_metadata)
    comparisons = _comparison_summaries(rows)
    fair_fusion_improvement = _pairwise_fair_improvement(
        rows,
        before_key="of_only",
        after_key="imu_of_fused",
    )

    return {
        "csv_path": str(path),
        "counts": of_summary["counts"],
        "duration_s": _duration_s(rows),
        "time_range": _time_range(rows),
        "mission_states": _value_counts(row.get("mission_state") for row in rows),
        "reject_reasons": _reject_reason_counts(rows),
        "fusion_reject_reasons": _fusion_reject_reason_counts(rows),
        "errors": of_summary["errors"],
        "z_errors": of_summary["z_errors"],
        "quality": _quality_summary(rows),
        "range": _range_summary(rows),
        "imu": _imu_summary(rows),
        "flow_diagnostics": _flow_diagnostics(rows),
        "drift": of_summary["drift"],
        "partial_drift": of_summary["partial_drift"],
        "fusion": fused_summary,
        "imu_only": imu_only_summary,
        "comparisons": comparisons,
        "post_complete": _post_complete_summary(rows),
        # Compatibility key, now calculated from identical mission rows.
        "fusion_improvement": fair_fusion_improvement,
    }


def write_report(
    csv_path: str | Path,
    *,
    output_dir: str | Path | None = None,
    plots: bool = False,
) -> dict[str, object]:
    path = Path(csv_path).expanduser()
    rows, imu_only_metadata = _prepare_imu_only_rows(_read_rows(path))
    summary = _generate_stats_from_rows(path, rows, imu_only_metadata)
    report_dir = (
        Path(output_dir).expanduser()
        if output_dir is not None
        else path.parent / "report"
    )
    report_dir.mkdir(parents=True, exist_ok=True)
    artifact_paths = []
    artifact_paths.extend(_write_timeseries_csv(rows, report_dir))
    artifact_paths.extend(_write_fusion_timeseries_csv(rows, report_dir))
    artifact_paths.extend(
        _write_imu_only_timeseries_csv(
            rows,
            report_dir,
            imu_only_metadata=imu_only_metadata,
        )
    )
    summary["artifacts"] = [
        artifact.relative_to(report_dir).as_posix()
        for artifact in artifact_paths
    ]
    summary["plots"] = _write_plots(rows, report_dir) if plots else []

    summary_path = report_dir / "summary.json"
    markdown_path = report_dir / "summary.md"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(render_markdown(summary), encoding="utf-8")
    return summary


def _estimator_summary(
    rows: Sequence[dict[str, str]],
    *,
    error_columns: Sequence[str],
    sample_valid_column: str,
    require_sample_valid: bool = True,
) -> dict[str, object]:
    valid_3d_rows = _eligible_rows(
        rows,
        columns=(error_columns[4],),
        require_sample_valid=require_sample_valid,
        sample_valid_column=sample_valid_column,
    )
    valid_xyz_rows = _eligible_rows(
        rows,
        columns=error_columns,
        require_sample_valid=require_sample_valid,
        sample_valid_column=sample_valid_column,
    )
    valid_z_rows = _eligible_rows(
        rows,
        columns=(error_columns[2],),
        require_sample_valid=require_sample_valid,
        sample_valid_column=sample_valid_column,
    )
    finite_pose_rows = _eligible_rows(
        rows,
        require_sample_valid=False,
        columns=error_columns,
        sample_valid_column=sample_valid_column,
    )

    errors: dict[str, object] = {}
    for column in error_columns:
        values = _numeric_values(row.get(column) for row in valid_3d_rows)
        errors[column] = _series_summary(values)

    return {
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
                1 for row in rows if _truthy(row.get(sample_valid_column))
            ),
        },
        "errors": errors,
        "z_errors": _series_summary(
            _numeric_values(row.get(error_columns[2]) for row in valid_z_rows)
        ),
        "drift": _drift_summary(
            valid_3d_rows,
            err_xy_column=error_columns[3],
            err_3d_column=error_columns[4],
        ),
        "partial_drift": _drift_summary(
            finite_pose_rows,
            err_xy_column=error_columns[3],
            err_3d_column=error_columns[4],
        ),
    }


def _improvement_summary(
    before_summary: dict[str, object],
    after_summary: dict[str, object],
    *,
    before_columns: Sequence[str] = ERROR_COLUMNS,
    after_columns: Sequence[str] = FUSED_ERROR_COLUMNS,
) -> dict[str, object]:
    before_errors = before_summary.get("errors", {})
    after_errors = after_summary.get("errors", {})
    before_drift = before_summary.get("drift", {})
    after_drift = after_summary.get("drift", {})
    return {
        "err_xy_rmse_delta_m": _delta(
            _nested(before_errors, before_columns[3], "rmse"),
            _nested(after_errors, after_columns[3], "rmse"),
        ),
        "err_xy_p95_delta_m": _delta(
            _nested(before_errors, before_columns[3], "p95_abs"),
            _nested(after_errors, after_columns[3], "p95_abs"),
        ),
        "err_3d_rmse_delta_m": _delta(
            _nested(before_errors, before_columns[4], "rmse"),
            _nested(after_errors, after_columns[4], "rmse"),
        ),
        "err_3d_p95_delta_m": _delta(
            _nested(before_errors, before_columns[4], "p95_abs"),
            _nested(after_errors, after_columns[4], "p95_abs"),
        ),
        "final_xy_drift_delta_m": _delta(
            _mapping_value(before_drift, "final_err_xy_m"),
            _mapping_value(after_drift, "final_err_xy_m"),
        ),
        "final_3d_drift_delta_m": _delta(
            _mapping_value(before_drift, "final_err_3d_m"),
            _mapping_value(after_drift, "final_err_3d_m"),
        ),
    }


def _prepare_imu_only_rows(
    source_rows: Sequence[dict[str, str]],
) -> tuple[list[dict[str, str]], dict[str, object]]:
    rows = [dict(row) for row in source_rows]
    native_pose_rows = sum(
        1
        for row in rows
        if all(
            _float_or_none(row.get(column)) is not None
            for column in ("imu_only_x_m", "imu_only_y_m", "imu_only_z_m")
        )
    )
    if native_pose_rows:
        replay_is_approximate = any(
            _truthy(row.get("replay_approximate")) for row in rows
        )
        for row in rows:
            _populate_imu_only_errors(row)
        return rows, {
            "source": (
                "approximate_legacy_replay"
                if replay_is_approximate
                else "logged"
            ),
            "approximate": replay_is_approximate,
            "pose_rows": native_pose_rows,
            "valid_update_rows": sum(
                1
                for row in rows
                if _truthy(row.get("imu_only_update_valid"))
            ),
            "limitations": (
                ["legacy replay limitations are recorded in replay_metadata.json"]
                if replay_is_approximate
                else []
            ),
        }

    replayed_rows = 0
    unique_imu_samples = 0
    origin: Optional[tuple[float, float, float]] = None
    position = (0.0, 0.0, 0.0)
    velocity = (0.0, 0.0, 0.0)
    last_imu_stamp_s: Optional[float] = None

    for row in rows:
        replay_update_valid = False
        next_origin = _row_origin(
            row,
            allow_mocap_fallback=origin is None,
        )
        if next_origin is not None and (
            origin is None or _vectors_differ(origin, next_origin)
        ):
            origin = next_origin
            position = next_origin
            velocity = (0.0, 0.0, 0.0)
            last_imu_stamp_s = None

        stamp_s = _float_or_none(row.get("imu_stamp_s"))
        is_new_sample = (
            stamp_s is not None
            and (
                last_imu_stamp_s is None
                or not math.isclose(
                    stamp_s,
                    last_imu_stamp_s,
                    rel_tol=0.0,
                    abs_tol=1e-9,
                )
            )
        )
        if origin is not None and is_new_sample:
            dt_s = (
                0.0
                if last_imu_stamp_s is None
                else max(
                    0.0,
                    min(
                        stamp_s - last_imu_stamp_s,
                        LEGACY_IMU_REPLAY_DEFAULTS["max_imu_dt_s"],
                    ),
                )
            )
            last_imu_stamp_s = stamp_s
            accel = tuple(
                _float_or_none(row.get(column))
                for column in (
                    "imu_map_accel_x_mps2",
                    "imu_map_accel_y_mps2",
                    "imu_map_accel_z_mps2",
                )
            )
            if _truthy(row.get("imu_update_valid")) and all(
                value is not None for value in accel
            ):
                replay_update_valid = True
                unique_imu_samples += 1
                accel_vector = _clamp_vector_norm(
                    tuple(float(value) for value in accel),
                    LEGACY_IMU_REPLAY_DEFAULTS["max_accel_mps2"],
                )
                decay = max(
                    0.0,
                    1.0
                    - (
                        LEGACY_IMU_REPLAY_DEFAULTS["velocity_decay_per_s"]
                        * dt_s
                    ),
                )
                position = tuple(
                    position[index]
                    + (velocity[index] * dt_s)
                    + (0.5 * accel_vector[index] * dt_s * dt_s)
                    for index in range(3)
                )
                velocity = _clamp_vector_norm(
                    tuple(
                        (velocity[index] * decay)
                        + (accel_vector[index] * dt_s)
                        for index in range(3)
                    ),
                    LEGACY_IMU_REPLAY_DEFAULTS["max_velocity_mps"],
                )

        if origin is None:
            continue
        replayed_rows += 1
        row["imu_only_x_m"] = _csv_number(position[0])
        row["imu_only_y_m"] = _csv_number(position[1])
        row["imu_only_z_m"] = _csv_number(position[2])
        row["imu_only_vx_mps"] = _csv_number(velocity[0])
        row["imu_only_vy_mps"] = _csv_number(velocity[1])
        row["imu_only_vz_mps"] = _csv_number(velocity[2])
        row["imu_only_pose_valid"] = "1"
        row["imu_only_update_valid"] = "1" if replay_update_valid else "0"
        row["imu_only_reject_reason"] = (
            "" if replay_update_valid else "no_new_logged_imu_snapshot"
        )
        row["imu_only_health_state"] = "approximate_legacy_replay"
        _populate_imu_only_errors(row)

    available = replayed_rows > 0
    return rows, {
        "source": "approximate_legacy_replay" if available else "unavailable",
        "approximate": available,
        "pose_rows": replayed_rows,
        "valid_update_rows": unique_imu_samples,
        "unique_imu_samples_replayed": unique_imu_samples,
        "replay_defaults": dict(LEGACY_IMU_REPLAY_DEFAULTS),
        "limitations": (
            [
                "The source CSV stores logger-rate snapshots, not every IMU callback.",
                "Replay uses already-processed map acceleration and cannot reconstruct "
                "discarded intermediate samples.",
            ]
            if available
            else ["No logged IMU-only pose and no usable origin were available."]
        ),
    }


def _row_origin(
    row: dict[str, str],
    *,
    allow_mocap_fallback: bool,
) -> Optional[tuple[float, float, float]]:
    origin = tuple(
        _float_or_none(row.get(column))
        for column in (
            "origin_reset_mocap_x_m",
            "origin_reset_mocap_y_m",
            "origin_reset_mocap_z_m",
        )
    )
    if all(value is not None for value in origin):
        return tuple(float(value) for value in origin)

    if not allow_mocap_fallback:
        return None
    mocap = tuple(
        _float_or_none(row.get(column))
        for column in ("mocap_x_m", "mocap_y_m", "mocap_z_m")
    )
    if all(value is not None for value in mocap):
        return tuple(float(value) for value in mocap)
    return None


def _vectors_differ(
    first: Sequence[float],
    second: Sequence[float],
    *,
    tolerance: float = 1e-9,
) -> bool:
    return any(
        not math.isclose(a, b, rel_tol=0.0, abs_tol=tolerance)
        for a, b in zip(first, second)
    )


def _populate_imu_only_errors(row: dict[str, str]) -> None:
    position = tuple(
        _float_or_none(row.get(column))
        for column in ("imu_only_x_m", "imu_only_y_m", "imu_only_z_m")
    )
    mocap = tuple(
        _float_or_none(row.get(column))
        for column in ("mocap_x_m", "mocap_y_m", "mocap_z_m")
    )
    if not all(value is not None for value in position + mocap):
        return
    errors = tuple(
        float(position[index]) - float(mocap[index])
        for index in range(3)
    )
    row["imu_only_err_x_m"] = _csv_number(errors[0])
    row["imu_only_err_y_m"] = _csv_number(errors[1])
    row["imu_only_err_z_m"] = _csv_number(errors[2])
    row["imu_only_err_xy_m"] = _csv_number(math.hypot(errors[0], errors[1]))
    row["imu_only_err_3d_m"] = _csv_number(
        math.sqrt(sum(value * value for value in errors))
    )
    if not str(row.get("imu_only_pose_valid", "")).strip():
        row["imu_only_pose_valid"] = "1"


def _comparison_summaries(
    rows: Sequence[dict[str, str]],
) -> dict[str, object]:
    mission_indices, mission_selection = _mission_candidate_indices(rows)
    common_mission_indices = [
        index
        for index in mission_indices
        if _row_has_estimators(rows[index], tuple(ESTIMATORS))
    ]
    delivery_indices, delivery_metadata = _unique_flow_delivery_indices(rows)
    delivery_set = set(delivery_indices)
    use_delivery_mask = bool(delivery_indices)
    common_valid_indices = [
        index
        for index in common_mission_indices
        if (not use_delivery_mask or index in delivery_set)
        and _truthy(rows[index].get("sample_valid"))
        and _fused_update_valid(rows[index])
        and _imu_only_update_valid(rows[index])
    ]
    return {
        "mission_window": _comparison_window_summary(
            rows,
            common_mission_indices,
            candidate_rows=len(mission_indices),
            selection=mission_selection,
        ),
        "valid_updates": _comparison_window_summary(
            rows,
            common_valid_indices,
            candidate_rows=len(mission_indices),
            selection=(
                f"{mission_selection}; shared valid updates; "
                f"flow delivery identity={delivery_metadata['identity_method']}"
            ),
        ),
    }


def _mission_candidate_indices(
    rows: Sequence[dict[str, str]],
) -> tuple[list[int], str]:
    has_mission_state = any(str(row.get("mission_state", "")).strip() for row in rows)
    if not has_mission_state:
        return list(range(len(rows))), "all rows (mission_state unavailable)"
    return [
        index
        for index, row in enumerate(rows)
        if str(row.get("mission_state", "")).strip().upper() == "WAYPOINTS"
    ], "mission_state == WAYPOINTS"


def _row_has_estimators(
    row: dict[str, str],
    estimator_keys: Sequence[str],
) -> bool:
    if not _truthy(row.get("frame_match")):
        return False
    for estimator_key in estimator_keys:
        definition = ESTIMATORS[estimator_key]
        error_columns = definition["error_columns"]
        if not all(
            _float_or_none(row.get(column)) is not None
            for column in error_columns
        ):
            return False
        if estimator_key != "of_only":
            valid_column = str(definition["pose_valid_column"])
            valid_value = str(row.get(valid_column, "")).strip()
            if valid_value and not _truthy(valid_value):
                return False
    return True


def _fused_update_valid(row: dict[str, str]) -> bool:
    value = str(row.get("fusion_flow_valid", "")).strip()
    if value:
        return _truthy(value)
    return _truthy(row.get("fusion_pose_valid"))


def _imu_only_update_valid(row: dict[str, str]) -> bool:
    value = str(row.get("imu_only_update_valid", "")).strip()
    if value:
        return _truthy(value)
    return _truthy(row.get("imu_only_pose_valid"))


def _comparison_window_summary(
    rows: Sequence[dict[str, str]],
    indices: Sequence[int],
    *,
    candidate_rows: int,
    selection: str,
) -> dict[str, object]:
    common_rows = [rows[index] for index in indices]
    estimator_summaries = {
        estimator_key: _estimator_summary(
            common_rows,
            error_columns=definition["error_columns"],
            sample_valid_column=str(definition["pose_valid_column"]),
            require_sample_valid=False,
        )
        for estimator_key, definition in ESTIMATORS.items()
    }
    of_summary = estimator_summaries["of_only"]
    return {
        "selection": selection,
        "candidate_rows": candidate_rows,
        "common_rows": len(common_rows),
        "common_start_stamp_s": _endpoint_stamp(common_rows, first=True),
        "common_end_stamp_s": _endpoint_stamp(common_rows, first=False),
        "estimators": estimator_summaries,
        "deltas_vs_of": {
            "imu_of_fused": _improvement_summary(
                of_summary,
                estimator_summaries["imu_of_fused"],
                before_columns=ERROR_COLUMNS,
                after_columns=FUSED_ERROR_COLUMNS,
            ),
            "imu_only": _improvement_summary(
                of_summary,
                estimator_summaries["imu_only"],
                before_columns=ERROR_COLUMNS,
                after_columns=IMU_ONLY_ERROR_COLUMNS,
            ),
        },
    }


def _pairwise_fair_improvement(
    rows: Sequence[dict[str, str]],
    *,
    before_key: str,
    after_key: str,
) -> dict[str, object]:
    indices, selection = _mission_candidate_indices(rows)
    common_rows = [
        rows[index]
        for index in indices
        if _row_has_estimators(rows[index], (before_key, after_key))
    ]
    before_definition = ESTIMATORS[before_key]
    after_definition = ESTIMATORS[after_key]
    before_summary = _estimator_summary(
        common_rows,
        error_columns=before_definition["error_columns"],
        sample_valid_column=str(before_definition["pose_valid_column"]),
        require_sample_valid=False,
    )
    after_summary = _estimator_summary(
        common_rows,
        error_columns=after_definition["error_columns"],
        sample_valid_column=str(after_definition["pose_valid_column"]),
        require_sample_valid=False,
    )
    result = _improvement_summary(
        before_summary,
        after_summary,
        before_columns=before_definition["error_columns"],
        after_columns=after_definition["error_columns"],
    )
    result.update(
        {
            "comparison_rows": len(common_rows),
            "comparison_start_stamp_s": _endpoint_stamp(common_rows, first=True),
            "comparison_end_stamp_s": _endpoint_stamp(common_rows, first=False),
            "selection": selection,
        }
    )
    return result


def _endpoint_stamp(
    rows: Sequence[dict[str, str]],
    *,
    first: bool,
) -> Optional[float]:
    if not rows:
        return None
    row = rows[0] if first else rows[-1]
    return _float_or_none(row.get("stamp_ros_s"))


def _post_complete_summary(
    rows: Sequence[dict[str, str]],
) -> dict[str, object]:
    candidates = [
        row
        for row in rows
        if str(row.get("mission_state", "")).strip().upper() == "COMPLETE"
    ]
    common_rows = [
        row for row in candidates if _row_has_estimators(row, tuple(ESTIMATORS))
    ]
    estimators: dict[str, object] = {}
    for estimator_key, definition in ESTIMATORS.items():
        summary = _estimator_summary(
            common_rows,
            error_columns=definition["error_columns"],
            sample_valid_column=str(definition["pose_valid_column"]),
            require_sample_valid=False,
        )
        summary["position_displacement"] = _position_displacement(
            common_rows,
            definition["pose_columns"],
        )
        estimators[estimator_key] = summary
    return {
        "selection": "mission_state == COMPLETE; shared finite poses",
        "candidate_rows": len(candidates),
        "common_rows": len(common_rows),
        "common_start_stamp_s": _endpoint_stamp(common_rows, first=True),
        "common_end_stamp_s": _endpoint_stamp(common_rows, first=False),
        "estimators": estimators,
    }


def _position_displacement(
    rows: Sequence[dict[str, str]],
    pose_columns: Sequence[str],
) -> dict[str, Optional[float]]:
    if not rows:
        return {"xy_m": None, "z_m": None, "3d_m": None}
    first = [_float_or_none(rows[0].get(column)) for column in pose_columns]
    last = [_float_or_none(rows[-1].get(column)) for column in pose_columns]
    if not all(value is not None for value in first + last):
        return {"xy_m": None, "z_m": None, "3d_m": None}
    delta = [float(last[index]) - float(first[index]) for index in range(3)]
    return {
        "xy_m": math.hypot(delta[0], delta[1]),
        "z_m": delta[2],
        "3d_m": math.sqrt(sum(value * value for value in delta)),
    }


def _flow_diagnostics(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    delivery_indices, identity = _unique_flow_delivery_indices(rows)
    flow_row_count = int(identity["flow_rows"])
    delivery_rows = [rows[index] for index in delivery_indices]
    delivery_times = [_flow_delivery_stamp(row) for row in delivery_rows]
    finite_delivery_times = [value for value in delivery_times if value is not None]
    intervals = [
        current - previous
        for previous, current in zip(finite_delivery_times, finite_delivery_times[1:])
        if current > previous
    ]
    logged_intervals = _numeric_values(
        row.get("flow_inter_message_dt_s") for row in delivery_rows
    )
    if logged_intervals:
        intervals = [value for value in logged_intervals if value > 0.0]
    integration_times = _numeric_values(row.get("flow_dt_s") for row in delivery_rows)
    span_s = (
        finite_delivery_times[-1] - finite_delivery_times[0]
        if len(finite_delivery_times) >= 2
        else None
    )
    coverage_ratio = (
        sum(integration_times) / span_s
        if span_s is not None and span_s > 0.0
        else None
    )
    interval_coverage = [
        integration_time / interval
        for integration_time, interval in zip(integration_times[1:], intervals)
        if interval > 0.0
    ]
    logged_coverage = _numeric_values(
        row.get("flow_integration_coverage_ratio") for row in delivery_rows
    )
    if logged_coverage:
        interval_coverage = logged_coverage

    gyro_xy_valid = sum(
        1
        for row in delivery_rows
        if _float_or_none(row.get("flow_integrated_xgyro")) is not None
        and _float_or_none(row.get("flow_integrated_ygyro")) is not None
    )
    gyro_xyz_valid = sum(
        1
        for row in delivery_rows
        if all(
            _float_or_none(row.get(column)) is not None
            for column in (
                "flow_integrated_xgyro",
                "flow_integrated_ygyro",
                "flow_integrated_zgyro",
            )
        )
    )
    velocity_correlation = _flow_mocap_velocity_correlation(delivery_rows)
    stationary_rms = _stationary_flow_rms(delivery_rows)
    repeat_rows = max(flow_row_count - len(delivery_rows), 0)
    return {
        "identity_method": identity["identity_method"],
        "identity_approximate": identity["identity_approximate"],
        "logger_rows_with_flow": flow_row_count,
        "unique_deliveries": len(delivery_rows),
        "repeated_logger_rows": repeat_rows,
        "repeat_rate": repeat_rows / flow_row_count if flow_row_count else None,
        "delivery_span_s": span_s,
        "delivery_rate_hz": (
            (len(finite_delivery_times) - 1) / span_s
            if span_s is not None
            and span_s > 0.0
            and len(finite_delivery_times) >= 2
            else None
        ),
        "inter_message_dt_s": _series_summary(intervals),
        "integration_time_s": _series_summary(integration_times),
        "total_integration_time_s": sum(integration_times),
        "integration_coverage_ratio": coverage_ratio,
        "per_delivery_coverage_ratio": _series_summary(interval_coverage),
        "gyro": {
            "xy_valid_deliveries": gyro_xy_valid,
            "xy_valid_rate": (
                gyro_xy_valid / len(delivery_rows) if delivery_rows else None
            ),
            "xyz_valid_deliveries": gyro_xyz_valid,
            "xyz_valid_rate": (
                gyro_xyz_valid / len(delivery_rows) if delivery_rows else None
            ),
        },
        "flow_vs_mocap_velocity_correlation": velocity_correlation,
        "stationary_flow_rms": stationary_rms,
    }


def _unique_flow_delivery_indices(
    rows: Sequence[dict[str, str]],
) -> tuple[list[int], dict[str, object]]:
    flow_indices = [
        index
        for index, row in enumerate(rows)
        if _row_contains_flow(row)
    ]
    sequence_values = [
        str(rows[index].get("flow_sequence", "")).strip()
        for index in flow_indices
    ]
    source_values = [
        _float_or_none(rows[index].get("flow_source_stamp_s"))
        for index in flow_indices
    ]
    if len({value for value in sequence_values if value}) > 1:
        method = "flow_sequence"
        identified = [
            (index, value)
            for index, value in zip(flow_indices, sequence_values)
            if value
        ]
        approximate = False
    elif len({value for value in source_values if value is not None}) > 1:
        method = "flow_source_stamp_s"
        identified = [
            (index, value)
            for index, value in zip(flow_indices, source_values)
            if value is not None
        ]
        approximate = False
    else:
        method = "legacy_payload_transition"
        identified = [
            (index, _flow_payload_signature(rows[index]))
            for index in flow_indices
        ]
        approximate = True

    deliveries: list[int] = []
    previous: object = object()
    for index, identity in identified:
        if not deliveries or identity != previous:
            deliveries.append(index)
        previous = identity
    return deliveries, {
        "identity_method": method,
        "identity_approximate": approximate,
        "flow_rows": len(flow_indices),
    }


def _row_contains_flow(row: dict[str, str]) -> bool:
    return any(
        str(row.get(column, "")).strip()
        for column in (
            "flow_sequence",
            "flow_source_stamp_s",
            "flow_integrated_x",
            "flow_integrated_y",
        )
    )


def _flow_payload_signature(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(
        str(row.get(column, "")).strip()
        for column in (
            "flow_integrated_x",
            "flow_integrated_y",
            "flow_integrated_xgyro",
            "flow_integrated_ygyro",
            "flow_integrated_zgyro",
            "flow_quality",
            "flow_distance_m",
            "flow_dt_s",
        )
    )


def _flow_delivery_stamp(row: dict[str, str]) -> Optional[float]:
    for column in (
        "flow_source_stamp_s",
        "flow_receive_stamp_s",
        "stamp_ros_s",
    ):
        value = _float_or_none(row.get(column))
        if value is not None:
            return value
    return None


def _flow_velocity(row: dict[str, str]) -> Optional[tuple[float, float]]:
    vx = _float_or_none(row.get("fusion_flow_vx_mps"))
    vy = _float_or_none(row.get("fusion_flow_vy_mps"))
    fusion_valid_value = str(row.get("fusion_flow_valid", "")).strip()
    if (
        vx is not None
        and vy is not None
        and (not fusion_valid_value or _truthy(fusion_valid_value))
    ):
        return vx, vy
    dx = _float_or_none(row.get("map_dx_m"))
    dy = _float_or_none(row.get("map_dy_m"))
    dt_s = _float_or_none(row.get("flow_dt_s"))
    sample_valid_value = str(row.get("sample_valid", "")).strip()
    delta_is_valid = (
        _truthy(fusion_valid_value)
        or _truthy(sample_valid_value)
        or (not fusion_valid_value and not sample_valid_value)
    )
    if (
        delta_is_valid
        and dx is not None
        and dy is not None
        and dt_s is not None
        and dt_s > 0.0
    ):
        return dx / dt_s, dy / dt_s

    flow_x = _float_or_none(row.get("flow_integrated_x"))
    flow_y = _float_or_none(row.get("flow_integrated_y"))
    distance_m = _float_or_none(row.get("flow_distance_m"))
    if distance_m is None:
        distance_m = _float_or_none(row.get("range_m"))
    if (
        flow_x is None
        or flow_y is None
        or distance_m is None
        or dt_s is None
        or dt_s <= 0.0
    ):
        return None
    xgyro = _float_or_none(row.get("flow_integrated_xgyro"))
    ygyro = _float_or_none(row.get("flow_integrated_ygyro"))
    if xgyro is not None:
        flow_x -= xgyro
    if ygyro is not None:
        flow_y -= ygyro
    body_vx = distance_m * flow_y / dt_s
    body_vy = distance_m * (-flow_x) / dt_s
    yaw_rad = _float_or_none(row.get("yaw_used_rad"))
    if yaw_rad is None:
        yaw_rad = _float_or_none(row.get("imu_yaw_rad"))
    if yaw_rad is None:
        return body_vx, body_vy
    return (
        (math.cos(yaw_rad) * body_vx) - (math.sin(yaw_rad) * body_vy),
        (math.sin(yaw_rad) * body_vx) + (math.cos(yaw_rad) * body_vy),
    )


def _flow_mocap_velocity_correlation(
    delivery_rows: Sequence[dict[str, str]],
) -> dict[str, object]:
    pairs: list[tuple[float, float, float, float]] = []
    for previous, current in zip(delivery_rows, delivery_rows[1:]):
        if str(current.get("mission_state", "")).strip().upper() != "WAYPOINTS":
            continue
        previous_stamp = _flow_delivery_stamp(previous)
        current_stamp = _flow_delivery_stamp(current)
        flow_velocity = _flow_velocity(current)
        previous_mocap = tuple(
            _float_or_none(previous.get(column))
            for column in ("mocap_x_m", "mocap_y_m")
        )
        current_mocap = tuple(
            _float_or_none(current.get(column))
            for column in ("mocap_x_m", "mocap_y_m")
        )
        if (
            previous_stamp is None
            or current_stamp is None
            or current_stamp <= previous_stamp
            or flow_velocity is None
            or not all(value is not None for value in previous_mocap + current_mocap)
        ):
            continue
        dt_s = current_stamp - previous_stamp
        mocap_vx = (float(current_mocap[0]) - float(previous_mocap[0])) / dt_s
        mocap_vy = (float(current_mocap[1]) - float(previous_mocap[1])) / dt_s
        pairs.append((flow_velocity[0], flow_velocity[1], mocap_vx, mocap_vy))

    flow_x = [pair[0] for pair in pairs]
    flow_y = [pair[1] for pair in pairs]
    mocap_x = [pair[2] for pair in pairs]
    mocap_y = [pair[3] for pair in pairs]
    flow_speed = [math.hypot(pair[0], pair[1]) for pair in pairs]
    mocap_speed = [math.hypot(pair[2], pair[3]) for pair in pairs]
    return {
        "samples": len(pairs),
        "flow_velocity_source": "fusion_flow_v* or map_delta/integration_time",
        "x_pearson": _pearson(flow_x, mocap_x),
        "y_pearson": _pearson(flow_y, mocap_y),
        "speed_pearson": _pearson(flow_speed, mocap_speed),
        "xy_component_pearson": _pearson(flow_x + flow_y, mocap_x + mocap_y),
    }


def _stationary_flow_rms(
    delivery_rows: Sequence[dict[str, str]],
) -> dict[str, object]:
    stationary_rows = [
        row
        for row in delivery_rows
        if str(row.get("mission_state", "")).strip().upper()
        in {"IDLE", "COMPLETE"}
    ]
    velocities = [
        velocity
        for velocity in (_flow_velocity(row) for row in stationary_rows)
        if velocity is not None
    ]
    integrated = [
        (x, y)
        for row in stationary_rows
        if (x := _float_or_none(row.get("flow_integrated_x"))) is not None
        and (y := _float_or_none(row.get("flow_integrated_y"))) is not None
    ]
    return {
        "selection": "mission_state in {IDLE, COMPLETE}",
        "deliveries": len(stationary_rows),
        "velocity_samples": len(velocities),
        "vx_rms_mps": _rms([value[0] for value in velocities]),
        "vy_rms_mps": _rms([value[1] for value in velocities]),
        "speed_rms_mps": _rms(
            [math.hypot(value[0], value[1]) for value in velocities]
        ),
        "integrated_x_rms_rad": _rms([value[0] for value in integrated]),
        "integrated_y_rms_rad": _rms([value[1] for value in integrated]),
        "integrated_xy_rms_rad": _rms(
            [math.hypot(value[0], value[1]) for value in integrated]
        ),
    }


def _pearson(first: Sequence[float], second: Sequence[float]) -> Optional[float]:
    if len(first) != len(second) or len(first) < 2:
        return None
    first_mean = mean(first)
    second_mean = mean(second)
    first_delta = [value - first_mean for value in first]
    second_delta = [value - second_mean for value in second]
    denominator = math.sqrt(
        sum(value * value for value in first_delta)
        * sum(value * value for value in second_delta)
    )
    if denominator <= 0.0:
        return None
    return sum(
        a * b for a, b in zip(first_delta, second_delta)
    ) / denominator


def _rms(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return math.sqrt(mean(value * value for value in values))


def _clamp_vector_norm(
    vector: Sequence[float],
    maximum_norm: float,
) -> tuple[float, ...]:
    norm = math.sqrt(sum(value * value for value in vector))
    if maximum_norm <= 0.0 or norm <= maximum_norm or norm <= 0.0:
        return tuple(float(value) for value in vector)
    scale = maximum_norm / norm
    return tuple(float(value) * scale for value in vector)


def _csv_number(value: float) -> str:
    return f"{float(value):.9f}"


def render_markdown(summary: dict[str, object]) -> str:
    counts = summary.get("counts", {})
    range_summary = summary.get("range", {})
    quality = summary.get("quality", {})
    imu = summary.get("imu", {})
    imu_only = summary.get("imu_only", {})
    comparisons = summary.get("comparisons", {})
    mission_window = (
        comparisons.get("mission_window", {})
        if isinstance(comparisons, dict)
        else {}
    )
    valid_updates = (
        comparisons.get("valid_updates", {})
        if isinstance(comparisons, dict)
        else {}
    )
    post_complete = summary.get("post_complete", {})
    flow = summary.get("flow_diagnostics", {})
    gyro = flow.get("gyro", {}) if isinstance(flow, dict) else {}
    correlation = (
        flow.get("flow_vs_mocap_velocity_correlation", {})
        if isinstance(flow, dict)
        else {}
    )
    stationary = (
        flow.get("stationary_flow_rms", {})
        if isinstance(flow, dict)
        else {}
    )
    plots = summary.get("plots", [])
    artifacts = summary.get("artifacts", [])

    lines = [
        "# IMU / Optical-Flow Estimator Summary",
        "",
        f"- CSV: `{summary.get('csv_path', '')}`",
        f"- Duration: {_fmt(summary.get('duration_s'))} s",
        f"- Rows: {counts.get('rows', 0)}",
        f"- Frame-match rows: {counts.get('frame_match_rows', 0)}",
        f"- IMU-only source: `{imu_only.get('source', 'unavailable')}`",
    ]
    if isinstance(imu_only, dict) and imu_only.get("approximate"):
        lines.extend(
            [
                "",
                "> **IMU-only is an approximate legacy replay.** The log stores "
                "logger-rate snapshots rather than every IMU callback, so the replay "
                "cannot reproduce the original high-rate propagation exactly.",
            ]
        )

    _append_comparison_markdown(
        lines,
        "Mission-window comparison",
        mission_window,
    )
    _append_comparison_markdown(
        lines,
        "Valid-update comparison",
        valid_updates,
    )

    lines.extend(
        [
            "",
            "## Post-COMPLETE drift",
            "",
            f"- Common rows: {post_complete.get('common_rows', 0)}",
            f"- Common interval: "
            f"{_fmt(post_complete.get('common_start_stamp_s'))} to "
            f"{_fmt(post_complete.get('common_end_stamp_s'))} s",
            "",
            "| Estimator | Error growth XY m | Final error XY m | "
            "Estimator displacement XY m |",
            "| --- | ---: | ---: | ---: |",
        ]
    )
    post_estimators = (
        post_complete.get("estimators", {})
        if isinstance(post_complete, dict)
        else {}
    )
    for estimator_key, definition in ESTIMATORS.items():
        estimator = post_estimators.get(estimator_key, {})
        drift = estimator.get("drift", {}) if isinstance(estimator, dict) else {}
        displacement = (
            estimator.get("position_displacement", {})
            if isinstance(estimator, dict)
            else {}
        )
        lines.append(
            f"| {definition['label']} | "
            f"{_fmt(drift.get('delta_err_xy_m'))} | "
            f"{_fmt(drift.get('final_err_xy_m'))} | "
            f"{_fmt(displacement.get('xy_m'))} |"
        )

    lines.extend(
        [
            "",
            "## Flow delivery and signal health",
            "",
            f"- Delivery identity: `{flow.get('identity_method', 'n/a')}` "
            f"(approximate: {flow.get('identity_approximate', False)})",
            f"- Unique deliveries / logger rows: "
            f"{flow.get('unique_deliveries', 0)} / "
            f"{flow.get('logger_rows_with_flow', 0)}",
            f"- Delivery rate: {_fmt(flow.get('delivery_rate_hz'))} Hz",
            f"- Repeated logger-row rate: {_fmt(flow.get('repeat_rate'))}",
            f"- Integration coverage: "
            f"{_fmt(flow.get('integration_coverage_ratio'))}",
            f"- Integration-window median: "
            f"{_fmt(_nested(flow, 'integration_time_s', 'median'))} s",
            f"- Inter-message median: "
            f"{_fmt(_nested(flow, 'inter_message_dt_s', 'median'))} s",
            f"- Flow gyro XY valid deliveries: "
            f"{gyro.get('xy_valid_deliveries', 0)} "
            f"({_fmt(gyro.get('xy_valid_rate'))})",
            f"- Flow/mocap velocity Pearson x/y/speed: "
            f"{_fmt(correlation.get('x_pearson'))} / "
            f"{_fmt(correlation.get('y_pearson'))} / "
            f"{_fmt(correlation.get('speed_pearson'))} "
            f"({correlation.get('samples', 0)} samples)",
            f"- Stationary flow speed RMS: "
            f"{_fmt(stationary.get('speed_rms_mps'))} m/s",
            f"- Stationary integrated-flow XY RMS: "
            f"{_fmt(stationary.get('integrated_xy_rms_rad'))} rad",
            f"- Range median: {_fmt(range_summary.get('median'))} m",
            f"- Range p95: {_fmt(range_summary.get('p95'))} m",
            f"- Quality median: {_fmt(quality.get('median'))}",
            "",
            "## IMU Health",
            "",
            f"- IMU rows: {imu.get('rows', 0)}",
            f"- Valid IMU update rows: {imu.get('valid_update_rows', 0)}",
            f"- IMU dt median: {_fmt(imu.get('dt_median_s'))} s",
            f"- Map accel norm median: "
            f"{_fmt(imu.get('map_accel_norm_median_mps2'))} m/s^2",
            f"- Map accel norm p95: "
            f"{_fmt(imu.get('map_accel_norm_p95_mps2'))} m/s^2",
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


def _append_comparison_markdown(
    lines: list[str],
    title: str,
    window: object,
) -> None:
    mapping = window if isinstance(window, dict) else {}
    estimators = mapping.get("estimators", {})
    lines.extend(
        [
            "",
            f"## {title}",
            "",
            f"- Selection: {mapping.get('selection', 'n/a')}",
            f"- Shared rows: {mapping.get('common_rows', 0)} / "
            f"{mapping.get('candidate_rows', 0)} candidates",
            f"- Common interval: {_fmt(mapping.get('common_start_stamp_s'))} to "
            f"{_fmt(mapping.get('common_end_stamp_s'))} s",
            "",
            "| Estimator | Rows | XY RMSE m | XY P95 m | Z RMSE m | "
            "3D RMSE m | Final XY error m |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for estimator_key, definition in ESTIMATORS.items():
        estimator = (
            estimators.get(estimator_key, {})
            if isinstance(estimators, dict)
            else {}
        )
        counts = estimator.get("counts", {}) if isinstance(estimator, dict) else {}
        errors = estimator.get("errors", {}) if isinstance(estimator, dict) else {}
        drift = estimator.get("drift", {}) if isinstance(estimator, dict) else {}
        error_columns = definition["error_columns"]
        lines.append(
            f"| {definition['label']} | {counts.get('valid_3d_rows', 0)} | "
            f"{_fmt(_nested(errors, error_columns[3], 'rmse'))} | "
            f"{_fmt(_nested(errors, error_columns[3], 'p95_abs'))} | "
            f"{_fmt(_nested(errors, error_columns[2], 'rmse'))} | "
            f"{_fmt(_nested(errors, error_columns[4], 'rmse'))} | "
            f"{_fmt(drift.get('final_err_xy_m'))} |"
        )
    deltas = mapping.get("deltas_vs_of", {})
    fused_delta = deltas.get("imu_of_fused", {}) if isinstance(deltas, dict) else {}
    imu_delta = deltas.get("imu_only", {}) if isinstance(deltas, dict) else {}
    lines.extend(
        [
            "",
            "Positive deltas mean the estimator reduced error vs OF-only on the "
            "same rows.",
            "",
            f"- IMU+OF XY RMSE delta: "
            f"{_fmt(fused_delta.get('err_xy_rmse_delta_m'))} m",
            f"- IMU-only XY RMSE delta: "
            f"{_fmt(imu_delta.get('err_xy_rmse_delta_m'))} m",
        ]
    )


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
    fusion = summary.get("fusion", {})
    if isinstance(fusion, dict):
        fusion_counts = fusion.get("counts", {})
        fusion_errors = fusion.get("errors", {})
        if fusion_counts.get("valid_3d_rows", 0):
            fused_xy = fusion_errors.get("fused_err_xy_m", {})
            fused_3d = fusion_errors.get("fused_err_3d_m", {})
            print(
                "fused_err_xy_rmse_m="
                f"{_fmt(fused_xy.get('rmse'))} "
                "fused_err_xy_p95_m="
                f"{_fmt(fused_xy.get('p95_abs'))} "
                "fused_err_3d_rmse_m="
                f"{_fmt(fused_3d.get('rmse'))}"
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
    sample_valid_column: str = "sample_valid",
) -> list[dict[str, str]]:
    result = []
    for row in rows:
        if not _truthy(row.get("frame_match")):
            continue
        if require_sample_valid and not _truthy(row.get(sample_valid_column)):
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


def _fusion_reject_reason_counts(rows: Sequence[dict[str, str]]) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for row in rows:
        reason = str(row.get("fusion_reject_reason", "") or "valid")
        counter[reason] += 1
    return dict(counter.most_common())


def _imu_summary(rows: Sequence[dict[str, str]]) -> dict[str, object]:
    imu_rows = [
        row
        for row in rows
        if _float_or_none(row.get("imu_stamp_s")) is not None
        or _float_or_none(row.get("imu_accel_x_mps2")) is not None
    ]
    accel_norms = []
    for row in rows:
        ax = _float_or_none(row.get("imu_map_accel_x_mps2"))
        ay = _float_or_none(row.get("imu_map_accel_y_mps2"))
        az = _float_or_none(row.get("imu_map_accel_z_mps2"))
        if ax is None or ay is None or az is None:
            continue
        accel_norms.append(math.sqrt((ax * ax) + (ay * ay) + (az * az)))

    dt_values = _numeric_values(row.get("imu_update_dt_s") for row in rows)
    return {
        "rows": len(imu_rows),
        "valid_update_rows": sum(
            1 for row in rows if _truthy(row.get("imu_update_valid"))
        ),
        "dt_median_s": median(dt_values) if dt_values else None,
        "dt_p95_s": _percentile(dt_values, 0.95) if dt_values else None,
        "map_accel_norm_median_mps2": (
            median(accel_norms) if accel_norms else None
        ),
        "map_accel_norm_p95_mps2": (
            _percentile(accel_norms, 0.95) if accel_norms else None
        ),
    }


def _value_counts(values: Iterable[object]) -> dict[str, int]:
    counter: Counter[str] = Counter(str(value or "") for value in values)
    return dict(counter.most_common())


def _drift_summary(
    rows: Sequence[dict[str, str]],
    *,
    err_xy_column: str = "err_xy_m",
    err_3d_column: str = "err_3d_m",
) -> dict[str, Optional[float]]:
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
    err_xy = _numeric_values(row.get(err_xy_column) for row in rows)
    err_3d = _numeric_values(row.get(err_3d_column) for row in rows)
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


def _write_plots(
    rows: Sequence[dict[str, str]],
    report_dir: Path,
) -> list[str]:
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
    outputs.extend(_plot_imu_only_estimator(rows, plot_dir, plt))
    outputs.extend(_plot_flow_health(rows, plot_dir, plt))
    outputs.extend(_plot_imu_health(rows, plot_dir, plt))
    return [path.relative_to(report_dir).as_posix() for path in outputs]


def _plot_errors(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]:
    mission_indices, _ = _mission_candidate_indices(rows)
    plot_rows = [
        rows[index]
        for index in mission_indices
        if _row_has_estimators(rows[index], ("of_only", "imu_of_fused"))
    ]
    if len(plot_rows) < 2:
        return []

    path = plot_dir / "errors_over_time.png"
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    times = _relative_times(plot_rows)
    axes[0].plot(times, _series(plot_rows, "err_x_m"), label="OF x")
    axes[0].plot(times, _series(plot_rows, "err_y_m"), label="OF y")
    axes[0].plot(times, _series(plot_rows, "err_z_m"), label="OF z")
    axes[1].plot(times, _series(plot_rows, "err_xy_m"), label="OF xy")
    axes[1].plot(times, _series(plot_rows, "err_3d_m"), label="OF 3d")
    axes[0].plot(
        times,
        _series(plot_rows, "fused_err_x_m"),
        "--",
        label="IMU+OF x",
    )
    axes[0].plot(
        times,
        _series(plot_rows, "fused_err_y_m"),
        "--",
        label="IMU+OF y",
    )
    axes[0].plot(
        times,
        _series(plot_rows, "fused_err_z_m"),
        "--",
        label="IMU+OF z",
    )
    axes[1].plot(
        times,
        _series(plot_rows, "fused_err_xy_m"),
        "--",
        label="IMU+OF xy",
    )
    axes[1].plot(
        times,
        _series(plot_rows, "fused_err_3d_m"),
        "--",
        label="IMU+OF 3d",
    )
    axes[0].set_ylabel("signed error (m)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel("error norm (m)")
    axes[1].set_xlabel("time since first valid sample (s)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("Mocap vs Estimator Error Over Time")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return [path]


def _plot_drift(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]:
    mission_indices, _ = _mission_candidate_indices(rows)
    plot_rows = [
        rows[index]
        for index in mission_indices
        if _row_has_estimators(rows[index], ("of_only", "imu_of_fused"))
    ]
    if len(plot_rows) < 2:
        return []

    path = plot_dir / "drift_over_time.png"
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    times = _relative_times(plot_rows)
    err_xy = _series(plot_rows, "err_xy_m")
    err_3d = _series(plot_rows, "err_3d_m")
    fused_xy = _series(plot_rows, "fused_err_xy_m")
    fused_3d = _series(plot_rows, "fused_err_3d_m")
    ax.plot(times, err_xy, label="OF xy drift")
    ax.plot(times, err_3d, label="OF 3d drift")
    ax.plot(times, fused_xy, "--", label="IMU+OF xy drift")
    ax.plot(times, fused_3d, "--", label="IMU+OF 3d drift")
    _plot_fit(ax, times, err_xy, "OF xy fit")
    _plot_fit(ax, times, fused_xy, "IMU+OF xy fit")
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
    mission_indices, _ = _mission_candidate_indices(rows)
    rows_to_plot = [
        rows[index]
        for index in mission_indices
        if _row_has_estimators(rows[index], ("of_only", "imu_of_fused"))
    ]
    if len(rows_to_plot) < 2:
        return []

    path = plot_dir / "trajectory_xy.png"
    fig, ax = plt.subplots(1, 1, figsize=(7, 7))
    ax.plot(
        _series(rows_to_plot, "mocap_x_m"),
        _series(rows_to_plot, "mocap_y_m"),
        label="mocap",
    )
    ax.plot(
        _series(rows_to_plot, "of_x_m"),
        _series(rows_to_plot, "of_y_m"),
        label="OF dead-reckon",
    )
    ax.plot(
        _series(rows_to_plot, "fused_x_m"),
        _series(rows_to_plot, "fused_y_m"),
        "--",
        label="IMU+OF fused",
    )
    ax.set_title("XY Trajectory (shared WAYPOINTS rows)")
    ax.set_xlabel("x map (m)")
    ax.set_ylabel("y map (m)")
    ax.axis("equal")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return [path]


def _plot_imu_only_estimator(
    rows: Sequence[dict[str, str]],
    plot_dir: Path,
    plt,
) -> list[Path]:
    mission_indices, _ = _mission_candidate_indices(rows)
    imu_rows = [
        rows[index]
        for index in mission_indices
        if _row_has_estimators(rows[index], ("imu_only",))
    ]
    if len(imu_rows) < 2:
        return []

    path = plot_dir / "imu_only_estimator.png"
    times = _relative_times(imu_rows)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].plot(times, _series(imu_rows, "imu_only_err_xy_m"), label="XY")
    axes[0].plot(times, _series(imu_rows, "imu_only_err_3d_m"), label="3D")
    axes[0].set_title("IMU-only error (separate scale)")
    axes[0].set_xlabel("time since first mission sample (s)")
    axes[0].set_ylabel("error norm (m)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(
        _series(imu_rows, "mocap_x_m"),
        _series(imu_rows, "mocap_y_m"),
        label="mocap",
    )
    axes[1].plot(
        _series(imu_rows, "imu_only_x_m"),
        _series(imu_rows, "imu_only_y_m"),
        label="IMU-only",
    )
    axes[1].set_title("IMU-only XY trajectory")
    axes[1].set_xlabel("x map (m)")
    axes[1].set_ylabel("y map (m)")
    axes[1].axis("equal")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
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
    axes[0].axhline(
        0.2,
        color="tab:red",
        linestyle="--",
        label="default min accepted range",
    )
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


def _plot_imu_health(rows: Sequence[dict[str, str]], plot_dir: Path, plt) -> list[Path]:
    imu_rows = [
        row
        for row in rows
        if _float_or_none(row.get("imu_accel_x_mps2")) is not None
        or _float_or_none(row.get("imu_map_accel_x_mps2")) is not None
    ]
    if len(imu_rows) < 2:
        return []

    times = _relative_times(imu_rows)
    raw_norm = []
    map_norm = []
    for row in imu_rows:
        raw_norm.append(
            _norm_or_zero(
                row.get("imu_accel_x_mps2"),
                row.get("imu_accel_y_mps2"),
                row.get("imu_accel_z_mps2"),
            )
        )
        map_norm.append(
            _norm_or_zero(
                row.get("imu_map_accel_x_mps2"),
                row.get("imu_map_accel_y_mps2"),
                row.get("imu_map_accel_z_mps2"),
            )
        )

    path = plot_dir / "imu_health_over_time.png"
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(times, raw_norm, label="raw accel norm")
    axes[0].plot(times, map_norm, label="map accel norm after gravity")
    axes[0].set_ylabel("accel norm (m/s^2)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(times, _series(imu_rows, "imu_yaw_rad"), label="yaw")
    axes[1].set_xlabel("time since first IMU row (s)")
    axes[1].set_ylabel("yaw (rad)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("IMU Health Over Time")
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


def _write_fusion_timeseries_csv(
    rows: Sequence[dict[str, str]],
    report_dir: Path,
) -> list[Path]:
    rows_to_write = _eligible_rows(
        rows,
        columns=FUSED_ERROR_COLUMNS,
        sample_valid_column="fusion_pose_valid",
    )
    if not rows_to_write:
        return []

    path = report_dir / "fusion_timeseries.csv"
    fields = [
        "t_s",
        "stamp_ros_s",
        "mission_state",
        "fused_err_x_m",
        "fused_err_y_m",
        "fused_err_z_m",
        "fused_err_xy_m",
        "fused_err_3d_m",
        "mocap_x_m",
        "mocap_y_m",
        "mocap_z_m",
        "fused_x_m",
        "fused_y_m",
        "fused_z_m",
        "fused_vx_mps",
        "fused_vy_mps",
        "fused_vz_mps",
        "imu_yaw_rad",
        "imu_update_dt_s",
        "imu_map_accel_x_mps2",
        "imu_map_accel_y_mps2",
        "imu_map_accel_z_mps2",
        "fusion_flow_valid",
        "fusion_update_type",
        "fusion_reject_reason",
        "fusion_map_dx_m",
        "fusion_map_dy_m",
        "fusion_flow_vx_mps",
        "fusion_flow_vy_mps",
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


def _write_imu_only_timeseries_csv(
    rows: Sequence[dict[str, str]],
    report_dir: Path,
    *,
    imu_only_metadata: dict[str, object],
) -> list[Path]:
    rows_to_write = _eligible_rows(
        rows,
        columns=IMU_ONLY_ERROR_COLUMNS,
        sample_valid_column="imu_only_pose_valid",
    )
    if not rows_to_write:
        return []

    path = report_dir / "imu_only_timeseries.csv"
    fields = [
        "t_s",
        "estimator_source",
        "approximate",
        "stamp_ros_s",
        "imu_stamp_s",
        "mission_state",
        "imu_only_err_x_m",
        "imu_only_err_y_m",
        "imu_only_err_z_m",
        "imu_only_err_xy_m",
        "imu_only_err_3d_m",
        "mocap_x_m",
        "mocap_y_m",
        "mocap_z_m",
        "imu_only_x_m",
        "imu_only_y_m",
        "imu_only_z_m",
        "imu_only_vx_mps",
        "imu_only_vy_mps",
        "imu_only_vz_mps",
        "imu_only_pose_valid",
        "imu_only_update_valid",
        "imu_only_reject_reason",
        "imu_only_health_state",
        "imu_map_accel_x_mps2",
        "imu_map_accel_y_mps2",
        "imu_map_accel_z_mps2",
        "imu_update_valid",
        "imu_reject_reason",
    ]
    times = _relative_times(rows_to_write)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for time_s, row in zip(times, rows_to_write):
            output = {field: row.get(field, "") for field in fields}
            output["t_s"] = f"{time_s:.9f}"
            output["estimator_source"] = imu_only_metadata.get("source", "")
            output["approximate"] = int(
                bool(imu_only_metadata.get("approximate"))
            )
            writer.writerow(output)
    return [path]


def _series(rows: Sequence[dict[str, str]], column: str) -> list[float]:
    return [_float_or_none(row.get(column)) or 0.0 for row in rows]


def _plot_fit(ax, times: Sequence[float], values: Sequence[float], label: str) -> None:
    slope = _linear_slope(times, values)
    if slope is None:
        return
    intercept = mean(values) - (slope * mean(times))
    ax.plot(
        times,
        [intercept + (slope * time_s) for time_s in times],
        ":",
        label=label,
    )


def _norm_or_zero(x_value: object, y_value: object, z_value: object) -> float:
    x = _float_or_none(x_value)
    y = _float_or_none(y_value)
    z = _float_or_none(z_value)
    if x is None or y is None or z is None:
        return 0.0
    return math.sqrt((x * x) + (y * y) + (z * z))


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


def _nested(mapping: object, outer: str, inner: str) -> object:
    if not isinstance(mapping, dict):
        return None
    child = mapping.get(outer)
    if not isinstance(child, dict):
        return None
    return child.get(inner)


def _mapping_value(mapping: object, key: str) -> object:
    if not isinstance(mapping, dict):
        return None
    return mapping.get(key)


def _delta(before: object, after: object) -> Optional[float]:
    before_value = _float_or_none(before)
    after_value = _float_or_none(after)
    if before_value is None or after_value is None:
        return None
    return before_value - after_value


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
