from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np

from drone_control_pkg.imu_optical_flow_fusion import (
    ImuOpticalFlowFusion,
    ImuSample,
)
from drone_control_pkg.optical_flow_dead_reckon import (
    OpticalFlowDeadReckoner,
    OpticalFlowSample,
)


ACTIVE_STATES = {
    "SYNC_TAKEOFF_PARAM",
    "SET_TAKEOFF_MODE",
    "ARMING",
    "REQUEST_TAKEOFF",
    "TAKEOFF",
    "WARMUP_OFFBOARD",
    "SET_OFFBOARD_MODE",
    "WAYPOINTS",
    "WAYPOINT_HOLD",
    "SET_LAND_MODE",
    "WAIT_TOUCHDOWN",
    "DISARMING",
    "RESTORE_TAKEOFF_PARAM",
    "RESTORE_SPEED_PROFILE",
}

IMU_ONLY_FIELDS = (
    "imu_only_x_m",
    "imu_only_y_m",
    "imu_only_z_m",
    "imu_only_vx_mps",
    "imu_only_vy_mps",
    "imu_only_vz_mps",
    "imu_only_err_x_m",
    "imu_only_err_y_m",
    "imu_only_err_z_m",
    "imu_only_err_xy_m",
    "imu_only_err_3d_m",
    "imu_only_pose_valid",
    "imu_only_health_state",
)

REPLAY_DIAGNOSTIC_FIELDS = (
    "replay_approximate",
    "flow_source_stamp_s",
    "flow_receive_stamp_s",
    "flow_sequence",
    "flow_frame_id",
    "flow_callback_dt_s",
    "flow_integration_coverage",
    "fusion_health_state",
    "fusion_nis",
    "fusion_gyro_source",
    "fusion_sensor_age_s",
)


@dataclass(frozen=True)
class ReplayOutput:
    rows: list[dict[str, str]]
    metadata: dict[str, object]


def replay_csv(
    csv_path: str | Path,
    output_path: str | Path,
    *,
    fusion_kwargs: Optional[dict[str, float]] = None,
) -> ReplayOutput:
    source = Path(csv_path).expanduser()
    rows = _read_rows(source)
    replay = replay_rows(rows, fusion_kwargs=fusion_kwargs)
    destination = Path(output_path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    _write_rows(destination, replay.rows)
    return replay


def replay_rows(
    rows: Sequence[dict[str, str]],
    *,
    fusion_kwargs: Optional[dict[str, float]] = None,
) -> ReplayOutput:
    """Replay a legacy 20 Hz comparison log through all three estimators.

    Legacy logs do not contain the full IMU quaternion or every sensor callback.
    The replay reconstructs a yaw-only quaternion and body specific force from
    the already logged map-frame acceleration. Results are useful for regression
    and relative tuning, but are explicitly marked approximate.
    """

    estimator_kwargs = dict(fusion_kwargs or {})
    frame_id = _first_text(rows, "comparison_frame_id") or "map"
    imu_only = ImuOpticalFlowFusion(frame_id=frame_id, **estimator_kwargs)
    fused = ImuOpticalFlowFusion(frame_id=frame_id, **estimator_kwargs)
    of_only = OpticalFlowDeadReckoner(
        frame_id=frame_id,
        quality_min=int(estimator_kwargs.get("quality_min", 10)),
        range_min_m=float(estimator_kwargs.get("range_min_m", 0.2)),
        range_max_m=float(estimator_kwargs.get("range_max_m", 5.0)),
        gyro_compensation_gain=float(
            estimator_kwargs.get("gyro_compensation_gain", 1.0)
        ),
        flow_scale_x=float(estimator_kwargs.get("flow_scale_x", 1.0)),
        flow_scale_y=float(estimator_kwargs.get("flow_scale_y", 1.0)),
        max_flow_dt_s=float(estimator_kwargs.get("max_flow_gap_s", 0.25)),
    )

    output_rows: list[dict[str, str]] = []
    initialized = False
    was_active = False
    frozen = False
    last_imu_stamp: Optional[float] = None
    last_flow_stamp: Optional[float] = None
    last_flow_fingerprint: Optional[tuple[object, ...]] = None
    flow_sequence = 0
    latest_flow_update = None
    latest_fusion_update = None
    latest_imu_update = None
    latest_imu_only_update = None
    flow_events = 0
    imu_events = 0

    for source_row in rows:
        row = dict(source_row)
        state = str(row.get("mission_state", "")).strip()
        active = state in ACTIVE_STATES
        if active and not was_active:
            initialized = _reset_estimators(row, imu_only, fused, of_only)
            frozen = False
            last_imu_stamp = None
            last_flow_stamp = None
            last_flow_fingerprint = None
            latest_flow_update = None
            latest_fusion_update = None
            latest_imu_update = None
            latest_imu_only_update = None
        was_active = active

        if state == "COMPLETE" and initialized and not frozen:
            imu_only.freeze()
            fused.freeze()
            frozen = True

        receive_stamp = _float_or_none(row.get("stamp_ros_s"))
        imu_stamp = _float_or_none(row.get("imu_stamp_s"))
        if (
            initialized
            and not frozen
            and imu_stamp is not None
            and imu_stamp != last_imu_stamp
        ):
            sample = _legacy_imu_sample(row, receive_stamp, imu_stamp)
            of_only.update_imu_gyro(
                stamp_s=imu_stamp,
                angular_velocity_x=sample.angular_velocity_x,
                angular_velocity_y=sample.angular_velocity_y,
                angular_velocity_z=sample.angular_velocity_z,
            )
            latest_imu_only_update = imu_only.update_imu(sample)
            latest_imu_update = fused.update_imu(sample)
            latest_fusion_update = latest_imu_update
            last_imu_stamp = imu_stamp
            imu_events += 1

        fingerprint = _flow_fingerprint(row)
        flow_present = fingerprint is not None
        is_new_flow = flow_present and fingerprint != last_flow_fingerprint
        if initialized and not frozen and is_new_flow:
            flow_stamp = (
                _float_or_none(row.get("flow_source_stamp_s"))
                or receive_stamp
            )
            if flow_stamp is not None:
                flow_sequence += 1
                sample = _legacy_flow_sample(
                    row,
                    source_stamp_s=flow_stamp,
                    receive_stamp_s=receive_stamp,
                    sequence=flow_sequence,
                )
                yaw = _float_or_none(row.get("imu_yaw_rad")) or 0.0
                latest_flow_update = of_only.update(sample, yaw_rad=yaw)
                latest_fusion_update = fused.update_flow(sample)
                flow_events += 1
                row["flow_source_stamp_s"] = _csv_float(flow_stamp)
                row["flow_receive_stamp_s"] = _csv_float(receive_stamp)
                row["flow_sequence"] = str(flow_sequence)
                row["flow_frame_id"] = str(row.get("of_frame_id", "px4flow"))
                callback_dt = (
                    flow_stamp - last_flow_stamp
                    if last_flow_stamp is not None
                    else None
                )
                integration_dt = _float_or_none(row.get("flow_dt_s"))
                row["flow_callback_dt_s"] = _csv_float(callback_dt)
                row["flow_integration_coverage"] = _csv_float(
                    integration_dt / callback_dt
                    if integration_dt is not None
                    and callback_dt is not None
                    and callback_dt > 0.0
                    else None
                )
                last_flow_stamp = flow_stamp
            last_flow_fingerprint = fingerprint

        if initialized:
            _write_pose_columns(row, "of", of_only.pose, row)
            _write_pose_columns(row, "imu_only", imu_only.pose, row)
            _write_pose_columns(row, "fused", fused.pose, row)
            row["sample_valid"] = str(
                int(bool(latest_flow_update and latest_flow_update.valid))
            )
            row["reject_reason"] = (
                latest_flow_update.reject_reason
                if latest_flow_update is not None
                else "no_flow"
            )
            row["imu_only_pose_valid"] = str(int(imu_only.pose is not None))
            row["fusion_pose_valid"] = str(int(fused.pose is not None))
            row["fusion_flow_valid"] = str(
                int(
                    bool(
                        latest_fusion_update
                        and latest_fusion_update.update_type == "flow"
                        and latest_fusion_update.valid
                    )
                )
            )
        else:
            _blank_pose_columns(row, "imu_only")
            row["imu_only_pose_valid"] = "0"

        row["replay_approximate"] = "1"
        row["imu_only_health_state"] = _update_text(
            latest_imu_only_update,
            "health_state",
        )
        row["fusion_health_state"] = _update_text(
            latest_fusion_update,
            "health_state",
        )
        row["fusion_nis"] = _csv_float(
            getattr(latest_fusion_update, "nis", None)
            if latest_fusion_update is not None
            else None
        )
        row["fusion_gyro_source"] = _update_text(
            latest_fusion_update,
            "gyro_source",
        )
        row["fusion_sensor_age_s"] = _csv_float(
            getattr(latest_fusion_update, "sensor_age_s", None)
            if latest_fusion_update is not None
            else None
        )
        output_rows.append(row)

    metadata = {
        "approximate_legacy_replay": True,
        "rows": len(output_rows),
        "imu_events": imu_events,
        "flow_events": flow_events,
        "fusion_kwargs": estimator_kwargs,
        "limitations": [
            "20 Hz logger snapshots omit intermediate IMU and flow callbacks",
            "full IMU quaternion is unavailable; yaw-only attitude is reconstructed",
            "logged processed map acceleration is converted back to body specific force",
            "legacy flow source timestamps are approximated by logger timestamps",
        ],
    }
    return ReplayOutput(rows=output_rows, metadata=metadata)


def tune_legacy_replay(
    rows: Sequence[dict[str, str]],
) -> tuple[dict[str, float], dict[str, object]]:
    """Select noise/filter parameters by blocked five-fold waypoint XY RMSE."""

    # The legacy stream is vibration dominated and its delivered flow windows
    # cover only a small fraction of wall time. Search damping, filtering, and
    # base flow trust while retaining quality/gap/gyro-dependent covariance.
    candidates = [
        {
            "accel_lpf_tau_s": tau,
            "accel_noise_mps2": 1.5,
            "flow_velocity_noise_mps": flow_noise,
            "velocity_decay_per_s": velocity_decay,
            "flow_gap_noise_scale": 0.05,
            "flow_quality_noise_scale": 1.0,
            "gyro_fallback_noise_scale": 1.5,
            "range_innovation_gate_nis": 25.0,
        }
        for tau, flow_noise, velocity_decay in product(
            (0.10, 0.50, 1.00),
            (0.03, 0.10, 0.30, 1.00),
            (0.50, 2.00, 5.00),
        )
    ]
    candidates.append({})
    results: list[dict[str, object]] = []
    for candidate in candidates:
        replay = replay_rows(rows, fusion_kwargs=candidate)
        fold_rmse = _blocked_fold_rmse(replay.rows, folds=5)
        finite = [value for value in fold_rmse if math.isfinite(value)]
        score = sum(finite) / len(finite) if finite else float("inf")
        results.append(
            {
                "parameters": candidate,
                "fold_xy_rmse_m": [
                    value if math.isfinite(value) else None
                    for value in fold_rmse
                ],
                "mean_held_out_xy_rmse_m": (
                    score if math.isfinite(score) else None
                ),
            }
        )

    ranked = sorted(
        results,
        key=lambda result: (
            result["mean_held_out_xy_rmse_m"] is None,
            result["mean_held_out_xy_rmse_m"] or float("inf"),
        ),
    )
    best = dict(ranked[0]["parameters"]) if ranked else {}
    report = {
        "objective": "mean blocked five-fold WAYPOINTS fused XY RMSE",
        "approximate_legacy_replay": True,
        "candidate_count": len(candidates),
        "best_parameters": best,
        "best_mean_held_out_xy_rmse_m": (
            ranked[0]["mean_held_out_xy_rmse_m"] if ranked else None
        ),
        "candidates": ranked,
    }
    return best, report


def _blocked_fold_rmse(
    rows: Sequence[dict[str, str]],
    *,
    folds: int,
) -> list[float]:
    values = [
        value
        for row in rows
        if str(row.get("mission_state", "")) == "WAYPOINTS"
        for value in [_float_or_none(row.get("fused_err_xy_m"))]
        if value is not None
    ]
    if not values:
        return [float("inf")] * folds
    return [
        math.sqrt(float(np.mean(np.square(block))))
        if len(block) > 0
        else float("inf")
        for block in np.array_split(np.asarray(values, dtype=float), folds)
    ]


def _reset_estimators(
    row: dict[str, str],
    imu_only: ImuOpticalFlowFusion,
    fused: ImuOpticalFlowFusion,
    of_only: OpticalFlowDeadReckoner,
) -> bool:
    x_m = _first_number(row, "origin_reset_mocap_x_m", "mocap_x_m")
    y_m = _first_number(row, "origin_reset_mocap_y_m", "mocap_y_m")
    z_m = _first_number(row, "origin_reset_mocap_z_m", "mocap_z_m")
    if x_m is None or y_m is None or z_m is None:
        return False
    yaw = _first_number(row, "origin_reset_mocap_yaw_rad", "mocap_yaw_rad")
    yaw = yaw if yaw is not None else 0.0
    range_m = _first_number(row, "flow_distance_m", "range_m")
    frame_id = str(row.get("comparison_frame_id", "")).strip() or "map"
    for estimator in (imu_only, fused):
        estimator.reset(
            x_m=x_m,
            y_m=y_m,
            z_m=z_m,
            yaw_rad=yaw,
            range_m=range_m,
            frame_id=frame_id,
        )
        estimator.unfreeze()
    of_only.reset(
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        yaw_rad=yaw,
        range_m=range_m,
        frame_id=frame_id,
    )
    return True


def _legacy_imu_sample(
    row: dict[str, str],
    receive_stamp_s: Optional[float],
    stamp_s: float,
) -> ImuSample:
    yaw = _float_or_none(row.get("imu_yaw_rad")) or 0.0
    accel_map = np.array(
        [
            _float_or_none(row.get("imu_map_accel_x_mps2")) or 0.0,
            _float_or_none(row.get("imu_map_accel_y_mps2")) or 0.0,
            _float_or_none(row.get("imu_map_accel_z_mps2")) or 0.0,
        ],
        dtype=float,
    )
    specific_force_map = accel_map + np.array([0.0, 0.0, 9.80665])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    accel_body_x = (
        cos_yaw * specific_force_map[0]
        + sin_yaw * specific_force_map[1]
    )
    accel_body_y = (
        -sin_yaw * specific_force_map[0]
        + cos_yaw * specific_force_map[1]
    )
    half_yaw = 0.5 * yaw
    return ImuSample(
        stamp_s=stamp_s,
        orientation_x=0.0,
        orientation_y=0.0,
        orientation_z=math.sin(half_yaw),
        orientation_w=math.cos(half_yaw),
        angular_velocity_x=_float_or_zero(row.get("imu_ang_vel_x_radps")),
        angular_velocity_y=_float_or_zero(row.get("imu_ang_vel_y_radps")),
        angular_velocity_z=_float_or_zero(row.get("imu_ang_vel_z_radps")),
        linear_acceleration_x=float(accel_body_x),
        linear_acceleration_y=float(accel_body_y),
        linear_acceleration_z=float(specific_force_map[2]),
        source_stamp_s=stamp_s,
        receive_stamp_s=receive_stamp_s,
        frame_id=str(row.get("imu_frame_id", "")),
    )


def _legacy_flow_sample(
    row: dict[str, str],
    *,
    source_stamp_s: float,
    receive_stamp_s: Optional[float],
    sequence: int,
) -> OpticalFlowSample:
    return OpticalFlowSample(
        integrated_x=_float_or_zero(row.get("flow_integrated_x")),
        integrated_y=_float_or_zero(row.get("flow_integrated_y")),
        integrated_xgyro=_float_or_nan(row.get("flow_integrated_xgyro")),
        integrated_ygyro=_float_or_nan(row.get("flow_integrated_ygyro")),
        integrated_zgyro=_float_or_nan(row.get("flow_integrated_zgyro")),
        quality=int(_float_or_zero(row.get("flow_quality"))),
        distance_m=(
            _first_number(row, "flow_distance_m", "range_m")
            or float("nan")
        ),
        integration_time_s=(
            _float_or_none(row.get("flow_dt_s")) or float("nan")
        ),
        source_stamp_s=source_stamp_s,
        receive_stamp_s=receive_stamp_s,
        frame_id=str(row.get("flow_frame_id", "px4flow")),
        sequence=sequence,
    )


def _write_pose_columns(
    row: dict[str, str],
    prefix: str,
    pose: object,
    truth_row: dict[str, str],
) -> None:
    if pose is None:
        _blank_pose_columns(row, prefix)
        return
    x_m = float(getattr(pose, "x_m"))
    y_m = float(getattr(pose, "y_m"))
    z_m = float(getattr(pose, "z_m"))
    row[f"{prefix}_x_m"] = _csv_float(x_m)
    row[f"{prefix}_y_m"] = _csv_float(y_m)
    row[f"{prefix}_z_m"] = _csv_float(z_m)
    if hasattr(pose, "vx_mps"):
        row[f"{prefix}_vx_mps"] = _csv_float(getattr(pose, "vx_mps"))
        row[f"{prefix}_vy_mps"] = _csv_float(getattr(pose, "vy_mps"))
        row[f"{prefix}_vz_mps"] = _csv_float(getattr(pose, "vz_mps"))
    truth = np.array(
        [
            _float_or_none(truth_row.get("mocap_x_m")),
            _float_or_none(truth_row.get("mocap_y_m")),
            _float_or_none(truth_row.get("mocap_z_m")),
        ],
        dtype=object,
    )
    if any(value is None for value in truth):
        return
    error = np.array([x_m, y_m, z_m], dtype=float) - truth.astype(float)
    error_prefix = "err" if prefix == "of" else f"{prefix}_err"
    row[f"{error_prefix}_x_m"] = _csv_float(error[0])
    row[f"{error_prefix}_y_m"] = _csv_float(error[1])
    row[f"{error_prefix}_z_m"] = _csv_float(error[2])
    row[f"{error_prefix}_xy_m"] = _csv_float(float(np.linalg.norm(error[:2])))
    row[f"{error_prefix}_3d_m"] = _csv_float(float(np.linalg.norm(error)))


def _blank_pose_columns(row: dict[str, str], prefix: str) -> None:
    for suffix in (
        "x_m",
        "y_m",
        "z_m",
        "vx_mps",
        "vy_mps",
        "vz_mps",
        "err_x_m",
        "err_y_m",
        "err_z_m",
        "err_xy_m",
        "err_3d_m",
    ):
        row[f"{prefix}_{suffix}"] = ""


def _flow_fingerprint(row: dict[str, str]) -> Optional[tuple[object, ...]]:
    values = tuple(
        _normalized_fingerprint_value(row.get(column))
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
    if all(value is None for value in values):
        return None
    return values


def _normalized_fingerprint_value(value: object) -> object:
    parsed = _float_or_none(value)
    return parsed if parsed is not None else None


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _write_rows(path: Path, rows: Sequence[dict[str, str]]) -> None:
    fieldnames: list[str] = []
    seen = set()
    for row in rows:
        for field in row:
            if field not in seen:
                fieldnames.append(field)
                seen.add(field)
    for field in (*IMU_ONLY_FIELDS, *REPLAY_DIAGNOSTIC_FIELDS):
        if field not in seen:
            fieldnames.append(field)
            seen.add(field)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _first_text(rows: Sequence[dict[str, str]], column: str) -> str:
    for row in rows:
        text = str(row.get(column, "")).strip()
        if text:
            return text
    return ""


def _first_number(row: dict[str, str], *columns: str) -> Optional[float]:
    for column in columns:
        parsed = _float_or_none(row.get(column))
        if parsed is not None:
            return parsed
    return None


def _float_or_none(value: object) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _float_or_zero(value: object) -> float:
    return _float_or_none(value) or 0.0


def _float_or_nan(value: object) -> float:
    parsed = _float_or_none(value)
    return parsed if parsed is not None else float("nan")


def _csv_float(value: object) -> str:
    parsed = _float_or_none(value)
    return "" if parsed is None else f"{parsed:.9f}"


def _update_text(update: object, attribute: str) -> str:
    if update is None:
        return ""
    return str(getattr(update, attribute, "") or "")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay legacy mocap/flow logs through IMU, OF, and EKF estimators."
    )
    parser.add_argument("csv_path", help="Input of_mocap_compare.csv")
    parser.add_argument(
        "--output",
        help="Output replay CSV. Default: <run>/replay/ekf_compare.csv",
    )
    parser.add_argument(
        "--no-tune",
        action="store_true",
        help="Use estimator defaults instead of blocked replay tuning.",
    )
    parser.add_argument(
        "--plots",
        action="store_true",
        help="Generate report plots for the replayed CSV.",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _build_parser().parse_args(list(argv) if argv is not None else None)
    source = Path(args.csv_path).expanduser()
    output = (
        Path(args.output).expanduser()
        if args.output
        else source.parent / "replay" / "ekf_compare.csv"
    )
    source_rows = _read_rows(source)
    best_parameters: dict[str, float] = {}
    tuning_report: dict[str, object] = {
        "objective": "tuning disabled",
        "best_parameters": {},
    }
    if not args.no_tune:
        best_parameters, tuning_report = tune_legacy_replay(source_rows)

    replay = replay_csv(
        source,
        output,
        fusion_kwargs=best_parameters,
    )
    tuning_path = output.parent / "tuning.json"
    tuning_path.write_text(
        json.dumps(tuning_report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    metadata_path = output.parent / "replay_metadata.json"
    metadata_path.write_text(
        json.dumps(replay.metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    from drone_control_pkg.of_compare_stats import write_report

    summary = write_report(
        output,
        output_dir=output.parent / "report",
        plots=bool(args.plots),
    )
    print(
        json.dumps(
            {
                "output_csv": str(output),
                "report_dir": str(output.parent / "report"),
                "tuning": str(tuning_path),
                "metadata": str(metadata_path),
                "comparisons": summary.get("comparisons", {}),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
