"""Estimate the ZED-to-airframe transform from guided, unpowered motion."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from drone_control_pkg.zed_vio import Mount


@dataclass(frozen=True)
class PoseSample:
    position: np.ndarray
    quaternion_xyzw: np.ndarray


@dataclass(frozen=True)
class PivotFit:
    center_to_camera_camera_m: np.ndarray
    rms_m: float
    p95_m: float
    condition: float
    min_rotation_spread: float
    sample_count: int
    axes: dict[str, dict[str, float | int]]


class PivotCalibrationError(ValueError):
    """A failed pivot fit, optionally attributable to one recapturable axis."""

    def __init__(self, message: str, axis: str | None = None):
        super().__init__(message)
        self.axis = axis


PIVOT_AXES = {"yaw": 2, "pitch": 1, "roll": 0}
MIN_AXIS_SPAN_DEG = {"yaw": 12.0, "pitch": 10.0, "roll": 10.0}


def stable_pose(samples: list[PoseSample]) -> PoseSample:
    """Summarize a stationary hold and reject motion/tracking scatter."""
    if len(samples) < 12:
        raise ValueError("too few healthy ZED frames in stationary hold")
    positions = np.array([sample.position for sample in samples])
    quaternions = np.array([sample.quaternion_xyzw for sample in samples])
    if not np.isfinite(positions).all() or not np.isfinite(quaternions).all():
        raise ValueError("non-finite ZED pose in stationary hold")
    center = np.median(positions, axis=0)
    rotation = Rotation.from_quat(quaternions).mean()
    position_p95 = np.percentile(np.linalg.norm(positions - center, axis=1), 95)
    angle_p95 = np.percentile(
        (rotation.inv() * Rotation.from_quat(quaternions)).magnitude(), 95)
    if position_p95 > 0.03 or angle_p95 > math.radians(5):
        raise ValueError("stationary hold moved or ZED tracking scattered; repeat it")
    return PoseSample(center, rotation.as_quat())


def camera_orientation_in_body(
    start: PoseSample, forward: PoseSample, left: PoseSample,
) -> Rotation:
    """Infer body FLU axes from an L-shaped forward then left translation."""
    base_rotation = Rotation.from_quat(start.quaternion_xyzw)
    for label, pose in (("forward", forward), ("left", left)):
        angle = (base_rotation.inv() * Rotation.from_quat(pose.quaternion_xyzw)).magnitude()
        if angle > math.radians(10):
            raise ValueError(
                f"{label}: drone orientation changed {math.degrees(angle):.1f} deg "
                "(limit 10 deg); repeat")
    x_delta = forward.position - start.position
    y_delta = left.position - forward.position
    x_length, y_length = np.linalg.norm(x_delta), np.linalg.norm(y_delta)
    if min(x_length, y_length) < 0.15:
        raise ValueError(
            f"measured forward {x_length:.2f} m and left {y_length:.2f} m; "
            "need at least 0.15 m each")
    x_axis, y_raw = x_delta / x_length, y_delta / y_length
    off_square_deg = math.degrees(math.asin(min(1.0, abs(np.dot(x_axis, y_raw)))))
    if off_square_deg > 20:
        raise ValueError(
            f"forward/left slides are {off_square_deg:.1f} deg away from a right angle "
            "(limit 20 deg)")
    z_axis = np.cross(x_axis, y_raw)
    z_axis /= np.linalg.norm(z_axis)
    if z_axis[2] < math.cos(math.radians(25)):
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, z_axis[2]))))
        raise ValueError(f"slide plane tilts {tilt:.1f} deg from level (limit 25 deg)")
    # Share small hand-guidance errors between X and Y instead of assigning
    # the entire nonorthogonality to the left slide.
    u, _, vh = np.linalg.svd(np.column_stack((x_axis, y_raw, z_axis)))
    body_in_world = Rotation.from_matrix(u @ np.diag((1, 1, np.linalg.det(u @ vh))) @ vh)
    return body_in_world.inv() * base_rotation


def validate_axis_motion(
    axis: str, samples: list[PoseSample], camera_in_body: Rotation,
) -> float:
    """Require a useful, mostly single-axis rotation about the airframe center."""
    if axis not in PIVOT_AXES:
        raise ValueError(f"unknown pivot axis: {axis}")
    if len(samples) < 60:
        raise PivotCalibrationError(f"{axis}: fewer than 60 healthy frames", axis)
    positions = np.array([sample.position for sample in samples])
    quaternions = np.array([sample.quaternion_xyzw for sample in samples])
    if not np.isfinite(positions).all() or not np.isfinite(quaternions).all():
        raise PivotCalibrationError(f"{axis}: non-finite ZED pose", axis)
    body_in_world = Rotation.from_quat(quaternions) * camera_in_body.inv()
    body_change = (body_in_world[0].inv() * body_in_world).as_rotvec()
    spreads = np.ptp(body_change, axis=0)
    target = PIVOT_AXES[axis]
    span_deg = math.degrees(spreads[target])
    if span_deg < MIN_AXIS_SPAN_DEG[axis]:
        raise PivotCalibrationError(
            f"{axis}: only {span_deg:.1f} deg of motion; "
            f"need at least {MIN_AXIS_SPAN_DEG[axis]:g} deg", axis)
    return span_deg


def fit_axis_pivots(
    axis_sweeps: dict[str, list[PoseSample]], camera_in_body: Rotation,
) -> PivotFit:
    """Fit one camera lever arm with a separate world pivot center per axis.

    Each sweep is demeaned independently. Yaw alone cannot observe the
    vertical lever arm; pitch and roll make the combined system full rank.
    """
    if set(axis_sweeps) != set(PIVOT_AXES):
        raise PivotCalibrationError("capture separate yaw, pitch, and roll sweeps")
    blocks = []
    diagnostics = {}
    for axis in PIVOT_AXES:
        samples = axis_sweeps[axis]
        span_deg = validate_axis_motion(axis, samples, camera_in_body)
        positions = np.array([sample.position for sample in samples])
        rotations = Rotation.from_quat(
            [sample.quaternion_xyzw for sample in samples]).as_matrix()
        blocks.append((axis, positions, rotations))
        diagnostics[axis] = {"frames": len(samples), "span_deg": span_deg}
    count = sum(len(positions) for _, positions, _ in blocks)
    model = np.zeros((3 * count, 12))  # lever arm + one fixed world center per axis
    observed = np.concatenate([positions.reshape(-1) for _, positions, _ in blocks])
    cursor = 0
    for axis_index, (_, positions, rotations) in enumerate(blocks):
        for rotation in rotations:
            model[cursor:cursor + 3, :3] = rotation
            model[cursor:cursor + 3, 3 + 3*axis_index:6 + 3*axis_index] = np.eye(3)
            cursor += 3
    initial = np.linalg.lstsq(model, observed, rcond=None)[0]
    solution = least_squares(
        lambda parameters: model @ parameters - observed, initial,
        loss="soft_l1", f_scale=0.02, max_nfev=50)
    if not solution.success:
        raise PivotCalibrationError("fixed-center fit did not converge; repeat pivot sweeps")
    center_to_camera = solution.x[:3]
    if np.linalg.norm(center_to_camera) > 1.0:
        raise PivotCalibrationError("fitted camera offset exceeds 1 m; check pivot center")
    all_residuals = []
    inlier_design = []
    inlier_count = 0
    for axis_index, (axis, positions, rotations) in enumerate(blocks):
        fixed_center = solution.x[3 + 3*axis_index:6 + 3*axis_index]
        residuals = np.linalg.norm(
            positions - fixed_center - np.einsum("nij,j->ni", rotations, center_to_camera),
            axis=1)
        inliers = residuals <= 0.05
        fraction = float(np.mean(inliers))
        accepted_residuals = residuals[inliers]
        rms = (float(np.sqrt(np.mean(accepted_residuals ** 2)))
               if len(accepted_residuals) else math.inf)
        p95 = (float(np.percentile(accepted_residuals, 95))
               if len(accepted_residuals) else math.inf)
        diagnostics[axis].update({
            "rms_m": rms, "p95_m": p95, "inlier_fraction": fraction,
            "raw_rms_m": float(np.sqrt(np.mean(residuals ** 2))),
        })
        if len(accepted_residuals):
            selected = rotations[inliers]
            inlier_design.append((selected - selected.mean(axis=0)).reshape(-1, 3))
            inlier_count += len(selected)
            all_residuals.extend(accepted_residuals)
    worst = max(PIVOT_AXES, key=lambda axis: max(
        diagnostics[axis]["rms_m"] / 0.025,
        diagnostics[axis]["p95_m"] / 0.05,
        0.85 / max(diagnostics[axis]["inlier_fraction"], 1e-12)))
    worst_rms = diagnostics[worst]["rms_m"]
    worst_p95 = diagnostics[worst]["p95_m"]
    worst_fraction = diagnostics[worst]["inlier_fraction"]
    if worst_rms > 0.025 or worst_p95 > 0.05 or worst_fraction < 0.85:
        raise PivotCalibrationError(
            f"{worst}: pivot center moved or tracking drifted "
            f"(inliers {worst_fraction:.0%}, RMS {worst_rms:.3f} m, "
            f"95th percentile {worst_p95:.3f} m)", worst)
    design = np.concatenate(inlier_design)
    singular_values = np.linalg.svd(design, compute_uv=False)
    condition = float(singular_values[0] / max(singular_values[-1], 1e-12))
    min_spread = float(singular_values[-1] / math.sqrt(inlier_count))
    if condition > 40 or min_spread < 0.04:
        raise PivotCalibrationError(
            f"combined healthy frames still lack 3D rotation coverage "
            f"(condition {condition:.1f}, spread {min_spread:.3f}); "
            "make wider pitch and roll sweeps")
    all_residuals = np.asarray(all_residuals)
    return PivotFit(
        center_to_camera,
        float(np.sqrt(np.mean(all_residuals ** 2))),
        float(np.percentile(all_residuals, 95)),
        condition, min_spread, count, diagnostics)


def mount_from_motion(
    start: PoseSample, forward: PoseSample, left: PoseSample,
    axis_sweeps: dict[str, list[PoseSample]] | None = None,
    measured_translation_m: tuple[float, float, float] | None = None,
) -> tuple[Mount, PivotFit | None]:
    """Produce a mount from separate axis pivots or an explicit ruler offset."""
    camera_in_body = camera_orientation_in_body(start, forward, left)
    if measured_translation_m is not None:
        if axis_sweeps:
            raise ValueError("choose either pivot fitting or measured translation")
        translation = np.asarray(measured_translation_m, dtype=float)
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError("measured translation must be three finite body FLU values")
        if np.linalg.norm(translation) > 1.0:
            raise ValueError("measured camera offset exceeds 1 m; check units and origin")
        fit = None
    else:
        fit = fit_axis_pivots(axis_sweeps or {}, camera_in_body)
        translation = camera_in_body.apply(fit.center_to_camera_camera_m)
    rpy = camera_in_body.as_euler("xyz")
    return Mount(tuple(map(float, translation)), tuple(map(float, rpy))), fit
