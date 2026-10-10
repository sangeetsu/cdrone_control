"""Interactive ZED 2i mount calibration with the motors disconnected."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

from drone_control_pkg.zed_mount_calibration import (
    PIVOT_AXES, PivotCalibrationError, PoseSample, camera_orientation_in_body,
    fit_axis_pivots, mount_from_motion, stable_pose, validate_axis_motion,
)
from drone_control_pkg.zed_vio import load_mount


class ZedPoseRecorder:
    """Keep SDK tracking alive while the operator reads each motion prompt."""

    def __init__(self, sl):
        self.sl = sl
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 30
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.camera = sl.Camera()
        result = self.camera.open(init)
        if result != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED camera open failed: {result}; stop other ZED users")
        tracking = sl.PositionalTrackingParameters()
        tracking.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        tracking.enable_imu_fusion = True
        result = self.camera.enable_positional_tracking(tracking)
        if result != sl.ERROR_CODE.SUCCESS:
            self.camera.close()
            raise RuntimeError(f"ZED positional tracking failed: {result}")
        self.sdk_version = str(sl.Camera.get_sdk_version())
        self.serial_number = int(self.camera.get_camera_information().serial_number)
        self.pose = sl.Pose()
        self.samples = deque(maxlen=3000)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.last_stamp_ns = 0
        self.last_problem = "waiting for healthy frames"
        self.thread = threading.Thread(target=self._capture, name="zed-calibration-capture")
        self.thread.start()

    def _capture(self):
        while not self.stop.is_set():
            if self.camera.grab() != self.sl.ERROR_CODE.SUCCESS:
                self.last_problem = "camera grab failed"
                time.sleep(0.03)
                continue
            state = self.camera.get_position(self.pose, self.sl.REFERENCE_FRAME.WORLD)
            if state != self.sl.POSITIONAL_TRACKING_STATE.OK:
                self.last_problem = f"tracking state {state}"
                continue
            stamp_ns = int(self.pose.timestamp.get_nanoseconds())
            age_ns = time.time_ns() - stamp_ns
            if (stamp_ns <= self.last_stamp_ns or not -100_000_000 <= age_ns <= 300_000_000):
                self.last_problem = "stale, duplicate, or future ZED frame"
                continue
            position = np.asarray(self.pose.get_translation().get(), dtype=float).copy()
            quaternion = np.asarray(self.pose.get_orientation().get(), dtype=float).copy()
            if (position.shape != (3,) or quaternion.shape != (4,)
                    or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
                    or not 0.9 <= np.linalg.norm(quaternion) <= 1.1):
                self.last_problem = "invalid ZED pose"
                continue
            self.last_stamp_ns = stamp_ns
            self.last_problem = "OK"
            with self.lock:
                self.samples.append((time.monotonic_ns(), PoseSample(position, quaternion)))

    def collect(self, duration_s):
        start_ns = time.monotonic_ns()
        time.sleep(duration_s)
        end_ns = time.monotonic_ns()
        with self.lock:
            recorded = [(arrival, pose) for arrival, pose in self.samples
                        if start_ns <= arrival <= end_ns]
        if len(recorded) < 8 * duration_s:
            raise ValueError(
                f"only {len(recorded)} healthy frames in {duration_s:g} s; "
                f"need at least 8 Hz (last status: {self.last_problem})")
        if any(b[0] - a[0] > 1_000_000_000 for a, b in zip(recorded, recorded[1:])):
            raise ValueError("ZED tracking had a gap over 1 s during capture")
        return [pose for _, pose in recorded]

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)
        self.camera.close()


def _hold(recorder, instruction):
    while True:
        input(f"\n{instruction}\nPress Enter once it is still: ")
        try:
            poses = recorder.collect(1.5)
            estimate = stable_pose(poses)
        except ValueError as exc:
            print(f"Hold rejected: {exc}. Tracking: {recorder.last_problem}", flush=True)
            continue
        print(f"Recorded {len(poses)} healthy frames.", flush=True)
        return estimate, len(poses)


def _pivot(recorder, duration_s, axis):
    directions = {
        "yaw": "turn nose left/right around body Z; keep the drone level",
        "pitch": "tip nose up/down around body Y (left); keep center fixed",
        "roll": "tip left/right around body X (forward); keep center fixed",
    }
    input(
        f"\n{axis.upper()} sweep: support the AIRFRAME CENTER at a fixed point. "
        f"After the countdown, {directions[axis]}. Move both directions and "
        "keep the ZED looking at a textured scene. Press Enter when ready: ")
    print("Starting in 2 seconds...", flush=True)
    time.sleep(2)
    print(f"CAPTURING for {duration_s:g} seconds: rotate now.", flush=True)
    poses = recorder.collect(duration_s)
    print(f"Captured {len(poses)} healthy frames. Stop moving.", flush=True)
    return poses


def _render_mount(mount):
    translation = ", ".join(f"{value:.8f}" for value in mount.body_to_camera_translation_m)
    rpy = ", ".join(f"{value:.8f}" for value in mount.body_to_camera_rpy_rad)
    return (
        "# Generated from guided ZED motion; verify X/Y/Z/yaw against physical moves.\n"
        "# Body FLU: X forward, Y left, Z up. RPY rotates camera axes into body axes.\n"
        "calibrated: true\n"
        f"body_to_camera_translation_m: [{translation}]\n"
        f"body_to_camera_rpy_rad: [{rpy}]\n"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Measure the mounted ZED 2i pose relative to the drone center.")
    parser.add_argument("--output", type=Path, required=True,
                        help="New YAML mount file; existing files are never overwritten")
    parser.add_argument("--pivot-seconds", type=float, default=10.0,
                        help="Seconds per yaw/pitch/roll sweep (default: 10)")
    parser.add_argument("--translation-body-m", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="Use ruler-measured body FLU offset instead of pivot sweeps")
    args = parser.parse_args()
    output = args.output.expanduser()
    report = output.with_suffix(".report.json")
    if output.exists() or report.exists():
        parser.error(f"output or report already exists: {output} or {report}")
    if not 10 <= args.pivot_seconds <= 60 or not math.isfinite(args.pivot_seconds):
        parser.error("--pivot-seconds must be between 10 and 60")
    try:
        import pyzed.sl as sl
    except ImportError as exc:
        raise SystemExit(f"ZED SDK Python binding unavailable: {exc}") from exc

    print("Disconnect motors/ESCs. Stop the hover launch and every other ZED process.")
    print("Keep the camera mounted exactly as it will be during flight.")
    recorder = ZedPoseRecorder(sl)
    try:
        while True:
            start, start_count = _hold(
                recorder, "Place the drone level at its starting mark, nose known.")
            forward, forward_count = _hold(
                recorder,
                "Slide it 20-30 cm straight FORWARD along its nose; do not rotate it.")
            left, left_count = _hold(
                recorder,
                "From that spot, slide it 20-30 cm straight LEFT; keep the same heading.")
            try:
                camera_orientation_in_body(start, forward, left)
            except ValueError as exc:
                print(f"Slides rejected: {exc}. Try the L-shaped move again.", flush=True)
                continue
            forward_m = np.linalg.norm(forward.position - start.position)
            left_m = np.linalg.norm(left.position - forward.position)
            print(f"Slides passed: forward {forward_m:.2f} m, left {left_m:.2f} m.",
                  flush=True)
            break
        axis_sweeps = None
        if args.translation_body_m is None:
            axis_sweeps = {}
            camera_in_body = camera_orientation_in_body(start, forward, left)
            while True:
                for axis in PIVOT_AXES:
                    if axis in axis_sweeps:
                        continue
                    while True:
                        try:
                            poses = _pivot(recorder, args.pivot_seconds, axis)
                            span_deg = validate_axis_motion(axis, poses, camera_in_body)
                        except ValueError as exc:
                            print(f"{axis.upper()} rejected: {exc}. Repeat this axis.", flush=True)
                            continue
                        print(f"{axis.upper()} motion passed: {span_deg:.1f} deg span.",
                              flush=True)
                        axis_sweeps[axis] = poses
                        break
                try:
                    fit = fit_axis_pivots(axis_sweeps, camera_in_body)
                except PivotCalibrationError as exc:
                    if exc.axis is None:
                        axis_sweeps.clear()
                        print(f"Combined pivot fit rejected: {exc}. Repeat all axes.",
                              flush=True)
                    else:
                        del axis_sweeps[exc.axis]
                        print(f"{exc.axis.upper()} fit rejected: {exc}. Repeat that axis.",
                              flush=True)
                    continue
                print(f"Three-axis fit passed: RMS {fit.rms_m:.3f} m, "
                      f"95th percentile {fit.p95_m:.3f} m.", flush=True)
                for axis, metrics in fit.axes.items():
                    print(f"  {axis}: {metrics['span_deg']:.1f} deg; "
                          f"{metrics['inlier_fraction']:.0%} healthy fit; "
                          f"RMS {metrics['rms_m']:.3f} m", flush=True)
                break
        mount, fit = mount_from_motion(
            start, forward, left,
            axis_sweeps=axis_sweeps, measured_translation_m=args.translation_body_m)
    finally:
        recorder.close()

    if output.exists() or report.exists():
        raise SystemExit(f"output/report already exists: {output} or {report}")
    output.parent.mkdir(parents=True, exist_ok=True)
    report_data = {
        "mount_file": str(output),
        "zed_sdk_version": recorder.sdk_version,
        "camera_serial_number": recorder.serial_number,
        "method": "separate_yaw_pitch_roll_pivots" if fit else "ruler_translation",
        "body_to_camera_translation_m": mount.body_to_camera_translation_m,
        "body_to_camera_rpy_rad": mount.body_to_camera_rpy_rad,
        "body_to_camera_rpy_deg": tuple(map(math.degrees, mount.body_to_camera_rpy_rad)),
        "stationary_frame_counts": [start_count, forward_count, left_count],
        "pivot_fit": (
            {"frames": fit.sample_count, "rms_m": fit.rms_m,
             "p95_m": fit.p95_m, "condition": fit.condition,
             "min_rotation_spread": fit.min_rotation_spread,
             "center_to_camera_camera_m": fit.center_to_camera_camera_m.tolist(),
             "axes": fit.axes}
            if fit else None),
        "pivot_samples": {
            axis: [{"position_m": pose.position.tolist(),
                    "quaternion_xyzw": pose.quaternion_xyzw.tolist()} for pose in sweep]
            for axis, sweep in (axis_sweeps or {}).items()
        },
        "validation": "Bench-check manual X/Y/Z/yaw and PX4 fusion before flight",
    }
    # Create the report first. The YAML is the final signal that all gates passed.
    with report.open("x", encoding="utf-8") as stream:
        json.dump(report_data, stream, indent=2)
        stream.write("\n")
    with output.open("x", encoding="utf-8") as stream:
        stream.write(_render_mount(mount))
    # Confirm the generated file remains accepted by the hover path's parser.
    if load_mount(output) is None:
        raise RuntimeError("generated mount was rejected by the hover parser")
    print(f"\nWrote mount: {output}\nWrote quality report: {report}")
    print("Use this YAML as zed_mount_config only after the bench motion/fusion check.")


if __name__ == "__main__":
    main()
