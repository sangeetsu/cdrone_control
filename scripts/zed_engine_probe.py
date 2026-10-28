#!/usr/bin/env python3
"""Isolated ZED 2i / OBB model probe. No ROS, MAVROS, or flight commands.

Run camera, engine, and combined modes as separate processes. Each mode writes
synced stage markers so the last completed step survives an abrupt reboot.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

import numpy as np


OPTICAL_TO_FLU = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]], dtype=np.float64)


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Frame:
    color_bgr: np.ndarray
    depth_m: np.ndarray
    timestamp_ns: int


def sample_obb_depth_m(depth_m: np.ndarray, polygon: np.ndarray,
                       max_range_m: float) -> float | None:
    """Median depth in the inner 70% of an OBB, ignoring invalid returns."""
    if depth_m.ndim != 2 or np.asarray(polygon).shape != (4, 2):
        return None
    polygon = np.asarray(polygon, dtype=np.float64)
    if not np.isfinite(polygon).all():
        return None
    polygon = polygon.mean(axis=0) + 0.7 * (polygon - polygon.mean(axis=0))
    height, width = depth_m.shape
    left = max(0, int(math.floor(float(polygon[:, 0].min()))))
    right = min(width, int(math.ceil(float(polygon[:, 0].max()))) + 1)
    top = max(0, int(math.floor(float(polygon[:, 1].min()))))
    bottom = min(height, int(math.ceil(float(polygon[:, 1].max()))) + 1)
    if right <= left or bottom <= top:
        return None
    yy, xx = np.mgrid[top:bottom, left:right]
    inside_positive = np.ones(xx.shape, dtype=bool)
    inside_negative = np.ones(xx.shape, dtype=bool)
    for idx in range(4):
        a, b = polygon[idx], polygon[(idx + 1) % 4]
        cross = (b[0] - a[0]) * (yy - a[1]) - (b[1] - a[1]) * (xx - a[0])
        inside_positive &= cross >= 0
        inside_negative &= cross <= 0
    values = depth_m[top:bottom, left:right][inside_positive | inside_negative]
    values = values[np.isfinite(values) & (values > 0) & (values <= max_range_m)]
    return float(np.median(values)) if values.size >= 3 else None


def pixel_to_optical_m(u: float, v: float, depth_m: float,
                       intrinsics: Intrinsics) -> np.ndarray:
    if intrinsics.fx <= 0 or intrinsics.fy <= 0 or depth_m <= 0:
        raise ValueError("invalid intrinsics or depth")
    return np.array([(u - intrinsics.cx) * depth_m / intrinsics.fx,
                     (v - intrinsics.cy) * depth_m / intrinsics.fy,
                     depth_m], dtype=np.float64)


def load_mount_transform(path: Path, rotate_180: bool) -> tuple[np.ndarray, np.ndarray]:
    """Use the same optical-to-body transform as the Milestone 4 tracker."""
    import yaml
    from scipy.spatial.transform import Rotation

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if data.get("calibrated") is not True:
        raise ValueError(f"mount is not marked calibrated: {path}")
    offset = np.asarray(data["body_to_camera_translation_m"], dtype=np.float64)
    rpy = np.asarray(data["body_to_camera_rpy_rad"], dtype=np.float64)
    if offset.shape != (3,) or rpy.shape != (3,) or not np.isfinite(offset).all() or not np.isfinite(rpy).all():
        raise ValueError("mount translation and RPY must be finite 3-vectors")
    rotation = Rotation.from_euler("xyz", rpy)
    if rotate_180:
        rotation = rotation * Rotation.from_euler("x", 180.0, degrees=True)
    return rotation.as_matrix() @ OPTICAL_TO_FLU, offset


def decode_obb(result, frame: Frame, intrinsics: Intrinsics,
               mount: tuple[np.ndarray, np.ndarray] | None,
               max_range_m: float) -> list[dict]:
    obb = getattr(result, "obb", None)
    if obb is None or getattr(obb, "xyxyxyxy", None) is None:
        raise RuntimeError("best.engine produced no OBB output")
    polygons = np.asarray(obb.xyxyxyxy.cpu().numpy(), dtype=np.float64)
    scores = np.asarray(obb.conf.cpu().numpy(), dtype=np.float64)
    classes = (np.asarray(obb.cls.cpu().numpy(), dtype=np.int64)
               if getattr(obb, "cls", None) is not None else np.zeros(len(polygons), dtype=np.int64))
    if polygons.ndim != 3 or polygons.shape[1:] != (4, 2) or len(scores) != len(polygons):
        raise RuntimeError(f"invalid OBB output shape: {polygons.shape}")
    detections = []
    for polygon, score, class_id in zip(polygons, scores, classes):
        center = polygon.mean(axis=0)
        depth = sample_obb_depth_m(frame.depth_m, polygon, max_range_m)
        optical = (pixel_to_optical_m(*center, depth, intrinsics)
                   if depth is not None else None)
        body = (mount[0] @ optical + mount[1]
                if optical is not None and mount is not None else None)
        detections.append({
            "timestamp_ns": frame.timestamp_ns,
            "class_id": int(class_id),
            "class_name": str(getattr(result, "names", {}).get(int(class_id), class_id)),
            "confidence": float(score),
            "u_px": float(center[0]), "v_px": float(center[1]),
            "depth_m": depth,
            "optical_x_m": None if optical is None else float(optical[0]),
            "optical_y_m": None if optical is None else float(optical[1]),
            "optical_z_m": None if optical is None else float(optical[2]),
            "body_forward_m": None if body is None else float(body[0]),
            "body_left_m": None if body is None else float(body[1]),
            "body_up_m": None if body is None else float(body[2]),
        })
    return detections


def read_power() -> dict:
    values = {"oc1_events": None, "vdd_in_mv": None}
    for hwmon in Path("/sys/class/hwmon").glob("hwmon*"):
        try:
            name = (hwmon / "name").read_text().strip()
            if name == "soctherm_oc":
                values["oc1_events"] = int((hwmon / "oc1_event_cnt").read_text())
            elif name == "ina3221":
                for label in hwmon.glob("in*_label"):
                    if label.read_text().strip() == "VDD_IN":
                        values["vdd_in_mv"] = int((hwmon / label.name.replace("_label", "_input")).read_text())
                        break
        except (OSError, ValueError):
            continue
    return values


class Trace:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND |
                          getattr(os, "O_DSYNC", 0), 0o644)

    def emit(self, stage: str, **details) -> None:
        event = {"utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                 "stage": stage, **read_power(), **details}
        line = json.dumps(event, allow_nan=False) + "\n"
        data = line.encode("utf-8")
        while data:
            data = data[os.write(self.fd, data):]
        os.fsync(self.fd)
        print(line, end="", flush=True)

    def close(self) -> None:
        os.close(self.fd)


def open_zed(fps: int, depth_mode: str, max_range_m: float, rotate_180: bool):
    import pyzed.sl as sl

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = fps
    init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.depth_minimum_distance = 0.4
    init.depth_maximum_distance = max_range_m
    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED open failed: {status}")
    config = camera.get_camera_information().camera_configuration
    calibration = config.calibration_parameters.left_cam
    width, height = int(config.resolution.width), int(config.resolution.height)
    intrinsics = Intrinsics(float(calibration.fx), float(calibration.fy),
                            float(calibration.cx), float(calibration.cy))
    if rotate_180:
        intrinsics = Intrinsics(intrinsics.fx, intrinsics.fy,
                                width - 1 - intrinsics.cx,
                                height - 1 - intrinsics.cy)
    runtime = sl.RuntimeParameters()
    runtime.enable_depth = True
    return sl, camera, runtime, sl.Mat(), sl.Mat(), intrinsics


def grab_frame(sl, camera, runtime, color_mat, depth_mat, rotate_180: bool) -> Frame:
    status = camera.grab(runtime)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED grab failed: {status}")
    camera.retrieve_image(color_mat, sl.VIEW.LEFT_BGR)
    camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
    color, depth = color_mat.get_data(), depth_mat.get_data()
    if color is None or depth is None or color.ndim != 3 or color.shape[2] < 3:
        raise RuntimeError("ZED returned an empty color or depth frame")
    color = np.ascontiguousarray(color[:, :, :3])
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    depth = np.ascontiguousarray(depth)
    if color.shape[:2] != depth.shape[:2]:
        raise RuntimeError(f"color/depth shape mismatch: {color.shape} vs {depth.shape}")
    if rotate_180:
        color = np.ascontiguousarray(np.rot90(color, 2))
        depth = np.ascontiguousarray(np.rot90(depth, 2))
    timestamp_ns = int(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
    if timestamp_ns <= 0:
        raise RuntimeError("ZED returned an invalid image timestamp")
    return Frame(color, depth, timestamp_ns)


def save_sample(path: Path, frame: Frame, intrinsics: Intrinsics) -> None:
    with path.open("wb") as output:
        np.savez(output, color_bgr=frame.color_bgr, depth_m=frame.depth_m,
                 timestamp_ns=frame.timestamp_ns,
                 intrinsics=np.array([intrinsics.fx, intrinsics.fy,
                                      intrinsics.cx, intrinsics.cy]))
        output.flush()
        os.fsync(output.fileno())


def load_sample(path: Path) -> tuple[Frame, Intrinsics]:
    with np.load(path, allow_pickle=False) as data:
        frame = Frame(data["color_bgr"], data["depth_m"],
                      int(data["timestamp_ns"]))
        intrinsics = Intrinsics(*[float(value) for value in data["intrinsics"]])
    if frame.color_bgr.shape[:2] != frame.depth_m.shape[:2]:
        raise ValueError("saved color and depth frames are not aligned")
    return frame, intrinsics


def engine_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_model(path: Path):
    from ultralytics import YOLO
    return YOLO(str(path), task="obb")


def predict(model, image: np.ndarray, args):
    kwargs = {"data": str(args.model_data_config)} if args.model_data_config else {}
    device = int(args.device) if args.device.isdigit() else args.device
    results = model.predict(source=image, conf=args.confidence,
                            imgsz=args.imgsz, device=device, verbose=False, **kwargs)
    if not results or getattr(results[0], "obb", None) is None:
        raise RuntimeError(f"{args.model.name} did not return OBB results")
    return results[0]


def rate_summary(times: list[float], elapsed_s: float, start_s: float = 0.0) -> dict:
    gaps = ([max(0.0, times[0] - start_s),
             max(0.0, start_s + elapsed_s - times[-1])]
            if times else [])
    if len(times) > 1:
        gaps.extend(np.diff(times).tolist())
    return {"count": len(times), "hz": round(len(times) / elapsed_s, 3) if elapsed_s > 0 else 0.0,
            "max_gap_s": round(float(max(gaps)), 3) if gaps else None}


def latency_summary(values_ms: list[float]) -> dict:
    return {"median_ms": round(float(statistics.median(values_ms)), 2) if values_ms else None,
            "p95_ms": round(float(np.percentile(values_ms, 95)), 2) if values_ms else None}


def passes_ten_hz_target_gate(valid_3d: dict, valid_depth_fraction: float | None) -> bool:
    """Gate detected metric coordinates, rather than empty camera frames."""
    return bool(valid_3d["count"] > 1 and valid_3d["hz"] >= 10.0
                and valid_3d["max_gap_s"] is not None
                and valid_3d["max_gap_s"] <= 0.5
                and valid_depth_fraction is not None
                and valid_depth_fraction >= 0.9)


def output_tag(args) -> str:
    return ("camera" if args.mode == "camera" else
            f"{args.mode}_{args.model.suffix[1:]}_{args.device}")


def run(args, trace: Trace) -> dict:
    sample_path = args.output_dir / "sample_frame.npz"
    mount = (load_mount_transform(args.mount_config, args.rotate_180)
             if args.mode != "camera" and args.mount_config else None)
    frame = None
    intrinsics = None
    camera_parts = None
    camera = None
    if args.mode in {"camera", "combined"}:
        trace.emit("opening_zed", depth_mode=args.depth_mode, fps=args.fps)
        camera_parts = open_zed(args.fps, args.depth_mode, args.max_range_m, args.rotate_180)
        sl, camera, runtime, color_mat, depth_mat, intrinsics = camera_parts
        trace.emit("zed_opened")
        try:
            frame = grab_frame(sl, camera, runtime, color_mat, depth_mat, args.rotate_180)
            save_sample(sample_path, frame, intrinsics)
            trace.emit("first_frame_saved", timestamp_ns=frame.timestamp_ns,
                       sample=str(sample_path), shape=list(frame.color_bgr.shape))
        except BaseException:
            camera.close()
            raise
    else:
        frame, intrinsics = load_sample(sample_path)
        trace.emit("saved_frame_loaded", sample=str(sample_path))

    model = None
    if args.mode in {"engine", "combined"}:
        if not args.model.is_file() or args.model.suffix not in {".engine", ".pt"}:
            raise ValueError(f"expected an existing .engine or .pt file: {args.model}")
        if args.model.suffix == ".engine" and args.device == "cpu":
            raise ValueError("TensorRT engines require a GPU; use best.pt for CPU isolation")
        sha = engine_sha256(args.model)
        trace.emit("model_hash", path=str(args.model), bytes=args.model.stat().st_size, sha256=sha)
        if args.expected_sha256 and sha != args.expected_sha256:
            raise ValueError("model SHA-256 differs from the trusted checksum")
        trace.emit("loading_model", model=args.model.name, device=args.device)
        model = load_model(args.model)
        trace.emit("model_loaded")
        trace.emit("first_inference_started")
        result = predict(model, frame.color_bgr, args)
        decode_obb(result, frame, intrinsics, mount, args.max_range_m)
        trace.emit("first_inference_completed", obb_count=len(result.obb))

    frame_times, detection_times, target_3d_times = [], [], []
    grab_ms, inference_ms = [], []
    detections = []
    duplicate_timestamps = 0
    last_timestamp_ns = 0
    start = time.perf_counter()
    last_progress = start
    try:
        while time.perf_counter() - start < args.duration_s:
            if args.mode != "engine":
                t0 = time.perf_counter()
                frame = grab_frame(sl, camera, runtime, color_mat, depth_mat, args.rotate_180)
                grab_ms.append((time.perf_counter() - t0) * 1000)
                if frame.timestamp_ns <= last_timestamp_ns:
                    duplicate_timestamps += 1
                    continue
                last_timestamp_ns = frame.timestamp_ns
            frame_times.append(time.perf_counter())
            if model is not None:
                t0 = time.perf_counter()
                result = predict(model, frame.color_bgr, args)
                inference_ms.append((time.perf_counter() - t0) * 1000)
                current = decode_obb(result, frame, intrinsics, mount, args.max_range_m)
                if current:
                    detection_times.append(time.perf_counter())
                    detections.extend(current)
                if any(item["depth_m"] is not None for item in current):
                    target_3d_times.append(time.perf_counter())
            now = time.perf_counter()
            if now - last_progress >= 1.0:
                latest = next((item for item in reversed(detections)
                               if item["depth_m"] is not None), None)
                trace.emit("progress", elapsed_s=round(now - start, 2),
                           frames=len(frame_times), detections=len(detections),
                           frames_with_3d=len(target_3d_times), latest_3d=latest)
                last_progress = now
    finally:
        if camera is not None:
            camera.close()
            trace.emit("zed_closed")

    elapsed_s = time.perf_counter() - start
    valid_count = sum(item["depth_m"] is not None for item in detections)
    summary = {
        "mode": args.mode, "duration_s": round(elapsed_s, 3),
        "camera_or_pipeline": rate_summary(frame_times, elapsed_s, start),
        "detected_frames": rate_summary(detection_times, elapsed_s, start),
        "valid_3d_frames": rate_summary(target_3d_times, elapsed_s, start),
        "detections": len(detections), "detections_with_valid_depth": valid_count,
        "valid_depth_fraction": round(valid_count / len(detections), 3) if detections else None,
        "duplicate_or_stale_timestamps": duplicate_timestamps,
        "grab_latency": latency_summary(grab_ms),
        "inference_latency": latency_summary(inference_ms),
        "latest_3d": next((item for item in reversed(detections)
                           if item["depth_m"] is not None), None),
    }
    if args.mode == "combined":
        summary["ten_hz_target_gate"] = passes_ten_hz_target_gate(
            summary["valid_3d_frames"], summary["valid_depth_fraction"])
    if args.known_range_m is not None:
        depths = [item["depth_m"] for item in detections if item["depth_m"] is not None]
        summary["known_range_m"] = args.known_range_m
        summary["median_range_error_m"] = (round(abs(float(np.median(depths)) - args.known_range_m), 3)
                                           if depths else None)

    if detections:
        with (args.output_dir / f"{output_tag(args)}_detections.csv").open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=list(detections[0]))
            writer.writeheader()
            writer.writerows(detections)
    (args.output_dir / f"{output_tag(args)}_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    trace.emit("completed", summary=summary)
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["camera", "engine", "combined"], required=True)
    parser.add_argument("--output-dir", type=Path,
                        default=Path("output_dump/zed_engine_probe"))
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--model", type=Path,
                        default=Path("/home/jetson/cdrone_control/models/best.engine"))
    parser.add_argument("--device", default="0", help="CUDA device (0) or cpu; .pt only on CPU")
    parser.add_argument("--model-data-config", type=Path,
                        default=Path("/home/jetson/cdrone_control/ros2/src/drone_vision_pkg/config/drone_detector.yaml"))
    parser.add_argument("--expected-sha256", default="")
    parser.add_argument("--mount-config", type=Path,
                        default=Path("/home/jetson/zed_hover_mount_run2.yaml"))
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--depth-mode", choices=["PERFORMANCE", "QUALITY", "NEURAL"],
                        default="PERFORMANCE")
    parser.add_argument("--max-range-m", type=float, default=20.0)
    parser.add_argument("--known-range-m", type=float)
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--rotate-180", action="store_true", default=True)
    parser.add_argument("--no-rotate-180", dest="rotate_180", action="store_false")
    args = parser.parse_args(argv)
    if args.duration_s <= 0 or args.fps <= 0 or args.max_range_m <= 0:
        parser.error("duration, fps, and max range must be positive")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    trace = Trace(args.output_dir / f"{output_tag(args)}_events.jsonl")
    try:
        trace.emit("started", mode=args.mode, pid=os.getpid())
        run(args, trace)
        return 0
    except Exception as exc:
        trace.emit("failed", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        trace.close()


if __name__ == "__main__":
    sys.exit(main())
