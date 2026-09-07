#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np

try:
    import cv2
except Exception as exc:  # pragma: no cover - platform specific
    raise SystemExit(f"OpenCV import failed: {exc}") from exc

try:
    import pyzed.sl as sl
except Exception as exc:  # pragma: no cover - platform specific
    raise SystemExit(
        "ZED SDK Python import failed. Install/fix pyzed.sl before running "
        f"this ladder capture. Original error: {exc}"
    ) from exc

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - optional detector path
    YOLO = None

REPO_ROOT = Path(__file__).resolve().parents[1]
ROS_SRC = REPO_ROOT / "ros2" / "src" / "drone_vision_pkg"
if str(ROS_SRC) not in sys.path:
    sys.path.insert(0, str(ROS_SRC))

from drone_vision_pkg.model_paths import resolve_model_path  # noqa: E402
from drone_vision_pkg.zed_depth_ladder import (  # noqa: E402
    FRAME_CSV,
    FRAME_FIELDNAMES,
    bbox_roi,
    center_roi,
    depth_roi_stats,
    empty_depth_stats,
    frame_row,
    range_label,
    summarize_capture_rows,
    write_run_summary,
)


def depth_colormap(depth_m: np.ndarray, max_depth_m: float) -> np.ndarray:
    """Render a depth visualization."""

    depth = np.nan_to_num(depth_m.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    clipped = np.clip(depth, 0.0, max(float(max_depth_m), 0.1))
    normalized = (255.0 * (clipped / max(float(max_depth_m), 0.1))).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored[depth <= 0.0] = (0, 0, 0)
    return colored


def normalize_enum_name(value: str, enum_cls, parameter_name: str) -> str:
    """Normalize a pyzed enum name."""

    normalized = str(value or "").strip().upper()
    allowed = {name for name in dir(enum_cls) if name.isupper()}
    if normalized not in allowed:
        options = ", ".join(sorted(name.lower() for name in allowed))
        raise SystemExit(f"{parameter_name} must be one of: {options}")
    return normalized


def load_detector(model_path: str):
    """Load YOLO when requested."""

    if not model_path:
        return None
    if YOLO is None:
        raise SystemExit("ultralytics is not available, but --model-path was supplied.")
    return YOLO(resolve_model_path(model_path), task="detect")


def best_detection(detector, image_bgr: np.ndarray, *, conf_threshold: float):
    """Return the highest-confidence YOLO bbox."""

    if detector is None:
        return None, 0
    results = detector.predict(
        image_bgr,
        imgsz=640,
        conf=float(conf_threshold),
        verbose=False,
    )
    if not results or results[0].boxes is None or results[0].boxes.xyxy is None:
        return None, 0
    boxes = results[0].boxes
    xyxy = boxes.xyxy.cpu().numpy()
    if len(xyxy) == 0:
        return None, 0
    conf = boxes.conf.cpu().numpy() if boxes.conf is not None else np.ones(len(xyxy))
    height, width = image_bgr.shape[:2]
    image_area = max(float(width * height), 1.0)
    image_center = np.array([width / 2.0, height / 2.0], dtype=np.float64)
    candidates: list[tuple[float, int]] = []
    for idx, bbox_raw in enumerate(xyxy):
        score = float(conf[idx])
        if not np.isfinite(score) or score < float(conf_threshold) or score > 1.0:
            continue
        x1, y1, x2, y2 = [float(value) for value in bbox_raw[:4]]
        if x2 <= x1 or y2 <= y1:
            continue
        area = (x2 - x1) * (y2 - y1)
        area_fraction = area / image_area
        if area_fraction < 0.001 or area_fraction > 0.50:
            continue
        bbox_center = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0])
        center_distance = float(np.linalg.norm(bbox_center - image_center))
        center_penalty = center_distance / max(float(width), float(height), 1.0)
        candidates.append((score - 0.25 * center_penalty, idx))
    if not candidates:
        return None, int(len(xyxy))
    _, best_index = max(candidates, key=lambda item: item[0])
    bbox = [float(value) for value in xyxy[best_index][:4]]
    area = max(0.0, (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]))
    return {
        "bbox_xyxy": bbox,
        "confidence": float(conf[best_index]),
        "bbox_area_px": area,
    }, int(len(xyxy))


def open_zed(args) -> sl.Camera:
    """Open the ZED using Milestone 4-compatible defaults."""

    resolution_name = normalize_enum_name(
        args.camera_resolution,
        sl.RESOLUTION,
        "camera_resolution",
    )
    depth_mode_name = normalize_enum_name(args.depth_mode, sl.DEPTH_MODE, "depth_mode")
    coordinate_system_name = normalize_enum_name(
        args.coordinate_system,
        sl.COORDINATE_SYSTEM,
        "coordinate_system",
    )

    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, resolution_name)
    init.camera_fps = max(int(args.fps), 1)
    init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode_name)
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = getattr(sl.COORDINATE_SYSTEM, coordinate_system_name)
    init.depth_minimum_distance = float(args.min_depth_m)
    init.depth_maximum_distance = float(args.max_depth_m)

    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"Could not open ZED 2i: {status}")
    return camera


def draw_detection_overlay(
    image_bgr: np.ndarray,
    detection: dict[str, object] | None,
    *,
    nominal_range_m: float,
    center_median_m: object,
    bbox_median_m: object,
    roi_center_x_px: float | None = None,
    roi_center_y_px: float | None = None,
) -> np.ndarray:
    """Draw a lightweight diagnostic overlay."""

    overlay = image_bgr.copy()
    height, width = overlay.shape[:2]
    center = (
        int(round(width / 2 if roi_center_x_px is None else roi_center_x_px)),
        int(round(height / 2 if roi_center_y_px is None else roi_center_y_px)),
    )
    cv2.drawMarker(
        overlay,
        center,
        (0, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=2,
    )
    if detection is not None:
        x1, y1, x2, y2 = [int(round(float(v))) for v in detection["bbox_xyxy"]]
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
    lines = [
        f"nominal={nominal_range_m:.2f}m",
        f"center={format_depth(center_median_m)}",
        f"bbox={format_depth(bbox_median_m)}",
    ]
    for idx, line in enumerate(lines):
        y = 28 + idx * 26
        cv2.putText(
            overlay,
            line,
            (18, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            line,
            (18, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return overlay


def format_depth(value: object) -> str:
    """Format optional depth."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return "n/a"
    if not np.isfinite(number):
        return "n/a"
    return f"{number:.2f}m"


def capture(args) -> int:
    """Capture one nominal range."""

    nominal_range_m = float(args.range_m)
    label = args.range_label or range_label(nominal_range_m)
    session_dir = Path(args.output_dir).expanduser()
    run_dir = session_dir / label
    frame_dir = run_dir / "frames"
    frame_dir.mkdir(parents=True, exist_ok=True)

    detector = load_detector(str(args.model_path or ""))
    camera = open_zed(args)
    runtime = sl.RuntimeParameters()
    runtime.enable_depth = True
    image_mat = sl.Mat()
    depth_mat = sl.Mat()
    rows: list[dict[str, object]] = []
    saved_samples: list[dict[str, object]] = []

    try:
        for _ in range(max(int(args.frames_to_skip), 0)):
            camera.grab(runtime)

        started_s = time.time()
        frame_index = 0
        while time.time() - started_s < float(args.duration_s):
            status = camera.grab(runtime)
            if status != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.01)
                continue

            camera.retrieve_image(image_mat, sl.VIEW.LEFT_BGR)
            camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            image = image_mat.get_data()
            depth = depth_mat.get_data()
            if image is None or depth is None:
                continue

            image_bgr = np.ascontiguousarray(image[:, :, :3])
            depth_m = np.asarray(depth, dtype=np.float32)
            if depth_m.ndim == 3:
                depth_m = depth_m[:, :, 0]
            depth_m = np.ascontiguousarray(depth_m)
            if args.rotate_180:
                image_bgr = np.ascontiguousarray(np.rot90(image_bgr, 2))
                depth_m = np.ascontiguousarray(np.rot90(depth_m, 2))

            h, w = depth_m.shape[:2]
            center_stats = depth_roi_stats(
                depth_m,
                center_roi(
                    width_px=w,
                    height_px=h,
                    half_size_px=args.center_half_size_px,
                    center_x_px=args.roi_center_x_px,
                    center_y_px=args.roi_center_y_px,
                ),
                max_valid_depth_m=args.max_depth_m,
            )
            detection, detections_count = best_detection(
                detector,
                image_bgr,
                conf_threshold=args.conf_threshold,
            )
            bbox_stats = empty_depth_stats()
            if detection is not None:
                bbox_stats = depth_roi_stats(
                    depth_m,
                    bbox_roi(
                        detection["bbox_xyxy"],
                        width_px=w,
                        height_px=h,
                        padding_fraction=args.bbox_padding_fraction,
                    ),
                    max_valid_depth_m=args.max_depth_m,
                )

            timestamp_s = time.time()
            rows.append(
                frame_row(
                    range_label_text=label,
                    nominal_range_m=nominal_range_m,
                    frame_index=frame_index,
                    timestamp_s=timestamp_s,
                    center_stats=center_stats,
                    bbox_stats=bbox_stats,
                    detections_count=detections_count,
                    best_confidence=(
                        None if detection is None else detection["confidence"]
                    ),
                    best_bbox_area_px=(
                        None if detection is None else detection["bbox_area_px"]
                    ),
                )
            )

            if args.save_every_n > 0 and frame_index % int(args.save_every_n) == 0:
                rgb_path = frame_dir / f"frame_{frame_index:04d}_rgb.png"
                depth_path = frame_dir / f"frame_{frame_index:04d}_depth.png"
                overlay_path = frame_dir / f"frame_{frame_index:04d}_overlay.png"
                cv2.imwrite(str(rgb_path), image_bgr)
                cv2.imwrite(str(depth_path), depth_colormap(depth_m, args.max_depth_m))
                cv2.imwrite(
                    str(overlay_path),
                    draw_detection_overlay(
                        image_bgr,
                        detection,
                        nominal_range_m=nominal_range_m,
                        center_median_m=center_stats.get("median_m"),
                        bbox_median_m=bbox_stats.get("median_m"),
                        roi_center_x_px=args.roi_center_x_px,
                        roi_center_y_px=args.roi_center_y_px,
                    ),
                )
                saved_samples.append(
                    {
                        "frame_index": frame_index,
                        "rgb": str(rgb_path),
                        "depth": str(depth_path),
                        "overlay": str(overlay_path),
                    }
                )

            frame_index += 1
            target_dt = 1.0 / max(float(args.sample_rate_hz), 0.1)
            elapsed = time.time() - timestamp_s
            if elapsed < target_dt:
                time.sleep(target_dt - elapsed)

        info = camera.get_camera_information()
        calibration = info.camera_configuration.calibration_parameters.left_cam
        manifest = {
            "range_label": label,
            "nominal_range_m": nominal_range_m,
            "captured_wall_time_s": time.time(),
            "sdk_version": sl.Camera.get_sdk_version(),
            "serial_number": int(info.serial_number),
            "camera_model": str(info.camera_model),
            "depth_mode": str(args.depth_mode).upper(),
            "camera_resolution": str(args.camera_resolution).upper(),
            "fps": int(args.fps),
            "rotate_180": bool(args.rotate_180),
            "depth_min_m": float(args.min_depth_m),
            "depth_max_m": float(args.max_depth_m),
            "center_half_size_px": int(args.center_half_size_px),
            "roi_center_x_px": args.roi_center_x_px,
            "roi_center_y_px": args.roi_center_y_px,
            "bbox_padding_fraction": float(args.bbox_padding_fraction),
            "model_path": str(args.model_path or ""),
            "outputs": {
                "frames_csv": str(run_dir / FRAME_CSV),
                "summary_json": str(run_dir / "summary.json"),
                "sample_frames": saved_samples,
            },
            "raw_intrinsics": {
                "fx": float(calibration.fx),
                "fy": float(calibration.fy),
                "cx": float(calibration.cx),
                "cy": float(calibration.cy),
            },
        }
        (run_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n",
            encoding="utf-8",
        )

        with (run_dir / FRAME_CSV).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=FRAME_FIELDNAMES)
            writer.writeheader()
            writer.writerows(rows)

        summary = summarize_capture_rows(
            rows,
            nominal_range_m=nominal_range_m,
            range_label_text=label,
            run_dir=run_dir,
        )
        write_run_summary(run_dir, summary)
        print(json.dumps(summary, indent=2))
        print(f"Wrote ZED depth ladder capture to {run_dir}")
        return 0
    finally:
        camera.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture one floor-mark ZED 2i depth ladder range."
    )
    parser.add_argument("--range-m", type=float, required=True)
    parser.add_argument("--range-label", default="")
    parser.add_argument(
        "--output-dir",
        default="/home/jetson/output_dump/zed2i_depth_ladder",
    )
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--sample-rate-hz", type=float, default=5.0)
    parser.add_argument("--frames-to-skip", type=int, default=15)
    parser.add_argument("--save-every-n", type=int, default=10)
    parser.add_argument("--center-half-size-px", type=int, default=20)
    parser.add_argument("--roi-center-x-px", type=float, default=None)
    parser.add_argument("--roi-center-y-px", type=float, default=None)
    parser.add_argument("--bbox-padding-fraction", type=float, default=0.10)
    parser.add_argument("--model-path", default="")
    parser.add_argument("--conf-threshold", type=float, default=0.40)
    parser.add_argument("--camera-resolution", default="HD720")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--depth-mode", default="NEURAL")
    parser.add_argument("--coordinate-system", default="RIGHT_HANDED_Z_UP_X_FWD")
    parser.add_argument("--min-depth-m", type=float, default=0.4)
    parser.add_argument("--max-depth-m", type=float, default=20.0)
    parser.add_argument("--rotate-180", action="store_true", default=False)
    return capture(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
