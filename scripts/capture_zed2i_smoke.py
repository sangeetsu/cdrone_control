#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
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
        f"this smoke test. Original error: {exc}"
    ) from exc


def _depth_colormap(depth_m: np.ndarray, max_depth_m: float) -> np.ndarray:
    depth = np.nan_to_num(depth_m.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    clipped = np.clip(depth, 0.0, max(max_depth_m, 0.1))
    normalized = (255.0 * (clipped / max(max_depth_m, 0.1))).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored[depth <= 0.0] = (0, 0, 0)
    return colored


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture a ZED 2i smoke-test image set.")
    parser.add_argument(
        "--output-dir",
        default="/home/jetson/output_dump/zed2i_smoke",
        help="Directory for rgb.png, depth.png, and metadata.json.",
    )
    parser.add_argument("--frames-to-skip", type=int, default=15)
    parser.add_argument("--max-depth-m", type=float, default=20.0)
    parser.add_argument("--min-depth-m", type=float, default=0.4)
    parser.add_argument(
        "--depth-mode",
        default="NEURAL",
        choices=["PERFORMANCE", "QUALITY", "ULTRA", "NEURAL", "NEURAL_PLUS"],
    )
    parser.add_argument(
        "--rotate-180",
        action="store_true",
        help="Rotate saved RGB/depth images 180 degrees for an upside-down mount.",
    )
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = max(args.fps, 1)
    init.depth_mode = getattr(sl.DEPTH_MODE, args.depth_mode)
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.depth_minimum_distance = float(args.min_depth_m)
    init.depth_maximum_distance = float(args.max_depth_m)

    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"Could not open ZED 2i: {status}")

    runtime = sl.RuntimeParameters()
    runtime.enable_depth = True
    image_mat = sl.Mat()
    depth_mat = sl.Mat()
    try:
        for _ in range(max(args.frames_to_skip, 0)):
            camera.grab(runtime)

        status = camera.grab(runtime)
        if status != sl.ERROR_CODE.SUCCESS:
            raise SystemExit(f"Could not grab ZED 2i frame: {status}")

        camera.retrieve_image(image_mat, sl.VIEW.LEFT_BGR)
        camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        image = image_mat.get_data()
        depth = depth_mat.get_data()
        if image is None or depth is None:
            raise SystemExit("ZED 2i returned an empty image or depth frame.")
        image_bgr = np.ascontiguousarray(image[:, :, :3])
        depth_m = np.asarray(depth, dtype=np.float32)
        if depth_m.ndim == 3:
            depth_m = depth_m[:, :, 0]
        depth_m = np.ascontiguousarray(depth_m)
        if args.rotate_180:
            image_bgr = np.ascontiguousarray(np.rot90(image_bgr, 2))
            depth_m = np.ascontiguousarray(np.rot90(depth_m, 2))

        rgb_path = output_dir / "rgb.png"
        depth_path = output_dir / "depth.png"
        metadata_path = output_dir / "metadata.json"

        cv2.imwrite(str(rgb_path), image_bgr)
        cv2.imwrite(str(depth_path), _depth_colormap(depth_m, args.max_depth_m))

        info = camera.get_camera_information()
        calibration = info.camera_configuration.calibration_parameters.left_cam
        raw_intrinsics = {
            "fx": float(calibration.fx),
            "fy": float(calibration.fy),
            "cx": float(calibration.cx),
            "cy": float(calibration.cy),
        }
        effective_intrinsics = dict(raw_intrinsics)
        if args.rotate_180:
            effective_intrinsics["cx"] = (
                float(image_bgr.shape[1]) - 1.0 - raw_intrinsics["cx"]
            )
            effective_intrinsics["cy"] = (
                float(image_bgr.shape[0]) - 1.0 - raw_intrinsics["cy"]
            )
        metadata = {
            "captured_wall_time_s": time.time(),
            "sdk_version": sl.Camera.get_sdk_version(),
            "serial_number": int(info.serial_number),
            "camera_model": str(info.camera_model),
            "resolution": {
                "width": int(image_bgr.shape[1]),
                "height": int(image_bgr.shape[0]),
            },
            "fps": int(args.fps),
            "depth_mode": args.depth_mode,
            "depth_min_m": float(args.min_depth_m),
            "depth_max_m": float(args.max_depth_m),
            "rotate_180": bool(args.rotate_180),
            "raw_intrinsics": raw_intrinsics,
            "effective_intrinsics": effective_intrinsics,
            "outputs": {
                "rgb": str(rgb_path),
                "depth": str(depth_path),
            },
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"Saved ZED 2i smoke-test capture to {output_dir}")
        return 0
    finally:
        camera.close()


if __name__ == "__main__":
    raise SystemExit(main())
