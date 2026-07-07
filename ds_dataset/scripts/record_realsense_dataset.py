#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_COLOR_WIDTH = 1280
DEFAULT_COLOR_HEIGHT = 800
DEFAULT_COLOR_FPS = 15
DEFAULT_DEPTH_WIDTH = 848
DEFAULT_DEPTH_HEIGHT = 480
DEFAULT_DEPTH_FPS = 15
DEFAULT_SAVE_FPS = 4.0
DEFAULT_STATUS_INTERVAL_S = 5.0
DEFAULT_WARMUP_FRAMES = 15
SCHEMA_VERSION = 1

FRAME_CSV_FIELDNAMES = [
    "frame_index",
    "frame_stem",
    "host_wall_time_iso",
    "host_wall_time_s",
    "host_monotonic_s",
    "color_frame_number",
    "color_frame_timestamp_ms",
    "depth_frame_number",
    "depth_frame_timestamp_ms",
    "rgb_relpath",
    "depth_relpath",
    "yolo_label_relpath",
    "annotation_status",
    "color_width_px",
    "color_height_px",
    "depth_width_px",
    "depth_height_px",
]

ANNOTATION_INDEX_FIELDNAMES = [
    "frame_index",
    "frame_stem",
    "image_relpath",
    "label_relpath",
    "annotation_status",
    "class_hint",
    "notes",
]


def isoformat_now(timestamp_s: float | None = None) -> str:
    if timestamp_s is None:
        timestamp_s = time.time()
    return (
        datetime.fromtimestamp(timestamp_s, tz=timezone.utc)
        .astimezone()
        .isoformat(timespec="seconds")
    )


def sanitize_tag(value: str) -> str:
    cleaned = "".join(
        char if char.isalnum() or char in {"-", "_", "."} else "_"
        for char in value.strip()
    )
    cleaned = cleaned.strip("._-")
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    return cleaned or "manual_flight"


def default_dataset_root() -> Path:
    return Path(__file__).resolve().parents[1]


def build_session_name(tag: str) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{timestamp}_{sanitize_tag(tag)}"


def load_optional_module(module_name: str, package_name: str):
    try:
        module = __import__(module_name)
    except Exception as exc:
        raise RuntimeError(
            f"Missing dependency '{package_name}'. Install it before recording."
        ) from exc
    return module


@dataclass
class SessionPaths:
    session_dir: Path
    rgb_dir: Path
    depth_dir: Path
    annotations_dir: Path
    manifest_path: Path
    frames_csv_path: Path
    annotation_index_path: Path


class StopRequested(Exception):
    pass


class RealsenseDatasetRecorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.dataset_root = args.dataset_root.resolve()
        self.sessions_root = self.dataset_root / "sessions"
        self.runtime_dir = args.runtime_dir.resolve()
        self.session_name = args.session_name or build_session_name(args.tag)
        self.session_paths = self._prepare_session_paths()
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.status_path = (
            args.status_path.resolve()
            if args.status_path is not None
            else self.runtime_dir / f"{self.session_name}.status.json"
        )
        self.pid = os.getpid()
        self.capture_started_wall_s = time.time()
        self.capture_started_monotonic_s = time.monotonic()
        self.stop_requested = False
        self.stop_reason = "unknown"
        self.total_frames_seen = 0
        self.total_frames_saved = 0
        self.total_frames_skipped = 0
        self.last_saved_monotonic_s = 0.0
        self.last_saved_wall_s = 0.0
        self.last_status_monotonic_s = 0.0
        self.frames_file = None
        self.annotation_file = None
        self.manifest: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "session_name": self.session_name,
            "pid": self.pid,
            "tag": self.args.tag,
            "notes": self.args.notes,
            "dataset_root": str(self.dataset_root),
            "session_dir": str(self.session_paths.session_dir),
            "created_at": isoformat_now(self.capture_started_wall_s),
            "completed": False,
            "stop_reason": None,
            "capture_settings": {
                "save_depth": self.args.save_depth,
                "align_depth_to_color": self.args.align_depth_to_color,
                "color_width": self.args.color_width,
                "color_height": self.args.color_height,
                "color_fps": self.args.color_fps,
                "depth_width": self.args.depth_width,
                "depth_height": self.args.depth_height,
                "depth_fps": self.args.depth_fps,
                "save_fps": self.args.save_fps,
                "jpeg_quality": self.args.jpeg_quality,
                "warmup_frames": self.args.warmup_frames,
                "max_frames": self.args.max_frames,
                "max_seconds": self.args.max_seconds,
                "device_serial": self.args.device_serial,
            },
            "directories": {
                "rgb": "rgb",
                "annotations_yolo": "annotations_yolo",
            },
            "annotation_scaffold": {
                "format": "yolo",
                "class_map": {"0": "small_drone"},
                "missing_label_means": "not_annotated_yet",
                "empty_label_means": "annotated_negative_frame",
            },
            "summary": {
                "total_frames_seen": 0,
                "total_frames_saved": 0,
                "total_frames_skipped": 0,
            },
        }
        if self.args.save_depth:
            self.manifest["directories"]["depth"] = "depth"

    def _prepare_session_paths(self) -> SessionPaths:
        session_dir = self.sessions_root / self.session_name
        if session_dir.exists():
            raise FileExistsError(
                f"Session already exists. Pick a new session name: {session_dir}"
            )
        rgb_dir = session_dir / "rgb"
        depth_dir = session_dir / "depth"
        annotations_dir = session_dir / "annotations_yolo"
        for path in (self.sessions_root, session_dir, rgb_dir, depth_dir, annotations_dir):
            path.mkdir(parents=True, exist_ok=True)
        return SessionPaths(
            session_dir=session_dir,
            rgb_dir=rgb_dir,
            depth_dir=depth_dir,
            annotations_dir=annotations_dir,
            manifest_path=session_dir / "manifest.json",
            frames_csv_path=session_dir / "frames.csv",
            annotation_index_path=session_dir / "annotation_index.csv",
        )

    def request_stop(self, reason: str) -> None:
        self.stop_requested = True
        self.stop_reason = reason

    def _write_manifest(self) -> None:
        self.manifest["summary"] = {
            "total_frames_seen": self.total_frames_seen,
            "total_frames_saved": self.total_frames_saved,
            "total_frames_skipped": self.total_frames_skipped,
        }
        self.session_paths.manifest_path.write_text(
            json.dumps(self.manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _write_status(self, state: str) -> None:
        status = {
            "state": state,
            "pid": self.pid,
            "session_name": self.session_name,
            "session_dir": str(self.session_paths.session_dir),
            "status_path": str(self.status_path),
            "updated_at": isoformat_now(),
            "capture_started_at": isoformat_now(self.capture_started_wall_s),
            "last_saved_at": (
                isoformat_now(self.last_saved_wall_s)
                if self.last_saved_wall_s > 0.0
                else None
            ),
            "total_frames_seen": self.total_frames_seen,
            "total_frames_saved": self.total_frames_saved,
            "total_frames_skipped": self.total_frames_skipped,
            "save_depth": bool(self.args.save_depth),
            "save_fps": float(self.args.save_fps),
        }
        self.status_path.write_text(
            json.dumps(status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _write_annotation_index_row(self, writer: csv.DictWriter, frame_index: int, stem: str) -> None:
        writer.writerow(
            {
                "frame_index": frame_index,
                "frame_stem": stem,
                "image_relpath": f"rgb/{stem}.jpg",
                "label_relpath": f"annotations_yolo/{stem}.txt",
                "annotation_status": "pending",
                "class_hint": "small_drone",
                "notes": "",
            }
        )

    def _initialize_metadata_files(
        self,
    ) -> tuple[Any, csv.DictWriter, Any, csv.DictWriter]:
        frames_file = self.session_paths.frames_csv_path.open(
            "w",
            newline="",
            encoding="utf-8",
            buffering=1,
        )
        frames_writer = csv.DictWriter(frames_file, fieldnames=FRAME_CSV_FIELDNAMES)
        frames_writer.writeheader()

        annotation_file = self.session_paths.annotation_index_path.open(
            "w",
            newline="",
            encoding="utf-8",
            buffering=1,
        )
        annotation_writer = csv.DictWriter(
            annotation_file, fieldnames=ANNOTATION_INDEX_FIELDNAMES
        )
        annotation_writer.writeheader()
        return frames_file, frames_writer, annotation_file, annotation_writer

    def _save_frame(
        self,
        cv2,
        color_image,
        depth_image,
        frame_index: int,
        color_frame_number: int,
        color_timestamp_ms: float,
        depth_frame_number: int | str,
        depth_timestamp_ms: float | str,
        frames_writer: csv.DictWriter,
        annotation_writer: csv.DictWriter,
    ) -> None:
        now_wall_s = time.time()
        now_monotonic_s = time.monotonic()
        frame_stem = f"frame_{frame_index:06d}"
        rgb_relpath = Path("rgb") / f"{frame_stem}.jpg"
        depth_relpath = Path("depth") / f"{frame_stem}.png"
        label_relpath = Path("annotations_yolo") / f"{frame_stem}.txt"
        rgb_path = self.session_paths.session_dir / rgb_relpath
        depth_path = self.session_paths.session_dir / depth_relpath

        if not cv2.imwrite(
            str(rgb_path),
            color_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.args.jpeg_quality)],
        ):
            raise RuntimeError(f"Failed to write RGB frame: {rgb_path}")
        depth_relpath_value = ""
        depth_width_px = 0
        depth_height_px = 0
        if depth_image is not None:
            if not cv2.imwrite(str(depth_path), depth_image):
                raise RuntimeError(f"Failed to write depth frame: {depth_path}")
            depth_relpath_value = depth_relpath.as_posix()
            depth_width_px = int(depth_image.shape[1])
            depth_height_px = int(depth_image.shape[0])

        frames_writer.writerow(
            {
                "frame_index": frame_index,
                "frame_stem": frame_stem,
                "host_wall_time_iso": isoformat_now(now_wall_s),
                "host_wall_time_s": f"{now_wall_s:.6f}",
                "host_monotonic_s": f"{now_monotonic_s:.6f}",
                "color_frame_number": color_frame_number,
                "color_frame_timestamp_ms": f"{color_timestamp_ms:.3f}",
                "depth_frame_number": depth_frame_number,
                "depth_frame_timestamp_ms": depth_timestamp_ms,
                "rgb_relpath": rgb_relpath.as_posix(),
                "depth_relpath": depth_relpath_value,
                "yolo_label_relpath": label_relpath.as_posix(),
                "annotation_status": "pending",
                "color_width_px": int(color_image.shape[1]),
                "color_height_px": int(color_image.shape[0]),
                "depth_width_px": depth_width_px,
                "depth_height_px": depth_height_px,
            }
        )
        self._write_annotation_index_row(annotation_writer, frame_index, frame_stem)
        if self.frames_file is not None:
            self.frames_file.flush()
        if self.annotation_file is not None:
            self.annotation_file.flush()
        self.total_frames_saved += 1
        self.last_saved_monotonic_s = now_monotonic_s
        self.last_saved_wall_s = now_wall_s

    def _log_status(self) -> None:
        now_monotonic_s = time.monotonic()
        if now_monotonic_s - self.last_status_monotonic_s < self.args.status_interval_s:
            return
        self.last_status_monotonic_s = now_monotonic_s
        elapsed_s = now_monotonic_s - self.capture_started_monotonic_s
        print(
            "capture_status "
            f"session={self.session_name} "
            f"elapsed_s={elapsed_s:.1f} "
            f"seen={self.total_frames_seen} "
            f"saved={self.total_frames_saved} "
            f"skipped={self.total_frames_skipped}",
            flush=True,
        )
        self._write_manifest()
        self._write_status("running")

    def _should_stop_from_limits(self) -> None:
        if self.args.max_frames > 0 and self.total_frames_saved >= self.args.max_frames:
            raise StopRequested("max_frames_reached")
        if self.args.max_seconds > 0:
            elapsed_s = time.monotonic() - self.capture_started_monotonic_s
            if elapsed_s >= self.args.max_seconds:
                raise StopRequested("max_seconds_reached")

    def run(self) -> int:
        self._write_manifest()
        self._write_status("starting")
        if self.args.dry_run:
            self.manifest["completed"] = True
            self.manifest["stop_reason"] = "dry_run"
            self.manifest["dry_run"] = True
            self.manifest["completed_at"] = isoformat_now()
            self._write_manifest()
            self._write_status("dry_run")
            print(f"Dry run prepared session at {self.session_paths.session_dir}")
            return 0

        numpy = load_optional_module("numpy", "numpy")
        cv2 = load_optional_module("cv2", "opencv-python")
        rs = load_optional_module("pyrealsense2", "pyrealsense2")

        frames_file, frames_writer, annotation_file, annotation_writer = (
            self._initialize_metadata_files()
        )
        self.frames_file = frames_file
        self.annotation_file = annotation_file
        pipeline = None
        align = None

        try:
            config = rs.config()
            if self.args.device_serial:
                config.enable_device(self.args.device_serial)
            config.enable_stream(
                rs.stream.color,
                self.args.color_width,
                self.args.color_height,
                rs.format.bgr8,
                self.args.color_fps,
            )
            if self.args.save_depth:
                config.enable_stream(
                    rs.stream.depth,
                    self.args.depth_width,
                    self.args.depth_height,
                    rs.format.z16,
                    self.args.depth_fps,
                )

            pipeline = rs.pipeline()
            profile = pipeline.start(config)
            align = (
                rs.align(rs.stream.color)
                if self.args.save_depth and self.args.align_depth_to_color
                else None
            )

            device = profile.get_device()
            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intrinsics = color_profile.get_intrinsics()
            realsense_info: dict[str, Any] = {
                "device_name": str(device.get_info(rs.camera_info.name)),
                "serial_number": str(device.get_info(rs.camera_info.serial_number)),
                "firmware_version": str(
                    device.get_info(rs.camera_info.firmware_version)
                ),
                "color_intrinsics": {
                    "width": int(intrinsics.width),
                    "height": int(intrinsics.height),
                    "fx": float(intrinsics.fx),
                    "fy": float(intrinsics.fy),
                    "cx": float(intrinsics.ppx),
                    "cy": float(intrinsics.ppy),
                },
            }
            if self.args.save_depth:
                depth_sensor = device.first_depth_sensor()
                realsense_info["depth_scale_m"] = float(depth_sensor.get_depth_scale())
            self.manifest["realsense"] = realsense_info
            self._write_manifest()

            for _ in range(max(self.args.warmup_frames, 0)):
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                if align is not None:
                    frames = align.process(frames)
                if self.stop_requested:
                    raise StopRequested(self.stop_reason or "signal_received")

            print(
                "capture_started "
                f"session={self.session_name} "
                f"dir={self.session_paths.session_dir}",
                flush=True,
            )
            self._write_status("running")

            while True:
                if self.stop_requested:
                    raise StopRequested(self.stop_reason or "signal_received")
                self._should_stop_from_limits()
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                if align is not None:
                    frames = align.process(frames)

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame() if self.args.save_depth else None
                if not color_frame:
                    continue
                if self.args.save_depth and not depth_frame:
                    continue

                self.total_frames_seen += 1
                now_monotonic_s = time.monotonic()
                min_interval_s = 1.0 / max(self.args.save_fps, 1e-6)
                if (
                    self.total_frames_saved > 0
                    and now_monotonic_s - self.last_saved_monotonic_s < min_interval_s
                ):
                    self.total_frames_skipped += 1
                    self._log_status()
                    continue

                color_image = numpy.asanyarray(color_frame.get_data())
                depth_image = (
                    numpy.asanyarray(depth_frame.get_data())
                    if depth_frame is not None
                    else None
                )
                if color_image is None:
                    self.total_frames_skipped += 1
                    self._log_status()
                    continue
                if self.args.save_depth and depth_image is None:
                    self.total_frames_skipped += 1
                    self._log_status()
                    continue

                self._save_frame(
                    cv2=cv2,
                    color_image=color_image,
                    depth_image=depth_image,
                    frame_index=self.total_frames_saved + 1,
                    color_frame_number=int(color_frame.get_frame_number()),
                    color_timestamp_ms=float(color_frame.get_timestamp()),
                    depth_frame_number=(
                        int(depth_frame.get_frame_number())
                        if depth_frame is not None
                        else ""
                    ),
                    depth_timestamp_ms=(
                        f"{float(depth_frame.get_timestamp()):.3f}"
                        if depth_frame is not None
                        else ""
                    ),
                    frames_writer=frames_writer,
                    annotation_writer=annotation_writer,
                )
                self._log_status()
        except StopRequested as exc:
            self.manifest["stop_reason"] = str(exc)
            self._write_status("stopping")
            print(
                f"capture_stopping session={self.session_name} reason={exc}",
                flush=True,
            )
        except KeyboardInterrupt:
            self.manifest["stop_reason"] = "keyboard_interrupt"
            self._write_status("stopping")
            print(
                f"capture_stopping session={self.session_name} reason=keyboard_interrupt",
                flush=True,
            )
        except Exception:
            self.manifest["stop_reason"] = "error"
            self._write_manifest()
            self._write_status("error")
            raise
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            frames_file.close()
            annotation_file.close()
            self.manifest["completed"] = True
            self.manifest["completed_at"] = isoformat_now()
            if self.manifest.get("stop_reason") is None:
                self.manifest["stop_reason"] = "completed"
            self._write_manifest()
            self._write_status("completed")

        print(
            "capture_finished "
            f"session={self.session_name} "
            f"saved={self.total_frames_saved} "
            f"seen={self.total_frames_seen} "
            f"skipped={self.total_frames_skipped} "
            f"dir={self.session_paths.session_dir}",
            flush=True,
        )
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record aligned RealSense color/depth data into a raw dataset session "
            "for later YOLO annotation."
        )
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=default_dataset_root(),
        help="Root of the ds_dataset workspace.",
    )
    parser.add_argument(
        "--runtime-dir",
        type=Path,
        default=default_dataset_root() / "runtime",
        help="Runtime state directory for live capture status.",
    )
    parser.add_argument(
        "--status-path",
        type=Path,
        default=None,
        help="Optional explicit path for the live status JSON file.",
    )
    parser.add_argument(
        "--tag",
        default="manual_flight",
        help="Human label for the session name.",
    )
    parser.add_argument(
        "--session-name",
        default="",
        help="Exact session folder name. Defaults to a timestamped name.",
    )
    parser.add_argument(
        "--notes",
        default="",
        help="Optional notes stored in manifest.json.",
    )
    parser.add_argument(
        "--device-serial",
        default="",
        help="Optional RealSense serial if more than one camera is connected.",
    )
    parser.add_argument(
        "--color-width",
        type=int,
        default=DEFAULT_COLOR_WIDTH,
    )
    parser.add_argument(
        "--color-height",
        type=int,
        default=DEFAULT_COLOR_HEIGHT,
    )
    parser.add_argument(
        "--color-fps",
        type=int,
        default=DEFAULT_COLOR_FPS,
    )
    parser.add_argument(
        "--depth-width",
        type=int,
        default=DEFAULT_DEPTH_WIDTH,
    )
    parser.add_argument(
        "--depth-height",
        type=int,
        default=DEFAULT_DEPTH_HEIGHT,
    )
    parser.add_argument(
        "--depth-fps",
        type=int,
        default=DEFAULT_DEPTH_FPS,
    )
    parser.add_argument(
        "--align-depth-to-color",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Align depth to color before saving. Only matters with --save-depth.",
    )
    parser.add_argument(
        "--save-depth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Also save aligned depth PNGs. Off by default for RGB-only YOLO capture.",
    )
    parser.add_argument(
        "--save-fps",
        type=float,
        default=DEFAULT_SAVE_FPS,
        help="How many frames per second to keep on disk. 2.0 means one image every 0.5 seconds.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="OpenCV JPEG quality for RGB frames.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=DEFAULT_WARMUP_FRAMES,
        help="Frames to discard before saving starts.",
    )
    parser.add_argument(
        "--status-interval-s",
        type=float,
        default=DEFAULT_STATUS_INTERVAL_S,
        help="How often to print status lines.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Optional stop limit on saved frames. Zero means unlimited.",
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=0.0,
        help="Optional stop limit on wall-clock recording time. Zero means unlimited.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Create the session scaffold without touching the camera.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    recorder = RealsenseDatasetRecorder(args)

    def _handle_signal(signum: int, _frame) -> None:
        recorder.request_stop(f"signal_{signum}")

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        return recorder.run()
    except Exception as exc:
        print(f"capture_error {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
