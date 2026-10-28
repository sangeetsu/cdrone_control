"""Run ZED VIO and drone detection with one SDK camera and direct frame handoff."""

import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def read_jetson_power() -> tuple[int | None, int | None]:
    """Read Orin Nano OC1 events and measured module VDD_IN when present."""
    oc1 = None
    vdd_in_mv = None
    for hwmon in Path("/sys/class/hwmon").glob("hwmon*"):
        try:
            name = (hwmon / "name").read_text().strip()
            if name == "soctherm_oc":
                oc1 = int((hwmon / "oc1_event_cnt").read_text().strip())
            elif name == "ina3221":
                for label in hwmon.glob("in*_label"):
                    if label.read_text().strip() == "VDD_IN":
                        vdd_in_mv = int((hwmon / label.name.replace(
                            "_label", "_input")).read_text().strip())
                        break
        except (OSError, ValueError):
            continue
    return oc1, vdd_in_mv


def _power_snapshot() -> str:
    """Include board power evidence in the durable startup trace when available."""
    oc1, vdd_in_mv = read_jetson_power()
    oc1 = oc1 if oc1 is not None else "unavailable"
    vdd_in_mv = vdd_in_mv if vdd_in_mv is not None else "unavailable"
    return f"oc1_events={oc1} vdd_in_mv={vdd_in_mv}"


def check_input_power(*, sample_s: float = 2.0, min_vdd_in_mv: int = 4850) -> str:
    """Reject detector startup if the Jetson's module supply is already unsafe.

    Use 100 mV of margin above the Orin Nano's 4.75 V minimum. This is a
    preflight gate, not a substitute for measuring fast load transients.
    """
    first_oc1, first_voltage = read_jetson_power()
    if first_oc1 is None or first_voltage is None:
        return "Jetson OC1 or VDD_IN telemetry unavailable; cannot verify input power"
    if first_voltage < min_vdd_in_mv:
        return (f"Jetson VDD_IN {first_voltage} mV is below {min_vdd_in_mv} mV; "
                "check the power regulator, wiring, and connector")
    time.sleep(sample_s)
    second_oc1, second_voltage = read_jetson_power()
    if second_oc1 is None or second_voltage is None:
        return "Jetson OC1 or VDD_IN telemetry unavailable; cannot verify input power"
    if second_voltage < min_vdd_in_mv:
        return (f"Jetson VDD_IN {second_voltage} mV is below {min_vdd_in_mv} mV; "
                "check the power regulator, wiring, and connector")
    if second_oc1 > first_oc1:
        return (f"Jetson OC1 undervoltage events rose from {first_oc1} "
                f"to {second_oc1} in {sample_s:.1f} s; check the power supply")
    return ""


def _startup_stage(message: str) -> None:
    # A sudden reset left ROS log files empty on this Jetson. Keep a separate
    # synced trace that survives an orderly flush failure as far as storage permits.
    timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    line = (f"[{timestamp}] pid={os.getpid()} milestone4 startup: {message}; "
            f"{_power_snapshot()}\n")
    print(line, end="", file=sys.stderr, flush=True)
    trace_path = Path(os.environ.get(
        "MILESTONE4_STARTUP_TRACE", "~/.ros/log/milestone4_startup_trace.log")).expanduser()
    try:
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND | getattr(os, "O_DSYNC", 0)
        fd = os.open(trace_path, flags, 0o644)
        try:
            data = line.encode("utf-8")
            while data:
                written = os.write(fd, data)
                if written == 0:
                    raise OSError("startup trace write returned zero bytes")
                data = data[written:]
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError as exc:
        print(f"milestone4 startup trace unavailable: {exc}", file=sys.stderr, flush=True)


def warm_detector_engine(tracker) -> None:
    """Allocate TensorRT inference resources before ZED processing begins."""
    import numpy as np

    kwargs = {"data": tracker.model_data_config} if tracker.model_data_config else {}
    results = tracker.model.predict(
        source=np.zeros((tracker.model_input_size, tracker.model_input_size, 3),
                        dtype=np.uint8),
        conf=tracker.conf_threshold,
        imgsz=tracker.model_input_size,
        verbose=False,
        device=tracker.inference_device,
        **kwargs,
    )
    output_name = "obb" if tracker.model_task == "obb" else "boxes"
    if not results or getattr(results[0], output_name, None) is None:
        raise RuntimeError(f"Detector warmup returned no {output_name} output")


def main(args=None):
    _startup_stage("checking Jetson input power")
    power_issue = check_input_power()
    if power_issue:
        _startup_stage(f"startup stopped: {power_issue}")
        raise RuntimeError(power_issue)
    _startup_stage("importing ROS and ZED VIO")
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from drone_control_pkg.zed_vio_node import ZedVioNode

    _startup_stage("importing detector dependencies")
    from drone_vision_pkg.realsense_tracker_node import RealsenseTrackerNode

    _startup_stage("initializing ROS")
    rclpy.init(args=args)
    tracker = None
    zed = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        _startup_stage("loading detector model")
        tracker = RealsenseTrackerNode()
        _startup_stage("detector model loaded")
        if tracker.source_mode != "embedded_zed":
            raise RuntimeError("combined ZED node requires source_mode=embedded_zed")
        power_issue = check_input_power()
        if power_issue:
            _startup_stage(f"startup stopped: {power_issue}")
            raise RuntimeError(power_issue)
        _startup_stage("warming detector engine before camera open")
        warm_detector_engine(tracker)
        _startup_stage("detector engine warmed")
        _startup_stage("opening ZED and enabling positional tracking")
        zed = ZedVioNode()
        _startup_stage("ZED constructor returned")
        if zed.camera is None or not zed.capture_frames:
            raise RuntimeError("combined ZED node could not start frame capture")
        zed.frame_sink = tracker.accept_zed_frame
        executor.add_node(zed)
        executor.add_node(tracker)
        _startup_stage("spinning ROS; waiting for healthy ZED frames")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if tracker is not None:
            tracker.destroy_node()
        if zed is not None:
            zed.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
