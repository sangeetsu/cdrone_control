from __future__ import annotations

import curses
import threading
import time
from collections import OrderedDict

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from drone_monitor_pkg.topic_utils import cdrone_topic
from drone_msgs.msg import FlightStatus, SystemAlert


def safe_addstr(window, row: int, col: int, text: str, attr: int = 0) -> None:
    height, width = window.getmaxyx()
    if row < 0 or row >= height or col >= width:
        return
    available = max(0, width - col - 1)
    if available <= 0:
        return
    window.addstr(row, col, text[:available], attr)


class DashboardTuiNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_tui_node")

        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("dashboard_rate_hz", 5.0)
        self.declare_parameter("flight_status_topic", "")
        self.declare_parameter("alerts_topic", "")

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.dashboard_rate_hz = float(self.get_parameter("dashboard_rate_hz").value)
        self.flight_status_topic = (
            str(self.get_parameter("flight_status_topic").value).strip()
            or cdrone_topic(self.drone_id, "monitor/flight_status")
        )
        self.alerts_topic = (
            str(self.get_parameter("alerts_topic").value).strip()
            or cdrone_topic(self.drone_id, "monitor/alerts")
        )

        self.latest_status: FlightStatus | None = None
        self.alerts: OrderedDict[str, SystemAlert] = OrderedDict()
        self._stop = False

        self.create_subscription(FlightStatus, self.flight_status_topic, self.status_callback, 10)
        self.create_subscription(SystemAlert, self.alerts_topic, self.alert_callback, 20)

        self.get_logger().info(
            f"Dashboard listening on {self.flight_status_topic} and {self.alerts_topic}."
        )

    def status_callback(self, msg: FlightStatus) -> None:
        self.latest_status = msg

    def alert_callback(self, msg: SystemAlert) -> None:
        if msg.latched:
            self.alerts[msg.code] = msg
        else:
            self.alerts.pop(msg.code, None)
        while len(self.alerts) > 8:
            self.alerts.popitem(last=False)

    def stop(self) -> None:
        self._stop = True

    def run_ui(self) -> None:
        curses.wrapper(self._draw_loop)

    def _draw_loop(self, screen) -> None:
        curses.curs_set(0)
        screen.nodelay(True)

        sleep_s = 1.0 / max(self.dashboard_rate_hz, 1.0)
        while not self._stop:
            key = screen.getch()
            if key in (ord("q"), ord("Q")):
                self._stop = True
                break

            screen.erase()
            self._render(screen)
            screen.refresh()
            time.sleep(sleep_s)

    def _render(self, screen) -> None:
        safe_addstr(screen, 0, 0, f"cdrone dashboard :: {self.drone_id} :: press q to quit", curses.A_BOLD)
        status = self.latest_status
        if status is None:
            safe_addstr(screen, 2, 0, "Waiting for FlightStatus ...")
            return

        flight_attr = curses.A_BOLD
        auto_attr = curses.A_BOLD
        health_attr = curses.A_BOLD
        safe_addstr(screen, 2, 0, "Flight", flight_attr)
        safe_addstr(screen, 2, 42, "Autonomy", auto_attr)
        safe_addstr(screen, 2, 84, "Health", health_attr)

        flight_lines = [
            f"Connected: {status.connected}",
            f"Armed:     {status.armed}",
            f"Mode:      {status.mode}",
            f"Altitude:  {status.altitude_m:.2f} m",
            f"Pose XYZ:  {status.x_m:.2f} {status.y_m:.2f} {status.z_m:.2f}",
            f"RPY:       {status.roll_rad:.2f} {status.pitch_rad:.2f} {status.yaw_rad:.2f}",
            f"Vel XYZ:   {status.vx_mps:.2f} {status.vy_mps:.2f} {status.vz_mps:.2f}",
            f"Battery:   {status.battery_voltage_v:.2f} V  {status.battery_remaining_pct * 100.0:.0f}%",
        ]
        autonomy_lines = [
            f"Enabled:   {status.autonomy_enabled}",
            f"Ready:     {status.autonomy_ready}",
            f"Blocked:   {status.autonomy_blocked}",
            f"Reason:    {status.autonomy_blocked_reason}",
            f"State:     {status.behavior_state}",
            f"Track ID:  {status.active_track_id}",
            f"Range:     {status.target_distance_m:.2f} m",
            f"Bearing:   {status.target_bearing_rad:.2f} rad",
            f"Target:    {status.target_visible}",
            f"Cmd XYZ/Y: {status.cmd_vx_mps:.2f} {status.cmd_vy_mps:.2f} {status.cmd_vz_mps:.2f} {status.cmd_yaw_rate_rps:.2f}",
            f"Obstacle:  {status.obstacle_blocked}",
            f"E-stop:    {status.estop}",
        ]
        health_lines = [
            f"VIO age:   {status.vio_age_s:.2f} s",
            f"Tracks age:{status.tracks_age_s:.2f} s",
            f"MAVROS age:{status.mavros_age_s:.2f} s",
            f"Cmd age:   {status.cmd_age_s:.2f} s",
            f"Watchdog:  {status.watchdog_healthy}",
            f"Min dist:  {status.min_distance_gate_active}",
            f"FPS:       {status.perception_fps:.1f}",
            f"Infer:     {status.inference_latency_ms:.1f} ms",
        ]

        for idx, line in enumerate(flight_lines, start=3):
            safe_addstr(screen, idx, 0, line)
        for idx, line in enumerate(autonomy_lines, start=3):
            safe_addstr(screen, idx, 42, line)
        for idx, line in enumerate(health_lines, start=3):
            safe_addstr(screen, idx, 84, line)

        alert_row = 16
        safe_addstr(screen, alert_row, 0, "Alerts", curses.A_BOLD)
        if not self.alerts:
            safe_addstr(screen, alert_row + 1, 0, "No active alerts")
            return

        severity_text = {
            SystemAlert.SEVERITY_INFO: "INFO",
            SystemAlert.SEVERITY_WARN: "WARN",
            SystemAlert.SEVERITY_ERROR: "ERROR",
            SystemAlert.SEVERITY_FATAL: "FATAL",
        }
        for idx, alert in enumerate(self.alerts.values(), start=1):
            text = f"[{severity_text.get(alert.severity, 'UNK')}] {alert.code}: {alert.message}"
            safe_addstr(screen, alert_row + idx, 0, text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardTuiNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
