from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_control_pkg.deployment_config import (
    configured_drone_id,
    configured_mavros_namespace,
)
from drone_control_pkg.topic_utils import cdrone_topic, join_topic


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min(value, max_v), min_v)


def quaternion_to_yaw_rad(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def project_forward_distance(
    start_xy: Tuple[float, float],
    current_xy: Tuple[float, float],
    heading_rad: float,
) -> float:
    dx = current_xy[0] - start_xy[0]
    dy = current_xy[1] - start_xy[1]
    return dx * math.cos(heading_rad) + dy * math.sin(heading_rad)


class AltctlDemoSequenceNode(Node):
    ACTIVE_MANUAL_STATES = {
        "SET_ALTCTL",
        "ARMING",
        "TAKEOFF",
        "HOVER_AFTER_TAKEOFF",
        "FORWARD_TRANSLATE",
        "HOVER_AFTER_TRANSLATE",
    }
    ALTCTL_REQUIRED_STATES = {
        "ARMING",
        "TAKEOFF",
        "HOVER_AFTER_TAKEOFF",
        "FORWARD_TRANSLATE",
        "HOVER_AFTER_TRANSLATE",
    }

    def __init__(self) -> None:
        super().__init__("altctl_demo_sequence_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("target_altitude_m", 0.7)
        self.declare_parameter("forward_distance_m", 1.2)
        self.declare_parameter("hover_after_takeoff_s", 2.0)
        self.declare_parameter("hover_after_translate_s", 2.0)
        self.declare_parameter("arm_zero_throttle_hold_s", 1.5)
        self.declare_parameter("takeoff_throttle_delta", 180.0)
        self.declare_parameter("hover_throttle_center", 500.0)
        self.declare_parameter("forward_stick_cmd", 220.0)
        self.declare_parameter("altitude_tolerance_m", 0.1)
        self.declare_parameter("position_tolerance_m", 0.1)
        self.declare_parameter("stage_timeout_s", 20.0)
        self.declare_parameter("land_detect_altitude_m", 0.15)
        self.declare_parameter("manual_control_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("local_pose_topic", "")
        self.declare_parameter("start_service", "")
        self.declare_parameter("abort_service", "")
        self.declare_parameter("status_topic", "")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.target_altitude_m = float(self.get_parameter("target_altitude_m").value)
        self.forward_distance_m = float(self.get_parameter("forward_distance_m").value)
        self.hover_after_takeoff_s = float(
            self.get_parameter("hover_after_takeoff_s").value
        )
        self.hover_after_translate_s = float(
            self.get_parameter("hover_after_translate_s").value
        )
        self.arm_zero_throttle_hold_s = float(
            self.get_parameter("arm_zero_throttle_hold_s").value
        )
        self.takeoff_throttle_delta = float(
            self.get_parameter("takeoff_throttle_delta").value
        )
        self.hover_throttle_center = float(
            self.get_parameter("hover_throttle_center").value
        )
        self.forward_stick_cmd = float(self.get_parameter("forward_stick_cmd").value)
        self.altitude_tolerance_m = float(
            self.get_parameter("altitude_tolerance_m").value
        )
        self.position_tolerance_m = float(
            self.get_parameter("position_tolerance_m").value
        )
        self.stage_timeout_s = float(self.get_parameter("stage_timeout_s").value)
        self.land_detect_altitude_m = float(
            self.get_parameter("land_detect_altitude_m").value
        )

        self.manual_control_topic = (
            str(self.get_parameter("manual_control_topic").value).strip()
            or join_topic(self.mavros_namespace, "manual_control/send")
        )
        self.state_topic = (
            str(self.get_parameter("state_topic").value).strip()
            or join_topic(self.mavros_namespace, "state")
        )
        self.local_pose_topic = (
            str(self.get_parameter("local_pose_topic").value).strip()
            or join_topic(self.mavros_namespace, "local_position/pose")
        )
        self.start_service_name = (
            str(self.get_parameter("start_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/start")
        )
        self.abort_service_name = (
            str(self.get_parameter("abort_service").value).strip()
            or cdrone_topic(self.drone_id, "demo/abort")
        )
        self.status_topic = (
            str(self.get_parameter("status_topic").value).strip()
            or cdrone_topic(self.drone_id, "demo/state")
        )

        self.mode_service = join_topic(self.mavros_namespace, "set_mode")
        self.arm_service = join_topic(self.mavros_namespace, "cmd/arming")

        self.pose_timeout_s = 1.0
        self.state_timeout_s = 2.0
        self.touchdown_dwell_s = 1.0
        self.altitude_kp = 220.0

        self.latest_state = State()
        self.latest_pose = PoseStamped()
        self.last_state_time_s = 0.0
        self.last_pose_time_s = 0.0

        self.demo_state = "IDLE"
        self.stage_started_s = self.now_s()
        self.forward_start_xy: Optional[Tuple[float, float]] = None
        self.forward_heading_rad: Optional[float] = None
        self.touchdown_started_s: Optional[float] = None

        self.pending_mode_name: Optional[str] = None
        self.mode_future = None
        self.pending_arm_value: Optional[bool] = None
        self.arm_future = None
        self.arm_request_sent = False
        self.disarm_request_sent = False

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.manual_pub = self.create_publisher(
            ManualControl, self.manual_control_topic, 10
        )

        state_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            state_qos,
        )
        self.create_subscription(
            PoseStamped, self.local_pose_topic, self.pose_callback, pose_qos
        )

        self.mode_client = self.create_client(SetMode, self.mode_service)
        self.arm_client = self.create_client(CommandBool, self.arm_service)

        self.create_service(
            Trigger, self.start_service_name, self.handle_start_request
        )
        self.create_service(
            Trigger, self.abort_service_name, self.handle_abort_request
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0), self.timer_callback
        )
        self.get_logger().info(
            "ALTCTL demo sequence node started. Waiting for explicit start request."
        )
        self.publish_status()

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def state_callback(self, msg: State) -> None:
        self.latest_state = msg
        self.last_state_time_s = self.now_s()

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.last_pose_time_s = self.now_s()

    def current_altitude_m(self) -> float:
        return float(self.latest_pose.pose.position.z)

    def current_xy(self) -> Tuple[float, float]:
        return (
            float(self.latest_pose.pose.position.x),
            float(self.latest_pose.pose.position.y),
        )

    def current_yaw_rad(self) -> float:
        q = self.latest_pose.pose.orientation
        return quaternion_to_yaw_rad(q.w, q.x, q.y, q.z)

    def pose_fresh(self) -> bool:
        return (self.now_s() - self.last_pose_time_s) <= self.pose_timeout_s

    def state_fresh(self) -> bool:
        return (self.now_s() - self.last_state_time_s) <= self.state_timeout_s

    def is_altctl_mode(self) -> bool:
        return str(self.latest_state.mode).upper() == "ALTCTL"

    def is_land_mode(self) -> bool:
        return "LAND" in str(self.latest_state.mode).upper()

    def startable(self) -> Tuple[bool, str]:
        if self.demo_state not in {"IDLE", "COMPLETE", "ABORT"}:
            return False, f"demo already active in state {self.demo_state}"
        if self.mode_future is not None or self.arm_future is not None:
            return False, "a prior mode or arm request is still in flight"
        if not self.mode_client.wait_for_service(timeout_sec=0.0):
            return False, "mode service is unavailable"
        if not self.arm_client.wait_for_service(timeout_sec=0.0):
            return False, "arming service is unavailable"
        if not self.state_fresh():
            return False, "MAVROS state is stale or missing"
        if not self.latest_state.connected:
            return False, "FCU is not connected"
        if not self.pose_fresh():
            return False, "local pose is stale or missing"
        if self.latest_state.armed:
            return False, "refusing to start because the vehicle is already armed"
        return True, "ready"

    def handle_start_request(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        ready, reason = self.startable()
        if not ready:
            response.success = False
            response.message = reason
            return response

        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.forward_start_xy = None
        self.forward_heading_rad = None
        self.touchdown_started_s = None
        self.transition_to("SET_ALTCTL", "start requested")
        response.success = True
        response.message = "demo sequence started"
        return response

    def handle_abort_request(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self.demo_state in {"IDLE", "COMPLETE"}:
            response.success = False
            response.message = "demo is not active"
            return response

        self.enter_abort("operator abort")
        response.success = True
        response.message = "abort accepted"
        return response

    def publish_status(self) -> None:
        msg = String()
        msg.data = self.demo_state
        self.status_pub.publish(msg)

    def publish_manual(
        self,
        x_cmd: float = 0.0,
        y_cmd: float = 0.0,
        throttle_cmd: Optional[float] = None,
        yaw_cmd: float = 0.0,
    ) -> None:
        throttle = self.hover_throttle_center if throttle_cmd is None else throttle_cmd

        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = float(clamp(x_cmd, -1000.0, 1000.0))
        msg.y = float(clamp(y_cmd, -1000.0, 1000.0))
        msg.z = float(clamp(throttle, 0.0, 1000.0))
        msg.r = float(clamp(yaw_cmd, -1000.0, 1000.0))
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def transition_to(self, new_state: str, reason: str = "") -> None:
        old_state = self.demo_state
        self.demo_state = new_state
        self.stage_started_s = self.now_s()

        if new_state == "FORWARD_TRANSLATE":
            self.forward_start_xy = self.current_xy()
            self.forward_heading_rad = self.current_yaw_rad()
        elif new_state != "WAIT_TOUCHDOWN":
            self.touchdown_started_s = None

        if reason:
            self.get_logger().info(f"State {old_state} -> {new_state}: {reason}")
        else:
            self.get_logger().info(f"State {old_state} -> {new_state}")
        self.publish_status()

    def enter_abort(self, reason: str) -> None:
        if self.demo_state == "ABORT":
            return

        self.get_logger().error(f"Aborting demo: {reason}")
        self.publish_manual(
            x_cmd=0.0,
            y_cmd=0.0,
            throttle_cmd=self.hover_throttle_center,
            yaw_cmd=0.0,
        )
        self.transition_to("ABORT", reason)

    def stage_elapsed_s(self) -> float:
        return self.now_s() - self.stage_started_s

    def stage_timeout_limit_s(self) -> Optional[float]:
        if self.demo_state in {"IDLE", "COMPLETE", "ABORT"}:
            return None
        if self.demo_state == "HOVER_AFTER_TAKEOFF":
            return max(self.stage_timeout_s, self.hover_after_takeoff_s + 2.0)
        if self.demo_state == "HOVER_AFTER_TRANSLATE":
            return max(self.stage_timeout_s, self.hover_after_translate_s + 2.0)
        if self.demo_state == "WAIT_TOUCHDOWN":
            return max(self.stage_timeout_s, 20.0)
        return self.stage_timeout_s

    def forward_progress_m(self) -> float:
        if self.forward_start_xy is None or self.forward_heading_rad is None:
            return 0.0
        return project_forward_distance(
            self.forward_start_xy,
            self.current_xy(),
            self.forward_heading_rad,
        )

    def altitude_hold_throttle(self, target_altitude_m: float) -> float:
        error = target_altitude_m - self.current_altitude_m()
        delta = clamp(
            error * self.altitude_kp,
            -self.takeoff_throttle_delta,
            self.takeoff_throttle_delta,
        )
        return clamp(self.hover_throttle_center + delta, 0.0, 1000.0)

    def request_mode(self, mode_name: str) -> None:
        if self.mode_future is not None:
            return
        if not self.mode_client.wait_for_service(timeout_sec=0.0):
            self.enter_abort(f"mode service unavailable for {mode_name}")
            return

        req = SetMode.Request()
        req.custom_mode = mode_name
        self.pending_mode_name = mode_name
        self.mode_future = self.mode_client.call_async(req)
        self.get_logger().info(f"Requested mode {mode_name}")

    def request_arm(self, arm_value: bool) -> None:
        if self.arm_future is not None:
            return
        if not self.arm_client.wait_for_service(timeout_sec=0.0):
            action = "arm" if arm_value else "disarm"
            self.enter_abort(f"{action} service unavailable")
            return

        req = CommandBool.Request()
        req.value = arm_value
        self.pending_arm_value = arm_value
        self.arm_future = self.arm_client.call_async(req)
        action = "arm" if arm_value else "disarm"
        self.get_logger().info(f"Requested {action}")

    def poll_service_futures(self) -> None:
        if self.mode_future is not None and self.mode_future.done():
            mode_name = self.pending_mode_name or "unknown"
            try:
                response = self.mode_future.result()
                if not response.mode_sent:
                    self.enter_abort(f"mode change to {mode_name} was rejected")
                else:
                    self.get_logger().info(f"Mode request accepted for {mode_name}")
            except Exception as exc:
                self.enter_abort(f"mode change to {mode_name} failed: {exc}")
            finally:
                self.mode_future = None
                self.pending_mode_name = None

        if self.arm_future is not None and self.arm_future.done():
            arm_value = bool(self.pending_arm_value)
            action = "arm" if arm_value else "disarm"
            try:
                response = self.arm_future.result()
                if not response.success:
                    self.enter_abort(f"{action} request was rejected")
                else:
                    self.get_logger().info(f"{action.capitalize()} request accepted")
            except Exception as exc:
                self.enter_abort(f"{action} request failed: {exc}")
            finally:
                self.arm_future = None
                self.pending_arm_value = None

    def run_safety_checks(self) -> None:
        if self.demo_state in {"IDLE", "COMPLETE"}:
            return

        if not self.state_fresh():
            self.enter_abort("lost MAVROS state updates")
            return
        if not self.latest_state.connected:
            self.enter_abort("FCU disconnected")
            return
        if not self.pose_fresh():
            self.enter_abort("lost local pose updates")
            return
        if (
            self.demo_state in self.ALTCTL_REQUIRED_STATES
            and not self.is_altctl_mode()
        ):
            self.enter_abort(
                f"mode mismatch during {self.demo_state}: "
                f"{self.latest_state.mode}"
            )
            return
        if self.demo_state in {"WAIT_TOUCHDOWN", "DISARMING"}:
            if self.latest_state.armed and not self.is_land_mode():
                self.enter_abort(
                    f"LAND mode dropped unexpectedly: {self.latest_state.mode}"
                )
                return

        timeout_limit = self.stage_timeout_limit_s()
        if timeout_limit is not None and self.stage_elapsed_s() > timeout_limit:
            self.enter_abort(f"stage timeout in {self.demo_state}")

    def step_state_machine(self) -> None:
        if self.demo_state == "IDLE":
            return

        if self.demo_state == "SET_ALTCTL":
            if self.is_altctl_mode():
                self.transition_to("ARMING", "ALTCTL confirmed")
            elif self.mode_future is None:
                self.request_mode("ALTCTL")
            return

        if self.demo_state == "ARMING":
            if self.latest_state.armed:
                self.transition_to("TAKEOFF", "vehicle armed")
                return
            if (
                self.stage_elapsed_s() >= self.arm_zero_throttle_hold_s
                and not self.arm_request_sent
            ):
                self.arm_request_sent = True
                self.request_arm(True)
            return

        if self.demo_state == "TAKEOFF":
            if self.current_altitude_m() >= (
                self.target_altitude_m - self.altitude_tolerance_m
            ):
                self.transition_to("HOVER_AFTER_TAKEOFF", "target altitude reached")
            return

        if self.demo_state == "HOVER_AFTER_TAKEOFF":
            if self.stage_elapsed_s() >= self.hover_after_takeoff_s:
                self.transition_to("FORWARD_TRANSLATE", "takeoff hover complete")
            return

        if self.demo_state == "FORWARD_TRANSLATE":
            if self.forward_progress_m() >= (
                self.forward_distance_m - self.position_tolerance_m
            ):
                self.transition_to(
                    "HOVER_AFTER_TRANSLATE",
                    "forward displacement reached",
                )
            return

        if self.demo_state == "HOVER_AFTER_TRANSLATE":
            if self.stage_elapsed_s() >= self.hover_after_translate_s:
                self.transition_to("SET_LAND", "translation hover complete")
            return

        if self.demo_state == "SET_LAND":
            if self.is_land_mode():
                self.transition_to("WAIT_TOUCHDOWN", "LAND mode confirmed")
            elif self.mode_future is None:
                self.request_mode("LAND")
            return

        if self.demo_state == "WAIT_TOUCHDOWN":
            if not self.latest_state.armed:
                self.transition_to("COMPLETE", "vehicle auto-disarmed after landing")
                return
            if self.current_altitude_m() <= self.land_detect_altitude_m:
                if self.touchdown_started_s is None:
                    self.touchdown_started_s = self.now_s()
                elif (
                    self.now_s() - self.touchdown_started_s
                ) >= self.touchdown_dwell_s:
                    self.transition_to("DISARMING", "touchdown detected")
            else:
                self.touchdown_started_s = None
            return

        if self.demo_state == "DISARMING":
            if not self.latest_state.armed:
                self.transition_to("COMPLETE", "vehicle disarmed")
                return
            if not self.disarm_request_sent:
                self.disarm_request_sent = True
                self.request_arm(False)
            return

        if self.demo_state == "ABORT":
            if self.latest_state.connected and self.latest_state.armed:
                if not self.is_land_mode() and self.mode_future is None:
                    self.request_mode("LAND")

    def publish_manual_for_state(self) -> None:
        if self.demo_state not in self.ACTIVE_MANUAL_STATES:
            return

        if self.demo_state in {"SET_ALTCTL", "ARMING"}:
            self.publish_manual(
                x_cmd=0.0,
                y_cmd=0.0,
                throttle_cmd=0.0,
                yaw_cmd=0.0,
            )
            return

        if self.demo_state in {
            "TAKEOFF",
            "HOVER_AFTER_TAKEOFF",
            "HOVER_AFTER_TRANSLATE",
        }:
            self.publish_manual(
                x_cmd=0.0,
                y_cmd=0.0,
                throttle_cmd=self.altitude_hold_throttle(self.target_altitude_m),
                yaw_cmd=0.0,
            )
            return

        if self.demo_state == "FORWARD_TRANSLATE":
            self.publish_manual(
                x_cmd=clamp(self.forward_stick_cmd, 0.0, 1000.0),
                y_cmd=0.0,
                throttle_cmd=self.altitude_hold_throttle(self.target_altitude_m),
                yaw_cmd=0.0,
            )

    def timer_callback(self) -> None:
        self.poll_service_futures()
        self.run_safety_checks()
        self.step_state_machine()
        self.publish_manual_for_state()
        self.publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AltctlDemoSequenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
