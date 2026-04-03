#!/usr/bin/env python3
"""
Keyboard teleop for cdrone_control via MAVROS.

Default backend publishes MAVLink MANUAL_CONTROL stick inputs to
/mavros/manual_control/send for ALTCTL/STABILIZED indoor teleop.
An optional velocity backend remains available for OFFBOARD workflows.
"""

import select
import sys
import termios
import time
import tty
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ManualControl
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from drone_control_pkg.topic_utils import cdrone_topic, join_topic


MOVE_BINDINGS: Dict[str, Tuple[str, float]] = {
    # key: (axis, value)
    'w': ('x', 1.0),    # Forward
    's': ('x', -1.0),   # Backward
    'a': ('y', 1.0),    # Left
    'd': ('y', -1.0),   # Right
    'r': ('z', 1.0),    # Up / more throttle
    'f': ('z', -1.0),   # Down / less throttle
    'q': ('yaw', 1.0),  # Yaw left
    'e': ('yaw', -1.0), # Yaw right
}

SPEED_BINDINGS = {
    't': 1.1,
    'y': 0.9,
}


MANUAL_INTERFACE = 'manual'
VELOCITY_INTERFACE = 'velocity'
POSITION_HOLD_MODE = 'POSCTL'


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.declare_parameter('command_interface', MANUAL_INTERFACE)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('manual_xy', 300.0)
        self.declare_parameter('manual_yaw', 250.0)
        self.declare_parameter('throttle_center', 500.0)
        self.declare_parameter('throttle_step', 200.0)
        self.declare_parameter('arm_throttle_hold_sec', 1.5)
        self.declare_parameter('disarm_request_delay_sec', 1.0)
        self.declare_parameter('drone_id', 'drone01')
        self.declare_parameter('mavros_namespace', '/mavros')
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('estop_topic', '')
        self.declare_parameter('manual_control_topic', '')
        self.declare_parameter('local_pose_timeout_s', 0.5)

        self.command_interface = str(
            self.get_parameter('command_interface').value
        ).strip().lower()
        if self.command_interface not in {MANUAL_INTERFACE, VELOCITY_INTERFACE}:
            self.get_logger().warn(
                f'Unknown command_interface={self.command_interface!r}; '
                f'falling back to {MANUAL_INTERFACE}. '
            )
            self.command_interface = MANUAL_INTERFACE

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.manual_xy = float(self.get_parameter('manual_xy').value)
        self.manual_yaw = float(self.get_parameter('manual_yaw').value)
        self.throttle_center = float(self.get_parameter('throttle_center').value)
        self.throttle_step = float(self.get_parameter('throttle_step').value)
        self.arm_throttle_hold_sec = float(
            self.get_parameter('arm_throttle_hold_sec').value
        )
        self.disarm_request_delay_sec = float(
            self.get_parameter('disarm_request_delay_sec').value
        )
        self.local_pose_timeout_s = float(
            self.get_parameter('local_pose_timeout_s').value
        )
        self.drone_id = str(self.get_parameter('drone_id').value)
        self.mavros_namespace = str(self.get_parameter('mavros_namespace').value)
        self.cmd_vel_topic = (
            str(self.get_parameter('cmd_vel_topic').value).strip()
            or cdrone_topic(self.drone_id, 'control/cmd_vel_body')
        )
        self.estop_topic = (
            str(self.get_parameter('estop_topic').value).strip()
            or cdrone_topic(self.drone_id, 'safety/estop')
        )
        self.manual_control_topic = (
            str(self.get_parameter('manual_control_topic').value).strip()
            or join_topic(self.mavros_namespace, 'manual_control/send')
        )
        self.arm_service = join_topic(self.mavros_namespace, 'cmd/arming')
        self.mode_service = join_topic(self.mavros_namespace, 'set_mode')
        self.command_service = join_topic(self.mavros_namespace, 'cmd/command')
        self.state_topic = join_topic(self.mavros_namespace, 'state')
        self.local_pose_topic = join_topic(self.mavros_namespace, 'local_position/pose')

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10
        )
        self.manual_pub = self.create_publisher(
            ManualControl,
            self.manual_control_topic,
            10,
        )
        self.estop_pub = self.create_publisher(
            Bool,
            self.estop_topic,
            10
        )

        self.arm_client = self.create_client(CommandBool, self.arm_service)
        self.mode_client = self.create_client(SetMode, self.mode_service)
        self.cmd_client = self.create_client(CommandLong, self.command_service)
        self.state_sub = self.create_subscription(State, self.state_topic, self.state_callback, 10)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            self.local_pose_topic,
            self.local_pose_callback,
            best_effort_qos,
        )

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.force_zero_throttle_until_s = 0.0
        self.pending_disarm_timer = None
        self.latest_state = State()
        self.last_local_pose_s = 0.0

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0), self.publish_current_command
        )

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            f'Keyboard teleop started with {self.command_interface} backend'
        )
        self.print_usage()

    def print_usage(self):
        if self.command_interface == MANUAL_INTERFACE:
            msg = """
========================================
Keyboard Teleop for cdrone_control
========================================

Backend:
  MANUAL_CONTROL -> /mavros/manual_control/send
  Intended modes: ALTCTL (preferred), STABILIZED, POSCTL

Movement:
  W/S : Forward/Backward stick
  A/D : Left/Right stick
  R/F : Increase/Decrease throttle around hover
  Q/E : Yaw Left/Right

Speed:
  T : Increase stick scale by 10%
  Y : Decrease stick scale by 10%

Commands:
  SPACE : Center sticks / stop
  0 : FORCE ARM (bench only, throttle forced low)
  1 : Arm drone (throttle forced low)
  2 : Set ALTCTL mode
  3 : Set STABILIZED mode
  4 : Disarm
  5 : Disable RC/joystick override (COM_RC_IN_MODE=1)
  6 : Set POSCTL mode (warn if local pose is stale)

Exit:
  ESC or Ctrl+C : Quit

Current scales:
  XY: {:.0f}
  Yaw: {:.0f}
  Throttle center: {:.0f}
  Throttle step: {:.0f}
  Disarm delay: {:.1f}s
========================================
""".format(
                self.manual_xy,
                self.manual_yaw,
                self.throttle_center,
                self.throttle_step,
                self.disarm_request_delay_sec,
            )
        else:
            msg = """
========================================
Keyboard Teleop for cdrone_control
========================================

Backend:
  Velocity -> /cdrone/control/cmd_vel_body
  Intended mode: OFFBOARD

Movement:
  W/S : Forward/Backward
  A/D : Left/Right
  R/F : Up/Down
  Q/E : Yaw Left/Right

Speed:
  T : Increase speed by 10%
  Y : Decrease speed by 10%

Commands:
  SPACE : Stop (zero velocity)
  0 : FORCE ARM (bench only)
  1 : Arm drone (normal)
  2 : Set ALTCTL mode
  3 : Set STABILIZED mode
  4 : Disarm
  5 : Disable RC/joystick override (COM_RC_IN_MODE=1)
  6 : Set OFFBOARD mode

Exit:
  ESC or Ctrl+C : Quit

Current speeds:
  Linear: {:.2f} m/s
  Angular: {:.2f} rad/s
========================================
""".format(self.linear_speed, self.angular_speed)
        print(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def now_s(self) -> float:
        return time.monotonic()

    def state_callback(self, msg: State) -> None:
        self.latest_state = msg

    def local_pose_callback(self, _msg: PoseStamped) -> None:
        self.last_local_pose_s = self.now_s()

    def publish_current_command(self):
        if self.command_interface == MANUAL_INTERFACE:
            self.publish_manual_control()
        else:
            self.publish_velocity()

    def publish_velocity(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.x * self.linear_speed
        msg.twist.linear.y = self.y * self.linear_speed
        msg.twist.linear.z = self.z * self.linear_speed
        msg.twist.angular.z = self.yaw * self.angular_speed
        self.velocity_pub.publish(msg)

    def publish_manual_control(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = float(self.x * self.manual_xy)
        msg.y = float(-self.y * self.manual_xy)
        msg.z = self.current_manual_throttle()
        msg.r = float(-self.yaw * self.manual_yaw)
        msg.buttons = 0
        self.manual_pub.publish(msg)

    def current_manual_throttle(self) -> float:
        if time.monotonic() < self.force_zero_throttle_until_s:
            return 0.0

        return float(
            max(0.0, min(1000.0, self.throttle_center + self.z * self.throttle_step))
        )

    def hold_zero_throttle_for_arming(self, action: str):
        if self.command_interface != MANUAL_INTERFACE:
            return

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.force_zero_throttle_until_s = max(
            self.force_zero_throttle_until_s,
            time.monotonic() + self.arm_throttle_hold_sec,
        )
        self.publish_current_command()
        self.get_logger().info(
            f'Holding manual throttle at zero for {action} '
            f'({self.arm_throttle_hold_sec:.1f}s)'
        )

    def stop_drone(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.publish_current_command()
        if self.command_interface == MANUAL_INTERFACE:
            self.get_logger().info('STICKS CENTERED')
        else:
            self.get_logger().info('STOPPED')

    def apply_axis_command(self, key: str):
        axis, value = MOVE_BINDINGS[key]
        setattr(self, axis, value)
        self.publish_current_command()
        self.log_current_command()

    def adjust_speed(self, scale: float):
        if self.command_interface == MANUAL_INTERFACE:
            self.manual_xy *= scale
            self.manual_yaw *= scale
            self.throttle_step *= scale
            self.get_logger().info(
                f'Scales: xy={self.manual_xy:.0f}, yaw={self.manual_yaw:.0f}, '
                f'throttle_step={self.throttle_step:.0f}'
            )
        else:
            self.linear_speed *= scale
            self.angular_speed *= scale
            self.get_logger().info(
                f'Speed: linear={self.linear_speed:.2f} m/s, '
                f'angular={self.angular_speed:.2f} rad/s'
            )

    def log_current_command(self):
        if self.command_interface == MANUAL_INTERFACE:
            self.get_logger().info(
                f'Manual: x={self.x * self.manual_xy:.0f}, '
                f'y={-self.y * self.manual_xy:.0f}, '
                f'z={self.current_manual_throttle():.0f}, '
                f'r={-self.yaw * self.manual_yaw:.0f}'
            )
        else:
            self.get_logger().info(
                f'Velocity: x={self.x*self.linear_speed:.2f}, '
                f'y={self.y*self.linear_speed:.2f}, '
                f'z={self.z*self.linear_speed:.2f}, '
                f'yaw={self.yaw*self.angular_speed:.2f}'
            )

    def arm_drone(self):
        self.hold_zero_throttle_for_arming('arm request')
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Arm service not available')
            return
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        future.add_done_callback(self._arm_callback)

    def force_arm_drone(self):
        self.hold_zero_throttle_for_arming('force-arm request')
        if not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Command service not available')
            return
        req = CommandLong.Request()
        req.command = 400
        req.param1 = 1.0
        req.param2 = 21196.0
        future = self.cmd_client.call_async(req)
        future.add_done_callback(self._force_arm_callback)

    def _force_arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('FORCE ARM successful')
            else:
                self.get_logger().error(f'Force arm failed (result={response.result})')
        except Exception as e:
            self.get_logger().error(f'Force arm service call failed: {e}')

    def disable_rc_override(self):
        if not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Command service not available')
            return
        req = CommandLong.Request()
        req.command = 23
        self.cmd_client.call_async(req)
        self.get_logger().info(
            'To disable QGC joystick override, run in another terminal:\n'
            '  ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "'
            '{param_id: COM_RC_IN_MODE, value: {integer: 1}}"\n'
            'Or in QGC: Application Settings -> Virtual Joystick -> disable'
        )

    def disarm_drone(self):
        self.hold_zero_throttle_for_arming('disarm request')
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Arm service not available')
            return
        if self.command_interface == MANUAL_INTERFACE:
            self.schedule_delayed_disarm()
            return

        self.send_disarm_request()

    def schedule_delayed_disarm(self):
        if self.pending_disarm_timer is not None:
            self.pending_disarm_timer.cancel()
            self.destroy_timer(self.pending_disarm_timer)
            self.pending_disarm_timer = None

        self.get_logger().info(
            f'Waiting {self.disarm_request_delay_sec:.1f}s before disarm '
            'so PX4 can re-enter landed state'
        )
        self.pending_disarm_timer = self.create_timer(
            self.disarm_request_delay_sec,
            self._delayed_disarm_callback,
        )

    def _delayed_disarm_callback(self):
        if self.pending_disarm_timer is not None:
            self.pending_disarm_timer.cancel()
            self.destroy_timer(self.pending_disarm_timer)
            self.pending_disarm_timer = None

        self.send_disarm_request()

    def send_disarm_request(self):
        req = CommandBool.Request()
        req.value = False
        future = self.arm_client.call_async(req)
        future.add_done_callback(self._arm_callback)

    def _arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arm/Disarm command successful')
            else:
                self.get_logger().error('Arm/Disarm command failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def set_mode(self, mode_name: str):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Mode service not available')
            return
        req = SetMode.Request()
        req.custom_mode = mode_name
        future = self.mode_client.call_async(req)
        future.add_done_callback(lambda f: self._mode_callback(f, mode_name))

    def _mode_callback(self, future, mode_name):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Mode changed to {mode_name}')
            else:
                self.get_logger().error(f'Failed to change mode to {mode_name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def warn_if_posctl_pose_unready(self):
        if not self.latest_state.connected:
            self.get_logger().warn(
                'Requesting POSCTL while MAVROS reports the FCU is disconnected.'
            )
            return

        if self.last_local_pose_s <= 0.0:
            self.get_logger().warn(
                'Requesting POSCTL before any local_position/pose message has arrived.'
            )
            return

        pose_age_s = self.now_s() - self.last_local_pose_s
        if pose_age_s > self.local_pose_timeout_s:
            self.get_logger().warn(
                f'Requesting POSCTL with stale local_position/pose age='
                f'{pose_age_s:.2f}s'
            )

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                rclpy.spin_once(self, timeout_sec=0)

                if not key:
                    continue

                if key == '' or key == '':
                    break

                if key in MOVE_BINDINGS:
                    self.apply_axis_command(key)
                elif key in SPEED_BINDINGS:
                    self.adjust_speed(SPEED_BINDINGS[key])
                elif key == ' ':
                    self.stop_drone()
                elif key == '0':
                    self.get_logger().warn(
                        'FORCE ARMING (bypasses preflight checks - bench use only!)'
                    )
                    self.force_arm_drone()
                elif key == '1':
                    self.get_logger().info('Arming...')
                    self.arm_drone()
                elif key == '2':
                    self.get_logger().info('Setting ALTCTL mode...')
                    self.set_mode('ALTCTL')
                elif key == '3':
                    self.get_logger().info('Setting STABILIZED mode...')
                    self.set_mode('STABILIZED')
                elif key == '4':
                    self.get_logger().info('Disarming...')
                    self.disarm_drone()
                elif key == '5':
                    self.disable_rc_override()
                elif key == '6':
                    if self.command_interface == VELOCITY_INTERFACE:
                        self.get_logger().info('Setting OFFBOARD mode...')
                        self.set_mode('OFFBOARD')
                    else:
                        self.warn_if_posctl_pose_unready()
                        self.get_logger().info('Setting POSCTL mode...')
                        self.set_mode(POSITION_HOLD_MODE)
                else:
                    self.get_logger().warn(f'Unknown key: {repr(key)}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.stop_drone()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
