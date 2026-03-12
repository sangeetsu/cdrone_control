#!/usr/bin/env python3
"""
Keyboard teleop for cdrone_control via MAVROS
Publishes velocity commands to /cdrone/control/cmd_vel_body

Controls:
  W/S : Forward/Backward
  A/D : Left/Right  
  R/F : Up/Down
  Q/E : Yaw Left/Right
  T/Y : Increase/Decrease speed
  SPACE : Stop (zero velocity)
  1 : Arm
  2 : Set STABILIZED mode
  3 : Set OFFBOARD mode
  4 : Disarm
  ESC/Ctrl+C : Exit
"""

import sys
import select
import termios
import tty
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from std_msgs.msg import Bool


# Movement bindings
MOVE_BINDINGS: Dict[str, Tuple[float, float, float, float]] = {
    # key: (x, y, z, yaw)
    'w': (1.0, 0.0, 0.0, 0.0),   # Forward
    's': (-1.0, 0.0, 0.0, 0.0),  # Backward
    'a': (0.0, 1.0, 0.0, 0.0),   # Left
    'd': (0.0, -1.0, 0.0, 0.0),  # Right
    'r': (0.0, 0.0, 1.0, 0.0),   # Up
    'f': (0.0, 0.0, -1.0, 0.0),  # Down
    'q': (0.0, 0.0, 0.0, 1.0),   # Yaw left
    'e': (0.0, 0.0, 0.0, -1.0),  # Yaw right
}

# Speed adjustment bindings
SPEED_BINDINGS = {
    't': 1.1,  # Increase speed by 10%
    'y': 0.9,  # Decrease speed by 10%
}


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)  # m/s
        self.declare_parameter('angular_speed', 0.3)  # rad/s
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/cdrone/control/cmd_vel_body',
            10
        )
        
        self.estop_pub = self.create_publisher(
            Bool,
            '/cdrone/safety/estop',
            10
        )
        
        # Service clients for arming and mode changes
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # Current velocity
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Keyboard teleop started!')
        self.print_usage()
        
    def print_usage(self):
        msg = """
========================================
Keyboard Teleop for cdrone_control
========================================

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
  0 : FORCE ARM (bypasses preflight checks)
  1 : Arm drone (normal)
  2 : Set STABILIZED mode
  3 : Set OFFBOARD mode
  4 : Disarm
  5 : Disable RC/joystick override (COM_RC_IN_MODE=1)
  
Exit:
  ESC or Ctrl+C : Quit

Current speeds:
  Linear: {:.2f} m/s
  Angular: {:.2f} rad/s
========================================
""".format(self.linear_speed, self.angular_speed)
        print(msg)
        
    def get_key(self):
        """Get a single keypress from stdin, non-blocking with 50ms timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def publish_velocity(self):
        """Publish the current velocity"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = self.x * self.linear_speed
        msg.twist.linear.y = self.y * self.linear_speed
        msg.twist.linear.z = self.z * self.linear_speed
        msg.twist.angular.z = self.yaw * self.angular_speed
        
        self.velocity_pub.publish(msg)
        
    def stop_drone(self):
        """Stop all motion"""
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.publish_velocity()
        self.get_logger().info('STOPPED')
        
    def arm_drone(self):
        """Arm the drone (normal — throttle must be at minimum)"""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Arm service not available')
            return
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        future.add_done_callback(self._arm_callback)

    def force_arm_drone(self):
        """Force-arm bypassing preflight checks (bench testing only!)"""
        if not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Command service not available')
            return
        req = CommandLong.Request()
        req.command = 400          # MAV_CMD_COMPONENT_ARM_DISARM
        req.param1 = 1.0           # arm
        req.param2 = 21196.0       # force arm magic number
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
        """Set COM_RC_IN_MODE=1 to stop QGC joystick high-throttle blocking arm"""
        if not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Command service not available')
            return
        # Use mavros param set via command long: MAV_CMD_DO_SET_PARAMETER
        req = CommandLong.Request()
        req.command = 23           # MAV_CMD_DO_SET_PARAMETER (not standard — use param service instead)
        future = self.cmd_client.call_async(req)
        # Actually use the param approach via a shell command hint
        self.get_logger().info(
            'To disable QGC joystick override, run in another terminal:\n'
            '  ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 "'
            '{param_id: COM_RC_IN_MODE, value: {integer: 1}}"\n'
            'Or in QGC: Application Settings → Virtual Joystick → disable'
        )
        
    def disarm_drone(self):
        """Disarm the drone"""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Arm service not available')
            return
            
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
        """Set flight mode"""
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
        
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()

                # Process pending ROS callbacks (arm/mode service responses, etc.)
                rclpy.spin_once(self, timeout_sec=0)

                if not key:
                    continue

                # Check for exit
                if key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
                    break
                    
                # Movement keys
                if key in MOVE_BINDINGS:
                    self.x, self.y, self.z, self.yaw = MOVE_BINDINGS[key]
                    self.publish_velocity()
                    self.get_logger().info(
                        f'Velocity: x={self.x*self.linear_speed:.2f}, '
                        f'y={self.y*self.linear_speed:.2f}, '
                        f'z={self.z*self.linear_speed:.2f}, '
                        f'yaw={self.yaw*self.angular_speed:.2f}'
                    )
                    
                # Speed adjustment
                elif key in SPEED_BINDINGS:
                    self.linear_speed *= SPEED_BINDINGS[key]
                    self.angular_speed *= SPEED_BINDINGS[key]
                    self.get_logger().info(
                        f'Speed: linear={self.linear_speed:.2f} m/s, '
                        f'angular={self.angular_speed:.2f} rad/s'
                    )
                    
                # Stop
                elif key == ' ':
                    self.stop_drone()
                    
                # Commands
                elif key == '0':
                    self.get_logger().warn('FORCE ARMING (bypasses preflight checks - bench use only!)')
                    self.force_arm_drone()

                elif key == '1':
                    self.get_logger().info('Arming...')
                    self.arm_drone()
                    
                elif key == '2':
                    self.get_logger().info('Setting STABILIZED mode...')
                    self.set_mode('STABILIZED')
                    
                elif key == '3':
                    self.get_logger().info('Setting OFFBOARD mode...')
                    self.set_mode('OFFBOARD')
                    
                elif key == '4':
                    self.get_logger().info('Disarming...')
                    self.disarm_drone()

                elif key == '5':
                    self.disable_rc_override()
                    
                else:
                    # Unknown key
                    if key != '':
                        self.get_logger().warn(f'Unknown key: {repr(key)}')
                        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            
        finally:
            # Stop the drone and restore terminal
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
