from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_msgs.msg import LightCommand
from drone_light_pkg.light_utils import duty_from_intensity

try:
    import Jetson.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover - hardware specific import
    GPIO = None

@dataclass
class LightState:
    enabled: bool = False
    intensity: float = 0.0
    strobe_hz: float = 0.0


class LightControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("light_controller_node")

        self.declare_parameter("light_gpio_pin", 33)
        self.declare_parameter("light_pwm_hz", 200)
        self.declare_parameter("light_default_intensity", 0.9)

        self.pin = int(self.get_parameter("light_gpio_pin").value)
        self.pwm_hz = int(self.get_parameter("light_pwm_hz").value)
        self.default_intensity = float(self.get_parameter("light_default_intensity").value)

        self.estop = False
        self.state = LightState(enabled=False, intensity=self.default_intensity, strobe_hz=0.0)
        self.last_duty = -1.0
        self.pwm: Optional[object] = None

        self._init_hardware()

        self.light_sub = self.create_subscription(
            LightCommand, "/cdrone/light/cmd", self.light_cmd_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, "/cdrone/safety/estop", self.estop_callback, 10
        )
        self.timer = self.create_timer(0.02, self.apply_output)
        self.get_logger().info("Light controller started.")

    def _init_hardware(self) -> None:
        if GPIO is None:
            self.get_logger().warn("Jetson.GPIO unavailable, running in dry-run mode.")
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.pwm_hz)
        self.pwm.start(0.0)
        self.get_logger().info(
            f"Jetson GPIO configured on BOARD pin {self.pin} at {self.pwm_hz} Hz PWM."
        )

    def estop_callback(self, msg: Bool) -> None:
        self.estop = bool(msg.data)
        if self.estop:
            self.state.enabled = False
            self.state.strobe_hz = 0.0

    def light_cmd_callback(self, msg: LightCommand) -> None:
        self.state.enabled = bool(msg.enabled)
        self.state.intensity = max(0.0, min(1.0, float(msg.intensity_0_to_1)))
        self.state.strobe_hz = max(0.0, float(msg.strobe_hz))

    def apply_output(self) -> None:
        if self.estop or not self.state.enabled:
            duty = 0.0
        elif self.state.strobe_hz > 0.0:
            period = 1.0 / self.state.strobe_hz
            phase = (self.get_clock().now().nanoseconds / 1e9) % period
            duty = duty_from_intensity(self.state.intensity) if phase < (period / 2.0) else 0.0
        else:
            duty = duty_from_intensity(self.state.intensity)

        if abs(duty - self.last_duty) < 1e-3:
            return
        self.last_duty = duty

        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(duty)
        else:
            self.get_logger().debug(f"[dry-run] light duty set to {duty:.1f}%")

    def destroy_node(self) -> bool:
        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(0.0)
            self.pwm.stop()
        if GPIO is not None:
            GPIO.cleanup(self.pin)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LightControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
