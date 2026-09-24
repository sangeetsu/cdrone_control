"""Short range-controlled climb; no horizontal position feedback or motor control.

PX4 keeps its attitude/rate controllers. This loop requests level attitude and
normalized collective thrust until position aiding is ready for handover.
"""

import math


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


class RangeClimb:
    def __init__(self, ground_distance, yaw, range_time):
        self.ground_distance = ground_distance
        self.yaw = yaw
        self.range_time = range_time
        self.last_distance = self.distance = ground_distance
        self.height = self.raw_height = self.speed = 0.0
        self.thrust = 0.0
        self.integral = 0.0
        self.aiding_since = None

    def update(self, distance, timestamp):
        """Differentiate new range samples only, with a 0.15 s low-pass filter."""
        dt = timestamp - self.range_time
        if dt <= 0:
            return
        alpha = dt / (0.15 + dt)
        self.speed += alpha * ((distance - self.last_distance) / dt - self.speed)
        self.distance += alpha * (distance - self.distance)
        self.last_distance, self.range_time = distance, timestamp
        self.raw_height = distance - self.ground_distance
        self.height = self.distance - self.ground_distance

    def command(self, settings, dt):
        """Bounded position -> velocity -> thrust loop, with anti-windup.

        Hover thrust must be supplied for the airframe. Gains use SI units;
        acceleration is mapped to normalized thrust through hover_thrust / g.
        The integral is frozen during spool-up and when thrust is saturated.
        """
        hover = settings.initial_climb_hover_thrust
        speed_sp = clamp(2.0 * (settings.initial_climb_height_m - self.height),
                         -settings.initial_climb_rate_m_s, settings.initial_climb_rate_m_s)
        error = speed_sp - self.speed
        acceleration = 3.0 * error + self.integral
        desired = hover * (1.0 + acceleration / 9.80665)
        if (self.thrust >= 0.9 * hover
                and 0.1 < desired < settings.initial_climb_max_thrust):
            self.integral = clamp(self.integral + error * dt, -1.0, 1.0)
        desired = clamp(desired, 0.1, settings.initial_climb_max_thrust)
        # Limit increases only: a reduction must not be delayed by a climb ramp.
        self.thrust = min(desired, self.thrust + settings.initial_climb_thrust_slew * dt)
        return self.yaw, self.thrust


def vertical_distance(telemetry):
    if telemetry.attitude is None or telemetry.ground_distance is None:
        return None
    if not all(math.isfinite(v) for v in (*telemetry.attitude, telemetry.ground_distance)):
        return None
    roll, pitch, _ = telemetry.attitude
    value = telemetry.ground_distance * math.cos(roll) * math.cos(pitch)
    return value if math.isfinite(value) else None
