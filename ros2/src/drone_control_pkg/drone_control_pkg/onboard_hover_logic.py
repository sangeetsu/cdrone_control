"""Local hover state machine; coordinates are MAVROS ENU, never studio coordinates."""

from dataclasses import dataclass, field
import math

from drone_control_pkg.onboard_initial_climb import RangeClimb, vertical_distance


@dataclass(frozen=True)
class HoverSettings:
    localization_source: str = "ark_flow"
    mission_mode: str = "hover"
    enable_flight: bool = False
    publish_rate_hz: float = 20.0
    takeoff_altitude_m: float = 1.0
    takeoff_rate_m_s: float = 0.3
    # The ZED path needs enough position error to overcome PX4's takeoff ramp.
    # Keep the requested height ahead of the measured height by at most 0.6 m.
    zed_takeoff_setpoint_lead_m: float = 0.6
    zed_takeoff_no_lift_timeout_s: float = 8.0
    hover_duration_s: float = 20.0
    altitude_tolerance_m: float = 0.15
    altitude_dwell_s: float = 1.0
    readiness_dwell_s: float = 2.0
    # PX4's predicted relative-position bit can briefly clear while its current
    # position and velocity remain valid. Only this predictive bit gets a grace.
    vision_prediction_loss_dwell_s: float = 1.0
    offboard_warmup_s: float = 1.5
    sensor_timeout_s: float = 0.5
    state_timeout_s: float = 2.5
    max_horizontal_excursion_m: float = 0.5
    square_edge_m: float = 1.0
    square_speed_m_s: float = 0.2
    square_setpoint_lead_m: float = 0.3
    square_waypoint_tolerance_m: float = 0.15
    square_waypoint_dwell_s: float = 1.0
    square_waypoint_timeout_s: float = 20.0
    square_max_excursion_m: float = 1.8
    square_max_path_error_m: float = 0.35
    max_pose_jump_m: float = 0.5
    max_yaw_jump_rad: float = 0.52
    altitude_overshoot_m: float = 0.3
    # Independent range checks for position-controlled takeoff over a level floor.
    max_altitude_range_error_m: float = 0.35
    altitude_range_error_dwell_s: float = 0.5
    takeoff_no_lift_timeout_s: float = 5.0
    stage_timeout_s: float = 30.0
    landing_timeout_s: float = 45.0
    command_timeout_s: float = 2.0
    command_retry_s: float = 1.0
    max_command_attempts: int = 3
    flow_min_quality: int = 1
    # Operator-requested ground-distance acceptance; PX4 fusion is checked separately.
    range_min_m: float = 0.05
    range_max_m: float = 30.0
    initial_climb_enabled: bool = False
    # Required explicitly when opting in; never guess an airframe's hover thrust.
    initial_climb_hover_thrust: float = 0.0
    initial_climb_max_thrust: float = 0.75
    initial_climb_thrust_slew: float = 0.25
    initial_climb_height_m: float = 0.25
    initial_climb_max_height_m: float = 1.25
    initial_climb_rate_m_s: float = 0.2
    initial_climb_min_range_m: float = 0.15
    initial_climb_aiding_dwell_s: float = 1.0
    initial_climb_timeout_s: float = 8.0
    initial_climb_no_lift_timeout_s: float = 5.0
    initial_climb_max_tilt_rad: float = 0.2
    initial_climb_max_speed_m_s: float = 1.0

    def __post_init__(self):
        if self.localization_source not in {"ark_flow", "zed"}:
            raise ValueError("localization_source must be ark_flow or zed")
        if self.mission_mode not in {"hover", "square"}:
            raise ValueError("mission_mode must be hover or square")
        if self.mission_mode == "square" and self.localization_source != "zed":
            raise ValueError("square mode requires ZED localization")
        if self.localization_source == "zed" and self.initial_climb_enabled:
            raise ValueError("ZED hover starts with position control; initial climb is unavailable")
        for name, value in vars(self).items():
            if name in {"enable_flight", "initial_climb_enabled", "localization_source", "mission_mode"}:
                continue
            if name == "initial_climb_hover_thrust" and value == 0:
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.publish_rate_hz < 10 or self.offboard_warmup_s < 1.0:
            raise ValueError("OFFBOARD requires >=10 Hz output and >=1 s warmup")
        if self.range_min_m >= self.range_max_m or self.flow_min_quality > 255:
            raise ValueError("invalid range limits or flow quality threshold")
        if self.altitude_tolerance_m >= self.takeoff_altitude_m:
            raise ValueError("altitude tolerance must be smaller than the climb")
        if (self.mission_mode == "square" and
                math.sqrt(2) * self.square_edge_m + self.square_waypoint_tolerance_m
                >= self.square_max_excursion_m):
            raise ValueError("square corners and tolerance must fit inside excursion limit")
        if not 0.3 <= self.zed_takeoff_setpoint_lead_m <= 0.6:
            raise ValueError("ZED takeoff setpoint lead must be between 0.3 and 0.6 m")
        if not 6.0 <= self.zed_takeoff_no_lift_timeout_s <= 10.0:
            raise ValueError("ZED no-liftoff timeout must be between 6 and 10 s")
        if not 0 <= self.initial_climb_hover_thrust < self.initial_climb_max_thrust <= 1:
            raise ValueError("initial climb needs 0 <= hover thrust < maximum thrust <= 1")
        if self.initial_climb_enabled:
            if not 0.15 <= self.initial_climb_height_m <= 0.4:
                raise ValueError("initial climb height must be between 0.15 and 0.4 m")
            if self.initial_climb_max_height_m <= self.initial_climb_height_m:
                raise ValueError("initial climb maximum height must exceed the target height")
            if self.initial_climb_height_m >= self.takeoff_altitude_m:
                raise ValueError("initial climb height must be below final takeoff altitude")
            if not 0.1 <= self.initial_climb_min_range_m < self.initial_climb_height_m:
                raise ValueError("initial climb handover range must be >=0.1 m and below climb height")
            if not (self.initial_climb_no_lift_timeout_s < self.initial_climb_timeout_s <= 10):
                raise ValueError("initial climb needs no-lift timeout < total timeout <= 10 s")
            if self.initial_climb_max_tilt_rad > 0.35 or self.initial_climb_rate_m_s > 0.3:
                raise ValueError("initial climb tilt/rate exceed supported limits")


@dataclass
class Telemetry:
    connected: bool = False
    armed: bool = False
    mode: str = ""
    landed: int = 0
    # x, y, z, yaw; all in the same local frame for a flight session.
    pose: tuple[float, float, float, float] | None = None
    frame: str = ""
    estimator_ok: bool = False
    estimator_reason: str = "missing"
    relative_position_predicted: bool = False
    relative_position_prediction_lost_since: float | None = None
    vertical_estimator_ok: bool = False
    vertical_estimator_reason: str = "missing"
    # Independent IMU orientation (ENU/FLU) and raw downward range.
    attitude: tuple[float, float, float] | None = None
    ground_distance: float | None = None
    position_target: tuple[float, float, float, float] | None = None
    vision_pose: tuple[float, float, float, float] | None = None
    vision_frame: str = ""
    vision_profile_ok: bool = False
    vision_profile_reason: str = "PX4 vision parameters not verified"
    flow_ok: bool = False
    flow_reason: str = "invalid integration or insufficient quality"
    range_ok: bool = False
    range_reason: str = "invalid or outside sensor limits"
    received: dict[str, float] = field(default_factory=dict)

    def fresh(self, name, now, timeout):
        timestamp = self.received.get(name)
        return timestamp is not None and 0 <= now - timestamp <= timeout

    def update_relative_position_prediction(self, available, now):
        if available:
            self.relative_position_prediction_lost_since = None
        elif self.relative_position_prediction_lost_since is None:
            self.relative_position_prediction_lost_since = now
        self.relative_position_predicted = available

    def blockers(self, now, settings, *, preflight=False, initial_climb=False):
        reasons = []
        inputs = ["state", "extended", "estimator", "pose"]
        inputs += ["vision"] if settings.localization_source == "zed" else ["flow", "range"]
        for name in inputs:
            timeout = (settings.state_timeout_s if name in
                       {"state", "extended", "estimator"} else settings.sensor_timeout_s)
            if not self.fresh(name, now, timeout):
                reasons.append(f"{name}: stale or missing")
        if not self.connected:
            reasons.append("FCU disconnected")
        if self.pose is None or not all(math.isfinite(v) for v in self.pose) or not self.frame:
            reasons.append("local pose: invalid coordinates/orientation/frame")
        if settings.localization_source == "zed":
            if not self.vision_profile_ok:
                reasons.append(self.vision_profile_reason)
            if (self.vision_pose is None or not all(math.isfinite(v) for v in self.vision_pose)
                    or not self.vision_frame):
                reasons.append("ZED pose: invalid coordinates/orientation/frame")
        if initial_climb:
            if not self.vertical_estimator_ok:
                reasons.append(f"estimator: {self.vertical_estimator_reason}")
            if not self.fresh("imu", now, settings.sensor_timeout_s):
                reasons.append("imu: stale or missing")
            if self.attitude is None or not all(math.isfinite(v) for v in self.attitude):
                reasons.append("imu: invalid attitude")
            elif math.acos(max(-1.0, min(1.0, math.cos(self.attitude[0]) *
                                       math.cos(self.attitude[1])))) > settings.initial_climb_max_tilt_rad:
                reasons.append("initial climb tilt limit exceeded")
            if vertical_distance(self) is None:
                reasons.append("initial climb: missing measured ground distance")
        elif not self.estimator_ok:
            reasons.append(f"estimator: {self.estimator_reason}")
        if settings.localization_source == "zed" and not self.relative_position_predicted:
            lost_since = self.relative_position_prediction_lost_since
            if (preflight or lost_since is None
                    or now - lost_since >= settings.vision_prediction_loss_dwell_s):
                reasons.append("estimator: relative-position prediction unavailable")
        if settings.localization_source != "zed" and not self.flow_ok and self.fresh("flow", now, settings.sensor_timeout_s):
            reasons.append(f"flow: {self.flow_reason}")
        if settings.localization_source != "zed" and not self.range_ok and self.fresh("range", now, settings.sensor_timeout_s):
            reasons.append(f"range: {self.range_reason}")
        if settings.localization_source != "zed" and self.range_ok and (self.ground_distance is None
                              or not math.isfinite(self.ground_distance)):
            reasons.append("range: missing measured ground distance")
        if preflight:
            if self.armed:
                reasons.append("vehicle already armed")
            if self.landed != 1:
                reasons.append("landed state is not ON_GROUND")
        return reasons


@dataclass
class Output:
    setpoint: tuple[float, float, float, float] | None = None
    attitude_setpoint: tuple[float, float] | None = None  # ENU yaw, normalized thrust
    command: tuple[str, object] | None = None


class HoverController:
    TERMINAL = {"IDLE", "COMPLETE", "ABORT", "OPERATOR_CONTROL", "FAILSAFE"}
    FLYING = {"WARMUP", "SET_OFFBOARD", "ARMING", "INITIAL_CLIMB",
              "POSITION_HANDOVER", "TAKEOFF", "HOVER", "SQUARE"}
    INITIAL = {"WARMUP", "SET_OFFBOARD", "ARMING", "INITIAL_CLIMB"}

    def __init__(self, settings=None):
        self.settings = settings or HoverSettings()
        self.telemetry = Telemetry()
        self.state = "IDLE"
        self.reason = "diagnostics only" if not self.settings.enable_flight else "waiting for start"
        self.entered = 0.0
        self.ready_since = None
        self.altitude_since = None
        self.ground_since = None
        self.ground_distance_start = None
        self.vision_start = None
        self.takeoff_start_range = None
        self.range_error_since = None
        self.origin = None
        self.origin_frame = ""
        self.target_z = None
        self.last_tick = None
        self.last_pose = None
        self.last_pose_time = None
        self.pose_fault_until = 0.0
        self.pending = None
        self.pending_since = 0.0
        self.attempts = 0
        self.last_command = -math.inf
        self.offboard_confirmed = False
        self.aborted = False
        self.arm_requested = False
        self.climb = None
        self.square_vertices = ()
        self.square_leg = 0
        self.square_progress_m = 0.0
        self.square_leg_started = None
        self.square_arrival_since = None
        self.square_hover_settled_since = None

    @staticmethod
    def square_path(origin, edge_m):
        """Return four heading-relative edges in the current local ENU frame."""
        x, y, _, yaw = origin
        forward = (edge_m * math.cos(yaw), edge_m * math.sin(yaw))
        left = (-edge_m * math.sin(yaw), edge_m * math.cos(yaw))
        return ((x, y), (x + forward[0], y + forward[1]),
                (x + forward[0] + left[0], y + forward[1] + left[1]),
                (x + left[0], y + left[1]), (x, y))

    def square_path_error_m(self):
        start = self.square_vertices[self.square_leg]
        end = self.square_vertices[self.square_leg + 1]
        edge = self.settings.square_edge_m
        unit = ((end[0] - start[0]) / edge, (end[1] - start[1]) / edge)
        pose = self.telemetry.pose
        along = max(0.0, min(edge, (pose[0] - start[0]) * unit[0]
                             + (pose[1] - start[1]) * unit[1]))
        return math.hypot(pose[0] - start[0] - along * unit[0],
                          pose[1] - start[1] - along * unit[1])

    def square_tick(self, now, dt):
        """Advance one edge while measured position limits setpoint lead."""
        s = self.settings
        if now - self.square_leg_started > s.square_waypoint_timeout_s:
            self.abort(now, f"square waypoint {self.square_leg + 1} timed out")
            return
        start = self.square_vertices[self.square_leg]
        end = self.square_vertices[self.square_leg + 1]
        unit = ((end[0] - start[0]) / s.square_edge_m,
                (end[1] - start[1]) / s.square_edge_m)
        pose = self.telemetry.pose
        measured = max(0.0, min(s.square_edge_m,
                                (pose[0] - start[0]) * unit[0]
                                + (pose[1] - start[1]) * unit[1]))
        if self.square_progress_m > measured + s.square_setpoint_lead_m + 1e-6:
            self.abort(now, "square setpoint lead limit exceeded")
            return
        self.square_progress_m = min(s.square_edge_m,
                                     self.square_progress_m + s.square_speed_m_s * dt,
                                     measured + s.square_setpoint_lead_m)
        arrived = (self.square_progress_m >= s.square_edge_m
                   and math.hypot(pose[0] - end[0], pose[1] - end[1])
                   <= s.square_waypoint_tolerance_m
                   and abs(pose[2] - self.target_z) <= self.settings.altitude_tolerance_m)
        if arrived:
            if self.square_arrival_since is None:
                self.square_arrival_since = now
            elif now - self.square_arrival_since >= s.square_waypoint_dwell_s:
                if self.square_leg == 3:
                    self.transition("LANDING", now, "square complete")
                else:
                    self.square_leg += 1
                    self.square_progress_m = 0.0
                    self.square_leg_started = now
                    self.square_arrival_since = None
                    self.reason = f"square waypoint {self.square_leg + 1}/4"
        else:
            self.square_arrival_since = None

    def uses_initial_climb(self):
        return self.settings.initial_climb_enabled and self.state in self.INITIAL

    def transition(self, state, now, reason=""):
        self.state, self.entered = state, now
        self.reason = reason
        self.attempts = 0
        self.last_command = -math.inf
        self.ground_since = None
        if state == "TAKEOFF":
            self.takeoff_start_range = self.telemetry.ground_distance
            self.range_error_since = None

    def update_pose(self, pose, frame, now):
        t = self.telemetry
        jump = False
        if self.last_pose is not None and self.last_pose_time is not None:
            if 0 <= now - self.last_pose_time <= self.settings.sensor_timeout_s:
                distance = math.dist(pose[:3], self.last_pose[:3])
                yaw_delta = abs(math.atan2(
                    math.sin(pose[3] - self.last_pose[3]), math.cos(pose[3] - self.last_pose[3])))
                if self.uses_initial_climb() and distance > self.settings.max_pose_jump_m:
                    # Aiding can initialize/reset XY before it is used for control.
                    if self.climb is not None:
                        self.climb.aiding_since = None
                    distance = 0.0
                jump = (distance > self.settings.max_pose_jump_m
                        or yaw_delta > self.settings.max_yaw_jump_rad or frame != t.frame)
        if self.state in self.FLYING and frame != self.origin_frame:
            jump = True
        self.last_pose, self.last_pose_time = pose, now
        t.pose, t.frame, t.received["pose"] = pose, frame, now
        if jump:
            self.pose_fault_until = now + self.settings.readiness_dwell_s
            self.ready_since = None
            if self.state in self.FLYING:
                self.abort(now, "local pose jumped or frame changed")

    def update_vision_pose(self, pose, frame, now):
        t = self.telemetry
        previous = t.vision_pose
        jump = False
        if previous is not None and t.fresh("vision", now, self.settings.sensor_timeout_s):
            yaw_delta = abs(math.atan2(math.sin(pose[3] - previous[3]),
                                       math.cos(pose[3] - previous[3])))
            jump = (math.dist(pose[:3], previous[:3]) > self.settings.max_pose_jump_m
                    or yaw_delta > self.settings.max_yaw_jump_rad or frame != t.vision_frame)
        t.vision_pose, t.vision_frame, t.received["vision"] = pose, frame, now
        if jump:
            self.pose_fault_until = now + self.settings.readiness_dwell_s
            self.ready_since = None
            if self.state in self.FLYING:
                self.abort(now, "ZED pose jumped or frame changed")

    def connection_reset(self, now):
        self.telemetry.received.clear()
        self.ready_since = None
        self.last_pose = self.last_pose_time = None
        if self.state in self.FLYING:
            self.abort(now, "FCU connection changed; flight reference invalid")

    def readiness(self, now, *, preflight=None):
        if preflight is None:
            preflight = self.state in self.TERMINAL
        initial = self.settings.initial_climb_enabled and (preflight or self.uses_initial_climb())
        reasons = self.telemetry.blockers(now, self.settings, preflight=preflight,
                                          initial_climb=initial)
        if initial and self.settings.initial_climb_hover_thrust <= 0:
            reasons.append("set initial_climb_hover_thrust to the verified airframe hover thrust")
        if now < self.pose_fault_until:
            reasons.append("local pose changed discontinuously; waiting to settle")
        if reasons:
            self.ready_since = None
        elif self.ready_since is None:
            self.ready_since = now
        if not reasons and now - self.ready_since < self.settings.readiness_dwell_s:
            duration = self.settings.readiness_dwell_s
            reasons.append(f"waiting for {duration:g} s continuous readiness")
        return reasons

    def start(self, now):
        reasons = self.readiness(now, preflight=True)
        if not self.settings.enable_flight:
            reasons.insert(0, "flight disabled (enable_flight:=false)")
        if self.state != "IDLE":
            if self.state in self.TERMINAL:
                reasons.insert(0, f"restart node after {self.state.lower()} run")
            else:
                reasons.insert(0, "sequence or command already active")
        elif self.pending is not None:
            reasons.insert(0, "sequence or command already active")
        if reasons:
            return False, "; ".join(reasons)
        self.origin = self.telemetry.pose
        self.origin_frame = self.telemetry.frame
        self.ground_distance_start = self.telemetry.ground_distance
        self.vision_start = self.telemetry.vision_pose
        self.takeoff_start_range = None
        self.range_error_since = None
        self.target_z = self.origin[2]
        self.square_vertices = (self.square_path(self.origin, self.settings.square_edge_m)
                                if self.settings.mission_mode == "square" else ())
        self.square_leg = 0
        self.square_progress_m = 0.0
        self.square_leg_started = None
        self.square_arrival_since = None
        self.square_hover_settled_since = None
        self.offboard_confirmed = self.aborted = self.arm_requested = False
        self.altitude_since = None
        self.last_tick = now
        if self.settings.initial_climb_enabled:
            self.climb = RangeClimb(vertical_distance(self.telemetry), self.telemetry.attitude[2],
                                    self.telemetry.received["range"])
        self.transition("WARMUP", now, "streaming measured starting pose")
        return True, ("ZED square started" if self.settings.mission_mode == "square"
                      else "onboard hover started")

    def abort(self, now, reason):
        if self.state in self.TERMINAL:
            return
        if self.state == "LANDING":
            # A second abort must not hide the original fault or restart the
            # landing timeout. An operator abort during normal landing still
            # marks the run as aborted.
            if not self.aborted:
                self.aborted = True
                self.reason = reason
            return
        t = self.telemetry
        if (self.offboard_confirmed and t.mode != "OFFBOARD"
                and t.fresh("state", now, self.settings.state_timeout_s)):
            # A fresh mode change belongs to PX4/operator. Never send a new
            # AUTO.LAND request over an observed takeover, even if a sensor
            # fault arrives in the same control cycle.
            self.transition("OPERATOR_CONTROL", now,
                            f"mode changed to {t.mode}; releasing control")
            return
        self.aborted = True
        if (self.state in {"WARMUP", "SET_OFFBOARD", "ARMING"}
                and not self.telemetry.armed and not self.arm_requested):
            # Nothing has been armed. Stop the sequence without changing the
            # disarmed FCU to AUTO.LAND.
            self.transition("ABORT", now, reason)
            return
        self.transition("LANDING", now, reason)

    def command_result(self, accepted, now, detail=""):
        command = self.pending
        self.pending = None
        if command is None:
            return
        if not accepted:
            self.reason = f"{command[0]} rejected/failed: {detail}"
            if command == ("arm", True) and self.state == "ARMING":
                self.abort(now, self.reason)

    def request(self, kind, value, now):
        if self.pending is not None or now - self.last_command < self.settings.command_retry_s:
            return None
        if self.attempts >= self.settings.max_command_attempts:
            if self.state == "LANDING":
                self.transition("FAILSAFE", now, "landing/disarm command attempts exhausted")
            else:
                self.abort(now, "command attempts exhausted")
            return None
        self.pending = (kind, value)
        self.pending_since = self.last_command = now
        self.attempts += 1
        if self.pending == ("arm", True):
            self.arm_requested = True
        return self.pending

    def tick(self, now):
        t, s = self.telemetry, self.settings
        dt = 0.0 if self.last_tick is None else max(0.0, min(now - self.last_tick, 0.1))
        tick_gap = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        self.readiness(now)
        output = Output()
        if not s.enable_flight or self.state in self.TERMINAL:
            return output
        if self.pending is not None and now - self.pending_since > s.command_timeout_s:
            # An unacknowledged arm may have reached the FCU. Do not retry it or
            # declare success/disarmed; stop setpoints and leave PX4 failsafes active.
            self.transition("FAILSAFE", now, "command response timed out; operator must verify FCU")
            return output
        if self.state in self.FLYING:
            initial = self.uses_initial_climb()
            reasons = t.blockers(now, s, initial_climb=initial)
            if tick_gap > s.sensor_timeout_s or tick_gap < 0:
                reasons.append("control loop timing discontinuity")
            if not initial and t.pose is not None:
                excursion_limit = (s.square_max_excursion_m if self.state == "SQUARE"
                                   else s.max_horizontal_excursion_m)
                if math.hypot(t.pose[0] - self.origin[0],
                              t.pose[1] - self.origin[1]) > excursion_limit:
                    reasons.append("horizontal excursion limit exceeded")
                if (self.state == "SQUARE" and
                        self.square_path_error_m() > s.square_max_path_error_m):
                    reasons.append("square path error limit exceeded")
            range_climb = None
            if (s.localization_source == "zed" and self.state in {"TAKEOFF", "HOVER", "SQUARE"}
                    and t.pose is not None and t.vision_pose is not None
                    and self.vision_start is not None):
                vision_climb = t.vision_pose[2] - self.vision_start[2]
                local_climb = t.pose[2] - self.origin[2]
                if abs(local_climb - vision_climb) > s.max_altitude_range_error_m:
                    if self.range_error_since is None:
                        self.range_error_since = now
                    elif now - self.range_error_since >= s.altitude_range_error_dwell_s:
                        reasons.append("local altitude disagrees with ZED height")
                else:
                    self.range_error_since = None
                if (self.state == "TAKEOFF"
                        and now - self.entered > s.zed_takeoff_no_lift_timeout_s
                        and vision_climb < 0.03):
                    reasons.append(
                        "takeoff: no ZED-confirmed liftoff "
                        f"(ZED rise={vision_climb:.3f} m, local rise={local_climb:.3f} m, "
                        f"requested rise={self.target_z - self.origin[2]:.3f} m, "
                        f"PX4 landed state={t.landed})")
            if (s.localization_source != "zed" and self.state in {"TAKEOFF", "HOVER"} and t.pose is not None
                    and t.range_ok and t.fresh("range", now, s.sensor_timeout_s)
                    and t.ground_distance is not None and self.ground_distance_start is not None):
                # Raw downward range is independent of PX4 local Z. During this
                # short hover demo the surface beneath the drone must be level.
                range_climb = t.ground_distance - self.ground_distance_start
                local_climb = t.pose[2] - self.origin[2]
                if abs(local_climb - range_climb) > s.max_altitude_range_error_m:
                    if self.range_error_since is None:
                        self.range_error_since = now
                    elif now - self.range_error_since >= s.altitude_range_error_dwell_s:
                        reasons.append("local altitude disagrees with downward range")
                else:
                    self.range_error_since = None
                if (self.state == "TAKEOFF" and self.takeoff_start_range is not None
                        and now - self.entered > s.takeoff_no_lift_timeout_s
                        and t.ground_distance - self.takeoff_start_range < 0.03):
                    reasons.append("takeoff: no range-confirmed liftoff")
            goal_z = self.origin[2] + s.takeoff_altitude_m
            if not initial and t.pose is not None and t.pose[2] > goal_z + s.altitude_overshoot_m:
                reasons.append("altitude overshoot limit exceeded")
            if (t.pose is not None and self.state in {"HOVER", "SQUARE"}
                    and abs(t.pose[2] - goal_z) > s.altitude_overshoot_m):
                reasons.append("altitude left hover band" if self.state == "HOVER"
                               else "altitude left square-flight band")
            if reasons:
                self.abort(now, "; ".join(reasons))
            elif self.offboard_confirmed and t.mode != "OFFBOARD":
                self.transition("OPERATOR_CONTROL", now,
                                f"mode changed to {t.mode}; releasing control")
                return output
            elif t.armed and not self.arm_requested:
                self.transition("OPERATOR_CONTROL", now,
                                "vehicle armed externally; releasing control")
                return output
            elif self.state in {"INITIAL_CLIMB", "POSITION_HANDOVER", "TAKEOFF", "HOVER", "SQUARE"} and not t.armed:
                self.transition("ABORT", now, "vehicle disarmed during flight sequence")
                return output

        if self.state in self.FLYING:
            if self.state not in {"HOVER", "SQUARE"} and now - self.entered > s.stage_timeout_s:
                self.abort(now, "stage timed out (no confirmed progress)")
            else:
                if self.state == "WARMUP" and now - self.entered >= s.offboard_warmup_s:
                    self.transition("SET_OFFBOARD", now)
                if self.state == "SET_OFFBOARD":
                    if t.mode == "OFFBOARD":
                        self.offboard_confirmed = True
                        self.transition("ARMING", now)
                    else:
                        output.command = self.request("mode", "OFFBOARD", now)
                elif self.state == "ARMING":
                    if t.armed:
                        self.transition("INITIAL_CLIMB" if s.initial_climb_enabled else "TAKEOFF", now)
                    else:
                        output.command = self.request("arm", True, now)
                elif self.state == "INITIAL_CLIMB":
                    self.initial_climb_tick(now, dt, output)
                elif self.state == "POSITION_HANDOVER":
                    target = t.position_target
                    if (target is not None and t.fresh("position_target", now, s.sensor_timeout_s)
                            and t.received["position_target"] > self.entered
                            and math.dist(target[:3], (*self.origin[:2], self.target_z)) < 0.08
                            and abs(math.atan2(math.sin(target[3] - self.origin[3]),
                                              math.cos(target[3] - self.origin[3]))) < 0.1):
                        self.transition("TAKEOFF", now, "PX4 position target confirmed")
                    elif now - self.entered > s.command_timeout_s:
                        self.abort(now, "PX4 position handover not confirmed")
                elif self.state == "TAKEOFF":
                    goal_z = self.origin[2] + s.takeoff_altitude_m
                    # Limit accumulated position error if the airframe fails to climb.
                    lead = (s.zed_takeoff_setpoint_lead_m if s.localization_source == "zed"
                            else 0.25)
                    self.target_z = min(goal_z, self.target_z + s.takeoff_rate_m_s * dt,
                                        t.pose[2] + lead)
                    observed_climb = (t.vision_pose[2] - self.vision_start[2]
                                      if s.localization_source == "zed" and t.vision_pose is not None
                                      else range_climb)
                    if (observed_climb is not None and t.landed == 2
                            and abs(t.pose[2] - goal_z) <= s.altitude_tolerance_m
                            and abs(observed_climb - s.takeoff_altitude_m)
                            <= s.altitude_tolerance_m):
                        if self.altitude_since is None:
                            self.altitude_since = now
                        if now - self.altitude_since >= s.altitude_dwell_s:
                            self.target_z = goal_z
                            self.transition("HOVER", now, "target altitude held for dwell")
                    else:
                        self.altitude_since = None
                elif self.state == "HOVER" and now - self.entered >= s.hover_duration_s:
                    if s.mission_mode == "hover":
                        self.transition("LANDING", now, "hover complete")
                    else:
                        at_start = (math.hypot(t.pose[0] - self.origin[0],
                                               t.pose[1] - self.origin[1])
                                    <= s.square_waypoint_tolerance_m
                                    and abs(t.pose[2] - self.target_z) <= s.altitude_tolerance_m)
                        if at_start:
                            if self.square_hover_settled_since is None:
                                self.square_hover_settled_since = now
                            elif now - self.square_hover_settled_since >= s.square_waypoint_dwell_s:
                                self.square_leg_started = now
                                self.transition("SQUARE", now, "square waypoint 1/4")
                        else:
                            self.square_hover_settled_since = None
                        if (self.state == "HOVER" and now - self.entered
                                > s.hover_duration_s + s.square_waypoint_timeout_s):
                            self.abort(now, "square start did not settle at hover point")
                elif self.state == "SQUARE":
                    self.square_tick(now, dt)
                if self.state in self.FLYING:
                    if self.uses_initial_climb():
                        if output.attitude_setpoint is None:
                            output.attitude_setpoint = (self.climb.yaw, 0.0)
                    else:
                        if self.state == "SQUARE":
                            start = self.square_vertices[self.square_leg]
                            end = self.square_vertices[self.square_leg + 1]
                            fraction = self.square_progress_m / s.square_edge_m
                            xy = (start[0] + (end[0] - start[0]) * fraction,
                                  start[1] + (end[1] - start[1]) * fraction)
                        else:
                            xy = self.origin[:2]
                        output.setpoint = (*xy, self.target_z, self.origin[3])

        if self.state == "LANDING":
            # No further position setpoints: especially important after an EKF reset.
            output.setpoint = None
            output.attitude_setpoint = None
            if now - self.entered > s.landing_timeout_s:
                self.transition("FAILSAFE", now, "landing not confirmed; operator must verify FCU")
                return output
            if not t.connected or not t.fresh("state", now, s.state_timeout_s):
                return output
            if self.offboard_confirmed and t.mode not in {"OFFBOARD", "AUTO.LAND"}:
                self.transition("OPERATOR_CONTROL", now,
                                f"mode changed to {t.mode}; releasing control")
                return output
            on_ground = (t.landed == 1 and t.fresh("extended", now, s.state_timeout_s)
                         and t.received["extended"] >= self.entered)
            if on_ground:
                if self.ground_since is None:
                    self.ground_since = now
            else:
                self.ground_since = None
            settled = self.ground_since is not None and now - self.ground_since >= 1.0
            if not t.armed and self.pending is None and settled:
                self.transition("ABORT" if self.aborted else "COMPLETE", now, self.reason)
            elif t.mode != "AUTO.LAND":
                output.command = self.request("mode", "AUTO.LAND", now)
            elif t.armed and settled:
                output.command = self.request("arm", False, now)
        return output

    def initial_climb_tick(self, now, dt, output):
        t, s, climb = self.telemetry, self.settings, self.climb
        climb.update(vertical_distance(t), t.received["range"])
        elapsed = now - self.entered
        yaw_error = abs(math.atan2(math.sin(t.attitude[2] - climb.yaw),
                                   math.cos(t.attitude[2] - climb.yaw)))
        reason = ""
        if climb.raw_height > s.initial_climb_max_height_m:
            reason = "initial climb height limit exceeded"
        elif abs(climb.speed) > s.initial_climb_max_speed_m_s:
            reason = "initial climb vertical speed limit exceeded"
        elif yaw_error > s.max_yaw_jump_rad:
            reason = "initial climb yaw changed unexpectedly"
        elif elapsed > s.initial_climb_no_lift_timeout_s and climb.height < 0.03:
            reason = "initial climb: no measured liftoff"
        elif elapsed > s.initial_climb_timeout_s:
            reason = "initial climb timed out before stable position aiding"
        if reason:
            self.abort(now, reason)
            return
        # Use independent range for actual climb, then require all normal checks.
        handover_ready = (not t.blockers(now, s)
                          and t.fresh("estimator", now, s.sensor_timeout_s)
                          and t.landed == 2
                          and climb.distance >= s.initial_climb_min_range_m
                          and abs(climb.height - s.initial_climb_height_m) <= 0.04
                          and abs(climb.speed) <= 0.12)
        if handover_ready:
            if climb.aiding_since is None:
                climb.aiding_since = now
        else:
            climb.aiding_since = None
        if climb.aiding_since is not None and now - climb.aiding_since >= s.initial_climb_aiding_dwell_s:
            # Anchor XY only after aiding settles. Preserve total height above
            # the measured launch surface despite unaided local-Z drift/resets.
            self.origin = (*t.pose[:2], t.pose[2] - climb.height, climb.yaw)
            self.origin_frame = t.frame
            self.target_z = t.pose[2]
            self.transition("POSITION_HANDOVER", now, "measured climb and stable aiding; confirming PX4 target")
            return
        output.attitude_setpoint = climb.command(s, dt)
