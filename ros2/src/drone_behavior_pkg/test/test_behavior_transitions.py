from drone_behavior_pkg.behaviors.approach import ApproachBehavior
from drone_behavior_pkg.behaviors.illuminate import IlluminateBehavior
from drone_behavior_pkg.math_utils import TrackSnapshot


class DummyManager:
    def __init__(self):
        self.engage_distance_m = 1.2
        self.lost_target_timeout_s = 0.75
        self.stability_time_s = 0.5
        self.dwell_time_s = 2.0
        self.light_default_intensity = 0.9
        self.approach_stable_start_s = None
        self.illumination_start_time_s = None
        self._target = None
        self._lost_for = 0.0

    def get_active_target(self, _now_s):
        return self._target

    def active_target_lost_for(self, _now_s):
        return self._lost_for

    def publish_zero_velocity(self):
        pass

    def publish_light(self, _enabled, _intensity, _strobe_hz):
        pass

    def publish_velocity_for_target(self, _target, desired_distance_m):
        assert desired_distance_m == self.engage_distance_m


def _target(distance_m: float) -> TrackSnapshot:
    return TrackSnapshot(
        track_id=1,
        x_b_m=distance_m,
        y_b_m=0.0,
        z_b_m=0.0,
        vx_b_mps=0.0,
        vy_b_mps=0.0,
        vz_b_mps=0.0,
        distance_m=distance_m,
        confidence=1.0,
        bbox_area_px=100.0,
        inbound=True,
        last_seen_s=0.0,
    )


def test_approach_transitions_to_illuminate_after_stability():
    mgr = DummyManager()
    mgr._target = _target(1.0)
    behavior = ApproachBehavior()
    assert behavior.step(mgr, 0.0) == "APPROACH"
    assert behavior.step(mgr, 0.6) == "ILLUMINATE"


def test_illuminate_transitions_to_advance_after_dwell():
    mgr = DummyManager()
    mgr._target = _target(1.0)
    behavior = IlluminateBehavior()
    assert behavior.step(mgr, 0.0) == "ILLUMINATE"
    assert behavior.step(mgr, 2.1) == "ADVANCE_QUEUE"
