from drone_behavior_pkg.behaviors.align import AlignBehavior
from drone_behavior_pkg.behaviors.approach import ApproachBehavior
from drone_behavior_pkg.behaviors.follow_standoff import FollowStandoffBehavior
from drone_behavior_pkg.behaviors.lost_target_hold import LostTargetHoldBehavior
from drone_behavior_pkg.math_utils import TrackSnapshot


class DummyManager:
    def __init__(self):
        self.follow_distance_m = 3.0
        self.follow_distance_tolerance_m = 0.4
        self.lost_target_timeout_s = 0.75
        self.light_default_intensity = 0.9
        self.align_yaw_tolerance_rad = 0.12
        self.reacquire_timeout_s = 2.0
        self._target = None
        self._lost_for = 0.0
        self._obstacle_blocked = False
        self.cleared = False

    def get_active_target(self, _now_s):
        return self._target

    def active_target_lost_for(self, _now_s):
        return self._lost_for

    def compute_obstacle_blocked(self, _target):
        return self._obstacle_blocked

    def publish_zero_velocity(self):
        pass

    def publish_light(self, _enabled, _intensity, _strobe_hz):
        pass

    def publish_align_for_target(self, _target):
        return 0.05

    def publish_velocity_for_target(self, _target, desired_distance_m):
        assert desired_distance_m == self.follow_distance_m

    def clear_active_target(self):
        self.cleared = True


def _target(distance_m: float, y_b_m: float = 0.0) -> TrackSnapshot:
    return TrackSnapshot(
        track_id=1,
        x_b_m=distance_m,
        y_b_m=y_b_m,
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


def test_align_transitions_to_approach_when_bearing_small():
    mgr = DummyManager()
    mgr._target = _target(1.0)
    behavior = AlignBehavior()
    assert behavior.step(mgr, 0.0) == "APPROACH"


def test_approach_transitions_to_follow_standoff_when_in_range():
    mgr = DummyManager()
    mgr._target = _target(3.2)
    behavior = ApproachBehavior()
    assert behavior.step(mgr, 0.0) == "FOLLOW_STANDOFF"


def test_follow_standoff_stays_active_with_visible_target():
    mgr = DummyManager()
    mgr._target = _target(3.5)
    behavior = FollowStandoffBehavior()
    assert behavior.step(mgr, 0.0) == "FOLLOW_STANDOFF"


def test_lost_target_hold_returns_to_search_after_timeout():
    mgr = DummyManager()
    mgr._target = None
    mgr._lost_for = 2.5
    behavior = LostTargetHoldBehavior()
    assert behavior.step(mgr, 0.0) == "SEARCH"
    assert mgr.cleared
