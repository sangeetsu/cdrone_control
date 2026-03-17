from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Protocol

from drone_behavior_pkg.behaviors.align import AlignBehavior
from drone_behavior_pkg.behaviors.approach import ApproachBehavior
from drone_behavior_pkg.behaviors.follow_standoff import FollowStandoffBehavior
from drone_behavior_pkg.behaviors.lost_target_hold import LostTargetHoldBehavior
from drone_behavior_pkg.behaviors.search import SearchBehavior


class BehaviorAction(Protocol):
    name: str

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        ...


@dataclass(frozen=True)
class FailsafeHoldBehavior:
    name: str = "FAILSAFE_HOLD"

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        manager.publish_zero_velocity()
        manager.publish_light(False, 0.0, 0.0)
        if manager.estop_latched:
            return self.name
        if manager.failsafe_enter_time_s is None:
            manager.failsafe_enter_time_s = now_s
        if now_s - manager.failsafe_enter_time_s >= manager.reacquire_timeout_s:
            manager.clear_active_target()
            manager.failsafe_enter_time_s = None
            return "SEARCH"
        return self.name


def build_behavior_registry() -> Dict[str, BehaviorAction]:
    return {
        "SEARCH": SearchBehavior(),
        "ALIGN": AlignBehavior(),
        "APPROACH": ApproachBehavior(),
        "FOLLOW_STANDOFF": FollowStandoffBehavior(),
        "LOST_TARGET_HOLD": LostTargetHoldBehavior(),
        "FAILSAFE_HOLD": FailsafeHoldBehavior(),
    }
