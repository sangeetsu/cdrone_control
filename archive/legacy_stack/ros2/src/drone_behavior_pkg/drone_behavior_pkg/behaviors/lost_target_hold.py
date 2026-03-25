from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LostTargetHoldBehavior:
    name: str = "LOST_TARGET_HOLD"

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        manager.publish_zero_velocity()
        manager.publish_light(False, 0.0, 0.0)

        target = manager.get_active_target(now_s)
        if target is not None:
            return "ALIGN"

        if manager.active_target_lost_for(now_s) >= manager.reacquire_timeout_s:
            manager.clear_active_target()
            return "SEARCH"
        return self.name
