from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AdvanceQueueBehavior:
    name: str = "ADVANCE_QUEUE"

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        manager.mark_active_target_completed(now_s)
        manager.publish_zero_velocity()
        manager.publish_light(False, 0.0, 0.0)
        manager.clear_active_target()
        return "SEARCH"
