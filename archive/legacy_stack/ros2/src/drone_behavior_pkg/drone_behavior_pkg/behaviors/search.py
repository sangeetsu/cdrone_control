from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SearchBehavior:
    name: str = "SEARCH"

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        candidate = manager.select_target(now_s)
        manager.publish_zero_velocity()
        manager.publish_light(False, 0.0, 0.0)
        if candidate is None:
            return self.name
        manager.set_active_target(candidate.track_id, now_s)
        return "ALIGN"
