from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class IlluminateBehavior:
    name: str = "ILLUMINATE"

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        target = manager.get_active_target(now_s)
        if target is None:
            if manager.active_target_lost_for(now_s) > manager.lost_target_timeout_s:
                return "FAILSAFE_HOLD"
            manager.publish_zero_velocity()
            manager.publish_light(False, 0.0, 0.0)
            return self.name

        manager.publish_velocity_for_target(target, desired_distance_m=manager.engage_distance_m)
        manager.publish_light(True, manager.light_default_intensity, 0.0)
        if manager.illumination_start_time_s is None:
            manager.illumination_start_time_s = now_s
        if now_s - manager.illumination_start_time_s >= manager.dwell_time_s:
            return "ADVANCE_QUEUE"
        return self.name
