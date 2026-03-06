from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ApproachBehavior:
    name: str = "APPROACH"

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        target = manager.get_active_target(now_s)
        if target is None:
            if manager.active_target_lost_for(now_s) > manager.lost_target_timeout_s:
                return "FAILSAFE_HOLD"
            manager.publish_zero_velocity()
            manager.publish_light(False, 0.0, 0.0)
            return self.name

        manager.publish_velocity_for_target(target, desired_distance_m=manager.engage_distance_m)
        manager.publish_light(False, 0.0, 0.0)

        if target.distance_m <= manager.engage_distance_m:
            if manager.approach_stable_start_s is None:
                manager.approach_stable_start_s = now_s
            if now_s - manager.approach_stable_start_s >= manager.stability_time_s:
                manager.illumination_start_time_s = None
                return "ILLUMINATE"
        else:
            manager.approach_stable_start_s = None
        return self.name
