from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ApproachBehavior:
    name: str = "APPROACH"

    def step(self, manager: "EngagementManagerNode", now_s: float) -> str:
        target = manager.get_active_target(now_s)
        if target is None:
            if manager.active_target_lost_for(now_s) > manager.lost_target_timeout_s:
                return "LOST_TARGET_HOLD"
            manager.publish_zero_velocity()
            manager.publish_light(False, 0.0, 0.0)
            return self.name

        if manager.compute_obstacle_blocked(target):
            manager.publish_zero_velocity()
            manager.publish_light(False, 0.0, 0.0)
            return self.name

        manager.publish_velocity_for_target(
            target, desired_distance_m=manager.follow_distance_m
        )
        manager.publish_light(False, 0.0, 0.0)

        if target.distance_m <= manager.follow_distance_m + manager.follow_distance_tolerance_m:
            return "FOLLOW_STANDOFF"
        return self.name
