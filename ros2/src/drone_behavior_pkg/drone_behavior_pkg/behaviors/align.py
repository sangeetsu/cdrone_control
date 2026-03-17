from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AlignBehavior:
    name: str = "ALIGN"

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

        yaw_error = manager.publish_align_for_target(target)
        manager.publish_light(False, 0.0, 0.0)
        if abs(yaw_error) <= manager.align_yaw_tolerance_rad:
            return "APPROACH"
        return self.name
