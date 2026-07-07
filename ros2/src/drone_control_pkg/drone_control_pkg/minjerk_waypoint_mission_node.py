from __future__ import annotations

import rclpy

from drone_control_pkg.position_goto_demo_sequence_node import (
    PositionGotoDemoSequenceNode,
)


class MinJerkWaypointMissionNode(PositionGotoDemoSequenceNode):
    def __init__(self) -> None:
        super().__init__("minjerk_waypoint_mission_node")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinJerkWaypointMissionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
