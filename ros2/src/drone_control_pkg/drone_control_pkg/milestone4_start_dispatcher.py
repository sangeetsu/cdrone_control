"""Expose the existing start service for one-shot startup of independent stacks."""

import json

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .milestone4_session import StartDispatcher, invoke


class Milestone4StartDispatcher(Node):
    def __init__(self):
        super().__init__("milestone4_start_dispatcher")

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        drone_id = parameter("drone_id", "cdrone3")
        peer_id = parameter("peer_drone_id", "cdrone4")
        peer = (parameter("peer_host", "192.168.0.121"),
                parameter("peer_user", "jetson"),
                parameter("peer_workspace", "/home/jetson/cdrone_control"))
        arguments = json.loads(parameter("peer_arguments_json", "{}"))
        self.dispatcher = StartDispatcher(drone_id, peer_id, peer)
        self.get_logger().info(f"Launching independent peer stack on {peer[0]}")
        self.peer_launch = invoke("launch", {"arguments": arguments}, peer)
        log = self.get_logger().info if self.peer_launch["success"] else self.get_logger().error
        log(self.peer_launch["message"])
        self.create_service(Trigger, f"/cdrone/{drone_id}/demo/milestone3_start",
                            self.start, callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Paired start service ready; no ongoing communication with peer")

    def start(self, request, response):
        del request
        if not self.peer_launch["success"]:
            result = dict(success=False, message="peer launch failed: " + self.peer_launch["message"])
        else:
            result = self.dispatcher.start()
        response.success, response.message = result["success"], result["message"]
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Milestone4StartDispatcher()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
