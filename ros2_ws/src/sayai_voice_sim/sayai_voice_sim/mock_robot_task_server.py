#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MockRobotTaskServer(Node):
    """Simulated robot task service server for voice command integration tests."""

    TASKS = {
        "stop": ("stop", "/task/stop", "SIMULATION: stop requested"),
        "open_curtain": ("open_curtain", "/task/open_curtain", "SIMULATION: opening curtain"),
        "close_curtain": ("close_curtain", "/task/close_curtain", "SIMULATION: closing curtain"),
        "bring_bag": ("bring_bag", "/task/bring_bag", "SIMULATION: bringing bag"),
        "bring_bottle": ("bring_bottle", "/task/bring_bottle", "SIMULATION: bringing bottle"),
        "move_forward": ("move_forward", "/task/move_forward", "SIMULATION: moving forward"),
        "move_backward": ("move_backward", "/task/move_backward", "SIMULATION: moving backward"),
    }

    def __init__(self):
        super().__init__("mock_robot_task_server")
        self._services = []

        for task_name, service_name, log_message in self.TASKS.values():
            service = self.create_service(
                Trigger,
                service_name,
                self._make_callback(task_name, log_message),
            )
            self._services.append(service)
            self.get_logger().info("Ready: %s" % service_name)

    def _make_callback(self, task_name, log_message):
        def callback(request, response):
            del request
            self.get_logger().info(log_message)
            response.success = True
            response.message = "Simulated task completed: %s" % task_name
            return response

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotTaskServer()

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
