#!/usr/bin/env python3

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger


class MobileRobotTaskServer(Node):
    """Safe mobile-base task adapter for the RViz fake base simulation."""

    def __init__(self):
        super().__init__("mobile_robot_task_server")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed_mps", 0.15)
        self.declare_parameter("move_duration_sec", 2.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._linear_speed_mps = float(self.get_parameter("linear_speed_mps").value)
        self._move_duration_sec = float(self.get_parameter("move_duration_sec").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._publisher = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._lock = threading.Lock()
        self._busy = False
        self._stop_event = threading.Event()
        self._motion_thread = None

        self._services = [
            self.create_service(Trigger, "/task/stop", self._stop_cb),
            self.create_service(Trigger, "/task/move_forward", self._move_forward_cb),
            self.create_service(Trigger, "/task/move_backward", self._move_backward_cb),
            self.create_service(Trigger, "/task/open_curtain", self._not_implemented_cb("open_curtain")),
            self.create_service(Trigger, "/task/close_curtain", self._not_implemented_cb("close_curtain")),
            self.create_service(Trigger, "/task/bring_bag", self._not_implemented_cb("bring_bag")),
            self.create_service(Trigger, "/task/bring_bottle", self._not_implemented_cb("bring_bottle")),
        ]

        self._publish_stop()
        self.get_logger().info(
            "Mobile task server ready: cmd_vel=%s, speed=%.3f m/s, duration=%.2f s"
            % (self._cmd_vel_topic, self._linear_speed_mps, self._move_duration_sec)
        )
        self.get_logger().info("Ready: /task/move_forward")
        self.get_logger().info("Ready: /task/move_backward")
        self.get_logger().info("Ready: /task/stop")

    def _move_forward_cb(self, request, response):
        del request
        return self._execute_move(response, self._linear_speed_mps, "move_forward")

    def _move_backward_cb(self, request, response):
        del request
        return self._execute_move(response, -self._linear_speed_mps, "move_backward")

    def _stop_cb(self, request, response):
        del request
        self._stop_event.set()
        self._publish_stop()
        with self._lock:
            was_busy = self._busy
        response.success = True
        response.message = "Mobile base stop requested" if was_busy else "Mobile base already stopped"
        self.get_logger().warn(response.message)
        return response

    def _not_implemented_cb(self, task_name):
        def callback(request, response):
            del request
            response.success = False
            response.message = (
                "Task '%s' is not implemented in the mobile-base adapter" % task_name
            )
            self.get_logger().warn(response.message)
            return response

        return callback

    def _execute_move(self, response, linear_x, task_name):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = "Rejected %s: mobile base is busy" % task_name
                self.get_logger().warn(response.message)
                return response
            self._busy = True
            self._stop_event.clear()

        self.get_logger().info(
            "Starting %s: linear_x=%.3f m/s for %.2f s"
            % (task_name, linear_x, self._move_duration_sec)
        )
        self._motion_thread = threading.Thread(
            target=self._motion_worker,
            args=(linear_x, task_name),
            daemon=True,
        )
        self._motion_thread.start()
        response.success = True
        response.message = "Mobile task started: %s" % task_name
        return response

    def _motion_worker(self, linear_x, task_name):
        try:
            self._publish_stop()
            completed = self._publish_motion_for_duration(linear_x)
            self._publish_stop()
            if completed:
                self.get_logger().info("Completed %s" % task_name)
            else:
                self.get_logger().warn("Stopped %s before completion" % task_name)
        finally:
            with self._lock:
                self._busy = False

    def _publish_motion_for_duration(self, linear_x):
        period = 1.0 / max(self._publish_rate_hz, 1.0)
        iterations = max(1, int(self._move_duration_sec * self._publish_rate_hz))
        twist = Twist()
        twist.linear.x = linear_x

        for _ in range(iterations):
            if not rclpy.ok() or self._stop_event.is_set():
                return False
            self._publisher.publish(twist)
            time.sleep(period)
        return True

    def _publish_stop(self):
        self._publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MobileRobotTaskServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_event.set()
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
