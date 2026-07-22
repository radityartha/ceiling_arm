#!/usr/bin/env python3
"""Run the real workcell sequences from voice Trigger services.

Each /task/<name> service launches the matching demo launch file as a
subprocess. Only one sequence runs at a time: a request that arrives while a
sequence is still running is rejected (success=False) so two spoken commands
can never move the arms at the same time. /task/stop sends SIGINT to the active
sequence for a clean shutdown.

The sequences assume my_workcell.launch.py and a dual_table_controller are
already running -- same as launching the demos by hand.
"""

import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RealRobotTaskServer(Node):
    # service name -> demo launch command (workcell_moveit_config)
    TASKS = {
        "/task/open_curtain": [
            "ros2", "launch", "workcell_moveit_config", "open_curtain_demo.launch.py"
        ],
        "/task/close_curtain": [
            "ros2", "launch", "workcell_moveit_config", "close_curtain_demo.launch.py"
        ],
        "/task/bring_bag": [
            "ros2", "launch", "workcell_moveit_config", "take_bag_demo.launch.py"
        ],
        "/task/bring_bottle": [
            "ros2", "launch", "workcell_moveit_config", "take_bottle_demo.launch.py"
        ],
        "/task/unitree_collab": [
            "ros2", "launch", "workcell_moveit_config", "unitree_collab_demo.launch.py"
        ],
    }

    def __init__(self):
        super().__init__("real_robot_task_server")

        self._proc = None          # running sequence subprocess, or None
        self._active_service = ""  # which task is running, for logging

        self._services = []
        for service_name in self.TASKS:
            self._services.append(
                self.create_service(
                    Trigger, service_name, self._make_task_callback(service_name)
                )
            )
            self.get_logger().info("Ready: %s" % service_name)

        self._services.append(
            self.create_service(Trigger, "/task/stop", self._stop_callback)
        )
        self.get_logger().info("Ready: /task/stop")

        # Clear the busy flag once a sequence finishes, without a new request.
        self._reaper = self.create_timer(0.5, self._reap)

    def _make_task_callback(self, service_name):
        def callback(request, response):
            del request
            if self._is_busy():
                response.success = False
                response.message = (
                    "busy: '%s' is still running; say 'stop' first"
                    % self._active_service
                )
                self.get_logger().warn(response.message)
                return response

            command = self.TASKS[service_name]
            self.get_logger().info("Starting %s" % service_name)
            try:
                # Own session so /task/stop can SIGINT the whole launch tree,
                # and so a Ctrl-C on this server does not kill the sequence.
                self._proc = subprocess.Popen(command, start_new_session=True)
            except Exception as exc:  # pylint: disable=broad-except
                self._proc = None
                self._active_service = ""
                response.success = False
                response.message = "failed to start %s: %s" % (service_name, exc)
                self.get_logger().error(response.message)
                return response

            self._active_service = service_name
            response.success = True
            response.message = "started %s" % service_name
            return response

        return callback

    def _stop_callback(self, request, response):
        del request
        if not self._is_busy():
            response.success = True
            response.message = "no sequence running"
            self.get_logger().info(response.message)
            return response

        self.get_logger().warn("STOP: cancelling %s" % self._active_service)
        self._terminate()
        response.success = True
        response.message = "stopped"
        return response

    def _is_busy(self):
        return self._proc is not None and self._proc.poll() is None

    def _reap(self):
        if self._proc is None:
            return
        code = self._proc.poll()
        if code is None:
            return
        if code == 0:
            self.get_logger().info("%s finished OK." % self._active_service)
        else:
            self.get_logger().error(
                "%s exited with code %d." % (self._active_service, code)
            )
        self._proc = None
        self._active_service = ""

    def _terminate(self):
        if self._proc is None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            try:
                self._proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Did not stop on SIGINT; sending SIGKILL.")
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self._proc = None
        self._active_service = ""


def main(args=None):
    rclpy.init(args=args)
    node = RealRobotTaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._terminate()  # never leave a sequence running after the server dies
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
