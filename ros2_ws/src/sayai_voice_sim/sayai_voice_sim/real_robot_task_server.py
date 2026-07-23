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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class RealRobotTaskServer(Node):
    # How long to wait after the sequence process exits for its /task/result
    # message to arrive before giving up on it.
    RESULT_GRACE_S = 2.0

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
        "/task/go_home": [
            "ros2", "launch", "workcell_moveit_config", "go_home_demo.launch.py"
        ],
    }

    def __init__(self):
        super().__init__("real_robot_task_server")

        self._proc = None          # running sequence subprocess, or None
        self._active_service = ""  # which task is running, for logging

        # Authoritative outcome of the running sequence, or None until it
        # reports. `ros2 launch` always exits 0 even when the runner it
        # launched exits non-zero, so the subprocess exit code cannot tell a
        # failed sequence from a successful one -- only this topic can.
        self._result = None
        self._exit_time = None
        self.create_subscription(
            Bool,
            "/task/result",
            self._on_result,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

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
            # Drop any result left over from the previous sequence, or it would
            # be reported as this one's outcome.
            self._result = None
            self._exit_time = None
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

    def _on_result(self, msg):
        self._result = bool(msg.data)

    def _reap(self):
        if self._proc is None:
            return
        code = self._proc.poll()
        if code is None:
            return

        # The runner publishes /task/result just before exiting, so the message
        # can still be in flight here. Give it a short grace period before
        # falling back to the (unreliable) exit code.
        if self._exit_time is None:
            self._exit_time = time.time()
        if self._result is None and (time.time() - self._exit_time) < self.RESULT_GRACE_S:
            return

        if self._result is not None:
            if self._result:
                self.get_logger().info("%s finished OK." % self._active_service)
            else:
                self.get_logger().error(
                    "%s FAILED: sequence did not complete." % self._active_service
                )
        elif code == 0:
            # Launch succeeded but the runner never reported. Do not claim
            # success -- an aborted sequence looks exactly like this.
            self.get_logger().warn(
                "%s: process exited 0 but never reported a result; "
                "outcome unknown (runner may have crashed)." % self._active_service
            )
        else:
            self.get_logger().error(
                "%s exited with code %d." % (self._active_service, code)
            )
        self._proc = None
        self._active_service = ""
        self._result = None
        self._exit_time = None

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
        self._result = None
        self._exit_time = None


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
