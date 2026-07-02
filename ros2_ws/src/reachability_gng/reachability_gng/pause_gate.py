"""Shared pause gate so a pick plans/executes against a STABLE planning scene.

A live octomap (collision_cloud) and the 0.5 s CollisionObject republish
(object_collision) each bump the planning-scene version. If that happens WHILE
move_group is planning, MoveIt drops the plan (MoveGroup err=-3
MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE, or -2 with the path invalid at the
start states). gantry_reach_executor latches std_msgs/Bool on /perception/pause
for the duration of a pick; the perception publishers skip publishing while it is
True, so the scene the planner sees is frozen. Robot-state collision (the other
arms, via /joint_states) stays live -- only the sensor-derived scene is frozen.

Safety: the subscribe side AUTO-RESUMES after `timeout` s, so a crashed or killed
executor can never freeze perception forever.
"""
import time

from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

PAUSE_TOPIC = '/perception/pause'


def latched_qos():
    """depth-1 TRANSIENT_LOCAL so late subscribers get the current pause state."""
    q = QoSProfile(depth=1)
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return q


class PauseGate:
    """Subscribe side for a perception node: `paused()` gates its publish path."""

    def __init__(self, node, timeout=30.0):
        self._node = node
        self._timeout = float(timeout)
        self._paused = False
        self._t = 0.0
        node.create_subscription(Bool, PAUSE_TOPIC, self._on, latched_qos())

    def _on(self, msg):
        self._paused = bool(msg.data)
        self._t = time.monotonic()
        self._node.get_logger().info(
            f'perception {"PAUSED" if self._paused else "resumed"} (pick gate)')

    def paused(self):
        if self._paused and (time.monotonic() - self._t) > self._timeout:
            self._paused = False
            self._node.get_logger().warn('pause gate auto-resumed (timeout)')
        return self._paused
