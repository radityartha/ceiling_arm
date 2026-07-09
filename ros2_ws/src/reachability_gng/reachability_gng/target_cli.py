"""Interactive target picker for reach_fusion (like pick_cli).

Lists live objects by Isaac prim name obj_N (ground truth, via ObjnLocalizer)
with world positions, and sends the chosen obj_N / class label / index to
/reach_fusion/set_target -- reach_fusion switches target without a restart.

    ros2 run reachability_gng reach_fusion     # backend
    ros2 run reachability_gng target_cli        # this menu (own terminal)
"""
from __future__ import annotations

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from std_msgs.msg import String

from reachability_gng.objn_localizer import ObjnLocalizer


class TargetCli(Node):
    def __init__(self):
        super().__init__('target_cli')
        self.declare_parameter('set_target_topic', '/reach_fusion/set_target')
        self._lock = threading.Lock()
        self.poses = np.empty((0, 3))
        self.last_sent = None
        self.objn = ObjnLocalizer(self)
        self.create_subscription(PoseArray, '/detected_objects',
                                 self._on_objects, 1)
        self.pub = self.create_publisher(
            String, self.get_parameter('set_target_topic').value, 1)

    def _on_objects(self, msg):
        with self._lock:
            self.poses = np.array([[p.position.x, p.position.y, p.position.z]
                                   for p in msg.poses]) if msg.poses \
                else np.empty((0, 3))

    def objn_map(self):
        with self._lock:
            poses = self.poses.copy()
        return self.objn.map(poses), len(poses)

    def send(self, value):
        self.pub.publish(String(data=value))
        self.last_sent = value


def _loop(node: TargetCli):
    print('\n=== target_cli ===  (Enter=refresh, q=quit)')
    print('type obj_N (ground truth) / a class label / an index -> reach_fusion target')
    while rclpy.ok():
        objn, n = node.objn_map()
        cur = f'  [current: {node.last_sent}]' if node.last_sent else ''
        print(f'\nDetected objects ({n} total){cur}')
        if not objn:
            print('  (no obj_N resolved -- is perception + seg_source:=isaac up?)')
        for name in sorted(objn):
            i, p = objn[name]
            print(f'  {name}  (index {i})  x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f}')
        try:
            raw = input('Set target> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if raw.lower() in ('q', 'quit', 'exit'):
            break
        if raw:
            node.send(raw)
            print(f'  -> sent "{raw}"')


def main():
    rclpy.init()
    node = TargetCli()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        _loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
