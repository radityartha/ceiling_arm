"""Interactive object picker for the gantry_reach_executor.

A small control terminal: it lists the live objects from /detected_objects
(with labels from /detected_objects/markers) and lets you type an index to fire
a pick -- replacing the repeated `ros2 topic pub .../pick` one-liner.

The index is the position in /detected_objects, exactly what the executor uses
when no /target_object is configured (target_label==""). If a target_label IS
set the executor ignores the index and always grasps that target.

Each object also shows a CHEAP distance estimate: the straight-line distance
from the object to each arm's current tool frame (via TF), and which arm is
nearer. This is only a geometric proxy -- the executor's true energy J (gantry
travel + hold + manipulability over the GNG maps) is computed at pick time and
printed in its own terminal / CSV, not here.

    ros2 launch reachability_gng gantry_pick.launch.py     # backend (terminal 1)
    ros2 run reachability_gng pick_cli                     # this menu (terminal 2)

Watch the gantry_pick.launch.py terminal for the chosen arm / plan result.
"""
from __future__ import annotations

import math
import threading

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import MarkerArray


class PickCli(Node):
    def __init__(self):
        super().__init__('pick_cli')
        self.declare_parameter('pick_topic', '/gantry_reach_executor/pick')
        # kept in sync with gantry_reach_executor's arm_names / arm_ee_frames
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        self.declare_parameter('tool_frames',
                               ['t1_a1_tool_frame', 't1_a2_tool_frame'])
        self._topic = self.get_parameter('pick_topic').value
        self.arm_names = list(self.get_parameter('arm_names').value)
        self.tool_frames = list(self.get_parameter('tool_frames').value)

        self._poses = []                 # latest /detected_objects poses
        self._world = 'world'            # /detected_objects header frame
        self._labels = {}                # marker id -> label text
        self._lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseArray, '/detected_objects',
                                 self._on_objects, 1)
        self.create_subscription(MarkerArray, '/detected_objects/markers',
                                 self._on_markers, 1)
        self.pick_pub = self.create_publisher(String, self._topic, 1)

    def _on_objects(self, msg):
        with self._lock:
            self._poses = list(msg.poses)
            if msg.header.frame_id:
                self._world = msg.header.frame_id

    def _on_markers(self, msg):
        with self._lock:
            for m in msg.markers:
                if m.ns == 'labels':
                    self._labels[m.id] = m.text

    def snapshot(self):
        with self._lock:
            return list(self._poses), dict(self._labels), self._world

    def _tool_xyz(self, frame, world):
        """Current tool-frame position in `world`, or None if TF not ready."""
        try:
            t = self.tf_buffer.lookup_transform(world, frame, Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, tr.z)

    def arm_distances(self, pose, world):
        """[(arm_name, distance_m or None)] from object to each arm's tool."""
        p = pose.position
        out = []
        for name, frame in zip(self.arm_names, self.tool_frames):
            xyz = self._tool_xyz(frame, world)
            if xyz is None:
                out.append((name, None))
            else:
                d = math.dist((p.x, p.y, p.z), xyz)
                out.append((name, d))
        return out

    def send_pick(self, idx):
        self.pick_pub.publish(String(data=str(idx)))


def _fmt_dist(node, pose, world):
    dists = node.arm_distances(pose, world)
    known = [(n, d) for n, d in dists if d is not None]
    if not known:
        return 'dist: TF not ready'
    parts = ' '.join(f'{n}={d:.2f}m' for n, d in known)
    nearest = min(known, key=lambda nd: nd[1])
    return f'{parts}  -> nearest {nearest[0]}'


def _print_menu(node, poses, labels, world):
    if not poses:
        print('  (no objects on /detected_objects yet -- '
              'check perception.launch.py)')
        return
    for i, p in enumerate(poses):
        pos = p.position
        label = labels.get(i, '?')
        print(f'  [{i}] {label:<12} '
              f'x={pos.x:+.2f} y={pos.y:+.2f} z={pos.z:+.2f}   '
              f'{_fmt_dist(node, p, world)}')


def _loop(node: PickCli):
    print('\n=== pick_cli ===  (Enter=refresh, q=quit)')
    print('dist = straight-line object->current arm tool (estimate, '
          'not energy J)')
    while rclpy.ok():
        poses, labels, world = node.snapshot()
        print('\nDetected objects:')
        _print_menu(node, poses, labels, world)
        try:
            raw = input(f'Select object [0-{max(len(poses) - 1, 0)}]: ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if raw.lower() in ('q', 'quit', 'exit'):
            break
        if raw == '':
            continue                     # refresh the menu
        if not raw.isdigit():
            print(f'  ! "{raw}" is not a number')
            continue
        idx = int(raw)
        if idx >= len(poses):
            print(f'  ! index {idx} out of range '
                  f'(only {len(poses)} objects)')
            continue
        node.send_pick(idx)
        print(f'  -> pick object {idx} sent to {node._topic} '
              f'(see executor terminal for the result)')


def main():
    rclpy.init()
    node = PickCli()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        _loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
