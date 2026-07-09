"""Interactive target picker for reach_fusion.

A small control terminal (like pick_cli): it lists the live objects by their
Isaac prim name obj_0..obj_6 (GROUND TRUTH, resolved by reverse-projecting each
/detected_objects centroid into the raw instance-segmentation image) with their
world positions, and lets you type an obj_N / class label / index to switch the
reach_fusion grasp target -- no need to stop and re-run reach_fusion.

Selecting publishes the value on /reach_fusion/set_target, which reach_fusion
reads live.

    ros2 run reachability_gng reach_fusion     # backend (terminal 1)
    ros2 run reachability_gng target_cli       # this menu (terminal 2 OR same)
"""
from __future__ import annotations

import json
import threading

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R


class TargetCli(Node):
    def __init__(self):
        super().__init__('target_cli')
        self.declare_parameter('set_target_topic', '/reach_fusion/set_target')
        self.declare_parameter('camera_namespaces', ['rgbd', 'rgbd2'])
        self.declare_parameter('optical_frame_suffix', '_camera_optical')
        self.declare_parameter('isaac_labels_topic',
                               '/rgbd/instance_segmentation_labels')
        self.declare_parameter('seg_image_suffix', 'instance_segmentation')

        self.cams = list(self.get_parameter('camera_namespaces').value)
        self.optical_suffix = self.get_parameter('optical_frame_suffix').value
        self._lock = threading.Lock()
        self.poses = np.empty((0, 3))
        self.id2objn = {}
        self.K = {}
        self.seg = {}
        self.last_sent = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseArray, '/detected_objects',
                                 self._on_objects, 1)
        self.create_subscription(
            String, self.get_parameter('isaac_labels_topic').value,
            self._on_isaac_labels, 1)
        seg_suffix = self.get_parameter('seg_image_suffix').value
        for ns in self.cams:
            self.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, n=ns: self.K.__setitem__(
                    n, (m.k[0], m.k[4], m.k[2], m.k[5])), 1)
            self.create_subscription(
                Image, f'/{ns}/{seg_suffix}',
                lambda m, n=ns: self.seg.__setitem__(n, self._decode_seg(m)), 1)
        self.pub = self.create_publisher(
            String, self.get_parameter('set_target_topic').value, 1)

    def _on_objects(self, msg):
        with self._lock:
            self.poses = np.array([[p.position.x, p.position.y, p.position.z]
                                   for p in msg.poses]) if msg.poses \
                else np.empty((0, 3))

    def _on_isaac_labels(self, msg):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.id2objn.update({k: v.rstrip('/').split('/')[-1]
                             for k, v in d.items()
                             if isinstance(v, str) and 'obj_' in v})

    @staticmethod
    def _decode_seg(msg):
        a = np.frombuffer(bytes(msg.data), dtype=np.int32)
        return a.reshape(msg.height, msg.step // 4)[:, :msg.width]

    def _instance_at(self, ns, P):
        if ns not in self.K or ns not in self.seg:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                f'{ns}{self.optical_suffix}', 'world',
                rclpy.time.Time()).transform
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t, q = tf.translation, tf.rotation
        pc = quat_to_R(q.x, q.y, q.z, q.w) @ P + np.array([t.x, t.y, t.z])
        if pc[2] <= 0.05:
            return None
        fx, fy, cx, cy = self.K[ns]
        u = int(fx * pc[0] / pc[2] + cx)
        v = int(fy * pc[1] / pc[2] + cy)
        seg = self.seg[ns]
        if 0 <= v < seg.shape[0] and 0 <= u < seg.shape[1]:
            return int(seg[v, u])
        return None

    def objn_map(self):
        """{obj_N: (index, xyz)} by reverse-projecting each detected centroid."""
        with self._lock:
            poses = self.poses.copy()
            id2objn = dict(self.id2objn)
        out = {}
        for i, P in enumerate(poses):
            for ns in self.cams:
                iid = self._instance_at(ns, P)
                objn = id2objn.get(str(iid)) if iid is not None else None
                if objn:
                    out[objn] = (i, P)
                    break
        return out, len(poses)

    def send(self, value):
        self.pub.publish(String(data=value))
        self.last_sent = value


def _loop(node: TargetCli):
    print('\n=== target_cli ===  (Enter=refresh, q=quit)')
    print('type obj_N (ground truth) / a class label / an index to set the '
          'reach_fusion target')
    while rclpy.ok():
        objn, n = node.objn_map()
        print(f'\nDetected objects ({n} total)'
              + (f'  [current target: {node.last_sent}]'
                 if node.last_sent else ''))
        if not objn:
            print('  (no obj_N resolved yet -- is reach_fusion perception + '
                  'seg_source:=isaac up?)')
        for name in sorted(objn):
            idx, p = objn[name]
            print(f'  {name}  (index {idx})  '
                  f'x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f}')
        try:
            raw = input('Set target> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if raw.lower() in ('q', 'quit', 'exit'):
            break
        if raw == '':
            continue
        node.send(raw)
        print(f'  -> sent "{raw}" to reach_fusion')


def main():
    rclpy.init()
    node = TargetCli()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        _loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
