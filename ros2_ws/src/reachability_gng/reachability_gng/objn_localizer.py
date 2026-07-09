"""Ground-truth object identity (Isaac prim obj_N) via reverse projection.

Maps prim names obj_N -> /detected_objects centroids by projecting each centroid
into the raw instance-segmentation image and reading the instance_id there.
`id -> obj_N` (raw Isaac labels) is stable across frames; seg_router's class
names are NOT (they swap), so we bypass them. Shared by reach_fusion & target_cli.
"""
from __future__ import annotations

import json

import numpy as np
import rclpy.time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from reachability_gng.object_localizer import quat_to_R


def _decode(msg: Image):
    a = np.frombuffer(bytes(msg.data), dtype=np.int32)
    return a.reshape(msg.height, msg.step // 4)[:, :msg.width]


class ObjnLocalizer:
    """Creates its camera/label/TF subscriptions on `node`; resolves obj_N."""

    def __init__(self, node, cams=('rgbd', 'rgbd2'),
                 optical_suffix='_camera_optical',
                 seg_suffix='instance_segmentation',
                 labels_topic='/rgbd/instance_segmentation_labels',
                 world_frame='world'):
        self.cams = list(cams)
        self.optical_suffix = optical_suffix
        self.world_frame = world_frame
        self.K, self.seg, self.id2objn = {}, {}, {}
        self.tf = Buffer()
        self._listener = TransformListener(self.tf, node)
        node.create_subscription(String, labels_topic, self._on_labels, 1)
        for ns in self.cams:
            node.create_subscription(
                CameraInfo, f'/{ns}/camera_info',
                lambda m, n=ns: self.K.__setitem__(
                    n, (m.k[0], m.k[4], m.k[2], m.k[5])), 1)
            node.create_subscription(
                Image, f'/{ns}/{seg_suffix}',
                lambda m, n=ns: self.seg.__setitem__(n, _decode(m)), 1)

    def _on_labels(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.id2objn.update({k: v.rstrip('/').split('/')[-1] for k, v in d.items()
                             if isinstance(v, str) and 'obj_' in v})

    def _instance_at(self, ns, P):
        """Instance id under world point P in camera `ns`'s seg image, or None."""
        if ns not in self.K or ns not in self.seg:
            return None
        try:
            tf = self.tf.lookup_transform(f'{ns}{self.optical_suffix}',
                                          self.world_frame,
                                          rclpy.time.Time()).transform
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t, q = tf.translation, tf.rotation
        pc = quat_to_R(q.x, q.y, q.z, q.w) @ P + np.array([t.x, t.y, t.z])
        if pc[2] <= 0.05:
            return None
        fx, fy, cx, cy = self.K[ns]
        u, v = int(fx * pc[0] / pc[2] + cx), int(fy * pc[1] / pc[2] + cy)
        s = self.seg[ns]
        return int(s[v, u]) if 0 <= v < s.shape[0] and 0 <= u < s.shape[1] else None

    def objn_of(self, P):
        """obj_N name at centroid P (tries every camera), or None."""
        for ns in self.cams:
            iid = self._instance_at(ns, P)
            if iid is not None and str(iid) in self.id2objn:
                return self.id2objn[str(iid)]
        return None

    def find(self, objn, poses):
        """Centroid of prim `objn` among `poses`, or None."""
        for P in poses:
            if self.objn_of(P) == objn:
                return P
        return None

    def map(self, poses):
        """{obj_N: (index, xyz)} for the given centroids."""
        out = {}
        for i, P in enumerate(poses):
            name = self.objn_of(P)
            if name:
                out[name] = (i, P)
        return out
