#!/usr/bin/env python3
"""G33 A1: record /<ns>/depth_cloud (frame `world`) for N seconds per camera and save
it de-duplicated at 1 cm, with the number of FRAMES each 1 cm cell was seen in.

Saved (np.savez_compressed): for each ns -> `<ns>_xyz` (cell centres, float32),
`<ns>_frames` (int32), `<ns>_nframes`, `<ns>_tf` (world->optical xyz+quat at capture),
plus `t_start`, `t_end`, `seconds`, `cell`. The map is built OFFLINE from this file
(build_env_map.py) -- the cameras are not needed again.

    python3 capture_cloud.py --out docs/results/p1_g33/capture_a.npz --seconds 12
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from tf2_ros import Buffer, TransformListener

CELL = 0.01


class Cap(Node):
    def __init__(self, nss):
        super().__init__('g33_capture')
        self.keys = {ns: [] for ns in nss}
        # /joint_states has TWO publishers (arms, gantry bridge) -> merge by name;
        # the robot config at capture time is what build_env_map self-filters with.
        self.js, self.js_t, self.js_lo, self.js_hi = {}, {}, {}, {}
        self.create_subscription(JointState, '/joint_states', self._js, 50)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        for ns in nss:
            self.create_subscription(PointCloud2, f'/{ns}/depth_cloud',
                                     lambda m, ns=ns: self._cb(ns, m),
                                     qos_profile_sensor_data)

    def _js(self, m):
        t = time.time()
        for k, v in zip(m.name, m.position):
            self.js[k], self.js_t[k] = v, t
            self.js_lo[k] = min(v, self.js_lo.get(k, v))
            self.js_hi[k] = max(v, self.js_hi.get(k, v))

    def _cb(self, ns, m):
        if m.header.frame_id != 'world' or m.point_step != 12:
            self.get_logger().error(f'{ns}: frame {m.header.frame_id!r} '
                                    f'step {m.point_step} -- not the depth_cloud contract')
            return
        p = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, 3)
        p = p[np.isfinite(p).all(1)]
        self.keys[ns].append(np.unique(np.floor(p / CELL).astype(np.int32), axis=0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--ns', nargs='+', default=['rgbd', 'rgbd2'])
    ap.add_argument('--joints', action='store_true',
                    help='also record /joint_states (robot config at capture)')
    a = ap.parse_args()

    rclpy.init()
    n = Cap(a.ns)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.1)
    t1 = time.time()

    out = {'t_start': t0, 't_end': t1, 'seconds': t1 - t0, 'cell': CELL}
    bad = []
    if a.joints:
        names = sorted(n.js)
        if not names or max(t1 - n.js_t[k] for k in names) > 2.0:
            bad.append(f'joint_states kosong/basi ({len(names)} sendi)')
        out['joint_names'] = np.array(names)
        out['joint_pos'] = np.array([n.js[k] for k in names])
        span = {k: n.js_hi[k] - n.js_lo[k] for k in names}
        moved = {k: round(v, 4) for k, v in span.items() if v > 0.009}  # 0.5 deg / 9 mm
        if moved:
            bad.append(f'robot BERGERAK saat tangkap: {moved}')
        print(f'joint_states: {len(names)} sendi, rentang maks {max(span.values()):.5f}')
    for ns in a.ns:
        fr = n.keys[ns]
        try:
            tf = n.tfb.lookup_transform('world', f'{ns}_camera_optical', rclpy.time.Time())
            tr, q = tf.transform.translation, tf.transform.rotation
            out[f'{ns}_tf'] = np.array([tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w])
        except Exception as e:  # noqa: BLE001
            bad.append(f'{ns}: no TF ({e})')
        if len(fr) < 10:
            bad.append(f'{ns}: only {len(fr)} frames')
            continue
        k, c = np.unique(np.concatenate(fr), axis=0, return_counts=True)
        out[f'{ns}_xyz'] = ((k + 0.5) * CELL).astype(np.float32)
        out[f'{ns}_frames'] = c.astype(np.int32)
        out[f'{ns}_nframes'] = len(fr)
        print(f'{ns}: {len(fr)} frames, {len(k)} cells @1cm, '
              f'bbox {out[f"{ns}_xyz"].min(0).round(2)} .. {out[f"{ns}_xyz"].max(0).round(2)}')
    n.destroy_node()
    rclpy.shutdown()
    if bad:
        print('TOLAK:', '; '.join(bad))
        sys.exit(2)
    np.savez_compressed(a.out, **out)
    print('saved', a.out)


if __name__ == '__main__':
    main()
