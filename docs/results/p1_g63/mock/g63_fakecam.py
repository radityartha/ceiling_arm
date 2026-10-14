#!/usr/bin/env python3
"""G63 end-to-end test camera for the MOCK (domain 77 only): renders boards A/B from the live arm TF.

TRUTH (hidden from the pipeline): camera extrinsic = incumbent perturbed (--rot-deg, --pos-m, seeded) and board mount
= mount JSON perturbed (--mount-pos-m, --mount-rot-deg). PUBLISHED (what the pipeline sees as prior): static TF
world -> <cam>_camera_optical = INCUMBENT, camera_info = nominal K. The capture/merge/solve chain must recover TRUTH;
the truth is written to --truth-out for the comparison. Refuses to run unless ROS_DOMAIN_ID=77.

    ROS_DOMAIN_ID=77 g63_fakecam.py --mount ../mount_guess.json --truth-out truth.json
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from g63_common import (ARM_PREFIX, BOARD_ARM, CAMERAS, IMG_H, IMG_W, K_NOMINAL, T_from_xyzrpy,  # noqa: E402
                        incumbent_T, inv, xyzrpy_from_T)
import g63_detect  # noqa: E402
from g63_solve import _perturb  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mount', required=True)
    ap.add_argument('--truth-out', required=True)
    ap.add_argument('--rot-deg', type=float, default=2.0)
    ap.add_argument('--pos-m', type=float, default=0.08)
    ap.add_argument('--mount-pos-m', type=float, default=0.02)
    ap.add_argument('--mount-rot-deg', type=float, default=5.0)
    ap.add_argument('--seed', type=int, default=77)
    ap.add_argument('--hz', type=float, default=10.0)
    a = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '77':
        raise SystemExit('fakecam hanya untuk mock domain 77')
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from scipy.spatial.transform import Rotation
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

    rng = np.random.default_rng(a.seed)
    mount = json.load(open(a.mount))
    truth_wc = {c: _perturb(incumbent_T(c), a.rot_deg, a.pos_m, rng) for c in CAMERAS}
    truth_lb = {b: _perturb(T_from_xyzrpy(*mount[b]['xyz'], *mount[b]['rpy']), a.mount_rot_deg, a.mount_pos_m, rng)
                for b in BOARD_ARM}
    json.dump(dict(Twc={c: T.tolist() for c, T in truth_wc.items()},
                   Twc_xyzrpy={c: list(xyzrpy_from_T(T)) for c, T in truth_wc.items()},
                   Tlb={b: T.tolist() for b, T in truth_lb.items()}, args=vars(a)), open(a.truth_out, 'w'), indent=1)

    rclpy.init()
    node = rclpy.create_node('g63_fakecam')
    buf = Buffer()
    TransformListener(buf, node)
    stb = StaticTransformBroadcaster(node)
    tfs = []
    for c in CAMERAS:
        T = incumbent_T(c)
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = 'world', f'{c}_camera_optical'
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in T[:3, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
        tfs.append(m)
    stb.sendTransform(tfs)
    pubs = {c: (node.create_publisher(Image, f'/{c}/{c}/color/image_raw', 5),
                node.create_publisher(CameraInfo, f'/{c}/{c}/color/camera_info', 5)) for c in CAMERAS}

    def tick():
        now = node.get_clock().now()
        links = {}
        for b in BOARD_ARM:
            try:
                tf = buf.lookup_transform('world', ARM_PREFIX[BOARD_ARM[b]] + mount[b]['link'], Time())
            except Exception:                                    # noqa: BLE001
                return
            t, q = tf.transform.translation, tf.transform.rotation
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [t.x, t.y, t.z]
            links[b] = T
        for c in CAMERAS:
            img = np.full((IMG_H, IMG_W), 110, np.uint8)
            for b in BOARD_ARM:
                Tcb = inv(truth_wc[c]) @ links[b] @ truth_lb[b]
                ctr = Tcb[:3, :3] @ np.array([0.1, 0.125, 0]) + Tcb[:3, 3]
                if ctr[2] < 0.4 or Tcb[2, 2] < 0.2:          # behind the camera / seen edge-on or from the back
                    continue
                try:
                    img = g63_detect.render(b, Tcb, K_NOMINAL[c], rng=rng, bg=img, margin_m=0.005)
                except Exception:                                # noqa: BLE001
                    pass
            bgr = np.repeat(img[:, :, None], 3, 2)
            msg = Image(height=IMG_H, width=IMG_W, encoding='bgr8', step=IMG_W * 3, data=bgr.tobytes())
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = f'{c}_camera_optical'
            info = CameraInfo(height=IMG_H, width=IMG_W, distortion_model='plumb_bob', d=[0.0] * 5,
                              k=K_NOMINAL[c].ravel().tolist(),
                              p=np.c_[K_NOMINAL[c], np.zeros(3)].ravel().tolist())
            info.header = msg.header
            pubs[c][0].publish(msg)
            pubs[c][1].publish(info)
    node.create_timer(1.0 / a.hz, tick)
    rclpy.spin(node)


if __name__ == '__main__':
    sys.exit(main())
