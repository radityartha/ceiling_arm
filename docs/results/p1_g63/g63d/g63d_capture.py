#!/usr/bin/env python3
"""G63d tangkap KAMERA SAJA (NOL gerak): N frame berstempel unik per kamera + camera_info -> DIR/<cam>/NN_<stamp>.png,
DIR/<cam>/camera_info.json. Frame berisi orang -> LOKAL, tidak di-commit.
--check PLAN: sesudah tangkap, jalankan detektor terkunci + penjaga ID pada frame dengan prior dari rencana (pose papan
rencana, galat penempatan +-10 cm) -> laporan per papan (cek penempatan SEBELUM pita; bukan data kalibrasi).

    python3 g63d_capture.py --out DIR [--frames 30] [--timeout 20] [--check g63d_plan.json]
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63d_common as G  # noqa: E402


def capture(out, n, timeout):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = rclpy.create_node('g63d_capture')
    br = CvBridge()
    frames = {c: [] for c in G.CAMS}
    seen = {c: set() for c in G.CAMS}
    info = {}

    def on_img(c):
        def cb(m):
            st = (m.header.stamp.sec, m.header.stamp.nanosec)
            if st in seen[c] or len(frames[c]) >= n:
                return
            seen[c].add(st)
            frames[c].append((st, br.imgmsg_to_cv2(m, 'bgr8')))
        return cb
    for c in G.CAMS:
        node.create_subscription(Image, f'/{c}/{c}/color/image_raw', on_img(c), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/{c}/{c}/color/camera_info', lambda m, c=c: info.__setitem__(c, m),
                                 qos_profile_sensor_data)
    t0 = time.time()
    while (any(len(frames[c]) < n for c in G.CAMS) or len(info) < 2) and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    rc = 0
    for c in G.CAMS:
        d = os.path.join(out, c)
        os.makedirs(d, exist_ok=True)
        if c not in info or len(frames[c]) < n:
            print('GAGAL: %s frame %d/%d info %s' % (c, len(frames[c]), n, c in info), flush=True)
            rc = 2
        for i, (st, img) in enumerate(frames[c]):
            cv2.imwrite(os.path.join(d, '%02d_%d.%09d.png' % (i, *st)), img)
        if c in info:
            ci = info[c]
            json.dump(dict(K=np.array(ci.k).reshape(3, 3).tolist(), dist=list(ci.d), frame_id=ci.header.frame_id,
                           width=ci.width, height=ci.height,
                           stamps=['%d.%09d' % st for st, _ in frames[c]], t_wall=time.time()),
                      open(os.path.join(d, 'camera_info.json'), 'w'), indent=1)
        span = (frames[c][-1][0][0] + frames[c][-1][0][1] * 1e-9 - frames[c][0][0][0] - frames[c][0][0][1] * 1e-9) if frames[c] else 0
        print('%s: %d frame, rentang stempel %.2f s' % (c, len(frames[c]), span), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return rc


def check(out, plan_path):
    import glob
    import g63d_solve as S
    plan = json.load(open(plan_path))
    sq = plan['sq']
    boards = {}
    for b in plan['boards']:
        k1, k2 = np.array(b['K1'][:2]), np.array(b['K2'][:2])
        boards[b['name']] = dict(T=G.pose_from_k(k1, k2, b['center'][2])[0])
    cp = G.cam_params()
    for c in G.CAMS:
        info = json.load(open(os.path.join(out, c, 'camera_info.json')))
        fr = sorted(glob.glob(os.path.join(out, c, '*.png')))
        obs = S.detect_cam(fr, boards, cp[c]['T_wc'], np.array(info['K']), np.asarray(info['dist']), sq)
        for b in plan['boards']:
            o = obs.get(b['name'], {})
            print('%s %s rencana %s: lolos %s/%s %s' % (c, b['name'], 'TERLIHAT' if b['vis'][c]['ok'] else '-',
                                                     o.get('n_ok', 0), o.get('n', 0), o.get('notes', '')), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', type=int, default=30)
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--check', default=None)
    ap.add_argument('--no-capture', action='store_true', help='hanya --check pada frame yang sudah ada')
    a = ap.parse_args()
    rc = 0 if a.no_capture else capture(a.out, a.frames, a.timeout)
    if a.check and rc == 0:
        check(a.out, a.check)
    return rc


if __name__ == '__main__':
    sys.exit(main())
