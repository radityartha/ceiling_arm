#!/usr/bin/env python3
"""G63 S0c: grab one live colour frame per camera and draw the NEW-voxel clusters
(g63_s0c.py *_clusters.npy) on it, projected with the camera TF stored in the capture.

    ROS_DOMAIN_ID=80 python3 g63_s0c_proj.py --capture hw_.../s0c_capture.npz \
        --clusters hw_.../s0c_report_clusters.npy --report hw_.../s0c_report.json --outdir hw_...
"""
from __future__ import annotations

import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture', required=True)
    ap.add_argument('--clusters', required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--outdir', required=True)
    ap.add_argument('--top', type=int, default=8, help='label the N clusters nearest to poses')
    a = ap.parse_args()
    cap = np.load(a.capture)
    vox = np.load(a.clusters)
    rep = json.load(open(a.report))
    top = [c['idx'] for c in rep['clusters'][:a.top]]

    rclpy.init()
    n = rclpy.create_node('g63_s0c_proj')
    got, br = {}, CvBridge()
    for ns in ('rgbd', 'rgbd2'):
        n.create_subscription(Image, f'/{ns}/{ns}/color/image_raw',
                              lambda m, ns=ns: got.setdefault(('img', ns), m), 1)
        n.create_subscription(CameraInfo, f'/{ns}/{ns}/color/camera_info',
                              lambda m, ns=ns: got.setdefault(('ci', ns), m), 1)
    t0 = time.time()
    while len(got) < 4 and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()
    for ns in ('rgbd', 'rgbd2'):
        if ('img', ns) not in got or ('ci', ns) not in got:
            print(f'{ns}: TIDAK ada citra/camera_info -- dilewati')
            continue
        img = cv2.cvtColor(br.imgmsg_to_cv2(got[('img', ns)], 'rgb8'), cv2.COLOR_RGB2BGR)
        K = np.array(got[('ci', ns)].k).reshape(3, 3)
        tf = cap[f'{ns}_tf']                      # world <- optical: xyz + quat xyzw
        R = Rotation.from_quat(tf[3:]).as_matrix()
        pc = (vox[:, :3] - tf[:3]) @ R            # world -> optical
        ok = pc[:, 2] > 0.1
        uv = (pc[ok] @ K.T)
        uv = uv[:, :2] / uv[:, 2:3]
        lab = vox[ok, 3].astype(int)
        h, w = img.shape[:2]
        for (u, v), k in zip(uv, lab):
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(img, (int(u), int(v)), 2, (0, 0, 255) if k in top else (0, 200, 255), -1)
        for k in top:
            m = lab == k
            if not m.any():
                continue
            u, v = uv[m].mean(0)
            if 0 <= u < w and 0 <= v < h:
                cv2.putText(img, f'#{k}', (int(u) + 5, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        out = os.path.join(a.outdir, f's0c_{ns}.jpg')
        cv2.imwrite(out, img)
        print('saved', out)


if __name__ == '__main__':
    main()
