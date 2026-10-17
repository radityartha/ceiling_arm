#!/usr/bin/env python3
"""DEMO-ZONE-LIVE: apakah meja demo (pembawa tanda pita) di posisi saat menandai? Banding frame rgbd2 sekarang vs acuan
CAM-6DOF k4_rgbd2.jpg (2026-10-14; data A 10-12 dan B 10-14 konsisten 0.4-2.2 cm -> meja belum digeser saat itu).
Sudut fitur di permukaan meja (acuan) -> template-match di frame sekarang (NCC >= 0.85) -> median geser piksel -> geser dunia di
bidang meja z 0.77 (sinar incumbent + P3; kira-kira). Kontrol: fitur STATIS (lantai/dinding) harus 0 px (kamera tidak bergeser).
    python3 table_check.py [frame.png]     (tanpa arg: ambil 1 frame /rgbd2/rgb, ROS_DOMAIN_ID dari env)
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path[:0] = [os.path.dirname(HERE), R + '/p1_g63/g63d']
REF = R + '/p1_cam6dof/hw_20261014/k4_rgbd2.jpg'
TABLE = np.array([[470, 225], [560, 185], [700, 225], [690, 260], [560, 320], [460, 300]], np.int32)   # permukaan meja (px)
STATIC = [(690, 255)]                                     # kabel lantai; + korelasi fase kiri-atas (dinding/tirai)


def grab():
    import rclpy
    from sensor_msgs.msg import Image
    rclpy.init()
    n = rclpy.create_node('table_check_grab')
    got = {}

    def f(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        got['a'] = a[:, :, ::-1].copy() if m.encoding == 'rgb8' else a.copy()
    n.create_subscription(Image, '/rgbd2/rgb', f, 1)
    import time
    t = time.time()
    while 'a' not in got and time.time() - t < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.shutdown()
    return got['a']


def match(cur, ref, x, y, h=9, r=30):
    t = ref[y - h:y + h + 1, x - h:x + h + 1]
    s = cur[y - h - r:y + h + r + 1, x - h - r:x + h + r + 1]
    res = cv2.matchTemplate(s, t, cv2.TM_CCOEFF_NORMED)
    _, mx, _, loc = cv2.minMaxLoc(res)
    return loc[0] - r, loc[1] - r, mx


def main():
    cur = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else grab()
    ref = cv2.imread(REF)
    g = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    m = np.zeros(g(ref).shape, np.uint8)
    cv2.fillConvexPoly(m, TABLE, 255)
    pts = cv2.goodFeaturesToTrack(g(ref), 40, 0.02, 8, mask=m).reshape(-1, 2).astype(int)
    st = [match(cur, ref, x, y) for x, y in STATIC]
    a, b = [np.float32(g(im)[0:300, 0:400]) for im in (cur, ref)]
    (fx, fy), fr = cv2.phaseCorrelate(b, a, cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F))
    print('STATIS (harus ~0): kabel', [(dx, dy, round(c, 2)) for dx, dy, c in st], 'kiri-atas fase (%+.1f, %+.1f) resp %.2f' % (fx, fy, fr))
    sh = np.array([(dx, dy) for (x, y) in pts for dx, dy, c in [match(cur, ref, x, y)] if c >= 0.85], float)
    print('fitur meja: %d sudut, %d NCC>=0.85' % (len(pts), len(sh)))
    if len(sh) < 3:
        print('TIDAK CUKUP fitur cocok -> tidak bisa dinilai (meja sangat bergeser / botol menutup / cahaya beda)')
        return
    d = np.median(sh, 0)
    import g63d_common as G
    import zone_correct_node as N
    cp = G.cam_params()['rgbd2']
    K, T = cp['K'], cp['T_wc']

    def plane(u, v, z=0.77):
        r = T[:3, :3] @ np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
        return T[:3, 3] + (z - T[2, 3]) / r[2] * r
    c = np.median(pts, 0)
    p3 = N.load_p3()
    w = N.correct_xy(plane(*(c + d))[:2], *p3) - N.correct_xy(plane(*c)[:2], *p3)
    print('geser meja median px (%+.1f, %+.1f), sebaran p10-p90 dx %s dy %s' % (
        d[0], d[1], np.percentile(sh[:, 0], [10, 90]), np.percentile(sh[:, 1], [10, 90])))
    print('=> dunia kira-kira (%+.3f, %+.3f) m, |%.1f cm|' % (w[0], w[1], 100 * np.linalg.norm(w)))


if __name__ == '__main__':
    main()
