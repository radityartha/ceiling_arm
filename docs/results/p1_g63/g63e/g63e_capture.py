#!/usr/bin/env python3
"""G63e tangkap KAMERA SAJA (NOL gerak lengan/rel). Frame berisi orang -> LOKAL, tidak di-commit.

  tags  : N frame berstempel unik per kamera (g63d_capture.capture) -> deteksi penanda TANPA prior (ID 20..35, tanpa batas px)
          -> per kamera per ID: laju per frame, sisi px (median), std sudut px (kestabilan), alasan tolak. Laporan DIR/tags.json.
  scan  : N frame depth TERALIGN ke warna per kamera (16UC1 mm) -> median per piksel (0 = tak valid, perlu >= 50 % frame)
          -> DIR/scan_<cam>.npz (depth_m, valid_frac, K warna). Awan dunia = luring (K + incumbent), untuk bidang tetap.
  board : loop papan polos GENGGAM. Tiap pasangan frame terbaru dua kamera -> detect_board_free. Pose DISIMPAN otomatis bila
          kedua kamera mendeteksi STILL_N deteksi berturut-turut dengan geser sudut maks <= STILL_PX (papan diam), dan pose
          berbeda dari pose tersimpan sebelumnya (pusat bergeser >= NEW_PX di salah satu kamera). Cetak + bel. Simpan
          DIR/board/NN_<cam>.png + DIR/board/poses.jsonl (uv median, stempel, std).

    python3 g63e_capture.py tags  --out DIR [--frames 30]
    python3 g63e_capture.py scan  --out DIR [--frames 30]
    python3 g63e_capture.py board --out DIR [--n 5] [--timeout 600]
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63e_common as E  # noqa: E402
import g63e_detect as D  # noqa: E402

STILL_N, STILL_PX, NEW_PX = 3, 0.5, 40.0
SQ_NOM = 0.041


def _to_bgr(msg):
    """= g63_capture._img_to_gray (terbukti HW); cv_bridge Humble tidak cocok dengan cv2 5.0 lokal (KeyError 16)."""
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return a[:, :, ::-1].copy()
    if msg.encoding == 'bgr8':
        return a.copy()
    raise ValueError('encoding %s tidak didukung' % msg.encoding)


def capture(out, n, timeout):
    """N frame berstempel unik per kamera + camera_info -> DIR/<cam>/NN_<stamp>.png, DIR/<cam>/camera_info.json."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = rclpy.create_node('g63e_capture')
    frames = {c: [] for c in E.CAMS}
    seen = {c: set() for c in E.CAMS}
    info = {}

    def on_img(c):
        def cb(m):
            st = (m.header.stamp.sec, m.header.stamp.nanosec)
            if st in seen[c] or len(frames[c]) >= n:
                return
            seen[c].add(st)
            frames[c].append((st, _to_bgr(m)))
        return cb
    for c in E.CAMS:
        node.create_subscription(Image, f'/{c}/{c}/color/image_raw', on_img(c), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/{c}/{c}/color/camera_info', lambda m, c=c: info.__setitem__(c, m),
                                 qos_profile_sensor_data)
    t0 = time.time()
    while (any(len(frames[c]) < n for c in E.CAMS) or len(info) < 2) and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    rc = 0
    for c in E.CAMS:
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
                           width=ci.width, height=ci.height, stamps=['%d.%09d' % st for st, _ in frames[c]],
                           t_wall=time.time()), open(os.path.join(d, 'camera_info.json'), 'w'), indent=1)
        print('%s: %d frame' % (c, len(frames[c])), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return rc


def scan(out, n, timeout):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = rclpy.create_node('g63e_scan')
    frames, info, seen = {c: [] for c in E.CAMS}, {}, {c: set() for c in E.CAMS}

    def cb(c):
        def f(m):
            st = (m.header.stamp.sec, m.header.stamp.nanosec)
            if st in seen[c] or len(frames[c]) >= n or m.encoding != '16UC1':
                return
            seen[c].add(st)
            frames[c].append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width].copy())
        return f
    for c in E.CAMS:
        node.create_subscription(Image, f'/{c}/{c}/aligned_depth_to_color/image_raw', cb(c), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/{c}/{c}/color/camera_info', lambda m, c=c: info.__setitem__(c, m),
                                 qos_profile_sensor_data)
    t0 = time.time()
    while (any(len(frames[c]) < n for c in E.CAMS) or len(info) < 2) and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    rc = 0
    os.makedirs(out, exist_ok=True)
    for c in E.CAMS:
        if c not in info or len(frames[c]) < n:
            print('GAGAL: %s depth %d/%d info %s' % (c, len(frames[c]), n, c in info), flush=True)
            rc = 2
            continue
        F = np.array(frames[c]).astype(np.float32) / 1000.0
        F[F <= 0] = np.nan
        vf = np.mean(np.isfinite(F), 0)
        med = np.nanmedian(F, 0)
        med[vf < 0.5] = 0.0
        np.savez_compressed(os.path.join(out, 'scan_%s.npz' % c), depth_m=med.astype(np.float32), valid_frac=vf.astype(np.float32),
                            K=np.array(info[c].k).reshape(3, 3), dist=np.array(info[c].d), t_wall=time.time(), n=len(frames[c]))
        print('%s: %d frame depth, valid %.2f' % (c, len(frames[c]), float(np.mean(med > 0))), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return rc


def tags_report(out):
    import glob
    rep = {}
    for c in E.CAMS:
        fr = sorted(glob.glob(os.path.join(out, c, '*.png')))
        acc, why = {}, {}
        for f in fr:
            g = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
            got, w = D.detect_tags_frame(g, None, min_px=0)
            for r, n in w.items():
                why[r] = why.get(r, 0) + n
            for i, uv in got.items():
                acc.setdefault(i, []).append(uv)
        rc = {}
        for i, L in sorted(acc.items()):
            U = np.array(L)
            side = np.median([np.linalg.norm(u - np.roll(u, 1, 0), axis=1).min() for u in U])
            rc[i] = dict(rate=round(len(L) / max(1, len(fr)), 3), side_px=round(float(side), 1),
                         std_px=round(float(U.std(0).max()), 3), center=np.median(U, 0).mean(0).round(1).tolist())
            print('%s ID %d: laju %d/%d  sisi %.1f px  std sudut %.3f px' % (c, i, len(L), len(fr), side, rc[i]['std_px']))
        print('%s alasan tolak: %s' % (c, why))
        rep[c] = dict(n_frames=len(fr), tags=rc, rejects=why)
    json.dump(rep, open(os.path.join(out, 'tags.json'), 'w'), indent=1)
    return rep


def board_loop(out, n, timeout):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('g63e_board')
    latest = {}

    def cb(c):
        def f(m):
            latest[c] = (m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m)
        return f
    for c in E.CAMS:
        node.create_subscription(Image, f'/{c}/{c}/color/image_raw', cb(c), qos_profile_sensor_data)
    d = os.path.join(out, 'board')
    os.makedirs(d, exist_ok=True)
    hist = {c: [] for c in E.CAMS}
    saved, last_ctr, seen = [], None, {c: None for c in E.CAMS}
    t0 = time.time()
    print('Pegang papan DIAM di volume tengah (z 0.4-1.1 m), menghadap kedua kamera. Bel = tersimpan, lalu pindah pose.', flush=True)
    while len(saved) < n and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        for c in E.CAMS:
            if c not in latest or latest[c][0] == seen[c]:
                continue
            seen[c] = latest[c][0]
            img = _to_bgr(latest[c][1])
            uv, note = D.detect_board_free(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), SQ_NOM)
            hist[c] = (hist[c] + [(seen[c], uv, img)])[-STILL_N:] if uv is not None else []
        if not all(len(hist[c]) == STILL_N for c in E.CAMS):
            continue
        still = {c: float(max(np.abs(h[1] - hist[c][0][1]).max() for h in hist[c])) for c in E.CAMS}
        if max(still.values()) > STILL_PX:
            continue
        ctr = {c: np.median([h[1] for h in hist[c]], 0) for c in E.CAMS}
        if last_ctr is not None and max(np.linalg.norm(ctr[c].mean(0) - last_ctr[c].mean(0)) for c in E.CAMS) < NEW_PX:
            continue
        k = len(saved)
        rec = dict(k=k, t_wall=time.time(), still_px=still)
        for c in E.CAMS:
            cv2.imwrite(os.path.join(d, '%02d_%s.png' % (k, c)), hist[c][-1][2])
            rec[c] = dict(uv=ctr[c].tolist(), stamps=[h[0] for h in hist[c]])
        open(os.path.join(d, 'poses.jsonl'), 'a').write(json.dumps(rec) + '\n')
        saved.append(rec)
        last_ctr = ctr
        hist = {c: [] for c in E.CAMS}
        print('\aPOSE %d/%d TERSIMPAN (diam %s px)' % (k + 1, n, {c: round(v, 2) for c, v in still.items()}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    print('selesai: %d pose dalam %.0f s' % (len(saved), time.time() - t0), flush=True)
    return 0 if len(saved) >= n else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=('tags', 'scan', 'board'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', type=int, default=30)
    ap.add_argument('--n', type=int, default=5)
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--no-capture', action='store_true', help='tags: hanya analisis frame yang sudah ada')
    a = ap.parse_args()
    if a.mode == 'tags':
        if not a.no_capture:
            rc = capture(a.out, a.frames, 20.0)
            if rc:
                return rc
        tags_report(a.out)
        return 0
    if a.mode == 'scan':
        return scan(a.out, a.frames, 30.0)
    return board_loop(a.out, a.n, a.timeout)


if __name__ == '__main__':
    sys.exit(main())
