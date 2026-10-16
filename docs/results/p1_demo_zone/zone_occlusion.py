#!/usr/bin/env python3
"""DEMO-ZONE langkah 2 (LURING): apakah lengan REST (arm_1/arm_2) menutupi tanda zona di citra rgbd2 pada g1 1.40 (sekarang)
vs 1.50 (saat tanda dibuat)?
Jejak botol di citra = proyeksi TRACK lama (incumbent) lewat T_wc incumbent -> piksel yang SAMA dengan saat rekam (localizer
mendeproyeksi dengan T_wc itu). Lengan = geometri tabrakan URDF REST (g63d_common.robot_points) diproyeksi lewat T_wc yang sama;
karena galat rgbd2 jarak jauh (DEMO-HW B1: titik kamera - URDF = arm_1 +0.15/+0.02, arm_2 +0.06/+0.02 m), lengan dinilai 2 cara:
  'urdf' (tanpa geser) dan 'geser' (titik lengan digeser galat terukur tsb = tempat lengan TAMPAK di citra). pad 3 cm.
Validasi: overlay pada frame nyata g1 1.40 (g63b B-R0_rgbd2.png, arm_1/arm_2 REST) -> PNG lokal (frame bisa memuat orang:
JANGAN commit *_real.png).      python3 zone_occlusion.py
"""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g63', 'g63d'))
import g63d_common as G  # noqa: E402

import zone_data as Z  # noqa: E402

SHIFT = {'arm_1': (0.15, 0.02, 0.0), 'arm_2': (0.06, 0.02, 0.0)}    # DEMO-HW B1 (rgbd2, REST, z 1.15-1.45)
PAD = 0.03
R_BOT, H_BOT, TABLE = 0.03, 0.07, 0.77
REAL = os.path.join(HERE, '..', 'p1_g63', 'g63b', 'hw_20261016_finger', 'raw', 'B-R0_rgbd2.png')


def arm_of(name):
    for a, p in (('arm_1', 't1_a1_'), ('arm_2', 't1_a2_')):
        if name.startswith(p):
            return a
    return None


def q_at(g1):
    model, q = G.rest_q()
    q[model.joints[model.getJointId('t1_linear_joint')].idx_q] = g1
    return model, q


def masks(cp, g1):
    model, q = q_at(g1)
    pts = G.robot_points(q, model)
    out = {}
    for mode in ('urdf', 'geser'):
        p2 = {}
        for k, v in pts.items():
            a = arm_of(k)
            if a is None:
                continue                      # hanya arm_1/arm_2 (lengan + jari) yang bisa menutup zona
            p2[k] = v + (np.array(SHIFT[a]) if mode == 'geser' else 0)
        out[mode] = G.robot_mask('rgbd2', cp, p2, PAD)
    return out


def bottle_px(cp, xyz):
    """Piksel jejak botol: silinder r 3 cm, z meja..meja+7 cm, berpusat di track xy (incumbent)."""
    a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    P = np.array([[xyz[0] + R_BOT * np.cos(t), xyz[1] + R_BOT * np.sin(t), z] for t in a for z in (TABLE, TABLE + H_BOT)])
    c = cp['rgbd2']
    uv, d = G.project(c['T_wc'], c['K'], c['dist'], P)
    m = np.zeros((G.IMG_H, G.IMG_W), np.uint8)
    cv2.fillConvexPoly(m, cv2.convexHull(uv.astype(np.float32)).astype(np.int32), 255)
    return m, uv.mean(0)


def main():
    cp = G.cam_params()
    obs = [o for o in Z.load() if o['ds'] == 'A']           # botol demo 7 cm, meja 0.77, track incumbent
    # sanity: T_wc rec = harness cam_up.sh rgbd2 (incumbent + G34)
    print('T_wc rgbd2 t', cp['rgbd2']['T_wc'][:3, 3].round(4), 'vs C_INC', Z.C_INC)
    M = {g1: masks(cp, g1) for g1 in (1.40, 1.50)}
    res = {}
    print('%-4s %12s %10s | %s' % ('tanda', 'piksel', 'n px', '  '.join('%s@%.2f' % (m, g) for g in M for m in M[g])))
    for o in obs:
        bm, c = bottle_px(cp, o['trk'])
        n = int((bm > 0).sum())
        cov = {'%s@%.2f' % (m, g): float(((bm > 0) & (M[g][m] > 0)).sum() / max(n, 1)) for g in M for m in M[g]}
        res[o['mark']] = dict(uv=c.round(1).tolist(), n_px=n, cover=cov)
        print('%-5s (%5.0f,%4.0f) %8d | %s' % (o['mark'], c[0], c[1], n, '  '.join('%10.0f%%' % (100 * v) for v in cov.values())))
    json.dump(res, open(os.path.join(HERE, 'zone_occlusion.json'), 'w'), indent=1)
    # gambar: model (aman di-commit) + frame nyata (lokal saja)
    real = cv2.imread(REAL)
    for name, base in (('zone_occlusion_model.png', np.full((G.IMG_H, G.IMG_W, 3), 40, np.uint8)),
                       ('zone_occlusion_real.png', real)):
        if base is None:
            print('TIDAK ADA frame nyata', REAL)
            continue
        img = base.copy()
        for g1, col in ((1.50, (255, 128, 0)), (1.40, (0, 0, 255))):
            for mode, th in (('urdf', 1), ('geser', 2)):
                cs, _ = cv2.findContours(M[g1][mode], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, cs, -1, col, th)
        for o in obs:
            bm, c = bottle_px(cp, o['trk'])
            cs, _ = cv2.findContours(bm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cs, -1, (0, 255, 0), 2)
            cv2.putText(img, o['mark'], (int(c[0]) + 8, int(c[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(img, 'merah g1 1.40, biru g1 1.50; tipis urdf, tebal geser galat; hijau botol (track lama)', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imwrite(os.path.join(HERE, name), img)
        print('tulis', name)


if __name__ == '__main__':
    main()
