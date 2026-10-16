"""G63c LURING: bersama untuk eksperimen penjaga identitas papan + model kinematik (data jari HW 2026-10-16).
Alat terkunci G63/G63b TIDAK diubah -- hanya diimpor."""
import json
import os
import sys

import numpy as np
import pinocchio as pin

G63 = '/home/user1/Documents/ceiling_arm/docs/results/p1_g63'
G63B = G63 + '/g63b'
FING = G63B + '/hw_20261016_finger'
sys.path[:0] = [G63, G63B]
import g63_detect  # noqa: E402
from g63_common import T_from_xyzrpy, board_points  # noqa: E402

URDF = G63B + '/hw_20261016/live2.urdf'      # = URDF hidup sesi jari (dipakai joint_offset_both, FK == TF <= 1 mm)
PRE = {'A': 't1_a2_', 'B': 't2_a1_'}         # papan A di jari arm_2 (gantry 1), B di jari arm_3 (gantry 2)
MOUNT = {k: v for k, v in json.load(open(G63B + '/mount_finger.json')).items() if not k.startswith('_')}
T_LB = {b: T_from_xyzrpy(*MOUNT[b]['xyz'], *MOUNT[b]['rpy']) for b in 'AB'}
SQ = 0.042

model = pin.buildModelFromUrdf(URDF)
data = model.createData()
FID = {b: model.getFrameId(PRE[b] + 'tool_frame') for b in 'AB'}


def records():
    return [json.loads(l) for l in open(FING + '/poses.jsonl')]


def q_of(rec):
    q = pin.neutral(model)
    for n, v in rec['joints'].items():
        if model.existJointName(n):
            q[model.joints[model.getJointId(n)].idx_q] = v
    return q


def T_wl(rec, b):
    pin.framesForwardKinematics(model, data, q_of(rec))
    return data.oMf[FID[b]].homogeneous.copy()


def priors_all(rec, cam):
    """Prior 12x2 uv untuk SEMUA papan (A dan B) dari FK sudut terekam + ekstrinsik hidup + mount, plus kedalaman pusat (m)."""
    c = rec['cams'][cam]
    K, d, Twc = np.array(c['K']), np.asarray(c['dist']), np.array(c['T_wc_live'])
    out = {}
    for b in 'AB':
        Twl = T_wl(rec, b)
        uv = g63_detect.predict_uv(b, Twl, T_LB[b], Twc, K, d, SQ)
        ctr = (np.linalg.inv(Twc) @ Twl @ T_LB[b] @ np.r_[np.mean(list(board_points(b, SQ).values()), 0), 1])[:3]
        out[b] = dict(uv=uv, depth=float(ctr[2]))
    return out
