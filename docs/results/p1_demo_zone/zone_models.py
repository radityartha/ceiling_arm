#!/usr/bin/env python3
"""ZONE-C: model koreksi lokal rgbd2 di zona meja demo (data yang ada). Galat yang dinilai = |xy koreksi - tanda| (cm).
Validasi: LOMO (tinggalkan semua observasi satu TANDA), dan silang dataset (fit B botol 165 mm -> uji A botol demo 7 cm, dan
sebaliknya). Model:
  T2  geser xy dunia (2)            T3  geser xyz (3)             P3  planar kaku x,y,yaw (3)
  S3  kamera: geser piksel du,dv + offset jarak dr (3)            S4  S3 + skala jarak (4)
  RP2 sinar track -> geser piksel du,dv -> potong bidang z = permukaan + h_eff (TANPA depth; h_eff per botol diketahui) (2)
  RP3 RP2 + h_eff bebas (3)        A6  afin xy (6)
"""
import itertools
import json
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

import zone_data as Z

F = 643.6
RWC = Rot.from_euler('xyz', Z.RPY_INC).as_matrix()
C = Z.C_INC


def cam(p):
    return RWC.T @ (p - C)


def world(pc):
    return RWC @ pc + C


def apply(model, x, o):
    t = o['trk']
    if model == 'none':
        return t[:2]
    if model == 'T2':
        return t[:2] + x[:2]
    if model == 'T3':
        return (t + x[:3])[:2]
    if model == 'P3':
        c, s = np.cos(x[2]), np.sin(x[2])
        q = t[:2] - Z0
        return Z0 + np.array([c * q[0] - s * q[1], s * q[0] + c * q[1]]) + x[:2]
    if model == 'A6':
        return np.array([[1 + x[0], x[1]], [x[2], 1 + x[3]]]) @ (t[:2] - Z0) + Z0 + x[4:6]
    pc = cam(t)
    r = np.linalg.norm(pc)
    u, v = pc[0] / pc[2], pc[1] / pc[2]
    u, v = u + x[0] / F, v + x[1] / F
    d = np.array([u, v, 1.0])
    d /= np.linalg.norm(d)
    if model in ('S3', 'S4'):
        rr = r + x[2] if model == 'S3' else r * (1 + x[3]) + x[2]
        return world(d * rr)[:2]
    if model in ('RP2', 'RP3'):
        h = HEFF[o['H']] if model == 'RP2' else x[2] * o['H']
        zc = o['surf'] + h
        dw = RWC @ d
        s = (zc - C[2]) / dw[2]
        return (C + s * dw)[:2]
    raise ValueError(model)


NP = dict(none=0, T2=2, T3=3, P3=3, S3=3, S4=4, RP2=2, RP3=3, A6=6)
HEFF = {0.07: 0.035, 0.165: 0.0825}


def fit(model, obs):
    n = NP[model]
    if n == 0:
        return np.zeros(0)
    x0 = np.zeros(n)
    if model == 'RP3':
        x0[2] = 0.5
    sol = least_squares(lambda x: np.concatenate([apply(model, x, o) - o['xy'] for o in obs]), x0)
    return sol.x


def err(model, x, obs):
    return np.array([100 * np.linalg.norm(apply(model, x, o) - o['xy']) for o in obs])


obs = Z.load()
Z0 = np.mean([o['xy'] for o in obs], axis=0)
marks = sorted({o['mark'] for o in obs})
A = [o for o in obs if o['ds'] == 'A']
B = [o for o in obs if o['ds'] == 'B']
out = {}
print('%-5s %22s %22s %16s %16s %10s' % ('model', 'fit-semua RMS/maks', 'LOMO med/p90/maks', 'B->A med/maks', 'A->B med/maks', 'LOMO-A'))
for m in NP:
    xall = fit(m, obs)
    e_fit = err(m, xall, obs)
    lomo, lomoA = [], []
    for k in marks:
        tr = [o for o in obs if o['mark'] != k]
        te = [o for o in obs if o['mark'] == k]
        x = fit(m, tr)
        e = err(m, x, te)
        lomo += list(e)
        lomoA += [ee for ee, o in zip(e, te) if o['ds'] == 'A']
    eBA = err(m, fit(m, B), A)
    eAB = err(m, fit(m, A), B) if NP[m] <= len(A) - 1 else np.array([np.nan])
    lomo = np.array(lomo)
    out[m] = dict(x=xall.round(4).tolist(), fit_rms=float(np.sqrt(np.mean(e_fit ** 2))), lomo=lomo.round(2).tolist(),
                  lomoA=np.round(lomoA, 2).tolist(), BA=eBA.round(2).tolist(), AB=eAB.round(2).tolist())
    print('%-5s %10.1f / %5.1f cm %8.1f / %4.1f / %4.1f %8.1f / %4.1f %8.1f / %4.1f %10.1f' % (
        m, np.sqrt(np.mean(e_fit ** 2)), e_fit.max(), np.median(lomo), np.percentile(lomo, 90), lomo.max(),
        np.median(eBA), eBA.max(), np.nanmedian(eAB), np.nanmax(eAB), np.median(lomoA)))
json.dump(out, open('zone_models.json', 'w'), indent=1)
