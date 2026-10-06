"""G31 A6 K-G3a..d: the rot_cmd patch of sched.py. OFFLINE.

    python3 g31_g3_controls.py      -> g31_g3_controls.log (stdout)
"""
import importlib.util
import json
import math
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g26'))
import g26_make as GM  # noqa: E402

RS, M, MI, sched = GM.RS, GM.M, GM.MI, GM.sched
ROT_CMD = 0.9235
SEEDS = [1, 2, 13, 16, 17, 18, 20, 21, 22, 23, 26, 29, 33, 35, 36, 40, 42]
spec = importlib.util.spec_from_file_location('sched_before', os.path.join(HERE, 'sched_before.py'))
SB = importlib.util.module_from_spec(spec)
sys.modules["sched_before"] = SB
spec.loader.exec_module(SB)


def js(x):
    return json.loads(json.dumps(x, default=float))


def k_a():
    rng = np.random.default_rng(31)
    dl = rng.uniform(-1.7, 1.7, 100_000)
    dr = rng.uniform(-2 * np.pi, 2 * np.pi, 100_000)
    dl[:5000] = 0.0
    dr[5000:10000] = 0.0
    bad = 0
    for tf in (0.0, 55.6135):
        for rc in (0.0, RS.C['r']):
            a = sched.traverse_time(dl, dr, tf, rc)
            b = SB.traverse_time(dl, dr, tf, rc)
            bad += int((a != b).sum())
    print(f'K-G3a traverse_time rot_cmd=0 vs fungsi lama: 4 x 1e5 pasangan, beda {bad}')
    return bad == 0


def k_b():
    cache = M.load_cache()
    s45 = {r['seed']: r for r in json.load(open(os.path.join(R, 'p1_g25/g25_sched45.json')))}
    bad, n = [], 0
    for row in GM.CAND24:
        if not row.get('ok_ii'):
            continue
        inst, _ = M.instance(row['seed'], row['nodes'], row['rejects'], cache)
        for name, serial in (("P1'", True), ("P1'-par", False)):
            x = RS.p1prime(inst, serial)
            s = sched.solve_exact(x)
            n += 1
            a = s45[row['seed']][name]
            if not (s.makespan == a['opt'] and js({str(g): v for g, v in MI.sched_json(x, s).items()}) == a['sched']):
                bad.append((row['seed'], name))
    print(f'K-G3b G25: {n - len(bad)}/{n} bit-identik (opt + jadwal, P1\' dan P1\'-par); beda {bad}')
    ok = not bad
    c26 = {r['seed']: r for r in json.load(open(os.path.join(R, 'p1_g26/g26_candidates.json')))}
    bad, n = [], 0
    for r24 in GM.CAND24:
        old = c26[r24['seed']]
        if not old.get('ok_i'):
            continue
        inst = GM.instance(r24, cache, {1: 0.0, 2: 0.0})
        x = RS.p1prime(inst)
        s = sched.solve_exact(x)
        n += 1
        if not (s.makespan == old['makespan']["P1'"] and
                js({str(g): v for g, v in MI.sched_json(x, s).items()}) == old['schedule_FISIK']):
            bad.append(r24['seed'])
    print(f'K-G3b G26 (p0 0/0): {n - len(bad)}/{n} bit-identik (makespan P1\' + schedule_FISIK); beda {bad}')
    return ok and not bad


def k_c():
    cache = M.load_cache()
    bad = []
    for seed in SEEDS:
        r24 = next(r for r in GM.CAND24 if r['seed'] == seed)
        x = RS.p1prime(GM.instance(r24, cache, {1: 0.0, 2: 0.0}))
        a, b = sched.solve_exact(x), sched.solve_exact(replace(x, rot_cmd=ROT_CMD))
        if not (a.makespan == b.makespan and a.stops == b.stops):
            bad.append(seed)
    print(f'K-G3c R0 17 seed: rot_cmd {ROT_CMD} vs 0 bit-identik {17 - len(bad)}/17; beda {bad}')
    return not bad


def k_d():
    REF = M.REF
    rd = np.degrees(REF.rot)
    R35 = REF.rot[np.abs(rd) <= 35 + 1e-9]
    P = np.stack(np.meshgrid(REF.lin, R35, indexing='ij'), -1).reshape(-1, 2)
    C = RS.C
    worst, nviol = -np.inf, 0
    for tf in (C['t_fold'], C['t_fold_first'], 0.0):
        T = sched.traverse_matrix(P, tf, C['r'], ROT_CMD)
        T1 = sched.traverse_matrix(P, C['t_fold'], C['r'], ROT_CMD)
        for a in range(len(P)):
            # T(a,c) - [T(a,b) + T1(b,c)] over all b, c: first leg priced with tf, later legs with t_fold
            v = T[a][None, :] - (T[a][:, None] + T1)
            worst = max(worst, float(v.max()))
            nviol += int((v > 1e-9).sum())
    print(f'K-G3d segitiga, {len(P)} pose R35 ({len(R35)} rot), 3 x {len(P)}^3 tripel (t_fold P1\', t_fold_first, 0): '
          f'pelanggaran > 1e-9 {nviol}, maks T(a,c) - T(a,b) - T(b,c) = {worst:+.6f} s')
    # unwrapped vs cyclic agree inside R35 (|d| <= 70 deg)
    dd = P[:, None, 1] - P[None, :, 1]
    same = np.array_equal(np.minimum(np.abs(np.degrees(dd)), 360 - np.abs(np.degrees(dd))), np.abs(np.degrees(dd)))
    print(f'   tanpa-bungkus == arah-pendek di R35: {same} (maks |drot| {np.degrees(np.abs(dd)).max():.1f} deg)')
    return nviol == 0 and same


if __name__ == '__main__':
    r = [k_a(), k_b(), k_c(), k_d()]
    print('K-G3 ' + ('LULUS' if all(r) else f'GAGAL {r}'))
