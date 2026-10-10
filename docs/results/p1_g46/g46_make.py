"""G46 (docs/p1_g46_sched_speed.md): E1 fine pose table for traverse edges, E3 tuple profile, E2 certified-epsilon stop.
OFFLINE, no ROS.

    python3 g46_make.py fine   -> g46_fine_pose.npz (A1)
    python3 g46_make.py ke1    -> K-E1 regression vs g43_edge_cache.jsonl
    python3 g46_make.py prof   -> E3 profile of 30 tuples
"""
import json
import math
import os
import random
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g43'))
import g43_make as M  # noqa: E402

K, G41, G4 = M.K, M.G41, M.G4
G = G4.G
FINE = os.path.join(HERE, 'g46_fine_pose.npz')
LIN = np.round(np.arange(0, 161) * 0.01, 2)            # 0.00 .. 1.60 m
ROT = np.arange(-180, 181).astype(float)               # deg
NPROC = int(os.environ.get('G46_NPROC', 15))


def _row(arg):
    g, li = arg
    ec, _ = G4.checkers()
    out = np.empty(len(ROT))
    for k, r in enumerate(ROT):
        out[k] = ec.check(ec.q_from(G.base_joints(g, float(LIN[li]), math.radians(r))), only=(G4.gp(g),))[0]
    return g, li, out


def fine():
    t0 = time.time()
    D = {g: np.full((len(LIN), len(ROT)), np.nan) for g in (1, 2)}
    with Pool(NPROC) as p:
        for g, li, row in p.imap_unordered(_row, [(g, li) for g in (1, 2) for li in range(len(LIN))], chunksize=1):
            D[g][li] = row
    np.savez(FINE, d1=D[1], d2=D[2], lin=LIN, rot=ROT)
    print(f'E1 tabel {2 * D[1].size} pose dalam {time.time() - t0:.0f} s; d min g1 {np.nanmin(D[1]):.4f} g2 {np.nanmin(D[2]):.4f}')


class FineEdges:
    """Edge verdict from the fine table: min d over every lattice point of the closed rectangle."""

    def __init__(self):
        z = np.load(FINE)
        self.D = {1: z['d1'], 2: z['d2']}

    def d(self, key):
        g, l0, r0, l1, r1 = key
        a, b = sorted((float(K.REF.lin[l0]), float(K.REF.lin[l1])))
        c, e = sorted((math.degrees(float(K.REF.rot[r0])), math.degrees(float(K.REF.rot[r1]))))
        ia, ib = int(round(a / 0.01)), int(round(b / 0.01))
        ic, ie = int(round(c)) + 180, int(round(e)) + 180
        return float(self.D[g][ia:ib + 1, ic:ie + 1].min())

    def verdict(self, key):
        d = self.d(key)
        return ('COLLIDE' if d <= 0.0 else 'MARGIN' if d < 0.05 else 'CLEAR'), d


def ke1():
    fe = FineEdges()
    rows = [json.loads(ln) for ln in open(os.path.join(R, 'p1_g43/g43_edge_cache.jsonl'))]
    same = band = bad = 0
    dd = []
    for r in rows:
        v, d = fe.verdict(tuple(r['key']))
        dd.append(d - r['d'])
        if v == r['verdict']:
            same += 1
        elif abs(r['d'] - 0.05) <= 0.002 and abs(d - 0.05) <= 0.002:
            band += 1
        else:
            bad += 1
            if bad <= 15:
                print(f"  BEDA {tuple(r['key'])}: cache {r['verdict']} {r['d'] * 1000:.2f} mm, E1 {v} {d * 1000:.2f} mm")
    dd = np.abs(dd)
    print(f'K-E1: {len(rows)} tepi; sama {same}, beda di pita +-2 mm {band}, beda lain {bad}; '
          f'|d_E1 - d_cache| median {np.median(dd) * 1000:.3f} mm, p99 {np.percentile(dd, 99) * 1000:.3f}, '
          f'maks {dd.max() * 1000:.3f} mm')




_T = {}


def _timed(name, fn):
    def w(*a, **k):
        t = time.process_time()
        try:
            return fn(*a, **k)
        finally:
            _T[name] = _T.get(name, 0.0) + time.process_time() - t
    return w


def _prof_job(key):
    if not _T.get('_patched'):
        ec, bx = G4.checkers()
        O2 = G41.O2
        O2.solve, O2.torq2, O2.gravity = (_timed('O2.solve', O2.solve), _timed('O2.torq2', O2.torq2),
                                          _timed('O2.gravity', O2.gravity))
        type(ec).check = _timed('ec.check', type(ec).check)
        type(ec).screen_trajectory = _timed('ec.screen_trajectory', type(ec).screen_trajectory)
        type(bx).dist = _timed('bx.dist', type(bx).dist)
        sc = G41.same_checker(1)
        type(sc).screen_trajectory = _timed('same.screen_trajectory', type(sc).screen_trajectory)
        G41.screen_line = _timed('screen_line', G41.screen_line)
        _T.clear()
        _T['_patched'] = True
    for k in [k for k in _T if k != '_patched']:
        _T[k] = 0.0
    t = time.process_time()
    d = G41._tuple_job(tuple(key))
    tot = time.process_time() - t
    return dict(key=list(key), ok41=d['ok41'], ok3=d['ok3'], n_sol=d['n_sol'], n_checked=d['n_checked'],
                whys=d['whys'], total=tot, parts={k: v for k, v in _T.items() if k != '_patched'})


def prof():
    rows = [json.loads(ln) for ln in open(os.path.join(R, 'p1_g43/g43_tuple_cache.jsonl'))]
    rng = random.Random(46)
    ok = [r['key'] for r in rows if r['ok41']]
    no = [r['key'] for r in rows if not r['ok41']]
    pick = rng.sample(ok, 15) + rng.sample(no, 15)
    with Pool(10) as p:
        res = p.map(_prof_job, pick, chunksize=1)
    with open(os.path.join(HERE, 'g46_prof.jsonl'), 'w') as f:
        for r in res:
            f.write(json.dumps(r) + '\n')
    tot = sum(r['total'] for r in res)
    agg = {}
    for r in res:
        for k, v in r['parts'].items():
            agg[k] = agg.get(k, 0.0) + v
    print(f'E3: 30 tuple, CPU total {tot:.1f} s (median {np.median([r["total"] for r in res]):.1f} s)')
    for k, v in sorted(agg.items(), key=lambda x: -x[1]):
        print(f'  {k:24s} {v:8.1f} s  {100 * v / tot:5.1f} %  (bersarang: screen_line memuat env/kotak/se/gravity)')
    for lab, sel in (('ok41', True), ('tolak', False)):
        rr = [r for r in res if r['ok41'] == sel]
        t = sum(r['total'] for r in rr)
        s = sum(r['parts'].get('O2.solve', 0) for r in rr)
        print(f'  {lab}: CPU {t:.1f} s, O2.solve {100 * s / t:.1f} %, ok3 {sum(r["ok3"] for r in rr)}/{len(rr)}, '
              f'whys {[r["whys"] for r in rr if r["whys"]][:4]}')


def _e3_job(key):
    G41.STOP_TORQ = True
    return G41._tuple_job(tuple(key))


def ke3():
    rows = [json.loads(ln) for ln in open(os.path.join(R, 'p1_g43/g43_tuple_cache.jsonl'))]
    ok = [r for r in rows if r['ok41']]
    no = random.Random(46).sample([r for r in rows if not r['ok3']], 100)
    pick = ok + no
    with Pool(NPROC) as p:
        res = p.map(_e3_job, [r['key'] for r in pick], chunksize=1)
    with open(os.path.join(HERE, 'g46_ke3.jsonl'), 'w') as f:
        for r in res:
            f.write(json.dumps(r) + '\n')
    bad = [(a['key'], (a['ok3'], a['ok41']), (b['ok3'], b['ok41'])) for a, b in zip(pick, res)
           if (a['ok3'], a['ok41']) != (b['ok3'], b['ok41'])]
    goal = sum(1 for a, b in zip(pick[:len(ok)], res[:len(ok)]) if a['goal'] != b['goal'])
    c_old = sum(a['cpu_s'] for a in no)
    c_new = sum(b['cpu_s'] for b in res[len(ok):])
    print(f"K-E3: ok41 {len(ok)} + ok3-gagal {len(no)}; verdict beda {len(bad)} {bad[:5]}; tujuan q6 beda (ok41) {goal}; "
          f"early_stop {sum('early_stop' in b for b in res)}; CPU ok3-gagal {c_old:.0f} -> {c_new:.0f} s "
          f"({100 * (1 - c_new / c_old):.1f} % hemat); CPU ok41 {sum(a['cpu_s'] for a in ok):.0f} -> "
          f"{sum(b['cpu_s'] for b in res[:len(ok)]):.0f} s")


if __name__ == '__main__':
    M.setup('R180', 'g43')
    {'fine': fine, 'ke1': ke1, 'prof': prof, 'ke3': ke3}[sys.argv[1]]()
