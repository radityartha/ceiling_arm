"""G43 A2 checks (docs/p1_g43_rot180.md): K-SYM (torque + (c41b) under the 180-deg flip) and the positive / negative
controls of the predicates at |rot| > 35 deg. OFFLINE. Small pools (the R180 run uses 15 processes).

    nice -n 19 python3 g43_k.py sym       -> g43_ksym.jsonl + verdict table
    nice -n 19 python3 g43_k.py controls
"""
import json
import math
import os
import random
import sys
import warnings
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g43_make as M  # noqa: E402

K, G41, G4, S = M.K, M.G41, M.G4, M.S
RDEG, REF, NR = M.RDEG, K.REF, K.NR
KSYM = os.path.join(HERE, 'g43_ksym.jsonl')


def flip(key):
    n, g, li, ri, s = key
    th = RDEG[ri]
    rj = ri - 36 if th > 0 else ri + 36
    assert abs(abs(RDEG[rj] - th) - 180.0) < 1e-6
    return (n, g, li, rj, 1 - s)


def sym():
    rows = [json.loads(ln) for ln in open(os.path.join(HERE, 'g43_tuple_cache.jsonl'))][:1201]   # = the G42 copy
    pool_ = [d for d in rows if 5 - 1e-6 <= abs(RDEG[d['key'][3]]) <= 35 + 1e-6]
    rng = random.Random(43)
    ok = [d for d in pool_ if d['ok3'] and d['ok41']]
    bad = [d for d in pool_ if not (d['ok3'] and d['ok41'])]
    pick = rng.sample(ok, 20) + rng.sample(bad, 20)
    have = {}
    if os.path.exists(KSYM):
        for ln in open(KSYM):
            d = json.loads(ln)
            have[tuple(d['key'])] = d
    todo = [flip(tuple(d['key'])) for d in pick if flip(tuple(d['key'])) not in have]
    with Pool(3) as p, open(KSYM, 'a') as f:
        for d in p.imap_unordered(G41._tuple_job, todo, chunksize=1):
            have[tuple(d['key'])] = d
            f.write(json.dumps(d) + '\n')
            f.flush()
    same = 0
    for d in pick:
        e = have[flip(tuple(d['key']))]
        a, b = (d['ok3'], d['ok41']), (e['ok3'], e['ok41'])
        same += a == b
        dq = '' if not (d['ok41'] and e['ok41']) else \
            f", |dq6| maks {math.degrees(np.max(np.abs(np.array(d['goal']['q6']) - np.array(e['goal']['q6'])))):.3f} deg"
        dt = np.max(np.abs(np.array(d['taumax']) - np.array(e['taumax'])))
        print(f"{tuple(d['key'])} rot {RDEG[d['key'][3]]:+.0f} -> {RDEG[e['key'][3]]:+.0f} slot {e['key'][4]}: "
              f"(ok3, ok41) {a} vs {b} {'SAMA' if a == b else 'BEDA'}; |dtaumax| {dt:.4f}{dq}; "
              f"whys {d['whys']} vs {e['whys']}")
    print(f'K-SYM: {same}/{len(pick)} verdict sama')


def controls():
    import g29_rot_screen as RS
    sc = S.sched_coll
    po = G41.pose_ok_b()
    rows = json.load(open(G4.POSE_TABLE))
    tab = {tuple(d['key']): d for d in rows}
    res = []
    # (a) pose
    rng = random.Random(43)
    big = [k for k in tab if 60 <= abs(RDEG[k[2]]) <= 120]
    rej = [k for k in big if not po[k[0]][k[1], k[2]] and tab[k]['d_env'] < G4.ENV_MIN]
    acc = [k for k in big if po[k[0]][k[1], k[2]]]
    sel = [k for g in (1, 2) for k in rng.sample([x for x in rej if x[0] == g], 1)] + \
          [k for g in (1, 2) for k in rng.sample([x for x in acc if x[0] == g], 1)]
    for k, want in zip(sel, (False, False, True, True)):
        d = G4._pose_job(k)
        ok = bool(d['ok'] and (d['d_box'] is None or d['d_box'] >= G41.BOX_MIN))
        print(f"(a) pose g{k[0]} ({REF.lin[k[1]]:.2f}, {RDEG[k[2]]:+.0f}): d_env {d['d_env'] * 1000:.1f} mm {d['geom']}, "
              f"kotak {d['d_box']} -> {'ok' if ok else 'TOLAK'} (harap {'ok' if want else 'TOLAK'}; tabel "
              f"{'ok' if po[k[0]][k[1], k[2]] else 'TOLAK'})")
        res.append(('(a)', want, ok))
    # (b) edge rot 0 -> 90 at a rail position whose end poses pass (a) but an intermediate rotation is env-refused
    r0, r90 = int(np.argmin(np.abs(RDEG))), int(np.argmin(np.abs(RDEG - 90)))
    found = None
    for g in (1, 2):
        for li in range(len(REF.lin)):
            mid = [ri for ri in range(r0 + 1, r90) if not po[g][li, ri] and tab[(g, li, ri)]['d_env'] < G4.ENV_MIN]
            if po[g][li, r0] and po[g][li, r90] and mid:
                found = (g, li, mid)
                break
        if found:
            break
    if found is None:
        print('(b) tidak ada lin dengan ujung ok + rotasi antara ditolak env -- kontrol (b) TIDAK dijalankan')
    else:
        g, li, mid = found
        d = G4._edge_job(G4.edge_key(g, (li, r0), (li, r90)))
        print(f"(b) tepi g{g} lin {REF.lin[li]:.2f}: rot 0 -> +90 (antara ditolak: {[round(RDEG[m]) for m in mid]}): "
              f"{d['verdict']} {d['d'] * 1000:.1f} mm {d['geom']}")
        res.append(('(b)', False, d['verdict'] == 'CLEAR'))
    # (iii) structure
    for (l1, t1, l2, t2), want in (((0.80, 90, 0.80, 90), False), ((0.80, 90, 0.80, -90), False),
                                   ((0.80, 90, 1.60, 90), True), ((0.80, 90, 1.60, -90), True)):
        d = float(sc.pair_distance(l1, math.radians(t1), l2, math.radians(t2)))
        ok = d > sc.C_CLEAR
        print(f"(iii) g1 ({l1}, {t1:+d}) vs g2 ({l2}, {t2:+d}): jarak {d * 1000:.1f} mm -> {'bebas' if ok else 'BLOCK'}")
        res.append(('(iii)', want, ok))
    # (iv') cross-gantry rotation sweep, arms REST
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        chk = RS.RotCrossChecker()
    for (g1, g2frm, g2to), want in ((((0.80, 90), (0.80, 0), (0.80, 90)), False),
                                    (((1.50, 0), (0.00, 0), (0.00, 90)), True)):
        st = {'t1_linear_joint': g1[0], 't1_rotation_joint': math.radians(g1[1]),
              't2_linear_joint': g2frm[0], 't2_rotation_joint': math.radians(g2frm[1])}
        for p in K.PREFIX.values():
            st.update({f'{p}joint_{i}': float(x) for i, x in enumerate(K.REST, 1)})
        sw = RS.sweep_rot(chk, 2, (g2frm[0], math.radians(g2frm[1])), (g2to[0], math.radians(g2to[1])), st)
        print(f"(iv') g1 {g1}, g2 {g2frm} -> {g2to}: {sw['verdict']} {sw['d'] * 1000:.1f} mm {sw['pair']}")
        res.append(("(iv')", want, sw['verdict'] == 'CLEAR'))
    pos = [(w, o) for _, w, o in res if not w]
    neg = [(w, o) for _, w, o in res if w]
    print(f'kontrol positif TOLAK {sum(not o for _, o in pos)}/{len(pos)}; negatif terima {sum(o for _, o in neg)}/{len(neg)}')


if __name__ == '__main__':
    M.setup('R180', 'g43')
    if sys.argv[1] != 'sym_e':
        {'sym': sym, 'controls': controls}[sys.argv[1]]()


def mirror_edge(key):
    """(g, l0, r0, l1, r1) -> the edge with both rotations shifted by -/+180 deg (canonical), or None."""
    g, l0, r0, l1, r1 = key
    if r0 >= 36 and r1 >= 36:
        d = -36
    elif r0 < 36 and r1 < 36:
        d = 36
    else:
        return None
    return G4.edge_key(g, (l0, r0 + d), (l1, r1 + d))


def sym_e():
    rows = [json.loads(ln) for ln in open(os.path.join(HERE, 'g43_edge_cache.jsonl'))]
    have = {tuple(d['key']): d for d in rows}
    cand = [d for d in rows[7456:] if mirror_edge(tuple(d['key'])) is not None
            and mirror_edge(tuple(d['key'])) not in have]
    rng = random.Random(43)
    clear = [d for d in cand if d['verdict'] == 'CLEAR']
    other = [d for d in cand if d['verdict'] != 'CLEAR']
    pick = rng.sample(clear, min(10, len(clear))) + rng.sample(other, min(10, len(other)))
    with Pool(4) as p:
        res = p.map(G4._edge_job, [mirror_edge(tuple(d['key'])) for d in pick], chunksize=1)
    same = 0
    for d, e in zip(pick, res):
        same += d['verdict'] == e['verdict']
        print(f"{tuple(d['key'])} -> {tuple(e['key'])}: {d['verdict']} {d['d'] * 1000:.1f} mm vs {e['verdict']} "
              f"{e['d'] * 1000:.1f} mm {'SAMA' if d['verdict'] == e['verdict'] else 'BEDA'}")
    with open(os.path.join(HERE, 'g43_ksym_e.jsonl'), 'w') as f:
        for e in res:
            f.write(json.dumps(e) + '\n')
    print(f'K-SYM-E: {same}/{len(pick)} verdict sama')


if __name__ == '__main__' and sys.argv[1] == 'sym_e':
    sym_e()
