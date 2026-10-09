"""G39-OFF (docs/p1_g39_base_frame_table.md A1-A3): oracle''' torque table in the ARM-BASE frame. OFFLINE.

One table serves every rail, every gantry rotation and all four arms (G29 K-EQ: gravity and tool-down-roll-free are
yaw-invariant). Grid h = 2 cm in (rel_x, rel_y), |rel_xy| <= 0.76 m, 5 layers rel_z = 1.9525 - z, z 1.00..1.32.
Each point = oracle2.solve(tilt, envelope) + torq2(MARGIN) for arm_1 at rail 0.80, rot 0 -- the exact G24 predicate.
Read rule: a tuple is ok iff all four corners of its grid cell are computed and ok.

    python3 g39_table.py k          -> K: the base-frame job reproduces cached EXACT tuples of other arms/rots
    python3 g39_table.py build      -> g39_table.jsonl (append-only, restartable)
    python3 g39_table.py validate   -> confusion vs every cached exact tuple + read-all timing (A3, D290-D294)
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g31'))
import g31_sched as S  # noqa: E402

M, sched, REF, NR = S.M, S.sched, S.REF, S.NR
O2, IS, IRM, SLOT_ARM, MARGIN = M.O2, S.GO.IS, S.GO.IRM, M.SLOT_ARM, M.MARGIN
X = REF.nodes
H, RHO_MAX, L0 = 0.02, 0.76, 0.80
ZW = [1.00, 1.08, 1.16, 1.24, 1.32]
BASE_Z = 1.9525
ZR = [round(BASE_Z - z, 4) for z in ZW]
NH = int(round(RHO_MAX / H)) + 1                    # index range -NH..NH
TABLE = os.path.join(HERE, 'g39_table.jsonl')
R0, P0 = (np.asarray(a) for a in IS.base_pose('arm1', L0, 0.0))


def grid_points():
    out = []
    for iz in range(len(ZR)):
        for ix in range(-NH, NH + 1):
            for iy in range(-NH, NH + 1):
                if np.hypot(ix * H, iy * H) <= RHO_MAX + 1e-9:
                    out.append((iz, ix, iy))
    return out


def _job_rel(rel):
    """Exact oracle''' at base-frame position rel for arm_1 @ (L0, rot 0)."""
    world = P0 + R0 @ np.asarray(rel, float)
    r = O2.solve(world, L0, 'arm_1', tilt=True, envelope=True)
    return bool(O2.torq2(r, MARGIN)), r


def _job(pt):
    iz, ix, iy = pt
    ok, r = _job_rel((ix * H, iy * H, ZR[iz]))
    return dict(key=[iz, ix, iy], ok=ok, saturated=bool(r['saturated']), n_sol=int(r['n_sol']),
                taumax=r['taumax'].tolist())


def load_table():
    T = {}
    if os.path.exists(TABLE):
        for ln in open(TABLE):
            d = json.loads(ln)
            T[tuple(d['key'])] = d
    return T


def table_arrays(T):
    n = 2 * NH + 1
    done = np.zeros((len(ZR), n, n), bool)
    ok = np.zeros((len(ZR), n, n), bool)
    for (iz, ix, iy), d in T.items():
        done[iz, ix + NH, iy + NH] = True
        ok[iz, ix + NH, iy + NH] = d['ok']
    return done, ok


def rel_of(nodes, g, li, ri, s):
    """Base-frame position of node targets for slot s of gantry g at (lin[li], rot[ri]); arrays broadcast."""
    arm = np.array([IRM[SLOT_ARM[(int(gg), int(ss))]] for gg, ss in zip(np.ravel(g), np.ravel(s))])
    out = np.empty((len(arm), 3))
    for a in np.unique(arm):
        m = arm == a
        Rb, pb = IS.base_pose(a, REF.lin[np.ravel(li)[m]], REF.rot[np.ravel(ri)[m]])
        v = X[np.ravel(nodes)[m]] - pb
        out[m] = np.einsum('nji,nj->ni', Rb, v)                 # R^T v
    return out


def lookup(rel, done, ok):
    """A2: ok iff all four cell corners computed and ok. rel (N, 3)."""
    iz = np.full(len(rel), -1)
    for k, zr in enumerate(ZR):
        iz[np.abs(rel[:, 2] - zr) < 1e-6] = k
    i0 = np.floor(rel[:, 0] / H).astype(int) + NH
    j0 = np.floor(rel[:, 1] / H).astype(int) + NH
    n = 2 * NH + 1
    good = (iz >= 0) & (i0 >= 0) & (j0 >= 0) & (i0 + 1 < n) & (j0 + 1 < n)
    res = np.zeros(len(rel), bool)
    z, a, b = iz[good], i0[good], j0[good]
    c = np.ones(len(z), bool)
    for da in (0, 1):
        for db in (0, 1):
            c &= done[z, a + da, b + db] & ok[z, a + da, b + db]
    res[good] = c
    return res


def exact_tuples():
    """Every cached EXACT tuple (z <= 1.32): (n, g, li, ri, s) -> ok."""
    out = {}
    for (n, g, p, s), d in M.load_cache().items():
        out[(n, g, p, M.ROT0, s)] = bool(d['ok'])
    for f in (os.path.join(R, 'p1_g29/g29_oracle_rot_cache.jsonl'), os.path.join(R, 'p1_g31/g31_oracle_rot_cache.jsonl'),
              os.path.join(R, 'p1_g38/g38_tuple_cache.jsonl')):
        for ln in open(f):
            d = json.loads(ln)
            out[tuple(d['key'])] = bool(d['ok3'])
    return {k: v for k, v in out.items() if X[k[0], 2] <= 1.32 + 1e-6}


def k_checks():
    """K-EQ': exact cached tuples of OTHER arms / gantries / rotations == base-frame job at their exact rel."""
    ex = exact_tuples()
    rng = np.random.default_rng(0)
    keys = list(ex)
    pick = []
    for want in (True, False):
        for g in (1, 2):
            for s in (0, 1):
                c = [k for k in keys if ex[k] == want and k[1] == g and k[4] == s and abs(S.RDEG[k[3]]) > 1e-9]
                pick += [c[i] for i in rng.choice(len(c), 2, replace=False)]
    bad = 0
    for k in pick:
        rel = rel_of(np.array([k[0]]), np.array([k[1]]), np.array([k[2]]), np.array([k[3]]), np.array([k[4]]))[0]
        ok, _ = _job_rel(rel)
        bad += ok != ex[k]
        print(f'K {k} rot {S.RDEG[k[3]]:+.0f} rel {np.round(rel, 4)}: tabel-job {ok} tepat {ex[k]} {"✓" if ok == ex[k] else "✗"}')
    print('K: LULUS' if not bad else f'K: GAGAL {bad}/{len(pick)}')
    return not bad


def build():
    T = load_table()
    todo = [p for p in grid_points() if p not in T]
    print(f'titik {len(grid_points())}, sudah {len(T)}, sisa {len(todo)}  [{time.strftime("%T")}]', flush=True)
    t0, k = time.time(), 0
    with Pool(15) as pool, open(TABLE, 'a') as f:
        for d in pool.imap_unordered(_job, todo, chunksize=1):
            f.write(json.dumps(d) + '\n')
            f.flush()
            k += 1
            if k % 500 == 0:
                rate = k / (time.time() - t0)
                print(f'  {k}/{len(todo)}  {rate:.2f} titik/s  sisa ~{(len(todo) - k) / rate / 3600:.1f} jam  '
                      f'[{time.strftime("%T")}]', flush=True)
    print(f'SELESAI {k} titik dalam {(time.time() - t0) / 3600:.2f} jam', flush=True)


def validate():
    T = load_table()
    done, ok = table_arrays(T)
    print(f'tabel: {len(T)}/{len(grid_points())} titik, ok {int(ok.sum())}')
    ex = exact_tuples()
    k = np.array(list(ex))
    truth = np.array([ex[tuple(x)] for x in k])
    pred = lookup(rel_of(k[:, 0], k[:, 1], k[:, 2], k[:, 3], k[:, 4]), done, ok)
    fa, fr = int((pred & ~truth).sum()), int((~pred & truth).sum())
    print(f'tuple tepat {len(k)} (ok {int(truth.sum())}): salah-terima {fa}, salah-tolak {fr} '
          f'({100 * fr / max(1, truth.sum()):.1f} % dari ok), benar {int((pred == truth).sum())}')
    rot0 = k[:, 3] == M.ROT0
    for name, m in (('rot 0', rot0), ('rot != 0', ~rot0)):
        print(f'  {name}: n {int(m.sum())}, salah-terima {int((pred & ~truth & m).sum())}, '
              f'salah-tolak {int((~pred & truth & m).sum())}')
    bad = k[pred & ~truth][:20]
    for x in bad:
        print('  salah-terima', tuple(int(v) for v in x), 'xyz', X[x[0]].round(3), 'rot', S.RDEG[x[3]])
    # D293/D294: read the table for every L1 tuple |rot| <= 90, z <= 1.32
    t = time.time()
    ri90 = [i for i, r in enumerate(S.RDEG) if abs(r) <= 90 + 1e-9]
    zok = np.flatnonzero(X[:, 2] <= 1.32 + 1e-6)
    tot = acc = 0
    for g in (1, 2):
        r = sched._gantry_oracles(M.CAPS[g], zok)[0].reshape(len(zok), len(REF.lin), NR, 2)[:, :, ri90, :]
        idx = np.argwhere(r)
        for c in range(0, len(idx), 500_000):
            b = idx[c:c + 500_000]
            p = lookup(rel_of(zok[b[:, 0]], np.full(len(b), g), b[:, 1], np.array(ri90)[b[:, 2]], b[:, 3]), done, ok)
            tot += len(b)
            acc += int(p.sum())
    print(f'baca semua tuple L1 |rot|<=90: {tot} dalam {time.time() - t:.1f} s; ok {acc} ({100 * acc / tot:.1f} %)')
    json.dump(dict(n_table=len(T), n_exact=len(k), false_accept=fa, false_reject=fr, n_ok_exact=int(truth.sum()),
                   read_all_n=tot, read_all_ok=acc, read_all_s=time.time() - t),
              open(os.path.join(HERE, 'g39_validate.json'), 'w'), indent=1)


if __name__ == '__main__':
    {'k': lambda: sys.exit(0 if k_checks() else 1), 'build': build, 'validate': validate}[sys.argv[1]]()
