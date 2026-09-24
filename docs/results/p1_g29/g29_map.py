"""G29 A3 (b): gantry-gantry distance map, arms REST, over (dx = lin1 - lin2, rot1, rot2). OFFLINE.

Every configuration on a FRESH GeometryData (RotCrossChecker.by_class, B1). d_LL / d_LS / d_SS per
configuration -> g29_map.npz (float32). Then the envelope analysis (A3) -> g29_map.json.

    python3 g29_map.py build      (~10 min, 15 processes)
    python3 g29_map.py analyse    (+ 1-deg verification of the symmetric envelope, + hull vs mesh)
"""
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g29_rot_screen as G  # noqa: E402

DX = np.round(np.arange(-160, 161) * 0.01, 2)
ROT = np.radians(np.arange(-180, 180, 5.0))           # = REF.rot
OUT = os.path.join(HERE, 'g29_map.npz')
MARGIN = 0.05
R_MAX = 0.455                                        # A0.2: plates 455.0, arm REST hull 439.9
_C = {}


def lins(dx):
    return (dx, 0.0) if dx >= 0 else (0.0, -dx)


def _chk():
    if 'c' not in _C:
        _C['c'] = G.RotCrossChecker()
    return _C['c']


def eval_cfg(dx, r1, r2, off=0.0):
    c = _chk()
    l1, l2 = lins(dx)
    b = c.by_class(c.q_from(G.rest_state(l1 + off, r1, l2 + off, r2)))
    return b['LL'][0], b['LS'][0], b['SS'][0]


def _row(i):
    dx = float(DX[i])
    out = np.zeros((3, len(ROT), len(ROT)), np.float32)
    for a, r1 in enumerate(ROT):
        for b, r2 in enumerate(ROT):
            out[:, a, b] = eval_cfg(dx, float(r1), float(r2))
    return i, out


def build():
    M = np.zeros((3, len(DX), len(ROT), len(ROT)), np.float32)
    with Pool(15) as pool:
        for n, (i, row) in enumerate(pool.imap_unordered(_row, range(len(DX))), 1):
            M[:, i] = row
            if n % 20 == 0:
                print(f'  {n}/{len(DX)}', flush=True)
    np.savez_compressed(OUT, LL=M[0], LS=M[1], SS=M[2], dx=DX, rot=ROT)
    print('->', OUT)


def _inv(k):
    rng = np.random.default_rng(1000 + k)
    dx = float(rng.choice(DX[np.abs(DX) <= 1.2]))
    r1, r2 = rng.choice(ROT, 2)
    off = rng.uniform(0.0, 1.6 - abs(dx))
    a = np.array(eval_cfg(dx, float(r1), float(r2)))
    b = np.array(eval_cfg(dx, float(r1), float(r2), off))
    return float(np.abs(a - b).max())


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def analyse():
    Z = np.load(OUT)
    LL, LS, SS = Z['LL'], Z['LS'], Z['SS']
    arm = np.minimum(LL, LS)
    D = np.minimum(arm, SS)
    deg = np.degrees(ROT)
    res = {}
    with Pool(15) as pool:
        inv = pool.map(_inv, range(200))
    res['translation_invariance_max_m'] = max(inv)
    print(f'invarian translasi (200 sampel, lin mutlak lain): maks |beda| {max(inv):.2e} m')

    safe = D >= MARGIN
    near0 = lambda th: np.minimum(np.abs(_wrap(deg)), np.abs(_wrap(deg - 180.0))) <= th + 1e-9
    # (i) theta_one: one gantry within th of 0/180 -> safe for ANY rot of the other, any dx
    th_one, th_one_cert, run, runc = None, None, True, True
    for th in np.arange(0, 91, 5.0):
        m = near0(th)
        run = run and bool(safe[:, m, :].all() and safe[:, :, m].all())
        runc = runc and bool((D[:, m, :] >= MARGIN + 0.0447).all() and (D[:, :, m] >= MARGIN + 0.0447).all())
        th_one = th if run else th_one
        th_one_cert = th if runc else th_one_cert
    res['theta_one_grid_deg'] = th_one
    res['theta_one_lipschitz_certified_deg'] = th_one_cert
    m0 = near0(0.0)
    res['one_at_rot0_min_mm'] = float(min(D[:, m0, :].min(), D[:, :, m0].min()) * 1000)
    print(f'(i) theta_one (grid 5 deg, margin 50 mm): {th_one} deg; tersertifikasi Lipschitz (grid >= 94.7 mm): '
          f'{th_one_cert} deg; satu gantry di 0/180: min {res["one_at_rot0_min_mm"]:.1f} mm')
    # (ii) theta_both(dx): symmetric |rot1|, |rot2| <= th around 0 (and around 180 reported separately)
    tb = {}
    for centre in (0.0, 180.0):
        per = []
        for i in range(len(DX)):
            best = None
            for th in np.arange(0, 181, 5.0):
                m = np.abs(_wrap(deg - centre)) <= th + 1e-9
                if safe[i][np.ix_(m, m)].all():
                    best = th
                else:
                    break
            per.append(best)
        tb[centre] = per
        vals = np.array([np.nan if p is None else p for p in per], float)
        print(f'(ii) theta_both sekitar {centre:.0f}: min atas dx {np.nanmin(vals):.0f} deg '
              f'(di dx {DX[np.nanargmin(vals)]:.2f}); |dx| >= 1.00: min {np.nanmin(vals[np.abs(DX) >= 1.0]):.0f}')
    res['theta_both'] = {str(k): v for k, v in tb.items()}
    th_b = float(np.nanmin(np.array(tb[0.0], float)))
    res['theta_both_min_deg'] = th_b
    # theta_both as a function of |dx| (table)
    tab = []
    for adx in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6):
        ii = [i for i in range(len(DX)) if abs(abs(DX[i]) - adx) < 1e-9]
        tab.append((adx, min(tb[0.0][i] for i in ii)))
    res['theta_both_vs_absdx'] = tab
    print('     theta_both(|dx|): ' + ', '.join(f'{a:.1f}:{t:.0f}' for a, t in tab))
    # (iii) unsafe fraction of (rot1, rot2) at worst dx
    uns = (~safe).any(axis=0)
    res['unsafe_pair_fraction_any_dx'] = float(uns.mean())
    res['unsafe_config_fraction'] = float((~safe).mean())
    print(f'(iii) pasangan (rot1, rot2) tidak aman pada dx terburuk: {uns.mean()*100:.1f} %; konfigurasi tidak aman '
          f'{(~safe).mean()*100:.1f} %')
    # (iv) where the old screen is blind
    b0 = int(((SS <= 0) & (arm >= MARGIN)).sum())
    b1 = int(((SS > 0) & (SS < MARGIN) & (arm >= MARGIN)).sum())
    ssc = int((SS <= 0).sum())
    res.update(blind_contact=b0, blind_margin=b1, ss_contact=ssc, ss_lt_margin=int((SS < MARGIN).sum()))
    print(f'(iv) SS <= 0 & lengan >= 50 mm (lama buta pada KONTAK): {b0}; 0 < SS < 50 & lengan >= 50: {b1}; '
          f'SS <= 0 total {ssc}; SS < 50 total {int((SS < MARGIN).sum())}')
    if b1:
        w = np.argwhere((SS > 0) & (SS < MARGIN) & (arm >= MARGIN))
        ex = [(float(DX[i]), float(deg[a]), float(deg[b]), float(SS[i, a, b] * 1000), float(arm[i, a, b] * 1000))
              for i, a, b in w[:10]]
        res['blind_margin_examples'] = ex
        print('     contoh (dx, rot1, rot2, SS mm, lengan mm):', [tuple(round(v, 1) for v in e) for e in ex[:5]])
    # class that binds on the unsafe set
    cls = np.argmin(np.stack([LL, LS, SS]), axis=0)
    res['binding_class_on_unsafe'] = {n: int(((cls == k) & ~safe).sum()) for k, n in enumerate(('LL', 'LS', 'SS'))}
    print('     kelas pengikat pada konfigurasi tidak aman:', res['binding_class_on_unsafe'])
    json.dump(res, open(os.path.join(HERE, 'g29_map.json'), 'w'), indent=1, default=float)
    return res


def _v1(args):
    dx, r1, r2 = args
    return min(eval_cfg(dx, math.radians(r1), math.radians(r2)))


def verify_1deg(th):
    """A3: symmetric envelope |rot1|, |rot2| <= th re-checked on a 1-deg grid x dx 0.01."""
    R = np.arange(-th, th + 0.5, 1.0)
    jobs = [(float(dx), float(a), float(b)) for dx in DX for a in R for b in R]
    with Pool(15) as pool:
        d = np.array(pool.map(_v1, jobs, chunksize=200))
    bad = int((d < MARGIN).sum())
    print(f'verifikasi 1 deg |rot| <= {th:.0f}: {len(jobs)} konfigurasi, min {d.min()*1000:.1f} mm, di bawah 50 mm: {bad}')
    return dict(theta=th, n=len(jobs), min_mm=float(d.min() * 1000), n_below=bad)


def _mesh(args):
    from interarm_collision import _package_dirs
    import pinocchio as pin
    dx, r1, r2 = args
    if 'm' not in _C:
        c = _chk()
        mg = pin.buildGeomFromUrdf(c.model, G.URDF, pin.GeometryType.COLLISION, _package_dirs())
        n = [g.name for g in mg.geometryObjects]
        ia = [i for i, x in enumerate(n) if x.startswith('t1_')]
        ib = [i for i, x in enumerate(n) if x.startswith('t2_')]
        for i in ia:
            for j in ib:
                if n[i].startswith(G.ARM) or n[j].startswith(G.ARM):
                    mg.addCollisionPair(pin.CollisionPair(i, j))
        _C['m'] = mg
    import pinocchio as pin
    c, mg = _chk(), _C['m']
    l1, l2 = lins(dx)
    q = c.q_from(G.rest_state(l1, r1, l2, r2))
    md = mg.createData()
    pin.updateGeometryPlacements(c.model, c.data, mg, md, q)
    pin.computeDistances(c.model, c.data, mg, md, q)
    dm = min(r.min_distance for r in md.distanceResults)
    b = c.by_class(q)
    return float(min(b['LL'][0], b['LS'][0])), float(dm)


def hull_vs_mesh():
    Z = np.load(OUT)
    arm = np.minimum(Z['LL'], Z['LS'])
    idx = np.argsort(np.abs(arm - MARGIN), axis=None)[:50]
    jobs = [(float(DX[i]), float(ROT[a]), float(ROT[b])) for i, a, b in zip(*np.unravel_index(idx, arm.shape))]
    with Pool(15) as pool:
        r = np.array(pool.map(_mesh, jobs))
    diff = (r[:, 0] - r[:, 1]) * 1000
    print(f'hull vs mesh (50 konfigurasi lengan ~50 mm): hull - mesh maks {diff.max():+.3f} / min {diff.min():+.3f} mm; '
          f'hull > mesh pada {int((diff > 0).sum())}')
    return dict(n=len(jobs), hull_minus_mesh_max_mm=float(diff.max()), min_mm=float(diff.min()),
                n_hull_above=int((diff > 0).sum()))


if __name__ == '__main__':
    if sys.argv[1] == 'build':
        build()
    elif sys.argv[1] == 'analyse':
        res = analyse()
        res['verify_1deg'] = verify_1deg(res['theta_both_min_deg'])
        res['hull_vs_mesh'] = hull_vs_mesh()
        json.dump(res, open(os.path.join(HERE, 'g29_map.json'), 'w'), indent=1, default=float)
