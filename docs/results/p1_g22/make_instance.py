"""G22 A2 syarat (i)-(iii): instance mekanis, OFFLINE, nol perangkat keras.

sched.py TIDAK diubah: gen_real apa adanya, lalu R0 (subset pose rot = 0) dan
R1 (p0 = rel terbaca) diterapkan pada objek Instance, bukan pada solver.

    python3 make_instance.py [--p0-lin1 0.55] [--p0-lin2 0.0] [--seeds 0-49]
Tulis g22_candidates.json (semua seed, syarat per seed) ke folder ini.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '../../../ros2_ws/src/reachability_gng')
sys.path.insert(0, PKG)
from reachability_gng import sched, sched_coll  # noqa: E402

MAPS = tuple(os.path.join(PKG, f'data/cap_g{g}_rail160.npz') for g in (1, 2))
T_FOLD = {'FISIK': 50.80, 'DINDING': 126.80}          # g21 A1, hanya ini
V = sched.V_LIN_MM_S


def restrict_rot0(inst, p0_lin):
    """R0 + R1. Same Instance class, pose axis cut to rot == 0."""
    out = {}
    for g in inst.gantries:
        P = inst.poses[g]
        keep = np.flatnonzero(np.abs(P[:, 1]) < 1e-9)
        out[g] = keep
    poses = {g: inst.poses[g][out[g]] for g in inst.gantries}
    reach = {g: inst.reach[g][:, out[g], :] for g in inst.gantries}
    zone = {g: inst.zone[g][:, out[g], :] for g in inst.gantries}
    hand = {g: inst.hand[g][:, out[g]] for g in inst.gantries}
    p0 = {g: int(np.argmin(np.abs(poses[g][:, 0] - p0_lin[g])))
          for g in inst.gantries}
    for g in inst.gantries:
        assert abs(poses[g][p0[g], 0] - p0_lin[g]) < 1e-9, (g, p0_lin[g])
    return sched.Instance(inst.kind, poses, reach, zone, hand, p0, inst.xyz,
                          inst.dwell, inst.t_fold, inst.label + '+rot0',
                          dict(inst.meta))


def with_tfold(inst, tf):
    return sched.Instance(inst.kind, inst.poses, inst.reach, inst.zone,
                          inst.hand, inst.p0, inst.xyz, inst.dwell, tf,
                          inst.label, dict(inst.meta))


def decompose(inst, sol, tf):
    """Per gantry: pindah, Σ T_lin, Σ T_cmd (bridge, g19 A2), Σ dwell."""
    out = {}
    for g, stops in sol.stops.items():
        prev = inst.p0[g]
        mv, tlin, tcmd, dw, legs = 0, 0.0, 0.0, 0.0, []
        for s in stops:
            p = s['pose']
            d_mm = abs(inst.poses[g][p, 0] - inst.poses[g][prev, 0]) * 1000
            if p != prev:
                mv += 1
                tl = 0.29 + d_mm / V
                tc = max(3.0, np.pi * d_mm / (2 * 0.9 * V))
                tlin += tl
                tcmd += tc
                legs.append(dict(from_mm=round(inst.poses[g][prev, 0] * 1000, 1),
                                 to_mm=round(inst.poses[g][p, 0] * 1000, 1),
                                 T_lin=round(tl, 3), T_cmd=round(tc, 3)))
            dw += s['dur']
            prev = p
        out[g] = dict(moves=mv, T_lin=tlin, T_cmd=tcmd, dwell=dw, legs=legs,
                      finish_model=tlin + mv * tf + dw,
                      finish_solver=sol.finish[g])
    return out


def sched_json(inst, sol):
    res = {}
    for g, stops in sol.stops.items():
        res[g] = []
        for s in stops:
            res[g].append(dict(
                rail_m=round(float(inst.poses[g][s['pose'], 0]), 3),
                rot_deg=round(float(np.degrees(inst.poses[g][s['pose'], 1])), 1),
                start=round(s['start'], 3), dur=s['dur'],
                tasks={int(i): a for i, a in s['assign'].items()},
                xyz={int(i): [round(float(v), 4) for v in inst.xyz[i]]
                     for i in s['assign']}))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--p0-lin1', type=float, default=0.55)
    ap.add_argument('--p0-lin2', type=float, default=0.00)
    ap.add_argument('--seeds', default='0-49')
    ap.add_argument('--fallback', action='store_true',
                    help='A2 cadangan: (ii) = >=1 pindah total + kedua gantry bertugas')
    ap.add_argument('--out', default=os.path.join(HERE, 'g22_candidates.json'))
    a = ap.parse_args()
    lo, hi = (int(x) for x in a.seeds.split('-'))
    p0_lin = {1: a.p0_lin1, 2: a.p0_lin2}
    rows = []
    for seed in range(lo, hi + 1):
        base = sched.gen_real(6, seed, 0, (1, 2), maps=MAPS)
        inst = restrict_rot0(base, p0_lin)
        row = dict(seed=seed, nodes=base.meta['nodes'])
        try:
            sols = {k: sched.solve_exact(with_tfold(inst, tf))
                    for k, tf in T_FOLD.items()}
        except ValueError as e:
            row.update(ok_i=False, why=str(e))
            rows.append(row)
            print(f'seed {seed:2d}: (i) GAGAL -- {e}')
            continue
        f = sols['FISIK']
        dec = {k: decompose(inst, s, T_FOLD[k]) for k, s in sols.items()}
        mv = {g: dec['FISIK'][g]['moves'] for g in (1, 2)}
        ntask = {g: bin(f.assign[g]).count('1') for g in (1, 2)}
        ok_ii = all(mv[g] >= 1 for g in (1, 2)) if not a.fallback else \
            (sum(mv.values()) >= 1 and all(ntask[g] >= 1 for g in (1, 2)))
        conf = sched_coll.schedule_conflict(inst, f.stops)
        same = all(
            [(s['pose'], s['tasks']) for s in sols['FISIK'].stops[g]] ==
            [(s['pose'], s['tasks']) for s in sols['DINDING'].stops[g]]
            for g in (1, 2))
        row.update(ok_i=True, ok_ii=bool(ok_ii), conflict=repr(conf),
                   moves=mv, ntask=ntask, same_schedule_FISIK_DINDING=same,
                   makespan={k: s.makespan for k, s in sols.items()},
                   decomp={k: {str(g): v for g, v in d.items()}
                           for k, d in dec.items()},
                   schedule_FISIK={str(g): v for g, v in
                                   sched_json(inst, f).items()},
                   schedule_DINDING={str(g): v for g, v in
                                     sched_json(inst, sols['DINDING']).items()})
        rows.append(row)
        print(f'seed {seed:2d}: pindah g1/g2 {mv[1]}/{mv[2]}, tugas {ntask[1]}/'
              f'{ntask[2]}, makespan FISIK {f.makespan:7.2f} DINDING '
              f'{sols["DINDING"].makespan:7.2f}, jadwal sama {same}, '
              f'konflik {conf!r}  (ii) {"LOLOS" if ok_ii else "-"}')
    json.dump(rows, open(a.out, 'w'), indent=1, default=float)
    ok = [r['seed'] for r in rows if r.get('ok_ii')]
    print(f'\nlolos (i)-(iii): {len(ok)} seed: {ok}\n  -> {a.out}')


if __name__ == '__main__':
    main()
