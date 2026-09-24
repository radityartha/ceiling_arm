"""G29 A0: INSTRUMEN geometri rotasi gantry -- dilihat SEBELUM §A dikunci. OFFLINE, nol HW.

URDF = G23 (sha f02e7c53...), = /tmp/reach_dwell_live.urdf yang dipakai G22-G26.
  1. sumbu rotasi (FK rotation_link) vs (lin, y_g) dan tinggi z; placement struktur.
  2. jari-jari horizontal dari sumbu: struktur, tiap lengan REST (verteks hull), rentang z.
  3. jarak struktur-struktur antar-gantry (pinocchio 3-D, primitif) vs sched_coll.pair_distance
     (G9 BLOCK 2-D, W0/W0b lulus) pada grid (dx, rot1, rot2): dua implementasi independen.
  4. jarak minimum antar-gantry per kelas pasangan (struktur-struktur / lengan-struktur /
     lengan-lengan) pada rot = 0, lengan REST, dx sapuan.
"""
import json
import os
import sys
import warnings

import numpy as np

warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '../../../scripts'))
sys.path.insert(0, os.path.join(HERE, '../../../ros2_ws/src/reachability_gng'))
import pinocchio as pin  # noqa: E402
from interarm_collision import CrossGantryChecker, ARM_PREFIX  # noqa: E402
from reachability_gng import sched_coll as SC  # noqa: E402

URDF = os.path.join(HERE, '../p1_g23/reach_dwell_live.urdf')
REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
ARM = tuple(ARM_PREFIX.values())
STRUCT = ('platform_link', 'rotation_link', 'mount_plate_left', 'mount_plate_right')


def full_checker():
    """CrossGantryChecker hulls, but ALL t1 x t2 pairs (incl. structure-structure)."""
    c = CrossGantryChecker(urdf=URDF)
    ia = [i for i, n in enumerate(c.names) if n.startswith('t1_')]
    ib = [i for i, n in enumerate(c.names) if n.startswith('t2_')]
    c.geom.removeAllCollisionPairs()
    for i in ia:
        for j in ib:
            c.geom.addCollisionPair(pin.CollisionPair(i, j))
    c.geom_data = c.geom.createData()
    return c


def state(l1, r1, l2, r2):
    s = {'t1_linear_joint': l1, 't2_linear_joint': l2,
         't1_rotation_joint': r1, 't2_rotation_joint': r2}
    for p in ARM:
        s.update({f'{p}joint_{i}': v for i, v in enumerate(REST, 1)})
    return s


def cls(a, b):
    sa, sb = a.startswith(ARM), b.startswith(ARM)
    return 'LL' if sa and sb else ('SS' if not sa and not sb else 'LS')


def by_class(c, q):
    pin.updateGeometryPlacements(c.model, c.data, c.geom, c.geom_data, q)
    pin.computeDistances(c.model, c.data, c.geom, c.geom_data, q)
    out = {}
    for k, res in enumerate(c.geom_data.distanceResults):
        cp = c.geom.collisionPairs[k]
        a, b = c.names[cp.first], c.names[cp.second]
        k2 = cls(a, b)
        if k2 not in out or res.min_distance < out[k2][0]:
            out[k2] = (res.min_distance, a, b)
    return out


def verts(go, M):
    g = go.geometry
    if hasattr(g, 'points'):
        P = np.array([g.points(i) for i in range(g.num_points)]) if callable(getattr(g, 'points', None)) \
            else np.asarray(g.points)
    elif isinstance(g, pin.hppfcl.Box) if hasattr(pin, 'hppfcl') else False:
        h = g.halfSide
        P = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    else:
        # cylinder: rim samples
        r, hl = g.radius, g.halfLength
        a = np.linspace(0, 2 * np.pi, 73)
        P = np.concatenate([np.c_[r * np.cos(a), r * np.sin(a), np.full_like(a, z)] for z in (-hl, hl)])
    return (M.rotation @ P.T).T + M.translation


def main():
    c = full_checker()
    out = {}
    # ---- 1. axis
    ax = {}
    for g, lin, rot in ((1, 0.55, 0.0), (1, 0.55, 0.7), (2, 1.2, -1.1)):
        q = c.q_from(state(lin if g == 1 else 0.0, rot if g == 1 else 0.0,
                           lin if g == 2 else 0.0, rot if g == 2 else 0.0))
        pin.framesForwardKinematics(c.model, c.data, q)
        M = c.data.oMf[c.model.getFrameId(f't{g}_rotation_link')]
        ax[f'g{g} lin {lin} rot {rot}'] = dict(origin=M.translation.round(6).tolist(),
                                                z_axis=M.rotation[:, 2].round(6).tolist(),
                                                bar_long_axis_world=M.rotation[:, 1].round(6).tolist())
    out['axis'] = ax
    for k, v in ax.items():
        print(f'1. sumbu {k}: asal {v["origin"]}, sumbu-z {v["z_axis"]}, batang (y lokal) -> dunia {v["bar_long_axis_world"]}')

    # ---- 2. radii from axis, z ranges (rot 0, lin 0.55 / 0.0, REST)
    q = c.q_from(state(0.55, 0.0, 0.0, 0.0))
    pin.updateGeometryPlacements(c.model, c.data, c.geom, c.geom_data, q)
    rad = {}
    for i, go in enumerate(c.geom.geometryObjects):
        n = go.name
        g = 1 if n.startswith('t1_') else 2
        centre = np.array([0.55 if g == 1 else 0.0, SC.Y_GANTRY[g]])
        V = verts(go, c.geom_data.oMg[i])
        grp = n[:6] if n.startswith(ARM) else f't{g}_struct:' + n[3:-2]
        r = np.linalg.norm(V[:, :2] - centre, axis=1)
        e = rad.setdefault(grp, [0.0, np.inf, -np.inf])
        e[0] = max(e[0], float(r.max()))
        e[1] = min(e[1], float(V[:, 2].min()))
        e[2] = max(e[2], float(V[:, 2].max()))
    out['radius_z'] = rad
    for k, (r, z0, z1) in sorted(rad.items()):
        print(f'2. {k:32s} r_maks dari sumbu {r*1000:7.1f} mm   z [{z0:.4f}, {z1:.4f}]')

    # ---- 3. structure-structure: pinocchio 3-D vs G9 2-D, random + structured
    rng = np.random.default_rng(29)
    ss = [k for k, cp in enumerate(c.geom.collisionPairs)
          if cls(c.names[cp.first], c.names[cp.second]) == 'SS']
    samples = [(0.8, 0.0, 0.8, 0.0), (0.8, -np.pi / 2, 0.8, np.pi / 2), (0.8, -np.pi / 2, 0.85, np.pi / 2),
               (0.8, -np.pi / 2, 0.9, np.pi / 2), (0.8, -np.pi / 2, 0.895, np.pi / 2)]
    samples += [(rng.uniform(0, 1.6), rng.uniform(-np.pi, np.pi), rng.uniform(0, 1.6),
                 rng.uniform(-np.pi, np.pi)) for _ in range(3000)]
    near = []
    for _ in range(600):   # biased near contact: both rotated towards each other, small dx
        l1 = rng.uniform(0.2, 1.4)
        near.append((l1, -np.pi / 2 + rng.normal(0, 0.4), l1 + rng.normal(0, 0.12),
                     np.pi / 2 + rng.normal(0, 0.4)))
    samples += near
    ssnp = [k for k in ss if 'platform' not in c.names[c.geom.collisionPairs[k].first]
            and 'platform' not in c.names[c.geom.collisionPairs[k].second]]
    diffs, n_pos, both_neg, sign_bad, rows, plat_bind = [], 0, 0, 0, [], 0
    for (l1, r1, l2, r2) in samples:
        q = c.q_from({'t1_linear_joint': l1, 't1_rotation_joint': r1,
                      't2_linear_joint': l2, 't2_rotation_joint': r2}, q=c.q_from(state(0, 0, 0, 0)))
        pin.updateGeometryPlacements(c.model, c.data, c.geom, c.geom_data, q)
        pin.computeDistances(c.model, c.data, c.geom, c.geom_data, q)
        d3all = min(c.geom_data.distanceResults[k].min_distance for k in ss)
        d3 = min(c.geom_data.distanceResults[k].min_distance for k in ssnp)   # G9 A2.1: platform dropped
        plat_bind += d3all < d3 and d3all <= 0.0
        d2 = float(SC.pair_distance(l1, r1, l2, r2))
        if d2 > 0 and d3 > 0:
            diffs.append(d3 - d2)
            n_pos += 1
        elif d2 <= 0 and d3 <= 0:
            both_neg += 1
        else:
            sign_bad += 1
            rows.append((l1, r1, l2, r2, d3, d2))
    diffs = np.array(diffs)
    out['ss_vs_g9'] = dict(n=len(samples), both_pos=n_pos, both_contact=both_neg, sign_mismatch=sign_bad,
                           max_abs_diff_m=float(np.abs(diffs).max()), mismatches=rows[:20],
                           platform_contact_not_in_g9=int(plat_bind))
    print(f'3. SS (tanpa platform, = G9 A2.1) pinocchio-3D vs G9 2-D: platform kontak saat G9 bebas: {plat_bind}; {len(samples)} sampel; keduanya >0 {n_pos} (maks |beda| '
          f'{np.abs(diffs).max()*1000:.4f} mm), keduanya kontak {both_neg}, TANDA BEDA {sign_bad}')
    for r in rows[:5]:
        print('   beda tanda:', np.round(r, 4))
    for s in samples[:5]:
        q = c.q_from({'t1_linear_joint': s[0], 't1_rotation_joint': s[1],
                      't2_linear_joint': s[2], 't2_rotation_joint': s[3]}, q=c.q_from(state(0, 0, 0, 0)))
        b = by_class(c, q)
        print(f'   {np.round(s, 3)}: SS {b["SS"][0]*1000:7.1f} mm ({b["SS"][1]} <-> {b["SS"][2]}), '
              f'G9 {float(SC.pair_distance(*s))*1000:7.1f}')

    # ---- 4. by class at rot 0, REST, dx sweep
    print('4. rot 0/0, lengan REST, per kelas pasangan:')
    out['rot0_by_class'] = {}
    for dx in (0.0, 0.2, 0.4, 0.8, 1.2, 1.6):
        b = by_class(c, c.q_from(state(dx, 0.0, 0.0, 0.0)))
        out['rot0_by_class'][dx] = {k: [v[0], v[1], v[2]] for k, v in b.items()}
        print(f'   dx {dx:.1f}: ' + '  '.join(f'{k} {v[0]*1000:6.1f} ({v[1]}<->{v[2]})' for k, v in sorted(b.items())))
    json.dump(out, open(os.path.join(HERE, 'g29_a0.json'), 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
