"""G64-C C1 (docs/p1_g64c.md A0.3, A1 C1): certify T1 at the fold positions. OFFLINE, no ROS.

At p0 = (rail, rot 0) of gantry g, the other gantry REST at its own p0:
  pose   both arms of g at T1: EnvChecker (t{g}_) >= 50 mm AND scene box >= 10 mm
  fold   ONE straight cosine 12-D joint line, both arms REST -> T1 (61 pts): g42 _screen (env, same-gantry, box >= 10 mm
         every point, static torque RNEA + 7.7 + 0.66 <= 14 every point every arm) + CrossGantryChecker (only t{g}_a)
The unfold is the same line reversed (same points, same verdicts).

    python3 g64c_cert.py      -> g64c_cert.json
"""
import json
import math
import os
import sys
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(R, 'p1_g55'))
warnings.simplefilter('ignore')
import g55_make as G55  # noqa: E402

K, G41, G4 = G55.K, G55.G41, G55.G4
O2, PREFIX, REST, LIM, OFFSET, MARGIN = G4.O2, G4.PREFIX, G41.REST, G41.LIM, G41.OFFSET, G4.MARGIN
T1 = np.array(json.load(open(os.path.join(R, 'p1_g64/g64_transit_T.json')))['q6'])
ARMS = {g: [a for a in PREFIX if O2.O.GANTRY[a] == g] for g in (1, 2)}
POS = [(1, 1.40, 0.0), (1, 1.50, 0.0), (2, 0.0, 1.40), (2, 0.0, 1.50)]   # (g, rail g, rail of the other gantry)


def cos_line(a, b, steps=60):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return [list(a + (b - a) * 0.5 * (1.0 - math.cos(math.pi * k / steps))) for k in range(steps + 1)]


def held(g, lin, other_lin, arms_q):
    j = G4.G.base_joints(g, lin, 0.0)
    j[f't{3 - g}_linear_joint'], j[f't{3 - g}_rotation_joint'] = other_lin, 0.0
    for arm, q6 in arms_q.items():
        j.update({f'{PREFIX[arm]}joint_{i}': float(v) for i, v in enumerate(q6, 1)})
    return j


def cert(g, lin, other_lin, cross):
    ec, bx = G4.checkers()
    a, b = ARMS[g]
    st = held(g, lin, other_lin, {a: T1, b: T1})
    q = ec.q_from(st)
    d, wg = ec.check(q, only=(G4.gp(g),))
    bb, wb = bx.dist(ec, q, G4.gp(g))
    pose_ok = bool(d >= G4.ENV_MIN and bb >= G41.BOX_MIN)
    names = [f'{PREFIX[x]}joint_{i}' for x in (a, b) for i in range(1, 7)]
    pts = cos_line(list(REST) * 2, list(T1) * 2)
    st0 = held(g, lin, other_lin, {a: REST, b: REST})
    ok_, line = K._screen(g, names, pts, st0, ec, bx)
    vc, dc, pc, _ = cross.screen_trajectory(names, pts, {n: x for n, x in st0.items() if n not in names}, only=f't{g}_a')
    j2 = max(float(O2.gravity(x, np.array(p[6 * k:6 * k + 6]), lin, 0.0)[1]) for p in pts for k, x in enumerate((a, b)))
    ok = bool(pose_ok and ok_ and vc == 'CLEAR')
    return dict(g=g, rail=lin, other_rail=other_lin, ok=ok,
                pose=dict(env_d=round(float(d), 4), env_geom=wg, box_d=round(float(bb), 4), box_geom=wb, ok=pose_ok),
                fold=dict(line, cross=vc, cross_d=round(float(dc), 4), cross_pair=str(pc), j2_static_max=round(j2, 3),
                          j2_sys=round(j2 + OFFSET[1] + MARGIN[1], 3) if np.ndim(OFFSET) else None))


def main():
    from interarm_collision import CrossGantryChecker
    cross = CrossGantryChecker()
    out = []
    for g, lin, other in POS:
        r = cert(g, lin, other, cross)
        out.append(r)
        f = r['fold']
        print(f"g{g} rel {lin:.2f} (lain {other:.2f}): pose env {r['pose']['env_d'] * 1000:.1f} mm kotak "
              f"{r['pose']['box_d'] * 1000:.1f} mm | lipat env {f['env']} {f['env_d'] * 1000:.1f}, se {f['same']} "
              f"{f['same_d'] * 1000:.1f}, kotak {f['box_d'] * 1000:.1f}, antar {f['cross']} {f['cross_d'] * 1000:.1f} mm, "
              f"torsi lebih {f['tau_excess']:+.2f}, j2 statis maks {f['j2_static_max']:.2f} -> "
              f"{'✓' if r['ok'] else '✗'}", flush=True)
    json.dump(dict(T1=T1.tolist(), rows=out), open(os.path.join(HERE, 'g64c_cert.json'), 'w'), indent=1, default=str)
    c1 = all(r['ok'] for r in out)
    print(f"C1 {'✓' if c1 else '✗'} (g2 dicek dengan g1 di 1.40 dan 1.50)")


if __name__ == '__main__':
    main()
