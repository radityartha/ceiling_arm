"""G28 dev diagnostic (OFFLINE, smoke-mock DEV tuples only, never V28): is the endpoint of a SAFE
z = 1.40 plan inside oracle'''s solution set? docs/p1_g28_hw.md B0b.

Decision (written before computing): per safe endpoint q_f
  KENDALA  q_f violates an oracle constraint (|pos err| > 2 mm, tool-axis tilt > 2.83 deg, joint limits)
  LUBANG   q_f satisfies them but max|q_f - s| > 0.05 rad for EVERY oracle''' solution s (grid roll, no tilt)
  ADA      some solution within 0.05 rad -> T3b ("every straight line from REST crosses") must be re-checked
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g27'))
import g27_diag as D  # noqa: E402

O2 = D.O2


def fk(arm, q6, rail):
    A = O2.model(arm)
    pin, m, d = A['pin'], A['m'], A['d']
    q = np.zeros(m.nq)
    q[A['rail']] = rail
    q[A['iq']] = q6
    pin.framesForwardKinematics(m, d, q)
    return d.oMf[A['tool']]


def main():
    recs = [json.loads(ln) for ln in open(os.path.join(HERE, 'smoke_mock_plans.jsonl'))]
    out = []
    sol_cache = {}
    for r in recs:
        if r['z'] != 1.4:
            continue
        arm, rail, xyz = r['arm'], r['rail'], np.array(r['xyz'])
        jn = r['traj']['joint_names']
        qf = np.array([r['traj']['pos'][-1][jn.index(n)] for n in r['arm_joints']])
        T = fk(arm, qf, rail)
        perr = np.linalg.norm(T.translation - xyz) * 1000
        tilt = np.degrees(np.arccos(np.clip(T.rotation[:, 2] @ O2.D0[:, 2], -1, 1)))
        A = O2.model(arm)
        inlim = bool(np.all(qf >= A['lo'] - 1e-6) and np.all(qf <= A['hi'] + 1e-6))
        k = tuple(r['key'])
        if k not in sol_cache:
            res = O2.solve(list(xyz), rail, arm, tilt=True, envelope=True, keep_solutions=True)
            S = np.array([s for _, s in res['solutions']])
            g2 = np.array([O2.gravity(arm, s, rail)[1] for s in S])
            P2 = np.array([D.path_max(arm, s, rail)[1] for s in S])
            sol_cache[k] = (res, S, g2, P2)
            print(f'tuple {k} {arm}: oracle n_sol {res["n_sol"]}, statis j2 solusi [{g2.min():.2f}, {g2.max():.2f}], '
                  f'P2 garis lurus [{P2.min():.2f}, {P2.max():.2f}]')
        res, S, g2, P2 = sol_cache[k]
        dist = np.max(np.abs(S - qf), axis=1)
        j = int(np.argmin(dist))
        stat = float(O2.gravity(arm, qf, rail)[1])
        p2f = float(D.path_max(arm, qf, rail)[1])
        viol = perr > 2.0 or tilt > 2.83 or not inlim
        cls = 'KENDALA' if viol else ('ADA' if dist[j] <= 0.05 else 'LUBANG')
        row = dict(i=r['i'], sample=r['sample'], verdict=r['verdict'], perr_mm=perr, tilt_deg=tilt, in_limits=inlim,
                   stat2=stat, P2_line=p2f, rnea2=r['rnea2'], nearest_dist_rad=float(dist[j]),
                   nearest_stat2=float(g2[j]), nearest_P2=float(P2[j]), q_final=qf.tolist(),
                   q_nearest=S[j].tolist(), cls=cls)
        out.append(row)
        print(f'  [{r["i"]}] s{r["sample"] + 1} {r["verdict"]:13s} pos {perr:.2f} mm, miring {tilt:.2f} deg, batas {inlim}; '
              f'statis j2 {stat:.2f}, P2 garis {p2f:.2f}, RNEA {r["rnea2"]:.2f}; solusi terdekat {dist[j]:.3f} rad '
              f'(statis {g2[j]:.2f}, P2 {P2[j]:.2f}) -> {cls}')
        print(f'      q_f     {np.round(qf, 3).tolist()}\n      terdekat {np.round(S[j], 3).tolist()}')
    json.dump(out, open(os.path.join(HERE, 'dev_branch.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
