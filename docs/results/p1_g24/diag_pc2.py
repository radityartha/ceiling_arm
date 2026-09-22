"""G24 B diagnosis of PC2 misses: is q_final on a branch the sweep never found (enumeration), or near-grid (resolution)?
From q_final: its roll psi_f; trace to nearest grid roll by 6-D Newton continuation; compare with sweep solutions."""
import json, numpy as np
import oracle2 as O2, oracle as O
d = json.load(open('g24_validate.json'))
for c in [c for c in d['C'] if c['g24_dmin'] >= 0.15]:
    A = O2.model(c['arm']); pin, m, dd = A['pin'], A['m'], A['d']
    q = np.zeros(m.nq); q[A['rail']] = c['rail']; q[A['iq']] = c['q_final']
    pin.framesForwardKinematics(m, dd, q); R = dd.oMf[A['tool']].rotation
    M = O2.D0.T @ R                      # = Rz(psi) * small tilt
    psi = np.degrees(np.arctan2(M[1, 0], M[0, 0])) % 360
    r = O2.solve(c['xyz'], c['rail'], c['arm'], keep_solutions=True)
    k = int(round(psi / 5.0)) % 72
    # newton at grid roll k seeded from q_final
    Rt = O2.D0 @ O2._rz(np.radians(5.0 * k)); qq = q.copy(); ok = False
    for _ in range(100):
        pin.framesForwardKinematics(m, dd, qq); T = dd.oMf[A['tool']]
        ep = np.array(c['xyz']) - T.translation; er = pin.log3(Rt @ T.rotation.T)
        if np.linalg.norm(ep) < O2.POS_TOL and np.linalg.norm(er) < O2.ROT_TOL: ok = True; break
        J = pin.computeFrameJacobian(m, dd, qq, A['tool'], pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, A['iv']]
        qq[A['iq']] = np.clip(qq[A['iq']] + J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), np.r_[ep, er]), A['lo'], A['hi'])
    near = min((np.max(np.abs(np.array(s) - qq[A['iq']])) for kk, s in r['solutions'] if kk == k), default=np.inf) if ok else None
    g = O2.gravity(c['arm'], np.array(c['q_final']), c['rail'])
    print(f"{c['sess']} #{c['trial']} {c['arm']} psi_f {psi:6.1f} tilt {c['fk_err_deg']:.2f}deg dmin {c['g24_dmin']:.3f} | "
          f"proj->grid {k*5}: conv {ok}, dist to sweep sol {near if near is None else round(near,4)} | "
          f"j2 q_final {g[1]:.2f} sweep j2max {r['taumax'][1]:.2f} nsol {r['n_sol']} jenuh {r['saturated']} "
          f"| q_final {np.round(c['q_final'],2).tolist()}")
