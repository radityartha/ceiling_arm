"""G61 C3 (docs/p1_g61_offline.md A3): measured torque of every recorded G57 task vs (1) the probe's screen (RNEA over
the planned trajectory + per-actuator offset, the last plan attempt before MOVED), (2) the plan's RNEA alone, (3) an
oracle''''-style straight joint line from the MEASURED start (REST or the previous task pose) to the measured end
(+ offset + m^p, G27 T3a), (4) static gravity along the MEASURED path (+ offset). HW data, OFFLINE.

    python3 g61_c3_torque.py <windows.pkl from g61_c3_extract.py>   -> g61_c3_torque.json + table
"""
import json
import math
import os
import pickle
import re
import sys

import numpy as np
import pinocchio as pin

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.join(HERE, '..', 'p1_g57', 'hw')
URDF = '/tmp/reach_dwell_live.urdf'
REST = np.array([-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885])
LIM = np.array([10.0, 14.0, 10.0, 7.0, 7.0, 7.0])
OFF = np.array([6.6, 7.7, 6.6, 1.98, 1.98, 1.98])
MP = np.array(json.load(open(os.path.join(HERE, '..', 'p1_g27', 'g27_t3.json')))['t3a']['m_p'])
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
RE_TQ = re.compile(r'torsi RNEA\+offset\(per-aktuator\) vs rating: (.*?)  \(terketat')
RE_J = re.compile(r'(\d)=([0-9.]+)->([0-9.]+)/(\d+)')
M = pin.buildModelFromUrdf(URDF)
D = M.createData()


def grav(arm, Q):
    """|static gravity torque| of the arm's 6 joints for each row of Q (n x 6)."""
    iq = [M.joints[M.getJointId(f'{PREFIX[arm]}joint_{i}')].idx_q for i in range(1, 7)]
    iv = [M.joints[M.getJointId(f'{PREFIX[arm]}joint_{i}')].idx_v for i in range(1, 7)]
    out = []
    for q6 in Q:
        q = pin.neutral(M)
        q[iq] = q6
        out.append(np.abs(pin.computeGeneralizedGravity(M, D, q)[iv]))
    return np.array(out)


def probe_pred(run, idx):
    """(rnea[6], corr[6]) of the last torque line before 'moveit MOVED' (None if the task never moved)."""
    fn = os.path.join(HW, f'p1hw_par_{run}_ev{idx:02d}_task.log')
    if not os.path.exists(fn):
        return None
    last = None
    for ln in open(fn, errors='ignore'):
        m = RE_TQ.search(ln)
        if m:
            js = {int(j): (float(v), float(c)) for j, v, c, _ in RE_J.findall(m.group(1))}
            last = (np.array([js[j][0] for j in range(1, 7)]), np.array([js[j][1] for j in range(1, 7)]))
        if 'moveit MOVED' in ln:
            return last
    return None


def one(w):
    arm = w['arm']
    names = [f'{PREFIX[arm]}joint_{i}' for i in range(1, 7)]
    t = w['js'][names[0]][0]
    Q = np.stack([np.interp(t, w['js'][n][0], w['js'][n][1]) for n in names], axis=1)
    E = np.stack([np.abs(np.interp(t, w['js'][n][0], w['js'][n][2])) for n in names], axis=1)
    inw = (t >= w['t_start']) & (t <= w['t_end'])
    t, Q, E = t[inw], Q[inw], E[inw]
    q_s = np.median(Q[t <= w['t_start'] + 1.0], axis=0)
    q_e = np.median(Q[t >= w['t_end'] - 1.0], axis=0)
    dev_e = np.max(np.abs(Q - q_e), axis=1)
    dev_s = np.max(np.abs(Q - q_s), axis=1)
    far = np.nonzero(dev_e > math.radians(0.5))[0]
    t_arr = t[far[-1]] if len(far) else t[0]
    mv = np.nonzero(dev_s > math.radians(0.2))[0]
    t_mv = t[mv[0]] if len(mv) else None
    k = int(np.argmax(E[:, 1]))
    pk = E.max(axis=0)
    hold = np.median(E[t >= w['t_end'] - 1.5], axis=0)
    line = np.array([q_s + (q_e - q_s) * s for s in np.linspace(0, 1, 101)])
    P = grav(arm, line).max(axis=0)
    sub = np.linspace(0, len(t) - 1, min(len(t), 300)).astype(int)
    G = grav(arm, Q[sub])
    g_pk = grav(arm, Q[k:k + 1])[0]
    pp = probe_pred(w['run'], w['idx'])
    moved = t_mv is not None and np.max(np.abs(q_e - q_s)) > math.radians(1.0)
    return dict(run=w['run'], idx=w['idx'], arm=arm, verdict=w['verdict'], moved=bool(moved),
                start_rest=bool(np.max(np.abs(q_s - REST)) < math.radians(2.0)),
                peak=pk.tolist(), peak_j2=float(pk[1]), t_peak_rel=float(t[k] - w['t_start']),
                q2_peak_deg=float(math.degrees(Q[k, 1])), q2_start_deg=float(math.degrees(q_s[1])),
                q2_end_deg=float(math.degrees(q_e[1])),
                phase='transit' if t[k] < t_arr else 'tahan', t_move_rel=None if t_mv is None else float(t_mv - w['t_start']),
                t_arrive_rel=float(t_arr - w['t_start']), exec_s=None if t_mv is None else float(t_arr - t_mv),
                hold=hold.tolist(), hold_j2=float(hold[1]),
                probe_rnea=None if pp is None else pp[0].tolist(), probe_corr=None if pp is None else pp[1].tolist(),
                line_P=P.tolist(), line_pred=(P + OFF + MP).tolist(),
                path_grav=G.max(axis=0).tolist(), path_pred=(G.max(axis=0) + OFF).tolist(),
                grav_at_peak_j2=float(g_pk[1]), meas_minus_grav_at_peak_j2=float(pk[1] - g_pk[1]))


def main():
    W = [w for w in pickle.load(open(sys.argv[1], 'rb')) if w['covered']]
    rows = [one(w) for w in W]
    mv = [r for r in rows if r['moved'] and r['probe_corr'] is not None]
    ex = [r['peak_j2'] - r['probe_corr'][1] for r in mv]
    eo = [r['peak_j2'] - r['probe_rnea'][1] for r in mv]
    hi = [r for r in mv if r['peak_j2'] > 11.0]
    anyj = [(r['run'], r['idx'], j + 1, round(r['peak'][j] - r['probe_corr'][j], 3)) for r in mv for j in range(6)
            if r['peak'][j] > r['probe_corr'][j]]
    s = dict(n_windows=len(rows), n_moved_with_pred=len(mv),
             j2_le_probe=sum(x <= 0 for x in ex), j2_exceed_max=max(ex), j2_exceed=[(r['run'], r['idx'], round(x, 3))
                                                                                    for r, x in zip(mv, ex) if x > 0],
             j2_eff_offset_max=max(eo), j2_eff_offset_p50=float(np.median(eo)), j2_eff_offset_p95=float(np.percentile(eo, 95)),
             n_peak_gt11=len(hi), peak_gt11_transit=sum(r['phase'] == 'transit' for r in hi),
             line_ge_meas_j2=sum(r['line_pred'][1] >= r['peak_j2'] for r in mv),
             path_ge_meas_j2=sum(r['path_pred'][1] >= r['peak_j2'] for r in mv),
             n_nonrest_start=sum(not r['start_rest'] for r in mv),
             nonrest_j2_exceed_line=sum(r['line_pred'][1] < r['peak_j2'] for r in mv if not r['start_rest']),
             rest_j2_exceed_line=sum(r['line_pred'][1] < r['peak_j2'] for r in mv if r['start_rest']),
             any_joint_exceed_probe=anyj, peak_j2_max=max(r['peak_j2'] for r in rows),
             hold_j2_max=max(r['hold_j2'] for r in mv))
    print(f'{"run":22s} {"ev":>3s} {"arm":5s} {"st":4s} {"pk2":>6s} {"fase":7s} {"probe2":>6s} {"rnea2":>5s} {"ukur-rn":>7s} '
          f'{"garis":>6s} {"jalur":>6s} {"tahan":>5s} {"q2 s->pk->e":>18s}')
    for r in sorted(mv, key=lambda r: -r['peak_j2']):
        print(f'{r["run"]:22s} {r["idx"]:3d} {r["arm"]:5s} {"REST" if r["start_rest"] else "tgs":4s} {r["peak_j2"]:6.2f} '
              f'{r["phase"]:7s} {r["probe_corr"][1]:6.2f} {r["probe_rnea"][1]:5.2f} {r["peak_j2"] - r["probe_rnea"][1]:+7.2f} '
              f'{r["line_pred"][1]:6.2f} {r["path_pred"][1]:6.2f} {r["hold_j2"]:5.2f} '
              f'{r["q2_start_deg"]:5.1f}>{r["q2_peak_deg"]:5.1f}>{r["q2_end_deg"]:5.1f}')
    print(json.dumps(s, indent=1, default=float))
    json.dump(dict(summary=s, rows=rows), open(os.path.join(HERE, 'g61_c3_torque.json'), 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
