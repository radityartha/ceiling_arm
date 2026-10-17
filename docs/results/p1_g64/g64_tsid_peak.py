"""G64 torque model, step 3: PHASE-ENVELOPE peak predictor for joint_2, leave-one-session-out. OFFLINE, HW data.

What step 2 showed (g64_tsid_fit): the residual e - RNEA of joint_2 is set by the MOTION PHASE, not by a constant:
lifting (j2 moving in the direction of its gravity torque) about +3.4..+4.8 N.m, lowering about -2.3..-2.8, and
with j2 still it ranges -3..+4.5 (stiction + the Kortex position servo winding up during holds: measured 0.1 -> 6.6 N.m
in 2 s at a constant RNEA 1.78). A per-sample average model misses peaks; the screen needs the PEAK.

Predictor:  peak_hat = max_t ( |tau(t)| + Q[phase(t)] ),  phase from qd2 and sgn(tau2):
   LIFT   sgn(tau2) qd2 >  V0      LOWER  sgn(tau2) qd2 < -V0      STILL  |qd2| <= V0
Q[phase] = quantile QQ of the sample residual  sgn(tau) e_rnea_convention - |tau|  in that phase over the TRAINING
sessions (pooled arms, or per arm). tau = RNEA at the measured q, qd, qdd (as g64_tsid_fit). On a PLAN the same formula
runs on the planned q, qd, qdd (+ the end pose held: STILL at the last point).
Held-out: per segment measured peak |e2| vs peak_hat; under = measured > peak_hat. Baseline = max|tau| + 7.7.

    python3 g64_tsid_peak.py SEGDIR     -> g64_tsid_peak.json + table
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g64_tsid_fit as F  # noqa: E402

V0 = 0.005                 # rad/s
QQS = (0.99, 0.995, 0.999, 1.0)
J = 1


def phases(tau, qd):
    u = np.sign(tau) * qd
    return np.where(u > V0, 0, np.where(u < -V0, 1, 2))      # 0 LIFT, 1 LOWER, 2 STILL


def resid(sg):
    tau = sg['tau'][:, J]
    e = F.SIGN * sg['e'][:, J]
    return np.sign(tau) * e - np.abs(tau), phases(tau, sg['qd'][:, J]), np.abs(tau)


def main(segdir):
    segs = F.load(segdir)
    tt = np.concatenate([s['tau'][:, 1] for s in segs])
    ee = np.concatenate([s['e'][:, 1] for s in segs])
    vv = np.concatenate([s['qd'][:, 1] for s in segs])
    m = (np.abs(vv) < 0.005) & (np.abs(tt) > 2.0)
    F.SIGN = float(np.sign(np.median(ee[m] / tt[m])))
    for s in segs:
        s['r'], s['ph'], s['at'] = resid(s)
        s['peak'] = float(np.max(np.abs(s['e'][:, J])))
    sessions = sorted({s['session'] for s in segs})
    print(f'{len(segs)} segmen, {len(sessions)} sesi; SIGN {F.SIGN:+.0f}')
    allr = np.concatenate([s['r'] for s in segs])
    allp = np.concatenate([s['ph'] for s in segs])
    for p, nm in enumerate(('ANGKAT', 'TURUN', 'DIAM')):
        x = allr[allp == p]
        print(f'  sisa {nm:6s} n {len(x):7d}  p1 {np.quantile(x, 0.01):+.2f}  p50 {np.median(x):+.2f}  p99 {np.quantile(x, 0.99):+.2f}  '
              f'p99.9 {np.quantile(x, 0.999):+.2f}  maks {x.max():+.2f}')
    out = dict(sign=F.SIGN, n_seg=len(segs), sessions=sessions, variants={})
    for var in ('POOL', 'ARM'):
        for qq in QQS:
            rows = []
            for hold in sessions:
                tr = [s for s in segs if s['session'] != hold]
                for s in [s for s in segs if s['session'] == hold]:
                    trr = tr if var == 'POOL' else [x for x in tr if x['arm'] == s['arm']]
                    if not trr:
                        continue
                    r = np.concatenate([x['r'] for x in trr])
                    ph = np.concatenate([x['ph'] for x in trr])
                    Q = np.array([np.quantile(r[ph == p], qq) if np.any(ph == p) else 0.0 for p in range(3)])
                    hat = float(np.max(s['at'] + Q[s['ph']]))
                    rows.append(dict(session=hold, arm=s['arm'], key=s['key'], meas=round(s['peak'], 3),
                                     hat=round(hat, 3), now=round(float(np.max(s['at'])) + F.OFF_NOW, 3),
                                     Q=[round(float(v), 3) for v in Q]))
            meas = np.array([r['meas'] for r in rows])
            hat = np.array([r['hat'] for r in rows])
            now = np.array([r['now'] for r in rows])
            big = meas >= 8.0
            sm = dict(n=len(rows), under_hat=int((meas > hat).sum()), under_now=int((meas > now).sum()),
                      worst_under_hat=round(float((meas - hat).max()), 3),
                      over_hat_mean=round(float((hat - meas).mean()), 3), over_now_mean=round(float((now - meas).mean()), 3),
                      over_hat_mean_ge8=round(float((hat - meas)[big].mean()), 3) if big.any() else None,
                      over_now_mean_ge8=round(float((now - meas)[big].mean()), 3) if big.any() else None,
                      n_ge8=int(big.sum()))
            out['variants'][f'{var}_q{qq}'] = dict(summary=sm, rows=rows)
            print(f'{var:4s} q{qq:<6} {json.dumps(sm)}')
    # full-data Q (pooled + per arm) for deployment
    full = {}
    for arm in ('ALL', 'arm_1', 'arm_2', 'arm_3', 'arm_4'):
        tr = segs if arm == 'ALL' else [s for s in segs if s['arm'] == arm]
        r = np.concatenate([x['r'] for x in tr])
        ph = np.concatenate([x['ph'] for x in tr])
        full[arm] = {str(qq): [round(float(np.quantile(r[ph == p], qq)), 3) for p in range(3)] for qq in QQS}
    out['Q_full'] = full
    print('Q penuh [ANGKAT, TURUN, DIAM]:', json.dumps(full))
    json.dump(out, open(os.path.join(HERE, 'g64_tsid_peak.json'), 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv[1])
