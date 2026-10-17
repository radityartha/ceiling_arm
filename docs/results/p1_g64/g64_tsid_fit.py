"""G64 torque model, step 2: fit + leave-one-session-out validation on the HW motion segments. OFFLINE.

Physics (G64 0c): Gen3 lite `effort` = torque derived from MOTOR CURRENT (user guide Tab. 21). The motor supplies the
load torque plus gearbox/bearing losses that oppose motion, so
    SIGN e_j = tau_j + Fc s_j + Fv qd_j + a+ tau_j max(u_j, 0) + a- tau_j max(-u_j, 0),   u_j = s_j sgn(tau_j)
s_j = Dahl friction state (builds over SIGMA rad of motion, HOLDS when the joint stops: the measured hysteresis);
SIGN = sign convention of Kortex effort vs RNEA (fitted: -1). a+/a- = gearbox loss lifting / lowering.
tau = full RNEA (pinocchio, oracle2 reduced arm model) at the MEASURED q, qd, qdd (Savitzky-Golay on a uniform 100 Hz
resample). At rest s ~ 0 -> e ~ tau (G16: held 9.60 measured vs 9.25 RNEA). Linear in (Fc, Fv, a+, a-) -> least squares.
Fitted per joint j1..j3 (KA large actuators) and per variant: POOL (all arms) / ARM (per arm).

Validation = leave-one-SESSION-out (session = CSV file): per segment, peak |e_j2| measured vs peak |model| on the same
measured path. Baselines: RNEA alone and the current screen |RNEA| + 7.7. Screen bound = model peak + delta, delta = q99
of (measured peak - model peak) over the TRAINING sessions; held-out under-predictions (measured > bound) counted.

    python3 g64_tsid_fit.py SEGDIR        -> g64_tsid_fit.json + table
"""
import glob
import json
import os
import sys
import warnings

import numpy as np
from scipy.signal import savgol_filter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g24'))
warnings.simplefilter('ignore')
import oracle2 as O2  # noqa: E402

SIGMA = 0.01               # rad, Dahl friction build-up displacement
FS = 100.0
SG_WIN, SG_POLY = 21, 3    # 0.2 s window
JOINTS = (0, 1, 2)         # j1..j3
OFF_NOW = 7.7
MIN_PEAK = 1.0             # segments whose measured j2 peak < 1 N.m carry no information (mock-like / tiny moves)


def rnea_series(arm, q, qd, qdd):
    A = O2.model(arm)
    pin, m, d = A['pin'], A['m'], A['d']
    iq, iv = A['iq'], A['iv']
    Q = np.zeros(m.nq)
    V = np.zeros(m.nv)
    Acc = np.zeros(m.nv)
    out = np.empty_like(q)
    for k in range(len(q)):
        Q[iq], V[iv], Acc[iv] = q[k], qd[k], qdd[k]
        out[k] = pin.rnea(m, d, Q, V, Acc)[iv]
    return out


def load(segdir):
    segs = []
    for fn in sorted(glob.glob(os.path.join(segdir, '*.npz'))):
        z = np.load(fn)
        for mt in json.loads(str(z['meta'])):
            t, q, e = z[f"t_{mt['key']}"], z[f"q_{mt['key']}"].astype(float), z[f"e_{mt['key']}"].astype(float)
            if len(t) < 3 * SG_WIN:
                continue
            tu = np.arange(t[0], t[-1], 1.0 / FS)
            qu = np.column_stack([np.interp(tu, t, q[:, j]) for j in range(6)])
            eu = np.column_stack([np.interp(tu, t, e[:, j]) for j in range(6)])
            qs = savgol_filter(qu, SG_WIN, SG_POLY, axis=0)
            qd = savgol_filter(qu, SG_WIN, SG_POLY, deriv=1, delta=1.0 / FS, axis=0)
            qdd = savgol_filter(qu, SG_WIN, SG_POLY, deriv=2, delta=1.0 / FS, axis=0)
            if np.max(np.abs(eu[:, 1])) < MIN_PEAK:
                continue
            tau = rnea_series(mt['arm'], qs, qd, qdd)
            segs.append(dict(session=mt['file'], arm=mt['arm'], key=mt['key'], t0=mt['t0'], tau=tau, qd=qd, e=eu))
    return segs


def dahl(qd, sigma=None, s0=0.0):
    """Dahl friction state in [-1, 1]: builds with DISPLACEMENT (not time), holds when the joint stops -- the
    measured 'hold stays where the last motion left it' (stiction/hysteresis). ds = (dq / sigma)(1 - s sgn(dq))."""
    sigma = SIGMA if sigma is None else sigma
    dq = qd / FS
    s = np.empty_like(qd)
    x = s0
    for k, d in enumerate(dq):
        x += (d / sigma) * (1.0 - x * np.sign(d))
        x = min(1.0, max(-1.0, x))
        s[k] = x
    return s


def design(tau, qd, s0=0.0):
    """RNEA convention (tau sign). u > 0 = friction state in the direction of tau (lifting), u < 0 = lowering."""
    s = dahl(qd, s0=s0)
    u = s * np.sign(tau)
    return np.stack([s, qd, tau * np.maximum(u, 0.0), tau * np.maximum(-u, 0.0)], axis=-1)


def fit(segs, j, groups=None):
    X = np.concatenate([design(sg['tau'][:, j], sg['qd'][:, j]) for sg in segs])
    y = np.concatenate([SIGN * sg['e'][:, j] - sg['tau'][:, j] for sg in segs])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return c


def predict(sg, j, c):
    return sg['tau'][:, j] + design(sg['tau'][:, j], sg['qd'][:, j]) @ c      # RNEA convention


SIGN = 1.0


def main(segdir):
    global SIGN
    segs = load(segdir)
    print(f'{len(segs)} segmen, {len({s["session"] for s in segs})} sesi, sampel '
          f'{sum(len(s["e"]) for s in segs)}', flush=True)
    # effort sign vs RNEA sign: slow samples, |tau| > 2
    tt = np.concatenate([s['tau'][:, 1] for s in segs])
    ee = np.concatenate([s['e'][:, 1] for s in segs])
    vv = np.concatenate([s['qd'][:, 1] for s in segs])
    m = (np.abs(vv) < 0.005) & (np.abs(tt) > 2.0)
    SIGN = float(np.sign(np.median(ee[m] / tt[m])))
    print(f'tanda effort/RNEA (diam, |tau|>2, n {int(m.sum())}): median rasio {np.median(ee[m] / tt[m]):.3f} -> SIGN {SIGN:+.0f}')
    sessions = sorted({s['session'] for s in segs})
    res = dict(n_seg=len(segs), sessions=sessions, sign=SIGN, variants={})
    for var in ('POOL', 'ARM'):
        rows = []
        coefs = {}
        for hold in sessions:
            train = [s for s in segs if s['session'] != hold]
            test = [s for s in segs if s['session'] == hold]
            for arm in sorted({s['arm'] for s in test}):
                tr = train if var == 'POOL' else [s for s in train if s['arm'] == arm]
                if not tr:
                    continue
                c = {j: fit(tr, j) for j in JOINTS}
                # training residual of the PEAK -> delta
                dtr = [np.max(np.abs(s['e'][:, 1])) - np.max(np.abs(predict(s, 1, c[1]))) for s in tr]
                delta = float(np.quantile(dtr, 0.99))
                for s in [s for s in test if s['arm'] == arm]:
                    pk = float(np.max(np.abs(s['e'][:, 1])))
                    pm = float(np.max(np.abs(predict(s, 1, c[1]))))
                    pr = float(np.max(np.abs(s['tau'][:, 1])))
                    rows.append(dict(session=hold, arm=arm, key=s['key'], meas=round(pk, 3), model=round(pm, 3),
                                     rnea=round(pr, 3), bound=round(pm + delta, 3), delta=round(delta, 3),
                                     now=round(pr + OFF_NOW, 3)))
        full = {}
        for arm in ('ALL',) if var == 'POOL' else ('arm_1', 'arm_2', 'arm_3', 'arm_4'):
            tr = segs if arm == 'ALL' else [s for s in segs if s['arm'] == arm]
            if tr:
                full[arm] = {f'j{j + 1}': [round(float(v), 4) for v in fit(tr, j)] for j in JOINTS}
        err = np.array([r['meas'] - r['model'] for r in rows])
        big = [r for r in rows if r['meas'] >= 8.0]
        eb = np.array([r['meas'] - r['model'] for r in big])
        summ = dict(
            n=len(rows),
            model_err_p50=round(float(np.median(err)), 3), model_abs_p95=round(float(np.quantile(np.abs(err), 0.95)), 3),
            model_abs_p95_meas_ge8=round(float(np.quantile(np.abs(eb), 0.95)), 3) if len(eb) else None,
            under_bound=sum(r['meas'] > r['bound'] for r in rows), under_now=sum(r['meas'] > r['now'] for r in rows),
            mean_bound_minus_meas=round(float(np.mean([r['bound'] - r['meas'] for r in rows])), 3),
            mean_now_minus_meas=round(float(np.mean([r['now'] - r['meas'] for r in rows])), 3),
            ge8_mean_bound_minus_meas=round(float(np.mean([r['bound'] - r['meas'] for r in big])), 3) if big else None,
            ge8_mean_now_minus_meas=round(float(np.mean([r['now'] - r['meas'] for r in big])), 3) if big else None,
            rnea_err_p50=round(float(np.median([r['meas'] - r['rnea'] for r in rows])), 3))
        res['variants'][var] = dict(summary=summ, coef_full=full, rows=rows)
        print(var, json.dumps(summ))
        print('   koef penuh (Fc, Fv, a+, a-):', json.dumps(full))
    json.dump(res, open(os.path.join(HERE, 'g64_tsid_fit.json'), 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv[1])
