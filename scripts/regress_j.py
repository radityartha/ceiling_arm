#!/usr/bin/env python3
"""OLS regression of traj_energy on normalized J-terms, over the friction-model
E3 picks (energy+nearest+random pooled), to recalibrate w_*/ref_* by REGRESSION
(not the old Spearman-heuristic in analyze_calib.py) -- Jalan B step 3.

ref_* = median raw term (unbiased-ish since we pool across 3 different selection
policies rather than one). w_* = OLS coefficient on the ref-normalized term
(so w_* stays a comparable "priority" knob, same convention as before).
"""
import csv
import numpy as np

CSVS = ['/tmp/e3b_energy.csv', '/tmp/e3b_nearest.csv', '/tmp/e3b_random.csv']

TERMS = ['d_gantry_lin', 'd_gantry_rot', 'd_arm', 'ee_dist', 'hold', 'manip']
SIGN = {'d_gantry_lin': +1, 'd_gantry_rot': +1, 'd_arm': +1,
        'ee_dist': +1, 'hold': +1, 'manip': -1}

rows = []
for fn in CSVS:
    with open(fn, newline='') as f:
        rows.extend(csv.DictReader(f))

def ff(x):
    try:
        v = float(x)
        return v if np.isfinite(v) else None
    except (TypeError, ValueError):
        return None

succ = [r for r in rows if ff(r.get('success')) == 1.0]
print(f'pooled {len(rows)} rows -> {len(succ)} successful picks '
      f'from {CSVS}')

data = {}
for t in TERMS:
    col = t if t != 'd_gantry_rot' else t  # raw column name matches
    vals = []
    for r in succ:
        v = ff(r.get(t))
        vals.append(abs(v) if v is not None else np.nan)
    data[t] = np.array(vals)
energy = np.array([ff(r.get('traj_energy')) for r in succ])

mask = np.all([np.isfinite(data[t]) for t in TERMS], axis=0) & np.isfinite(energy)
print(f'{mask.sum()} rows with all terms + finite traj_energy')

ref = {t: float(np.median(data[t][mask])) for t in TERMS}
X = np.column_stack([SIGN[t] * data[t][mask] / ref[t] for t in TERMS])
y = energy[mask]

# OLS with intercept
Xd = np.column_stack([X, np.ones(len(y))])
coef, res, rank, sv = np.linalg.lstsq(Xd, y, rcond=None)
w = dict(zip(TERMS, coef[:-1]))
intercept = coef[-1]

yhat = Xd @ coef
ss_res = float(np.sum((y - yhat) ** 2))
ss_tot = float(np.sum((y - y.mean()) ** 2))
r2 = 1 - ss_res / ss_tot

print('\n=== OLS fit: traj_energy ~ sum(sign_t * w_t * |term_t| / ref_t) + b ===')
print(f'{"term":<14} {"ref(median)":>12} {"w(OLS coef)":>12}')
for t in TERMS:
    print(f'{t:<14} {ref[t]:>12.4f} {w[t]:>12.4f}')
print(f'intercept b = {intercept:.4f}')
print(f'R^2 = {r2:.4f}  (n={mask.sum()})')

# Spearman of the resulting J vs traj_energy (sanity, same metric used before)
def spearman(a, b):
    def ranks(v):
        order = np.argsort(v)
        r = np.empty(len(v))
        r[order] = np.arange(1, len(v) + 1)
        return r
    ra, rb = ranks(a), ranks(b)
    return float(np.corrcoef(ra, rb)[0, 1])

J = X @ coef[:-1]
rho = spearman(J, y)
print(f'Spearman(J_new, traj_energy) = {rho:.4f}')

print('\n--- suggested declare_parameter defaults ---')
name_map = {'d_gantry_lin': 'gantry_lin', 'd_gantry_rot': 'gantry_rot',
            'd_arm': 'arm', 'ee_dist': 'dist', 'hold': 'hold', 'manip': 'manip'}
for t in TERMS:
    n = name_map[t]
    wv = w[t]
    print(f"w_{n} = {wv:.3f}   ref_{n} = {ref[t]:.4f}")
