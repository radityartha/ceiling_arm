#!/usr/bin/env python3
"""ARM1-ZONE A0 (ii): offline self-test of arm_fk_check.arm_filter (function extracted from the script, no ROS).
Arm-like surface (two 5 cm-radius cylinders, 0.30 m, elbow) seen at +0.15/+0.02 m with 5 mm noise, plus a vertical
'curtain' plane = 40 % of the points -> must recover the shift and drop the curtain. Control: curtain only -> a tangent strip is still
kept (~25 %), so kept_frac alone cannot tell them apart; median residual of the kept points can (arm ~ noise, strip ~ band)."""
import ast, os, sys
import numpy as np
from scipy.spatial import cKDTree
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../p1_calib_charuco/arm_fk_check.py')).read()
fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == 'arm_filter')
ns = {'np': np}; exec(compile(ast.Module([fn], []), 'arm_filter', 'exec'), ns); arm_filter = ns['arm_filter']
rng = np.random.default_rng(1)
def cyl(a, b, r, n):
    u = rng.random(n); th = rng.random(n) * 2 * np.pi; d = (b - a) / np.linalg.norm(b - a)
    e1 = np.cross(d, [0, 0, 1.0]) if abs(d[2]) < 0.9 else np.cross(d, [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(d, e1)
    return a + np.outer(u, b - a) + r * (np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2))
A0, A1, A2 = np.array([1.9, 0.36, 1.45]), np.array([1.95, 0.40, 1.30]), np.array([1.85, 0.45, 1.15])
S = np.r_[cyl(A0, A1, 0.05, 20000), cyl(A1, A2, 0.05, 20000)]; tree = cKDTree(S)
t_true = np.array([0.15, 0.02, 0.0])
arm = np.r_[cyl(A0, A1, 0.05, 1500), cyl(A1, A2, 0.05, 1500)]
arm = arm[(arm - [2.4, 0.9, 1.3]) @ [-0.5, -0.5, 0] > -0.3]            # camera-facing half only, like a real view
arm = arm + t_true + rng.normal(0, 0.005, arm.shape)
curtain = np.c_[np.full(2000, 2.25), rng.uniform(0.2, 0.7, 2000), rng.uniform(1.15, 1.45, 2000)]
ok = True
for lab, P, want in (('arm+curtain', np.r_[arm, curtain], 'shift'), ('curtain only', curtain, 'reject')):
    Pw, t, m, n0, d = arm_filter(P, tree, 0.03)
    is_arm = Pw[:, 0] < 2.2; med = np.median(d[m]) * 1000
    print(f'{lab:13s} n {len(Pw)} (arm {is_arm.sum()}) kept {m.sum()} dropped {(~m).sum()} kept@0 {n0} med {med:.1f} mm t {t.round(4)} '
          f'(true {t_true}) kept-curtain {int((m & (Pw[:, 0] > 2.24) & (Pw[:, 0] < 2.26)).sum())}')
    if want == 'shift':
        ok &= np.linalg.norm(t - t_true) < 0.01 and m[is_arm].mean() > 0.9 and not m[~is_arm].any() and med < 8
    else:
        ok &= med > 10        # tangent strip: residuals spread over the band, not at the noise level
print('PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)
