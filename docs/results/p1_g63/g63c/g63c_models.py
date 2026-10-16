"""Tugas 2: bandingkan model kinematik (fit FIT kedua lengan, nilai HOLDOUT G3 + G6′ R0 + rank)."""
import json
import sys

import numpy as np

from g63c_kin import Fit, jn, cam_diff, DS
from g63_common import incumbent_T

A, B = 'AJ', 'BJ'
def J(b, ks): return [('j', f'{b}.j{k}', [jn(b, k)]) for k in ks]
def BASE(b, ks): return [('base', f'{b}.base.{"rx ry rz tx ty tz".split()[k]}', b, k) for k in ks]
SH = [('j', 'shared.j1', [jn(A, 1), jn(B, 1)])]
GR = [('j', 'g1.rot', ['t1_rotation_joint']), ('j', 'g2.rot', ['t2_rotation_joint'])]
MODELS = {
    'M0 kaku': [],
    'Ma j1-j5/lengan': J(A, range(1, 6)) + J(B, range(1, 6)),
    'Mb_raw Ma+basis B 6': J(A, range(1, 6)) + J(B, range(1, 6)) + BASE(B, range(6)),
    'Mb Ma-j1A+basisB(rx,ry,t)': J(A, range(2, 6)) + J(B, range(1, 6)) + BASE(B, [0, 1, 3, 4, 5]),
    'Mt Ma-j1A+basisB(t)': J(A, range(2, 6)) + J(B, range(1, 6)) + BASE(B, [3, 4, 5]),
    'Mc1 j1 bersama saja': SH,
    'Mc2 rot gantry saja': GR,
    'Md rot gantry+j2-j5': GR + J(A, range(2, 6)) + J(B, range(2, 6)),
    'Me j1 bersama+j2-j5': SH + J(A, range(2, 6)) + J(B, range(2, 6)),
}
only = sys.argv[1:] or list(MODELS)
out = {}
for name in only:
    f = Fit(MODELS[name]).solve()
    rk = f.rank()
    n = f.jac.shape[0]
    rss = (f.sigma ** 2) * f.dof
    bic = n * np.log(rss / n) + f.jac.shape[1] * np.log(n)
    ho = f.lateral(('HOLDOUT',))
    g3 = {c: (np.median(v) * 1e3, np.percentile(v, 95) * 1e3) for c, v in ho.items()}
    g3ok = all(m <= 15 and p <= 30 for m, p in g3.values())
    g6 = f.g6p()
    g6max = max(max(v) for v in g6.values() if v) * 1e3
    Twc, _Tlb, xe = f.unpack(f.x)
    sd = f.std()
    vs = {c: cam_diff(incumbent_T(c), Twc[c]) for c in Twc}
    print(f'=== {name}: k={f.jac.shape[1]} n={n} sigma {f.sigma:.2f} px BIC {bic:.0f} | rank {rk["rank"]}/{rk["n"]} '
          f'rel_min {rk["rel_min"]:.1e} rel_2nd {rk["rel_2nd"]:.1e} null~{rk["null"]}')
    print('   G3 uji-tahan med/p95 mm ' + ', '.join(f'{c} {m:.1f}/{p:.1f}' for c, (m, p) in g3.items()) +
          f' -> {"LULUS" if g3ok else "GAGAL"} | G6′ R0 RMS mm ' +
          ', '.join(f'{b} {np.round(np.array(v) * 1e3, 2).tolist()}' for b, v in g6.items()) +
          f' -> {"LULUS" if g6max <= 2 else "GAGAL"}')
    print('   kamera std ' + ', '.join(f'{c} {a:.3f}°/{t * 1e3:.1f}mm' for c, (a, t) in f.cam_std().items()) +
          ' | vs incumbent ' + ', '.join(f'{c} {a:.2f}°/{t * 100:.1f}cm' for c, (a, t) in vs.items()))
    k0 = len(f.x) - len(xe)
    if len(xe):
        print('   extras ' + ', '.join(f'{e[1]} {np.degrees(v) if e[0] == "j" or (e[0] == "base" and e[3] < 3) else v * 1e3:+.2f}'
                                     f'±{np.degrees(s) if e[0] == "j" or (e[0] == "base" and e[3] < 3) else s * 1e3:.2f}'
                                     for e, v, s in zip(f.ex, xe, sd[k0:])) + '  (° atau mm)')
    out[name] = dict(k=int(f.jac.shape[1]), sigma=f.sigma, bic=float(bic), rank=rk, G3=g3, G3_ok=g3ok, G6p_mm=g6,
                     cams={c: Twc[c].tolist() for c in Twc}, cam_std=f.cam_std(), x=f.x.tolist(),
                     extras=[e[1] for e in f.ex])
    sys.stdout.flush()
json.dump(out, open('models_' + ('all' if len(only) == len(MODELS) else 'part') + '.json', 'w'), indent=1, default=float)
