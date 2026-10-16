"""Tugas 3: validasi silang antar-lengan. Sumber X: fit HANYA lengan X (kamera + T_lb X + j2-j5 X; j1/rotasi vertikal
dikunci = gauge, dibuktikan rank). Target Y: kamera DIBEKUKAN dari X, fit T_lb Y + parameter lengan Y (varian), nilai Y:
sigma FIT, G3 uji-tahan, G6′ R0. Juga: pose relatif rgbd->rgbd2 (invarian gauge dunia) dari tiap fit dibandingkan."""
import json
import sys

import numpy as np

from g63c_kin import Fit, jn, cam_diff, DS
from g63_common import inv

A, B = 'AJ', 'BJ'
GROT = {A: 't1_rotation_joint', B: 't2_rotation_joint'}
def J(b, ks): return [('j', f'{b}.j{k}', [jn(b, k)]) for k in ks]
def BASE(b, ks): return [('base', f'{b}.base.{"rx ry rz tx ty tz".split()[k]}', b, k) for k in ks]
TARGET = {
    'a: j1-j5': lambda b: J(b, range(1, 6)),
    'd: rotG+j2-j5': lambda b: [('j', f'{b}.grot', [GROT[b]])] + J(b, range(2, 6)),
    'b: j1-j5+basis(rx,ry,t)': lambda b: J(b, range(1, 6)) + BASE(b, [0, 1, 3, 4, 5]),
}


def rel(Twc):
    return inv(Twc['rgbd']) @ Twc['rgbd2']


def report(tag, f):
    ho = f.lateral(('HOLDOUT',))
    g3 = {c: (np.median(v) * 1e3, np.percentile(v, 95) * 1e3) for c, v in ho.items()}
    ok = all(m <= 15 and p <= 30 for m, p in g3.values())
    g6 = f.g6p()
    g6max = max((max(v) for v in g6.values() if v), default=float('nan')) * 1e3
    print(f'  {tag}: k={f.jac.shape[1]} sigma {f.sigma:.2f} px rank {f.rank()["rank"]}/{f.jac.shape[1]} | G3 ' +
          ', '.join(f'{c} {m:.1f}/{p:.1f}' for c, (m, p) in g3.items()) + f' -> {"LULUS" if ok else "GAGAL"} | '
          f'G6′ maks {g6max:.2f} mm -> {"LULUS" if g6max <= 2 else "GAGAL"}', flush=True)
    return dict(sigma=f.sigma, G3=g3, G3_ok=ok, G6p_max_mm=g6max)


out = {}
src = {}
for X in (A, B):
    print(f'=== sumber {X} saja', flush=True)
    for lock_name, ex in (('j1 bebas (uji gauge)', J(X, range(1, 6))), ('j1 dikunci', J(X, range(2, 6)))):
        f = Fit(ex, boards=[X]).solve()
        rk = f.rank()
        print(f'  [{lock_name}] sigma {f.sigma:.2f} px rank {rk["rank"]}/{rk["n"]} rel_min {rk["rel_min"]:.1e} '
              f'null~{rk["null"]} | std ' + ', '.join(f'{c} {a:.3f}°/{t * 1e3:.1f}mm' for c, (a, t) in f.cam_std().items()),
              flush=True)
    src[X] = f
    Twc = f.unpack(f.x)[0]
    Y = B if X == A else A
    for tn, mk in TARGET.items():
        g = Fit(mk(Y), boards=[Y], cams_fixed=Twc).solve()
        xe = g.unpack(g.x)[2]
        sd = g.std()[-len(xe):]
        r = report(f'target {Y} [{tn}] kamera dari {X}', g)
        print('     ' + ', '.join(f'{e[1]} {np.degrees(v) if e[0] == "j" or e[3] < 3 else v * 1e3:+.2f}±'
                                  f'{np.degrees(s) if e[0] == "j" or e[3] < 3 else s * 1e3:.2f}' for e, v, s in zip(g.ex, xe, sd)))
        out[f'{X}->{Y} {tn}'] = r
    # self-fit target for reference (kamera bebas, hanya lengan Y, j1 dikunci)
print('=== kesepakatan kamera antar-lengan (fit lengan tunggal, j1 dikunci)')
TA, TB = src[A].unpack(src[A].x)[0], src[B].unpack(src[B].x)[0]
for c in ('rgbd', 'rgbd2'):
    a, t = cam_diff(TA[c], TB[c])
    print(f'  {c} absolut: {a:.2f}° / {t * 100:.1f} cm (std A {src[A].cam_std()[c]}, B {src[B].cam_std()[c]})')
a, t = cam_diff(rel(TA), rel(TB))
print(f'  relatif rgbd->rgbd2 (invarian gauge dunia): {a:.2f}° / {t * 100:.1f} cm')
mod = json.load(open('models_all.json'))
for name in ('Ma j1-j5/lengan', 'Mb Ma-j1A+basisB(rx,ry,t)', 'Md rot gantry+j2-j5', 'M0 kaku'):
    T = {c: np.array(v) for c, v in mod[name]['cams'].items()}
    for tag, S_ in (('A', TA), ('B', TB)):
        a, t = cam_diff(rel(T), rel(S_))
        print(f'  relatif {name} vs fit-{tag}: {a:.2f}° / {t * 100:.1f} cm')
json.dump(out, open('cross.json', 'w'), indent=1, default=float)
