"""G28 A2 scorer: V28 (v28_plans.jsonl) + (iv) (g28_screen.json). docs/p1_g28_hw.md A2/A4.

Definitions are the locked ones -- nothing here is tuned after data:
  tuple UNSAFE <=> >= 1 of its 3 samples TORQUE-UNSAFE; SAFE <=> 3/3 PLANNED; else LAIN
  A4 strict: reject correct <=> UNSAFE, accept correct <=> SAFE (LAIN wrong both ways)
  D132: every z=1.40 plan with a trajectory and per-point RNEA j2 > 6.30:
        k_peak2 < N-1 and |static j2|(last point) <= 5.83

    python3 v28_score.py [--in v28_plans.jsonl] [--screen g28_screen.json] [--out v28_score.json]
"""
import argparse
import json
import os
import sys
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'p1_g27'))

M2 = 0.659


def frac(a, b):
    return f'{a}/{b}' + (f' ({a / b:.0%})' if b else '')


def p2_own(r):
    import g27_diag as D
    jn = r['traj']['joint_names']
    q = np.array([r['traj']['pos'][-1][jn.index(n)] for n in r['arm_joints']])
    return float(D.path_max(r['arm'], q, r['rail'])[1])


def score_v28(recs, n_samples=3):
    out, D = {}, {}
    by = defaultdict(list)
    for r in recs:
        by[r['i']].append(r)
    tup = []
    for i, rs in sorted(by.items()):
        vs = [r['verdict'] for r in rs]
        cls = ('UNSAFE' if 'TORQUE-UNSAFE' in vs else
               'SAFE' if len(vs) == n_samples and all(v == 'PLANNED' for v in vs) else 'LAIN')
        t = rs[0]
        tup.append(dict(i=i, z=t['z'], arm=t['arm'], ok4=t['ok4'], n=len(vs), verdicts=vs, cls=cls,
                        n_torque=vs.count('TORQUE-UNSAFE')))
    incomplete = [t['i'] for t in tup if t['n'] != n_samples]
    print(f'tuple: {len(tup)}; sampel tidak lengkap: {incomplete or "tidak ada"}')

    acc = [t for t in tup if t['ok4']]
    rej = [t for t in tup if not t['ok4']]
    safe = [t for t in tup if t['cls'] == 'SAFE']
    a_ok = [t for t in acc if t['cls'] == 'SAFE']
    r_ok = [t for t in rej if t['cls'] == 'UNSAFE']
    print('\nA4 (dikunci G27) -- ketat:')
    print(f'  tolak => UNSAFE : {frac(len(r_ok), len(rej))}   (salah: {[(t["i"], t["cls"]) for t in rej if t["cls"] != "UNSAFE"]})')
    print(f'  terima => SAFE  : {frac(len(a_ok), len(acc))}   (salah: {[(t["i"], t["verdicts"]) for t in acc if t["cls"] != "SAFE"]})')
    prec = len(a_ok) / len(acc) if acc else float('nan')
    rec = len(a_ok) / len(safe) if safe else float('nan')
    print(f'  presisi terima = {prec:.3f}; recall (SAFE) = {rec:.3f} ({len(a_ok)}/{len(safe)} SAFE); '
          f'laju tolak-benar = {frac(len(r_ok), len(rej))}')
    acc_t = [t for t in acc if t['n_torque'] == 0]
    print(f'  varian torsi-saja: terima => 0/3 TORQUE: {frac(len(acc_t), len(acc))}')
    out['A4'] = dict(reject_correct=[len(r_ok), len(rej)], accept_correct=[len(a_ok), len(acc)],
                     precision=prec, recall=rec, accept_torque_only=[len(acc_t), len(acc)],
                     A4_holds=len(r_ok) == len(rej) and len(a_ok) == len(acc))
    print(f'  A4 berlaku penuh: {out["A4"]["A4_holds"]}')

    print('\nverdict per z (per rencana):')
    for z in sorted({r['z'] for r in recs}):
        print(f'  z {z:.2f}: {dict(Counter(r["verdict"] for r in recs if r["z"] == z))}')

    wt = [r for r in recs if r.get('traj')]
    # D132
    s132 = [r for r in wt if r['z'] == 1.4 and r['rnea2'] > 6.30]
    bad = [(r['i'], r['sample'], r['k_peak2'], r['N'] - 1, round(r['stat2_end'], 3)) for r in s132
           if not (r['k_peak2'] < r['N'] - 1 and r['stat2_end'] <= 5.83)]
    D['D132'] = (len(s132) > 0 and not bad, f'{len(s132) - len(bad)}/{len(s132)} memenuhi; langgar {bad}')

    print('\nposisi puncak RNEA j2 per z (rencana ber-lintasan):')
    out['peak_pos'] = {}
    for z in sorted({r['z'] for r in wt}):
        rs = [r for r in wt if r['z'] == z]
        c = Counter('start' if r['k_peak2'] == 0 else 'akhir' if r['k_peak2'] == r['N'] - 1 else 'interior'
                    for r in rs)
        dl = [r['rnea2'] - r['stat2_end'] for r in rs]
        rel = [r['k_peak2'] / (r['N'] - 1) for r in rs]
        print(f'  z {z:.2f}: n {len(rs)} {dict(c)}; RNEA2 - statis_akhir median {np.median(dl):+.3f} '
              f'maks {max(dl):+.3f}; k/(N-1) median {np.median(rel):.2f}')
        out['peak_pos'][str(z)] = dict(n=len(rs), **c, d_end_med=float(np.median(dl)), d_end_max=float(max(dl)))

    t140 = [r for r in recs if r['z'] == 1.4]
    tq140 = [r for r in t140 if r['verdict'] == 'TORQUE-UNSAFE']
    rej140 = [t for t in rej if t['z'] == 1.4]
    D['D133'] = (sum(t['cls'] == 'UNSAFE' for t in rej140) >= 38,
                 f'{sum(t["cls"] == "UNSAFE" for t in rej140)}/{len(rej140)} tuple z1.40 UNSAFE (>= 38)')
    rate = len(tq140) / len(t140) if t140 else float('nan')
    D['D134'] = (0.55 <= rate <= 0.85, f'laju TORQUE z1.40 {len(tq140)}/{len(t140)} = {rate:.3f} ([0.55, 0.85])')
    a132 = [t for t in acc if t['z'] == 1.32]
    D['D135'] = (len(a132) > 0 and all(t['n_torque'] == 0 for t in a132),
                 f'{sum(t["n_torque"] == 0 for t in a132)}/{len(a132)} tuple z1.32 0/3 TORQUE (20/20)')
    D['D136'] = (sum(t['cls'] == 'SAFE' for t in a132) >= 19,
                 f'{sum(t["cls"] == "SAFE" for t in a132)}/{len(a132)} tuple z1.32 SAFE (>= 19)')
    w140 = [r for r in wt if r['z'] == 1.4]
    mid = [r for r in w140 if 4.0 < r['rnea2'] < 6.0]
    D['D137'] = (len(w140) > 0 and len(mid) <= 0.10 * len(w140),
                 f'{len(mid)}/{len(w140)} rencana z1.40 dengan RNEA j2 di (4.0, 6.0) (<= 10 %)')
    wtq = [r for r in w140 if r['verdict'] == 'TORQUE-UNSAFE']
    ok138 = [r for r in wtq if r['stat2_at_peak'] >= r['rnea2'] - M2]
    D['D138'] = (len(wtq) > 0 and len(ok138) >= 0.90 * len(wtq),
                 f'{len(ok138)}/{len(wtq)} TORQUE z1.40: statis@puncak >= RNEA - 0.659 (>= 90 %)')
    p132 = [r for r in wt if r['z'] == 1.32 and r['verdict'] == 'PLANNED']
    ok139 = [r for r in p132 if r['rnea2'] - r['stat2_end'] <= M2]
    D['D139'] = (len(p132) > 0 and len(ok139) >= 0.95 * len(p132),
                 f'{len(ok139)}/{len(p132)} PLANNED z1.32: RNEA2 - statis_akhir <= 0.659 (>= 95 %)')
    d140 = [r['rnea2'] - p2_own(r) for r in wtq]
    ok140 = [x for x in d140 if -0.5 <= x <= 0.7]
    D['D140'] = (len(wtq) > 0 and len(ok140) >= 0.70 * len(wtq),
                 f'{len(ok140)}/{len(wtq)} TORQUE z1.40: RNEA2 - P2_own di [-0.5, 0.7] (>= 70 %); '
                 + (f'median {np.median(d140):+.3f} min {min(d140):+.3f} maks {max(d140):+.3f}' if d140 else ''))
    # D144-D145: locked AFTER the dev smoke, before V28 -- scored apart from the main tally (B5-2)
    p140 = [r for r in w140 if r['verdict'] == 'PLANNED']
    b144 = ([(r['i'], r['sample'], 'P', round(r['stat2_end'], 2)) for r in p140 if r['stat2_end'] >= 4.0]
            + [(r['i'], r['sample'], 'T', round(r['stat2_end'], 2)) for r in wtq if r['stat2_end'] < 4.5])
    D['D144 (dev)'] = (len(p140) + len(wtq) > 0 and not b144,
                       f'PLANNED z1.40 statis_akhir < 4.0 ({len(p140)}), TORQUE >= 4.5 ({len(wtq)}); langgar {b144}')
    d145 = [abs(r['rnea2'] - p2_own(r)) for r in wt]
    D['D145 (dev)'] = (len(d145) > 0 and sum(x <= 0.30 for x in d145) >= 0.80 * len(d145),
                       f'{sum(x <= 0.30 for x in d145)}/{len(d145)} |RNEA2 - P2_own| <= 0.30 (>= 80 %)')
    wall = sum(r['wall'] for r in recs)
    D['D143'] = (wall <= 3600, f'Σ wall rencana {wall / 60:.1f} menit (<= 60; wall sesi ada di log)')
    out['tuples'] = tup
    return out, D


def score_iv(path):
    rows = json.load(open(path))
    D = {}
    ok = [r['seed'] for r in rows if r['ok']]
    print(f'\n(iv): {frac(len(ok), len(rows))} seed LOLOS: {ok}')
    for r in rows:
        if not r['ok']:
            print(f'  seed {r["seed"]}: {r["samples"]}')
    plans = [ln for r in rows for ln in r['log'] if '] task t' in ln]
    c = Counter(ln.rsplit(': ', 1)[1].split(' (')[0] for ln in plans)
    print(f'  per rencana tugas: {dict(c)}')
    D['D141'] = (len(ok) >= 12, f'{len(ok)}/{len(rows)} LOLOS (>= 12/14)')
    D['D142'] = (c.get('TORQUE-UNSAFE', 0) == 0, f'TORQUE-UNSAFE {c.get("TORQUE-UNSAFE", 0)} (== 0)')
    return dict(pass_seeds=ok, n=len(rows), per_plan=dict(c)), D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='inp', default=os.path.join(HERE, 'v28_plans.jsonl'))
    ap.add_argument('--screen', default=os.path.join(HERE, 'g28_screen.json'))
    ap.add_argument('--out', default=os.path.join(HERE, 'v28_score.json'))
    a = ap.parse_args()
    recs = [json.loads(ln) for ln in open(a.inp)]
    out, D = score_v28(recs)
    if os.path.exists(a.screen):
        o2, d2 = score_iv(a.screen)
        out['iv'] = o2
        D.update(d2)
    print('\npapan skor:')
    for k in sorted(D):
        print(f'  {k}: {"✅" if D[k][0] else "❌"}  {D[k][1]}')
    out['D'] = {k: dict(ok=bool(v[0]), txt=v[1]) for k, v in D.items()}
    json.dump(out, open(a.out, 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
