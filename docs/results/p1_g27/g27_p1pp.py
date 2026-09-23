"""G27 A6 (secondary): P1'' = P1' with c_rovh'' and c_skip refitted, leave-one-seed-out over the
4 executed seeds (G24b s1, G26 s1/s2/s13). c_task, c_ret, r UNCHANGED (g25_constants.json).

T_cmd per traverse = the model's own input (planned Delta): G26 from g26_predict_locked.json,
G24b recovered from its P1' prediction, T_cmd = (pred - c_rovh) / r (g25_predict_g24b.json).
    python3 g27_p1pp.py -> g27_p1pp.json
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, '..')
C = json.load(open(os.path.join(R, 'p1_g25/g25_constants.json')))


def seeds():
    out = {}
    p = json.load(open(os.path.join(R, 'p1_g25/g25_predict_g24b.json')))
    comp = json.load(open(os.path.join(R, 'p1_g25/g24b_components.json')))
    ev = []
    for e, c in zip(p['events'], comp['events']):
        assert e['event'] == c['event'] and e['kind'] == c['kind']
        d = dict(kind=e['kind'], pred=e['pred'], meas=c['wall'], skip=False)
        if e['kind'] == 'traverse':
            d['T_cmd'] = (e['pred'] - C['c_rovh']) / C['r']
        ev.append(d)
    out['G24b s1'] = dict(events=ev, makespan=comp['makespan'])
    lock = json.load(open(os.path.join(R, 'p1_g26/g26_predict_locked.json')))
    for s in ('1', '2', '13'):
        comp = json.load(open(os.path.join(R, f'p1_g26/g26_s{s}_components.json')))
        ev = []
        for e, c in zip(lock[s]['events'], comp['events']):
            assert e['event'] == c['event'] and e['kind'] == c['kind']
            d = dict(kind=e['kind'], pred=e['pred'], meas=c['wall'], skip=bool(e.get('skip')))
            if e['kind'] == 'traverse':
                d['T_cmd'] = e['T_cmd']
            ev.append(d)
        out[f'G26 s{s}'] = dict(events=ev, makespan=comp['makespan'])
    return out


def main():
    S = seeds()
    r = C['r']
    rows = []
    for h in S:
        tr = [e['meas'] - r * e['T_cmd'] for k, s in S.items() if k != h for e in s['events'] if e['kind'] == 'traverse']
        sk = [e['meas'] for k, s in S.items() if k != h for e in s['events'] if e['kind'] == 'retract' and e['skip']]
        c_rovh, c_skip = float(np.median(tr)), float(np.median(sk))
        p1 = sum(e['pred'] for e in S[h]['events'])
        p2 = 0.0
        for e in S[h]['events']:
            if e['kind'] == 'traverse':
                p2 += c_rovh + r * e['T_cmd']
            elif e['kind'] == 'retract' and e['skip']:
                p2 += c_skip
            else:
                p2 += e['pred']
        m = S[h]['makespan']
        rows.append(dict(seed=h, n_trav_fit=len(tr), n_skip_fit=len(sk), c_rovh=c_rovh, c_skip=c_skip,
                         meas=m, p1prime=p1, p1pp=p2, err_p1prime_pct=100 * (p1 - m) / m,
                         err_p1pp_pct=100 * (p2 - m) / m))
        print(f"{h:8s}: fit dari {len(tr)} traverse / {len(sk)} retract-dilewati -> c_rovh'' {c_rovh:.3f}, "
              f"c_skip {c_skip:.3f};  terukur {m:.2f}  P1' {p1:.2f} ({100 * (p1 - m) / m:+.2f} %)  "
              f"P1'' {p2:.2f} ({100 * (p2 - m) / m:+.2f} %)")
    a1 = np.mean([abs(x['err_p1prime_pct']) for x in rows])
    a2 = np.mean([abs(x['err_p1pp_pct']) for x in rows])
    all_tr = [e['meas'] - r * e['T_cmd'] for s in S.values() for e in s['events'] if e['kind'] == 'traverse']
    all_sk = [e['meas'] for s in S.values() for e in s['events'] if e['kind'] == 'retract' and e['skip']]
    print(f"|galat| rerata 4 seed: P1' {a1:.2f} %, P1'' (LOSO) {a2:.2f} %;  fit penuh: c_rovh'' "
          f"{np.median(all_tr):.3f} (n = {len(all_tr)}), c_skip {np.median(all_sk):.3f} (n = {len(all_sk)})")
    json.dump(dict(rows=rows, mean_abs_p1prime=a1, mean_abs_p1pp=a2, c_rovh_full=float(np.median(all_tr)),
                   n_trav=len(all_tr), c_skip_full=float(np.median(all_sk)), n_skip=len(all_sk)),
              open(os.path.join(HERE, 'g27_p1pp.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
