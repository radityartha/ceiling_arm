#!/usr/bin/env python3
"""DEMO-ZONE-LIVE §A2/A3: skor satu tanda dari rekaman 15 s (track_median.py jsonl).
    python3 live_score.py C3 <dir>      # baca <dir>/<M>_raw.jsonl, <M>_cor.jsonl, <M>_rej.jsonl -> cetak + <dir>/<M>_score.json
Gerbang (tidak dilonggarkan): L1 tepat SATU track diterima, tid sama, hadir >= 90 % pesan; L2 |median xy terkoreksi - tanda|
<= 5 cm; L3 median z MENTAH (dari x_trk/y_trk/z_trk track diterima) in [0.79, 0.84]."""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import zone_correct_node as N  # noqa: E402

MARKS = {'C3': (1.8486, 0.6786), 'M1': (1.7497, 0.5493)}      # TCP arm_1 terukur (p1_rgbd2_zone fit_data.json)


def rows(p):
    return [json.loads(l)['tracks'] for l in open(p)] if os.path.exists(p) else []


def score(mark, d):
    mk = np.array(MARKS[mark])
    raw, cor, rej = (rows(os.path.join(d, '%s_%s.jsonl' % (mark, s))) for s in ('raw', 'cor', 'rej'))
    n = len(cor)
    tids = sorted({t['tid'] for m in cor for t in m})
    per = [len(m) for m in cor]
    one = n > 0 and len(tids) == 1 and max(per) == 1
    pres = sum(1 for m in cor if len(m) == 1) / n if n else 0.0
    out = dict(mark=mark, n_cor=n, n_raw=len(raw), n_rej=len(rej), accepted_tids=tids, presence=round(pres, 3))
    out['L1'] = bool(one and pres >= 0.90)
    if tids:
        A = [t for m in cor for t in m if t['tid'] == tids[0]]
        cxy = np.median([[t['x'], t['y']] for t in A], 0)
        trk = np.median([[t['x_trk'], t['y_trk'], t['z_trk']] for t in A], 0)
        e = float(np.linalg.norm(cxy - mk))
        off = N.correct_xy(trk[:2], *N.load_p3())                # D653: P3 luring dari median mentah
        out.update(cor_xy=cxy.round(4).tolist(), raw_xyz=trk.round(4).tolist(), err_cor_cm=round(100 * e, 2),
                   err_raw_cm=round(100 * float(np.linalg.norm(trk[:2] - mk)), 2),
                   spread_p5_p95_mm=(1000 * (np.percentile([[t['x'], t['y']] for t in A], 95, 0)
                                             - np.percentile([[t['x'], t['y']] for t in A], 5, 0))).round(1).tolist(),
                   node_vs_offline_mm=round(1000 * float(np.linalg.norm(off - cxy)), 2),
                   label=A[-1]['label'], conf_med=round(float(np.median([t['conf'] for t in A])), 2))
        out['L2'] = bool(e <= 0.05)
        out['L3'] = bool(0.79 <= trk[2] <= 0.84)
    else:
        out['L2'] = out['L3'] = False
    rlab = sorted({(r['tid'], r['label'], r['reason'].split(' (')[0]) for m in rej for r in m})
    out['rejected'] = [list(x) for x in rlab]
    out['raw_tids'] = sorted({(t['tid'], t['label']) for m in raw for t in m})
    out['PASS'] = out['L1'] and out['L2'] and out['L3']
    return out


if __name__ == '__main__':
    mark, d = sys.argv[1], sys.argv[2]
    s = score(mark, d)
    json.dump(s, open(os.path.join(d, '%s_score.json' % mark), 'w'), indent=1)
    for k, v in s.items():
        print('%-18s %s' % (k, v))
