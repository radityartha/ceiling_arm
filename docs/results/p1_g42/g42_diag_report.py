"""G42 A4 report on g42f_diag: binding joint + excess of the 'fails ONLY torque' tuples, and why the torque-ok tuples
fail pose (a) (pose-table body / distance) -- from the caches, no recomputation."""
import json
import math
import os
import sys

import numpy as np

import g42_make as K

K._copy_caches()
K.FIX_CHOICES = True
pose_ok = K.G41.pose_ok_b()
F = K.G41.Feas41(pose_ok)
tq, c4 = K._load(K.DIAG_TORQ), K._load(K.DIAG_C41)
ptab = {tuple(d['key']): d for d in json.load(open(K.G4.POSE_TABLE))}
rows = {r['seed']: r for r in json.load(open('g42f_candidates.json'))}
diag = json.load(open('g42f_diag.json'))
BAR = np.array(K.LIM) - np.array(K.OFFSET) - K.MARGIN       # torq2: taumax <= LIM - OFFSET - MARGIN


def taum(key):
    for src in (F.mine, F.g40, F.g41, tq, F.m38):
        if key in src and 'taumax' in src[key]:
            return np.array(src[key]['taumax'])
    return None


def torque(key):
    for src in (F.mine, F.g40, F.g41, tq):
        if key in src:
            return bool(src[key]['ok3'])
    return F.exact_torque(key)


def c41(key):
    d = F.mine.get(key)
    if d is not None and d['ok3']:
        return bool(d['ok41']), d['whys']
    d = F.g40.get(key)
    if d is not None and d['ok3'] and not d['env2']:
        return False, {'e1 (G40 env2)': 1}
    d = F.g41.get(key)
    if d is not None and d['ok3'] and not d['ok41']:
        return False, d['whys']
    d = c4.get(key)
    return (bool(d['ok41']), d['whys']) if d is not None else (None, {})


print(f'BAR (LIM - OFFSET - MARGIN) per sendi: {np.round(BAR, 2).tolist()}')
kinds = {}
allx, allj, tok_geom, tok_c41 = [], [], {}, {}
for d in diag:
    kinds[d['kind']] = kinds.get(d['kind'], 0) + 1
    nodes = rows[d['seed']]['nodes']
    inst, _ = K.G4.build(nodes, F)
    cols = inst.meta['cols']
    keys = []
    for g in (1, 2):
        raw, _, _ = K.sched._gantry_oracles(K.G4.G.M.CAPS[g], np.array(nodes))
        for j, s in np.argwhere(raw[:, cols, :][d['task']]):
            li, ri = divmod(cols[j], K.NR)
            keys.append((d['node'], g, int(li), int(ri), int(s)))
    ex, jj, nunk = [], [], 0
    rots_tok = []
    for k in keys:
        p, t = bool(pose_ok[k[1]][k[2], k[3]]), torque(k)
        cz, wz = c41(k)
        if p and t is False and cz is True:
            tm = taum(k)
            if tm is None:
                nunk += 1
                continue
            e = tm - BAR
            ex.append(float(e.max()))
            jj.append(int(np.argmax(e)) + 1)
        if t and not p:
            pt = ptab[(k[1], k[2], k[3])]
            why = 'env' if pt['d_env'] < K.G4.ENV_MIN else 'kotak'
            body = (pt['geom'] if why == 'env' else pt['geom_box']) or '?'
            body = body.split('_')[1] if body.startswith('t') else body
            tok_geom[(why, body)] = tok_geom.get((why, body), 0) + 1
            rots_tok.append(round(math.degrees(float(K.REF.rot[k[3]]))))
            for w in (wz or {'(c41b) lolos' if cz else '(c41b) ?': 1}):
                tok_c41[w] = tok_c41.get(w, 0) + 1
    allx += ex
    allj += jj
    print(f"s{d['seed']} t{d['task']} {d['kind']}: HANYA-torsi {len(ex)} (taumax tak ada {nunk}); lebih minimum "
          f"{min(ex) if ex else float('nan'):+.2f} N.m, median {np.median(ex) if ex else float('nan'):+.2f}, sendi "
          f"pengikat {dict(zip(*np.unique(jj, return_counts=True))) if jj else {}}; torsi-ok rot {sorted(set(rots_tok))}")
allx = np.array(allx)
print(f'jenis tugas: {kinds}')
print(f'HANYA-torsi semua: n {len(allx)}, lebih min {allx.min():+.3f} N.m, p10 {np.percentile(allx, 10):+.2f}, median '
      f'{np.median(allx):+.2f}; sendi pengikat {dict(zip(*np.unique(allj, return_counts=True)))}; '
      f'<= +0.5 N.m: {int((allx <= 0.5).sum())}, <= MARGIN j2 {K.MARGIN[1]:.4f}: {int((allx <= K.MARGIN[1]).sum())}, <= +1.0: {int((allx <= 1.0).sum())}')
print(f'tuple torsi-ok yang gagal pose (a): sebab pose {tok_geom}; (c41b) mereka {tok_c41}')
