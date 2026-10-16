"""Tugas 1b: deteksi ulang SEMUA frame tersimpan (detektor terkunci, prior sama dgn capture = papan pose itu) + cari juga papan
lain (prior FK) -> penjaga ID per frame -> median_over_frames (>= 50 % frame) seperti capture. Bandingkan dgn poses.jsonl."""
import collections
import glob
import json
import sys

import cv2
import numpy as np

from g63c_common import FING, g63_detect, priors_all, records
import g63b_detect
import g63c_guard as G

out = []
seen = collections.Counter()
for r in records():
    k = seen[r['id']]; seen[r['id']] += 1
    own = r['boards'][0]
    for cam, c in r['cams'].items():
        fs = sorted(glob.glob(f"{FING}/frames/{r['id']}/{cam}/*.png"), key=lambda f: float(f.rsplit('_', 1)[1][:-4]))
        sel = [f for f in fs if abs(float(f.rsplit('_', 1)[1][:-4]) - r['t_wall']) < 15]
        if not sel:
            sel = fs[30 * k:30 * k + 30]
        pr = priors_all(r, cam)
        wh = (c['width'], c['height'])
        res = {b: dict(raw=[], guard=[], notes=collections.Counter()) for b in 'AB'}
        for f in sel:
            gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)   # = g63_capture._img_to_gray (IMREAD_GRAYSCALE beda: A-11 14 vs 16/30)
            det = g63b_detect.detect(gray, {b: pr[b]['uv'] for b in 'AB'}, {'A': 0.042, 'B': 0.042})
            for b in 'AB':
                d = det[b]
                res[b]['raw'].append(d)
                if d['ids']:
                    ok, note = G.check(b, d['uv'], pr, wh)
                    res[b]['notes'][note.split(' ')[0] if not ok else 'ok'] += 1
                    res[b]['guard'].append(d if ok else dict(ids=[], uv=[], method='none'))
                else:
                    res[b]['guard'].append(d)
        rec = dict(id=r['id'], k=k, cam=cam, own=own, n_frames=len(sel),
                   recorded=len(c['boards'][own]['ids']))
        for b in 'AB':
            mr = g63_detect.median_over_frames(res[b]['raw']) if sel else dict(ids=[])
            mg = g63_detect.median_over_frames(res[b]['guard']) if sel else dict(ids=[])
            rec[b] = dict(raw=len(mr['ids']), guard=len(mg['ids']), notes=dict(res[b]['notes']),
                          uv_guard=mg.get('uv', []))
        out.append(rec)
        print(f"{r['id']:5s}#{k} {cam:5s} own {own} n{len(sel):2d} rec {rec['recorded']:2d} | "
              f"A raw {rec['A']['raw']:2d} grd {rec['A']['guard']:2d} {rec['A']['notes']} | "
              f"B raw {rec['B']['raw']:2d} grd {rec['B']['guard']:2d} {rec['B']['notes']}", flush=True)
json.dump(out, open('guard_frames.json', 'w'), indent=1)
