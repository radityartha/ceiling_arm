#!/usr/bin/env python3
"""G63d pemindaian luring: kelayakan rencana vs (a) syarat papan dua-kamera, (b) posisi rel saat TANGKAP (tanda unting dibuat
di g1 1.40 / g2 0 lalu rel digeser -- butuh izin operator; di sini HANYA hitungan). Margin 0.05 dan 0.10."""
import copy
import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g63d_common as G  # noqa: E402
import g63d_plan as P  # noqa: E402

cp = G.cam_params()
env = P.env_points()
base = P.candidates(env)
out = []
cfgs = [(None, 1)] + [((g1, g2), 2) for g1, g2 in ((0.0, 0.0), (0.0, 1.4), (1.6, 1.4), (0.7, 0.7), (1.6, 0.0))]
for margin in (0.05,):
    for rails, mb in cfgs:
        c = copy.deepcopy(base)
        P.visibility(c, cp, env, margin, rails)
        nv = {k: sum(b['vis'][k]['ok'] for b in c) for k in G.CAMS}
        (sc, sel, info), pool, both = P.select(c, cp, n=6, margin=margin, min_both=mb, restarts=40)
        row = dict(margin=margin, rails=rails, min_both=mb, n_vis=nv, pool=len(pool), both=len(both), score=sc,
                   pred=info, sel=None if sel is None else [(b['surf'], b['cx'], b['cy'], round(float(np.degrees(b['psi']))),
                                                             {k: b['vis'][k]['ok'] for k in G.CAMS}) for b in sel])
        out.append(row)
        print(json.dumps(row, default=str), flush=True)
json.dump(out, open(os.path.join(G.HERE, 'g63d_scan.json'), 'w'), indent=1, default=str)
print('SELESAI', flush=True)
