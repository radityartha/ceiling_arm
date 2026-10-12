#!/usr/bin/env python3
"""CAM-6DOF: build a cam6dof_fit.py cfg from track_median.py jsonl recordings (no hand copying).
plan.json: {"ns", "old": [6], "bottle": {"r", "H"}, "capture": npz|null, "label": "bottle",
            "marks": {name: [x, y]}, "roles": {mark: "fit"|"holdout"},
            "placements": [{"mark", "surf_z", "file"}]}
Per placement: confirmed tracks whose label contains `label`, within 0.35 m (xy) of the mark; the one seen in most messages;
median xyz over the file. Prints n, spread and any rival track (two candidates = flagged, not silently picked).
    python3 cfg_from_tracks.py plan.json cfg.json
"""
import json, sys
import numpy as np
P = json.load(open(sys.argv[1])); obs = []
for pl in P['placements']:
    mk = P['marks'][pl['mark']]; by = {}
    for line in open(pl['file']):
        for o in json.loads(line)['tracks']:
            if P.get('label', 'bottle') in o['label'] and np.hypot(o['x'] - mk[0], o['y'] - mk[1]) < 0.35:
                by.setdefault(o['tid'], []).append((o['x'], o['y'], o['z']))
    if not by:
        print(f"{pl['mark']}@{pl['surf_z']}: NO track within 0.35 m -- skipped"); continue
    tid = max(by, key=lambda k: len(by[k])); X = np.array(by[tid]); med = np.median(X, 0)
    sp = np.percentile(X, 95, 0) - np.percentile(X, 5, 0)
    rivals = {k: len(v) for k, v in by.items() if k != tid}
    print(f"{pl['mark']}@{pl['surf_z']}: tid {tid} n {len(X)} median {med.round(4)} p5-p95 xy {sp[:2].round(3)}"
          + (f"  !! RIVALS {rivals}" if rivals else ''))
    obs.append(dict(name=f"{pl['mark']}@{pl['surf_z']}", mark=pl['mark'], xy=mk, surf_z=pl['surf_z'], track=med.round(5).tolist(),
                    role=P['roles'][pl['mark']]))
json.dump(dict(ns=P['ns'], old=P['old'], bottle=P['bottle'], capture=P.get('capture'), obs=obs), open(sys.argv[2], 'w'), indent=1)
print(f'{len(obs)} obs -> {sys.argv[2]}')
