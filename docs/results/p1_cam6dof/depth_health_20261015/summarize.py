#!/usr/bin/env python3
"""Summarize a rec_health jsonl: depth valid-fraction series, fps, and metadata fields joined by timestamp."""
import json
import sys

import numpy as np

rows = [json.loads(x) for x in open(sys.argv[1])]
dep = [r for r in rows if r['kind'] == 'depth']
meta = {round(r['t'], 4): r['meta'] for r in rows if r['kind'] == 'meta'}
ir = [r for r in rows if r['kind'] == 'ir']
v = np.array([r['valid'] for r in dep])
t = np.array([r['t'] for r in dep])
print(f'depth frames {len(v)} over {t[-1] - t[0]:.1f}s = {len(v) / (t[-1] - t[0]):.1f} fps; '
      f'valid median {np.median(v):.3f} p10 {np.percentile(v, 10):.3f} p90 {np.percentile(v, 90):.3f}; '
      f'frac<0.10 {np.mean(v < 0.10):.2f} frac>0.60 {np.mean(v > 0.60):.2f}')
dt = np.diff(t)
print(f'frame gaps >50ms: {(dt > 0.05).sum()}, max gap {dt.max() * 1000:.0f} ms')
print('first 40 valid%:', ' '.join(f'{100 * x:.0f}' for x in v[:40]))
keys = ['frame_emitter_mode', 'frame_laser_power', 'actual_exposure', 'gain_level', 'auto_exposure',
        'exposure_priority', 'frame_counter', 'hw_timestamp']
joined = [(r['valid'], meta.get(round(r['t'], 4))) for r in dep]
hit = [(x, m) for x, m in joined if m]
print(f'metadata joined {len(hit)}/{len(dep)}; keys sample: {sorted(hit[0][1].keys()) if hit else None}')
for k in keys:
    vals = [m.get(k) for _, m in hit if k in m]
    if vals and k not in ('frame_counter', 'hw_timestamp'):
        u = sorted(set(vals))
        print(f'  {k}: unique {u[:12]}{"..." if len(u) > 12 else ""}')
        if 1 < len(u) <= 12:
            for val in u:
                xs = [x for x, m in hit if m.get(k) == val]
                print(f'     {k}={val}: n {len(xs)} valid median {np.median(xs):.3f}')
fc = [m.get('frame_counter') for _, m in hit if 'frame_counter' in m]
if fc:
    print(f'  frame_counter skips: {int((np.diff(fc) > 1).sum())}, total skipped {int((np.diff(fc) - 1).clip(0).sum())}')
if ir:
    print(f'ir frames {len(ir)} mean median {np.median([r["mean"] for r in ir]):.1f}')
