"""Tugas 1a: deteksi median tercatat (poses.jsonl) vs prior SEMUA papan. Jarak pusat (m, pada kedalaman prior)."""
import numpy as np
from g63c_common import records, priors_all
for r in records():
    for cam, c in r['cams'].items():
        for b, det in c['boards'].items():
            if not det['ids']:
                continue
            uv = np.array(det['uv']); pr = priors_all(r, cam)
            ppm = {k: np.median(np.linalg.norm(np.diff(v['uv'].reshape(4, 3, 2), axis=1), axis=2)) / 0.042 for k, v in pr.items()}
            dm = {k: np.linalg.norm(uv.mean(0) - v['uv'].mean(0)) / ppm[k] for k, v in pr.items()}
            o = 'B' if b == 'A' else 'A'
            print(f"{r['id']:5s} {cam:5s} claim {b} d_own {dm[b]:.3f} m  d_other {dm[o]:.3f} m  ratio {dm[o]/dm[b]:6.1f}  depth own {pr[b]['depth']:.2f} other {pr[o]['depth']:.2f}")
