"""G64 torque model, step 1: cut every archived HW /joint_states CSV into per-arm MOTION SEGMENTS. OFFLINE.

Per file (same list as g64_torque_time.files()) and per arm: the 6 joints pivoted on the message stamp (100 Hz), a
sample is 'moving' when any joint of the arm moves > 0.2 deg within +-0.25 s; segments = moving runs merged across
gaps < 1 s, padded 2 s each side (so the holds before/after are included), shorter than 1 s of motion dropped.
Output: one .npz per file in OUTDIR (large, NOT committed): for each segment k  t_k (n,), q_k (n, 6), e_k (n, 6),
meta = json list (file, arm, k, t0, t1, n).

    python3 g64_tsid_extract.py OUTDIR
"""
import json
import os
import sys
from multiprocessing import Pool

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import g64_torque_time as TT  # noqa: E402

PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
ARM_J = [f'{p}joint_{i}' for p in PREFIX.values() for i in range(1, 7)]
OUTDIR = sys.argv[1] if len(sys.argv) > 1 else None


def segments(t, q):
    mv = np.zeros(len(t), bool)
    w = 25                                   # +-0.25 s at 100 Hz
    for j in range(6):
        x = q[:, j]
        lo = pd.Series(x).rolling(2 * w + 1, center=True, min_periods=1).min().to_numpy()
        hi = pd.Series(x).rolling(2 * w + 1, center=True, min_periods=1).max().to_numpy()
        mv |= (hi - lo) > np.radians(0.2)
    out, i, n = [], 0, len(t)
    while i < n:
        if not mv[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and (mv[j + 1] or (t[min(n - 1, j + 1)] - t[j] < 1.0 and np.any(mv[j + 1:j + 100]))):
            j += 1
        if t[j] - t[i] >= 1.0:
            out.append((t[i] - 2.0, t[j] + 2.0))
        i = j + 1
    return out


def job(fn):
    rel = os.path.relpath(fn, TT.R)
    parts = []
    for ch in pd.read_csv(fn, usecols=['t', 'joint', 'pos', 'eff'], chunksize=3_000_000):
        ch = ch[ch['joint'].isin(ARM_J)]
        parts.append(ch.astype({'joint': 'category', 'pos': np.float32, 'eff': np.float32}))
    df = pd.concat(parts)
    df['joint'] = df['joint'].astype(str)
    arrays, meta = {}, []
    for arm, p in PREFIX.items():
        d = df[df['joint'].str.startswith(p)]
        if d.empty:
            continue
        P = d.pivot_table(index='t', columns='joint', values='pos', aggfunc='last')
        E = d.pivot_table(index='t', columns='joint', values='eff', aggfunc='last')
        cols = [f'{p}joint_{i}' for i in range(1, 7)]
        if any(c not in P.columns for c in cols):
            continue
        P, E = P[cols].dropna(), E[cols].reindex(P.index)
        P = P.loc[E.dropna().index]
        E = E.loc[P.index]
        t, q, e = P.index.to_numpy(float), P.to_numpy(float), E.to_numpy(float)
        for k, (a, b) in enumerate(segments(t, q)):
            m = (t >= a) & (t <= b)
            key = f'{arm}_{k}'
            arrays[f't_{key}'], arrays[f'q_{key}'], arrays[f'e_{key}'] = t[m], q[m].astype(np.float32), e[m].astype(np.float32)
            meta.append(dict(file=rel, arm=arm, key=key, t0=float(t[m][0]), t1=float(t[m][-1]), n=int(m.sum())))
    name = rel.replace('/', '__').replace('.csv.gz', '.npz')
    np.savez_compressed(os.path.join(OUTDIR, name), meta=json.dumps(meta), **arrays)
    return rel, len(meta)


if __name__ == '__main__':
    os.makedirs(OUTDIR, exist_ok=True)
    fs = TT.files()
    with Pool(4) as pool:
        for rel, n in pool.imap_unordered(job, fs):
            print(f'{rel}: {n} segmen', flush=True)
