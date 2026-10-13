"""G61 C3/C4 step 1: cut the G57 joint_states recordings (UNTRACKED p1_g57/hw/g57_joint_states_part{1,2}.csv.gz, HW data,
columns t,joint,pos,eff) to the window of every task event (arm of the task, 6 joints) of every G57 run that the
recorder covered. OFFLINE. Output = a pickle in the scratchpad (large, not committed):

    python3 g61_c3_extract.py <out.pkl>
"""
import glob
import json
import os
import pickle
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.join(HERE, '..', 'p1_g57', 'hw')
PREFIX = {'arm_1': 't1_a1_', 'arm_2': 't1_a2_', 'arm_3': 't2_a1_', 'arm_4': 't2_a2_'}
PAD = 1.0                                             # s before t_start / after t_end


def windows():
    out = []
    for fn in sorted(glob.glob(os.path.join(HW, 'p1hw_par_*_run.json'))):
        run = os.path.basename(fn)[len('p1hw_par_'):-len('_run.json')]
        for e in json.load(open(fn))['events']:
            if e['kind'] == 'task' and e.get('t_start'):
                out.append(dict(run=run, idx=e['idx'], arm=e['arm'], g=e['gantry'], t0=e['t_start'] - PAD,
                                t1=e['t_end'] + PAD, t_start=e['t_start'], t_end=e['t_end'], goal=e.get('goal'),
                                xyz=e['xyz'], t_success=e.get('t_success'), verdict=e.get('verdict_probe')))
    return out


def main():
    W = windows()
    keep = {f'{p}joint_{i}' for p in PREFIX.values() for i in range(1, 7)}
    t0s = np.array([w['t0'] for w in W])
    t1s = np.array([w['t1'] for w in W])
    acc = {k: [] for k in range(len(W))}
    for part in ('part1', 'part2'):
        fn = os.path.join(HW, f'g57_joint_states_{part}.csv.gz')
        for ch in pd.read_csv(fn, chunksize=2_000_000, dtype={'t': float, 'joint': str, 'pos': float, 'eff': float}):
            ch = ch[ch['joint'].isin(keep)]
            tmin, tmax = ch['t'].iloc[0], ch['t'].iloc[-1]
            for k in np.nonzero((t1s >= tmin) & (t0s <= tmax))[0]:
                w = W[k]
                sel = ch[(ch['t'] >= w['t0']) & (ch['t'] <= w['t1']) & ch['joint'].str.startswith(PREFIX[w['arm']])]
                if len(sel):
                    acc[k].append(sel)
            print(f'{part} chunk {tmin:.0f}-{tmax:.0f}', flush=True)
    res = []
    for k, w in enumerate(W):
        if not acc[k]:
            w['covered'] = False
            res.append(w)
            continue
        df = pd.concat(acc[k])
        w['covered'] = True
        w['js'] = {j: (g['t'].to_numpy(), g['pos'].to_numpy(), g['eff'].to_numpy()) for j, g in df.groupby('joint')}
        res.append(w)
    pickle.dump(res, open(sys.argv[1], 'wb'))
    print(f'{sum(w["covered"] for w in res)}/{len(res)} jendela tugas terekam -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
