#!/usr/bin/env python3
"""DEMO-SYS tugas 1: recorded track snapshots (track_median.py jsonl, one file per FILTERED source topic) -> tasks.json
for demo_solve.py. Per track id: median xyz over the window; kept iff present in >= 90 % of the messages (= DEMO-ZONE-HW
Z2). p_cmd = median + (0, 0, 0.25) (x, y, z of a track = object TOP: DEMO-ZONE-HW B1 / TAS-RAK; tool down, stand-off 0.25).
Sources are already zone-filtered upstream: /demo_zone/tracks (arm_1 table, rgbd2 + P3 correction), /rack_box/tracks
(rack box, uncorrected = limitation). Nothing is chosen here: every kept track becomes a task; the solver accepts/rejects.

    python3 snapshot_tasks.py out.json --p0 1.40 0.0 SRC=path.jsonl [SRC=path.jsonl ...]
"""
import argparse
import json

import numpy as np

STANDOFF = 0.25
PRESENT = 0.90


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--p0', type=float, nargs=2, required=True, metavar=('G1', 'G2'), help='K0 measured rail (rot 0)')
    ap.add_argument('src', nargs='+')
    a = ap.parse_args()
    tasks, dropped = [], []
    for s in a.src:
        name, path = s.split('=', 1)
        rows = [json.loads(ln) for ln in open(path) if ln.strip()]
        if not rows:
            raise SystemExit(f'{name}: 0 pesan di {path} -- TOLAK')
        by = {}
        for r in rows:
            for o in r['tracks']:
                by.setdefault(o['tid'], []).append(o)
        for tid, L in sorted(by.items()):
            P = np.array([[o['x'], o['y'], o['z']] for o in L])
            med = np.median(P, 0)
            frac = len(L) / len(rows)
            d = dict(id=f'{name}_t{tid}', label=L[-1]['label'], src=name, tid=tid, n=len(L), msgs=len(rows),
                     present=round(frac, 3), median=med.round(4).tolist(),
                     spread_xy_p5_p95=(np.percentile(P, 95, 0) - np.percentile(P, 5, 0))[:2].round(4).tolist(),
                     p_cmd=[round(float(med[0]), 4), round(float(med[1]), 4), round(float(med[2]) + STANDOFF, 4)])
            (tasks if frac >= PRESENT else dropped).append(d)
            print(f"{'TUGAS' if frac >= PRESENT else 'buang (hadir < 90 %)'} {d['id']} {d['label']!r} hadir "
                  f"{100 * frac:.0f} % median {d['median']} sebar {d['spread_xy_p5_p95']} -> p_cmd {d['p_cmd']}")
    json.dump(dict(p0={'1': a.p0[0], '2': a.p0[1]}, p0_rot_deg={'1': 0.0, '2': 0.0}, tasks=tasks, dropped=dropped),
              open(a.out, 'w'), indent=1)
    print(f'{len(tasks)} tugas -> {a.out}')
    return 0 if tasks else 2


if __name__ == '__main__':
    raise SystemExit(main())
