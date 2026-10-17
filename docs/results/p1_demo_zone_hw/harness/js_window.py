#!/usr/bin/env python3
"""Gerak per sendi + torsi puncak dalam jendela waktu dari CSV js_record (t,joint,pos,eff).
    python3 js_window.py <csv> <t0> <t1> [pad_s]
  'sebelum' = median 3 s sebelum t0, 'sesudah' = median 3 s sesudah t1; maks |pos - sebelum| di jendela; |eff| maks di jendela."""
import csv
import math
import sys
from collections import defaultdict

import numpy as np

f, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
pad = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
pre, post, win, eff = defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list)
for r in csv.DictReader(open(f)):
    t = float(r['t'])
    if t < t0 - pad or t > t1 + pad:
        continue
    j, p = r['joint'], float(r['pos'])
    (pre if t < t0 else post if t > t1 else win)[j].append(p)
    if t0 <= t <= t1 and r['eff']:
        eff[j].append(abs(float(r['eff'])))
for j in sorted(pre):
    a, b = np.median(pre[j]), np.median(post[j]) if post[j] else math.nan
    w = max([abs(x - a) for x in win[j]] or [0.0])
    lin = 'linear' in j
    sc, u = (1000, 'mm') if lin else (180 / math.pi, 'deg')
    print('%-34s sebelum-sesudah %+9.4f %s  maks-jendela %8.4f %s  |eff|maks %s' % (
        j, (b - a) * sc, u, w * sc, u, '%.2f' % max(eff[j]) if eff[j] else '-'))
