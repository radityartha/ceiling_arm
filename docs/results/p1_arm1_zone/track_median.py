#!/usr/bin/env python3
"""ARM1-ZONE step 3: record /detected_objects/tracks for N s; per tid: label, n, median xyz, spread. Raw saved to --out.
    python3 track_median.py 15 out.jsonl [topic]   (CAM-6DOF: topic /loc_<ns>/tracks)"""
import json, sys, time
import numpy as np, rclpy
from std_msgs.msg import String
dur, out = float(sys.argv[1]), sys.argv[2]; topic = sys.argv[3] if len(sys.argv) > 3 else '/detected_objects/tracks'
rclpy.init(); n = rclpy.create_node('track_median'); rows = []
n.create_subscription(String, topic, lambda m: rows.append((time.time(), json.loads(m.data))), 10)
t0 = time.time()
while time.time() - t0 < dur: rclpy.spin_once(n, timeout_sec=0.1)
with open(out, 'w') as f:
    for t, d in rows: f.write(json.dumps({'t': t, 'tracks': d}) + '\n')
by = {}
for _, d in rows:
    for o in d: by.setdefault(o['tid'], []).append(o)
print(f'{len(rows)} msgs in {dur} s')
for tid, L in by.items():
    P = np.array([[o['x'], o['y'], o['z']] for o in L])
    print(f"tid {tid} {L[-1]['label']!r} n {len(L)} median {np.median(P, 0).round(4)} p5-p95 xy "
          f"{(np.percentile(P, 95, 0) - np.percentile(P, 5, 0))[:2].round(3)} conf {np.median([o['conf'] for o in L]):.2f}")
