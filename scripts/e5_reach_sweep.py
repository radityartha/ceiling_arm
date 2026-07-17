#!/usr/bin/env python3
"""E5: reachable-vs-unreachable classification via reachability_cloud's
voxel-reach percentage, swept over reach_radius, against ground truth.

Ground truth (binary, from independent sources -- E3 IK/plan sweeps + E6's
35/35 live picks + a fresh live re-check here, NOT from this voxel tool):
  reachable:   obj_0,1,3,4,5,6,7 (all 7 picked successfully in E6)
  unreachable: obj_2 (tomato_soup_can) -- confirmed unreachable-by-design

Subscribes to /reachability/voxels (MarkerArray, ns='reach_voxel_label',
text "objN XX% reach"), averages the published percentage per object over a
sample window, and classifies "predicted reachable" iff percentage > 0.
Does NOT restart reachability_cloud itself -- that must be done externally
per reach_radius value (absolute radius requires a fresh process, no live
param callback exists), this script only samples whatever is currently live.
"""
import argparse
import re
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

GT_REACHABLE = {'obj_0', 'obj_1', 'obj_3', 'obj_4', 'obj_5', 'obj_6', 'obj_7'}
GT_UNREACHABLE = {'obj_2'}
NAMES = {
    'obj_0': 'cracker_box', 'obj_1': 'scissors', 'obj_2': 'tomato_soup_can',
    'obj_3': 'mustard_bottle', 'obj_4': 'teddy_bear', 'obj_5': 'banana',
    'obj_6': 'mug', 'obj_7': 'bowl',
}
_PAT = re.compile(r'^(obj_\d+)\s+(\d+)% reach$')


class Sampler(Node):
    def __init__(self):
        super().__init__('e5_sampler')
        self.pcts = defaultdict(list)
        self.create_subscription(MarkerArray, '/reachability/voxels',
                                  self._on_markers, 5)

    def _on_markers(self, msg):
        for m in msg.markers:
            if m.ns == 'reach_voxel_label':
                mt = _PAT.match(m.text)
                if mt:
                    self.pcts[mt.group(1)].append(int(mt.group(2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=20.0)
    ap.add_argument('--reach-radius', type=str, default='?',
                     help='label only, for the printed/JSON output')
    args = ap.parse_args()

    rclpy.init()
    node = Sampler()
    deadline = time.time() + args.duration
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f'reach_radius={args.reach_radius}')
    all_labels = sorted(GT_REACHABLE | GT_UNREACHABLE)
    tp = fp = tn = fn = 0
    seen = set()
    for lab in all_labels:
        pcts = node.pcts.get(lab, [])
        mean_pct = sum(pcts) / len(pcts) if pcts else 0.0
        gt = 'reachable' if lab in GT_REACHABLE else 'unreachable'
        pred = 'reachable' if mean_pct > 0 else 'unreachable'
        seen.add(lab)
        ok = 'OK' if pred == gt else 'MISCLASSIFIED'
        print(f'  {lab:8s} {NAMES[lab]:16s} n={len(pcts):3d} mean%={mean_pct:6.1f} '
              f'gt={gt:12s} pred={pred:12s} {ok}')
        if gt == 'reachable' and pred == 'reachable':
            tp += 1
        elif gt == 'reachable' and pred == 'unreachable':
            fn += 1
        elif gt == 'unreachable' and pred == 'reachable':
            fp += 1
        else:
            tn += 1
    missing = set(all_labels) - seen
    n = tp + fp + tn + fn
    acc = (tp + tn) / n if n else float('nan')
    print(f'  confusion: TP={tp} FN={fn} FP={fp} TN={tn}  accuracy={acc:.1%}')
    if missing:
        print(f'  !! never sampled (0 markers received): {sorted(missing)}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
