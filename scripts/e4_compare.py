#!/usr/bin/env python3
"""E4: YOLOE detection/localization accuracy vs Isaac ground truth.

Two phases against a LIVE pick_stack.launch.py (perception + executor already
running):
  1. GT capture: switch /seg_source to 'isaac', average /detected_objects over
     a few seconds to get a stable per-object reference xyz (object_localizer's
     own track-smoothed output, keyed by GT label obj_0..obj_7).
  2. YOLOE window: switch /seg_source to 'yoloe' (optionally set /seg_conf via
     a param set, since seg_conf is a launch-time arg not a runtime topic --
     conf sweeps therefore need separate launches, driven externally), collect
     /detected_objects + labels snapshots for `duration` seconds.

For every YOLOE snapshot, each detection is matched to the nearest GT object
within `match_radius`; unmatched detections are false positives, GT objects
with zero matches across the whole window are missed. Writes a per-frame CSV
(detection, label, gt_obj matched or '', localization error) plus prints a
summary. This mirrors experiment_plan.md's E4 section: no invented ground
truth, no fabricated numbers -- every row is a real subscribed message.
"""
import argparse
import json
import subprocess
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

GT_NAMES = {
    'obj_0': 'cracker_box', 'obj_1': 'scissors', 'obj_2': 'tomato_soup_can',
    'obj_3': 'mustard_bottle', 'obj_4': 'teddy_bear', 'obj_5': 'banana',
    'obj_6': 'mug', 'obj_7': 'bowl',
}


class Probe(Node):
    def __init__(self):
        super().__init__('e4_probe')
        self.poses = []
        self.labels = {}
        self.confs = {}
        self.frame_id = 'world'
        self._n_frames = 0
        self.create_subscription(PoseArray, '/detected_objects', self._on_poses, 5)
        self.create_subscription(MarkerArray, '/detected_objects/markers',
                                  self._on_markers, 5)
        self.src_pub = self.create_publisher(String, '/seg_source', 1)

    def _on_poses(self, msg):
        self.poses = list(msg.poses)
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id
        self._n_frames += 1

    def _on_markers(self, msg):
        for m in msg.markers:
            if m.ns == 'labels':
                self.labels[m.id] = m.text
            elif m.ns == 'conf':
                self.confs[m.id] = m.text

    def snapshot(self):
        return (list(self.poses), dict(self.labels), dict(self.confs))

    def set_source(self, src):
        self.src_pub.publish(String(data=src))


def spin_for(node, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def capture_gt(node, settle_s, avg_s):
    """Switch to isaac GT, let tracks settle, then average positions/label per
    obj_N over avg_s seconds (multiple snapshots -> mean xyz per label)."""
    node.set_source('isaac')
    spin_for(node, settle_s)
    sums = defaultdict(lambda: [0.0, 0.0, 0.0, 0])
    deadline = time.time() + avg_s
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        poses, labels, _ = node.snapshot()
        for i, p in enumerate(poses):
            lab = labels.get(i, f'obj{i}')
            s = sums[lab]
            s[0] += p.position.x
            s[1] += p.position.y
            s[2] += p.position.z
            s[3] += 1
    gt = {}
    for lab, (sx, sy, sz, n) in sums.items():
        if n:
            gt[lab] = (sx / n, sy / n, sz / n, n)
    return gt


def run_yoloe_window(node, gt, duration_s, match_radius, csv_path, warmup_s):
    node.set_source('yoloe')
    spin_for(node, warmup_s)   # model load / first frames

    with open(csv_path, 'w') as f:
        f.write('t,det_idx,label,x,y,z,matched_gt,gt_dist,is_fp\n')

    frame_labels_at = {}   # gt_obj -> previous matched label (flicker proxy)
    stats = {
        'gt_matches': defaultdict(int),      # gt_obj -> #frames matched
        'gt_err': defaultdict(list),         # gt_obj -> [dist,...]
        'gt_err_xyz': defaultdict(list),     # gt_obj -> [(dx,dy,dz),...]
        'fp': 0, 'dets': 0, 'frames': 0,
        'flicker': defaultdict(int),         # gt_obj -> label-change count
        'flicker_total': defaultdict(int),   # gt_obj -> #matched frames (denom)
    }
    t_first_frame = None
    t_last_frame = None
    n_pub = 0
    seen_frame_ids = set()

    deadline = time.time() + duration_s
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        poses, labels, confs = node.snapshot()
        if not poses:
            continue
        # de-dup on message identity via id(poses) list contents is unreliable;
        # use a lightweight content hash to avoid double-counting an unchanged
        # snapshot polled twice between publishes.
        key = tuple((round(p.position.x, 4), round(p.position.y, 4),
                     round(p.position.z, 4)) for p in poses)
        if key in seen_frame_ids:
            continue
        seen_frame_ids.add(key)
        if len(seen_frame_ids) > 500:
            seen_frame_ids = {key}
        now = time.time()
        if t_first_frame is None:
            t_first_frame = now
        t_last_frame = now
        n_pub += 1
        stats['frames'] += 1

        matched_gt_this_frame = set()
        with open(csv_path, 'a') as f:
            for i, p in enumerate(poses):
                lab = labels.get(i, f'det{i}')
                x, y, z = p.position.x, p.position.y, p.position.z
                best_gt, best_d = None, match_radius
                for gt_obj, (gx, gy, gz, _n) in gt.items():
                    d = ((x - gx) ** 2 + (y - gy) ** 2 + (z - gz) ** 2) ** 0.5
                    if d < best_d:
                        best_gt, best_d = gt_obj, d
                stats['dets'] += 1
                if best_gt is not None:
                    matched_gt_this_frame.add(best_gt)
                    stats['gt_matches'][best_gt] += 1
                    stats['gt_err'][best_gt].append(best_d)
                    gx, gy, gz, _n = gt[best_gt]
                    stats['gt_err_xyz'][best_gt].append(
                        (abs(x - gx), abs(y - gy), abs(z - gz)))
                    stats['flicker_total'][best_gt] += 1
                    prev = frame_labels_at.get(best_gt)
                    if prev is not None and prev != lab:
                        stats['flicker'][best_gt] += 1
                    frame_labels_at[best_gt] = lab
                else:
                    stats['fp'] += 1
                f.write(f'{now:.3f},{i},{lab},{x:.4f},{y:.4f},{z:.4f},'
                        f'{best_gt or ""},{best_d if best_gt else ""},'
                        f'{0 if best_gt else 1}\n')

    span = (t_last_frame - t_first_frame) if (t_first_frame and t_last_frame
                                               and n_pub > 1) else 0.0
    rate_hz = (n_pub - 1) / span if span > 0 else float('nan')
    return stats, n_pub, rate_hz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=280.0,
                     help='YOLOE collection window (s); target >=200 frames')
    ap.add_argument('--warmup', type=float, default=15.0)
    ap.add_argument('--gt-settle', type=float, default=5.0)
    ap.add_argument('--gt-avg', type=float, default=6.0)
    ap.add_argument('--match-radius', type=float, default=0.40)
    ap.add_argument('--csv', default='/tmp/e4_yoloe.csv')
    args = ap.parse_args()

    rclpy.init()
    node = Probe()

    print('>>> capturing GT reference (source=isaac)...', flush=True)
    gt = capture_gt(node, args.gt_settle, args.gt_avg)
    print(f'    GT objects captured: {len(gt)}')
    for lab, (x, y, z, n) in sorted(gt.items()):
        name = GT_NAMES.get(lab, '?')
        print(f'      {lab:8s} {name:16s} n={n:4d} xyz=({x:+.3f},{y:+.3f},{z:+.3f})')
    if len(gt) < 7:
        print(f'!! WARNING: expected 8 GT objects, only captured {len(gt)} -- '
              f'check camera/segmentation before trusting the YOLOE comparison',
              file=sys.stderr)

    print(f'>>> switching to YOLOE, collecting for {args.duration:.0f}s '
          f'(warmup {args.warmup:.0f}s)...', flush=True)
    stats, n_pub, rate_hz = run_yoloe_window(
        node, gt, args.duration, args.match_radius, args.csv, args.warmup)

    print(f'\n>>> YOLOE window done: {n_pub} distinct published frames, '
          f'rate={rate_hz:.2f} Hz, total detections={stats["dets"]}, '
          f'false positives={stats["fp"]}\n')

    print(f"{'gt_obj':8s} {'name':16s} {'det_rate':9s} {'mean_err':9s} "
          f"{'med_err':9s} {'flicker':8s}")
    for lab in sorted(gt.keys()):
        name = GT_NAMES.get(lab, '?')
        matches = stats['gt_matches'][lab]
        det_rate = matches / n_pub if n_pub else 0.0
        errs = stats['gt_err'][lab]
        mean_e = sum(errs) / len(errs) if errs else float('nan')
        med_e = sorted(errs)[len(errs) // 2] if errs else float('nan')
        flick = stats['flicker'][lab]
        flick_tot = stats['flicker_total'][lab]
        flick_rate = flick / flick_tot if flick_tot else float('nan')
        print(f"{lab:8s} {name:16s} {det_rate:8.1%} {mean_e:9.3f} "
              f"{med_e:9.3f} {flick_rate:7.1%} ({flick}/{flick_tot})")

    fp_rate = stats['fp'] / stats['dets'] if stats['dets'] else float('nan')
    print(f"\nfalse-positive rate: {fp_rate:.1%} ({stats['fp']}/{stats['dets']})")

    summary_path = args.csv.replace('.csv', '_summary.json')
    out = {
        'n_frames': n_pub, 'rate_hz': rate_hz, 'match_radius': args.match_radius,
        'gt': {k: v[:3] for k, v in gt.items()},
        'per_obj': {
            lab: {
                'name': GT_NAMES.get(lab, '?'),
                'detections': stats['gt_matches'][lab],
                'detection_rate': stats['gt_matches'][lab] / n_pub if n_pub else 0,
                'mean_err_m': (sum(stats['gt_err'][lab]) / len(stats['gt_err'][lab])
                               if stats['gt_err'][lab] else None),
                'errs_xyz_mean_m': (
                    [sum(c) / len(stats['gt_err_xyz'][lab])
                     for c in zip(*stats['gt_err_xyz'][lab])]
                    if stats['gt_err_xyz'][lab] else None),
                'flicker_rate': (stats['flicker'][lab] / stats['flicker_total'][lab]
                                 if stats['flicker_total'][lab] else None),
            } for lab in gt.keys()
        },
        'false_positive_rate': fp_rate,
        'total_detections': stats['dets'],
        'total_false_positives': stats['fp'],
    }
    with open(summary_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f'\nsummary written to {summary_path}, raw frames to {args.csv}')

    node.set_source('isaac')   # leave the pipeline in the safe GT default
    rclpy.shutdown()


if __name__ == '__main__':
    main()
