#!/usr/bin/env python3
"""DEMO-OFF (docs/p1_demo_off.md A2): target_cli '#tid' + 'g' -> screened approach.

Stands in for reach_fusion on its three topics, so target_cli is unchanged:
    /reach_fusion/set_target  String  '#<tid>'      (latest selection)
    /reach_fusion/execute     Empty                 (operator typed 'g')
    /reach_fusion/status      String  'success ...' | 'failed: ...'

Writes NO motion code and NO screen. Each step is an existing HW-proven tool run
as a subprocess, exactly as run_g32.py runs it:
    task -> scripts/reach_dwell_probe.py --arms <arm> --target=<p_cmd> --approach 0
            --trials 1 --tau-max <--tau-max, 12> [--move | --plan-check] --monitor-csv <mon>
            (tuck, RNEA+offset, S18 vs all 3 other arms, env map, retract_check;
            it moves only when EVERY screen says CLEAR)
    home -> scripts/return_rest.py --arms arm_1 arm_2 --move   (only after a MOVED)
The gantry is never commanded (d3: g1 parked, arm-only groups).

p_cmd = track xyz + (0, 0, standoff), tool pointing down. Arm order: the arm_1/
arm_2 base nearest the object in xy first; the other only if the first refused.

    python3 demo_runner.py --mon <csv>_samples.csv              # executes on 'g'
    python3 demo_runner.py --mon <csv>_samples.csv --plan-only  # screens, no motion
    ... --tracks-topic /demo_zone/tracks --arms arm_1             # DEMO-ZONE-HW (argparse: no --ros-args -r)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

SCRIPTS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        '..', '..', '..', 'scripts'))
BASE = {'arm_1': 't1_a1_base_link', 'arm_2': 't1_a2_base_link'}
# Refused BEFORE any motion because of the pose/plan -> the other arm may try.
# Anything else (EXEC-FAIL, *-UNSCREENED, timeout) = machinery: stop, go home.
POSE_REFUSALS = ('NO-PLAN', 'TUCK-REFUSED', 'TORQUE-UNSAFE', 'INTERARM-COLLIDE',
                 'RETRACT-BLOCKED', 'ENV-COLLIDE')


class DemoRunner(Node):
    def __init__(self, a):
        super().__init__('demo_runner')
        self.a = a
        self.busy = False
        self.target = None
        self.tracks = []
        self.tf = Buffer()
        TransformListener(self.tf, self)
        tq = QoSProfile(depth=1)
        tq.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, a.tracks_topic, self._on_tracks, tq)
        self.create_subscription(String, '/reach_fusion/set_target', self._on_target, 1)
        self.create_subscription(Empty, '/reach_fusion/execute', self._on_execute, 1)
        self.status_pub = self.create_publisher(String, '/reach_fusion/status', 1)
        os.makedirs(a.out, exist_ok=True)
        self.n = 0
        # resume numbering so a restarted runner never overwrites earlier runs
        self.run_id = sum(f.startswith('run') and f.endswith('.json') for f in os.listdir(a.out))

    def _on_tracks(self, msg):
        try:
            self.tracks = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def _on_target(self, msg):
        self.target = msg.data.strip()
        self.get_logger().info(f'target -> {self.target}')

    def _status(self, s):
        self.get_logger().info(f'STATUS {s}')
        self.status_pub.publish(String(data=s))

    def _on_execute(self, _msg):
        if self.busy:                                     # r1
            self.get_logger().error('execute DITOLAK: run sebelumnya masih jalan (r1)')
            return          # no status: target_cli is still waiting for the running one
        n_exec = self.count_subscribers('/reach_fusion/execute')
        if n_exec > 1:                                    # r2
            self._status(f'failed: {n_exec} pemakai /reach_fusion/execute '
                         '(reach_fusion hidup?) -- r2')
            return
        t = self.target or ''
        tr = next((x for x in self.tracks if f"#{x.get('tid')}" == t), None)
        if tr is None:                                    # r3
            self._status(f'failed: target {t or "(none)"} tidak ada di track sekarang (r3)')
            return
        self.busy = True
        threading.Thread(target=self._run, args=(dict(tr),), daemon=True).start()

    def _arm_order(self, xyz):
        d = {}
        for arm in self.a.arms:
            frame = BASE[arm]
            try:
                tr = self.tf.lookup_transform('world', frame, Time()).transform.translation
                d[arm] = math.hypot(tr.x - xyz[0], tr.y - xyz[1])
            except Exception:                             # noqa: BLE001
                d[arm] = math.inf
        return sorted(self.a.arms, key=lambda k: d[k]), d

    def _tool(self, tag, cmd):
        self.n += 1
        logf = os.path.join(self.a.out, f'run{self.run_id:02d}_{self.n:02d}_{tag}.log')
        t0 = time.time()
        with open(logf, 'w') as f:
            f.write(f'# {t0:.4f} CMD {" ".join(cmd)}\n')
            pr = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, bufsize=1, cwd=SCRIPTS)
            out = []
            for ln in pr.stdout:
                f.write(f'{time.time():.4f} {ln}')
                f.flush()
                out.append(ln)
            rc = pr.wait()
        return rc, ''.join(out), round(time.time() - t0, 2), logf

    def _run(self, tr):
        self.run_id += 1
        self.n = 0
        t0 = time.time()
        xyz = (float(tr['x']), float(tr['y']), float(tr['z']))
        p = (xyz[0], xyz[1], xyz[2] + self.a.standoff)
        order, dist = self._arm_order(xyz)
        rec = dict(run=self.run_id, t0=t0, track=tr, p_cmd=p, plan_only=self.a.plan_only,
                   arm_dist={k: round(v, 3) for k, v in dist.items()}, tries=[])
        self.get_logger().info(f'run {self.run_id}: #{tr["tid"]} {tr["label"]} '
                               f'p_cmd {p} urutan {order}')
        moved, stray = None, False
        for arm in order:
            if os.path.exists('/tmp/g20_step5.json'):
                os.remove('/tmp/g20_step5.json')
            cmd = ['timeout', '600', 'python3', '-u', 'reach_dwell_probe.py', '--arms', arm,
                   f'--target={p[0]:.4f},{p[1]:.4f},{p[2]:.4f}', '--approach', '0',
                   '--trials', '1', '--tau-max', str(self.a.tau_max), '--monitor-csv', self.a.mon,
                   '--plan-check' if self.a.plan_only else '--move']
            rc, out, dt, logf = self._tool(f'probe_{arm}', cmd)
            # G59: not 'percobaan rencana: [...]' (G47 per-attempt list) nor 'sepanjang rencana: <mm>'
            m = re.findall(r'(?:moveit|(?<![a-z] )rencana:) (\S+)', out)
            v = m[-1] if m else f'rc{rc}'
            refusals = re.findall(r'DITOLAK[^\n]*|TABRAKAN[^\n]*|JALUR PULANG[^\n]*', out)
            screens = re.findall(r'(torsi RNEA[^\n]*|jarak antar-lengan[^\n]*|'
                                 r'jarak ke peta lingkungan[^\n]*|percobaan rencana[^\n]*|'
                                 r'retract g[^\n]*|penyaring [^\n]*)', out)
            fin = re.findall(r'\[ 1\] (\S+)', out)
            rec['tries'].append(dict(arm=arm, rc=rc, verdict=v, final=fin[-1] if fin else None,
                                     dt=dt, log=logf, screens=screens, refusals=refusals))
            self.get_logger().info(f'  {arm}: {v} (rc {rc}, {dt} s)')
            if v in ('MOVED', 'PLANNED'):
                moved = arm
                break
            if v not in POSE_REFUSALS:
                stray = True
                break
        if (moved or stray) and not self.a.plan_only:
            rc, out, dt, logf = self._tool('return_rest', ['python3', '-u', 'return_rest.py',
                                                           '--arms', *self.a.arms, '--move'])
            tau = re.findall(r'torsi puncak ([0-9.]+)', out)
            rec['return_rest'] = dict(rc=rc, dt=dt, log=logf, tau_peak=tau[-1] if tau else None)
        rec['t_total'] = round(time.time() - t0, 2)
        with open(os.path.join(self.a.out, f'run{self.run_id:02d}.json'), 'w') as f:
            json.dump(rec, f, indent=1)
        vs = ', '.join(f"{t['arm']}:{t['verdict']}" for t in rec['tries'])
        rr = rec.get('return_rest')
        if moved and (self.a.plan_only or (rr and rr['rc'] == 0)):
            self._status(f'success {moved} ({vs}) {rec["t_total"]} s')
        else:
            self._status(f'failed: {vs}' + (f' return_rest rc {rr["rc"]}' if rr else ''))
        self.busy = False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mon', required=True, help='<csv_log>_samples.csv of reach_dwell_monitor')
    ap.add_argument('--standoff', type=float, default=0.25,
                    help='m above the track centroid (A2: bottle not in env map, T11)')
    ap.add_argument('--plan-only', action='store_true', help='probe --plan-check, no motion')
    ap.add_argument('--tau-max', type=float, default=12.0,
                    help='probe --tau-max N*m (DEMO-HW: 14, operator 2026-10-12)')
    ap.add_argument('--out', default='/tmp/demo_runner')
    ap.add_argument('--tracks-topic', default='/detected_objects/tracks',
                    help='track source (DEMO-ZONE-HW: /demo_zone/tracks)')
    ap.add_argument('--arms', nargs='+', choices=sorted(BASE), default=['arm_1', 'arm_2'],
                    help='arms tried (nearest first) and sent to return_rest (DEMO-ZONE-HW: arm_1)')
    a = ap.parse_args()
    rclpy.init()
    node = DemoRunner(a)
    node.get_logger().info(f'demo_runner up ({"PLAN-ONLY" if a.plan_only else "EXECUTE"}), '
                           f'standoff {a.standoff} m, tau-max {a.tau_max}, tracks {a.tracks_topic}, '
                           f'arms {a.arms}, out {a.out}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
