#!/usr/bin/env python3
"""p1_state.md 8c STEP 2: one arm, reach-and-dwell to a PERCEIVED target.

docs/p1_g16_hw.md A8. This is the COMMANDER. It is deliberately a separate
process from reach_dwell_monitor, which is the SCORER, because whatever scores
success must not also choose the candidates -- otherwise the criterion drifts
toward the result. The monitor commands nothing; this commands and scores
nothing.

The contract between them, and the reason the ordering matters:

    1. perceive          -> p_perceived        (from /target_object, or --target)
    2. choose            -> p_cmd              (perceived + approach offset)
    3. PUBLISH p_cmd to /reach_dwell/target/<arm>   <-- BEFORE any motion
    4. command MoveIt to p_cmd
    5. the monitor independently scores  ||p_tool - p_cmd|| and the 2.0 s dwell

Step 3 must precede step 4 so the monitor scores the pose that was actually
COMMANDED. Publishing the PERCEIVED pose instead would silently fold perception
error (L3, 3-5 cm) into the execution criterion (L2, < 5 mm) and make the
success number meaningless -- that is exactly the confusion p1_g4 A2 exists to
prevent. L3 is reported by this script, separately, and is NOT part of success.

SAFETY (docs/p1_g16_hw.md A6). Read-only unless --move is typed on purpose:
  * default is a DRY RUN: it perceives, computes p_cmd, publishes the target,
    and prints the plan -- but sends no trajectory.
  * it REFUSES to command the tuck pose [0, 2.6, 2.6, 0, 0, 0]; that draws
    12.78 of 14 N.m and is a fake-system artefact, not a real rest pose.
  * it aborts on any joint effort above --tau-max.
  * the rest pose is HANGING.

NOTE ON THE PERCEPTION SOURCE. /detected_object_pose does NOT exist in this
checkout: it comes from lidar_filter.py, which needs livox_ros_driver2, and that
source tree is empty (there is no .gitmodules). What exists is the RGBD chain
ending in /target_object. And because the locked criterion scores COMMANDED ->
ACHIEVED, a hand-given --target measures exactly the same thing with the cameras
and the segmentation model taken out of the failure surface -- which is what day
one should do, so that a failure can actually be attributed.

    # day one: no perception at all, measures the same L2
    python3 scripts/reach_dwell_probe.py --arm arm_1 --target 0.9,0.3,1.2
    # once the RGBD chain is up
    python3 scripts/reach_dwell_probe.py --arm arm_1 --trials 10 --move
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

TOOL_FRAME = {'arm_1': 't1_a1_tool_frame', 'arm_2': 't1_a2_tool_frame',
              'arm_3': 't2_a1_tool_frame', 'arm_4': 't2_a2_tool_frame'}
# NOT /detected_object_pose: that is published by workcell_description's
# lidar_filter.py, which needs livox_ros_driver2 -- and that source tree is
# EMPTY in this checkout (there is no .gitmodules either), so the LIDAR driver
# cannot run and the topic never appears. The RGBD chain is what exists:
#   rgbd_perception.launch.py -> instance segmentation -> object_localizer
#                             -> /target_object
PERCEPT_TOPIC = '/target_object'
# A6/S2. Never commanded. Kept here so the refusal is explicit and greppable.
FORBIDDEN_TUCK = [0.0, 2.6, 2.6, 0.0, 0.0, 0.0]


class Probe(Node):
    def __init__(self, arm, approach, tau_max, move, topic=PERCEPT_TOPIC,
                 fixed=None):
        super().__init__('reach_dwell_probe')
        self.arm, self.approach, self.tau_max, self.move = \
            arm, approach, tau_max, move
        self.fixed = fixed
        self.perceived = None
        self.status = []
        self.tau_peak = 0.0
        self.tau_worst_joint = ''

        self.create_subscription(PoseStamped, topic, self._on_percept, 10)
        self.create_subscription(String, '/reach_dwell/status',
                                 self._on_status, 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)
        self.target_pub = self.create_publisher(
            PoseStamped, f'/reach_dwell/target/{arm}', 10)
        self.clear_pub = self.create_publisher(String, '/reach_dwell/clear', 10)

    # ------------------------------------------------------------- inputs
    def _on_percept(self, msg):
        self.perceived = msg

    def _on_status(self, msg):
        try:
            self.status.append(json.loads(msg.data))
        except (ValueError, TypeError):
            self.status.append({'raw': msg.data})

    def _on_js(self, msg):
        # A6/S3. Real N.m at ~96 Hz. Once ARMSTATE_IN_FAULT latches, the red LED
        # needs a PHYSICAL reset -- fault_controller is not spawned -- so the
        # only useful guard is one that trips below the firmware threshold.
        for name, eff in zip(msg.name, msg.effort or []):
            if abs(eff) > self.tau_peak:
                self.tau_peak, self.tau_worst_joint = abs(eff), name

    # ------------------------------------------------------------- helpers
    def wait_percept(self, timeout):
        """A fixed target is a legitimate source, and on day one the better one.

        The locked success criterion (A1) scores COMMANDED -> ACHIEVED (L2).
        Perception error (L3) is explicitly NOT part of it. So where p_cmd comes
        from does not change what is being measured -- it only changes which
        pose is measured. Running L2 against a hand-given pose therefore
        measures exactly the same quantity while removing the cameras, the
        segmentation model and the extrinsics from the failure surface. Couple
        them on day one and a failure cannot be attributed.
        """
        if self.fixed is not None:
            p = PoseStamped()
            p.header.frame_id = 'world'
            p.pose.position.x, p.pose.position.y, p.pose.position.z = self.fixed
            p.pose.orientation.w = 1.0
            return p
        t0 = time.time()
        while time.time() - t0 < timeout and self.perceived is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.perceived

    def command_pose(self, p):
        """p_cmd = perceived + a straight-up approach offset."""
        out = PoseStamped()
        out.header.frame_id = 'world'          # the monitor rejects other frames
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = p.pose.position.x
        out.pose.position.y = p.pose.position.y
        out.pose.position.z = p.pose.position.z + self.approach
        # Approach axis pointing down; roll is free by the locked definition.
        out.pose.orientation.x = 1.0
        out.pose.orientation.y = 0.0
        out.pose.orientation.z = 0.0
        out.pose.orientation.w = 0.0
        return out

    def publish_target(self, p_cmd):
        """MUST happen before motion -- see the module docstring."""
        self.target_pub.publish(p_cmd)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)

    def clear(self):
        self.clear_pub.publish(String(data=self.arm))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)

    def collect(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tau_peak > self.tau_max:
                self.get_logger().error(
                    f'ABORT: torsi {self.tau_peak:.2f} N.m pada '
                    f'{self.tau_worst_joint} > batas {self.tau_max:.2f}')
                return 'TORQUE-ABORT'
            for s in self.status:
                if s.get('arm') == self.arm and s.get('event') == 'success':
                    return 'SUCCESS'
        return None

    def verdict(self, raw):
        """A3's THREE-WAY split. Never collapse these into one bucket."""
        if raw in ('SUCCESS', 'TORQUE-ABORT'):
            return raw
        touched = any(s.get('arm') == self.arm and s.get('in_tol')
                      for s in self.status)
        return 'REACHED-NOT-HELD' if touched else 'NO-PLAN'


def move_to(arm, p_cmd):
    """Command MoveIt. Left as the single explicit integration point.

    Deliberately NOT wired blind: hardware_check.py already drives MoveIt for
    the --arms stage, and which of that path to reuse is a decision to make with
    the arms physically present and someone watching, not the night before.

    WHEN WIRING THIS, the A6/S2 refusal must be enforced HERE, at the point the
    joint command is formed: reject any goal whose joint vector is within a few
    degrees of FORBIDDEN_TUCK. The constant above is declared, not enforced,
    precisely because there is as yet no place that forms a joint command.
    """
    raise NotImplementedError(
        'Jalur perintah MoveIt sengaja BELUM disambung -- lihat A7 tahap 3. '
        'Sambungkan setelah tahap 3 (regresi joint_6 +5 deg) LULUS, dengan '
        'lengan terpasang dan ada yang mengawasi.')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--arm', default='arm_1', choices=sorted(TOOL_FRAME))
    ap.add_argument('--trials', type=int, default=10,
                    help='A2: dikunci 10; ubah hanya kalau A2 diubah dulu')
    ap.add_argument('--approach', type=float, default=0.10,
                    help='offset approach di atas objek terpersepsi (m)')
    ap.add_argument('--tau-max', type=float, default=10.0,
                    help='A6/S3 batas torsi abort (N.m); tuck = 12.78/14')
    ap.add_argument('--settle', type=float, default=8.0,
                    help='detik menunggu monitor per percobaan')
    ap.add_argument('--topic', default=PERCEPT_TOPIC,
                    help=f'topik pose terpersepsi (default {PERCEPT_TOPIC}; '
                         '/detected_object_pose BUTUH livox yang TIDAK ADA)')
    ap.add_argument('--target', metavar='X,Y,Z',
                    help='pose tetap di frame world, lewati persepsi sama '
                         'sekali. Mengukur L2 yang SAMA; pakai ini hari pertama')
    ap.add_argument('--move', action='store_true',
                    help='A6/S7: BENAR-BENAR menggerakkan lengan. '
                         'Tanpa ini skrip DRY RUN.')
    a = ap.parse_args()

    fixed = None
    if a.target:
        try:
            fixed = tuple(float(v) for v in a.target.split(','))
            if len(fixed) != 3:
                raise ValueError
        except ValueError:
            ap.error('--target harus X,Y,Z (meter, frame world)')

    if not a.move:
        print('\033[33mDRY RUN\033[0m -- tidak ada gerak. '
              'Tambahkan --move untuk menggerakkan (A6/S7).\n')

    rclpy.init()
    node = Probe(a.arm, a.approach, a.tau_max, a.move, a.topic, fixed)
    src = f'TETAP {fixed} (persepsi dilewati)' if fixed else f'topik {a.topic}'
    print(f'probe: {a.arm} ({TOOL_FRAME[a.arm]}), {a.trials} percobaan, '
          f'approach {a.approach*1000:.0f} mm, abort torsi {a.tau_max} N.m')
    print(f'sumber target: {src}')
    print('kriteria A1 TERKUNCI: pos < 5 mm, ori < 5 deg, dwell 2.0 s KONTINU\n')

    tally = {'SUCCESS': 0, 'REACHED-NOT-HELD': 0, 'NO-PLAN': 0,
             'TORQUE-ABORT': 0, 'INVALID': 0}
    rows = []
    try:
        for i in range(1, a.trials + 1):
            node.status.clear()
            node.clear()
            p = node.wait_percept(timeout=10.0)
            if p is None:
                print(f'  [{i:2d}] TIDAK VALID (mesin): tidak ada pose di '
                      f'{a.topic} -- A5, diulang, tidak masuk penyebut. '
                      'Pakai --target X,Y,Z untuk melewati persepsi.')
                tally['INVALID'] += 1
                continue

            p_cmd = node.command_pose(p)
            node.publish_target(p_cmd)          # BEFORE motion. Always.

            if a.move:
                try:
                    move_to(a.arm, p_cmd)
                except NotImplementedError as e:
                    print(f'\033[31m  {e}\033[0m')
                    break
                raw = node.collect(a.settle)
            else:
                raw = None
                print(f'  [{i:2d}] DRY RUN  p_cmd = '
                      f'({p_cmd.pose.position.x:+.3f}, '
                      f'{p_cmd.pose.position.y:+.3f}, '
                      f'{p_cmd.pose.position.z:+.3f}) world')
                continue

            v = node.verdict(raw)
            tally[v] += 1
            rows.append(dict(trial=i, verdict=v, tau_peak=node.tau_peak))
            print(f'  [{i:2d}] {v:17s} torsi puncak {node.tau_peak:5.2f} N.m')
            if v == 'TORQUE-ABORT':
                print('\033[31m  BERHENTI: batas torsi. A6/S4 -- red LED butuh '
                      'reset FISIK.\033[0m')
                break
    except KeyboardInterrupt:
        print('\ndihentikan pengguna')
    finally:
        valid = sum(tally[k] for k in
                    ('SUCCESS', 'REACHED-NOT-HELD', 'NO-PLAN'))
        if valid:
            print(f'\n  A2: {tally["SUCCESS"]} / {valid} sukses  '
                  f'(LULUS butuh >= 8/10)')
            print(f'  A3 pembagian tiga arah: SUCCESS {tally["SUCCESS"]}, '
                  f'REACHED-NOT-HELD {tally["REACHED-NOT-HELD"]}, '
                  f'NO-PLAN {tally["NO-PLAN"]}')
            print(f'  TIDAK VALID (mesin, A5, diulang): {tally["INVALID"]}')
            json.dump(rows, open('/tmp/g16_step2.json', 'w'), indent=1)
            print('  -> /tmp/g16_step2.json')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
