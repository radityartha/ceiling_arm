#!/usr/bin/env python3
"""Stage 0/1 gate after the arms are re-mounted. READ-ONLY -- commands nothing.

docs/p1_g16_hw.md A5/A7. Every check here corresponds to a failure mode that has
already cost a full debugging session, and each one presents as a symptom that
looks like something else:

    no IP on 192.168.2.x   -> ros2_control_node hangs FOREVER at "Connecting"
    an arm still mid-boot  -> SIGABRT(-6), and ALL FOUR arms' controllers die
    a Kinova web UI tab    -> SIGPIPE(-13) mid-session, INVALID_USER_SESSION_ACCESS
    stale move_group       -> MoveIt CONTROL_FAILED (-4)
    /joint_states at 10 Hz -> that is dual_table_controller ALONE; the control
                              stack is already dead, and the grippers are not
                              the problem
    stale static_transform_publisher -> "I changed the config but the view never
                              changes"; they accumulate and all publish DIFFERENT
                              values to the SAME frames

Stage 0 needs no ROS at all. Stage 1 (--ros) needs a sourced workspace and a
running my_workcell.launch.py.

    python3 scripts/remount_check.py            # stage 0
    python3 scripts/remount_check.py --ros      # stage 0 + stage 1
"""

from __future__ import annotations

import argparse
import glob
import os
import socket
import subprocess
import sys

ARMS = {'arm_4': '192.168.2.10', 'arm_3': '192.168.2.11',
        'arm_2': '192.168.2.12', 'arm_1': '192.168.2.13'}
SUBNET_PREFIX = '192.168.2.'
PORTS = ('/dev/ttyUSB0', '/dev/ttyUSB1')
# Processes that must NOT already be running. pgrep/ps match their own command
# text, so /proc/*/cmdline is scanned instead -- that mistake is itself one of
# the recorded failure modes.
# reach_dwell_monitor is here because the SCORER leaks: `ros2 run` spawns it as
# a child, so killing the wrapper leaves it alive, and a leaked scorer scores the
# next run -- silently, with whatever code the leaked one was built from.
STALE = ('move_group', 'ros2_control_node', 'static_transform_publisher',
         'realsense2_camera_node', 'reach_dwell_monitor')

FAILS: list[str] = []
WARNS: list[str] = []


def say(state, label, detail=''):
    tag = {'ok': '\033[32m OK \033[0m', 'warn': '\033[33mWARN\033[0m',
           'bad': '\033[31mFAIL\033[0m'}[state]
    print(f'  [{tag}] {label:<44s} {detail}', flush=True)
    if state == 'bad':
        FAILS.append(label)
    elif state == 'warn':
        WARNS.append(label)


def procs_matching(name):
    """Scan /proc/*/cmdline for an exact executable match, not `ps | grep`."""
    hits = []
    for d in glob.glob('/proc/[0-9]*'):
        try:
            with open(os.path.join(d, 'cmdline'), 'rb') as fh:
                parts = fh.read().split(b'\0')
        except OSError:
            continue
        if any(os.path.basename(p.decode('utf-8', 'replace')) == name
               for p in parts if p):
            hits.append(int(os.path.basename(d)))
    return hits


# ------------------------------------------------------------------ stage 0
def stage0():
    print('\nTAHAP 0 -- READ-ONLY, tanpa ROS  (docs/p1_g16_hw.md A7)')

    for p in PORTS:
        say('ok' if os.path.exists(p) else 'bad', f'port serial {p}',
            'ada' if os.path.exists(p) else 'TIDAK ADA -- gantry tidak akan init')

    # The subnet IP. Its absence is silent: the interface is UP, the link is
    # fine, and ros2_control_node simply never returns.
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr'], capture_output=True,
                             text=True, timeout=10).stdout
    except Exception as e:                                   # noqa: BLE001
        out = ''
        say('warn', 'baca alamat IP', f'gagal: {e}')
    mine = [ln.split()[3] for ln in out.splitlines()
            if SUBNET_PREFIX in ln.split()[3]] if out else []
    if mine:
        say('ok', f'IP di subnet {SUBNET_PREFIX}x', ', '.join(mine))
    else:
        say('bad', f'IP di subnet {SUBNET_PREFIX}x',
            'TIDAK ADA -- ros2_control_node akan MENGGANTUNG SELAMANYA. '
            'Perbaiki: sudo ip addr add 192.168.2.100/24 dev <iface>')

    for name, ip in sorted(ARMS.items()):
        rc = subprocess.run(['ping', '-c', '1', '-W', '1', ip],
                            capture_output=True).returncode
        say('ok' if rc == 0 else 'bad', f'ICMP {name} ({ip})',
            'menjawab' if rc == 0 else 'TIDAK menjawab')

    # A Kortex session opened by anything else (a left-open web UI tab) evicts
    # the driver mid-session with INVALID_USER_SESSION_ACCESS -> SIGPIPE(-13).
    for name, ip in sorted(ARMS.items()):
        s = socket.socket()
        s.settimeout(1.0)
        reachable = s.connect_ex((ip, 443)) == 0
        s.close()
        if reachable:
            say('warn', f'web UI {name} ({ip}:443) hidup',
                'TUTUP tab browser ke IP ini -- sesi kedua = SIGPIPE(-13)')
        else:
            say('ok', f'web UI {name} tidak dipakai', '')

    for name in STALE:
        pids = procs_matching(name)
        if pids:
            say('bad', f'proses basi: {name}',
                f'PID {pids} -- MATIKAN DENGAN PID dulu (jangan pkill -f)')
        else:
            say('ok', f'tidak ada {name} basi', '')

    try:
        lsusb = subprocess.run(['lsusb'], capture_output=True, text=True,
                               timeout=10).stdout
        n = lsusb.lower().count('intel') + lsusb.count('8086:')
        say('ok' if n >= 2 else 'warn', 'kamera RealSense terdeteksi',
            f'{n} perangkat Intel di lsusb (diharapkan 2x D455)')
    except Exception as e:                                   # noqa: BLE001
        say('warn', 'lsusb', f'gagal: {e}')

    print('\n  ⚠️  PERIKSA DENGAN MATA, tidak bisa diotomatiskan:')
    print('      - LED tiap lengan TIDAK amber/init (mid-boot = SIGABRT -6,')
    print('        dan keempat lengan mati bersama). Beri jeda setelah power-on.')
    print('      - LED TIDAK merah (fault terkunci; fault_controller tidak')
    print('        di-spawn, jadi reset harus FISIK).')
    print('      - Board kalibrasi SUDAH DIKELUARKAN dari ruang kerja')
    print('        (kecuali sengaja dipakai sebagai target persepsi).')
    print('      - Lengan menggantung bebas; TIDAK ada yang menyentuh rel.')


# ------------------------------------------------------------------ stage 1
def stage1(seconds=4.0):
    print('\nTAHAP 1 -- bring-up, MASIH tanpa gerak lengan')
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from tf2_ros import Buffer, TransformListener
    except ImportError as e:
        say('bad', 'impor rclpy', f'{e} -- source install/setup.bash dulu')
        return

    rclpy.init()
    node = Node('remount_check')
    got = {'n': 0, 'width': 0, 'effort': False, 'names': []}

    def on_js(msg):
        got['n'] += 1
        got['width'] = len(msg.name)
        got['names'] = list(msg.name)
        got['effort'] = bool(msg.effort) and any(abs(e) > 0 for e in msg.effort)

    node.create_subscription(JointState, '/joint_states', on_js, 50)
    buf = Buffer()
    TransformListener(buf, node)

    import time
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    hz = got['n'] / max(time.time() - t0, 1e-9)

    # 10 Hz exactly is the tell: that is dual_table_controller alone, i.e. the
    # ros2_control stack is already dead. ~96 Hz with 28 values is healthy.
    if got['n'] == 0:
        say('bad', '/joint_states', 'TIDAK ADA -- stack tidak jalan')
    elif hz < 20.0:
        say('bad', '/joint_states laju', f'{hz:.1f} Hz, {got["width"]} nilai -- '
            '~10 Hz = dual_table_controller SENDIRIAN, ros2_control MATI')
    else:
        say('ok', '/joint_states laju', f'{hz:.1f} Hz, {got["width"]} nilai')

    say('ok' if got['width'] >= 28 else 'warn', 'lebar /joint_states',
        f'{got["width"]} (diharapkan 28: 4 lengan x 6 + gripper + gantry)')
    say('ok' if got['effort'] else 'bad', 'effort (torsi N.m) terisi',
        'ada -- penjaga torsi bisa jalan' if got['effort']
        else 'KOSONG -- S3/S4 tidak bisa dipantau')

    for arm, frame in (('arm_1', 't1_a1_tool_frame'),
                       ('arm_2', 't1_a2_tool_frame'),
                       ('arm_3', 't2_a1_tool_frame'),
                       ('arm_4', 't2_a2_tool_frame')):
        try:
            buf.lookup_transform('world', frame, rclpy.time.Time())
            say('ok', f'TF world -> {frame}', 'hidup')
        except Exception:                                    # noqa: BLE001
            # A missing arm TF also makes the collision self-filter silently
            # no-op, so this is not only a monitor prerequisite.
            say('bad', f'TF world -> {frame}',
                'HILANG -- monitor tidak bisa menilai, self-filter no-op')

    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ros', action='store_true',
                    help='juga jalankan TAHAP 1 (butuh workspace + launch aktif)')
    a = ap.parse_args()

    print('=' * 74)
    print('REMOUNT CHECK -- READ-ONLY. Skrip ini TIDAK menggerakkan apa pun.')
    print('=' * 74)
    stage0()
    if a.ros:
        stage1()

    print('\n' + '=' * 74)
    if FAILS:
        print(f'\033[31mGERBANG GAGAL: {len(FAILS)}\033[0m -> ' + '; '.join(FAILS))
        print('A7: tahap TIDAK dilompati. Perbaiki dulu, baru lanjut.')
    else:
        print('\033[32mGERBANG LULUS.\033[0m Lanjut ke tahap berikutnya (A7).')
    if WARNS:
        print(f'\033[33mPERINGATAN: {len(WARNS)}\033[0m -> ' + '; '.join(WARNS))
    print('=' * 74)
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
