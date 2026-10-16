#!/usr/bin/env python3
"""DEMO-ZONE (docs/p1_demo_zone.md, arah C): pasca-proses track rgbd2 -> koreksi LOKAL P3 HANYA di zona meja demo arm_1.

Ekstrinsik/launch TIDAK diubah; yaw P3 (14.7 deg) adalah peta lokal di bidang meja, BUKAN yaw kamera. Di luar zona -> DITOLAK.
  in : /loc_rgbd2/tracks         String JSON [{tid,label,x,y,z,conf}]  (object_localizer rgbd2 saja, p1_cam6dof/harness/loc_cam.sh)
  out: /demo_zone/tracks         skema SAMA (x,y,z terkoreksi) + x_trk,y_trk,z_trk  -> runner/CLI cukup di-remap ke topik ini
       /demo_zone/rejected       [{tid,label,x_trk,y_trk,z_trk,reason}]
Gerbang (semua harus lolos; tidak ada yang 'dikoreksi paksa'):
  track xy di kotak TRACK (selubung 20 obs + 5 cm), xy terkoreksi di kotak ZONA (selubung tanda + 5 cm),
  |z_track - z_out| <= dz_max, z_out = meja + setengah tinggi benda (botol demo 7 cm -> 0.805).
    python3 zone_correct_node.py [--ros-args -p object_height:=0.07 ...]
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ZONE_XY = ((1.60, 1.99), (0.45, 0.73))      # dunia, selubung tanda M1/C5/C3/C6/N1/N2 + 5 cm (p1_demo_zone.md)
TRACK_XY = ((1.78, 2.17), (0.49, 0.81))     # track rgbd2 incumbent, selubung 20 obs (x 1.831-2.117, y 0.538-0.758) + 5 cm
TABLE_Z = 0.77                              # meja demo terukur operator
DZ_MAX = 0.05                               # A: botol 7 cm track z 0.810-0.833 vs 0.805 -> <= 2.8 cm


def load_p3(path=os.path.join(HERE, 'zone_p3_fit.json')):
    d = json.load(open(path))
    assert d['model'] == 'P3', d['model']
    return np.array(d['center']), np.array(d['x'][:2]), float(d['x'][2])


def correct_xy(xy, center, shift, yaw):
    """P3 = zone_models.apply('P3'): rotasi yaw di sekitar center, lalu geser."""
    c, s = np.cos(yaw), np.sin(yaw)
    q = np.asarray(xy, float)[:2] - center
    return center + np.array([c * q[0] - s * q[1], s * q[0] + c * q[1]]) + shift


def _in(xy, box):
    return box[0][0] <= xy[0] <= box[0][1] and box[1][0] <= xy[1] <= box[1][1]


def process(tracks, p3, object_height, table_z=TABLE_Z, dz_max=DZ_MAX):
    """tracks: list dict object_localizer. -> (diterima, ditolak). Murni (tanpa ROS) untuk uji unit."""
    z_out = table_z + 0.5 * object_height
    ok, rej = [], []
    for t in tracks:
        trk = np.array([t['x'], t['y'], t['z']], float)
        cxy = correct_xy(trk[:2], *p3)
        why = []
        if not _in(trk, TRACK_XY):
            why.append('track xy (%.3f, %.3f) di luar kotak track' % tuple(trk[:2]))
        if not _in(cxy, ZONE_XY):
            why.append('xy terkoreksi (%.3f, %.3f) di luar zona' % tuple(cxy))
        if abs(trk[2] - z_out) > dz_max:
            why.append('z track %.3f vs z_out %.3f (> %.2f)' % (trk[2], z_out, dz_max))
        base = dict(tid=int(t['tid']), label=t['label'], x_trk=float(trk[0]), y_trk=float(trk[1]), z_trk=float(trk[2]))
        if why:
            rej.append(dict(base, reason='; '.join(why)))
        else:
            ok.append(dict(base, x=float(cxy[0]), y=float(cxy[1]), z=float(z_out), conf=float(t.get('conf', 0.0))))
    return ok, rej


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    class ZoneCorrect(Node):
        def __init__(self):
            super().__init__('demo_zone_correct')
            self.declare_parameter('in_topic', '/loc_rgbd2/tracks')
            self.declare_parameter('object_height', 0.07)
            self.declare_parameter('fit', os.path.join(HERE, 'zone_p3_fit.json'))
            self.h = float(self.get_parameter('object_height').value)
            self.p3 = load_p3(self.get_parameter('fit').value)
            q = QoSProfile(depth=1)
            q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL   # = object_localizer tracks_pub
            self.pub_ok = self.create_publisher(String, '/demo_zone/tracks', q)
            self.pub_rej = self.create_publisher(String, '/demo_zone/rejected', q)
            inp = self.get_parameter('in_topic').value
            self.create_subscription(String, inp, self._on, q)
            self.last = None
            self.get_logger().info('P3 center %s shift %s yaw %.2f deg; in %s; object_height %.3f -> z_out %.3f' % (
                self.p3[0].round(4), self.p3[1].round(4), np.degrees(self.p3[2]), inp, self.h, TABLE_Z + self.h / 2))

        def _on(self, m):
            ok, rej = process(json.loads(m.data), self.p3, self.h)
            self.pub_ok.publish(String(data=json.dumps(ok)))
            self.pub_rej.publish(String(data=json.dumps(rej)))
            key = (tuple(t['tid'] for t in ok), tuple((r['tid'], r['reason'].split(' (')[0]) for r in rej))
            if key != self.last:      # log hanya saat himpunan diterima/ditolak berubah
                self.last = key
                for t in ok:
                    self.get_logger().info('TERIMA #%d %s trk (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)' % (
                        t['tid'], t['label'], t['x_trk'], t['y_trk'], t['z_trk'], t['x'], t['y'], t['z']))
                for r in rej:
                    self.get_logger().warn('TOLAK #%d %s: %s' % (r['tid'], r['label'], r['reason']))

    rclpy.init()
    n = ZoneCorrect()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
