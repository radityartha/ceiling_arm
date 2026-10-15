#!/usr/bin/env python3
"""G63 payload URDF (§A5 T4): live URDF + one fixed link per taped board, with mass and a collision box.

The link is named <arm prefix>g63_board (t1_a2_g63_board, t2_a1_g63_board), so every existing prefix-based screen
(interarm_collision InterArmChecker / CrossGantryChecker, env_collision EnvChecker) automatically treats the board as
part of its arm, and pinocchio RNEA (probe predict_peak_torque) carries its weight. Nothing shared is modified: the
G63 probe wrapper points those screens at THIS file.

Mount JSON (per board; operator measures it from the taped board -- until then `"guess": true` and every result built
on it is provisional):
  {"A": {"link": "right_finger_prox_link", "xyz": [..], "rpy": [..], "mass_kg": 0.06, "guess": true}, "B": {...}}
xyz/rpy = board frame (OpenCV ChArUco: origin = pattern corner, x along the 4 squares, y along the 5, z INTO the
board, i.e. away from the printed face) expressed in the finger link frame.

    python3 g63_urdf.py --in /tmp/reach_dwell_live.urdf --mount mount.json --out g63_payload.urdf [--check]
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np

from g63_common import ARM_PREFIX, BOARD_ARM

# Physical A4 sheet relative to the pattern (make_g63_boards: pattern 5 mm from the left edge, 8 mm from the top).
SHEET_X = (-0.005, 0.205)
SHEET_Y = (-0.008, 0.289)
CARD_T = 0.006                 # cardboard + tape thickness (m), behind the paper (+z)
COLL_MARGIN = 0.010            # collision box grown by this on every side (unmeasured cardboard edges, flex)


def _sheet(mt):
    """Measured sheet extent (board frame) + card thickness from the mount JSON, else the A4-print defaults."""
    mt = mt or {}
    return tuple(mt.get('sheet_x', SHEET_X)), tuple(mt.get('sheet_y', SHEET_Y)), float(mt.get('card_t', CARD_T))


def board_box(mt=None):
    """Collision box (centre xyz in the board frame, size) = sheet + cardboard, grown by COLL_MARGIN."""
    sx, sy, ct = _sheet(mt)
    c = [(sx[0] + sx[1]) / 2, (sy[0] + sy[1]) / 2, ct / 2]
    s = [sx[1] - sx[0] + 2 * COLL_MARGIN, sy[1] - sy[0] + 2 * COLL_MARGIN, ct + 2 * COLL_MARGIN]
    return c, s


def board_inertial(mass, mt=None):
    """Uniform plate inertia about its centre, in the board frame."""
    c, _ = board_box(mt)
    sx, sy, ct = _sheet(mt)
    a, b, t = sx[1] - sx[0], sy[1] - sy[0], ct
    ixx = mass * (b * b + t * t) / 12
    iyy = mass * (a * a + t * t) / 12
    izz = mass * (a * a + b * b) / 12
    return c, (ixx, iyy, izz)


def add_boards(root, mount):
    for name, mt in sorted((k, v) for k, v in mount.items() if not k.startswith('_')):
        prefix = ARM_PREFIX[BOARD_ARM[name]]
        parent = prefix + mt['link']
        if root.find(f"link[@name='{parent}']") is None:
            raise SystemExit(f'link induk {parent} tidak ada di URDF')
        child = prefix + 'g63_board'
        if root.find(f"link[@name='{child}']") is not None:
            raise SystemExit(f'{child} sudah ada (URDF masukan sudah dimodifikasi?)')
        link = ET.SubElement(root, 'link', name=child)
        c, (ixx, iyy, izz) = board_inertial(float(mt['mass_kg']), mt)
        ine = ET.SubElement(link, 'inertial')
        ET.SubElement(ine, 'origin', xyz=' '.join(f'{v:.6f}' for v in c), rpy='0 0 0')
        ET.SubElement(ine, 'mass', value=f"{float(mt['mass_kg']):.6f}")
        ET.SubElement(ine, 'inertia', ixx=f'{ixx:.3e}', ixy='0', ixz='0', iyy=f'{iyy:.3e}', iyz='0', izz=f'{izz:.3e}')
        bc, bs = board_box(mt)
        for tag in ('collision', 'visual'):
            el = ET.SubElement(link, tag)
            ET.SubElement(el, 'origin', xyz=' '.join(f'{v:.6f}' for v in bc), rpy='0 0 0')
            geo = ET.SubElement(el, 'geometry')
            ET.SubElement(geo, 'box', size=' '.join(f'{v:.6f}' for v in bs))
        j = ET.SubElement(root, 'joint', name=child + '_joint', type='fixed')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=' '.join(f'{v:.6f}' for v in mt['xyz']),
                      rpy=' '.join(f'{v:.6f}' for v in mt['rpy']))


def build(in_path, mount, out_path):
    tree = ET.parse(in_path)
    add_boards(tree.getroot(), mount)
    tree.write(out_path)
    return out_path


def check(base_urdf, out_urdf, mount):
    """(1) pinocchio loads; (2) board geometry carries the arm prefix; (3) static joint_2 torque increase at REST
    equals m*g*lever computed independently from the board COM position (RNEA vs hand calculation)."""
    import pinocchio as pin
    from interarm_collision import _package_dirs
    ok = True
    m0, m1 = pin.buildModelFromUrdf(base_urdf), pin.buildModelFromUrdf(out_urdf)
    geom = pin.buildGeomFromUrdf(m1, out_urdf, pin.GeometryType.COLLISION, _package_dirs())
    REST = [-0.46352, 0.10710, 0.12916, -1.38653, -0.17648, 1.73885]
    for name, mt in sorted((k, v) for k, v in mount.items() if not k.startswith('_')):
        arm = BOARD_ARM[name]
        pre = ARM_PREFIX[arm]
        gnames = [g.name for g in geom.geometryObjects if g.name.startswith(pre + 'g63_board')]
        good = len(gnames) == 1
        ok &= good
        print(f'{name}/{arm}: geometri {gnames} -> {"OK" if good else "GAGAL"}')
        taus = []
        for m in (m0, m1):
            d = m.createData()
            q = pin.neutral(m)
            for i, v in enumerate(REST):
                q[m.joints[m.getJointId(f'{pre}joint_{i + 1}')].idx_q] = v
            q[m.joints[m.getJointId('t1_linear_joint' if pre.startswith('t1') else 't2_linear_joint')].idx_q] = \
                1.5 if pre.startswith('t1') else 0.0
            taus.append(pin.rnea(m, d, q, np.zeros(m.nv), np.zeros(m.nv)))
            if m is m1:
                pin.framesForwardKinematics(m, d, q)
                fid = m.getFrameId(pre + 'g63_board')
                com = d.oMf[fid].act(np.array(board_inertial(1.0, mt)[0]))
                jid = m.getJointId(pre + 'joint_2')
                jp = d.oMi[jid]
                axis = jp.rotation[:, 2]
                g = np.array([0, 0, -9.81 * float(mt['mass_kg'])])
                hand = float(np.dot(np.cross(com - jp.translation, g), axis))
        iv = m1.joints[m1.getJointId(pre + 'joint_2')].idx_v
        d_rnea = float(taus[1][iv] - taus[0][iv])
        good = abs(abs(d_rnea) - abs(hand)) < 1e-3
        ok &= good
        print(f'   REST: delta tau j2 RNEA {d_rnea:+.4f} N.m vs tangan {-hand:+.4f} -> {"OK" if good else "GAGAL"}'
              f'{"   (dudukan TEBAKAN)" if mt.get("guess") else ""}')
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in', dest='inp', required=True)
    ap.add_argument('--mount', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    sys.path.insert(0, '/home/user1/Documents/ceiling_arm/scripts')
    mount = json.load(open(a.mount))
    build(a.inp, mount, a.out)
    print(f'tulis {a.out}')
    if a.check:
        return 0 if check(a.inp, a.out, mount) else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
