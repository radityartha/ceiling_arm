"""G63d bersama: papan polos STATIS berjangkar pita (tanpa FK sendi lengan), proyeksi, masker lengan REST.

Konvensi papan (= cetakan g63b, kotak (0,0) kiri-atas HITAM, label teks kecil di tepi bawah lembar):
  dilihat dari ATAS dengan teks tegak: x_b = kanan sepanjang sisi 4 kotak, y_b = ke bawah halaman (ke arah teks) sepanjang
  sisi 5 kotak, z_b = x_b x y_b = MASUK permukaan (ke bawah). Asal O = sudut luar pola kiri-atas.
  Sudut dalam id = j*3 + i di ((i+1) sq, (j+1) sq), i 0..2, j 0..3 (= g63_common.board_points, dipakai detektor terkunci).
  Titik ukur pita: K1 = O (sudut luar kotak hitam kiri-atas), K2 = (0, 5 sq) (sudut luar kotak hitam kiri-bawah).
  Sudut luar kanan TIDAK terlihat (kotak putih bertemu kertas putih) -> tidak dipakai.
Pose dunia papan datar: K1 (x, y), permukaan z, yaw psi = arah x_b di dunia; diturunkan dari K1, K2 terukur (K1->K2 = +y_b).
"""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G63 = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(G63, '..', '..', '..'))
sys.path[:0] = [G63, G63 + '/g63b', G63 + '/g63c', REPO + '/scripts']

NX, NY = 4, 5
SQ_NOMINAL = 0.042            # PDF; operator 2026-10-16: cetakan "kurang lebih 41 mm" -> angka TERUKUR menggantikan
PAPER = (0.210, 0.297)        # A4; pola di tengah lembar (cetak diskalakan di sekitar pusat halaman)
IMG_W, IMG_H = 1280, 720
CAMS = ('rgbd', 'rgbd2')
URDF = G63 + '/g63b/hw_20261016/live2.urdf'     # URDF hidup sesi jari (g1 1.40, g2 0; FK == TF <= 1 mm)
ENV_MAP = REPO + '/docs/results/p1_g34/env_static_map.npz'

# Sumbu j1 di dunia URDF pada g1 1.40 / g2 0.0009, rot 0/0 (FK live2.urdf, 2026-10-16). Unting lantai P1..P4.
PLUMB = {'P1': (1.8000, 0.3600), 'P2': (1.0000, 0.3600), 'P3': (0.4009, -0.3600), 'P4': (-0.3991, -0.3600)}
PLUMB_ARM = {'P1': 'arm_1', 'P2': 'arm_2', 'P3': 'arm_3', 'P4': 'arm_4'}


def rec_rest():
    """Rekaman B-R0 terakhir sesi jari: K/dist camera_info + T_wc incumbent (G34) + sudut sendi (arm_3 di R0, BUKAN REST)."""
    recs = [json.loads(l) for l in open(G63 + '/g63b/hw_20261016_finger/poses.jsonl')]
    return [r for r in recs if r['id'] == 'B-R0'][-1]


def cam_params():
    r = rec_rest()
    return {c: dict(K=np.array(r['cams'][c]['K']), dist=np.asarray(r['cams'][c]['dist'], float),
                    T_wc=np.array(r['cams'][c]['T_wc_live'])) for c in CAMS}


def R_board(psi):
    ex = np.array([np.cos(psi), np.sin(psi), 0.0])
    ez = np.array([0.0, 0.0, -1.0])
    ey = np.cross(ez, ex)
    return np.c_[ex, ey, ez]


def T_wb(k1_xy, z, psi):
    T = np.eye(4)
    T[:3, :3] = R_board(psi)
    T[:3, 3] = (k1_xy[0], k1_xy[1], z)
    return T


def corners_b(sq):
    """12 sudut dalam (m) di bingkai papan, urutan id."""
    return np.array([[(i + 1) * sq, (j + 1) * sq, 0.0] for j in range(NY - 1) for i in range(NX - 1)])


def paper_b(sq):
    """4 sudut lembar A4 di bingkai papan (pola di tengah lembar)."""
    mx, my = (PAPER[0] - NX * sq) / 2, (PAPER[1] - NY * sq) / 2
    return np.array([[-mx, -my, 0], [NX * sq + mx, -my, 0], [NX * sq + mx, NY * sq + my, 0], [-mx, NY * sq + my, 0.0]])


def to_w(T, p):
    p = np.atleast_2d(p)
    return p @ T[:3, :3].T + T[:3, 3]


def k_points(T, sq):
    """K1, K2 di dunia."""
    return to_w(T, np.array([[0, 0, 0], [0, NY * sq, 0.0]]))


def pose_from_k(k1, k2, z):
    """Pose papan dari K1, K2 terukur (xy dunia) + permukaan z. -> (T_wb, |K1K2| m)."""
    k1, k2 = np.asarray(k1, float), np.asarray(k2, float)
    ey = (k2 - k1) / np.linalg.norm(k2 - k1)
    # ey = (sin psi, -cos psi)
    psi = np.arctan2(ey[0], -ey[1])
    return T_wb(k1, z, psi), float(np.linalg.norm(k2 - k1))


def project(T_wc, K, dist, pw):
    """-> uv (n,2), depth (n,)."""
    Tcw = np.linalg.inv(T_wc)
    pc = np.atleast_2d(pw) @ Tcw[:3, :3].T + Tcw[:3, 3]
    uv, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, dist)
    return uv.reshape(-1, 2), pc[:, 2]


def rest_q():
    """Keadaan sel G63d: semua lengan REST (sudut REST peta G34 cal_00), g1 1.40, g2 0.0009, rot 0/0."""
    import pinocchio as pin
    prov = json.loads(str(np.load(ENV_MAP)['provenance']))
    j = dict(prov['cal_00.npz:config'])
    j.update(t1_linear_joint=1.3999984, t2_linear_joint=0.0009006, t1_rotation_joint=0.0, t2_rotation_joint=0.0)
    model = pin.buildModelFromUrdf(URDF)
    q = pin.neutral(model)
    for n, v in j.items():
        if model.existJointName(n):
            q[model.joints[model.getJointId(n)].idx_q] = v
    return model, q


def robot_points(q=None, model=None):
    """Titik permukaan geometri tabrakan (lengan REST + gantry) di dunia: {nama: (n,3)}."""
    import pinocchio as pin
    from interarm_collision import _package_dirs
    if q is None:
        model, q = rest_q()
    geom = pin.buildGeomFromUrdf(model, URDF, pin.GeometryType.COLLISION, _package_dirs())
    data, gdata = model.createData(), geom.createData()
    pin.updateGeometryPlacements(model, data, geom, gdata, q)
    out = {}
    for i, o in enumerate(geom.geometryObjects):
        g = o.geometry
        t = type(g).__name__
        if t.startswith('BVH'):
            v = np.asarray(g.vertices())
        elif t == 'Box':
            h = np.asarray(g.halfSide)
            v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
        elif t == 'Cylinder':
            a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
            ring = np.c_[g.radius * np.cos(a), g.radius * np.sin(a)]
            v = np.r_[np.c_[ring, np.full(16, -g.halfLength)], np.c_[ring, np.full(16, g.halfLength)]]
        else:
            continue
        M = gdata.oMg[i]
        R, p = np.array(M.rotation), np.array(M.translation)   # salin (memori pinocchio-omg-aliasing)
        out[o.name] = v @ R.T + p
    return out


def robot_mask(cam, cp, pts, pad_m):
    """Masker citra (uint8) lengan/gantry: hull proyeksi per geometri, dilatasi pad_m pada kedalaman geometri."""
    K, d, T = cp[cam]['K'], cp[cam]['dist'], cp[cam]['T_wc']
    m = np.zeros((IMG_H, IMG_W), np.uint8)
    Tcw = np.linalg.inv(T)
    for v in pts.values():
        pc = v @ Tcw[:3, :3].T + Tcw[:3, 3]
        z = pc[:, 2]
        if (z <= 0.05).any():
            continue
        # pinhole tanpa distorsi: polinom distorsi meledak untuk titik jauh di luar FOV (geometri dekat kamera)
        uv = np.clip((pc[:, :2] / z[:, None]) @ K[:2, :2].T + K[:2, 2], -1e5, 1e5)
        h = cv2.convexHull(uv.astype(np.float32)).astype(np.int32)
        r = int(np.ceil(pad_m * K[0, 0] / np.median(z)))
        g = np.zeros_like(m)
        cv2.fillConvexPoly(g, h, 255)
        if r > 0:
            g = cv2.dilate(g, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
        m |= g
    return m
