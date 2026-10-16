"""G63e bersama: penanda ArUco BESAR statis (satu per A4) berjangkar pita + papan polos genggam (stereo).

Penanda: kamus DICT_4X4_50 (sel terbesar per sisi = 6 sel -> paling tahan blur/jarak), ID 20.. (ID 0-19 = papan ChArUco
G63 lama DICT_4X4_250 yang masih ada di ruangan; 4X4_50 adalah awalan 4X4_250 -> ID < 20 DITOLAK detektor).
Bingkai penanda t: asal = pusat kotak hitam, x_t = kanan halaman, y_t = bawah halaman (ke arah teks), z_t = MASUK permukaan.
Sudut C0..C3 = sudut LUAR kotak hitam: C0 kiri-atas, C1 kanan-atas, C2 kanan-bawah, C3 kiri-bawah (= urutan sudut OpenCV).
Lembar A4 tegak: kotak hitam di tengah horizontal, tepi atas kotak MARGIN_TOP dari tepi atas kertas; bawah = label + skala.
Pita: tiap sudut Ci diukur (xy trilaterasi dari unting P1..P4, z tinggi) -> pose penanda = Procrustes kaku kotak TAG ke
sudut terukur (bentuk diketahui, sisi TERUKUR operator).
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G63 = os.path.dirname(HERE)
sys.path[:0] = [HERE, G63 + '/g63d']
import g63d_common as GD  # noqa: E402

DICT = cv2.aruco.DICT_4X4_50
IDS = list(range(20, 36))         # 16 lembar: 12 pakai + cadangan
ID_MIN_OLD = 20                   # ID < 20 = papan lama di ruangan -> tolak
TAG_NOMINAL = 0.180               # sisi hitam PDF (m); angka TERUKUR operator menggantikan sebelum solve
PAPER = (0.210, 0.297)
MARGIN_TOP = 0.015
MIN_TAG_PX = 40.0                 # syarat aman (G63: penanda ~22 px gagal sebagian)
CAMS = GD.CAMS
IMG_W, IMG_H = GD.IMG_W, GD.IMG_H


def dictionary():
    return cv2.aruco.getPredefinedDictionary(DICT)


def tag_corners_t(tag=TAG_NOMINAL):
    h = tag / 2
    return np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0.0]])


def paper_t(tag=TAG_NOMINAL, core=False):
    """4 sudut lembar di bingkai t. core=True: hanya bagian persegi 210 x (tag + 2 margin) (label bawah boleh digunting)."""
    h = tag / 2
    top = -h - MARGIN_TOP
    bot = top + (PAPER[0] if core else PAPER[1])
    return np.array([[-PAPER[0] / 2, top, 0], [PAPER[0] / 2, top, 0], [PAPER[0] / 2, bot, 0], [-PAPER[0] / 2, bot, 0.0]])


def R_flat(psi):
    """Penanda datar menghadap ATAS; x_t di dunia = (cos psi, sin psi, 0)."""
    return GD.R_board(psi)


def R_vert(face_deg):
    """Penanda di bidang VERTIKAL, normal luar f = (cos, sin, 0) arah face_deg; teks tegak (y_t = -z dunia)."""
    a = np.radians(face_deg)
    f = np.array([np.cos(a), np.sin(a), 0.0])
    z = -f
    y = np.array([0, 0, -1.0])
    return np.c_[np.cross(y, z), y, z]


def T_of(R, c):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = c
    return T


def to_w(T, p):
    return GD.to_w(T, p)


def procrustes_pose(Pw, tag=TAG_NOMINAL):
    """Pose kaku (tanpa skala) kotak penanda ke 4 sudut dunia terukur. -> T_wt, residu per sudut (m)."""
    A = tag_corners_t(tag)
    ma, mb = A.mean(0), Pw.mean(0)
    H = (A - ma).T @ (Pw - mb)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = T_of(R, mb - R @ ma)
    return T, np.linalg.norm(to_w(T, A) - Pw, axis=1)


def project(T_wc, K, dist, pw):
    return GD.project(T_wc, K, dist, pw)
