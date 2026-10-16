"""G63e render sintetis: lembar penanda A4 / papan polos genggam -> citra kamera (supersampel 3x, blur Gauss 1.3-1.6 px,
noise 2.3, hitam 30 / putih 230 seperti g63d_selftest; latar = frame lama abu atau abu 90). Distorsi diabaikan (dist 0
dipakai juga oleh detektor di uji mandiri)."""
import glob

import cv2
import numpy as np

import g63e_common as E

PPM = 2000
SS = 3


def tag_texture(i, tag=E.TAG_NOMINAL, core=False):
    """Halaman (atau inti) penanda sebagai citra abu; piksel (0,0) = sudut kiri-atas lembar."""
    pap = E.paper_t(tag, core)
    w, h = int(round((pap[1, 0] - pap[0, 0]) * PPM)), int(round((pap[2, 1] - pap[0, 1]) * PPM))
    t = np.full((h, w), 230, np.uint8)
    bits = cv2.aruco.generateImageMarker(E.dictionary(), i, 6, borderBits=1)
    cell = tag / 6 * PPM
    x0 = (-tag / 2 - pap[0, 0]) * PPM
    y0 = (-tag / 2 - pap[0, 1]) * PPM
    for r in range(6):
        for k in range(6):
            if bits[r, k] < 128:
                t[int(round(y0 + r * cell)):int(round(y0 + (r + 1) * cell)), int(round(x0 + k * cell)):int(round(x0 + (k + 1) * cell))] = 30
    return t, pap


def board_texture(sq, border=0.021, nx=4, ny=5):
    """Papan polos (= cetakan g63b): pola di tengah, tepi putih border; asal bingkai papan = sudut luar kiri-atas pola."""
    w, h = nx * sq + 2 * border, ny * sq + 2 * border
    t = np.full((int(round(h * PPM)), int(round(w * PPM))), 230, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                t[int(round((border + j * sq) * PPM)):int(round((border + (j + 1) * sq) * PPM)),
                  int(round((border + i * sq) * PPM)):int(round((border + (i + 1) * sq) * PPM))] = 30
    pap = np.array([[-border, -border, 0], [nx * sq + border, -border, 0], [nx * sq + border, ny * sq + border, 0],
                    [-border, ny * sq + border, 0.0]])
    return t, pap


def backgrounds():
    out = {}
    for c in E.CAMS:
        f = sorted(glob.glob(E.G63 + '/g63b/hw_20261016_finger/frames/B-R0/%s/*.png' % c))
        out[c] = cv2.cvtColor(cv2.imread(f[-1]), cv2.COLOR_BGR2GRAY) if f else np.full((E.IMG_H, E.IMG_W), 90, np.uint8)
    return out


def render(bg, items, T_wc, K, rng, blur=None):
    """items: list (tex, pap_t 4x3, T_w_obj). Supersampel SS x lalu INTER_AREA; blur pada resolusi akhir."""
    Ks = K.copy()
    Ks[:2] *= SS
    Ks[:2, 2] = K[:2, 2] * SS + (SS - 1) / 2.0           # pusat piksel: u_s = SS*u + (SS-1)/2
    W, H = E.IMG_W * SS, E.IMG_H * SS
    img = cv2.resize(bg, (W, H), interpolation=cv2.INTER_NEAREST).astype(np.float32)
    for tex, pap, T in items:
        uv, z = E.project(T_wc, Ks, np.zeros(5), E.to_w(T, pap))
        if (z <= 0.1).any():
            continue
        h, w = tex.shape
        src = np.float32([[-0.5, -0.5], [w - 0.5, -0.5], [w - 0.5, h - 0.5], [-0.5, h - 0.5]])
        Hm = cv2.getPerspectiveTransform(src, uv.astype(np.float32))
        m = cv2.warpPerspective(np.full_like(tex, 255), Hm, (W, H), flags=cv2.INTER_LINEAR)
        p = cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_LINEAR)
        a = m.astype(np.float32) / 255.0
        img = a * p + (1 - a) * img
    img = cv2.resize(img, (E.IMG_W, E.IMG_H), interpolation=cv2.INTER_AREA)
    img = cv2.GaussianBlur(img, (0, 0), rng.uniform(1.3, 1.6) if blur is None else blur)
    return np.clip(img + rng.normal(scale=2.3, size=img.shape), 0, 255).astype(np.uint8)
