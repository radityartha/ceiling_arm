#!/usr/bin/env python3
"""G63 detector (§A3 "Deteksi"): ChArUco corners of boards A/B in one colour image.

1. ChArUco (marker IDs decoded): board A = IDs 0-9, B = 10-19 -> identity is certain.
2. Fallback when markers are too small to decode (far): chessboard 3x4 inner corners (findChessboardCornersSB) inside
   a ROI around the corners PREDICTED from FK + a prior extrinsic; the labeling (4 candidate orderings) is chosen by
   distance to the prediction. Registered rejection rules (before data):
     * < MIN_CORNERS corners                                  -> no observation for that board/camera
     * fallback: second-best labeling RMS < 2x best RMS       -> AMBIGUOUS, rejected
     * fallback: best labeling RMS > PRIOR_MAX_PX             -> rejected (wrong pattern / prior far off)
Per-pose capture: median per corner over frames; a corner counts only if seen in >= 50 % of the frames.

    python3 g63_detect.py --self-test
"""
import argparse
import sys

import cv2
import cv2.aruco as aruco
import numpy as np

from g63_common import BOARDS, IMG_H, IMG_W, K_NOMINAL, board_points, build_board, incumbent_T, inv

MIN_CORNERS = 8
PRIOR_MAX_PX = 60.0       # incumbent off by <= 0.15 m -> <= ~50 px at 2 m
AMBIG_RATIO = 2.0
ROI_PAD_PX = 40
PATTERN = (3, 4)          # inner corners (cols, rows) = squares 4 x 5


def _charuco(gray, name):
    det = aruco.CharucoDetector(build_board(name))
    c, ids, mc, mi = det.detectBoard(gray)
    n_m = 0 if mi is None else len(mi)
    if ids is None or len(ids) == 0:
        return None, n_m
    return (ids.ravel().astype(int), c.reshape(-1, 2).astype(float)), n_m


def _labelings():
    """Index permutations of the 3x4 row-major corner grid: identity, 180 deg, flip rows, flip cols."""
    g = np.arange(12).reshape(PATTERN[1], PATTERN[0])
    return [g.ravel(), g[::-1, ::-1].ravel(), g[::-1, :].ravel(), g[:, ::-1].ravel()]


def _square_sets(name):
    """Board-plane sample grids (m) for every BLACK square and every WHITE (marker) square."""
    b = build_board(name)
    sx, sy = b.getChessboardSize()
    L = b.getSquareLength()
    img = b.generateImage((sx * 40, sy * 40), marginSize=0, borderBits=1)
    g = np.linspace(-0.4, 0.4, 5) * L
    grid = np.array([[x, y] for x in g for y in g])
    blk, wht = [], []
    for j in range(sy):
        for i in range(sx):
            pts = grid + ((i + 0.5) * L, (j + 0.5) * L)
            (blk if img[j * 40 + 3, i * 40 + 3] < 128 else wht).append(pts)
    return blk, wht


def _parity_contrast(gray, name, ids_uv):
    """median(white-square means) - MAX(black-square means) under this hypothesis' homography. One black square that
    lands on something bright (a white square, the paper margin, light cardboard) sinks a wrong hypothesis."""
    P = board_points(name)
    src = np.array([P[i][:2] for i in range(12)], np.float32)
    H, _ = cv2.findHomography(src, ids_uv.astype(np.float32))
    if H is None:
        return -1e9
    blk, wht = _square_sets(name)

    def mean(pts):
        uv = np.round(cv2.perspectiveTransform(pts.reshape(-1, 1, 2).astype(np.float32), H).reshape(-1, 2)).astype(int)
        if (uv < 0).any() or (uv[:, 0] >= gray.shape[1]).any() or (uv[:, 1] >= gray.shape[0]).any():
            return np.nan
        return float(np.mean(gray[uv[:, 1], uv[:, 0]]))
    mb = [mean(p) for p in blk]
    mw = [mean(p) for p in wht]
    if np.isnan(mb).any() or np.isnan(mw).any():
        return -1e9
    return float(np.median(mw) - np.max(mb))


PARITY_MIN = 40.0         # grey levels; registered rule: best hypothesis contrast must exceed this
PARITY_AMBIG = 0.5        # and the second best (a DIFFERENT corner set) must be below 0.5 x best
SHIFTS = range(-3, 4)
SADDLE_FRAC = 0.25        # corner kept if its saddle response >= 0.25 x median of the stronger half


def _dihedral():
    """8 maps (i, j) -> (a, b) of the plane lattice: rotations by 90 deg and mirrors."""
    return [np.array(m) for m in ([[1, 0], [0, 1]], [[-1, 0], [0, -1]], [[0, -1], [1, 0]], [[0, 1], [-1, 0]],
                                  [[-1, 0], [0, 1]], [[1, 0], [0, -1]], [[0, 1], [1, 0]], [[0, -1], [-1, 0]])]


def _fallback_once(gray, pred_uv, name, sig_frac):
    """Chessboard 3x4 found in a ROI (prior sets the ROI only). SB may lock onto a window shifted by a square or
    transposed (thin white margin -> edge corners look like inner corners), so the found grid only defines the
    LATTICE: every dihedral x shift hypothesis of where the true inner grid sits is refined (cornerSubPix) and scored
    by black-square parity; mirrored (physically impossible) hypotheses are discarded."""
    lo = np.floor(pred_uv.min(0) - ROI_PAD_PX).astype(int).clip(0)
    hi = np.ceil(pred_uv.max(0) + ROI_PAD_PX).astype(int)
    hi = np.minimum(hi, [gray.shape[1], gray.shape[0]])
    if (hi - lo < 20).any():
        return None, 'roi kosong'
    # Wipe the markers' internal cells (~L/8) before SB, keep the squares (L): sigma = 0.12 L (predicted L in px).
    pg = pred_uv.reshape(PATTERN[1], PATTERN[0], 2)
    L_px = float(np.median(np.linalg.norm(np.diff(pg, axis=1), axis=2)))
    sig = sig_frac * L_px
    work = cv2.GaussianBlur(gray, (0, 0), sig) if sig > 0.5 else gray
    roi = work[lo[1]:hi[1], lo[0]:hi[0]]
    ok, cc = cv2.findChessboardCornersSB(roi, PATTERN, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        return None, 'chessboard tidak ditemukan'
    cc = cc.reshape(-1, 2) + lo
    orig, gray = gray, work          # parity + coarse refinement on the marker-free image, final on the original
    return _verify_lattice(gray, cc, pred_uv, name, orig)


def _verify_lattice(gray, cc, pred_uv, name, orig=None):
    """cc = 12 image points of SOME 3x4 window of the board lattice (found order). Score every dihedral x shift
    hypothesis of the true inner grid by parity; refine the winner; uniqueness + prior sanity."""
    lat = np.array([[a, b] for b in range(PATTERN[1]) for a in range(PATTERN[0])], np.float32)   # found order
    H0, _ = cv2.findHomography(lat, cc.astype(np.float32))
    sq_px = float(np.median(np.linalg.norm(np.diff(cc.reshape(PATTERN[1], PATTERN[0], 2), axis=1), axis=2)))
    win = max(2, int(0.35 * sq_px))
    true_ij = np.array([[i, j] for j in range(PATTERN[1]) for i in range(PATTERN[0])], float)  # corner id order
    P = board_points(name)
    src = np.array([P[i][:2] for i in range(12)], np.float32)
    hyps = {}
    for D in _dihedral():
        for sx in SHIFTS:
            for sy in SHIFTS:
                ab = true_ij @ D.T + [sx, sy]
                uv = cv2.perspectiveTransform(ab.reshape(-1, 1, 2).astype(np.float32), H0).reshape(-1, 2)
                if (uv < 2).any() or (uv[:, 0] > gray.shape[1] - 3).any() or (uv[:, 1] > gray.shape[0] - 3).any():
                    continue
                Hb, _ = cv2.findHomography(src, uv.astype(np.float32))
                if Hb is None or np.linalg.det(Hb[:2, :2] - np.outer(Hb[:2, 2], Hb[2, :2])) <= 0:
                    continue                                               # mirrored view: impossible
                key = tuple(sorted(map(tuple, np.round(ab).astype(int))))  # same corner set, other labeling
                sc = _parity_contrast(gray, name, uv)
                if sc > hyps.get(key, (-1e9,))[0]:
                    hyps[key] = (sc, uv)
    if not hyps:
        return None, 'tidak ada hipotesis kisi sah'
    ranked = sorted(hyps.values(), key=lambda t: -t[0])
    best = ranked[0]
    second = ranked[1][0] if len(ranked) > 1 else -1e9
    if best[0] < PARITY_MIN:
        return None, f'paritas lemah ({best[0]:.0f} < {PARITY_MIN:.0f})'
    if second > PARITY_AMBIG * best[0]:
        return None, f'AMBIGU paritas ({best[0]:.0f} vs {second:.0f})'
    uv = cv2.cornerSubPix(gray, best[1].reshape(-1, 1, 2).astype(np.float32), (win, win), (-1, -1),
                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)).reshape(-1, 2)
    if np.max(np.linalg.norm(uv - best[1], axis=1)) > 0.5 * sq_px:
        return None, 'cornerSubPix lari > setengah kotak'
    if orig is not None and sq_px >= 20:   # final refinement on the unblurred image (near only: far it got worse)
        w2 = max(2, int(0.12 * sq_px))
        uv2 = cv2.cornerSubPix(orig, uv.reshape(-1, 1, 2).astype(np.float32), (w2, w2), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)).reshape(-1, 2)
        mv = np.linalg.norm(uv2 - uv, axis=1)
        uv = np.where((mv < 0.25 * sq_px)[:, None], uv2, uv)
    # per-corner saddle test: an occluded / blank corner has no X structure -> dropped, never kept as a guess
    sig = max(0.8, 0.15 * sq_px)
    sm = cv2.GaussianBlur((orig if orig is not None else gray).astype(np.float32), (0, 0), sig)
    Ixx = cv2.Sobel(sm, cv2.CV_32F, 2, 0, ksize=3)
    Iyy = cv2.Sobel(sm, cv2.CV_32F, 0, 2, ksize=3)
    Ixy = cv2.Sobel(sm, cv2.CV_32F, 1, 1, ksize=3)
    ij = np.round(uv).astype(int)
    resp = np.maximum(0, -(Ixx * Iyy - Ixy ** 2)[ij[:, 1], ij[:, 0]])
    ref = float(np.median(np.sort(resp)[len(resp) // 2:]))
    keep = resp >= SADDLE_FRAC * ref
    if keep.sum() < MIN_CORNERS:
        return None, f'uji sudut-X: hanya {int(keep.sum())} sudut sah'
    prior_rms = float(np.sqrt(np.mean(np.sum((uv - pred_uv) ** 2, 1))))
    if prior_rms > PRIOR_MAX_PX:
        return None, f'pelabelan jauh dari prediksi ({prior_rms:.1f} px)'
    ids = np.arange(12)[keep]
    return (ids, uv[keep].astype(float)), (f'fallback paritas {best[0]:.0f}/{second:.0f}, prior {prior_rms:.1f} px, '
                                           f'sudut-X {int(keep.sum())}/12')


def _template(gray, pred_uv, name):
    """No SB: render the board (with the 5 mm paper margin) through the PRIOR homography, find its translation by
    normalised cross-correlation (board edges break the checker periodicity), then the same lattice verification."""
    b = build_board(name)
    sx, sy = b.getChessboardSize()
    L = b.getSquareLength()
    P = board_points(name)
    src = np.array([P[i][:2] for i in range(12)], np.float32)
    H, _ = cv2.findHomography(src, pred_uv.astype(np.float32))
    ppm = 2000
    m = 0.005
    tex = b.generateImage((int(sx * L * ppm), int(sy * L * ppm)), marginSize=0, borderBits=1)
    pad = int(m * ppm)
    tex = cv2.copyMakeBorder(tex, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    S = np.array([[1 / ppm, 0, -m], [0, 1 / ppm, -m], [0, 0, 1]])               # texture px -> board metres
    lo = np.floor(pred_uv.min(0) - 0.25 * (pred_uv.max(0) - pred_uv.min(0)) - 5).astype(int)
    hi = np.ceil(pred_uv.max(0) + 0.25 * (pred_uv.max(0) - pred_uv.min(0)) + 5).astype(int)
    Hs = np.array([[1, 0, -lo[0]], [0, 1, -lo[1]], [0, 0, 1]]) @ H @ S
    size = tuple(int(v) for v in (hi - lo))
    tmpl = cv2.warpPerspective(tex, Hs, size, flags=cv2.INTER_AREA, borderValue=0)
    msk = cv2.warpPerspective(np.full_like(tex, 255), Hs, size, flags=cv2.INTER_AREA)
    R = PRIOR_MAX_PX
    x0, y0 = lo - int(R)
    x1, y1 = hi + int(R)
    if x0 < 0 or y0 < 0 or x1 > gray.shape[1] or y1 > gray.shape[0]:
        return None, 'template: jendela cari keluar bingkai'
    srch = gray[y0:y1, x0:x1]
    res = cv2.matchTemplate(srch, tmpl, cv2.TM_CCORR_NORMED, mask=msk)
    res = np.nan_to_num(res, nan=-1)
    _, peak, _, loc = cv2.minMaxLoc(res)
    shift = np.array(loc, float) - R
    cc = pred_uv + shift
    return _verify_lattice(gray, cc, pred_uv, name, gray)


SIGMA_FRACS = (0.0, 0.08, 0.12, 0.18)


def _fallback(gray, pred_uv, name='A'):
    """Try several pre-blurs (SB is sensitive to it); a wrong lattice cannot pass parity + uniqueness, so the first
    hypothesis that passes is accepted. All attempts' reasons are kept in the note."""
    why = []
    for f in SIGMA_FRACS:
        r, w = _fallback_once(gray, pred_uv, name, f)
        if r is not None:
            return r, f'{w}, sigma {f:.2f}L'
        why.append(f'{f:.2f}L: {w}')
    r, w = _template(gray, pred_uv, name)
    if r is not None:
        return r, f'{w}, template'
    why.append(f'template: {w}')
    return None, '; '.join(why)


def predict_uv(name, T_wl, T_lb, T_wc, K, dist=None):
    P = board_points(name)
    Pb = np.array([P[i] for i in range(12)])
    Tcb = inv(T_wc) @ T_wl @ T_lb
    rv, _ = cv2.Rodrigues(Tcb[:3, :3])
    uv, _ = cv2.projectPoints(Pb, rv, Tcb[:3, 3], K, np.zeros(5) if dist is None else np.asarray(dist))
    return uv.reshape(-1, 2)


def detect(gray, priors=None):
    """-> {board: dict(ids, uv, method, n_markers, note)} ; priors = {board: predicted 12x2 uv} (for the fallback)."""
    out = {}
    for name in BOARDS:
        r, n_m = _charuco(gray, name)
        if r is not None and len(r[0]) >= MIN_CORNERS:
            out[name] = dict(ids=r[0].tolist(), uv=r[1].tolist(), method='charuco', n_markers=n_m, note='')
            continue
        note = f'charuco {0 if r is None else len(r[0])} sudut, {n_m} marker'
        if priors and name in priors:
            fb, why = _fallback(gray, np.asarray(priors[name], float), name)
            if fb is not None:
                out[name] = dict(ids=fb[0].tolist(), uv=fb[1].tolist(), method='fallback', n_markers=n_m,
                                 note=f'{note}; {why}')
                continue
            note += f'; {why}'
        out[name] = dict(ids=[], uv=[], method='none', n_markers=n_m, note=note)
    return out


def median_over_frames(dets):
    """dets = list of detect()[board] for one board/camera over frames -> median per corner id (>= 50 % of frames)."""
    acc = {}
    for d in dets:
        for i, uv in zip(d['ids'], d['uv']):
            acc.setdefault(int(i), []).append(uv)
    need = 0.5 * len(dets)
    ids = sorted(i for i, v in acc.items() if len(v) >= need)
    uv = [np.median(np.array(acc[i]), 0).tolist() for i in ids]
    std = [float(np.max(np.std(np.array(acc[i]), 0))) for i in ids]
    methods = sorted({d['method'] for d in dets})
    return dict(ids=ids, uv=uv, std_px=std, n_frames=len(dets), methods=methods) if len(ids) >= MIN_CORNERS \
        else dict(ids=[], uv=[], std_px=[], n_frames=len(dets), methods=methods)


# ------------------------------------------------------------------------------------------------------- self-test
def render(name, Tcb, K, blur_px=1.0, noise=3.0, rng=None, bg=None, margin_m=0.005, bg_level=90):
    """Synthetic camera image of the printed board at pose Tcb (pattern only, white surround = cardboard/paper)."""
    b = build_board(name)
    sx, sy = b.getChessboardSize()
    L = b.getSquareLength()
    ppm = 4000                                     # board texture: 4 px/mm
    tex = b.generateImage((int(sx * L * ppm), int(sy * L * ppm)), marginSize=0, borderBits=1)
    pad = int(margin_m * ppm)                      # white paper margin around the pattern (print: 5 mm sides)
    tex = cv2.copyMakeBorder(tex, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    h, w = tex.shape
    m = margin_m
    obj = np.array([[-m, -m, 0], [sx * L + m, -m, 0], [sx * L + m, sy * L + m, 0],
                    [-m, sy * L + m, 0]])
    rv, _ = cv2.Rodrigues(Tcb[:3, :3])
    dst, _ = cv2.projectPoints(obj, rv, Tcb[:3, 3], K, np.zeros(5))
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst.reshape(-1, 2).astype(np.float32))
    img = np.full((IMG_H, IMG_W), bg_level, np.uint8) if bg is None else bg.copy()
    mask = cv2.warpPerspective(np.full_like(tex, 255), H, (IMG_W, IMG_H), flags=cv2.INTER_AREA)
    warped = cv2.warpPerspective(tex, H, (IMG_W, IMG_H), flags=cv2.INTER_AREA)
    img = np.where(mask > 127, warped, img).astype(np.uint8)
    if blur_px > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur_px)
    rng = rng or np.random.default_rng(0)
    img = np.clip(img + rng.normal(scale=noise, size=img.shape), 0, 255).astype(np.uint8)
    return img


def self_test():
    """Distance sweep: board facing the camera, 25 deg tilt; truth corners vs detected (charuco / fallback),
    + fallback labeling with a prior 30 px off, + both boards in one image keep their identities."""
    from scipy.spatial.transform import Rotation
    K = K_NOMINAL['rgbd']
    P = board_points('A')
    Pb = np.array([P[i] for i in range(12)])
    ok = True
    rng = np.random.default_rng(5)
    for bg_level, need_to in ((200, 2.5), (90, None)):
      print(f'latar {bg_level} (200 = pinggir putih lebar, 90 = kardus gelap); wajib lulus sampai {need_to} m')
      print('jarak  metode    sudut  marker  galat maks px  (blur 1.0 px, noise 3)')
      for d in (1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
          Tcb = np.eye(4)
          Tcb[:3, :3] = Rotation.from_euler('xyz', [25, -15, 10], degrees=True).as_matrix()
          Tcb[:3, 3] = [-0.10, -0.12, d]
          img = render('A', Tcb, K, rng=rng, bg_level=bg_level)
          rv, _ = cv2.Rodrigues(Tcb[:3, :3])
          truth = cv2.projectPoints(Pb, rv, Tcb[:3, 3], K, np.zeros(5))[0].reshape(-1, 2)
          prior = truth + [30.0, -20.0]                               # prior off by 36 px
          r = detect(img, {'A': prior})['A']
          if r['ids']:
              err = float(np.max(np.linalg.norm(np.array(r['uv']) - truth[r['ids']], axis=1)))
          else:
              err = float('nan')
          sq = float(np.linalg.norm(truth[1] - truth[0]))
          good = (r['method'] != 'none' and err < 0.3 * sq) or (need_to is None or d > need_to)
          good &= not (r['method'] != 'none' and not err < 0.3 * sq)   # a WRONG (mislabelled) detection always fails
          ok &= good
          print(f'{d:4.1f} m  {r["method"]:9s} {len(r["ids"]):3d}   {r["n_markers"]:3d}     {err:6.3f} (kotak {sq:4.1f} px)  '
                f'{"OK" if good else "GAGAL"}  {r["note"]}')
    # both boards, identity
    TA, TB = np.eye(4), np.eye(4)
    TA[:3, 3], TB[:3, 3] = [-0.25, 0, 1.4], [0.25, 0, 1.4]
    img = render('A', TA, K, rng=rng, bg_level=200)
    img = render('B', TB, K, rng=rng, bg=img)
    P_ = np.array([P[i] for i in range(12)])
    pri = {n: cv2.projectPoints(P_, np.zeros(3), T[:3, 3], K, np.zeros(5))[0].reshape(-1, 2) for n, T in (('A', TA), ('B', TB))}
    r = detect(img, pri)
    good = r['A']['method'] != 'none' and r['B']['method'] != 'none' and \
        np.mean(np.array(r['A']['uv'])[:, 0]) < np.mean(np.array(r['B']['uv'])[:, 0])
    ok &= good
    print(f'dua papan 1.4 m: A {r["A"]["method"]} {len(r["A"]["ids"])}, B {r["B"]["method"]} {len(r["B"]["ids"])},'
          f' identitas benar -> {"OK" if good else "GAGAL"}')
    # negative: the lower 40 % of the board hidden (e.g. by the gripper) -> no detection or a CORRECT one, never wrong
    nbad = 0
    for d in (1.5, 2.0, 2.5):
        Tcb = np.eye(4)
        Tcb[:3, :3] = Rotation.from_euler('xyz', [20, 10, -5], degrees=True).as_matrix()
        Tcb[:3, 3] = [0.05, -0.1, d]
        img = render('A', Tcb, K, rng=rng, bg_level=200)
        rv, _ = cv2.Rodrigues(Tcb[:3, :3])
        truth = cv2.projectPoints(Pb, rv, Tcb[:3, 3], K, np.zeros(5))[0].reshape(-1, 2)
        y_cut = int(truth[:, 1].min() + 0.6 * (truth[:, 1].max() - truth[:, 1].min()))
        img[y_cut:, :] = 60
        r = detect(img, {'A': truth + [20.0, 15.0]})['A']
        if r['ids']:
            err = float(np.max(np.linalg.norm(np.array(r['uv']) - truth[r['ids']], axis=1)))
            nbad += err > 2.0
        print(f'   tertutup 40 % @ {d} m: {r["method"]} {len(r["ids"])} sudut ({r["note"]})')
    good = nbad == 0
    ok &= good
    print(f'kontrol negatif papan tertutup: {nbad} deteksi SALAH -> {"OK" if good else "GAGAL"}')
    print('SELF-TEST', 'LULUS' if ok else 'GAGAL')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--image', help='deteksi satu citra (tanpa prior)')
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if a.image:
        r = detect(cv2.imread(a.image, cv2.IMREAD_GRAYSCALE))
        for k, v in r.items():
            print(k, v['method'], len(v['ids']), 'sudut', v['n_markers'], 'marker', v['note'])
        return 0
    ap.error('--self-test atau --image')


if __name__ == '__main__':
    sys.exit(main())
