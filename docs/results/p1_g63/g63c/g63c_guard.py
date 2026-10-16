"""G63c penjaga IDENTITAS papan (eksperimen; detektor terkunci g63b_detect TIDAK diubah, penjaga = saringan SESUDAH deteksi).
Aturan didaftarkan 2026-10-16 sebelum uji frame (dari skala median tercatat, bukan dari frame):
  ID-1 jarak pusat deteksi ke prior papan sendiri <= 0.18 m (pada skala prior; = asumsi galat prior §A3)
  ID-2 untuk SETIAP papan lain yang prior-nya di depan kamera dan pusatnya di bingkai: jarak ke prior itu >= 2 x jarak sendiri
  ID-3 skala kotak deteksi / prior sendiri dalam [1/1.3, 1.3]
Gagal salah satu -> deteksi frame ditolak (papan identik tidak bisa dibedakan dari citra, hanya dari geometri prior)."""
import numpy as np

MAX_OWN_M, MIN_RATIO, MAX_SCALE = 0.18, 2.0, 1.3


def _sq(uv):
    return float(np.median(np.linalg.norm(np.diff(np.asarray(uv, float).reshape(4, 3, 2), axis=1), axis=2)))


def check(name, uv, priors, wh, sq_m=0.042):
    """priors = {papan: dict(uv=12x2, depth=m)} untuk SEMUA papan; -> (ok, catatan)."""
    uv = np.asarray(uv, float)
    p = priors[name]
    ppm = _sq(p['uv']) / sq_m
    d_own = np.linalg.norm(uv.mean(0) - p['uv'].mean(0)) / ppm
    if d_own > MAX_OWN_M:
        return False, f'ID-1 jarak prior sendiri {d_own:.3f} m'
    s = _sq(uv) / _sq(p['uv'])
    if not 1 / MAX_SCALE <= s <= MAX_SCALE:
        return False, f'ID-3 skala {s:.2f}'
    for o, q in priors.items():
        if o == name or q['depth'] <= 0:
            continue
        c = q['uv'].mean(0)
        if not (0 <= c[0] < wh[0] and 0 <= c[1] < wh[1]):
            continue
        d_o = np.linalg.norm(uv.mean(0) - c) / (_sq(q['uv']) / sq_m)
        if d_o < MIN_RATIO * d_own:
            return False, f'ID-2 ambigu: {o} {d_o:.3f} m vs sendiri {d_own:.3f} m'
    return True, f'ok d {d_own:.3f} m skala {s:.2f}'
