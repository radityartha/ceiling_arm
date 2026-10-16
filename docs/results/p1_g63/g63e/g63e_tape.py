"""G63e: angka pita operator -> pose dunia penanda ArUco statis (trilaterasi dari unting P1..P4, tanpa FK sendi).

Berkas pita (JSON, angka HARFIAH operator, cm kecuali disebut):
  {"tag_side_mm": {"20": [180.1, 179.9], ...},              # sisi hitam terukur C0-C1, C0-C3 tiap penanda
   "plumb": {"P1-P2": 80.0, ...},                            # jarak antar tanda unting lantai (>= 5 dari 6 pasang)
   "tags": {
     "20": {"surf": "lantai", "z_cm": 0,
            "C0": {"P2": .., "P3": .., "P4": ..}, "C1": {..}, "C2": {..}, "C3": {..}},
     "21": {"surf": "kotak_B", "z_cm": 48.0,                 # atas platform: E1/E2 = sudut ATAS platform diunting ke lantai
            "E1": {"P2": .., "P3": .., "P4": ..}, "E2": {..}, "E1E2_top": 42.3,
            "C0": {"E1": .., "E2": ..}, "C1": {..}, "C2": {..}, "C3": {..}},
     "22": {"surf": "laci_pindah_sisi",                     # sisi vertikal: F1/F2 = sudut BAWAH muka itu di lantai (langsung)
            "F1": {"P2": .., "P3": .., "P4": ..}, "F2": {..}, "F1F2": 33.0,
            "C0": {"along": .., "z": ..}, ...}}}              # along = dari F1 menuju F2 sepanjang muka; z = tinggi dari lantai
Cek (gagal = penanda ditandai, TIDAK dibuang diam-diam): residu trilaterasi, |E1E2| / |F1F2|, sisa Procrustes per sudut
(bentuk kotak terukur -> satu angka pita salah baca terlihat), unting vs URDF.
"""
import numpy as np

import g63e_common as E
import g63d_tape as TD

TRI_RES_MAX_M = 0.005
EDGE_TOL_M = 0.005
PROC_RES_MAX_M = 0.005
PLUMB_FIT_MAX_M = TD.PLUMB_FIT_MAX_M


def tag_side(t, i):
    s = t.get('tag_side_mm', {}).get(str(i))
    return float(np.mean(s)) / 1000.0 if s else None


def tags_world(t, plan=None):
    """-> ({id: dict(T, corners, side, checks, ok)}, meta). plan (g63e_plan.json) = tebakan awal (ambiguitas cermin)."""
    Pw, pres, prms = TD.plumb_world(t)
    guess = {str(r['id']): np.array(r['corners']) for r in (plan or {}).get('tags', [])}
    out = {}
    for name, b in t['tags'].items():
        chk, refs = {}, dict(Pw)
        g = guess.get(name, np.tile([0.8, 0.0, 0.0], (4, 1)))
        side = tag_side(t, name) or E.TAG_NOMINAL
        C = np.zeros((4, 3))
        if 'F1' in b:                                   # sisi vertikal
            for e in ('F1', 'F2'):
                xy, rr = TD.trilat([Pw[p] for p in b[e]], [v / 100 for v in b[e].values()], g[0, :2])
                refs[e] = xy
                chk['tri_' + e] = round(rr, 4)
            u = refs['F2'] - refs['F1']
            chk['F1F2_err'] = round(abs(np.linalg.norm(u) - b['F1F2'] / 100), 4)
            u = u / np.linalg.norm(u)
            for k in range(4):
                c = b['C%d' % k]
                C[k] = np.r_[refs['F1'] + c['along'] / 100 * u, c['z'] / 100]
        else:
            z = b['z_cm'] / 100.0
            if 'E1' in b:
                for e in ('E1', 'E2'):
                    xy, rr = TD.trilat([Pw[p] for p in b[e]], [v / 100 for v in b[e].values()], g[0, :2])
                    refs[e] = xy
                    chk['tri_' + e] = round(rr, 4)
                chk['E1E2_err'] = round(abs(np.linalg.norm(refs['E1'] - refs['E2']) - b['E1E2_top'] / 100), 4)
            for k in range(4):
                c = b['C%d' % k]
                xy, rr = TD.trilat([refs[p] for p in c], [v / 100 for v in c.values()], g[k, :2])
                C[k] = np.r_[xy, z]
                chk['tri_C%d' % k] = round(rr, 4)
        T, res = E.procrustes_pose(C, side)
        chk['proc_res_max'] = round(float(res.max()), 4)
        ok = (all(v <= TRI_RES_MAX_M for kk, v in chk.items() if kk.startswith('tri_'))
              and chk.get('E1E2_err', 0) <= EDGE_TOL_M and chk.get('F1F2_err', 0) <= EDGE_TOL_M
              and chk['proc_res_max'] <= PROC_RES_MAX_M)
        out[int(name)] = dict(T=T, corners=C, side=side, surf=b['surf'], checks=chk, ok=bool(ok))
    return out, dict(plumb_res=pres, plumb_rms=prms, plumb_ok=max(pres.values()) <= PLUMB_FIT_MAX_M)
