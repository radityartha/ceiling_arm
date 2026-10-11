## A. Protokol — DIKUNCI

### A0. Pengungkapan

- 17 jadwal siap-HW (G43 + G49, rehearsal G47/G50). Rentang rotasi tiap jadwal dibaca dari `g43_candidates.json` /
  `g49_candidates.json` / `g42f_candidates.json` (sebelum §A): **keluar ±90 = 5**: s0 g1 −125, s2 g1 −105, s11 g2 +95,
  s12 g1 −110, s17 g1 −100. **12 lain di dalam ±90** (s14 g1 tepat +90 → batas INKLUSIF). s8 HW = jadwal G42f (|rot| ≤ 35).
- Asal: s0 s2 s11 s17 = G43 (Feas43, predikat tuple ok41 = ∀ cabang torsi), s12 = G49 (Feas49, ok49 = ∃ cabang). Kelimanya
  G46 (EdgeMatrix + henti-ε 3 % + STOP_TORQ), cermin 180° hidup. Gap R180: s0 0, s2 0.71 %, s11 1.95 %, s12 0, s17 0.87 %.
- Waktu R180 per seed 15–16 menit (Pool 15).
- Amplop HW terverifikasi G51: g1 −125…+90, g2 −35…+95. R90 ⊂ amplop g1; g2 −90…−40 BELUM terverifikasi.

### A1. 🔒 Metode

- `results/p1_g52/g52_make.py` = pembungkus `g49_make`/`g43_make` TANPA mengubah predikat: hanya `g40_make.RSET` diganti
  `R90 = {i : |RDEG[i]| ≤ 90}` (37 rotasi). Solver/predikat/henti = run asal tiap seed (ok41 untuk s0 s2 s11 s17, ok49 untuk
  s12). Semua cache = SALINAN di `p1_g52/` (berkas G43/G49 tidak ditulis).
- Cadangan (ditulis di §B bila dipakai): seed ok41 yang gagal (i)–(iii) di R90 dijadwal ulang dengan ok49 (pola G49).
- Pasca-jadwal: (iv′) `RotCrossChecker.sweep_rot` + (iv″) `G41.chain` (sama dengan G43/G49).
- Job > 10 menit: `setsid nohup`, log line-buffered, kandidat JSON dapat-dilanjut (seed selesai dilewati).

### A2. 🔒 Uji

1. **K-R180:** pembungkus di R180 mereproduksi s0 (G43) dan s12 (G49) — makespan dan jadwal identik (gap 0 keduanya).
2. **K-R90:** s5 (G49, |rot| ≤ 25) di R90 = jadwal R180-nya (identik).
3. **Jadwal R90** s0 s2 s11 s17 (ok41) + s12 (ok49) → (i)–(iii), lalu (iv′) + (iv″). Kontrol: semua stop |rot| ≤ 90;
   makespan R90 ≥ LB R180 seed itu (R90 ⊂ R180; ε 3 %).
4. Tabel 17 jadwal di R90 (12 tetap + 5 baru); daftar siap-HW R90 = lolos (i)–(iv″) DAN di dalam amplop G51 (yang memakai
   g2 < −35 → butuh kaki sweep tambahan, dicatat).
5. **Bila 3 lolos:** DRY-CLI mock k = 3 (`g31_screen.py --task-cli --final-retract --repeats 3`, `G47_GOAL_TOL_DEG=0.05`,
   pola G50) untuk tiap jadwal R90 baru.

### A3. 🔒 Gerbang 7a

Tabel 17 jadwal di R90 + daftar jadwal siap-HW di R90 ditulis; jadwal R90 baru DRY-CLI 3/3.

### A4. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D440 | K-R180 2/2 identik dan K-R90 1/1 identik. |
| D441 | ≥ **4/5** seed lolos (i)–(iii) di R90. |
| D442 | Seed lolos: makespan R90 ≤ **1.05 ×** R180. |
| D443 | Jadwal R90 baru lolos (iv′) ∧ (iv″) 100 %. |
| D444 | ≥ 1 jadwal R90 baru memakai g2 < −35 (di luar amplop G51). |
| D445 | DRY-CLI mock 3/3 untuk semua jadwal R90 baru. |

