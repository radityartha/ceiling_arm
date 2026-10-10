# P1 / G46 — evaluasi + percepatan penjadwal (E1 tabel tepi, E2 henti-ε bersertifikat, E3 profil tuple)

> Sesi 2026-10-10. Operator: "sistem dengan waktu sampai berhari-hari sudah pasti tidak efisien dan efektif" → pilihan
> **E1+E2+E3 dulu** (AskUserQuestion). Fase evaluasi disisipkan di Fase 1 kampanye (G43 dijeda 16:15, cache utuh).
> OFFLINE, NOL gerak HW, tanpa mock. **§A DIKUNCI SEBELUM mengukur** (salinan `docs/results/p1_g46/g46_A_locked.md` +
> sha256). §B menang atas §A dan DITULIS. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Pengungkapan (dari G43 B0, sudah terukur)

- 30 menit, Pool 15: +915 tuple (median 10.7 s CPU, **93 % tak layak**), +594 tepi (median 3 612 titik sapuan, ≈ 29 s CPU
  per tepi) → tepi ≈ 60 % CPU, tuple ≈ 35 %, solver 2.5 s/iterasi. Seed 0 R180 tidak konvergen dalam ~45 menit.
- `EnvChecker.screen_trajectory` = min `check()` atas titik (B&B eksak G35); verdict = CLEAR / MARGIN (< 50 mm) / COLLIDE.
  `rect_points` = persegi lin × rot, langkah ≤ 10 mm × 1°, TETAPI `ceil` float menambah satu titik pada 370 pasangan lin
  dan 1 232 pasangan rot (titik bergeser ≤ 5 mm / 0.5° dari kisi) — sapuan lama tidak selalu di kisi.
- Tuple-tuple layak pertama kali sudah ada di cache G42 (R35) → jadwal G42f tersedia untuk batas atas sejak iterasi 1.

### A1. 🔒 E1 — tabel pose halus untuk tepi (b)

- Per gantry, kisi **lin 0.00…1.60 langkah 0.01 m (161) × rot −180…+180 langkah 1° (361)**: d = `ec.check(q, only=t{g}_)`
  dengan lengan REST, gantry lain 0/0 (= `_edge_job` per titik). Disimpan `g46_fine_pose.npz`.
- Verdict tepi E1 = klasifikasi (min d atas SEMUA titik kisi di persegi tertutup [l0,l1] × [r0,r1]) dengan ambang yang sama
  (≤ 0 COLLIDE, < 50 mm MARGIN). Kerapatan = langkah terkunci G40 (10 mm × 1°), tanpa titik ekstra float.
- **K-E1 regresi:** semua tepi terhitung di `g43_edge_cache.jsonl` (bukan cermin): verdict E1 vs cache. Beda diterima HANYA
  bila |d_cache − 50 mm| dan |d_E1 − 50 mm| ≤ 2 mm (diskretisasi beda di tepi ambang); beda lain = E1 GAGAL (tidak dipakai).
  Juga: d_E1 vs d_cache selisih maks dilaporkan.

### A2. 🔒 E2 — henti-ε bersertifikat (ε = 3 %)

- Tiap iterasi lazy: **LB** = makespan relaksasi (tak diketahui = layak) — batas bawah sah. **UB** = `solve_exact_f` pada
  instans yang sama dengan tuple/tepi TAK DIKETAHUI = TAK LAYAK (hanya fakta terverifikasi); stop ≥ 2 tugas UB yang belum
  dihitung → (s42) dihitung, tolak → larangan, ulang UB. Tidak ada jadwal UB → UB = ∞.
- Berhenti bila **UB ≤ (1 + ε)·LB** → jadwal UB dipakai (semua tuple, tepi, stop terverifikasi = predikat sama), dicatat
  LB, UB, gap. Juga berhenti seperti lama bila relaksasi terverifikasi penuh (gap 0).
- Batas atas waktu per seed: tidak ada (dilaporkan); laporan per seed: iterasi, wall, gap akhir.

### A3. 🔒 E3 — profil tuple

- 30 kunci tuple acak (seed 46) dari `g43_tuple_cache.jsonl` (15 ok41, 15 tolak), `G41._tuple_job` dengan timer pada:
  `O2.solve`, `O2.torq2`, `ec.check` titik akhir, `bx.dist`, `screen_line` (torsi garis / env / kotak / se-gantry).
- Bila satu komponen ≥ 50 % CPU dan ada syarat-perlu murah yang TIDAK pernah menolak tuple layak (dibuktikan pada semua
  600+ tuple ok41 di cache), dipasang sebagai pra-tolak; selain itu E3 = laporan saja.

### A4. 🔒 Uji akhir

- K-R35 ulang dengan E1+E2(ε)+E3: 6 seed G42f lolos, makespan ≤ G42f (UB tersedia sejak iterasi 1).
- G43 R180 dilanjutkan dengan E1+E2+E3 → hasil G43 §B (Gerbang 1).

### A5. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D380 | E1 tabel 116 k pose ≤ **15 menit** wall (Pool 15). |
| D381 | K-E1: 0 beda verdict di luar pita ±2 mm; beda di pita ≤ **1 %** tepi. |
| D382 | E3: `O2.solve` ≥ **60 %** CPU tuple. |
| D383 | E2: seed 0 R180 selesai ≤ **30 menit** wall (E1+E2). |
| D384 | E2: 6 seed G42f R180 selesai total ≤ **3 jam**. |
| D385 | Gap akhir median ≤ 3 % (definisi) dan makespan R180 < G42f pada ≥ 3/6 seed. |
