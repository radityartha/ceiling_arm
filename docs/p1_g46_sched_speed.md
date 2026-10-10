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

## B. Hasil terukur

### B1. E1 — [tabel](results/p1_g46/g46_fine.log), [K-E1](results/p1_g46/g46_ke1.log), [matriks](results/p1_g46/g46_ke1_matrix.log)

- Tabel 116 242 pose (2 × 161 × 361) dalam **428 s** (Pool 15) → D380 ✓. d min g1 5.9 mm, g2 −20.4 mm (pose tabrak ada).
- **K-E1: 8 196/8 196 tepi cache verdict SAMA**, 0 beda di pita, 0 beda lain; |d_E1 − d_cache| median 0.002 mm, p99 1.46,
  maks 2.78 mm (titik ekstra float `rect_points`) → D381 ✓.
- Untuk solver: `EdgeMatrix` = verdict (b) SEMUA pasangan pose grid (2 376² per gantry) lewat RMQ 2D di tabel halus, 7.1 s
  sekali; regresi ulang 8 196/8 196. `solve_gantry_e1` = `solve_gantry_f` VERBATIM kecuali loop pasangan `E.get` → satu indeks
  matriks. Biaya tepi per instans: dari ≈ 60 % CPU run menjadi 0.

### B2. E3 — [profil](results/p1_g46/g46_prof.log)

- 30 tuple: CPU 381 s (median 9.2 s/tuple). `O2.solve` **68 %** (D382 ✓), `screen_line` 30.5 % (kotak 9.3, env 6.7, se-gantry
  6.1). 15 tuple tolak: **semua gagal torsi (ok3 0/15), 100 % waktunya di `O2.solve`**.
- Syarat-perlu eksak ditemukan di kode: `torq2` = ∀ solusi ∀ tilt dalam batas, `taumax` = envelope yang hanya naik ⇒ saat
  envelope pertama kali melewati batas (ekspresi `torq2` persis) verdict sudah PASTI False. `oracle2.solve(stop_torq=…)`
  berhenti di situ (`early_stop`); default `None` = tidak berubah. `g41_make.STOP_TORQ` (default False) mengaktifkannya
  hanya di `_tuple_job` (bukan diag `_c41_job`). Tuple ok3 tidak pernah memicu (envelope akhir ≤ batas) ⇒ hasil identik.
- **K-E3** ([log](results/p1_g46/g46_ke3.log)): 694 tuple ok41 + 100 tuple ok3-gagal dihitung ulang dengan `STOP_TORQ`:
  **0 verdict beda, 0 tujuan q6 beda**; `early_stop` 45/100 tuple gagal (55 sisanya gagal karena 0 solusi / tak jenuh, bukan
  torsi), CPU tuple-gagal 976 → 626 s (**−36 %**). CPU ok41 14 407 → 16 669 s = beban mesin (jalur kode identik, tidak ada
  `early_stop` di ok41).

### B3. E2 + uji akhir — [K-R35](results/p1_g46/g46_kr35.log), [run G43](results/p1_g43/g43_run.log)

- **K-R35 dengan E1+E2+E3: 20/20 identik G42f** (6 lolos: stop, penugasan, tujuan, makespan sama; 14 gagal dengan alasan
  sama), gap 0 di keenamnya.
- R180, 6 seed G42f (Pool 15, nice 19):

| seed | iterasi | wall | LB | UB = makespan | gap | G42f |
|---|---|---|---|---|---|---|
| 0 | 30 | **902 s** | 246.19 | 246.19 | 0 | 254.64 |
| 3 | 20 | 608 s | 320.20 | 323.81 | 1.13 % | 323.81 |
| 7 | 7 | 380 s | 270.60 | 275.94 | 1.97 % | 435.00 |
| 8 | 15 | 563 s | 312.59 | 315.89 | 1.06 % | 315.89 |
| 10 | 20 | 550 s | 323.45 | 323.45 | 0 | 367.09 |
| 18 | 17 | 435 s | 310.79 | 318.17 | 2.37 % | 318.17 |

  Total **57 menit** untuk 6 seed (sebelum G46: seed 0 saja > 45 menit tanpa selesai, iterasi ~3 menit → sekarang ~15–30 s).
  Iterasi pertama LB naik 167 → 204 s karena semua tepi kini EKSAK (tidak lagi relaksasi optimistis).

### B4. Papan skor D380–D385

| # | Dugaan | Hasil |
|---|---|---|
| D380 | tabel ≤ 15 menit | ✓ 428 s |
| D381 | K-E1 0 beda di luar pita, ≤ 1 % di pita | ✓ 0 / 0 |
| D382 | `O2.solve` ≥ 60 % | ✓ 68 % |
| D383 | seed 0 ≤ 30 menit | ✓ 15 menit |
| D384 | 6 seed ≤ 3 jam | ✓ 57 menit |
| D385 | gap median ≤ 3 % dan R180 < G42f pada ≥ 3/6 | ✓ gap median 1.1 %; lebih cepat 3/6 (s0 −3.3 %, s7 −36.6 %, s10 −11.9 %) |

**Tally G46: 6/6 tepat** (D385 tepat di batas). Percepatan: tepi 60 % → 0, tuple-gagal −36 %, iterasi berkurang lewat
henti-ε; predikat tidak berubah (K-E1 8 196/8 196, K-E3 0 beda, K-R35 20/20).
