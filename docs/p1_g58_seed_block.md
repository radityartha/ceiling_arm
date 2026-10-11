# P1 / G58 — blok seed tambahan siap-HW (lama vs baru, runner paralel) — OFFLINE + mock

> Sesi 2026-10-12 (operator: "offline dulu, baru HW"; prompt `docs/p1_g58_offline_prompt.md`). NOL gerak HW, stack HW mati;
> mock `ROS_DOMAIN_ID=77` (fake HW, BUKAN data HW). §A dikunci (salinan `docs/results/p1_g58/g58_A_locked.md` + sha256)
> SEBELUM mock. §B menang atas §A. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Pengungkapan (dari berkas, sebelum mock)

- **s8 "lama" = jadwal G42f (`p1_g42/g42f_candidates.json`, varian `G42`), BUKAN G43 R180.** Prompt menyebut
  `g43_candidates.json` R180 untuk s3 s8 s18, tetapi s8 R180 G43 GAGAL (iv′) (G43 B `p1_g43_rot180.md:127`, G48 R7: "s8 memakai
  jadwal G42f"); jadwal itu tidak pernah siap-HW. **Celah G55:** `g55_eval.py` `rows()` hanya memeriksa (i)–(iii), sehingga
  s8 G43 R180 (sim 402.11) ikut sebagai kandidat "lama" dan tertulis sebagai "lama terbaik" di G55 B2 — seleksi tidak
  terpengaruh (baru 264.44 menang), tetapi tabel G55 B2 baris s8 salah label. Pembanding s8 di sini = G42f (sim 409.42).
- **DRY-CLI jadwal lama sudah 3/3 dengan tol 0.05°** (tidak diulang): s3 R180 + s8 G42 = `p1_g47/g47_drycli3.json`, s18 R180 =
  `p1_g47/g47_ab005.json`, s12 R90 = `p1_g52/g52_drycli.json`; s7 R180 = `g47_ab005`, s10 G42 = `g47_drycli3`. Probe sesudahnya
  berubah (G54 `--reserve-dir`/`--out-json`, bawaan-mati) — smoke run_g54 memanggil probe untuk tiap tugas, jadi jalur kini
  teruji ulang di smoke.
- **s12: jadwal lama = terpilih** (G55: G52 R90 sim 313.49, struktur P1″ sama) → tidak ada pasangan; satu smoke saja.
- Semua jadwal di dalam amplop HW (g1 −125…+90, g2 −35…+95): s3 ±35, s8 ±35, s18 ±35, s12 g1 +40 / g2 −5, s7 lama g1 +70.
- Jadwal baru s3 s8 s18 sudah smoke mock di G55 (sesi mock lain); di sini DIULANG di sesi mock yang sama dengan jadwal lama
  (pembanding sama-sesi, seperti yang direncanakan untuk HW).

### A1. 🔒 Uji mock (domain 77; launch SIG_DFL + setsid, SigIgn tanpa 0x2, 7/7 controller; monitor `reach_dwell_monitor` +
perekam `p1_g22/js_record.py` HIDUP sebelum smoke; REST + g1 PARK 1.50 / g2 0; K0 LULUS)

Smoke paralel `p1_g54/g54_smoke.sh` (run_g54 `--final-retract --goal-tol-deg 0.05`, lalu pulang retrace `g47_home.sh` + K0),
urutan selang-seling, satu job `setsid nohup`:

| # | run | plan / varian | sim P1″ (g54_sim) |
|---|---|---|---|
| 1 | s3 lama | `p1_g43/g43_candidates.json` / `R180` | 426.08 |
| 2 | s3 baru | `p1_g55/g55_selected.json` / `R90sel` | 335.75 |
| 3 | s8 baru | `p1_g55/g55_selected.json` / `R90sel` | 264.44 |
| 4 | s8 lama | `p1_g42/g42f_candidates.json` / `G42` | 409.42 |
| 5 | s18 lama | `p1_g43/g43_candidates.json` / `R180` | 372.16 |
| 6 | s18 baru | `p1_g55/g55_selected.json` / `R90sel` | 298.54 |
| 7 | s12 (lama = baru) | `p1_g52/g52_candidates.json` / `R90` | 313.49 |
| 8 | s7 lama (ulang) | `p1_g43/g43_candidates.json` / `R180` | 396.91 |
| 9 | s7 baru (ulang) | `p1_g55/g55_selected.json` / `R90sel` | 283.62 |
| 10 | s10 baru (ulang) | `p1_g55/g55_selected.json` / `R90sel` | 278.50 |
| 11 | s10 lama (ulang) | `p1_g42/g42f_candidates.json` / `G42` | 360.31 |

Dicatat per run: makespan mock, rasio mock/sim, tugas SUCCESS, RESERVE-BLOCKED, AUTO-STOP, waktu dinding run + pulang
(dasar perkiraan waktu sesi HW). Mock sebelumnya: s7 lama 380.16 / baru 280.98, s10 lama 393.57 / baru 313.24 (G54/G55).

### A2. 🔒 Gerbang G58

11/11 run 6/6 SUCCESS, 0 AUTO-STOP, pulang + K0 LULUS; tabel pasangan siap-HW (lama vs baru) + prediksi sim + perkiraan
waktu sesi HW per pasangan.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D500 | Smoke 11/11: 6/6 SUCCESS, 0 AUTO-STOP, 0 RESERVE-BLOCKED, pulang + K0 LULUS. |
| D501 | mock/sim ∈ [0.8, 1.25] di 11/11 run. |
| D502 | Δ(lama − baru) mock > 0 untuk s3, s8, s18, s7, s10 (tanda = sim). |
| D503 | Ulangan s7/s10 (run 8–11) dalam ±5 % dari mock G54/G55 seed + varian sama. |
| D504 | Rasio Δ mock / Δ sim (lama − baru) ∈ [0.5, 1.5] untuk 5/5 pasangan. |

## B. Hasil terukur

§A sha256 `0e7dbdf0…` ([salinan](results/p1_g58/g58_A_locked.md)). Mock 77 baru 06:05 (SIG_DFL, SigIgn `0x1001001`, 7/7, 0 node
ganda), monitor + perekam hidup sebelum smoke, REST + g1 PARK 1.50 (`pose_to_g`) + K0 LULUS. Job
[g58_mock.sh](results/p1_g58/g58_mock.sh) 06:11–08:04, [log](results/p1_g58/g58_mock.log), laporan
[g58_report.py](results/p1_g58/g58_report.py) → [JSON](results/p1_g58/g58_report.json); arsip per run `p1_g54/smoke/g58_*`.
Selama job hanya kerja ringan lain (nice 19, 1 utas: K-SIM/K-DP G60, < 2 menit CPU) — tidak ada Pool.

### B1. Smoke paralel mock (fake HW, BUKAN data HW)

| # | run | makespan mock | sim | rasio | bersamaan | dinding run | pulang | mock sebelumnya |
|---|---|---|---|---|---|---|---|---|
| 1 | s3 lama (G43 R180) | 446.13 | 426.08 | 1.047 | 171.8 s | 8.3 mnt | 4.5 mnt | — |
| 2 | s3 baru (R90sel) | 374.37 | 335.75 | 1.115 | 126.0 s | 7.0 | 3.6 | 376.18 (−0.5 %) |
| 3 | s8 baru | 272.00 | 264.44 | 1.029 | 118.4 s | 5.4 | 2.8 | 267.44 (+1.7 %) |
| 4 | s8 lama (G42f) | 443.86 | 409.42 | 1.084 | 197.6 s | 8.2 | 4.0 | — |
| 5 | s18 lama (G43 R180) | 431.06 | 372.16 | 1.158 | 147.9 s | 7.9 | 3.6 | — |
| 6 | s18 baru | 352.67 | 298.54 | 1.181 | 118.2 s | 6.7 | 3.8 | 354.37 (−0.5 %) |
| 7 | s12 (G52 R90 = terpilih) | 332.32 | 313.49 | 1.060 | 118.5 s | 6.3 | 2.9 | — |
| 8 | s7 lama (G43 R180) | 383.64 | 396.91 | 0.967 | 170.9 s | 7.2 | 3.1 | 380.16 (+0.9 %) |
| 9 | s7 baru | 276.40 | 283.62 | 0.975 | 150.2 s | 5.4 | 2.4 | 280.98 (−1.6 %) |
| 10 | s10 baru (= R35-P1″) | 308.87 | 278.50 | 1.109 | 99.1 s | 5.9 | 2.0 | 313.24 (−1.4 %) |
| 11 | s10 lama (G42f) | 383.17 | 360.31 | 1.063 | 99.8 s | 7.1 | 2.5 | 393.57 (−2.6 %) |

- 11/11: 6/6 tugas SUCCESS, semua event rc 0, 0 RESERVE-BLOCKED, 0 AUTO-STOP, pulang retrace + K0 LULUS → **D500 ✓**.
- Rasio mock/sim 0.967–1.181 → **D501 ✓**. Pola sama G55: mock lebih lambat dari sim bila banyak tugas bersamaan
  (penyaringan reservasi + perencanaan terserialkan); s7 sedikit lebih cepat.
- Pasangan lama − baru (mock, sesi sama): **s3 −71.8 s (−16.1 %), s8 −171.9 s (−38.7 %), s18 −78.4 s (−18.2 %), s7 −107.2 s
  (−28.0 %), s10 −74.3 s (−19.4 %)**; sim −90.3 / −145.0 / −73.6 / −113.3 / −81.8 → tanda sama 5/5 → **D502 ✓**; rasio Δmock/Δsim
  0.79 / 1.19 / 1.06 / 0.95 / 0.91 → **D504 ✓**.
- Ulangan s7/s10 vs G54/G55 (sesi mock lain): +0.9 / −1.6 / −1.4 / −2.6 % → **D503 ✓** (variansi run-ke-run mock ≤ 3 %).

### B2. Tabel pasangan siap-HW (runner paralel `run_g54`, urutan selang-seling)

Semua jadwal: DRY-CLI 3/3 tol 0.05° (A0), smoke paralel mock 6/6, di dalam amplop HW. Perkiraan waktu HW per run = 1.05 ×
dinding run mock + pulang mock + 3 mnt izin/DRY pulang + 6.5 mnt plan-only nyata k = 3 (G51: 13 mnt per 2 jadwal) + 1 mnt DRY
run_g54.

| blok | run | plan / varian | sim | mock | perkiraan HW (mnt) |
|---|---|---|---|---|---|
| G57 (prioritas) | s7 lama / baru | `p1_g43/g43_candidates.json` R180 / `p1_g55/g55_selected.json` R90sel | 396.9 / 283.6 | 383.6 / 276.4 | 21 / 19 |
| G57 | s10 baru / lama | g55_selected R90sel / `p1_g42/g42f_candidates.json` G42 | 278.5 / 360.3 | 308.9 / 383.2 | 19 / 21 |
| G57 | s0 baru | g55_selected R90sel | 148.2 | 163.4 (G55) | ~15 |
| G58 | s8 baru / lama | g55_selected R90sel / g42f G42 | 264.4 / 409.4 | 272.0 / 443.9 | 19 / 23 |
| G58 | s3 lama / baru | `p1_g43/g43_candidates.json` R180 / g55_selected R90sel | 426.1 / 335.8 | 446.1 / 374.4 | 24 / 22 |
| G58 | s18 baru / lama | g55_selected R90sel / g43 R180 | 298.5 / 372.2 | 352.7 / 431.1 | 21 / 22 |
| G58 (opsional) | s12 | `p1_g52/g52_candidates.json` R90 (= terpilih) | 313.5 | 332.3 | 20 |
| ulangan variansi | s7 / s10 pasangan kedua | sama dengan G57 | | ±3 % mock | 80 |

Total blok G58 (s8 s3 s18, 6 run) ≈ **2 jam 10 mnt**; s12 +20 mnt; ulangan s7/s10 +80 mnt. Pasangan dengan Δ mock terbesar
(paling tahan derau HW): s8 (−172 s), s7 (−107 s); terkecil s3 (−72 s), s10 (−74 s) — masih ≫ variansi mock 3 %.

### B3. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D500 | smoke 11/11 bersih | ✓ |
| D501 | mock/sim ∈ [0.8, 1.25] | ✓ 0.967–1.181 |
| D502 | Δ(lama − baru) > 0, 5/5 | ✓ |
| D503 | ulangan s7/s10 ±5 % | ✓ ≤ 2.6 % |
| D504 | Δmock/Δsim ∈ [0.5, 1.5] | ✓ 0.79–1.19 |

**Tally G58: 5/5. Gerbang G58: LULUS.** Catatan: label G55 B2 baris s8 ("lama terbaik G43 R180 402.11") salah — jadwal itu gagal
(iv′); pembanding sah = G42f 409.42 (A0). Ditulis di sini, dokumen G55 tidak diubah.
