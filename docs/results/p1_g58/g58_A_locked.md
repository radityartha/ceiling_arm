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

