# P1 / G54 — Fase 7c: runner PARALEL (mock)

> Sesi 2026-10-11 malam. Kampanye FASE 7c. NOL gerak HW; mock `ROS_DOMAIN_ID=77` (fake HW, BUKAN data HW). §A dikunci
> (salinan `docs/results/p1_g54/g54_A_locked.md` + sha256) SEBELUM uji mock. §B menang atas §A. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Pengungkapan

- **Apa yang dijamin jadwal untuk paralel:** (iii) `sched_coll` = tabrakan STRUKTUR gantry (bar + plat, XY) terhadap
  waktu, c_clear 0. LENGAN antar-gantry TIDAK. (iv′)/(iv″) memeriksa jalan SERIAL. Penyaring probe/return_rest memakai
  keadaan gantry lain TERUKUR (basi bila gantry itu bergerak). → runner paralel butuh mekanisme sendiri.
- `pose_to_g` gagal (rc 3) bila lengan MANA PUN bergeser > 0.5° atau rel/rotasi lain bergerak; S12/S28 menyaring dari keadaan
  terukur → traverse harus EKSKLUSIF (alat tidak diubah).
- Satu controller per gantry (`gantry_g_with_arm_controller`) untuk traverse/retract/tugas; dua gantry = dua controller. Probe
  mengeksekusi lewat `move_group` ExecuteTrajectory — eksekusi bersamaan dua goal di MoveIt 2.5.10 BELUM diketahui (A2.1).
- Simulator `g54_sim.py` sudah dijalankan SEBELUM §A ini (ia adalah prediksi model, bukan pengukuran mock). Versi pertama
  (tanpa prioritas traverse) memberi paralel ≈ serial (traverse g2 kelaparan) → aturan keadilan ditambahkan (A1), prediksi
  final di A3.
- Perubahan kode (bawaan-mati, argv lama identik): `scripts/reach_dwell_probe.py` `--reserve-dir`, `--out-json`, refaktor
  `_execute` = ekor lama `move_to` apa adanya. Disunting SAAT DRY-CLI G52 berjalan (jalur bawaan tidak berubah).

### A1. 🔒 Runner `results/p1_g54/run_g54.py`

Event per gantry = `g31_screen.events(schedule, p0, final_retract)` gantry itu, berurutan (= run_g32). Gerbang:
- **traverse**: hanya bila gantry lain tidak menjalankan apa pun; selama traverse gantry lain tidak memulai apa pun.
  **Keadilan:** traverse yang sudah menunggu didahulukan atas event BARU gantry lain (dua traverse menunggu: yang lebih dulu).
- **retract**: gantry lain tidak traverse DAN garis lurus return_rest (kedua lengan, dari keadaan terukur) CLEAR 50 mm terhadap
  SETIAP keadaan reservasi gantry lain → reservasi sendiri = garis itu → `return_rest --no-plan`.
- **tugas**: gantry lain tidak traverse; probe `--reserve-dir` menyaring RENCANA NYATA vs reservasi gantry lain di bawah flock,
  CLEAR → reservasi sendiri = terukur + rencana → eksekusi; RESERVE-BLOCKED = tidak bergerak → tunggu batas event gantry lain,
  ulang (maks 30).
- Sesudah tiap event: reservasi = keadaan terukur. Keduanya terhalang dan tidak ada yang berjalan = DEADLOCK → AUTO-STOP.
- Penjaga S26 KONTINU (10 Hz): proses hidup, fault baru, torsi ≤ 14, rel ±2 mm / rot ±1° untuk gantry yang tidak traverse,
  /joint_states segar. K0 + amplop rotasi sebelum t0. Stop apa pun → AUTO-STOP KEDUA gantry: SIGINT ke semua alat berjalan +
  CancelGoal-semua di `/execute_trajectory` dan kedua `gantry_g_with_arm_controller`; rc 3.
- makespan = t_success tugas terakhir − t0 (= run_g32).

### A2. 🔒 Uji (mock 77, monitor + perekam hidup, mulai REST + PARK/p0, K0 LULUS)

1. **Eksekusi bersamaan:** dua probe `--move` bersamaan (satu lengan g1, satu lengan g2, target tugas jadwal dari keadaan
   REST + p0) → keduanya MOVED/SUCCESS dan selang eksekusi tumpang-tindih.
2. **DRY** run_g54 5 run: s7 R180, s7 R35 (G42f), s10 R180, s10 R35 (G42f), s0 R90 (G52; = jadwal G42f s0).
3. **Smoke `--move`** 5 run, pulang retrace + K0 sesudah tiap run.
4. **Kontrol negatif:** s7 R180 `--inject-stop 2:4` (sim: g2 akan retract saat g1 sedang retract). Sah hanya bila gantry lain
   sedang menjalankan event bergerak saat stop (`inject.other_running`); bila tidak, ulang dengan titik lain (ditulis).
   Lulus: rc 3 dan gerak gantry lain sesudah stop (maks |dq| di [t_stop+1, t_stop+3] s) ≤ 0.002 rad / 1 mm.

### A3. 🔒 Prediksi model (g54_sim, biaya P1″ G53, garis lurus)

| run | paralel P1″ (sim) | P1′-par solver | P1′-serial | blok reservasi |
|---|---|---|---|---|
| s7 R180 | 396.91 | 275.94 | 549.18 | 0 |
| s7 R35 | 470.33 | 435.00 | 548.14 | 0 |
| s10 R180 | 413.83 | 323.45 | 567.52 | 0 |
| s10 R35 | 360.31 | 367.09 | 472.32 | 0 |
| s0 R90 | 281.06 | 254.64 | 467.04 | 0 |

### A4. 🔒 Gerbang 7c

A2.1 lulus (atau jalur eksekusi diganti dan diuji ulang), DRY 5/5, smoke 5/5 (semua tugas SUCCESS, 0 HALTED, 0 AUTO-STOP,
urutan event per gantry = jadwal), kontrol negatif lulus.

### A5. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D460 | move_group mengeksekusi 2 ExecuteTrajectory beda gantry bersamaan (A2.1 lulus tanpa ubah jalur). |
| D461 | Smoke 5/5 lulus (6/6 SUCCESS, 0 HALTED, 0 AUTO-STOP). |
| D462 | 0 RESERVE-BLOCKED di smoke (sim: 0). |
| D463 | makespan mock / sim P1″ ∈ [0.8, 1.2] untuk 5/5 run. |
| D464 | Tanda Δ(R180 − R35) mock = sim: s7 negatif, s10 positif. |
| D465 | Kontrol negatif lulus (gantry lain berhenti, rc 3). |

## B. Hasil terukur

§A sha256 `bba58b2d…` ([salinan](results/p1_g54/g54_A_locked.md)). Mock 77 (launch SIG_DFL, SigIgn `0x1001001`, 7/7 controller),
REST + PARK/p0, monitor + perekam hidup, K0 LULUS sebelum dan sesudah tiap run. Kode: [run_g54.py](results/p1_g54/run_g54.py),
[reserve.py](results/p1_g54/reserve.py), [g54_sim.py](results/p1_g54/g54_sim.py), probe `--reserve-dir/--out-json`; laporan
[g54_report.log](results/p1_g54/g54_report.log), arsip per run `results/p1_g54/smoke/`.

### B1. A2.1 eksekusi bersamaan — [log](results/p1_g54/g54_concurrent_exec.log), [skrip](results/p1_g54/g54_concurrent_exec.py)

- **move_group `/execute_trajectory` MENSERIALKAN**: goal ke-2 baru "Received goal request" sesudah goal ke-1 selesai (log
  move_group); gerak terukur arm_1 0.00–5.96 s, arm_3 6.09–12.05 s (tumpang-tindih −0.13 s) → **D460 ✗**. (Versi pertama skrip
  menghitung tumpang-tindih dari waktu klien dan keliru melaporkan "sukses bersamaan" — diganti dengan /joint_states.)
- **Jalur ganti (A4):** probe `--reserve-dir` mengirim rencana yang SAMA langsung ke `gantry_g_with_arm_controller`
  (FollowJointTrajectory, = jalur return_rest/pose_to_g; cek awal 0.01 rad + batas waktu durasi × 3 + 5 s seperti TEM): gerak
  0.00–5.96 s keduanya, tumpang-tindih **5.96 s** → A2.1 LULUS dengan jalur ganti. Juga diperbaiki: `_reserve` memakai
  `GANTRY_OF` (string 'gantry_1') sebagai int — bug sebelum dipakai.

### B2. Smoke s7 R180 percobaan 1 — GAGAL, dua cacat runner (arsip `results/p1_g54/smoke/try1/`)

1. AUTO-STOP penjaga "/joint_states basi" di t ≈ 157 s: penyaringan retract (CrossGantryChecker vs reservasi g2) berjalan
   16.7 s DI PROSES runner → thread Watch kelaparan GIL → keadaan sendi tampak basi. **Positif palsu penjaga.**
2. **Cacat keselamatan:** sesudah AUTO-STOP, worker g1 yang sedang menyaring tetap MELUNCURKAN `return_rest` (3.8 s sesudah stop,
   arm_1 bergerak 31 s, fake HW); jendela pasca-stop [t+1, t+3] tidak menangkapnya (gerak mulai t + 15 s).
   Perbaikan: (a) `Popen` hanya di bawah kunci cv sesudah cek stop ("TIDAK diluncurkan: AUTO-STOP"), stop_all mengambil daftar
   proses di bawah kunci yang sama; (b) penyaringan reservasi di proses pekerja `spawn` (`reserve.screen_job`); (c) gerak pasca-stop
   diukur dari t_stop + 1 s sampai 2 s sesudah worker TERAKHIR keluar. Juga: rc keluar via `os._exit` (sebelumnya 134 dari thread
   spin = rc 0/3 hilang).

### B3. DRY + smoke `--move` (fake HW, BUKAN data HW)

DRY 5/5 rc 0 (K0 LULUS, tidak ada traverse di luar amplop). Smoke sesudah perbaikan B2:

| run | makespan mock | sim P1″ | rasio | tugas | RESERVE-BLOCKED | dua gantry bersamaan | jarak reservasi min | serial HW G51 |
|---|---|---|---|---|---|---|---|---|
| s7 R180 | **380.16** | 396.91 | 0.958 | 6/6 | 0 | 172.1 s | 150.5 mm | 535.05 |
| s7 R35 | **464.20** | 470.33 | 0.987 | 6/6 | 0 | 70.2 s | 104.8 mm | 523.25 |
| s10 R180 | **453.05** | 413.83 | 1.095 | 6/6 | 0 | 165.3 s | 113.2 mm | 582.23 |
| s10 R35 | **393.57** | 360.31 | 1.092 | 6/6 | 0 | 100.0 s | 317.6 mm | 468.37 |
| s0 R90 | **337.66** | 281.06 | **1.201** | 6/6 | 0 | 166.9 s | 201.1 mm | — |

- Semua 18/18 atau 16/16 event rc 0, urutan event per gantry = jadwal 5/5, 0 HALTED, 0 AUTO-STOP, semua tugas `['PLANNED']`
  percobaan pertama, pulang retrace + K0 LULUS 5/5 → **D461 ✓** (sesudah perbaikan B2; percobaan 1 s7 R180 gagal, B2).
- 0 RESERVE-BLOCKED (sim 0) → **D462 ✓**; jarak min rencana nyata vs reservasi gantry lain 104.8–317.6 mm.
- Rasio mock/sim 0.958–1.201 → **D463 ✗** (s0 1.201 di luar [0.8, 1.2] sebesar 0.001). Sebab: tugas mock 31–39 s vs P1″ 29.0 s
  (penyaringan reservasi di bawah flock + perencanaan `move_action` terserialkan + CPU bersama dua probe); s0 rata-rata 38.9 s.
- **Δ(R180 − R35) paralel:** s7 **−84.04 s** (sim −73.42), s10 **+59.48 s** (sim +53.52) → tanda sama → **D464 ✓**. Runner paralel
  MEMBUKA keuntungan R180 di s7 (serial HW: +11.8 s) tetapi s10 R180 tetap lebih lambat: lebih banyak traverse yang di sini
  EKSKLUSIF (A0) dan jadwal dioptimalkan untuk P1′-par tanpa eksklusivitas.

### B4. Kontrol negatif — [run](results/p1_g54/smoke/neg2_s7r180_run.json), [log](results/p1_g54/g54_smoke_neg2_s7r180.log)

- Percobaan 1 (`--inject-stop 2:4`, arsip `smoke/neg_try1/`) **TIDAK SAH**: g1 sudah lolos gerbang traverse tetapi alatnya
  "TIDAK diluncurkan: AUTO-STOP" → tidak ada yang bergerak saat stop (sekaligus bukti perbaikan B2(a)). Titik baru dipilih dari
  linimasa smoke s7 R180: g1 ke-6 (retract) siap saat retract g2 sudah 30.8 s berjalan, sisa 12.8 s.
- Percobaan 2 `--inject-stop 1:6`: stop saat **g2 sedang retract** (arm_3 bergerak ~6°/s menuju REST, dari 59.2° di t − 4 s ke
  35.21° di t_stop), lalu **35.21° konstan** dari t_stop + 0.5 s sampai + 4 s (perekam /joint_states, [jendela ±10 s](results/p1_g54/g54_neg2_joint_states_window.csv.gz)); `post_stop_motion` {1: 0.0,
  2: 0.0}, rc 3, pulang + K0 LULUS → **D465 ✓**. CancelGoal-semua menghentikan gerak gantry LAIN dalam ≤ 0.5 s (mock).

### B5. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D460 | move_group eksekusi bersamaan tanpa ubah jalur | ✗ serial; jalur FJT langsung lulus |
| D461 | smoke 5/5 | ✓ (sesudah perbaikan B2) |
| D462 | 0 RESERVE-BLOCKED | ✓ |
| D463 | mock/sim ∈ [0.8, 1.2] 5/5 | ✗ s0 1.201 |
| D464 | tanda Δ(R180 − R35) = sim | ✓ s7 −84.0, s10 +59.5 |
| D465 | kontrol negatif lulus | ✓ (percobaan 2; percobaan 1 tidak sah) |

**Tally G54: 4/6. Gerbang 7c: LULUS** (A2.1 dengan jalur ganti sesuai A4, DRY 5/5, smoke 5/5, kontrol negatif sah + lulus).
Runbook HW paralel: [docs/p1_hw_runbook_parallel.md](p1_hw_runbook_parallel.md).
