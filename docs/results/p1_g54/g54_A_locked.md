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

