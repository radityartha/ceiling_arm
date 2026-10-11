# P1 / G53 — Fase 7b: makespan HW G51 vs P1′-serial (OFFLINE)

> Sesi 2026-10-11 malam. Kampanye FASE 7b. NOL gerak HW, tanpa mock. §A dikunci (salinan
> `docs/results/p1_g53/g53_A_locked.md` + sha256). §B menang atas §A.

## A. Protokol — DIKUNCI

### A0. Pengungkapan (⚠ pelanggaran urutan)

- **Pass pertama `results/p1_g53/g53_p1p.py` jalan SEBELUM §A ini ditulis** (total + jumlah per komponen sudah TERLIHAT).
  Karena itu tidak ada dugaan tentang rasio total atau jumlah komponen; dugaan A3 hanya untuk uji yang BELUM dihitung.
  Yang sudah terlihat: rasio terukur / P1′-serial 0.955–1.026 (5 run); tugas terukur 167–182 s vs model 261.9 (6 × c_task);
  traverse (layar + gerak) 138–220 vs model 95–153; gerak saja ≈ 0.90–0.93 × model traverse; retract-akhir ≈ 44 s per run
  tidak ada di model; jeda antar-event < 1 s. K (Σ model per event = `serial` solver) 5/5.
- Model = G25 (`g25_constants.json` apa adanya) + ROT_CMD 0.9235 (G31) = biaya yang dipakai solver G43/G49.
  makespan run_g32 = t_success tugas terakhir − t0 (retract akhir gantry terakhir tidak termasuk).

### A1. 🔒 Uji baru

1. **P1″-serial (rekalibrasi, leave-one-run-out):** biaya per event = rata-rata terukur 4 run lain: tugas a_t (per tugas),
   traverse = l̄ (layar per traverse) + gerak (rasio terukur/model traverse G25 × model), retract a_r (per retract tak
   dilewati, TERMASUK retract-akhir = satu per run serial). Prediksi run ke-5; galat |pred − terukur| / terukur.
2. **Δ(R180 − R35) sama seed** (s7, s10): terukur vs P1′-serial vs P1″-serial.
3. Rotasi: traverse yang didominasi rotasi (ROT_CMD·T_rot > r·T_lin) — gerak terukur / model per event.

### A2. 🔒 Gerbang 7b

Tabel per seed × varian (terukur, P1′-serial, rasio, dekomposisi) + penjelasan setiap selisih > 15 % (total ATAU komponen).

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D450 | P1″-serial LOO: \|galat\| ≤ **5 %** untuk 5/5 run. |
| D451 | Tanda Δ(R180 − R35) terukur = tanda P1′-serial untuk s7 dan s10 (runner serial tidak memakai keuntungan paralel). |
| D452 | Traverse dominan-rotasi: gerak terukur / model ∈ **[0.85, 1.10]** median. |

## B. Hasil terukur

§A sha256 `a47cd5fe…` ([salinan](results/p1_g53/g53_A_locked.md)). Skrip [g53_p1p.py](results/p1_g53/g53_p1p.py), log
[g53_p1p.log](results/p1_g53/g53_p1p.log), JSON [g53_p1p.json](results/p1_g53/g53_p1p.json) + [g53_tests.json](results/p1_g53/g53_tests.json).

### B1. Tabel per seed × varian (runner SERIAL, makespan = t_success terakhir − t0)

K: Σ model per event = `serial` solver **5/5** (selisih < 0.05 s).

| run | terukur | P1′-serial | rasio | P1′-par | tugas terukur / model | traverse (layar + gerak) / model | retract / model | retract-akhir | jeda |
|---|---|---|---|---|---|---|---|---|---|
| s7 R180 | 535.05 | 549.18 | 0.974 | 275.94 | 169.1 / 261.9 (6) | 60.7 + 124.7 / 134.7 (5) | 136.5 / 152.6 (3) | 43.8 / 0 | 0.3 |
| s7 R35 (G42) | 523.25 | 548.14 | 0.955 | 435.00 | 166.9 / 261.9 (6) | 51.2 + 124.2 / 133.6 (5) | 136.5 / 152.6 (3) | 43.9 / 0 | 0.5 |
| s10 R180 | 582.23 | 567.52 | 1.026 | 323.45 | 181.1 / 261.9 (6) | 77.1 + 142.5 / 153.0 (5) | 136.3 / 152.6 (3) | 44.4 / 0 | 0.8 |
| s10 R35 (G42) | 468.37 | 472.32 | 0.992 | 367.09 | 182.0 / 261.9 (6) | 49.3 + 99.6 / 108.7 (4) | 94.2 / 101.7 (2) | 43.2 / 0 | 0.1 |
| s0 R180 | 447.34 | 458.60 | 0.975 | 246.19 | 172.2 / 261.9 (6) | 51.8 + 85.8 / 95.0 (4) | 92.8 / 101.7 (2) | 44.4 / 0 | 0.3 |

**Total: semua run dalam ±5 % P1′-serial** (bias −4.5 … +2.6 %; G26: −0.4 … −11.9 %). Tidak ada selisih total > 15 %.

### B2. Selisih komponen > 15 % — penjelasan (Gerbang 7b)

Total cocok karena tiga galat komponen SALING MENUTUP:
1. **Tugas −31 … −36 %** (29.0 s/tugas terukur vs c_task 43.65). c_task dikalibrasi G25 pada run G24b, ketika "plan" 34 s
   didominasi penyaring antar-lengan (G25). Sejak itu probe berubah (penyaring G35/G36 B&B, tujuan SENDI G41, runner G42).
   **Sebab pasti TIDAK diukur di G53** (dekomposisi di dalam event tugas tidak dilakukan) — dugaan untuk G54/7c bila perlu.
2. **Traverse +37 … +44 %** (layar + gerak vs model). (a) **Layar** 7–19 s per traverse (rata-rata 12.6 s): waktu
   `t_start → t_send` di `pose_to_g` (penyaring sapuan S12/S23″/G33/S28 sebelum perintah) — tidak ada di model G25. (b) Gerak sendiri =
   0.92 × model (c_rovh 4.74 s G25 sebagian adalah overhead yang kini terukur sebagai layar).
3. **Retract-akhir +43–44 s per run** (bukan di model): runner serial meretract gantry pertama sebelum gantry kedua mulai;
   solver menghitung finish gantry = tugas terakhir. Retract biasa −7 … −11 % (45.3 vs c_ret 50.87).

### B3. Uji A1

- **P1″-serial LOO** (biaya per event = rata-rata 4 run lain): galat **+1.88, +4.57, −4.84, −1.95, +0.30 %** → maks 4.84 % →
  **D450 ✓**. Kalibrasi 5 run: tugas **29.04 s**, layar **12.61 s/traverse**, gerak **0.923 × model G25**, retract
  **45.34 s** (termasuk retract-akhir).
- **Δ(R180 − R35) serial:** s7 terukur **+11.80** (P1′-serial +1.04, P1″ +0.96, P1′-par **−159.06**); s10 terukur **+113.86**
  (P1′-serial **+95.20**, P1″ +98.86, P1′-par −43.65). Tanda sama → **D451 ✓**. Model serial SUDAH meramal R180 lebih lambat
  di runner serial: solver meminimalkan makespan PARALEL; jadwal R180 s10 memakai 5 traverse + 3 retract (R35: 4 + 2) karena
  pekerjaan dipindah ke gantry yang menganggur. Keuntungan R180 (−159 / −44 s P1′-par) hanya terukur dengan **runner paralel**
  (7c). Sisa selisih s7 (+10.8 s terukur vs model) = layar traverse (R180 60.7 vs R35 51.2 s) + tugas.
- **Traverse dominan-rotasi:** n = 4, gerak / model = 0.670, 0.666, 0.715, 0.888 → median **0.692** → **D452 ✗**. Kaki
  rotasi pendek (10–25°, T_cmd 3–4.4 s) memakan 5.0–6.3 s, model 7.5–8.8 (c_rovh 4.74 + 0.9235·T_cmd); kaki panjang
  −30 → −125° 0.888 (t_rot 15.3 vs T_cmd_rot 16.6 = 0.92). Model rotasi G31 benar untuk waktu putar (t_rot ≈ T_cmd, G30), tetapi
  c_rovh terlalu besar bila layar dipisah — sama dengan B2.2(b).

### B4. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D450 | P1″ LOO \|galat\| ≤ 5 % 5/5 | ✓ maks 4.84 % |
| D451 | tanda Δ(R180 − R35) terukur = P1′-serial | ✓ (+11.8/+1.0; +113.9/+95.2) |
| D452 | dominan-rotasi gerak/model median ∈ [0.85, 1.10] | ✗ 0.692 (c_rovh) |

**Tally G53: 2/3. Gerbang 7b: LULUS** (tabel B1 + penjelasan B2 untuk setiap komponen > 15 %). Implikasi untuk 7c: model
paralel harus memakai biaya P1″ (tugas 29.0, layar 12.6/traverse, gerak 0.923×, retract 45.3) dan tidak ada retract-akhir per
gantry di runner paralel (kedua blok jalan bersamaan); keuntungan R180 yang diramal P1′-par = −159 s (s7), −44 s (s10).
