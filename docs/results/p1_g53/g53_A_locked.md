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

