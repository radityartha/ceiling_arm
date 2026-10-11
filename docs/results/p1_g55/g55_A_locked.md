## A. Protokol — DIKUNCI

### A0. Pengungkapan

- G54: di runner paralel (traverse eksklusif) s7 R180 menang (−84 s mock) tetapi s10 R180 KALAH (+59.5 s). Jadwal G43/G49/G52
  dioptimalkan dengan biaya G25 (tugas 43.65 s) dan TANPA traverse eksklusif. G53: biaya terukur P1″ = tugas 29.04, layar 12.61
  s/traverse, gerak 0.923 × model G25, retract 45.34.
- Operator 2026-10-11: rotasi HW dibatasi ±90 (R90). Kelas pembanding: R90 dan R35.
- Solver tidak memodelkan traverse eksklusif (penggandeng waktu antar-gantry); hanya g54_sim yang memodelkannya.

### A1. 🔒 Metode

1. `results/p1_g55/g55_make.py`: solver G49 (ok49, G46, cermin, henti-ε 3 %) dengan biaya P1″ dipasang lewat `p1prime` /
   `ROT_CMD` (A0); cache = salinan G52. RSET R90 untuk s0 s3 s7 s8 s10 s11 s12 s18; R35 untuk s0 s3 s7 s8 s10 s18.
2. **K:** pembungkus TANPA tambalan biaya, R90 s12 → identik G52 s12 (241.58).
3. **Seleksi** `g55_eval.py`: per seed × kelas, skor = makespan paralel g54_sim (biaya P1″, reservasi garis lurus, traverse
   eksklusif + prioritas). Kandidat kelas R90 = jadwal lama siap-HW (G43 R180 bila |rot| ≤ 90 / G49 / G52 R90) + jadwal P1″;
   kelas R35 = G42f + P1″ R35. Terpilih = skor terkecil (seri → jadwal lama, sudah teruji).
4. Jadwal terpilih yang BARU: (iv′) + (iv″), lalu DRY-CLI mock k = 3 (pola G52) dan smoke paralel `run_g54` + pulang + K0.

### A2. 🔒 Gerbang G55

Tabel seleksi 8 seed; setiap jadwal terpilih baru lolos (iv′), (iv″), DRY-CLI 3/3, smoke 6/6; daftar pasangan HW besok
(R90 vs R35 sama seed) dengan prediksi sim.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D470 | K identik (s12 R90 241.58). |
| D471 | Jadwal P1″ R90 menang seleksi (sim lebih kecil dari jadwal lama) di ≥ 4/8 seed. |
| D472 | Sesudah seleksi, sim R90 < sim R35 untuk s7 DAN s10. |
| D473 | Jadwal baru terpilih lolos (iv′) ∧ (iv″) 100 %. |
| D474 | DRY-CLI 3/3 semua jadwal baru terpilih. |
| D475 | Smoke paralel semua 6/6; mock/sim ∈ [0.8, 1.25]. |

