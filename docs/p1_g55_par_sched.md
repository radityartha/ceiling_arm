# P1 / G55 — jadwal untuk runner PARALEL: solver dengan biaya P1″ + seleksi g54_sim (OFFLINE + mock)

> Sesi 2026-10-11 malam (operator: "HW baru besok, selesaikan semua offline dulu"). NOL gerak HW; mock 77 hanya untuk
> DRY-CLI/smoke. §A dikunci (salinan `docs/results/p1_g55/g55_A_locked.md` + sha256) SEBELUM mengukur. §B menang atas §A.
> MARGIN = TOLAK; predikat = G49 (ok49), tidak dilonggarkan.

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

## B. Hasil terukur

§A sha256 `3f48ae9e…` ([salinan](results/p1_g55/g55_A_locked.md)). Kode [g55_make.py](results/p1_g55/g55_make.py),
[g55_eval.py](results/p1_g55/g55_eval.py), [g55_iv.py](results/p1_g55/g55_iv.py); log [g55_run.log](results/p1_g55/g55_run.log),
[g55_eval.log](results/p1_g55/g55_eval.log).

### B1. K + solver P1″

- K (tanpa tambalan biaya) R90 s12 = 241.580, **identik** G52 → **D470 ✓**.
- Job 20:47–22:10 (restart sekali 20:47 untuk mendahulukan pasangan HW; tuple tercache, baris terakhir cache diperiksa utuh).
  **R90 8/8, R35 6/6 lolos (i)–(iii)**. P1″-par (model, tanpa traverse eksklusif): s0 121.17, s3 231.26, s7 237.20, s8 215.22,
  s10 207.85 (R90, |rot| 90) / 253.94 (R35), s11 191.62, s12 212.85 (= struktur jadwal G52), s18 207.91.
- **Dengan biaya terukur, optimum R90 hanya memakai |rot| ≤ 30° di 6/8 seed** (s8: 10°); R35 = jadwal yang sama untuk s0 s3 s7
  s8 s18. Hanya s10 (90°) dan s12 (40°) memakai rotasi > 35.

### B2. Seleksi g54_sim — [JSON](results/p1_g55/g55_eval.json), [terpilih](results/p1_g55/g55_selected.json)

**Penyimpangan dari A1.3 (§B menang):** kelas = BATAS rotasi, jadi kandidat kelas R90 juga memuat jadwal R35 (sah di ±90). Tanpa
itu s10 memilih R90-P1″ (sim 315.04) padahal R35-P1″ (278.50) sah dan lebih cepat — traverse eksklusif menghukum jadwal R90 yang
lebih banyak traverse; solver tidak melihatnya.

| seed | lama terbaik (sim) | baru (sim) | terpilih | Δ sim | \|rot\| terpilih |
|---|---|---|---|---|---|
| s0 | G52 R90 281.06 | R90-P1″ 148.16 | baru | **−132.90** | 30 |
| s3 | G43 R180 426.08 | R90-P1″ 335.75 | baru | −90.33 | 25 |
| s7 | G43 R180 396.91 | R90-P1″ 283.62 | baru | **−113.29** | 30 |
| s8 | G43 R180 402.11 | R90-P1″ 264.44 | baru | −137.67 | 10 |
| s10 | G42f 360.31 | R90-P1″ 315.04 / **R35-P1″ 278.50** | baru (R35) | **−81.81** | 30 |
| s11 | G52 R90 456.94 | R90-P1″ 257.52 | baru | −199.42 | 25 |
| s12 | G52 R90 313.49 | = G52 (struktur sama) | lama | 0 | 40 |
| s18 | G43 R180 372.16 | R90-P1″ 298.54 | baru | −73.62 | 30 |

- Jadwal P1″ R90 menang di 6/8 (s0 s3 s7 s8 s11 s18) → **D471 ✓**.
- **R90sel = R35sel di semua 6 seed yang punya keduanya** (|rot| terpilih ≤ 30) → **D472 ✗**: di runner paralel dengan biaya
  terukur, rotasi > 35° tidak memberi untung pada seed ini. Pasangan HW "R90 vs R35" tidak bermakna; yang bermakna = **jadwal lama
  (P1′) vs terpilih (P1″) seed sama**.
- **s11 terpilih |rot| ≤ 25** → tidak butuh kaki sweep g2 −85 (amplop G51 cukup).
- 0 blok reservasi di semua sim.

### B3. (iv′) + (iv″) — [iv](results/p1_g55/g55_iv.log), [chain](results/p1_g55/g55_chain.log)

**8/8 dan 8/8** → **D473 ✓**.

### B4. Mock (DRY-CLI + smoke paralel) — [log job](results/p1_g55/g55_mock.log), [DRY-CLI](results/p1_g55/g55_drycli.json), arsip `results/p1_g54/smoke/g55_s*`

Mock 77 baru (SIG_DFL, SigIgn `0x1001001`, 7/7, tanpa node ganda), monitor + perekam, REST + PARK, K0 LULUS; solver selesai lebih dulu
(tidak bersamaan Pool 15).
- **DRY-CLI k = 3: 7/7 seed baru 3/3** (s0 s3 s7 s8 s10 s11 s18; s12 = jadwal G52, sudah 3/3 di G52 B4). 126/126 tugas PLANNED
  percobaan pertama, 0 GOAL-MISMATCH / START-JUMP / START-UNSCREENED / NO-PLAN, ujung median 0.34 mm, maks 0.97 → **D474 ✓**.
- **Smoke paralel `run_g54`: 7/7 seed 6/6 SUKSES**, semua event rc 0, 0 RESERVE-BLOCKED, 0 AUTO-STOP, semua tugas PLANNED percobaan
  pertama, pulang retrace + K0 LULUS 7/7:

| seed | mock | sim | rasio | lama: mock G54 (sim) |
|---|---|---|---|---|
| s0 | **163.39** | 148.16 | 1.103 | 337.66 (281.06) |
| s3 | 376.18 | 335.75 | 1.120 | — (426.08) |
| s7 | **280.98** | 283.62 | 0.991 | 380.16 (396.91) |
| s8 | 267.44 | 264.44 | 1.011 | — (402.11) |
| s10 | **313.24** | 278.50 | 1.125 | 393.57 (360.31) |
| s11 | 298.47 | 257.52 | 1.159 | — (456.94) |
| s18 | 354.37 | 298.54 | 1.187 | — (372.16) |

  Rasio 0.991–1.187 ∈ [0.8, 1.25] → **D475 ✓**. Mock seed sama: s0 −174.3 s (−52 %), s7 −99.2 s (−26 %), s10 −80.3 s (−20 %) vs
  jadwal lama (mock G54, sesi mock lain).

### B5. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D470 | K identik | ✓ |
| D471 | P1″ R90 menang ≥ 4/8 | ✓ 6/8 |
| D472 | sim R90 < R35 untuk s7 dan s10 | ✗ sama (|rot| terbaik ≤ 30) |
| D473 | (iv′) ∧ (iv″) 100 % | ✓ 8/8 |
| D474 | DRY-CLI 3/3 jadwal baru | ✓ 7/7 |
| D475 | smoke 6/6, mock/sim ∈ [0.8, 1.25] | ✓ 7/7, 0.991–1.187 |

**Tally G55: 5/6. Gerbang G55: LULUS.** Pasangan HW: `docs/p1_hw_day_2026-10-12.md` §2 (lama vs terpilih).
