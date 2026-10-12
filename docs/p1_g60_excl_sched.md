# P1 / G60 — penjadwal sadar traverse EKSKLUSIF (riset OFFLINE + mock)

> Sesi 2026-10-12 (prompt `docs/p1_g58_offline_prompt.md`). NOL gerak HW. §A dikunci (salinan
> `docs/results/p1_g60/g60_A_locked.md` + sha256) SEBELUM job solver. §B menang atas §A. MARGIN = TOLAK; predikat = G49
> (ok49), cache = SALINAN (`p1_g60/g60_*_cache.jsonl` dari G55).

## A. Protokol — DIKUNCI

### A0. Pengungkapan (sebelum §A)

- G55: solver P1″ TIDAK memodelkan traverse eksklusif; hanya g54_sim. Seleksi g54_sim memilih |rot| ≤ 30 di semua seed (D472 ✗).
- Sebelum §A ditulis SUDAH dijalankan (alat + kontrol, bukan hasil fase):
  1. **K-SIM** `g60_sim.py k`: `fastsim` (g54_sim tanpa penyaringan reservasi) vs g54_sim pada SEMUA 19 jadwal yang pernah
     disimulasikan G54/G55 → **19/19 identik** (semua 0 blok reservasi). [log](results/p1_g60/g60_ksim.log)
  2. **K-DP** `g60_make.py k` (instans terverifikasi G55, tanpa menghitung tuple): (a) 747/747 rantai min-Tr dijalankan
     sendiri = Tr + W; (b) 0 pelanggaran: DP min-Tr ≤ Tr jadwal G55 (62 gantry), LB ≤ sim g54 di semua 31 baris g55_eval
     (sering LB = sim). [log](results/p1_g60/g60_kdp.log)
  3. **Coba search pada instans TERVERIFIKASI saja** (s7 s10 s0, R90 + R35): optimum = jadwal G55 terpilih (283.62 / 278.50 /
     148.16, s10 g2 (1.00, −20) alih-alih (0.85, −20), makespan sama) dengan **LB*ver = makespan (celah 0)**. Relaksasi
     (tuple belum diketahui = layak): LB*rel s7 265.07 (R90) / 266.63 (R35), s10 275.04 (R90). Dugaan di bawah TIDAK
     menebak angka-angka ini.

### A1. 🔒 Pendekatan dan alasannya

**Model** = runner `run_g54` (`g60_sim.fastsim` ≡ g54_sim dengan 0 blok): traverse gantry g mulai hanya bila gantry lain tidak
menjalankan apa pun dan menahan gantry lain selama berjalan; tugas/retract tidak mulai selama gantry lain traverse; traverse yang
menunggu didahulukan. Biaya P1″ (G53 B3): tugas a_t 29.04, retract a_r 45.34 (2.7 bila lengan sudah REST), traverse
l_bar + g·(c_rovh + max(r·T_lin, ROT_CMD·T_rot)).

**Batas bawah terbukti** (untuk SETIAP pasangan urutan event per gantry): semua event gantry g sampai tugas terakhirnya dan
semua traverse gantry lain o saling lepas dalam waktu dan selesai sebelum makespan →
`makespan ≥ W_g + Tr_g + Tr_o` → **LB = Tr_1 + Tr_2 + max(W_1, W_2)**. W hanya bergantung pada (#tugas, #traverse m, stop
pertama di p0 f0); Tr diminimalkan per (himpunan tugas, m, f0) oleh DP = rekursi `solve_gantry_f` + satu indeks m (larik
kelayakan yang sama: reach ok49, larangan (s42), EdgeMatrix (b), tanpa dua stop berurutan di pose sama).

**Pencarian:** semua alokasi (A₁, A₂) × (m₁, f0₁) × (m₂, f0₂) dengan (ii) (m ≥ 1 kedua gantry) → LB + kandidat = rantai
min-Tr kunci itu; dievaluasi `fastsim` berurutan LB naik sampai LB ≥ incumbent; (iii) `schedule_conflict` pada linimasa fastsim
(awal stop = akhir traversenya) — kandidat dengan konflik dilewati. **Sertifikat** LB* = min LB atas SEMUA kunci (berlaku
untuk semua jadwal, bukan hanya kandidat). Pencarian ini EKSAK hanya sejauh rantai min-Tr per kunci adalah yang terbaik untuk
fastsim; celah makespan/LB* dilaporkan apa adanya (= 0 → optimum terbukti).

**Lazy (G46):** jadwal = instans TERVERIFIKASI (+ loop (s42) seperti UB G46). LB* = instans RELAKSASI. Setiap kunci relaksasi
dengan LB < makespan_ver/(1 + 0.03) yang rantainya memakai tuple/stop belum terverifikasi → diverifikasi (+ tetangga G31),
≤ 300 kunci per putaran; berhenti bila tidak ada lagi. Celah akhir = makespan / LB*rel − 1.

**Mengapa ini (paling sederhana yang benar):** penalti saja (mis. LB sebagai objektif) tidak menjamin urutan yang baik untuk
gerbang runner; solver eksak atas penjadwalan waktu-kontinu dua mesin dengan eksklusi parsial butuh B&B atas pasangan urutan
(ruang pose × urutan ≈ 10⁶+ per gantry). DP min-Tr per (A, m, f0) memakai kembali solver G42–G55 tanpa mengubah kelayakan,
LB-nya terbukti, dan evaluasi kandidat = model runner itu sendiri. Kontrol (A2.1) memastikan model = g54_sim.

### A2. 🔒 Uji

1. **K-SIM** (A0.1, sudah) + **K-ULANG:** `fastsim` jadwal tersimpan (lewat `g31_screen.events`) = nilai pencarian (per seed).
2. **Job** `g60_make.py run R90 / R35` seed 0 3 7 8 10 11 12 18 (R35: 0 3 7 8 10 18, seperti G55; s11/s12 juga dicoba R35 —
   G55 tidak). Setiap jadwal: g54_sim PENUH (dengan penyaringan reservasi) = makespan G60 (identik bila 0 blok).
3. Pertanyaan utama: seed mana jadwal R90 dengan |rot| > 35 MENANG atas R35 (makespan eksklusif lebih kecil). Bila ada:
   (iv′) + (iv″) (pola `g55_iv.py`), DRY-CLI mock k = 3 (tol 0.05°), smoke paralel mock `run_g54` → pasangan HW R90 vs R35.
   Bila tidak ada: ditulis sebagai hasil.
4. Jadwal G60 yang lebih baik dari G55 terpilih (> 1 %) → jalur yang sama (iv′)(iv″) → DRY-CLI → smoke, sebagai kandidat
   pengganti R90sel/R35sel.

### A3. 🔒 Gerbang G60

K-SIM 19/19 + K-ULANG semua; tabel 8 seed × {R90, R35}: makespan G60, LB*rel, celah, |rot| maks, vs G55 terpilih; jawaban
pertanyaan A2.3; jadwal baru (bila ada) lolos (iv′), (iv″), DRY-CLI 3/3, smoke 6/6.

### A4. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D520 | Makespan G60 ≤ g54_sim jadwal G55 terpilih di 8/8 seed (R90). |
| D521 | Celah akhir (makespan / LB*rel − 1) ≤ 3 % di ≥ 6/8 seed R90. |
| D522 | TIDAK ada seed di mana jadwal R90 dengan \|rot\| > 35 menang atas R35 lebih dari 1 %. |
| D523 | G60 lebih baik dari G55 terpilih > 1 % di ≥ 2/8 seed. |
| D524 | g54_sim penuh = makespan G60 (0 blok reservasi) di semua jadwal G60. |
| D525 | Jadwal baru yang diuji di mock: (iv′) ∧ (iv″) 100 %, DRY-CLI 3/3, smoke 6/6. |

## B. Hasil terukur

§A sha256 `7af94f7b…` ([salinan](results/p1_g60/g60_A_locked.md)). Kode [g60_sim.py](results/p1_g60/g60_sim.py),
[g60_make.py](results/p1_g60/g60_make.py), [g60b_make.py](results/p1_g60/g60b_make.py), [g60_eval.py](results/p1_g60/g60_eval.py),
[g60_iv.py](results/p1_g60/g60_iv.py); log `g60_job.log`, `g60b_job.log`, `g60c_job.log`, `g60{,b,c}_eval.log`. Cache = salinan G55
(sha awal sama dengan G55), +1155 tuple / +16 stop dihitung. Job Pool 15 hanya saat mock MATI (08:31–09:25).

### B1. Tiga putaran (§B menang atas §A1)

1. **G60 (= §A1)**, 08:31–08:56: keluarga kandidat = satu rantai min-Tr per (A, #traverse m, f0).
2. **G60b** (penyimpangan dari §A1, ditulis): celah 5–11 % di s3/s8/s12/s18 berasal dari keluarga kandidat, bukan dari LB.
   Keluarga diperluas ke satu rantai min-Tr per (A, UKURAN stop berurutan (|U₁|, …, |U_k|), f0); LB* tidak berubah (min atas
   ukuran = min per m; **K-B** 2304/2304 + rantai 164/164, [log](results/p1_g60/g60b_k.log)). Plus DP kedua yang hanya menerima
   urutan dengan ≥ 1 stop di |rot| > 35° → **LB_lebar** = batas bawah untuk SETIAP jadwal yang memakai rotasi lebar (relaksasi,
   tanpa verifikasi tuple → berlaku juga untuk tuple yang belum diketahui).
3. **G60c** (cacat G60b ditemukan oleh A2.2): s12 G60b 270.36 di fastsim tetapi **g54_sim PENUH 373.78 (3 blok reservasi)** —
   jadwal itu menaruh g1 (1.50–1.55) dan g2 (1.00–1.25) berdekatan sehingga lengan saling menunggu; fastsim (K-SIM: = g54_sim
   HANYA bila 0 blok) tidak melihatnya → **D524 ✗**. Perbaikan: kandidat yang akan menjadi incumbent dinilai dengan g54_sim
   PENUH (LB tetap sah: blok hanya menambah tunggu, disjointness tidak berubah). s12: 4 dinilai penuh, 2 terhalang → 270.70.

### B2. Tabel (G60c final; R90 dan R35 IDENTIK di 8/8 seed — semua jadwal |rot| ≤ 30)

| seed | G55 terpilih (sim) | G60 | G60b | **G60c** | LB*rel | celah | \|rot\| | LB_lebar (> 35°) | lebar terbaik ditemukan |
|---|---|---|---|---|---|---|---|---|---|
| s0 | 148.16 | 148.16 | 148.16 | **148.16** (=) | 148.16 | 0.00 % | 30 | 150.60 ≥ ✓ | 150.60 |
| s3 | 335.75 | 303.69 | 295.80 | **295.80 (−11.9 %)** | 277.35 | 6.65 % | 30 | 283.30 < | 300.29 |
| s7 | 283.62 | 283.62 | 283.62 | **283.62** (=) | 273.94 | 3.53 % | 30 | 283.81 ≥ ✓ | 283.81 |
| s8 | 264.44 | 264.44 | 264.44 | **264.44** (=) | 251.69 | 5.07 % | 10 | 273.81 ≥ ✓ | 313.06 |
| s10 | 278.50 | 278.50 | 278.50 | **278.50** (=) | 275.04 (R35 278.50) | 1.26 % | 30 | 275.04 < (≤ 1.24 %) | 281.07 |
| s11 | 257.52 | 257.52 | 257.52 | **257.52** (=) | 252.71 | 1.90 % | 25 | 257.45 < (≤ 0.03 %) | 264.22 |
| s12 | 313.49 (G52) | 299.40 | 270.36 ✗ (penuh 373.78) | **270.70 (−13.6 %)** | 270.36 | 0.13 % | 20 | 273.75 ≥ ✓ | 275.91 |
| s18 | 298.54 | 244.95 | 232.21 | **232.21 (−22.2 %)** | 232.21 | 0.00 % | 10 | 233.28 ≥ ✓ | 262.33 |

- **K-SIM 19/19** (A0.1), **K-ULANG** 48/48 (G60, G60b, G60c × R90, R35) (fastsim jadwal tersimpan lewat `g31_screen.events` = nilai pencarian, kolom
  `makespan_json`), **g54_sim penuh = G60c 16/16, 0 blok** ([log](results/p1_g60/g60c_eval.log)).
- G60c ≤ G55 terpilih di 8/8 → **D520 ✓**. Lebih baik > 1 % di s3, s12, s18 → **D523 ✓** (3/8). Celah ≤ 3 % di 5/8 (s0 s10 s11
  s12 s18) → **D521 ✗**; s3 6.65 %, s8 5.07 %, s7 3.53 % = sisa heuristik keluarga (LB tidak memodelkan tunggu gerbang).
- **Pertanyaan A2.3 — jawaban: TIDAK ADA seed di mana rotasi > 35° menang.** Terbukti (LB_lebar ≥ makespan R35) di 5/8 (s0 s7 s8
  s12 s18); s11 paling banyak 0.03 %, s10 paling banyak 1.24 % (batas, tidak tercapai: lebar terbaik 281.07 > 278.50), s3 paling
  banyak 4.2 % (lebar terbaik 300.29 > 295.80). Di model runner eksklusif + biaya P1″, rotasi lebar tidak membantu: traverse
  rotasi besar lama (10 °/s, eksklusif → menahan gantry lain) dan tuple torsi-ok tetap ada di |rot| ≤ 30. **D522 ✓** (terukur
  8/8; tersertifikasi ≤ 1 % di 6/8). → **Tidak ada pasangan HW R90 vs R35 yang bermakna** (tidak dipaksakan).
- Jadwal baru untuk HW: **s3, s12, s18 varian `G60`** (`results/p1_g60/g60_selected.json`); s0 s7 s8 s10 s11 = G55 terpilih
  (identik). (iv′) 3/3, (iv″) 3/3 ([iv](results/p1_g60/g60_iv.log), [chain](results/p1_g60/g60_chain.log)); di dalam amplop HW
  (g1 −30…+10, g2 −20…0).

### B3. Mock (DRY-CLI + smoke paralel) — [log](results/p1_g60/g60_mock.log), [DRY-CLI](results/p1_g60/g60_drycli.json), arsip `results/p1_g54/smoke/g60_s*`

Mock 77 baru 09:50 (SIG_DFL, SigIgn `0x1001005` — bit SIGINT 0x2 tidak ada; 0x4 dari subshell peluncur), 7/7, 0 node ganda,
monitor + perekam, REST + PARK, K0 LULUS; job solver sudah selesai (tidak bersamaan Pool 15). Fake HW, BUKAN data HW.
- **DRY-CLI k = 3 (tol 0.05°): s3 s12 s18 3/3**, 54/54 tugas `['PLANNED']` percobaan pertama, 0 GOAL-MISMATCH / START-JUMP / NO-PLAN.
- **Smoke paralel `run_g54` 3/3 6/6 SUKSES**, semua event rc 0, 0 RESERVE-BLOCKED, 0 AUTO-STOP, pulang retrace + K0 LULUS:

| seed | mock G60 | sim G60c | rasio | mock jadwal pembanding (sesi lain, variansi ≤ 3 %) |
|---|---|---|---|---|
| s3 | **305.54** | 295.80 | 1.033 | G55 R90sel 374.37 (−68.8 s), lama G43 446.13 (−140.6 s) — G58 |
| s12 | **289.55** | 270.70 | 1.070 | G52 = G55 terpilih 332.32 (−42.8 s) — G58 |
| s18 | **257.50** | 232.21 | 1.109 | G55 R90sel 352.67 (−95.2 s), lama G43 431.06 (−173.6 s) — G58 |

→ **D525 ✓**. Mock mati bersih 10:29 (node 0, daemon 77 dihentikan, tanpa crash dump).

### B4. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D520 | G60 ≤ G55 terpilih 8/8 | ✓ (G60c, g54_sim penuh) |
| D521 | celah ≤ 3 % di ≥ 6/8 | ✗ 5/8 (s3 6.65, s8 5.07, s7 3.53 %) |
| D522 | rotasi > 35° tidak menang > 1 % | ✓ terukur 8/8; tersertifikasi 6/8 (s10 ≤ 1.24, s3 ≤ 4.2 % terbuka) |
| D523 | lebih baik > 1 % di ≥ 2/8 | ✓ 3/8 (s3 −11.9, s12 −13.6, s18 −22.2 %) |
| D524 | g54_sim penuh = makespan G60 | ✗ (G60b s12: 3 blok, 373.78 vs 270.36) → G60c 16/16 |
| D525 | (iv′)(iv″), DRY-CLI 3/3, smoke 6/6 | ✓ 3/3 |

**Tally G60: 4/6. Gerbang G60: LULUS** (K-SIM 19/19 + K-ULANG 48/48; tabel 8 × {R90, R35}; A2.3 dijawab: rotasi lebar tidak
membantu, TIDAK ada pasangan HW R90 vs R35; jadwal baru s3 s12 s18 lolos (iv′), (iv″), DRY-CLI 3/3, smoke 6/6).
Catatan metode: model LB tidak tahu reservasi lengan lintas-gantry → setiap pemakaian berikut WAJIB penilaian g54_sim penuh
(G60c), bukan fastsim saja.
