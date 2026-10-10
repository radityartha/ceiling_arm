# P1 / G49 — seed gagal: torsi per CABANG yang dipakai (∃), bukan ∀ cabang (OFFLINE)

> Sesi 2026-10-10 malam. Operator: "analisis yang gagal, lakukan perbaikan supaya berhasil; asalkan masih terjangkau harusnya
> berhasil". NOL gerak HW, tanpa mock. **§A DIKUNCI SEBELUM mengukur** (salinan `docs/results/p1_g49/g49_A_locked.md` +
> sha256). §B menang atas §A. MARGIN = TOLAK: batas torsi (LIM − OFFSET − MARGIN), margin peta 50 mm, kotak 10 mm, antar-lengan
> TIDAK dilonggarkan.

## A. Protokol — DIKUNCI

### A0. Pengungkapan

- G43 R180 (+ G46): lolos (i)–(iii) **11/19** (0 2 3 7 8 10 11 14 16 17 18), gagal **8** (1 4 5 6 9 12 13 15: enam "task k is
  infeasible on every gantry", dua "no feasible schedule" (s13, s15)); s19 tidak selesai (dihentikan).
- G42 B5 (R35): pengikat utama tugas tak layak = **torsi joint_2** (17/19 tugas punya tuple yang gagal HANYA torsi).
- Predikat torsi tuple = `torq2` oracle‴: **SETIAP** solusi IK (semua cabang, semua roll, envelope tilt ±2°) dalam batas
  (G23). Alasan G23: tujuan POSE membiarkan MoveIt memilih cabangnya sendiri. Sejak G41–G47 tujuan tugas = **SENDI dari satu
  cabang terpilih** (`--goal-joints`, toleransi 0.05°, GOAL-MISMATCH), dan probe tetap menyaring torsi rencana nyata
  (`TORQUE-UNSAFE`, RNEA + offset per aktuator) — syarat ∀ cabang sudah tidak melindungi apa pun yang dieksekusi.
- `oracle2.solve` kini merekam envelope torsi PER cabang (`solutions_tau`, gravity yang sama dengan `env`, tanpa biaya).

### A1. 🔒 Predikat G49 (tuple)

ok49 ⟺ ada cabang q6 (urutan = G41: maks |q − REST| naik) dengan:
(t) envelope torsi CABANG ITU (titik akhir + tilt ±2°) memenuhi ekspresi `torq2` (+ OFFSET + MARGIN ≤ LIM),
(e1) env ≥ 50 mm, kotak ≥ 10 mm di titik akhir, (e2/e3) `screen_line` (torsi garis, env, kotak, se-gantry) — persis G41b.
Tujuan = cabang pertama yang lolos. Tanpa syarat `saturated` (∃ tidak butuh kelengkapan; cabang yang tak ditemukan hanya
mengurangi pilihan). ok41 ⇒ ok49 (cabang pertama sama) → tuple G43 dengan ok3 = True dibawa apa adanya; ok3 = False
dihitung ulang. Cermin 180° (K-SYM) tetap. Tepi = EdgeMatrix G46, stop = (s42), henti-ε 3 %. Lain-lain = G43/G46.

### A2. 🔒 Uji

- **K-SUP:** 40 tuple G43 ok41 = True dihitung ulang dengan job G49 → ok49 = True dan tujuan sama 40/40.
- **Jadwal** seed 1 4 5 6 9 12 13 15 19 (R180) → `g49_candidates.json`; seed lolos → (iv′) + (iv″).
- **Diagnosa** seed yang masih gagal: pengikat per tugas (torsi-cabang / e1-env / e1-kotak / garis-*) dari `whys`.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D410 | K-SUP 40/40. |
| D411 | ≥ **5/9** seed gagal menjadi lolos (i)–(iii). |
| D412 | Pada seed yang tetap gagal, pengikat dominan = e1-env / garis-env (lengan pasangan REST × rak), bukan torsi-cabang. |
| D413 | Seed baru lolos (iv′) ∧ (iv″) ≥ 80 %. |

## B. Hasil terukur

### B1. K-SUP — [log](results/p1_g49/g49_ksup.log)

**40/40** tuple G43 ok41 → ok49 dengan tujuan q6 sama → D410 ✓.

### B2. Jadwal 9 seed — [run](results/p1_g49/g49_run.log), [JSON](results/p1_g49/g49_candidates.json), [(iv′)/(iv″)](results/p1_g49/g49_iv_chain.log)

22:56–01:51 (Pool 15, nice 19). **6/9 lolos (i)–(iii)**: s4 260.13 s (rot ≤ 30°), s5 201.66 (25°), s6 209.58 (10°),
s9 191.10 (20°), s12 233.22 (110°), s13 212.22 (20°); gap ≤ 1.1 %. (iv′) 6/6, (iv″) 6/6 → D411 ✓, D413 ✓.
Seed lolos G43 yang belum dicek: s2, s11, s14, s16, s17 → (iv′) 5/5, (iv″) 5/5.
Tuple G49 baru 876 (sampai s1): ok49 152, **150 di antaranya gagal ∀ (ok3) tetapi lolos ∃** — itulah yang dibuka predikat.

**Total P1 R180: 17/20 seed punya jadwal lolos (i)–(iv″)** (G42f R35: 6/20): G43 0 2 3 7 10 11 14 16 17 18 + s8 (jadwal G42f),
G49 4 5 6 9 12 13. Tetap gagal: **s1 (t3), s15 (t5), s19 (t3)**.

### B3. Diagnosa 3 tugas yang tetap tak layak — [log](results/p1_g49/g49_diag_quick.log)

Ketiganya DINDING rendah (z 1.08; s1 t3 dan s15 t5 di x 0.79, y −0.60/−0.53; s19 t3 x 1.29, y −0.53) dekat dinding sisi g2.
**Semua tuple pose-ok sudah dihitung (habis), tidak ada yang layak:**

| tugas | L1 | pose (a) gagal | 0 IK tegak | semua cabang torsi | titik akhir kotak/env | garis transit |
|---|---|---|---|---|---|---|
| s1 t3 | 1 752 | 740 | 494 | 58 | 258 | 202 |
| s15 t5 | 1 942 | 774 | 576 | 67 | 311 | 214 |
| s19 t3 | 1 543 | 948 | 320 | 39 | 222 | 14 |

Per cabang: torsi-cabang 77–178 rb, **e1-kotak 51–115 rb** (ujung/jari di dalam kotak peta MoveIt = voxel +5 cm per sisi
(ketidakpastian kamera) + margin 10 mm), garis-kotak/env ratusan. → D412 ✗ (pengikat bukan env lengan pasangan, tetapi
kotak peta di titik akhir + torsi + IK). Membukanya butuh keputusan di luar predikat keselamatan: (a) kalibrasi kamera lebih
baik → kotak peta +5 cm diperkecil, (b) pose simpan lain untuk lengan REST (740–948 tuple gugur di pose (a)), (c) tool boleh
miring (spesifikasi tugas). Tidak dilakukan tanpa operator.

### B4. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D410 | K-SUP 40/40 | ✓ |
| D411 | ≥ 5/9 seed lolos | ✓ 6/9 |
| D412 | sisa gagal terikat env lengan pasangan, bukan torsi | ✗ kotak peta di titik akhir + IK + torsi |
| D413 | seed baru (iv′) ∧ (iv″) ≥ 80 % | ✓ 6/6 (+ G43 5/5) |

**Tally G49: 3/4.** Kode: `oracle2.solve` `solutions_tau` (envelope per cabang; keluaran lama tidak berubah), `results/p1_g49/g49_make.py`.
Catatan HW: kandidat Fase 6 TETAP 6 jadwal yang sudah melewati rehearsal G47; jadwal G49 baru perlu DRY-CLI + smoke dulu.
