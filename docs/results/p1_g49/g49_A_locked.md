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
