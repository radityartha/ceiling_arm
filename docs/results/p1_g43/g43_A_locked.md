# P1 / G43 — penjadwal rotasi maksimum R180 (Fase 1 kampanye, OFFLINE)

> Sesi 2026-10-10, kampanye `docs/p1_campaign_prompt.md` Fase 1 (keadaan: `docs/p1_campaign_state.md`). NOL gerak HW,
> tanpa mock. Dasar = G42f (solver `g42_make.solve_exact_f` + `FIX_CHOICES` + (s42) + (c41b) margin kotak 10 mm, R35):
> (i)–(iii) 6/20 (seed 0 3 7 8 10 18). Keputusan operator 2026-10-10: rotasi TIDAK dibatasi ±10°, maksimalkan.
> **§A ditulis dan DIKUNCI SEBELUM pengukuran** (salinan `docs/results/p1_g43/g43_A_locked.md` + sha256). §B menang
> atas §A dan DITULIS. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

- Kode: `g42_make` (solver, (s42), diag, iv), `g41_make` (Feas41, `_tuple_job`, `screen_line`, `pose_ok_b`),
  `g40_make` (`build`, `_pose_job`, `_edge_job`, `neighbour_edges`, Feas2 — torsi `exact` = cache, tak diketahui = lazy),
  `g31_sched.neighbours/used_unverified`, `sched.traverse_time`, `sched_coll` (jejak bar + pelat, rumus eksak untuk rotasi
  apa pun), `g29_rot_screen.rect_points/sweep_rot` (persegi lin × rot 10 mm × 1°, tanpa wrap), bridge
  `dual_table_controller` (goto absolut 10 °/s, debounce, `bridge.max_deg` 180), `pose_to_g.py` (T_ROT = rumus model;
  gerbang S23″ |rot| ≤ 10 — urusan Fase 4/5).
- Hitungan eksplorasi (tanpa jadwal): grid rotasi = 72 (−180…+175, 5°). Tabel pose (a) G41b per rotasi: simetris 180°
  (g1 rot 0 = 33/33 lin, ±90 = 17; g2 rot 0 = 31, −70 / +110 = 11). Tuple L1 110 node 20 seed: R35 46 843 (pose-ok
  36 114), R180 198 852 (pose-ok 119 822). Cache G42: 1 201 tuple (CPU median 11.5 s, p90 24.1 s), 7 456 tepi.
- **Simetri balik (dihitung):** `base_pose(arm_a, lin, θ)` = `base_pose(arm_b, lin, θ−180°)` persis (rotasi + posisi,
  kedua gantry); OFFSET/LIM torsi sama untuk semua lengan. ⇒ konfigurasi (θ, slot s) secara fisik = (θ ∓ 180, slot 1−s):
  kelas fisik berbeda = θ mod 180. R180 menambah geometri baru hanya di |θ| ∈ (35, 145); sisi |θ| ≥ 145 hanya memberi
  representan yang lebih DEKAT untuk traverse.
- Waktu: satu iterasi build + `solve_exact_f` R180 (2 343 pose/gantry) seed 0 = 0.1 + 2.5 s (hanya waktu dicetak).

### A1. 🔒 RSET R180

- **R180 = semua rotasi grid dengan |rot| < 180° = 71 rotasi (−175…+175).** −180 DIKELUARKAN: tepat di batas sendi URDF
  (±π) dan `bridge.max_deg` — tujuan di batas + toleransi sendi/lacak bisa keluar batas; MARGIN = TOLAK. (Prompt menyebut
  72; −180 ≡ 0 dengan lengan bertukar, jadi tidak ada kelas fisik yang hilang.)
- Kode `docs/results/p1_g43/g43_make.py` = pembungkus `g42_make`: `FIX_CHOICES = True`, `g40_make.RSET ← R180` (dibaca
  saat panggilan oleh `build` dan `neighbour_edges`), cache tuple/tepi/stop = SALINAN cache G42 (`g43_*_cache.jsonl`),
  baris baru ditambahkan; berkas G42 tidak ditulis. Pool 15 (tidak ada mock). Seed = 20 seed G38 (node dari
  `g42f_candidates.json`), p0 = (1.50, 0) / (0.00, 0).
- Predikat TIDAK diubah: (a) tabel pose G41b, (b) tepi persegi `rect_points` vs peta, torsi tepat (oracle‴ `torq2`),
  (c41b) cabang + garis (margin kotak 10 mm), (s42), (iii) `sched_coll`, pasca-jadwal (iv′) `RotCrossChecker.sweep_rot`
  dan (iv″) `G41.chain`. Model waktu traverse = `sched.traverse_time` dengan `rot_cmd` 0.9235 (Δrot TANPA wrap = bridge).

### A2. 🔒 Uji

- **K-R35:** pembungkus dengan RSET = R35 pada 20 seed = `g42f_candidates.json` (stop, penugasan, tujuan, makespan / alasan
  gagal identik).
- **Jadwal R180** 20 seed → `g43_candidates.json` (kriteria (i)–(iii) G38 A4). Per seed: lolos, makespan, Δ vs G42f,
  |rot| maks, Δrot maks per traverse (dari p0 rot 0), jumlah pindah. Lalu (iv′) + (iv″) untuk tiap seed lolos.
- **K-SYM** (torsi + (c41b) di rotasi besar): 40 tuple cache (20 ok41, 20 tolak; |rot| ∈ [5, 35], acak seed 43) dibalik
  ke (θ ∓ 180, 1−s) dan dihitung ulang dengan `G41._tuple_job` (berkas terpisah `g43_ksym.jsonl`): verdict `ok3`, `ok41`
  harus sama.
- **Kontrol positif di |rot| > 35° (harus TOLAK)** + kontrol negatif (harus terima):
  - (a) pose: 2 kunci pose-tolak di |rot| ∈ [60, 120] dihitung ulang `_pose_job` → d_env < 50 mm atau kotak; 2 kunci
    pose-ok → ok.
  - (b) tepi: g di lin L dari rot 0 ke rot 90 di mana suatu (L, θ) antara ditolak (a) karena env → `_edge_job` bukan CLEAR.
  - (iii) `sched_coll.pair_distance` g1 (0.80, +90°) vs g2 (0.80, +90°) dan vs g2 (0.80, −90°) = 0 (BLOCK); g2 di
    (1.60, ±90°) > 0.
  - (iv′) `sweep_rot`: g1 di (0.80, +90°) lengan REST, g2 berputar (0.80, 0) → (0.80, +90°) = bukan CLEAR; g1 di (1.50, 0)
    dan g2 (0.00, 0 → 90°) = CLEAR.
- **Model waktu Δrot besar (A6, offline):** rumus T_ROT `pose_to_g` = `traverse_time` (sudah dibaca, sama); `rot_cmd`
  hanya terukur di 10° (G30) → dilaporkan berapa detik makespan terpilih bergantung pada Δrot > 10° (risiko; diukur di 6b).

### A3. 🔒 Diagnosa seed yang masih gagal

Bila ada seed gagal (i): diagnosa `g42_make.diag` (hierarki L1 → pose → torsi tepat → (c41b), strata pose ✗ ∧ torsi ✓
penuh) dengan RSET R180, cache diag G42 disalin. Torsi tak diketahui > 5 000 tuple → sampel acak 5 000 (seed 42), DITULIS.
Dilaporkan: apa yang berubah di |rot| > 35° (tuple torsi-ok per pita |rot|, pengikat).

### A4. 🔒 Hasil yang diserahkan

ROT_NEED = |rot| maks dan Δrot maks pada jadwal seed lolos (i)–(iv″); daftar seed kandidat HW (lolos (i)–(iv″)).
**Gerbang 1:** seed lolos R180 ≥ 6, keenam seed G42f tetap lolos dengan makespan ≤ G42f, predikat sama (K-R35 20/20).

### A5. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D350 | K-R35: **20/20** identik G42f. |
| D351 | R180 (i)–(iii): **≥ 12/20** seed lolos. |
| D352 | Keenam seed G42f lolos dengan makespan ≤ G42f; turun tegas pada **≥ 3/6**. |
| D353 | ROT_NEED \|rot\| maks ≤ **95°** (kelas fisik cukup di ±90; > 90 hanya untuk jalan pintas traverse). |
| D354 | Δrot maks per traverse ≤ **120°**. |
| D355 | K-SYM: verdict `ok3` dan `ok41` sama pada **≥ 38/40** pasangan. |
| D356 | Kontrol positif **6/6** TOLAK ((a) 2, (b) 1, (iii) 2, (iv′) 1); kontrol negatif **5/5** terima ((a) 2, (iii) 2, (iv′) 1). |
| D357 | (iv′) dan (iv″) lolos pada **≥ 90 %** seed lolos (i)–(iii). |
| D358 | Paling sedikit **1** seed tetap gagal (i) di R180. |
| D359 | Wall jadwal R180 20 seed ≤ **8 jam**. |
