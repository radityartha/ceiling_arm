# P1 / G44 — penyaring START-JUMP (Fix-Start-State) (Fase 2 kampanye)

> Sesi 2026-10-10, kampanye `docs/p1_campaign_prompt.md` Fase 2 (= G37 A dari G36). NOL gerak HW; mock `ROS_DOMAIN_ID=77`
> hanya SESUDAH job berat G43 selesai (CPU). **§A ditulis dan DIKUNCI SEBELUM mengukur** (salinan
> `docs/results/p1_g44/g44_A_locked.md` + sha256). §B menang atas §A dan DITULIS. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

- G36 B1/B6.2: dari start "in collision" (jari +0.05 rad, titik akhir arsip ev 10 `g34_planonly_plans_g35b_hw_R10.jsonl`
  s0, +103.7°) adaptor `FixStartStateCollision` menggeser start 0.07–1.12 rad dan MoveIt menjawab SUCCESS; penyaring hanya
  memeriksa waypoint. G40 B: G38 s1 t2 PLANNED dengan start (g1 (1.55, −30°) REST) tabrak `env_static_map`, geser 0.17 m.
- Adaptor = default `moveit_configs_utils` (`AddTimeOptimalParameterization` terluar … `FixStartStateCollision`): start
  asli ditambahkan sebagai waypoint 0, jadi lompatan = segmen 0 → 1 yang lalu diparameterkan waktu.
- Arsip: plan-only jsonl memuat `start` + `traj {joint_names, t, pos}` (G31–G41; DRY-CLI G42 hanya `end`). Probe HW JSON
  (`g32hw_*_probe.json`) TIDAK menyimpan lintasan → lintasan yang benar-benar dieksekusi di HW tidak diarsip. Proksi
  terdekat = plan-only di **stack NYATA** sebelum run HW (`g34_planonly_plans_{g34b,g35b}_hw_*`,
  `g36_planonly_plans_g36b_hw_*`): move_group, URDF dan keadaan nyata yang sama, rencana dari distribusi yang sama.
- Distribusi |dq| BELUM dilihat. `vel_scale` 0.15 = plan-only = HW (`move_to` default).

### A1. 🔒 Ukuran (offline)

Per rencana PLANNED dalam arsip (kelompok **MOCK** = semua plan-only jsonl G31–G41 bukan `_hw_`; kelompok **NYATA** =
tiga arsip `_hw_` di atas): d01 = maks_j |p1 − p0| (rad, sendi lengan), d_in = maks_{k≥1} maks_j |p_{k+1} − p_k|, dan
|p0 − start| (sendi lengan terencana). Dilaporkan: kuantil p50/p99/p99.9/maks per kelompok, rencana dengan d01 > d_in.

### A2. 🔒 Batas (dari MOCK, divalidasi di NYATA)

- **T01 = 3 × p99.9(d01 MOCK)**, **T_in = 3 × p99.9(d_in MOCK)**, keduanya dibulatkan ke atas ke 0.001 rad.
- Verdict baru **START-JUMP** di `_plan_and_screen`, SEGERA sesudah cek titik kosong (sebelum tuck): menolak bila
  d01 > T01 ∨ d_in > T_in ∨ |p0 − start| > T01 (start = keadaan terukur atau `start_joints`). Masuk `RETRYABLE` (sampel OMPL
  lain; dari start yang benar-benar tabrak semua sampel melompat → menolak terus = benar). Batas = konstanta modul
  dengan komentar sumber; default tidak bisa dimatikan.
- Bila validasi NYATA tidak 100 % diterima: batas salah → ditulis, aturan diperbaiki (B), ulang (maks 3 putaran).

### A3. 🔒 Uji

- **K-ARSIP (offline, fungsi yang SAMA dengan probe — `start_jump(traj_pts, start)` dipisah agar bisa diuji):**
  NYATA 100 % diterima; MOCK: jumlah yang akan ditolak dilaporkan (harap hanya yang start-nya tabrak).
- **Kontrol positif mock (domain 77, in-process `_plan_and_screen`, `start_joints`, tanpa `--move`):**
  (P1) G36 B1: titik akhir ev 10 hw_s0 +103.7° + `right_finger_bottom` arm_3 = +0.05 rad, tujuan REST, n = 5 →
  `/check_state_validity` start INVALID; setiap jawaban SUCCESS MoveIt harus START-JUMP; tidak ada PLANNED.
  (P2) G38 s1 t2: start rencana arsip `g38_planonly_plans.jsonl` (s1 t2), tujuan sama, n = 5 → sama.
  Kontrol negatif: (N1) ujung yang sama dengan jari 0 (start VALID) n = 5 → tidak ada START-JUMP;
  (N2) DRY-CLI satu seed G42f (s0, k = 1) → 0 START-JUMP.
- **Gerbang 2:** P1 dan P2 DITOLAK (semua SUCCESS MoveIt → START-JUMP), K-ARSIP NYATA 100 % diterima, N1/N2 0 START-JUMP.

### A4. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D360 | d01 MOCK p99.9 ≤ **0.01 rad** (mulai dari diam, TOTG percepatan terbatas). |
| D361 | d_in MOCK p99.9 ≤ **0.05 rad**. |
| D362 | T01 ≤ 0.07 rad (lompatan terkecil G36) → B1 terpisah dari normal. |
| D363 | K-ARSIP NYATA: **100 %** diterima. |
| D364 | K-ARSIP MOCK: ditolak ≤ **0.5 %** rencana, dan G38 s1 t2 termasuk. |
| D365 | P1: MoveIt SUCCESS ≥ 3/5 dan semuanya START-JUMP; P2: semua START-JUMP. |
| D366 | N1 dan N2: 0 START-JUMP. |

## B. Hasil terukur

### B1. Distribusi |dq| — [log](results/p1_g44/g44_dist.log), [K-ARSIP aturan terkunci](results/p1_g44/g44_karsip_locked.log)

| kelompok | rencana | d01 p50 / p99.9 / maks | d_in p50 / p99.9 / maks | \|p0 − start\| maks |
|---|---|---|---|---|
| MOCK (20 arsip G31–G41) | 853 | 0.005 / 0.005 / **0.005** | 0.024 / 0.048 / **0.048** | **0** |
| NYATA (6 arsip `_hw_`) | 95 | 0.005 / 0.005 / 0.005 | 0.024 / 0.048 / 0.048 | 0 |

- **d01 = 0.005 rad persis di 948/948 rencana**: lintasan arsip = keluaran TOTG yang DISAMPEL ULANG tiap 0.1 s dari diam
  (langkah pertama = ½·a·dt², sama untuk semua). p0 = start persis di semua rencana.
- Batas terkunci T01 = 0.015, T_in = 0.144 rad → K-ARSIP: MOCK 0/853 ditolak, NYATA 0/95 ditolak (D363 ✓) — **tetapi G38 s1 t2
  (kontrol positif P2, start TABRAK) DITERIMA**: lintasannya (126 titik) mulus, d01 0.005, d_in ≤ 0.048. → D362/D364 ✗,
  **Gerbang 2 putaran 1 GAGAL** (offline, sebelum mock).
- **Diagnosa (data):** premis A0 salah. Angka G36 "0.07–1.12 rad" adalah jarak jiggle di log move_group (`Found a valid state
  near the start state at distance …`), bukan langkah waypoint. Adaptor menambahkan start asli sebagai prefiks, lalu
  `AddTimeOptimalParameterization` (adaptor terluar) memparameterkan dan menyampel ulang SELURUH jalur — segmen start →
  start-geser menjadi puluhan langkah ≤ 0.048 rad yang memang dilihat penyaring waypoint. Yang tidak dijamin: segmen itu
  tidak pernah dicek tabrakan oleh MoveIt, dan rencana berangkat dari keadaan yang MoveIt sendiri anggap tabrak. Ukuran
  |dq| tidak bisa membedakannya; pemicunya = **validitas start di scene MoveIt**.

### B2. Perbaikan putaran 2 (ditulis SEBELUM diuji)

- START-JUMP di `_plan_and_screen` = (i) **start TIDAK VALID** menurut `/check_state_validity` (`group_name=''` = seluruh
  robot, keadaan = keadaan monitor move_group + diff `start_joints`, persis yang dipakai perencana) — dicek SEBELUM
  meminta rencana; ATAU (ii) aturan dq A2 (T01 0.015 / T_in 0.144 rad dari data MOCK; pengaman bila lintasan tanpa sampel
  ulang muncul). Layanan tidak terjawab → `START-UNSCREENED` (menolak, tidak diulang). START-JUMP tetap `RETRYABLE`
  (keadaan terukur bisa berubah antar percobaan; dari start tabrak diam semua percobaan menolak).
- Uji (mock 77, sesudah G43 berat selesai): P1, P2, N1 seperti A3; **K-ARSIP′** = `/check_state_validity` pada start
  SEMUA rencana NYATA (95) dan MOCK (853, jari 0 seperti arsip): NYATA harus 100 % VALID; MOCK yang INVALID dilaporkan
  (harap G38 s1 t2 termasuk). N2 DRY-CLI s0 k = 1.
- Dugaan tambahan: **D367** K-ARSIP′ NYATA 95/95 VALID; **D368** MOCK INVALID ≤ 5 rencana, termasuk G38 s1 t2.

### B3. Uji putaran 2 (mock 77) — [K-ARSIP′](results/p1_g44/g44_karsip_valid.log), [kontrol](results/p1_g44/g44_controls.log), [N2](results/p1_g44/g44_n2_drycli.log)

Mock domain 77: launch `SIG_DFL…execvp` + `setsid nohup`, SigIgn `0x1001001`, 7/7 controller, 1 node per nama
(spawner `joint_state_broadcaster` "already loaded" = pola sama di log mock G42, 7×). Job G43 berjalan bersamaan **nice 19 +
Pool 8** (aturan kampanye "kalau terpaksa"); uji G44 tidak bergantung CPU (validitas deterministik; N1/N2 hanya menghitung
START-JUMP).

- **K-ARSIP′:** start NYATA **95/95 VALID** → D367 ✓. MOCK **837/853** VALID; 16 INVALID = **15 rencana G31** (seed 2, 13, 29,
  33; jari/gripper lengan pasangan REST × `env_static_map`) + **G38 s1 t2**. G31 dijalankan 2026-10-06, SEBELUM peta
  lingkungan masuk scene (G33, 10-07): start itu sah saat itu dan kini tabrak di scene sekarang — jadwal R35 lama, tidak
  pernah dieksekusi di HW. → D368 ✗ (16 > 5), tetapi G38 s1 t2 termasuk.
- **P1** (G36 B1, jari +0.05): start INVALID (`t2_a1_right_finger_prox_link|env_static_map`), MoveIt mentah **SUCCESS 5/5**,
  penyaring **START-JUMP 5/5** → LULUS. **P2** (G38 s1 t2): INVALID (jari t1_a2 × peta), MoveIt mentah SUCCESS 5/5,
  **START-JUMP 5/5** → LULUS. D365 ✓.
- **N1** (jari 0): start VALID, **PLANNED 5/5** (torsi ≤ 89 %, env ≥ 54.1 mm). **N2** DRY-CLI G42f s0 k = 1 (monitor hidup;
  percobaan pertama tanpa monitor = `CLI-FAIL` mesin A5, [log](results/p1_g44/g44_n2_drycli_nomonitor.log), bukan
  penyaring): **6/6 PLANNED percobaan pertama**, 0 START-JUMP → D366 ✓.

### B4. Papan skor D360–D368

| # | Dugaan | Hasil |
|---|---|---|
| D360 | d01 MOCK p99.9 ≤ 0.01 rad | ✓ 0.005 |
| D361 | d_in MOCK p99.9 ≤ 0.05 rad | ✓ 0.048 |
| D362 | T01 ≤ 0.07 memisahkan B1 dari normal | ✗ lompatan tidak tampak sebagai langkah (TOTG sampel ulang) |
| D363 | K-ARSIP NYATA 100 % diterima | ✓ 95/95 |
| D364 | MOCK ditolak ≤ 0.5 % termasuk G38 s1 t2 | ✗ 0 ditolak, s1 t2 DITERIMA (aturan dq) |
| D365 | P1 SUCCESS ≥ 3/5 semua START-JUMP; P2 semua | ✓ 5/5, 5/5 (aturan B2) |
| D366 | N1, N2 0 START-JUMP | ✓ |
| D367 | K-ARSIP′ NYATA 95/95 VALID | ✓ |
| D368 | MOCK INVALID ≤ 5, termasuk s1 t2 | ✗ 16 (15 = G31 pra-peta) |

**Tally G44: 6 tepat / 3 meleset.** Meleset = satu salah paham (angka G36 = jarak jiggle, bukan langkah waypoint).

### B5. Gerbang 2 — **LULUS (putaran 2)**

P1 + P2 DITOLAK (START-JUMP 10/10 jawaban SUCCESS MoveIt), NYATA 95/95 diterima (aturan dq) dan VALID (aturan validitas),
N1/N2 0 START-JUMP. Putaran 1 gagal offline (B1), diperbaiki (B2).

## C. Keadaan akhir

- NOL gerak HW. Kode: `scripts/reach_dwell_probe.py` — `start_valid` (`/check_state_validity`, group ''), `start_jump` +
  `START_JUMP_T01/TIN`, verdict START-JUMP (sebelum rencana; dan aturan dq sesudah rencana), `START-UNSCREENED`,
  `RETRYABLE` + START-JUMP. Berlaku untuk SEMUA pemanggil `_plan_and_screen` (probe HW, plan-only `g31_screen`, retract
  MoveIt `return_rest`). Biaya: 1 panggilan layanan + (tanpa `start_joints`) `wait_joints` 1 s per percobaan.
- Celah sisa (ditulis): validitas start di HW memakai keadaan monitor move_group (jari dari `/joint_states` HW); keadaan
  yang MoveIt anggap tabrak tetapi EnvChecker CLEAR akan DITOLAK (lebih ketat — MARGIN = TOLAK).
