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
