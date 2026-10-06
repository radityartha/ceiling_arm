# P1 / G32-OFF — JADWAL BEROTASI: penjadwal ulang seed 36 R10, patch `return_rest`, alat `pose_to_g` + `run_g32`, PLAN-ONLY + DRY MOCK

> Sesi 2026-10-06. **Operator TIDAK di lokasi → hanya bagian robot-mati.** Nol gerak fisik; robot tidak dinyalakan.
> Sumber prompt: [p1_g31_rot_sched.md §D](p1_g31_rot_sched.md). Kode/hasil: `docs/results/p1_g32/`.
> Bagian HW (tahap 0, plan-only di stack NYATA, eksekusi) → sesi G32-HW (§D).

---

## A0. Gerbang (jawaban operator 2026-10-06, SEBELUM §A)

| # | Gerbang | Jawaban |
|---|---|---|
| G1 | patch `return_rest`: held += kedua rotasi, nama ketat, hilang = TOLAK | **IZIN** |
| G2 | rentang rotasi | **R10** (\|rot\| ≤ 10°, terverifikasi HW G30) |
| G3 | alat pose | **satu trajektori JTC lin + rot bersama** (bridge target absolut) — model `max(T_lin, T_rot)` |
| G4 | seed | **36** (G31 R10 −14 %) |
| — | stack plan-only (prompt: NYATA) | robot mati → **MOCK** (`use_fake_hardware:=true enable_gantry_bridge:=false`, `ROS_DOMAIN_ID=77`) — pertentangan dengan prompt, ditulis; plan-only NYATA pindah ke G32-HW |

### A0.1 Mekanisme yang dibaca (kode, sebelum §A)

- **Bridge** (`dual_table_controller._on_hw_command`): per gantry, kirim ulang bila setpoint JTC bergeser ≥ 1.0 mm **atau** ≥ 0.3°
  dari target terakhir **dan** tidak ada thread; satu `go_to_absolute(lin_mm, rot_deg)` untuk **ketiga** motor sekaligus;
  thread kembali hanya saat **ketiga** motor ≤ 50 pulsa. ⇒ JTC lin + rot dalam satu goal = satu urutan perintah absolut
  gabungan; sumbu lambat menentukan kapan thread kembali. Satu JTC sah untuk G3; tidak ada jalur kode baru di bridge.
- **`return_rest.py:150`** `held` = lengan gantry lain + rel lain, **tanpa rotasi**; `CrossGantryChecker.q_from` mulai dari
  `neutral` → rotasi dinilai 0 diam-diam. Penyaring se-gantry: kedua lengan pada platform yang sama → invarian terhadap rotasi
  gantry sendiri (tidak perlu rotasi). Model ketiga checker mengenal `t1/t2_linear_joint`, `t1/t2_rotation_joint` (diperiksa).
- **`run_g22.py`**: `s26` menolak \|rot\| > 0.5° (keras), traverse = `rail_to_g` (S23 menolak \|rot\| > 0.5°), event dari
  `g22_plan.events` (rel saja). `g31_screen.events` sudah sadar rotasi (traverse bila rel **atau** rot berubah).
- **G31 seed 36 R10** (`g31_candidates.json`): g1 (0.90, 0) → **(1.35, −10)**; g2 **(0.60, −10)** {0, 1, 5} → (1.45, 0) {2};
  makespan 311.478875. Rotasi g1 −10 berbarengan dengan rel 0.45 m (T_lin mendominasi) → dugaan **seri** (pemutus seri DP bebas).

---

## A. Protokol — DIKUNCI sebelum patch, alat, dan data

> 🔒 §B menang atas §A; pertentangan DITULIS.

### A1. 🔒 Jadwal ulang seed 36 R10 + pemutus seri rot 0 (`g32_sched.py`)

Instance = `g31_sched.build(row 36, R10)` apa adanya (P1′ + `rot_cmd 0.9235`, oracle‴ malas-eksak, cache G31 dipakai ulang).
Pemutus seri = biaya leksikografis: `sched.traverse_matrix` dibungkus **di skrip** (sched.py tidak diubah) menambah
`ε = 1e-6 s` untuk setiap pindah **ke** pose rot ≠ 0 (juga `T0`). Ketaksamaan segitiga tetap (tujuan berotasi selalu
membayar ε di kedua sisi). Makespan sejati dihitung ulang tanpa ε dari jadwal (traverse + durasi stop, per gantry).
Lulus bila: makespan sejati = R10 tanpa ε (|Δ| ≤ 1e-9) dan Σε < selisih biaya bukan-nol terkecil (dinyatakan).
Keluaran `g32_candidates.json` (format G31; varian `R10` = jadwal ber-ε, `R10_noeps`, `R0`).

### A2. 🔒 Patch `return_rest.py` (G1)

`held += ['t1_rotation_joint', 't2_rotation_joint']` (keduanya, selalu) → masuk ke keadaan antar-gantry; hilang dari
`/joint_states` → penolakan `missing` yang sudah ada (rc 1). Nama ketat: setiap nama di `names + [rail] + held` (antar)
dan `names + [rail] + partner` (se-gantry) harus `model.existJointName` di checker masing-masing — tidak → 🔴 TOLAK rc 1.
Tidak ada perubahan lain (interpolasi, 30 s, penjaga torsi).

### A3. 🔒 Alat `pose_to_g.py` (baru; gabungan `rail_to_g` + `rot_to_g`, G3)

`python3 pose_to_g.py --gantry G --seed S --plan P LIN ROT_DEG [--move]`, DRY tanpa `--move`. Menolak (rc 1) bila:
- **S12** lengan gantry G maks \|q − REST\| ≥ 0.5° (lengan gantry lain dilaporkan);
- **S13″** (LIN, ROT) bukan pose jadwal gantry G seed S (+ p0 (0, 0)); LIN ∉ [0, 1.600];
- **S23″** \|ROT\| > 10.0°; \|rot terukur G\| > 10.5°; \|rot gantry lain\| > 10.5° (R10; amplop B3 G29 "keduanya ≤ 35°, lengan
  REST" dipenuhi dengan sisa);
- **S28** `sweep_rot(RotCrossChecker, rect)` dari keadaan **terukur** (24 sendi lengan, kedua rel, kedua rotasi, nama ketat —
  hilang = tolak) `(lin, rot) → (LIN, ROT)` bukan **CLEAR** (margin 50 mm), atau penyaring tidak bisa jalan.

Trajektori: **satu** goal `[t{G}_linear_joint, t{G}_rotation_joint]`, `gantry_{G}_with_arm_controller`, kedua sumbu dari
**terukur** ke target jadwal, cosinus, `T_cmd = max(t_cmd_lin(Δlin), t_cmd_rot(Δrot))` (`g22_plan.t_cmd`, rumus `rot_to_g`;
sumbu tanpa gerak = 0), `STEPS = max(10, 2·T_cmd)`, kecepatan titik 0. Ukur: akhir = kedua encoder diam (lin < 0.05 mm,
rot < 0.01° selama 1.0 s) sesudah gerak (lin > 0.5 mm atau rot > 0.05°), batas 30 s. JSON: `err_mm`, `err_deg`,
`t_traverse` (gerak pertama sumbu mana pun → perubahan terakhir sumbu mana pun), `t_lin`, `t_rot`, `T_cmd`, `T_cmd_lin`,
`T_cmd_rot`, drift lengan (keempat), rel lain, rot lain, torsi puncak lengan (jendela gerak), `sweep_*`, `jtc_error_code`.
**rc 3** bila \|err_mm\| > 2.0, \|err_deg\| > 1.0, drift lengan > 0.5°, rel lain > 2 mm, rot lain > 0.5°, torsi > 14 N·m.

### A4. 🔒 Runner `run_g32.py` (salinan `run_g22`)

Perubahan saja: event = `g31_screen.events(schedule R10, p0)` (traverse bila rel **atau** rot berubah); traverse =
`pose_to_g --move`; retract = `return_rest` terpatch; `s26`: rel g = diharapkan ± 2 mm **dan** rot g = diharapkan ± 1.0°
(ganti "rot bukan 0"); diharapkan diperbarui dari target jadwal sesudah traverse rc 0. Lain (`FAULT_RE`, monitor/perekam/
`ros2_control_node` hidup, torsi > 14, basi, A3 awal keempat lengan REST, HALTED dicatat lanjut) apa adanya.
Rotasi hanya dengan lengan REST: dijamin S12 `pose_to_g` + urutan retract → traverse; satu gantry bergerak per waktu: runner serial.

### A5. 🔒 Plan-only MOCK + DRY + smoke

1. `g31_screen.py --seeds 36 --variant R10 --plan g32_candidates.json` k-of-k 3, lintasan disimpan. **Data plan-only (mock).**
2. `run_g32 --dry` di mock (perekam + monitor hidup) → daftar event + prasyarat.
3. `pose_to_g` DRY per traverse jadwal dari keadaan mock yang sesuai — lewat smoke (4).
4. **Smoke MOCK `run_g32` penuh `--move`** — `use_fake_hardware` (sendi palsu mengikuti JTC sempurna; bridge mati).
   **BUKAN data**: tidak dinilai, waktu/galat tidak dipakai; hanya bukti alat jalan ujung-ke-ujung.

### A6. 🔒 Kontrol

| id | jenis | isi | lulus bila |
|---|---|---|---|
| K-S0 | negatif | `g32_sched` tanpa ε vs `g31_candidates` R10 seed 36 | makespan + jadwal bit-identik |
| K-S1 | lulus | makespan sejati ber-ε vs tanpa ε | \|Δ\| ≤ 1e-9 |
| K-RR0 | negatif | `return_rest` DRY, `/joint_states` sintetis rot 0/0 (fake_js, domain 77, tanpa stack), lengan g2 +1.0°: patch vs `git show HEAD` | baris penyaring bit-identik |
| K-RRp | positif | sama, rot1 = −10°, rot2 = −10° (rel 0.9 / 0.6) | antar-gantry min **berbeda** dari pra-patch |
| K-RRm | positif | publisher tanpa `t1_rotation_joint` | TOLAK rc 1 |
| K-RRn | positif | nama sendi tak dikenal disuntikkan ke `held` (uji unit, monkeypatch) | TOLAK rc 1 |
| K-P | DRY sintetis | `pose_to_g` pada fake_js: (a) g2 (0,0)→(0.60,−10) CLEAR; (b) g1 (0.90,0)→(1.35,0) CLEAR; (c) lengan g2 +1° → S12; (d) target bukan jadwal → S13″; (e) ROT 15 → S23″; (f) tanpa rotasi di `/joint_states` → tolak | 6/6 sesuai |
| K-E | event | `run_g32` event list vs `g31_screen.events` (fungsi sama) | identik |

### A7. 🔒 Dugaan D189–D199 (SEBELUM patch, alat, data)

Prior tally G31: mekanisme kode dibaca → tepat; besaran perilaku baru tanpa hitung → meleset; kode sendiri → sering gagal jalan pertama.

| D | dugaan | dasar |
|---|---|---|
| D189 | jadwal ber-ε: g1 **tanpa rotasi** ((0.90, 0), (1.35, 0)); g2 tetap (0.60, −10) → (1.45, 0); makespan sejati **311.478875** | A0.1: rotasi g1 berbarengan rel 0.45 m → seri |
| D190 | K-S0 bit-identik | instance + cache sama |
| D191 | lazy: **1** iterasi (tuple rot ≠ 0 terpakai sudah di cache G31) | cache 23 500 tuple; tetangga terdekat diverifikasi |
| D192 | K-RR0 bit-identik | rot 0 = `neutral` |
| D193 | K-RRp antar-gantry berubah \|Δ\| ≥ **1 mm** | base bergeser 0.4·sin 10° ≈ 69 mm |
| D194 | K-RRm, K-RRn TOLAK | kode `missing` + `existJointName` |
| D195 | K-P 6/6 — tetapi **≥ 1 galat kode sendiri** sebelum DRY bersih (pola D157 G30) | prior |
| D196 | plan-only mock seed 36 R10 **LOLOS 3/3**, 18 PLANNED; traverse min ≥ **300 mm** | G31 seed 36 R35 (g1 −10 ⊃) LOLOS; R10-ε lebih sedikit rotasi |
| D197 | event: **14** (4 retract — 2 dilewati di REST awal, 4 traverse, 6 tugas); traverse berotasi **2** (g2 0 → −10, −10 → 0) | A1 + `events` |
| D198 | `pose_to_g` g2 (0,0)→(0.60,−10): T_cmd = T_cmd_lin = **33.33 s** (rot 3.0 s); P1′ traverse = 0.9235·… tak relevan, rel `rail_cmd` mendominasi | rumus |
| D199 | smoke mock penuh `--move` 6/6 tugas, rc 0 (sesudah perbaikan galat kode sendiri bila ada) | mock mengikuti JTC |

Aturan: NOL gerak fisik; mock `ROS_DOMAIN_ID=77`, dimatikan bersih (PID launch ASLI, `ros2 node list --no-daemon` = 0);
crash dump milik sesi dihapus; job > 10 menit via `setsid nohup`.

---

## B. Hasil terukur

> §A dikunci **2026-10-06 20:47:38**, sha256 `2548b284ffa3…` ([sectionA_locked.md](results/p1_g32/sectionA_locked.md)).

### B1. Jadwal seed 36 R10 + pemutus seri ([g32_sched.py](results/p1_g32/g32_sched.py) → [log](results/p1_g32/g32_sched.log), [g32_candidates.json](results/p1_g32/g32_candidates.json))

| varian | makespan sejati | iterasi | rot g1 / g2 | pindah |
|---|---|---|---|---|
| R0 | 362.352875 (= G31) | 1 | 0, 0 / 0, 0, 0 | 2 / 2 |
| R10 tanpa ε | 311.478875 | 1 | 0, **−10** / −10, 0 | 2 / 2 |
| **R10 (ε, terkunci G32)** | **311.478875** (DP 311.478876) | 1 | **0, 0** / −10, 0 | 2 / 2 |

- **K-S0** tanpa ε == G31 R10 (makespan + jadwal) **True**; **K-S1** \|ε − tanpa ε\| sejati **0.000** ✅. Selisih biaya bukan-nol
  terkecil 0.080 s ≫ ε terpakai 2·10⁻⁶ s.
- Jadwal terkunci: g1 (0.90, 0) {t4 arm1} → (1.35, 0) {t3 arm1}; g2 **(0.60, −10)** {t0 arm3, t1 arm4, t5 arm3} → (1.45, 0) {t2 arm3}.
  Rotasi g1 G31 = seri murni (pemutus seri DP bebas); satu-satunya pose berotasi = g2 stop 0.

### B2. Patch `return_rest` — kontrol ([k_rr.sh](results/p1_g32/k_rr.sh) → [log](results/p1_g32/k_rr.log))

Diff: `held += t1/t2_rotation_joint`; nama ketat di kedua checker (se-gantry: `names + rail + partner`, antar: `names + rail + held`)
→ 🔴 TOLAK rc 1. Pra-patch = `git show HEAD:scripts/return_rest.py` ([salinan](results/p1_g32/return_rest_pre_g32.py)).
`/joint_states` sintetis ([fake_js.py](results/p1_g32/fake_js.py), salinan G31 + `--drop`), domain 77, tanpa stack; lengan g2 +1.0°.

| id | keadaan | pra-patch | terpatch | |
|---|---|---|---|---|
| K-RR0 | rel 0/0, rot 0/0 | CLEAR, se 615.2, antar 558.7 | **bit-identik** (semua baris) | ✅ |
| K-RRp | rel 0.9/0.6, rot −10/−10 | antar **514.8 mm** (titik 1, rot dinilai 0) | antar **583.8 mm** (titik 0) | ✅ Δ +69.0 |
| K-RRm | tanpa `t1_rotation_joint` | jalan, CLEAR (rot dibuang diam) | **TOLAK rc 1** "hilang ['t1_rotation_joint']" | ✅ |
| K-RRn | model antar buta `t1_rotation_joint` ([wrap](results/p1_g32/k_rrn_wrap.py)) | — | **TOLAK rc 1** "sendi tidak dikenal" | ✅ |

Arah K-RRp: di keadaan ini pra-patch **lebih konservatif** (514.8 < 583.8) — kebetulan keadaan, bukan jaminan; G31 K-C2p
menunjukkan arah sebaliknya (rot dibuang → CLEAR palsu). Harness K-RR gagal 2× sebelum data (kode sendiri: salinan pra-patch
tak menemukan `interarm_collision`, lalu `PYTHONPATH` menimpa jalur ROS) — bukan alat.

### B3. `pose_to_g` DRY sintetis ([k_p.sh](results/p1_g32/k_p.sh) → [log](results/p1_g32/k_p.log)) — **7/7 sesuai**, jalan pertama

| kasus | hasil |
|---|---|
| (a) g2 (0,0)→(0.60,−10) | CLEAR **380.0 mm** (SS platform, konstan), lengan 446.1, n 671; **T_cmd 33.33 s = lin** (rot 3.00) |
| (b) g1 (0.90,0)→(1.35,0) | CLEAR 509.7 mm (lengan), T_cmd 25.00 (rot 0) |
| (c) arm_3 +1° | TOLAK S12 |
| (d) (0.50, 0) | TOLAK S13″ (pose sah g2: (0,0), (0.6,−10), (1.45,0)) |
| (e1) ROT 15 | TOLAK **S13″** (bukan S23″ — S13″ diperiksa dulu; pose jadwal R10 tak pernah > 10°) |
| (e2) rot g1 terukur +12° | TOLAK S23″ (cabang terukur, ditambah karena e1 tak mencapai S23″) |
| (f) tanpa `t2_rotation_joint` | TOLAK "hilang" |

### B4. `run_g32` ([run_g32.py](results/p1_g32/run_g32.py))

Salinan `run_g22`; diff = A4 saja (event `g31_screen.events`, `pose_to_g`, `s26` rel ± 2 mm **dan** rot ± 1.0° vs diharapkan,
`len(tasks)` ganti 6 keras, nama/OUT g32).

### B5. Stack MOCK, plan-only, DRY, smoke

Stack mock domain 77 ([launch log](results/p1_g32/g32_mock_launch.log.gz)): launch PID **1843683** (`SIG_DFL…execvp` via
`setsid nohup`, SigIgn `0x1001001`, SIGINT tidak diabaikan); **7/7** controller; 7 spawner mati + 4 octomap updater = baseline;
`/tmp/reach_dwell_live.urdf` sha `f02e7c532cdb`.

**Plan-only seed 36 R10 — LOLOS 3/3** ([log](results/p1_g32/g32_screen.log), [json](results/p1_g32/g32_screen.json),
lintasan [g32_screen_plans.jsonl.gz](results/p1_g32/g32_screen_plans.jsonl.gz)):

| ukuran (3 sampel) | nilai |
|---|---|
| tugas | **18 PLANNED / 0 lain**; 9 di g2 rot −10 (t0 arm3, t1 arm4, t5 arm3) |
| traverse (`sweep_rot` rect) | 12, semua CLEAR; min **380.0 mm** (SS platform); berotasi g2 (0,0)→(0.60,−10) **498.9** (lengan), (0.60,−10)→(1.45,0) 380.0 (lengan 420.7–460.5) |
| retract g2 @ rot −10 | se-gantry min **369.7** mm, antar **569.1** mm — CLEAR |
| S18 probe se-gantry min | 369.6 mm |

**Persiapan mock:** `return_rest --move` (terpatch) g1 lalu g2 dari pose awal mock (99.63° dari REST) → galat 0.000°
([log](results/p1_g32/g32_mock_return_rest.log)); antar-gantry 558.7 mm.

**DRY `run_g32`** ([log](results/p1_g32/dry/g32_dry_runner.log)): monitor/perekam/`ros2_control_node` hidup; **14 event** (4 retract,
4 traverse — **2 berotasi**: ev 7 (0,0)→(0.60,−10°), ev 12 (0.60,−10°)→(1.45,0) — 6 tugas); A3 awal 0.000° / rel 0/0; tidak dikirim.

**Smoke MOCK penuh `--move` — BUKAN data** ([smoke/](results/p1_g32/smoke/)): **14/14 event rc 0, 6/6 tugas sukses (penilai)**;
retract ev 0/6 dilewati (REST); `pose_to_g` 4/4: galat 0, `t_lin`≈`t_rot`≈T_cmd (mock mengikuti JTC — pada HW rotasi motor
10 °/s mengejar setpoint bertahap, **tidak** diwakili mock). Waktu mock (601.88 s) **tidak** dinilai.

### B6. Papan skor D189–D199 — **10 / 11 tepat**

| D | dugaan | terukur | |
|---|---|---|---|
| D189 | g1 tanpa rotasi, g2 (0.60,−10)→(1.45,0), 311.478875 | tepat | ✅ |
| D190 | K-S0 bit-identik | True | ✅ |
| D191 | 1 iterasi | 1 (ketiga varian) | ✅ |
| D192 | K-RR0 bit-identik | ya | ✅ |
| D193 | K-RRp \|Δ\| ≥ 1 mm | +69.0 mm | ✅ |
| D194 | K-RRm, K-RRn TOLAK | keduanya rc 1 | ✅ |
| D195 | K-P 6/6 **dan** ≥ 1 galat kode sendiri sebelum DRY alat bersih | 7/7, **`pose_to_g` bersih jalan pertama** (galat hanya di harness K-RR) | ❌ |
| D196 | LOLOS 3/3, 18 PLANNED, traverse min ≥ 300 | 3/3, 18, 380.0 | ✅ |
| D197 | 14 event, 2 retract dilewati, 2 traverse berotasi | tepat | ✅ |
| D198 | T_cmd g2 33.33 = lin, rot 3.0 | 33.333 / 3.0 | ✅ |
| D199 | smoke 6/6, rc 0 | 6/6, 14/14 event rc 0 (proses runner keluar **134** — pola benign `run_g22`, lihat B7 (5)) | ✅ |

Pola: semua tepat = mekanisme kode yang dibaca (seri DP, `neutral`, `events`, rumus T_cmd). Meleset = prior "kode sendiri gagal
dulu" dipakai untuk alat yang disalin dari dua alat teruji — salinan dekat dari pola teruji tidak mewarisi prior itu.

### B7. Pertentangan §B lawan §A / prompt

| # | Pertentangan |
|---|---|
| (1) | Prompt: plan-only di stack **NYATA**. Robot mati → **MOCK** (A0). Plan-only nyata pindah ke G32-HW. |
| (2) | 🔶 **G4 dasar "−14 %" adalah makespan paralel (max).** Runner **serial** → pembanding HW = P1′-serial Σ finish: R10 **530.39 s** vs R0 **581.26 s** = **−8.75 %** (−50.87 s) — **di dalam** rentang bias model HW G26 (−0.4 … −11.9 %). Satu run R10 saja **tidak** dapat memisahkan manfaat rotasi dari bias model ⇒ usul G32-HW: jalankan **R0 seed 36 juga** (pembanding sama-sesi). |
| (3) | K-P (e) "ROT 15 → S23″" tertolak di **S13″** lebih dulu; (e2) ditambah untuk cabang S23″ terukur. |
| (4) | Harness K-RR gagal 2× sebelum data (impor, `PYTHONPATH`) — kode sendiri, bukan alat. |
| (5) | `run_g32` keluar **rc 134** ("terminate called without an active exception", thread spin daemon) pada DRY dan smoke — warisan `run_g22` (memori G24b "rc 134 benign"); arsip `--dry` kosong (jalur `--dry` keluar sebelum `finish()`, warisan) → disalin manual. |
| (6) | move_group "Found empty JointState" 24× = 4 per tugas `--move` — sama dengan G26 HW (72 = 18 × 4); tidak ada di plan-only. Baseline probe, bukan regresi. |

## C. Keadaan akhir (Rule 12)

- **Nol gerak fisik**; robot tidak dinyalakan. Mock: perekam + monitor SIGINT; launch 1843683 SIGINT → keluar **12 s**;
  `ros2 node list --no-daemon` (domain 77) **0**; nol proses sisa. Crash dump milik sesi **dihapus**: python 21:06 (DRY), 21:17
  (smoke), `move_group` 21:26 (200 MB, shutdown — pola G31). Disk **57 GB**.
- **Diubah (belum di-commit):** `scripts/return_rest.py` (A2). **Baru:** `docs/results/p1_g32/` (`g32_sched.py`, `pose_to_g.py`,
  `run_g32.py`, harness K-RR/K-P, log), dokumen ini. **Tidak diubah:** `sched.py`, `interarm_collision.py`, probe, `g31_screen.py`,
  `rail_to_g.py`, `rot_to_g.py`, `run_g22.py`, `dual_table_controller.py`, `joint_limits.yaml`.
- **Tidak dilakukan (butuh operator / HW):** tahap 0 (LED, origin, ICMP), plan-only di stack **nyata**, `pose_to_g`/`run_g32` DRY
  pada keadaan **terukur nyata**, eksekusi, makespan vs P1′-serial, galat rotasi per pose, drift, torsi.
- **Belum pernah di HW:** satu goal JTC lin + rot (bridge mengirim keduanya sebagai satu target absolut — mekanisme dibaca, tidak diukur);
  lengan bergerak pada gantry rot ≠ 0 (tugas t0/t1/t5 di g2 −10°); `return_rest` terpatch `--move` nyata.
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi multi-langkah; dilaporkan.

---

## D. Prompt G32-HW (salin ke chat BARU) — HW: eksekusi seed 36 R10 berotasi

**Rekomendasi: Opus, effort TINGGI** — gerak lengan nyata pada rot ≠ 0 pertama kali + goal JTC lin+rot pertama di bridge
nyata; galat penyaring diam dan satu arah, dan pembanding R0 menentukan apakah manfaat rotasi terukur sama sekali.

```
Sesi G32-HW -- HW: eksekusi jadwal seed 36 R10 (rotasi g2 -10 deg, lengan REST saat berputar). Repo ceiling_arm,
branch feat/rgbd-topo-deploy. Operator di lokasi. Bagian OFF SELESAI (docs/p1_g32_rot_exec.md).

BACA PENUH: CLAUDE.md; docs/p1_g32_rot_exec.md (semua, terutama B7, C); docs/p1_g30_rot_hw.md (A0.3, A2, B1);
docs/p1_g26_hw.md (B0 tahap 0); docs/results/p1_g32/run_g32.py, pose_to_g.py; scripts/return_rest.py (patch A2).

GERBANG (tanya operator SEBELUM apa pun):
 H1. Tahap 0: LED 4/4 tidak merah; origin g1 dan g2 = home fisik; rel + ruang antar-gantry bebas s.d. ~1.6 m; g2 bebas
     henti/kabel di -10 deg pada rel 0.6 m (G30 hanya menguji rel 0).
 H2. Pembanding R0 seed 36 di sesi yang sama (B7 (2): -8.75 % serial ada DI DALAM bias model G26)? Usul: R0 dulu, lalu R10.
 H3. Izin uji pose_to_g HW satu kaki dulu (g2 (0,0) -> (0.60,-10) -> (0,0), lengan REST) sebelum jadwal penuh?
1. KUNCI §A (G32-HW, dugaan D200+; prior: mekanisme dibaca -> tepat, besaran perilaku baru tanpa hitung -> meleset):
   bring-up use_fake_hardware:=false enable_gantry_bridge:=true use_sim_time:=false (SIG_DFL, cek SigIgn); 4x Actuator
   count '6', 7/7 controller, bridge ARMED; plan-only seed 36 R10 (dan R0 bila H2) di stack NYATA via g31_screen 3/3;
   run_g32 --dry dari keadaan terukur.
2. (H3) kaki uji pose_to_g; ukur galat lin/rot, t_lin/t_rot vs T_cmd, drift, torsi; jumlah dispatch bridge di launch log.
3. Eksekusi run_g32 --seed 36 (R0 lalu R10 bila H2); ukur makespan vs P1'-serial (R10 530.39, R0 581.26), galat rotasi per
   pose, drift, torsi, sukses penilai 6/6.
4. Laporan §B/§C, p1_state + tally, prompt G33 + rekomendasi model/effort.
ATURAN: rotasi HANYA dengan lengan REST (S12 pose_to_g); satu gantry bergerak per waktu (runner serial); jangan
move_dual_table selama ARMED; B menang atas A dan DITULIS; stack dimatikan bersih (PID launch ASLI, node list --no-daemon = 0);
job > 10 menit via setsid nohup; rc 134 runner saat keluar = benign (B7 (5)).
```
