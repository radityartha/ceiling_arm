# P1 / G57 — HW PARALEL (`run_g54`) + blok seed G58/G60 (+ DEMO G59 opsional)

> Sesi 2026-10-12, prompt `docs/p1_g57_hw_prompt.md`, urutan/waktu/titik henti = `docs/p1_hw_day_2026-10-12.md`. Runbook
> `docs/p1_hw_runbook.md` §0–2, §7 + `docs/p1_hw_runbook_parallel.md` (risiko P1–P7 DITERIMA 2026-10-11). **SETIAP gerak nyata
> ditanya dulu (satu langkah per persetujuan, termasuk pemulihan K0 dan pulang).** **§A DIKUNCI SEBELUM gerak HW pertama**
> (salinan `docs/results/p1_g57/g57_A_locked.md` + sha256). §B menang atas §A. MARGIN = TOLAK. Arsip HW `docs/results/p1_g57/hw/`.

## A. Protokol — DIKUNCI

### A0. Pengungkapan (sebelum mengunci, NOL gerak)

- `remount_check` 11:13 JST: GERBANG LULUS — 192.168.2.100/24, ICMP .10–.13, web UI tidak dipakai, tanpa move_group /
  ros2_control_node / realsense / monitor basi, ttyUSB0/1 ada, RealSense terdeteksi. Tidak ada proses ROS hidup. Disk / 98 %
  (51 GB bebas), RAM 56 GB bebas.
- `/tmp/reach_dwell_live.urdf` ada, sha `f02e7c532cdb` (= G51). Sisa `/tmp/p1hw_*` G51 (sudah diarsip di `p1_g51/hw/`)
  dipindah ke `/tmp/g51_leftover/` agar tidak tercampur.
- Domain: shell ini `ROS_DOMAIN_ID` kosong (= 0, HW). `g47_home.sh` default **77** → selalu `ROS_DOMAIN_ID=0` di depannya
  (memori G51); `run_g54.py` mewarisi env (0); `g54_smoke.sh` memaksa 77 → TIDAK dipakai di HW.
- Amplop `p1_g48/rot_envelope.json`: g1 −125…+90, g2 −35…+95. Jadwal hari ini: lama s7 g1 +70 terbesar; baru |rot| ≤ 30.
- Keadaan terakhir (G51 akhir, tak ada gerak HW sejak): g1 PARK 1.50, g2 0.00, lengan REST, rot 0/0. Memori: g2 bisa bergeser
  antar sesi (G51 B0: −18.6 mm tanpa gerak perangkat lunak) → K0 terukur menentukan, bukan K0 lisan.
- Runner paralel `run_g54` BELUM PERNAH di HW. Bridge: cancel di tengah TRAVERSE tidak menghentikan rel seketika (G56 A0) →
  uji stop hanya saat retract lengan.

### A1. 🔒 Urutan + gerbang (rencana hari HW §0–§7)

- **§0 (tanpa gerak):** bring-up nyata runbook §1 (SIG_DFL + setsid; PID launch ASLI di-resolve sesudah controller naik,
  SigIgn tanpa 0x2; 4× `Actuator count … '6'`; 7/7 active; `table1 ARMED` + `table2 ARMED`; `/table_hw_commands` diam;
  `env_static_map` di scene) → monitor + perekam → K0 `--rail1 1.50 --rail2 0.00`. Gerbang gagal → tidak ada gerak; pemulihan
  (DRY dulu) hanya sesudah persetujuan.
- **§1 (tanpa gerak):** plan-only nyata k = 3 (`g31_screen.py --task-cli --final-retract --repeats 3`, tol 0.05°) s7 lama + s7
  baru; lolos ⟺ 3/3. DRY `run_g54 --dry`: K0 LULUS, "traverse di luar: []", rc 0.
- **§2 uji stop-semua HW:** s7 `g55_selected.json R90sel --inject-stop-on-motion 2`. Lulus ⟺ rc 3 ∧ `inject` tercatat ∧ lengan
  g2 berhenti sebelum REST (> 10° dari REST) ∧ perubahan sendi g2 ≤ 0.5° dari t_stop + 1 s sampai + 4 s (perekam) ∧
  `post_stop_motion` ≤ 0.5° ∧ tidak ada alat diluncurkan sesudah stop. GAGAL → henti total, tanya. Lalu pulang `g47_home.sh`
  (DRY dulu) + K0.
- **§3 G57:** s7 lama → s7 baru → s10 baru → s10 lama → s0 baru. Tiap jadwal: plan-only k = 3 (sekali per jadwal) → DRY → run
  (tanya) → pulang DRY → pulang (tanya) → K0. Henti A bila waktu habis (minimum: s7 + s10).
- **§4 G58/G60:** s8 baru → s8 lama; s18 lama → s18 G60; s3 G60 → s3 lama; s12 lama (G52 R90) → s12 G60. Henti B sesudah
  pasangan mana pun.
- **§5 (opsional):** ulangan s7 baru → s7 lama → s10 lama → s10 baru. **§6 (opsional):** cek kamera tanpa gerak + DEMO-HW
  (`p1_demo_off.md` §D) hanya sesudah semua run_g54. **§7:** matikan bersih + arsip `/tmp/p1hw_*`.
- **Berhenti + amankan** (lengan REST, DRY dulu, tulis, tanya) pada: fault / LED merah / `REJECTED` / `NOT armed` / torsi
  > 14 N·m / benturan / kabel / AUTO-STOP tak terduga / rc ≠ 0 tak terjelaskan. Tidak ada job offline berat / depth_cloud
  selama runner HW jalan. `run_g54` satu-satunya runner paralel.
- **Analisis per pasangan** (pola `p1_g58/g58_report.py`, `p1_g54/g54_report.py`): makespan HW vs `g54_sim` dan vs mock G58/G60;
  Δ lama − baru; dekomposisi (tugas / traverse / tunggu gerbang / tumpang-tindih dua gantry); RESERVE-BLOCKED; torsi puncak.

### A2. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D530 | Bring-up: 7/7, kedua bridge ARMED, K0 LULUS tanpa pemulihan. |
| D531 | Plan-only nyata k = 3: 3/3 di setiap jadwal yang diuji. |
| D532 | Uji stop-semua HW lulus pada percobaan pertama (kriteria §A1 §2). |
| D533 | Setiap run HW paralel: 6/6 tugas SUCCESS, 0 AUTO-STOP, 0 fault, pulang + K0 LULUS. |
| D534 | Makespan HW / mock (G58/G60, seed + varian sama) ∈ [0.90, 1.15] di setiap run. |
| D535 | Makespan HW / `g54_sim` ∈ [0.90, 1.25] di setiap run. |
| D536 | Δ(lama − baru) HW > 0 di setiap pasangan yang dijalankan, dan Δ_HW / Δ_mock ∈ [0.5, 1.5]. |
| D537 | Torsi puncak terukur ≤ 14 N·m (rating) di semua sendi semua run. |
| D538 | RESERVE-BLOCKED ≤ 2 per run (tunggu reservasi HW tidak jauh lebih sering dari mock 0). |

## B. Hasil terukur

§A sha256 `02a14717…` ([salinan](results/p1_g57/g57_A_locked.md)).

### B0. §0 bring-up (11:15–11:19) — NOL gerak — [launch 1](results/p1_g57/hw/g57_bringup1_launch.log.gz), [K0](results/p1_g57/hw/k0_bringup.json)

- Operator 11:14: di lokasi, e-stop, LED/board/lengan/rel dicek mata → izin bring-up.
- **Bring-up 1 (11:15) GAGAL tanpa gerak:** `ros2_control_node` SIGABRT (−6) di `on_init` lengan ke-3 (arm_3, 192.168.2.11):
  `terminate … std::runtime_error what(): timeout detected: BaseClient::SetServoingMode` sesudah "Session created" (arm_1, arm_2
  '6'). Kelas "lengan belum siap saat sambung" (memori workcell-known-issues; varian baru: timeout SetServoingMode, bukan
  `Actuator count '0'`). Launch SIGINT (SigIgn `0x1001001`), node 0, crash dump sesi dihapus, ICMP .11 sehat.
- **Bring-up 2 (11:16):** PID launch asli 302022, SigIgn `0x1001001` (SIGINT tidak diabaikan); 4× `Actuator count … '6'`;
  spawner gripper_3 mati "already loaded" (balapan dikenal) tetapi **7/7 active**. **table2 ARMED** 0.0 mm / 0.0°. **table1
  NOT armed** (perintah 0.0 mm vs terukur 1499.1 mm, 2× dalam 5 s sebelum JTC aktif = penjaga slam-to-zero, sama G32 B-stack2 /
  G51 bring-up 3); sesudahnya JTC `gantry_1_with_arm_controller` reference = feedback = 1.49907 m / −0.03°, `/table_hw_commands`
  diam (0 pesan / 5 s) → table1 akan ARMED pada perintah pertama. `env_static_map` di scene. Monitor + perekam hidup 11:18.
- **K0 LULUS** (11:18): lengan maks |q − REST| arm_1 0.218° / arm_2 0.194 / arm_3 0.427 / arm_4 0.394; g1 1.49907 m / −0.03°;
  g2 0.000 m / 0.00°; jari ≈ 0. (g2 tidak bergeser sejak G51.)


### B1. §1 plan-only nyata k = 3 + DRY s7 (11:18–11:33) — NOL gerak — [skrip](results/p1_g57/p1hw_planonly.sh)

- Plan-only nyata (`g31_screen --task-cli --final-retract --repeats 3`, tol 0.05°): **s7 lama G43 R180 3/3 LOLOS** (11:18–11:25,
  [json](results/p1_g57/hw/planonly_s7_R180.json)); **s7 baru G55 R90sel 3/3 LOLOS** (11:25–11:30,
  [json](results/p1_g57/hw/planonly_s7_R90sel.json)).
- DRY `run_g54 --dry` s7 R180 + s7 R90sel: K0 LULUS (sama B0), amplop g1 (−125, 90) g2 (−35, 95), "traverse di luar: []", rc 0 ×2.

### B2. §2 uji stop-semua HW (11:31–11:35) — GERAK NYATA, runner paralel pertama di HW — [run](results/p1_g57/hw/p1hw_par_inj_s7_R90sel_run.json), [runner](results/p1_g57/hw/p1hw_par_inj_s7_R90sel_runner.log), [jendela perekam](results/p1_g57/g57_stopwin.py)

- Operator 11:31: izin (di e-stop). s7 G55 `R90sel` `--inject-stop-on-motion 2`, t0 11:31:46. **table1 ARMED 1499.1 mm** pada
  perintah pertama (retract ev 0), 0 `NOT armed` / `REJECTED` baru.
- Linimasa (paralel): retract REST g1/g2 2.6–2.8 s; **g2 traverse 0 → (1.40, −10°) 97.57 s** (g1 menunggu 97.6 s — eksklusif);
  g1 traverse → (1.40, +30°) 16.76 s; lalu tugas BERSAMAAN: g2 arm_3 ev 10 25.81 s ∥ g1 arm_2 ev 2 35.86 s; arm_3 ev 11 25.39 s ∥
  arm_2 ev 3 27.56 s → 4/4 tugas SUCCESS (pos_err_max_settled ≤ 1.18 mm). Retract g2 ev 12 disaring vs reservasi g1: 243.3 mm CLEAR.
- **AUTO-STOP KEDUA gantry** (11:34:50.2): "INJEKSI kontrol negatif: g2 retract di tengah gerak (2.0 deg bergerak, 106.2 deg dari
  REST)". Retract g1 ev 4 yang sudah lolos gerbang: **"TIDAK diluncurkan: AUTO-STOP"**. Keluar lewat jalur `finish(3)` (rc 3).
- Perekam: arm_3 (t2_a1) bergerak 1.27°/s sebelum stop (j3 113.8 → 112.6, j5 97.4 → 96.1), **diam ≤ 0.1 s sesudah t_stop**;
  rentang t_stop + 1…+4 s: t2_a1 **0.0074°**, t2_a2 0.0032, t1_a1 0.0026, t1_a2 0.152° (arm_2 di pose tugas, rambatan 0.156° juga
  1 s SEBELUM stop = bukan gerak perintah); rel diam (t1 1.400 / 30.0°, t2 1.3999 / −9.99°). `post_stop_motion` {1: 0.0024,
  2: 0.0001}. → kriteria §A1 §2 semua terpenuhi → **D532 ✓**. (Mock G56: 0.000°/0.5 s dari t_stop + 0.5 s — HW sama.)
- Catatan: traverse g2 97.57 s = layar G33/S28 18.5 s (event mulai → `t_send`) + T_cmd 77.78 s (t_traverse terukur 76.13 s,
  galat −0.13 mm / +0.01°, lengan bergeser ≤ 0.059°) + penutupan ~1 s.
- Pulang DRY (`g47_home.sh 7 … R90sel --dry`) rc 0: return_rest 1 segmen per pasangan lengan; rute rel g1 (1.40, +30) → (1.50, 0),
  g2 (1.40, −10) → (0, 0) (= traverse jadwal dibalik); "REFUSE S12" di DRY karena lengan belum REST (urutan nyata: lengan dulu).
- **Pulang nyata** (izin 11:38, [log](results/p1_g57/hw/p1hw_home_inj.log)) rc 0: return_rest arm_3+4 galat 0.380°, torsi puncak 7.40
  `t2_a1_joint_2`; arm_1+2 0.217°, 5.75 `t1_a2_joint_2`; g1 → (1.499995, 0.0) −0.01 mm, 5.64 s; g2 → (0.000618, 0.0) +0.62 mm,
  75.39 s (T_cmd 77.77); lengan bergeser ≤ 0.062°; 0 `REJECTED` / `NOT armed` baru. **K0 LULUS.**

### B3. §3 G57 — run HW paralel (log per run `results/p1_g57/hw/p1hw_par_s<S>_<V>_*`, pulang `p1hw_home_*`; [run](results/p1_g57/p1hw_run.sh), [pulang](results/p1_g57/p1hw_home.sh))

Pulang = DRY `g47_home` (rute retrace jadwal) lalu kaki per kaki `pose_to_g --move`, BERHENTI pada REFUSE pertama (g47_home sendiri
melanjutkan ke kaki berikut). Di DRY, kaki ke-2+ disaring dari pose SEKARANG (DRY tidak bergerak) → "REFUSE G33" di sana = artefak;
hanya kaki pertama bermakna (ditulis sesudah s7 lama; operator menyetujui).

| # | run | mulai | makespan HW | sim | mock G58 | 6/6 | AUTO-STOP | blok | τ puncak | pulang |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | s7 lama G43 R180 | 11:41:15 | **392.21** | 396.9 | 383.6 | ✓ | 0 | 0 | 9.67 `t2_a1_j2` | 5 kaki CLEAR, K0 LULUS |
| 2 | s7 baru G55 R90sel | 11:57:01 | **283.34** | 283.6 | 276.4 | ✓ | 0 | 0 | 11.66 `t2_a1_j2` | 4 kaki CLEAR, K0 LULUS |
| 3 | s10 baru G55 R90sel | 12:19:28 | 5/6 resmi; **317.06** turunan (aturan operator) | 278.5 | 308.9 | 5/6 + ev 12 sampai (verdict TORQUE-ABORT) | 0 | 0 | 12.44 `t2_a1_j2` | lengan REST (retract ev 13 rc 0) |
| 4 | s10 lama G42f G42 | 13:32:36 | **TIDAK SAH 0/6** — move_group mati 13:11:45 | 360.3 | 383.2 | 0/6 HALTED (EXEC-FAIL, nol gerak lengan) | 0 | 0 | 1.24 | rel 3 kaki CLEAR, K0 LULUS |
| 5 | s10 lama G42f G42 (ulang, stack 4, `--tau-max 14`) | 13:49:04 | **391.91** | 360.3 | 383.2 | ✓ | 0 | 0 | 9.39 `t2_a2_j2` | 4 kaki CLEAR, K0 LULUS |
| 6 | s0 baru G55 R90sel | 14:06:13 | **190.70** | 148.2 | 163.4 (G55) | ✓ | 0 | 0 | 11.73 `t2_a1_j2` | 2 kaki CLEAR, K0 LULUS |

- s10 plan-only nyata k = 3 (12:06–12:19): **G55 R90sel 3/3 LOLOS**, **G42f G42 3/3 LOLOS**; DRY run_g54 keduanya rc 0, K0 LULUS,
  traverse di luar []. Traverse: baru g1 → (1.40, −30°), g2 → (0.85, −20°) → (1.20, −10°); lama g1 → (1.25, −30°), g2 → (0.70, −20°)
  → (1.00, −20°) → (1.45, 0°).

- **🔴 s10 baru TIDAK LENGKAP 5/6** (12:19–12:25): ev 12 arm_3 → (1.4286, −0.4588, 1.08) @ g2 (1.20, −10°): probe `PLANNED` →
  `moveit MOVED` → **TORQUE-ABORT 12.44 N·m `t2_a1_joint_2` > `--tau-max` 12.0**; tugas tidak dihitung, runner tidak AUTO-STOP
  (rc 0, rancangan: tugas gagal ≠ stop), retract ev 13 rc 0 → semua lengan REST (K0 di stop: lengan ≤ 0.045°, g1 (1.40, −30),
  g2 (1.20, −10)), 7/7 controller, nol Kortex exception / fault di log.
- Perekam ([ev12](results/p1_g57/hw/p1hw_par_s10_R90sel_ev12_task.log)): puncak terjadi saat **TRANSIT**, bukan di target — j2
  22.7° → 54.9° dalam ~14 s, torsi ~10 ± 1.5 N·m dengan 7 sampel > 12 dalam 7 s (puncak −12.44 di q2 32.6°); di target ditahan
  ~4 N·m. Verdict probe dibuat sesudah gerak (lengan SAMPAI target). Prediksi probe (statis di ujung, RNEA+offset) j2 5.50 →
  13.20/14 = 94 % rating — penyaring menerima karena < rating, padahal batas abort probe 12 → **celah: jadwal G55 tidak memakai
  batas abort 12 untuk torsi TRANSIT** (G27: puncak torsi di tengah jalan, bukan di ujung). Mock tidak mengukur torsi nyata.
  Tidak melanggar aturan keras (> 14). → D533 ✗ (run ini).
- **Keputusan operator 12:33:** "batas 12 N·m kita terlalu ketat; arm masih bekerja → jangan anggap gagal bila hanya lewat 12".
  Kode: `--tau-max` probe hanya MENGAWASI (`collect()`: verdict sesudah gerak, tidak memotong gerak); penyaring rencana sudah
  memakai rating (j2 14, j1/j3 10, pergelangan 7) dan TIDAK berubah. → `run_g54 --tau-max` baru (default 12.0 = argv lama bit demi
  bit; `run_g32.task_cmd(tau_max=12.0)`), HW mulai run berikut **`--tau-max 14`**. Batas keras tetap: penjaga runner AUTO-STOP
  bila torsi terukur > 14 (retract > 13.5), aturan prompt > 14 → BERHENTI.
- **s10 baru di bawah aturan operator:** monitor (`/tmp/p1hw_mon_samples.csv`, pos < 5 mm, ori < 5°) — arm_3 di dalam toleransi
  dari 1791775495.43 dan terus bertahan; jendela 2.0 s penuh 1791775497.43 → t_success ev 12 = **317.06 s** dari t0 = makespan
  turunan (tugas lain terakhir 281.37). Turunan dari monitor yang sama dengan kriteria probe; ditandai "turunan" di tabel.
- Pulang s10 baru (izin 12:33) rc 0: g1 (1.40, −30) → (1.50, 0); g2 (1.20, −10) → (0.85, −20) → (0, 0), +0.15 mm; **K0 LULUS**.

### B4. move_group OOM → s10 lama tidak sah; perbaikan kebocoran + bring-up 4 (13:32–13:50) — [launch 2](results/p1_g57/hw/g57_bringup2_launch.log.gz), [launch 3](results/p1_g57/hw/g57_bringup3_launch.log.gz)

- Operator memberi izin s10 lama 13:32 (jeda ~1 jam sesudah s10 baru). **move_group sudah mati 13:11:45: `exit code -9`** (OOM),
  ~115 mnt sesudah bring-up 2 — kebocoran MoveIt 2.5.10 `forgetShape` dengan 2 octomap updater × `env_static_map` 22 246 kotak
  (memori move-group-hw-memory-leak; perbaikan diusulkan 2026-10-11, BELUM terpasang). Run s10 lama: keenam tugas `EXEC-FAIL`
  (rencana None, ~16 s, nol gerak lengan) → HALTED; traverse rel (JTC langsung) dan retract tetap jalan → 0/6, **tidak sah**.
- **🔴 Celah runner:** `run_g54` tidak memeriksa move_group hidup (K0/DRY lolos tanpa move_group) dan HALTED tidak memicu AUTO-STOP
  → runner terus menggerakkan rel tanpa tugas. Tidak berbahaya di sini (lengan REST, traverse disaring), tetapi run sia-sia.
  Usul (belum dikerjakan): pra-cek proses move_group + AUTO-STOP pada HALTED/EXEC-FAIL.
- **Perekam macet 13:08:49** (proses hidup, futex, nol baris; kemungkinan DDS saat tekanan memori OOM). Run s7/s10 baru + pulang
  (≤ 12:35) terekam; s10 lama tidak. Berkas lama → `/tmp/p1hw_joint_states_part1.csv` (850 MB), perekam baru 13:46.
- Pulang s10 lama (izin 13:42): g1 (1.25, −30) → (1.50, 0); g2 (1.45, 0) → (1.00, −20) → (0.70, −20) → (0, 0); K0 LULUS.
- **Perbaikan (operator 13:42: "pasang arg launch")**: arg `octomap_updaters` (default true = perilaku lama). Bring-up 3 menunjukkan
  move_group TETAP memuat 2 updater: move_group dibuat oleh `move_group.launch.py` paket (di-include `generate_demo_launch`, membangun
  MoveItConfigsBuilder sendiri) → filter dipasang DI SANA (baca `octomap_updaters:=false` dari argv, sama proses `ros2 launch`;
  diuji: param `sensors` hilang hanya dengan flag). HW tidak punya publisher `collision_cloud` → perencanaan tidak berubah; peta
  `env_static_map` tetap di scene.
- **Bring-up 4 (13:47, `octomap_updaters:=false`)**: PID 348058, SigIgn `0x1001001`, 4× '6', 7/7, **table1 + table2 ARMED**,
  0 baris `pointcloud_octomap_updater`, `env_static_map` di scene (False → publish ulang → True, pola sama bring-up 2), K0 LULUS
  ([json](results/p1_g57/hw/k0_bringup4.json)). RSS move_group 639 MB pada 50 s (dipantau).
- s10 lama ulang (izin 13:49) 6/6 **391.91 s**. Tugas arm_3 → (1.4286, −0.4588, 1.08) yang abort di s10 baru di sini j2 puncak
  hanya 6.23 N·m (stop g2 (1.00, −20°) vs (1.20, −10°) — konfigurasi/lintasan lain). DRY ulang di stack 4 rc 0; arsip run tidak sah
  diganti nama `p1hw_par_s10_G42_tidaksah_*`. RSS move_group 639 MB stabil.
- **Minimum paper (s7 + s10) TERCAPAI** ([laporan](results/p1_g57/g57_report.py) → `g57_report.json`): s7 Δ HW **+108.87 s
  (−27.8 %)**, mock +107.24 (HW/mock 1.02), sim +113.29 (0.96); s10 Δ HW **+74.85 s (−19.1 %, s10 baru turunan)**, mock +74.30
  (1.01), sim +81.81 (0.91).
- s0 baru: plan-only 3/3 (14:00–14:05), DRY rc 0, run 6/6 **190.70 s** — HW/sim **1.287**, HW/mock 1.167 (di luar D534/D535).
  Penyebab terlihat: ketiga tugas arm_3 (g2 di (0.15, −30°)) time_to_dwell 31.7 / 47.5 / 54.0 s vs arm_2 24.7–34.2 s; jeda
  rencana → `MOVED` 27 / 43 / 50 s (satu percobaan `PLANNED` masing-masing, τ ≤ 11.73) → durasi eksekusi lintasan panjang
  (sendi besar), bukan penyaringan ulang. Analisis offline (lintasan tersimpan di log probe).
- RSS move_group 889 MB pada 24:50 (≈ +10 MB/mnt, bukan 560 MB/mnt) — perbaikan bekerja.
- **§3 G57 SELESAI 14:13.**

### B5. §4 G58/G60 — pasangan (stack 4, `--tau-max 14`; [prep](results/p1_g57/p1hw_prep.sh) = plan-only k = 3 + DRY)

| # | run | mulai | makespan HW | sim | mock | 6/6 | AUTO-STOP | blok | τ puncak | pulang |
|---|---|---|---|---|---|---|---|---|---|---|
| 7 | s8 baru G55 R90sel | 14:26:28 | **274.47** | 264.4 | 272.0 | ✓ | 0 | 0 | 12.57 `t2_a2_j2` | g1 + g2 home, K0 LULUS |
| 8 | s8 lama G42f G42 | 14:36:28 | **469.93** | 409.4 | 443.9 | ✓ | 0 | 0 | 11.15 `t2_a1_j2` | 6 kaki CLEAR, K0 LULUS |
| 9 | s18 lama G43 R180 | 15:06:05 | **444.24** | 372.2 | 431.1 | ✓ | 0 | 0 | 7.33 `t1_a1_j2` | 5 kaki CLEAR (1 ganda dilewati), K0 LULUS |
| 10 | s18 G60 | 15:20:09 | **261.17** | 232.2 | 257.5 | ✓ | 0 | 0 | 12.41 `t2_a2_j2` | 2 kaki CLEAR, K0 LULUS |
| 11 | s3 G60 | 15:41:25 | **323.95** | 295.8 | 305.5 | ✓ | 0 | 0 | **13.22** `t2_a2_j2` (94 %) | 4 kaki CLEAR, K0 LULUS |

- Prep s8 (14:12–14:26): baru 3/3 + DRY rc 0; lama G42f 3/3 + DRY rc 0.
- s8 baru τ puncak 12.57 > 12 (< 14) — dicatat, bukan gagal (keputusan operator 12:33).
- Pulang s8 baru: rute g2 `0.0:0.0 0.0:0.0` (stop pertama jadwal = p0 → kaki ganda); kaki ke-2 = gerak nol → `pose_to_g` rc 3,
  skrip berhenti sebelum K0 (g2 sudah 0.87 mm). K0 manual LULUS. `p1hw_home.sh` kini melewati kaki yang sama dengan kaki
  sebelumnya.

- s8 pasangan: Δ HW **+195.46 s (−41.6 %)**, mock +171.86 (HW/mock 1.14), sim +144.98 (HW/sim 1.35); Δtraverse +117.1 s
  (lama 6 traverse vs baru 2), Δbersamaan +78.6 s.
- Prep s18 (14:52–15:05): lama G43 R180 3/3 + DRY rc 0; G60 3/3 + DRY rc 0. s18 pasangan: Δ HW **+183.07 s (−41.2 %)**, mock +173.56
  (HW/mock 1.05), sim +139.95 (HW/sim 1.31); Δtraverse +75.7 s. Stack 4 sesudah 101 mnt: RSS move_group 1.05 GB, perekam hidup.
- Prep s3 (15:28–15:41): G60 3/3 + DRY rc 0; lama G43 R180 3/3 + DRY rc 0. s3 G60 τ puncak **13.22 N·m** (ev 10 arm_4 j2, 94 %
  rating, tertinggi hari ini; prediksi probe RNEA+offset 12.91/14 → galat +0.31) — < 14, dilaporkan ke operator.

