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
