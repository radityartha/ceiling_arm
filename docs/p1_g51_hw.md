# P1 / G51 — Fase 6 HW nyata: 6a origin g2 → 6b sweep rotasi v2 → 6c DRY → 6d blok seed

> Sesi 2026-10-11, operator di lokasi. Runbook `docs/p1_hw_runbook.md`. Gerbang 5 LULUS (R1–R9 diterima). **SETIAP gerak
> nyata ditanya dulu (satu langkah per persetujuan).** **§A DIKUNCI SEBELUM menyentuh HW** (salinan
> `docs/results/p1_g51/g51_A_locked.md` + sha256). §B menang atas §A. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Pengungkapan

- `remount_check` (sebelum mengunci): GERBANG LULUS — 192.168.2.100/24, ICMP .10–.13, web UI tidak dipakai, tanpa
  move_group / ros2_control_node / realsense / monitor basi, ttyUSB0/1 ada, kamera RealSense terdeteksi.
- Rotasi HW terverifikasi ±10° (G30) = `results/p1_g48/rot_envelope.json`. ROT_NEED 17 jadwal: g1 −125…+90, g2 −35…+95.
- Origin g2: G36b membaca −34.9 mm / +2.91° lalu didorong ke encoder 0; operator melihat ~35 mm dari stopper. Baseline kamera
  (G48, data G34): rgbd tx g2 −0.0916 / −0.0955 m (keterulangan ≈ 4 mm).

### A1. 🔒 Urutan + gerbang (runbook §1–§6)

- **6a** bring-up nyata (runbook §1) → monitor + perekam → K0 `--rail1 1.50 --rail2 0.00` (TOLAK → tidak lanjut, pemulihan
  ditanya) → kamera + `capture_cloud --joints --seconds 20` → `rail_calib --groups g2 --diff` vs cal_00/cal_07 (rgbd):
  Δx = tx − (−0.0936): |Δx| ≤ 10 mm → origin OK; ≥ 25 mm → slip (re-zero hanya bersama operator); di antaranya pita ukur.
- **6b** `g50_sweep.sh` per kaki (DRY lalu `--move` sesudah persetujuan); tiap kaki: |err_deg| ≤ 0.3, |err_mm| ≤ 2,
  t_traverse ≤ 1.2 T_cmd, drift ≤ 0.5°, rel/rot lain diam, torsi ≤ 14, operator cek kabel. Amplitudo lulus →
  `rot_envelope.py widen`. Berhenti pada pelanggaran mana pun.
- **6c** `run_g32 --dry` (stack nyata) untuk seed yang rotasinya sudah terverifikasi.
- **6d** plan-only NYATA k = 3 → `run_g32 --final-retract --goal-tol-deg 0.05` → `g47_home.sh` → K0; pembanding R0
  sama-sesi bila waktu cukup.

### A2. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D430 | K0 bring-up LULUS tanpa pemulihan (lengan REST, g1 1.50, g2 0.00 ±2 mm, rot ±0.5°). |
| D431 | Origin g2: \|Δx\| ≤ 10 mm (stopper di luar URDF 0, BUKAN slip). |
| D432 | Sweep: semua kaki sampai ROT_NEED lulus (err ≤ 0.3°). |
| D433 | t_rot / T_cmd_rot pada Δ ≥ 50° = 0.85–1.0 (model rot_cmd 0.9235 berlaku di Δ besar). |
| D434 | 6d: seed pertama 6/6 SUCCESS penilai, 0 HALTED. |

## B. Hasil terukur

### B0. Bring-up 1 (13:21–13:30, sesi sebelum pindah ke sesi chat baru) — NOL gerak — [launch](results/p1_g51/hw/g51_bringup1_launch.log.gz), [K0](results/p1_g51/hw/k0_bringup_probe.json)

- Gerbang bring-up: 4× `Actuator count … '6'`, 7/7 controller active, **table1 ARMED** di 1499.9 mm / 0.0°; **table2 NOT armed**
  (perintah 0.0 mm ≠ terukur **−18.6 mm / −0.2°** → bridge menolak, sesuai rancangan). Galat log yang dikenal: KDL "not a chain"
  untuk `gantry_*_with_arm`, `joint_state_broadcaster` already loaded, `/recognize_objects` tidak ada.
- **K0 TOLAK** (D430 ✗): g2 rel **−18.58 mm** (harap 0.00 ±2), rot −0.16°; arm_4 maks |q − REST| **0.573°** (joint_5, batas 0.5);
  arm_1 0.257, arm_2 0.203, arm_3 0.455°; g1 1.49986 m / 0.0°; jari ≈ 0.
- Operator memilih melanjutkan di sesi chat baru → stack dimatikan bersih (SIGINT PID launch asli, SigIgn `0x1001001`, node 0;
  crash dump shutdown rviz2/move_group sesi ini dihapus).
- **Untuk sesi berikut:** g2 −18.6 mm di encoder = data baru untuk 6a (G36b: −34.9 mm lalu didorong ke 0 → kini −18.6 tanpa
  gerak perangkat lunak yang diketahui) → kamera DULU sebelum pemulihan; pemulihan K0 (`return_rest` arm_4, `pose_to_g` g2 →
  0) hanya sesudah keputusan origin dan persetujuan operator. table2 hanya ARMED bila perintah = terukur.
