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
