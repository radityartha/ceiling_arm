# P1 / G48 — paket SIAP HW (Fase 5 kampanye)

> Kampanye Fase 5. NOL gerak HW; mock `ROS_DOMAIN_ID=77` (fake HW) hanya untuk rehearsal alat sweep. **§A DIKUNCI SEBELUM
> mengukur** (salinan `docs/results/p1_g48/g48_A_locked.md` + sha256). §B menang atas §A. Runbook: `docs/p1_hw_runbook.md`.

## A. Protokol — DIKUNCI

### A0. Pengungkapan

- Fase 1–4 LULUS (G43+G46, G44, G45, G47). ROT_NEED dari 6 kandidat: **g1 −125…+70°, g2 −30…+35°** (|rot| 125, Δrot 95).
- Jalur rotasi HW (dibaca): URDF `t*_rotation_joint` ±π; `joint_limits.yaml` hanya kecepatan 1.0 rad/s; bridge
  `bridge.min_deg/max_deg` default ±180 (tidak di-override di launch), `rotate_speed` 1000 pulse/s = 10 °/s, `rot_tol_deg`
  0.3, `arm_tol_deg` 1.0; `moving_table.go_to_absolute` pulse absolut `int(deg × 100)` (±18 000), "sudah di target" ≤ 50
  pulse = 0.5°. `pose_to_g --rot-max` (G47) default 10; `run_g32 --rot-max`. `rot_to_g` (G30) TIDAK dipakai (tanpa sapuan peta
  G33). Operator 2026-10-10: ±180° mekanis OK, kedua arah aman.
- Stasiun sweep (EdgeMatrix G46): g1 lin 0.65 (sapuan 0 → −125 dan 0 → +70 CLEAR, env min 93.9 mm), g2 lin 0.15 (±35, min
  301 mm).

### A1. 🔒 Isi

1. `docs/p1_hw_runbook.md` — bring-up, K0, monitor + perekam, cek origin g2 (kamera), sweep rotasi bertahap, blok seed, pulang,
   matikan; perintah persis.
2. Sweep: `results/p1_g48/g48_sweep_plan.json` (seed 48, varian SWEEP; stop g1 (0.65, 0/±10/±20/±35/±50/±70/−90/−110/−125),
   g2 (0.15, 0/±10/±20/±35)) + `g48_sweep_legs.json` (g1 28 kaki, g2 14 kaki: stasiun, 0 → +A → 0 → −A → 0 tiap amplitudo,
   negatif saja di atas 70 untuk g1, lalu PARK/p0) + `g48_sweep.sh` (satu kaki per panggilan di HW).
3. **Rehearsal mock** seluruh kaki g1 lalu g2 (`--move`, fake HW): setiap kaki rc 0 (S12, S13″, S23″ dengan `--rot-max 125`,
   G33 sapuan peta, S28 lintas-gantry). Mock tidak menilai waktu/galat HW.
4. Daftar risiko terbuka → tanya operator (Gerbang 5).

### A2. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D400 | Rehearsal mock: **42/42** kaki rc 0 (tidak ada REFUSE). |
| D401 | Sapuan G33 min di kaki g1 −125 ≥ 80 mm (EdgeMatrix 93.9 mm di lin 0.65). |
