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

## B. Hasil terukur

### B1. Rehearsal sweep mock — [g1](results/p1_g48/g48_mock_sweep_g1.log), [g2](results/p1_g48/g48_mock_sweep_g2.log)

- **42/42 kaki rc 0** (g1 28: stasiun (0.65, 0), ±10/±20/±35/±50/±70, −90/−110/−125, PARK; g2 14: (0.15, 0), ±10/±20/±35,
  (0, 0)); galat mock 0, drift 0 → D400 ✓. 19:57–20:09.
- G33 kaki −125: log `pose_to_g` terpotong oleh `tail -3` saya (penyimpangan, ditulis) → dihitung ulang dengan fungsi sapuan
  yang sama (`_edge_job`, persegi 10 mm × 1°): **CLEAR 93.9 mm** (`t1_a2_right_finger_dist_link`), +70: 94.2 mm → D401 ✓.
- Baseline cek origin g2 dari data G34 (offline, [JSON](results/p1_g48/g2_origin_baseline.json)): rgbd tx −0.0916 / −0.0955
  m di g2 = 0 (keterulangan ≈ 4 mm, 800 titik); rgbd2 fit satu lengan, yaw 15–17° → tidak dipakai. Slip 35 mm terdeteksi.

### B2. Konfigurasi rotasi jalur HW (A0) — tidak ada perubahan yang diperlukan

URDF ±π, bridge ±180 (default, tidak di-override), 10 °/s, pulse absolut ±18 000, `pose_to_g`/`run_g32 --rot-max 125`
eksplisit (default tetap 10). ROT_NEED (g1 −125…+70, g2 ±35) DIDUKUNG tanpa mengubah konfigurasi HW.

### B3. Risiko terbuka (untuk operator, Gerbang 5)

| # | Risiko | Penanganan |
|---|---|---|
| R1 | Rotasi HW terverifikasi hanya ±10° (G30) | sweep 6b bertahap, tiap kaki ditanya |
| R2 | Origin g2 ≈ −35 mm (G36b) | cek kamera 6a (operator di lokasi 2026-10-11) |
| R3 | Model waktu rotasi `rot_cmd` hanya terukur di 10° | diukur di 6b; dampak makespan ≤ 12.5 s |
| R4 | Retract MoveIt (cadangan) belum pernah di HW | semua retract kandidat lurus di DRY-CLI/smoke |
| R5 | Model jari netral di EnvChecker (gripper terbuka menonjol) | START-JUMP memakai jari HW nyata di MoveIt; K0 jari ≤ 0.02 rad |
| R6 | Pulang sesudah stop terakhir di luar jadwal | `g47_home.sh` retrace (diuji mock) |
| R7 | s8 memakai jadwal G42f (R180 s8 gagal (iv′)) | makespan sama 315.89 s |
| R8 | 14 seed G43 sisa + diagnosa belum dihitung | tidak diperlukan untuk HW; jalan malam |
| R9 | Tujuan JTC lengan-saja saat perintah rel basi (G36b, belum diuji) | runbook: `/table_hw_commands` harus diam sebelum gerak |

Papan skor: D400 ✓, D401 ✓ → **2/2**.
