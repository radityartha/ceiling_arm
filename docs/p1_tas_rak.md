# P1 / TAS-RAK — arm ke ATAS tas kertas di lemari (sisi gantry 2) — LURING/MOCK saja, NOL gerak HW

> 2026-10-17 11:00–11:45. Operator: "lanjutkan gerak arm ke bag di rak"; izinkan rencana gerak gantry 2 (mock dulu); papan sudah
> dilepas dari semua lengan (operator). Alat: [results/p1_tas_rak/](results/p1_tas_rak/). §A belum dibuat — gerbang luring tidak lulus.

**Siapa yang memilih:** kandidat stop g2 (rel, rot) + lengan = **dipilih manual (Claude), grid luring** (`cand_geom.py`,
`scan_rest_env.py`, daftar di `tas_rak_plan.json` hanya untuk S13 pose_to_g) — **BUKAN penjadwal P1** (oracle/P1″, G40 env-aware).
Penyaring (S28, G33, torsi RNEA, antar-lengan, retract, START-JUMP) = sistem, tidak diubah.

## Data
- Tas (DEMO-ZONE-HW B5, prompt `paper bag`): rgbd (1.001, −0.807, 1.094) conf 0.29, rgbd2 (0.956, −0.855, 1.076) conf 0.26.
  Target uji (0.978, −0.831, z_top + 0.25 = 1.34; juga 1.40).
- Basis (URDF, g2 0/0): arm_3 (0.40, −0.36) 0.75 m dari tas, arm_4 (−0.40, −0.36) 1.47 m; arm_1/arm_2 ≥ 1.19 m. Atas lemari di peta
  env ≈ 0.95–1.10 m; **REST menggantung sampai z ≈ 0.96 → REST di atas lemari menabrak**.
- Rotasi g2 positif membawa arm_4 ke −y (amplop terverifikasi −35…+95°).

## Mock 77 (fake HW, BUKAN data), g1 1.40, lengan REST, tiap kandidat dari g2 (0, 0)
**Ronde 1** (basis 7–27 cm dari tas): (1.00, 90) (1.10, 70) (1.15, 60) (1.20, 50) (1.25, 40) (1.30, 30) → **6/6 REFUSE G33**
(jari arm_4 REST menembus peta −18…−20 mm; S28 CLEAR semua).
**Pindai statis** (`scan_rest_env.json`, 891 stop): 604 REST CLEAR; terdekat basis lengan **0.29–0.33 m** dari tas (jari 52–86 mm).
**Ronde 2** (8 stop dari pindai):

| g2 (rel, rot) | lengan | sapuan S28 / G33 | plan-check z 1.34 | z 1.40 |
|---|---|---|---|---|
| 0.70, +85 | arm_4 | CLEAR / CLEAR 85.7 mm | TORQUE-UNSAFE (j2 14.26–15.55 / 14) | RETRACT-BLOCKED |
| 0.85, +65 | arm_4 | CLEAR / CLEAR 75.8 | RETRACT-BLOCKED | TORQUE-UNSAFE (j2 ≤ 16.71) |
| 0.65, −25 | arm_3 | CLEAR / **MARGIN 24.5 → REFUSE** | — | — |
| 0.70, +90 | arm_4 | CLEAR / CLEAR 54.2 | START-JUMP | START-JUMP |
| 0.80, +75 | arm_4 | CLEAR / CLEAR 56.6 | START-JUMP | START-JUMP |
| 0.75, +80 | arm_4 | CLEAR / CLEAR 70.8 | NO-PLAN | RETRACT-BLOCKED |
| 0.70, −25 | arm_3 | CLEAR / **MARGIN 24.5 → REFUSE** | — | — |
| 1.00, +45 | arm_4 | CLEAR / CLEAR 51.6 | START-JUMP | START-JUMP |

**Hasil: 0/14 kandidat menghasilkan rencana sah → NOL gerak HW.** START-JUMP = REST di stop itu sudah bertabrakan dengan scene
MoveIt (kotak peta +5 cm) walau G33 (jarak titik, margin 50 mm) CLEAR — dua penyaring env memakai geometri berbeda.
Penyebab fisik: tas di lemari tinggi (puncak ~1.09) + lengan gantung (REST z 0.96) → basis harus ≥ 0.3 m dari tas → jangkauan samping
0.3 m pada z 1.34 membebani j2 > 14 (memori fold-torque) atau jalur pulang terhalang.

## Pilihan berikut (keputusan operator)
1. **Tugas tas ke penjadwal P1** (sistem yang dirancang: oracle/P1″ env-aware + rotasi + torsi) luring → jawaban resmi feasible/tidak.
2. Pindahkan tas ke tempat terjangkau (meja demo arm_1 → jalur A4 DEMO-ZONE-HW, sudah dikunci).
3. Pose simpan TINGGI (memori stow-pose-high) agar lengan bisa melintas di atas lemari — butuh fase sendiri (S12 pose_to_g = REST).

Kode siap (belum dipakai HW): `demo_runner.py` BASE + arm_3/arm_4 (default tetap arm_1 arm_2), `rack_box_node.py` (uji unit lulus:
tas rak diterima, tas lantai + botol ditolak), harness `results/p1_tas_rak/harness/` (fusi rgbd+rgbd2, domain 0).
