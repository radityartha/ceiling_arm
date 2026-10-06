# Sesi G28-YAML — `joint_limits.yaml` ≤ URDF vendor, PLAN-ONLY, stack NYATA (2026-09-28)

Gerbang G28 B0b / C sebelum eksekusi rencana lengan apa pun. Prompt: [p1_g28_hw.md §F](p1_g28_hw.md).
Hasil: [results/p1_g28y/](results/p1_g28y/).

## A. Protokol — ditulis SEBELUM `joint_limits.yaml` diubah, SEBELUM data SESUDAH

### A0. Keputusan operator (dijawab sebelum §A ini ditulis)

| Gerbang | Jawaban |
|---|---|
| (a) stack | **NYATA** — data SEBELUM = G28-ON (stack nyata yang sama, start REST nyata, R1 0/0); hanya SESUDAH diukur |
| (b) izin ubah yaml | **ya** — batas **posisi** saja; kecepatan/akselerasi tidak disentuh; sendi lain yang longgar ikut |
| (c) nilai | **URDF persis** (sama dengan oracle‴) |

### A1. Instrumen — 24 sendi lengan ([limits_table.py](results/p1_g28y/limits_table.py) → [limits_before.log](results/p1_g28y/limits_before.log))

URDF = cache hidup `/tmp/reach_dwell_live.urdf` (sha `f02e7c532cdb`, = KD2); efektif MoveIt = yaml bila
`has_position_limits`, selain itu URDF.

| sendi | URDF | yaml | ros2_control (`kortex.ros2_control.xacro`) |
|---|---|---|---|
| `t1_a1_joint_2`, `t2_a1_joint_2`, `t2_a2_joint_2` | ±2.61 | **±2.76** LONGGAR | −2.69 / **+2.36** |
| `t1_a1_joint_5` | ±2.53 | **±2.70** LONGGAR | ±2.57 |
| `t2_a1_joint_3`, `t2_a2_joint_3` | ±2.61 | **±2.85** LONGGAR | ±2.69 |
| 18 sendi lain (j1 ±2.68, j2/j3 ±2.61, j4/j6 ±2.60, j5 ±2.53) | URDF | — (= URDF) | j1 ±2.69, j2 −2.69/+2.36, j3 ±2.69, j4/j6 ±2.59, j5 ±2.57 |

**LONGGAR 6 / 24** = persis 6 sendi B0b. Tidak ada sendi lain yang perlu diubah.
Catatan (tidak diubah, di luar izin (c)): batas perintah ros2_control berbeda dari URDF, **lebih ketat** pada
j2 atas (+2.36 < 2.61) dan j4/j6 (2.59 < 2.60). Sesudah perbaikan, MoveIt boleh merencanakan j2 ∈ (2.36, 2.61]
yang lalu dijepit/ditolak oleh lapisan perintah → dilaporkan ke operator, bukan diperbaiki di sini.

Pemuatan: `my_workcell.launch.py` → `MoveItConfigsBuilder(...).to_moveit_configs()` memuat
`config/joint_limits.yaml` ke `robot_description_planning`; file di `install/` = symlink ke `src/` → **rebuild tidak perlu**,
cukup launch ulang. Kontrol runtime KY1: `ros2 param get /move_group robot_description_planning.joint_limits.<j>.max_position`
harus = URDF untuk 6 sendi sebelum saringan SESUDAH dijalankan.

### A2. SEBELUM = data G28-ON (tidak diukur ulang)

| set | rencana | verdict | pelanggaran URDF | wall per rencana |
|---|---|---|---|---|
| V28 z = 1.32 (20 × 3) | 60 | PLANNED 60 | 0 / 60 | median 24.19 s, Σ 1463.8 s |
| V28 z = 1.40 (41 × 3) | 123 | PLANNED 21, TORQUE-UNSAFE 102, NO-PLAN 0 | PLANNED 20/21 (j5 arm_1, j3 arm_3); TORQUE 20/102 (j3) | median 0.08 s, Σ 506.0 s |
| (iv) 14 seed G28 | 252 tugas | PLANNED 252; 14/14 seed LOLOS | tidak diukur (tanpa lintasan) | dari log |

Tuple 5 dan 40 (arm_1, dua salah A4, SAFE) = 3/3 PLANNED, **semuanya** lewat j5 > 2.53.

### A3. Langkah SESUDAH

1. Ubah 6 angka yaml ke URDF; `limits_table.py` → LONGGAR 0/24.
2. Tahap 0 g22 A5 (operator ditanya ulang), bring-up `use_fake_hardware:=false enable_gantry_bridge:=true`, 7/7 controller,
   R1 dari `/joint_states`. **KY1** (param runtime). DRY V28 (KD1–KD3 + K-RNEA off).
3. `v28_screen.py --r1 <R1> --out ../p1_g28y/v28_after_plans.jsonl` (61 × 3; z 1.32 + z 1.40).
4. `sched_screen.py` seed G28 (14) → `p1_g28y/g28y_screen.json`.
5. `v28_score.py --in … --screen … --out p1_g28y/v28_after_score.json`; perbandingan sebelum/sesudah per z.

### A4. 🔒 Dugaan D167–D174 (prior tally: mekanisme terukur → tepat; aturan/kode sendiri → meleset)

| # | Dugaan | Dasar |
|---|---|---|
| D167 | **KONTROL**: `urdf_viol` = 0 pada **semua** rencana SESUDAH ber-lintasan (semua z, semua verdict) | mekanisme B0b terukur |
| D168 | z 1.32: PLANNED ≥ 57 / 60 (NO-PLAN + TORQUE ≤ 3) | SEBELUM 0/60 menyentuh batas |
| D169 | z 1.32: median wall per rencana SESUDAH dalam ±25 % dari 24.19 s | batas tidak aktif di z 1.32 |
| D170 | (iv): ≥ 13 / 14 seed LOLOS; TORQUE-UNSAFE per tugas = 0 | lapis tugas = z ≤ 1.32 |
| D171 | z 1.40: PLANNED ≤ 3 / 123 | 20/21 PLANNED SEBELUM ilegal; sisanya tuple 4 arm_2 |
| D172 | tuple 5 **dan** 40 menjadi UNSAFE (≥ 1/3 TORQUE) → A4 tolak ⇒ UNSAFE 41/41 | oracle‴: tak ada cabang j2-rendah dalam URDF (B0b) |
| D173 | z 1.40: NO-PLAN ≤ 12 / 123 (≤ 10 %) — perencana menemukan cabang lain (TORQUE), bukan gagal | 102/123 sudah TORQUE; cabang sah ada |
| D174 | Σ wall z 1.40 SESUDAH ≤ 506 s × 1.5 | TORQUE ditolak dini (median 0.08 s) |

Aturan: B menang atas A dan DITULIS; nol gerak; §A ini tidak diubah sesudah kunci.
