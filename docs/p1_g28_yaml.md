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

---
§A dikunci **13:43**, sha `31b22f785074` (salinan: [sectionA_locked.md](results/p1_g28y/sectionA_locked.md)). Di bawah = sesudah kunci.

## B. Hasil terukur

### B0. Perubahan + tahap 0 (2026-09-28, sel NYATA, operator di lokasi)

| Langkah | Hasil |
|---|---|
| yaml | 6 sendi → URDF persis (12 baris; salinan lama [joint_limits_before.yaml](results/p1_g28y/joint_limits_before.yaml)); `limits_table.py` → **LONGGAR 0/24** ([log](results/p1_g28y/limits_after.log)). Rebuild tidak dijalankan (symlink) |
| operator (ditanya ULANG) | LED 4/4 normal; origin g1 + g2 = home; sel kosong |
| `remount_check.py` + ICMP | **GERBANG LULUS**; .10–.13 ✅; nol stack sisa; enp112s0 = 192.168.2.100 |
| bring-up `use_fake_hardware:=false enable_gantry_bridge:=true use_sim_time:=false` | launch PID **1387985** (pgrep sesudah controller naik, PPID 1), SigIgn `0x1001001` (SIGHUP dari nohup; SIGINT tidak diabaikan); **4×** "Actuator count … '6'"; **7/7** controller; 7 spawner "process has died" + octomap gagal = baseline; nol fault ([log](results/p1_g28y/g28y_launch.log.gz)) |
| **KY1** `ros2 param get /move_group robot_description_planning.joint_limits.<j>.max_position` | **6/6 = URDF** (2.61 ×5, 2.53) ✅ |
| `/joint_states` (40 pesan) | rel **0.000712 / 0.000681** → **R1 = 0.00 / 0.00** (= G28-ON, jadi SEBELUM sah); rot 0.000° / −0.290° |
| DRY ([log](results/p1_g28y/v28_dry_after.log)) | KD1 ✅ KD2 ✅ (`f02e7c532cdb`) KD3 ✅; K-RNEA off 200/200 bit-identik |

**Nol gerak** lengan / rel / rotasi sepanjang sesi (plan-only).

### B1. SEBELUM vs SESUDAH ([compare.py](results/p1_g28y/compare.py) → [log](results/p1_g28y/compare.log))

V28 SESUDAH: [screen log](results/p1_g28y/v28_after_screen.log), [plans](results/p1_g28y/v28_after_plans.jsonl.gz),
[score log](results/p1_g28y/v28_after_score.log). (iv) SESUDAH: [log](results/p1_g28y/g28y_screen.log), [json](results/p1_g28y/g28y_screen.json).

| set | SEBELUM (G28-ON, yaml lama) | SESUDAH (yaml = URDF) |
|---|---|---|
| V28 z = 1.32 (60) | PLANNED **60**, NO-PLAN 0, TORQUE 0; wall median 24.19 s, Σ 1463.8 s | PLANNED **60**, NO-PLAN 0, TORQUE 0; wall median **24.93 s (+3.1 %)**, Σ 1470.4 s |
| V28 z = 1.40 (123) | PLANNED 21 (**20 ilegal**), TORQUE 102, NO-PLAN 0; Σ 506.0 s | PLANNED **5 (0 ilegal)**, TORQUE **118**, NO-PLAN **0**; Σ 135.2 s |
| (iv) 14 seed (252 tugas) | 14/14 LOLOS; PLANNED 252; median 23.70 s, Σ 5635.7 s | **14/14 LOLOS**; PLANNED **252**; median 23.40 s, Σ 5532.7 s |
| pelanggaran batas URDF (lintasan) | z1.32 0/60; z1.40 PLANNED 20/21, TORQUE 20/102 | **0 / 183** (semua z, semua verdict) |
| pelanggaran batas perintah ros2_control (informatif) | — | **0 / 183** (j2 +2.36 tidak pernah tercapai) |

**A4 G27 (ketat) SESUDAH: tolak ⇒ UNSAFE 41/41, terima ⇒ SAFE 20/20** — dua kesalahan G28-ON (tuple 5, 40) hilang;
keduanya kini 3/3 TORQUE-UNSAFE. "SAFE" z = 1.40 di G28-ON **seluruhnya** artefak batas yaml.

5 PLANNED z = 1.40 SESUDAH: tuple 14 s2, 19 s0, 19 s2, 30 s0 (arm_3), 28 s0 (arm_1); statis-akhir j2 0.80–1.19;
**semuanya sah URDF**. Cabang j2-rendah **sah** ada pada 4 tuple (bukan hanya tuple 4 arm_2 G28-ON), tetapi setiap tuple itu
tetap UNSAFE (sampel lain TORQUE) — perencana memilih cabang secara acak.

### B2. Papan skor D167–D174 — **7 / 8 tepat**

| # | Dugaan | Terukur | |
|---|---|---|---|
| D167 | KONTROL urdf_viol = 0 | 0 / 183 | ✅ |
| D168 | z1.32 PLANNED ≥ 57/60 | 60/60 | ✅ |
| D169 | z1.32 median wall ±25 % | +3.1 % | ✅ |
| D170 | (iv) ≥ 13/14, TORQUE 0 | 14/14, 0 | ✅ |
| D171 | z1.40 PLANNED ≤ 3/123 | **5** | ❌ |
| D172 | tuple 5 + 40 → UNSAFE, A4 41/41 | keduanya 3/3 TORQUE; 41/41 | ✅ |
| D173 | z1.40 NO-PLAN ≤ 12 | 0 | ✅ |
| D174 | Σ wall z1.40 ≤ 759 s | 135 s | ✅ |

D171 meleset: saya menurunkan "≤ 3" dari **satu** tuple dev (B0b, oracle‴: tak ada cabang j2-rendah sah) dan menganggap
tuple 4 arm_2 satu-satunya pengecualian; terukur, cabang sah itu ada pada 4 tuple lain. Pola tally tetap: tujuh yang tepat
dari mekanisme terukur (B0b, data G28-ON); yang meleset = generalisasi dari n = 1.

### B3. Pertentangan §B lawan §A / dokumen lama

1. **D134 G28** (pita laju TORQUE z1.40 [0.55, 0.85], dinilai ✅ 0.829 di G28-ON) jatuh ke **0.959** dengan yaml = URDF —
   pita itu terkalibrasi pada perencana yang boleh keluar batas. Skor G28 **tidak** diubah (dinilai pada stack saat itu);
   dicatat di sini.
2. B0b "tidak ada cabang j2-rendah dalam batas URDF" berlaku untuk tuple dev (a) saja, **bukan** z = 1.40 umum (B1).
   Konsekuensi untuk opsi "oracle per-cabang + kendala cabang tujuan" (G28 C-ON): nilainya **lebih besar** dari perkiraan
   G28 (arm_2 1/30) — 4/41 tuple punya cabang sah; tetap keputusan operator, tidak dikerjakan.

## C. Keadaan akhir (Rule 12) — 📣 LAPORAN KE OPERATOR (permintaan G28 C, ukuran (2))

**yaml ≤ URDF TIDAK membuat gerak sering gagal.** Pada set yang sama di stack nyata yang sama:

| z | PLANNED sebelum → sesudah | NO-PLAN | waktu rencana (median) |
|---|---|---|---|
| **1.32** (lapis tugas) | 60/60 → **60/60** | 0 → 0 | 24.19 → 24.93 s (+3 %, derau) |
| (iv) 14 seed | 252/252 → **252/252**; seed 14/14 → **14/14** | 0 → 0 | 23.70 → 23.40 s |
| **1.40** (oracle⁗ sudah menolak) | 21 → 5 (yang hilang 20 = ilegal; 4 baru sah) | 0 → 0 | 0.08 → 0.08 s |

- **Gerbang G28 B0b/C TERTUTUP**: `joint_limits.yaml` 6 sendi = URDF, dipasang di `src/` (dipakai lewat symlink), KY1
  terverifikasi di runtime. Eksekusi rencana lengan tidak lagi terhalang gerbang ini.
- **Tidak diubah:** kecepatan/akselerasi yaml, URDF, batas ros2_control, probe, `sched_screen`, oracle, §A G28/G28-YAML.
- 🔶 **Terbuka (informatif, di luar izin (c))**: batas perintah ros2_control Kortex (`kortex.ros2_control.xacro`) lebih ketat
  dari URDF — j2 atas **+2.36** (URDF 2.61), j4/j6 ±2.59 (2.60). Sekarang MoveIt boleh merencanakan j2 ∈ (2.36, 2.61].
  Terukur 0/183 rencana menyentuhnya; pada eksekusi, rencana seperti itu akan dijepit/ditolak di lapisan perintah. Keputusan
  operator bila perlu (mis. yaml j2 max 2.36).
- **Stack mati bersih**: SIGINT ke 1387985 → keluar **12 s**; `ros2 node list --no-daemon` **0**; nol proses ROS sisa.
  Crash dump `rviz2` 16:11 (139 MB, milik sesi) **dihapus**; dump python 09-21 (bukan milik sesi) dibiarkan.
  `v28_after_plans.jsonl` 15 MB → gzip 5.9 MB. Disk akhir **1.5 GB**.
- **Keadaan sel**: tidak berubah (rel ≈ 0.7 mm, rot 0.00 / −0.29°, lengan REST/bring-up).
- Insiden kecil (tanpa efek): penanti `pgrep -f sched_screen.py` mencocokkan shell-nya sendiri dan `pkill` pembersihnya
  membunuh shell pemanggil; hanya shell penanti — proses saringan sudah selesai rc 0 sebelumnya.

## D. Berikutnya: G31 = prompt [p1_g30_rot_hw.md §D](p1_g30_rot_hw.md), dengan dua koreksi

- **G1 terjawab**: G28-ON (G28 §B) dan G28-YAML (dokumen ini) sudah selesai → G31 langsung; tanya hanya G2, G3.
- Dugaan G31 mulai **D175** (bukan D167). Prior tally G28-YAML: mekanisme terukur tepat 7/7; generalisasi dari n = 1 meleset.

**Rekomendasi: Opus, effort TINGGI** (tetap seperti g30 §D — patch penyaring bersama, galat diam dan satu arah).
