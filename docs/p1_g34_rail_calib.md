# P1 / G34 — KALIBRASI REL (kamera ↔ URDF), peta gabungan, retract sadar-lingkungan, HW ulang R10

> Sesi G34, 2026-10-07. Lanjutan [p1_g33_map.md](p1_g33_map.md) (B2 offset kamera↔URDF ~0.1 m, B9 offset bergantung
> posisi gantry, B6 R10 ditolak di ev 11 retract).
> **§A ditulis dan DIKUNCI SEBELUM kamera dinyalakan / gantry digerakkan.** §B diisi sesudah. §B menang atas §A; konflik DITULIS.

## A0. Gerbang (jawaban operator 2026-10-07, sebelum apa pun)

| | Jawaban |
|---|---|
| K0 sel sejak akhir G33 | **sama** (4 lengan REST, rel ~0/0, rot 0/0, rak tidak dipindah, benda lain tidak berubah) |
| K1 octomap hidup di HW | **tidak — peta beku saja** di G34 (awan hidup belum diregistrasi; offset bergantung posisi gantry) |
| K2 gerak gantry (lengan REST) untuk tangkapan kalibrasi | **izin, operator di e-stop** |

Tahap 0 (`remount_check.py`, read-only): ICMP .10–.13 4/4, nol proses basi, 2× D455 terdeteksi → LULUS.

## A0.1 Analisis offline A/B (data G33, sebelum tangkapan baru) — FAKTA

Registrasi 4-DOF **per gantry** (`p1_g34/rail_calib.py`, [json](results/p1_g34/rail_calib_ab.json)):

| tangkap | kamera | g1 t / yaw | g2 t / yaw |
|---|---|---|---|
| A (0, 0) | rgbd | (−0.068, +0.047, **+0.014**) / +1.17° | (−0.074, +0.103, **+0.004**) / −0.55° |
| B (0.90, 1.45) | rgbd | (−0.136, +0.120, **+0.118**) / −5.35° | (satu lengan) |
| B (0.90, 1.45) | rgbd2 | (−0.081, +0.132, **+0.096**) / −4.20° | (−0.108, +0.131, **+0.021**) / −1.50° |

- **tz g1 @ 0.90 ≈ +0.10 m di KEDUA kamera**, g2 @ 1.45 ≈ +0.02–0.05, keduanya @ 0 ≈ 0.01. Kedua kamera sepakat →
  bukan derau satu kamera.
- Yaw 4-DOF diputar di titik asal dunia → t dan yaw berkopel; yaw dari satu lengan tak terdefinisi (rgbd2 A g2 +19°).
  Angka t per baris **tidak** bisa dibandingkan langsung — maka uji 6-DOF gabungan (A2).

## A. Protokol — DIKUNCI

### A1. 🔒 Tangkapan kalibrasi (HW, hanya gantry, lengan REST, rot 0)

- Bring-up NYATA `my_workcell.launch.py use_fake_hardware:=false enable_gantry_bridge:=true use_sim_time:=false`
  (SIG_DFL + setsid nohup, PID launch ASLI, SigIgn dicek); 4× Actuator '6', 7/7 controller, table1/table2 ARMED;
  `env_static_map_pub.py` keep-alive (peta kanonik G33 reg3) + verifikasi `/get_planning_scene`; perekam `js_record`.
- Kamera: `realsense_dual.launch.py with_color_cloud:=false` + `depth_cloud` (ekstrinsik launch TIDAK diubah).
- Gerak HANYA via `p1_g32/pose_to_g.py --plan p1_g34/g34_calib_plan.json --seed 34 --variant CAL` (S12 lengan REST,
  S13" target terkunci di berkas, S23", S28 sapuan, G33 sapuan vs peta) — DRY dulu, lalu `--move`; satu gantry per waktu.
- Urutan (g1, g2) m, tangkap `capture_cloud.py --joints --seconds 20` di tiap titik → `p1_g34/cal_NN.npz`:

| # | 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 |
|---|---|---|---|---|---|---|---|---|
| (g1, g2) | (0, 0) | (0.45, 0) | (0.90, 0) | (1.35, 0) | (0, 0.45) | (0, 0.90) | (0, 1.45) | (0, 0) |

  (g1 1.35 → 0 sebelum 04; 06 → 0 sebelum 07.) 00 vs 07 vs `capture_a` = **keterulangan**.
- Berhenti bila: rc ≠ 0, REFUSE, fault, drift lengan > 0.5°, `capture_cloud` TOLAK (robot bergerak / < 10 frame).

### A2. 🔒 Model (offline, berurutan; pemenang pertama yang lulus)

Metrik: per tangkapan × gantry, **residual** = rata-rata vektor titik→permukaan lengan sesudah koreksi; |resid| dan
median |d|. Gerbang konsistensi: **|resid| ≤ 2 cm di SEMUA tangkapan × gantry yang terlihat (≥ 50 titik)**.

1. **H-cam (satu koreksi kaku 6-DOF per kamera, semua tangkapan):** dunia kamera miring/bergeser terhadap URDF,
   rel = URDF. Lulus gerbang → peta dikoreksi 6-DOF per kamera, **URDF tidak disentuh**.
2. **H-rail:** H-cam gagal → per gantry, residual sesudah H-cam dimodelkan linear di s_g (arah rel + skala encoder,
   ±tinggi). Lulus bila residual model ≤ 2 cm. Perbaikan = URDF `t*_base` / sumbu linear (+ skala bridge) —
   **perubahan model robot = keputusan operator** (dampak: MoveIt, semua penyaring). Tidak diterapkan tanpa izin.
3. Tidak ada yang lulus → per-posisi koreksi tidak bisa dipakai MoveIt (satu scene) → peta TETAP tangkapan A (G33),
   dilaporkan, dan HW R10 hanya dengan peta A.

### A3. 🔒 Peta gabungan + kontrol

Peta dari semua tangkapan (tiap tangkapan dengan konfigurasinya sendiri, koreksi pemenang A2), self-pad 0.20, sinar 0.05
(sama seperti reg3). Lulus bila: **(P+) ev 8 nyata COLLIDE/MARGIN dan ditolak ≥ 1.0 s sebelum kontak**; **(N−) R0 nyata
≥ 13/14 CLEAR** (ev13 nyaris-tabrak nyata boleh MARGIN); R10 ev0–7 nyata 8/8 CLEAR; ICP peta-A ↔ peta-B (kini dari
tangkapan baru) **< 2 cm / 0.3°**. Gagal → peta kanonik tetap reg3 (ditulis).

### A4. 🔒 Retract sadar-lingkungan (step 3)

`return_rest`: bila interpolasi lurus ditolak EnvChecker → minta rencana MoveIt (`env_static_map` di scene) ke REST,
lalu lintasan itu disaring SEMUA penyaring (antar-lengan S18, torsi, lingkungan); gagal → TOLAK (lengan tertahan, seperti
G33). Lulus: plan-only mock **R10 seed 36 3/3** dengan peta final, R0 3/3 tetap.

### A5. 🔒 HW R10 (step 4) — hanya bila A3 (P+) lulus pada peta final, A4 3/3, operator di e-stop

Bring-up nyata + `env_static_map_pub` keep-alive + cek `/get_planning_scene`; R0 sama-sesi lalu R10 (`run_g32`).

### A6. 🔒 Dugaan (SEBELUM tangkapan baru)

| # | Dugaan |
|---|---|
| D219 | 8/8 tangkapan rc 0, keempat lengan ≤ 0.5° dari REST di tiap tangkapan. |
| D220 | Keterulangan: 00 vs 07 registrasi semua-lengan beda ≤ 1 cm / 0.2°. |
| D221 | **H-cam LULUS** (dunia kamera miring; satu 6-DOF per kamera menjelaskan tz +0.10 @ g1 0.90). |
| D222 | Bila H-cam lulus: kemiringan dunia kamera (roll/pitch) 2–6°. |
| D223 | Peta gabungan final: (N−) R0 ≥ 13/14 dan (P+) lulus. |
| D224 | Retract sadar-lingkungan: R10 plan-only mock 3/3. |

## B. Hasil terukur

> Data: `docs/results/p1_g34/` — tangkapan `cal_00..07.npz` (+ `.log`), kaki gantry `g34hw_*.log`, perekam
> `g34hw_joint_states.csv.gz`, launch/realsense `*.log.gz`, fit `rail_calib_*.json`, `rail_joint{4,6}_diff.json`.

### B0. Bring-up dan tangkapan (HW, hanya gantry, lengan REST)

Bring-up nyata (launch 2055666, SIGINT tidak diabaikan): 4× Actuator '6', table1/table2 **ARMED** 0.2 / 0.9 mm, **7/7**
controller active (5 spawner "died" = balapan baseline G33). `env_static_map` (reg3) di scene — publisher pertama melapor
`False` lalu publish ulang; `--once` → **True**. `depth_cloud` dijalankan ulang dengan `stride:=2` (default 3; G33 = 2).

| kaki | rel (m) | err mm | t / T_cmd s | drift lengan ° | torsi N·m | S28 mm | lingkungan G33 mm |
|---|---|---|---|---|---|---|---|
| g1_to045 | 0.0002 → 0.4494 | −0.63 | 23.7 / 25.0 | 0.051 | 0.91 | 380.0 | 205.4 |
| g1_to090 | 0.4494 → 0.8995 | −0.53 | 23.8 / 25.0 | 0.043 | 0.96 | 392.6 | 96.6 |
| g1_to135 | 0.8995 → 1.3494 | −0.56 | 23.8 / 25.0 | 0.046 | 0.99 | 509.8 | 90.7 |
| g1_home | 1.3494 → 0.0003 | +0.34 | 73.0 / 75.0 | 0.067 | 1.06 | 380.0 | 90.7 |
| g2_to045 | 0.0009 → 0.4495 | −0.47 | 23.7 / 25.0 | 0.091 | 1.13 | 380.0 | 205.6 |
| g2_to090 | 0.4495 → 0.8998 | −0.16 | 24.1 / 25.0 | 0.059 | 1.16 | 392.7 | 196.5 |
| g2_to145 | 0.8998 → 1.4491 | −0.86 | 29.1 / 30.6 | 0.079 | 1.14 | 583.8 | 69.6 |
| g2_home | 1.4491 → 0.0008 | +0.83 | 78.0 / 80.5 | 0.114 | 1.13 | 380.0 | 69.6 |

8/8 tangkapan rc 0, 27–33 frame per kamera, `/joint_states` 32 sendi rentang maks ≤ 0.00008 (diam) → **D219 ✓**.
Matikan: helper SIGINT, realsense launch SIGINT, launch HW SIGINT → `ros2 node list --no-daemon` **0**; crash dump
`ros2_control_node` saat shutdown (baseline) dihapus. Akhir: g1 0.3 mm, g2 0.8 mm, rot 0/0, lengan ≤ 0.114°.

### B1. 🔴 Dua artefak yang hampir menyesatkan (keduanya DITULIS)

1. **Meja di bawah gripper.** Registrasi per-gantry naif memberi g1 @ 0.90 **tz −0.11 m di kedua kamera** (yaw −5°) —
   tampak seperti rel melendut. Penyebab: benda statis (meja kerja, z ≤ 0.81 m) 0.15 m di bawah ujung gripper REST
   (z ≈ 0.96) di x 0.7–1.1, di dalam radius pilih 0.25 m → titik meja ikut dianggap lengan. **Ini juga yang membuat G33 B9
   registrasi B ≠ A** (tz +0.085, peta A↔B 5.7 cm / 0.98°): capture_b memotret g1 tepat di 0.90.
   Perbaikan: **pengurangan latar** — buang titik yang sel 1 cm-nya (±1) juga terisi di tangkapan acuan tempat gantry itu
   ≥ 0.4 m jauhnya (`rail_calib.py --diff`). Sesudahnya g1 @ 0.90 tz **−0.004**, yaw −0.16°.
2. **Bug alat sendiri:** `RobotGeom.place()` mengembalikan `oMg` INTERNAL pinocchio; mode gabungan menyimpan referensinya →
   semua tangkapan memakai pose robot tangkapan TERAKHIR → gantry yang digerakkan "hilang" (residual ≈ seluruh perpindahan
   rel, 0.09–0.10 m). Fit gabungan A+B pertama (A0.1 lanjutan, "H-cam lemah") **TIDAK SAH** karena bug ini. Diperbaiki
   (salin `pin.SE3`); `build_env_map.py` diaudit: setiap `oMg` dipakai sebelum `place()` berikutnya → peta G33 tidak terdampak.

### B2. Model (A2) — H-cam, satu koreksi 4-DOF per kamera

Fit gabungan, 8 tangkapan, pengurangan latar, set satu-lengan dibuang (yaw tak terdefinisi):

| kamera | dof | t (m) | rpy (°) | median \|d\| sebelum → sesudah | set lulus (\|resid\| ≤ 2 cm) |
|---|---|---|---|---|---|
| rgbd | 4 | (−0.0992, +0.0705, +0.0037) | (0, 0, −0.549) | 9.1 → **1.2 cm** | **14/14** (maks 0.8 cm) |
| rgbd | 6 | (−0.0748, +0.0755, −0.0034) | (0.16, −1.13, −0.61) | 9.1 → 1.1 cm | 14/14 (maks 0.8 cm) |
| rgbd2 | 4 | (−0.0795, +0.1091, +0.0138) | (0, 0, −1.30) | 8.6 → **1.7 cm** | **5/6** — cal_03 g1 @ 1.35: **2.2 cm** |
| rgbd2 | 6 | (−0.0961, +0.0798, +0.0328) | (−1.43, 0.69, −1.42) | 8.6 → 1.6 cm | 5/6 — sama, 2.1 cm |

- **Rel = URDF** (dalam 1 cm sepanjang 0–1.45 m, kedua gantry, dilihat rgbd): **tidak ada galat arah/skala rel** yang
  terukur. Offset ~0.1 m G33 = **dunia kamera vs URDF, tetap**, bukan kinematika.
- Satu-satunya set di atas 2 cm: rgbd2 melihat g1 @ 1.35 (jarak ~2.2 m dari rgbd2), sementara rgbd melihat set yang sama
  dengan residual 0.1 cm → galat kedalaman rgbd2 di jarak jauh, **bukan rel** (model rel harus sama untuk kedua kamera,
  dan rgbd tidak menunjukkan kemiringan). H-rail tidak dapat menjelaskannya.
- 6-DOF tidak lebih baik dari 4-DOF; kemiringan dunia kamera hanya 1.1–1.4° → **D221 ✓ (sebagian: rgbd 14/14, rgbd2 5/6)**,
  **D222 ✗** (bukan 2–6°, dan tidak diperlukan).
- Keterulangan cal_00 vs cal_07: residual per gantry beda ≤ 0.1 cm → **D220 ✓**.

**B-konflik 1:** gerbang A2.1 "≤ 2 cm di SEMUA" gagal tipis (2.2 cm, 1/20 set, satu kamera, sebab teridentifikasi). Bukan
pindah ke H-rail (tidak cocok data) dan bukan mundur ke peta A (A2.3): **dipakai H-cam 4-DOF**, dan gerbang yang menentukan
dipindah ke kontrol A3 (P+/N−) pada peta gabungan. Residual rgbd2 jauh ≤ 2.2 cm ditulis sebagai ketidakpastian peta.

### B3. Retract sadar-lingkungan (A4) — `return_rest.plan_retract`

Satu fungsi dipakai `return_rest.py` (HW) DAN `g31_screen.py` (plan-only): (1) garis lurus dua lengan, disaring se-gantry +
antar-gantry + lingkungan; (2) ditolak → lengan per lengan, kedua urutan: garis lurus lengan itu bila CLEAR, kalau tidak
`_plan_and_screen(arm, {sendi: REST})` (MoveIt + tuck + torsi ≤ 12 + S18 + lingkungan). `reach_dwell_probe` kini menerima
goal sendi (`_joint_goal`). HW: tiap segmen diukur ulang lalu direncana ulang sebelum dikirim; segmen MoveIt pertama
direncana dari keadaan TERUKUR (bukan `start_joints`). `--no-plan` = perilaku G33.

Plan-only mock (domain 77, peta reg3, [log](results/p1_g34/g34_planonly_reg3.log)): **R10 3/3 LOLOS** — ev 11 retract g2 @ −10°:
sampel 1 dan 3 `moveit+lurus` (arm_3 via MoveIt, arm_4 lurus), sampel 2 lurus CLEAR (rencana tugas OMPL berbeda);
**R0 3/3 LOLOS**. (Diulang pada peta final, B5.)

### B4. Peta gabungan (A3) — empat versi, satu disimpan (B menang atas A, ditulis)

Semua: 8 tangkapan `cal_00..07`, `--transform rail_joint4_diff.json` (satu koreksi 4-DOF per kamera, BUKAN registrasi per
tangkapan), self-pad 0.20, sinar 0.05, voxel 2 cm. Kontrol = `g33_controls.py` (lintasan NYATA G32-HW).

| versi | filter tambahan | voxel | (P+) ev 8 | tolak sebelum kontak | (N−) R0 | R10 ev0–7 | catatan |
|---|---|---|---|---|---|---|---|
| reg3 (G33) | — (1 tangkapan, registrasi sendiri) | 71 763 | COLLIDE −0.012 | 2.0 s | 13/14 (ev13 0.026) | 8/8 | 0 voxel di benda x≈2.1 (benda NYATA, operator) |
| g34 | — (gabungan polos) | 138 707 | COLLIDE −0.012 | 1.7 s | 13/14 (ev13 0.026) | 8/8 | plan-only R10 2/3, R0 0/1: t2 arm_3 NO-PLAN |
| g34f | voxel ≥ 25 % frame | 74 461 | COLLIDE −0.012 | 1.5 s | 14/14 (ev13 **0.070**) | 8/8 | **DITOLAK**: memakan 14/18 voxel tepi atas rak dekat ev13 |
| g34c | ≥ 2 tangkapan, voxel persis | 112 551 | COLLIDE −0.012 | 1.7 s | 14/14 (ev13 **0.068**) | 8/8 | **DITOLAK**: tepi rak berkedip antar voxel per tangkapan |
| **g34n** | ≥ 2 tangkapan, **tetangga ±1 voxel** | 136 558 | **COLLIDE −0.012** | **1.7 s** | **13/14** (ev13 MARGIN 0.032) | **8/8** | dibuang 2 149 voxel satu-tangkapan |

- **Lintas kamera** (`cam_icp.py`, permukaan statis, awan 1 cm): rgbd2→rgbd **rot 1.7–2.7°, geser 4.8–5.8 cm**, median
  jarak tetangga 4.0–4.5 cm — mentah dan terkoreksi hampir sama (koreksi robot tidak menentukan kemiringan kamera; titik
  lengan hanya z 1.0–1.9). = klaim kalibrasi 07-30 (< 5 cm). **B-konflik 2:** kriteria A3 "ICP peta-A ↔ peta-B < 2 cm /
  0.3°" kehilangan makna dengan satu koreksi tetap per kamera (identitas menurut konstruksi); diganti uji lintas-kamera →
  **GAGAL** (~5 cm / 2–3°). Konsekuensi: di bawah z ≈ 1 m peta kurang pasti daripada di volume lengan; margin 5 cm
  EnvChecker ≈ ketidakpastian ini (tidak ada cadangan tambahan).
- **Benda di x 1.97–2.21, y −0.55…−0.11, z 0.87–1.17** (dekat target t2): rgbd2 saja, 2–6 dari 30 frame, di 6/8 tangkapan.
  Operator: **benda NYATA** → disimpan (konservatif). reg3 tidak memilikinya sama sekali. Rencana MoveIt ke t2 kadang
  mendarat di sana (NO-PLAN: "Computed path is not valid", jari kiri t2_a1 ↔ `env_static_map`), eksekusi nyata G32 ev12
  tetap 10 cm darinya.
- `build_env_map.py`: `--transform`, `--min-frac` (tidak dipakai, alasan di atas), `--min-captures` (tetangga).
  Regresi capture_a `--register` tetap identik (registrasi = `select_arm_points` + `icp_to_robot` dof 4, matematika sama).

### B5. Peta final = g34n → kanonik; MoveIt + plan-only + smoke

- `scripts/env_collision.ENV_MAP` → **`docs/results/p1_g34/env_static_map.npz`** (= `env_map_g34n.npz`, 136 558 voxel;
  `env_static_map_pub` → 22 246 kotak). Peta G33 reg3 tetap di `p1_g33/env_static_map.npz` (cadangan). Regresi: membangun
  ulang reg3 dengan kode sekarang → **71 763 = 71 763 voxel identik**.
- MoveIt (`g33_moveit_check`, mock 2.5.10): REST **VALID**; ev 8 VALID −3.0 s, **INVALID −2.0/−1.0/−0.5/0 s** (hanya
  gripper/jari t2_a1 ↔ peta); R0 135 konfigurasi → **1 INVALID** (ev13) — identik dengan reg3.
- Plan-only mock seed 36 k=3 ([log](results/p1_g34/g34_planonly_g34n.log)): **R10 3/3 LOLOS** (ev 11: sampel 1
  `moveit+lurus`, 2–3 lurus), **R0 3/3 LOLOS** → **D224 ✓**. (Peta g34 polos: R10 2/3, R0 0/1 — NO-PLAN t2 di benda x≈2.1;
  peta reg3: R10 3/3, R0 3/3.) ⚠ NO-PLAN t2 bersifat acak (IK goal); 3/3 ≠ jaminan di HW.
- **Smoke MOCK `run_g32` R10** (bukan data): **14/14 event rc 0, 6/6 tugas**, makespan mock 884.67 s; ev 11 di jalur
  EKSEKUSI: lurus dua lengan MARGIN 28.1 mm → arm_3 lurus MARGIN → **arm_3 via MoveIt** (torsi j2 11.37/14, antar-lengan
  330 mm, lingkungan 66 mm) dikirim 164 titik → ukur ulang → arm_4 lurus CLEAR 135 mm → galat akhir 0.175°.
  `return_rest` juga memulangkan lengan mock dari 99.6° (lurus, rc 0). rc 134 runner saat keluar = warisan benign.

### B6. Papan skor D219–D224

| # | Dugaan | Hasil |
|---|---|---|
| D219 | 8/8 tangkapan rc 0, lengan ≤ 0.5° | ✓ (drift maks 0.114°) |
| D220 | keterulangan 00 vs 07 ≤ 1 cm / 0.2° | ✓ (≤ 0.1 cm) |
| D221 | H-cam lulus | ✓ sebagian — rgbd 14/14, rgbd2 5/6 (2.2 cm, jarak jauh); dipakai (B-konflik 1) |
| D222 | kemiringan dunia kamera 2–6° | ✗ 1.1–1.4°, 6-DOF tak lebih baik dari 4-DOF |
| D223 | peta final (N−) ≥ 13/14 dan (P+) lulus | ✓ 13/14, COLLIDE −0.012, tolak 1.7 s sebelum kontak |
| D224 | retract sadar-lingkungan R10 plan-only 3/3 | ✓ (dan smoke eksekusi mock 14/14) |

Temuan terbesar lagi-lagi bukan dari dugaan: (1) "rel melendut" = meja di radius pilih; (2) bug aliasing `oMg` di alat
sendiri; (3) filter derau yang tampak wajar (≥ 25 % frame, ≥ 2 tangkapan persis) memakan tepi rak — hanya kontrol ev13
yang menangkapnya.

### B-konflik (lanjutan)

3. A3 "ICP peta-A ↔ peta-B" → uji lintas-kamera (gagal ~5 cm / 2–3°, B4).
4. A3 tak menyebut filter transien; ditambah `--min-captures 2` (tetangga ±1 voxel) sesudah dua filter lain ditolak kontrol.
5. A5 (HW R10) **TIDAK dijalankan**: bring-up nyata ke-2 sudah naik (launch 2112565, 4× Actuator '6', ARMED 0.3/0.8 mm,
   7/7 controller, peta di scene True), lalu **operator menunda tes langsung ke besok** → dimatikan tanpa gerak → **G34b (§B7)**.

### B7. G34b — HW R0 + R10 seed 36, peta g34n + retract sadar-lingkungan (2026-10-08)

> Prompt = §D. Gerbang (operator, sebelum apa pun): **K0 sel sama seperti akhir G34 — ya**; **K1 operator di e-stop — ya**.
> Data: `docs/results/p1_g34/hw/` (event log + `run.json` per varian, `g34hw_home_*`, launch `.log.gz`, monitor summary);
> plan-only `g34_planonly_g34b_hw*`; skrip `g34b_run.sh` (salinan `g32hw_run.sh`), `g34b_home.sh`. Perekam
> `hw/g34hw_joint_states.csv.gz` (68 MB) **lokal saja, tidak di-commit** (seperti G32-HW).

**B7.0 Sebelum gerak.** `remount_check` LULUS (ICMP 4/4, nol basi, 2× D455). Bring-up nyata launch **2158899** (SIG_DFL + setsid
nohup, SigIgn `0x1001001` → SIGINT tidak diabaikan): 4× Actuator '6', **7/7** active (7 spawner "died" = baseline), table1/table2
**ARMED 0.3 / 0.8 mm**, rot 0/0. `env_static_map_pub` keep-alive: g34n 136 558 voxel → 22 246 kotak, di scene **True**; `--once`
rc 0 **True**. `js_record` + `reach_dwell_monitor` (csv `/tmp/g34hw_step`) hidup. Lengan maks 0.111° dari REST.

**B7.1 Plan-only NYATA** (`DOMAIN=0 g34_planonly.sh g34b_hw R10 R0`, [log](results/p1_g34/g34_planonly_g34b_hw.log)):
**R10 3/3 LOLOS, R0 3/3 LOLOS** (36/36 PLANNED, **0 NO-PLAN t2**). Semua 12 retract tak-kosong **lurus CLEAR** (MoveIt tidak
terpicu; mock G34 sampel 1 memakai MoveIt di ev 11 — rencana tugas OMPL berbeda). Lingkungan min **58.3 mm**.

**B7.2 Eksekusi** (`run_g32` apa adanya, DRY dulu: 14 event, S26 lolos):

| | R0 | R10 |
|---|---|---|
| sukses penilai | **6/6**, 14/14 rc 0, nol auto-stop | **6/6**, 14/14 rc 0, nol auto-stop |
| sukses @ s | 139.7 / 312.3 / 378.9 / 525.4 / 737.5 / 783.3 | 129.7 / 280.9 / **441.0 (ev 8)** / 510.3 / 597.8 / 849.3 |
| **makespan** | **783.33 s = 1.348 ×** P1′-serial 581.26 | **849.29 s = 1.601 ×** P1′-serial 530.39 |
| torsi puncak (tugas) | 10.51 `t2_a1_joint_2` (ev 9) | 8.64 `t2_a1_joint_2` (ev 10) |
| lingkungan min (rencana) | 58.1 mm (ev 13 t2 arm_4) | **56.8 mm** (ev 10 t5 arm_3) |
| fault launch log | 0 | 0 |

- **ev 8 R10 (tugas yang menabrak rak di G32-HW): SUCCESS**, torsi 8.57 N·m (G32: 12.30 → TORQUE-ABORT), lingkungan 95.4 mm,
  antar-lengan 496.0 mm. Tidak ada kontak dilaporkan operator.
- Traverse (lin, rot): galat lin −0.56…−0.04 mm, **ev 7 → (0.5997, −9.99°)** −0.32 mm / +0.01°, ev 12 → (1.4498, 0) −0.17 mm /
  0.00°; t_gerak/T_cmd 0.96–0.98; drift lengan ≤ 0.197°.

**B7.3 Tiap retract (jenis, lingkungan min, torsi):**

| varian | ev | lengan (galat awal) | jenis | lingkungan | torsi | galat akhir |
|---|---|---|---|---|---|---|
| R0 | 3 | arm_1 (135.6°) | lurus | 88.6 mm | 4.23 | 0.088° |
| R0 | 7 | arm_3 (170.6°) | lurus | 209.3 mm | 5.36 | 0.111° |
| R0 | 10 | arm_3 (137.0°) | lurus | 112.1 mm | 7.99 | 0.112° |
| R10 | 3 | arm_1 (169.4°) | lurus | 88.6 mm | 6.23 | 0.089° |
| **R10** | **11** | **arm_3 + arm_4 @ g2 −10° (170.5°)** | **lurus dua lengan** | **67.8 mm** | 6.64 | 0.095° |
| pulang R0 | — | g2 / g1 | lurus / lurus | — | 5.18 / 5.31 | 0.049 / 0.089° |
| pulang R10 | — | g2 / g1 | lurus / lurus | — | 5.70 / 5.18 | 0.048 / 0.089° |

ev 0/6 = skip (REST). **Jalur MoveIt `return_rest` TETAP belum pernah dipakai lengan nyata** — tidak ada retract yang ditolak
garis lurus (G33 B6 / mock G34 ev 11 menolak garis lurus karena rencana tugas berakhir lebih dekat ke rak).
Rel pulang: g1 0.157 / 0.136 mm, g2 0.963 / 0.764 mm, rot 0/0.

**B7.4 Δ(R10 − R0) terukur = +65.96 s** (model P1′ **−50.87 s**) — tanda BERLAWANAN. Atribusi per jenis event (run.json):

| | R0 | R10 | Δ |
|---|---|---|---|
| traverse | 221.8 | 326.3 | **+104.5** |
| retract | 190.9 | 132.0 | −58.9 (R10 lewati 1 retract — mekanisme model) |
| tugas | 370.6 | 390.8 | +20.2 |

Traverse berotasi menghabiskan **55.3 s dan 81.1 s sebelum kirim** (sapuan `pose_to_g` vs peta + S28; n 671 / 946 sampel) vs
13.3–19.5 s traverse rel-saja; gerak itu sendiri = T_cmd (32.1/33.3, 45.9/47.2 s). Selisih pra-kirim ≈ **+103.6 s**.
**Inferensi (belum diukur terpisah):** tanpa overhead saringan sapuan, Δ ≈ −37.7 s — tanda sama dengan model.
Makespan lebih tinggi dari G32-HW R0 (647.47 → 783.33, +136 s) = **muat + jalankan EnvChecker** di tiap alat: tugas +78, traverse
+33, retract +25 s (retract: ~13 s muat model + ~17 s saring sebelum kirim; gerak 30 s sama).

### B8. G34b — pertentangan / Rule 12

1. 🔶 **Tidak ada dugaan dikunci sebelum bring-up** (prompt §D tidak memintanya; kebiasaan §A sejak G23 tidak diikuti) — kesalahan
   proses, sama kelas dengan G22. Tally G34b = 0/0; tidak ada dugaan yang ditulis sesudah data.
2. Δ(R10 − R0) terukur **bukan** uji model P1′: biaya hitung penyaring (tak ada di model) mendominasi, dan berskala dengan jumlah
   sampel sapuan (rotasi). Makespan G34b ≠ makespan G32-HW (alat berbeda).
3. Lingkungan min 56.8 mm vs margin EnvChecker 50 mm vs ketidakpastian lintas-kamera ~5 cm (B4): cadangan nyata **≈ 0–7 mm**
   di ev 10 R10 / ev 13 R0. Tidak ada kontak, tetapi "CLEAR" di sini tidak lebih kuat dari kalibrasi kamera.
4. `g34_planonly.sh`: `export ROS_DOMAIN_ID=${DOMAIN:-77}` (default mock tidak berubah).

## C. Keadaan akhir (Rule 12)

- **Gerak:** hanya gantry (B0, 8 kaki, rc 0). **Lengan nyata tidak pernah digerakkan.** Akhir: lengan ≤ 0.111° dari REST,
  rel g1 0.34 / g2 0.83 mm, rot 0/0.
- **Stack:** HW #1 (2055666) dan HW #2 (2112565) SIGINT → `ros2 node list --no-daemon` **0**; mock domain 77 (2065512)
  SIGINT → **0**. Crash dump (ros2_control_node / move_group saat shutdown, run_g32 rc 134) dihapus. Disk 55 GB.
- **Diubah:** `scripts/env_collision.py` (ENV_MAP → p1_g34), `scripts/return_rest.py` (`plan_retract`, eksekusi per segmen,
  `--no-plan`), `scripts/reach_dwell_probe.py` (`_joint_goal`), `docs/results/p1_g31/g31_screen.py` (retract =
  `plan_retract`), `docs/results/p1_g33/build_env_map.py` (`select_arm_points`/`icp_to_robot`, `--transform`,
  `--min-frac`, `--min-captures`). **Baru:** `docs/results/p1_g34/` (`rail_calib.py`, `rail_table.py`, `cam_icp.py`,
  `leg.sh`, `g34_planonly.sh`, `g34_calib_plan.json`, tangkapan, peta, kontrol, smoke).
- **Belum diuji di HW:** jalur MoveIt `return_rest` (hanya mock); peta g34n di HW (hanya traverse gantry dengan reg3).
- **Tidak dikerjakan:** HW R0/R10 (ditunda operator); kemiringan lintas-kamera ~5 cm; octomap hidup (K1: tidak);
  pose simpan tinggi; p1_state + tally (menunggu hasil HW).
- **Belum di-commit.** Anggaran token Rule 6 (30k/sesi) **terlampaui jauh** — dilaporkan.

## D. Prompt lanjutan (sesi besok, salin ke chat BARU)

> ✅ Dijalankan 2026-10-08 sebagai G34b → §B7, §B8, §C2.

**Rekomendasi: Opus, effort TINGGI** — gerak lengan nyata pertama sejak tabrakan rak; jalur pulang MoveIt baru di HW.

```
Sesi G34b -- HW R0 + R10 seed 36 dengan peta G34 + retract sadar-lingkungan. Repo ceiling_arm, branch feat/rgbd-topo-deploy.
BACA PENUH: CLAUDE.md; docs/p1_g34_rail_calib.md (B2, B4, B5, C); docs/p1_g32_hw.md A1 (urutan HW); scripts/return_rest.py
(plan_retract + main); docs/results/p1_g32/run_g32.py.
FAKTA G34: offset kamera<->URDF TETAP per kamera (rel = URDF <= 1 cm); peta kanonik docs/results/p1_g34/env_static_map.npz
(g34n, 136 558 voxel; ENV_MAP sudah menunjuk ke sana); (P+) ev 8 COLLIDE ditolak 1.7 s sebelum kontak, (N-) R0 13/14 (ev13
nyaris-tabrak rak NYATA), MoveIt = G33; plan-only mock R10 3/3, R0 3/3; smoke mock run_g32 R10 14/14 (ev 11 pulang via
MoveIt). Benda NYATA di x~2.1 y~-0.3 z~0.9-1.2 (operator) -> NO-PLAN t2 kadang (acak). Lengan nyata BELUM pernah memakai
jalur MoveIt return_rest. Lintas-kamera masih ~5 cm / 2-3 deg (di bawah z~1 m peta kurang pasti).
GERBANG (tanya operator SEBELUM apa pun): K0 sel sama seperti akhir G34 (lengan REST, rel ~0/0, rot 0/0, rak & benda x~2.1
tidak dipindah, tidak ada benda baru)? K1 operator di e-stop selama gerak lengan?
1. remount_check; bring-up nyata (SIG_DFL + setsid nohup, PID launch ASLI, SigIgn), 4x Actuator '6', 7/7, ARMED;
   env_static_map_pub keep-alive + --once True; js_record p1_g22 + reach_dwell_monitor (csv /tmp/g34hw_step).
2. Plan-only NYATA: docs/results/p1_g34/g34_planonly.sh dengan ROS_DOMAIN_ID=0 (ubah export), R10 lalu R0, k=3 -> harus 3/3.
3. run_g32 --seed 36 --variant R0 (bergerak tanpa flag; --out-dir/--archive p1_g34/hw --prefix g34hw_R0_); pulang
   (return_rest g1,g2; pose_to_g g1,g2 -> 0,0); lalu R10 sama; pulang; matikan bersih (node list --no-daemon = 0).
4. Laporan p1_g34_rail_calib.md §B7+ (makespan vs P1'-serial, delta R10-R0, tiap retract: lurus/moveit, lingkungan min),
   p1_state + tally, commit, prompt G35 + rekomendasi model/effort.
ATURAN: tak ada gerak tanpa env_static_map di scene DAN EnvChecker; MARGIN = TOLAK; B menang atas A dan DITULIS;
job > 10 menit via setsid nohup; pkill -f membunuh shell sendiri (pilih PID via ps/awk).
```

## C2. Keadaan akhir G34b (Rule 12)

- **Gerak:** lengan nyata R0 (14 event) + R10 (14 event) + 2× pulang; semua rc 0, nol fault, nol auto-stop, tidak ada kontak
  dilaporkan. Akhir: **keempat lengan REST** (≤ 0.089°), **g1 0.136 mm, g2 0.764 mm, rot 0/0**.
- **Stack:** helper (monitor, perekam, `env_static_map_pub`) SIGINT; launch 2158899 SIGINT → keluar 8 s;
  `ros2 node list --no-daemon` **0**. Crash dump `ros2_control_node` saat shutdown (baseline) dihapus. Disk 53 GB.
- **Belum diuji di HW:** jalur MoveIt `return_rest` (tidak terpicu, B7.3). **Tidak diukur:** Δ(R10 − R0) bersih dari overhead
  saringan (B7.4 hanya inferensi).
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi HW multi-langkah; dilaporkan.

## E. Prompt G35 (salin ke chat BARU)

> ✅ Dijalankan 2026-10-08 sebagai G35 → [p1_g35_env_speed.md](p1_g35_env_speed.md).

**Rekomendasi: Opus, effort TINGGI** — mengubah biaya penyaring keselamatan; penyaring yang dipercepat lalu diam-diam
melewatkan tabrakan adalah galat yang tidak terlihat sampai lengan menabrak.

```
Sesi G35 -- OFFLINE (nol gerak): overhead penyaring lingkungan mendominasi makespan HW; ukur, percepat TANPA mengubah verdict.
Repo ceiling_arm, branch feat/rgbd-deploy (dulu feat/rgbd-topo-deploy). BACA PENUH: CLAUDE.md; docs/p1_g34_rail_calib.md (B7, B8, C2);
scripts/env_collision.py (EnvChecker.screen_trajectory, self_filter); docs/results/p1_g32/pose_to_g.py (sapuan); scripts/
reach_dwell_probe.py (screen_env); scripts/return_rest.py (main: muat penyaring).
FAKTA G34b: HW seed 36 R0 6/6 783.33 s (1.348x P1'-serial), R10 6/6 849.29 s (1.601x); ev 8 (tabrakan rak G32) SUCCESS
8.57 N.m; semua retract lurus (MoveIt return_rest belum pernah di HW); lingkungan min 56.8 mm (margin 50, lintas-kamera ~5 cm).
Delta R10-R0 = +65.96 s vs model -50.87: traverse berotasi 55.3/81.1 s SEBELUM kirim (sapuan n 671/946) vs 13-20 s rel-saja;
vs G32-HW R0 +136 s = muat+jalankan EnvChecker (tugas +78, traverse +33, retract +25). Data: docs/results/p1_g34/hw/.
1. KUNCI §A (dugaan D225+) SEBELUM mengukur -- G34b lupa mengunci dugaan (B8.1); jangan ulangi.
2. Profil: muat (URDF/peta) vs per-sampel vs jumlah sampel sapuan, per alat; cocokkan dengan pre-send B7.4 (+-10 %).
3. Percepat (contoh: muat sekali/cache, broadphase hanya badan bergerak, sampel sapuan adaptif) -- REGRESI WAJIB: verdict
   identik + d_min selisih <= 0.5 mm pada SEMUA lintasan arsip (g34_controls_g34n, plan-only jsonl G34/G34b, V28, ev 8 P+
   COLLIDE tetap ditolak >= 1.7 s sebelum kontak). Sampel lebih jarang hanya bila dibuktikan tak melewatkan (jarak antar
   sampel < margin). Gagal regresi -> kembalikan, DITULIS.
4. Prediksi ulang makespan G34b dengan overhead baru (+ suku overhead di model P1' bila tetap ada); prompt G35b HW
   (R0+R10 seed 36 sama-sesi, + 1 seed lain bila operator setuju) + rekomendasi model/effort.
ATURAN: nol gerak; MARGIN = TOLAK tidak berubah; B menang atas A dan DITULIS; job > 10 menit via setsid nohup.
```
