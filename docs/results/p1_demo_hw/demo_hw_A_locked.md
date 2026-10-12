# P1 / DEMO-HW — suara → YOLOE (objek nyata) → lengan ke ATAS objek (approach-only), HW NYATA

> Sesi 2026-10-12 (prompt [p1_demo_hw_prompt.md](p1_demo_hw_prompt.md)). SCOPE: suara/teks → YOLOE `bottle` → `object_localizer`
> → `target_cli` → `demo_runner` → `reach_dwell_probe` → arm_1 berhenti stand-off 0.25 m di atas centroid botol, dwell, REST.
> Tanpa grasp, tanpa angka akurasi (eval = P2). 1–3 jalan bergerak. g1 PARKIR 1.50, gantry TIDAK diperintah.
> §A dikunci (salinan `results/p1_demo_hw/demo_hw_A_locked.md` + sha256) sebelum kamera dibuka dan sebelum gerak pertama.

## A. Protokol — DIKUNCI

### A0. Gerbang + pengungkapan (sebelum mengunci, NOL gerak)

**Gerbang operator (~17:47):** K0 sel = akhir G57 ✓; K1 operator di e-stop fisik tiap gerak ✓; K2 lampu nyala, satu botol
≤ 0.30 m di meja sekitar (1.75, 0.55), tak ada botol lain ✓ (posisi **kira-kira**, lihat A0.5); K3 tak ada task server /
reach_fusion ✓ — dicek juga `ps`: nol proses ROS, `enp112s0` 192.168.2.100.

| # | Temuan / keputusan | Bukti |
|---|---|---|
| A0.1 | 🔴 **Domain ROS.** Harness DEMO-OFF/G59: kamera + localizer domain 79, runner/CLI 77 (mock). Di HW `demo_runner` (sub `/detected_objects/tracks`, panggil probe → MoveIt) HARUS di domain stack 0 → kamera, `object_localizer`, `voice_web_ui`, `target_cli`, `demo_runner`, monitor, perekam **semua domain 0**. Domain 79 hanya untuk langkah 1 (kamera tanpa stack). Prompt `p1_demo_off.md` §D ("kamera tetap domain 79") **salah** untuk demo bergerak. Harness baru [results/p1_demo_hw/harness/](results/p1_demo_hw/harness/) (`env.sh`: `ROS_DOMAIN_ID=${DEMO_DOMAIN:-0}`); harness lama tidak diubah. | `demo_runner.py:62`; `p1_demo/harness/env*.sh` |
| A0.2 | 🔴 **Ekstrinsik kamera belum dikoreksi G34.** `realsense_dual.launch.py` memakai kalibrasi 2026-07-30; peta `env_static_map` dibangun DENGAN koreksi tetap G34 (`p1_g34/rail_joint4_diff.json`, p_urdf = R·p_cam + t). Tanpa koreksi, botol di URDF (1.75, 0.55, 0.97) terlihat di (1.845, 0.497) [rgbd] / (1.819, 0.482) [rgbd2] → **|dxy| ≈ 0.10–0.11 m** → kriteria 5 cm gagal dan lengan berhenti ~10 cm di samping botol. **Keputusan operator ~17:50: pakai koreksi G34** lewat arg launch di `harness/cam_up.sh` (rgbd t (1.7166, 0.8656, 2.0782) yaw 2.1688; rgbd2 t (−0.1547, −1.0197, 2.0905) yaw −0.9189; roll/pitch tetap — koreksi murni yaw). Default launch TIDAK diubah. Sisa: beda antar-kamera ~5 cm / 2–3° (G34), di bawah z ≈ 1 kurang tepercaya. | memori camera-world-vs-urdf-offset; `realsense_dual.launch.py:163-174` |
| A0.3 | `demo_runner` hardcode `--tau-max 12.0` → opsi `--tau-max` (default 12.0 = argv lama identik `'12.0'`); demo HW `--tau-max 14` (operator 2026-10-12: > 12 bukan gagal, batas keras > 14). | diff `demo_runner.py` |
| A0.4 | `g` hanya lewat `harness/cli.fifo`, ditulis Claude **sesudah** persetujuan operator untuk SATU jalan; satu persetujuan = satu jalan utuh (probe `--move` → dwell → `return_rest --arms arm_1 arm_2 --move` otomatis oleh runner). Suara = Chrome mic ke `voice_web_ui` (https :18080) atau POST `/api/transcript` teks. `g` yang tiba selama misi dibuang `target_cli` (G59 B1.2). | G59 B1 |
| A0.5 | **Posisi botol acuan = kira-kira** (operator tidak tahu titik nol world). Cara ukur yang diberikan: unting dari pusat basis arm_1 (world (1.90, 0.36) saat g1 1.50) ke meja; titik (1.75, 0.55) = **15 cm ke arah arm_2 (−x), 19 cm menjauhi gantry 2 (+y)**. Kriteria H2 dibandingkan ke posisi yang dilaporkan operator sesudah ukur; bila operator tak bisa mengukur, H2 dilaporkan "acuan kasar" (tidak dihitung LULUS). | `p1_demo_off.md` A5 |
| A0.6 | Botol TIDAK ada di `env_static_map` (T11) → pemisah dari botol = stand-off saja. Puncak botol ≤ centroid + 0.15 → ujung tool ≥ 0.10 m di atas puncak bila z track benar. Track z terlalu RENDAH = tool lebih dekat ke botol → H2 z ≥ 0.85 wajib sebelum gerak. | DEMO-OFF A2 |
| A0.7 | `POSE_REFUSALS` tidak memuat START-JUMP / START-UNSCREENED / GOAL-MISMATCH → runner berhenti + `return_rest` (tidak coba lengan lain). Tidak diubah. | G59 A0 |
| A0.8 | `object_localizer` memfusi kedua kamera (`fuse_radius` 0.10) — track = rata-rata; `rgbd_perception.launch.py` tidak menjalankan localizer → `harness/loc_up.sh` (remap `seg/`). | `object_localizer.py:55,125` |

### A1. 🔒 Kriteria

| # | Kriteria | Lulus bila |
|---|---|---|
| H1 | Deteksi stabil | log `seg_router` tanpa `inference failed`; `/rgbd/seg/instance_segmentation_labels` (atau rgbd2) memuat `bottle`; tepat SATU track `bottle` dengan tid sama sepanjang rosbag 20 s |
| H2 | Lokalisasi | track `bottle` \|dxy\| ≤ 5 cm dari posisi botol terukur operator; z ∈ [0.85, 1.05]; di domain 79 (langkah 1) DAN domain 0 dengan stack hidup (langkah 2) |
| H3 | Plan-only | `success arm_1 (arm_1:PLANNED)`; /joint_states sebelum = sesudah (≤ 0.05°, rel ≤ 1 mm) |
| H4 | Jalan bergerak | MOVED → dwell SUCCESS → `return_rest` rc 0 → status `success arm_1`; torsi terukur ≤ 14 N·m semua sendi; 0 gerak kedua tak diminta; t1_linear tetap 1.4999 ± 5 mm, rot ± 0.3° (gantry tak diperintah) |
| H5 | Bersih | lengan REST, K0 akhir LULUS; `ros2 node list --no-daemon` = 0 di domain 0 dan 79; /tmp diarsip |

**Berhenti + amankan (tanya operator):** fault / LED merah / REJECTED / NOT armed sesudah ARMED / torsi > 14 / benturan / kabel /
gerak tak diminta / track z < 0.85 sebelum `g`. MARGIN = TOLAK; penyaring TIDAK dilonggarkan. `failed TORQUE-UNSAFE` = `g` lagi
(dengan persetujuan baru), bukan ubah penyaring.

### A2. 🔒 Urutan

1. Kamera tanpa gerak, domain 79, stack MATI (`DEMO_DOMAIN=79 harness/cam_up.sh` + `loc_up.sh`): H1, H2, rgb (`p1_demo/harness/grab.py`),
   rosbag 20 s (`harness/bag.sh`, topik ringan). Matikan, node 0 di 79.
2. Self-test `env_collision` + `interarm_collision`; `remount_check`; bring-up nyata `harness/stack_up.sh` (G57: SIG_DFL, SigIgn,
   4× '6', 7/7, ARMED, `env_static_map` di scene); K0 `--rail1 1.50 --rail2 0.00`; monitor + perekam (domain 0, cek berkas tumbuh);
   kamera + localizer domain 0 → ulang H1/H2.
3. `voice_web_ui` + `target_cli` (fifo).
4. Runner `EXTRA=--plan-only` → perintah → `g` (persetujuan) → H3.
5. Runner tanpa `--plan-only` → per jalan: tanya → perintah → `g` → H4. 1–3 jalan.
6. K0, matikan bersih (runbook §7), arsip, laporan + gambar arsitektur.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D540 | Langkah 1: H1 lulus (lampu nyala, YOLOE dari `ros2_ws/`); track `bottle` \|dxy\| ≤ 5 cm dari (1.75, 0.55) dengan ekstrinsik terkoreksi. |
| D541 | Track z ∈ [0.90, 1.05] (kamera atap melihat bagian atas botol → centroid permukaan di atas centroid sejati). |
| D542 | Domain 0 dengan stack hidup: track sama ≤ 2 cm dari langkah 1, nol `inference failed`. |
| D543 | Plan-only: arm_1 PLANNED pada `g` pertama (≤ 3 sampel); penyaring terketat = j3 RNEA+offset 88–94 % (mock 90–92 %). |
| D544 | Jalan bergerak 1: MOVED → SUCCESS → REST rc 0 pada `g` pertama atau kedua; perintah → status 60–110 s (mock 70–73 s). |
| D545 | Torsi terukur puncak (semua sendi arm_1, seluruh jalan) ≤ 13.0 N·m, terjadi di TRANSIT (bukan di dwell). |
| D546 | 0 START-JUMP / START-UNSCREENED; 0 gerak tak diminta; gantry diam (H4). |
| D547 | TCP dwell vs p_cmd ≤ 1 cm (monitor `pos_err_max_settled_mm`). |

