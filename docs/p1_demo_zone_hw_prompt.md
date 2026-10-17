# Prompt pembuka sesi DEMO-ZONE-HW (demo bergerak dengan koreksi zona) — salin ke sesi baru

```text
Lanjutkan DEMO-ZONE-HW di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak lengan/rel/gripper tanpa izin.
Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan), docs/p1_campaign_state.md (baris DEMO-ZONE terbaru),
docs/p1_demo_zone.md (§Node, §DEMO-ZONE-LIVE A+B), docs/p1_demo_hw.md (rantai demo, B1–B6 temuan F1–F7, harness),
memori demo-zone-p3-correction, demo-hw-rgbd2-range-error, ceiling-arm-fold-is-torque-limited, ros2-launch-pid-leak.

SUDAH TERBUKTI (10-17, kamera saja): track rgbd2 → results/p1_demo_zone/zone_correct_node.py → /demo_zone/tracks: C3 2.36 cm,
M1 2.63 cm (mentah 15–17 cm), g1 1.40. BELUM PERNAH DIUJI: rantai GERAK memakai /demo_zone/tracks.

TUGAS (urut, ✓/✗ harfiah):
0. LURING: (a) demo_runner.py memakai argparse parse_args() → `--ros-args -r` GAGAL: tambah opsi `--tracks-topic` (default
   /detected_objects/tracks = perilaku lama identik); target_cli sudah rclpy.init(sys.argv) → remap `--ros-args -r
   /detected_objects/tracks:=/demo_zone/tracks` (cek). (b) Harness baru (jangan ubah harness lama): kamera cam_up.sh seg_imgsz:=1280
   + loc_cam.sh rgbd2 + zone_up.sh, SEMUA domain 0 (runner harus di domain stack). (c) Uji mock/replay: track lama → node →
   runner --plan-only menerima p_cmd = (xy terkoreksi, 0.805 + 0.25). Commit.
1. 🔴 Gerbang sel SEBELUM bring-up (tanya operator): papan polos masih di jari arm_2/arm_3 — probe/return_rest TIDAK tahu papan
   (tanpa payload); runner bisa memilih arm_2 bila arm_1 gagal. Pilihan: lepas papan dulu (buka gripper = gerak, izin) ATAU batasi
   runner ke arm_1 saja (opsi baru, default lama). g1 1.40 (zona tervalidasi di sini; gripper arm_1 menutupi N1 — taruh botol di
   C3/M1/C5). Meja: jalankan results/p1_demo_zone/live/table_check.py (0 px) — operator pernah menggeser meja.
2. §A DEMO-ZONE-HW (dugaan lanjut dari D654) → kunci (salinan + sha256) SEBELUM bring-up. Kriteria z dari meja 0.77 + botol 7 cm
   (BUKAN [0.85, 1.05] lama, DEMO-HW F3). Torsi: --tau-max 14, j2 di titik jauh ~100 % (DEMO-HW F4) — MARGIN = TOLAK.
3. HW (izin per langkah): self-test env/interarm + remount_check; stack_up.sh (octomap_updaters:=false), K0 --rail1 1.40 --rail2 0;
   monitor + perekam; kamera+localizer+node domain 0; botol putih 7 cm di C3; perintah teks (POST /api/transcript) → `g` →
   PLAN-ONLY dulu (H3) → lalu jalan bergerak 1–3× (tiap `g` = izin sendiri) → REST → K0.
4. Matikan per PID: ⚠ launch bisa SigIgn SIGINT dan SIGTERM ke launch meninggalkan anak → SIGINT anak per PID (live_down.sh pola);
   node 0 di domain 0. Papan skor §B, berkas keadaan, memori, rekomendasi model/effort.

ATURAN: penyaring tidak dilonggarkan; fail loud; setsid nohup; jangan sentuh /home/kubota; frame (ada orang) tidak di-commit; commit
per bagian hanya berkas sendiri (git add <path>). Bahasa Indonesia, singkat (caveman).
Keadaan sel: papan polos di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack + kamera mati, meja di posisi tanda.
```

Model: Opus, effort **high** (gerak arm_1 dekat batas torsi j2; galat penyaring/payload bisa diam).
