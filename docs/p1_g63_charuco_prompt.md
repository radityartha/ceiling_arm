# Prompt pembuka G63 — kalibrasi ekstrinsik D455 dengan target RGB-saja (ChArUco/tag + FK lengan)

Salin blok di bawah ke sesi chat baru. Sesi pertama G63 = **DISKUSI DESAIN dengan operator**, bukan pengambilan data.

```text
Mulai fase G63 di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy): kalibrasi ekstrinsik kedua D455
(rgbd 241122302297, rgbd2 234222303079) dengan target RGB-saja yang dipegang lengan, ground truth = FK lengan.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md (§2 aturan, §4.2), docs/p1_campaign_state.md,
docs/p1_cam6dof.md (§A + B57–B65), docs/p1_calib_charuco.md, laporan
docs/results/p1_cam6dof/diagnosis_20261015_offline/report.md dan docs/results/p1_cam6dof/depth_health_20261015/report.md.
Memori Claude (bila ada): /home/user1/.claude/projects/-home-user1-Documents-ceiling-arm/memory/MEMORY.md.

KENAPA G63 ADA (jangan diulang/diperdebatkan tanpa data baru):
- CAM-6DOF (botol + YOLOE + median depth) GAGAL gerbang; M6/M7 menggeser kamera 0.13–0.25 m / skala 0.89.
- B64: permukaan botol terbaca +8.4 cm lebih jauh di KEDUA kamera (bias material), mask ikut box penyangga →
  botol+depth bukan alat ukur kalibrasi. Ray-plane RGB saja: rgbd2 0.22 cm, rgbd 5.2 cm di F5 → ekstrinsik rgbd memang salah.
- B65: depth rgbd sehat pagi 2026-10-15 (kedipan 10-14 tidak tereproduksi, penyebab terbuka). Firmware rgbd sudah
  disamakan ke 5.17.0.10 (backup 5.16.0.1 di /home/user1/realsense_fw_backup/).
- CALIB-CHARUCO lama (papan diam di satu tempat, satu bidang) lebih buruk dari incumbent → butuh banyak pose 3D tersebar.

TUGAS SESI INI = DISKUSI + §A DRAF, NOL gerak HW:
1. Tanyakan operator dan catat jawabannya:
   a. Target: ChArUco kecil atau AprilTag di gripper / link 6 / flange? Ukuran, kekakuan dudukan, cara mengukur offset
      target→TCP (±1 mm?). Lengan mana (pertimbangkan torsi joint_2 ≤ 14 N·m, beban tambahan).
   b. D435IF (327122073095) sudah DIPASANG operator di gripper salah satu lengan untuk sistem berikut, saat ini tidak
      dipakai: lengan MANA, massa + dudukan? URDF belum memuatnya → dampak ke tabrakan (probe/MoveIt) dan torsi
      (prediksi RNEA). Harus dimodelkan atau dibuktikan aman SEBELUM gerak lengan itu.
   c. Apakah D435IF (eye-in-hand) mau dipakai juga di G63 (mis. memeriksa target), atau tetap tidak dipakai.
2. Rancang protokol hand-eye eye-to-hand (target di lengan, kamera tetap): jumlah & sebaran pose (dekat/jauh, z 0.8–1.4,
   zona arm_1…arm_4 per kamera), visibilitas di kedua kamera, plan-check torsi/tabrakan/env (peta env perlu verifikasi:
   operator memindahkan laci), solver (PnP per gambar → AX=XB / pose-graph), uji-tahan (pose terpisah), gerbang
   (lengan/target ≤ 5 cm? lebih ketat? — tentukan sebelum data), dan uji bias depth-vs-jarak dengan target yang sama
   (rasio d_ukur/d_PnP; keputusan on-chip/Tare terpisah, JANGAN skala via TF).
3. Tulis docs/p1_g63_charuco.md §A DRAF (dugaan bernomor mulai D630). §A dikunci (salinan + sha256) hanya sesudah
   operator setuju; commit per bagian, hanya berkas sendiri (jangan git add -A).
4. Siapkan alat luring (deteksi target, solver, self-test sintetis, plan-check mock domain 77 octomap_updaters:=false)
   bila waktu cukup.

ATURAN: NOL gerak HW tanpa izin operator per langkah (termasuk pulang/K0). Kamera saja boleh hanya dengan izin.
Jalankan rec_health.py (docs/results/p1_cam6dof/depth_health_20261015/) terus-menerus setiap kali kamera hidup.
Margin = tolak; penyaring tidak dilonggarkan. Fail loud. Jangan sentuh /home/kubota. Bahasa Indonesia, jawaban ke
pengguna singkat, dokumen lengkap. Dokumen fase + git menang atas prompt ini. Akhir sesi: rekomendasi model + effort.
```

Rekomendasi: model kuat, effort **high** (desain protokol; galat desain bisa diam).
