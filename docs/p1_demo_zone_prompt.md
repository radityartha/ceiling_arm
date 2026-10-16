# Prompt pembuka sesi DEMO-ZONE langkah 1+2 (salin ke sesi baru)

```text
Lanjutkan DEMO-ZONE (arah C) di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak lengan/rel tanpa izin.
Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), satu langkah per izin.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan), docs/p1_campaign_state.md (baris DEMO-ZONE),
docs/p1_demo_zone.md (analisis + model P3), docs/p1_rgbd2_zone.md (tanda + protokol lama), docs/p1_demo_hw.md (rantai demo),
memori demo-zone-p3-correction, demo-hw-rgbd2-range-error. Data/alat: docs/results/p1_demo_zone/ (zone_data.py,
zone_models.py, zone_p3_fit.json).

INTI: terima galat kamera; koreksi LOKAL P3 (x, y, yaw 14.72° di sekitar pusat (1.8312, 0.6048), geser (−0.1237, −0.1143))
HANYA untuk track rgbd2 di zona meja demo arm_1 (dunia ≈ x 1.60–1.99, y 0.45–0.73, meja 0.77). Ekstrinsik/launch default
TIDAK diubah; yaw 14.7° JANGAN dimasukkan ke TF kamera (merusak lengan dekat > 25 cm). Di luar zona → target DITOLAK.

TUGAS (urut, ✓/✗ harfiah):
1. LURING — node pasca-proses (berkas baru, mis. results/p1_demo_zone/zone_correct_node.py): baca track rgbd2 (cek topik +
   format di rantai demo/object_localizer, p1_demo_hw harness), uji kotak zona (pada xy TERKOREKSI dan track), terapkan P3, z =
   meja 0.77 + setengah tinggi benda (botol demo 7 cm → 0.805; atau track z bila konsisten), terbitkan target terkoreksi + alasan
   tolak. Uji unit: 20 obs lama → residu = analisis (fit RMS 2.6 cm); titik di luar zona ditolak. Commit.
2. LURING — cek oklusi: g1 sekarang 1.40 (tanda dibuat di g1 1.50). Proyeksikan arm_1/arm_2 REST (g63d_common.robot_mask) ke
   rgbd2 di g1 1.40 vs 1.50 → apakah tanda C6/N1 (dan lain) tertutup? Bila tertutup → usul geser g1 1.40→1.50 (gerak rel, IZIN
   terpisah, pose_to_g dgn REST; papan masih di jari arm_2/arm_3 → cek ulang screen/rakitan dulu).
3. §A singkat DEMO-ZONE-LIVE (dugaan bernomor lanjut dari D648, gerbang: tiap tanda uji ≤ 5 cm xy sesudah koreksi, z ∈ [0.79, 0.84],
   satu track tunggal) → kunci (salinan + sha256) SEBELUM data.
4. HW kamera saja (izin): stack kamera + localizer (harness incumbent p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280, domain terpisah,
   tanpa controller), lampu NYALA. Operator: cek tanda C6 + N1 masih di meja (bila hilang pakai M1/C3/C5/N2), botol demo putih 7 cm
   tepat di tanda, menjauh → rekam 15 s per tanda (track mentah + terkoreksi). Matikan bersih (PID milik sesi; jangan pkill -f).
5. Papan skor + §B di docs/p1_demo_zone.md; keputusan berikut: bila > 3 cm atau demo butuh presisi → opsi tanda baru di sudut zona
   + di tinggi pendekatan demo z ~1.05 (gerak arm_1, izin per langkah); opsional botol 3× per tanda (kamera saja).

ATURAN: penyaring/gerbang tidak dilonggarkan; fail loud; job panjang setsid nohup; jangan sentuh /home/kubota; frame (ada orang)
tidak di-commit; commit per bagian hanya berkas sendiri (git add <path>). Bahasa Indonesia, singkat (caveman).
Akhir: berkas keadaan, memori, rekomendasi model/effort.
Keadaan sel: papan polos di jari arm_2/arm_3 (belum dilepas), gripper tertutup, semua REST, g1 1.40, g2 0, stack mati.
```

Model: Opus, effort **medium–high** (node kecil + ground truth tanda menangkap galat; HW kamera saja).
