# Prompt pembuka sesi G63e — M3: ArUco besar statis + stereo papan genggam (salin ke sesi baru)

```text
Lanjutkan G63d → G63e di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak lengan/rel. Kamera BOLEH
untuk uji kecil kamera-saja sesudah izin operator (tombol AskUserQuestion, satu langkah per izin; operator tidak suka mengetik).

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2, docs/p1_campaign_state.md (baris G63d), docs/p1_g63d_static.md
(semua — kenapa papan statis 41 mm gagal + perbandingan metode B4), memori g63d-static-boards-infeasible, g63-charuco-gripper-calib.
Alat G63d (pakai ulang, jangan ubah yang terkunci g63b/g63c): results/p1_g63/g63d/{g63d_common,g63d_plan,g63d_tape,g63d_solve,
g63d_selftest,g63d_methods,g63d_capture}.py.

INTI M3: (a) penanda ArUco/AprilTag BESAR, satu per A4 (sisi hitam ~18 cm, ID unik), 8–12 lembar STATIS di lantai, kotak A
(310)/B (480), laci (boleh dipindah, operator 10-16 "boleh"), meja tinggi 0.98, dan PERMUKAAN VERTIKAL (muka depan meja tinggi/
lemari, sisi laci, dinding); posisi = trilaterasi pita dari unting sumbu j1 P1..P4 (g1 1.40/g2 0) + tinggi. (b) papan polos 4x5
yang sudah dicetak DIPEGANG operator ~25 pose diam di volume tengah (z 0.4–1.1) → pose relatif rgbd↔rgbd2 (stereo). Solve
gabungan: rig 6 DOF + T_rel. MC G63d: titik ruang kerja med 3.4–3.9 / p95 10–13 mm (tanpa model gagal deteksi).

TUGAS (urut, ✓/✗ harfiah):
1. LURING: PDF penanda (kamus + ukuran; cek raster), detektor ArUco baru (berkas baru) + syarat px aman ≥ 40 px (G63: ~22 px
   gagal); tambah permukaan vertikal di g63d_plan/methods; pilih 8–12 penanda (rgbd ≥ 5).
2. LURING: uji mandiri M3 render (ArUco + papan genggam, blur 1.3–1.6, latar frame lama) + kontrol negatif (penanda salah 3 cm
   HARUS tertangkap tinggal-satu-penanda) → ambang G3/G4 dikalibrasi SEBELUM data.
3. Rencana UJI KECIL kamera-saja (~15 menit, izin): 2–3 penanda di titik terjauh + 5 pose papan genggam → deteksi + ketelitian
   sudut nyata → ganti angka peluang dengan angka terukur. Stack kamera saja (domain terpisah), lampu sesuai B3 (silau → mati).
4. Draf §A G63e (dugaan lanjut dari D648) → persetujuan operator (tombol) → kunci (salinan + sha256) → kampanye penuh (izin).
5. Bila lulus: kamera beku → S-B LURING offset kinematik lengan dari data jari 10-16 (laporan saja).

TANYA OPERATOR (tombol) di awal: printer bisa cetak penanda hitam pekat? papan di jari arm_2/arm_3 sudah dilepas (jawaban 10-16
"Dilepas" tapi belum dikerjakan)? permukaan vertikal mana yang boleh ditempel? ukuran kotak papan polos terukur (4 dan 5 kotak, mm)?

ATURAN: penyaring/gerbang tidak dilonggarkan; fail loud; job panjang setsid nohup; jangan sentuh /home/kubota; frame (ada orang)
tidak di-commit; commit per bagian hanya berkas sendiri (git add <path>). Bahasa Indonesia, singkat (caveman).
Keadaan sel: papan di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack mati.
```

Model: Opus, effort **high** (detektor + solver gabungan baru; galat pita/deteksi bisa salah diam-diam).
