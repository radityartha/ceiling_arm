# Prompt pembuka sesi G63b — desain ulang target LURING (salin ke sesi baru)

```text
Lanjutkan G63 di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy) sebagai fase G63b: DESAIN ULANG target kalibrasi
ekstrinsik kedua D455 (eye-to-hand, acuan = FK lengan) sesudah HW P0 2026-10-15 GAGAL deteksi. Sesi ini LURING: NOL gerak HW,
NOL kamera, NOL mock HW kecuali diizinkan. Hasil akhir = §A baru (draf) + alat teruji + daftar keputusan operator.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan), docs/p1_campaign_state.md (baris G63, update terakhir),
docs/p1_g63_charuco.md (§A terkunci + A8 + §B1–B3 — WAJIB, terutama B1.8–B1.11 dan B3), docs/results/p1_g63/g63_runbook.md.
Memori: /home/user1/.claude/projects/-home-user1-Documents-ceiling-arm/memory/MEMORY.md (g63-charuco-gripper-calib).

KEADAAN (2026-10-15 akhir, commit d1b45f4): papan+kardus A/B DILEPAS, gripper arm_2/arm_3 DIBUKA, semua lengan REST, g1 PARKIR
1.50, g2 0, stack/kamera MATI. ⚠ jari arm_2 di batas buka terbaca −0.0284 rad → scripts/k0_check.py standar MENOLAK jari arm_2
(selisih 0.0084 rad, mekanis, bukan perintah) — catat, jangan diubah diam-diam; usulkan ke operator (terima offset / ukur ulang).
Ekstrinsik incumbent tetap; DEMO-HW tetap menunggu.

TEMUAN P0 (dasar desain ulang, sudah terukur — jangan diperdebatkan tanpa data baru):
 (a) Papan A4 4×5 @ 48.43 mm, marker ChArUco 4X4 36 mm: di 1.04 m hanya 4/10 marker terbaca, 2 sudut ChArUco; di 1.44 m 0 marker.
     Fallback kisi + paritas: lemah/AMBIGU (28 vs 26; 48 vs 41) karena kotak putih berisi marker (~50 % hitam) dan kotak hitam
     cetakan abu + garis terang horizontal (artefak cetak). Prior FK tak bisa melabel (galat prior ≤ 0.18 m > 1 kotak).
 (b) Pose terkunci berjarak 1.0–2.2 m dari kamera (kamera di z ≈ 2.08 m, sudut sel berlawanan) — sebagian besar > 1.2 m.
 (c) Papan ditempel di jari gripper → jari arm_2 membuka 1.4° di bawah beban/momen papan (G7 ✗), kembali saat papan lepas.
 (d) j4 arm_2 berburu terus pp 3.2e-3 rad (0.18°) di pose A-02 (torsi j4 0.3–1.1 N·m, dekat nol) → capture "tidak diam" ditolak.
 (e) Lemari laci + barang lemari samping TIDAK di peta env → probe_g63 --g63-extra-env results/p1_g63/s0c_extra_env.npz (wajib
     bila HW lagi; peta bersama tidak diubah; S0c ulang bila tata letak berubah).
 (f) Data pengembangan (BUKAN data kalibrasi): 30 frame mentah/kamera di results/p1_g63/hw_20261015b/cap/frames/ (A-R0 rgbd2
     1.44 m, A-01 rgbd 1.04 m) + cap/poses.jsonl (TF, K, joints). Ada orang di bingkai → jangan di-commit / dipublikasi.

TUGAS (diskusi dulu, lalu luring):
 1. Pertanyaan operator SEBELUM desain (satu putaran, opsi + rekomendasi): ukuran cetak maksimum yang bisa dibuat (A3? A2? foam
    board?), bahan kaku tersedia, cara dudukan yang TIDAK lewat jari (pelat/braket di gripper_base_link/flens, atau penjepit),
    massa maksimum, boleh/tidak papan dua sisi, toleransi jumlah gerak.
 2. Usulan target berbasis angka (piksel per kotak di 1.0/1.5/2.0/2.5 m dengan fx ≈ 643, pada kemiringan pose): kandidat utama =
    papan catur POLOS (tanpa marker; 180° dibedakan oleh jumlah kotak ganjil×genap; identitas papan A/B dari lengan pembawa/ROI,
    buktikan tidak tertukar) dengan kotak ≥ 2× sekarang; bandingkan dengan alternatif (ChArUco kotak besar, AprilTag besar, papan
    lingkaran asimetris). Kriteria: label 0 salah + akurasi sudut di rentang jarak pose, paritas/identitas kuat, massa/torsi.
 3. Uji detektor: (i) sintetis (render g63_detect.render, blur/noise/kemiringan, jarak 0.8–3.0 m), (ii) frame P0 tersimpan (f)
    sebagai cek realisme (papan lama — hanya untuk kalibrasi model blur/kontras/artefak, bukan bukti papan baru). Aturan buang
    didaftarkan SEBELUM data baru. Detektor/alat baru di results/p1_g63/ (atau g63b/); kode bersama TIDAK diubah.
 4. Dudukan + model: URDF payload baru (g63_urdf.py) dengan dudukan kaku di link non-jari, massa/torsi RNEA (batas 14, 97 %),
    tabrakan papan; pose dibangkitkan ulang (g63_poses.py) dengan batas jarak dari butir 3 dan penghindaran pose j4 torsi≈0
    (usul aturan, mis. |τ_j4 statis| ≥ ambang, atau toleransi diam berbasis gerak papan — beri angka, operator memutuskan).
 5. Plan-check mock domain 77 (octomap_updaters:=false, --extra-env, --goal-joints — cek bug eec3a4e sudah diperbaiki) bila
    operator setuju; tanpa HW.
 6. §A baru DRAF (docs/p1_g63_charuco.md bagian baru "G63b" atau berkas p1_g63b_*.md): protokol, gerbang (G1–G8 diulas; G4′/G6′
    dibawa), dugaan bernomor baru (D639…), keputusan operator harfiah. Kunci hanya atas persetujuan operator.

ATURAN: margin = tolak; penyaring tidak dilonggarkan; skala via TF dilarang; launch/TF default tidak diubah; fail loud; commit
per bagian hanya berkas sendiri (git add <path>, jangan -A); berkas keadaan diperbarui (jangan timpa "Langkah berikut" sesi
lain); job panjang setsid nohup + log; jangan sentuh /home/kubota; disk / 99 % (jangan simpan render besar). Bahasa Indonesia,
jawaban singkat. Akhir sesi: laporan + papan skor + rekomendasi model + effort sesi berikut (HW nanti = high).
```

Model: kuat (Opus), effort **medium** (luring; sintetis + frame tersimpan menangkap galat detektor). Naikkan ke **high** bila
sesi menyentuh mock berantai/keputusan dudukan yang mempengaruhi torsi/tabrakan HW.
