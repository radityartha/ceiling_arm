# Prompt pembuka sesi G63d — papan statis berjangkar pita (salin ke sesi baru)

```text
Lanjutkan G63c → G63d di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak lengan/rel. Kamera BOLEH
sesudah §A dikunci + izin operator. Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), satu langkah per izin.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan), docs/p1_campaign_state.md (baris G63c terakhir),
docs/p1_g63b_redesign.md §B5–B7 (G63c: kenapa data jari gagal + rencana), memori g63-charuco-gripper-calib,
camera-world-vs-urdf-offset, demo-hw-rgbd2-range-error.

INTI: ekstrinsik D455 TIDAK boleh lewat FK sendi (G63c: kinematika lengan 1–3°/sendi, gauge yaw ↔ kamera). Jangkar = papan
catur POLOS statis (4x5 @ 42 mm, PDF g63b/boards/g63b_board_A.pdf, operator cetak 8 lembar: 6 pakai + 2 cadangan; ukuran kotak
TERUKUR operator menggantikan 42.0) ditempel datar di alas kaku, di meja kerja (z 0.770), lantai (z 0), atas lemari laci.
Posisi diukur pita dari titik unting-unting SUMBU j1 tiap lengan di REST (basis hanya bergantung rel+dudukan: G34 rel = URDF
<= 1 cm, pita basis-basis 0.80 = URDF) — bukan dari ujung tool.

TUGAS (urut, tiap langkah hasil terukur, ✓/✗ harfiah):
1. LURING — rencana penempatan: pilih 6 lokasi (tiap kamera ~4 papan, 2 terlihat kedua kamera), sudut pandang <= 55°, jarak
   <= 2.5 m, tersebar >= 1 m, tidak tertutup lengan REST / papan di jari / lemari (peta env beku + incumbent, margin galat
   incumbent ~10–15 cm). Tiap papan: posisi pusat + arah (yaw) relatif titik unting-unting basis terdekat, dalam cm yang
   bisa diukur operator dengan pita. Gambar rencana proyeksi ke tiap kamera (overlay di frame lama, lokal).
2. LURING — alat: detektor terkunci g63b_detect + penjaga ID g63c_guard (prior dari posisi pita); solver multi-papan per
   kamera (PnP gabungan, papan = pose dunia dari pita, 6 DOF kamera); self-test sintetis (galat pita ±3 mm/±0.5°, blur
   terkalibrasi) + kontrol negatif (satu papan salah 3 cm HARUS tertangkap uji tinggalkan-satu-papan).
3. Draf §A G63d (doc baru docs/p1_g63d_static.md): protokol, aturan buang didaftarkan sebelum data, gerbang (G3 dari
   tinggalkan-satu-papan per kamera, G4 kesepakatan dua kamera pada papan bersama, cek pita lama ARM1-ZONE sebagai info),
   dugaan bernomor (lanjut dari D645). Minta persetujuan operator (tombol) → kunci (salinan + sha256).
4. HW kamera saja (izin): stack kamera saja (domain terpisah, octomap_updaters:=false bila move_group ikut), lampu sesuai
   B3 (silau → mati), tangkap 30 frame per kamera, simpan frame lokal. Operator menempel + mengukur; catat angka harfiah.
5. Solve + gerbang + papan skor. Bila lulus → S-B LURING: kamera dibekukan, data jari 10-16 (g63c_kin.Fit cams_fixed) →
   offset kinematik per lengan, konsisten arm_2 vs arm_3? → laporan saja; koreksi kinematik/ekstrinsik di launch = keputusan
   operator terpisah.

TANYA OPERATOR (tombol) di awal: ukuran kotak terukur; papan di jari arm_2/arm_3 dilepas atau tetap (tetap = ikut prior
penjaga ID, dari FK REST); siapa mengukur pita + alat (meteran, unting-unting); lokasi yang boleh ditempel.

ATURAN: penyaring/gerbang tidak dilonggarkan; alat terkunci (g63b_A_locked.sha256, g63_A_locked.sha256) tidak diubah —
berkas baru di docs/results/p1_g63/g63d/; fail loud; job panjang setsid nohup; jangan sentuh /home/kubota; frame (ada orang)
tidak di-commit; commit per bagian hanya berkas sendiri (git add <path>). Bahasa Indonesia, singkat (caveman).
Akhir: §B di docs/p1_g63d_static.md, berkas keadaan, memori, rekomendasi model/effort.
Keadaan sel awal: papan di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack mati.
```

Model: Opus, effort **high** (rencana jangkar + solver baru; galat pita/penempatan bisa salah diam-diam).
