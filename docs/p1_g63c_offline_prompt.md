# Prompt pembuka sesi G63c LURING (salin ke sesi baru)

```text
Lanjutkan G63b → G63c LURING di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak HW, NOL kamera.
Mock domain 77 hanya bila perlu dan dengan izin (tombol AskUserQuestion, operator tidak suka mengetik).

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan), docs/p1_campaign_state.md (baris G63b terakhir),
docs/p1_g63b_redesign.md §B3 + §B4 (hasil HW 2026-10-16), memori g63-charuco-gripper-calib.

DATA (lengkap, jangan ambil ulang): docs/results/p1_g63/g63b/hw_20261016_finger/
  poses.jsonl (sudut sendi + deteksi tiap tangkap), ds_finger_all.json, ds_finger_all_noB13.json, frames/ (lokal, ada orang
  → TIDAK di-commit), skrip eksperimen: flex_models.py, joint_offset_model.py, joint_offset_both.py (+ .log).
Temuan: solve kaku (g63b_solve) TOLAK; model offset nol sendi j1–j5 per lengan σ 5.96 → 1.19 px, tapi uji-tahan p95 27 mm,
R0 5 mm, dan j1 ≈ +3° di KEDUA lengan (arm_2 & arm_3, beda gantry) → curiga rotasi bersama tak dimodelkan.

TUGAS (urut, tiap langkah = hasil terukur, ✓/✗ harfiah):
1. Penjaga detektor papan IDENTIK: B-13 rgbd2 = papan A (arm_2 REST) terbaca sebagai B. Rancang penjaga (mis. prior SEMUA
   papan di bingkai → tolak bila kandidat lebih dekat ke prior papan lain / ambigu), uji ulang pada frames/ semua pose:
   B-13 harus ditolak, deteksi benar lain tidak hilang.
2. Model kinematik: bandingkan (a) offset j1–j5 per lengan, (b) + rotasi/geser basis per gantry, (c) hanya rotasi gantry/dunia
   bersama; j6 ≡ T_lb (kunci 0). Waspadai degenerasi kamera ↔ basis lengan (cek rank/std, jangan percaya std kecil saja).
3. Validasi silang: fit dengan arm_2 saja → uji pose arm_3 (dan sebaliknya); model sah hanya bila kamera sepakat antar-lengan
   dan uji-tahan/R0 lolos gerbang §A4 (G3, G6′). Hasil lalu dibandingkan dengan cek pita ukur lama (memori demo-hw-rgbd2-range-error).
4. Bila ada model tervalidasi → draf amandemen §A (model + gerbang) untuk persetujuan operator SEBELUM ekstrinsik dipakai;
   bila tidak → tulis apa yang kurang (data/HW) dan rencana minimal.

ATURAN: penyaring/gerbang tidak dilonggarkan; alat terkunci (sha g63b_A_locked.sha256) tidak diubah — eksperimen di berkas baru;
fail loud; job panjang setsid nohup; jangan sentuh /home/kubota; frame tidak di-commit; commit per bagian hanya berkas sendiri.
Bahasa Indonesia, singkat. Akhir: §B baru di docs/p1_g63b_redesign.md (atau doc G63c), berkas keadaan, memori, rekomendasi model/effort.
Keadaan sel (tidak disentuh): papan A/B di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack mati.
```

Model: Opus, effort **high** (model kinematik/degenerasi bisa salah diam-diam).
