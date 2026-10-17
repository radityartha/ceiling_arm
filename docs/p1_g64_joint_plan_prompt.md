# Prompt pembuka sesi G64 JOINT-PLAN (perencanaan gabungan gantry + lengan) — PRIORITAS operator 2026-10-17 — salin ke sesi baru

```text
Lanjutkan G64 JOINT-PLAN di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak lengan/rel/gripper tanpa izin.
Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin.

MASALAH (operator 2026-10-17, PRIORITAS sebelum DEMO-SYS dan sebelum data paper): sistem merencanakan gantry dan lengan TERPISAH —
gantry bergerak dulu dengan lengan WAJIB REST (pose_to_g S12), lalu MoveIt merencanakan lengan saja. Lengan REST menggantung ke
z ≈ 0.96 → target di dekat perabot tinggi (tas di atas lemari sisi gantry 2, TAS-RAK) gagal: dekat = sapuan gantry REFUSE G33 /
START-JUMP (REST sudah bertabrakan di scene); jauh ≥ 0.3 m = torsi j2 > 14 / RETRACT-BLOCKED (docs/p1_tas_rak.md, 0/14).
Operator: "sistem kita masih kurang pintar … buat perencanaan gabungan gantry dan arm, bisa melipat dulu sambil bergerak". Kalau perlu
dari awal.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 + §4.1, docs/p1_campaign_state.md (TAS-RAK, DEMO-ZONE-HW terbaru),
docs/p1_tas_rak.md, results/p1_g32/pose_to_g.py (S12/S13/S23/S28/G33), scripts/reach_dwell_probe.py (_plan_and_screen, START-JUMP,
retract_check, predict_peak_torque), scripts/env_collision.py, results/p1_g29/g29_rot_screen.py, docs/p1_g54_parallel.md (FJT
langsung), memori: ceiling-arm-fold-is-torque-limited, stow-pose-high-for-occupants, gantry-bridge-debounce, gantry-traverse-speed,
interarm-collision-srdf-disabled, g40-env-aware-sched, g42-joint-runner-diag, no-shared-zone-at-rot0, demo-sys-uses-p1-scheduler.

SUDAH ADA (cek, jangan tebak): grup MoveIt 8-DOF `gantry_N_with_arm_K` (rel + rot + 6 sendi; rantai, "for GNG-seeded IK") di
trailer_workcell.srdf; controller HW `gantry_N_with_arm_controller` (rel + rot + 12 sendi, aktif 7/7); bridge gantry mengikuti
perintah JTC (debounce, traverse ≈ durasi perintah, berhenti ≤ 1 mm pendek, rotasi pendek ≤ 0.81°, 31.4 mm/s, 10 °/s). KDL "not a
chain" hanya grup 14-DOF.

TUGAS (urut, ✓/✗ harfiah):
0. DIAGNOSIS LURING (sebelum desain): pisahkan kegagalan karena (i) PERENCANAAN TERPISAH (target terjangkau secara statis —
   IK + torsi statis RNEA ≤ 14 + bebas env — tetapi gagal karena REST saat traverse / START-JUMP / retract) dari (ii) FISIKA
   (tidak ada konfigurasi statis yang torsi-aman/bebas env). Himpunan uji: TAS-RAK (0/14), DEMO-OFF rak, G42 14 seed gagal
   (joint_2 × rak), G38/no-shared-zone, peta kapabilitas. Angka: berapa % titik "terjangkau statis" yang gagal oleh sistem sekarang
   → itulah ruang perbaikan. Ukur juga torsi statis pose lipat parsial (angkat jari 10–30 cm dari REST) — lipat penuh dokumen 12.78
   N·m terukur, RNEA meremehkan naik ×2.74 (memori fold).
   0b. KALIBRASI ULANG BATAS TORSI (operator 2026-10-17: "J2 terlalu ketat? pernah diuji mati di 14?" — BELUM PERNAH; 17.61 terukur
   2026-08-17 tanpa fault; 14 = rating URDF, bukan pemutus; firmware fault di puncak KA-75+ ≈ 30.5 (belum dicek PDF primer);
   nominal kontinu 12 = batas panas). Penyaring = RNEA + offset tetap j2 +7.7 vs 14 → RNEA mentah ≤ 6.3. DEMO-ZONE-HW: prediksi 13.61,
   terukur 10.40 (offset nyata 4.5). Kerjakan: kumpulkan SEMUA pasangan prediksi (log probe 'torsi RNEA+offset') vs puncak terukur
   (joint_states CSV, js_window.py) dari G57/G60/CAM-6DOF/DEMO-ZONE-HW, per lengan/sendi/fase (transit vs tahan); p95 sisa per
   lengan; hitung ulang berapa penolakan TORQUE-UNSAFE (TAS-RAK, G42) yang lolos dengan offset terkalibrasi. Usulkan batas ke
   operator (tombol) dengan angka — mis. terukur-setara 14 + transit singkat ≤ X dengan batas durasi. JANGAN uji sampai fault (LED
   merah = reset fisik, fault_controller tidak di-spawn). Batas keras AUTO-STOP > 14 terukur tetap sampai operator memutuskan.
   0c. ARTI `effort` (SEBELUM 0b memutuskan batas): `/joint_states` effort = kortex_driver `feedback_.actuators(i).torque()`. Gen3
   LITE kemungkinan TANPA sensor torsi sendi (Gen3 penuh punya) → nilai mungkin estimasi ARUS MOTOR (termasuk gesekan gearbox;
   cocok dengan offset +3…+9 saat bergerak, sedangkan tahan diam cocok RNEA 9.60 vs 9.25). BELUM diverifikasi. Cek dokumentasi
   Kinova (Kortex API ActuatorFeedback, manual Gen3 Lite / spec KA-75+: apakah 14 N·m = torsi KELUARAN; sensor torsi ada/tidak) —
   kutip sumber primer; bila tidak didapat, tulis "tidak terverifikasi". Konsekuensi: bila effort = motor + gesekan dan rating =
   keluaran, batas "terukur ≤ 14" sendiri ikut terlalu ketat → masuk usulan batas 0b (keputusan operator).
1. DESAIN — tulis 2–3 opsi ke operator (tombol) SEBELUM kode besar, dengan rekomendasi:
   A) Bertahap minimal: lengan ke POSE TRANSIT bersertifikat (lipat parsial, torsi statis + transit tersaring) via MoveIt → gantry
      traverse dengan pose transit itu (S12 digeneralisasi: "REST atau pose transit bersertifikat", S28/G33 memakai konfigurasi
      TERUKUR, tidak dilonggarkan) → lengan ke target via MoveIt. Pakai ulang semua penyaring.
   B) Whole-body MoveIt: OMPL di grup 8-DOF (gantry + lengan) dengan scene env + validitas torsi (penyaring pasca-rencana + replan) +
      antar-lengan S18; eksekusi via `gantry_N_with_arm_controller`/FJT — risiko: bridge gantry tertinggal dari trajektori (lipat vs
      traverse tidak sinkron) → ukur galat pelacakan bridge di mock/HW dulu, skala waktu ≤ 31 mm/s / 10 °/s.
   C) Varian A di dalam penjadwal (oracle/P1″ tahu pose transit, biaya lipat/buka, amplop) — perlu agar DEMO-SYS memakai sistem.
   Kriteria pilih: % titik (i) yang jadi bisa; tidak ada regresi pada jadwal yang dulu lulus (G57 6/6, G60); torsi ≤ 14 tanpa
   melonggarkan; kesederhanaan (CLAUDE.md Rule 2).
2. §A G64 (dugaan bernomor lanjut D667) → kunci (salinan + sha256) SEBELUM kode/data. Lalu implementasi + uji unit + MOCK 77:
   TAS-RAK sebagai tolok ukur (tas ~(0.98, −0.83, 1.09), p_cmd z 1.34), regresi seed lama, kontrol negatif (titik fisika-mustahil
   tetap DITOLAK dengan alasan).
3. HW hanya sesudah mock lulus + izin per gerak: mulai dari pose transit tunggal di tempat (lengan saja, gantry diam) → torsi terukur
   vs prediksi → traverse pendek dengan pose transit → baru TAS-RAK. --tau-max 14; MARGIN = TOLAK.
4. Dampak paper: P1 sekarang = model terpisah; catat apa yang berubah (biaya waktu lipat, kelayakan) dan apakah data P1 lama tetap sah
   (keputusan operator). Papan skor §B, state, memori, rekomendasi model/effort.

ATURAN: penyaring tidak dilonggarkan; fail loud; setsid nohup; jangan pkill -f / pola regex yang juga muncul literal di baris
perintah sendiri; jangan sentuh /home/kubota (disk / 99 %, ~16 GB bebas); commit per bagian hanya berkas sendiri. Bahasa Indonesia,
singkat (caveman). Keadaan sel (2026-10-17 11:50): papan DILEPAS dari semua lengan (K0: ukur ulang jari), semua REST, g1 1.40, g2 0,
stack + kamera + mock MATI, botol putih di M1, tas di atas lemari sisi gantry 2.
Sesudah G64: DEMO-SYS (docs/p1_demo_sys_prompt.md) memakai sistem yang sudah diperbaiki.
```

Model: Opus, effort **high** (perubahan arsitektur perencana + gerak gantry/lengan gabungan; galat sinkron/torsi bisa diam).
