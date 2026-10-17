# Prompt kampanye G64-C (mode lipat T1 per gantry di penjadwal P1″) — salin ke sesi baru

```text
Lanjutkan G64-C di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak lengan/rel/gripper tanpa izin.
Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin.

KEPUTUSAN OPERATOR 2026-10-17 sore (tombol): desain **C** = penjadwal P1″ memilih per gantry per jadwal mode REST (= sistem
sekarang, predikat bit-identik, nol regresi) atau T1 (lipat REST→T1 sebelum traverse pertama, T1 saat traverse dan di stop, tugas
T1→q→T1, buka T1→REST di akhir). Biaya lipat/buka masuk makespan. A (manual) dan B (whole-body; lag bridge 71 mm/13° > margin)
ditolak. Dasar: docs/p1_g64_joint_plan.md §1.2 (TAS lewat T1 40/40 statis, paksa-T regresi 44/48, garis lipat j2 statis 11.52).
Sesudah G64-C: DEMO-SYS ulang (docs/p1_demo_sys.md, adapter results/p1_demo_sys/demo_solve.py; tas = kontrol POSITIF,
kontrol negatif = (1.0, 0, 0.60); z 1.40 BUKAN negatif).

BACA PENUH dulu: CLAUDE.md, docs/p1_g64_joint_plan.md (§0.1–0.2, §1–1.2), docs/p1_demo_sys.md, docs/p1_campaign_state.md (baris
G64 + DEMO-SYS terbaru), results/p1_g64/g64_whatif.py + g64_whatif_reg.py + g64_transit_T.json, solver P1″ (results/p1_g55/
g55_make.py → p1_g52 → p1_g49 (Feas49/_tuple_job49) → p1_g43 (EdgeMatrix, solve_lazy46) → p1_g42 (G41.set_b: kotak 10 mm!) →
p1_g41 (screen_line, pose_ok_b) → p1_g40 (build, pose table, Edges)), results/p1_g46/g46_make.py (fine lattice), runner
(results/p1_g54/run_g54.py, reserve.py, p1_g32/run_g32.py, pose_to_g.py S12/S28/G33, p1_g31/g31_screen.py events/DRY,
scripts/return_rest.py, scripts/reach_dwell_probe.py start/retract/START-JUMP, p1_g47/g47_home.sh). Memori: demo-sys-bag-rejected-offline,
g64-torque-model-verified, ceiling-arm-fold-is-torque-limited, stow-pose-high-for-occupants, gantry-bridge-debounce,
interarm-collision-srdf-disabled, demo-sys-uses-p1-scheduler, feedback-tau-max-14-operator, ros2-launch-pid-leak.

FASE (✓/✗ harfiah, gerbang per fase; berhenti di gerbang terakhir yang lulus bila waktu habis):
1. §A G64-C (D668…) → kunci (salinan + sha256) SEBELUM kode. Isi minimal: T1 = g64_transit_T.json q6 (sertifikasi ulang di p0
   g1 1.40 DAN 1.50, g2 0, kotak 10 mm); definisi mode; biaya lipat/buka (ukur di mock, bukan tebak); kriteria lulus di bawah.
2. LURING solver: (a) g46 fine lattice dengan kedua lengan T1 (berkas BARU, g46_fine_pose.npz tidak ditimpa); (b) tabel pose (a)
   T1 (berkas baru); (c) predikat tuple T1 = _tuple_job49 dengan partner T1 + garis T1→q (screen_line_any) + kotak 10 mm, cache
   terpisah ber-mode; (d) solver: 4 kombinasi mode (g1 × g2), mode REST memakai objek/cache SEKARANG apa adanya; pilih makespan
   terkecil. Gerbang: K-REST = jadwal 8 seed g55_selected bit-identik di kombinasi REST/REST; TAS (tasks_synth) DITERIMA dengan
   mode g2 T1, M1 tetap, (1.0, 0, 0.60) DITOLAK; rot dalam amplop HW (tambahkan rot_envelope ke solver ATAU catat bahwa runner menolak).
3. RUNNER: event 'fold'/'unfold' (garis sendi lurus REST↔T1 kedua lengan, disaring seperti return_rest: torsi RNEA+7.7, env,
   kotak, se-gantry, antar-gantry; tolak = TUCK-REFUSED); pose_to_g S12 menerima "REST atau T1 tersertifikasi ±0.5°" (flag
   eksplisit, default REST = argv lama bit-identik); retract tugas → T1 (return_rest target); probe start dari T1; g31_screen DRY
   menempatkan T1; reserve.py keadaan T1; g47_home buka T1→REST sebelum PARK. Gerbang: argv/jalur mode REST bit-identik (diff + DRY seed lama).
4. MOCK 77: (i) regresi 2 seed lama mode REST (plan-only + eksekusi, = G57); (ii) TAS sintetis mode T1: plan-only PLANNED + mock
   eksekusi, gerak gantry 2 OLEH SISTEM; (iii) kontrol negatif ditolak; ukur waktu lipat/buka + torsi rencana lipat.
5. HW (izin per gerak, --tau-max 14, octomap_updaters:=false, K0 ukur ulang): lipat di tempat 1 lengan → torsi terukur vs prediksi
   → buka → kedua lengan → traverse pendek dengan T1 → baru TAS lewat run_g54.
6. Papan skor §B, state, memori, dampak paper (P1 lama = mode REST; data tetap sah? keputusan operator), rekomendasi model/effort.

ATURAN: penyaring tidak dilonggarkan (MARGIN = TOLAK, kotak 10 mm G41b — g64_diag lama memakai 0!); fail loud; setsid nohup;
jangan pkill -f / pola regex yang muncul literal di baris perintah sendiri; jangan sentuh /home/kubota; commit per bagian hanya
berkas sendiri (git add <path>); anggaran token CLAUDE.md Rule 6 — checkpoint + sesi baru per fase bila terlampaui. Bahasa
Indonesia, singkat (caveman).
Keadaan sel (2026-10-17 ~14:30, tidak disentuh sejak 11:50): semua lengan REST, papan DILEPAS, g1 1.40, g2 0, stack + kamera + mock
MATI, botol putih di M1, tas kertas di atas lemari sisi gantry 2.
```

Model: Opus, effort **high** (perubahan arsitektur penjadwal + runner; regresi diam-diam dan torsi lipat j3 136° belum terukur).
