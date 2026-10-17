# Prompt G64-C fase 4–6 (mock 77 → HW → laporan) — salin ke sesi baru

```text
Lanjutkan G64-C di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy) mulai FASE 4 (MOCK 77). NOL gerak lengan/rel/
gripper HW tanpa izin. Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin.

BACA PENUH dulu: CLAUDE.md; docs/p1_g64c.md (§A TERKUNCI sha256 712fa8c7… — JANGAN diubah; §B1–B4 = fase 1–3);
docs/p1_g64c_prompt.md (fase + ATURAN asli, tetap berlaku); results/p1_g64c/ (g64c_make.py, g64c_demo.py, g64c_strict.py,
g64c_k3.py, run_c3/demo_candidates.json = jadwal TAS mode {1: REST, 2: T1}: g1 (1.40, 0) M1 arm_1; g2 (0.45, −35°) TAS arm_3);
diff fase 3 (commit 6747034): scripts/return_rest.py (--target, --strict), scripts/reach_dwell_probe.py (--retract-target),
p1_g31/g31_screen.py (events mode, walk), p1_g54/run_g54.py (rr_cmd, traverse_cmd), p1_g32/pose_to_g.py (--allow-t1),
p1_g32/run_g32.py (menolak T1), p1_g47/g47_home.sh. Harness mock lama: results/p1_g55/g55_mock.sh, p1_demo_sys/mock_sys.sh,
p1_g57 (HW paralel), memori ros2-launch-pid-leak / move-group-hw-memory-leak / bg-jobs-die-with-session / workcell-known-issues.

KEADAAN (2026-10-17 sore): fase 1 ✓, fase 2 ✓ (C2 harfiah 7/8, s11 = drift cache), fase 3 ✓ luring (C5 909 event 0 beda).
🔴 T1 di batas j3 (9.98/10, semua garis T1 tau_excess −0.028). Biaya lipat/buka solver SEMENTARA 45.34 s.
Keputusan terbuka untuk operator (tombol): stop TAS terpilih (0.45, −35) j2 cabang 5.35 (sistem 13.71/14); (0.55, −35) j2 4.04 —
penjadwal memilih makespan, bukan sisa torsi; mengubahnya = kriteria baru = §A baru. Tanyakan SEBELUM HW.

FASE 4 (MOCK 77, fake HW, domain 77, octomap_updaters:=false):
 0. Gerbang C5 hidup: `run_g54 --dry` (seed lama g55_selected s7 + s10) = DRY sebelum fase 3 (argv tugas sama) dan kandidat TAS
    DRY mencetak argv fold/retract-T1/traverse --allow-t1/unfold; `g31_screen --task-cli` plan-only seed lama = G57.
 i.  Regresi 2 seed lama mode REST plan-only + eksekusi run_g54 (= G57: semua tugas SUCCESS, urutan event sama).
 ii. TAS mode T1 (run_c3/demo_candidates.json, variant DEMO, seed 0): plan-only (g31_screen --task-cli) PLANNED; eksekusi
     run_g54 di mock: fold → traverse g2 OLEH SISTEM → tugas SUCCESS → retract T1 → pulang p0 → unfold; 0 TUCK-REFUSED/START-JUMP.
 iii. Kontrol negatif (1.0, 0, 0.60) ditolak solver (0 event).
 iv. Ukur waktu fold/unfold (k = 3, dinding, per event) + RNEA rencana lipat (j2 DAN j3) → G64C_C_FOLD → solve ulang C3
     (g64c_demo.py, dir run baru) + ulang D670 (g64c_chain2.sh) dengan biaya terukur; laporkan apakah s18 membalik.
FASE 5 HW + FASE 6 laporan: sesuai §A C9 + p1_g64c_prompt.md (lipat di tempat 1 lengan → ukur j2 + j3 vs RNEA+offset → buka →
dua lengan → traverse pendek T1 → TAS lewat run_g54; --tau-max 14, K0 ukur ulang). Lalu §B, state, memori, dampak paper
(P1 lama = mode REST), DEMO-SYS ulang dengan g64c_demo.py (G64C_ENV=1).

ATURAN: = p1_g64c_prompt.md (penyaring tidak dilonggarkan, MARGIN = TOLAK, kotak 10 mm, fail loud, setsid nohup, jangan pkill -f /
pola yang muncul di baris perintah sendiri, jangan sentuh /home/kubota, commit per bagian hanya berkas sendiri, Rule 6 checkpoint).
Bahasa Indonesia, singkat (caveman). Keadaan sel (tidak disentuh sejak 11:50 10-17): semua lengan REST, papan DILEPAS, g1 1.40,
g2 0, stack + kamera + mock MATI, botol putih di M1, tas kertas di atas lemari sisi gantry 2.
```

Model: Opus, effort **high** (mock pertama jalur lipat; fase 5 menggerakkan HW dengan j3 tanpa cadangan).
