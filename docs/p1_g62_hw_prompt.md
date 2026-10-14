P1 — G62 HW: validasi perubahan G61 di HW (pra-cek move_group/scene/perekam, kebijakan HALTED, `--goal-correct`) + ulang s12 lama
(GOAL-MISMATCH G57) + pasangan s12 bersih + s0 baru (penyaring reservasi). Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus,
effort HIGH (gerak HW nyata; galat diam: koreksi goal yang tidak terpicu / penjaga yang tidak memicu). Tempel ulang prompt yang SAMA bila
sesi putus; lanjut dari docs/p1_campaign_state.md ("Langkah berikut"). Sesi CAM-6DOF punya prompt sendiri — jangan dicampur.

BACA DULU (penuh): CLAUDE.md; MEMORY.md; docs/p1_campaign_state.md; docs/p1_g61_offline.md (§A, §B — perubahan runner, C2/C3/C4);
docs/p1_g57_hw.md (§A, B0–B8); docs/p1_hw_runbook.md §0–2, §7 (bring-up kini `octomap_updaters:=false`); docs/p1_hw_runbook_parallel.md
(§1b G61, risiko P1–P9); docs/p1_hw_day_2026-10-12.md §0, §1–§5 (pola perintah; arsip kini results/p1_g62/hw/).

KEADAAN (sesudah G61, 2026-10-14): run_g54 + pra-cek (move_group, /get_planning_scene ≤ 5 s + env_static_map, perekam tumbuh),
penjaga move_group kontinu, HALTED "sistem rusak" (EXEC-FAIL) → AUTO-STOP / "lengan tidak bisa" → lanjut, peringatan perekam macet,
`--goal-correct` (IK goal di pose gantry TERUKUR bila alat bergeser ≥ 1 mm; semua penyaring probe tetap). Mock 77: A5.1/A5.2 lulus (G61 B4);
A5.3/A5.4 LULUS (G61 B6): kill move_group saat kedua probe tugas aktif dalam penyaringan → AUTO-STOP 0.027 s, 0 CMD sesudah stop,
rc 3; DRY tanpa move_group ditolak sebelum t0, rc 3. G61 skor 8/13; pemulihan REST/PARK + K0 dan shutdown mock dicatat di B6.
BELUM di HW. Sel terakhir (G57 B8): g1 PARK 1.50, g2 0.00, lengan REST, stack MATI. G57: 6/6 pasangan lama vs baru −11…−42 %.

TUGAS — dokumen fase docs/p1_g62_hw.md: §A + dugaan D570+ DIKUNCI sebelum gerak HW pertama (salinan results/p1_g62/g62_A_locked.md +
sha256), §B ditulis, commit per bagian, berkas keadaan diperbarui sesudah tiap langkah berarti. Arsip HW results/p1_g62/hw/.
- §0 TANPA gerak: remount_check → bring-up nyata runbook §1 (WAJIB `octomap_updaters:=false`; SIG_DFL + setsid; PID launch asli;
  SigIgn tanpa 0x2; 4× '6'; 7/7; table1 + table2 ARMED; 0 baris pointcloud_octomap_updater; env_static_map di scene) → monitor +
  perekam (`/tmp/p1hw_joint_states.csv`) → K0 `--rail1 1.50 --rail2 0.00`.
- §1 TANPA gerak: DRY `run_g54 --dry --tau-max 14 --goal-correct` s12 lama (`p1_g52/g52_candidates.json R90`) + s12 G60
  (`p1_g60/g60_selected.json G60`) + s0 baru (`p1_g55/g55_selected.json R90sel`): K0 LULUS, "traverse di luar: []", baris
  `pra-cek: /get_planning_scene … ['env_static_map']` + `pra-cek: perekam … +N B / 3 s`, rc 0. Plan-only nyata k = 3 per jadwal
  (g31_screen --task-cli, tol 0.05°) bila jadwal belum diuji di stack hari ini.
- §2 s12 lama `--goal-correct` (tanya): lulus ⟺ 6/6; ev 5/6 mencatat `koreksi goal` (DITERAPKAN bila rotasi g1 pendek ≥ ~0.15°,
  "tidak perlu" bila tidak) dan PLANNED percobaan pertama; 0 GOAL-MISMATCH karena galat gantry. Lalu pulang (DRY dulu, tanya) + K0.
  Lalu s12 G60 (tanya) → pasangan s12 bersih (G57 memakai coba 2).
- §3 s0 baru (tanya): ulang untuk memisahkan penyaring reservasi (G61 B3: 0.76 + 0.136 s × N keadaan) dari derau; laporkan per tugas
  reservasi N/detik vs G57.
- §4 (opsional) ulangan s7/s10 baru + lama (G57 §5 tidak dijalankan) dengan kode baru.
- §7 matikan bersih + arsip (/tmp hilang saat reboot); joint_states gz tidak di-commit.
- Analisis: pola results/p1_g57/g57_report.py; torsi puncak vs prediksi probe (results/p1_g61/g61_c3_*.py), koreksi goal per tugas
  (`er['goal_correct']` di run.json).

ATURAN KERAS: SETIAP gerak nyata ditanya dulu (AskUserQuestion, satu langkah per persetujuan, termasuk pemulihan K0 dan pulang).
Fault / LED merah / REJECTED / NOT armed / torsi > 14 / benturan / kabel / AUTO-STOP tak terduga / `KOREKSI GAGAL` → BERHENTI, amankan
(lengan REST, DRY dulu), tulis, tanya. JANGAN uji stop saat TRAVERSE. JANGAN SIGKILL move_group di HW (uji itu hanya mock, G61 B6).
JANGAN pkill -f (ps|awk lalu kill PID). Jangan move_dual_table saat ARMED. MARGIN = TOLAK; GOAL-MISMATCH 3 mm tidak dilonggarkan.
`--tau-max 14` (operator 2026-10-12), batas keras > 14. Hanya run_g54. Tidak ada job offline berat selama runner HW jalan. Akhir
sesi: laporan + rekomendasi model/effort sesi berikut; berkas keadaan "Langkah berikut" satu kalimat.
