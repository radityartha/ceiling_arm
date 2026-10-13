P1 — G61 OFFLINE: tutup celah G57 (runner sehat-move_group + HALTED, koreksi target thd rotasi gantry terukur, torsi transit,
s0 HW/sim 1.29). Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus, effort HIGH (perubahan di jalur keselamatan runner HW +
penyaring; galat yang diam: penjaga yang tak pernah memicu, koreksi target yang lolos penyaring tanpa disaring ulang).
Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari docs/p1_campaign_state.md ("Langkah berikut").

BACA DULU (penuh): CLAUDE.md; MEMORY.md; docs/p1_campaign_state.md; docs/p1_g57_hw.md (§A, B0–B8 — data HW nyata); docs/p1_g54_parallel.md
§A–B (rancangan runner); docs/p1_g56_hw_day.md §B (injeksi stop, FAULT_RE); docs/p1_g27_z140_diag.md (oracle'''' = torsi transit
REST → s); docs/p1_g53_p1p_serial.md (biaya P1″); docs/results/p1_g54/run_g54.py; scripts/reach_dwell_probe.py (GOAL-MISMATCH
`GOAL_POS_TOL_M` 0.003, `--goal-joints`, `collect()` tau watch); docs/results/p1_g32/run_g32.py (`task_cmd`).

NOL gerak HW. Stack HW MATI (sel home sejak G57 17:30). Mock `ROS_DOMAIN_ID=77` (fake HW, BUKAN data HW) dengan
`octomap_updaters:=false` (2 updater bocor juga di mock saat lengan bergerak). Sesi lain (CAM-6DOF) bisa aktif di repo ini: SEBELUM
mock cek `ROS_DOMAIN_ID=77 ros2 node list --no-daemon` kosong + tidak ada job berat; berkas keadaan = milik bersama (tambah baris
G61, jangan timpa "Langkah berikut" sesi lain tanpa membaca isinya). Job panjang: setsid nohup + log line-buffered +
cache bisa dilanjut (memori bg-jobs-die-with-session). JANGAN pgrep/pkill -f dengan nama yang ada di baris perintah sendiri.

DATA HW G57 (sudah ada): `docs/results/p1_g57/hw/` — per run `p1hw_par_s<S>_<V>_{run.json,runner.log,evNN_task.log,evNN_probe_try0.json,
evNN_traverse.log}`, pulang `p1hw_home_*`, `g57_report.json`, monitor `p1hw_mon_samples.csv.gz`, joint_states
`g57_joint_states_part1.csv.gz` (11:18–13:08:49) + `part2` (13:46–17:30) (UNTRACKED, ada di disk; kolom t,joint,pos,eff).

TUGAS — dokumen fase docs/p1_g61_offline.md: §A + dugaan D550+ DIKUNCI sebelum mengubah kode (salinan results/p1_g61/g61_A_locked.md
+ sha256), §B ditulis, commit per bagian, berkas keadaan diperbarui sesudah tiap langkah berarti. §B menang atas §A.

C1 — Runner sehat (run_g54). G57 B4: move_group mati (OOM) 13:11:45, run s10 lama 13:32 lolos K0/DRY, keenam tugas EXEC-FAIL →
   HALTED, runner TETAP menggerakkan rel. Rancang + uji: (a) pra-cek sebelum t0: proses move_group hidup + `/move_group` menjawab
   (mis. `get_planning_scene` ≤ 5 s) + env_static_map di scene; (b) penjaga kontinu: move_group/ros2_control_node mati → AUTO-STOP;
   (c) kebijakan HALTED/EXEC-FAIL: AUTO-STOP KEDUA gantry vs lanjut (G54 A3 "dicatat, lanjut") — putuskan di §A dengan alasan; G57
   s12 lama GOAL-MISMATCH juga HALTED (nol gerak, runner lanjut, ev berikut sukses) — bedakan "lengan tidak bisa" vs "sistem
   rusak". (d) perekam: berkas joint_states tidak tumbuh > N s → peringatan/stop (G57 13:08 perekam macet diam).
   Uji mock: kontrol negatif — matikan move_group (SIGKILL per PID) di tengah smoke → AUTO-STOP ≤ 2 s, tak ada alat diluncurkan
   sesudahnya; pra-cek menolak start tanpa move_group; kontrol positif smoke normal 6/6 tanpa pemicu palsu. Default lama tetap
   bit-identik bila memungkinkan (opsi baru), atau tulis kenapa tidak.
C2 — Koreksi target thd gantry terukur. G57 s12 lama: traverse g1 → (0.65, +40°) berakhir +39.52 / +39.58° (bridge pendek ≤ 0.81°,
   memori G51) → `--goal-joints` (dihitung untuk 40.00°) meleset 3.18–3.86 mm > 3.0 → GOAL-MISMATCH (coba 1 5/6; coba 2 lolos tipis
   2.69 mm). Ukur dulu: dari SEMUA traverse G57 (`evNN_traverse.log` "akhir:") sebaran galat rot/lin; untuk tiap tugas G57 hitung
   galat TCP yang ditimbulkan (FK goal-joints di pose gantry terukur vs target) → berapa tugas yang dekat 3 mm. Opsi (pilih di §A):
   (i) IK ulang goal-joints di pose gantry TERUKUR (seed = goal lama), lalu SEMUA penyaring tetap jalan di probe (torsi, env, interarm,
   reservasi) — tidak ada penyaring yang dilewati; (ii) traverse koreksi sampai |err_rot| ≤ x (bridge deadband 0.3 + driver 0.5 →
   mungkin tak bisa); (iii) lainnya. Batas GOAL-MISMATCH TIDAK dilonggarkan (MARGIN = TOLAK). Uji: mock tidak punya rotasi pendek →
   suntik (pose_to_g ke rot −0.5° dari jadwal, atau kontrol negatif offline dengan FK) dan tunjukkan s12 lama ev 5 lolos dengan
   koreksi + ditolak tanpa koreksi; regresi: jadwal tanpa galat → argv/goal identik.
C3 — Torsi transit. G57: s10 baru ev 12 arm_3 TORQUE-ABORT 12.44 (j2) di TRANSIT (q2 22.7 → 54.9°, ~10 ± 1.5 N·m, 7 sampel > 12,
   di target 4 N·m; prediksi probe statis-ujung 13.20/14); s3 G60 ev 10 arm_4 13.22 (prediksi 12.91). Analisis semua tugas G57: torsi
   puncak terukur per sendi (joint_states eff, jendela tiap evNN_task) vs prediksi probe (RNEA+offset ujung) vs oracle'''' (transit
   REST → s, G27). Catatan: tugas ke-2 di satu stop berangkat dari pose tugas sebelumnya, BUKAN dari REST — apakah oracle''''
   memodelkan transit tugas → tugas? Keluaran: tabel pred vs ukur, galat, rekomendasi penyaring (mis. transit sepanjang lintasan
   rencana di probe). Keputusan operator 2026-10-12: > 12 bukan gagal, batas keras > 14, HW `--tau-max 14` — jangan ubah itu.
C4 — s0 baru HW 190.70 vs sim 148.16 (1.287), mock 163.4: tiga tugas arm_3 (g2 (0.15, −30°)) time_to_dwell 31.7 / 47.5 / 54.0 s,
   jeda rencana → MOVED 27 / 43 / 50 s (satu percobaan PLANNED). Dekomposisi dari log probe (layar, perencanaan, eksekusi, dwell),
   bandingkan dengan biaya P1″ (G53) dan mock G55; jelaskan selisih; usul perbaikan model bila perlu. Juga: sim meremehkan jadwal lama
   banyak-traverse (s8/s18 lama HW/sim 1.15/1.19) — periksa biaya traverse/layar G33 di g54_sim vs HW.

GERBANG G61: C1 kontrol negatif + positif mock lulus; C2 s12 ev 5 lolos dengan koreksi + ditolak tanpa koreksi, regresi bit-identik;
C3 tabel pred vs ukur untuk semua tugas G57 + rekomendasi tertulis; C4 dekomposisi + penjelasan s0. Smoke paralel mock 6/6 untuk
s7 baru + s12 lama dengan kode baru (pulang + K0). Lalu runbook paralel (`docs/p1_hw_runbook_parallel.md`) + prompt hari HW berikut
diperbarui (bring-up `octomap_updaters:=false`, `--tau-max 14`, pra-cek baru).

ATURAN: NOL gerak HW. MARGIN = TOLAK; penyaring tidak dilonggarkan. Perubahan surgical, cocokkan gaya kode. Fail loud. Akhir sesi:
laporan + rekomendasi model/effort sesi berikut; berkas keadaan "Langkah berikut" satu kalimat.
