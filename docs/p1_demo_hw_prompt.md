P1 — DEMO-HW: YOLOE deteksi objek nyata → lengan ke ATAS objek (approach-only). Repo ceiling_arm, branch feat/rgbd-deploy.
Model: Opus, effort HIGH (gerak nyata pertama rantai persepsi → lengan; galat diam: lokalisasi nyata belum pernah jalan, domain ROS
salah = runner tak pernah melihat track, YOLOE bisa mati diam-diam). Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari
docs/p1_campaign_state.md ("Langkah berikut").

BACA DULU (penuh): CLAUDE.md; MEMORY.md; docs/p1_campaign_state.md; docs/p1_demo_off.md (A2 desain, A5, B1–B4, C, §D);
docs/p1_g59_demo.md §B (perbaikan r1 `g` ganda + regex verdict, J1–J7, YOLOE luring); docs/p1_g57_hw.md §B0, B4, B8 (bring-up HW,
OOM move_group, celah terbuka); docs/results/p1_demo/demo_runner.py; docs/results/p1_demo/harness/*.sh; docs/results/p1_g59/harness/*.sh;
docs/p1_hw_runbook.md §0–2, §7.

SCOPE: suara/teks → YOLOE (prompt `bottle`) → object_localizer → target_cli → demo_runner → probe → lengan berhenti di stand-off
0.25 m DI ATAS centroid objek, dwell, lalu REST. TANPA grasp, TANPA angka akurasi (eval = P2). 1–3 jalan bergerak. g1 tetap PARKIR 1.50,
gantry TIDAK diperintah.

KEADAAN (akhir G57, 2026-10-12 17:30): sel home — lengan REST, g1 1.4999 m / −0.25°, g2 0.0008 m / −0.01°, rak tidak dipindah; stack
MATI. Mock G59: P1 meja (1.75, 0.55, 0.97) arm_1 MOVED→SUCCESS→REST 3/3, 70–73 s (sering 1 sampel TORQUE-UNSAFE dulu = normal);
P3 rak / P6 dinding ditolak; jangkauan tool-bawah ~0.7–0.8 m → hanya meja di depan arm_1. YOLOE+mobileclip jalan dari `ros2_ws/`
(`src/reachability_gng/mobileclip_blt.ts` TERPOTONG — jangan luncurkan dari sana). Persepsi NYATA belum pernah (DEMO-OFF lab gelap).

PELAJARAN G57 (WAJIB):
- Bring-up HW: `my_workcell.launch.py use_fake_hardware:=false enable_gantry_bridge:=true use_sim_time:=false octomap_updaters:=false`
  (tanpa arg terakhir move_group OOM ~100–115 mnt). SIG_DFL + setsid, PID launch asli, SigIgn tanpa 0x2, 4× Actuator '6', 7/7,
  table1/table2 ARMED (table1 bisa "NOT armed" sekali sebelum JTC aktif → ARMED pada perintah pertama, tol 5 mm). Bring-up bisa
  SIGABRT `SetServoingMode` timeout (lengan belum siap) → SIGINT, hapus crash dump, ulang sekali.
- Torsi (operator 2026-10-12): lewat 12 N·m BUKAN gagal; batas keras > 14. `demo_runner` masih hardcode `--tau-max 12.0` → tambah
  opsi `--tau-max` (default 12 = argv lama), demo HW pakai 14. Puncak torsi HW ada di TRANSIT, bukan di target.
- `--goal-joints`/target tidak dikoreksi rotasi gantry terukur; di demo gantry diam (PARKIR) → tidak relevan, tetapi K0 g1 rot ≈ 0.
- Perekam `js_record` bisa macet diam (G57 13:08) → cek berkas tumbuh sebelum tiap jalan.
- JANGAN `pgrep -f`/`pkill -f` dengan nama yang muncul di baris perintah bash sendiri (G57: membunuh shell sendiri). ps|awk lalu kill PID.

🔴 DOMAIN ROS (cek di kode SEBELUM mengunci §A): harness DEMO-OFF menaruh kamera + localizer di domain 79 (`env79.sh`) dan
runner/CLI di 77 (mock). Di HW `demo_runner` (sub `/detected_objects/tracks`, panggil probe → MoveIt) harus di domain stack (0), jadi
kamera + `object_localizer` + `target_cli` + `voice_web_ui` + `demo_runner` + monitor juga domain 0. Domain 79 HANYA untuk cek kamera
tanpa stack. Prompt §D lama ("kamera tetap domain 79") salah untuk demo bergerak — tulis di §A0. Buat harness HW baru
(`docs/results/p1_demo_hw/harness/`, env domain 0), jangan ubah harness lama.

GERBANG (tanya operator SEBELUM apa pun, AskUserQuestion): K0 sel = akhir G57 (lengan REST, g1 PARKIR 1.50, g2 0, rot 0/0, rak
tidak dipindah)? K1 operator di e-stop FISIK selama gerak (voice_web_ui TIDAK punya stop)? K2 lampu lab NYALA; satu botol ≤ 0.30 m
berdiri di meja sekitar (1.75, 0.55) world; tak ada botol lain dalam pandangan? K3 tidak ada voice_command_manager /
real_robot_task_server / reach_fusion lain hidup (r2)?

TUGAS — dokumen fase docs/p1_demo_hw.md: §A (kriteria + dugaan D540+) DIKUNCI sebelum gerak pertama (salinan
results/p1_demo_hw/demo_hw_A_locked.md + sha256), §B ditulis, commit per bagian, berkas keadaan diperbarui sesudah tiap langkah
berarti. Arsip di results/p1_demo_hw/.
0. Kunci §A (kriteria: deteksi `bottle` stabil; track |dxy| ≤ 5 cm dari posisi botol yang diukur operator, z 0.85–1.05; plan-only
   `success arm_1 (arm_1:PLANNED)`; jalan bergerak = MOVED → SUCCESS dwell → REST rc 0; torsi ≤ 14; 0 gerak kedua tak diminta).
1. CEK KAMERA TANPA GERAK (stack lengan MATI, domain 79, harness lama `cam_up.sh` + `loc_up.sh`, cwd `ros2_ws/`,
   with_depth_cloud:=false): log seg_router tanpa `inference failed`; `/rgbd/seg/instance_segmentation_labels` memuat `bottle`;
   `/detected_objects/tracks` `bottle` dengan |dxy| ≤ 5 cm dari (1.75, 0.55) dan z 0.85–1.05; simpan rgb (`grab.py`) + rosbag 20 s.
   Gagal → perbaiki penempatan/lampu dulu (jangan perlebar). Matikan kamera (node 0 di 79).
2. Self-test env_collision + interarm; remount_check; bring-up nyata (pelajaran G57); env_static_map di scene; K0
   `--rail1 1.50 --rail2 0.00`; js_record + reach_dwell_monitor (domain 0). Kamera + localizer di domain 0 (harness HW baru);
   ulang kriteria langkah 1 di domain 0 dengan stack hidup. TIDAK ada run_g54 / job berat bersamaan (beban CPU kamera, G51 B5).
3. voice_web_ui (https, Chrome mic) + target_cli `voice_topic:=/voice/transcript status_timeout:=900`.
4. demo_runner `--plan-only` (`--tau-max 14`): ucapkan/ketik perintah → `g` → harus `success arm_1 (arm_1:PLANNED)`; catat
   penyaring (torsi RNEA+offset, S18, env, interarm). /joint_states sebelum = sesudah.
5. Restart demo_runner TANPA `--plan-only` (TANYA operator, satu jalan per persetujuan): perintah → `g` → MOVED → dwell → return_rest.
   Rekam video + waktu perintah→selesai + torsi terukur vs prediksi + posisi TCP vs centroid (monitor). 1–3 jalan; `failed
   TORQUE-UNSAFE` = `g` lagi, bukan ubah penyaring. `g` yang diketik SELAMA misi dibuang target_cli (ketik lagi sesudah status).
6. Akhir: lengan REST, g1 tetap PARKIR (tak pernah diperintah); K0; matikan bersih (runbook §7, node 0 di domain 0 dan 79); arsip
   /tmp (hilang saat reboot) ke results/p1_demo_hw/; laporan + gambar arsitektur rantai demo.

ATURAN KERAS: SETIAP gerak nyata ditanya dulu (satu langkah per persetujuan, termasuk pemulihan K0 dan pulang). Fault / LED merah /
REJECTED / NOT armed (sesudah ARMED) / torsi > 14 / benturan / kabel / gerak tak diminta → BERHENTI, amankan (lengan REST, DRY dulu),
tulis, tanya. MARGIN = TOLAK; penyaring TIDAK dilonggarkan. Botol hanya di meja depan arm_1 (rak/dinding = ditolak, benar).
Akhir sesi: laporan + rekomendasi model/effort sesi berikut; berkas keadaan "Langkah berikut" satu kalimat.
