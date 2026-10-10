KAMPANYE G43+ — dari G42 sampai SIAP HW, SATU prompt (tempel ulang prompt yang SAMA bila sesi putus).
Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus, effort HIGH.

CARA KERJA (berlaku seluruh kampanye)
- Berkas keadaan docs/p1_campaign_state.md (buat bila belum ada): fase aktif, gerbang LULUS/GAGAL + bukti (log, commit),
  keputusan operator, ROT_NEED, risiko terbuka, "langkah berikut" (satu kalimat yang bisa langsung dikerjakan).
  Perbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Awal tiap sesi: baca CLAUDE.md + MEMORY.md + berkas
  keadaan + dokumen fase aktif, lanjut dari "langkah berikut". Fase LULUS tidak diulang.
- Tiap fase = satu dokumen docs/p1_g4N_*.md (N naik dari 43): §A dikunci (salinan results/ + sha256) sebelum mengukur,
  dugaan bernomor (D350+), §B menang atas §A dan DITULIS, papan skor, commit per fase. JANGAN menulis prompt sesi baru:
  fase selesai -> langsung fase berikut.
- Gerbang gagal -> diagnosa dengan data, perbaiki, ulang. Maks 3 putaran per gerbang; sesudahnya BERHENTI dan tanya
  operator (AskUserQuestion: opsi + rekomendasi).
- BERHENTI/TANYA hanya untuk: (a) SETIAP gerak HW nyata, (b) cek fisik/kamera, (c) keputusan di luar repo (rentang mekanis
  rotasi, kabel, stopper), (d) 3 putaran gagal. Selain itu jalan terus.
- ATURAN: MARGIN = TOLAK; nol gerak HW sebelum Fase 6; job > 10 menit via setsid nohup + cache dapat-dilanjut + log
  line-buffered; JANGAN pkill -f (ps|awk lalu kill PID); mock = ROS_DOMAIN_ID 77, launch via wrapper SIG_DFL, cek SigIgn,
  matikan bersih (node list --no-daemon 0, crash dump sesi dihapus); job offline berat (Pool 15) JANGAN bersamaan dengan
  plan-only mock (CPU merusak verdict OMPL) — kalau terpaksa, nice 19 + Pool 8. Solver baru = salinan
  docs/results/p1_g42/g42_make.py (solve_exact_f + FIX_CHOICES + (s42) + (c41b) margin kotak 10 mm), BUKAN
  g40_make.solve_exact_e (pemangkasan salah).

FAKTA AWAL (G42, commit 77e5114)
- G42f (R35 = ±35°) lolos (i)–(iii) 6/20: seed 0 3 7 8 10 18; DRY-CLI mock 6/6 seed 3/3 (108/108 PLANNED, GOAL-MISMATCH 6
  retry ok). 14 seed terhalang torsi joint_2; tuple torsi-ok hanya ada di |rot| 25–35° dan di sana lengan REST masuk pita
  50 mm peta/rak.
- probe --goal-joints belum pernah --move (bahkan mock). Penyaring Fix-Start-State belum ada (G38 s1 t2 PLANNED dengan start
  tabrak). HW rotasi terverifikasi hanya ±10° (G30); origin g2 ≈ −35 mm (G36b) belum dicek kamera; g1 PARKIR 1.50.
- URDF rotasi ±180°; tabel pose G40 sudah 33 × 72 rotasi (5°).
- KEPUTUSAN OPERATOR 2026-10-10: rotasi TIDAK dibatasi ±10°; MAKSIMALKAN pemakaian rotasi.

FASE 1 — Penjadwal rotasi maksimum (OFFLINE)
- RSET R180 (72 rotasi) di samping R35; tuple/tepi baru lazy (cache disalin, bukan ditimpa). Jadwal ulang 20 seed G38
  dengan R180, pembanding R35 (G42f): seed lolos, makespan, rotasi maks dan Δrot maks per traverse.
- Periksa model waktu rotasi untuk Δrot besar (rot_cmd, G30 T ≈ T_cmd) dan bahwa sweep_rot (iv′), env traverse (b),
  (iv″) serta RotCrossChecker valid di rotasi > 35° (kontrol positif: pose yang diketahui tabrak harus ditolak).
- Diagnosa ulang seed yang masih gagal (pola g42 diag + laporan) — apa yang berubah di |rot| > 35°.
- Hasil: ROT_NEED = |rot| maks dan Δrot maks yang dipakai jadwal terpilih; daftar seed kandidat HW.
- Gerbang 1: seed lolos R180 ≥ 6 dan keenam seed G42f tetap lolos dengan makespan ≤ R35; semua predikat sama ketatnya.

FASE 2 — Penyaring Fix-Start-State (G37 A)
- Ukur dulu distribusi |dq| waypoint 0→1 dan antar-waypoint dari arsip plans jsonl G31–G42 (+ Fase 1). Verdict baru
  START-JUMP (bisa diulang) di _plan_and_screen, batas dari data.
- Gerbang 2: kontrol positif G36 B1 (jari +0.05 rad) dan G38 s1 t2 DITOLAK; 100 % rencana yang pernah dieksekusi di HW
  (G24b–G36b) tetap diterima (bila tidak: batas salah, tulis, ulang).

FASE 3 — K0 terukur (G37 C)
- Skrip: gabungan /joint_states ≥ 0.5 s → maks |q − REST| per lengan, rel + rotasi tiap gantry, jari; TOLAK bila tidak
  cocok p0/PARK. Dipasang sebagai prasyarat run_g32 (ganti cek A3 awal). Uji di mock + fake_js (domain lain): cocok →
  lulus; lengan 1° dari REST / rel 3 mm / rotasi 1° → tolak.
- Gerbang 3: semua kasus uji benar.

FASE 4 — Rehearsal mock penuh
- task_cli mencatat verdict TIAP percobaan probe (celah G42 B1). Plan-only + DRY-CLI k = 3 `--final-retract` untuk semua
  seed kandidat Fase 1 (dengan START-JUMP aktif).
- Smoke mock `run_g32 --final-retract --move` (fake HW, BUKAN data) pada ≥ 2 seed, termasuk yang rotasinya terbesar; mock
  dibawa ke REST + p0 dulu (return_rest / pose_to_g), K0 Fase 3 sebagai prasyarat; catat verdict penilai per tugas,
  GOAL-MISMATCH, torsi prediksi.
- Bila GOAL-MISMATCH > 5 % percobaan: uji A/B toleransi tujuan sendi tugas lebih ketat (mis. 0.05°) di mock; pakai hanya
  bila plan-only tidak memburuk.
- Gerbang 4: DRY-CLI 3/3 semua seed kandidat; smoke: semua tugas sukses penilai, event = DRY-CLI, tidak ada penolakan
  penyaring saat eksekusi.

FASE 5 — Paket SIAP HW (OFFLINE)
- docs/p1_hw_runbook.md: bring-up (my_workcell.launch.py use_fake_hardware:=false enable_gantry_bridge:=true, remount_check,
  subnet 192.168.2.x, PID launch asli + SigIgn, 7/7 controller, bridge ARMED), K0, monitor + perekam, perintah run_g32 persis.
- Periksa batas rotasi di jalur HW: bridge gantry (yaml/max, skala 100 pulse/°), joint_limits.yaml, dual_table_controller,
  kecepatan rotasi 10 °/s — semua mengizinkan ROT_NEED? Bila tidak: perubahan yang diperlukan ditulis (BELUM dipasang ke HW
  tanpa operator).
- Prosedur cek origin g2 dengan kamera (apa yang diukur, ambang ±2 mm, tindakan bila gagal — ppreset HANYA dengan operator).
- Prosedur sweep rotasi HW bertahap ±10 → ±20 → ±35 → … → ROT_NEED: satu gantry, lengan REST, gantry lain parkir; sesudah
  tiap langkah cek galat rotasi ≤ 0.3°, torsi, waktu ≈ T_cmd, dan operator cek kabel/benturan; kriteria stop.
- Blok seed HW pertama: R_max vs R0 di sesi yang sama (pembanding sama-sesi, G32 B7 (2)).
- Gerbang 5 = SIAP HW: Fase 1–4 LULUS, runbook ditulis, ROT_NEED didukung konfigurasi, risiko terbuka nol atau diterima
  operator (tanya).

FASE 6 — HW (hanya bila operator menjawab "lanjut"; SETIAP gerak ditanya dulu)
- 6a cek origin g2 kamera → 6b sweep rotasi bertahap sampai ROT_NEED → 6c run_g32 DRY di stack nyata → 6d blok seed HW.
  Fault/torsi/benturan → berhenti, amankan (REST), tulis, tanya.

SELESAI bila Gerbang 5 LULUS (atau Fase 6 selesai bila operator memilih lanjut). Laporan akhir: ringkasan per fase, tally
dugaan, keadaan HW, risiko sisa — di berkas keadaan dan di jawaban.
