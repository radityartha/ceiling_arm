P1 — G57 HW PARALEL (hari HW 2026-10-12). Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus, effort HIGH.
Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari docs/p1_campaign_state.md ("Langkah berikut").

BACA DULU (penuh): CLAUDE.md; MEMORY.md; docs/p1_campaign_state.md; docs/p1_hw_day_2026-10-12.md (URUTAN + perintah persis hari
ini); docs/p1_hw_runbook.md §0–2, §7 (bring-up, K0, matikan); docs/p1_hw_runbook_parallel.md (runner paralel, risiko P1–P7
DITERIMA operator 2026-10-11); docs/p1_g55_par_sched.md §B (jadwal terpilih + prediksi sim); docs/p1_g56_hw_day.md §A0, §B
(analisis cancel bridge, injeksi stop berbasis gerak, celah FAULT_RE).

KEADAAN: semua offline SELESAI (commit ed46f70). Runner paralel docs/results/p1_g54/run_g54.py (traverse eksklusif, reservasi
lintas-gantry, probe --reserve-dir → FJT langsung, AUTO-STOP kedua gantry) lulus mock; BELUM PERNAH jalan di HW. Jadwal terpilih
P1″ = docs/results/p1_g55/g55_selected.json varian R90sel (|rot| ≤ 30°, R90sel = R35sel → pembanding bermakna = jadwal LAMA vs
TERPILIH seed sama). Amplop HW (results/p1_g48/rot_envelope.json): g1 −125…+90, g2 −35…+95 — semua jadwal hari ini di dalamnya.
Sel terakhir: g1 PARK 1.50, g2 0.00, lengan REST, stack MATI. Mock MATI. Operator di lokasi.

TUGAS — dokumen fase docs/p1_g57_hw.md: §A + dugaan D490+ DIKUNCI sebelum gerak HW pertama (salinan results/p1_g57/g57_A_locked.md
+ sha256), §B ditulis, commit per langkah besar, perbarui berkas keadaan sesudah tiap langkah berarti. Arsip HW di results/p1_g57/hw/
(bukan p1_g56/hw; sesuaikan --archive dan --out di perintah rencana). Urutan = docs/p1_hw_day_2026-10-12.md:
- §0 TANPA gerak: remount_check → bring-up nyata (SIG_DFL + setsid, PID launch asli, SigIgn tanpa 0x2, 7/7 controller, bridge
  table1+table2 ARMED, /table_hw_commands diam) → monitor + perekam → K0 (k0_check.py --rail1 1.50 --rail2 0.00; TOLAK → pulihkan,
  DRY dulu, gerak sesudah izin) → plan-only NYATA k=3 (s7/s10/s0/s11 R90sel + s7 R180 dari g43_candidates.json + s10 G42 dari
  g42f_candidates.json) → run_g54 --dry tiap run (K0 LULUS, traverse di luar: [], rc 0).
- §1 Uji stop-semua HW: s7 R90sel --inject-stop-on-motion 2. Lulus = rc 3, inject tercatat, lengan g2 berhenti sebelum REST dan
  diam (perekam), post_stop_motion ≈ 0, tidak ada alat diluncurkan sesudah stop. GAGAL → BERHENTI, jangan run paralel lain, tanya.
- §2 Pasangan sama-sesi run_g54 paralel, selang-seling: s7 lama (R180) → s7 R90sel → s10 R90sel → s10 lama (G42) → s0 R90sel →
  (opsional) s11 R90sel. Sesudah tiap run: g47_home.sh <S> <PLAN> <V> --dry → izin → gerak → K0.
- Analisis: makespan HW vs sim P1″ (g54_sim) dan vs mock G54/G55; Δ lama−baru per seed; dekomposisi (tugas / traverse / tunggu
  gerbang / tumpang-tindih dua gantry); RESERVE-BLOCKED; torsi puncak. Pakai results/p1_g54/g54_report.py sebagai pola.
- §4 Matikan bersih (runbook §7), arsip /tmp/p1hw_* ke results/p1_g57/hw/, crash dump sesi dihapus.

ATURAN KERAS: SETIAP gerak nyata ditanya dulu (AskUserQuestion, satu langkah per persetujuan, termasuk pemulihan K0 dan pulang).
Fault / LED merah / REJECTED / NOT armed / torsi > 14 / benturan / kabel / AUTO-STOP tak terduga → BERHENTI, amankan (lengan REST,
DRY dulu), tulis, tanya. JANGAN uji stop saat TRAVERSE (bridge menyelesaikan kaki go_to_absolute yang sudah terkirim, G56 A0).
JANGAN pkill -f (ps|awk lalu kill PID). Jangan move_dual_table saat ARMED. MARGIN = TOLAK. Hanya run_g54 untuk run paralel
(FAULT_RE run_g32 tidak menangkap error ros2_control_node generik). Tidak ada job offline berat selama stack HW hidup.
Akhir sesi: laporan + rekomendasi model/effort sesi berikut; berkas keadaan "Langkah berikut" satu kalimat.
