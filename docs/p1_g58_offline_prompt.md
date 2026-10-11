P1 — G58–G60 OFFLINE (persiapan semua HW berikut). Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus, effort HIGH.
Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari docs/p1_campaign_state.md ("Langkah berikut"). NOL gerak HW —
stack HW TIDAK dinyalakan di sesi ini. Mock = ROS_DOMAIN_ID 77 saja.

BACA DULU (penuh): CLAUDE.md; MEMORY.md; docs/p1_campaign_state.md; docs/p1_campaign_prompt.md bagian "CARA KERJA" (aturan
fase: §A dikunci + sha256 sebelum mengukur, dugaan bernomor, §B menang dan DITULIS, commit per fase, maks 3 putaran per gerbang);
docs/p1_g54_parallel.md; docs/p1_g55_par_sched.md; docs/p1_g56_hw_day.md; docs/p1_hw_day_2026-10-12.md; docs/p1_g57_hw_prompt.md;
docs/p1_demo_off.md (§A–§D).

KEADAAN (commit ed46f70): runner paralel results/p1_g54/run_g54.py lulus mock; jadwal terpilih P1″ results/p1_g55/g55_selected.json
varian R90sel (DRY-CLI + smoke paralel mock 7/7: s0 s3 s7 s8 s10 s11 s18; s12 = jadwal G52 R90, hanya DRY-CLI). Di semua seed
|rot| terbaik ≤ 30° → R90sel = R35sel — TETAPI solver tidak memodelkan traverse eksklusif (hanya g54_sim). HW G57 (prompt
docs/p1_g57_hw_prompt.md) BELUM jalan; operator: offline dulu, lalu HW.

TUGAS (urut; tiap fase = dokumen docs/p1_g5N_*.md, dugaan D500+; fase selesai → langsung fase berikut):

G58 — siap HW blok seed tambahan (lama vs baru, paralel, sama-sesi; untuk tabel paper n > 3):
- Smoke paralel MOCK `run_g54` (results/p1_g54/g54_smoke.sh, pulang retrace + K0) untuk jadwal LAMA s3 s8 s18 (results/p1_g43/
  g43_candidates.json R180) dan s12 (results/p1_g52/g52_candidates.json R90); bila jadwal lama belum DRY-CLI 3/3 dengan tol 0.05°,
  DRY-CLI k=3 dulu (pola g55_mock.sh). Catat mock vs g54_sim.
- Tabel pasangan siap-HW (lama vs baru) + prediksi sim + perkiraan waktu sesi; ulangan s7/s10 untuk variansi.

G59 — DEMO voice → YOLOE → approach (P1 demo saja: tanpa grasp, tanpa angka akurasi):
- Jalankan ulang DEMO-OFF di mock dengan kode SEKARANG (probe sudah berubah: --reserve-dir/--out-json/_execute — jalur bawaan harus
  tetap sama). Perhatikan: mobileclip_blt.ts TERPOTONG → perception diluncurkan dari ros2_ws/; rgbd_perception.launch.py tidak
  menyalakan object_localizer (remap seg/, harness/loc_up.sh). Perbarui prompt DEMO-HW (p1_demo_off.md §D) sesuai keadaan sekarang,
  termasuk langkah cek kamera TANPA gerak (lampu lab menyala, YOLOE melihat objek) yang bisa disisipkan di hari HW.

G60 — penjadwal sadar traverse eksklusif (riset offline):
- Model: selama traverse gantry g, gantry lain tidak memulai/menjalankan event (= aturan run_g54 + prioritas traverse menunggu);
  biaya P1″ (G53). Pilih pendekatan paling sederhana yang benar (mis. solver eksak/heuristik atas pasangan jadwal per gantry dengan
  evaluasi g54_sim-setara, atau penalti yang terbukti batas bawah) — tulis alasannya di §A. Kontrol: pada jadwal yang sudah ada,
  makespan model baru = g54_sim (identik).
- Jadwal ulang R90 dan R35 untuk s0 s3 s7 s8 s10 s11 s12 s18 (predikat ok49 G49, MARGIN = TOLAK, cache = SALINAN). Laporkan: apakah
  ada seed di mana jadwal R90 dengan |rot| > 35 MENANG atas R35 di model eksklusif. Bila ada: (iv′)+(iv″), DRY-CLI k=3, smoke
  paralel mock → pasangan HW R90 vs R35 yang bermakna. Bila tidak ada: tulis sebagai hasil (rotasi lebar tidak membantu di model
  ini), jangan dipaksakan.

AKHIR: perbarui docs/p1_hw_day_2026-10-12.md (atau buat rencana hari HW baru) dan docs/p1_g57_hw_prompt.md supaya sesi HW
berikut memuat: G57 (uji stop-semua + pasangan s7/s10/s0) → blok seed G58 → (bila G60 menemukan) pasangan R90 vs R35 → cek kamera
+ DEMO-HW G59 — dengan perkiraan waktu per bagian dan titik henti bila waktu habis. Berkas keadaan "Langkah berikut" = satu kalimat
yang menunjuk prompt HW. Laporan akhir: ringkasan per fase, tally dugaan, risiko sisa, rekomendasi model/effort sesi HW.

ATURAN: NOL gerak HW, stack HW tidak dinyalakan. Mock: launch via wrapper SIG_DFL + setsid, cek SigIgn, monitor reach_dwell +
perekam js_record WAJIB hidup sebelum DRY-CLI/smoke (G52 percobaan 1 gagal karena lupa), REST + PARK + K0 sebelum smoke, matikan
bersih (node list --no-daemon 0, crash dump sesi dihapus). JANGAN pkill -f (ps|awk lalu kill PID). Job > 10 menit via setsid nohup
+ cache dapat-dilanjut + log line-buffered. Job offline berat (Pool 15) JANGAN bersamaan dengan plan-only/smoke mock. MARGIN = TOLAK.
Solver = salinan pola g55_make.py / g49_make.py (BUKAN g40_make.solve_exact_e). Jangan ubah run_g32 / pose_to_g / return_rest tanpa
alasan tertulis; perubahan probe harus bawaan-mati (argv lama identik).
