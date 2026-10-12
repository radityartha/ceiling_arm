P1 — G57 HW PARALEL + blok seed G58/G60 + DEMO G59 (hari HW). Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus, effort HIGH.
Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari docs/p1_campaign_state.md ("Langkah berikut").

BACA DULU (penuh): CLAUDE.md; MEMORY.md; docs/p1_campaign_state.md; docs/p1_hw_day_2026-10-12.md (URUTAN, jadwal, perintah persis,
perkiraan waktu, titik henti); docs/p1_hw_runbook.md §0–2, §7; docs/p1_hw_runbook_parallel.md (risiko P1–P7 DITERIMA 2026-10-11);
docs/p1_g56_hw_day.md §A0, §B (cancel bridge, injeksi stop berbasis gerak, FAULT_RE); docs/p1_g58_seed_block.md §B; docs/p1_g60_excl_sched.md
§B; docs/p1_g59_demo.md §B + docs/p1_demo_off.md §D.

KEADAAN (sesudah G58–G60, 2026-10-12): runner paralel docs/results/p1_g54/run_g54.py lulus mock (G54–G60: 25+ smoke 6/6), BELUM
PERNAH di HW. Pasangan lama vs baru per seed (tabel di rencana HW): s7 s10 s0 s8 (baru = p1_g55/g55_selected.json R90sel) dan s3 s18
s12 (baru = p1_g60/g60_selected.json G60, penjadwal sadar traverse eksklusif). Mock lama − baru: s7 −107, s10 −74, s8 −172, s3 −141, s18
−174, s12 −43 s (s3/s18/s12 lintas sesi mock). R90 vs R35 TIDAK diuji (G60: rotasi > 35° tidak menang). Semua jadwal di dalam amplop HW
(results/p1_g48/rot_envelope.json: g1 −125…+90, g2 −35…+95). Sel terakhir: g1 PARK 1.50, g2 0.00, lengan REST, stack MATI.

TUGAS — dokumen fase docs/p1_g57_hw.md: §A + dugaan D530+ DIKUNCI sebelum gerak HW pertama (salinan results/p1_g57/g57_A_locked.md +
sha256), §B ditulis, commit per bagian, berkas keadaan diperbarui sesudah tiap langkah berarti. Arsip HW di results/p1_g57/hw/.
Urutan + waktu = docs/p1_hw_day_2026-10-12.md:
- §0 TANPA gerak (30 mnt): remount_check → (opsional, lampu nyala: cek kamera tanpa gerak §6.1 SEBELUM bring-up) → bring-up nyata
  (SIG_DFL + setsid, PID launch asli, SigIgn tanpa 0x2, 7/7, table1+table2 ARMED, /table_hw_commands diam) → monitor + perekam → K0.
- §1–§2 (35 mnt): plan-only nyata k=3 + DRY s7 lama/baru → uji stop-semua HW (s7 R90sel --inject-stop-on-motion 2). GAGAL → henti
  total, tanya.
- §3 G57 (1 jam 45 mnt): s7 lama → s7 baru → s10 baru → s10 lama → s0 baru. HENTI A bila waktu habis (minimum paper = s7 + s10).
- §4 G58/G60 (55 mnt per pasangan): s8, s18, s3, s12 (plan-only per pasangan dulu). HENTI B sesudah pasangan mana pun.
- §5 (opsional, 1 jam 20 mnt): ulangan s7/s10 untuk variansi. §6 (opsional, 1 jam): cek kamera (bila belum) + DEMO-HW prompt
  p1_demo_off.md §D (bisa sesi lain). §7 matikan bersih + arsip (/tmp hilang saat reboot).
- Analisis per pasangan: makespan HW vs sim (g54_sim) dan vs mock G58/G60; Δ lama − baru; dekomposisi (tugas / traverse / tunggu
  gerbang / tumpang-tindih dua gantry); RESERVE-BLOCKED; torsi puncak. Pola results/p1_g58/g58_report.py + p1_g54/g54_report.py.

ATURAN KERAS: SETIAP gerak nyata ditanya dulu (AskUserQuestion, satu langkah per persetujuan, termasuk pemulihan K0 dan pulang).
Fault / LED merah / REJECTED / NOT armed / torsi > 14 / benturan / kabel / AUTO-STOP tak terduga → BERHENTI, amankan (lengan REST,
DRY dulu), tulis, tanya. JANGAN uji stop saat TRAVERSE. JANGAN pkill -f (ps|awk lalu kill PID). Jangan move_dual_table saat ARMED.
MARGIN = TOLAK. Hanya run_g54 untuk run paralel (FAULT_RE run_g32 tidak menangkap error ros2_control_node generik). Tidak ada job
offline berat / kamera depth_cloud selama runner HW jalan. Akhir sesi: laporan + rekomendasi model/effort sesi berikut; berkas keadaan
"Langkah berikut" satu kalimat.
