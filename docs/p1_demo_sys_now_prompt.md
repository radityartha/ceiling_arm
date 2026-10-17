# Prompt pembuka sesi DEMO-SYS SEKARANG (2026-10-17 sore) — salin ke sesi baru

```text
Lanjutkan DEMO-SYS di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). Operator mau demo HARI INI.
NOL gerak lengan/rel/gripper tanpa izin. Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin.

TUJUAN: demo = UJI SISTEM P1 sebelum data paper: kamera → snapshot track → tugas → solver P1″ (allocation: lengan + stop gantry
rel/rotasi; scheduling: urutan + paralel) → run_g54 (pose_to_g + reach_dwell_probe, TUJUAN SENDI) → eksekusi → REST. Claude TIDAK
memilih lengan/stop gantry dengan tangan; target tak layak harus DITOLAK SISTEM dengan alasan (memori demo-sys-uses-p1-scheduler).

FAKTA BARU HARI INI (G64 §0.3–0.8, docs/p1_g64_joint_plan.md — jangan diulang):
- Rumus torsi BENAR: penyaring RNEA rencana + 7.7 ≤ 14 (513 segmen HW; model lain tidak lebih aman). Batas tetap: --tau-max 14,
  AUTO-STOP > 14 terukur. effort Kortex = torsi dari ARUS motor (tanda −RNEA).
- Selisih mock/HW selama ini = TUJUAN POSE (MoveIt pilih cabang IK sendiri; TAS-RAK RNEA 7.5–9.0 vs cabang penjadwal 4.93;
  DEMO-ZONE-HW plan-only 61 % vs eksekusi 97 %). Dengan TUJUAN SENDI penjadwal (run_g54 → run_g32.task_cmd → --goal-joints):
  penjadwal = probe = HW 87/87 (G57). → GERAK HANYA lewat solver → run_g54. demo_runner (tujuan pose) JANGAN dipakai untuk gerak.
- TAS = KONTROL POSITIF (operator 2026-10-17 sore): tas kertas di atas lemari sisi gantry 2 (~(0.98, −0.83, 1.09), p_cmd z 1.34)
  HARUS dicapai OLEH SISTEM (solver P1″ → run_g54); berhasil = sistem bekerja. Bukti luring (g64_diag.jsonl, TASRAK_z134):
  emulasi predikat penjadwal sekarang menemukan 1 tuple layak — arm_3 @ g2 (0.50, −20°), j2 statis 5.625 vs batas 5.64 (sangat
  tipis; grid manual TAS-RAK tidak pernah mencobanya); 26 tuple statis lain gagal karena REST di stop / garis pulang lurus (kelas
  "perencanaan terpisah", obatnya G64 lipat T1 yang BELUM diimplementasikan). Bila sistem menolak tas di mock/plan-only: TULIS
  alasan persis (fail loud) → itu temuan G64, BUKAN dicari jalan lain di luar sistem (tanpa pilih stop/lengan manual).
- KONTROL NEGATIF (fisika): titik di atas tas z 1.40 → 0 IK di seluruh grid (g64_diag TASRAK_z140) → harus DITOLAK sistem dengan
  alasan. Cukup di mock (tugas sintetis), tidak perlu benda nyata.

BACA PENUH dulu: CLAUDE.md, docs/p1_demo_sys_prompt.md (tugas 0–5 lengkap + catatan 10-17; teks lamanya menyebut tas = kontrol
negatif — SUDAH DIKOREKSI operator, prompt INI yang berlaku), docs/HANDOFF_2026-10-13.md §2,
docs/p1_campaign_state.md (baris DEMO-ZONE-HW, TAS-RAK, G64 terbaru), docs/p1_demo_zone_hw.md (rantai persepsi HW yang terbukti),
docs/p1_g54_parallel.md + docs/p1_g57_hw.md + docs/p1_hw_runbook_parallel.md (runner HW), solver P1″ (results/p1_g55/g55_make.py,
g55_selected.json; results/p1_g54/run_g54.py, reserve.py), docs/p1_g64_joint_plan.md §0.7–0.8. Memori: demo-sys-uses-p1-scheduler,
g64-torque-model-verified, demo-zone-p3-correction, g57-hw-parallel-pairs, g52-g54-phase7-parallel, feedback-tau-max-14-operator,
ros2-launch-pid-leak, workcell-known-issues.

URUTAN (✓/✗ harfiah; ringkas dari p1_demo_sys_prompt.md, karena demo hari ini → utamakan jalur terpendek yang SAH):
0. LURING (≤ 1 jam): format masukan/keluaran solver P1″ ↔ run_g54; apakah solver menerima titik SEMBARANG (bukan node peta) dan
   bagaimana tugas tak layak ditandai; orientasi tool-bawah + stand-off 0.25 m. Tulis temuan + rencana adapter ke operator (tombol)
   SEBELUM kode besar. Bila solver hanya menerima node peta: usulkan cara jujur (mis. node terdekat ≤ toleransi, dicatat) — operator memilih.
1. LURING: adapter snapshot track (median per track, 15 s) → tugas solver (xyz objek + 0.25 z). Zona meja arm_1 = rgbd2 + koreksi P3
   (zone_correct_node, ≤ 3.4 cm); zona lain = keterbatasan dicatat. Kotak penyaring track (pola p1_tas_rak/rack_box_node.py).
2. MOCK 77 (wajib sebelum HW): tugas sintetis (botol M1 + TAS z 1.34 kontrol POSITIF + titik z 1.40 kontrol negatif) → solver →
   jadwal → run_g54 plan-only → mock eksekusi. Harus: tas DIJADWALKAN + PLANNED oleh sistem (gerak gantry 2 oleh SISTEM), titik z 1.40
   DITOLAK oleh solver/oracle dengan alasan, ≥ 2 lengan bila tugas di dua zona. Tas ditolak = catat alasan per penyaring, lapor
   operator (tombol), jangan longgarkan.
3. §A DEMO-SYS (dugaan lanjut dari D667 — cek nomor terakhir di docs/*.md) → kunci (salinan + sha256) SEBELUM bring-up. Objek & posisi
   DITENTUKAN OPERATOR di lokasi (tombol).
4. HW (izin per langkah): self-test env/interarm + remount_check; stack_up.sh (octomap_updaters:=false); K0 ukur ulang (papan sudah
   DILEPAS dari semua lengan → k0_check polos, catat jari); kamera + localizer domain 0; table_check bila tugas di meja arm_1; snapshot
   → solve → run_g54 plan-only → izin → eksekusi (--tau-max 14) → REST → K0.
5. Matikan per PID (launch bisa SigIgn SIGINT; anak dulu, pola perc_down.sh; JANGAN pola regex yang muncul literal di baris perintah
   sendiri). Papan skor §B, state, memori, rekomendasi model/effort.

ATURAN: penyaring tidak dilonggarkan (MARGIN = TOLAK); fail loud; setsid nohup; jangan pkill -f; jangan sentuh /home/kubota (disk /
~13 GB bebas); frame berisi orang tidak di-commit; commit per bagian hanya berkas sendiri (git add <path>). Bahasa Indonesia, singkat
(caveman). Bila waktu habis sebelum HW: berhenti di gerbang terakhir yang lulus, tulis, jangan potong jalan.
Keadaan sel (2026-10-17 ~13:30, tidak disentuh sejak 11:50): semua lengan REST, papan DILEPAS, g1 1.40, g2 0, stack + kamera + mock
MATI, botol putih di M1 meja demo, tas kertas di atas lemari sisi gantry 2.
```

Model: Opus, effort **high** (integrasi solver ↔ persepsi bisa salah diam-diam; gerak gantry + 4 lengan; j2 dekat batas).

> **2026-10-17 ~13:50 — sesi ini dijalankan (luring saja): TAS DITOLAK SISTEM → DEMO DITUNDA, G64 dulu (operator, tombol).**
> Hasil + adapter: `docs/p1_demo_sys.md`. Klaim di atas yang terbantah: "1 tuple layak arm_3 (0.50, −20°)" = artefak BOX_MIN 0
> di g64_diag (sistem 10 mm → garis-kotak); "z 1.40 = 0 IK" tidak berlaku di grid solver (214/532 tuple ber-IK) → kontrol negatif
> diganti (1.0, 0, 0.60). Prompt ini dipakai ulang SESUDAH G64 (tasks_synth.json / tasks_neg.json, dir run baru).
