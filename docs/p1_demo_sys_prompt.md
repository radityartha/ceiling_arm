# Prompt pembuka sesi DEMO-SYS (demo = uji sistem P1: allocation + move planning + scheduling) — salin ke sesi baru

```text
Lanjutkan DEMO-SYS di /home/user1/Documents/ceiling_arm (branch feat/rgbd-deploy). NOL gerak lengan/rel/gripper tanpa izin.
Izin selalu lewat tombol AskUserQuestion (operator tidak suka mengetik), SATU gerak per izin.

TUJUAN (operator 2026-10-17): demo dipakai untuk MENGUJI sistem kita sebelum ambil data paper — kamera → tugas → penjadwal P1
(task allocation: lengan + stop gantry rel/rotasi; move planning: pose_to_g + reach_dwell_probe; scheduling: urutan + paralel
run_g54) → eksekusi. Sampai sekarang demo MELEWATI allocation+scheduling (demo_runner = lengan terdekat, gantry diam; TAS-RAK =
stop gantry dipilih manual oleh Claude). Itu yang harus diganti. P1 = LURING (operator 2026-10-06): snapshot persepsi → solve →
eksekusi, bukan pipeline online.

BACA PENUH dulu: CLAUDE.md, docs/HANDOFF_2026-10-13.md §2 (aturan) + §4.1 (peta P1), docs/p1_campaign_state.md (baris
DEMO-ZONE-HW + TAS-RAK terbaru), docs/p1_demo_zone_hw.md (rantai demo HW yang terbukti, B0–B8), docs/p1_tas_rak.md (0/14,
mengapa), docs/p1_g54_parallel.md + docs/p1_g57_hw.md (runner paralel HW), docs/p1_hw_runbook_parallel.md, dan dokumen solver
P1″ (g53/g55: cari dari campaign_state; results/p1_g55/g55_make.py, g55_selected.json, results/p1_g54/run_g54.py, reserve.py).
Memori: demo-zone-p3-correction, campaign-g43-g48-hw-ready, g52-g54-phase7-parallel, g57-hw-parallel-pairs, g58-g60-offline,
g40-env-aware-sched, no-shared-zone-at-rot0, ceiling-arm-fold-is-torque-limited, feedback-tau-max-14-operator, ros2-launch-pid-leak.

TUGAS (urut, ✓/✗ harfiah):
0. LURING — pahami dulu, JANGAN tebak: (a) format masukan solver P1″ (seed: xyz tugas, p0, rotasi; dari mana 'drawn/nodes'
   berasal — peta kapabilitas?) dan keluaran yang dibaca run_g54; (b) apakah solver menerima titik tugas SEMBARANG (bukan node
   peta) dan bagaimana tugas tak layak ditandai (oracle⁗: IK/torsi/env/antar-lengan) — tugas tak layak harus DITOLAK SISTEM
   dengan alasan, bukan oleh Claude; (c) orientasi tugas reach-and-dwell vs tool-bawah demo (stand-off 0.25 m di atas puncak).
   Tulis temuan + rencana adapter ke operator SEBELUM kode besar.
1. LURING: adapter snapshot track (domain 0, rec 15 s, median per track) → berkas seed/tugas solver (xyz = objek + 0.25 z, tool
   bawah) + label/tid. Persepsi per zona: meja arm_1 = rgbd2 + koreksi P3 (`zone_correct_node`, teruji ≤ 3.4 cm); zona lain =
   localizer fusi tanpa koreksi (rgbd dekat ≤ ~4 cm, BELUM diuji botol) → dicatat sebagai keterbatasan, ukur bila ada tanda.
   Kotak penyaring track per zona (pola results/p1_tas_rak/rack_box_node.py) agar benda luar sel tidak jadi tugas.
2. MOCK 77: tugas sintetis (posisi objek dari operator/data lama, mis. botol M1 + tas rak (0.98, −0.83, 1.09) sebagai kontrol
   negatif + 1–2 titik zona gantry 2) → solver → jadwal → run_g54 plan-only → mock eksekusi. Harus: alokasi ≥ 2 lengan bila
   ada tugas di dua zona, gerak gantry oleh SISTEM, tas rak ditolak oleh oracle/solver dengan alasan.
3. §A DEMO-SYS (dugaan bernomor lanjut dari D666) → kunci (salinan + sha256) SEBELUM bring-up. Objek & posisi = DITENTUKAN
   OPERATOR di lokasi (tanya lewat tombol). Gerbang: snapshot satu track per objek; jadwal plan-only sah; torsi --tau-max 14;
   MARGIN = TOLAK; penyaring tidak dilonggarkan.
4. HW (izin per langkah): self-test env/interarm + remount_check; stack_up.sh (octomap_updaters:=false); K0 ukur ulang (papan
   SUDAH DILEPAS dari semua lengan → jari tidak lagi memegang papan; k0_check polos dulu, catat nilai jari); kamera+localizer
   domain 0; table_check (meja demo) bila tugas di meja arm_1; snapshot → solve → run_g54 plan-only → izin → eksekusi → REST → K0.
5. Matikan per PID (launch bisa SigIgn SIGINT; SIGTERM ke launch meninggalkan anak → anak dulu, pola perc_down.sh; JANGAN taruh
   pola regex yang juga muncul literal di baris perintah — DEMO-ZONE-HW B6 shell sendiri ter-TERM). Papan skor §B, state, memori,
   rekomendasi model/effort.

ATURAN: penyaring tidak dilonggarkan; fail loud; setsid nohup; jangan sentuh /home/kubota (disk / 99 %, ~16 GB bebas); frame
berisi orang tidak di-commit; commit per bagian hanya berkas sendiri (git add <path>). Bahasa Indonesia, singkat (caveman).
Keadaan sel (2026-10-17 11:50): papan DILEPAS dari semua lengan (operator), semua REST, g1 1.40, g2 0, stack + kamera + mock MATI,
botol putih di M1 meja demo, tas kertas di atas lemari sisi gantry 2.
Alat siap: demo_runner --tracks-topic/--arms (BASE arm_1..arm_4), harness p1_demo_zone_hw/ (perc_up/down, cli, rec, view_up
RViz+rqt), p1_tas_rak/ (rack_box_node, harness fusi), js_window.py (torsi/gerak per jendela).
```

Model: Opus, effort **high** (integrasi solver ↔ persepsi bisa salah diam-diam; gerak gantry + 4 lengan; torsi j2 di batas).
