# Keadaan kampanye P1 G43+ (docs/p1_campaign_prompt.md)

> Diperbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Fase LULUS tidak diulang.

## Fase aktif

**G61 OFFLINE SELESAI** (2026-10-14, `docs/p1_g61_offline.md` B0–B6, skor **8/13**): kode run_g54 tetap 831db96. A5.1 6/6 ×2 argv identik; injeksi rot −0.48 tanpa koreksi 4/6, dengan koreksi 6/6; peringatan perekam OK. A5.4 kill move_group mock → AUTO-STOP **0.027 s**, 0 CMD sesudah stop, rc 3; A5.3 DRY ditolak sebelum t0, rc 3. C3 85/87 j2 ≤ prediksi probe; C4 s0 = penyaring reservasi. Gerbang C1–C4 LULUS. Pemulihan mock REST + PARK, K0 LULUS, seluruh PID milik G61 keluar, domain 77 kosong (shutdown launch memakai eskalasi otomatis SIGTERM/SIGKILL untuk move_group/RViz; B6). Runbook + prompt G62 final. NOL gerak HW; CAM-6DOF tidak disentuh.

**CAM-6DOF LURING SELESAI, HW BELUM** (2026-10-12 21:00–22:10, `docs/p1_cam6dof.md` §A0, commit dcde98d): operator pulang sebelum gerbang
("tes online lanjut besok"; "siapkan yang offline dulu"). Alat fit 6-DOF/M7 per kamera + self-test PASS (2 cacat diperbaiki: tanda geser ICP;
lengan REST vertikal → lengan hanya mengikat xy, z dari botol 3 tinggi), TF round-trip 0.000 mm, `loc_cam.sh`, tangkap/proyeksi/cfg tools,
plan-check MOCK 10 kandidat lantai arm_3 (9 lolos, (0.40, −0.50) j2 99 % dibuang). K4 luring: lantai di bawah arm_3 terlihat rgbd 2.6–3.1 m.
§A = DRAF (kunci besok sesudah angka balok/diameter botol + K4 hidup + plan-check HW). Sel tak disentuh (akhir RGBD2-ZONE), 0 node 0/77/78/79.

**RGBD2-ZONE SELESAI** (2026-10-12 malam, `docs/p1_rgbd2_zone.md`, tally 2/5): 7 gerak penunjuk arm_1 (6 tanda unting di meja, C2/C4 di luar meja → N1/N2), botol di 6 tanda: incumbent 0.14–0.23 m; fit planar θ **13.1°** → R1 GAGAL, tidak dipakai. Info: dengan fit, uji-tahan hidup C6 1.9 / C3 0.9 cm tetapi lengan dekat rusak > 25 cm (zona-saja). Sel home, K0 LULUS, stack MATI.

**ARM1-ZONE SELESAI** (2026-10-12 19:15–19:45, `docs/p1_arm1_zone.md`, tally 5/8): bug titik ORIGIN diperbaiki + `arm_fk_check --arm-filter`; penunjuk arm_1 di (1.75, 0.55, 1.22) TCP 1.5 mm; botol di tanda ter-track **0.151 m** sepanjang sinar rgbd2 → Z4 GAGAL, BERHENTI (tanpa demo); pita basis arm_1–arm_2 0.80 ± 0.005 = URDF → galat = **kamera rgbd2 jarak jauh**. 2 gerak nyata, torsi ≤ 8.91. Sel home, K0 LULUS, stack MATI.

**CALIB-CHARUCO SELESAI** (2026-10-12 18:30–19:00, `docs/p1_calib_charuco.md`): penunjuk arm_2 + ChArUco, 7 gerak nyata OK; kandidat GAGAL C2/C4 → incumbent tetap; rgbd2 arm_1 +0.15 m pada kedua ekstrinsik (dugaan: bukan kamera); bug konvensi titik ORIGIN 3.5 cm. Sel home, K0 LULUS, stack MATI.

**DEMO-HW SELESAI tanpa gerak** (2026-10-12 17:45–18:23, `docs/p1_demo_hw.md`, 2/5 diuji): H2 lokalisasi GAGAL (rgbd2 +0.15 m di zona arm_1 — hanya rgbd2 melihat zona itu, dari ~3 m); rantai teks → YOLOE nyata → localizer → CLI → runner → probe terbukti plan-only di domain 0 (`success arm_1 (arm_1:PLANNED)`, j2 99.6 %). Sel home, K0 LULUS, stack MATI.

**G57 HW SELESAI** (2026-10-12 11:13–17:30, `docs/p1_g57_hw.md`, tally 5/9): 6/6 pasangan lama vs baru di HW, sel di home, stack MATI. Sebelumnya: **G58–G60 OFFLINE SELESAI** (2026-10-12 10:30, prompt `docs/p1_g58_offline_prompt.md`). Berikut = sesi HW dengan
`docs/p1_g57_hw_prompt.md` + rencana `docs/p1_hw_day_2026-10-12.md` (diperbarui). Mock 77 MATI bersih.
Sebelumnya: KAMPANYE G43–G54 SELESAI — Gerbang 7 LULUS 2026-10-11.

## Gerbang

| Fase | Gerbang | Status | Bukti |
|---|---|---|---|
| 1 | seed lolos R180 ≥ 6, keenam seed G42f tetap lolos, makespan ≤ R35, predikat sama ketat | **LULUS** | `docs/p1_g43_rot180.md` B3; percepatan G46 `docs/p1_g46_sched_speed.md` |
| 2 | START-JUMP: kontrol positif ditolak, 100 % rencana HW tetap diterima | **LULUS** (putaran 2) | `docs/p1_g44_start_jump.md` B5, commit 6ab5370 |
| 3 | K0 terukur: semua kasus uji benar | **LULUS** | `docs/p1_g45_k0.md` B3, commit 4ad8394 |
| 4 | rehearsal mock: DRY-CLI 3/3 semua kandidat + smoke `--move` | **LULUS** | `docs/p1_g47_rehearsal.md` B6, commit bfc752e |
| 5 | SIAP HW | **LULUS** (operator 2026-10-10 20:12 menerima R1–R9) | `docs/p1_g48_hw_ready.md`, `docs/p1_hw_runbook.md`, commit d81c929 |
| 6 | HW (hanya bila operator "lanjut") | 6a–6c **LULUS**, 6d sebagian (5 run 6/6) | `docs/p1_g51_hw.md` B1–B15 |
| 7a | tabel 17 jadwal R90 + DRY-CLI 3/3 jadwal baru | **LULUS** (6/6) | `docs/p1_g52_r90.md` B3–B5, commit 8abd81c |
| 7b | tabel + penjelasan selisih > 15 % | **LULUS** (2/3) | `docs/p1_g53_p1p_serial.md` B1–B4, commit f69a1e9 |
| 7c | runner paralel: A2.1, DRY 5/5, smoke 5/5, kontrol negatif | **LULUS** (4/6) | `docs/p1_g54_parallel.md` B1–B5 |
| 7 | runbook HW paralel + risiko → operator | **LULUS** (operator 2026-10-11: terima P1–P6) | `docs/p1_hw_runbook_parallel.md` |
| G55 | jadwal P1″ + seleksi sim; DRY-CLI + smoke paralel | **LULUS** (5/6) | `docs/p1_g55_par_sched.md` B5 |
| G56 | persiapan hari HW: injeksi gerak, fault, sweep v3, analisis P2 | 2/3 (D482 salah rumus) | `docs/p1_g56_hw_day.md` B3 |
| G58 | blok seed lama vs baru: smoke paralel mock 11/11 + tabel siap-HW | **LULUS** (5/5) | `docs/p1_g58_seed_block.md` B1–B3 |
| G59 | DEMO-OFF ulang (kode sekarang) + prompt DEMO-HW + cek kamera | **LULUS** (4/6; 2 cacat diperbaiki: regex runner, `g` ganda) | `docs/p1_g59_demo.md` B1–B4 |
| G60 | penjadwal sadar traverse eksklusif (LB terbukti + g54_sim penuh) | **LULUS** (4/6) | `docs/p1_g60_excl_sched.md` B1–B4 |
| G57 | HW paralel run_g54: uji stop + pasangan s7 s10 s8 s18 s3 s12 (+ s0) | **SELESAI** (5/9) | `docs/p1_g57_hw.md` B2–B8 |
| G61 | runner sehat + koreksi goal; analisis torsi/waktu; A5.1–A5.4 mock | **LULUS** (8/13) | `docs/p1_g61_offline.md` B0–B6; A5.4 stop 0.027 s, 0 CMD sesudah stop, A5.3 rc 3 |
| DEMO-HW | YOLOE nyata → arm_1 di atas botol (approach-only) | **GAGAL H2** (2/5 diuji; plan-only ✓, nol gerak) | `docs/p1_demo_hw.md` B1–B6 |
| ARM1-ZONE | galat zona arm_1 lengan vs kamera; botol di tanda ≤ 5 cm → demo | **GAGAL Z4** (0.151 m; atribusi kamera; 5/8) | `docs/p1_arm1_zone.md` B1–B6 |
| RGBD2-ZONE | koreksi planar rgbd2 dari botol di 6 tanda; uji-tahan ≤ 5 cm | **GAGAL R1** (θ 13.1°; info uji-tahan 1.9/0.9 cm; 2/5) | `docs/p1_rgbd2_zone.md` B1–B7 |
| CAM-6DOF | 6-DOF per kamera (botol × 3 tinggi + lengan); uji-tahan hidup ≤ 5 cm; "hanya rgbd2 salah?" | **LURING SELESAI** (A0; HW besok) | `docs/p1_cam6dof.md` A0 |

## Keputusan operator

- 2026-10-10: rotasi TIDAK dibatasi ±10°; maksimalkan pemakaian rotasi (memori feedback-rotation-max-campaign-prompt).
- 2026-10-10 16:10 (operator): (1) batas mekanis rotasi tiap gantry **±180° bisa** (kabel/stopper/driver tidak membatasi);
  (2) operator **di lokasi besok 2026-10-11** → cek origin g2 dengan kamera (Fase 6a) besok; (3) arah sweep rotasi **aman
  semua** (+ dan −). Sweep HW tetap bertahap dan TIAP gerak tetap ditanya (aturan Fase 6).
- 2026-10-12 12:33 (operator): batas abort torsi probe 12 terlalu ketat — lewat 12 N·m bukan gagal; HW `run_g54 --tau-max 14` (batas keras tetap > 14).
- 2026-10-12 17:50 (operator, DEMO-HW): ekstrinsik kamera demo = launch + koreksi G34 (arg launch, default tidak diubah); 18:14: H2 gagal → uji rantai plan-only saja, nol gerak.
- 2026-10-12 (operator): urutan hari HW = runner HW (G57 → G58/G60) dulu, DEMO paling akhir; cek kamera TEPAT sebelum demo (bukan di §0).
- 2026-10-11 (operator, Gerbang 7): risiko runner paralel P1–P6 DITERIMA. HW paralel = sesi lain, tiap gerak ditanya, mulai uji
  stop-semua (runbook paralel §3).
- 2026-10-10 20:12 (operator, Gerbang 5): risiko R1–R9 DITERIMA; **lanjut Fase 6 besok mulai 6a** (cek origin g2 kamera),
  lalu 6b sweep (tiap kaki ditanya).

## ROT_NEED

**Sesudah G50 (17 jadwal siap-HW): g1 −125…+90°, g2 −35…+95°** (sweep v2 `results/p1_g50/g50_sweep.sh`). Lama:

**|rot| 125°, Δrot 95°** (s0 g1 (0.80, −125°)); tanpa s0 70° / 65°. Kandidat HW: s0 s3 s7 s10 s18 (R180, `results/p1_g43/g43_candidates_6.json`) + s8 jadwal G42f (`results/p1_g42/g42f_candidates.json`).

## Risiko terbuka

- HW rotasi terverifikasi hanya ±10° (G30). Origin g2 ≈ −35 mm (G36b) belum dicek kamera. g1 PARKIR 1.50.
- probe `--goal-joints` belum pernah `--move` (bahkan mock). Penyaring Fix-Start-State belum ada.

## Laporan akhir kampanye (G43–G54)

| Fase | Dokumen | Tally | Inti |
|---|---|---|---|
| 1 | G43 (+G46, G49) | — | R180: 17/20 seed lolos (i)–(iv″) (R35 6/20) |
| 2 | G44 | — | START-JUMP: kontrol positif ditolak, rencana HW diterima |
| 3 | G45 | — | K0 terukur prasyarat runner |
| 4 | G47 (+G50) | — | DRY-CLI 3/3 + smoke 17 jadwal |
| 5 | G48 | — | runbook HW serial, risiko R1–R9 diterima |
| 6 | G51 | — | HW: amplop g1 −125…+90 / g2 −35…+95; 5 run 6/6 (serial) |
| 7a | G52 | 6/6 | R90: 5/5 jadwal ulang ≤ 1.043× R180; 16/17 siap-HW (s11 butuh g2 −85) |
| 7b | G53 | 2/3 | HW serial dalam ±5 % P1′-serial; komponen saling menutup; P1″ LOO ≤ 4.84 % |
| 7c | G54 | 4/6 | runner paralel (mock): s7 R180 380 vs R35 464 s; s10 R180 453 > R35 394; stop-semua ≤ 0.5 s |

Dugaan G52–G54: 12/15 (G52 6/6, G53 2/3, G54 4/6). Gagal: D452 (c_rovh terlalu besar), D460 (move_group serial),
D463 (s0 mock/sim 1.201).

**Keadaan HW:** tidak ada gerak HW di Fase 7. Sel terakhir (G51): g1 PARK 1.50, g2 0, lengan REST, stack mati. Mock 77 mati bersih.

**Risiko sisa (diterima):** P1–P6 runner paralel; s11 R90 butuh kaki sweep g2 → −85; HW paralel belum pernah; penjadwal belum
memakai model P1″ + traverse eksklusif (s10 R180 kalah di runner paralel).

## Laporan G58–G60 (2026-10-12, offline + mock, NOL gerak HW)

| Fase | Dokumen | Tally | Inti |
|---|---|---|---|
| G58 | `p1_g58_seed_block.md` | 5/5 | 11 smoke paralel mock 6/6; lama − baru s3 −72, s8 −172, s18 −78, s7 −107, s10 −74 s; s8 lama = G42f (G43 R180 gagal (iv′)) |
| G59 | `p1_g59_demo.md` | 4/6 | DEMO-OFF ulang: 2 cacat diperbaiki (regex verdict runner; `g` ganda → gerak kedua tak diminta, kini dibuang `target_cli`); r1–r3 lulus; YOLOE luring OK dari `ros2_ws/` |
| G60 | `p1_g60_excl_sched.md` | 4/6 | LB = ΣTr + max(W) terbukti; R90 = R35 di 8/8 (rotasi > 35° terbukti tidak menang di 5/8); jadwal baru s3 −11.9, s12 −13.6, s18 −22.2 % (sim); s12 G60b terhalang reservasi → G60c nilai penuh |

Dugaan G58–G60: 13/18. Gagal: D510 (regex runner rusak oleh keluaran G47), D512 (J7 stokastik MOVED), D521 (celah ≤ 3 % hanya 5/8),
D524 (fastsim buta reservasi, s12).

**Risiko sisa:** runner paralel belum pernah di HW (P1–P7 diterima); cancel bridge saat traverse tidak berhenti seketika (G56 A0);
celah G60 s3 6.65 % / s8 5.07 % (jadwal mungkin belum optimum); LB G60 tidak memodelkan reservasi lengan → wajib g54_sim penuh;
persepsi nyata demo belum pernah (lab gelap DEMO-OFF), `mobileclip_blt.ts` di `src/reachability_gng/` masih terpotong; j3 demo P1
90–92 % rating terprediksi.

## Laporan G57 (2026-10-12, HW NYATA)

Uji stop-semua HW lulus percobaan pertama (arm_3 diam ≤ 0.1 s). 15 run paralel `run_g54`: 12 bersih 6/6, 0 RESERVE-BLOCKED,
torsi maks 13.22. Pasangan (lama − baru, s): s7 +108.9, s10 +74.9, s8 +195.5, s18 +183.1, s3 +142.5, s12 +38.5 (−11 … −42 %;
Δ_HW/Δ_mock 0.90–1.14). Perubahan: arg launch `octomap_updaters:=false` (HW wajib; move_group OOM 115 mnt tanpa itu),
`run_g54 --tau-max` (HW 14, keputusan operator), `p1hw_*.sh`. Celah: runner tidak cek move_group / HALTED tidak stop; goal-joints
tidak dikoreksi rotasi gantry terukur (GOAL-MISMATCH s12); torsi transit > prediksi ujung; s0 HW/sim 1.29.

## Laporan G61 (ditutup 2026-10-14, offline + mock, NOL gerak HW)

C1–C4 LULUS; dugaan 8/13 (salah D553/D554/D555/D560/D562, tidak diubah sesudah hasil). Penutupan A5.4: cap waktu kill
1791945205.800444055 → stop 1791945205.8274, 0 CMD sesudah stop, gerak pasca-stop g1/g2 nol, rc 3. Kedua probe tugas aktif dalam
penyaringan saat kill; bukan uji kill saat lengan bergerak. A5.3 `s26` menolak DRY tanpa move_group sebelum t0, rc 3. Pemulihan
mock 7/7 → REST → g1 PARK 1.50 → K0 LULUS → shutdown; domain 77 kosong. Log dan JSON di `results/p1_g61/`; kode runner tidak
diubah. Validasi HW perubahan G61 BELUM (G62); rekomendasi offset j2 8.1, transit tugas→tugas, susut reservasi sesudah MOVED,
model biaya reservasi belum diterapkan. Model sesi berikut: model kuat setara Opus, effort **high** untuk G62/CAM-6DOF HW;
medium–high untuk penulisan/analisis offline. Catatan proses: anggaran 4.000 token per tugas CLAUDE.md Rule 6 terlampaui saat
melanjutkan pemeriksaan, pemulihan dan penutupan; tidak disembunyikan.

## Langkah berikut

G61 selesai (terpisah dari CAM-6DOF): sesi runner berikut mengikuti `docs/p1_g62_hw_prompt.md`, model kuat effort high; setiap gerak HW menunggu izin operator per langkah.

Sesi CAM-6DOF HW: tempel ulang `docs/p1_cam6dof_prompt.md` SAMA; langkah 0 sudah selesai (§A0 `docs/p1_cam6dof.md`) → mulai gerbang K0–K3b + angka (balok A/B ±2 mm, diameter botol, lantai z 0), bring-up, `cam6dof_capture` + `arm_fk_check` ×3, `proj_marks` K4, plan-check HW tanda F, lalu KUNCI §A (draf A1–A4) sebelum data botol / gerak pertama.

### Checkpoint CAM-6DOF 2026-10-14 (pra-bring-up)

Operator mengonfirmasi gerbang verbal K0–K3b, botol H70/diameter60 mm, lantai z0/meja0.770 m; box A310/B480 mm kokoh, rata, stabil. Ketelitian ±2 mm + ukuran/penempatan box masih menunggu jawaban. Pemeriksaan host read-only remount LULUS, domain0/77/79 kosong (bukti `results/p1_cam6dof/preflight_20261014/`). Nol bring-up/gerak/data botol; K0 terukur, K4 dan §A terkunci BELUM. G61/G62 tidak disentuh. Langkah berikut CAM-6DOF: konfirmasi box, bring-up tanpa gerak + K0 terukur, baseline REST/proyeksi/plan-check HW sebelum kunci §A; model kuat effort high.
