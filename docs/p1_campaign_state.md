# Keadaan kampanye P1 G43+ (docs/p1_campaign_prompt.md)

> Diperbarui sesudah tiap langkah berarti dan SEBELUM tiap job panjang. Fase LULUS tidak diulang.

## Fase aktif

**G61 OFFLINE SELESAI** (2026-10-14, `docs/p1_g61_offline.md` B0–B6, skor **8/13**): kode run_g54 tetap 831db96. A5.1 6/6 ×2 argv identik; injeksi rot −0.48 tanpa koreksi 4/6, dengan koreksi 6/6; peringatan perekam OK. A5.4 kill move_group mock → AUTO-STOP **0.027 s**, 0 CMD sesudah stop, rc 3; A5.3 DRY ditolak sebelum t0, rc 3. C3 85/87 j2 ≤ prediksi probe; C4 s0 = penyaring reservasi. Gerbang C1–C4 LULUS. Pemulihan mock REST + PARK, K0 LULUS, seluruh PID milik G61 keluar, domain 77 kosong (shutdown launch memakai eskalasi otomatis SIGTERM/SIGKILL untuk move_group/RViz; B6). Runbook + prompt G62 final. NOL gerak HW; CAM-6DOF tidak disentuh.

**CAM-6DOF HW AKTIF, §A AWAL+AMENDEMEN DIKUNCI** (2026-10-14): A0/G61/baselineREST×3/meja18/18 tidakdiulang. Kunci awald081044/hash3d13ea22…utuh; amendemen commit `3b507b9`, `cam6dof_A_floor_amendment_locked.md` sha256 `2022dbbc72f93e953daefa5d4d197a7c09aa0502214f1a280ed350c2e7ddff80` sebelum botolFlantai pertama. Operator menggantiA/B besar→BASE/D140/CD245(Cbawah,Datas);F1/F5z0/140/245 mm,F3/F4/F6/F7z70/210/315 mm;role/kriteria/modeltetap. Enam penunjuk+GERAK #7pulang arm_3 MOVED/SUCCESS,C6LULUS;peaksesi12.4826≤13,D597✓,D590✗. **SEKARANG SEMUA REST,g1PARK1.499262/g2home0.000901,rot0/0,K0LULUS**,bukanF7. Pulangpeakeksekusi8.4515,job8.7493,TCP0.104 mm. Baseline rgbdarm2/4=5.3/3.0 cm;rgbd2arm1/2/3=13.0/6.2/2.3 cm;N1/C5anomali/ambiguitas/C1RGBD2belumterpenuhi tetap. Botollantai1/18F1BASE selesai:rgbd30/rgbd2 28pesan semuaemptytracks,missing tersimpan;belumfit/kandidat/liveholdout. **KOREKSI OPERATOR H165/diameter65 botolyangsama DIKUNCI shaccd69a959c06f5b92fa917aef04c00424c1f5deb725e3ae7a09437df0d58b6fc;ketelitianbotol±2tidakdijawabeksplisit. Laporan19penempatanterkoreksiterpisah,id/n/median/rawtetap.** StackHIDUP(launch786857,camera787847,monitor787845→787875,perekam787846,localizer804824/804825);verifikasiPID sebelumshutdown. Kunci amendemen3b507b9 selesai;F1BASE15 s kedua rc0,tetapmissing. CekRGBDbaru31label{}+16tracks[],streams hidup;laci dipindahkan operator. Tunggu identifikasibotol7cm/F1tegaksemula sebelumD;scene mapbeku perlu diverifikasi bilaadaHWbaru. CSVbesar tidakcommit. Bukti `docs/results/p1_cam6dof/hw_20261014/`.

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

Sesi CAM-6DOF HW AKTIF: semuaREST/PARK,K0sesudahpulangLULUS,stack/localizer/monitor/perekamHIDUP. Meja18/18+F1BASEmissing rawtetap;enampenunjuk+pulangselesai. Operator mengoreksi **H165/diameter65(r32.5 mm),botolyangsama18meja**; kunciukuran commit `a8a9167`, `cam6dof_A_bottle_geometry_locked.md` sha`ccd69a959c06f5b92fa917aef04c00424c1f5deb725e3ae7a09437df0d58b6fc` sebelumcaptureberikut/fit. Kunci awal/amendemenpenyangga3b507b9utuh;configketigahash+ukuranbenar,helpermembaca r/Hconfig. Semua19penempatanlaporancentroid/dzterkoreksiterpisah,median/id/nidentikraw;C3T0dz−5.98,xy19.38cm. Role/threshold tetap,belumfit/LOO/kandidat. Kunciukuran a8a9167 selesai,ketigahash cocok. Penempatan2/18F1/D140 diminta; tungguoperatorsiap/menjauh sebelumcapture15 s keduakamera; F1BASEmissingtidakditimpa. D590✗/D597✓ tetap;C1N1ambiguitybelumselesai. Laci dipindahkan,peta bekudiverifikasi sebelumHWbaru. CSVbesar tidakcommit;modelkuat efforthigh.

### Checkpoint CAM-6DOF 2026-10-14 (pra-bring-up)

Operator mengonfirmasi gerbang verbal K0–K3b, botol H70/diameter60 mm, lantai z0/meja0.770 m; box A310/B480 mm kokoh, rata, stabil. Ketelitian ±2 mm + ukuran/penempatan box masih menunggu jawaban. Pemeriksaan host read-only remount LULUS, domain0/77/79 kosong (bukti `results/p1_cam6dof/preflight_20261014/`). Nol bring-up/gerak/data botol; K0 terukur, K4 dan §A terkunci BELUM. G61/G62 tidak disentuh. Langkah berikut CAM-6DOF: konfirmasi box, bring-up tanpa gerak + K0 terukur, baseline REST/proyeksi/plan-check HW sebelum kunci §A; model kuat effort high.

### Checkpoint CAM-6DOF hidup 2026-10-14

Box final A310/B480 mm±2 mm, atas470×350/423×268 mm, lantai. Bring-up nyata PID786857 tanpa perintah gerak;7/7active, K0 LULUS(g1 1.499262/g2 0.000901,rot0/0;arm_3REST0.455°). Kamera incumbent serial benar, capture REST selesai. Baseline pertama rgbd:arm_2 5.3 cm/arm_4 3.2 cm;rgbd2:arm_1 12.7 cm/arm_2 6.2 cm/arm_3 2.4 cm;2/3 sedang dihitung. Proyeksi F ditampilkan;K4 operator menunggu, F4 lantai OUT rgbd2,F7 dekat batas bingkai. §A belum terkunci, belum track botol/localizer/plan-check/gerak. Stack+kamera+monitor+perekam CAM-6DOF HIDUP, jangan disentuh sesi G62. Dwell CSV kosong tanpa target (bukan macet terbukti);perekam sendi tumbuh. Bukti `results/p1_cam6dof/hw_20261014/`,log `/tmp/cam6dof_20261014_*`. Langkah berikut CAM-6DOF: baseline×3→K4/ruang box→plan-check HW→kunci§A;setiap gerak tetap izin per langkah.

CAM-6DOF update baseline final: REST×3 selesai, median rgbd arm_2/4=5.3/3.0 cm; rgbd2 arm_1/2/3=13.0/6.2/2.3 cm. rgbd incumbent belum memenuhi C3, atribusi belum selesai. K4 operator area lantai kosong boleh dipakai. F7/F1/F6/F4 plan-check HW PLANNED tanpa --move; F5/F3 menyusul. Pemakaian box A/B pada enam tanda meja masih ditanyakan (tinggi/tepi/jarak REST), sehingga§A tetap DRAF. Stack milik CAM-6DOF tetap HIDUP; belum track botol/localizer/gerak.

CAM-6DOF checkpoint terbaru: baseline REST×3 dan plan-check HW keenam F SELESAI (rencana terpilih91–93%, sampel1/4/2/1/1/2 untuk F1/F6/F4/F5/F3/F7); K0 sesudahnya LULUS, NOL perintah gerak. Operator menolak box31/48 di meja, lalu menawarkan box10/14 cm; ukuran/ketelitian/kekokohan/jarak bebas box kecil masih ditanyakan. Box31/48 tetap untuk lantai. Satu botol putih cukup dan dipakai bergantian. §A masih DRAF, belum botol/fit/uji-tahan/localizer/gerak; jangan mulai ulang baseline atau plan-check yang selesai. Stack/kamera/monitor/perekam CAM-6DOF masih HIDUP; proses milik sesi tercatat `hw_20261014/process_checkpoint.json`. Segmentasi sedang diverifikasi terbit, belum dianggap siap. Langkah berikut CAM-6DOF: konfirmasi box10/14 dan kesiapan segmentasi→kunci§A dengan tinggi meja/lantai aktual→data botol pertama; model kuat effort high, tiap gerak izin satu langkah.
Segmentasi CAM-6DOF terverifikasi hidup:27 gambar/kamera pada subscription8 s,1280×720, cap waktu bertambah (`hw_20261014/seg_readiness.json`). Tidak ada restart atau gerak. Satu-satunya gerbang pra-kunci yang masih pending: detail/kelayakan box10/14 cm di meja.

CAM-6DOF §A DIKUNCI 2026-10-14 sebelum botol/gerak: sha256 3d13ea22a89d87bb3ac789bda902a6f2f489f9594d40b166d9426a14a78dc802, salinan `results/p1_cam6dof/cam6dof_A_locked.md`. Operator box C100 atas95×70/D140 atas92×50 mm muat/kokoh; meja z0.770/0.870/0.910, lantai z0/0.310/0.480. K0 pra-kunci LULUS. D590 box tertinggi meja=D, bukan B lantai; kriteria tetap. Stack CAM-6DOF hidup. Langkah berikut: commit kunci→localizer terisolasi per kamera→rekam botol yang sudah di C3/T0 selama15 s setelah operator menjauh; nol gerak sampai izin satu langkah.

CAM-6DOF localizer terisolasi aktif sesudah kunci commit d081044 (PID804824/804825), topik demo tanpa publisher. Penempatan pertama C3/T0 masih menunggu operator siap; belum data botol atau gerak.

CAM-6DOF botol1/18 selesai C3/T0: rgbd2 satu white bottle, median(2.0269,0.7546,0.8072), galatxy19.4 cm, sebaran6/4 mm;rgbd tanpa track (zona di luar FOV). C3 UJI-TAHAN tidak dipakai fit/LOO. Stack+localizer hidup, nol gerak. Langkah berikut menunggu C3/TC siap (box C100 mm), lalu rekam15 s kedua kamera.

CAM-6DOF botol2/18 selesai C3/TC: rgbd2 satu track bottle tid0,label blue,median(2.0206,0.7611,0.9101),galatxy19.1 cm,sebaran11/9 mm;rgbd tanpa track. Selektor bottle/radius tidak berubah;C3 UJI-TAHAN tidak dipakai fit/LOO. Stack/localizer hidup,nol gerak. Berikut C3/TD box D140 mm tanpa ditumpuk,menunggu siap.

Koreksi penyampaian angka C3/TC: hasil terhitung0.190807 m=19.1 cm, bukan18.8 cm; JSON dan data mentah sudah benar, tidak diubah.

CAM-6DOF botol3/18 C3/TD selesai: rgbd2 lokal satu white bottle tid70,n29,median(2.0226,0.7514,0.9082),galatxy18.9 cm,dz−4.64 cm,sebaranz96 mm,conf0.312. Data/mask mentah tidak dibuang atau diubah;C3 UJI-TAHAN bukan FIT. FotoRGB/overlaydisimpan;operator diminta konfirmasi fisik botol7 cm di boxD14 cm,tanpa tumpukan. Stack/localizer hidup,nol gerak;menunggu diagnosis sebelum C6/T0.

Operator memastikan C3/TD benar botol7 cm di box D14 cm,tanpa tumpukan. Usul menumpuk C+D tidak diterapkan,tinggi§A tetap. Anomali disimpan,model/parameter tidak diubah. Berikut menunggu C6/T0 siap,nol gerak,stack/localizer hidup.

CAM-6DOF checkpoint meja 4/18: C6_T0 disimpan; berikut C6/TC (box C100 mm), operator sudah siap, rekam15 s kedua kamera. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 5/18: C6_TC disimpan; berikut C6/TD (box D140 mm mengganti C, tanpa tumpukan), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 6/18: C6_TD disimpan; berikut M1/T0 (botol langsung di meja, tanpa box), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 7/18: M1_T0 disimpan; berikut M1/TC (box C100 mm), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 8/18: M1_TC disimpan; berikut M1/TD (box D140 mm mengganti C, tanpa tumpukan), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 9/18: M1_TD disimpan; berikut C5/T0 (botol langsung di meja, tanpa box), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 10/18: C5_T0 disimpan; berikut C5/TC (box C100 mm), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 11/18: C5_TC disimpan; berikut C5/TD (box D140 mm mengganti C, tanpa tumpukan), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 12/18: C5_TD disimpan; berikut N1/T0 (botol langsung di meja, tanpa box), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 13/18: N1_T0 disimpan; berikut N1/TC (box C100 mm), menunggu operator siap; N1/T0 tidak ada track, tidak dikarang atau diganti. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 14/18: N1_TC disimpan; berikut N1/TD (box D140 mm tanpa tumpukan), menunggu operator siap; N1 butuh TD terdeteksi agar minimal dua tinggi FIT terpenuhi. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 15/18: N1_TD disimpan; berikut N2/T0 (botol langsung di meja, tanpa box), menunggu operator siap; C1 rgbd2 belum terpenuhi pada N1 (T0 kosong, TC tunggal, TD ambigu). C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.
C1 CAM-6DOF rgbd2 belum terpenuhi: FIT N1/T0 kosong,TC tunggal,TD3 ID kandidat lokal/putus-putus. Saingan tidak digabung/dibuang dan titik tidak diganti; numerik tidak boleh disebut kandidat sah tanpa gerbang. RGBD lantai tetap belum dinilai. Menunggu N2/T0 siap;stack/localizer hidup,nol gerak.

CAM-6DOF checkpoint meja 16/18: N2_T0 disimpan; berikut N2/TC (box C100 mm), menunggu operator siap. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.
CAM-6DOF kesiapan penunjuk: wrapper sesi `harness/probe_cam6dof.py` menegakkan>97% pada rencana yang benar-benar akan dieksekusi (probe bersama hanya menolak>100%); semua saringan/model/offset/batas14 tetap.5 kontrol unit lulus dan--help lolos;plan-only wrapper hidup belum diperiksa. Ini bukan izin gerak. Menunggu N2/TC siap,meja16/18,nol gerak,stack/localizer hidup.

CAM-6DOF checkpoint meja 17/18: N2_TC disimpan; berikut N2/TD (box D140 mm mengganti C, tanpa tumpukan), menunggu operator siap; penempatan meja terakhir. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF checkpoint meja 18/18: N2_TD disimpan; berikut persiapan unting/penanda lantai dan pemeriksaan plan-only penjaga97%; belum izin gerak pertama F1. C3/TD anomali dipertahankan. §A/model/selektor tetap, stack/localizer hidup, nol gerak HW.

CAM-6DOF meja18/18 selesai,D590✗ (dua pasangan M1/N2 sudah gagal,max4/6 walau N1missing terpenuhi),C1rgbd2 N1belum sah. K0 pra-penunjuk LULUS dan wrapperF1 plan-only PLANNED91.475802%,semua collision/retractCLEAR. Operator unting/penanda/area siap,lalu secara eksplisit mengizinkan GERAK #1 arm_3REST→F1(0.25,−0.50,1.22) SAJA. Stack/monitor/perekam/localizer aktif. Langkah berikut: eksekusi F1 dengan semua guard,verifikasi MOVED/SUCCESS/torsi/TCP;belum izin F6/pulang.

CAM-6DOF GERAK #1 selesai F1,MOVED/SUCCESS,torsi5.615,TCPgalat1.281 mm;F1ditandai. Arm_3menahanF1 (BUKAN REST),lainREST,gantrydiam. Berikut plan-onlyF6 dariF1 sebelum izin;BELUM izinF6/pulang. Semua proses stack/localizer hidup.
