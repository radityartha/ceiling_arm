# P1 / ARM1-ZONE — galat +0.15 m di zona arm_1: lengan/gantry atau kamera? (lalu DEMO-HW ulang bila lulus)

> Sesi 2026-10-12 malam (prompt [p1_arm1_zone_prompt.md](p1_arm1_zone_prompt.md); lanjutan [p1_calib_charuco.md](p1_calib_charuco.md)
> B6 F1–F3 dan [p1_demo_hw.md](p1_demo_hw.md) B6 F1). §A dikunci (salinan `results/p1_arm1_zone/arm1_zone_A_locked.md` + sha256)
> sebelum gerak pertama. Data: [results/p1_arm1_zone/](results/p1_arm1_zone/).

## A. Protokol — DIKUNCI

### A0. Gerbang + kerja luring (NOL gerak)

**Gerbang operator:** K0 sel = akhir CALIB-CHARUCO ✓; K1 operator di e-stop fisik tiap gerak ✓; K2 lampu nyala, botol putih 7 cm
siap, botol wastafel disingkirkan, papan ChArUco boleh diangkat ✓; K3 tak ada task server / reach_fusion ✓ — `ps`: hanya
`ros2-daemon` domain 0 dan 79.

| # | Temuan / keputusan | Bukti |
|---|---|---|
| A0.1 | **Bug titik ORIGIN diperbaiki (opsi 1).** `--board-xyz` = titik ORIGIN TERCETAK (corner id0), sesuai docstring kedua berkas dan cetakan; objek PnP kini di-flip lalu digeser −id0 (`_to_origin_dot_frame`, dipakai jalur ChArUco DAN fallback chessboard polos). Docstring tidak perlu diubah (sudah benar; kodenya yang menyimpang). Uji sintetis round-trip, kebenaran = anotasi tercetak (`_annotate_axes`: titik di id0, +X ke id1, +Y dicermin), 4 pose kamera (dekat, papan yaw 0.40, mirip rgbd, mirip rgbd2), kedua pelabelan 180° fallback: **eksak (tanpa noise) ≤ 0.0024 mm / 0.00007°** (ambang semula 1 µm saya longgarkan ke 10 µm sesudah melihat 2.4 µm = presisi float32 + toleransi iteratif solvePnP; tetap 20 000× di bawah bug); **kontrol negatif (konvensi lama) 49.50 mm = (+3.5, −3.5) cm di frame papan, semua kasus**; render ujung-ke-ujung jalur ChArUco 1.6–2.2 mm. → **PASS**. | `ros2_ws/src/reachability_gng/calibrate_extrinsics.py`; [origin_roundtrip.py](results/p1_arm1_zone/origin_roundtrip.py) + [log](results/p1_arm1_zone/origin_roundtrip.log) |
| A0.2 | **Temuan sampingan fallback chessboard polos (tidak diperbaiki, di luar lingkup).** Pada render ~9 px/kotak (rgbd2-like, ~2.4 m) sudut terdeteksi meleset ≤ 3.6 px dan solvePnP jatuh ke solusi planar ambigu: pelabelan benar → dR 92°, pelabelan lain → posisi 5.5 cm tetapi dR 180° (dipilih hint). Pada render rgbd-like (11 px) 2.5 cm / 0.95°. Artefak render sebagian (data nyata RMS 0.31–0.45 px), tetapi: kalibrasi fallback berikut WAJIB memeriksa rotasi hasil (mis. proyeksi lengan), bukan RMS. | debug di sesi; tidak di-commit sebagai uji |
| A0.3 | **`arm_fk_check.py --arm-filter TOL`.** Jendela xy 0.40 m (z 1.15–1.45 sama), permukaan hull URDF lengan (`RobotGeom`, sampel ~5 mm) dipose oleh TF hidup yang sama dengan sumbu metode lama; geser terbaik = grid xy ±0.25 m / 1 cm (skor = jumlah titik ≤ TOL) lalu ICP translasi xyz; lapor `t` (kamera − URDF), n jendela/dipakai/disimpan/dibuang, n tersimpan di t = 0, median residu tersimpan. Metode lama tidak diubah (dicetak berdampingan). Self-test luring (fungsi diekstrak, tanpa ROS): lengan-mirip digeser (+0.15, +0.02) + noise 5 mm + "tirai" 40 % → `t` (0.150, 0.020, 0.001), tirai 0 tersimpan, lengan 100 %; **kontrol tirai-saja: 25 % tetap tersimpan** (bidang menyinggung silinder) → `kept_frac` sendiri BUKAN pembeda; median residu (lengan 3.3 mm vs strip 13.1 mm) ya. → PASS. Catatan: di 2.7 m noise depth D455 ~2–3 cm → median residu nyata dibandingkan dengan lengan dekat (kontrol positif), bukan dengan ambang sintetis. | [arm_fk_check.py](results/p1_calib_charuco/arm_fk_check.py); [arm_filter_selftest.py](results/p1_arm1_zone/arm_filter_selftest.py) + [log](results/p1_arm1_zone/arm_filter_selftest.log) |
| A0.4 | **Cek `--board` lama tak bermakna.** Pusat papan dihitung di frame sudut-luar MENTAH (+Y ke bawah halaman) → 21 cm dari pusat sebenarnya, dan papan rata di permukaan laci setinggi sama → median depth di jendela = permukaan laci. Pusat dibetulkan ke konvensi A0.1 (titik + R(yaw)·(0.0525, −0.0875)); masalah "papan rata di permukaan rata" tetap → cek ini tidak dipakai. | `p1_calib_charuco/eval_incumbent_g1130.json` (`rgbd:board` d (0.003, −0.044)) |
| A0.5 | **Data lama memberi petunjuk sebagian (a).** g1 1.30 (incumbent): rgbd melihat arm_1 (0.9 m) di (+0.075, −0.035) vs arm_2 (+0.037, −0.003); rgbd2 arm_1 (+0.135, +0.090) vs arm_2 (+0.061, +0.043). **Kedua kamera: arm_1 − arm_2 ≈ +0.04…+0.07 di x** (y tak sepakat). Bisa arm_1 relatif arm_2 ≠ URDF ~5 cm, bisa galat kamera bergantung posisi. | `eval_incumbent_g1130.json` |
| A0.6 | 🔴 **Botol-di-tanda TIDAK membedakan lengan vs kamera.** Tanda = unting dari ujung jari FISIK arm_1. Bila arm_1/gantry fisik di FK + δ dan kamera benar → track − FK = δ; bila lengan benar dan kamera +δ → track − FK = δ. Angka sama (galat diam yang disebut prompt). Botol-di-tanda menguji **kesiapan demo** (lengan dikirim ke FK dari track → meleset δ di kedua kasus), bukan atribusi. Atribusi butuh acuan BEBAS kamera: **pita fisik** (jarak basis arm_1–arm_2 vs URDF; atau tanda unting arm_1 ke tanda unting arm_2 vs jarak FK) + gantry dicek lewat arm_2 (dekat rgbd, ≤ 4 cm). Langkah pita dijadikan WAJIB untuk atribusi (bila operator bisa); tanpa pita atribusi dilaporkan "tak teratribusi". | penalaran; A0.5 |
| A0.7 | **p_cmd demo ≠ T.** Botol 7 cm di meja 0.77 → centroid ~0.80 → p_cmd demo ≈ (x, y, **1.05**), bukan 1.22 (mock 3/3 di 1.22 berasumsi meja 0.97). 3D dari basis arm_1 ~0.93 m; DEMO-HW B3 (1.94, 0.70, 1.07) PLANNED j2 99.6 %. Plan-only demo akan memutuskan; penyaring tidak dilonggarkan. | `p1_demo_hw.md` B3 |
| A0.8 | Kamera satu peluncuran: `p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280` (YOLOE + depth; ekstrinsik incumbent identik dengan `cam_rs.sh`, cek `ps`) dipakai untuk `arm_fk_check` DAN track botol → tanpa relaunch. Penyimpangan kecil dari prompt (cam_rs.sh), alasan: adegan + ekstrinsik sama, satu proses lebih sedikit. | harness |

### A1. 🔒 Urutan (setiap gerak nyata ditanya, satu langkah per persetujuan)

1. Self-test `env_collision` + `interarm_collision`, `remount_check`; bring-up nyata (`p1_demo_hw/harness/stack_up.sh`,
   `octomap_updaters:=false`; gerbang runbook §1); K0 `--rail1 1.50 --rail2 0.00`; monitor + perekam (cek tumbuh). Kamera
   (A0.8, domain 0) + localizer. `arm_fk_check --arm-filter 0.03 --out` (lama + baru), ulang `--arm-filter 0.05` (sensitivitas, info).
2. Plan-only arm_1 → T = (1.75, 0.55, 1.22) (`reach_dwell_probe --arms arm_1 --target 1.75,0.55,1.22 --approach 0 --trials 1
   --tau-max 14`). **Gerak #1** (tanya): sama + `--move`. TCP = TF `t1_a1_tool_frame` dari sendi terukur, 4 s. Operator menandai
   unting di meja. `arm_fk_check --arm-filter 0.03` saat arm_1 melayang di T. **Gerak #2** (tanya): `return_rest --arms arm_1 arm_2 --move`.
3. Operator meletakkan botol putih 7 cm TEPAT di tanda. Track `/detected_objects/tracks` (≥ 10 s, median) vs TCP terukur xy.
4. Pita (operator, bila bisa): jarak basis arm_1–arm_2 (URDF = jarak TF `t1_a1_base_link`–`t1_a2_base_link`) atau tanda unting
   arm_1 ↔ arm_2 (perlu gerak arm_2, tanya dulu).
5. Keputusan A2 Z5. Lulus → DEMO-HW ulang (langkah 3–6 `p1_demo_hw_prompt.md`, `--tau-max 14`, botol di tanda, 1–3 jalan, tiap
   jalan ditanya). Gagal → BERHENTI, lapor atribusi (Z6), TANPA koreksi lokal.
6. Akhir: lengan REST, g1 PARKIR (tak diperintah), K0, matikan bersih (node 0 di domain 0 dan 79, 0 `static_transform_publisher`),
   arsip /tmp, laporan.

### A2. 🔒 Kriteria + keputusan

| # | Kriteria | Lulus bila |
|---|---|---|
| Z1 | Kontrol positif penyaring | lengan dekat (rgbd:arm_2, rgbd2:arm_3 di g1 1.50): \|t_xy(saring) − d_xy(lama)\| ≤ 3 cm |
| Z2 | (b) titik bukan-lengan? | rgbd2:arm_1 tersaring (TOL 0.03): **\|t_xy\| ≥ 0.10 dan tersimpan ≥ 30 % jendela → (b) DITOLAK** (titik berbentuk lengan memang di +δ); \|t_xy\| ≤ 0.05 → (b) DITERIMA; lainnya = tak tentu. Z1 gagal → Z2 tak sah. |
| Z3 | Penunjuk | arm_1 MOVED, dwell SUCCESS, TCP terukur ≤ 5 mm dari T, torsi terukur ≤ 14 |
| Z4 | Galat kamera di zona demo (keputusan) | track botol − TCP terukur: **\|dxy\| ≤ 5 cm DAN z ∈ [0.79, 0.84]** (meja 0.77 + 0.02 … + 0.07) |
| Z5 | Keputusan | Z3 ✓ dan Z4 ✓ → DEMO-HW ulang di sesi ini (kriteria DEMO-HW §A1 H1, H3–H5 tetap; **H2 diganti Z4** — batas z lama [0.85, 1.05] salah asumsi, `p1_demo_hw.md` B6 F3). Lainnya → BERHENTI. |
| Z6 | Atribusi (hanya bila Z4 gagal; tidak mengubah keputusan) | pita: \|ukur − URDF\| ≥ 5 cm → **lengan/mount arm_1**; ≤ 2 cm → **kamera** (rgbd2 jarak jauh); di antaranya / tanpa pita → tak teratribusi. Konsistensi: (track − TCP) ≈ t tersaring di T (≤ 3 cm). |

**Berhenti + amankan (tanya):** fault / LED merah / REJECTED / NOT armed sesudah ARMED / torsi > 14 / benturan / kabel / gerak tak
diminta. MARGIN = TOLAK; penyaring TIDAK dilonggarkan; kriteria tidak diubah sesudah melihat data.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D570 | Metode lama di g1 1.50 mengulang rgbd2:arm_1 \|d\| ∈ [0.12, 0.18]. |
| D571 | Z1 lulus (kontrol positif ≤ 3 cm). |
| D572 | Z2: (b) DITOLAK — titik berbentuk lengan di \|t_xy\| ≥ 0.10. |
| D573 | Plan-only arm_1 → T PLANNED dalam ≤ 2 sampel. |
| D574 | Z3 lulus; torsi puncak terukur ≤ 12 N·m. |
| D575 | t tersaring rgbd2:arm_1 di T dalam 3 cm dari t di REST. |
| D576 | **Z4 GAGAL**: track − TCP \|dxy\| > 5 cm dengan Δx ≥ +0.08 → BERHENTI (tidak ada demo). |
| D577 | Track z ∈ [0.79, 0.84]. |
| D578 | Pita (bila ada): basis arm_1–arm_2 dalam 2 cm dari URDF → galat = kamera (rgbd2 jarak jauh). |

## B. Hasil terukur

(diisi sesudah §A dikunci)
