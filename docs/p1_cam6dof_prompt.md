P1 — CAM-6DOF: kalibrasi 6-DOF rgbd2 DAN cek/kalibrasi rgbd dengan titik 3D beracuan lengan (tanda unting + botol di beberapa
tinggi). Repo ceiling_arm, branch feat/rgbd-deploy. Model: Opus, effort HIGH (galat diam: tilt, skala depth dan bias centroid botol
bisa memberi angka yang sama; uji dekat selalu lulus). Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari
docs/p1_campaign_state.md ("Langkah berikut"). Waktu: ~3–4 jam (luring ~1 jam tanpa operator; HW ~2–2.5 jam dengan operator).

BACA DULU (penuh): CLAUDE.md; MEMORY.md (terutama demo-hw-rgbd2-range-error, camera-world-vs-urdf-offset, rgbd-extrinsic-calibration);
docs/p1_campaign_state.md; docs/p1_rgbd2_zone.md (penuh); docs/p1_arm1_zone.md (A0, B1–B6); docs/p1_calib_charuco.md (B2, B3);
docs/results/p1_rgbd2_zone/{rgbd2_zone_fit.py, mv_mark.sh, fit_data.json}; docs/results/p1_arm1_zone/{track_median.py, tcp_read.py};
docs/results/p1_calib_charuco/arm_fk_check.py; docs/results/p1_demo_hw/harness/*.sh; object_localizer.py (param `camera_namespaces`,
`fuse_radius`).

KEADAAN (akhir RGBD2-ZONE, 2026-10-12 malam): sel home — lengan REST, g1 PARKIR 1.4993 m / 0°, g2 0.0009 m / 0°; stack MATI.
Ekstrinsik dipakai = launch default + koreksi G34 lewat arg (`cam_up.sh`/`cam_rs.sh`); fit planar rgbd2 DITOLAK (θ 13.1°).
Tanda unting di meja kayu zona arm_1 (puncak meja 0.77 m, tepi jauh ≈ x 2.00), TCP terukur @ g1 1.50: M1 (1.7497, 0.5493),
C5 (1.6506, 0.6503), C3 (1.8486, 0.6786), C6 (1.9004, 0.6004), N1 (1.9389, 0.4984), N2 (1.9388, 0.6806).

FAKTA YANG DIPEGANG:
- rgbd2 di meja zona arm_1 (~2.9 m): galat botol 0.14–0.23 m (incumbent); di tinggi lengan: arm_3 1.15 m ≈ 0.02, arm_2 2.1 m 0.07,
  arm_1 2.7 m 0.12 (`--arm-filter 0.03`). Galat tumbuh dengan jarak ≈ jarak × 2–3° → dugaan utama TILT (G34 hanya koreksi yaw),
  alternatif skala depth unit rgbd2. Yaw planar 13° menutup meja ke 1–2 cm tetapi merusak lengan dekat > 25 cm → bukan yaw.
- rgbd: lengan ≤ 2.5 m 2–5 cm (arm_4 2.5 m 0.02–0.04) — TAPI rgbd tidak pernah dicek di permukaan meja jauh (z ~0.8, ~3 m);
  zona arm_1 di luar bingkai rgbd. G34: kedua kamera tidak sepakat ~5 cm / 2–3° di adegan statis. Kedua kamera melihat arm_2 di
  ~+0.05 x @ g1 1.50 (belum diatribusi). "Hanya rgbd2 yang salah" BELUM terbukti — sesi ini harus menjawabnya dengan data.
- `object_localizer` memfusi kedua kamera (`fuse_radius` 0.10) → fit PER KAMERA butuh localizer per kamera:
  `-p camera_namespaces:="['rgbd2']" -r __node:=loc_rgbd2` dan remap SEMUA keluarannya (`/detected_objects`, `/detected_objects/tracks`,
  `/target_object`, marker/box) ke `/loc_rgbd2/...` — cek nama topik di kode SEBELUM dipakai; jangan ada dua publisher di topik demo.
- Penunjuk lengan + unting: TCP ≤ 2.1 mm (11 gerak, 2 sesi). Botol: sebaran track ≤ 7 mm, penempatan ulang ~1.3 cm. Centroid track
  bias ke sisi kamera (bergantung arah pandang) — masuk ke model sebagai bagian rantai, catat.
- `arm_fk_check --arm-filter 0.03` = acuan bentuk lengan (metode lama bias ~+4–5 cm); kestabilan ~±3 cm di lengan jauh; link jari
  tanpa TF dilewati + dicetak.
- Plan-check zona arm_1 @ z 1.22: j2/j3 90–96 %; titik y ≥ 0.75 TORQUE-UNSAFE; torsi transit bisa 13 N·m (batas keras > 14).
- SIGINT ke wrapper `ros2 run` tidak mematikan anak → kill PID `lib/<pkg>/<node>`. `pgrep/pkill -f` DILARANG.

GERBANG (AskUserQuestion SEBELUM gerak/bring-up; luring boleh jalan dulu): K0 sel = akhir RGBD2-ZONE? K1 operator di e-stop FISIK
tiap gerak? K2 lampu nyala; botol putih 7 cm; tanda unting masih di meja; DUA balok kokoh rata, tinggi diukur ±2 mm (~15 cm dan
~35 cm, botol bisa berdiri); K3 tak ada task server / reach_fusion (cek `ps`)? K3b operator konfirmasi fisik: rgbd2 = kamera di plafon sisi GANTRY 2 dekat
ujung home rel (ekstrinsik (−0.15, −1.02, 2.09)), rgbd = sisi gantry 1 dekat ujung jauh ((1.72, 0.87, 2.08)) — bila tertukar,
BERHENTI (namespace/serial tertukar = temuan tersendiri). K4 (rgbd): adakah permukaan rata (meja/lantai/balok
yang boleh ditaruh) di sisi gantry 2 yang TERLIHAT rgbd pada ~2.5–3 m dan TERJANGKAU penunjuk arm_3/arm_4 dengan g2 di 0 (tanpa
gerak rel)? — beri operator gambar proyeksi kandidat di citra rgbd (pola DEMO-HW B1) sebelum bertanya.

TUGAS — dokumen fase docs/p1_cam6dof.md: §A (kriteria + model + pilihan titik fit/uji-tahan + dugaan D590+) DIKUNCI sebelum gerak
pertama (salinan results/p1_cam6dof/cam6dof_A_locked.md + sha256), §B ditulis, commit per bagian, berkas keadaan diperbarui sesudah
tiap langkah berarti.
0. LURING (nol gerak): (i) alat fit per kamera `cam6dof_fit.py`: titik botol (track localizer SATU kamera → frame kamera lewat
   ekstrinsik incumbent) ↔ titik world (tanda xy + tinggi permukaan + offset centroid), DITAMBAH titik lengan tersaring
   (point-to-surface hull URDF, lengan dekat DAN jauh); model M6 = 6-DOF kaku (utama, dikunci), M7 = M6 + skala depth (dilaporkan;
   dipakai HANYA bila aturan terkunci berbasis LOO pada set FIT — bukan uji-tahan — memilihnya). Bobot botol vs lengan dikunci di §A.
   (ii) self-test sintetis: tilt 2.5° diketahui + skala 1.04 diketahui pulih; konvensi arg `static_transform_publisher` round-trip
   (TF hidup = prediksi ≤ 1 mm / 0.01°); kontrol negatif (yaw-saja tidak menutup tilt). (iii) harness `loc_cam.sh <ns>` untuk
   localizer per kamera. Tulis di §A0.
1. Bring-up nyata (pola DEMO-HW B2, `octomap_updaters:=false`), K0 `--rail1 1.50 --rail2 0.00`, monitor + perekam (cek tumbuh).
   Kamera incumbent (`cam_up.sh seg_imgsz:=1280`) + localizer per kamera. `arm_fk_check --arm-filter 0.03` REST (kedua kamera).
2. Zona rgbd2 (NOL gerak lengan, tanda sudah ada): botol di 6 tanda × 3 tinggi (meja, balok ~15, balok ~35; tinggi diukur);
   `track_median` 15 s per posisi dari `/loc_rgbd2/tracks`. Uji-tahan dipilih di §A SEBELUM data (mis. C6 + C3 di dua tinggi).
3. Zona rgbd (hanya bila K4 ya): plan-check penunjuk arm_3/arm_4 ke ≥ 5 titik di permukaan itu (g2 di 0; rel TIDAK digerakkan
   kecuali operator setuju + `pose_to_g` stop jadwal); tiap gerak ditanya; tanda unting + label; `return_rest`; botol di tanda × 2–3
   tinggi dari `/loc_rgbd/tracks`. Bila tidak ada permukaan: rgbd dinilai hanya dari lengan tersaring + balok di meja yang terlihat
   rgbd (lapor batasnya).
4. Fit per kamera (M6) → arg baru → kamera diluncurkan ulang (TF hidup = prediksi) → uji-tahan HIDUP: botol di tanda uji-tahan
   (semua tinggi) + `arm_fk_check` REST + botol di satu titik yang terlihat KEDUA kamera (kesepakatan antar-kamera).
5. Keputusan (dikunci di §A sebelum langkah 2): kamera DITERIMA bila uji-tahan hidup |dxy| ≤ 5 cm (dan z dalam ± 3 cm dari
   permukaan + centroid botol) di SEMUA tinggi, lengan terlihat tersaring |t_xy| ≤ 5 cm dan tidak lebih buruk dari incumbent > 1 cm,
   kesepakatan antar-kamera ≤ 5 cm. rgbd yang incumbent-nya sudah lulus kriteria yang sama → TIDAK diubah (dilaporkan "rgbd tidak
   salah", jawab pertanyaan operator). Hasil diterima = harness BARU (`cam_up_6dof.sh`); default launch TIDAK diubah tanpa operator.
   Gagal → incumbent tetap, laporkan model mana (tilt / skala / bias botol) yang gagal dan kenapa.
6. Akhir: lengan REST, g1 PARKIR, K0, matikan bersih (node 0 di domain 0 dan 79, 0 `static_transform_publisher`), arsip /tmp,
   laporan + jawaban eksplisit "apakah hanya rgbd2 yang salah?", rekomendasi model/effort sesi berikut, "Langkah berikut" satu kalimat
   (DEMO-HW ulang bila diterima).

ATURAN KERAS: SETIAP gerak nyata ditanya dulu (satu langkah per persetujuan, termasuk rel dan pulang). Fault / LED merah / REJECTED /
NOT armed sesudah ARMED / torsi > 14 / benturan / kabel / gerak tak diminta → BERHENTI, amankan, tulis, tanya. MARGIN = TOLAK
(plan-check terketat > 97 % dibuang); penyaring TIDAK dilonggarkan. Kriteria terkunci tidak diubah sesudah melihat data; titik
uji-tahan TIDAK pernah dipakai untuk fit atau memilih model. Gantry hanya digerakkan bila operator setuju dan lewat `pose_to_g`.
