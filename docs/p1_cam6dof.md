# P1 / CAM-6DOF — kalibrasi 6-DOF rgbd2 + cek/kalibrasi rgbd dengan titik 3D beracuan lengan (tanda unting + botol × 3 tinggi)

> Sesi 2026-10-12 malam (prompt [p1_cam6dof_prompt.md](p1_cam6dof_prompt.md); lanjutan [p1_rgbd2_zone.md](p1_rgbd2_zone.md) B7 (B) dan
> [p1_arm1_zone.md](p1_arm1_zone.md) B6 F1/F3). Operator sudah pulang sebelum gerbang → **sesi ini LURING saja** ("siapkan yang offline
> dulu"); HW dilanjutkan 2026-10-14. §A di bawah **DIKUNCI 2026-10-14**, sesudah angka operator, K4 hidup dan enam plan-check HW; salinan `results/p1_cam6dof/cam6dof_A_locked.md` + sha256 dibuat SEBELUM data botol pertama dan SEBELUM gerak pertama. Data: [results/p1_cam6dof/](results/p1_cam6dof/).

> **Status operasional lantai:** amendemen atas permintaan operator dikunci pada commit `3b507b9`, [salinan](results/p1_cam6dof/cam6dof_A_floor_amendment_locked.md), sha256 `2022dbbc72f93e953daefa5d4d197a7c09aa0502214f1a280ed350c2e7ddff80`. BASE/D140/CD245 mengganti tinggi penyangga lantai A/B; §A awal/salinan/hash tetap sebagai arsip. Role, model dan batas penerimaan tetap. B33–B34 mencatat REST/K0 lulus dan kunci baru sebelum botolF pertama; jangan memakai tinggi LA/LB lama untuk capture berikut.

## A. Protokol

### A0. Luring (NOL gerak HW) — SELESAI 2026-10-12

| # | Isi | Bukti |
|---|---|---|
| A0.1 | **Alat fit** [cam6dof_fit.py](results/p1_cam6dof/cam6dof_fit.py). Model kamera p_w = R (s p_c) + t, R = R_lama·Exp(w) (w di frame kamera), t = t_lama + τ. **M6** s = 1 (utama); **M7** s bebas. Obs botol: p_c = R_lamaᵀ(track − t_lama) (track localizer SATU kamera, ekstrinsik lama) ↔ target = xy tanda + z permukaan + **offset centroid** (silinder r, H; median per-sumbu piksel terlihat berbobot luas·cos/jarak² = cara `object_localizer`). Obs lengan: piksel depth di jendela tiap lengan (0.40 m xy, z 1.15–1.45, dipilih dengan ekstrinsik LAMA, ≤ 4000), asosiasi = titik permukaan hull URDF terdekat sesudah geser terbaik lengan itu (grid ±0.25/1 cm di awal, lalu ICP, TOL 0.03); **residu point-to-plane** n·(p − q) hanya di permukaan hampir vertikal (\|n_z\| ≤ 0.5). Bobot: botol σ 0.015 per obs 3-D; tiap lengan = 6 obs 1-D σ 0.03. LS berbobot polos (tanpa loss robust — titik buruk harus terlihat). Mulai = fit botol-saja M6, lalu botol + lengan. Pemilihan model: LOO tinggalkan-satu-TANDA (semua tinggi) pada set FIT saja; **M7 bila LOO_M7 ≤ LOO_M6 − max(0.005, 0.15·LOO_M6)**. Gerbang fit: konvergen, \|w\| ≤ 5°, \|τ\| ≤ 0.15 m (geser POSISI kamera, bukan t terhadap origin world — cacat R1 RGBD2-ZONE), \|s − 1\| ≤ 0.06. Arg keluaran `tf{1,2}_*:=` 5 desimal. | kode |
| A0.2 | **Dua cacat ditemukan + diperbaiki saat membangun (sebelum data apa pun):** (a) prediksi geser ICP antar-iterasi bertanda salah (`t − Δ` → `t + Δ`) → fit menyimpang; (b) residu lengan point-to-point memaksa geser "luncur sepanjang link" (tak teramati) ke ekstrinsik → minimum lokal salah (rot 4.3° vs 2.5° benar). Lengan REST tergantung vertikal → **lengan mengikat xy world, BUKAN z**; z datang dari botol di 3 tinggi (fit botol-saja tanpa noise pulih eksak). Itu sebabnya point-to-plane \|n_z\| ≤ 0.5 + inisialisasi botol-saja. | debug di sesi |
| A0.3 | **Self-test sintetis PASS** ([log](results/p1_cam6dof/self_test.log); kamera = rgbd2 incumbent, 6 tanda × 3 tinggi, lengan sintetis arm_1/2/3 di 2.7/2.1/1.15 m): **T1a** tilt 2.5° (sumbu x kamera) + τ (0.02, −0.01, 0.01), tanpa noise → rot 0.0027°, pos 0.13 mm, uji-tahan 0.00 mm, pilih M6. **T1b** + noise botol 5 mm / lengan 3 mm → rot 0.123°, pos 5.7 mm, uji-tahan 8.7 mm, LOO M6 = M7 → pilih M6. **T2** tilt 2.5° + skala 1.04 → M7 s **1.0397**, rot 0.122°, LOO M6 15.3 vs M7 9.9 mm → **pilih M7**. **T3 kontrol negatif** yaw-saja (3-DOF planar, cara RGBD2-ZONE) pada data T1b: xy uji-tahan 10 mm tetapi **z 125 mm** → yaw tidak menutup tilt (M6 3-D 9 mm). **T4** Euler arg 5 desimal ↔ R 0.0002°. Pengungkapan: ambang rot T1 semula 0.1° saya tetapkan sendiri lalu meleset (0.126° dengan noise 5 mm ≈ lantai noise 5 mm / 2.9 m ≈ 0.1°/obs) → dipecah jadi T1a eksak (≤ 0.01°/1 mm) + T1b lantai noise (≤ 0.25°/15 mm/12 mm). Sintetis memakai model centroid yang sama dengan fit (uji solver); ketergantungan offset centroid pada posisi kamera (lama vs baru) < 1 mm. | log |
| A0.4 | **Round-trip `static_transform_publisher`** (biner, domain 78, 3 set arg: incumbent rgbd2, incumbent rgbd, arg fit RGBD2-ZONE): TF hidup = arg **0.000 mm / 0.00000°** ([log](results/p1_cam6dof/tf_roundtrip.log)). Konvensi = scipy `'xyz'` = Rz(yaw)Ry(pitch)Rx(roll). | log |
| A0.5 | **Harness** [loc_cam.sh](results/p1_cam6dof/harness/loc_cam.sh) `<ns>`: `object_localizer` satu kamera, node `loc_<ns>`, keluaran `/loc_<ns>/{detected_objects,tracks,target_object,markers,target_collision_boxes}`, masukan `/grasp_target` → `/loc_<ns>/grasp_target`; menjalankan skrip entri langsung (tanpa wrapper `ros2 run`) → PID = node. Dicek di domain 78: langganan hanya `/rgbd2/*`, 0 penerbit di topik demo; SIGINT PID → hilang. `track_median.py` kini menerima argumen topik ke-3 (bawaan tetap `/detected_objects/tracks`). | sesi |
| A0.6 | **Alat tangkap** [cam6dof_capture.py](results/p1_cam6dof/cam6dof_capture.py): median per-piksel N frame depth, K, TF kamera saat itu, permukaan hull URDF + normal ke luar tiap lengan (TF hidup), sumbu lengan. **Belum pernah dijalankan hidup** (butuh kamera + stack). [proj_marks.py](results/p1_cam6dof/proj_marks.py): proyeksi titik world ke citra kamera hidup (gambar K4, pola DEMO-HW B1) — belum hidup. | kode |
| A0.7 | **Pola galat incumbent rgbd2 (data RGBD2-ZONE B2, sudah dilihat):** galat botol 0.14–0.23 m **sepanjang sinar pandang horizontal** (0.72, 0.70) sementara **z track benar** (0.81–0.83 vs centroid model 0.818). Tilt murni (sumbu x kamera, depresi ~26°) menggeser titik ~0.9 vertikal / 0.44 horizontal → 15 cm horizontal butuh ~30 cm z — tidak teramati. Skala depth murni menggeser sepanjang sinar (horizontal 0.9, z −0.44). **Horizontal-saja dengan z benar = skala + tilt bersama** (≈ s 1.05 + tilt ~1.5° di 2.9 m) — atau kombinasi lain dengan τ; fit yang memutuskan. Ini dasar D591. | penalaran |
| A0.8 | **K4 luring (awan G33 `capture_a.npz` dipetakan ke ekstrinsik incumbent):** meja zona arm_1 di luar bingkai rgbd; **lantai di bawah jangkauan arm_3 (g2 = 0) terlihat rgbd pada 2.6–3.1 m** dan rgbd2 pada 2.3–2.5 m; zona dua-kamera x ≥ 0.55, y −0.45…−0.75 ([peta](results/p1_cam6dof/k4_topdown.png)). Lantai z ≈ 0 (rgbd −0.02, rgbd2 −0.03 di peta G33). Penunjuk arm_3 di z 1.22 + unting ke lantai (1.22 m tali) + botol di lantai / balok 15 / balok 35 → rgbd diuji di ~2.8 m seperti rgbd2 di meja. Permukaan lain dekat: laci (z 0.62) hanya 1.9 m dari rgbd. **G33 lama — tata letak lantai harus dicek hidup (proj_marks) sebelum K4.** | peta |
| A0.9 | **Plan-check MOCK** (domain 77, `use_fake_hardware`, REST + g1 PARK 1.50 lewat `pose_to_g` + g2 0 = K0 HW; arm_3 → (x, y, 1.22) tool-bawah, `--plan-attempts 8`, penyaring tak diubah) [logs](results/p1_cam6dof/mock/): (0.25, −0.50) PLANNED j3 92 %; **(0.40, −0.50) j2 99 % → DIBUANG** (> 97 %); (0.55, −0.50) j3 91; (0.65, −0.48) j3 91; (0.25, −0.60) j3 90; (0.40, −0.60) j3 91 (5× TORQUE-UNSAFE dulu); (0.55, −0.60) j3 90; (0.65, −0.58) j2 91; (0.40, −0.70) j3 92; (0.60, −0.66) j2 96. Antar-lengan min ≥ 408 mm. **Wajib diulang di HW** (plan-check nyata) sebelum dikunci. Mock mati bersih (0 node di 77/78/79/0; crash dump shutdown `move_group` dihapus). | logs |
| A0.11 | **Uji pipa pada rekaman RGBD2-ZONE B2** (6 tanda, meja saja, tanpa lengan; [dryrun/](results/p1_cam6dof/dryrun/)) — UJI KODE, bukan hasil: `cfg_from_tracks.py` → track sama dengan B2 (0 saingan dalam 0.35 m); fit M6 \|w\| **13.7°**, \|τ\| 0.62 → **gerbang GAGAL** (satu tinggi + tanpa lengan = terdegenerasi, sama seperti R1 RGBD2-ZONE). Gerbang menangkapnya; itu sebabnya 3 tinggi + lengan wajib. Alat tambahan: [cfg_from_tracks.py](results/p1_cam6dof/cfg_from_tracks.py) (track terbanyak berlabel botol ≤ 0.35 m dari tanda, saingan DICETAK), [harness/mv_mark.sh](results/p1_cam6dof/harness/mv_mark.sh) `<arm> <tanda> x y`. | dryrun |
| A0.12 | **Tanpa balok? (operator tak punya balok, 2026-10-12)** Sintetis [noblock_synth.py](results/p1_cam6dof/noblock_synth.py) ([log](results/p1_cam6dof/noblock_synth.log)), tilt 2.5° ± skala 1.04, noise 5/3 mm, lengan silinder vertikal: **rgbd2** (meja 0.77 + lantai 0 = dua tinggi alami) tanpa penyangga tetap pulih (kaku rot 0.07°, uji-tahan 17 mm; skala → M7 dipilih, s 1.040). **rgbd** (lantai saja, satu tinggi): kaku pulih (0.04°), TETAPI dengan skala 1.04 **LOO memilih M6** — pose kamera salah **112 mm** padahal uji-tahan di zona hanya 12 mm (galat diam: lulus di zona, rusak di tempat lain). Dengan 3 tinggi rgbd memilih M7 benar. → penyangga (benda kokoh rata apa pun, tinggi diukur) WAJIB minimal untuk zona lantai rgbd. | log |
| A0.10 | **Batas penerapan M7:** kamera nyata → `depth_image_proc/convert_metric` tanpa parameter skala; TF statis tidak bisa membawa skala. Bila LOO memilih M7: **M6 tetap yang diuji-tahan hidup** (satu-satunya yang bisa dipasang lewat arg), M7 dilaporkan + prediksi luring; penerapan skala = perubahan kode, keputusan operator. | `realsense_dual.launch.py` |

### A1. DIKUNCI urutan (setiap gerak nyata ditanya, satu langkah per persetujuan)

0. Gerbang K0–K3b operator terpenuhi; K0 terukur LULUS sebelum kunci (B1–B3). Acuan lantai world z=0, meja z=0.770 m. Satu botol putih silinder: H=0.070 m, diameter=0.060 m, r=0.030 m; dipakai bergantian, tidak mengganti botol. Semua tinggi penyangga diukur operator ±2 mm, kokoh/rata/stabil. Box lantai A=0.310 m, atas0.470×0.350 m; B=0.480 m, atas0.423×0.268 m. Box meja C=0.100 m, atas0.095×0.070 m; D=0.140 m, atas0.092×0.050 m. Operator mengonfirmasi C/D dapat dipakai pada enam tanda tanpa menyentuh REST/jatuh. A/B tidak ditaruh di meja (operator menolak). Box D lebih sempit5 mm per sisi daripada diameter botol; botol harus tegak/stabil dengan pusat alas tepat di tanda, bukan mengikuti tepi box. Bila tidak stabil, STOP penempatan dan catat missing; tidak mengarang koordinat/tinggi.
   **Tinggi permukaan terkunci:** meja T0=0.770, TC=0.870, TD=0.910 m; lantai L0=0, LA=0.310, LB=0.480 m. Tidak ditumpuk. **Konflik draf:** D590 semula “balok B di meja”; B kini nama box lantai operator, sehingga dugaan yang sama diterapkan pada box tertinggi di meja **D** (tinggi14 cm), sebelum data botol. Model/bobot/gerbang A0.1 dan C1–C6 tidak dilonggarkan. A0.7/A0.12/A0.10 menang atas dugaan tilt murni/satu tinggi/penerapan M7 lewat TF di prompt lama.
1. Bring-up nyata (`p1_demo_hw/harness/stack_up.sh`, `octomap_updaters:=false`), K0 `--rail1 1.50 --rail2 0.00`, monitor + perekam
   (cek tumbuh). Kamera incumbent (`p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280`); localizer per kamera dimulai SESUDAH kunci §A agar tidak ada data botol sebelum kunci. Keluaran terisolasi mengikuti A0.5; topik demo harus0 publisher dari localizer sesi.
   `arm_fk_check --arm-filter 0.03` REST ×3 (baseline incumbent) + `cam6dof_capture.py rest_inc.npz` (data lengan untuk fit).
   `proj_marks.py rgbd` + `rgbd2` dengan kandidat F → gambar → **K4** ke operator. Plan-check HW F terpilih.
   → isi angka + set F → **KUNCI §A** (sha256 + commit).
2. Zona rgbd2 (NOL gerak): botol di 6 tanda meja × 3 tinggi (T0/TC/TD = meja, box C, box D), `track_median.py 15 … /loc_rgbd2/tracks`
   (+ `/loc_rgbd/tracks` direkam juga). Pusat alas botol tepat di atas tanda, box rata dan kokoh, garis bantu untuk xy. Rekam15 s per kamera setelah penempatan selesai dan operator menjauh; satu penempatan per instruksi, tidak berpindah selama rekaman. Urutan meja: C3 (botol sudah ada), C6, M1, C5, N1, N2; di setiap tanda T0→TC→TD. Urutan tidak mengubah peran FIT/UJI-TAHAN.
3. Zona rgbd (hanya bila K4 ya): arm_3 penunjuk ke F (tiap gerak ditanya; pola `mv_mark.sh` dengan arm_3) → unting + label → `return_rest
   --arms arm_3 arm_4 --move` → botol di tiap F × 3 tinggi (L0/LA/LB = lantai, box A, box B), rekam kedua localizer. Urutan penunjuk arm_3 dari REST: F1→F6→F4→F5→F3→F7, lalu pulang REST; tiap langkah harus disaring ulang dan ditanya, tidak ada izin gabungan. Koordinat tanda aktual = TCP terukur saat dwell, bukan nominal perintah; peran dan tinggi tidak berubah. Tidak menaruh box pada jalur lengan sebelum penunjuk selesai REST.
4. Fit per kamera (M6/M7, `cam6dof_fit.py`) → arg baru → kamera diluncurkan ulang (`cam_up_6dof.sh`) → TF hidup = prediksi →
   uji-tahan HIDUP (botol di tanda uji-tahan semua tinggi) + `arm_fk_check` REST ×3 + botol di titik dua-kamera.
5. Akhir: lengan REST, g1 PARKIR, K0, matikan bersih (0 node domain 0 & 79, 0 `static_transform_publisher`), arsip, laporan.

### A2. DIKUNCI peran titik (dikunci SEBELUM data botol)

| zona | tanda | peran rgbd2 | peran rgbd |
|---|---|---|---|
| meja arm_1 (T0/TC/TD: z0.770/0.870/0.910) | M1, C5, N1, N2 | FIT | — (tak terlihat) |
| | **C6, C3** | **UJI-TAHAN** | — |
| lantai arm_3 (L0/LA/LB: z0/0.310/0.480), keenam F lulus plan-check HW B3 | F1 (0.25, −0.50), F6 (0.40, −0.70) | — (tak terlihat) | FIT |
| | F4 (0.55, −0.60), F5 (0.65, −0.48) | FIT (F4: LA/LB; L0 di luar bingkai incumbent; F5: L0/LA/LB) | FIT |
| | **F3 (0.25, −0.60)** | — | **UJI-TAHAN** |
| | **F7 (0.65, −0.58)** = titik dua-kamera | **UJI-TAHAN** | **UJI-TAHAN** |
| cadangan luring, **TIDAK masuk set terkunci** | (0.55, −0.50), (0.60, −0.66), (0.40, −0.60) | — | — |

**xy meja terukur (m), dari RGBD2-ZONE:** M1=(1.7497,0.5493), C5=(1.6506,0.6503), N1=(1.9389,0.4984), N2=(1.9388,0.6806), C6=(1.9004,0.6004), C3=(1.8486,0.6786). Angka tidak diubah setelah data. F nominal ada di tabel; actual xy diperbarui hanya dari TCP terukur penunjuk sebelum botol lantai, dicatat dengan nama sama. Jika tanda tidak bisa dibuat/terlihat/track tidak tunggal, missing/gerbang gagal dicatat; tidak mengganti tanda/peran berdasarkan galat botol. Uji-tahan tidak masuk fit, inisialisasi, LOO atau pemilihan M6/M7. Rekaman incumbent di tanda uji-tahan dipakai hanya menilai incumbent/delta penerimaan.

Lengan (REST, tangkapan `hw_20261014/rest_inc.npz`) masuk FIT bila lolos gerbang incumbent (jendela ≥ 300, tersimpan ≥ 30 % dan ≥ 200, median ≤ 15 mm).

### A3. DIKUNCI kriteria (per kamera)

| # | Lulus bila |
|---|---|
| C1 | fit sah: gerbang A0.1 + tiap tanda FIT punya track botol tunggal di ≥ 2 tinggi |
| C2 | **uji-tahan HIDUP, ekstrinsik baru, SEMUA tinggi: \|track_xy − tanda_xy\| ≤ 0.05 DAN \|track_z − (z permukaan + h_c model)\| ≤ 0.03** |
| C3 | lengan HIDUP REST (`arm_fk_check --arm-filter 0.03` ×3, median per lengan): tiap lengan yang terlihat kamera itu di baseline: \|t_xy\| ≤ 0.05 DAN ≤ incumbent + 0.01, masih terlihat (tersimpan ≥ 30 %) |
| C4 | dua-kamera (F7): \|basis_rgbd − basis_rgbd2\|xy ≤ 0.05, basis = track − offset centroid model kamera itu (mentah dilaporkan) |
| C5 | TF hidup = prediksi ≤ 1 mm / 0.01° |
| C6 | penunjuk: tiap gerak MOVED + dwell SUCCESS, TCP ≤ 5 mm, torsi terukur ≤ 14 |

**Incumbent pembanding:** baseline REST median B2 tetap sebagai pembanding C3; botol incumbent juga direkam di tanda uji-tahan, semua tinggi zona yang bersangkutan, sebelum fit dan tidak dipakai memilih model. rgbd incumbent baseline sudah gagal C3 (arm_2=5.3 cm); fit rgbd tetap dicoba dengan gerbang sama, tidak menyatakan “rgbd tidak salah” dari botol saja. Bila M7 terpilih, lapor hasil dan prediksi luring; **M6 tetap kandidat uji hidup** (A0.10), tanpa mengubah depth atau launch default.

**Keputusan:** rgbd2 baru DITERIMA bila C1–C5 (C6 untuk sesi). rgbd: bila **incumbent** lulus C2 + C3 → TIDAK diubah ("rgbd tidak salah");
lainnya → fit rgbd, kriteria sama. Diterima = harness BARU `cam_up_6dof.sh`; default launch TIDAK diubah tanpa operator. Gagal → incumbent
tetap; lapor model mana yang gagal (tilt / skala / bias botol) dan kenapa. Berhenti + amankan: fault / LED merah / REJECTED / NOT armed
sesudah ARMED / torsi > 14 / benturan / kabel / gerak tak diminta. MARGIN = TOLAK (> 97 %); penyaring tidak dilonggarkan. Kriteria tidak
diubah sesudah melihat data; uji-tahan tidak pernah dipakai untuk fit / memilih model.

### A4. DIKUNCI dugaan

| # | Dugaan |
|---|---|
| D590 | rgbd2 incumbent: \|dxy\| botol di box D (tertinggi di meja, lebih dekat kamera) < di meja pada ≥ 5/6 tanda. |
| D591 | rgbd2: **LOO memilih M7**, s ∈ [1.02, 1.07] (A0.7: horizontal-saja dengan z benar). |
| D592 | rgbd2 M6 \|w\| ∈ [1°, 4°], \|τ\| ≤ 0.10 m, gerbang lulus. |
| D593 | rgbd2 M6 lulus C2 (semua tinggi) — walau M7 dipilih. |
| D594 | rgbd2 M6 lulus C3 (arm_1 2.7 m \|t_xy\| ≤ 0.05). |
| D595 | rgbd incumbent di lantai (~2.8 m): \|dxy\| ≤ 0.05 di uji-tahan → rgbd TIDAK diubah ("hanya rgbd2 yang salah" = ya). |
| D596 | C4 dua-kamera ≤ 0.05 sesudah rgbd2 baru. |
| D597 | arm_3 penunjuk: semua tanda PLANNED ≤ 8 sampel di HW, torsi terukur puncak ≤ 13 N·m. |
| D598 | z track − (permukaan + h_c) setelah fit ≤ 1.5 cm di semua tinggi (model centroid silinder memadai). |

## B. Hasil terukur

### B0. Checkpoint pra-bring-up 2026-10-14 — NOL gerak HW

- Melanjutkan HEAD `178874f` pada `feat/rgbd-deploy`; G61 tidak diulang. Berkas tak terlacak sesi lain tidak disentuh. Memori Claude tersedia dan dibaca; ringkasan lama G61 yang masih menyebut dijeda kalah oleh git + dokumen fase.
- Operator: K0 sel aman/keadaan sesuai pertanyaan REST/PARK, K1 siap di e-stop fisik, K2 lampu + tanda meja + botol tersedia, K3 tidak ada sesi/task server/reach_fusion, K3b identitas fisik rgbd/rgbd2 benar; lantai world z=0 dan meja 0.770 m dikonfirmasi. K0 TERUKUR belum dijalankan; pernyataan operator bukan pengganti K0.
- Botol putih: H=0.070 m, diameter=0.060 m (r=0.030 m), saat ini di C3. Tidak ada track botol yang direkam.
- Dua box dilaporkan kokoh, rata, stabil: A=0.310 m dan B=0.480 m. Ketelitian ±2 mm, ukuran permukaan atas, penempatan lantai/tumpuk masih ditanyakan. Angka ini belum dikunci. Box dimaksudkan untuk zona lantai arm_3/arm_4; posisi tepat menunggu proyeksi K4 hidup dan pemeriksaan jarak bebas. Penyangga lebih tinggi dari usulan 0.15/0.35 m merupakan perubahan draf sebelum data, bukan perubahan kriteria.
- Pemeriksaan sandbox awal hanya melihat namespace proses sandbox; TIDAK dipakai menyatakan host kosong. Pemeriksaan host melalui eskalasi read-only: remount_check LULUS (USB0/1 ada, IP 192.168.2.100/24, empat ICMP menjawab, tanpa proses basi); node domain 0/77/79 kosong. Bukti di `results/p1_cam6dof/preflight_20261014/`. lsusb melaporkan 10 perangkat Intel, bukan bukti 10 kamera; serial kamera belum diverifikasi hidup.
- Konflik prompt awal: dugaan utama tilt murni dan kemungkinan rgbd lantai satu tinggi mengikuti hasil A0.7/A0.12 yang lebih baru: tilt belum terbukti, beberapa tinggi untuk rgbd wajib. M7 hanya hasil fit/LOO; TF statis tidak menerapkan skala (A0.10). Tidak ada pelonggaran penyaring.
- Belum dijalankan: bring-up, monitor/perekam, K0 terukur, baseline REST ×3, capture lengan, proyeksi/K4 hidup, plan-check HW, penguncian §A, seluruh data botol/fit/uji-tahan/pemasangan kandidat. Nol gerak, nol proses stack milik sesi yang perlu dimatikan. REST/PARK fisik hanya konfirmasi operator, bukan hasil ukur.
- D590–D598 seluruhnya **— belum diuji**; belum ada skor ✓/✗ atau keputusan C1–C6. Jawaban sementara: “hanya rgbd2 yang salah” BELUM terbukti; bukti lama rgbd2 meja 0.143–0.229 m, rgbd belum diuji jauh pada beberapa tinggi.
- Rekomendasi model kalibrasi tetap M6 utama, M7 lewat aturan LOO FIT saja; model agen kuat, effort high karena galat diam. Langkah berikut: konfirmasi ketelitian/ukuran/penempatan box, lalu bring-up tanpa perintah gerak dan ukur K0 sebelum baseline.

### B1. Bring-up + K0 + capture/proyeksi hidup 2026-10-14 — NOL gerak diperintah

- Operator mengonfirmasi box A310/B480 mm telah diukur ±2 mm, permukaan A470×350/B423×268 mm, keduanya langsung di lantai. Angka mengganti usulan draf A1 (belum terkunci). Botol H70/diameter60 mm. Posisi box terhadap lintasan/tinggi meja masih harus dinilai sebelum pemindahan; box B di meja akan mencapai z1.25 m (botol hingga1.32 m), jangan menganggap aman terhadap lengan REST.
- Pra-cek tepat sebelum bring-up: remount LULUS, domain0 kosong; bukti `preflight_20261014/*latest.log`. Launch asli PID786857, SIGINT tidak diabaikan (SigIgn0x1001005), HW nyata + bridge true + octomap_updaters false. Empat actuator count6, 7/7 controller active. Table1 NOT armed saat inisialisasi menolak tujuan awal nol di g1=1499.3 mm: kondisi pra-ARMED dikenal (A1 runbook/ARM1-ZONE B1), tidak ada NOT armed sesudah ARMED; table2 ARMED. Belum ada perintah gerak yang dikirim.
- K0 awal LULUS (`hw_20261014/k0_initial.json`): g1=1.499262262 m, g2=0.000900590 m, rot0/0. Galat REST maksimum per lengan 0.092/0.198/0.455/0.147° (arm_3 dekat batas0.5°, tidak dilonggarkan). Jari tertutup.
- Log spawner duplikat gagal tetapi 7/7 controller aktif; KDL grup gantry gabungan bukan rantai, plugin CHOMP/Pilz gagal dimuat (library2.5.10/2.5.9 tidak tersedia). OMPL siap (`Using planning interface OMPL`, `You can start planning now`). Peta awal False lalu publisher memulihkan; cek layanan read-only `scene_check.json` berisi env_static_map. Plugin gagal bukan ditutupi; plan-check belum dijalankan.
- Monitor hidup observer-only; CSV dwell0 byte karena `_tick` langsung kembali tanpa target (header belum flush). Ini bukan bukti monitor telah merekam target. Perekam sendi tumbuh323056→3126167→14862362 byte. Tidak mengirim target buatan untuk membuat bukti dwell. PID monitor wrapper787845/anak787875, perekam787846; PID adalah riwayat dan harus diverifikasi ulang sebelum kill.
- Kamera incumbent launch+G34: rgbd serial241122302297, rgbd2 serial234222303079. Capture `rest_inc.npz` selesai, rc0:10 frame per kamera720×1280, depth valid0.73/0.80; TF posisi(1.7166,0.8656,2.0782)/(−0.1547,−1.0197,2.0905).12 geometri jari tanpa TF dilewati dan dicetak sesuai A0.6. All-NaN piksel dicetak, bukan pengganti nilai depth. Belum ada localizer/track botol.
- Baseline REST pertama (`rest_inc_1.json`): |t_xy| rgbd:arm_2=0.053 m, arm_4=0.032 m; rgbd2:arm_1=0.127 m, arm_2=0.062 m, arm_3=0.024 m. Kept0.918–1.000, median3.4–10.0 mm. Baseline2/3 masih berjalan; jangan menyimpulkan keputusan dari satu pembacaan. Nilai rgbd:arm_2 sudah sedikit di atas5 cm, belum teratribusi kamera vs world/gantry.
- Proyeksi F hidup `k4_rgbd{,2}.jpg/.log`: keenam titik lantai IN di rgbd (2.70–2.94 m), tetapi sebagian proyeksi tampak di balik lengan/laci (IN bukan bukti bebas halangan). rgbd2: F1/F6/F4/F3 OUT; F5 IN(v687), F7 IN(v708, hanya12 px dari batas bawah). Konflik dengan peta lama A0.8: F4 lantai tidak terlihat pada incumbent hidup; draf peran FIT rgbd2 di F4 bersyarat tidak boleh diasumsikan terpenuhi. Proyeksi A/B menyusul. K4 operator ditanyakan setelah gambar ditampilkan, masih menunggu jawaban. Kandidat(0.40,−0.50) tetap dibuang.
- §A DRAF, belum dikunci. Belum data botol, fit, uji-tahan, localizer per kamera, plan-check HW, gerak penunjuk/pulang. D590–D598 belum diuji. Stack/kamera/monitor/perekam milik CAM-6DOF masih HIDUP selama gerbang K4; ini checkpoint aktif, bukan penutupan/shutdown. Berkas CSV besar tidak akan di-commit.
- Langkah berikut: selesaikan baseline REST×3, konfirmasi K4/ruang box, lalu plan-check HW titik yang benar-benar bebas sebelum kunci§A.

### B2. Baseline REST final + K4 operator (checkpoint aktif)

Baseline incumbent tiga pengukuran selesai, semua rc0; ringkasan `hw_20261014/rest_inc_summary.json`.

| Kamera:lengan | tiga galat xy (cm) | median (cm) | minimum tersimpan | maksimum median residu |
|---|---|---|---|---|
| rgbd:arm_2 | 5.3 / 5.1 / 5.3 | **5.3** | 100% | 3.4 mm |
| rgbd:arm_4 | 3.2 / 2.7 / 3.0 | 3.0 | 91.8% | 8.2 mm |
| rgbd2:arm_1 | 12.7 / 13.0 / 14.1 | **13.0** | 97.2% | 10.0 mm |
| rgbd2:arm_2 | 6.2 / 5.8 / 6.2 | **6.2** | 97.5% | 7.1 mm |
| rgbd2:arm_3 | 2.4 / 2.3 / 2.2 | 2.3 | 99.8% | 9.5 mm |

Semua lengan yang terlihat memenuhi gerbang kualitas FIT A2 (jendela≥300, tersimpan≥30% dan≥200, residu≤15 mm). rgbd incumbent **belum lulus C3** karena arm_2 median5.3 cm>5 cm; bukan bukti atribusi bahwa ekstrinsik rgbd pasti salah (kedua kamera melihat arm_2 bergeser). C2 botol beberapa tinggi belum diuji. Klaim hanya rgbd2 salah belum didukung; kandidat rgbd tetap harus dinilai dengan uji-tahan independen.

Operator K4: area kosong boleh dipakai untuk kalibrasi, box dapat dipindahkan ke sana. Tidak ada instruksi pemindahan box ke jalur lengan. Proyeksi tinggi aktual: F4 lantai OUT rgbd2(v739), A310 IN(v691), B480 IN(v660); F7 semua tinggi IN(v708/659/627). F4 boleh menjadi FIT rgbd2 bila botol tunggal ter-track pada dua tinggi A/B, sesuai syarat A2/C1; lantai yang tidak ter-track dicatat, tidak dikarang. F7 lantai dekat batas, visibilitas botol penuh masih perlu diperiksa.

Swa-uji penyaring env PASS dan interarm LULUS sebagai pemeriksaan wajib pra-plan, bukan pengulangan A0 solver/TF/G61. OMPL plan-check HW tanpa --move sedang berjalan: F7 PLANNED sampel2 (sampel1 TORQUE-UNSAFE, rencana terpilih j2 92%), F1 PLANNED sampel1(j3 92%), F6 PLANNED sampel4(tiga TORQUE-UNSAFE, terpilih j2 93%), F4 PLANNED; F5/F3 menyusul. Semua rencana terpilih harus≤97%; penolakan lintasan sebelumnya tidak disembunyikan dan tidak dieksekusi.

Monitor CSV tumbuh setelah probe plan-only menerbitkan target pengamatan; ini galat terhadap target rencana yang tidak dieksekusi, BUKAN dwell SUCCESS/gerak nyata. Perekam sendi tetap tumbuh. Pemakaian box di enam tanda meja ditanyakan terpisah karena ukuran/tinggi aktual dapat bertabrakan dengan lengan REST atau melewati tepi meja; jawaban belum tiba. §A belum dikunci sampai kelayakan ini jelas; localizer/track botol belum dimulai. NOL gerak HW diperintah.

### B3. Pra-penguncian: plan-check final dan penyangga meja

Keenam kandidat F pada HW nyata PLANNED, tanpa --move. Bukti JSON/log per titik dan `plans_summary.json` di `hw_20261014/`.

| Tanda | xy (m) | sampel diterima | rencana sebelumnya | torsi terketat terpilih |
|---|---|---|---|---|
| F1 | (0.25, −0.50) | 1 | — | j3 92% |
| F6 | (0.40, −0.70) | 4 | 3× TORQUE-UNSAFE | j2 93% |
| F4 | (0.55, −0.60) | 2 | 1× TORQUE-UNSAFE | j3 91% |
| F5 | (0.65, −0.48) | 1 | — | j3 91% |
| F3 | (0.25, −0.60) | 1 | — | j3 92% |
| F7 | (0.65, −0.58) | 2 | 1× TORQUE-UNSAFE | j2 92% |

Rencana terpilih semuanya≤97%; (0.40,−0.50) tidak dipakai kembali. Retract plan-check se-gantry/antar-gantry/lingkungan CLEAR pada semuanya. Ini rencana dari REST, bukan validasi lintasan transit tanda→tanda; gerak nanti wajib disaring ulang dan izin operator per langkah. K0 sesudah plan-check LULUS (`k0_after_plans.json`), keadaan sama dengan awal; tidak ada perintah gerak. Audit pasif perekam snapshot (`passive_audit.json`): 1724352 baris, effort maksimum0.229 N·m t1_a1_joint_2, bukan torsi eksekusi rencana.

Operator menolak box31/48 cm di meja arm_1 karena terlalu besar/dekat lengan. Alternatif berbasis A0.12 (meja sebagai uji-tahan + FIT beberapa tinggi lantai) dipertimbangkan, belum diterapkan/dikunci; syarat tiap tanda FIT≥2 tinggi tidak diubah. Kemudian operator menawarkan box kecil10/14 cm. Tinggi terukur±2 mm, ukuran permukaan, kekokohan dan jarak bebas pada enam tanda meja masih ditanyakan. Penamaan usulan: T10/T14 khusus meja, A31/B48 khusus lantai. Botol putih tunggal H70/diameter60 mm dipakai bergantian; botol lain beda bentuk/warna tidak digunakan.

Jika kedua box kecil muat, draf pengukuran meja dapat kembali ke tiga tinggi0.770/0.870/0.910 m; lantai0/0.310/0.480 m. Angka ini BELUM terkunci sampai operator memastikan. D590 tetap dugaan harfiah, B=tinggi atas di zona meja sesuai protokol final; bila meja hanya satu tinggi, D590 dilaporkan tidak diuji dan tidak diberi skor palsu. Belum track botol, localizer per kamera, fit, penerapan kandidat, uji-tahan atau gerak penunjuk.

Stack milik sesi masih HIDUP (roots786857/787847, monitor787845→787875, perekam787846; `process_checkpoint.json`), tanpa proses probe/baseline tersisa. Log /tmp disalin sebagai snapshot; capture17 MB diarsip lokal dengan sha256 `capture_manifest.json`, tidak di-stage; CSV besar tidak di-commit. Pemeriksaan sesudah interupsi: proses inti/camera/monitor/perekam hidup, tidak ada REJECTED/TORQUE-ABORT baru. Segmentasi YOLOE log terakhir hanya “loading”; publisher endpoint ada, penerbitan gambar sedang diperiksa, belum dinyatakan siap.

| Dugaan | Status pada checkpoint |
|---|---|
| D590 | — belum diuji |
| D591 | — belum diuji |
| D592 | — belum diuji |
| D593 | — belum diuji |
| D594 | — belum diuji |
| D595 | — belum diuji |
| D596 | — belum diuji |
| D597 | — plan-check6/6 lulus; torsi gerak belum diuji, dugaan gabungan belum dapat diberi ✓/✗ |
| D598 | — belum diuji |

Belum penutupan sesi: REST/PARK terukur, stack tetap aktif menunggu penyangga meja/penguncian. Batas bukti: rgbd2 jauh jelas bergalat pada baseline, rgbd arm_2 juga melampaui5 cm; “hanya rgbd2 salah” belum terbukti dan atribusi kamera vs world belum selesai. Model kalibrasi M6 utama/M7 hanya lewat LOO FIT; model agen kuat effort high. Anggaran4.000 token per tugas CLAUDE.md terlampaui saat verifikasi HW/dokumentasi sesi ini; dicatat, pekerjaan tidak dilabel selesai.

**Pembaruan kesiapan segmentasi:** subscription read-only selama8 s menerima27 gambar per kamera,1280×720, cap waktu bertambah1791948926→1791948934 (`seg_readiness.json`). YOLOE terbit; log tanpa “ready” tidak membuktikan hang. Tidak restart kamera, tidak merekam track botol. Semua plan-check/baseline selesai; menunggu jawaban box kecil sebelum kunci§A.

### B4. Penguncian §A 2026-10-14

Operator mengonfirmasi box C100 mm (atas95×70 mm) dan D140 mm (atas92×50 mm), kokoh/rata serta muat di tanda meja; tinggi±2 mm dijawab “ya” pada pertanyaan gabungan. Box A310/B480 tetap lantai. Satu botol putih H70/diameter60 mm dipakai bergantian; botol lain tidak digunakan. K0 pra-kunci LULUS (`k0_before_lock.json`). Protokol final memakai18 penempatan meja dan18 lantai per set incumbent; peran FIT/uji-tahan tetap draf, F4 rgbd2 hanya LA/LB yang dalam bingkai. Tidak mengubah kriteria/bobot/model/ambang.

Salinan §A `results/p1_cam6dof/cam6dof_A_locked.md`, sha256 **3d13ea22a89d87bb3ac789bda902a6f2f489f9594d40b166d9426a14a78dc802** (`cam6dof_A_locked.sha256`). Konflik nama B draf terhadap box operator diselesaikan sebelum data: D590 menguji **box D tertinggi di meja** dengan predikat≥5/6 yang sama; bukan box B lantai. §A kini dikunci; data botol dan gerak penunjuk belum dimulai pada pembuatan hash. Localizer per kamera dimulai setelah commit kunci. Semua baseline REST dan enam plan-check HW selesai, tidak diulang.

### B5. Localizer per kamera siap; menunggu penempatan pertama

§A dikunci pada commit `d081044`; hash diperiksa ulang cocok sebelum localizer dimulai. `loc_rgbd` PID804824 dan `loc_rgbd2` PID804825 berjalan setelah commit. Discovery awal `node info /loc_rgbd` belum menemukan node, pembacaan ulang berhasil; bukan alasan memulai proses duplikat. `loc_rgbd{,2}_info.txt` memperlihatkan subscriber kamera masing-masing saja, semua output ke `/loc_rgbd{,2}/...`, masukan grasp_target terisolasi. Topik demo `/detected_objects/tracks` dan `/target_object` tidak ada (0 publisher); tracks terisolasi punya1 publisher per namespace. Jangan membunuh PID ini tanpa memverifikasi identitas ulang.

Plan konfigurasi `plan_rgbd_inc.json` dan `plan_rgbd2_inc.json` mengikuti peran/tinggi terkunci, F nominal menunggu TCP actual sebelum data lantai. Belum ada berkas track botol; penempatan pertama C3/T0 (tanpa box) diminta ke operator, menunggu konfirmasi posisi dan menjauh. C3 adalah UJI-TAHAN dan tidak masuk fit/LOO/model-selection. Tidak ada gerak HW atau perubahan launch default. Stack+monitor+perekam+camera+dua localizer hidup. Langkah berikut: operator siap C3/T0→rekam15 s kedua localizer→periksa track tunggal dan arsip, kemudian C3/TC.

### B6. Botol incumbent meja — penempatan1/18 C3/T0

Operator mengonfirmasi botol putih langsung di C3 dan siap rekam. Hash§A cocok sebelum capture; rekaman setelah commit d081044,15 s per kamera secara bersamaan, kedua rc0 (`tracks_inc/C3_T0_*`). rgbd26 pesan tanpa track (zona di luar bingkai), rgbd2 28 pesan dengan satu track white bottle tid0, conf median0.86; tidak ada saingan≤0.35 m. Median rgbd2=(2.0269,0.7546,0.8072) m, tanda=(1.8486,0.6786), dxy=(+0.1783,+0.0760), |dxy|=0.1938 m. p5–p95 xy6/4 mm. z expected model=0.8164 m, dz=-0.0092 m.

C3 adalah UJI-TAHAN: tidak masuk fit/inisialisasi/LOO/model-selection. Ini evaluasi incumbent, bukan uji hidup kandidat baru C2. Galat xy incumbent>5 cm diketahui; kalibrasi dilanjutkan sesuai tujuan, tanpa melonggarkan C2. D590 belum bisa dinilai sebelum TD/semua tanda. Nol gerak HW; stack+localizer milik sesi hidup. Penempatan2/18 C3/TC (box C100 mm) sudah diminta, menunggu operator siap.

### B7. Botol incumbent meja — penempatan2/18 C3/TC

Operator siap C3 di box C100 mm. Rekaman15 s per kamera, kedua rc0: rgbd28 pesan tanpa track; rgbd2 26 pesan, satu track tid0 berkelas bottle, label terakhir **blue bottle** (sebelumnya white), conf median0.68. Selektor terkunci `label contains bottle` + radius0.35 m tidak berubah; warna bukan dasar pergantian objek, botol fisik sesuai instruksi tetap botol putih yang sama. Tidak ada saingan. Median=(2.0206,0.7611,0.9101) m, |dxy|=0.1908 m; sebaran p5–p95 xy11/9 mm. Permukaan0.870, centroid model z=0.9164, dz=-0.0063 m. Bukti `tracks_inc/C3_TC_*`.

C3 tetap UJI-TAHAN, tidak masuk fit/LOO. Warna label berubah diungkapkan, bukan alasan mengganti data/model. NOL gerak HW, default launch tetap; stack/localizer hidup. Berikut C3/TD: box D140 mm mengganti C (tidak ditumpuk), menunggu operator siap.

Koreksi komunikasi C3/TC: commentary awal menyebut18.8 cm; hasil numerik `C3_TC_summary.json` adalah0.190807 m (**19.1 cm**). Data mentah/JSON tidak berubah, angka terhitung yang dipakai.

### B8. Botol incumbent meja — penempatan3/18 C3/TD, anomali z

Operator mengonfirmasi C3 pada box D140 mm, tidak ditumpuk. Rekaman15 s/kamera, kedua rc0: rgbd28 pesan tanpa track; rgbd2 29 pesan, track lokal tunggal white bottle tid70 n29, median(2.0226,0.7514,0.9082) m. |dxy|=0.1886 m, z model=0.9546 m, **dz=-0.0464 m**. P5–p95 xy24/22 mm, **z96 mm**, conf median0.312 (TC0.6755); empat track lain jauh dari tanda, tidak ada saingan≤0.35 m. Selector/model/bobot tidak berubah. Semua track mentah disimpan termasuk deteksi jauh.

Anomali: median z TC0.9101→TD0.9082 padahal permukaan naik40 mm. Deteksi tidak stabil, berbeda dari T0/TC. Citra RGB dan overlay terbit ditangkap sesudah rekaman (`C3_TD_rgb.png`, `C3_TD_debug.png`) untuk diagnosis penempatan/mask; gambar bukan frame persis setiap observasi. Belum dapat mengatribusi pada depth/extrinsik/model centroid vs mask penyangga/penempatan. Operator diminta mengonfirmasi botol tetap7 cm dan box D14 cm tanpa tumpukan, biarkan posisi dulu. Tidak mengubah parameter detektor/model dari titik UJI-TAHAN, tidak membuang data, tidak memilih ulang sampel yang lebih baik. C3 tetap tidak masuk FIT/LOO.

Penyampaian awal “sekitar−4.8 cm” dipertegas hasil model−4.64 cm; angka terhitung dipakai. Ini botol incumbent; belum C2 kandidat hidup. D590 belum dinilai sampai semua tanda/tinggi lengkap. NOL gerak HW, stack/localizer hidup; penempatan berikut C6/T0 menunggu diagnosis fisik C3/TD.

**Konfirmasi operator sesudah B8:** benar botol7 cm pada box D14 cm tanpa tumpukan. Operator menawarkan menumpuk C+D; tidak dipakai karena tinggi dan peran§A sudah terkunci. Anomali C3/TD tetap disimpan sebagai data incumbent uji-tahan, penyebab belum teratribusi; tidak mengubah mask/parameter/centroid dari hasil uji-tahan. Penempatan berikut4/18 C6/T0 diminta (box disingkirkan, botol langsung meja), menunggu siap; nol gerak HW, stack/localizer tetap aktif.

### B9. Rekap botol incumbent meja (diperbarui per penempatan)

18/18 penempatan selesai. Bukti mentah, metadata dan ringkasan: `hw_20261014/tracks_inc/<tanda>_<tinggi>_*`. Galat adalah incumbent, belum keputusan kandidat C2; C3/C6 tidak dipakai FIT/LOO. Selektor bottle dalam radius0.35 m tetap, saingan ditampilkan.

| Penempatan | peran | n | median xyz (m) | galat xy (cm) | dz model (cm) | p5–p95 z (mm) | conf | track |
|---|---|---|---|---|---|---|---|---|
| C3/T0 | UJI-TAHAN | 28 | (2.0269,0.7546,0.8072) | 19.38 | -0.92 | 3.6 | 0.859 | tunggal |
| C3/TC | UJI-TAHAN | 26 | (2.0206,0.7611,0.9101) | 19.08 | -0.63 | 7.2 | 0.675 | tunggal |
| C3/TD | UJI-TAHAN | 29 | (2.0226,0.7514,0.9082) | 18.86 | -4.64 | 96.3 | 0.312 | tunggal |
| C6/T0 | UJI-TAHAN | 27 | (2.0244,0.6570,0.8190) | 13.63 | +0.26 | 5.9 | 0.855 | tunggal |
| C6/TC | UJI-TAHAN | 31 | (2.0088,0.6438,0.9441) | 11.68 | +2.77 | 4.2 | 0.621 | tunggal |
| C6/TD | UJI-TAHAN | 25 | (2.0191,0.6583,0.9768) | 13.21 | +2.22 | 7.4 | 0.682 | tunggal |
| M1/T0 | FIT | 31 | (1.8610,0.6568,0.8290) | 15.47 | +1.08 | 2.2 | 0.844 | tunggal |
| M1/TC | FIT | 28 | (1.8889,0.6733,0.9160) | 18.64 | -0.04 | 4.9 | 0.881 | tunggal |
| M1/TD | FIT | 28 | (1.8916,0.6794,0.9572) | 19.25 | +0.08 | 33.3 | 0.639 | tunggal |
| C5/T0 | FIT | 30 | (1.8479,0.7471,0.8342) | 21.98 | +1.61 | 2.8 | 0.768 | tunggal |
| C5/TC | FIT | 31 | (1.8398,0.7489,0.8722) | 21.34 | -4.42 | 23.1 | 0.346 | tunggal |
| C5/TD | FIT | 25 | (1.8314,0.7462,0.9078) | 20.47 | -4.85 | 29.5 | 0.485 | tunggal |
| N1/T0 | FIT | 0 | — | — | — | — | — | tidak ada track |
| N1/TC | FIT | 31 | (2.0576,0.5528,0.9484) | 13.05 | +3.21 | 4.2 | 0.367 | tunggal |
| N1/TD | FIT | 8 | (2.0455,0.5381,0.9977) | 11.37 | +4.31 | 3.6 | 0.277 | saingan |
| N2/T0 | FIT | 31 | (2.1069,0.7504,0.8099) | 18.20 | -0.65 | 7.0 | 0.844 | tunggal |
| N2/TC | FIT | 30 | (2.1043,0.7226,0.9366) | 17.08 | +2.20 | 7.9 | 0.616 | tunggal |
| N2/TD | FIT | 30 | (2.1167,0.7465,0.9617) | 18.97 | +0.70 | 6.7 | 0.609 | tunggal |

Berikut: persiapan unting/penanda lantai dan pemeriksaan plan-only penjaga97%; belum izin gerak pertama F1. NOL gerak HW; stack/kamera/monitor/perekam/localizer per kamera masih hidup. Tidak mengganti data anomali atau tinggi/model terkunci.

### B10. Catatan anomali pada data FIT

C5/TC adalah FIT dan menunjukkan dz−4.42 cm, confidence0.346, sebaranz23.1 mm. Jadi anomali centroid/tinggi tidak terbatas pada C3 UJI-TAHAN; penyebab belum teratribusi. Data tetap, tidak ada perubahan selector, parameter detector, bobot/centroid/model atau pembuangan titik.

### B11. Syarat FIT N1 belum terpenuhi pada set terkunci

N1/T0 tidak ada track; N1/TC satu track n31 (dz+3.21 cm); N1/TD tiga ID kandidat lokal (750 n2,752 n7,753 n8), banyak pesan kosong, conf0.277 pada track terbanyak. Ringkasan memilih track terbanyak hanya untuk informasi mengikuti cfg_from_tracks, dan **mencetak saingan**, tidak menyatakan data tunggal/lulus C1. Detail urutan ada pada `N1_TD_ambiguity.json`; tidak menggabungkan ID, mengubah radius/threshold, mengganti FIT atau melakukan ulang sampai mendapat angka yang lebih baik. N1 hanya satu tinggi yang terkonfirmasi track tunggal pada selektor terkunci, sehingga syarat C1 per tanda FIT≥2 tinggi belum terpenuhi. Fit numerik kelak, bila dijalankan, harus dilabel informasi tidak sah untuk penerimaan rgbd2 selama masalah ini belum terselesaikan sesuai protokol. Kamera rgbd tetap dinilai dari set lantainya; belum menyimpulkan hanya rgbd2 salah.

Lanjut ke N2/T0 (penempatan16/18) untuk melengkapi data sebagaimana protokol; tidak ada gerak HW, stack/localizer hidup.

### B12. Penjaga margin rencana aktual — sebelum gerak pertama

Inspeksi kode menemukan `reach_dwell_probe._plan_and_screen` menolak torsi prediksi>100% rating, sedangkan CAM-6DOF terkunci mensyaratkan>97% TOLAK. Plan-check enam kandidat sudah91–93%, tetapi `--move` merencanakan ulang secara stokastik sehingga persentase rencana aktual bisa berbeda. Agar peraturan97% tetap ditegakkan sebelum eksekusi, ditambahkan wrapper khusus sesi `harness/probe_cam6dof.py`. Tidak mengubah probe bersama, model RNEA, offset7.7, batas torsi terukur14 atau saringan lain. Semua saringan asli dijalankan lebih dulu; rencana yang PLANNED diperiksa ulang pada trajectory yang sama dengan prediksi/offset/rating asli, nilai pecahan tidak dibulatkan. >0.97→TORQUE-UNSAFE sebelum eksekusi (mekanisme retry asli); prediksi tidak lengkap/nonfinite/negatif→TORQUE-UNSCREENED; penolakan asli tetap. Tidak mengirim gerak melalui pengujian.

Kontrol unit5/5 lulus (`margin_guard_unit.log`): rencana aman mempertahankan trajectory identik; persentase97.14 yang tercetak97% tetap ditolak; tepat di bawah97% diterima; ENV-COLLIDE asli tidak dihitung ulang/dilonggarkan; prediksi missing/NaN/negatif ditolak. `--help` wrapper berhasil dimuat tanpa ROS init/gerak (`margin_guard_help.log`). Plan-only wrapper hidup belum dijalankan, akan diperiksa sebelum izin gerak pertama. Pengujian ini tambahan untuk penegakan aturan sesi, bukan pengulangan solver/TF/self-test/mock A0 atau G61. §A dan hash tetap.

### B13. Meja18/18 selesai; persiapan gerak penunjuk #1

Seluruh18 penempatan meja direkam,17 punya median lokal rgbd2 dan N1/T0 kosong. N1/TD3 ID bergantian (max1 bersamaan,11/28 pesan kosong); ambiguitas identitas/centroid tetap dicatat, belum diklaim memenuhi C1 minimal2 tinggi tunggal untuk N1. Tidak menggabung ID atau mengganti FIT; kandidat rgbd2 belum sah untuk penerimaan. RGBD meja tidak terlihat; evaluasi RGBD lantai belum dilakukan. Bukti `table_inc_gate.json`.

**D590 ✗ harfiah:** box D mengurangi galatxy pada C3/C6/C5 (3 pasangan), tetapi memperbesar pada M1 dan N2 (2 pasangan); N1 tanpa pasangan T0. Karena dua dari enam sudah gagal, bahkan bila N1 memenuhi, maksimum4/6<5/6. Jadi dugaan ditolak dengan batas missing diungkapkan; tidak menyebut3/5 seolah≥5/6. Uji-tahan tidak dipakai memilih model. D591–D598 belum diuji penuh.

K0 pra-penunjuk LULUS (`k0_before_pointer.*`); arm_3REST0.455°, g1 1.499262/g2 0.000901,rot0/0. Integrasi wrapper margin pada F1 plan-only rc0: dua lintasan awal>100% ditolak oleh saringan asli, lintasan ketiga PLANNED, **j3 91.475802%** (nilai tidak dibulatkan); interarm min426.3 mm, env115.3 mm, retract se424.1/antar860.9/env115.3 mm CLEAR. Kontrol hidup ini menunjukkan penjaga dipanggil pada rencana yang sama sebelum eksekusi, bukan uji negatif97–100% hidup; kontrol negatif bagian itu hanya unit. Belum gerak saat plan-only.

Operator mengonfirmasi unting/penanda siap dan area bebas; kemudian **secara eksplisit mengizinkan GERAK HW #1: arm_3 REST→F1=(0.25,−0.50,1.22) tool-bawah saja**. Ini tidak mengizinkan titik berikut/pulang. Perintah aktual memakai `harness/probe_cam6dof.py --arms arm_3 --target 0.25,-0.50,1.22 --approach0 --trials1 --tau-max14 --plan-attempts8 --monitor-csv /tmp/cam6dof_20261014_mon_samples.csv --move`; semua rencana baru disaring ulang termasuk batas97%. Status sebelum command: gerak belum dieksekusi, stack/monitor/perekam/localizer aktif; selanjutnya verifikasi MOVED/SUCCESS/torsi/TCP sebelum meminta tanda atau langkah berikut.

### B14. GERAK HW #1 arm_3 REST→F1 — MOVED/SUCCESS

Izin operator eksplisit F1 saja, diikuti satu command melalui supervisor mandiri tanpa gerak berikut otomatis. Bukti `move_F1_job.json` (PID probe828523, supervisor828522; keduanya selesai, bukan target kill sesi baru), `move_F1_exit.json` rc0, `move_F1.log/.json`. Rencana aktual PLANNED sampel1, j3 **91.515705%**, interarm418.5 mm/env115.3 mm/retractCLEAR; penjaga97% berjalan sebelum eksekusi. Gerak **MOVED, dwell SUCCESS**; monitor tahan2 s, settled1.25 mm, ori maks1.87°,42 sampel@20 Hz. Torsi puncak probe **5.615 N·m**.

TCP terukur3 s,n87: **(0.2512,−0.4998,1.2204) m**, std cetak0/0/0 mm, galat terhadap perintah **1.281 mm** (`tcp_F1.log`, `mark_F1.json`). Operator mengonfirmasi F1 sudah ditandai dan unting aman. xy aktualF1=(0.2512,−0.4998) masuk planRGBD hanya sebagai pengukuran ground truth sebelum botol lantai; peran FIT/tinggi tidak berubah, tidak memakai galat kamera untuk mengganti titik.

Audit perekam di seluruh jendela job35 s (`move_F1_recorder_audit.json`):97580 baris,jendela tertutup≤10 ms,puncak5.6153 N·m j3; rel/rotasi kedua gantry rentang **0**, lengan lain maks rentang0.008709°. Tidak ditemukan REJECTED/FAULT/TORQUE-ABORT baru pada log launch; NOT armed table1 hanya catatan inisialisasi sebelumARMED. Tidak ada gerak tak diminta terukur. C6 untuk langkah1 lulus; D597 belum skor final karena langkah lainnya/pulang belum.

**Keadaan kini:** arm_3 menahan F1 (BUKAN REST), arm_1/2/4 REST,g1 1.499262/g2 0.000901,rot0/0. Stack/monitor/perekam/kamera/localizer masih hidup. Belum izin F6 atau pulang. Berikut plan-only F6 dari keadaan aktualF1, lalu izin satu langkah bila semua guard lulus.

### B15. Plan-only F1→F6 lulus; menunggu izin gerak #2

Dari arm_3 di F1, wrapper97% plan-only F6=(0.40,−0.70,1.22) tool-bawah menghasilkan DRY/PLANNED percobaan1, j3 **91.530809%**. Interarm minimum500.3 mm, lingkungan115.3 mm; retract se410.4/antar860.9/env115.3 mm CLEAR. Bukti `from_F1_to_F6_plan.json/.log`. Tidak ada --move atau eksekusi. Plan-only dapat menerbitkan target monitor F6; galat monitor ke target itu bukan bukti gerak.

Arm_3 tetap F1, tanda F1 dikonfirmasi dan unting aman; lengan lain REST, gantry tetap. Belum izin F6/pulang. Berikut meminta izin **satu langkah F1→F6 saja**; bila disetujui, rencana aktual disaring ulang dengan batas97%, kemudian verifikasi MOVED/SUCCESS/torsi/TCP.

**Izin operator gerak #2 diterima:** “Izinkan arm_3 F1→F6 saja”. Hanya F6, bukan titik berikut/pulang. Rencana aktual akan disaring ulang dan dieksekusi satu kali; keadaan sebelum eksekusi arm_3 F1.

### B16. GERAK HW #2 arm_3 F1→F6 — MOVED/SUCCESS

Izin operator “Izinkan arm_3 F1→F6 saja”. Satu command, supervisor832919/probe832920 kini selesai rc0 (`move_F6_job/exit.json`, `move_F6.log/.json`). Rencana aktual pertama j2 114% ditolak sebelum eksekusi; kedua PLANNED, **j2 92.529263%** melalui penjaga97%. Ini penolakan lintasan oleh saringan, bukan REJECTED controller. Interarm489.5 mm/env115.3 mm, retract se623.1/antar731.3/env115.3 mm CLEAR. MOVED, dwell SUCCESS:2 s,42 sampel@20 Hz, pos maks4.88 mm/settled0.59 mm, ori maks1.39°. Torsi puncak probe **12.208 N·m** (<14; juga≤13 dugaan D597).

TCP3 s n87 **(0.3998,−0.6998,1.2197) m**, std cetak0 mm, galat **0.412 mm** (`tcp_F6.log`, `mark_F6.json`). Audit perekam seluruh36.2 s job:101360 baris, cakupan≤9 ms, puncak12.2078 N·m j2; rentang rel/rotasi kedua gantry0, lengan lain maksimum0.006303°. Semua effort lengan finite. Effort rel/rotasi NaN pada3620 sampel per sendi (kanal tidak melaporkan torsi); ini diungkapkan, tidak dianggap torsi nol. Posisi kanal tersebut finite dan diam. Audit awal terlalu ketat menolak NaN kanal rel/rotasi; diperbaiki untuk mencatatnya, tetap menolak posisi/nonfinite effort lengan, tanpa mengubah saringan/hasil gerak. Tidak ada REJECTED/FAULT/TORQUE-ABORT baru dalam log jendela yang diperiksa; controller SUCCEEDED.

Arm_3 kini menahan F6 (BUKAN REST), lengan lainREST, gantry tetap. C6 langkah2 lulus; D597 belum final. Tanda F6 belum dikonfirmasi; xy belum dimasukkan ke plan sampai konfirmasi. Belum izin F4 atau pulang. Berikut operator membuat label unting F6 dan mengamankan tali, lalu plan-only F4 sebelum izin langkah3. Stack/kamera/monitor/perekam/localizer tetap hidup; §A/hash tetap.

### B17. F6 ditandai di box tetap7 cm — penyimpangan fisik dari tinggi terkunci

Operator melaporkan: “sudah ditandai, tetapi tandanya7 cm dari lantai karena ada box besar tpi pendek setinggi7 cm yang tidak bisa saya pindah”. Tanda unting F6 berada pada permukaan box tetap, **bukan lantai z0**. xy tetap TCP terukur (0.3998,−0.6998); beda tinggi permukaan tidak mengubah proyeksi vertikal unting selama penandaan benar. Konfirmasi label permukaan dicatat pada `mark_F6.json`, actualxy planRGBD diperbarui sebelum botol; peran FIT/tinggi tidak diubah. Tinggi7 cm adalah laporan operator, ketelitian±2 mm belum dikonfirmasi dan tidak dimasukkan ke model sebagai pengukuran sah.

**Konflik dengan K4 awal “area kosong” dan §A1/A2:** F6/L0 terkunci z0 tidak tersedia pada keadaan fisik ini, dicatat missing; botol di atas box7 cm tidak boleh diberi z0. Tidak menumpuk A/B pada box itu dan mengaku z0.310/0.480. §A/salinan/hash tetap. F6/LA dan LB masih belum diketahui kelayakannya: harus A/B langsung lantai, kokoh, botol tepat xyF6, permukaan world tetap0.310/0.480. Bila dua tinggi terkunci tidak bisa diperoleh, C1 tandaFITF6 gagal dan fit rgbd hanya informasi sampai protokol tersendiri disiapkan; tidak mengganti tanda/FIT atau melonggarkan kriteria. Belum data botol lantai atau fit/modelselection.

Arm_3 tetap F6, lengan lainREST; belum gerak F4/pulang. Jawaban ini belum eksplisit mengonfirmasi tali diamankan/lengan tidak tersentuh; diminta klarifikasi e-stop+unting, apakah box sudah sejak awal, serta apakah box menghalangi F4/F5/F3/F7 dan A/B langsung lantai diF6. **Gerak berikut ditahan hingga keadaan sel/jalur jelas**, lalu plan-check dari pose aktual. Tidak ada perubahan stack/parameter atau perintah gerak.

### B18. F6 dua tinggi terkunci tetap tersedia; persiapan F4

Operator mengonfirmasi unting aman/lengan tidak tersentuh/e-stop, box pendek sudah sejak awal (bukan objek baru setelah peta/capture). A/B bisa kokoh langsung lantai, bergantian, botol tepat di atas xyF6, sehingga **LA0.310/LB0.480 tetap tersedia**; L0z0 tetap missing. Konfirmasi fisik belum data botol: syarat C1 minimaldua tinggi pada F6 kelak masih harus dibuktikan track tunggal. Operator menanyakan penggantian A/B ke C/D atau tumpukan C+D; tidak diterapkan pada set ini karena §A tinggi/peran sudah dikunci. C/D tetap khusus meja100/140 mm, tanpa tumpuk; A/B lantai310/480 mm. Tidak mengarang z box7 cm atau mengubah hash.

Keadaan arm_3 tetapF6, lengan lainREST, gantry tetap; belum izin F4/pulang. Klarifikasi keamanan F6 selesai. Status permukaan F4/F5/F3/F7 belum dijawab eksplisit dan harus dicatat saat konfirmasi penandaan; tidak menganggapnya semua lantai z0. Berikut **plan-only F6→F4=(0.55,−0.60,1.22)**, semua saringan asli + margin97%, lalu izin satu langkah bila lulus. Tidak ada eksekusi otomatis.

### B19. Plan-check F6→F4 lulus; menunggu izin gerak #3

Perintah DRY awal lupa arg `--plan-check`, menghasilkan DRY tanpa `plans` atau saringan (`from_F6_to_F4_plan.json/.log`). Tidak diklaim PLANNED/lulus dan tidak ada gerak. Dikoreksi menjadi command terpisah dengan `--plan-check` (bukti awal dipertahankan): `from_F6_to_F4_screened_plan.json/.log`, rc0 **DRY/PLANNED percobaan1**, wrapper j2 **92.543450%**≤97. Interarm666.9 mm/env115.3 mm, retract se623.1/antar710.3/env115.3 mm semua CLEAR. Ini plan-check dari pose aktualF6, bukan mengulang enam kandidat dariREST/A0. Tidak ada --move atau eksekusi; monitor targetF4 bukan bukti lengan pindah.

Arm_3 tetap F6, unting aman, box pendek sejak awal; A/B langsung lantai pada F6 tetap tersedia, L0missing. Tinggi/peran/hash tetap. Belum izin F4/pulang; berikut meminta **GERAK #3 arm_3 F6→F4=(0.55,−0.60,1.22) m tool-bawah saja**, rencana aktual akan disaring ulang. Permukaan tandaF4 harus dikonfirmasi saat penandaan (lantai atau box tetap), tidak diasumsikan z0.

**Izin gerak #3 diterima:** “Izinkan arm_3 F6→F4 saja”. Hanya arm_3 keF4, bukan titik berikut/pulang. Sebelum eksekusi arm_3F6; command --move akan merencanakan ulang dan menyaring semua lintasan dengan batas97% dan abort torsi14.

### B20. GERAK HW #3 arm_3 F6→F4 — MOVED/SUCCESS

Izin operator “Izinkan arm_3 F6→F4 saja”. Satu command, supervisor839261/probe839265 selesai rc0 (`move_F4_job/exit.json`, `move_F4.log/.json`). Rencana aktual PLANNED sampel1, j2 **92.543622%**≤97 melalui penjaga; interarm666.9 mm/env115.3 mm, retract se623.1/antar738.6/env115.3 mm CLEAR. MOVED, dwellSUCCESS2 s,41 sampel@20 Hz,pos maks4.02 mm/settled1.25 mm,ori maks1.72°. Torsi puncak **9.826 N·m**,≤14 dan≤13.

TCP3 s,n87 **(0.5491,−0.6003,1.2206) m**, std cetak0 mm, galat **1.122 mm** (`tcp_F4.log`, `mark_F4.json`). Audit seluruh27.85 s job (`move_F4_recorder_audit.json`):77980 baris,cakupan≤6 ms,puncak9.8258 N·m j2; rentang rel/rotasi0,lengan lain maks0.004469°. Seluruh effort lengan finite; effort rel/rotasi NaN2785 per sendi dicatat sebagai kanal tidak tersedia, bukan torsi0. Log jendela tidak menunjukkan FAULT/REJECTED/TORQUE-ABORT/NOTarmed baru; controllerSUCCEEDED. C6 langkah3 lulus; D597 belum final.

**Keadaan:** arm_3 menahanF4 (BUKAN REST),lengan lainREST,gantry tetap. TandaF4 belum dikonfirmasi; actualxy belum dimasukkan ke plan sampai operator membuat tanda. Permukaan tanda harus dinyatakan lantai z0 atau penyangga tetap; tidak mengulang asumsiF6. Belum izin F5/pulang. Berikut operator tandauntingF4, konfirmasi permukaan dan untingaman; lalu plan-checkF5 dariF4 sebelum izin satu langkah. Stack/kamera/monitor/perekam/localizer hidup,§A/hash tetap.

### B21. Operator meminta mengganti penyangga lantai — amendemen belum dikunci

Operator menjelaskan A/B terlalu besar/sulit dipindahkan dan lebih mudah C/D atau tumpukan keduanya. **Instruksi terbaru operator mengganti pilihan penyangga** yang sebelumnya ditetapkan A/B; jawaban sebelumnya mempertahankan A/B kini tidak menjadi kewajiban penempatan fisik. Amendemen desain pengukuran lantai akan disiapkan transparan, bukan mengubah §A lama diam-diam. Ini perubahan tinggi input atas kendala operator sebelum data lantai, bukan pelonggaran kriteria atau pemilihan desain dari galat botol lantai. Belum ada capture botol F di `tracks_inc`; data meja18/18, baseline, peranFIT/UJI-TAHAN, dugaan D590✗ dan anomali sebelumnya tetap dipertahankan.

§A awal commitd081044/hash3d13ea22… masih utuh. Amendemen nanti harus mencatat konflik terhadap tinggi A1/A2 lama, semua tinggi world aktual/alas penyangga, role tetap, dan dikunci dengan salinan+hash+commit **sebelum botol lantai pertama**. Karena gerak penunjuk1–3/data meja sudah terjadi di protokol lama, tidak boleh menyatakan seluruh sesi sejak awal mengikuti amendemen atau bahwa kunci baru mendahului semua data/gerak. Penerimaan bila kelak diuji dilabel menurut protokol amendemen; kepatuhan tinggi protokol awal diungkapkan tidak lengkap. Tidak memulai ulang G61/A0/baseline/data meja.

C100/D140 mm individual sudah diukur; **tinggi/stabilitas/urutan tumpukan C+D belum diketahui**, tidak menganggap nominal100+140 sebagai pengukuran sah. Acuan F6 bila penyangga kecil di atas box tetap7 cm harus memakai tinggi world aktual yang terukur, bukan mengaku alasz0; status langsung lantai vs di atas box masih diminta. Pengurangan rentang tinggi dariA/B dapat mengurangi informasi pemisahan skala/pose; tidak menjanjikan fit akan lolos. M6 utama/M7 lewat LOOFIT, batasrot5°/pos0.15/skala0.06, bobot/centroid/plainLS dan C1–C6 semuanya tetap, holdout tidak dipakai memilih model. Rincian pending `floor_support_amendment_pending.json`; **belum amendemen final atau data lantai**.

Arm_3 tetapF4 (BUKAN REST),lengan lainREST,gantry tetap; belum izinF5/pulang. Konfirmasi tandaF4/permukaan/untingaman masih diminta bersama fakta C/D/tumpukan/F6. Jangan menaruh atau menumpuk box dekat lengan saat penunjuk belum selesaiREST. Stack/perekam/monitor/localizer tetap hidup.

### B22. F4/F6 sama-sama di penyangga tetap7 cm; ukuran amendemen belum lengkap

Jawaban operator menegaskan tandaF4/F6 berada pada **penyangga tetap yang sama tinggi7 cm**, kuat menahan C/D. Ini mengoreksi interpretasi sebelumnya bahwa penyangga bisa langsung lantai pada F6: untuk penyangga kecil sekarang alasnya penyangga tetap, bukan z0. Konfirmasi lama A/B langsung lantai tidak dipakai sebagai bukti untuk C/D. TandaF4 sudah dibuat; actualxyF4=(0.5491,−0.6003) diperbarui pada kedua plan kamera hanya dariTCP penunjuk, role tetap FIT. TandaF6=(0.3998,−0.6998), roleFITRGBD tetap. Bukti `mark_F4/F6.json` dan pendingamendment.

Tinggi7 cm masih **laporan, belum ketelitian±2 mm terkonfirmasi**. Bila benar70 mm, permukaan C/D di atasnya akan bertambah terhadap lantai; belum dimasukkan sebagai angka metrologi sah dan tidak merekam botol dengan z0/100/140 yang salah. Tinggi aktual/urutan/stabilitas tumpukan C+D belum dijawab; tidak mengarang240 mm atau menganggap kuat menahan beban berarti tumpukan stabil dengan botol. Klarifikasi diminta terpisah: penyangga70 mm±2, tinggi/urutan/stabilitas tumpukan, serta untingF4 dilepas/lengan tidak tersentuh/e-stop.

§A awal/hash tetaputuh, amendemen belum dikunci, belum data botol lantai/fit. C1–C6, model/bobot/LOO/role dan hasil meja tetap. Arm_3 tetapF4, lengan lainREST,gantry tetap; belum izinF5/pulang. **Gerak berikut dan penempatan box dekat lengan belum dilakukan**; menunggu keamananF4 sebelum persiapan langkah berikut.

### B23. Tinggi alas/tumpukan diterima; draf tiga tingkat kecil dan persiapan F5

Operator mengonfirmasi penyangga tetapF4/F6 **70 mm±2 mm**, tumpukanC+D **245 mm** (jawaban terhadap permintaan ukur±2 mm), stabil/rata dengan botol; untingF4 aman,lengan tidak tersentuh,e-stop dijaga. **245 bukan nominal100+140=240**; angka terukur245 yang dipakai. Urutan box bawah belum disebutkan; tidak mengarang, harus konfirmasi susunan terukur yang sama sebelum penempatan tumpukan pertama. Pengukuran dapat dilakukan jauh dari lengan, bukan instruksi memasang box pada jalur penunjuk.

Pilihan draf lantai: **BASE (tanpa box kecil), D140, CD245**, tiga tingkat dengan rentang terbesar dari penyangga kecil yang tersedia; C100 hanya dipakai dalam tumpukan, tidak menambah tingkat keempat. UntukF4/F6 yang alasz0.070, permukaan world **0.070/0.210/0.315 m**; F1 tanda lantai telah dikonfirmasi sehingga draf0/0.140/0.245. Tinggi dasarF5/F3/F7 belum diketahui: jangan mengasumsikannya0 atau0.070, ukur/konfirmasi pada penandaan sebelum kunci amendemen. RoleFIT/UJI-TAHAN tetap; F4RGBD2 hanya dua tingkat terangkat bila terlihat dan menghasilkan tracktunggal, BASE tidak ditambah keFIT RGBD2 dari pengamatan baru. Semua kriteria/bobot/model/LOO dan data lama tetap. Draf `cam6dof_A_floor_amendment_draft.md`; **belum dikunci**, belum data botolF.

Arm_3 tetapF4,lengan lainREST,gantry tetap; keamananunting/e-stop telahterkonfirmasi. Berikut plan-checkF4→F5=(0.65,−0.48,1.22) tool-bawah, tanpaeksekusi; lalu izin satu langkah jika lulus97% dan saringanlain. Belum izinF5/pulang. §A awal/hash utuh; amendemen akan dikunci sebelum botol lantai setelah semuaalas diketahui/proyeksi diperiksa.

### B24. Plan-check F4→F5 lulus; menunggu izin gerak #4

Plan-only dengan --plan-check tanpa --move,rc0 (`from_F4_to_F5_plan.json/.log`). Lintasan1 j2 116% ditolakTORQUE-UNSAFE sebelumeksekusi; lintasan2 **PLANNED**, penjaga97% j3 **91.179055%**. Interarm693.4 mm/env115.3 mm, retractse623.1/antar721.5/env115.3 mm CLEAR. Penyaring/offset/modeltorsi tetap, tidak ada gerak saat plan-check. Monitor dapat menargetF5, bukan bukti gerak.

Arm_3 tetapF4,untingaman,e-stopdijaga; belum izinF5/pulang. Berikut meminta **GERAK #4 F4→F5=(0.65,−0.48,1.22) tool-bawah saja**; rencana aktual akan disaring ulang. Draf amendemen penyangga tersimpan281fc99,belumdikunci; alasF5/F3/F7 akan dikonfirmasi saat penandaan sebelum datafloor.

**Izin gerak #4 diterima:** “Izinkan arm_3 F4→F5 saja”, diikuti “lanjutkan”. Izin hanya F5; tidak mencakup F3/F7/pulang. Sebelum eksekusi arm_3F4,untingaman,monitor/perekam hidup; rencana aktual disaring ulang melalui wrapper97%, torsiukur batas14.

### B25. GERAK HW #4 arm_3 F4→F5 — MOVED/SUCCESS

Izin operator “Izinkan arm_3 F4→F5 saja”, lalu “lanjutkan”; satu command, supervisor869361/probe869362 selesai rc0 (`move_F5_job/exit.json`, `move_F5.log/.json`). Rencana aktual PLANNED sampel1, penjaga j3 **91.143720%**≤97. Interarm693.4 mm/env115.3 mm, retractse623.1/antar698.5/env115.3 mm CLEAR. GerakMOVED,dwellSUCCESS2 s,41 sampel@20 Hz,posmaks3.49 mm/settled2.64 mm,orimaks1.20°. Torsi puncak probe **9.643 N·m**,≤14 dan≤13.

TCP3 s,n87 **(0.6497,−0.4779,1.2185) m**,std cetak0 mm,galat **2.598 mm** (`tcp_F5.log`, `mark_F5.json`). Audit seluruh23.97 s job:67116baris,cakupan≤10 ms,puncak9.6425 N·m j2; rentang rel/rot0,lengan lainmaks0.004870°. Effortlenganfinite; kanalrel/rot effortNaN2397 per sendi diungkapkan, bukan torsi0. Loglaunch sejak gerakF4selesai hingga sesudahF5 (mencakup jeda operator) tidak menunjukkan FAULT/REJECTED/TORQUE-ABORT/NOTarmed baru (`health_before_after_F5.json`); pencarian awal substringFAULT mengenai nama logger `default_capabilities`, dikoreksi menjadi batas-kata pada pesan supaya kejadian sebenarnya dibedakan. ControllerSUCCEEDED; ini verifikasi log, bukan asumsi dari rc0. C6 langkah4lulus; D597belumfinal.

**Keadaan:** arm_3 menahanF5 (BUKAN REST),lenganlainREST,gantrytetap. LabelF5 dan permukaan dasar belumdikonfirmasi; actualxy belum dimasukkan ke plan sampai label dibuat. Belum izinF3/F7/pulang. Berikut operator tandauntingF5,tinggi dasar terhadaplantai(±2 mm bila penyangga),untingaman/lengantidakter sentuh; lalu plan-checkF3 dariF5 sebelum izin satu langkah. Draf amendemenBASE/D140/CD245 tetapbelumdikunci; alasF5/F3/F7 dan urutanstack belumlengkap,belum data botollantai/fit. Stack/kamera/monitor/perekam/localizer hidup. CSVbesar lokal sekarang>2.5GB;tidak di-commit.

### B26. F5 ditandai di lantai; persiapan F3

Operator menjawab “F5 di Z=0 aman sekanag” terhadap pertanyaan tanda/unting/lengan/permukaan. F5 dicatat ditandai langsung di lantai z0 dan unting aman; actual xy=(0.6497,−0.4779) dimasukkan ke kedua plan kamera hanya dari TCP terukur sebelum data botol, role FIT tetap. Draf BASE/D140/CD245 pada F5 menjadi permukaan world0/0.140/0.245 m. Alas F3/F7 dan urutan susunan tumpukan belum lengkap; amendemen tetap DRAF, belum data botol lantai atau fit. §A/hash awal tidak diubah.

Arm_3 menahan F5, lengan lain REST, gantry tetap; belum izin F3/F7/pulang. Berikut plan-check **F5→F3=(0.25,−0.60,1.22) m tool-bawah**, tanpa --move; bila semua guard lulus≤97%, minta izin satu langkah. F3 tetap UJI-TAHAN rgbd; tidak dipakai untuk fit/inisialisasi/LOO/model selection.

### B27. Plan-check F5→F3 lulus; menunggu izin gerak #5

Plan-only --plan-check tanpa --move selesai rc0 (`from_F5_to_F3_plan.json/.log`): **DRY/PLANNED percobaan1**, penjaga97% j3 **91.206818%**. Interarm508.3 mm/env115.3 mm; retract se508.8/antar759.0/env115.3 mm seluruhnya CLEAR. Semua saringan tetap. Tidak ada eksekusi, target monitorF3 bukan bukti gerak.

Arm_3 tetapF5, tanda lantai z0/untingaman dikonfirmasi. Belum izinF3/F7/pulang; berikut meminta **GERAK #5 F5→F3=(0.25,−0.60,1.22) tool-bawah saja**. Rencana aktual akan disaring ulang≤97%; kemudian verifikasi MOVED/SUCCESS/TCP≤5 mm/torsi≤14 sebelum label dan tinggi dasarF3. F3 tetap UJI-TAHAN rgbd, tidak masukfit/modelchoice. Amendemen penyangga belumdikunci dan belum data botollantai.

**Izin gerak #5 diterima:** “Izinkan arm_3 F5→F3 saja”. Izin hanya F3, tidak mencakup F7/pulang. Pemeriksaan log/TCP pasif sebelum gerak tersimpan `health_before_F3.json`/`tcp_before_F3.log`; rencana aktual disaring ulang97% dan batas torsiukur14.

### B28. GERAK HW #5 arm_3 F5→F3 — MOVED/SUCCESS

Izin operator “Izinkan arm_3 F5→F3 saja”; satu command, supervisor874017/probe874018 selesai rc0 (`move_F3_job/exit.json`, `move_F3.log/.json`). Tiga lintasan awal TORQUE-UNSAFE ditolak sebelum eksekusi; lintasan4 PLANNED, penjaga97% j3 **91.176809%**. Interarm517.8 mm/env115.3 mm; retract se518.3/antar757.2/env115.3 mm CLEAR. GerakMOVED,dwellSUCCESS2 s,41 sampel@20 Hz,posmaks4.23 mm/settled2.54 mm,orimaks0.52°. Torsi puncak **10.673 N·m**,≤14 dan≤13.

TCP3 s,n91 **(0.2513,−0.5991,1.2185) m**,std0.12/0.18/0.22 mm,galat **2.179 mm** (`tcp_F3.log`, `mark_F3.json`). Audit seluruh30.45 s job:85260baris,cakupan≤7 ms,puncak10.6732 N·m j2; rentang rel/rot0,lengan lainmaks0.004870°. Effort lengan finite; rel/rot effortNaN3045 per sendi dicatat kanal tidak tersedia. Log eksekusi menunjukkan SUCCEEDED, tidak ada FAULT/REJECTED/TORQUE-ABORT/NOTarmed pada jendela gerak (`health_after_F3.json`); sebelum commandTCP masihF5galat2.449 mm dan tidak ada fault baru (`health_before_F3.json`). C6 langkah5lulus; D597belumfinal sampaiF7/pulang dan data lengkap.

**Keadaan:** arm_3 menahanF3 (BUKAN REST),lengan lainREST,gantrytetap. Tanda/tinggi dasarF3 belum dikonfirmasi, actualxy belum dimasukkan ke plan sampai operator label; F3 tetap UJI-TAHAN rgbd, tidak dipakai fit/LOO. Belum izinF7/pulang. Berikut operator tandauntingF3,konfirmasi permukaan/tinggi(±2 mm bila penyangga),untingaman; lalu plan-checkF7 dariF3 sebelum izin satu langkah. Draf amendemenBASE/D140/CD245 belumdikunci,alasF3/F7 dan susunanstack belumlengkap; belum botollantai/fit. Stack/kamera/localizer/monitor/perekam hidup.

### B29. F3 ditandai pada penyangga70 mm; persiapan F7

Operator: “F3 tandanya di penyangga7 cm sama sepertiF4/F6. sudah saya tandai”. TandaF3 dikonfirmasi, alasnya penyangga yang sama **0.070 m±0.002** yang sudah diukur padaB23; actualxy=(0.2513,−0.5991) dimasukkan ke planRGBD hanya dariTCP penunjuk. F3 tetap **UJI-TAHAN**, tidak digunakan fit/inisialisasi/LOO/model-choice. DrafBASE/D140/CD245 menjadi permukaanF3 world0.070/0.210/0.315; bukan z0. Data botol belum dimulai; amendemen belumdikunci, alasF7 dan urutanstack belumlengkap. §A/hash awal tetap.

Jawaban tidak secara eksplisit menyebut tali diamankan/lengan tidak tersentuh; konfirmasi itu digabungkan dalam permintaan izin gerak berikut, tidak dianggap sudah dijawab. Arm_3 tetapF3, lenganlainREST,gantrytetap; belum izinF7/pulang. Berikut plan-check **F3→F7=(0.65,−0.58,1.22) tool-bawah** tanpaeksekusi. Bila seluruh guard lulus≤97%, minta izin satu langkah disertai konfirmasi e-stop/unting/area aman.

### B30. Plan-check F3→F7 lulus; menunggu izin gerak #6

Plan-only --plan-check tanpa --move rc0 (`from_F3_to_F7_plan.json/.log`): lintasan1 TORQUE-UNSAFE ditolak sebelum eksekusi; lintasan2 **DRY/PLANNED**, penjaga97% j2 **91.517370%**. Interarm517.8 mm/env115.3 mm; retractse623.1/antar697.9/env115.3 mm seluruhnya CLEAR. Rencana baru harus tetap disaring ulang sebelum eksekusi. Tidak ada gerak saat plan-check, monitor targetF7 bukan bukti gerak.

Arm_3 tetapF3, tanda pada penyangga70 mm dikonfirmasi; keamanan tali setelah penandaan belum eksplisit dijawab dan diminta dalam izin berikut. Belum izinF7/pulang. Berikut meminta **GERAK #6 F3→F7=(0.65,−0.58,1.22) tool-bawah saja**, disertai konfirmasi unting diamankan/lengan tidak tersentuh/area bebas/e-stop dijaga. F7 tetap UJI-TAHAN kedua kamera+kesepakatan, tidak masukfit/LOO/modelchoice. AlasF7/susunanstack masih pending, amendemen belumdikunci,belum botollantai.

**Izin gerak #6 diterima:** “Izinkan F3→F7 saja; unting/area aman, e-stop dijaga”. Keamananunting setelah labelF3 kini eksplisit terkonfirmasi. Hanya F7, bukan pulang. Pemeriksaan log/TCP pasif sebelumgerak tersimpan `health_before_F7.json`/`tcp_before_F7.log`; rencana aktual disaring ulang97% dan torsiukur batas14.

### B31. GERAK HW #6 arm_3 F3→F7 — MOVED/SUCCESS; enam tanda penunjuk selesai

Izin “IzinkanF3→F7 saja; unting/areaaman,e-stopdijaga”; satu command, supervisor878199/probe878200 selesai rc0 (`move_F7_job/exit.json`, `move_F7.log/.json`). Dua lintasan TORQUE-UNSAFE ditolak sebelum eksekusi; lintasan3 PLANNED, penjaga97% j2 **92.653601%**. Interarm517.8 mm/env115.3 mm; retractse623.1/antar704.2/env115.3 mm CLEAR. GerakMOVED,dwellSUCCESS2 s,42 sampel@20 Hz,posmaks2.84 mm/settled2.40 mm,orimaks1.82°. Torsi puncak probe **12.483 N·m**,≤14 dan≤13.

TCP3 s,n86 **(0.6483,−0.5809,1.2186) m**,stdcet ak0 mm,galat **2.379 mm** (`tcp_F7.log`, `mark_F7.json`). Audit seluruh29.17 s job:81676baris,cakupan≤8 ms,puncak12.4826 N·m j2; rentang rel/rot0,lenganlainmaks0.004641°. Effortlenganfinite; rel/roteffortNaN2917 per sendi dicatat sebagai kanal tidak tersedia. LogSUCCEEDED dan tidak ada FAULT/REJECTED/TORQUE-ABORT/NOTarmed pada jendela (`health_after_F7.json`); sebelumgerakTCP masihF3galat2.406 mm/logtanpafaultbaru (`health_before_F7.json`). C6 langkah6lulus.

Ringkasan enam penunjuk (`pointer_six_summary.json`): F1/F6/F4/F5/F3/F7 masing-masing MOVED/SUCCESS, percobaanrencana1/2/1/1/4/3≤8, seluruhTCP≤5 mm, peak terukur sesi **12.4826 N·m≤13**. Predicate D597 untuk keenam tanda terpenuhi; penutupan/pulang belum dijalankan dan tidak diklaim selesai. Tidak menggunakan ukuran galat kamera untuk mengganti titik atau membuang data. F7 tetap UJI-TAHAN keduakamera/kesepakatan, tidak dipakai fit/LOO.

**Keadaan:** arm_3 menahanF7 (BUKAN REST),lenganlainREST,gantrytetap. Label/permukaanF7 belum dikonfirmasi; actualxy belum masukplan sampai label. Belum izin pulang; originalreturn_rest perlu ditinjau terhadap aturan keras margin97%/stop>14 sebelum dipakai. Berikut operator labelF7/tinggi dasar/untingaman dan susunan tumpukan245 mm yang terukur; kemudian rencana pulang disaring dan izin satu langkah. Draf amendemenBASE/D140/CD245 belumdikunci,belum botollantai/fit/uji hidup kandidat. Stack/kamera/localizer/monitor/perekam hidup.

### B32. F7/stack lengkap; proyeksi revisi dan rencana pulang terjaga lulus

Operator mengonfirmasi F7 ditandai di penyangga yang sama70 mm±2; actualxy=(0.6483,−0.5809) masuk kedua plan hanya dariTCP, role tetap UJI-TAHAN kedua kamera+kesepakatan. Susunan tumpukan terukur245 mm: **C di bawah, D di atas**, stabil. Seluruh alas/xy revisi tersedia: F1/F5 z0; F3/F4/F6/F7 z0.070; D+0.140, CD+0.245. Tinggi/peran meja dan hasil lama tetap. Konfirmasi untingF7 aman/lengan tidak tersentuh belum eksplisit dalam jawaban, diminta bersama izin pulang.

Proyeksi hidup semua18 permukaan (`amended_rgbd{,2}.log/.jpg`, gambar ditampilkan): RGBD semuaIN; RGBD2F4 BASE OUT(v729), D/CD IN(v708/690); F5 semuaIN(v686/665/647), F7 semuaIN(v698/676/659). F1/F6/F3 semuaOUT RGBD2, sesuai peran tidakdipakai. Visibilitas baru tidak mengubah role/menambahBASEF4 keFIT RGBD2. IN bukan bukti track tunggal, keamananfisik atau galat≤5 cm; penempatan/deteksi nanti diverifikasi.

**Konflik return_rest bawaan terhadap aturan keras sesi:** alat lama tidak menyaring margin97% pada garis lurus dan sengaja hanya melaporkan torsi pemulihan tanpa membatalkan. Tidak dipakai --move secara langsung. Ditambah wrapper khusus sesi `harness/return_rest_cam6dof.py`: reuse cosine30 s/60 langkah dan seluruh saringan plan_retract(se-gantry/antar-gantry/lingkungan), satu segmenlurus saja tanpa fallback/retry/gerak rel; cek titikawal terukur, map benar di planning scene, dan START-CHANGED>0.01 rad. Prediksi memakaiRNEA+offset/rating asli untuk enam sendi finite; >97% atau prediksi tidak lengkap ditolak sebelum command. Eksekusi satu goal, guard membatalkan bila torsiukur>14, effortlenganinvalid atau timeout; controllerREJECTED/error tidak diklaimMOVED. Targetmonitor REST dihasilkan dariFK URDF hidup pada tujuanREST+rel/rot terukur, kemudian dwell2 s/5 mm/5° dan galatsendiREST≤0.5° tetap haruslulus. Tidak mengubah alat bersama/defaultlaunch, tidak melonggarkan aturan pemulihan; aturan keras operator menang atas pengecualian kode lama.

Kontrol perangkatlunak5/5 (`return_guard_unit.log`): nilai97.14 yang tercetak97 tetap>97 ditolak; modelmissing/NaN/negatif ditolak; >14 meminta pembatalan walau hasil sudahSUCCESS; tepat14 tidakdibatalkan; errorcontroller tidakMOVED. Uji tidak membuat ROSnode atau gerak; --help lulus (`return_guard_help.log`). Tidak melakukan injeksi torsi berbahaya diHW.

**DRY HW pulang dariF7 lulus**, rc0 `return_guard_plan.json/.log`: satu segmen arm_3lurus30 s (arm_4 sudahREST, tidak ikut bergerak), margin j2 **92.587812%**, interarm tambahanmin623.1 mm/env115.3 mm CLEAR; saringanoriginalretract jugaCLEAR. Torsi8.589 pada hasilDRY adalah pengamatan pasif saatmenahanF7, **bukan torsi pulang**. Belum --move/izin pulang. Amendemen tetapDRAF sampai REST+K0, lalu salinan/hash/commit sebelum botollantai pertama; tidak menyatakan kunci revisi mendahului penunjuk atau datameja yang telahselesai.

Arm_3 tetapF7 (BUKAN REST),lenganlainREST,gantrytetap; stack/kamera/localizer/monitor/perekam hidup. Berikut izin **satu gerak pulang arm_3keREST saja**, unting/areaaman/e-stopdijaga; rencana aktual disaring ulang dan tidak pulang otomatis saatmenyiapkan draf.

**Izin gerak #7 diterima:** “Izinkan arm_3 pulangREST saja; unting/areaaman,e-stopdijaga”. Hanya pulang arm_3keREST satu lintasan30 s; arm_4 sudahREST,gantry/lenganlain tetap. Pemeriksaan awalpasifTCP/log `tcp_before_REST.log`/`health_before_REST.json` lulus. Command aktual memakai wrapperpulang97%/cancel>14, menyaring kembali dari keadaan terukur; belum REST/K0 sampai hasil diverifikasi.

### B33. GERAK #7 pulang REST lulus; K0 dan amendemen tinggi DIKUNCI

Operator mengizinkan arm_3 pulangREST saja,unting/areaaman,e-stopdijaga. Supervisor884262/probe884263 selesai rc0 (`move_REST_job/exit.json`). Wrappermerencanakan ulang dariF7, margin **92.587800%**≤97, seluruhsaringanCLEAR, mengirim satu goal arm_3cosine30 s; arm_4 sudahREST dan tidak ikut bergerak. **MOVED+dwellSUCCESS**: monitor2 s,41 sampel@20 Hz,posmaks3.65 mm/settled0.21 mm,orimaks0.20°. Torsiukur eksekusi **8.4515 N·m**; audit seluruhjob57.81 s (termasuk holding/planning) peakterbesar8.7493,161840baris,cakupan≤9 ms,gantry rel/rot0,lenganlainmaks0.011860°. Kanalrel/roteffortNaN5780 per sendi diungkapkan; semuaeffortlenganfinite. Tidak ada FAULT/REJECTED/TORQUE-ABORT/NOTarmed baru (`health_after_REST.json`).

TCP REST3 s,n88=(0.4029,−0.2936,0.9557),stdcet ak0 mm; terhadapFK tujuan galat **0.104 mm**,≤5 (`return_verified.json`). K0sesudahpulang **LULUS** (`k0_after_pointer_return.json/.log`): semuaREST,max0.090/0.199/0.044/0.147°, g1=1.499262262,g2=0.000900590,rot0/0. Jariarm_3−0.001403 m dilaporkan oleh K0 dan berada dalam gerbang tertutup alat; tidak mengarang nilai0. Tidak ada gerakrel atau lenganlain terukur. D597 **✓ untuk6tanda**: semuaPLANNED≤8 dan peaksesi12.4826≤13, pulang juga≤13. C6seluruh7gerak lulus.

**Amendemen penyangga lantai dikunci sesudah REST/K0, sebelum botolF pertama:** `results/p1_cam6dof/cam6dof_A_floor_amendment_locked.md`, sha256 **2022dbbc72f93e953daefa5d4d197a7c09aa0502214f1a280ed350c2e7ddff80** (`.sha256`). Kunci awald081044/hash3d13ea22…utuh; perubahanataspermintaanoperator A/B sulitdipindah, bukan memilihmodel/desain dari galat botollantai. TingkatbaruBASE/D140/CD245, Cdibawah/Ddiatas. F1/F5 z0/0.140/0.245; F3/F4/F6/F7 z0.070/0.210/0.315. Role/bobot/model/LOO/C1–C6/D590–D598 tetap, data meja/geraklama tetap; **revisi tidak mendahului data meja atau penunjuk**, hasilkalibrasi kelak dilabel protokolamendemen dan tinggi awal0/0.310/0.480 tidakdiklaimtelahdiuji. Rentanglebihpendek dan ketidaklengkapan C1N1 tetapdiungkapkan.

Snapshot configsebelumrevisi `plan_rgbd{,2}_inc_pre_floor_amendment.json`; configaktif sekarangmemuat kedua `protocol_hashes`, seluruh `floor_level_heights` dan namaBASE/D/CD. Semua actualxy dariTCP, peranF4RGBD2hanya D/CD; BASEstream direkam tetapi tidakFIT. Selektor/radius/bobot/detector tidakberubah. Tidak ada filecapture botolF saatpenguncian, tidak menjalankanfit atau kandidat/liveholdout.

**Keadaan:** semuaREST,g1PARK/g2home,K0LULUS;stack/kamera/localizer/monitor/perekam tetapHIDUP. Tidak perlu atau diizinkan gerakbaru. Setelah commitkunci, berikut penempatanlantai1/18 **F1/BASE**, botolputihyangsama langsunglantai tepatditanda; minta konfirmasi siap/menjauh sebelum15 s kedua kamera.

### B34. Papan skor checkpoint sesudah penunjuk/pulang, sebelum botol lantai

| Dugaan | Status harfiah | Batas bukti |
|---|---|---|
| D590 | **✗** | D tertinggi meja memperbaiki3 pasangan, memperburukM1/N2; walauN1missing dianggaplulus maksimum4/6<5/6. |
| D591 | **— belum diuji** | Belum fit/LOO FIT; tidak menyimpulkanM7 dari pola galat. |
| D592 | **— belum diuji** | Belum parameter/gerbangM6 RGBD2. |
| D593 | **— belum diuji** | Belum uji-tahan hidupM6 RGBD2 semua tinggi. |
| D594 | **— belum diuji** | Belum C3 hidupM6 RGBD2 REST×3. |
| D595 | **— belum diuji** | Belum botol incumbent lantai pada uji-tahan; baselinearm_2rgbd5.3 cm sudahgagalC3, atribusi belumselesai. |
| D596 | **— belum diuji** | Belum kesepakatanbasis botolF7 setelah kandidat. |
| D597 | **✓** | Keenam tandaHW PLANNED1/2/1/1/4/3≤8, torquepeak12.4826≤13 N·m; pulang juga≤13. |
| D598 | **— belum diuji** | Belum residualz sesudahfit semua tinggi. |

Skor **1✓/1✗,2/9 diuji**; ini checkpointaktif, bukan penutupanakhir. C6 tujuhgerak lulus; C1–C5 kandidat belumdisimpulkan, belumfit/pemasangan/liveholdout/DEMO-HW. “Apakah hanya rgbd2 yang salah?” **belum terbukti**; rgbd juga belumlulus gerbang baselineC3, tetapi belum dapat mengatribusikannya ke ekstrinsik/depth/gantry atau centroid. M6utama/M7 lewatLOOFIT saja; modelagenkuat efforthigh. Langkahberikut: setelahcommitamendemen minta penempatanF1/BASE dan rekam15 s kedua kamera saatoperatorsiap/menjauh.

**Checkpoint siap capture:** commit kunci amendemen `3b507b9` telah dibuat dan hash dicek ulang cocok. Penempatan lantai1/18F1/BASE langsunglantai diminta ke operator, menunggu siap/menjauh; belum captureF, semuaREST/PARK,K0LULUS,stack+dua localizer hidup. Helper capture lama hanya diadaptasi untuk membaca tinggiworld per tanda dari config dan memverifikasi kedua hash; selektor/detector/centroid/radius/bobot tidakberubah. Tidak ada gerakbaru.
