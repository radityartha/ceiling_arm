# P1 / CAM-6DOF — kalibrasi 6-DOF rgbd2 + cek/kalibrasi rgbd dengan titik 3D beracuan lengan (tanda unting + botol × 3 tinggi)

> Sesi 2026-10-12 malam (prompt [p1_cam6dof_prompt.md](p1_cam6dof_prompt.md); lanjutan [p1_rgbd2_zone.md](p1_rgbd2_zone.md) B7 (B) dan
> [p1_arm1_zone.md](p1_arm1_zone.md) B6 F1/F3). Operator sudah pulang sebelum gerbang → **sesi ini LURING saja** ("siapkan yang offline
> dulu"); HW dilanjutkan 2026-10-14. §A di bawah **DIKUNCI 2026-10-14**, sesudah angka operator, K4 hidup dan enam plan-check HW; salinan `results/p1_cam6dof/cam6dof_A_locked.md` + sha256 dibuat SEBELUM data botol pertama dan SEBELUM gerak pertama. Data: [results/p1_cam6dof/](results/p1_cam6dof/).

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
