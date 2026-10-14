# CAM-6DOF — amendemen penyangga lantai DIKUNCI 2026-10-14

**DIKUNCI sebelum data botol lantai pertama. Capture hanya setelah operator mengonfirmasi penempatan/menjauh; setiap gerak nyata tetap memerlukan izin satu langkah.** Amendemen diminta operator karena A/B besar/sulit dipindahkan. Dokumen ini melengkapi, tidak menimpa protokol awal `cam6dof_A_locked.md` (commit d081044, sha256 `3d13ea22a89d87bb3ac789bda902a6f2f489f9594d40b166d9426a14a78dc802`). Semua ketentuan asli berlaku kecuali desain penyangga/tinggi lantai yang dirinci di bawah.

## Alasan, waktu, dan batas klaim

Permintaan operator sesudah data incumbent meja18/18 dan gerak penunjuk1–3, tetapi **sebelum data botol lantai apa pun dan sebelum fit/LOO/model-selection**. Data lama, baseline REST×3, peran tanda, hash awal, dan bukti gerak tetap. Hasil kelak dilabel protokol amendemen; tidak diklaim seluruh sesi mengikuti revisi sejak awal. Kepatuhan tinggi protokol asli z0/0.310/0.480 tidak lengkap. Tidak memulai ulang A0/G61/REST baseline/meja.

## Tinggi dan susunan

- Botol fisik tetap satu botol putih H70 mm, diameter60 mm, radius30 mm.
- C100 mm, atas95×70 mm; D140 mm, atas92×50 mm: terukur±2 mm. Data meja tetap T0/TC/TD=0.770/0.870/0.910 m, tanpa tumpukan.
- Tumpukan C+D **245 mm terukur±2 mm**, stabil/rata dengan botol. Nilai245 dipakai, bukan penjumlahan nominal240. Susunan operator: **C di bawah, D di atas**; pertahankan susunan terukur ini, tanpa tambahan benda di antaranya.
- Lantai: tiga tingkat **BASE** (botol langsung permukaan dasar tanda), **D** (box D140), **CD** (tumpukan245). C individual tidak menambah tingkat keempat.
- z world permukaan = z dasar tanda + tinggi penyangga. F3/F4/F6/F7 pada penyangga tetap yang sama **0.070 m±0.002**, kuat menahan C/D; BASE/D/CD=0.070/0.210/0.315 m. F1 sudah dikonfirmasi lantai, BASE/D/CD=0/0.140/0.245. F5 langsung lantai z0 dikonfirmasi; F3 memakai penyangga0.070 m yang sama denganF4/F6; F7 dikonfirmasi pada penyangga0.070 m yang sama. Semua permukaan baru harus diverifikasi sebelum kunci; bila bukan lantai, tinggi dasar diukur±2 mm.
- Posisi pusat alas botol tepat di atas xy tanda. Box/stack tidak digeser mengikuti deteksi kamera. D top lebih sempit5 mm per sisi dari diameter botol; hanya gunakan penempatan yang operator pastikan tegak/stabil. Jika tidak stabil/terhalang, missing dicatat, tidak mengubah posisi/peran atau memakai z palsu.
- Box kecil hanya masuk area penunjuk sesudah arm_3/4 kembali REST dan K0 lulus; setiap gerak termasuk pulang tetap memerlukan izin satu langkah.

| tanda | xy actual (m) | z dasar (m) | BASE / D / CD permukaan world (m) | role rgbd | role rgbd2 |
|---|---|---|---|---|---|
| F1 | 0.2512,−0.4998 | 0 | 0 /0.140 /0.245 | FIT | tidak dipakai |
| F6 | 0.3998,−0.6998 | 0.070 | 0.070 /0.210 /0.315 | FIT | tidak dipakai |
| F4 | 0.5491,−0.6003 | 0.070 | 0.070 /0.210 /0.315 | FIT | FIT dua tingkat terangkat D/CD |
| F5 | 0.6497,−0.4779 | 0 | 0 /0.140 /0.245 | FIT | FIT tiga tingkat |
| F3 | 0.2513,−0.5991 | 0.070 | 0.070 /0.210 /0.315 | UJI-TAHAN | tidak dipakai |
| F7 | 0.6483,−0.5809 | 0.070 | 0.070 /0.210 /0.315 | UJI-TAHAN | UJI-TAHAN + kesepakatan kamera |

Seluruh actualxy di tabel diisi hanya dari TCP terukur penunjuk dan konfirmasi label, sebelum capture botol. Tidak mengubah nama/peran mengikuti galat deteksi. BASE F4 tidak ditambahkan keFIT rgbd2 dari pengamatan baru; pengecualian dua tingkat terangkat pada F4 tetap seperti protokol asli.

## Urutan dan penguncian

Penunjuk F1→F6→F4→F5→F3→F7 dan pulangREST telah selesai MOVED/SUCCESS dengan izin satu langkah dan guard97%/torsiukur14; semua actualxy/alas/operatorlabel sudah dicatat. K0 sesudah pulang LULUS: semuaREST,g1=1.499262262/g2=0.000900590 m,rot0/0. Plan-only tidak dianggap eksekusi. Gerak baru tidak dicakup oleh kunci ini.

Sebelum data lantai pertama telah selesai: seluruh xy/z aktual dan susunan stack, proj_marks hidup untuk BASE/D/CD kedua kamera memakai tinggi baru (IN bukan jaminan deteksi atau aman), serta K0 sesudah pulangREST. Salinan amendemen ini+sha256 di-commit dan commit dicatat di§B/state sebelum capture diizinkan. Config capture/fit harus menyertakan identitas kedua kunci, tinggi baru dan actualxy; tidak memakai config LA/LB lama. Jangan memilih ulang model/threshold atau titik dari data uji-tahan.

Capture incumbent lantai setelah operator siap/menjauh,15 s kedua kamera, urutan tanda asli F1→F6→F4→F5→F3→F7, masing-masing BASE→D→CD. Capture yang tidak terlihat/missing tetap diungkapkan. Uji-tahan hidup kandidat kelak memakai tinggi amended yang sama; semua tingkat harus lulus. Tidak menghapus data buruk atau mengulang sampai mendapat angka lebih baik.

## Model, gerbang, dan dugaan tetap

Protokol asli A0/A3/A4 utuh: M6 utama; M7 hanya lewat LOO leave-one-mark seluruh tinggi **FIT saja**, perbaikan≥max(0.005 m,15% LOO M6). Tidak memakai holdout untuk fit/inisialisasi/model-choice. Fit polos berbobot, σbottle0.015, tiap arm6 obs1D σ0.03, point-to-plane |nz|≤0.5; tanpa robust loss/pembuangan data buruk. Konvergen, |w|≤5°, |τ|≤0.15 m, |s−1|≤0.06. M7 tidak bisa dipasang melalui TF statis; lapor M7, kandidat hidup tetapM6; skala depth memerlukan keputusan/change tersendiri.

C1: fit sah dan tiap tandaFIT track tunggal≥2 tinggi. C2: holdout HIDUP **semua tinggi**, xy≤0.05 m dan z−(permukaan+offset centroid)≤0.03 m. C3: tiap lengan baseline terlihat REST×3, medianxy≤0.05 m dan≤incumbent+0.01, visibility≥30%. C4: basis botol kedua kamera F7 setelah offsetcentroid, xyagreement≤0.05 m; mentah dilaporkan. C5: TFhidup cocokprediksi≤1 mm/0.01°. C6: tiap gerakMOVED+dwellSUCCESS, TCP≤5 mm,torsi≤14. MARGIN=TOLAK>97%, semua penyaring asli tetap. Defaultlaunch tidak diubah; penerimaan memakai harnessbaru, rgbd incumbent lulus tetapdipakai.

D590–D598 asli dipertahankan harfiah, D590 sudah✗ dari meja. D591–D598 dinilai pada set amendemen dengan perubahan desain diungkapkan, tidak diberi skor untuk bagian belum dijalankan. Jawaban “hanya rgbd2 salah” belum terbukti; incumbent rgbd baselineC3 arm_2=5.3 cm gagal batas5 cm, atribusi belum selesai. DataN1 FIT rgbd2 belum memenuhi C1 menurut checkpointB11/B13, tidak diperbaiki diam-diam lewat amendemen penyangga.

Rentang penyangga baru0.245 m lebih kecil dari0.480 m asli; informasi pemisahan pose/skala dapat berkurang. Keputusan model tetap oleh LOO FIT dan penerimaan ketat; tidak menjanjikan bahwa revisi akan lulus.

## Bukti persiapan sebelum penguncian

Enam actualxy dan alas sudah diisi dari penunjuk+operator, susunanC-bawah/D-atas245 mm dikonfirmasi. Proyeksi hidup `hw_20261014/amended_rgbd{,2}.log/.jpg`: rgbd semua18 titikpermukaan IN; rgbd2F4 BASE OUT(v729), D/CD IN(v708/690), F5 danF7 semuaIN. F1/F6/F3 OUT rgbd2 dan tidak dimasukkan ke perannya. IN hanya geometri bingkai, bukan bukti track tunggal atau galat lokalisasi. Gambar ditampilkan kepadaoperator.

Return khusus sesi `harness/return_rest_cam6dof.py` mempertahankan cosine30 s/60 langkah dan tiga saringan `return_rest.plan_retract`, tanpa fallback atau segmen otomatis; menambah validasi modeltorsi6sendi/finite, margin97%, cek map dalam planning scene, START-CHANGED dan pembatalan>14/effortlenganinvalid/timeout. Target monitorREST dibuat dariFK URDF hidup pada tujuan sendi dan rel/rot terukur; MOVED+dwellSUCCESS sertaK0 harus diverifikasi. DRY hidup dariF7 lulus92.587812% (`return_guard_plan.json`), tanpaeksekusi. Pulang disetujui operator satu langkah dan selesaiMOVED/SUCCESS (`move_REST.json`), peakeksekusi8.4515 N·m, galatREST maksimum0.147°, TCPterukur cocoktargetFK0.104 mm (monitor settled0.21 mm), K0LULUS (`k0_after_pointer_return.json`). Pulang dan data meja/penunjuk mendahului kunci amendemen ini; kunci mendahului seluruh capture botol lantai.

## Identitas data dan gerbang capture

Capture incumbent lantai memakai file baru `tracks_inc/F*_BASE_*`, `F*_D_*`, `F*_CD_*`; file LA/LB lama tidak diisi atau disamarkan sebagai tinggi baru. Config `plan_rgbd_inc.json`/`plan_rgbd2_inc.json` memuat `protocol_hashes` kedua kunci dan `floor_level_heights` world per tanda. Snapshot config lama sebelum perubahan tinggi diarsip terpisah. Sebelum setiap capture, periksa hash kedua dokumen, kecocokan config/tinggi, belum ada file penempatan itu, lalu catat operator siap dan waktu/rc kedua perekam. Selektor lama tetap label contains bottle +radius0.35, track terbanyak dan semua saingan dicetak; tidak menggabung ID/buangbadpoint atau mengubah detector/centroid. Role BASEF4 pada rgbd2 tidak dipakai walau stream tetap direkam, dua tingkat terangkat tetapFIT. Holdout tidak dipakai fit/init/LOO/modelchoice.

## Protokol dasar yang tetap berlaku

Bagian asli A0–A4 direproduksi di bawah sebagai referensi lengkap. **Amendemen ini menggantikan hanya pilihan penyangga/tinggi lantai dan cara capture lantai serta penjaga pulang yang lebih ketat.** Keterangan tinggi A/B dan larangan tumpuk pada versi dasar tetap catatan protokol awal, bukan tinggi operasional revisi. Nilai model/bobot, roleFIT/UJI-TAHAN, C1–C6 dan dugaan tetap seperti aslinya.

# CAM-6DOF §A DIKUNCI 2026-10-14 — sebelum data botol / gerak penunjuk

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

