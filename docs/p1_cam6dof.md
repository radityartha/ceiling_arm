# P1 / CAM-6DOF — kalibrasi 6-DOF rgbd2 + cek/kalibrasi rgbd dengan titik 3D beracuan lengan (tanda unting + botol × 3 tinggi)

> Sesi 2026-10-12 malam (prompt [p1_cam6dof_prompt.md](p1_cam6dof_prompt.md); lanjutan [p1_rgbd2_zone.md](p1_rgbd2_zone.md) B7 (B) dan
> [p1_arm1_zone.md](p1_arm1_zone.md) B6 F1/F3). Operator sudah pulang sebelum gerbang → **sesi ini LURING saja** ("siapkan yang offline
> dulu"); HW = sesi berikut. §A di bawah = **DRAF**; dikunci (salinan `results/p1_cam6dof/cam6dof_A_locked.md` + sha256) sesudah angka
> operator + K4 hidup + plan-check HW, SEBELUM data botol pertama dan SEBELUM gerak pertama. Data: [results/p1_cam6dof/](results/p1_cam6dof/).

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

### A1. DRAF urutan (setiap gerak nyata ditanya, satu langkah per persetujuan)

0. Gerbang (AskUserQuestion): K0, K1, K2 (+ **angka: tinggi balok A/B ±2 mm, diameter botol putih, lantai = acuan meja 0.77 → z lantai 0**), K3 (`ps`), K3b.
1. Bring-up nyata (`p1_demo_hw/harness/stack_up.sh`, `octomap_updaters:=false`), K0 `--rail1 1.50 --rail2 0.00`, monitor + perekam
   (cek tumbuh). Kamera incumbent (`p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280`) + `loc_cam.sh rgbd` + `loc_cam.sh rgbd2`.
   `arm_fk_check --arm-filter 0.03` REST ×3 (baseline incumbent) + `cam6dof_capture.py rest_inc.npz` (data lengan untuk fit).
   `proj_marks.py rgbd` + `rgbd2` dengan kandidat F → gambar → **K4** ke operator. Plan-check HW F terpilih.
   → isi angka + set F → **KUNCI §A** (sha256 + commit).
2. Zona rgbd2 (NOL gerak): botol di 6 tanda meja × 3 tinggi (meja 0.77, balok A, balok B), `track_median.py 15 … /loc_rgbd2/tracks`
   (+ `/loc_rgbd/tracks` direkam juga). Balok dipusatkan di tanda (garis bantu), botol di tengah balok.
3. Zona rgbd (hanya bila K4 ya): arm_3 penunjuk ke F (tiap gerak ditanya; pola `mv_mark.sh` dengan arm_3) → unting + label → `return_rest
   --arms arm_3 arm_4 --move` → botol di tiap F × 3 tinggi (lantai, A, B), rekam kedua localizer.
4. Fit per kamera (M6/M7, `cam6dof_fit.py`) → arg baru → kamera diluncurkan ulang (`cam_up_6dof.sh`) → TF hidup = prediksi →
   uji-tahan HIDUP (botol di tanda uji-tahan semua tinggi) + `arm_fk_check` REST ×3 + botol di titik dua-kamera.
5. Akhir: lengan REST, g1 PARKIR, K0, matikan bersih (0 node domain 0 & 79, 0 `static_transform_publisher`), arsip, laporan.

### A2. DRAF peran titik (dikunci SEBELUM data botol)

| zona | tanda | peran rgbd2 | peran rgbd |
|---|---|---|---|
| meja arm_1 (z 0.77 / +A / +B) | M1, C5, N1, N2 | FIT | — (tak terlihat) |
| | **C6, C3** | **UJI-TAHAN** | — |
| lantai arm_3 (z 0 / A / B) — usulan dari A0.8/A0.9, ganti dengan cadangan bila tak terlihat hidup / gagal plan-check HW | F1 (0.25, −0.50), F6 (0.40, −0.70) | — (tak terlihat) | FIT |
| | F4 (0.55, −0.60), F5 (0.65, −0.48) | FIT (bila ter-track) | FIT |
| | **F3 (0.25, −0.60)** | — | **UJI-TAHAN** |
| | **F7 (0.65, −0.58)** = titik dua-kamera | **UJI-TAHAN** | **UJI-TAHAN** |
| cadangan | (0.55, −0.50), (0.60, −0.66) j2 96, (0.40, −0.60) | — | — |

Lengan (REST, tangkapan `rest_inc.npz`) masuk FIT bila lolos gerbang incumbent (jendela ≥ 300, tersimpan ≥ 30 % dan ≥ 200, median ≤ 15 mm).

### A3. DRAF kriteria (per kamera)

| # | Lulus bila |
|---|---|
| C1 | fit sah: gerbang A0.1 + tiap tanda FIT punya track botol tunggal di ≥ 2 tinggi |
| C2 | **uji-tahan HIDUP, ekstrinsik baru, SEMUA tinggi: \|track_xy − tanda_xy\| ≤ 0.05 DAN \|track_z − (z permukaan + h_c model)\| ≤ 0.03** |
| C3 | lengan HIDUP REST (`arm_fk_check --arm-filter 0.03` ×3, median per lengan): tiap lengan yang terlihat kamera itu di baseline: \|t_xy\| ≤ 0.05 DAN ≤ incumbent + 0.01, masih terlihat (tersimpan ≥ 30 %) |
| C4 | dua-kamera (F7): \|basis_rgbd − basis_rgbd2\|xy ≤ 0.05, basis = track − offset centroid model kamera itu (mentah dilaporkan) |
| C5 | TF hidup = prediksi ≤ 1 mm / 0.01° |
| C6 | penunjuk: tiap gerak MOVED + dwell SUCCESS, TCP ≤ 5 mm, torsi terukur ≤ 14 |

**Keputusan:** rgbd2 baru DITERIMA bila C1–C5 (C6 untuk sesi). rgbd: bila **incumbent** lulus C2 + C3 → TIDAK diubah ("rgbd tidak salah");
lainnya → fit rgbd, kriteria sama. Diterima = harness BARU `cam_up_6dof.sh`; default launch TIDAK diubah tanpa operator. Gagal → incumbent
tetap; lapor model mana yang gagal (tilt / skala / bias botol) dan kenapa. Berhenti + amankan: fault / LED merah / REJECTED / NOT armed
sesudah ARMED / torsi > 14 / benturan / kabel / gerak tak diminta. MARGIN = TOLAK (> 97 %); penyaring tidak dilonggarkan. Kriteria tidak
diubah sesudah melihat data; uji-tahan tidak pernah dipakai untuk fit / memilih model.

### A4. DRAF dugaan

| # | Dugaan |
|---|---|
| D590 | rgbd2 incumbent: \|dxy\| botol di balok B (lebih dekat kamera) < di meja pada ≥ 5/6 tanda. |
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
