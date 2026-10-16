# P1 / DEMO-ZONE (arah C) — koreksi lokal rgbd2 di zona meja demo arm_1, dari data yang ADA

> 2026-10-17, LURING (NOL HW/kamera/gerak). Operator: "lanjutkan C. Terima galat kamera sekarang. Untuk demo, pakai koreksi lokal
> per zona (cara RGBD2-ZONE) daripada ekstrinsik 6-DOF penuh. tapi lakukan anaisis mendalam dengan data yang ada, tekan galat
> sekcil mungkin". G63e (M3) DITUNDA. Alat + angka: [results/p1_demo_zone/](results/p1_demo_zone/).

## Data (semua rgbd2, ekstrinsik incumbent + G34 yang sama, tanda = TCP arm_1 terukur/FK, plumb ke meja)
- **A** RGBD2-ZONE 2026-10-12: botol DEMO putih 7 cm di meja 0.77, 6 tanda M1/C5/C3/C6/N1/N2 (`p1_rgbd2_zone/fit_data.json`).
- **B** CAM-6DOF 2026-10-14: botol putih H 165 / D 65 mm, tanda sama, permukaan 0.77 / 0.87 / 0.91 (kotak C/D), 14 obs ada
  track (N1/T0 missing; N1/TD rival) (`p1_cam6dof/hw_20261014/tracks_inc/*_summary.json`).
- rgbd tidak melihat meja ini. Total 20 observasi.

## Temuan
1. **Galat incumbent 14–23 cm** (median 17.6). Diurai di bingkai kamera: **jarak +10…+22 cm** (track terlalu jauh: bias material
   botol B64 +8 cm + skala depth di ~3 m) dan **sudut sinar 0.8–2.8°** (du +0…+16, dv −28…0 px).
2. **Sinar RGB saja ∩ bidang meja TIDAK menolong di zona ini** (galat 17–29 cm pada z_c 0.805): beda dengan F5 dekat (0.22 cm).
   Sinar rgbd2 sendiri meleset ~1–3° pada 3 m → bergantung posisi citra (kemiringan/ekstrinsik), bukan hanya depth.
3. **C5** tersisa 3–4 cm di KEDUA sesi (du +12…+15 px vs M1 −1 px, tanda bersebelahan di citra) → galat sistematis tanda (FK
   pointer arm_1, sejalan G63c: kinematika 1–3°/sendi) atau tepi zona. Catatan: untuk demo, tanda FK = acuan yang BENAR (lengan
   diperintah ke koordinat FK yang sama kembali ke titik fisik yang sama, asal konfigurasi mirip).
4. **Lantai keterulangan data:** tanda sama, beda hari/botol 0.4–2.2 cm; beda tinggi 1–3 cm.

## Model (validasi tinggalkan-satu-TANDA = LOMO, dan silang botol)

| Model | fit RMS | LOMO med / p90 / maks | botol 165→7 cm med/maks | 7→165 cm |
|---|---|---|---|---|
| tanpa koreksi | 17.3 | 17.6 / 21.4 / 22.8 | 17.4 / 22.8 | 17.6 / 22.0 |
| T2 geser xy | 4.1 | 4.8 / 6.4 / 7.2 | 2.7 / 6.3 | 4.4 / 6.5 |
| **P3 planar x, y, yaw** | **2.6** | **3.0 / 4.8 / 5.3** | **2.1 / 3.9** | **3.0 / 4.9** |
| S3 kamera (du, dv, offset jarak) | 3.9 | 4.8 / 6.5 / 7.2 | 3.0 / 5.3 | 4.2 / 6.8 |
| S4 + skala jarak | 3.9 | 5.7 / 7.1 / 7.8 | | |
| RP2/RP3 sinar-bidang (tanpa depth) | 4.5–6.6 | 4.3–5.2 / 7.4–10.5 | gagal | gagal |
| A6 afin | 2.3 | 4.7 / 7.6 / 8.9 (overfit) | | |

Subset: meja saja (A + B_T0) P3 LOMO 2.9 / 4.9 / 5.1; A saja (botol demo) P3 2.3 / 3.9 / 4.5 (n = 6, rapuh).
**P3 dipilih** (fit A + B, 20 obs): pusat (1.8312, 0.6048), geser (−0.1237, −0.1143), yaw +14.72° → `results/p1_demo_zone/zone_p3_fit.json`.
Yaw besar = peta lokal di bidang meja, BUKAN yaw kamera (RGBD2-ZONE B4: dipasang ke kamera merusak lengan dekat > 25 cm).
Konfirmasi hidup yang sudah ada (RGBD2-ZONE B4, koreksi planar setara): uji-tahan C6 1.9 cm, C3 0.9 cm.

Bootstrap tanda (1σ prediksi, dalam koordinat TRACK): 1.0–1.7 cm di cakupan tanda (track x 1.85–2.11, y 0.55–0.80), naik ke 2.5–3 cm
di luar (track x < 1.7) → **zona sah = selubung tanda + ~5 cm** (dunia ≈ x 1.60–1.99, y 0.45–0.73, meja 0.77).

## Rekomendasi untuk demo (belum diterapkan; keputusan operator)
1. Ekstrinsik/launch TIDAK diubah. Koreksi P3 dipasang sebagai **pasca-proses track rgbd2 HANYA di dalam kotak zona** (di luar zona
   → target ditolak, bukan dikoreksi); z target = meja 0.77 + setengah tinggi benda (track z 0.81–0.83 konsisten untuk botol 7 cm).
2. Perkiraan galat xy demo di zona: **median ~2–3 cm, p90 ~5 cm** (vs 14–23 cm sekarang); plus galat FK bila konfigurasi pendekatan
   (z ~1.05) beda dari saat menandai (z 1.22) — belum terukur.
3. Penerimaan hidup (kamera saja, NOL gerak): botol demo di 2 tanda (mis. C6, N1) ≤ 5 cm sesudah koreksi.

## Cara menekan lebih jauh (butuh data baru)
- **Kamera saja:** ulang penempatan botol demo 3× di 6 tanda (placement ±1 cm dirata-rata) → fit noise turun; tidak menembus
  galat FK tanda.
- **Dengan gerak (izin per langkah):** 3–4 tanda baru di sudut zona (pointer arm_1) → kurangi ekstrapolasi + uji C5; tanda di
  konfigurasi pendekatan demo (z ~1.05) → galat FK-konfigurasi ikut terserap.

**Tidak dijalankan:** HW/kamera; node pasca-proses belum ditulis; validasi hidup belum.

## Node pasca-proses + cek oklusi (langkah 1–2, LURING, 2026-10-17)

- **Node** [zone_correct_node.py](results/p1_demo_zone/zone_correct_node.py) (commit 38d9a6c): masuk `/loc_rgbd2/tracks` (localizer rgbd2
  saja, `p1_cam6dof/harness/loc_cam.sh rgbd2`) → keluar `/demo_zone/tracks` (skema SAMA dengan `/detected_objects/tracks`, xy P3,
  z = 0.77 + H/2, + `x_trk/y_trk/z_trk`) dan `/demo_zone/rejected` (+ alasan). Gerbang (semua wajib): track xy di kotak TRACK
  (selubung 20 obs + 5 cm: x 1.78–2.17, y 0.49–0.81), xy terkoreksi di kotak ZONA (x 1.60–1.99, y 0.45–0.73), |z_track − z_out| ≤ 0.05.
  Runner/CLI tidak diubah — cukup remap `/detected_objects/tracks:=/demo_zone/tracks` (belum dipakai).
- **Uji unit** [test_zone_correct.py](results/p1_demo_zone/test_zone_correct.py) SEMUA PASS ([log](results/p1_demo_zone/test_zone_correct.log)):
  20 obs RMS **2.576 cm = analisis** (zone_models.json P3); A 6/6 diterima, galat 0.6–3.9 cm, z 0.805; benda di kotak C/D (z 0.87–1.00)
  ditolak dengan H 0.07; wastafel / meja arm_2 / ekstrapolasi track ditolak; 3 pojok kotak track ditolak HANYA oleh gerbang zona.
  Asap ROS (domain 93): track C6 lama → (1.884, 0.612) = 2.0 cm dari tanda, wastafel `TOLAK`, SIGINT bersih.
- **Oklusi** [zone_occlusion.py](results/p1_demo_zone/zone_occlusion.py) (commit bcafc83): jejak botol = proyeksi track lama (piksel
  sama dengan saat rekam); lengan REST URDF via T_wc incumbent, 'urdf' dan 'geser' (galat jarak jauh DEMO-HW B1).
  Mode 'urdf' memprediksi N1 tertutup 63 % di g1 1.50 padahal N1 TERLIHAT saat itu → 'urdf' salah, 'geser' cocok (0 %).
  g1 1.40 'geser': N1 **34 %**, lainnya 0 %. **Frame nyata** g63b `B-R0_rgbd2.png` (g1 1.40, arm_1/arm_2 REST; lokal, tidak di-commit):
  gripper arm_1 jatuh di N1 dan menyentuh tepi kanan C6/N2; frame CAM-6DOF `k4_rgbd2.jpg` (g1 1.50): zona bersih. **M1/C3/C5 bersih
  di keduanya.** Geser g1 → 1.50 butuh lepas papan arm_2 (REST + papan 43 mm ke meja di 1.50, G63 A8) + bring-up + `pose_to_g`.
  **Keputusan operator (tombol):** uji hidup **C3 + M1, g1 tetap 1.40, NOL gerak.**

## DEMO-ZONE-LIVE

### A. Protokol — DIKUNCI (sebelum kamera dibuka)

**A0.** Keadaan: papan polos di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack MATI. NOL gerak lengan/rel.
Tanda uji **C3 (1.8486, 0.6786)** dan **M1 (1.7497, 0.5493)** — dipilih karena bersih dari oklusi (bukan dari galat). Keduanya ADA di
fit P3 (20 obs) → ini uji **penempatan ulang + node hidup**, BUKAN uji-tahan tanda (LOMO P3 = analisis luring). Pengungkapan: data A
dari localizer fusi (rgbd tidak melihat zona → = rgbd2), B + uji ini dari localizer rgbd2 saja.

**A1. Urutan** (kamera saja, domain terpisah 79 bila kosong; harness incumbent, ekstrinsik incumbent + G34, default launch tidak diubah):
1. `ros2 node list --no-daemon` domain 79 kosong; `cam_up.sh seg_imgsz:=1280` + `loc_cam.sh rgbd2` + `zone_up.sh` (node, H 0.07),
   semua `setsid nohup`, PID dicatat. Lampu NYALA (operator).
2. **Kosong:** tanpa botol di zona, rekam 5 s `/demo_zone/tracks` + `/demo_zone/rejected`.
3. Per tanda (C3 lalu M1): operator meletakkan botol demo putih 7 cm tepat di tanda, menjauh → rekam **15 s** bersamaan
   `/loc_rgbd2/tracks` (mentah) dan `/demo_zone/tracks` (terkoreksi) + `/demo_zone/rejected` (`track_median.py`).
4. Matikan per PID (SIGINT, anak `object_localizer` dibunuh per PID; tanpa `pkill -f`); node 0 di domain 79.

**A2. 🔒 Gerbang (per tanda uji, KEDUA tanda wajib lulus; tidak dilonggarkan)**

| # | Lulus bila |
|---|---|
| L1 | satu track tunggal: tepat SATU track diterima di `/demo_zone/tracks`, tid sama, hadir di ≥ 90 % pesan 15 s |
| L2 | \|median xy terkoreksi − tanda\| ≤ **5 cm** |
| L3 | median z track MENTAH ∈ **[0.79, 0.84]** |

L1–L3 ✓ di C3 DAN M1 → koreksi zona DITERIMA untuk demo (pemakaian = keputusan operator; runner tetap belum di-remap). Lainnya → ✗,
ditulis, tidak dipakai. Berhenti: kamera/driver gagal, `inference failed` berulang, orang di zona saat rekam (ulang tanda itu).

**A3. 🔒 Dugaan**

| # | Dugaan |
|---|---|
| D649 | L1 ✓ di kedua tanda; botol wastafel (`cyan bottle`) muncul di `/demo_zone/rejected`, tidak pernah diterima. |
| D650 | L2 dengan galat ≤ **3 cm** di kedua tanda (residu fit A: C3 0.6, M1 2.2 + penempatan ulang 0.4–2.2). |
| D651 | z mentah ∈ [0.80, 0.83] di kedua tanda. |
| D652 | galat MENTAH (tanpa koreksi) 0.15–0.20 m di kedua tanda (A: M1 0.164, C3 0.188). |
| D653 | output node hidup = P3 luring dari median mentah ≤ 1 mm. |
| D654 | Kosong (A1.2): nol track diterima selama 5 s. |

### B. Hasil terukur

§A sha256 `829892b9…` ([salinan](results/p1_demo_zone/demo_zone_live_A_locked.md)), commit d2b394e — dikunci SEBELUM kamera dibuka.

**B0. Alat luring (sesudah kunci, sebelum data botol; tidak mengubah gerbang):** [live/](results/p1_demo_zone/live/) `live_up.sh` / `live_rec.sh`
/ `live_down.sh` + `live_score.py` (L1–L3 + D650–D653 otomatis). Uji: rekaman lama RGBD2-ZONE old_C3/old_M1 lewat `process()` →
C3 0.61 / M1 2.17 cm, mentah 18.77 / 16.42 (= tabel A); replay ROS domain 79 (old_C3 → node → `live_rec.sh`) 30/30 pesan, L1–L3 ✓,
node vs luring 0.09 mm; kontrol negatif (2 track diterima / nol track) → L1 ✗.

**B1. Sesi 1 (07:43–07:48), kamera saja domain 79, NOL gerak** — [hw_20261017/](results/p1_demo_zone/live/hw_20261017/)
- Izin operator (tombol): lampu NYALA, C3 + M1 ada, zona kosong. Preflight: 0 node di domain 0/79, 2× D455, GPU 217 MiB, disk 17 GB bebas.
- Launch 1676946 (SigIgn `0x1001001`, SIGINT tidak diabaikan), localizer rgbd2 1677541, node 1677542. `seg_router` imgsz **1280**
  (param terbaca), label rgbd2 3.5 Hz, 0 `inference failed`.
- **A1.2 kosong 5 s:** mentah 9 pesan / terkoreksi 8 / tolak 10 — **semua kosong** (botol wastafel hari ini tidak terdeteksi).
- Operator bertanya apakah botol HITAM lebih baik → dijawab: §A + fit P3 pakai botol PUTIH (bias depth bahan terserap); hitam = info
  sesudah gerbang saja. Operator: "belum siap … jam 9 saya baru ke lokasi". Kamera dimatikan bersih (launch hilang 4 s, 0 proses,
  0 node di 79); galat `seg_router` saat shutdown = pola SIGINT dikenal.
- **Belum:** A1.3 botol di C3 / M1 (jam 9).
