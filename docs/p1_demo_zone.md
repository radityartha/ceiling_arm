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
