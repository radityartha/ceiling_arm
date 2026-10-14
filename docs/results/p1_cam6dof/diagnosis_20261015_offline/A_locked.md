# Diagnosis luring F5/D 2026-10-15 — §A (DIKUNCI sebelum kode/analisis)

Sumber data SAJA: 70 snapshot NPZ lokal `diagnosis_20261014_F5D_01/{capture,capture_rgbd_only}/*.npz` + metadata
`diagnosis.json` (K, D, TF incumbent+G34 per frame). FIT F5/D: botol 165×65 mm di atas box D, permukaan z 0.140,
xy aktual (0.6497, −0.4779). NOL kamera/stack/gerak. Tidak mengubah raw/skor/kunci kampanye lama, TF, launch, atau kode
produksi. F7/C3/C6 holdout TIDAK dipakai. Hasil informasional: satu penempatan, dua run; tidak menerima kalibrasi.

## Uji
- **U1 kesehatan depth:** per kamera/run, fraksi depth valid (finite, 0.1<d<12) seluruh citra; peta validitas grid 16×9;
  stabilitas peta invalid antar-snapshot (IoU biner piksel invalid tiap snapshot vs median); hubungan validitas dengan
  kecerahan RGB (saturasi ≥250, gelap ≤20).
- **U2 bidang lantai:** titik dunia dari depth+TF incumbent; RANSAC bidang pada |z|<0.20 m (iterasi tetap, seed 0, ambang
  2 cm). Laporkan tilt normal vs +z (°), offset z di titik tengah inlier, dan rasio d_ukur/d_prediksi per piksel inlier
  (d_prediksi = jarak sepanjang sinar piksel ke bidang z=0 dengan TF incumbent) dibinning terhadap d_prediksi.
- **U3 lokalisasi botol:** instance = mask terbesar per snapshot (tanpa memakai GT untuk memilih). Metode:
  M-base median titik dunia mask penuh (setara localizer); M-top piksel teratas mask (persentil 2 v, median u pada baris
  itu) sinar ∩ bidang z = 0.140+0.165 = 0.305, lalu digeser radius 0.0325 m menjauhi kamera dalam xy (piksel teratas ≈
  tepi jauh rim); M-ctr centroid piksel mask sinar ∩ z = 0.2225. Galat xy vs (0.6497, −0.4779).
- **U4 bias depth di botol:** piksel mask dengan depth valid yang sinarnya memotong silinder GT (TF incumbent): residu
  d_ukur − d_silinder (permukaan depan). Median per kamera.

## Dugaan (dikunci, papan skor harfiah)
- **D600** rgbd: peta invalid stabil (median IoU ≥ 0.7) pada kedua run → penyebab statis (sensor/konfigurasi/adegan),
  bukan derau acak.
- **D601** rgbd2 U2: |median rasio − 1| > 0.03 pada bin d_prediksi ≥ 2.5 m (bias depth/sudut di jarak jauh).
- **D602** rgbd U2: |median rasio − 1| ≤ 0.03 di semua bin dengan ≥ 1000 piksel.
- **D603** U3 rgbd2: M-top galat xy ≤ 5 cm (median snapshot) DAN M-base > 5 cm.
- **D604** U4: tanda median residu depth botol sama dengan tanda (rasio−1) U2 untuk kamera yang sama.

## Batas
Satu penempatan FIT; U2 mengasumsikan lantai datar z=0 (lantai pernah diukur operator z0, kerataan tidak diukur);
rasio U2 mencampur galat ekstrinsik dan skala depth — dipisahkan hanya bila pola terhadap baris citra konsisten satu model.
