# Diagnosis luring F5/D 2026-10-15 — depth rgbd berkedip, botol bias +8 cm, ray-plane

**Luring saja:** 70 snapshot NPZ arsip `diagnosis_20261014_F5D_01` (45 dua-kamera + 25 rgbd-only). NOL kamera/stack/gerak.
Protokol dikunci sebelum kode: [A_locked.md](A_locked.md), sha256 `ae080482…` (commit b870cf9). Tidak mengubah raw/skor/
kunci lama, TF, launch, atau kode produksi. F7/C3/C6 holdout tidak dipakai. **Satu penempatan FIT; informasional,
kalibrasi tetap belum diterima, DEMO-HW tetap belum siap.**

Cek deproyeksi: `rgbd_000` instance 3 mereproduksi `median_world` localizer persis (n 1498, (0.6677, −0.5101, 0.1092)).

## Temuan utama

1. **Depth rgbd BERKEDIP per frame, bukan wilayah statis.** Deret validitas (dua kamera, %): 70 19 51 16 1 70 0 70 1 1 70 1
   17 15 1 70 1 18 70 0 50 70 1 52; rgbd-only sama polanya (1 72 2 19 36 1 70 …). rgbd2 konstan **78 %** di 21/21 frame.
   Frame "antara" bukan pita horizontal (bukan pola paket USB hilang): yang tersisa hanya area bertekstur
   ([flicker_rgbd.png](flicker_rgbd.png)) — cocok dengan **emitter IR mati/lemah atau bergantian** (passive stereo hanya di
   tekstur), lalu frame mati 0–1 %. Mematikan rgbd2 tidak memulihkan → bukan interferensi antar-kamera. Kedua kamera
   USB 3.2; rgbd lewat hub 2 tingkat (port 2-2.1.1), rgbd2 langsung (2-1). Launch tidak menyetel `emitter_enabled`,
   `emitter_on_off`, `laser_power`, `visual_preset` (default driver). Penyebab pasti (opsi emitter tersimpan, laser
   rusak/termal, hub) **belum terbukti** — perlu cek kamera hidup pasif.
2. **Permukaan botol terbaca ~+8 cm lebih jauh di KEDUA kamera.** Piksel interior (sinar ≤ 0.5 R dari sumbu silinder GT,
   tepi dibuang, [u4_interior.json](u4_interior.json)): rgbd2 **+8.42 cm, IQR 0.8 cm**, 21/21 frame di Z 1.88 m; rgbd
   +8.9 / +8.1 cm (dua run) di Z 2.39 m. Lantai pada jarak sama hanya rasio 1.001–1.011 (≤ ~2 cm). Dua kamera dari sisi
   berlawanan sama-sama lebih jauh → bukan geseran posisi botol; konsisten dengan **material botol (putih tembus IR /
   hamburan bawah permukaan)**. Mekanisme fisik belum diuji. Akibat: setiap centroid depth botol tergeser ~8 cm
   **sepanjang sinar kamera**, ditambah kontaminasi mask penyangga (B61) — fit M6/M7 kampanye lama harus menyerap bias
   ini (konsisten dengan geser kamera 0.13–0.25 m dan skala 0.89–0.96 yang gagal gerbang; tidak dibuktikan kausal).
3. **Ray-plane RGB saja (M-top) akurat untuk rgbd2: 0.22 cm** (median 21 frame) — tidak bergantung depth. Untuk rgbd
   5.17 cm, sama dengan M-base 4.99 cm → galat rgbd di F5 ada di **ekstrinsik/geometri RGB**, bukan hanya depth.
   M-ctr (centroid piksel mask ∩ z 0.2225) buruk (6.5–9.4 cm) karena mask ikut box penyangga.
4. **Lantai (U2):** tilt bidang terukur vs z=0: rgbd 1.9°/2.7°, rgbd2 2.6°; offset −1.5…−3.1 cm. Rasio d_ukur/d_prediksi
   rgbd2 naik dengan jarak (1.001 @1.5–2 m → 1.027 @2.5–3 → 1.062 @4–4.5 → 1.072 @4.5–5 m); rgbd turun pelan
   (1.017 → 0.98, rgbd-only 0.955 @4.5–5 m). Satu bidang tidak memisahkan pitch ekstrinsik vs bias disparitas (batas §A).
   Catatan: rgbd2 di F5 (Z 1.9 m) akurat 2 mm lewat RGB, jadi kemiringan 2.6° bukan rotasi kamera seragam yang memindah
   semua titik; bias depth jarak-jauh rgbd2 lebih mungkin di sisi depth — **dugaan, bukan bukti**.

## Papan skor (harfiah)

| Dugaan | Hasil | Catatan |
|---|---|---|
| D600 peta invalid rgbd stabil IoU ≥ 0.7 | **✓** (0.807 / 0.883) | Lolos harfiah, **tafsiran salah**: median peta ≈ frame mati, kedipan per frame membuat IoU tinggi secara trivial. Penyebab temporal, bukan statis. Uji dirancang buruk. |
| D601 rgbd2 \|rasio−1\| > 0.03 @ ≥ 2.5 m | **✓** (gabungan 1.0324, n 740 845) | Tipis. Teks §A ambigu per-bin vs gabungan; per-bin 2.5–3.0 = 1.027 (<0.03), bin lain > 0.03. RANSAC RNG berbeda dari run utama ([d601_pooled.log](d601_pooled.log)). |
| D602 rgbd \|rasio−1\| ≤ 0.03 semua bin ≥ 1000 px | **✗** | dual lolos (0.982–1.018), rgbd-only bin 4.5–5.0 = 0.955 (n 1608). |
| D603 rgbd2 M-top ≤ 5 cm DAN M-base > 5 cm | **✗** | M-top 0.22 cm ✓, tapi M-base 0.60 cm (xy rgbd2 di F5 tidak rusak; bias penyangga/depth terutama di z). |
| D604 tanda residu botol = tanda (rasio−1) | **✓** (+/+ kedua kamera) | Tidak informatif: besaran 8 cm vs ≤ 2 cm lantai → bias khusus botol. Bin yang dibandingkan (Z botol) tidak tertulis di §A. |

**3 ✓ / 2 ✗.** Penyimpangan dari §A (ditulis): (a) `svd` pertama memakai `full_matrices=True` → OOM 64 GB, rc 137, tidak
ada keluaran; diperbaiki `full_matrices=False` dan diulang (bug implementasi, metode sama). (b) Tambahan di luar §A:
U4 interior ([u4_interior_check.py](u4_interior_check.py)), deret waktu validitas + dua gambar
([validity.png](validity.png), [flicker_rgbd.png](flicker_rgbd.png)), D601 gabungan.

## Implikasi untuk perbaikan kalibrasi

- **Botol + depth tidak layak sebagai alat ukur kalibrasi** (bias material +8 cm + mask penyangga + depth rgbd berkedip).
  Kampanye baru sebaiknya memakai target RGB-saja (tag/ChArUco, PnP) dengan ground truth FK lengan — dibahas dengan
  operator (tag di gripper).
- **rgbd perlu dicek kesehatan depth** sebelum dipakai untuk apa pun: cek hidup pasif (tanpa gerak) opsi
  `depth_module.emitter_enabled`, `emitter_on_off`, `laser_power`, firmware, citra IR (pola titik projector), dan uji
  tanpa hub. Butuh kamera hidup → sesi di lokasi, gerbang biasa.
- Lokalisasi demo: ray-plane RGB pada tinggi tumpuan diketahui menghindari bias depth botol; sah hanya bila ekstrinsik
  benar (rgbd di F5 masih 5 cm).
- Bias depth jarak-jauh rgbd2 (+3…7 % di 2.5–5 m) diukur ulang dengan target RGB pada beberapa jarak sebelum memutuskan
  on-chip/Tare calibration; jangan memasang skala via TF.

## Berkas

`A_locked.md/.sha256`, `analyze_offline.py` → `offline_result.json`, `analyze_offline.log`; `u4_interior_check.py` →
`u4_interior.json`, `.log`; `d601_pooled.py` → `.log`; `fig_validity.py` → `validity.png`; `fig_flicker.py` →
`flicker_rgbd.png`. Raw NPZ tetap di `diagnosis_20261014_F5D_01` (lokal, tidak di git).
