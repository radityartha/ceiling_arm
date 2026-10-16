# P1 / G63d — papan polos STATIS berjangkar pita: TIDAK LAYAK → arah baru M3 (ArUco besar + stereo papan genggam)

> Sesi 2026-10-16/17 (prompt [p1_g63d_static_prompt.md](p1_g63d_static_prompt.md)). **LURING SAJA**: NOL gerak lengan/rel, NOL
> kamera, NOL mock. Alat: [results/p1_g63/g63d/](results/p1_g63/g63d/). **§A TIDAK DIKUNCI** (prasyarat tugas 1–2 gagal).
> Lanjutan: sesi baru [p1_g63e_m3_prompt.md](p1_g63e_m3_prompt.md).

## A0. Jawaban operator (harfiah, tombol / chat)

| # | Pertanyaan | Jawaban |
|---|---|---|
| 1 | Ukuran kotak cetak | "sepertinya sekarng jadi kurang lebih 41 mm" (angka terukur 4/5 kotak WAJIB sebelum solve) |
| 2 | Papan di jari arm_2/arm_3 | "Dilepas (Rekomendasi)" — **belum dilepas**, papan masih di jari |
| 3 | Pengukur + alat | "Operator, meteran+unting" |
| 4 | Lokasi tempel | "Meja kerja (z 0.770), Lantai (z 0), Atas lemari laci, meja tinggi mepet tembok di zona gantry 2" |
| 5 | Unting dari basis REST | "Bisa dari tepi basis" (2 tanda berlawanan → titik tengah = sumbu j1) |
| 6 | Metode pita | "Trilaterasi (Rekomendasi)" |
| 7 | Tinggi meja tinggi | "98 cm" |
| 8 | 7–8 papan / laci dipindah + kotak A/B / §A | "1. boleh 2. boleh 3boleh" — "3" = persetujuan di muka; TIDAK dipakai mengunci (§A tidak lulus prasyarat) |
| 9 | Arah sesudah analisis metode | "oke kita coba" M3, mulai di sesi baru |

Jangkar: sumbu j1 di REST, g1 1.40 / g2 0.0009, rot 0/0 (FK live2.urdf): P1 (1.800, 0.360), P2 (1.000, 0.360),
P3 (0.401, −0.360), P4 (−0.399, −0.360); z sumbu j1 1.824. Laci sekarang ≈ x 0.65–0.98, y 0.40–0.72, atas ~0.70
(triangulasi dua kamera frame 10-16, ±5 cm) — tepat di bawah arm_2 REST.

## B. Hasil luring

### B1. Batas deteksi papan polos nyata (frame HW jari 10-16, detektor terkunci diulang)
Sisi kotak terproyeksi terkecil: sukses terkecil **9.2 px**; SEMUA ≤ 8.8 px gagal "SB tidak menemukan grid" (5.8/7.4/7.6/8.8 px);
kegagalan lain = muka membelakangi. Sukses terjauh 2.18 m. → kriteria V6 ≥ 9.5 px. 2.5 m TIDAK dilonggarkan ke 3.0 m.

### B2. Rencana papan statis (`g63d_plan.py`: V1 bingkai+margin, V2 ≤ 2.5 m, V3 ≤ 55°, V4 masker lengan REST, V5 sinar env,
V6 ≥ 9.5 px, oklusi antar-platform, stok kotak A/B/laci = 1)
- Meja kerja tak terpakai (rgbd luar bingkai; rgbd2 > 2.5 m, > 55°, tertutup lengan). Meja tinggi tak terpakai (rgbd > 55°; rgbd2
  terhalang benda/rak z 1.09–1.11). Laci di tempat: rgbd2 tertutup arm_3 REST.
- Lantai rgbd (margin 10 cm) hanya x 1.10–1.35, y −0.40…−0.15.
- **6 papan: tidak layak** di margin 10 cm; margin 5 cm + oklusi platform: 6/7/8 papan tidak layak (8: rgbd 4, rgbd2 3).
- Pemindaian rel saat tangkap (hitungan saja; geser rel = gerak HW): g1/g2 = 0/0, 0/1.4, 0.7/0.7, 1.6/0 tidak layak; 1.6/1.4 layak
  tetapi prediksi kamera 1.3–1.5° / 54–59 mm ✗.

### B3. Uji mandiri M1 (rencana dev margin 5 cm pra-oklusi, 20 seed + 20 negatif 3 cm, blur 1.3–1.6 px, latar frame lama)
Galat kamera median 0.71/0.66°, 28 mm (maks 2.56° / 108 mm); titik ruang kerja med 9–11 mm, p95 terburuk 33/66 mm. Deteksi papan
jauh 3–64 % frame → 27/40 kamera-seed < 4 papan. LOBO nominal p95 42 mm vs negatif min 36 mm → **tidak ada ambang G3** (T 35:
11/13 dan 6/6; T 40: 12/13 dan 5/6); papan salah teridentifikasi 1/6. G4 PnP satu papan 38 mm nominal → diganti triangulasi
(belum diuji).

### B4. Perbandingan metode (`g63d_methods.py`, MC 300, pita 3 mm/0.5°, sudut 0.3 px, rel 1.40/0)

| Metode | rot kamera med | posisi med | titik med / p95 |
|---|---|---|---|
| M1 papan catur 41 mm statis | 0.66–0.71° | 28 mm | 9–11 / 33–66 mm (✗ kontrol negatif) |
| M2 ArUco 18 cm / A4 statis + pita, 8–9 penanda | 0.29–0.39° | 12–17 mm | 4.3–5.6 / 15–24 mm |
| **M3** = M2 + stereo papan polos DIPEGANG operator (25 pose) | **0.19–0.23°** | **7–9 mm** | **3.4–3.9 / 10–13 mm** |

Stereo papan genggam: pose relatif rgbd→rgbd2 0.06° / 1.9 mm (median), p95 0.14° / 4.7 mm.
Batas bukti: MC M2/M3 TIDAK memodelkan kegagalan deteksi ArUco (G63: marker ~22 px gagal sebagian → syarat aman ≥ 40 px, MC pakai
30 px); kontrol negatif M2/M3 belum; pemilihan penanda kasar (rgbd hanya 3–4); permukaan vertikal belum dihitung.
Tidak cocok: laser pointer Svoboda (≥ 3 kamera), depth D455 sebagai ukuran (≤ 2 % RMS, bias botol +8 cm), lengan sebagai acuan
(galat kinematik; berguna sesudah kamera beku → S-B).

### B5. Papan skor (draf dugaan, tidak dikunci)
| # | Dugaan | Hasil |
|---|---|---|
| D646 | Rencana 6 papan memenuhi V1–V6, margin 10 cm | ✗ |
| D647 | Ada ambang G3: nominal ≥ 95 % lulus dan negatif 3 cm 100 % tertangkap | ✗ (rencana dev) |
| D648 | Titik ruang kerja med ≤ 10 mm, p95 ≤ 30 mm | ✗ (rencana dev: p95 33/66 mm) |

**Tidak dijalankan:** HW/kamera/mock; `g63d_capture.py` belum diuji dengan ROS; G4 triangulasi belum diuji; S-B.
Keadaan sel tidak disentuh: papan di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack mati.
