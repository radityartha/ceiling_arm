# P1 / G6 — MAP: peta statis dari kamera NYATA

> Sesi G6, 2026-08-13. Melanjutkan [p1_g5_msbl_gcs.md](p1_g5_msbl_gcs.md) —
> Jalur C di [p1_next_steps.md](p1_next_steps.md).
>
> **§A ditulis dan DIKUNCI SEBELUM kamera dinyalakan.** §B diisi sesudah.
> Kalau §B bertentangan dengan §A, yang menang **§B**, dan pertentangannya
> ditulis eksplisit.
>
> Tujuan tunggal sesi ini: mengubah angka determinisme Sesi C dari **proksi**
> (titik yang disebar di sekitar node peta 2026-08-02) menjadi **angka
> lapangan** (awan titik yang benar-benar keluar dari dua D455).

---

## A. Protokol — DIKUNCI SEBELUM MENANGKAP APA PUN

### A0. Yang TIDAK dibuka ulang

Terkunci di tempat lain, dipakai apa adanya, **tidak ditulis ulang di sini**:

| Hal | Terkunci di | Status |
|---|---|---|
| Definisi D1 / D2 / D3 | `p1_g5 §A1/C1` | dipakai apa adanya |
| Metrik `node_set_equal` / `hausdorff` / `mean_nn` | `p1_g5 §A1` | dipakai apa adanya |
| Ambang paritas kualitas (QE, cov@5cm, n_nodes) | `p1_g5 §A1/C2` | dipakai apa adanya |
| Index capability = grid | `p1_state §5.2` | tidak disentuh |
| `gng.py` | `p1_g5 §A0` | nol perubahan |
| Ekstrinsik kamera (2026-07-30) | memori sesi | **tidak disentuh** |

`n_comp` **tidak punya ground truth** untuk adegan nyata. Dilaporkan sebagai
konteks; **dilarang** disebut perbaikan maupun kemunduran. (Ini menutup utang
`p1_next_steps` Jalur B-4 dengan jawaban "tidak bisa dijawab", bukan dengan angka.)

### A1. Keadaan fisik saat penangkapan — dicatat karena menentukan tafsir

| | Status 2026-08-13 21:26 |
|---|---|
| Area kerja | dibersihkan (diverifikasi pengguna) |
| Lengan 4× | 🔴 **DILEPAS FISIK** |
| Kamera 2× D455 | 🟢 terpasang (`234222303079`, `241122302297`), ekstrinsik 2026-07-30 |
| Gantry | diam |

**`SELF_FILTER=false`, dan alasannya ada dua — yang kedua yang penting:**

1. Lengan tidak ada, jadi tidak ada yang perlu difilter.
2. `robot_state_publisher` **sedang jalan** (PID 1247632, sejak 17:34) dan
   tetap menyiarkan TF keempat lengan dari URDF. Jadi TF lengan **resolve**
   walaupun lengannya tidak ada. Dengan `self_filter=true`, filter itu akan
   melubangi awan **nyata** di lokasi lengan **hantu**. Jadi `false` di sini
   bukan sekadar "boleh", tapi **wajib**.

`my_workcell.launch.py` **tidak** dijalankan: satu-satunya alasan `build_topo.sh`
menyebutnya prereq adalah TF self-filter 4 lengan, dan itu justru yang tidak
diinginkan sekarang. Ini penangkapan **terbersih** yang mungkin: tidak ada lengan
untuk difilter, jadi tidak ada peluang filter itu salah.

### A2. Apa persisnya yang disimpan — ditulis sekarang supaya angkanya tidak salah baca

`SAVE_CLOUD` menyimpan **pool yang dipakai fit**, bukan awan sensor mentah.
Urutannya di `map_topo_static`: crop (`min_z`/`max_z`/`max_x_from_camera`) →
`_voxel_downsample(leaf=0.02)` → `_radius_outlier_removal` → gabung antar-frame →
voxel lagi → (kalau > `fit_max_points=12000`) subsampel dari pool **terurut
kanonik**. Yang tersimpan adalah hasil akhir itu, plus `captured` = jumlah titik
sebelum subsampel.

Konsekuensi yang harus disebut di setiap tempat angkanya muncul: **"jarak
awan_a ↔ awan_b" adalah jarak antar pool ter-voxel 2 cm, bukan antar awan sensor
mentah.** Lantai derau resolusinya ~2 cm karena itu.

Konsekuensi yang menguntungkan: kalau pool tersimpan tepat 12000 titik, replay D2
**tidak** akan men-subsampel ulang (`0 < fit_max_points < len(pool)` salah), jadi
permutasi baris menguji jalur fit itu sendiri, tanpa lapisan subsampel di depannya.

### A3. Kriteria — hanya yang khas lapangan

#### G1 — VALIDITAS PENANGKAPAN (dicek SEBELUM analisis apa pun)

Kedua berkas awan **wajib ada dan berisi**. Penangkapan sah bila:

- `len(cloud) ≥ 3000` titik, **dan**
- rentang bbox ≥ 1.0 m pada x **dan** y (kalau tidak, kamera cuma melihat satu
  bidang sempit dan "peta statis" itu bukan peta ruangan).

Kalau gagal: **LAPORKAN dan BERHENTI.** Dilarang menambal dengan proksi lalu
menyebutnya lapangan. Selisih jumlah titik antara a dan b **dilaporkan sebagai
angka**, bukan sebagai gate — dua penangkapan adegan diam boleh beda.

#### G2 — D2 LAPANGAN: **INI SATU-SATUNYA GATE LULUS/GAGAL**

```
W_a    = fit_static_map_bl(cloud_a)
W_perm = fit_static_map_bl(cloud_a[permutasi acak baris])
LULUS bila node_set_equal(W_a, W_perm) == (True, 0.0)
```

Pool **identik** (awan nyata yang sama), hanya urutan barisnya diacak. Setelan
produksi: `max_nodes=1800`, `fit_max_points=12000` — sama dengan yang node pakai,
bukan setelan benchmark 400-node.

GNG dijalankan pada awan yang **sama** sebagai pembanding, **tanpa** diharapkan
lulus (persis seperti `p1_g5 §A1/C1`). Kalau GNG ikut lulus di awan nyata,
katakan begitu.

Ini gate karena inilah satu-satunya klaim naskah yang bergantung pada sesi ini:
*"peta statis tidak bergantung pada urutan penyajian sampel"* — dan sampai
sekarang klaim itu hanya pernah diuji di proksi.

#### G3 — PISAHKAN DERAU SENSOR DARI DRIFT ALGORITMA

Diukur **lebih dulu**, sebelum peta manapun dibandingkan. Tanpa ini, drift peta
tidak bisa diatribusikan ke sensor atau ke algoritma.

```
S_ab = map_distance(cloud_a, cloud_b)     # hausdorff + mean_nn, metrik §A1 g5
M_ab = map_distance(W_a, W_b)             # dua peta dari dua penangkapan
```

**Cara pelaporan ditetapkan sekarang** (rasio dilaporkan, bukan diambangi):

| Bila | Dibaca sebagai |
|---|---|
| `M_ab.mean_nn ≲ S_ab.mean_nn` | drift peta **didominasi derau sensor**; algoritma tidak menambah drift |
| `M_ab.mean_nn ≫ S_ab.mean_nn` | algoritma **memperkuat** derau sensor — ini temuan, wajib disebut |

Rasio `M_ab / S_ab` dilaporkan untuk **kedua** algoritma pada awan yang sama.
Tidak ada ambang: tidak ada ground truth yang membolehkannya.

#### G4 — D3 LAPANGAN: dilaporkan TANPA AMBANG

Dua bentuk, dua arti, tidak boleh dicampur:

- **D3-recapture** = `M_ab` di atas — dua penangkapan nyata terpisah. Ini D3
  lapangan yang sebenarnya, dan yang tidak akan pernah bisa diukur dari proksi.
- **D3-95%** = subsampel 95% dari `cloud_a`, fit ulang, bandingkan dengan `W_a` —
  definisi D3 `p1_g5 §A1` apa adanya, supaya bisa disandingkan langsung dengan
  baris `proxy` di `p1_g5 §B1`.

Keduanya untuk GNG **dan** MS-BL. Arah yang diharapkan (`MS-BL ≤ GNG`) tetap
seperti `p1_g5 §A1`, tapi **tidak ada gate**: tidak ada ground truth untuk adegan
nyata, dan ambang **tidak boleh** ditetapkan setelah melihat angkanya.

#### G5 — PARITAS KUALITAS DI AWAN NYATA

Metrik dan ambang **sudah ada** di `p1_g5 §A1/C2`, dipakai apa adanya, pada
`cloud_a` yang sama, setelan produksi:

- `QE_mean(MS-BL) ≤ 1.05 × QE_mean(GNG)`
- `cov@5cm(MS-BL) ≥ cov@5cm(GNG) − 0.02`
- `n_nodes(MS-BL) ≥ 0.95 × max_nodes` (≥1710 dari 1800)

`spacing`, `n_edges`, `n_comp`, dan waktu fit dilaporkan **tanpa ambang**.

### A4. Yang TIDAK diklaim di sesi ini

- **Bukan** "peta identik antar penangkapan". Itu D3, dan itu mustahil —
  penangkapan berbeda = awan titik berbeda (§G3/G4).
- **Bukan** klaim tentang jumlah komponen adegan nyata (§A0).
- **Bukan** klaim akurasi metrik/geometri peta terhadap ruangan sebenarnya.
  Tidak ada ground truth terukur untuk adegan ini; ekstrinsik 2026-07-30 tidak
  divalidasi ulang di sesi ini dan tidak disentuh.
- **Bukan** paritas action map / IK. Itu C3 Sesi C, tidak diulang di sini.

### A5. Perkakas — apa yang ditulis, apa yang dipakai ulang

Dipakai ulang tanpa diubah: `map_distance`, `node_set_equal`, `quality`,
`n_components`, `fit` dari `test/bench_topo_determinism.py`; `fit_static_map` /
`fit_static_map_bl` dari `map_topo_static.py` (jalur kode yang sama dengan yang
node kirim).

Baru: `test/analyze_field_capture.py` — hanya memuat dua `.npz` awan, menjalankan
G1–G5, dan menulis JSON. Tidak ada metrik baru yang didefinisikan di sana.

---

## B. Hasil — diisi SESUDAH §A dikunci

_(kosong sampai penangkapan selesai)_
