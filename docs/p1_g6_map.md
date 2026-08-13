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

## B. Hasil — diukur SESUDAH §A dikunci

> Data mentah: `/tmp/g6_field.jsonl` (G1–G5), `/tmp/g6_grow.jsonl` (§B6).
> Awan: `/tmp/topo_cloud_{a,b}.npz`. Peta: `/tmp/topo_static_{a,b}.npz`.
> Perkakas: `test/analyze_field_capture.py`, `test/bench_bl_grow.py`.

### B0. Penangkapan — G1 LULUS untuk keduanya

Dua penangkapan, 2026-08-13, jarak ~36 menit, parameter identik kecuali nama
berkas keluaran (diverifikasi dari baris perintah node, bukan dari niat).

| | A (21:36) | B (22:07) |
|---|---|---|
| titik ditangkap | 80.848 | 82.511 (+2,1%) |
| pool fit | 12.000 | 12.000 |
| bbox span (m) | 3,142 × 2,966 × 1,729 | 3,141 × 2,968 × 1,729 |
| node / sisi | 1800 / 3152 | 1800 / 3136 |
| wall-clock node | 1224 s | 1562 s |

Ambang G1 (≥3000 titik, bbox ≥1,0 m di x dan y) terlampaui jauh. bbox kedua
penangkapan sepakat sampai **2 mm** — konsisten dengan adegan yang memang diam.

Pool = tepat 12.000 = `fit_max_points`, jadi seperti yang diantisipasi §A2,
replay D2 **tidak** men-subsampel ulang: permutasi baris menguji jalur fit itu
sendiri, tanpa lapisan subsampel di depannya.

### B1. G3 — LANTAI DERAU SENSOR (diukur lebih dulu, sesuai §A3)

| `cloud_a ↔ cloud_b` | nilai |
|---|---|
| mean-NN | **0,0235 m** |
| hausdorff | 0,8330 m |

mean-NN 2,35 cm ≈ leaf voxel 2 cm. Sesuai peringatan §A2, angka ini **bukan**
akurasi depth D455 — ia terkuantisasi oleh voxel. Hausdorff 0,83 m berarti ada
titik di satu penangkapan tanpa pasangan dekat di penangkapan lain: bagian
adegan yang tertangkap sekali saja, bukan pergeseran sistematis.

### B2. G2 — D2 LAPANGAN: **LULUS**. Ini gate-nya, dan ini hasil utama sesi.

| | GNG | MS-BL-GNG |
|---|---|---|
| bit-identik setelah permutasi baris | ❌ | ✅ |
| `max\|ΔW\|` | **2,712 m** | **0,0** |
| drift hausdorff | 0,2052 m | **0,0000 m** |

Klaim yang sekarang boleh masuk naskah **dengan angka lapangan**, bukan proksi:
*"peta statis tidak bergantung pada urutan penyajian sampel"*.

Dua hal yang layak dicatat:

1. **Masalahnya nyata di lapangan, bukan artefak proksi.** `max|ΔW|` GNG di awan
   nyata **2,712 m** — bahkan lebih besar dari 2,53 m di proksi Sesi C. Node bisa
   mendarat di sisi ruangan yang lain hanya karena baris masukan berbeda urutan.
2. **Lulusnya sekaligus membuktikan round-trip `SAVE_CLOUD`.** Yang dibandingkan
   adalah peta **tersimpan** (`topo_static_a.npz`) melawan fit ulang atas awan
   **tersimpan** yang dipermutasi. Bit-identik hanya mungkin kalau awan yang
   disimpan memang awan yang menghasilkan peta itu. Jadi pasangan awan/peta di
   disk terverifikasi, bukan diasumsikan — dan itu yang membuat sesi-sesi
   berikutnya tidak perlu kamera lagi.

### B3. G4 — D3 LAPANGAN (dilaporkan tanpa ambang, sesuai §A3/G4)

| | GNG | MS-BL |
|---|---|---|
| D3-recapture mean-NN | 0,0478 | **0,0377** |
| D3-recapture hausdorff | **0,5750** | 0,7947 ⚠ |
| D3-95% mean-NN | 0,0257 | **0,0206** |
| D3-95% hausdorff | 0,2600 | **0,1715** |
| rasio drift peta ÷ derau sensor (mean-NN) | 2,035× | **1,606×** |

**Atribusi, memakai aturan baca yang ditetapkan §A3/G3 sebelum angkanya terlihat:**
kedua algoritma punya `M_ab > S_ab`, jadi **keduanya memperkuat derau sensor** —
bukan hanya meneruskannya. MS-BL memperkuat **lebih sedikit** (1,61× vs 2,04×).
Tidak ada gate di sini, dan tidak ada yang ditetapkan setelah melihat angkanya.

⚠️ **MS-BL tidak unggul seragam, dan ini tidak dihaluskan.** Pola Sesi C terulang
persis di lapangan: MS-BL menggeser **seluruh** peta lebih sedikit (mean-NN
menang di kedua bentuk D3), tapi node **terjauhnya** bisa bergeser lebih banyak
(hausdorff kalah di D3-recapture). Di Sesi C ini muncul di S1 dan S4; di lapangan
muncul di recapture. Artinya konsisten: yang membaik adalah perilaku massal, yang
memburuk adalah ekor.

### B4. G5 — PARITAS KUALITAS DI AWAN NYATA: **LULUS ketiga ambangnya**

Ambang dari `p1_g5 §A1/C2`, dipakai apa adanya, `cloud_a` yang sama.

| | GNG | MS-BL | ambang | |
|---|---|---|---|---|
| QE_mean | 0,03521 | **0,03048** | ≤ 1,05× → **0,866×** | ✅ |
| cov@5cm | 0,8661 | **0,9453** | ≥ −0,02 → **+0,079** | ✅ |
| n_nodes | 1800 | 1800 | ≥ 1710 | ✅ |
| QE_median | 0,03469 | 0,02979 | — | |
| cov@2cm | 0,1336 | 0,1922 | — | |
| spacing median | 0,0624 | 0,0612 | — | |
| n_edges | 3666 | 3152 | — | |
| n_comp | 5 | 10 | **tanpa ground truth** | — |

`n_comp` **tidak boleh** disebut perbaikan maupun kemunduran (§A0). Ini menutup
utang `p1_next_steps` Jalur B-4 dengan jawaban **"tidak bisa dijawab untuk adegan
nyata"**, bukan dengan angka. Yang punya ground truth adalah S1 sintetis (harus
2), dan di sana MS-BL benar (`p1_g5 §B3`).

### B5. Proksi vs lapangan — apakah Sesi C menyesatkan?

| | proksi (`p1_g5 §B1`, 400 node) | lapangan (1800 node) |
|---|---|---|
| D2 GNG `max\|ΔW\|` | 2,53 m | **2,712 m** |
| D2 MS-BL | bit-identik | **bit-identik** |
| D3 hausdorff GNG→MS-BL | 0,2927 → 0,1680 | 0,2600 → **0,1715** (D3-95%) |
| D3 mean-NN GNG→MS-BL | 0,0533 → 0,0504 | 0,0257 → **0,0206** (D3-95%) |

**Proksi Sesi C ternyata memprediksi arah dengan benar di keempat baris**, dan
besarannya sepadan untuk D3-95%. Jadi proksi itu **tidak** menyesatkan — tapi
perlu dicatat bahwa itu baru diketahui **sekarang**, setelah diukur. Angka proksi
tetap tidak boleh dikutip sebagai hasil lapangan; yang berubah adalah kita punya
alasan untuk mempercayainya sebagai alat eksplorasi.

### B6. Biaya — dan koreksi terhadap pembingkaian §B7b Sesi C

Setelan produksi, awan nyata, `grow=1`: MS-BL **1406,7 s**, GNG **53,4 s**
(GNG di sini lebih lambat dari 25,5 s §B7b karena pool nyata 12.000 titik dengan
23 epoch auto, bukan pool proksi).

`p1_g5 §B7` menyebut perlambatan ini "struktural". Itu benar tapi terlalu longgar,
dan sempat terbaca seolah batch learning itu sendiri yang mahal. **Yang mahal
adalah kadens pertumbuhan**, dan itu terverifikasi di sumbernya:

```c
// Meso-HSR/GNG.h, akhir MS_GNG_learning
if (GNGinfo[m].ngn < GNGinfo[m].maxNeuron)   //  add neuron per every it iteration
    GNG_add(m);
```

Satu batch = satu node → tumbuh ke 1800 node butuh **1800 lintasan data**,
sementara GNG online menyisipkan tiap `lam=100` sampel **di dalam** satu lintasan
(23 epoch cukup). Bandingkan kerjanya: ~1,3e10 vs ~4,6e8 evaluasi jarak (~28×),
terukur 26×. Portnya **setia**; yang mahal adalah kadensnya.

Konsekuensi yang tidak berubah: **online tetap lebih murah dengan MS-BL** — di
`env_gng` satu tick = satu batch, jadi ~800 `step()` Python diganti satu operasi
matriks (`p1_g5 §B7`).

**`fit(grow=k)` — diukur, menjawab utang Jalur B-3:**

| grow | detik | percepatan | QE_mean | cov@5cm | n_nodes | D2 bit-identik |
|---|---|---|---|---|---|---|
| 1 | 1406,7 | 1,0× | 0,03048 | 0,945 | 1800 | ✅ |
| 4 | 370,2 | 3,8× | 0,03071 (+0,8%) | 0,940 | 1800 | ✅ |
| 8 | 189,4 | 7,4× | 0,03087 (+1,3%) | 0,941 | 1800 | ✅ |
| 16 | **99,7** | **14,1×** | 0,03099 (+1,7%) | 0,936 | 1800 | ✅ |

Biaya ~linier terhadap `1/grow` (14,1× pada k=16 dari ideal 16×), QE bergerak
<2%, `n_nodes` tetap 1800, dan **D2 bertahan di setiap k** — yang menentukan,
karena D2-lah alasan MS-BL ada di sini. 23 menit → 100 detik masih lulus kedua
ambang paritas §A1/C2 dengan margin.

⚠️ Angka `grow` diukur **sementara RViz jalan**, jadi sedikit pesimis. Biasnya
melawan klaim percepatan, bukan mendukungnya.

**Jalur produksi tetap `grow=1`** dan seluruh angka §B1–§B5 memakai `grow=1`.
Tabel ini menyatakan perubahan itu **tersedia dan aman**, bukan bahwa ia sudah
diambil — memilih default adalah keputusan naskah.

Tersangka berikutnya, **belum diukur**: `learn_batch` mengalokasi matriks
ko-aktivasi `(n, n)` per batch (26 MB pada n=1800) lalu memindainya dengan
`np.triu`. Bisa jadi porsi besar sisa waktunya, dan bisa hilang tanpa mengubah
hasil sama sekali.

### B7. Yang TIDAK dikerjakan, dan kenapa

- **Ekstrinsik tidak disentuh** (§A0). Tidak ada alasan, dan itu jam kerja hilang.
- **Akurasi geometri peta terhadap ruangan tidak diukur** — tidak ada ground
  truth terukur untuk adegan ini (§A4).
- **Paritas action map / IK tidak diulang** — itu C3 Sesi C.
- **`grow` tidak diubah di jalur produksi** (§B6).
- **Optimasi `(n,n)` tidak dikerjakan** — ditemukan saat gate sedang diukur;
  menyentuh jalur fit saat itu akan mencemari angka yang sedang diambil.

### B8. Papan skor §7.2 — dugaan ke-9, dan polanya AKHIRNYA putus

Delapan dugaan sebelumnya meleset, **semuanya** ke arah menduga kendala lebih
mengikat daripada kenyataannya.

**Ke-9 (sesi ini):** "penangkapan ulang di lapangan akan jauh lebih berantakan
daripada proksi; D2 lapangan adalah taruhan yang sesungguhnya." **Diukur: D2
lulus bit-identik, dan proksi memprediksi arah dengan benar di keempat baris
(§B5).** Jadi kali ini dugaannya menaksir masalah **lebih mengikat** juga — pola
yang sama — tapi hasilnya **bukan** kejutan yang membalik kesimpulan: ia
mengonfirmasi.

Yang layak dibawa: dugaan ini pun diturunkan dari **intuisi tentang lapangan**,
bukan dari jalur data kode. Pembeda yang sama seperti Sesi A tetap berlaku —
dugaan yang tepat datang dari menelusuri variabel (`params.seed`, `GNG_add` di
akhir batch), bukan dari menakar keketatan.
