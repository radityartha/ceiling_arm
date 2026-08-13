# P1 / G5 — Port MS-BL-GNG (2 layer lingkungan) + GCS (action map)

> Sesi C, 2026-08-13. Melanjutkan [p1_g4_reach_dwell.md](p1_g4_reach_dwell.md).
> Prompt tugas: [p1_prompt_gcs_msbl.md](p1_prompt_gcs_msbl.md).
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode port dijalankan** — disiplin
> yang sama seperti G3/G4. §B diisi sesudah. Kalau §B bertentangan dengan §A,
> yang menang **§B**, dan pertentangannya ditulis eksplisit, bukan dihaluskan.
>
> Yang dipakai dari `p1_plan.md`: **hanya** §2b–§2e, §3 "Utang teknis metode",
> §6, §7. §1/§3-lapisan/§4 dan angka §2 **tidak dipakai**.
>
> Tanpa perangkat keras. Kamera/lengan/gantry **tidak** dinyalakan untuk ini.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Ruang lingkup — apa yang disentuh dan apa yang TIDAK

| Tempat | Sekarang | Jadi | Berkas |
|---|---|---|---|
| Layer lingkungan STATIS | GNG Fritzke online | **MS-BL-GNG** | `map_topo_static.py`, `topo_static_pub.py` |
| Layer lingkungan DINAMIS | GNG Fritzke online | **MS-BL-GNG** | `env_gng.py` |
| Action map `xyz → q` | GNG Fritzke online | **GCS** (+ perbaikan simpleks) | `train.py` → `arm{1..4}_gcs*.npz` |
| **Index capability** | **grid** | **TETAP grid** | `capability.py` — **JANGAN DISENTUH** |

`gng.py` **tidak diubah sama sekali**. File baru: `bl_gng.py`, `gcs.py`.

### A1. Kriteria sukses — TERKUNCI 2026-08-13, sebelum melihat hasil apa pun

#### C1 — DETERMINISME (kemenangan utama; harus diukur SEBELUM dan SESUDAH)

Tiga tingkat, dipisah karena artinya berbeda dan mudah dirancukan:

| ID | Perlakuan | Yang ditanya | Lulus bila |
|---|---|---|---|
| **D1** | Pool titik **identik**, urutan baris **identik**, fit 2×. | Apakah ada sumber acak yang tidak ter-seed? | **Bit-identik** (`W`, `edges`) untuk **kedua** algoritma. Ini batas paling lemah; kalau GNG pun lulus D1, katakan begitu. |
| **D2** | Pool titik **sama**, **urutan baris dipermutasi**. | Ini kondisi NYATA: penangkapan ulang mengembalikan titik yang sama dalam urutan lain. | **MS-BL-GNG: bit-identik** setelah pengurutan kanonik node (permutation-invariant). **GNG: diukur dan dilaporkan apa adanya**, tidak diharapkan lulus. |
| **D3** | Pool disubsampel ulang 95%. | Kestabilan terhadap gangguan data nyata (jitter sensor). | **Tidak ada ambang lulus/gagal.** Dilaporkan sebagai jarak Hausdorff + rata-rata NN antar dua himpunan node. Angka MS-BL harus **≤** angka GNG; kalau tidak, katakan. |

**D2 adalah kriteria yang menentukan.** Klaim yang boleh masuk naskah hanya:
"peta statis tidak bergantung pada urutan penyajian sampel", bukan "peta statis
identik antar penangkapan" — penangkapan berbeda berarti awan titik berbeda (D3),
dan tidak ada algoritma yang membuat itu identik.

**Metrik pembanding node antar dua peta** (dipakai D2/D3), ditetapkan sekarang:
```
node_set_equal : sama jumlah node DAN max|W_a[sort] - W_b[sort]| == 0.0   (bit-identik)
hausdorff      : max( max_i min_j ||a_i-b_j||, max_j min_i ||b_j-a_i|| )   [m]
mean_nn        : mean_i min_j ||a_i-b_j||   (simetris, dirata-rata dua arah) [m]
```

#### C2 — PARITAS KUALITAS PETA (metrik ditetapkan SEKARANG, sebelum port)

Diukur pada pool yang sama, untuk GNG (sebelum) dan MS-BL-GNG (sesudah):

```
QE_mean    rata-rata jarak titik-data ke node terdekat            [m]   <- utama
QE_median  median dari hal yang sama                              [m]
cov@2cm    fraksi titik data yang punya node dalam 0.02 m         [-]
cov@5cm    fraksi titik data yang punya node dalam 0.05 m         [-]
n_nodes    jumlah node akhir                                      [-]
spacing    median jarak node-ke-node-terdekat                     [m]
n_edges    jumlah sisi                                            [-]
n_comp     jumlah komponen terhubung                              [-]
```

**Ambang lulus (dikunci):**
- `QE_mean(MS-BL) ≤ 1.05 × QE_mean(GNG)` — boleh 5% lebih buruk, tidak lebih.
- `cov@5cm(MS-BL) ≥ cov@5cm(GNG) − 0.02` (2 poin absolut).
- `n_nodes(MS-BL) ≥ 0.95 × max_nodes` — kalau peta baru tidak tumbuh, portnya gagal.

`spacing`, `n_edges`, `n_comp` **dilaporkan tanpa ambang** — konteks, bukan gate.

#### C3 — PARITAS ACTION MAP (pakai metrik yang SUDAH ADA, jangan bikin baru)

`eval.py ik` sudah membandingkan strategi seeding atas pose reachable held-out.
Tambahkan `gcs` sebagai strategi keempat; laporkan **berdampingan** dengan `gng`:
success rate · mean/median waktu solve · mean manipulability.

- Ambang: `success_rate(gcs) ≥ success_rate(gng) − 0.02`, dan
  `median_ms(gcs) ≤ 1.25 × median_ms(gng)`.
- ⚠️ Sub-perintah `ik` **butuh `move_group` hidup** (`/compute_ik`). Kalau
  `move_group` tidak dijalankan, laporkan **"paritas IK BELUM DIUJI"**.
  **Dilarang** mengganti dengan metrik proksi dan menyebutnya paritas IK.
- Proksi offline **boleh** dilaporkan sebagai tambahan, **dengan label proksi**:
  jarak seed-ke-`q` benar pada pasangan held-out (`seed_err`), karena itu yang
  sebenarnya menentukan apakah seed membantu solver — tapi itu **bukan** C3.

#### C4 — INVARIAN SIMPLEKS GCS (bukti, bukan klaim)

Invarian: **setelah setiap `add`**, node baru `g` yang disisipkan antara `h` dan `k`
tersambung ke **setiap tetangga bersama** `h` dan `k`.

```
∀ i ∉ {h,k} :  (i~h) ∧ (i~k)  ⟹  (i~g)
```

Tes harus **GAGAL** pada implementasi tanpa perbaikan (`simplex_repair=False`,
persis blok yang dikomentari di `Meso-HSR/GNG.h:634-640`) dan **LULUS** dengan
perbaikan. Satu tes, dua mode — kalau mode "tanpa perbaikan" ikut lulus, berarti
tesnya tidak menguji apa pun dan harus dibuang.

#### C5 — TIDAK ADA REGRESI PADA JALUR LAMA

- `git diff` pada `reachability_gng/gng.py` **kosong**.
- `test/test_gng.py` tetap hijau.
- `/tmp/arm{1..4}_model.npz` tetap termuat lewat `GNG.load` dan `seed_q` tetap
  memberi hasil yang sama (baseline ablation).
- Keluaran GCS memakai nama berkas **tersendiri** (`arm{n}_gcsx.npz`), bukan
  menimpa `arm{n}_model.npz` **maupun** `arm{n}_gcs.npz` (yang terakhir sudah ada
  di `/tmp` dari eksperimen 2026-08-02 dan **bukan** keluaran tugas ini).

### A2. Ground truth dulu (§7.1) — data uji ditetapkan sekarang

Port diuji terhadap struktur yang **sudah diketahui jawabannya**, sebelum
dilepas ke awan titik nyata:

| Data | Struktur | Yang harus terjadi |
|---|---|---|
| `S1` dua gugus Gauss terpisah 1.0 m | 2 komponen | jaring pecah jadi **2 komponen terhubung**, tidak ada node menggantung di ruang kosong antara gugus |
| `S2` permukaan bidang 1×1 m, z=0 | 2-manifold | node tersebar di bidang, `QE_mean` turun monoton terhadap jumlah node |
| `S3` kulit bola r=0.5 m | 2-manifold tertutup | `‖node‖` semua ≈ 0.5 (dilaporkan: mean ± std deviasi radius) |
| `S4` derau seragam dalam kubus | tanpa struktur | tidak boleh runtuh; node ≈ tersebar seragam |

Awan nyata: **tanpa kamera**. Dipakai proksi dari peta statis nyata yang tersimpan
(`/tmp/topo_static.npz`, ditangkap 2026-08-02 dari adegan nyata) — node-nya
disebar-ulang jadi awan titik. **Ini proksi distribusi, BUKAN penangkapan ulang**,
dan harus disebut begitu di setiap tempat angkanya muncul.

### A3. Yang TIDAK diklaim

- MS-BL-GNG dan GCS **bukan kontribusi** — dikutip: Ardilla, Saputra, Kubota
  (IJAT 17(3):206–216, 2023); Fritzke (Growing Cell Structures).
- Perbaikan invarian simpleks adalah **GCS Fritzke standar**, jadi juga **bukan
  kontribusi** — tapi wajib supaya nama "GCS" di naskah cocok dengan yang jalan.
- Determinisme yang diklaim = **invarian terhadap urutan sampel (D2)**, bukan
  "peta identik antar penangkapan" (itu D3, dan itu mustahil).
- Index capability **tetap grid** dan **tidak diukur ulang** di sesi ini.

### A4. Konstanta multi-scale — TIDAK ADA di repo, jadi ditetapkan dan dicatat

`MS_GNG_learning` memakai `MSN[u]` (langkah subsampel per level) dan `MSNO[k]`
(ambang jumlah node untuk naik level). **Deklarasi ketiganya tidak ada di mana pun
di repo ini** (`grep -rn "maxMSN" .` → kosong; `GNG.h` memakainya tapi tidak
mendeklarasikannya). Jadi nilainya **tidak bisa diport, harus ditetapkan**, dan
itu dicatat di sini supaya tidak terbaca sebagai hasil port.

Semantik yang terbaca dari kode (`GNG.h:960-961`, `1442-1451`):
`MSN[u]` = langkah; batch pada level `u`, offset `v`, mengambil
`MSID[v], MSID[v+MSN[u]], …` → yaitu **1/MSN[u] bagian data**. `MSD` berputar
`0..MSN[u]-1` sehingga batch berurutan menutupi seluruh data. Naik level saat
`ngn > MSNO[k]`.

Ditetapkan: `MSN = (8, 4, 2, 1)`, `MSNO` = ambang node pada 25%/50%/75% dari
`max_nodes`. Arah: **kasar → halus** (batch kecil saat node sedikit, batch penuh
saat jaring besar). Level terakhir `MSN=1` = **full batch**, dan itu yang membuat
D2 bisa lulus.

**Penyimpangan sadar dari sumber:** `GNG_MS_DATA` mengacak dengan `rnd()`.
Diganti **permutasi kanonik yang diturunkan dari isi data** (lexsort atas vektor),
bukan RNG. Alasannya langsung ke C1/D2: dengan `rnd()`, partisi mini-batch
bergantung pada indeks baris, jadi permutasi masukan mengubah hasil. Ini
**rekayasa determinisme**, bukan klaim algoritmik.

### A5. Kendala integrasi yang mengikat rancangan

`gcs.py` **wajib** API-kompatibel dengan `GNG` — 10 berkas konsumen memuat action
map (`seed_ik`, `seed_server`, `gantry_reach_executor`, `reach_fusion`,
`visualize`, `eval`, `reachability_check`, `reachability_cloud`,
`topo_static_pub`, `env_gng`). Permukaan minimum: `load()` (classmethod),
`query(task_vec,k)`, `query_radius(task_vec,radius,max_k)`, `.W`, `.task_dim`,
`.seed_q()`, `.save()`. Sama untuk `bl_gng.py` terhadap konsumen peta statis
(`.W`, `._edges`, `.pinned`, `.save/load`).

### A6. Pemilihan sumber port — DIBANDINGKAN, bukan diasumsikan

Prompt memperingatkan ada tiga versi `nRobot.h` berbeda. **Sudah dibandingkan**
(`awk '/void GCS_add/,/^}/'` atas ketiganya): `GCS_add` di `FRD-01`,
`Artha-HSR-01/ori`, `Artha-HSR-01/Artha-HSR-01`, dan `Meso-HSR` **identik secara
fungsional** — satu-satunya beda adalah `printf` yang dikomentari. Jadi pilihan
`FRD-01` (yang dikutip plan) **tidak berkonsekuensi**, dan itu terverifikasi,
bukan diterima.

---

## B. Hasil — diisi SESUDAH §A dikunci

*(kosong saat §A dikunci)*
