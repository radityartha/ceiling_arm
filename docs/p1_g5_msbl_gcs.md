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

## B. Hasil — diukur SESUDAH §A dikunci

> Semua angka di bawah dari `test/bench_topo_determinism.py`,
> `test/validate_bl_gng.py`, `test/test_gcs.py`, dan `eval.py ik`.
> Data mentah: `/tmp/g5_bench.jsonl`, `/tmp/g5_ik_arm{1..4}.csv`.

### B0. Yang berubah

| Berkas | Perubahan |
|---|---|
| `bl_gng.py` | **BARU** — MS-BL-GNG |
| `gcs.py` | **BARU** — GCS + perbaikan simpleks |
| `map_topo_static.py` | `fit_static_map()` dipisah (pemindahan murni, agar terukur) + `fit_static_map_bl()`; node memakai yang BL |
| `topo_static_pub.py` | `GNG.load` → `BLGNG.load` |
| `env_gng.py` | `GNG`→`BLGNG`; loop `step()` per titik → satu `partial_fit()` per tick |
| `train.py` | `--algo {gng,gcs}`, default **gng** (baseline tidak berubah) |
| `eval.py` | strategi seeding `gcs` + `--gcs-model` |
| `build_maps.sh` | `ALGO=gcs` → `arm{n}_gcsx.npz`; dataset dipakai ulang |
| `gng.py` | **NOL perubahan** (`git diff` kosong) |

### B1. C1 — DETERMINISME

#### 🔴 Dugaan yang meleset — DUGAAN KE-8, dan lagi-lagi ke arah yang sama

Prompt (dan `p1_plan.md §3`) menyatakan: *"peta statis sekarang berubah tiap
run"*. **Diukur: TIDAK.** `gng.py` memakai `np.random.default_rng(params.seed)`
dengan `seed=0`, jadi atas pool **identik dengan urutan identik** GNG sudah
**bit-identik** (D1 = True, delta = 0.0) — di kelima adegan.

Yang benar-benar gagal adalah **D2**: pool titik yang **sama** disajikan dalam
**urutan baris berbeda** menghasilkan peta berbeda. Itu justru kondisi nyata,
karena penangkapan ulang mengembalikan titik yang sama dalam urutan lain.

Perbedaannya penting untuk naskah: klaim yang boleh ditulis adalah
**"tidak invarian terhadap urutan penyajian"**, bukan "tidak deterministik".
Yang kedua salah dan mudah dibantah reviewer yang membaca kode.

Konsisten dengan papan skor §7.2 p1_state: dugaan menaksir masalah **lebih
mengikat** daripada kenyataannya. Sekarang **delapan** dari delapan.

#### Angka

| Adegan | D1 (urutan sama) | D2 (baris dipermutasi) | D3 hausdorff (m) | D3 mean-NN (m) |
|---|---|---|---|---|
| | GNG → MS-BL | GNG → MS-BL | GNG → MS-BL | GNG → MS-BL |
| proxy | ✅ → ✅ | **❌ → ✅** | 0.2927 → **0.1680** | 0.0533 → **0.0504** |
| S1 dua gugus | ✅ → ✅ | **❌ → ✅** | 0.0874 → 0.1065 ⚠️ | 0.0346 → **0.0312** |
| S2 bidang | ✅ → ✅ | **❌ → ✅** | 0.0434 → **0.0387** | 0.0168 → **0.0148** |
| S3 kulit bola | ✅ → ✅ | **❌ → ✅** | 0.0762 → **0.0637** | 0.0297 → **0.0271** |
| S4 derau | ✅ → ✅ | **❌ → ✅** | 0.1084 → 0.1228 ⚠️ | 0.0529 → **0.0463** |

**D2 LULUS di kelima adegan**: MS-BL bit-identik (delta **0.0**, drift
hausdorff **0.0000 m**) terhadap permutasi baris masukan. GNG gagal di kelimanya
— drift hausdorff **0.087–0.294 m**, dengan `max|ΔW|` sampai **2.53 m** pada
proxy (yaitu node bisa mendarat di sisi ruangan yang lain).

**D3 (subsampel 95%) tidak dijanjikan lulus, dan memang campur.** MS-BL menang
pada **mean-NN di kelima adegan** dan pada hausdorff di 3 dari 5; kalah hausdorff
di S1 dan S4 (⚠️). Artinya: MS-BL menggeser **seluruh** peta lebih sedikit, tapi
node **terjauhnya** bisa bergeser lebih banyak pada distribusi tanpa struktur.
Dilaporkan apa adanya, tidak dihaluskan.

### B2. C2 — PARITAS KUALITAS: **LULUS, dan lebih baik, di kelima adegan**

`max_nodes=400`, `lam=100`, pool sama, jalur kode yang sama dengan yang dikirim.

| Adegan | QE_mean (m) | rasio | cov@5cm | cov@2cm | node | sisi | komponen |
|---|---|---|---|---|---|---|---|
| proxy | 0.0642 → **0.0594** | **0.926×** | 0.327 → **0.381** | 0.032 → 0.041 | 400 → 400 | 805 → 708 | 2 → 3 ⚠️ |
| S1 | 0.0409 → **0.0379** | **0.926×** | 0.799 → **0.830** | 0.098 → 0.123 | 400 → 400 | 1666 → 1411 | 2 → 2 ✅ |
| S2 | 0.0182 → **0.0174** | **0.957×** | 1.000 → 1.000 | 0.589 → 0.626 | 400 → 400 | 980 → 869 | 1 → 1 |
| S3 | 0.0318 → **0.0305** | **0.957×** | 0.925 → **0.943** | 0.194 → 0.217 | 400 → 400 | 1012 → 932 | 1 → 1 |
| S4 | 0.0632 → **0.0593** | **0.939×** | 0.277 → **0.320** | 0.017 → 0.021 | 400 → 400 | 1507 → 1301 | 1 → 1 |

Ambang §A1 menuntut `QE ≤ 1.05×` dan `cov@5cm ≥ −0.02`; hasilnya **0.926–0.957×**
dan cov naik di semua adegan. `n_nodes` = 400/400 di semua adegan (ambang ≥380).

⚠️ **Satu hal yang harus disebut, bukan disembunyikan:** pada `proxy`, MS-BL
menghasilkan **3** komponen terhubung, GNG **2**. Peta proxy memang bukan satu
benda tersambung (lantai + meja + fixture), jadi 3 tidak otomatis salah — tapi
**tidak ada ground truth** untuk jumlah komponen adegan nyata, jadi ini
**tidak bisa disebut perbaikan maupun kemunduran**. Yang punya ground truth
adalah S1 (harus 2), dan di sana MS-BL **benar**.

### B3. §A2 — VALIDASI TERHADAP GROUND TRUTH SINTETIS: **semua lulus**

`test/validate_bl_gng.py`, dijalankan untuk **kedua** algoritma:

| Cek | GNG | MS-BL |
|---|---|---|
| S1 tepat 2 komponen | ✅ 2 | ✅ 2 |
| S1 tidak ada node di celah kosong (0.3<x<0.7) | ✅ 0 | ✅ 0 |
| S2 QE turun monoton terhadap jumlah node | ✅ 0.0558→0.0387→0.0271→0.0182 | ✅ 0.0549→0.0385→0.0259→**0.0174** |
| S3 node berada di kulit bola r=0.5 | ✅ 0.4936 ± 0.0013 | ✅ **0.4963 ± 0.0008** |
| S4 tidak runtuh | ✅ bbox [0.878 0.883 0.865] | ✅ bbox [0.919 0.909 0.890] |

### B4. C3 — PARITAS ACTION MAP: **DIUJI SUNGGUHAN**, bukan proksi

`move_group` **dijalankan** (`my_workcell.launch.py use_fake_hardware:=true`),
`/compute_ik` hidup, 500 pose reachable held-out per lengan, seed 0.
Model GCS dilatih dengan **resep identik** dengan baseline GNG lewat
`ALGO=gcs build_maps.sh` (`max_nodes=3000`, `lam=60`, `epochs=2`,
`boundary=600`) — hasilnya 3000 node / 600 pinned pada **keduanya**, jadi
perbedaan yang terukur adalah jaringnya, bukan setelannya.

| Lengan | success GCS | success GNG | median ms GCS | median ms GNG |
|---|---|---|---|---|
| arm1 | **90.2%** | 89.4% | 1.45 | 1.46 |
| arm2 | **90.2%** | 89.0% | 1.45 | 1.46 |
| arm3 | **91.4%** | 89.8% | 1.49 | 1.48 |
| arm4 | **89.0%** | 88.8% | 1.58 | 1.53 |

Ambang §A1 (`success ≥ GNG − 0.02`, `median ≤ 1.25× GNG`): **LULUS di keempat
lengan**, dan GCS sedikit **di atas** GNG di keempatnya (+0.2 … +1.6 poin).
Sisi jaring: GCS 21.3k–21.4k vs GNG 15.8k–16.0k — konsisten dengan simpleks yang
ditutup, bukan graf biasa.

#### 🔴 Temuan yang TIDAK diminta tapi wajib dilaporkan

Pada benchmark yang sama, seed **`none`** (vektor nol) mengalahkan **kedua**
action map, dan `random` (10 restart) mengalahkan semuanya:

| Lengan | gcs | gng | **none** | random (10×) |
|---|---|---|---|---|
| arm1 | 90.2% | 89.4% | **94.8%** | 98.4% |
| arm2 | 90.2% | 89.0% | **93.4%** | 98.4% |
| arm3 | 91.4% | 89.8% | **95.4%** | 98.6% |
| arm4 | 89.0% | 88.8% | **94.6%** | 98.2% |

Jadi **action map tidak membantu KDL IK pada benchmark ini** — ~4–6 poin di
bawah seed nol, untuk GNG maupun GCS. Ini **bukan regresi dari sesi ini**
(GNG baseline sama buruknya) dan **bukan bagian dari C3**, tapi ini klaim yang
bisa langsung diserang reviewer kalau naskah menyebut action map sebagai
pemercepat IK. `random` juga tidak sebanding lurus: ia dapat **10** percobaan
per pose, sementara gcs/gng hanya **1**.

➜ **Harus diselesaikan sebelum naskah mengklaim manfaat seeding.** Bukan di
sesi ini: memperbaikinya berarti mengubah metrik/desain benchmark, dan §A1
mengunci "pakai metrik yang sudah ada, jangan bikin baru".

⚠️ Kolom `mean manip` benchmark **tidak informatif**: 1.0e-07 untuk arm1/3/4
(semua metode identik sampai 3 angka penting). Nilai wajar hanya muncul di arm2
(0.196–0.232). Jadi manipulability **tidak dipakai** untuk menyimpulkan apa pun
di sini. Penyebabnya belum dicari — di luar lingkup, dicatat sebagai utang.

### B5. C4 — INVARIAN SIMPLEKS: **terbukti, bukan diklaim**

`test/test_gcs.py`, fit identik dijalankan dua kali, invarian diperiksa setelah
**setiap** `add` (60 add eksplisit; `add_every` dinaikkan agar `step()` tidak
menambah node diam-diam sehingga hitungannya eksak):

| | pelanggaran | node | sisi |
|---|---|---|---|
| `simplex_repair=False` (persis sumber sensei) | **91** | 63 | 152 |
| `simplex_repair=True` (blok GNG.h:634-640 dipulihkan) | **0** | 63 | **254** |

Tes yang gagal pada implementasi belum-diperbaiki **memang gagal**, dan lulus
setelah diperbaiki. Perbaikannya **hanya menambah sisi**, tidak menghapus/
memindah apa pun (`test_repair_only_adds_edges`).

### B6. C5 — TIDAK ADA REGRESI: **lulus**

- `git diff ros2_ws/src/reachability_gng/reachability_gng/gng.py` → **kosong**.
- `pytest test/test_gng.py test/test_gcs.py` → **10 lulus**.
- `/tmp/arm{1..4}_model.npz` tetap termuat lewat `GNG.load`, `seed_q` berjalan,
  3000 node / 15.8k–16.0k sisi utuh.
- Tata-letak npz sama, jadi `GNG.load` dan `BLGNG.load` bisa saling membaca —
  **10 berkas konsumen tidak perlu diubah sama sekali** (kendala integrasi
  prompt terpenuhi).
- Keluaran GCS di `arm{n}_gcsx.npz`; `arm{n}_model.npz` **dan**
  `arm{n}_gcs.npz` (eksperimen 2026-08-02) keduanya tidak tersentuh.

### B7. Biaya — MS-BL lebih lambat, dan itu harus disebut

Pada `max_nodes=400`: GNG **2.6–3.2 s**, MS-BL **34.0–38.7 s** → **~12× lebih
lambat**. Sebabnya struktural: MS-BL menambah **satu node per batch**, jadi
biayanya ≈ `max_nodes × |batch| × n_node`, sementara GNG menyisipkan setiap
`lam` sampel.

#### B7b. Setelan produksi — DIUKUR, bukan diekstrapolasi

`map_topo_static` sebenarnya berjalan pada `max_nodes=1800`,
`fit_max_points=12000`. Diukur langsung di sana (mesin sepi, pool proxy sama):

| | node | sisi | QE_mean | waktu |
|---|---|---|---|---|
| GNG | 1800 | 3220 | 0.0292 m | **25.5 s** |
| MS-BL-GNG | 1800 | 2732 | **0.0270 m** | **1033.5 s** (17.2 menit) |

➜ **40× lebih lambat pada setelan produksi**, bukan 12×. Perlambatannya
**superlinier** terhadap `max_nodes`, persis seperti yang diduga strukturnya
(`max_nodes × |batch| × n_node`), jadi ekstrapolasi dari angka 400-node akan
**meleset 3×** — alasan lain untuk mengukur.

Apakah ini bisa diterima? Untuk `map_topo_static` **ya**: ia jalan **sekali**,
offline, saat adegan statis ditangkap. 17 menit sekali seumur tata-letak, ditukar
dengan peta yang tidak berubah saat awan titik yang sama disajikan ulang.
Tapi angkanya harus **disebut**, bukan disembunyikan, dan kalau nanti
`max_nodes` dinaikkan, biayanya naik kuadratik.

Yang **tidak** terpengaruh: `env_gng` (online). Di sana satu tick persepsi = satu
batch, jadi 800 panggilan `step()` Python diganti **satu** operasi matriks —
lebih murah, bukan lebih mahal. Diverifikasi langsung dengan mengonstruksi node
dan menyuapinya awan sintetis (tanpa kamera): 12 tick → 98 node, 234 sisi, dan
peta hidupnya **identik** ketika awan yang sama disajikan dalam urutan
teracak.

> ⚠️ Invariansi urutan di `env_gng` **tidak gratis** dan sempat gagal: subsampel
> per-tick semula memilih berdasarkan **indeks baris**, sehingga awan yang
> dipermutasi memilih titik berbeda. Diperbaiki dengan menarik sampel dari pool
> terurut-konten, sama seperti `fit_static_map_bl`. Jebakan yang sama muncul dua
> kali di dua tempat berbeda — layak diingat kalau ada jalur ketiga ditambahkan.

### B8. Yang TIDAK dikerjakan, dan kenapa

- **Index capability tidak disentuh** — terkunci grid (§5.2 p1_state).
- **Peta statis nyata belum dibangun ulang** dari kamera: sesi ini tanpa
  perangkat keras, sesuai instruksi. `/tmp/topo_static.npz` masih peta GNG
  2026-08-02. Awan "nyata" di seluruh §B adalah **proksi** dari node peta itu —
  distribusi realistis, **bukan** penangkapan ulang.
  ➜ Menjalankan `map_topo_static` dengan kamera nyata adalah langkah berikutnya
  yang wajib sebelum angka D2 dikutip sebagai hasil di lapangan.
- **Action map GCS belum menggantikan GNG di jalur runtime.** `train.py --algo`
  default tetap `gng`, dan konsumen masih memuat `arm{n}_model.npz`. Itu
  disengaja: keduanya adalah dua sisi ablation, dan pemilihannya keputusan
  naskah, bukan keputusan port.
