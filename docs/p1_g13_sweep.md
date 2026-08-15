# P1 / G13 — REACH-3: menghabiskan sapuan, lalu menutup oracle.

> Sesi G13, 2026-08-16. Melanjutkan [p1_g12_armsolve.md](p1_g12_armsolve.md).
> G12 menaruh lengan **ke dalam solver**, meluluskan M0/M1/M2b/M4/M5, dan
> mengukur `Δ_arm = 0.0000` **TERBUKTI** pada 36/40 (120 s) dan 39/40 (600 s)
> pada `c_arm = 0.05`. Dua hal yang G12 **tidak** punya: angka di luar
> `c_arm = 0.05` (S2 dan sapuan **TIDAK DIUKUR**, `p1_g12 §B10` poin 4), dan
> oracle yang bisa membedakan solver longgar dari ketat (**M3 GAGAL**,
> `p1_g12 §B8`).
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.** §B
> diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**. §A tidak ditulis ulang belakangan.
>
> Sesi ini **sepenuhnya offline**. Lengan 4× masih dilepas.
>
> **CATATAN MESIN, di muka:** `load average` awal sesi **2.98 / 3.45 / 3.44**
> pada 16 core. G12 diukur pada 4.5–5.0, G11 pada 3.3–3.7, G10 pada 42. Detik
> **tidak** dibandingkan lintas sesi tanpa menyebut ini.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang, dan yang BEKU

Seluruh `p1_g12 §A0` berlaku utuh. Yang ditambahkan sebagai **sudah tegak**:

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| `solve_armfull` + `ArmCtx` + `arm_conflict` + `arm_serial_ub` | `p1_g12 §B2`/`§B8` | apa adanya, **tidak** dibangun ulang |
| Gerbang M0 (31/31 W2, 39/40 S1), M1 (0/40), M2b (0/1864), M4 (0/40), M5 (route 25/5/10) | `p1_g12 §B8` | **tidak dijalankan ulang kecuali sebagai regresi yang disebut** |
| Lemma C LULUS: 107 perhentian, 0 tanpa pose parkir, min 83 % \|P\| bebas | `p1_g12 §B6` | dasar bahwa `arm_serial_ub` selalu mengembalikan sesuatu |
| Tutup 60 pose G11 **DICABUT**; `arm_serial_ub` menyeluruh = 0.23 s | `p1_g12 §B6` | **jangan dipasang lagi** |
| Lemma B **BENAR sebagai teorema, DILEPAS sebagai mekanisme** (tebing 540× di `c_clear`) | `p1_g12 §B4` | kedua sisi jalan di `c_clear = 0.0` |
| `Δ_struct` S1 = [0, 0.4679] BUKAN MAHAL; `Δ_arm` = 0.0000 pada 36/40 @120 s, 39/40 @600 s, `c_arm = 0.05` | `p1_g12 §B7` | **regresi + basis pembanding**, tidak dihitung ulang |
| U3 LUNAS: `n4_s7_mr0` = 39.3041 = LB, `proved`, @600 s | `p1_g12 §B7.2` | tutup |
| Cloud kanonik `data/irm_cloud_pol.npz`, provenansi BIT-IDENTIK | `p1_g12 §B1` | apa adanya |
| `feasible_starts` **tidak** dilewati `U` → substitusi predikat mustahil | `p1_g12 §B2` | **jangan diturunkan ulang** |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi:**
`sched.py`, `sched_heur.py`, `sched_coll.py`, `sched_coupled.py`,
`sched_arm.py`, **`sched_armfull.py`**, `test/verify_sched_exact.py`,
`test/verify_sched_coll.py`, `test/verify_sched_coupled.py`,
`test/gate_sched_coupled.py`, `test/verify_sched_arm.py`,
`test/eval_sched_arm.py`, **`test/verify_sched_armfull.py`**.

⚠️ `sched_armfull.py` dan `verify_sched_armfull.py` **masuk daftar beku sesi
ini**: setiap angka G13 dibaca terhadap solver dan oracle yang sama persis
dengan yang G12 luluskan. Kode M3 sesi ini **mengimpor** `brute_arm`,
`gen_small_armreal`, dan `arm_binding_fraction` dari berkas beku, **tidak
menyalinnya** — supaya "instance mengikat" diukur oleh enumerator yang identik
dengan yang melaporkan 0/18.

🔓 **BOLEH DIUBAH, dan hanya seperti ini:** `test/eval_sched_armfull.py` —
**satu** perubahan mekanis, `_path(tag)` → `_path(tag, c)` (satu berkas per
**(set, c_arm)**, bukan per set) plus `report()` yang menggabungkan berkas-berkas
itu dan mencatat `load average`. Alasannya diukur bukan diselerakan: sapuan ini
menjalankan beberapa sel **serentak** (A2.2), dan `p1_g12` sudah mencatat "dua
proses menulis satu JSON = saling menimpa". **Tidak ada perubahan pada apa yang
dihitung.** Palangnya: baris S1 `c_arm = 0.05` yang lama harus keluar dari
`report()` baru **identik** dengan `p1_g12 §B7.1` (regresi R0, A3).

Aturan `p1_g10 §A0` berlaku utuh: **bug di berkas beku dilaporkan sebagai
temuan, diperbaiki di tempatnya, pembekuannya dinyatakan gugur.** G10 tiga kali,
G11 nol, G12 dua kali (dua-duanya di `sched_arm.py`).

### A1. Yang dikerjakan sesi ini, BERURUTAN

Urutannya wajib dan **tidak dilompati**:

```
0. R0   regresi harness: report() baru mereproduksi p1_g12 B7.1 PERSIS
1. BIAYA SAPUAN sebagai ANGKA, diukur pada c_arm = 0.20  <- SEBELUM 40 instance
2. SAPUAN c_arm pada S1: {0.20, 0.10, 0.15, 0.00}
3. S2 penuh (gen_real_rotcrowded), c_arm = 0.05 lalu sapuan
4. n6_s4_mr0 -- KENAPA ia terkurung. Diagnosis, bukan anggaran lebih besar
5. M3 -- utang oracle (A2.3)
6. U4 -- W2 max_evade
```

**TIDAK dibangun, dan tidak boleh menyelinap masuk:** solver ketiga, heuristik
sadar-lengan, perencanaan gerak lengan, `T_fold ≠ 0`, kapsul/ketebalan tautan,
policy kanonik selain `manip`, pelonggaran anggaran 120 s untuk angka utama,
dan `c_arm` di luar himpunan sapuan.

### A2. KRITERIA YANG DIKUNCI SEBELUM KODE

#### A2.1 🔒 BIAYA SAPUAN SEBAGAI ANGKA — INI SESI KEENAM DENGAN JEBAKAN YANG SAMA

`p1_g8 §B4`, `p1_g9 §B3`, `p1_g10 §B3-1`, `p1_g11 §B4`, `p1_g12 §B4` — lima
sesi berturut-turut di mana **yang lambat ternyata pemeriksanya**, dan G12
adalah yang pertama di mana pengukuran-sebelum-sapuan **membalikkan** rancangan
§A. Jadi ini diukur lebih dulu, dengan aturan keputusan yang ditulis **sebelum
angkanya dilihat**.

**Yang diukur (probe, bukan sapuan):** wall satu instance **medium** (`n4_s0_mr0`)
dan satu instance **terburuk yang diketahui** (`n6_s4_mr0`) pada `c_arm = 0.20`,
anggaran 120 s, **bersama** jumlah panggilan gerbang lengan dan fraksi wall-nya.

Alasan `c_arm = 0.20` dan bukan yang lain: ia ujung sapuan, dan arah biayanya
**belum diukur sama sekali**. Dua mekanisme yang berlawanan dan tidak ada yang
sudah diukur — makin besar `c_arm`, makin banyak aksi **ditolak** (pohon
mengecil, lebih murah) tetapi makin sering `ctx.ok` dipanggil sebelum sebuah
aksi lolos, dan makin sulit menemukan incumbent (pohon membesar, lebih mahal).
`p1_g12 §B7.1` mengukur gerbang lengan = **38 % wall** pada `c_arm = 0.05`.

🔒 **ATURAN KEPUTUSAN, ditulis sebelum angkanya dilihat:**

| Terukur pada probe | Yang dilakukan |
|---|---|
| kedua probe **< 120 s** | sapuan penuh dijalankan apa adanya, konkurensi A2.2 |
| probe medium < 120 s, probe terburuk = 120 s | sama — itu **persis** perilaku `c_arm = 0.05` (wall mean 31.0 s, maks 120.1 s) |
| **probe medium menyentuh 120 s** | sapuan tetap dijalankan **penuh** — anggaran 120 s membatasi biayanya secara konstruksi pada 40 × 120 s = **80 menit per sel** — tetapi urutan sel A2.2 jadi **mengikat**, dan sel yang tidak tercapai dilaporkan **TIDAK DIUKUR**, bukan diperkirakan |
| probe **gagal/tidak mengembalikan jadwal** | itu **temuan**, ditulis sebagai judul, dan sapuan berhenti di situ sampai sebabnya diketahui |

🔴 **Instance DILARANG dibuang untuk menghemat waktu.** Yang boleh dikorbankan
adalah **sel** (kombinasi set × `c_arm`) yang **seluruhnya** tidak dijalankan
dan disebut namanya di §B. Sapuan sebagian atas 40 instance **tidak pernah**
dilaporkan sebagai baris sapuan.

#### A2.2 🔒 KONKURENSI DAN BEBAN MESIN — CONFOUND BARU, DIKUNCI DI MUKA

Ini pertama kali sebuah sesi P1 menjalankan sel **serentak**, dan konsekuensinya
ditulis sekarang supaya tidak ditemukan belakangan:

> Anggaran 120 s adalah **wall clock**. Menjalankan `k` sel serentak memberi
> tiap instance **lebih sedikit node** dalam 120 s-nya. Jumlah instance yang
> `proved` — yaitu **sisi `Δ_lo`** dari vonis dua-sisi — karena itu **turun
> bersama beban**. Membandingkan sel yang jalan pada konkurensi berbeda
> membandingkan dua hal yang berbeda.

🔒 Yang dikunci:

1. **Konkurensi TETAP = 3 sel** sepanjang seluruh sapuan. Tidak dinaikkan
   ketika terlihat lambat, tidak diturunkan ketika terlihat cepat.
2. Setiap sel mencatat `load average` **awal dan akhir**, dan angkanya masuk
   tabel §B. Sel tanpa catatan beban **tidak dilaporkan**.
3. Baris S1 `c_arm = 0.05` **TIDAK dijalankan ulang** — ia **dimigrasikan** dari
   `/tmp/g12_eval_s1.json` apa adanya, supaya angka utama G12 tidak bergeser
   karena mesin. Ia dilabeli **konkurensi 1** di tabel.
4. Karena (3), perbandingan lintas-`c_arm` **tidak sepenuhnya adil**. Kalau
   sapuan selesai dan waktu tersisa, satu **sel kontrol** dijalankan: S1
   `c_arm = 0.05` pada konkurensi 3, dilaporkan sebagai **baris terpisah**
   (pola `p1_g12 §B7.2`). Kalau tidak dijalankan: **TIDAK DIUKUR**, dan
   ketidakadilan perbandingannya tetap ditulis.

**Urutan sel, dikunci sekarang, ekstrem lebih dulu** — karena kalau waktu habis,
`c_arm = 0.20` memberi lebih banyak informasi daripada 0.10 atau 0.15:

```
S1: 0.20, 0.10, 0.15, 0.00        S2: 0.05, 0.20, 0.10, 0.15, 0.00
```

#### A2.3 🔒 M3 — APA YANG MEMBUAT SEBUAH INSTANCE "MENGIKAT", DITULIS SEBELUM MEMBANGUNNYA

Ini kunci kriteria nomor 2 dari prompt G13, dan seluruh gunanya adalah supaya
**"tidak bisa dibangun" bisa dibedakan dari "kriterianya digeser"**.

**DEFINISI, tidak berubah dari G12, ditulis ulang supaya pergeserannya
terdeteksi:**

```
BINDING(inst, c_arm)  <=>  brute_arm(inst, c_arm) > brute_arm(inst, +inf) + 1e-9
```

dengan `brute_arm` **diimpor** dari `test/verify_sched_armfull.py` yang beku —
diskretisasi yang sama (`step = 0.25`, `k < 6`), `c_clear = 0.0` yang sama, dan
uji lengan lewat gerbang **LAMBAT** `sa.arm_schedule_conflict` yang sama.
**Solver tidak masuk definisi sama sekali.**

**TUAS YANG BOLEH DIPUTAR** (dan G12 hanya memutar dua yang pertama):

| # | Tuas | Sudah dipakai G12? |
|---|---|---|
| 1 | \|P\| ∈ {2, 3, 4}, pose dipilih lewat **predikat lengan langsung** (`arm_binding_fraction`) | ✅ ya |
| 2 | ambang fraksi mengikat `need` | ✅ ya (0.02; tercapai 0.12–0.75) |
| 3 | **Tugas dipaksa per gantry** — tugas yang hanya **satu** gantry bisa jangkau, sehingga pekerjaan **tidak bisa berpindah** | ❌ **TIDAK** |
| 4 | **Keseimbangan beban** — jumlah tugas per gantry sama dan durasi dwell sebanding, sehingga makespan ditentukan **konkurensi**, bukan rantai serial satu gantry | ❌ **TIDAK** |
| 5 | Skala dwell (`inst.dwell`), dilaporkan | ❌ tidak |
| 6 | `c_arm` ∈ himpunan sapuan {0.05, 0.10, 0.15, 0.20}; instance mengikat dilaporkan **dengan `c_arm`-nya** dan **tidak** dipromosikan ke angka utama | ❌ hanya 0.05 |

**Alasan tuas 3 dan 4 ada di daftar, ditulis sebagai hipotesis SEBELUM diuji:**
mekanisme "harga nol" yang G12 ukur (`§B7.3` poin 2) adalah *ada tempat
menyingkir, dan menyingkir gratis*. Pada instance kecil ada **cara menyingkir
kedua** yang G12 tidak pernah tutup: **menunggu**. Kalau makespan ditentukan
oleh rantai serial satu gantry, gantry lain boleh diserialisasi **seluruhnya**
tanpa menaikkan `max`. Tuas 3 + 4 menutup jalan itu. Kalau setelah ditutup
optimum **tetap** tidak naik, temuannya jauh lebih kuat daripada 0/18 G12.

**YANG DILARANG — ini yang membuat "tidak bisa dibangun" bermakna:**

1. Mengubah definisi `BINDING`.
2. Mengubah diskretisasi `brute_arm` (grid waktu, `k`, `max_states`) untuk
   memanufaktur selisih.
3. Instance dengan pose/oracle **dikarang** tanpa provenansi peta kapabilitas —
   `ArmGeom` tidak terdefinisi di situ (`verify_sched_armfull.gen_small_armreal`
   sudah mencatat alasannya) dan hasilnya mengukur predikat yang tidak ada.
4. Menyebut instance "mengikat" berdasarkan **solver**, bukan brute force.
5. `c_arm` di luar himpunan sapuan.

**LANTAI USAHA — sebelum kalimat "tidak bisa dibangun" boleh ditulis:**

| | Minimum |
|---|---|
| kelas instance berbeda (kombinasi tuas 1, 3, 4, 5) | **≥ 5** |
| kandidat dibangkitkan dan disaring | **≥ 300** |
| instance yang benar-benar di-brute-force **dua arah** | **≥ 40** |
| tabel per kelas: kandidat, fraksi mengikat maks, jumlah brute-force, jumlah `BINDING` | **wajib** |

**DIAGNOSTIK YANG DILAPORKAN APA PUN HASILNYA** — ini yang mengubah "0" dari
ketiadaan menjadi **pengukuran**. Setiap instance yang di-brute-force
diklasifikasikan tiga arah:

| Kelas | Artinya |
|---|---|
| `vacuous` | optimum **tanpa** kendala lengan sudah bebas `ARM_BLOCK` → kendala tidak pernah menyentuh optimum |
| `rerouted` | **setiap** optimum tanpa-kendala terblokir lengan, tapi ada jadwal **bermakespan sama** yang bebas → **kendala mengikat, harganya nol** |
| `binding` | optimum **naik** |

Pembagian tiga arah itu **adalah kalimat naskahnya**, terlepas dari apakah
kolom ketiga nol. G12 melaporkan `0 binding` tanpa memisahkan `vacuous` dari
`rerouted`, dan dua-duanya berarti hal yang sangat berbeda.

#### A2.4 🔒 ANGGARAN

| | |
|---|---|
| **Angka utama** (S1 dan S2, setiap `c_arm`) | **120 s per instance, TERKUNCI** — `p1_g10 §K4`, tidak dilonggarkan |
| Anggaran lain | dilaporkan di **baris TERPISAH** dengan anggarannya disebut, pola `p1_g12 §B7.2` |
| M3 brute force | `max_states` = nilai beku `verify_sched_armfull`, wall dilaporkan |
| Diagnosis `n6_s4_mr0` (A1 langkah 4) | **diagnostik, bukan angka utama** — profil, bukan anggaran lebih besar |

🔴 **Vonis `TIDAK DAPAT DITENTUKAN` adalah hasil yang sah dan TIDAK dinaikkan
dengan menaikkan anggaran di baris utama.** Ambang **5.0 %** tidak digeser.
Aturan dua-sisi `p1_g10 §A3-K3` dipakai ulang apa adanya. **S1 dan S2 TIDAK
PERNAH dirata-ratakan bersama** — probe S2 membatasi **himpunan pose**
(`p1_g11 §B3`), dan setiap angkanya membawa kualifikasi itu.

#### A2.5 🔒 APA YANG DILAPORKAN KALAU LAGI-LAGI TIDAK SEMUANYA BISA DIBUKTIKAN

Kunci kriteria nomor 4. Per sel, **wajib**, terlepas dari hasilnya:

1. `Δ_struct`, `Δ_full`, `Δ_arm` sebagai kurungan dua-sisi + vonis.
2. Jumlah instance `exact` / `proved`, dan jumlah **TIDAK ADA JADWAL DITEMUKAN**
   (dengan anggarannya, dan **tidak pernah** disebut "tidak layak" —
   `p1_g12 §A2.4` tiga tingkat pelaporan berlaku utuh).
3. Jumlah jadwal yang gagal gerbang, per penjadwal. Target **0**.
4. `load average` awal/akhir sel dan konkurensinya (A2.2).
5. Sel yang **tidak dijalankan** disebut namanya sebagai kalimat
   **TIDAK DIUKUR**. Dilarang menulis "diperkirakan tidak berubah".
6. Lemma C diukur ulang pada setiap `c_arm` sapuan — ia yang menjamin
   `arm_serial_ub` mengembalikan sesuatu, dan G12 hanya mengukurnya di 0.05.
7. Papan skor A4, dinilai. Dugaan yang dinilai memakai solver yang belum lulus
   gerbangnya **ditandai TERTUNDA**, tidak dinilai.

### A3. PALANG — regresi yang harus lulus sebelum angka baru disebut

Tidak ada gerbang baru: M0/M1/M2b/M4/M5 sudah lulus di G12 dan solvernya beku.
Yang dituntut sesi ini adalah **regresi**, karena harness-nya disentuh:

| | Apa yang diadu | Kalau gagal |
|---|---|---|
| **R0** | `report()` baru atas baris S1 `c_arm = 0.05` yang dimigrasikan **identik** dengan `p1_g12 §B7.1`: `Δ_struct` [0, 0.4679], `Δ_full` [0, 7.6155], `Δ_arm` [+0, +7.1476], exact 36/40, route berubah 10/40 | harness-nya berubah semantik → **temuan**, sapuan berhenti |
| **R1** | M1 dijalankan ulang pada **setiap sel sapuan**, bukan sekali: 100 % jadwal lolos `validate_coupled` **dan** `sa.arm_schedule_conflict` | jadwal tidak sound pada `c_arm` itu → sel itu **tidak dilaporkan sebagai angka**, dilaporkan sebagai kegagalan gerbang |
| **R2** | M4 (Lemma 3, `ub ≥ lb`) pada setiap sel | idem |

R1 dan R2 sudah terpasang di dalam `eval_sched_armfull.run()` (kolom `gate` dan
`arm_slow`), jadi ini pernyataan bahwa kolomnya **dibaca**, bukan kode baru.

### A4. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor berdiri di **23 meleset, 10 tepat** (`p1_g12 §B9`).
Prior **DUNIA**: LONGGAR — dan contoh tandingan G11 (D21) **sudah
dikualifikasi** dan tidak bertahan; kendala boleh mengikat ketat dan tetap
berharga nol.
Prior **KODE SENDIRI**: meleset **dua sesi berturut-turut**, dua-duanya menduga
kode sendiri terlalu **LAMBAT** (D22 tebak 100× mahal → 3× murah; D25 tebak
≥ 5× → 0.02×). Sekarang **5 dari 7**. Tetap ditulis di sisi **pesimis**, tapi
arah melesetnya disebut di muka.

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D30** | Wall mean per instance pada `c_arm = 0.20` **≥ 2×** wall mean pada 0.05 (yaitu **≥ 62 s**), dan **≥ 8 dari 40** menyentuh anggaran 120 s | **kode sendiri** | ditulis pesimis dengan sengaja: 52.55 % pasangan `(pose, U)` terlarang di 0.20 (L4) → incumbent lebih sulit ditemukan, `arm_serial_ub` lebih sering jadi satu-satunya batas atas. **Prior kode-sendiri sudah meleset dua kali ke arah "ternyata lebih cepat", dan itu disebut sekarang, bukan sesudahnya** |
| **D31** | **Lemma C LULUS pada SELURUH sapuan S1**, termasuk `c_arm = 0.20`: 0 perhentian tanpa pose parkir | dunia | di 0.05 minimum fraksi bebas **83 %**; L4 naik dari 24.3 % ke 52.6 % antara 0.05 dan 0.20, jadi fraksi bebas turun tapi menuntut **nol dari 2376** tetap ekor yang jauh |
| **D32** | `Δ_arm = 0.0000` pada **setiap instance yang terbukti** sampai `c_arm = 0.15`; pada **0.20** setidaknya **satu** instance terbukti punya `Δ_arm > 0` | dunia | ditulis sebagai dugaan **berpatah**, bukan monoton, supaya ia bisa salah di dua arah. Sisi kedua bersandar pada 52.6 % terlarang + pose aman universal struktur tinggal 66/2376 di 0.20 (`p1_g11` Pertentangan 4) |
| **D33** | S2 pada `c_arm = 0.05`: `Δ_arm = 0.0000` pada setiap instance terbukti, dan vonis `Δ_full` **BUKAN MAHAL** pada anggaran terkunci 120 s | dunia | `Δ_struct` S2 **GRATIS** (40/40 exact, `lemma4`, `p1_g11 §B6.3`), jadi lengan satu-satunya yang bisa menggerakkannya; dan S1 sudah menunjukkan harganya nol |
| **D34** | Sisa 62 % wall `n6_s4_mr0` **didominasi `sk._dive`** (≥ 40 % wall), yaitu **kode BEKU**, bukan pembukuan fork | **kode sendiri** | `_dive` dipanggil di **setiap** node (baris 557) dan G10 W2b(ii) sudah mengukur bahwa dive-lah yang menanggung beban pencarian |
| **D35** | Instance `BINDING` **BISA** dibangun dengan tuas 3+4 (tugas dipaksa per gantry + beban seimbang), pada `c_arm ≥ 0.15` | dunia | G12 hanya memutar tuas 1–2; jalan keluar "menunggu" tidak pernah ditutup. Ini dugaan yang **melawan** prior LONGGAR, dan ditulis begitu dengan sadar |
| **D36** | Menaikkan `max_evade` 1 → 2 menutup **≥ 5 dari 9** celah `solver < W2` | **kode sendiri** | `p1_g10 §B9` sudah menyebut `max_evade` sebagai sebab paling mungkin, dan menghalfkan grid menutup **0 dari 9** |

### A5. Berkas

| Berkas | Isi | Status |
|---|---|---|
| `test/eval_sched_armfull.py` | satu berkas per **(set, c_arm)**, `report()` menggabungkan, `load average` dicatat | 🔓 satu perubahan mekanis (A0) |
| `test/m3_bind_g13.py` | A2.3: kelas instance, pencarian `BINDING`, klasifikasi tiga arah | **BARU** |
| `test/diag_g13.py` | A1 langkah 4 (profil `n6_s4_mr0`) dan langkah 6 (U4, `max_evade`) | **BARU** |
| `docs/p1_g13_sweep.md` | dokumen ini | — |

### A6. Rule 6

**Rule 6 (anggaran 30 000 token/sesi) adalah PENGECUALIAN EKSPLISIT untuk sesi
protokol-panjang P1, dinyatakan di muka**, sebagaimana `p1_g12 §A8`. Anggaran
yang benar-benar mengikat dan dilaporkan: **120 s wall per instance** (A2.4) dan
**biaya sapuan** (A2.1).

---

## B. Hasil terukur

> §A dikunci 2026-08-16 sebelum satu baris kode sesi ini dijalankan. Semua
> angka di bawah keluar sesudahnya. Setiap tempat di mana §B bertentangan
> dengan §A ditandai 🔺. **§A TIDAK ditulis ulang.**

### B0. Cara menjalankan ulang, dan beban mesin

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 test/eval_sched_armfull.py s1 0.20      # satu SEL sapuan
python3 test/eval_sched_armfull.py report       # semua sel + bebannya
python3 test/m3_bind_g13.py scan                # apakah tuas 3 ADA di peta ini
python3 test/m3_bind_g13.py m3                  # tabel kelas + tiga-arah
python3 test/m3_bind_g13.py gate                # solver vs W3 pada yang MENGIKAT
python3 test/diag_g13.py n6                     # profil n6_s4_mr0
python3 test/diag_g13.py u4 2                   # U4, max_evade
```

⚠️ **Beban mesin:** awal sesi `load average` **2.98**, selama sapuan **6–8.5**
pada **16 core**. Itu **di bawah jenuh** — jumlah proses runnable tidak pernah
melewati jumlah core — jadi wall per proses tidak ter-throttle, dan itulah
pembenaran A2.2 yang sesungguhnya, bukan konkurensi 3 itu sendiri. G12 diukur
pada 4.5–5.0, G11 pada 3.3–3.7, G10 pada 42.

🔒 **Berkas beku (§A0), diperiksa:** dua belas dari tiga belas `git diff`
**KOSONG**. **`test/verify_sched_armfull.py` GUGUR** — satu bug nyata di
`brute_arm` (§B3). `test/eval_sched_armfull.py` diubah sesuai izin §A0 (kunci
berkas per (set, `c_arm`) + `load average`), dan **R0 membuktikan semantiknya
tidak berubah**. Berkas baru: `test/m3_bind_g13.py`, `test/diag_g13.py`.

### B1. R0 — harness diubah, semantiknya TIDAK

Baris S1 `c_arm = 0.05` **dimigrasikan** dari G12 (A2.2 poin 3), bukan
dijalankan ulang, lalu dibaca lewat `report()` yang baru:

| | `p1_g12 §B7.1` | `report()` G13 | |
|---|---|---|---|
| `Δ_struct` | [0.0000, 0.4679] | [0.0000, 0.4679] | ✅ |
| `Δ_full` | [0.0000, 7.6155] | [0.0000, 7.6155] | ✅ |
| `Δ_arm` | [+0.0000, +7.1476] | [+0.0000, +7.1476] | ✅ |
| exact | 36 / 40 | 36 / 40 | ✅ |
| route berubah | 10 / 40 | 10 / 40 | ✅ |
| route (struct→full) | — | `lemma4→lemma4` 18, `bnb→bnb` 10, `lemma4→bnb` 4, `lemma4→dive-lb` 3, `dive-lb→bnb` 3, `dive-lb→dive-lb` 2 | ✅ |
| wall mean / maks | 31.0 / 120.1 s | 31.0 / 120.1 s | ✅ |

➜ **R0 LULUS.** Perubahan harness adalah kunci berkas, bukan perhitungan.

### B2. 🔺 PERTENTANGAN 1 — BIAYA SAPUAN, dan ANGGARAN 120 s TIDAK MENGIKAT

A2.1 mengunci probe pada `c_arm = 0.20` sebelum 40 instance. Terukur:

| probe | wall @0.05 | wall @0.20 | rasio | node | panggilan `ctx.ok` |
|---|---|---|---|---|---|
| `n4_s0_mr0` (medium) | **1.39 s** (`lemma4`, `proved`) | **139.92 s** (`bnb`, **tidak** terbukti) | **101×** | 0 | **0** |
| `n6_s4_mr0` (terburuk diketahui) | 120.06 s | **120.11 s** | **1.00×** | 11 | 4016 |

🔴 **Dua hal yang tidak bisa dibaca dari rasio itu, dan dua-duanya penting.**

**(a) 139.92 s > anggaran 120 s.** Anggaran "TERKUNCI 120 s" (`p1_g10 §K4`,
dipakai G7/G9/G10/G11/G12) **hanya diperiksa di dalam loop B&B dan
`feasible_starts`**. Konstruktor pra-loop tidak melihatnya sama sekali. Jadi
kalimat "anggaran 120 s per instance" **tidak pernah benar sebagai batas wall**
— ia batas atas pada **pencarian**, bukan pada instance. Ini bukan penemuan
tentang sesi ini saja: ia berlaku surut ke setiap sesi yang mengutip anggaran
itu. Terukur di sapuan: wall maks **144.3 s** pada `c_arm = 0.10`.

**(b) Biayanya bukan di gerbang lengan — ia di `arm_serial_ub`.** Probe medium
menghabiskan 139.92 s dengan **nol** panggilan `ctx.ok` dan **nol** node.
Ditelusuri per tahap:

| tahap, `n4_s0_mr0` | `c_arm = 0.05` | `c_arm = 0.20` | rasio |
|---|---|---|---|
| uji lengan pada rute Lemma 4 | 0.015 s → **bebas** | 0.010 s → **TERBLOKIR** | — |
| `_repair_ub` (struktur, beku) | 0.001 s | 0.001 s | 1× |
| **`arm_serial_ub`** (A2.4, tutup 60 pose dicabut) | **0.039 s** | **138.009 s** | **3500×** |

🔺 **Anggaran A2.5 G12 (`arm_serial_ub ≤ 1.0 s`) diverifikasi pada
satu-satunya `c_arm` yang loop-nya TIDAK PERNAH BERITERASI.** G12 mengukur
0.23 s dan menyatakannya ✅. Biaya sebenarnya **bukan per instance** — ia **per
pose yang DITOLAK**, dan yang mengendalikan jumlah penolakan adalah `c_arm`.
Pada 0.05 pose parkir termurah langsung diterima dan loop `break` pada iterasi
pertama; pada 0.20 ia menyapu ribuan pose, masing-masing membayar dua
`schedule_conflict` **dan** dua `schedule_arm_conflict` atas **seluruh** jadwal.

➜ **Ini sesi KEENAM berturut-turut dengan jebakan "yang lambat adalah
PEMERIKSA"** (`p1_g8 §B4`, `p1_g9 §B3`, `p1_g10 §B3-1`, `p1_g11 §B4`,
`p1_g12 §B4`), dan yang **kedua** di mana pengukuran-sebelum-sapuan mengubah
apa yang boleh ditulis tentang biayanya.

🔒 **Keputusan A2.1 tetap dipakai apa adanya** (probe medium menyentuh 120 s →
sapuan dijalankan **penuh**, urutan sel mengikat). Yang **gugur** adalah
aritmetika biayanya: batas "40 × 120 s = 80 menit per sel" **salah**, karena
120 s bukan batas wall. `arm_serial_ub` **TIDAK diperbaiki** — ia di berkas
beku, ia **benar** (mengembalikan 47.9603 di kedua `c_arm`), dan yang cacat
adalah anggaran yang mengukurnya, bukan kodenya.

### B3. 🔴 BUG NYATA di `brute_arm` — BERKAS BEKU, PEMBEKUANNYA GUGUR

Ditemukan oleh **uji yang dijalankan** (gerbang M3 pada instance yang
mengikat), bukan oleh pembacaan ulang. **Enam dari enam** untuk pola
`p1_g10 §B8`.

Gerbang M3 melaporkan `solver < W3` pada satu instance: `s10`, `c_arm = 0.15`,
solver **10.7600 `proved`** melawan W3 **12.2600**. Dua bacaan mungkin — solver
terlalu longgar (bug soundness) atau W3 terlalu kasar. **Diadu, tidak
dinalar:** jadwal solver dijalankan lewat **tiga** gerbang independen —
`validate_coupled` (sadar-tunggu), `sa.arm_schedule_conflict` (jalan lambat
G11), dan `sc.schedule_conflict` (struktur). **Ketiganya lulus.** Jadwalnya
layak; W3 yang salah.

Sebabnya, di `verify_sched_armfull.brute_arm`:

```python
for p in range(P[g]):
    dur, assign = sched.stop_duration(inst, g, U, p)
    if not np.isfinite(dur):
        U = (U - 1) & R
        break            # <- meninggalkan SELURUH pose sisanya untuk U ini
```

Pada pose **pertama** yang tidak layak untuk sebuah subset `U`, ia membuang
setiap pose **p+1 … P−1** untuk subset itu dan langsung pindah ke `U`
berikutnya. **Enumeratornya tidak menyeluruh**, jadi W3 adalah **taksiran
lebih** atas optimum, bukan optimum.

🔒 Perbaikan: `continue` alih-alih `U = (U-1)&R; break`, dan `U` dimundurkan
sesudah loop pose selesai. Sesudahnya `s10 c=0.15` memberi W3 **10.7600**,
**persis** nilai solver.

**Apa yang ini runtuhkan, dan apa yang TIDAK:**

1. 🔴 **Angka M3 `p1_g12 §B8` ("0 dari 18 mengikat") diukur dengan enumerator
   yang cacat**, dan begitu juga jalan pertama M3 sesi ini. Keduanya dihitung
   ulang di §B5.
2. ✅ **Kesimpulan `solver > W3` pada 0 dari 18 TETAP SAH.** Setiap jadwal yang
   `brute_arm` kembalikan sudah lolos kedua gerbangnya, jadi W3 selalu
   **batas atas yang sah** atas optimum sejati; membuatnya tidak menyeluruh
   membuatnya **lebih longgar**, tidak salah arah. Palang soundness berdiri.
3. ⚠️ **`enum_opt` sesi ini mewarisi bentuk yang sama**, jadi "0 selisih
   enumerator" di jalan pertama **bukan** bukti kebenaran — dua instrumen
   dengan cacat yang sama akan selalu sepakat. Itu ditulis, bukan dihaluskan:
   silang-periksa itu hanya menguji pembukuan, bukan kelengkapan. Keduanya
   diperbaiki dan dijalankan ulang.

### B4. 🟢 M3 LULUS — utang oracle G12 LUNAS, dan gerbangnya DISKRIMINATIF

`p1_g12 §B8` menutup dengan: *"W3 mengonfirmasi solver tidak pernah lebih buruk
dari brute force, tapi ia **tidak bisa** mendeteksi solver yang terlalu
longgar, karena tidak ada instance kecil di mana longgar dan ketat berbeda."*
Sekarang ada **sebelas**.

**Kenapa G12 tidak menemukannya, dan itu bukan tuas 3 atau 4:** G12 menjalankan
M3 **hanya pada `c_arm = 0.05`**. Sapuan `c_arm` — tuas 6, satu-satunya yang
G12 tidak putar — adalah yang membuka gerbangnya. Pada 0.05 sesi ini juga
mendapat **0 mengikat**, jadi angka G12 **tereproduksi** dan sekarang
**terjelaskan**: ia pernyataan tentang `c_arm = 0.05`, bukan tentang kelas
instance-nya.

| | Terukur |
|---|---|
| instance `BINDING` (`brute_arm(c) > brute_arm(∞)`) | **11**, semuanya pada `c_arm ≥ 0.10` |
| pada `c_arm = 0.05` | **0** — mereproduksi `p1_g12 §B8` |
| kenaikan optimum | 10.2600 → **12.2600** (10 instance) dan → **10.7600** (1) |
| **`solver > W3`** (tidak sound) | **0 / 11** ✅ |
| **`solver < W3`** (terlalu longgar) — arah yang G12 **tidak bisa** uji | **0 / 11** ✅ |
| solver `proved` pada semuanya | **11 / 11** |

🔒 **Inilah yang membuatnya diskriminatif:** pada instance ini solver yang
mengabaikan atau kurang-menerapkan `ARM_BLOCK` akan mengembalikan **10.2600**,
optimum struktur. Ia mengembalikan **12.2600** (dan 10.7600), **persis** nilai
enumerator sadar-lengan yang tidak berbagi satu baris pun dengan pencariannya.
Kenaikannya **+2.0000 s = tepat satu `DWELL`** — kendalanya memaksa satu dwell
diserialkan, dan itu mekanisme yang bisa disebut, bukan sekadar angka.

➜ **Kualifikasi `p1_g12 §B10` poin 2 DICABUT.** "Exact" sekarang berarti exact
terhadap M0+M1+M2 **dan** terhadap enumerator independen yang diskriminatif —
pada kelas instance kecil, pada `c_arm ≥ 0.10`.

