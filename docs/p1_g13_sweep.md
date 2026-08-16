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

⚠️ **Yang TIDAK tercapai, dan A2.3 menuntut ini disebut:** lantai usaha A2.3
(≥ 5 kelas, ≥ 300 kandidat, ≥ 40 brute-force) **TIDAK terpenuhi** — dengan
enumerator yang sudah diperbaiki pencariannya jauh lebih mahal, dan tutup wall
1800 s memotong di **1 kelas, 10 kandidat, 32 brute-force**. Lantai itu ditulis
sebagai syarat untuk menulis **"tidak bisa dibangun"**; karena hasilnya
**positif**, ia tidak lagi menjadi gerbang. Tabel tiga-arah karena itu
dilaporkan **hanya untuk kelas `G12-baseline`**, dan tuas 3 + 4 (`gen_forced`)
**TIDAK DIUKUR** — bukan "diperkirakan tidak menambah".

| kelas `G12-baseline`, 10 seed × 4 `c_arm` | |
|---|---|
| di-brute-force dua arah | **32** |
| **`binding`** — optimum naik | **7** |
| **`rerouted`** — mengikat, harga **nol** | **9** |
| **`vacuous`** — tidak menyentuh optimum | **16** |
| selisih antar enumerator | **0** |

🔒 **Pembagian `vacuous` / `rerouted` adalah kalimat naskahnya**, dan ia yang
G12 tidak punya: pada **9 dari 32**, `ARM_BLOCK` benar-benar **membuang
sebagian optimum** dan yang lain bertahan dengan makespan yang sama. Itu
mekanisme "mengikat dan gratis" §B7.3 G12, sekarang **terhitung**, bukan
disimpulkan.

### B5. 🟢 ANGKA UTAMA — sapuan `c_arm` pada S1 dan S2

Anggaran **120 s TERKUNCI** di setiap sel (A2.4). `c_clear = 0.0` di kedua
sisi. Konkurensi 3 sel (A2.2), beban tercatat per sel.

#### B5.1 S1 — `gen_real`, 40 instance per sel

| `c_arm` | `Δ_struct` | `Δ_full` mean % dua-sisi | vonis `Δ_full` | `Δ_arm` | exact | route diubah lengan | wall mean/maks | beban |
|---|---|---|---|---|---|---|---|---|
| **0.00** (degenerate) | [0, 0.4679] | **[0.0000, 1.3529]** | 🟢 **BUKAN MAHAL** | [+0.0000, +0.8850] | **38/40** | 3/40 | 20.9 / 120.1 s | 5.96→8.37 |
| **0.05** (utama G12) | [0, 0.4679] | [0.0000, 7.6155] | TIDAK DAPAT DITENTUKAN | [+0.0000, +7.1476] | **36/40** | 10/40 | 31.0 / 120.1 s | 4.75 (konk. **1**) |
| **0.10** | [0, 0.4679] | [0.0000, 14.9514] | TIDAK DAPAT DITENTUKAN | [+0.0000, +14.4836] | **31/40** | 14/40 | 45.5 / **144.3 s** | 3.68→6.42 |
| **0.15** | [0, 0.4679] | **TIDAK DIUKUR pada 4 / 40** | — | — | 23 terbukti | — | — | 3.68→9.89 |
| **0.20** | [0, 0.4679] | **TIDAK DIUKUR pada 6 / 40** | — | — | 19 terbukti | — | — | 3.68→4.14 |

#### B5.2 S2 — `gen_real_rotcrowded`, 40 instance per sel

⚠️ **S1 dan S2 TIDAK PERNAH dirata-ratakan bersama.** Probe S2 membatasi
**himpunan pose** (`p1_g11 §B3`), dan setiap angka di bawah membawa
kualifikasi itu.

| `c_arm` | `Δ_struct` | `Δ_full` | vonis | `Δ_arm` | exact | route diubah lengan | wall mean/maks |
|---|---|---|---|---|---|---|---|
| **0.05** | [0, 0] **GRATIS** | **[0.0000, 0.0000]** | 🟢 **GRATIS** | [+0.0000, +0.0000] | **40/40** | **16/40** | 4.8 / 71.6 s |
| **0.10** | [0, 0] GRATIS | [0.0000, 15.2127] | TIDAK DAPAT DITENTUKAN | [+0.0000, +15.2127] | **33/40** | **22/40** | 30.2 / 120.0 s |
| **0.15** | [0, 0] GRATIS | **TIDAK DIUKUR pada 1 / 40** | — | — | 25 terbukti | — | — |
| **0.20** | [0, 0] GRATIS | **TIDAK DIUKUR pada 11 / 38** | — | — | 14 terbukti | — | — |
| **0.00** (degenerate) | [0, 0] GRATIS | **[0.0000, 0.0000]** | 🟢 **GRATIS** | [+0.0000, +0.0000] | **40/40** | 1/40 | 0.1 / 0.2 s |

🔒 **`Δ_struct` S2 tetap GRATIS di seluruh sapuan** (`lemma4` 40/40), jadi pada
set ini **lengan adalah satu-satunya hal yang bisa menggerakkan angka** —
`Δ_arm ≡ Δ_full`. Itu yang membuat S2 pengukuran lengan yang lebih bersih
daripada S1, dan itu juga yang membuat §B5.4 mungkin.

#### B5.3 🟢 ANGKA SESI INI: `Δ_arm = 0.0000` pada SETIAP instance terbukti

Diperiksa langsung, bukan disimpulkan dari mean: berapa instance **terbukti**
punya `Δ_arm > 0`?

| set | 0.00 | 0.05 | 0.10 | 0.15 | 0.20 |
|---|---|---|---|---|---|
| **S1** terbukti | 38 | 36 | 31 | 23 | 19 |
| **S1** dengan `Δ_arm > 0` | **0** | **0** | **0** | **0** | **0** |
| **S2** terbukti | 40 | 40 | 33 | 25 | 14 |
| **S2** dengan `Δ_arm > 0` | **0** | **0** | **0** | **1** | **1** |

> **BOLEH DIKUTIP:** pada **S1**, `Δ_arm = 0.0000` **TERBUKTI OPTIMAL** pada
> **setiap** instance yang bisa dibuktikan, pada **kelima** nilai `c_arm`
> sampai **0.20** — yaitu **empat kali** clearance utama G12. **147 bukti
> `Δ_arm = 0` dan nol contoh tandingan.** Pada **S2**, hal yang sama berlaku di
> `c_arm ≤ 0.10` (73 instance terbukti, nol positif).
>
> **TIDAK BOLEH:** vonis `Δ_full` pada `c_arm ≥ 0.15`. Di sana baris **TIDAK
> ADA JADWAL DITEMUKAN** (§B5.5) membuat mean dua-sisi tidak terdefinisi, dan
> ambang 5.0 % **tidak digeser**.

#### B5.4 🔴 CONTOH TANDINGAN PERTAMA: `Δ_arm` POSITIF, TERBUKTI

`n6_s0_mr1` pada **S2**, dan ia satu-satunya di seluruh sapuan:

| | |
|---|---|
| batas bawah takterkopel | **9.2600** |
| optimum struktur (`lemma4`, gratis) | **9.2600** |
| **optimum sadar-lengan** | **9.7600, `proved`** |
| **`Δ_arm`** | **+0.5000 s = +5.40 %** |
| route | `lemma4` → **`bnb`** |
| pada `c_arm` | **0.15 DAN 0.20** (identik) |
| `validate_coupled` / jalan lambat G11 | **lulus / lulus** |

🔒 **Ini harga positif pertama untuk koordinasi lengan–lengan yang pernah
dibuktikan di P1**, dan ia dilaporkan dengan seluruh kualifikasinya: satu
instance, pada set **S2** yang himpunan posenya dibatasi, pada `c_arm` **tiga
sampai empat kali** nilai utama. Ia **tidak** mengubah vonis mana pun — tapi ia
mematahkan kalimat "kendala lengan tidak pernah punya harga", dan G12 tidak
punya satu pun contoh seperti ini.

#### B5.5 🔴 KATEGORI BARU: **TIDAK ADA JADWAL DITEMUKAN**, dan ia BUKAN ketidaklayakan

Pertama kali kategori `p1_g12 §A2.4` ini benar-benar terisi:

| set / `c_arm` | instance tanpa jadwal | nama |
|---|---|---|
| S1 / 0.15 | **4 / 40** | `n4_s4_mr1`, `n6_s1_mr1`, `n6_s2_mr1`, `n6_s4_mr1` |
| S1 / 0.20 | **6 / 40** | + `n4_s5_mr1`, … |
| S2 / 0.15 | **1 / 40** | `n6_s1_mr1` |
| S2 / 0.20 | **11 / 38** | `n4_s3_mr0`, `n4_s5_mr1`, `n4_s6_mr0`, `n4_s7_mr0`, `n4_s8_mr1`, … |

🔒 **Dilarang menyebutnya "tidak layak".** Lemma C diukur ulang di kedua
`c_arm`, seperti A2.5 poin 6 wajibkan:

| `c_arm` | perhentian | tanpa pose parkir | fraksi \|P\| bebas: min / p5 / p50 |
|---|---|---|---|
| 0.05 (G12) | 107 | **0** | 0.8300 / 0.8841 / 0.9731 |
| **0.15** | 107 | **0** | **0.5640** / 0.7035 / 0.8872 |
| **0.20** | 107 | **0** | **0.4310** / 0.6227 / 0.8283 |

**Lemma C LULUS di seluruh sapuan.** Jadi mekanisme "tidak ada tempat bagi
gantry lawan di seluruh sel" **DIKESAMPINGKAN oleh pengukuran** — kategorinya
adalah **anggaran**, dan ditulis `TIDAK ADA JADWAL DITEMUKAN (anggaran 120 s)`.

🔺 **PERTENTANGAN 2 — dan ia terhadap G12, bukan terhadap §A.** `p1_g12 §B6`
menyimpulkan: *"Serialisasi selalu ada, jadi `arm_serial_ub` selalu
mengembalikan sesuatu dan kurungan `Δ_full` selalu terdefinisi."* **Inferensi
itu SALAH, dan sesi ini punya contoh tandingannya:** Lemma C lulus di 0.15 dan
0.20 dan `arm_serial_ub` tetap tidak mengembalikan apa-apa pada 4/6/1/11
instance. Sebabnya struktural, bukan numerik — **Lemma C hanya menyertifikasi
fase PARKIR** (lawan menggantung di satu pose melawan perhentian bertugas),
sementara `arm_serial_ub` juga menuntut **seluruh ekor serial** (gantry parkir
kemudian mengerjakan perhentiannya sendiri) bebas pada **kedua** predikat.
Lemma C tidak mengatakan apa pun tentang ekor itu.

➜ Ini juga menjelaskan §B2: pada `c_arm` besar `arm_serial_ub` menyapu **semua
2376** pose, menolak setiap satu, dan mengembalikan `inf` setelah membayar
penuh. Biayanya 138 s **dan** hasilnya kosong.

### B6. 🔴 BUG NYATA KEDUA — `sched_arm.arm_first_block`, gerbang R1 pada S2

`p1_g12 §B8` melaporkan M1 **40/40 pada S1**. M2b G12 juga hanya memakai
**delapan instance S1** (`itertools.islice(_s1(), 8)`). Sel S2 sesi ini adalah
pertama kalinya gerbang lambat dijalankan atas jadwal `rotcrowded` — dan ia
**gagal**, pada `n4_s3_mr0`, `c_arm = 0.05`.

Diadu, tidak dinalar. Pada `t = 3.7600`:

| | |
|---|---|
| jarak lengan–lengan sejati | **0.055417 m** |
| ambang `c_arm + ε` | **0.055000 m** |
| **vonis benar** | **BEBAS** (margin 0.417 mm) |
| jalan **cepat** (`sched_armfull`) | **bebas** ✅ |
| jalan **lambat** (`sched_arm`) | **BLOK** ❌ |

Sebabnya: `t = 3.76` adalah instan **terakhir** sebuah leg **dan** instan
pertama sebuah dwell. `ArmTraj.moving()` **tertutup di kanan**, jadi ia
melaporkan "bergerak" untuk potongan `[3.76, 7.76)` yang **seluruhnya statis**.
Jalan itu lalu tidak `break`, menghitung `step = (d − c − ε)/V_REL_ARM =
0.00188 s < STEP_MIN = 0.01`, dan **lantai STEP_MIN** — yang ada hanya untuk
benda yang benar-benar bergerak — mengembalikan blok yang tidak ada.

🔒 Perbaikan: uji statis ditanyakan pada **POTONGAN**, di titik tengahnya, bukan
pada instan `t`. Di dalam satu potongan keadaan-bergerak konstan (marks memuat
setiap ujung leg), jadi titik tengah **eksak** — dan itu **persis konvensi yang
`sched_armfull.arm_conflict` sudah pakai**, yang menjelaskan kenapa jalan cepat
benar dan jalan lambat salah.

**Ini kelas yang sama dengan `p1_g12 §B3.1`** (tepi kanan tertutup), di fungsi
yang perbaikan itu tidak sentuh. **`sched_arm.py` gugur pembekuannya untuk
sesi kedua berturut-turut.**

**Regresi sesudah perbaikan — semuanya dijalankan, bukan diasumsikan:**

| | |
|---|---|
| `verify_sched_arm.py all` (L0–L5 G11) | **LULUS enam-enamnya**, angka L2/L4/L5 identik G11 |
| **M2a** — δ Lemma B pada 2376² di **setiap** `c_arm` | **LULUS**, δ ≤ 0.004 di kelimanya (G12 melaporkan ini **TIDAK DIJALANKAN**) |
| **M2b** — cepat vs lambat, 1864 jadwal termutasi | **0 selisih vonis**, 630 dengan pelanggaran nyata |
| S2 `c_arm = 0.05` dijalankan ulang | **0 ARMVIOL / 40** |
| tiga tanda ARMVIOL sisa (S2 0.10 ×2, 0.20 ×1) | diperiksa langsung: **ketiganya BASI**, pasca-perbaikan `SLOW = None` |

➜ **R1 dan R2 LULUS di seluruh sapuan: 0 kegagalan gerbang nyata, 0 pelanggaran
Lemma 3**, pada 9 sel × 40 instance.

### B7. Diagnosis `n6_s4_mr0` — dan `_dive` BUKAN jawabannya

`p1_g12 §B10.3`: satu-satunya instance S1 yang terkurung pada **kedua**
anggaran. Diprofil pada anggaran terkunci 120 s, `c_arm = 0.05`:

| | |
|---|---|
| node diperluas | **7** (bukan 45 — itu angka 600 s) |
| makespan / LB | 79.0166 / 39.7126, `bnb`, **tidak terbukti** |
| **gerbang lengan** | 2602 panggilan, **71.3 s = 59.4 % wall** |
| **`feasible_starts`** (struktur, **BEKU**) | **44.6 s = 37 % wall** |
| **`_dive`** | **8 panggilan, tidak muncul di daftar kumulatif sama sekali** |
| `action_hit` | **7** — anggaran aksi tersentuh di **setiap** node |
| `multi_skipped` | 2434 |

🔴 **D34 MELESET**, dan arah melesetnya adalah pola sesi keenam: yang mahal
adalah **PEMERIKSA** — gerbang lengan (59 %) plus pemeriksa struktur beku
(37 %) = **96 % wall**. Pencariannya sendiri praktis gratis; ia hanya tidak
pernah sampai ke mana-mana karena setiap aksi harus dibayar dua kali.

**Dan satu angka yang menjelaskan 59 % itu:** `_pair_dist` dipanggil **78 121**
kali, `ArmView.polys` **148 210**, `irm_sweep.base_pose` **296 420**.
`ArmView.pts` menyimpan cache **hanya konfigurasi dwell** (`key = (pose,
tasks)`); konfigurasi **MENGGANTUNG** dihitung ulang dari `base_pose` setiap
kali. Itu **tidak diperbaiki** — `sched_armfull.py` beku, ia **benar**, dan
mengubahnya di sesi yang sama yang memakainya untuk mengukur `Δ` akan
membatalkan M0. Dicatat sebagai **utang terukur**, bukan dugaan.

➜ **Vonis: menaikkan anggaran ke-3 kalinya adalah jawaban yang salah.** 96 %
wall ada di dua pemeriksa; anggaran 600 s membeli 45 node, bukan bukti.

### B8. U4 — `max_evade` BUKAN sebabnya. Terbantah oleh pengukuran.

`p1_g10 §B9` menyebut `solver < W2` pada **9 dari 31** dan menamai tersangkanya:
*"W2 dibatasi `max_evade = 1` per gantry, sementara solver memakai sampai 3"*.
**TIDAK DIUKUR sejak G10.** Diukur sekarang:

| | |
|---|---|
| instance dengan `solver < W2` pada `max_evade = 1` | **9** (mereproduksi G10 persis) |
| W2 **turun** pada `max_evade = 2` | **0 / 9** |
| W2 **tertutup** ke solver | **0 / 9** |
| timeout | **0 / 9** |
| node W2 (contoh `crowd_n2P4mr0s0`) | **1026 → 10 566** (10×) |

🔴 **D36 MELESET telak.** Enumeratornya mencari **sepuluh kali lebih banyak**
dan mengembalikan **nilai yang sama persis**. Jadi celahnya **bukan** grid
(G10 mengukur: menghalfkan menutup 0 dari 9) dan **bukan** `max_evade`.

**Yang tersisa, dan ia sekarang satu-satunya kandidat yang berdiri:** `_starts`
W2 menyampel waktu mulai pada **grid seragam** berjangkar di `t[g]`, sementara
`feasible_starts` solver menghitung waktu mulai layak **kontinu** — dan sebuah
bilangan real yang sembarang tidak pernah masuk grid seragam mana pun, jadi
**menghalfkan grid memang tidak bisa menutupnya.** Uji G10 benar kelasnya tapi
salah instrumennya. Ini **belum diukur** dan ditulis sebagai kandidat, bukan
sebagai kesimpulan.

⚠️ `Brute._rec` **diperiksa** untuk cacat yang sama dengan §B3 dan **tidak
punya** — ia melewati aksi tak-layak tanpa `break`. Diperiksa karena
`brute_arm` adalah keturunannya, bukan karena diduga.

### B9. Papan skor §7.2

| # | Dugaan (§A4, ditulis di muka) | Hasil |
|---|---|---|
| **D30** | wall @0.20 ≥ 2× wall @0.05, dan ≥ 8 dari 40 menyentuh 120 s | 🟡 **SEPARUH** — pada probe medium **101×**, jauh melewati "≥ 2×", tapi **mekanismenya bukan yang diduga**: bukan pencarian yang membengkak, melainkan `arm_serial_ub` (§B2). Ditulis pesimis dan **tetap terlalu optimis**, yaitu arah yang **berlawanan** dengan dua sesi terakhir |
| **D31** | Lemma C LULUS di seluruh sapuan S1 | ✅ **TEPAT** — 107 perhentian, **0** tanpa pose parkir, di 0.15 **dan** 0.20; minimum fraksi bebas turun 83 % → **43 %** tapi tidak pernah nol |
| **D32** | `Δ_arm = 0` sampai 0.15; pada 0.20 **≥ 1** instance terbukti positif | 🟡 **SEPARUH, dan menariknya di kedua sisi** — separuh pertama **TEPAT dan lebih kuat** (0 positif pada **seluruh** S1 sampai 0.20, 147 bukti); separuh kedua **TEPAT tapi di set yang salah dan `c_arm` yang salah** — positifnya ada di **S2**, dan sudah muncul di **0.15**, bukan 0.20 |
| **D33** | S2 @0.05 `Δ_arm = 0` pada setiap yang terbukti, vonis BUKAN MAHAL @120 s | ✅ **TEPAT, dan lebih kuat** — **40/40 exact**, vonis **GRATIS**, bukan sekadar BUKAN MAHAL |
| **D34** | sisa wall `n6_s4_mr0` didominasi `_dive` (≥ 40 %) | ❌ **MELESET** — `_dive` **tidak muncul sama sekali**; 96 % wall ada di gerbang lengan (59 %) + `feasible_starts` beku (37 %) |
| **D35** | instance `BINDING` bisa dibangun **dengan tuas 3+4**, pada `c_arm ≥ 0.15` | 🟡 **SEPARUH** — instance mengikat **BISA** dibangun (≥ 11), tapi **tanpa** tuas 3 dan 4 sama sekali: yang membukanya adalah **tuas 6 (`c_arm`)**, satu-satunya yang G12 tidak putar. `c_arm ≥ 0.15` **hampir** tepat — yang pertama mengikat ada di **0.10**. Tuas 3+4 **TIDAK DIUKUR** |
| **D36** | `max_evade` 1→2 menutup ≥ 5 dari 9 | ❌ **MELESET telak** — **0 dari 9**, dengan 10× node |

**Dua tepat, dua meleset, tiga separuh.** Papan skor `p1_g12 §B9` berdiri di
**23 meleset / 10 tepat**; sesi ini menambah **2/2** (separuh tidak dihitung ke
kolom mana pun) → **25 meleset, 12 tepat**.

Empat bacaan:

1. **Prior DUNIA ("kendala yang belum diukur itu LONGGAR") menang lagi, dan
   sekarang dengan batasnya yang pertama terukur.** 147 bukti `Δ_arm = 0` di
   S1 sampai `c_arm = 0.20` melawan **satu** contoh tandingan terbukti di S2
   (+5.40 %). Prior itu benar sebagai prior; ia bukan teorema, dan sekarang ada
   satu titik data yang menunjukkan di mana ia patah — **himpunan pose yang
   dibatasi**, bukan clearance yang dinaikkan.
2. **Prior KODE SENDIRI berbalik arah.** D22/D25 dua sesi berturut-turut
   menduga kode sendiri terlalu **LAMBAT** dan meleset. Sesi ini D30 dan D34
   meleset ke arah **sebaliknya** — biayanya lebih besar dan di tempat yang
   tidak diduga (`arm_serial_ub`, `feasible_starts`). Sekarang **5 dari 9**,
   dan arah melesetnya **tidak lagi bisa disebut**. Yang bertahan adalah
   pernyataan yang lebih sempit: **yang mahal selalu PEMERIKSA, bukan
   pencarian** — enam sesi berturut-turut.
3. **D29 (G12) tetap prior paling berguna yang dimiliki proyek ini, dan
   sekarang ENAM sesi berturut-turut.** Dua bug nyata sesi ini
   (`brute_arm` §B3, `arm_first_block` §B6), **dua-duanya di berkas beku**,
   **dua-duanya ditemukan oleh uji yang dijalankan**, **nol oleh pembacaan
   ulang**. Dan keduanya berada di tempat yang gerbang sebelumnya **tidak
   pernah jangkau**: `brute_arm` hanya dipanggil pada instance yang mengikat
   (tidak ada sebelum sesi ini), `arm_first_block` hanya gagal pada jadwal S2
   (M1/M2b G12 keduanya hanya S1).
4. 🔴 **Satu bacaan yang tidak menyenangkan tentang metode sesi ini sendiri:**
   `enum_opt` ditulis sebagai instrumen **kedua** untuk menyilang-periksa
   `brute_arm`, dan ia mewarisi **cacat yang sama persis** karena ditulis dari
   bentuk yang sama. "0 selisih enumerator" karena itu **tidak membuktikan apa
   pun tentang kelengkapan** — ia hanya menguji pembukuan. Yang benar-benar
   menemukan bugnya adalah **membandingkan dengan artefak yang tidak berbagi
   bentuk** (solver + tiga gerbang independen). Itu pelajaran yang bisa
   dipakai, bukan sekadar pengakuan.

### B10. Batasan setelah sesi ini

Seluruh `p1_g7 §A4`, `p1_g8 §B10`, `p1_g9 §A4`, `p1_g10 §B12`, `p1_g11 §B9`,
`p1_g12 §B10` **masih berlaku** kecuali yang dicabut eksplisit di atas
(`p1_g12 §B10` poin **2** DICABUT — M3 sekarang menyala; poin **4** DICABUT —
S2 dan sapuan sekarang terukur). Yang ditambahkan:

1. 🔴 **Anggaran "120 s TERKUNCI" BUKAN batas wall**, dan tidak pernah (§B2a).
   Ia batas atas pada **loop pencarian**; konstruktor pra-loop
   (`solve_exact`, `_repair_ub`, `_dive` akar, `arm_serial_ub`) tidak
   melihatnya. **97 dari 400 instance** sesi ini melewatinya. Tapi besarnya
   **sangat tidak merata**, dan itu yang menentukan apa yang harus dikerjakan:

   | sel | wall mean | maks | > 120 s | overshoot maks |
   |---|---|---|---|---|
   | S1 0.00 / 0.05 | 20.9 / 31.0 s | 120.1 s | 2 / 4 | **0.1 s** |
   | S1 0.10 | 45.5 s | 144.3 s | 9 | 24.3 s |
   | **S1 0.15** | 96.6 s | **437.4 s** | 16 | **317.4 s** |
   | **S1 0.20** | 108.9 s | 380.2 s | 19 | 260.2 s |
   | S2 (kelimanya) | 0.1–82.9 s | 120.0 s | 0–25 | **0.0 s** |

   🔒 **Yang TIDAK berubah: tidak satu pun angka `Δ`.** Setiap makespan adalah
   bukti optimalitas (bebas anggaran) atau batas atas yang sah, dan vonis
   dua-sisi bergantung pada `exact`/`proved`, yaitu pada anggaran **pencarian**
   — yang memang sama untuk semua instance.

   🔴 **Yang berubah: labelnya.** `TIDAK ADA JADWAL DITEMUKAN (anggaran 120 s)`
   menyesatkan — instance itu mendapat **437 s wall** dan tetap kosong. Label
   yang benar menyebut **dua** angka: anggaran pencarian **dan** wall.

   ⚠️ **Cakupan surutnya lebih sempit dari kesan pertama, dan itu diukur.**
   Pada sel utama G12 (S1, `c_arm = 0.05`) overshoot maksimum **0.1 s**, dan
   pemindai tak-terbatasnya baru ada sejak G12 **mencabut tutup 60 pose** G11
   (`p1_g12 §B6`) — sebelum itu `arm_serial_ub` terbatas oleh konstruksi.
   S2 **nol overshoot di kelima sel**, karena probe-nya membatasi himpunan pose
   sehingga sapuan parkirnya pendek. ➜ **Opsi (a) §C tugas 1 ("jalankan ulang
   semuanya") kemungkinan besar TIDAK diperlukan**; yang diperlukan adalah
   menamai ulang dan melaporkan wall terpisah. Itu dugaan yang diturunkan dari
   pengukuran di atas, **bukan** izin untuk melewatkan tugas 1.
2. 🔴 **`Δ_full` TIDAK DAPAT DITENTUKAN pada `c_arm ≥ 0.15`** di kedua set,
   karena baris **TIDAK ADA JADWAL DITEMUKAN** (§B5.5). Itu **anggaran**, bukan
   ketidaklayakan — Lemma C lulus di kedua nilai.
3. 🔴 **Inferensi `p1_g12 §B6` "serialisasi selalu ada" SALAH** (§B5.5
   Pertentangan 2). Lemma C hanya menyertifikasi fase **parkir**, bukan ekor
   serialnya.
4. 🔴 **`arm_serial_ub` berbiaya 138 s pada `c_arm = 0.20`**, 138× anggaran
   A2.5 G12 — yang diverifikasi pada satu-satunya `c_arm` yang loop-nya tidak
   beriterasi. **Tidak diperbaiki** (berkas beku, kodenya benar).
5. 🔴 **Satu `Δ_arm` positif terbukti** (§B5.4), pada **satu** instance, di
   **S2**, pada `c_arm ≥ 0.15`. Ia tidak mengubah vonis apa pun tapi ia mencabut
   kalimat "harganya selalu nol".
6. **Tuas 3 + 4 M3 (`gen_forced`) TIDAK DIUKUR** — lantai usaha A2.3 tidak
   terpenuhi (1 kelas dari ≥ 5, 10 kandidat dari ≥ 300). Tersedia dan siap.
   Diukur: tuas 3 **ADA tapi tipis** (6 + 6 tugas satu-gantry dari 864; 90 %
   node dijangkau **kedua** gantry — sel ini nyaris **invarian-gantry**,
   analog dengan invarian-rotasi `p1_g11 §B3`).
7. **Sembilan dari sepuluh sel sapuan dijalankan penuh** (S1 ×5, S2 ×5 — S2
   `c_arm = 0.00` selesai paling akhir, 40/40 GRATIS). Tidak ada sel yang
   dilaporkan sebagian.
8. **Sel kontrol A2.2 poin 4 TIDAK DIUKUR:** S1 `c_arm = 0.05` tetap satu-satunya
   baris pada konkurensi 1. Perbandingan lintas-`c_arm` karena itu **tidak
   sepenuhnya adil** pada baris itu, dan itu ditulis, bukan dihaluskan.
9. **`ArmView` tidak meng-cache konfigurasi MENGGANTUNG** (§B7): 296 420
   panggilan `base_pose` dalam satu instance 120 s. Utang terukur, tidak
   diperbaiki karena memperbaikinya membatalkan M0 di sesi yang sama yang
   memakainya.
10. **Celah `solver < W2` masih terbuka** (§B8), dengan dua sebab tersingkir
    (grid halving, `max_evade`) dan satu kandidat tersisa yang **belum diukur**
    (grid seragam vs waktu mulai kontinu).
11. Proksi polyline tetap **meremehkan** volume sapuan; `T_fold = 0`; kanonik
    `manip`; exact tetap relatif terhadap grid 33 × 72; `safe_poses` tetap tidak
    diganti (`p1_g12 §B10.5`).
12. 🔴 **Rule 6 JEBOL**, dan §A6 menyatakannya di muka. Anggaran yang benar-benar
    mengikat dan dilaporkan: §B2 dan tabel per sel §B5.


---

## C. Prompt sesi berikutnya — G14

> **Rekomendasi: Opus 5, effort TINGGI.** Naik lagi dari SEDANG, dan alasannya
> spesifik: G13 menutup sapuan dan oracle, tapi ia membuka **tiga hal yang
> tidak punya sinyal error** — (a) `Δ_arm` positif pertama muncul di S2 dan
> belum ada yang tahu **kenapa S2 dan bukan S1**; (b) baris TIDAK ADA JADWAL
> adalah anggaran, dan menutupnya menuntut memutuskan **di mana** `arm_serial_ub`
> boleh menyerah tanpa membuat "tidak ditemukan" berarti "tidak ada"; (c)
> anggaran 120 s ternyata tidak pernah mengikat wall, dan memperbaikinya
> mengubah **setiap** angka yang pernah dikutip. Ketiganya keputusan rancangan
> yang salahnya hanya terlihat sebagai angka yang masuk akal.

```
Sesi G14 -- REACH-4: kenapa S2, dan apa arti "tidak ditemukan".
Sapuan LUNAS (9+1 sel, S1 dan S2, c_arm 0.00-0.20). Oracle LUNAS (M3 menyala,
11 instance mengikat, 0 solver>W3 DAN 0 solver<W3). Yang tersisa bukan
pengukuran ulang -- ia tiga pertanyaan yang G13 baru bisa ajukan.

BACA DULU:
1. docs/p1_g13_sweep.md -- SELURUHNYA. Khususnya:
   B2 (anggaran 120 s BUKAN batas wall -- berlaku SURUT ke G7-G12),
   B3 (brute_arm TIDAK menyeluruh; dan kenapa "0 selisih enumerator" tidak
       membuktikan apa pun -- B9 bacaan 4),
   B5.4 (Delta_arm POSITIF pertama, +5.40%, S2, terbukti),
   B5.5 (TIDAK ADA JADWAL = ANGGARAN; dan kenapa inferensi p1_g12 B6
         "serialisasi selalu ada" SALAH),
   B6 (bug kedua, arm_first_block; M1/M2b G12 hanya menguji S1),
   B7 (n6_s4_mr0: 96% wall di dua PEMERIKSA, _dive nol),
   B8 (max_evade TERBANTAH; satu kandidat tersisa, belum diukur),
   B10 (batasan -- terutama 1, 3, 4, 6, 8, 10)
2. docs/p1_g12_armsolve.md B2 (kenapa substitusi predikat mustahil), B4
   (Lemma B teorema bukan mekanisme), B7 (angka utama G12)
3. reachability_gng/sched_armfull.py, sched_arm.py, test/m3_bind_g13.py

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.
CATATAN MESIN: G13 diukur pada load 6-8.5 dari 16 core (DI BAWAH JENUH, itu
pembenarannya, bukan konkurensi 3-nya). G12 4.5-5.0, G11 3.3-3.7, G10 42.

=== YANG SUDAH TEGAK, JANGAN BANGUN ULANG ===
- Sapuan penuh: S1 dan S2, c_arm {0.00,0.05,0.10,0.15,0.20}, 40 instance/sel,
  0 kegagalan gerbang nyata, 0 pelanggaran Lemma 3. Angkanya ada di
  /tmp/g13_eval_{s1,s2}_{c}.json dan di B5. PAKAI, jangan hitung ulang.
- ANGKA UTAMA: Delta_arm = 0.0000 TERBUKTI pada SETIAP instance S1 yang bisa
  dibuktikan, kelima c_arm sampai 0.20 (147 bukti, nol contoh tandingan).
  S2 sama sampai c_arm <= 0.10 (73 bukti).
- M3 LULUS dan DISKRIMINATIF: 11 instance mengikat, 0 solver>W3, 0 solver<W3.
  brute_arm SUDAH DIPERBAIKI. Jangan pakai angka M3 p1_g12 B8 -- ia diukur
  dengan enumerator cacat.
- Lemma C LULUS di 0.05/0.15/0.20 (107 perhentian, 0 tanpa pose parkir,
  min fraksi bebas 83%/56%/43%).
- M2a LULUS (delta <= 0.004 pada 2376^2 di kelima c_arm) -- G12 tidak
  menjalankannya.
- U4 LUNAS SEBAGIAN: max_evade BUKAN sebabnya, 0 dari 9. Jangan ulangi.

=== TUGAS, BERURUTAN. JANGAN LOMPAT. ===
1. ANGGARAN 120 s (B10.1). Ia tidak pernah membatasi wall. Putuskan SATU dari
   dua, dan tulis mana SEBELUM mengukur: (a) anggaran ditegakkan sungguhan --
   maka SETIAP angka G7-G13 berubah dan harus dijalankan ulang, katakan
   berapa banyak; (b) anggaran didefinisikan ulang sebagai "anggaran
   PENCARIAN" dan wall dilaporkan terpisah -- maka tidak ada angka yang
   berubah, tapi kalimatnya berubah di seluruh naskah. JANGAN campur.
2. KENAPA S2 DAN BUKAN S1. Satu-satunya Delta_arm positif ada di
   n6_s0_mr1 pada S2, +0.5000 s = +5.40%, terbukti, c_arm 0.15 DAN 0.20.
   S2 membatasi HIMPUNAN POSE (p1_g11 B3). Hipotesis yang bisa diuji:
   harga lengan muncul ketika ruang keluar dibatasi, bukan ketika clearance
   dinaikkan. UJI: kecilkan |P| pada S1 secara terkendali dan lihat apakah
   Delta_arm positif muncul. Kalau ya, itu KALIMAT NASKAH -- harga koordinasi
   ditentukan kekayaan ruang pose, bukan geometri lengan.
3. TIDAK ADA JADWAL DITEMUKAN (B5.5): 4/6/1/11 instance. Lemma C lulus, jadi
   ini anggaran. Dua jalan, pilih dengan mengukur BUKAN dengan menebak:
   (a) beri arm_serial_ub anggaran waktu + urutan pose yang lebih baik
       (lemma_c() sudah menghitung pose bebas untuk SELURUH 2376 secara
       tervektorisasi dalam 0.23 s -- pakai sebagai PRAPENYARING, dan
       BUKTIKAN ia syarat perlu sebelum memakainya);
   (b) terima dan laporkan sebagai kurungan.
   Kalau (a): itu mengubah sched_armfull.py yang BEKU, jadi M0 harus
   dijalankan ulang dan itu bagian dari harganya.
4. TUAS 3 + 4 M3 (B10.6). gen_forced SUDAH ADA dan TIDAK DIUKUR. Tuas 3 tipis
   (6+6 dari 864) tapi ada. Pertanyaannya bukan lagi "bisakah dibangun" --
   sudah bisa -- melainkan apakah menutup jalan keluar "menunggu" MENAIKKAN
   fraksi mengikat di c_arm = 0.05, yaitu di nilai utama.
5. U4 sisa (B8): satu kandidat tersisa, grid seragam vs waktu mulai kontinu.
   Ambil satu dari 9 instance, ambil jadwal solver, dan periksa apakah waktu
   berangkatnya ada di grid W2. Itu uji lima menit yang G10 tidak jalankan.

=== KUNCI KRITERIA SEBELUM KODE, ke docs/p1_g14_*.md A ===
1. Keputusan anggaran (tugas 1) ditulis SEBELUM mengukur apa pun, dengan
   jumlah angka yang harus dijalankan ulang kalau (a) dipilih.
2. Apa yang membuat tugas 2 "menjawab" -- berapa penyusutan |P| dianggap
   terkendali, dan apa yang membedakan "harga muncul" dari "instance lain".
   DITULIS SEBELUM MEMBANGUNNYA. Ini persis jebakan M3 G12.
3. Kalau tugas 3 memilih (a): M0 dijalankan ulang adalah GERBANG, bukan
   catatan. Tidak ada angka baru sebelum ia lulus.
4. Apa yang dilaporkan kalau lagi-lagi tidak semuanya bisa dibuktikan.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- ENAM SESI: yang lambat adalah PEMERIKSA, bukan pencarian. n6_s4_mr0 = 96%
  wall di gerbang lengan + feasible_starts; _dive nol.
- Anggaran diverifikasi pada satu titik operasi bisa salah 3500x di titik
  lain. arm_serial_ub: 0.039 s @0.05, 138 s @0.20. Ukur di UJUNG sapuan.
- Instrumen kedua yang ditulis dari BENTUK yang sama mewarisi cacat yang sama.
  enum_opt vs brute_arm: 0 selisih, dan dua-duanya salah. Silang-periksa hanya
  bernilai kalau artefaknya tidak berbagi bentuk.
- Gerbang yang hanya pernah dijalankan pada S1 belum diuji. M1 40/40 dan M2b
  0/1864 dua-duanya HANYA S1; jadwal S2 pertama langsung menemukan bug.
- Tepi kanan TERTUTUP adalah kelas bug berulang: p1_g12 B3.1 di polys(),
  p1_g13 B6 di moving(). Tanya keadaan pada TITIK TENGAH POTONGAN.
- Lemma C hanya menyertifikasi fase PARKIR. Ia TIDAK menjamin arm_serial_ub
  mengembalikan sesuatu. p1_g12 B6 menyimpulkan sebaliknya dan itu salah.
- Sel kalau dijalankan serentak: satu berkas per (set, c_arm), dan beban
  dicatat. Beban di BAWAH jumlah core = tidak ada throttling; itu
  pembenarannya, bukan angka konkurensinya.
- `tail` pada proses latar MENELAN keluaran sampai proses selesai. Tulis per
  baris ke berkas.
- np.bool_ tidak JSON-serializable.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 25 meleset, 12 tepat.
  Prior DUNIA: LONGGAR -- dan sekarang punya SATU contoh tandingan terbukti
  (B5.4), yang datang dari HIMPUNAN POSE DIBATASI, bukan dari clearance.
  Prior KODE SENDIRI: arahnya TIDAK LAGI BISA DISEBUT (5 dari 9, dua sesi
  meleset "terlalu lambat", sesi ini dua meleset "terlalu cepat"). Yang
  bertahan lebih sempit: yang mahal selalu PEMERIKSA.
  Prior PALING BERGUNA: setiap bug nyata ditemukan oleh UJI YANG DIJALANKAN,
  nol oleh pembacaan ulang. ENAM sesi berturut-turut, dan G13 menambahkan
  syaratnya: uji yang dijalankan pada BAHAN YANG BELUM PERNAH DIPAKAI.
- DUGAAN YANG DINILAI MEMAKAI SOLVER YANG BELUM LULUS GERBANGNYA TIDAK
  DINILAI. Tandai TERTUNDA.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G10 lima, G11 empat, G12 dua, G13 dua -- dan yang kedua kali ini bertentangan
  dengan G12, bukan dengan A-nya sendiri.
- Rule 6 (30k token) adalah PENGECUALIAN EKSPLISIT untuk sesi protokol-panjang
  P1, dinyatakan di muka (A6), bukan dilaporkan sesudahnya.
- Akhiri dengan prompt sesi berikutnya (G15).
```
