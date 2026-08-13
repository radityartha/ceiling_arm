# P1 / G7 — SCHED-1: solver exact sebagai GROUND TRUTH

> Sesi G7, 2026-08-14. Melanjutkan [p1_next_steps.md](p1_next_steps.md) Jalur A.
> Prompt sesi: [p1_prompt_sched1.md](p1_prompt_sched1.md).
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode solver dijalankan.** §B diisi
> sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**, bukan dihaluskan.
>
> Sesi ini **sepenuhnya offline**. Tidak ada kamera, gantry, lengan, `move_group`.
>
> Lingkup sesi: **langkah 1 dan 2** dari `p1_state §7.1` — generator instance +
> solver exact. **Heuristik (langkah 3) dan baseline (langkah 4) BUKAN bagian
> sesi ini** dan tidak boleh ditulis di sini.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| Index capability = **grid** | `p1_state §5.2` | dibaca apa adanya, nol pengukuran ulang |
| Peta `cap_g{1,2}_rail160.npz` (33×72) | `p1_state §3` | satu-satunya oracle kelayakan |
| Biaya setup `T_traverse = max(T_lin, T_rot)` | `p1_state §5.6` | dipakai, **tidak** diturunkan ulang |
| Dwell 2.0 s kontinu | `p1_state §5.8 / §8b` | durasi tugas |
| Batas rel 1600 mm | `p1_state §5.7` | sudah tertanam di `lin` peta (33 kolom) |
| `gng.py` | `p1_g5 §A0` | nol perubahan |
| Formulasi `XD [ST-MR-TA]` | `p1_state §5.1` | tidak ditawar |

Konstanta yang dipakai, disalin apa adanya dari §5.6 (**bukan** diukur ulang):

```
v_lin = 3000 / 95.4930 = 31.4160 mm/s      (bridge.linear_speed = 3000 pulse/s)
v_rot = 1000 / 100.0   = 10.0    deg/s     (bridge.rotate_speed = 1000 pulse/s)
T_lin(Δmm)  = 0.29 + Δmm  / v_lin          hanya bila Δmm  > 0, else 0
T_rot(Δdeg) = 0.26 + Δdeg / v_rot          hanya bila Δdeg > 0, else 0
T_traverse  = max(T_lin, T_rot)            KONKUREN
```

⚠️ **Offset hanya dikenakan pada sumbu yang benar-benar bergerak.** §5.6 menulis
`T_lin(0) = 0.29`, tapi itu biaya perintah untuk sumbu yang **diperintah**; kalau
sumbu linier tidak diperintah sama sekali ia tidak menyumbang. Tanpa aturan ini,
`T_traverse(p, p) = 0.29 s` — gantry membayar untuk **tidak** bergerak, dan itu
jelas salah. Konsekuensi teknis yang dipakai solver: **ketaksamaan segitiga tetap
berlaku** (lihat A2-K4), jadi mengunjungi ulang sebuah pose tidak pernah
menguntungkan, dan DP tetap exact walau ia tetap mengizinkannya.

Grid pose gantry, dibaca dari npz (bukan diketik ulang): `lin` = 33 nilai
0.00…1.60 m langkah 0.05 m; `rot` = 72 nilai **radian**, setengah-terbuka
[−180°, 180°) langkah 5°. Rotasi **siklik** → jarak rotasi = jarak melingkar
`min(|Δ|, 360−|Δ|)`. |P| = 2376 pose per gantry.

### A1. MODEL — ditulis lengkap sekarang supaya §B tidak bisa menggesernya

Ini bagian terpenting §A. Semua yang tidak tertulis di sini **tidak dimodelkan**,
dan A4 mendaftar akibatnya.

**Sumber daya.** Dua gantry, `g ∈ {1, 2}`. Gantry 1 membawa `arm1` (kanan) +
`arm2` (kiri); gantry 2 membawa `arm3` + `arm4`. Dua lengan satu gantry berjarak
setengah putaran persis (`np.roll(mask, 36)`, eksak 2e-16 — `p1_state §3`).
Pose gantry `p = (lin, rot)` **dibagi oleh kedua lengannya** — inilah `XD`-nya.

**Garis waktu satu gantry.** Barisan **perhentian** (stop):

```
p_g^0  --traverse-->  p_g^1  --traverse-->  ...  --traverse-->  p_g^K
       [ kerja ]              [ kerja ]                          [ kerja ]
```

Kedua lengan gantry **berhenti total** selama traverse (tidak ada kerja saat
gantry bergerak). Kerja hanya terjadi di dalam perhentian, jadi blok kerja per
perhentian **kontinu menurut definisi**.

**Tugas.** `n` tugas reach-and-dwell, masing-masing durasi `d = 2.0 s` (§5.8).
- **SR**: dikerjakan **tepat satu** lengan `a`, pada pose `p` dengan
  `mask_a[t, p] = 1` pada toleransi L1 = 0.05 m.
- **MR (handover)**: dikerjakan **dua lengan satu gantry SERENTAK**, pada pose
  `p` di mana **kedua** lengan menjangkau ketat (tol 0.05). Satu jendela 2.0 s
  bersama; kedua lengan sibuk penuh selama jendela itu.

**Mutex sumber daya (r = 0.20).** Untuk gantry pada pose `p`, tugas `t` disebut
**menempati zona** bagi lengan `a` bila
`mask_a[t, p, tol=0.05] ∧ mask_partner(a)[t, p, tol=0.20]` — yaitu `a` benar-benar
menjangkaunya **dan** titik itu berada dalam 20 cm dari himpunan terjangkau
partnernya pada pose yang sama. **Paling banyak satu lengan boleh menempati zona
pada satu waktu**, per gantry. Tugas MR menempati mutex selama jendelanya (memang
begitu maunya — handover *menuntut* irisan).

Terukur pada peta nyata: **51.8%** pasangan (node, pose) yang terjangkau `arm1`
juga menempati zona. Jadi mutex ini **mengikat**, bukan hiasan.

**Durasi satu perhentian — bentuk tertutup.** Semua durasi tugas sama (2.0 s),
jadi perhentian bisa dinormalkan ke **slot** selebar 2.0 s. Untuk himpunan tugas
`U` pada pose `p`, dengan `m` = jumlah tugas MR, `s_A`/`s_B` = jumlah tugas SR
yang ditugaskan ke lengan A/B, dan `z_A`/`z_B` = berapa di antaranya menempati
zona:

```
a = m + s_A          beban lengan A, dalam slot
b = m + s_B          beban lengan B
z = m + z_A + z_B    permintaan terhadap mutex tunggal
dur(U, p) = 2.0 s × max(a, b, z)
```

Batas bawah jelas (tiap lengan butuh slotnya sendiri; mutex unary menyerialkan
`z` slot). Tercapai: taruh MR di slot `1..m`, zona-A di `m+1..m+z_A`, zona-B di
`m+z_A+1..m+z_A+z_B`, lalu isi sisa tugas non-zona ke slot yang masih kosong per
lengan — selalu cukup karena `M ≥ a` dan `M ≥ b`.

🔴 **Bentuk tertutup ini adalah bagian model yang paling mungkin salah, dan ia
tidak boleh dipercaya karena buktinya terlihat rapi.** K4 mewajibkan ia
diverifikasi terhadap enumerasi slot brute-force yang **tidak memakai rumus ini
sama sekali**.

**Objektif.** Minimalkan makespan (A2-K2).

**Penugasan lengan bersifat lokal per perhentian** — satu tugas SR muncul di
tepat satu perhentian, jadi pilihan lengannya tidak berinteraksi dengan
perhentian lain. Ini dipakai solver, dan disebut di sini supaya bisa dibantah.

### A2. Kriteria yang DIKUNCI

#### K1 — UKURAN INSTANCE, sebagai angka

| | Angka terkunci |
|---|---|
| Tugas | `2 ≤ n ≤ 6` |
| Lengan | **4** (2 gantry × 2 lengan); instance 1-gantry (2 lengan) juga sah |
| Kandidat pose | **|P| = 2376 penuh per gantry** (33 × 72), tanpa penjarangan |
| Anggaran waktu solver exact | **≤ 120 s dinding per instance** |
| Anggaran memori | **≤ 4 GB** |
| Silang brute-force | `n ≤ 4`, `|P| ≤ 8`, himpunan pose **identik** untuk kedua solver |

Kalau `n = 6` pada `|P| = 2376` melampaui 120 s, **yang dilaporkan adalah `n`
terbesar yang muat, sebagai angka terukur** — bukan anggarannya yang dilonggarkan
diam-diam. Penjarangan pose **tidak** boleh dipakai untuk menyelamatkan angka;
kalau dipakai, ia disebut sebagai penjarangan dan solver disebut exact
**terhadap himpunan kandidat itu**, bukan exact.

#### K2 — DEFINISI MAKESPAN

```
t = 0   : kedua gantry berada di pose awal p_g^0 (bagian dari instance),
          keempat lengan menganggur dan siap. Tidak ada biaya start-up.
makespan: waktu saat jendela dwell tugas TERAKHIR SELESAI,
          diambil maksimum atas kedua gantry.
```

**Yang TIDAK termasuk**, ditulis eksplisit supaya tidak ditambahkan diam-diam
belakangan: tidak ada kembali-ke-home, tidak ada lipat penutup, tidak ada biaya
persepsi, tidak ada waktu perencanaan, tidak ada gerak lengan antar tugas di
dalam satu pose (hanya dwell). Satu-satunya biaya adalah **traverse gantry** +
**dwell**.

Pose awal `p_g^0` adalah **bagian dari instance**, bukan konstanta global —
karena `T_traverse` bergantung urutan, makespan tanpa pose awal tidak
terdefinisi.

#### K3 — HANDOVER = **SATU tugas dua-lengan**, bukan dua tugas tergandeng

**Dipilih: satu tugas, dua lengan, satu jendela 2.0 s bersama.**

Alasannya bukan selera, melainkan definisi sukses yang **sudah terkunci** di
`p1_state §5.8`:

> "Sukses N-lengan = **SATU jendela bersama** di mana semua N memenuhi toleransi
> **serentak** — bukan 'masing-masing sukses di suatu titik', karena itu bisa
> dipenuhi **bergantian**, dan bergantian persis yang dilakukan prior work."

Dua tugas tergandeng adalah representasi yang **mengizinkan** pola bergantian
itu (dua tugas dengan presedensi, atau dengan jendela yang cuma bersinggungan),
yaitu pola yang §5.8 tolak. Memakainya akan membuat model penjadwalan
bertentangan dengan kriteria sukses yang dipakai mengukur hardware — dan
`p1_state §7` melarang membiarkan dua kriteria yang bertentangan hidup
berdampingan.

Konsekuensi struktural yang diterima sadar: tugas MR memaksa **kedua** lengan
satu gantry menganggur bersamaan **dan** memaksa pose gantry ke irisan ketat.
Itu justru kopling `XD` yang ingin diukur.

#### K4 — BUKTI OPTIMALITAS

Solver exact = **DP atas (himpunan tugas selesai, pose sekarang)**, bukan MIP.
Alasan memilih DP disebut sekarang supaya tidak terdengar seperti pembenaran
belakangan: biaya setup `T_traverse` **kontinu** dan bergantung urutan, jadi MILP
ter-indeks-waktu hanya exact setelah waktu didiskretkan, dan diskretisasi itu
akan jadi sumber galat yang tidak bisa dipisahkan dari galat model. DP atas
subset **tidak mendiskretkan waktu sama sekali**.

**Solver tanpa pembanding independen bukan ground truth, ia cuma solver kedua.**
Maka tiga tingkat verifikasi, ketiganya WAJIB LULUS:

| | Apa | Independen dalam hal apa |
|---|---|---|
| **V1** | Bentuk tertutup `dur(U, p)` vs **enumerasi slot brute-force** | tidak memakai rumus `max(a,b,z)` sama sekali; mencari jadwal slot per lengan secara tuntas dan mengecek mutex langsung |
| **V2** | DP vs **enumerator jadwal brute-force** (partisi terurut × pose × penugasan lengan, tanpa memo, tanpa min-plus) | algoritma berbeda, kode berbeda, tanpa berbagi fungsi biaya perhentian |
| **V3** | DP vs **jawaban tertutup instance patologis** (A3) yang diketahui **tanpa solver mana pun** | tidak ada solver di sisi kebenarannya |

V2 dijalankan pada instance acak (`n ≤ 4`, `|P| ≤ 8`) dengan **himpunan pose yang
sama persis**, dan harus cocok pada **makespan optimal**, sampai `1e-9`.
Jadwal boleh berbeda (optimum berganda) — yang dibandingkan **nilai**, bukan
jadwal.

🔴 **Kalau V1, V2, atau V3 gagal dan tidak bisa dijelaskan, sesi ini melaporkan
KEGAGALAN dan berhenti.** Dilarang menurunkan solver jadi "heuristik yang bagus"
lalu tetap menyebutnya ground truth. Dilarang melonggarkan toleransi
perbandingan agar cocok.

#### K5 — INSTANCE PATOLOGIS, jawabannya ditetapkan SEKARANG

Ditulis sebelum solver pernah dijalankan sekali pun. Semua memakai mask
**sintetis** (dibentuk tangan), bukan peta nyata, supaya jawabannya betul-betul
diketahui. `d = 2.0 s`.

| # | Nama | Konstruksi | Makespan yang DIHARAPKAN | Yang diisolasi |
|---|---|---|---|---|
| **P1** | serial paksa | `n` tugas SR, **hanya** terjangkau `arm1`, semuanya di pose awal, non-zona | `n × 2.0` | `ST`: satu lengan, satu tugas pada satu waktu |
| **P2** | paralel sempurna | 2 tugas SR di pose awal, satu hanya `arm1`, satu hanya `arm2`, **non-zona** | `2.0` | konkurensi dua lengan se-gantry |
| **P3** | traverse murni | 1 tugas SR, hanya terjangkau di pose `p ≠ p_0` | `T_traverse(p_0, p) + 2.0` | biaya setup |
| **P4** | mutex menggigit | seperti P2 tapi **kedua** tugas menempati zona | `4.0` (bukan 2.0) | mutex `r = 0.20` |
| **P5** | handover | 1 tugas MR di pose awal | `2.0` | MR = satu jendela dua lengan |
| **P5b** | handover + zona | P5 ditambah 1 tugas SR zona untuk `arm1`, pose awal | `4.0` | MR ikut memakai mutex |
| **P6** | tur pose | `k` tugas, masing-masing hanya terjangkau di satu pose berbeda | `min` atas permutasi `Σ T_traverse` + `k × 2.0` | urutan bergantung-setup (TSP kecil) |
| **P7** | dua gantry lepas | 2 tugas identik, satu hanya terjangkau gantry 1, satu hanya gantry 2, pose awal | `2.0` | makespan = **maks**, bukan jumlah |

Nilai P6 dihitung dari permutasi lengkap **di dalam berkas uji**, dari rumus
§5.6 langsung — bukan dari DP.

**P4 adalah yang paling penting.** Kalau P4 keluar 2.0, mutex tidak terpasang,
dan seluruh argumen kuantitatif inti paper (`p1_state §2`: 87.5% → 61.8%, "selisih
25 poin itu yang dibeli scheduling") tidak punya wakil di dalam model.

### A3. Generator instance

Dua mode, dan **keduanya wajib ada** — mode sintetis untuk K5, mode nyata untuk
angka yang boleh dikutip:

1. **`synthetic`** — mask dibentuk tangan. Dipakai HANYA untuk A2-K5 dan V2.
2. **`real`** — titik tugas diambil dari node `cap_g{1,2}_rail160.npz`, mask
   dibaca dari oracle apa adanya. Ini satu-satunya mode yang angkanya boleh
   masuk naskah.

Generator ber-**seed** dan deterministik: `seed` yang sama → instance
bit-identik. (Disiplin `p1_g5 §B1`: "tidak invarian terhadap urutan" adalah cacat
yang nyata dan mudah tak terlihat — generator ini tidak boleh mengulanginya.)

Instance mode `real` yang tidak layak **dibuang, bukan ditambal**: tugas SR wajib
terjangkau setidaknya satu lengan pada setidaknya satu pose; tugas MR wajib punya
setidaknya satu pose dengan irisan ketat. Jumlah yang dibuang **dilaporkan sebagai
angka**.

### A4. Yang TIDAK dimodelkan — ditulis SEKARANG, bukan ditemukan di akhir

1. 🔴 **Tabrakan struktur gantry–gantry BELUM dimodelkan.** Pelat mount menyapu
   lingkaran `r = 0.4 m` di `y = ±0.36`, jadi beririsan di `y ∈ [−0.04, 0.04]`.
   Proksi polyline **meremehkan** volume sapuan. Akibatnya: **semua angka rugi
   dari model ini adalah BATAS BAWAH, bukan nilai.** Jadwal optimum yang
   dihasilkan bisa memerintahkan konfigurasi `(lin₁, rot₁, lin₂, rot₂)` yang
   merusak hardware; jangan pernah dieksekusi tanpa lapis pengaman.
2. **Konsekuensi langsung dari (1) yang harus disebut, bukan disembunyikan:**
   dalam model ini kedua gantry **hanya** berkopling lewat penugasan tugas →
   gantry. Tidak ada kopling waktu antar gantry. Jadi `XD` yang benar-benar
   dimodelkan sesi ini adalah kopling **di dalam** satu gantry (pose bersama +
   mutex), sesuai `p1_state §1` ("kopling = variabel pose bersama"). Kopling
   antar gantry menunggu (1).
3. **Biaya lipat/rentang-ulang lengan tidak diketahui.** `p1_state §7.3` menuntut
   `biaya pindah = lipat + traverse + rentang-ulang`, tapi hanya traverse yang
   pernah **diukur** (§5.6). §7.2 melarang mengarang konstanta. Maka: parameter
   `T_fold` ada di model, **default 0.0**, dan dilaporkan sebagai suku aditif yang
   hilang. Ia menggeser setiap traverse dengan konstanta yang sama, jadi ia
   **memperbesar** nilai gerak gantry — arah biasnya diketahui.
4. **Waktu gerak lengan di dalam satu pose = 0.** Hanya dwell yang dihitung.
   Ini meremehkan durasi perhentian, lagi-lagi ke arah yang sama.
5. **Kapabilitas dianggap statis.** `p1_state §1` menyebut "kapabilitas yang
   berubah online"; penghalang dinamis (19.6%, `p1_state §2`) tidak ada di model
   ini.
6. **Toleransi L1 = 0.05 m adalah pembangkit kandidat, bukan akurasi** (`§8b`:
   L1/L2/L3 tiga hal berbeda). Kelayakan di model ini berarti "ada solusi di
   sekitar sini".

### A5. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

`p1_state §7.2`: **sembilan dugaan meleset, semuanya ke arah yang sama** —
menduga kendala **lebih mengikat** daripada kenyataannya. Maka dugaan sesi ini
ditulis sekarang supaya bisa dinilai, bukan dirasionalisasi belakangan:

| # | Dugaan | Arah kalau pola sembilan-dari-sembilan berulang |
|---|---|---|
| D1 | `n = 6` pada `|P| = 2376` **tidak** muat di 120 s; perlu penjarangan pose | pola meramalkan ini **terlalu pesimis** — kemungkinan muat |
| D2 | Mutex `r = 0.20` mengikat pada instance nyata (makespan naik vs tanpa mutex) | pola meramalkan **kurang mengikat** dari dugaan |
| D3 | Tugas MR jauh lebih mahal karena mengunci dua lengan + pose | pola meramalkan **lebih murah** dari dugaan |

Ketiganya **diukur**, tidak diperdebatkan. Hasilnya masuk §B apa adanya,
termasuk kalau ketiganya meleset lagi ke arah yang sama.

### A6. Berkas

| Berkas | Isi |
|---|---|
| `reachability_gng/sched.py` | model + generator + solver exact (DP) + CLI |
| `test/verify_sched_exact.py` | V1 + V2 + V3 — **oracle independen**, tidak mengimpor fungsi biaya `sched.py` |
| `docs/p1_g7_sched.md` | dokumen ini |

Tidak ada berkas lain yang disentuh. `gng.py`, `capability.py`, `irm_sweep.py`
**dibaca saja**.

---

## B. Hasil terukur

> §A dikunci sebelum solver dijalankan sekali pun. Semua angka di bawah ini
> keluar sesudahnya. **Tiga tempat di mana §B bertentangan dengan §A ditandai
> 🔺 dan ditulis eksplisit, sesuai aturan sesi. §A TIDAK ditulis ulang** — ia
> tetap seperti saat dikunci, termasuk kalimat-kalimatnya yang ternyata salah.

### B0. Cara menjalankan ulang

```bash
cd /home/user1/Documents/ceiling_arm
python3 ros2_ws/src/reachability_gng/test/verify_sched_exact.py   # V0-V4
python3 -m reachability_gng.sched solve  --n-tasks 6 --n-mr 1 --gantries 1 2
python3 -m reachability_gng.sched sweep  --tasks 2 3 4 5 6        # K1
python3 -m reachability_gng.sched ablate --n-tasks 6 --n-mr 1     # D2, D3
python3 -m reachability_gng.sched diag   --n-tasks 6              # kenapa D2
```

Peta `cap_g{1,2}_rail160.npz` dibaca apa adanya; tidak ada yang dibangun ulang.

### B1. Bukti optimalitas — LIMA tingkat, SEMUA LULUS

| | Apa yang diadu | Cakupan | Hasil |
|---|---|---|---|
| **V0** | `traverse_matrix` vs rumus §5.6 diketik ulang skalar | 1 600 pasangan pose | maks \|selisih\| **5.3e-15** |
| **V1** | bentuk tertutup `max(a,b,z)` vs **enumerasi slot tuntas** | 2 700 kasus (himpunan tugas × pose) | **0 ketidakcocokan** |
| **V2** | DP vs **enumerator jadwal brute-force** | **96 instance**, `n ≤ 4`, `|P| ≤ 6`, 1 dan 2 gantry, MR 0/1/2 | **0 ketidakcocokan** |
| **V3** | DP vs **jawaban tertutup A2-K5** | 8 instance patologis | **8/8 LULUS** |
| **V4** | jadwal yang DILAPORKAN diputar ulang dari aturan | 33 jadwal (sintetis + nyata) | **0 pelanggaran** |

**V4 adalah tambahan di luar §A.** K4 menuntut tiga tingkat; tingkat keempat
ditambahkan karena ketiganya hanya mengadu **nilai optimal**, dan nilai yang
benar tidak membuktikan **jadwal** yang dikembalikan bersamanya bisa dieksekusi.
V4 memutar ulang setiap perhentian dari aturan model (traverse skalar, panjang
perhentian dari enumerasi slot, kelayakan langsung dari oracle) dan mengecek
setiap tugas dikerjakan tepat sekali, setiap penugasan lengan sah, waktu mulai
konsisten, dan makespan benar-benar maksimum atas kedua gantry.

➜ **Solver ini boleh disebut GROUND TRUTH.** Toleransi 1e-9, tidak dilonggarkan.

Perlu dicatat sebagai batasan yang jujur: V2 exact hanya pada `|P| ≤ 6`, karena
enumerator brute-force tidak bisa lebih besar. Yang diuji pada `|P| = 2376`
adalah **algoritma yang sama** yang lulus di `|P| ≤ 6`, ditambah V4 yang jalan
langsung di instance nyata `|P| = 2376`.

### B2. 🔺 K1 — ukuran instance: §A TERLALU PESIMIS, batasnya `n = 10`

`|P| = 2376` penuh (33 × 72), 2 gantry, 4 lengan, peta nyata, tanpa penjarangan:

| n | wall 1 gantry | wall 2 gantry | puncak memori |
|---|---|---|---|
| 4 | 0.66 s | 1.15 s | |
| 6 | 2.11 s | 4.06 s | |
| 8 | 10.25 s | 17.41 s | |
| 9 | | 37.84 s | 435 MB |
| **10** | | **83.84 s** | **499 MB** |
| 11 | | 188.87 s ❌ | 573 MB |

**§A-K1 mengunci `2 ≤ n ≤ 6`. Terukur: `n = 10` muat di anggaran 120 s.
§B menang** — batas yang dilaporkan adalah **`n ≤ 10` tugas, 4 lengan,
`|P| = 2376` penuh, ≤ 500 MB**. Anggaran 120 s tidak dilonggarkan; `n = 11`
memang di luar dan dilaporkan begitu.

Biayanya `~2.1×` per tugas tambahan, sesuai bentuk DP-nya: `3ⁿ` operasi vektor
untuk `w`, `2ⁿ` operasi min-plus `|P| × |P|` untuk `h`. Yang membatasi adalah
min-plus, bukan enumerasi subsetnya.

### B3. 🔺 D2 — mutex `r = 0.20` TIDAK PERNAH berbiaya. Ini temuan utama sesi ini.

**105 pasangan solve (dengan mutex / tanpa mutex), selisih makespan `0.000 s`
pada SETIAP satu pasang.** Cakupannya: `n ∈ {4, 6, 8, 10}`, 1 dan 2 gantry,
dengan dan tanpa tugas MR, 10 seed per konfigurasi.

Dan ia tetap nol di rezim yang sengaja dibuat agar mutex mengikat — sensitivitas
dwell (**dwell TETAP TERKUNCI 2.0 s di §5.8**; ini hanya menjawab "rezim seperti
apa yang akan membuatnya mengikat"):

| dwell | makespan | pangsa dwell | tugas maks / perhentian | mutex mengikat |
|---|---|---|---|---|
| 2.0 s | 43.30 s | 16.5% | 5 | **0%** |
| 10 s | 72.10 s | 49.5% | 4 | **0%** |
| 30 s | 144.10 s | 74.5% | 5 | **0%** |
| 60 s | 252.10 s | 85.4% | 5 | **0%** |
| 120 s | 468.10 s | 92.1% | 5 | **0%** |

**Sebabnya diukur, bukan ditakar** (`sched diag`, `n = 6`, 8 seed, 1 gantry):

| | Pertanyaan | Jawaban |
|---|---|---|
| **Q1** | Apakah mutex mengikat *secara lokal* sama sekali? | **Ya — 10.9%** (18 370 / 168 830) pasangan (himpunan tugas, pose) benar-benar membayar slot tambahan. Modelnya **tidak vacuous**. |
| **Q2** | Apakah selalu ada jalan keluar? | **Ya — 92%** (55 dari 60) himpunan tugas yang mengikat di suatu pose punya pose lain yang tetap layak dan bebas-mutex **tanpa slot tambahan**. |
| **Q3** | Apakah perhentian yang dipakai optimum membayar? | **Tidak — 0 dari 25 perhentian.** Optimum memakai rata-rata **2.40 tugas per perhentian**. |

Rantai sebabnya lengkap: mutex baru berbiaya kalau `z > max(a, b)`, yaitu butuh
**≥3 tugas zona di satu perhentian dengan beban dua lengan seimbang**. Optimum
tidak pernah membangun perhentian sebesar itu, karena garis waktunya
**didominasi setup** (§B5), dan kalaupun ia terpaksa, tersedia pose lain yang
gratis pada 92% kasus. Perhatikan juga jalan keluar yang lebih murah dan sering
terlewat: **memindahkan kedua tugas ke lengan yang SAMA berbiaya sama persis**
dengan diserialkan mutex (`max(a,b) = z = 2`), jadi "dipaksa bergantian" sering
kali bukan kerugian sama sekali.

🔺 **Kalimat terakhir §A1 — "Jadi mutex ini mengikat, bukan hiasan" — GUGUR.**
Yang benar dari kalimat itu hanya statistik masknya (51.8% pasangan (node, pose)
menempati zona; itu tetap terukur). Lompatan dari "zonanya sering" ke "mutexnya
mengikat" **tidak punya dasar dan sekarang terbantah**: sering ≠ mahal. §A
sengaja tidak diperbaiki, supaya kesalahannya bisa dibaca.

**Pertentangan dengan `p1_state §2`, ditulis terbuka, bukan dihaluskan.**
`p1_state §2` menyebut zona-eksklusi `r = 0.20` sebagai kendala yang **mengikat**
dan menjadikan "selisih 25 poin (87.5% → 61.8%) itu yang dibeli scheduling"
sebagai argumen kuantitatif inti. Yang diukur sesi ini **tidak membantah** angka
itu, tapi **membatasi tafsirnya secara tajam**:

- 87.5% → 61.8% adalah pernyataan tentang **ko-kelayakan/konkurensi statis**.
  Itu tetap sah.
- **Berapa detik yang dibeli scheduling adalah pertanyaan lain, dan jawabannya
  di sini `0.000 s`.** Membagi zona dalam waktu memang menyelamatkan konkurensi,
  tapi konkurensi itu **tidak pernah berada di jalur kritis** pada `n ≤ 10`
  dengan dwell 2.0 s.
- ⚠️ Maka **dilarang menulis "selisih 25 poin itu yang dibeli scheduling" sebagai
  klaim makespan/throughput.** Sebagai klaim kelayakan ia benar; sebagai klaim
  waktu ia belum punya bukti, dan bukti yang ada saat ini menunjuk ke arah nol.

**Yang TIDAK boleh disimpulkan:** bahwa mutexnya salah, atau boleh dibuang. Q1
menunjukkan ia mengikat pada 10.9% pasangan; P4 menunjukkan modelnya menangkapnya
persis; dan tabrakan struktur gantry–gantry (§A4.1) **belum dimodelkan sama
sekali**, sehingga biaya koordinasi sebenarnya **hanya bisa naik** dari sini.
Yang benar dikatakan: *pada instance yang optimumnya bisa dihitung, mutex bukan
penggerak makespan.*

### B4. D3 — MR memang mahal (dugaan TEPAT), tapi mekanismenya bukan yang diduga

`n = 6`, satu tugas MR, 10 seed:

| Konfigurasi | biaya MR rata-rata | berbiaya > 0 |
|---|---|---|
| 1 gantry | **+7.02 s** | 10/10 |
| 2 gantry | **+11.23 s** | 9/10 |

(Biaya MR = makespan instance itu dikurangi makespan instance **identik** yang
tugas MR-nya diperlakukan sebagai SR. Titik tugas, pose, seed, semuanya sama.)

**Dugaan A5-D3 berbunyi "mahal karena mengunci dua lengan + pose". Arahnya
benar; separuh mekanismenya salah**, dan itu penting untuk naskah:

- Mengunci dua lengan berbiaya **paling banyak satu slot = 2.0 s**. Dan memang:
  pada 4 dari 10 seed 1-gantry biaya MR **persis 2.000 s**, yaitu tak lebih dari
  dwell-nya sendiri.
- Sisanya (sampai **+25.9 s**) datang dari **kelangkaan pose**, bukan dari
  penguncian lengan. Terukur pada peta: hanya **46.9%** node punya pose handover
  sama sekali; di antara yang punya, median **390** pose dari 2376, tapi
  **p10 = 26**. Tugas MR yang jatuh di ekor itu memaksa traverse jauh.

➜ Konsekuensi untuk penjadwalan: **handover adalah kendala POSE, bukan kendala
lengan.** Heuristik G8 harus memilih pose handover lebih dulu dan mengatur sisanya
di sekitarnya — bukan memperlakukan MR sebagai "tugas biasa yang butuh dua lengan".

### B5. Struktur makespan — jadwal ini didominasi SETUP, bukan kerja

Pangsa waktu yang benar-benar dipakai dwell, pada gantry kritis:

| n | 1 gantry | 2 gantry |
|---|---|---|
| 4 | 12.0% | 8.3% |
| 6 | 16.5% | 10.2% |
| 8 | 19.6% | 13.1% |
| 10 | 25.7% | 14.2% |

**74–92% makespan adalah gerak gantry.** Dan dengan 2 gantry, makespan hampir
tidak tumbuh terhadap `n` (33.0 → 34.5 → 35.7 → 35.9 s untuk n = 4 → 10):
tugas tambahan diserap ke perhentian yang sudah ada nyaris gratis.

Ini konsisten dengan `p1_state §2` ("geometri statis permisif; yang langka
adalah waktu") dan menajamkannya: yang langka bukan waktu lengan, melainkan
**waktu gantry**. Rel penuh 52.8 s versus dwell 2.0 s adalah rasio 26:1, dan
rasio itulah yang menentukan bentuk masalahnya.

➜ **Konsekuensi langsung untuk G8:** masalah sebenarnya adalah **tur pose gantry
yang bergantung urutan** (TSP kecil dengan cakupan), bukan penugasan lengan.
Heuristik yang benar tur pose-nya akan punya optimality gap kecil; heuristik yang
pintar soal alokasi lengan tapi buruk soal tur akan kalah jauh. **Ini dugaan, dan
G8 wajib mengukurnya, bukan mengasumsikannya.**

### B6. Papan skor §7.2 — sekarang **sebelas meleset, satu tepat**

| # | Dugaan (ditulis di muka, §A5) | Hasil |
|---|---|---|
| **10** | D1: `n = 6` tidak akan muat 120 s, perlu penjarangan pose | ❌ **MELESET** — `n = 10` muat, tanpa penjarangan. Arah sama: menduga kendala lebih mengikat. |
| **11** | D2: mutex `r = 0.20` akan mengikat pada instance nyata | ❌ **MELESET, paling telak** — `0.000 s` pada 105 pasang. Arah sama lagi. |
| — | D3: tugas MR jauh lebih mahal | ✅ **TEPAT** (+7.0 / +11.2 s) — tapi **mekanismenya keliru**: pose langka, bukan lengan terkunci. |

**Sebelas dari dua belas dugaan meleset, dan sepuluh dari sebelas ke arah yang
persis sama: menduga kendala lebih mengikat daripada kenyataannya.** Pola ini
sekarang cukup kuat untuk dipakai sebagai prior kerja: **kalau sebuah kendala
belum diukur, dugaan awal yang benar adalah "ia longgar", bukan "ia mengikat".**

Dan pengecualiannya menguatkan aturan `§7.2` yang lain: D3 satu-satunya yang
tepat, dan satu-satunya yang bisa ditelusuri ke **jalur data** — kolom
`handover  22p` yang tercetak apa adanya oleh `Instance.describe()`. Bagian yang
salah dari D3 (mekanismenya) justru bagian yang **tidak** ditelusuri ke data.

### B7. Batasan yang tetap berlaku setelah sesi ini

Semua isi §A4 masih berlaku tanpa perubahan. Yang perlu ditegaskan ulang karena
§B membuatnya lebih penting, bukan kurang:

1. 🔴 **Tabrakan struktur gantry–gantry belum dimodelkan.** Semua angka rugi di
   §B adalah **BATAS BAWAH**. Khususnya `mutex cost = 0.000 s` adalah batas
   bawah biaya koordinasi, bukan pernyataan bahwa koordinasi gratis.
2. **Kedua gantry hanya berkopling lewat alokasi tugas.** Terkonfirmasi di kode:
   `solve_exact` menyelesaikan tiap gantry independen lalu mengambil maksimum.
   Kopling waktu antar gantry akan muncul hanya setelah (1) dimodelkan. `XD`
   yang benar-benar terukur sesi ini adalah kopling **di dalam** satu gantry.
3. **`T_fold` = 0.0**, tidak pernah diukur. Ia menambah konstanta ke setiap
   traverse, jadi ia hanya akan **memperbesar** dominasi setup di §B5 — arah
   biasnya diketahui, besarnya tidak.
4. **Dua statistik zona yang berbeda, jangan dicampur** (jebakan yang sama dengan
   "42.9% vs 53.1%"): **51.8%** = pangsa agregat pasangan (node, pose) terjangkau
   yang menempati zona; **31%** = median per-node dari rasio yang sama. Angka
   berbeda dari data yang sama. Sebut mana yang dipakai.
5. Solver exact hanya exact **terhadap grid pose 33 × 72**. Grid itu sendiri
   diskretisasi (`p1_state §5.2`), dan itu keputusan terkunci, bukan galat sesi ini.

---

## C. Prompt sesi berikutnya — G8 = SCHED-2 (heuristik + optimality gap)

> Rekomendasi: **Opus, effort tinggi.** Alasannya sama dengan G7 dan diperkuat
> §B3: keluaran G8 adalah *gap*, dan gap yang salah hitung terlihat persis seperti
> gap yang benar. Tidak ada perangkat keras.

```
Sesi G8 -- SCHED-2: heuristik, baseline, dan optimality gap terhadap
ground truth G7. Langkah 3 dan 4 dari p1_state 7.1.

BACA DULU, berurutan:
1. docs/p1_g7_sched.md    -- SELURUHNYA. A1 (model, terkunci), B2 (batas
                             n<=10), B3 (mutex 0.000 s -- BACA DUA KALI),
                             B4 (MR = kendala POSE), B5 (74-92% makespan
                             adalah gerak gantry), B6 (papan skor), B7 (batasan)
2. docs/p1_state.md       -- 5 (terkunci), 7.1 (urutan wajib), 7.2 (ukur
                             jangan menduga; papan skor kini 11 meleset, 1 tepat)
3. reachability_gng/sched.py + test/verify_sched_exact.py -- API dan V0-V4

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.

=== YANG SUDAH SELESAI, JANGAN BANGUN ULANG ===
- Model penjadwalan: terkunci di p1_g7 A1. JANGAN diubah diam-diam. Kalau ia
  harus berubah, tulis pertentangannya eksplisit seperti B2/B3.
- Solver exact: sched.solve_exact, terbukti exact lewat V0-V4. INI GROUND TRUTH.
  Kalau angka heuristik tidak cocok dengan intuisi, yang salah heuristiknya.
- Generator instance: sched.gen_real (ber-seed, deterministik) dan
  gen_random_small. Pakai seed yang SAMA dengan G7 supaya bisa dibandingkan.

=== TUGAS ===
1. Heuristik. Rancang dari temuan B5, bukan dari selera: 74-92% makespan
   adalah TUR POSE GANTRY. Heuristik yang benar tur pose-nya akan menang.
2. Baseline pembanding (empat, p1_state 6): fixed assignment - greedy/nearest
   - sequential - dan MIP/DP-optimal (sudah ada = ground truth).
3. Optimality gap dilaporkan terhadap solve_exact pada n <= 10, lalu
   heuristik dijalankan JAUH di atas n = 10 di mana exact tidak bisa ikut --
   dan di sana yang dilaporkan BUKAN gap, melainkan batas bawah/atas. Jangan
   menyebut selisih terhadap heuristik lain sebagai "gap".

=== KUNCI KRITERIA SUKSES SEBELUM SATU BARIS KODE, ke docs/p1_g8_sched2.md A ===
1. AMBANG GAP yang dianggap "cukup baik", sebagai ANGKA, dengan alasannya.
2. HIMPUNAN INSTANCE UJI: berapa seed, n berapa saja, 1 atau 2 gantry, berapa
   MR. Ditetapkan SEBELUM melihat hasil heuristik mana pun.
3. APA YANG DILAPORKAN DI ATAS n = 10, di mana exact tidak tersedia.
4. Bagaimana Anda tahu heuristiknya tidak diam-diam melanggar model -- pakai
   ulang validate_schedule() dari test/verify_sched_exact.py; jadwal heuristik
   WAJIB lulus pemutaran ulang yang sama, bukan cuma menghasilkan angka.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- Mutex r=0.20 TIDAK menggerakkan makespan (B3). Heuristik yang "pintar soal
  mutex" akan mengejar nol. Jangan habiskan sesi di sana.
- MR = kendala POSE (B4). Pilih pose handover DULU.
- Kedua gantry hanya berkopling lewat alokasi (B7.2). Kalau heuristik Anda
  mencoba menyinkronkan waktu antar gantry, ia memodelkan sesuatu yang tidak
  ada di model.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor kini SEBELAS meleset, sepuluh ke arah
  yang sama. Prior kerja yang benar: kendala yang belum diukur itu LONGGAR.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
- Tabrakan struktur gantry-gantry masih belum dimodelkan -- semua angka rugi
  tetap BATAS BAWAH. Sebut di naskah.
- Akhiri dengan prompt sesi berikutnya (G9).
```
