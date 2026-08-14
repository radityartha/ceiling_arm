# P1 / G9 — SCHED-3: tabrakan struktur gantry–gantry, dan kopling WAKTU

> Sesi G9, 2026-08-14. Melanjutkan [p1_g8_sched2.md](p1_g8_sched2.md).
> Menutup lubang model terbesar yang tersisa (`p1_state §6`, `p1_g7 §A4.1`,
> `p1_g8 §B10.1`).
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.**
> §B diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**, bukan dihaluskan. §A tidak ditulis
> ulang belakangan — kesalahannya dibiarkan bisa dibaca (disiplin `p1_g7`,
> `p1_g8`).
>
> Sesi ini **sepenuhnya offline**. Tidak ada kamera, gantry, lengan,
> `move_group`. Lengan 4× masih dilepas.
>
> **Rekomendasi model: Opus 5, effort TINGGI, untuk seluruh sesi.** Alasannya
> berbeda dari G7 dan G8 dan harus dibaca: di G7 ada instance patologis yang
> menegur, di G8 ada solver exact yang menegur. **Sesi ini tidak punya keduanya
> sampai ia membangunnya sendiri.** Predikat tabrakan yang salah 3 cm tetap
> mencetak jadwal yang rapi; solver terkopel yang tidak exact tetap mencetak
> "biaya koordinasi" yang terlihat masuk akal; dan aturan waktu yang salah
> (kapan boleh menunggu, kapan tidak) **tidak punya sinyal error sama sekali**.
> Tiga dari empat fase sesi ini gagal secara senyap.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci di | Dipakai bagaimana |
|---|---|---|
| Model penjadwalan intra-gantry (sumber daya, timeline, SR/MR, mutex, `dur = 2.0 × max(a,b,z)`) | `p1_g7 §A1` | dibaca apa adanya, **nol perubahan** |
| `T_traverse = max(T_lin, T_rot)`, offset per sumbu yang bergerak | `p1_state §5.6` + `p1_g7 §A0` | dipakai; **durasinya** tidak diturunkan ulang (tapi lihat 🔒 A2.3) |
| Dwell 2.0 s kontinu | `p1_state §5.8 / §8b` | durasi tugas |
| Definisi makespan | `p1_g7 §A2-K2` | diperluas ke waktu tunggu, lihat A2.5 |
| Handover = SATU tugas dua-lengan | `p1_g7 §A2-K3` | tidak ditawar |
| Peta `cap_g{1,2}_rail160.npz` (33×72, \|P\| = 2376) | `p1_state §3` | satu-satunya oracle kelayakan |
| `sched.solve_exact` | `p1_g7 §B1` (V0–V4) | **GROUND TRUTH untuk model TANPA tabrakan.** Nol perubahan. Dipakai sebagai **batas bawah yang sah** (A3-K1 Lemma 3) |
| `sched.gen_real`, `gen_random_small` | `p1_g7 §A3` | generator, seed sama dengan G7/G8 |
| `sched_heur.pose_tour` + 3 baseline | `p1_g8 §B2` | dipanggil, **tidak diubah**; dibungkus, bukan disunting |
| `test/verify_sched_exact.validate_schedule` | `p1_g8 §A3-K4` | dipakai ulang apa adanya untuk aturan **intra**-gantry |
| Geometri URDF gantry | `moving_table.urdf.xacro` | dibaca sebagai konstanta, **tidak diarang** (A2.1) |

`sched.py`, `sched_heur.py`, `test/verify_sched_exact.py`, `test/eval_sched_heur.py`
**dibaca dan diimpor saja, nol perubahan** — `git diff` keempatnya wajib kosong di
akhir sesi. Alasannya sama dengan G8: mengubah ground truth di sesi yang mengukur
terhadap ground truth adalah cara paling rapi untuk menipu diri sendiri.

### A1. Yang dibangun sesi ini, dan yang TIDAK

Dibangun:
1. **Predikat tabrakan** struktur gantry–gantry (A2.2), diturunkan dari URDF.
2. **Model waktu terkopel** (A2.3–A2.5): lintasan traverse, kendala kontinu,
   aturan menunggu.
3. **Ground truth baru** (`solve_coupled`) + buktinya (A3-K1, W0–W4).
4. **Pengukuran** (A3-K3/K5): berapa yang dibeli koordinasi, dan apakah B3-G7
   (mutex 0.000 s) dan B6-G8 (urutan tur 0.00%) selamat pada model baru.

**TIDAK dibangun sesi ini, dan tidak boleh menyelinap masuk:**
`T_fold ≠ 0`, kapabilitas dinamis, tabrakan **lengan–lengan** antar gantry
(sudah diukur terpisah, `p1_g2 §10`, dan **tidak mengikat**: 0.00% pasangan
target hilang pada clearance ≤ 0.15 m, 0.62% pada 0.20 m), tabrakan lengan
dengan lingkungan statis, dan heuristik baru apa pun **sebelum** ground truth
lulus W0–W4 (`p1_state §7.1`, urutan wajib).

### A2. MODEL TABRAKAN — diturunkan sekarang, dikunci sekarang

#### A2.1 Geometri, dibaca dari URDF, bukan dari prosa

Semua angka di bawah **disalin** dari
[`workcell_description/urdf/moving_table.urdf.xacro`](../ros2_ws/src/workcell_description/urdf/moving_table.urdf.xacro)
dan dari rantai kinematik yang **sudah diverifikasi terhadap pinocchio sampai
1e-16** (`irm_sweep.base_pose`, `p1_g2 §1`). Tidak satu pun diarang. Ini penting
karena `p1_state §7.2` melarang mengarang konstanta, dan seluruh sesi ini
bergantung pada tiga angka.

| Benda | Geometri URDF | Baris |
|---|---|---|
| `rotation_link` (batang) | box `0.08 × 0.80 × 0.015` m, berpusat di sumbu rotasi, sumbu panjang ikut `rot` | `moving_table.urdf.xacro:54` |
| `mount_plate_{left,right}` | silinder `r = 0.055`, `L = 0.05`, di `±0.40` m sepanjang batang | `:86–90` |
| `platform_link` (tidak berputar) | box `0.35 × 0.34 × 0.065`, berpusat di `(lin, y_g)` | `:23` |
| Sumbu rotasi gantry `g` | vertikal di `(x = lin, y = y_g)`, `y_1 = +0.36`, `y_2 = −0.36` | `irm_sweep.py:90`, `p1_g2 §1` |

Arah batang di dunia pada pose `(lin, rot)`: `u(rot) = (cos rot, sin rot)` —
diturunkan dari `p = (lin − y_p·sin(π/2+rot), y_g + y_p·cos(π/2+rot), 1.9525)`
dengan `y_p = ∓0.4`, yaitu bentuk tertutup yang **sudah** dipakai membangun peta
kapabilitas. Jadi predikat ini memakai transformasi yang **sama persis** dengan
oraclenya, bukan transformasi kedua yang mirip.

🔒 **Dua penyederhanaan yang sah dan dibuktikan sekarang, bukan diasumsikan:**

1. **Ujinya 2-D (bidang XY), dan itu EXACT, bukan proksi.** Batang berada di
   `z ∈ [−0.0475, −0.0325]` relatif pusat platform, pelat di `[−0.0975,
   −0.0475]`; **kedua gantry berada di pita `z` yang identik** (`z` platform
   sama, `p1_state §3`). Dua benda pada pita `z` yang sama beririsan di 3-D
   ⟺ proyeksi XY-nya beririsan. Tidak ada informasi yang hilang.
2. **`platform_link` bisa dibuang dari uji, dan itu dibuktikan.** Platform
   membentang `y_g ± 0.17`: gantry 1 di `y ∈ [0.19, 0.53]`, gantry 2 di
   `[−0.53, −0.19]` — **terpisah 0.38 m dan tidak berputar**, jadi platform–platform
   mustahil bertemu. Dan jangkauan `y` terjauh batang+pelat gantry 1 ke bawah
   adalah `0.36 − 0.455 = −0.095`, masih `0.095 m` di atas tepi atas platform
   gantry 2 (`−0.19`). Jadi platform tidak pernah ikut menentukan. **Hanya
   batang + dua pelat yang diuji.**

#### A2.2 🔒 PREDIKAT `BLOCK` — inilah "definisi tabrakan sebagai predikat yang bisa diuji"

Jejak (footprint) gantry `g` pada pose `p = (lin, rot)`, di bidang XY:

```
c        = (lin, y_g)                       y_1 = +0.36,  y_2 = −0.36
u        = (cos rot, sin rot)
Body_g(p) = OBB( pusat c, sumbu u dan u⊥, setengah-ukuran (0.40, 0.04) )
          ∪ Disc( c + 0.40·u, r = 0.055 )
          ∪ Disc( c − 0.40·u, r = 0.055 )

BLOCK(p1, p2 ; c_clear)  ⟺  dist_XY( Body_1(p1), Body_2(p2) )  ≤  c_clear
```

`dist_XY` = minimum atas 3×3 pasangan primitif (OBB–OBB, OBB–disc, disc–disc),
semuanya bentuk tertutup 2-D. **Ini jejak URDF apa adanya**, bukan kapsul
pembungkus dan bukan lingkaran `r = 0.4` yang selama ini dipakai di prosa: prosa
lama (`p1_state §6`, `p1_g7 §A4.1`) memakai **cakram sapuan** `r = 0.4`, yang
adalah gabungan atas SEMUA rotasi. Cakram itu benar untuk pertanyaan "bisakah
mereka bertemu sama sekali"; ia **salah** sebagai predikat per-pose, dan
memakainya akan melarang sebagian besar ruang pose secara palsu.

🔒 **`c_clear = 0.0 m` adalah DEFAULT TERKUNCI.** Margin keselamatan tidak pernah
diukur, dan `p1_state §7.2` melarang mengarangnya — perlakuannya sama persis
dengan `T_fold = 0.0` (`p1_g7 §A4.3`). `c_clear` tetap jadi parameter, dan
sensitivitasnya **dilaporkan sebagai sapuan** `c_clear ∈ {0.00, 0.05, 0.10}`
(K5.6). Angka yang dikutip naskah adalah yang `c_clear = 0.0`.

**Dua syarat perlu, murni aljabar, diturunkan sekarang** (dipakai sebagai
prapenyaring tervektorisasi **dan** sebagai uji akal sehat terhadap
implementasinya):

Setengah-rentang jejak pada sumbu `y` adalah `h_y(rot) = 0.40·|sin rot| + 0.055`
(suku `0.055` menang atas `0.04·|cos rot|` selalu), dan pada sumbu `x` adalah
`h_x(rot) = 0.40·|cos rot| + 0.055`. Maka `BLOCK` **mensyaratkan**:

```
(N1)   |sin rot1| + |sin rot2|  ≥  (0.61 − c_clear) / 0.40  =  1.525   (c_clear = 0)
(N2)   |lin1 − lin2|            ≤  0.40·(|cos rot1| + |cos rot2|) + 0.11 + c_clear
```

Konsekuensi (N1) yang harus dicatat karena ia membentuk seluruh sesi: **tabrakan
menuntut KEDUA gantry berputar lebih dari 31.7° dari sejajar-rel** — karena
`|sin rot2| ≤ 1` memaksa `|sin rot1| ≥ 0.525`, dan sebaliknya; pada kasus
simetris syaratnya lebih keras lagi, `|sin rot| ≥ 0.7625` = 49.7°.
Kalau salah satu gantry berada dekat `rot = 0` atau
`rot = ±180°`, tabrakan **mustahil secara aljabar**, berapa pun `lin`-nya.

🔒 **LEMMA 1 — "parkir aman" selalu ada, jadi instance TIDAK PERNAH deadlock.**
Pada `rot = 0`, `h_y = 0.055`, jadi jejak gantry 2 memenuhi `y ≤ −0.305`,
sementara jejak gantry 1 pada pose **apa pun** memenuhi `y ≥ 0.36 − 0.455 =
−0.095`. Selisihnya `0.21 m > 0`. Maka **sembarang pose gantry 1 kompatibel
dengan sembarang pose `rot = 0` gantry 2**, dan sebaliknya. ∎
Akibatnya: (a) jadwal layak selalu ada (serialkan: satu gantry parkir di
`rot = 0` sementara yang lain bekerja); (b) `p0` bawaan `gen_real`
(`lin = 0`, `rot = 0` untuk kedua gantry) **bebas tabrakan menurut konstruksi**,
jadi keadaan awal selalu sah; (c) margin 0.21 m berarti kesimpulan sesi ini
tidak berubah untuk `c_clear` sampai 0.21 m.

🔒 **LEMMA 2 — predikatnya TIDAK vacuous, dengan contoh tertutup.**
Ambil `rot1 = −90°`, `rot2 = +90°`. Jejak gantry 1 membentang `y ∈ [−0.095,
0.815]` pada `x = lin1 ± 0.055`; gantry 2 membentang `y ∈ [−0.815, 0.095]` pada
`x = lin2 ± 0.055`. Keduanya beririsan pada `y ∈ [−0.095, 0.095]`, jadi
`BLOCK ⟺ |lin1 − lin2| ≤ 0.11`. Pada grid rel 0.05 m itu berarti `Δlin ∈
{0, ±0.05, ±0.10}` **terlarang**. ∎
Ini adalah P4-nya sesi ini: kalau uji ini keluar "tidak pernah tabrakan", modelnya
tidak terpasang, dan seluruh sesi tidak punya isi. Ia masuk W3 sebagai fixture.

#### A2.3 🔒 LINTASAN TRAVERSE — komitmen model BARU, disebut sebagai baru

`p1_state §5.6` mengunci **durasi** traverse. Ia **tidak** mengunci **lintasan**,
dan sebelum sesi ini lintasan tidak pernah dibutuhkan. Sekarang ia dibutuhkan,
jadi ia dikunci di sini sebagai penambahan model yang sadar:

```
traverse p = (l, r) -> p' = (l', r'), mulai pada t0, durasi T = max(T_lin, T_rot)

lin(t) = l  + clamp( (t − t0 − 0.29) · v_lin , 0, |l'−l| ) · sign(l'−l)   bila l' ≠ l
       = l                                                                 bila l' = l
rot(t) = r  + clamp( (t − t0 − 0.26) · v_rot , 0, |Δr|  ) · arah-pendek     bila r' ≠ r
       = r                                                                 bila r' = r

v_lin = 31.4160 mm/s,  v_rot = 10.0 deg/s,  arah-pendek = jalur < 180 deg
```

Yaitu: **offset 0.29 / 0.26 s ditafsirkan sebagai waktu mati perintah**, sumbu
diam selama itu lalu bergerak dengan kecepatan tetap sampai tiba, lalu menahan.
Kedua sumbu berjalan **konkuren** — persis itulah asal `max(T_lin, T_rot)`.

⚠️ **Ini tafsir, dan tafsir alternatifnya disebut supaya tidak hilang:** offset
bisa juga akselerasi (sumbu mulai segera, ramp). Yang dipilih adalah waktu mati
karena ia **konsisten dengan `T(p, p) = 0`** yang sudah terkunci (`p1_g7 §A0`:
sumbu yang tidak diperintah tidak menyumbang) dan karena ia tidak memerlukan
konstanta akselerasi yang tidak pernah diukur. Arah biasnya diketahui: waktu mati
membuat benda **lebih lama diam di pose awal** dibanding ramp, jadi ia
**menggeser** jendela konflik, tidak menghapusnya. Kalau ini keliru, ia keliru
pada urutan detik, bukan pada urutan struktur.

#### A2.4 🔒 KENDALA WAKTU KONTINU, dan bagaimana ia diuji tanpa mendiskretkan waktu

```
Untuk SETIAP t ≥ 0:   ¬ BLOCK( p1(t), p2(t) ; c_clear )
```

Satu aturan, dan ia mencakup ketiga kasus sekaligus (diam–diam, traverse–diam,
traverse–traverse). Tidak ada aturan tambahan, dan **tidak boleh** ada.

Menguji ini secara tuntas menuntut menyelesaikan pertidaksamaan trigonometri
sepanjang lintasan. Yang dipakai adalah **sertifikat conservative advancement**,
bukan sampling yang berharap:

```
R_max      = 0.40 + 0.055 = 0.455 m       titik terjauh jejak dari sumbu rotasi
v_titik    ≤ v_lin + ω·R_max = 0.031416 + 0.174533·0.455 = 0.110828 m/s
v_relatif  ≤ 2 × 0.110828 = 0.221656 m/s
```

Uji pada waktu diskret `t_k` dengan langkah `Δt ≤ ε / 0.221656`, memakai clearance
`c_clear + ε`. Karena jarak antar-benda adalah fungsi **1-Lipschitz** terhadap
perpindahan relatif, tidak ada tabrakan yang bisa lolos di antara dua sampel.

🔒 **`ε = 0.005 m` DIKUNCI.** Ia sepuluh kali di bawah langkah grid rel (0.05 m)
dan delapan kali di bawah busur satu langkah rotasi pada `R_max`
(`5° × 0.455 = 0.0397 m`), jadi ia **tidak bisa membalik keputusan pada level
grid** kecuali tepat di batas. Konsekuensinya ditulis apa adanya: ujinya
**konservatif sebesar ε** — ia boleh melaporkan "terhalang" ketika jarak
sebenarnya ada di `(c_clear, c_clear + ε]`, tapi **tidak pernah** sebaliknya.
Arah biasnya diketahui: makespan yang dilaporkan adalah **batas atas** terhadap
model kontinu yang tepat, dan selisihnya dibatasi ε.

Implementasi boleh memakai langkah adaptif (`Δt = jarak_sekarang / v_relatif`)
karena itu memberi jawaban yang **sama persis** dengan lebih sedikit sampel; yang
dikunci adalah **jaminannya**, bukan cara mencapainya. W1 mengujinya.

#### A2.5 🔒 APA YANG DIOPTIMALKAN — makespan terkopel, dan aturan MENUNGGU

`p1_g7 §A2-K2` diperluas seminimal mungkin:

```
t = 0    : kedua gantry di p_g^0, keempat lengan menganggur. Nol biaya start-up.
           (p_g^0 bebas tabrakan menurut Lemma 1.)
menunggu : sebuah gantry boleh MENAHAN di pose sekarang selama durasi > 0
           sebelum memulai traverse berikutnya atau pekerjaan berikutnya.
           Menunggu GRATIS dan tidak dibatasi.
           Menunggu DI TENGAH traverse TIDAK BOLEH -- sekali traverse mulai,
           kedua sumbu berjalan seperti A2.3 sampai tiba. (Sumbu stepper
           dikomando ke target; berhenti di tengah bukan gerak yang dimodelkan
           di mana pun, dan mengizinkannya berarti mengarang profil gerak baru.)
makespan : waktu jendela dwell tugas TERAKHIR selesai, maks atas kedua gantry.
```

Semua yang dikecualikan `p1_g7 §A2-K2` tetap dikecualikan (tidak ada pulang ke
home, tidak ada lipat, tidak ada biaya persepsi/perencanaan).

🔒 **LEMMA 3 — `makespan_terkopel ≥ makespan_takterkopel`, selalu.**
Model takterkopel (G7) adalah **relaksasi**: hapus kendala A2.4 dan hapus
kemungkinan menunggu. Setiap jadwal terkopel yang layak adalah jadwal
takterkopel yang layak dengan biaya yang sama atau lebih besar (menunggu hanya
menambah). Maka optimum takterkopel adalah **batas bawah yang sah** untuk
optimum terkopel. ∎
Ini adalah tulang punggung sesi: ia memberi (a) definisi biaya koordinasi
`Δ = terkopel − takterkopel ≥ 0`, (b) heuristik admissible untuk B&B (A3-K1),
dan (c) **invarian yang bisa diuji pada setiap instance** (W4-Q4). Kalau `Δ < 0`
pernah muncul sekali saja, ada bug, dan sesi ini melapor GAGAL.

🔒 **LEMMA 4 — kalau jadwal optimum takterkopel kebetulan bebas tabrakan, ia
optimum terkopel.** Langsung dari Lemma 3: ia layak di model terkopel dan
nilainya sudah menyentuh batas bawah. ∎
Prior papan skor (`p1_g8 §B9`: kendala yang belum diukur itu LONGGAR) meramalkan
ini akan sering terjadi, jadi ia dipakai sebagai jalur cepat — **tapi ia tidak
boleh dipakai sebagai pengganti solver.** Instance yang lolos lewat Lemma 4
dihitung dan **dilaporkan sebagai angka** (K5.3), karena kalau angkanya 100%
maka sesi ini tidak pernah benar-benar menjalankan solver terkopelnya, dan itu
persis pelajaran gerbang-yang-tidak-pernah-menyala di `p1_g8 §B1`.

### A3. Kriteria yang DIKUNCI

#### K1 — BAGAIMANA GROUND TRUTH BARU DIBUKTIKAN EXACT

Bentuk solvernya dikunci sekarang karena ia menentukan apa yang harus dibuktikan.

**`solve_coupled(inst)` = branch & bound atas keadaan bersama**, dengan
`sched.solve_gantry` (G7, sudah exact) sebagai **heuristik admissible**:

```
keadaan : (R1, p1, t1, R2, p2, t2) -- R_g = tugas tersisa, p_g = pose,
          t_g = waktu gantry g bebas di p_g, plus lintasan yang SUDAH dikomit
ekspansi: SELALU kembangkan gantry dengan t_g lebih kecil (yang lain sudah
          terkomit sampai t-nya, jadi lintasannya diketahui untuk seluruh t
          yang perlu diuji)
aksi    : pilih (pose tujuan p', himpunan tugas U ⊆ R_g yang layak di p'),
          lalu pilih waktu mulai
LB      : max_g ( t_g + h_g(R_g, p_g) )     h_g dari sched.solve_gantry
UB awal : serialisasi Lemma 1 (satu gantry parkir di rot = 0), plus jadwal
          takterkopel yang sudah diperbaiki bila ia layak (Lemma 4)
```

🔒 **LEMMA 5 — himpunan waktu mulai yang perlu dicoba itu BERHINGGA.**
Ambil jadwal terkopel optimum. Geser setiap aksi semaju mungkin (dalam urutan
waktu) tanpa melanggar A2.4 dan tanpa mendahului kesiapan gantrynya sendiri;
menggeser maju tidak pernah menaikkan makespan. Hasilnya setiap aksi mulai
**tepat pada** salah satu dari: (i) `t_g`, waktu gantry itu siap, atau (ii) ujung
kanan sebuah selang terhalang, yaitu waktu di mana kendala A2.4 baru saja lepas.
Selang terhalang berhingga banyaknya (lintasan gantry lawan tersusun dari
berhingga banyak fase). Maka calon waktu mulai berhingga. ∎
🔴 **Lemma 5 adalah baris paling mencurigakan di §A** — perannya persis seperti
bentuk tertutup `max(a,b,z)` di `p1_g7 §A1`, yaitu argumen yang terlihat rapi dan
tidak punya sinyal error. W2 **wajib** menyerangnya dengan enumerator yang tidak
memakainya.

**Lima tingkat verifikasi, SEMUA WAJIB LULUS** (setara V0–V4 G7):

| | Apa yang diadu | Independen dalam hal apa |
|---|---|---|
| **W0** | `BLOCK` bentuk tertutup vs **uji titik-rapat** pada jejak (rasterisasi kedua benda, jarak minimum pasangan titik brute force) | tidak memakai satu pun rumus jarak primitif; hanya definisi himpunan |
| **W0b** | pusat pelat dari `BLOCK` vs `irm_sweep.base_pose` (rantai URDF yang **sudah** dicek ke pinocchio 1e-16) | mengikat predikat ke oracle peta, bukan ke transformasi kedua |
| **W1** | sertifikat A2.4 vs sampling **10× lebih rapat** dan vs 10⁵ waktu acak per traverse | menyerang ε, bukan mengasumsikannya |
| **W2** | `solve_coupled` vs **enumerator jadwal bersama brute force** (partisi terurut × pose × penugasan lengan × urutan konflik × waktu mulai, tanpa B&B, tanpa DP, tanpa Lemma 5) | algoritma dan kode berbeda; **inilah yang menguji Lemma 5** |
| **W2b** | `solve_coupled` dengan `BLOCK ≡ False` vs `sched.solve_exact` | harus **identik sampai 1e-9** pada instance nyata `\|P\| = 2376`; menguji seluruh mesin baru di luar bagian tabrakannya, pada skala penuh |
| **W3** | `solve_coupled` vs **jawaban tertutup instance patologis** (K1b), ditulis sebelum kode ada | tidak ada solver di sisi kebenarannya |
| **W4** | jadwal yang DILAPORKAN diputar ulang dari aturan, termasuk A2.4 | `validate_schedule` G7/G8 apa adanya untuk aturan intra-gantry + pemutar ulang tabrakan yang terpisah |

W2 dijalankan pada `n ≤ 3`, `\|P\| ≤ 4`, 2 gantry, ≥ 24 instance, toleransi
**1e-9, tidak dilonggarkan**. W2b dijalankan pada ≥ 20 instance `gen_real`
`n ∈ {4, 6}`.

🔴 **Kalau W0–W4 tidak semuanya lulus, sesi ini melaporkan KEGAGALAN dan
berhenti.** Dilarang menyebut `solve_coupled` ground truth sebelum itu, dan
dilarang menyentuh heuristik apa pun sebelum itu (`p1_state §7.1`).

#### K1b — INSTANCE PATOLOGIS, jawabannya ditetapkan SEKARANG

Semua sintetis, jawabannya dihitung tangan dari A2, bukan dari solver.

| # | Nama | Konstruksi | Makespan yang DIHARAPKAN | Yang diisolasi |
|---|---|---|---|---|
| **Q1** | tabrakan mustahil | 2 tugas, satu per gantry, keduanya hanya layak di pose `rot = 0` | **sama persis** dengan `solve_exact` | (N1): dekat sejajar-rel, kopling tidak ada |
| **Q2** | pose saling mengunci | tiap gantry 1 tugas, masing-masing hanya layak di `(lin = 0.8, rot = ∓90°)` — pasangan itu `BLOCK` menurut Lemma 2 | `T(p0,p) + 2.0` **+** `2.0` (serialisasi paksa: yang kedua harus menunggu yang pertama selesai dwell; keduanya sudah di pose, jadi tidak ada traverse tambahan) | pose–pose statis |
| **Q3** | traverse dipotong | gantry 2 tanpa tugas, `p0_2 = (0.8, +90°)`; gantry 1 harus melintasi `rot` dari −45° ke −135° pada `lin = 0.8` | > nilai takterkopel; nilainya dihitung tangan dari A2.3 + A2.4 di berkas uji | traverse memotong pose gantry lain |
| **Q4** | menunggu itu berguna | seperti Q2 tapi gantry 1 punya tugas kedua di pose `rot = 0`; optimum **menukar urutan** supaya gantry 2 lewat lebih dulu | dihitung tangan, dan **harus lebih kecil** dari varian yang melarang menunggu | aturan menunggu A2.5 |
| **Q5** | Lemma 1 | kedua gantry dipaksa ke `rot = 0` dan `rot = 180°` bergantian | `Δ = 0.000` persis | parkir aman |

🔴 **Q2 adalah P4-nya sesi ini.** Kalau Q2 keluar sama dengan nilai takterkopel,
predikatnya tidak terpasang dan seluruh angka §B kosong.
🔴 **Q4 adalah satu-satunya uji terhadap aturan menunggu.** Kalau solver hanya
pernah "berangkat secepat mungkin", ia akan lulus Q1–Q3 dan Q5 dan tetap salah.

#### K2 — HIMPUNAN INSTANCE UJI, ditetapkan sebelum melihat hasil apa pun

Seed **sama dengan G7/G8** supaya bisa dibandingkan langsung.

| Set | Generator | n | seed | gantry | n_mr | jumlah | Apa perannya |
|---|---|---|---|---|---|---|---|
| **S1 alami** | `gen_real` | 4, 6 | 0–9 | (1,2) | 0, 1 | **40** | angka yang boleh dikutip naskah |
| **S2 sempit** | `gen_real_crowded` | 4, 6 | 0–9 | (1,2) | 0, 1 | **40** | **probe adversarial**, dilaporkan TERPISAH |
| **S3 nol** | `gen_real` | 4 | 0–9 | (1,) | 0, 1 | **20** | kontrol: `Δ` wajib `0.000` pada 20/20 |
| **S4 patologis** | sintetis | — | — | — | — | **5** | K1b |

🔒 **`gen_real_crowded` dikunci sekarang** (ia adalah rancangan probe, bukan
konstanta fisik, dan disebut begitu): sama dengan `gen_real`, kecuali kolam node
dibatasi ke `|x_node − 0.80| ≤ 0.30 m`, dan `p0` **kedua** gantry adalah pose
grid terdekat ke `(lin = 0.80, rot = 0)`. Alasannya: (N1)+(N2) baru saja
membuktikan tabrakan menuntut `lin` berdekatan **dan** kedua rotasi besar; S1
tidak dijamin pernah menghasilkan itu, dan sesi yang hanya melaporkan "tidak
pernah mengikat" tanpa pernah menguji rezim di mana ia **bisa** mengikat
mengulang persis kelemahan gerbang-tidak-pernah-menyala (`p1_g8 §B1`).

🔴 **S1 dan S2 TIDAK PERNAH dirata-ratakan bersama.** S2 dipilih supaya mengikat;
mencampurnya ke S1 akan mengarang angka yang tidak mewakili apa pun.

**Anggaran.** `≤ 120 s dinding per instance` untuk `solve_coupled` (angka yang
sama dengan `p1_g7 §A2-K1`, sengaja). Kalau `n = 6` tidak muat, **yang
dilaporkan adalah `n` terbesar yang muat, sebagai angka terukur** — anggarannya
tidak dilonggarkan diam-diam, dan penjarangan pose tidak dipakai untuk
menyelamatkan angka. Pembanding takterkopel memakai `sched.solve_exact` apa
adanya (terukur 1.15 s pada `n = 4`, 4.06 s pada `n = 6`, 2 gantry, `p1_g7 §B2`).

#### K3 — "KOORDINASI ITU MAHAL" SEBAGAI ANGKA, ditulis sebelum diukur

```
Δ(instance)     = makespan_terkopel − makespan_takterkopel        [detik, ≥ 0 (Lemma 3)]
Δ%(instance)    = Δ / makespan_takterkopel × 100 %

MAHAL                 :=  mean Δ% ≥ 5.0 %   pada S1
TERUKUR TAPI MURAH    :=  0 < mean Δ% < 5.0 %   pada S1
GRATIS                :=  max Δ = 0.000 s   pada SELURUH S1
```

Alasan ambang 5.0%, tiga-tiganya angka, bukan selera:

1. **5% dari makespan tipikal 33–47 s (`p1_g7 §B5`) adalah 1.65–2.35 s ≈ satu
   slot dwell (2.0 s)** — kuantum waktu terkecil yang dimiliki model untuk
   *kerja*. Ini ambang yang **sama persis** dengan `p1_g8 §A3-K1`, dipakai ulang
   dengan sengaja supaya biaya koordinasi dan gap heuristik bisa dibaca pada
   skala yang sama.
2. **Ia juga satu langkah penghindaran di grid.** Manuver mengelak termurah yang
   ada: satu langkah rotasi 5° = `0.26 + 0.5 = 0.76 s`; satu langkah rel 50 mm =
   `0.29 + 1.59 = 1.88 s`. Biaya di bawah ~0.76 s **tidak bisa** berasal dari
   mengelak sama sekali — hanya dari menunggu. 5% ≈ 2.2 s ≈ satu langkah rel
   penuh.
3. **Pembanding yang harus dikalahkan sudah ada dan sudah terukur:** biaya MR
   `+7.02 s` pada 46.9 s = **15.0%** (`p1_g7 §B4`), dan mutex **0.000 s**
   (`p1_g7 §B3`). Koordinasi di bawah 5% berada jauh lebih dekat ke mutex
   daripada ke MR, dan naskah harus menuliskannya begitu.

Dilaporkan juga, tanpa syarat: **fraksi instance dengan `Δ > 1e-9`**, mean dan
max `Δ` dalam **detik** (bukan hanya persen), dan p90.

🔴 Ambang ini **dilarang** digeser setelah melihat hasil, dan **dilarang**
membuang instance dari rata-rata (`p1_g8 §A3-K1`, dipatuhi di sana, dipatuhi di
sini).

#### K4 — GERBANG KEABSAHAN, dan pelajaran `p1_g8 §B4` dipatuhi di muka

Setiap jadwal dari setiap penjadwal wajib lolos **dua** gerbang:

1. **Aturan intra-gantry**: `validate_schedule()` dari `test/verify_sched_exact.py`,
   fungsi yang sama tanpa modifikasi.
2. **Aturan tabrakan**: pemutar ulang **terpisah** yang membangun ulang kedua
   lintasan dari A2.3 dan mengecek A2.4 dengan sertifikat A2.4 pada `ε/10`
   (sepuluh kali lebih ketat dari yang dipakai solver — gerbang yang selonggar
   solvernya bukan gerbang).

⚠️ **`validate_schedule()` EKSPONENSIAL dalam tugas per perhentian** (`p1_g8 §B4`:
dua proses membakar 25 menit CPU sebelum sebabnya jelas). Konsekuensinya
**dikunci sekarang, bukan ditemukan lagi**: K2 dibatasi `n ≤ 6`, di mana
perhentian terbesar yang mungkin adalah 6 tugas → `2⁶ × 6⁶ = 3.0e6` penempatan,
**di bawah** anggaran `5e6` yang G8 pakai. Jadi gerbang penuh berlaku **100%,
tanpa pengecualian**, di seluruh sesi ini. Kalau K2 harus melewati `n = 6`,
batasannya diberlakukan ulang secara eksplisit dan cakupannya dilaporkan sebagai
angka, seperti `p1_g8 §B4`.

🔴 **Jadwal yang gagal → angkanya TIDAK MASUK laporan sama sekali.** Jumlah
kegagalan dilaporkan sebagai angka.

#### K5 — YANG DILAPORKAN, TERLEPAS DARI HASILNYA

1. `Δ` dan `Δ%` per konfigurasi `(set, n, n_mr)`: mean, median, p90, max, dan
   **fraksi yang mengikat**. S1 dan S2 terpisah.
2. Waktu dinding `solve_coupled` vs `solve_exact`, dan `n` terbesar yang muat
   anggaran 120 s — sebagai angka terukur (K2).
3. **Berapa instance selesai lewat Lemma 4** (jadwal takterkopel kebetulan bebas
   tabrakan) versus berapa yang benar-benar memaksa B&B. Kalau 100% lewat Lemma 4,
   itu **dikatakan**, karena artinya solver terkopel belum pernah teruji pada
   instance nyata.
4. Statistik predikat pada grid: fraksi dari `2376 × 2376` pasangan pose yang
   `BLOCK`, dan fraksi pasangan `(rot1, rot2)` yang memenuhi (N1). Ini **jalur
   data**, dan `p1_state §7.2` mencatat bahwa satu-satunya dugaan tepat selalu
   yang ditelusuri ke jalur data.
5. **(a)** Apakah mutex `r = 0.20` masih `0.000 s` pada model terkopel
   (`p1_g7 §B3` diuji ulang, bukan diasumsikan selamat).
   **(b)** Apakah urutan tur masih menyumbang `0.00%` (`p1_g8 §B6`) — diukur
   dengan `pose-tour` yang dibungkus sadar-tabrakan, varian `nn-only` /
   `cover-order` yang sama.
   **(c)** Berapa makespan yang dibeli koordinasi = K3.
6. Sapuan `c_clear ∈ {0.00, 0.05, 0.10}` pada S1 dan S2 — arah dan besar
   sensitivitasnya.
7. Jumlah jadwal yang gagal gerbang K4, per penjadwal.

### A4. Yang TIDAK dimodelkan — setelah sesi ini

🔴 **Yang harus paling keras dikatakan: memodelkan tabrakan STRUKTUR tidak
mengubah angka rugi menjadi nilai. Ia tetap BATAS BAWAH.** Yang belum ada:

1. **Tabrakan lengan–lengan antar gantry selama gerak.** `p1_g2 §10` mengukurnya
   dan menemukan **tidak mengikat** (0.00% pasangan target hilang sampai
   clearance 0.15 m) — tapi itu diukur pada konfigurasi **menjangkau**, dengan
   proksi polyline yang **meremehkan** volume sapuan, dan **tidak pernah** pada
   lengan yang sedang dibawa melintas. Ujung lengan terentang berjarak 1.4 m dari
   sumbu rotasi (`p1_g2 §12`) — **tiga kali** `R_max = 0.455` yang dimodelkan
   sesi ini. Jadi kalau ada kendala yang lebih besar dari yang diukur di sini,
   ia ada di sana.
2. `T_fold = 0.0`, tidak pernah diukur. Ia menambah konstanta ke **setiap**
   traverse. Di model terkopel ia juga **memperpanjang** setiap jendela konflik,
   jadi arahnya sekarang ganda: memperbesar setup **dan** memperbesar biaya
   koordinasi.
3. Gerak lengan di dalam satu pose = 0.
4. Exact hanya exact **terhadap grid 33 × 72** dan **terhadap `c_clear = 0.0`**.
5. Uji A2.4 konservatif sebesar `ε = 0.005 m` (arah diketahui, besar dibatasi).
6. Lintasan traverse A2.3 adalah **tafsir** offset sebagai waktu mati, bukan
   pengukuran.

### A5. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor: **15 meleset, 2 tepat** (`p1_g8 §B9`). Prior kerja: **kendala yang
belum diukur itu LONGGAR** — tapi `p1_g8 §B9` catatan 1: prior itu berlaku untuk
**DUNIA**, bukan untuk kode yang kita tulis sendiri. Dugaan sesi ini ditulis
mengikuti pembedaan itu secara sengaja, supaya priornya sendiri ikut diuji:

| # | Dugaan | Ini dugaan tentang | Kenapa |
|---|---|---|---|
| **D9** | Pada **S1**, koordinasi **MURAH**: `Δ = 0.000 s` pada ≥ 80% instance, mean `Δ%` < 5% | dunia | (N1) menuntut **kedua** gantry > 31.7° dari sejajar-rel **dan** `lin` berdekatan. Prior + aljabar sepakat. |
| **D10** | Pada **S2** (sempit, adversarial), ia **MENGIKAT**: mean `Δ%` ≥ 5% | dunia | kalau S2 pun tidak mengikat, predikatnya efektif vacuous di grid ini, dan itu temuan yang lebih besar |
| **D11** | Urutan tur (`p1_g8 §B6` = 0.00%) tetap **0.00% pada S1**, tapi **> 0 pada S2** | dunia | urutan baru berbiaya kalau ada kopling waktu; S1 diduga tidak punya |
| **D12** | Mutex `r = 0.20` tetap **0.000 s** pada model terkopel | dunia | `p1_g7 §B3` Q2: 92% himpunan tugas punya pose lepas-mutex; menambah kendala pose tidak mengurangi itu |
| **D13** | `solve_coupled` **muat 120 s pada `n = 6`, 2 gantry, `\|P\| = 2376`** | **kode kita sendiri** | ⚠️ prior "longgar" TIDAK berlaku di sini; `p1_g8 §B9` catatan 1 memperingatkan kita justru terlalu optimistis tentang kode sendiri. Dugaan ini yang paling mungkin meleset. |

Kelimanya **diukur**, tidak diperdebatkan. Hasilnya masuk §B apa adanya,
termasuk kalau semuanya meleset lagi ke arah yang sama.

### A6. Berkas

| Berkas | Isi |
|---|---|
| `reachability_gng/sched_coll.py` | geometri + `BLOCK` + lintasan traverse + `solve_coupled` + `gen_real_crowded` + pembungkus heuristik sadar-tabrakan + CLI |
| `test/verify_sched_coll.py` | W0, W0b, W1, W2, W2b, W3, W4 |
| `test/eval_sched_coll.py` | pengukuran K3/K5, sapuan `c_clear`, ablasi (a)/(b) |
| `docs/p1_g9_sched3.md` | dokumen ini |

`sched.py`, `sched_heur.py`, `test/verify_sched_exact.py`, `test/eval_sched_heur.py`
**tidak disentuh** — `git diff` keempatnya wajib kosong. Heuristik sadar-tabrakan
dibuat dengan **membungkus** `sched_heur.pose_tour`, bukan menyuntingnya.

### A7. Urutan kerja, dan apa yang terjadi kalau waktu habis

Dikunci karena `p1_state §7.1` menuntut urutan, dan karena sesi ini punya empat
fase yang bisa saling makan waktu:

```
1. geometri + BLOCK + W0/W0b            <- tanpa ini tidak ada yang berarti
2. lintasan + A2.4 + W1
3. solve_coupled + W2/W2b/W3/W4         <- GERBANG: tidak lanjut kalau gagal
4. K3 pada S1/S2/S3 + K5.1-K5.4         <- angka utama sesi ini
5. K5.5(a) mutex, K5.6 sapuan c_clear
6. K5.5(b) ablasi urutan tur            <- perlu pembungkus heuristik
```

🔴 **Apa pun yang tidak tercapai dilaporkan sebagai TIDAK DIUKUR, sebagai
kalimat eksplisit di §B.** Dilarang menuliskan "diperkirakan tidak berubah" untuk
sesuatu yang tidak dijalankan — itu persis kesalahan yang `p1_state §7.2` ada
untuk mencegah, dan `p1_g8 §B6` baru saja menunjukkan bahwa dugaan yang paling
"jelas" (§B5-G7 tentang urutan tur) adalah yang meleset paling telak.

---

## B. Hasil terukur

> §A dikunci 2026-08-14 sebelum `sched_coll.py` ada. Semua angka di bawah keluar
> sesudahnya. Setiap tempat di mana §B bertentangan dengan §A ditandai 🔺 dan
> ditulis eksplisit. **§A TIDAK ditulis ulang** — ia tetap seperti saat dikunci,
> termasuk kalimat-kalimatnya yang ternyata salah (disiplin `p1_g7`, `p1_g8`).
>
> ⏳ **Sesi ini masih berjalan.** Yang sudah ada: A7 langkah 1. Sisanya belum
> diukur dan **tidak boleh** dibaca sebagai "diperkirakan tidak berubah".

### B0. Cara menjalankan ulang

```bash
cd /home/user1/Documents/ceiling_arm/ros2_ws/src/reachability_gng
python3 test/verify_sched_coll.py            # W0, W0b, W0c   (~5 menit)
```

`sched.py`, `sched_heur.py`, `verify_sched_exact.py`, `eval_sched_heur.py`
**tidak disentuh** — `git diff` keempatnya kosong.

### B1. Predikat `BLOCK` — W0/W0b LULUS, dan §A salah satu angka

| | Apa yang diadu | Cakupan | Hasil |
|---|---|---|---|
| **W0** | `pair_distance` vs **oracle sampling batas** (benda sebagai himpunan titik; nol rumus jarak, nol SAT) | 1 184 kasus: 384 terstruktur, 400 acak, 400 **dekat-sentuh** | **0 ketidakcocokan** > 3.0e-3 m |
| **W0** | kesahihan prapenyaring (N1)/(N2) | 3 552 pasangan (kasus, `c_clear`) | **0 kali tidak sah** |
| **W0b** | pusat pelat vs `irm_sweep.base_pose` (rantai URDF yang sudah dicek ke pinocchio 1e-16) | 2 000 pose | maks \|selisih\| **2.2e-16 m** |
| **W0c** | Lemma 1 (parkir aman) | 2376 × 33 pasangan pose | clearance minimum **0.2100 m** — persis yang A2.2 hitung |
| **W0c** | Lemma 2 (tidak vacuous) | grid rel penuh pada `rot = ∓90°` | **LULUS setelah dikoreksi** (lihat 🔺 di bawah) |

Maks \|selisih\| W0 per kelas: terstruktur **1.4e-05 m**, acak **4.3e-05 m**,
dekat-sentuh **3.4e-04 m**. Kelas dekat-sentuh sengaja dibuat menempel di ambang
dan memang yang paling jelek — tapi ia masih **empat kali** di bawah toleransi
sampling `2H = 3.0e-3 m`, jadi selisihnya adalah galat oracle-nya, bukan galat
predikatnya.

⚠️ Dicatat karena ia **melemahkan** bukti: **252 dari 3 552** pasangan (kasus,
`c_clear`) berada dalam toleransi sampling dari ambangnya, jadi untuk pasangan
itu W0 mengadu **jaraknya** tapi tidak bisa memutuskan **boolean**-nya ke arah
mana pun. Itu konsekuensi langsung dari memilih 400 kasus yang sengaja
ditempatkan di dekat kontak; ia tidak bisa diperbaiki dengan sampling lebih
rapat tanpa biaya kuadratik, dan ia disebut apa adanya.

**W0b adalah mata rantai yang paling penting di sini**, dan bukan karena
angkanya kecil: ia mengikat predikat ke transformasi yang **sama persis** yang
dipakai membangun `cap_g{1,2}_rail160.npz`. Predikat yang benar terhadap URDF
tapi tidak terhadap oracle peta akan melarang pose yang salah, dan **tidak ada
satu pun uji lain di sesi ini yang bisa melihatnya**.

#### 🔺 Pertentangan 1 dengan §A2.2 — ambang Lemma 2 salah, `0.11` seharusnya `0.095`

§A2.2 Lemma 2 mengunci: pada `rot1 = −90°, rot2 = +90°`, `BLOCK ⟺ |Δlin| ≤ 0.11`,
jadi `Δlin ∈ {0, ±0.05, ±0.10}` terlarang di grid. **Terukur: `Δlin = 0.10`
TIDAK terhalang.** Yang terhalang hanya `{0, 0.05}`.

Kesalahannya bukan aritmetika, melainkan **memakai syarat PERLU sebagai syarat
CUKUP**. `0.11 = h_x + h_x = 0.055 + 0.055` adalah (N2), yang hanya bilang
rentang-`x` kedua benda bertumpang tindih; ia tidak bilang apa pun tentang
apakah bagian yang bertumpang-tindih di `x` juga bertemu di `y`. Pada
`rot = ∓90°` kedua pelat duduk di `y = ∓0.04`, jadi pasangan fitur yang mengikat
adalah **pelat lawan SISI batang**, bukan pelat lawan pelat:

```
pelat vs sisi batang :  |Δlin| ≤ BAR_HALF_WID + PLATE_R = 0.040 + 0.055 = 0.095   <- mengikat
batang vs batang     :  |Δlin| ≤ 0.080
pelat vs pelat       :  sqrt(Δlin² + 0.08²) ≤ 0.110  ->  |Δlin| ≤ 0.0756
```

Ambang yang benar **0.095 m**, dan predikatnya cocok dengannya persis di grid.
**Kesimpulan substantif Lemma 2 — predikatnya tidak vacuous — TIDAK berubah**,
dan itulah yang sekarang ditegaskan W0c. Yang gugur hanya angkanya.

Dua hal yang perlu dicatat dari kegagalan ini, karena keduanya berlaku untuk
sisa sesi:

1. **(N1) dan (N2) adalah prapenyaring, bukan jawaban.** Di mana pun sisa
   dokumen ini memakainya, ia dipakai untuk membuang pasangan yang **pasti
   tidak** bertabrakan, tidak pernah untuk menyatakan pasangan bertabrakan.
   W0 memeriksa kesahihannya secara terpisah (0 dari 3 552).
2. Ini kesalahan pertama sesi ini dan ia **tertangkap oleh uji, bukan oleh
   pembacaan ulang** — persis alasan A2.2 menulis Lemma 2 sebagai fixture W0c
   dan bukan sebagai prosa. Kalau Lemma 2 dibiarkan sebagai kalimat di §A, angka
   `0.11` akan masuk naskah tanpa ada yang menegur.

Sebagai catatan yang bukan pertentangan: (N1) menuntut `|sin rot| ≥ 0.525`
(31.7°) pada **kedua** gantry, dan rotasi grid terkecil yang memenuhinya adalah
**35.0°** — jadi syarat itu memang memotong grid, bukan lolos otomatis.

➜ **A7 langkah 1 SELESAI.** `BLOCK` boleh dipakai oleh langkah 2 dan seterusnya.

### B2. Lintasan traverse + sertifikat waktu kontinu — W1 LULUS, empat-empatnya

| | Apa yang diadu | Cakupan | Hasil |
|---|---|---|---|
| **W1a** | durasi lintasan A2.3 vs `sched.traverse_time` (**terkunci** `p1_state §5.6`) | 3 000 leg acak | maks \|selisih\| **3.6e-15 s**; galat titik-ujung **1.1e-13** |
| **W1b** | sertifikat A2.4 vs **sapuan rapat 10×** langkah sertifikatnya sendiri + 5 000 waktu acak per pasangan | 250 pasangan lintasan | **0 terlewat** |
| **W1c** | kasus hitung-tangan: gantry 1 memutar melewati −90° di depan gantry 2 yang parkir | 1 | blok pertama **2.9568 s**, batas A2.3 **4.7600 s** |
| **W1d** | Lemma 1 **dalam waktu**: gantry 2 parkir di `rot = 0` | 200 lintasan gantry-1 acak | **0 blok** |

W1a adalah yang paling penting untuk kesinambungan naskah: ia membuktikan
lintasan yang baru dikunci A2.3 **tidak menggeser durasi** yang sudah dipakai
G7 dan G8. Kalau ia bergeser, seluruh perbandingan `Δ = terkopel − takterkopel`
akan mengukur dua model yang berbeda dan bukan biaya koordinasi.

**Konservatisme ε terukur, dan ia kecil:** dari 250 pasangan, sertifikatnya
menyebut "terhalang" padahal sapuan rapat tidak **hanya 1 kali**, dan pada kasus
itu clearance sebenarnya **0.00425 m** — di bawah `ε = 0.005` seperti yang
dijanjikan A2.4, bukan di luarnya. Jadi arah bias A2.4 (**batas atas**, tidak
pernah meremehkan) terkonfirmasi sebagai angka, bukan sebagai argumen.

⚠️ **Berbeda dari gerbang G8, gerbang ini menyala terus.** **197 dari 250**
pasangan lintasan benar-benar terhalang. `p1_g8 §B1` harus mencatat gerbangnya
tidak pernah menolak apa pun; di sini kebalikannya, dan itu membuat bukti W1b
jauh lebih kuat daripada bukti K4 di G8.

➜ **A7 langkah 2 SELESAI.**

### B3. Solver terkopel — dua pertentangan struktural, dan D13 MELESET

#### 🔺 Pertentangan 2 dengan §A2.5 — GERAK MENGHINDAR tidak ada di §A, dan ia wajib ada

§A memodelkan setiap gerak gantry sebagai gerak **menuju pose kerja**. Itu benar
di model takterkopel (`p1_g7`: mengunjungi pose tanpa kerja tidak pernah
menguntungkan, akibat ketaksamaan segitiga). **Di model terkopel itu salah.**
Gantry yang sudah menyelesaikan semua tugasnya tetap menempati ruang, dan bisa
**menghalangi** gantry lain selamanya. Tanpa gerak menghindar, instance yang
sebenarnya layak akan dilaporkan **tidak layak**.

Ini bukan detail implementasi — ia mengubah ruang aksi model. Konsekuensinya
ruang aksi bukan lagi `keep_g`, melainkan seluruh `|P| = 2376`. Yang dipakai:
`C_g = keep_g ∪ safe_g`, dan hanya pose aman **termurah** yang dicoba.

#### 🔺 Pertentangan 3 dengan §A3-K1 Lemma 5 — himpunan waktu mulai dibatasi

§A2.5 mengizinkan menunggu selama durasi real apa pun, dan Lemma 5 berargumen
himpunan yang berguna berhingga: `{kesiapan sendiri} ∪ {ujung selang terhalang}`.
Menghitung "instan sebuah selang terhalang berakhir" secara exact berarti
menyelesaikan pertidaksamaan trigonometri sepanjang lintasan A2.3. Yang dipakai:

```
{kesiapan sendiri}  ∪  {batas-batas leg gantry LAWAN}
```

Berhenti dijamin karena setelah leg terakhir lawan, lawan **statis**, sehingga
aksi menjadi geseran waktu murni dan kelayakannya berhenti bergantung pada `s`.

➜ Maka `solve_coupled` adalah **exact TERHADAP kedua himpunan kandidat itu**,
bukan exact begitu saja. Bahasa ini **sudah disediakan** `p1_g7 §A2-K1` untuk
penjarangan pose; di sini ia dipakai pada sumbu yang berbeda. Yang **tidak**
boleh dilakukan adalah menyebutnya ground truth tanpa kualifikasi.

🔴 **Dan inilah yang paling penting untuk G10: W2 — enumerator jadwal bersama
brute force — TIDAK DIBANGUN sesi ini.** §A3-K1 menugaskan W2 secara spesifik
sebagai satu-satunya uji yang **menyerang Lemma 5**, dan §A sendiri menandai
Lemma 5 sebagai "baris paling mencurigakan di §A". W2b/W3/W4 menguji mesin di
sekelilingnya; **tidak satu pun menguji Lemma 5**. Jadi menurut kriteria §A3-K1
sendiri, **ground truth terkopel BELUM tegak**, dan `p1_state §7.1` melarang
menyentuh heuristik sampai ia tegak. Ini dilaporkan sebagai kegagalan memenuhi
kriteria, bukan dihaluskan.

#### 🔺 D13 MELESET — dan tepat ke arah yang `p1_g8 §B9` catatan 1 peringatkan

Dugaan: `solve_coupled` muat 120 s pada `n = 6`, 2 gantry, `|P| = 2376`.
Terukur, pada instance yang tabrakannya benar-benar mengikat
(`n = 6, seed = 2, mr = 1`, `gen_real`):

| | |
|---|---|
| takterkopel (`solve_exact`) | **50.735 s** |
| B&B setelah **100 s** dinding | **12 node** dikembangkan |
| yang dikembalikan | UB serialisasi **83.963 s**, `proved = False` |

Jadi yang dilaporkan untuk instance itu adalah **kurungan `[50.735, 83.963]`**,
bukan `Δ`. Dugaan D13 **meleset**, dan arahnya adalah arah yang `p1_g8 §B9`
catatan 1 sebut eksplisit: prior "kendala yang belum diukur itu longgar"
berlaku untuk **dunia**, **bukan** untuk kode yang kita tulis sendiri, dan D13
adalah satu-satunya dari lima dugaan sesi ini yang tentang kode kita sendiri.
Ia satu-satunya yang meleset ke arah pesimis.

**Sebabnya diukur, bukan ditakar, dan ia adalah `p1_g8 §B4` yang terulang di
tempat baru:** yang lambat bukan penjadwalnya, melainkan **pemeriksanya**. Satu
node dengan incumbent longgar membangkitkan sampai `2368 pose × 2⁶ subset ≈
1.5e5` aksi, dan tiap aksi memanggil `_first_start`, yang menjalankan satu
sertifikat A2.4. Satu ekspansi node berjalan **menit**, dan tenggat waktu di
puncak loop tidak pernah kebagian giliran.

Tiga hal dikerjakan setelah itu, semuanya dicatat karena keduanya mengubah
angka:

1. **Batas bawah `_split_lb` diperbaiki.** Versi pertama memakai `h_g[R]` dengan
   `R` yang masih **dibagi bersama** dua gantry. Karena `h_g` monoton, `h_g[R]`
   **melebih-lebihkan** apa yang harus dikerjakan `g`, sehingga heuristiknya
   **tidak admissible** dan B&B-nya boleh mengembalikan jawaban suboptimal.
   Yang benar: `min` atas seluruh pembagian tugas, yaitu optimum takterkopel
   dari keadaan itu, yang Lemma 3 sahkan. Bug ini ada di **dua** tempat dan
   perbaikan pertama hanya menutup satu — gejalanya: `evade = n_node = 1846`,
   yaitu setiap aksi ditolak dan setiap node jatuh ke gerak menghindar.
2. **Batas bawah penerus dihitung TERVEKTORISASI dan SEBELUM pencarian waktu
   mulai**, supaya sertifikat hanya berjalan pada aksi yang sudah lolos
   pemangkasan. `3^|R| ≤ 729` operasi vektor menggantikan `1.5e5` sertifikat.
3. **Anggaran aksi per node** (`ACTION_BUDGET = 400`) + tenggat waktu **di dalam**
   loop aksi. Kalau salah satunya tersentuh, hasilnya **diturunkan jadi
   kurungan**, tidak pernah dilaporkan sebagai `Δ`.


### B4. 🔴 GERBANG W0–W4 **TIDAK LULUS**. Ground truth terkopel BELUM tegak.

§A3-K1 menulis, dengan tinta merah: *"Kalau W0–W4 tidak semuanya lulus, sesi ini
melaporkan KEGAGALAN dan berhenti."* Hasilnya:

| | Hasil | Catatan |
|---|---|---|
| **W0** | ✅ LULUS | §B1 |
| **W0b** | ✅ LULUS | §B1 |
| **W0c** | ✅ LULUS (setelah koreksi ambang) | §B1, Pertentangan 1 |
| **W1** | ✅ LULUS (a/b/c/d) | §B2 |
| **W2** | ⛔ **TIDAK DIBANGUN** | §B3, Pertentangan 3 |
| **W2b** | ❌ **GAGAL** | di bawah |
| **W3** | ✅ LULUS (Q1, Q2, Q4, Q5) | di bawah |
| **W4** | ❌ **GAGAL** — tapi karena §A, bukan karena solver | di bawah |

➜ **`solve_coupled` TIDAK BOLEH disebut ground truth**, dan `p1_state §7.1`
melarang menyentuh heuristik sampai ia tegak. Itulah keadaan akhir sesi ini,
ditulis sebagai vonis, bukan sebagai catatan kaki.

**W2b GAGAL, dan kegagalannya nyata.** 12 instance nyata, `|P| = 2376`,
tabrakan **dimatikan** (`c_clear = −1`), Lemma 4 dipaksa mati, incumbent
di-seed **2.0 s di atas** optimum sejati supaya pencariannya harus bekerja:

- 11 dari 12 mengembalikan **nilai yang benar** tapi `proved = False` —
  anggaran aksi/waktu tersentuh sebelum heap habis. Jawaban benar, bukti tidak.
- **1 dari 12 mengembalikan nilai yang SALAH**: `n = 4, mr = 1, seed = 1` →
  **46.7772** versus exact **44.7772**, meleset **tepat 2.0 s = satu slot dwell**.
  Dan 46.7772 adalah **persis nilai seed**-nya: pencarian tidak menemukan
  apa pun yang lebih baik dari incumbent yang diberikan padanya.

Jadi yang gagal bukan model tabrakannya (tabrakan sedang dimatikan) melainkan
**kelengkapan pencariannya**. Dengan slack hanya 2 s pun ia tidak selalu
menemukan optimum yang `sched.solve_exact` temukan. Ini persis kegunaan W2b:
ia mengadu mesin baru dengan oracle yang sudah terbukti, pada skala penuh, dan
ia menangkap sesuatu.

**W3 LULUS, dan Q2 informatif.** Q1 dan Q5 (tabrakan mustahil secara aljabar)
memberi `Δ = 0` persis. **Q2 — P4-nya sesi ini — hidup:**

| | |
|---|---|
| takterkopel | **11.2600 s** |
| terkopel | **22.5200 s** |
| lantai hitung-tangan (dwell tidak boleh tumpang tindih) | 13.2600 s |
| gerak menghindar dipakai | **1** |

Yaitu: pada pose yang saling mengunci, tabrakan **menggandakan** makespan, dan
mekanisme gerak menghindar (Pertentangan 2) benar-benar dipakai dan benar-benar
diperlukan. Predikatnya bukan hiasan. Q4 (invarian Lemma 3, `Δ ≥ 0`) lulus
0 pelanggaran pada 5 instance nyata.

#### 🔺 Pertentangan 4 dengan §A3-K4 — `validate_schedule()` TIDAK BISA memvalidasi jadwal terkopel

W4 melaporkan 3 dari 12 jadwal "melanggar". Ditelusuri ke jadwalnya, dan
**bukan solver yang salah**:

```
gantry 1: stop di t = 0.000 (dur 2.0)  ->  MENUNGGU sampai t = 46.445  ->  leg 7.76 s
          stop berikutnya di t = 54.205
validate_schedule memutar ulang:  t += traverse; t += dur   ->  9.76
```

`validate_schedule()` menjumlahkan traverse dan dwell **tanpa tempat untuk waktu
menunggu** — sewajarnya, karena di model G7/G8 menunggu tidak pernah berguna dan
karenanya tidak ada. **A2.5 sesi ini menambahkan menunggu ke model**, jadi
gerbang yang §A3-K4 kunci ("fungsi yang sama, tanpa modifikasi") **secara
struktural tidak bisa** memvalidasi keluaran model baru. §A3-K4 tidak bisa
dijalankan seperti tertulis, dan itu **kesalahan §A**, bukan temuan tentang
solver.

Yang **tetap** berlaku dan tetap dijalankan: separuh tabrakan dari gerbang itu,
diputar ulang pada `ε/10` (sepuluh kali lebih ketat dari sertifikat solvernya
sendiri) → **0 dari 12 jadwal bertabrakan**. Jadi aturan A2.4 terpenuhi pada
setiap jadwal yang dikembalikan; yang tidak terperiksa adalah aturan
intra-gantry pada jadwal yang mengandung tunggu.

➜ **Untuk G10: `validate_schedule()` perlu penerus yang sadar-tunggu**, dan itu
harus ditulis sebagai gerbang baru, bukan sebagai modifikasi diam-diam terhadap
berkas G7 yang sampai sekarang `git diff`-nya kosong.

### B5. S3 — kontrol nol LULUS 20/20

`gen_real`, `n = 4`, seed 0–9, **satu** gantry, mr 0 dan 1. Tidak ada gantry
kedua, jadi tidak ada yang bisa ditabrak, jadi `Δ` **wajib** 0.

```
20 dari 20 instance:  Δ = +0.000 s   (route 'single')
```

Ini kontrol yang membosankan dan itu memang gunanya: kalau satu saja bukan nol,
`solve_coupled` menambahkan biaya yang tidak berasal dari tabrakan, dan setiap
angka `Δ` di §B akan tercemar. Ia nol.

### B6. S1 — 🔴 vonis K3 **TIDAK DAPAT DITENTUKAN**, dan **D9 MELESET**

40 instance, `gen_real`, `n` = 4/6, seed 0–9, 2 gantry, mr 0/1. Anggaran 120 s.

| | jumlah | |
|---|---|---|
| `Δ` **EXACT** (Lemma 4 menjawab, atau `UB = LB`) | **27 / 40** | semuanya `Δ = +0.0000 s` |
| **KURUNGAN** (optimum takterkopel **melanggar** A2.4) | **13 / 40** | `Δ > 0`, besarnya **tidak diketahui** |
| route: `lemma4` 25, `bnb` 15 | | wall rata-rata 41.9 s, maks 120.1 s |

Dua dari 15 instance ber-`bnb` tetap terhitung EXACT karena `UB` menyentuh `LB`:
menurut Lemma 3 batas bawah itu sah, jadi menyentuhnya **adalah** bukti
optimalitas, terlepas dari bagaimana ia ditemukan dan terlepas dari `proved`.
Bendera `proved` solver lebih ketat dari yang diperlukan di sini.

🔴 **Vonis A3-K3 pada seluruh S1: TIDAK DAPAT DITENTUKAN.**
Skrip mencetak `GRATIS` atas subset yang bisa dibuktikan, dan **angka itu
dilarang dikutip**: subset itu **didefinisikan** oleh sifat "optimum
takterkopelnya bebas tabrakan", yaitu didefinisikan oleh `Δ = 0`. Mean-nya
**wajib** nol; ia mengukur definisinya sendiri, bukan dunia. Ini persis jebakan
yang §B6 versi awal antisipasi, dan ia terjadi.

Yang **boleh** dikatakan, dan hanya ini:

> Pada **27 dari 40** instance S1, koordinasi berbiaya **tepat nol**, dan itu
> exact. Pada **13 dari 40**, jadwal optimum takterkopel **tidak bisa
> dieksekusi** tanpa tabrakan, jadi koordinasi berbiaya **> 0** — dan **berapa,
> sesi ini tidak tahu**.

Kurungan pada ke-13 itu (`UB` dari serialisasi Lemma 1, sangat longgar):

| instance | LB | UB | rasio |
|---|---|---|---|
| n4 s4 mr0 | 37.713 | 73.017 | 1.936 |
| n6 s9 mr0 | 32.938 | 59.876 | 1.818 |
| n4 s8 mr0 / n6 s8 mr0 | 35.713 | 63.757 | 1.785 |
| n6 s6 mr1 | 31.346 | 52.326 | 1.669 |
| n6 s2 mr1 | 50.735 | 83.963 | 1.655 |
| n4 s9 mr1 / n6 s9 mr1 | 50.036 | 78.974 | 1.578 |
| n4 s7 mr0 / n6 s7 mr0 | 39.304 | 58.693 | 1.493 |
| n4 s7 mr1 | 39.594 | 58.983 | 1.490 |
| n6 s1 mr1 | 44.777 | 60.574 | 1.353 |
| n4 s2 mr1 | 48.445 | 56.205 | 1.160 |

⚠️ Rasio itu adalah rasio terhadap **UB serialisasi**, bukan terhadap optimum.
`p1_g8 §B10.2` sudah memaksa pelajaran yang sama untuk rasio kurungannya: rasio
1.94 **tidak** berarti koordinasi berbiaya 94%.

#### 🔺 **D9 MELESET — dan ke arah yang BERLAWANAN dengan prior papan skor**

Dugaan D9: pada S1, `Δ = 0` pada **≥ 80%** instance. Terukur: **67.5%**
(27/40). Meleset.

Dan arah melesetnya adalah yang penting: **kendalanya lebih MENGIKAT daripada
dugaan**, bukan kurang. Papan skor `p1_state §7.2` berdiri di **16 meleset**,
dan hampir semuanya ke arah "menduga kendala lebih mengikat daripada
kenyataannya" — sehingga prior kerjanya jadi *"kendala yang belum diukur itu
LONGGAR"*. **Tabrakan struktur gantry–gantry adalah kendala pertama dalam
proyek ini yang meleset ke arah sebaliknya.**

Ini harus ditulis dengan hati-hati, karena ia menggoda untuk digeneralisasi
terlalu cepat. Yang terukur: **sepertiga instance 2-gantry alami punya optimum
takterkopel yang tidak bisa dieksekusi.** Yang **tidak** terukur: berapa
mahalnya memperbaikinya. Bisa jadi 13 instance itu semuanya diperbaiki dengan
menunggu 2 detik. Prior "longgar" belum gugur — yang gugur adalah versi
kuatnya, *"kendalanya tidak akan mengikat sama sekali"*.

➜ Untuk naskah: **`p1_g7 §B7.1` dan `p1_g8 §B10.1` — "semua angka rugi adalah
batas bawah karena tabrakan belum dimodelkan" — sekarang punya dukungan
kuantitatif pertamanya.** Bukan lagi kehati-hatian teoretis: pada 32.5%
instance, jadwal yang G7/G8 sebut optimum **benar-benar** akan menabrakkan
perangkat keras.


#### Kurungan yang dipersempit — probe lanjutan, 2026-08-14

⚠️ **Dijalankan SESUDAH §B6 versi pertama ditulis, dan tetap bukan `Δ`.** Yang
berubah hanya **konstruktor batas atas**, bukan solver dan bukan klaim
exactness: sebuah jadwal yang layak adalah batas atas yang sah bagaimanapun ia
ditemukan. `LB` tidak berubah sama sekali (optimum takterkopel, exact dari G7).

Tiga hal diperbaiki di `_repair_ub`, dan dua yang pertama **tidak menghasilkan
apa-apa** — dicatat karena mereka menunjukkan diagnosis awal saya salah:

1. gantry yang **sudah selesai** boleh menyingkir → **nol perubahan** pada 13/13;
2. kalau satu gantry terhalang, **gantry lain jalan duluan** → juga nol;
3. **himpunan waktu mulai diperlebar** (40 probe seragam, hanya di konstruktor
   UB) → **inilah yang bekerja**.

Sebabnya terbaca dari datanya, bukan ditebak. Pada `n4_s4_mr0` optimum
takterkopel menyuruh **kedua** gantry menyapu `rot ≈ −90°` pada `lin` yang sama
**pada saat yang sama** (t = 8.475 s, jarak 0.0034 m). Tabrakannya **di
tengah traverse**, bukan di tujuan — tujuannya justru aman (`rot = −180°`,
(N1) melarang tabrakan di sana). Yang memperbaikinya adalah **menunda satu
gantry beberapa detik**, dan waktu itu **bukan batas leg apa pun**. Jadi
kurungan yang longgar itu adalah **Pertentangan 3 yang mengikat**, bukan B&B
yang lambat.

| instance | LB | UB lama | UB baru | rasio lama | rasio baru | `Δ` |
|---|---|---|---|---|---|---|
| `n4_s2_mr1` | 48.445 | 56.205 | **48.445** | 1.160 | **1.000** | **0.000 (EXACT)** |
| `n4_s4_mr0` | 37.713 | 73.017 | **45.656** | 1.936 | **1.211** | ≤ 7.943 |
| `n4_s7_mr0` | 39.304 | 58.693 | **47.060** | 1.493 | **1.197** | ≤ 7.755 |
| `n4_s7_mr1` | 39.594 | 58.983 | **47.350** | 1.490 | **1.196** | ≤ 7.755 |
| `n4_s8_mr0` | 35.713 | 63.757 | **37.630** | 1.785 | **1.054** | ≤ 1.918 |
| `n4_s9_mr1` | 50.036 | 78.974 | **50.036** | 1.578 | **1.000** | **0.000 (EXACT)** |
| `n6_s1_mr1` | 44.777 | 60.574 | **53.325** | 1.353 | **1.191** | ≤ 8.548 |
| `n6_s2_mr1` | 50.735 | 83.963 | **58.542** | 1.655 | **1.154** | ≤ 7.807 |
| `n6_s6_mr1` | 31.346 | 52.326 | **32.683** | 1.669 | **1.043** | ≤ 1.337 |
| `n6_s7_mr0` | 39.304 | 58.693 | **47.060** | 1.493 | **1.197** | ≤ 7.755 |
| `n6_s8_mr0` | 35.713 | 63.757 | **37.630** | 1.785 | **1.054** | ≤ 1.918 |
| `n6_s9_mr0` | 32.938 | 59.876 | **38.896** | 1.818 | **1.181** | ≤ 5.958 |
| `n6_s9_mr1` | 50.036 | 78.974 | **57.994** | 1.578 | **1.159** | ≤ 7.958 |

```
rasio kurungan   lama  mean 1.599   max 1.936
                 baru  mean 1.126   max 1.211
BATAS ATAS Delta       mean 5.127 s   median 7.755 s   max 8.548 s
terbukti Delta = 0     2 dari 13
```

➜ **Jawaban atas pertanyaan yang memicu probe ini: koordinasi berbiaya ORDE
SATU SAMPAI EMPAT SLOT DWELL, bukan puluhan detik.** Batas atasnya maks
8.55 s pada makespan 31–51 s, dan pada **2 dari 13** instance
`UB` menyentuh `LB` sehingga `Δ = 0` **terbukti exact** — tabrakannya diserap
habis hanya dengan menunda satu gantry, tanpa memperpanjang makespan sama
sekali.

🔴 **Yang TETAP tidak berubah:** ini semua **batas atas**. `Δ` sebenarnya ada di
`[0, angka itu]`, dan vonis A3-K3 atas seluruh S1 **tetap TIDAK DAPAT
DITENTUKAN** sampai gerbang §A3-K1 lulus. Dan arah tafsirnya sekarang jelas:
kalau `Δ` sejati mendekati batas atas ini, koordinasi berbiaya ~4–17% — di
sekitar ambang 5% A3-K3. Kalau ia mendekati nol, tabrakan mengulang pola mutex
(sering mengikat, tidak berbiaya). **G10 memutuskan yang mana**, dan itu
menaikkan nilai G10, bukan menurunkannya.

### B7. Langkah 5 dan 6 — **TIDAK DIUKUR**

§A7 mengunci aturannya: *"Apa pun yang tidak tercapai dilaporkan sebagai TIDAK
DIUKUR, sebagai kalimat eksplisit di §B. Dilarang menuliskan 'diperkirakan tidak
berubah' untuk sesuatu yang tidak dijalankan."* Maka:

| K5 | Isi | Status |
|---|---|---|
| K5.5(a) | apakah mutex `r = 0.20` masih 0.000 s pada model terkopel | **TIDAK DIUKUR** |
| K5.5(b) | apakah urutan tur masih menyumbang 0.00% (D11) | **TIDAK DIUKUR** |
| K5.6 | sapuan `c_clear ∈ {0.00, 0.05, 0.10}` | **TIDAK DIUKUR** |
| K5.4 | statistik `BLOCK` atas 2376 × 2376 pasangan pose | **TIDAK DIUKUR** |
| S2 | probe adversarial (`gen_real_crowded`) | **TIDAK DIJALANKAN** (generatornya ada dan sudah diuji hidup) |

Sebabnya satu dan disebut apa adanya: **gerbang §A3-K1 tidak lulus**, dan
`p1_state §7.1` melarang membangun di atas ground truth yang belum tegak.
Mengukur (b) menuntut heuristik sadar-tabrakan, yaitu persis yang dilarang.
Mengukur (a) dan K5.6 menuntut `solve_coupled` yang bisa dipercaya di luar
jalur Lemma 4, dan W2b menunjukkan ia belum bisa.

⚠️ **Yang secara khusus TIDAK boleh disimpulkan dari sesi ini:** bahwa mutex
masih gratis pada model terkopel, dan bahwa urutan tur masih menyumbang nol.
Keduanya adalah dugaan `p1_g8` yang sesi ini **berniat** menguji dan **tidak**
menguji. `p1_g8 §B10.1` sudah memperingatkan bahwa "tahap 3 menyumbang nol"
berlaku **pada model yang tidak punya kopling waktu** — peringatan itu masih
berdiri, tanpa perubahan, dan sekarang punya alasan tambahan: model dengan
kopling waktu sekarang **ada**, dan ia belum pernah dijalankan terhadap
pertanyaan itu.

### B8. Papan skor §7.2 — satu dinilai, empat tergantung

| # | Dugaan (§A5, ditulis di muka) | Hasil |
|---|---|---|
| **D9** | S1: koordinasi MURAH, `Δ = 0` pada ≥ 80% instance | ❌ **MELESET** — 67.5% (27/40), dan melesetnya ke arah **LEBIH MENGIKAT**, berlawanan dengan prior papan skor. §B6 |
| **D10** | S2 (sempit) mengikat, mean `Δ%` ≥ 5% | ⛔ **TIDAK DIUKUR** |
| **D11** | urutan tur 0.00% di S1, > 0 di S2 | ⛔ **TIDAK DIUKUR** |
| **D12** | mutex tetap 0.000 s | ⛔ **TIDAK DIUKUR** |
| **D13** | `solve_coupled` muat 120 s pada `n = 6` | ❌ **MELESET** — §B3 |

**Dua dinilai, dua-duanya meleset. Papan skor jadi 17 meleset, 2 tepat.**
D10–D12 **tidak dinilai**, dan karena §A5 menuntut kelimanya diukur, itu sendiri
adalah kegagalan memenuhi §A yang dicatat di sini dan bukan di catatan kaki.

**D9 dan D13 meleset ke dua arah yang BERBEDA, dan itu justru yang informatif:**

| | tentang | arah meleset |
|---|---|---|
| **D9** | **dunia** (geometri tabrakan) | kendalanya **lebih mengikat** dari dugaan |
| **D13** | **kode kita sendiri** (solver) | kodenya **lebih lambat/lebih sulit** dari dugaan |

D13 mengikuti aturan `p1_g8 §B9` catatan 1 dengan patuh. **D9 melanggar prior
utama papan skor** — dan ia dugaan pertama dalam 19 yang melakukannya. Satu
titik data tidak membatalkan pola 16-dari-18, tapi ia menandai batasnya: prior
"longgar" dibangun dari kendala **kelayakan statis** (interference, zona,
co-feasibility), dan tabrakan gantry–gantry adalah kendala **eksklusi ruang
bersama**, kelas yang berbeda. Yang jujur dikatakan: *prior itu diturunkan dari
satu kelas kendala, dan kelas keduanya baru saja memberi contoh tandingan
pertamanya.*

Yang membuat D13 layak dicatat bukan rasionya, melainkan **arahnya**.
`p1_g8 §B9` catatan 1 menulis: prior "kendala yang belum diukur itu LONGGAR"
berlaku untuk **DUNIA**, bukan untuk **kode yang kita tulis sendiri**. D13
adalah satu-satunya dari lima dugaan sesi ini yang tentang kode kita sendiri,
dan ia satu-satunya yang meleset — ke arah **pesimis-terlalu-optimis** persis
seperti yang catatan itu ramalkan. Catatan itu sekarang **dua dari dua**
(D4-max di G8, D13 di sini) dan layak dinaikkan dari catatan kaki jadi aturan:

> **Dugaan tentang dunia: tebak LONGGAR. Dugaan tentang kode sendiri: tebak
> lebih LAMBAT, lebih RUMIT, dan lebih SALAH dari yang terasa.**

Dan satu pengamatan yang konsisten dengan `p1_state §7.2`: dua kesalahan nyata
sesi ini (ambang Lemma 2, dan `h_g[R]` yang tidak admissible) **dua-duanya**
ditemukan oleh **uji yang dijalankan**, bukan oleh pembacaan ulang. Yang ketiga
(`validate_schedule` tidak bisa memodelkan tunggu) ditemukan dengan **membaca
jalur data jadwalnya** — sekali lagi jalur data, sekali lagi sesuai pola.

### B9. Batasan setelah sesi ini

Seluruh `p1_g7 §A4`/`§B7`, `p1_g8 §B10`, dan `p1_g9 §A4` **masih berlaku**.
Yang ditambahkan:

1. 🔴 **`solve_coupled` BUKAN ground truth.** W2 tidak dibangun, W2b gagal.
   Nilai yang boleh dikutip hanya yang datang lewat Lemma 4 (§B6).
2. 🔴 **Angka rugi TETAP BATAS BAWAH walau tabrakan struktur sudah dimodelkan.**
   Tabrakan **lengan–lengan** antar gantry selama gerak masih di luar model
   (§A4.1), dan ujung lengan terentang 1.4 m dari sumbu rotasi — **tiga kali**
   `R_MAX = 0.455` yang dimodelkan di sini.
3. **§A3-K4 tidak bisa dijalankan seperti terkunci** pada model yang punya
   tunggu (§B4, Pertentangan 4). Gerbang intra-gantry untuk jadwal terkopel
   **belum ada**.
4. Predikatnya konservatif sebesar `ε = 0.005 m` (terukur mengikat 1 kali dari
   250 di W1b, clearance sejati 0.00425 m).
5. `c_clear = 0.0`, tidak pernah diukur, dan sapuannya **tidak dijalankan**.
6. `T_fold = 0.0`, masih tidak pernah diukur.
7. Lintasan traverse A2.3 adalah **tafsir** offset sebagai waktu mati, bukan
   pengukuran. W1a hanya membuktikan ia konsisten dengan durasi terkunci.

---

## C. Prompt sesi berikutnya — G10

> Rekomendasi: **Opus 5, effort TINGGI.** Alasannya berbeda lagi dari G7/G8/G9,
> dan harus dibaca: G9 **gagal memenuhi kriterianya sendiri**, dan gagal dengan
> cara yang paling instruktif — modelnya benar dan terverifikasi, solvernya
> tidak. G10 tidak boleh mengulang pola itu: ia harus **membangun oraclenya
> lebih dulu** dan baru solvernya, bukan sebaliknya. Bagian yang gagal senyap di
> G10 adalah W2 sendiri: enumerator brute force yang salah tetap mencetak
> "0 ketidakcocokan".

```
Sesi G10 -- SCHED-4: menegakkan ground truth terkopel yang G9 TIDAK berhasil
tegakkan, lalu baru mengukur biaya koordinasi.

BACA DULU, berurutan:
1. docs/p1_g9_sched3.md   -- SELURUHNYA. B1 (predikat + Pertentangan 1:
                             syarat PERLU dipakai sebagai CUKUP), B2 (sertifikat
                             A2.4 lulus), B3 (Pertentangan 2 dan 3 + D13
                             meleset + bug admissibility), B4 (GERBANG GAGAL --
                             BACA SEBELUM MENULIS APA PUN), B6 (kenapa subset
                             Lemma 4 bias ke bawah), B7 (yang TIDAK diukur),
                             B9 (batasan)
2. docs/p1_g8_sched2.md   -- B4 (validate_schedule eksponensial), B6, B9
3. docs/p1_g7_sched.md    -- A1 (model, terkunci), A2-K1 (bahasa "exact
                             TERHADAP himpunan kandidat"), B3, B5
4. reachability_gng/sched.py, sched_coll.py, sched_heur.py,
   test/verify_sched_coll.py, test/eval_sched_coll.py

=== KEADAAN FISIK ===
Lengan 4x MASIH DILEPAS. Sesi ini SEPENUHNYA OFFLINE.

=== YANG SUDAH SELESAI DAN TERVERIFIKASI, JANGAN BANGUN ULANG ===
- Predikat BLOCK (sched_coll.pair_distance / blocked): jejak URDF exact,
  reduksi 2-D terbukti, W0 vs oracle sampling batas 1184 kasus 0 mismatch,
  W0b vs irm_sweep.base_pose 2.2e-16.
- Lintasan traverse A2.3 + sertifikat waktu kontinu A2.4: W1a/b/c/d LULUS.
  Durasi cocok dengan sched.traverse_time sampai 3.6e-15 s.
- Lemma 1 (parkir aman di rot = 0, margin 0.21 m) dan Lemma 3 (terkopel >=
  takterkopel): terverifikasi, dan Lemma 3 adalah batas bawah yang sah.
- gen_real_crowded (probe S2) ada dan hidup, TAPI BELUM PERNAH DIJALANKAN.
- Ambang Lemma 2 yang BENAR: 0.095 m, bukan 0.11. Jangan pakai (N1)/(N2)
  sebagai syarat cukup -- itu kesalahan pertama G9.
- KONSTRUKTOR BATAS ATAS SUDAH LAYAK (probe 2026-08-14, commit 40bfbf9):
  _repair_ub + UB_START_PROBES memberi rasio kurungan mean 1.126 max 1.211
  pada 13 instance yang mengikat, dari 1.600/1.936. Artinya G10 MULAI dengan
  incumbent yang ketat -- itu persis handicap terbesar B&B G9, dan ia sudah
  hilang. Jangan bangun ulang; pakai.
- DUA instance sudah punya Delta = 0 TERBUKTI EXACT (n4_s2_mr1, n4_s9_mr1:
  UB menyentuh LB). Jangan hitung ulang; pakai sebagai uji regresi -- solver
  G10 WAJIB tetap memberi 0.000 di keduanya.

=== YANG GAGAL DAN HARUS DIPERBAIKI DULU ===
- W2 TIDAK DIBANGUN. Ia satu-satunya uji yang menyerang Lemma 5 (himpunan
  waktu mulai berhingga). Tanpa W2, tidak ada ground truth.
- W2b GAGAL: dengan tabrakan DIMATIKAN, B&B mengembalikan 46.7772 di mana
  solve_exact memberi 44.7772 (n=4 mr=1 seed=1). Kelengkapan pencarian, bukan
  model tabrakan.
- W4 GAGAL karena validate_schedule() TIDAK BISA memodelkan waktu TUNGGU yang
  A2.5 tambahkan. Butuh penerus sadar-tunggu, DITULIS BARU, bukan mengubah
  test/verify_sched_exact.py (git diff-nya masih kosong, pertahankan).

=== TUGAS, BERURUTAN. JANGAN LOMPAT. ===
1. Gerbang sadar-tunggu: penerus validate_schedule() yang memutar ulang
   traverse + dwell + TUNGGU + A2.4. Uji ia MENOLAK jadwal yang dirusak
   sengaja (gerbang yang tidak pernah menyala = bukti lemah, p1_g8 B1).
2. W2: enumerator jadwal bersama brute force pada n <= 3, |P| <= 4, DUA
   gantry, dengan grid waktu-mulai RAPAT (bukan {batas leg lawan}), tanpa
   B&B, tanpa Lemma 5. Inilah yang menilai Pertentangan 3.
3. Perbaiki kelengkapan solver sampai W2b LULUS pada 12/12 dengan
   proved=True.
   ➜ REKOMENDASI KUAT, dan ini BUKAN selera -- ia diukur di B6: GANTI
     BENTUKNYA jadi DP atas (R1, p1, R2, p2) dengan frontier Pareto (t1, t2),
     yang menghindari pencarian waktu mulai SAMA SEKALI.
     Buktinya: probe B6 menunjukkan yang membuat kurungan longgar BUKAN B&B
     yang lambat, melainkan himpunan waktu mulai yang terlalu kasar
     (Pertentangan 3). Pada n4_s4_mr0 kedua gantry menyapu rot ~ -90 deg pada
     lin yang sama pada detik yang sama; obatnya menunda satu gantry beberapa
     detik, dan waktu itu BUKAN batas leg apa pun. Menambal daftar kandidat
     waktu mulai akan mengejar ekor terus-menerus; representasi yang membawa
     (t1, t2) sebagai state menghilangkan masalahnya.
4. BARU kemudian: K3 pada S1 DAN S2, K5.4, K5.6, lalu (a) mutex dan
   (b) urutan tur.
   Pertanyaan biner yang menentukan pembingkaian naskah, dan yang membuat
   G10 layak dibiayai: batas ATAS Delta pada 13 instance yang mengikat
   adalah mean 5.13 s / max 8.55 s (~4-17% dari makespan), TAPI 2 dari 13
   sudah terbukti Delta = 0 persis. Jadi:
     - kalau Delta sejati dekat batas atas -> koordinasi melewati ambang 5%
       A3-K3, dan ia mekanisme WAKTU pertama yang ditemukan sejak
       mutex = 0.000 s (g7 B3);
     - kalau dekat nol -> tabrakan mengulang pola mutex (sering mengikat,
       tidak berbiaya), dan lubang klaim-waktu paper TETAP terbuka.
   Kedua sisi sudah punya bukti pendukung. JANGAN menebak mana; ukur.

=== KUNCI KRITERIA SEBELUM KODE, ke docs/p1_g10_sched4.md A ===
1. Bagaimana W2 sendiri dibuktikan benar -- ia oracle, dan oracle yang salah
   mencetak "0 mismatch" dengan senang hati.
2. Berapa besar instance yang WAJIB proved=True, sebagai angka, sebelum
   solver boleh disebut ground truth. Patokan konkret yang sudah ada:
   13 instance S1 yang mengikat sekarang terkurung <= 1.211; menutup
   ke-13 itu adalah palang yang wajar, dan 2 di antaranya sudah 1.000.
3. Apa yang dilaporkan kalau lagi-lagi tidak semua instance bisa dibuktikan.

=== JEBAKAN YANG SUDAH DIUKUR, JANGAN DITEMUKAN ULANG ===
- Yang lambat adalah PEMERIKSA, bukan penjadwal. TIGA kali sekarang, semuanya
  dalam dua sesi: g8 B4 (validate_schedule eksponensial), g9 B3 (_first_start
  per aksi, 1.5e5 aksi per node), dan g9 B6 (evade menyapu 858 pose aman,
  masing-masing satu certificate walk, DI LUAR time budget). Pola ketiganya
  sama: sebuah loop O(besar) yang setiap iterasinya memanggil pemeriksa.
  Ukur pemeriksanya SEBELUM menjalankan sapuan, dan beri SETIAP loop yang
  memanggilnya sebuah batas eksplisit.
- h_g[R] dengan R yang masih dibagi bersama TIDAK admissible. Pakai min atas
  pembagian tugas. Bug ini ada di DUA tempat di G9 dan perbaikan pertama
  hanya menutup satu.
- Subset "Lemma 4 menjawab" BIAS KE BAWAH. Jangan hitung mean Delta atasnya
  lalu menyebutnya biaya koordinasi (g9 B6).
- Tabrakan lengan-lengan masih di luar model; lengan 1.4 m vs struktur
  0.455 m. Angka rugi TETAP batas bawah.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Papan skor 17 meleset, 2 tepat.
  Prior dunia: LONGGAR. Prior kode sendiri (g9 B8, kini 2 dari 2):
  LEBIH LAMBAT, LEBIH RUMIT, LEBIH SALAH dari yang terasa.
- Kalau B bertentangan dengan A, yang menang B, dan pertentangannya DITULIS.
  G9 punya EMPAT; itu bukan aib, itu prosesnya bekerja.
- Akhiri dengan prompt sesi berikutnya (G11).
```
