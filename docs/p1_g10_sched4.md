# P1 / G10 — SCHED-4: menegakkan ground truth terkopel, lalu mengukur

> Sesi G10, 2026-08-14. Melanjutkan [p1_g9_sched3.md](p1_g9_sched3.md).
> G9 **gagal memenuhi kriterianya sendiri** (`p1_g9 §B4`): modelnya benar dan
> terverifikasi, solvernya tidak. Sesi ini menegakkan solvernya lebih dulu, dan
> baru sesudah itu mengukur.
>
> **§A ditulis dan DIKUNCI SEBELUM satu baris kode sesi ini dijalankan.**
> §B diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**, dan
> pertentangannya ditulis **eksplisit**, bukan dihaluskan. §A tidak ditulis
> ulang belakangan — kesalahannya dibiarkan bisa dibaca (disiplin `p1_g7`,
> `p1_g8`, `p1_g9`).
>
> Sesi ini **sepenuhnya offline**. Tidak ada kamera, gantry, lengan,
> `move_group`. Lengan 4× masih dilepas.
>
> **Rekomendasi model: Opus 5, effort TINGGI, untuk seluruh sesi.** Alasannya
> berbeda lagi dari G7/G8/G9 dan harus dibaca: sesi ini membangun **oracle untuk
> oracle**. W2 adalah enumerator brute force yang menghakimi solver, dan
> **enumerator brute force yang salah mencetak "0 ketidakcocokan" dengan sama
> senangnya** dengan yang benar. G9 punya tiga fase yang gagal senyap; G10 punya
> satu, dan ia yang paling dalam. Karena itu §A3-K1 di bawah dimulai bukan dari
> "bagaimana solver dibuktikan", melainkan dari **"bagaimana W2 sendiri
> dibuktikan"**.

---

## A. Protokol — DIKUNCI SEBELUM MENJALANKAN APA PUN

### A0. Yang TIDAK dibuka ulang

| Hal | Terkunci / terverifikasi di | Dipakai bagaimana |
|---|---|---|
| Model intra-gantry (sumber daya, timeline, SR/MR, mutex, `dur = 2.0 × max(a,b,z)`) | `p1_g7 §A1`, V0–V4 | dibaca apa adanya, **nol perubahan** |
| `T_traverse = max(T_lin, T_rot)` | `p1_state §5.6`, W1a (3.6e-15 s) | dipakai apa adanya |
| Dwell 2.0 s, definisi makespan, handover = SATU tugas dua-lengan | `p1_g7 §A2-K2/K3` | tidak ditawar |
| Peta `cap_g{1,2}_rail160.npz` (33×72, \|P\| = 2376) | `p1_state §3` | satu-satunya oracle kelayakan |
| `sched.solve_exact` | `p1_g7 §B1` | **batas bawah yang sah** (Lemma 3), pembanding W2b |
| Predikat `BLOCK` (`sched_coll.pair_distance`, `blocked`) | `p1_g9 §B1`: W0 (1184 kasus, 0 mismatch), W0b (2.2e-16) | dipakai apa adanya, **nol perubahan** |
| Lintasan traverse A2.3 + sertifikat A2.4 (`Traj`, `first_block`) | `p1_g9 §B2`: W1a/b/c/d | dipakai apa adanya |
| Lemma 1 (parkir aman, margin 0.21 m), Lemma 2 (ambang **0.095 m**), Lemma 3 (terkopel ≥ takterkopel), Lemma 4 | `p1_g9 §A2.2/§A2.5`, W0c | dipakai; Lemma 2 memakai angka yang **sudah dikoreksi** |
| `_repair_ub` + `UB_START_PROBES` | `p1_g9 §B6` probe (rasio 1.126) | **dipakai, tidak dibangun ulang** |
| `gen_real`, `gen_random_small`, `gen_real_crowded` | `p1_g7 §A3`, `p1_g9 §A3-K2` | generator, seed sama dengan G7/G8/G9 |
| `sched_heur.pose_tour` + 3 baseline | `p1_g8 §B2` | dipanggil, **dibungkus, bukan disunting** |

🔒 **BEKU — `git diff` wajib kosong di akhir sesi:**

```
reachability_gng/sched.py          reachability_gng/sched_heur.py
reachability_gng/sched_coll.py     test/verify_sched_exact.py
test/verify_sched_coll.py          test/eval_sched_heur.py
test/eval_sched_coll.py
```

`sched_coll.py` **ikut dibekukan sesi ini**, dan itu keputusan sadar: W0/W0b/
W0c/W1 memverifikasi **berkas itu**, bukan idenya. Menyuntingnya berarti
verifikasi G9 tidak lagi berlaku pada kode yang dijalankan. Yang baru masuk
berkas baru dan mengimpor dari sana.

🔴 **Kalau bug ditemukan di berkas beku:** ia **dilaporkan sebagai temuan**,
diperbaiki di tempatnya, dan pembekuannya dinyatakan gugur secara eksplisit di
§B — tidak ditambal diam-diam di berkas baru.

### A1. Yang dibangun sesi ini, dan yang TIDAK

Dibangun, berurutan:

1. **Gerbang sadar-tunggu** (A2.4) — penerus `validate_schedule()` yang bisa
   memvalidasi jadwal yang mengandung waktu tunggu. Plus **uji mutasi**: gerbang
   yang tidak pernah menolak apa pun bukan gerbang (`p1_g8 §B1`).
2. **`feasible_starts`** (A2.2) — jawaban terhadap `p1_g9` Pertentangan 3.
   Conservative advancement **dalam `s`** (waktu mulai), bukan hanya dalam `t`,
   dengan konstanta Lipschitz yang diturunkan di A2.2 dan bukan ditebak.
3. **`solve_coupled2`** (A2.3) — B&B yang sama bentuknya, ditambah **dive**
   (konstruktor jadwal layak di setiap node) sebagai sumber incumbent.
4. **W2** — enumerator jadwal bersama brute force, **plus buktinya sendiri**
   (A3-K1).
5. **Pengukuran**: K3 pada S1 **dan** S2, K5.4 statistik predikat, K5.6 sapuan
   `c_clear`, ablasi mutex, ablasi urutan tur.

**TIDAK dibangun sesi ini, dan tidak boleh menyelinap masuk:** `T_fold ≠ 0`,
tabrakan lengan–lengan antar gantry, tabrakan lengan–lingkungan, kapabilitas
dinamis, dan **heuristik baru apa pun** sebelum gerbang A3-K2 lulus
(`p1_state §7.1`).

### A2. PENAMBAHAN MODEL DAN ALGORITMA — diturunkan sekarang, dikunci sekarang

#### A2.1 🔺 Pertentangan 0 — dengan prompt G10 sendiri: `(R, p1, p2)` + Pareto `(t1, t2)` **BUKAN statistik cukup**

`p1_g9 §C` menulis **"REKOMENDASI KUAT, dan ini BUKAN selera"**: ganti bentuk
solver jadi DP atas `(R1, p1, R2, p2)` dengan frontier Pareto `(t1, t2)`.
Rekomendasi itu **ditolak di sini, sebelum kode ditulis**, dengan alasan yang
bisa diperiksa. Menuliskannya di §A adalah intinya: kalau saya salah, §B akan
menegur saya dan pertentangannya sudah punya tempat.

**Kenapa ia tidak cukup.** Kendala A2.4 berbunyi *untuk setiap `t`*. Kalau gantry
`g` memulai leg pada `s` dan gantry `h` sedang **di tengah leg** pada
`[s, s+T]`, ujinya menuntut pose `h` **selama** selang itu. Pasangan
`(p_h, t_h)` hanya menyebut **di mana `h` berakhir** dan **kapan ia bebas** — ia
tidak menyebut lintasannya. Dua keadaan dengan `(R, p1, p2, t1, t2)` **identik**
bisa berbeda kelayakannya:

```
h leg A:  (0.80, +90 deg) -> (0.80, 0 deg)      berakhir di p_h, waktu t_h
h leg B:  (0.80, -90 deg) -> (0.80, 0 deg)      berakhir di p_h, waktu t_h  (sama)
```

Selama leg A, `h` menyapu `rot` dari +90°; selama leg B dari −90°. Menurut
Lemma 2 (ambang terkoreksi 0.095 m) hanya salah satunya yang bisa menabrak sapuan
`g` yang bersamaan. Jadi DP itu **tidak sound** (ia akan meloloskan jadwal yang
menabrak) kecuali ia melarang leg tumpang tindih sama sekali — dan melarang
konkurensi antar gantry berarti membuang paralelisme, yaitu membuang seluruh
alasan punya dua gantry.

**Statistik cukup yang minimal**, diturunkan sekarang dan dipakai sebagai
keadaan:

```
(R, g, p_g, t_g, ekor_h)      ekor_h = (p_h_prev, s_h, a_h, p_h, t_h)
```

dengan `g` = gantry yang `t`-nya lebih kecil (aturan ekspansi G9,
dipertahankan), `s_h` = waktu berangkat leg terakhir `h`, `a_h` = waktu tiba.
Kalau `a_h ≤ t_g` maka `h` statis di `p_h` dan ekornya menyusut jadi `p_h` saja.

**Dan dominasi Pareto komponen-per-komponen tidak sah**, yang membunuh separuh
sisa rekomendasinya:

🔒 **LEMMA 6 — "lebih awal" TIDAK selalu lebih baik.** Ambil dua keadaan yang
sama persis kecuali `h` tiba di `p_h` pada `a_h` versus `a_h' > a_h`. Kalau
`p_h` memblokir pose yang dibutuhkan `g`, keadaan dengan `a_h` yang **lebih
awal** memblokir `g` **lebih lama**, sehingga bisa memaksa makespan lebih besar.
Maka `(t1, t2)` terurut komponen-per-komponen bukan relasi dominasi yang sah pada
model ini. ∎
Akibatnya: memoisasi Pareto pada `(t1, t2)` **boleh membuang optimum**. Yang
dipakai sebagai gantinya adalah pemangkasan B&B murni (batas bawah admissible),
yang tidak pernah membuang optimum menurut konstruksi.

➜ **Yang dilakukan sebagai gantinya**, dan alasannya diukur, bukan dipilih:
`p1_g9 §B3` dan `§B6` sudah mengukur **dua** sebab kegagalan G9 yang **berbeda**,
dan masing-masing dapat obatnya sendiri:

| sebab terukur | di mana | obat sesi ini |
|---|---|---|
| himpunan waktu mulai terlalu kasar (tabrakan **di tengah traverse**, obatnya menunda beberapa detik, dan detik itu bukan batas leg apa pun) | `p1_g9 §B6` | **A2.2** `feasible_starts` |
| pencarian tidak lengkap: 1 dari 12 W2b mengembalikan nilai seed-nya sendiri | `p1_g9 §B4` | **A2.3** dive |

#### A2.2 🔒 `feasible_starts` — himpunan waktu mulai, DITURUNKAN

Aksi: gantry `g` dengan lintasan terkomit `A`, berangkat pada `s ≥ t_min`,
traverse `T` ke `q`, lalu menahan `dur`. Lawan `B` sudah terkomit penuh.
Jendela aksi `W = T + dur`. Definisikan

```
f(s) = min over t in [s, s+W] of dist( A_s(t), B(t) )
```

🔒 **LEMMA 7 — `f` adalah `V_POINT`-Lipschitz dalam `s`.**
`A_s(t) = A(t − s)` adalah pergeseran waktu murni: lintasan `g` **dalam jamnya
sendiri** tidak berubah oleh `s`. Ambil `s1 < s2`, `δ = s2 − s1`, dan `t*` yang
mencapai minimum untuk `s1`. Ambil `t' = t* + δ ∈ [s2, s2+W]`. Maka
`A_{s2}(t') = A(t* − s1)` — **titik yang sama persis**. Yang berubah hanya
`B`, dan setiap titik `B` bergerak paling cepat `V_POINT`, jadi
`dist` berubah paling banyak `V_POINT·δ`. Karena itu
`f(s2) ≤ f(s1) + V_POINT·δ`, dan simetrisnya. ∎

```
V_POINT = v_lin + omega * R_MAX = 0.031416 + 0.174533 * 0.455 = 0.110828 m/s
```

Perhatikan: konstantanya `V_POINT`, **bukan** `V_REL = 2 V_POINT` yang dipakai
A2.4 — di sana kedua benda bergerak relatif satu sama lain; di sini lintasan `g`
diam dalam jamnya sendiri dan hanya `B` yang bergerak. Ini konstanta yang
**lebih ketat**, dan itu diturunkan, bukan diambil dari A2.4 karena kebetulan
ada.

🔒 **LANGKAH MAJU YANG SAH.** Kalau `s` terhalang, `first_block` (A2.4, sudah
terverifikasi W1) memberi instan pelanggaran pertama `τ`, dan `d* =
dist(A_s(τ), B(τ)) ≤ c_clear + ε`. Untuk `s' = s + δ`, pada instan `τ + δ`
konfigurasi `g` **identik** dan `B` bergeser ≤ `V_POINT·δ`, jadi

```
dist( A_{s'}(τ+δ), B(τ+δ) )  <=  d* + V_POINT * delta
```

Maka setiap `s'` dengan `δ ≤ (c_clear − d*) / V_POINT` **pasti masih terhalang**
— dilewati dengan sah, bukan dengan harapan. Langkahnya:

```
delta = max( (c_clear - d*) / V_POINT , EPS_S )
```

🔒 **`EPS_S = 0.01 s` DIKUNCI**, dan konsekuensinya ditulis apa adanya: lantai
`EPS_S` bisa **melewati** selang layak yang lebih pendek dari `EPS_S`, jadi waktu
mulai yang dikembalikan boleh **terlambat** sampai `EPS_S` dari yang sejati.
Arah biasnya sama dengan `ε` A2.4: makespan yang dilaporkan adalah **batas atas**.
Besarnya dibatasi dan dihitung sekarang, bukan nanti: paling banyak satu aksi per
tugas plus satu gerak menghindar per gantry, jadi `≤ (n+1)·EPS_S = 0.07 s` pada
`n = 6` — **0.17% dari makespan tipikal 40 s**, dan **dua kali lipat di bawah**
resolusi yang dibutuhkan ambang 5% A3-K3 (2.0 s). `EPS_S` juga 76× di bawah
manuver mengelak termurah yang ada (0.76 s, `p1_g9 §A3-K3`), jadi ia tidak bisa
membalik satu pun keputusan struktural.

🔒 **HIMPUNAN KANDIDAT.** Jangkar (anchors):

```
{ t_min } U { batas leg B yang > t_min } U { B.end_time() }
```

dan untuk **setiap** jangkar, `feasible_starts` menjalankan langkah maju di atas
sampai layak atau melewati cakrawala `max(t_min, B.end_time())`. Hasilnya
himpunan **waktu mulai layak paling awal di atau sesudah setiap jangkar** —
superset sejati dari himpunan G9 (yang memakai jangkar itu **mentah**, tanpa maju
ke titik layak), dan ia memuat waktu mulai layak paling awal secara global.
Setelah `B.end_time()` lawan statis, jadi kelayakan berhenti bergantung pada `s`
dan satu uji menyelesaikan seluruh ekor tak hingga. Berhingga menurut
konstruksi.

🔒 **`K_START = 8`** waktu mulai per aksi, diambil dari yang paling awal.
Kalau tutup ini tersentuh, **dihitung dan dilaporkan sebagai angka** (K5.8) —
karena tutup yang tidak pernah dilaporkan adalah penjarangan yang disembunyikan.

➜ Maka solver tetap **exact TERHADAP himpunan kandidat ini**, bahasa yang
`p1_g7 §A2-K1` sediakan. Yang berubah dari G9: himpunannya sekarang **diturunkan
dengan bukti** dan **diuji oleh W2**, bukan dipilih karena murah.

#### A2.3 🔒 DIVE — dari mana incumbent berasal

`p1_g9 §B4` mengukur kegagalan kelengkapan: 1 dari 12 instance W2b mengembalikan
**persis nilai seed incumbent-nya**, yaitu pencarian tidak pernah menemukan apa
pun. Sebabnya bukan pemangkasan yang salah melainkan **tidak ada mekanisme yang
menghasilkan jadwal lengkap** kecuali dengan mengembangkan sampai `R = 0`.

```
di SETIAP node yang di-pop:
    UB_node = dive(keadaan node)     jadwal LAYAK penuh dari keadaan itu,
                                     dibangun dengan kelanjutan optimal DP
                                     per gantry, diperbaiki oleh feasible_starts
    kalau UB_node < best_m           -> incumbent baru
    kalau UB_node <= node_lb + 1e-9  -> node ini optimal
```

🔒 **LEMMA 8 — aturan berhenti.** Best-first: node yang di-pop punya `node_lb`
minimum di seluruh heap, dan `_split_lb` admissible (Lemma 3), jadi
`node_lb ≤ optimum`. Kalau dive dari node itu menghasilkan jadwal layak bernilai
`node_lb`, maka `optimum ≤ node_lb ≤ optimum`. **Selesai, terbukti optimal.** ∎

⚠️ **Dan justru karena itu dive berbahaya, dan bahayanya dikunci sekarang.**
Dengan tabrakan dimatikan, dive di **akar** langsung mengembalikan optimum
takterkopel = LB, jadi W2b akan lulus **tanpa mesin pencariannya pernah
berjalan** — persis gerbang-yang-tidak-pernah-menyala `p1_g8 §B1`, dalam bentuk
yang lebih halus karena ia terlihat seperti kemenangan.

🔴 **Maka W2b DIJALANKAN DUA KALI**, dan keduanya wajib lulus:
`(i)` dive menyala (jalur normal), `(ii)` **`no_dive = True`** — pencarian harus
menemukan optimum dengan ekspansi saja. Hanya `(ii)` yang menghitung sebagai uji
kelengkapan.

#### A2.4 🔒 GERBANG SADAR-TUNGGU — dan representasi jadwal yang diperlebar

`p1_g9 §B4` Pertentangan 4 mengukur: `validate_schedule()` menjumlahkan traverse
dan dwell **tanpa tempat untuk waktu tunggu**, jadi ia tidak bisa memvalidasi
keluaran model A2.5. Diperbaiki di sini, dan perbaikannya menyingkap sesuatu yang
§A9 G9 lewatkan:

🔺 **Representasi jadwal G7/G8/G9 AMBIGU pada model bertunggu.** Sebuah
perhentian `(pose, start, dur)` tidak menyebut **kapan traverse-nya terjadi**.
A2.5 mengizinkan menahan di pose sekarang **sebelum** berangkat, dan juga
menahan **setelah tiba** sebelum mulai bekerja. Dua penempatan leg yang berbeda
punya jejak tabrakan yang berbeda. Maka:

🔒 **Setiap perhentian membawa `depart` (waktu berangkat) DAN `start` (waktu
dwell mulai)**, dengan

```
depart >= perhentian sebelumnya .start + .dur          (tunggu di pose asal)
start  >= depart + T(pose sebelumnya, pose)            (tunggu di pose tujuan)
```

Gerbangnya (`validate_coupled`) memutar ulang **dari aturan**, bukan dari solver:
`T` dari `ref_traverse` (fungsi independen yang sudah dipakai V0/V4), panjang
dwell dari `ref_stop_slots` (fungsi yang sama, tanpa modifikasi, yang G7 V4 dan
G8 K4 pakai), kelayakan lengan langsung dari array oracle, lalu **kendala A2.4
diputar ulang pada `ε/10`** dari lintasan yang direkonstruksi dari `depart`.

🔴 **UJI MUTASI WAJIB.** Gerbang diadu dengan jadwal yang **sengaja dirusak**,
minimal lima kelas, dan wajib menolak **setiap** mutasi:

| mutasi | apa yang diuji |
|---|---|
| M1 `start` dimajukan 0.5 s | aritmetika traverse/tunggu |
| M2 `depart` dimajukan sampai leg tumpang tindih dengan leg lawan yang menabrak | separuh A2.4 gerbang |
| M3 satu tugas dibuang dari `tasks` | cakupan tugas |
| M4 satu tugas dipindah ke lengan yang tidak menjangkau | kelayakan lengan |
| M5 `dur` dikurangi satu slot | mutex/slot |

Jumlah mutasi yang **lolos** (yaitu gerbang gagal menolak) dilaporkan sebagai
angka, dan wajib **0**.

#### A2.5 Apa yang tidak berubah

Model A2.1–A2.5 `p1_g9` berlaku utuh: geometri, `BLOCK`, lintasan A2.3, kendala
kontinu A2.4 (`ε = 0.005 m`), aturan menunggu, definisi makespan, `c_clear = 0.0`
sebagai default terkunci. Gerak menghindar (`p1_g9` Pertentangan 2) **tetap
bagian model** dan tidak diperdebatkan ulang.

### A3. Kriteria yang DIKUNCI

#### K1 — BAGAIMANA **W2 SENDIRI** DIBUKTIKAN BENAR

Ini pertanyaan pertama prompt G10, dan ia ditaruh pertama karena W2 adalah satu-
satunya bagian sesi ini yang **tidak punya apa pun di atasnya**.

**Bentuk W2 dikunci:** pencarian **tuntas** atas jadwal bersama pada instance
kecil. Dua gantry. Di setiap langkah, gantry dengan `t` lebih kecil bertindak;
aksinya adalah `(pose tujuan ∈ P, himpunan tugas U ⊆ R yang layak di sana atau
U = ∅ untuk gerak menghindar, waktu mulai dari GRID SERAGAM)`. Tanpa B&B, tanpa
DP, tanpa `_split_lb`, tanpa `feasible_starts`, **tanpa Lemma 5 dan tanpa
Lemma 7**. Pemangkasan satu-satunya adalah incumbent (`acc ≥ best` → potong),
yang tidak membuang optimum. Grid waktu mulai `δ` **seragam**, dikunci
`δ = 0.25 s`, dan **dihalfkan jadi 0.125 s** pada instance yang mismatch.

**Lima bukti untuk W2 itu sendiri, semua wajib lulus** — dinamai `P0..P4` supaya
tidak tertukar dengan `W`:

| | Apa yang diadu | Kenapa itu mengikat |
|---|---|---|
| **P0** | lintasan dan `pose_at` versi W2 ditulis ULANG dari teks A2.3 di berkas uji, diadu dengan `sched_coll.Traj.pose_at` pada 2 000 waktu acak | kalau W2 memakai lintasan solver, ia tidak independen pada sumbu yang paling penting |
| **P1** | W2 dengan `BLOCK ≡ False` vs `sched.solve_exact` pada instance kecil yang **sama** | tanpa tabrakan, brute force wajib mereproduksi oracle yang sudah dibuktikan V0–V4 |
| **P2** | setiap jadwal yang W2 kembalikan wajib lolos **gerbang A2.4** (`validate_coupled`) | angka optimal yang menempel pada jadwal tak-tereksekusi tetap jawaban salah (pelajaran V4) |
| **P3** | `W2 ≥ optimum takterkopel` pada setiap instance (Lemma 3) | invarian yang tidak bisa dipenuhi enumerator yang bocor |
| **P4** | monotonisitas grid: `δ = 0.125` tidak boleh memberi nilai **lebih besar** dari `δ = 0.25` | enumerator yang melewatkan cabang biasanya tidak monoton; grid yang lebih rapat adalah superset (0.25 = kelipatan 0.125) |

🔴 **Kalau P0–P4 tidak semuanya lulus, W2 tidak boleh dipakai menghakimi apa
pun**, dan sesi ini melapor GAGAL di titik itu, bukan melanjutkan.

**Apa yang W2 hakimi, dan ke arah mana.** Solver memakai himpunan kandidat
waktu mulai (A2.2); W2 memakai grid seragam. Keduanya **restriksi**, jadi
perbandingannya asimetris dan aturannya dikunci sekarang:

```
solver > W2 + 1e-9   ->  KEGAGALAN SOLVER. Himpunan kandidatnya membuang
                         solusi yang grid rapat menemukan. Dilaporkan sebagai
                         angka dan sebagai instance.
solver < W2 - 1e-9   ->  BUKAN kegagalan: solver memakai waktu mulai di luar
                         grid. Dihitung, dilaporkan, dan diperiksa dengan
                         menghalfkan delta -- kalau W2 turun ke solver, gridnya
                         yang kasar; kalau tidak, itu jadi temuan.
```

#### K2 — PALANG "GROUND TRUTH", SEBAGAI ANGKA

Pertanyaan kedua prompt G10. Dijawab sebagai daftar angka, bukan sebagai
kalimat. `solve_coupled2` boleh disebut **ground truth (terhadap himpunan
kandidat A2.2)** jika dan hanya jika **semua** baris ini lulus:

| | Palang, sebagai angka |
|---|---|
| **Gerbang mutasi** | 5 kelas mutasi × ≥ 3 jadwal = **≥ 15 mutasi, 0 lolos** |
| **P0–P4** | semua lulus (K1) |
| **W2** | **≥ 24 instance**, `n ≤ 3`, `\|P\| ≤ 4`, 2 gantry; `solver > W2` pada **0 dari 24** |
| **W2b (i)** dive menyala | **12 dari 12** identik dengan `solve_exact` sampai **1e-9**, `proved = True` |
| **W2b (ii)** `no_dive` | **12 dari 12** identik sampai **1e-9**, `proved = True` |
| **W3** | **Q1–Q5 lima-limanya**, termasuk **Q3 yang G9 tidak pernah bangun** (`p1_g9 §B4` mencatat W3 lulus atas Q1/Q2/Q4/Q5 saja) |
| **W4′** | **100%** jadwal yang dilaporkan lolos gerbang sadar-tunggu, **0 pelanggaran** |
| **Regresi G9** | `n4_s2_mr1` dan `n4_s9_mr1` tetap `Δ = 0.000` (keduanya sudah terbukti exact, `p1_g9 §B6`) |

🔒 **Palang terpisah, dan JANGAN dicampur dengan yang di atas: "S1 TERJAWAB"**
= 40 dari 40 instance S1 punya `Δ` exact. Patokan konkret dari `p1_g9 §B6`: 27
sudah exact, 13 terkurung ≤ 1.211, 2 di antaranya sudah 1.000. **Menutup ke-13
itu adalah palang yang wajar dan itulah targetnya.** Tapi ia palang
**pengukuran**, bukan palang kebenaran: solver boleh jadi ground truth dan tetap
kehabisan waktu pada instance tertentu. Mencampur keduanya adalah cara termudah
menggeser definisi setelah melihat hasil.

#### K3 — VONIS BIAYA KOORDINASI, DAN APA YANG DILAPORKAN KALAU TIDAK SEMUA TERBUKTI

Pertanyaan ketiga prompt G10. Ambang **tidak digeser** — persis `p1_g9 §A3-K3`,
yang sendiri persis `p1_g8 §A3-K1`:

```
Delta      = makespan_terkopel - makespan_takterkopel      [detik, >= 0, Lemma 3]
Delta%     = Delta / makespan_takterkopel * 100 %

MAHAL              := mean Delta% >= 5.0 %  pada S1
TERUKUR TAPI MURAH := 0 < mean Delta% < 5.0 %  pada S1
GRATIS             := max Delta = 0.000 s pada SELURUH S1
```

🔒 **ATURAN VONIS DUA-SISI — dikunci sekarang, sebelum tahu berapa instance yang
bisa dibuktikan.** `p1_g9 §B6` terpaksa menulis "TIDAK DAPAT DITENTUKAN" karena
ia hanya bisa merata-ratakan subset yang **didefinisikan oleh `Δ = 0`**. Itu
tidak boleh terulang sebagai satu-satunya kemungkinan. Maka:

```
untuk instance terkurung, Delta_lo = 0            (Lemma 3)
                          Delta_hi = UB - LB      (kurungan yang dilaporkan)

mean_lo  = mean atas SELURUH S1 dengan setiap kurungan diambil Delta_lo
mean_hi  = mean atas SELURUH S1 dengan setiap kurungan diambil Delta_hi

kalau mean_lo dan mean_hi jatuh di SISI YANG SAMA dari 5.0 %  -> VONIS SAH
kalau tidak                                                   -> TIDAK DAPAT
                                                                 DITENTUKAN
```

Ini rigor, bukan kelonggaran: `mean_lo` dan `mean_hi` dihitung atas **seluruh 40
instance**, bukan atas subset, jadi tidak ada instance yang dibuang dan tidak ada
subset yang mengukur definisinya sendiri. Kalau vonisnya sah, ia sah **apa pun
nilai `Δ` sejati di dalam kurungan**.

🔴 Ambang 5.0% **dilarang** digeser setelah melihat hasil; instance **dilarang**
dibuang dari rata-rata. S1 dan S2 **tidak pernah** dirata-ratakan bersama.

#### K4 — HIMPUNAN INSTANCE UJI

Seed sama dengan G7/G8/G9. `S1`, `S2`, `S3` **persis** seperti `p1_g9 §A3-K2`
(masing-masing 40 / 40 / 20 instance), dipakai ulang tanpa perubahan supaya
angkanya bisa diadu langsung dengan `p1_g9 §B5/§B6`. `S4` = patologis Q1–Q5.
Baru sesi ini:

| Set | Isi | Peran |
|---|---|---|
| **S5 W2** | `gen_random_small`, `n ∈ {2,3}`, `\|P\| ∈ {3,4}`, 2 gantry, mr 0/1, ≥ 24 instance | satu-satunya bahan bakar W2 |

**Anggaran: `≤ 120 s dinding per instance`**, angka yang sama dengan
`p1_g7 §A2-K1` dan `p1_g9 §A3-K2`, **tidak dilonggarkan**. Instance yang tidak
muat dilaporkan sebagai kurungan dan masuk aturan dua-sisi K3.

#### K5 — YANG DILAPORKAN, TERLEPAS DARI HASILNYA

1. `Δ`, `Δ%` per konfigurasi `(set, n, n_mr)`: mean, median, p90, max, fraksi
   yang mengikat. **S1 dan S2 terpisah.**
2. `mean_lo` / `mean_hi` dan vonis dua-sisi K3, untuk S1 dan S2.
3. Berapa instance selesai lewat Lemma 4, lewat dive, dan lewat ekspansi penuh —
   tiga angka terpisah. Kalau dive menjawab 100%, **itu dikatakan**.
4. Statistik predikat pada grid: fraksi `2376 × 2376` pasangan pose yang `BLOCK`,
   dan fraksi pasangan `(rot1, rot2)` yang memenuhi (N1).
5. **(a)** mutex `r = 0.20` pada model terkopel — masih `0.000 s`?
   **(b)** urutan tur (`nn-only` / `cover-order`) pada model terkopel — masih
   `0.00%`? **(c)** biaya koordinasi = K3.
6. Sapuan `c_clear ∈ {0.00, 0.05, 0.10}` pada S1 dan S2.
7. Waktu dinding `solve_coupled2` vs `solve_exact`; `n` terbesar yang muat 120 s.
8. **Berapa kali tutup `K_START = 8` tersentuh, berapa kali `ACTION_BUDGET`
   tersentuh, berapa kali `EPS_S` menjadi lantai langkah** — tiga angka, karena
   ketiganya adalah penjarangan dan penjarangan yang tidak dilaporkan adalah
   penjarangan yang disembunyikan.
9. Jumlah jadwal yang gagal gerbang, per penjadwal. Jumlah mutasi yang lolos.

### A4. Yang TIDAK dimodelkan — setelah sesi ini

Seluruh `p1_g7 §A4`, `p1_g8 §B10`, `p1_g9 §A4`/`§B9` **masih berlaku**, dan
diulang di sini karena §B akan menggodanya:

1. 🔴 **Tabrakan lengan–lengan antar gantry masih di luar model.** Ujung lengan
   terentang 1.4 m dari sumbu rotasi, **tiga kali** `R_MAX = 0.455` yang
   dimodelkan. **Angka rugi TETAP BATAS BAWAH.**
2. `T_fold = 0.0`, tidak pernah diukur.
3. Gerak lengan di dalam satu pose = 0.
4. Exact hanya terhadap grid 33 × 72, terhadap `c_clear = 0.0`, dan terhadap
   himpunan kandidat waktu mulai A2.2.
5. Konservatisme: `ε = 0.005 m` (A2.4, terukur mengikat 1 dari 250) **dan**
   `EPS_S = 0.01 s` (A2.2, arah diketahui, besar dibatasi ≤ 0.07 s).
6. Lintasan traverse A2.3 tetap **tafsir** offset sebagai waktu mati.

### A5. Papan skor §7.2 — dugaan sesi ini, ditulis di muka

Papan skor: **17 meleset, 2 tepat** (`p1_g9 §B8`). Dua prior kerja, dan sesi ini
sengaja menguji keduanya:

* **tentang DUNIA:** kendala yang belum diukur itu **LONGGAR** — tapi `p1_g9 §B6`
  baru memberi contoh tandingan pertamanya (D9), pada kelas kendala **eksklusi
  ruang bersama**, bukan kelayakan statis.
* **tentang KODE SENDIRI** (`p1_g9 §B8`, kini 2 dari 2): **lebih lambat, lebih
  rumit, lebih salah dari yang terasa.**

| # | Dugaan | Tentang | Kenapa |
|---|---|---|---|
| **D14** | W2 menemukan **≥ 1** instance dengan `solver > W2` | **kode sendiri** | prior kode sendiri dipakai secara harfiah: himpunan kandidat A2.2 saya sendiri, dan saya menduga ia masih bocor di suatu tempat |
| **D15** | `feasible_starts` + dive menutup **≥ 8 dari 13** instance S1 yang mengikat jadi exact dalam 120 s | **kode sendiri** | probe `p1_g9 §B6` sudah menutup 2 dari 13 hanya dengan 40 probe seragam; kandidat yang punya bukti seharusnya jauh lebih baik — tapi prior kode sendiri menahan angkanya di 8, bukan 13 |
| **D16** | Vonis K3 pada S1 sah dan berbunyi **TERUKUR TAPI MURAH** (0 < mean `Δ%` < 5%) | dunia | `p1_g9 §B6`: batas atas `Δ` mean 5.13 s pada makespan 31–51 s → `mean_hi` ≈ 3–4%, sudah di bawah 5% **sebelum** dipersempit |
| **D17** | S2 (sempit) **MENGIKAT**: mean `Δ%` ≥ 5% | dunia | D10 G9 yang tidak pernah diukur, diterbitkan ulang tanpa perubahan |
| **D18** | Mutex `r = 0.20` tetap **0.000 s** pada model terkopel | dunia | D12 G9, diterbitkan ulang tanpa perubahan |
| **D19** | Urutan tur tetap **0.00% pada S1**, tapi **> 0 pada S2** | dunia | D11 G9, diterbitkan ulang tanpa perubahan |

D17–D19 adalah dugaan G9 yang **tidak pernah dinilai** karena gerbangnya gagal.
Menerbitkannya ulang **tanpa mengubah kata-katanya** adalah bagian dari
disiplinnya: dugaan yang menghindari penilaian dua sesi berturut-turut adalah
dugaan yang tidak pernah berisiko.

### A6. Berkas

| Berkas | Isi |
|---|---|
| `test/gate_sched_coupled.py` | `validate_coupled` (gerbang sadar-tunggu) + uji mutasi M1–M5 |
| `reachability_gng/sched_coupled.py` | `feasible_starts` (A2.2), `dive` (A2.3), `solve_coupled2` |
| `test/verify_sched_coupled.py` | P0–P4, W2, W2b(i)/(ii), W3 (Q1–Q5), W4′, regresi G9 |
| `test/eval_sched_coupled.py` | K3 S1/S2/S3, K5.1–K5.9, sapuan `c_clear`, ablasi mutex + urutan tur |
| `docs/p1_g10_sched4.md` | dokumen ini |

### A7. Urutan kerja, dan apa yang terjadi kalau waktu habis

```
1. gerbang sadar-tunggu + uji mutasi M1-M5      <- tanpa ini tidak ada yang bisa dinilai
2. feasible_starts + dive + solve_coupled2
3. W2 + P0-P4                                   <- oracle DULU, baru penilaian
4. W2b(i)/(ii), W3 Q1-Q5, W4', regresi G9       <- GERBANG K2: tidak lanjut kalau gagal
5. K3 pada S1, S2, S3 + K5.1-K5.4 + K5.7-K5.9   <- angka utama sesi ini
6. K5.5(a) mutex, K5.6 sapuan c_clear
7. K5.5(b) ablasi urutan tur                    <- perlu pembungkus heuristik
```

🔴 **Apa pun yang tidak tercapai dilaporkan sebagai TIDAK DIUKUR, sebagai kalimat
eksplisit di §B.** Dilarang menuliskan "diperkirakan tidak berubah" untuk sesuatu
yang tidak dijalankan. `p1_g9 §B7` mematuhi aturan ini dengan lima baris TIDAK
DIUKUR; kalau sesi ini harus menulis lima baris lagi, ia menulisnya.

---

## B. Hasil terukur

> §A dikunci 2026-08-14 sebelum `sched_coupled.py` ada. Semua angka di bawah
> keluar sesudahnya. Setiap tempat di mana §B bertentangan dengan §A ditandai 🔺.
> **§A TIDAK ditulis ulang.**
>
> ⏳ **Sesi ini masih berjalan.**
