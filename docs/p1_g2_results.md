# G2 — hasil sapuan inverse reachability

> Menjalankan §5 dari [p1_plan.md](p1_plan.md). Dijalankan 2026-08-12.
> Skrip: [`irm_sweep.py`](../ros2_ws/src/reachability_gng/reachability_gng/irm_sweep.py)
>
> **Angka di sini menggantikan §2 p1_plan.md.** Semua angka §2 berasal dari peta
> seeding IK 3000-node; sapuan ini memakai 2 juta sampel. Selisihnya besar dan
> arahnya tidak menguntungkan — lihat §5.

---

## 1. Verifikasi URDF — pose base lengan memang fungsi kaku dari (linear, rotation)

```
python3 -m reachability_gng.irm_sweep verify
```

Rantai di `workcell_full.urdf` dari `world` ke base lengan:

```
world --fixed--> t{k}_base_link --prismatic x--> t{k}_platform_link
      --revolute z--> t{k}_rotation_link --fixed--> mount_plate --fixed--> arm base
```

Tidak ada sendi lain di antaranya. Tiga uji, masing-masing atas 4 lengan:

| Uji | Isi | Deviasi maks |
|---|---|---|
| A | pose base tidak berubah terhadap 6 sendi lengan | **1.1e-16** |
| B | bentuk tertutup `T_a(g)` vs pinocchio | 2.65e-06 |
| C | pose tool dunia == `T_a(g) · (pose tool di frame base)` | 2.56e-06 |

Residu 2.65e-06 pada B/C **bukan** galat model: URDF menulis roll base lengan
sebagai literal `3.14159`, yaitu `π − 2.65e-06`. Memakai literal itu di bentuk
tertutup, deviasi turun ke 2.8e-16. Jadi 2.7 µm di tool — empat orde di bawah
toleransi jangkauan 5 cm. **Asumsi sapuan sah.**

### Letak sumbu rotasi (yang diminta dicek khusus)

Sumbu rotasi **vertikal**, di `(x = linear, y = ±0.36, z = 2.01)`. Pelat mount
berjarak **0.4 m** dari sumbu itu, jadi base lengan mengayun di lingkaran radius
0.4 m — bukan berputar di tempat. Bentuk tertutupnya:

```
p_base = ( linear − y_p·sin(ψ),  y_b + y_p·cos(ψ),  1.9525 ),   ψ = π/2 + rotation
R_base = Rz(rotation + yaw_off) · Rx(π)
```

`y_b = ±0.36` (rel), `y_p = ∓0.4` (offset pelat), `yaw_off = 0` (pelat kanan) atau
`π` (pelat kiri).

### Temuan struktural: kopling intra-gantry adalah pergeseran π yang eksak

Dua lengan pada satu gantry duduk diametral. Dari bentuk tertutup di atas:

> **`T_arm2(linear, rotation) ≡ T_arm1(linear, rotation + π)`** — cocok sampai 2.2e-16.

Konsekuensinya `G_arm2(t)` adalah `G_arm1(t)` yang digeser setengah putaran; di
sapuan ini kedua peta terbukti identik setelah digeser (0.000% sel berbeda).
Jadi kopling intra-gantry bukan sekadar "berbagi `g`" — ia **satu batasan
geser-π**. Ini pernyataan yang lebih kuat dan lebih tajam daripada yang ada di
draf, dan ia gratis (murni geometri URDF).

Cloud tool di frame base juga **bit-identik** untuk keempat lengan (Gen3 Lite
sama persis), jadi satu cloud + empat transformasi sudah cukup.

---

## 2. Metode sapuan

```
cloud   2.000.000 konfigurasi lengan acak, FK di frame base lengan
        -> polyline (base, elbow, wrist, tool) + sumbu approach tool
sweep   grid gantry 41 (linear, 5 cm) × 72 (rotation, 5°) = 2952 pose
        154 target: x 0..2.0, y −0.6..0.6 (langkah 0.2 m), z ∈ {1.05, 1.25}
        terjangkau ⟺ min‖P − T_a(g)⁻¹·t‖ < tol
```

Target ditransformasi, bukan cloud — satu query KD-tree per (target, pose).
Sapuan penuh 4 lengan × 154 target × 2952 pose = 1.8 juta query: **~10 detik**.

**Kendala grasp.** Peta lama posisi-saja. Di sini ditambahkan batas approach:
sumbu +z tool harus dalam 45° dari tegak-lurus-bawah (`grasp_tilt_max = 45.0`
di `gantry_reach_executor.py`). Filter ini **eksak dan tidak bergantung pose**:
karena `R_base = Rz(θ)·Rx(π)`, syarat "approach ke bawah di dunia" identik dengan
"sumbu tool ≈ +z di frame base" — `Rz(θ)` tidak bisa memiringkan vektor dari
sumbu z. Terverifikasi numerik (beda sudut 2.5e-05°).

Cek kepadatan sampling: jarak tetangga-terdekat median 0.47 cm (p95 0.97 cm),
jauh di bawah toleransi 5 cm.

---

## 3. X0 — derajat koordinasi vs sebaran tugas

tol 0.05 m, approach ≤45°, 8000 set-tugas acak per baris. Sebaran = keempat
target diambil dalam radius `r` dari satu titik pusat acak.

| Sebaran | 1 lengan | 2 | 3 | 4 | rata-rata | fixed 4-lengan | sadar-kopling 4-lengan | gain |
|---|---|---|---|---|---|---|---|---|
| r ≤ 0.30 m | 0.00% | 0.00% | 62.82% | **37.18%** | 3.37 | 18.09% | 37.18% | 2.1× |
| r ≤ 0.50 m | 0.00% | 0.00% | 49.23% | **50.78%** | 3.51 | 22.70% | 50.78% | 2.2× |
| r ≤ 0.80 m | 0.00% | 0.00% | 23.36% | **76.64%** | 3.77 | 35.73% | 76.64% | 2.1× |
| r ≤ 1.20 m | 0.00% | 0.01% | 9.56% | **90.42%** | 3.90 | 42.70% | 90.42% | 2.1× |
| r ≤ 1.80 m | 0.00% | 0.00% | 11.09% | **88.91%** | 3.89 | 38.45% | 88.91% | 2.3× |
| uniform | 0.00% | 0.01% | 11.38% | **88.61%** | 3.89 | 36.49% | 88.61% | 2.4× |

**Kurvanya nyata: derajat koordinasi naik tajam seiring sebaran tugas, lalu
jenuh.** Monoton dari r ≤ 0.3 m sampai r ≤ 1.2 m (37% → 90%), setelah itu datar
di 88–90% (r ≤ 1.8 m dan uniform sedikit di bawah puncak — selisih ~1.5 poin,
masih dalam derau 8000 sampel). Jadi rentang dinamis yang bisa diklaim adalah
**37% → 90%, jenuh di sekitar r ≈ 1.2 m**.

Tugas yang menggerombol **menurunkan** konkurensi 4 lengan. Mekanismenya
langsung dari geser-π: dua lengan se-gantry berjarak 0.8 m sementara jangkauan
horizontalnya pada ketinggian target hanya 0.44 m (z = 1.05) sampai 0.71 m
(z = 1.25). Irisan dua jangkauan itu tipis, jadi dua target berdekatan justru
sulit dilayani serentak dari satu pose gantry.

Ini X0 yang bisa dipakai — kurva, bukan satu angka. Dan arahnya berlawanan
dengan intuisi naif, jadi layak jadi hasil.

## 4. X1 — penugasan tetap vs sadar-kopling

**Terkunci: 36.5% → 88.6%, gain 2.4×** (uniform, tol 0.05 m, approach ≤45°).

Konvergen terhadap kepadatan cloud (tanpa kendala approach, tugas uniform):

| Sampel cloud | 4-lengan fixed | 4-lengan sadar-kopling | gain |
|---|---|---|---|
| 125.000 | 35.24% | 87.04% | 2.5× |
| 500.000 | 37.41% | 88.54% | 2.4× |
| 2.000.000 | 38.61% | 89.56% | 2.3× |

Naik 16× jumlah sampel menggeser headline 2.5 poin persen. **Angka ini stabil.**

Kendala approach 45° hampir tidak berbiaya (cakupan 23.11% → 22.64%): lengan
6-DOF + 2 DOF gantry cukup redundan sehingga orientasi nyaris gratis.

### Bonus untuk X5 — pose handover sudah terukur

Diagonal matriks ko-kelayakan justru persis syarat handover di §3 rencana
(`G_a(t) ∩ G_b(t) ≠ ∅`), jadi ikut terhitung gratis:

| Pasangan | Target yang punya pose handover |
|---|---|
| intra-gantry (arm1+arm2, arm3+arm4) | **42.9%** (46.2% dari target yang memang dijangkau keduanya) |
| inter-gantry (semua 4 kombinasi) | **85.7%** — pose gantry independen, jadi ini murni jangkauan |

Handover lintas-gantry hampir selalu tersedia secara kinematik; yang mengikat
adalah handover se-gantry, karena keduanya harus berbagi satu pose. Angka ini
belum memperhitungkan tabrakan lengan–lengan (lihat §8), yang justru paling
mengikat tepat pada konfigurasi handover.

---

## 5. Yang berubah dari §2 — dan mengapa

| Besaran | §2 (provisional) | G2 (terkunci) |
|---|---|---|
| `|G_a(t)|` median @ tol 0.15 | 6 pose | **954 dari 2952 pose** — 159× |
| `|G_a(t)|` median @ tol 0.20 | 14 pose | **1102 dari 2952 pose** — 79× |
| 4 lengan, penugasan tetap | 1.2% | **36.5%** |
| 4 lengan, sadar-kopling | 11.4% | **88.6%** |
| gain | 9.5× | **2.4×** |
| derajat rata-rata | 2.95 | **3.89** |

Penyebabnya tunggal dan sudah dipastikan: **kerapatan peta, bukan metrik.**
`arm1_model.npz` punya `task_dim = 3` — jadi peta lama juga posisi-saja, sama
seperti sapuan ini. Bedanya 3000 node menyebar di seluruh ruang 8-DOF vs 2 juta
sampel. Peta lama menemukan ~6 pose gantry per target, sapuan menemukan ~688:
selisih 100×, murni artefak sampling. Ini persis peringatan yang sudah ditulis
sendiri di §2.

### Akibatnya untuk paper — ini serius, karena §2b menaikkan taruhannya

§2b p1_plan.md menutup hampir seluruh novelty mesin reachability (Seraji 1995,
IEEE 7363551) dan menyimpulkan: *"Angka **1.2% → 11.4%** naik jadi hasil yang
memikul paper."* Angka itulah yang baru saja mengempis jadi **36.5% → 88.6%
(2.4×)**. Jadi hasil yang dijadikan tumpuan setelah penelusuran literatur
ternyata tidak sekuat yang dikira.

- ❌ **"Figure 1" versi 9.5× tidak selamat.** Gain sebenarnya 2.1–2.4×, dan stabil
  di semua sebaran tugas. Masih hasil yang sah dan konsisten, tapi bukan headline
  dramatis.
- ⚠️ **Klaim "derajat koordinasi adalah variabel" melemah kalau diukur dari
  jangkauan saja.** Derajatnya nyaris tidak pernah di bawah 3, dan 4 lengan
  tersedia 89% waktu. *Jangkauan bukan kendala yang mengikat.*
- ✅ **Tapi X0 menyelamatkannya lewat sebaran tugas**: 37% → 90% konkurensi 4
  lengan sepanjang sumbu sebaran. Variabel keputusannya nyata, hanya saja yang
  menggerakkannya adalah **geometri sebaran tugas**, bukan kelangkaan jangkauan.

Saran framing: pindahkan Figure 1 dari "fixed vs sadar-kopling" ke **"derajat
koordinasi vs sebaran tugas"** (tabel §3), dengan gain 2.4× sebagai hasil
pendukung. Cerita itu didukung angka dan tidak menuntut kelangkaan yang tidak
terbukti.

Kurva itu juga **lebih cocok dengan sumbu novelty §2b** daripada angka lama:
sumbu #1 adalah "simultanitas sebagai objektif", dan tabel §3 secara harfiah
adalah berapa lengan bekerja serentak sebagai fungsi geometri tugas. Sumbu #2
("base bergerak majemuk + penugasan lintas base") juga dapat isi baru dan
gratis: kopling intra-gantry adalah **geser-π yang eksak** (§1) sementara
inter-gantry terpisah penuh — itu struktur yang tidak dimiliki prior work
base-tunggal.

> 🔴 **Diperbarui setelah §10.** Interference sudah diukur dan **tidak mengikat**.
> Jadi klaim "derajat koordinasi adalah variabel keputusan kelayakan" **tidak
> pulih** — ia memang mati. Baik reach maupun interference permisif; 4 lengan
> tersedia ~87% waktu. Yang langka adalah **waktu dan lingkungan**, bukan
> geometri. Konsekuensinya paper harus diformulasikan ulang sebagai
> **allocation & scheduling** (XD [ST-SR-TA] dengan shared positioning resource
> dan sequence-dependent setup times), dengan derajat koordinasi sebagai
> **outcome yang dilaporkan**, bukan sebagai keputusan. Lihat §10.

---

## 6. Pertanyaan terbuka §8 nomor 2 — apakah `G_a(t)` terputus? **TIDAK.**

Komponen terhubung 8-tetangga pada grid `(linear, rotation)`, dihitung dua cara:
sebagai bidang, dan sebagai **silinder** (rotasi menyambung di ±π).

| tol | silinder: >1 komponen | rata-rata komponen | bidang: >1 komponen | selisih = jahitan saja |
|---|---|---|---|---|
| 0.05 m | **0.0%** | 1.00 | 36.4% | 36.4% |
| 0.10 m | **0.0%** | 1.00 | 37.8% | 37.8% |
| 0.15 m | **0.0%** | 1.00 | 30.5% | 30.5% |
| 0.20 m | **0.0%** | 1.00 | 27.6% | 27.6% |

**`G_a(t)` selalu terhubung sederhana.** Nol pengecualian dari 616 pasangan
(lengan, target), di semua toleransi, bahkan tanpa membuang komponen 1-sel.
Kolom terakhir persis sama dengan kolom "bidang": **setiap** kasus yang tampak
terputus disebabkan memotong jahitan ±π, tidak satu pun nyata.

Jadi angka 29–48% dari uji 1D sepanjang rel di §8 memang artefak — dua artefak
sekaligus (jahitan terpotong + lubang sampling). Logika penggabungan jahitan
(union-find, termasuk sentuhan diagonal-8) diuji terhadap 6 kasus sintetis;
semuanya lolos.

**Konsekuensi — ini negatif dan harus dipakai apa adanya:** representasi
topologis **tidak diperlukan** oleh struktur `G_a(t)`. Ia diwarisi dari
instrumentasi, bukan dituntut oleh masalahnya. Ini memperkuat larangan yang
sudah ada di §2 ("jangan klaim topologi C-space") dan menutup satu jalur klaim.
Lebih baik ditemukan sekarang daripada oleh reviewer.

## 7. Pertanyaan terbuka §8 nomor 1 — apakah penghalang teramplifikasi kopling? **Sebagian.**

Penghalang seukuran orang: silinder tegak r = 0.25 m (+0.05 m radius lengan),
tinggi 1.8 m, 12 posisi acak di area kerja. Lengan diwakili polyline
base→siku→pergelangan→tool. Grid gantry 21 × 36 = 756 pose.

### Baseline nol yang wajib dipakai

Ini yang membuat angka bisa ditafsirkan. Ko-kelayakan adalah **produk dua
himpunan**. Kalau penghalang membuang konfigurasi tiap lengan secara
**independen** dengan laju `s`, himpunan pasangan menyusut
`1 − (1−s)² ≈ 2s` **dengan sendirinya**. Jadi amplifikasi ≈ 2× adalah
**baseline nol, bukan bukti kopling**. Kopling baru dikatakan mengamplifikasi
kalau angkanya melewati baseline itu.

### Hasil (12 penghalang, tol 0.05 m, approach ≤45°)

| Tingkat | Rugi 1 lengan | Rugi terkopling | Amplifikasi | Baseline nol | Vonis |
|---|---|---|---|---|---|
| **Konfigurasi** (penyusutan `G_a(t)` vs penyusutan irisannya) | 16.37% (maks 23.81%) | 28.76% (maks 38.28%) | **1.79×** | 1.84× | **TIDAK teramplifikasi** |
| **Eksistensi** (target hilang vs pasangan-target hilang) | 7.55% (maks 9.09%) | 17.23% (maks 20.56%) | **2.29×** | 1.92× | **TERAMPLIFIKASI 1.19×** |

**Jawabannya bergantung pada apa yang dihitung, dan keduanya perlu dilaporkan:**

- Di tingkat **konfigurasi**, tidak ada amplifikasi — 1.79× justru sedikit di
  *bawah* 1.84×. Artinya kedua lengan se-gantry kehilangan **pose yang sama**
  (rugi berkorelasi positif), masuk akal karena satu penghalang menghalangi
  kedua lengan pada pose gantry yang berdekatan.
- Di tingkat **eksistensi**, ada amplifikasi nyata: 2.29× vs baseline 1.92×.
  Pasangan target yang irisannya sudah tipis **tumbang seluruhnya**, bukan
  sekadar menyusut. Ini persis mekanisme yang dihipotesiskan di §8.

Jadi hipotesis §8 **benar untuk kelayakan biner (yang dipakai keputusan), salah
untuk volume konfigurasi**. Yang bisa diklaim di paper: penghalang seukuran
orang menghapus 7.6% target dari satu lengan tapi **17.2% pasangan target** dari
lengan se-gantry — lipat 2.3×, dan 1.19× di atas yang bisa dijelaskan kebetulan.

Catatan: rugi 1 lengan tingkat-eksistensi **7.55%** hampir persis angka
provisional §2 (rata-rata 7.5%) — satu-satunya angka §2 yang selamat.

### Canonical vs any-of-K — harga berkomitmen pada satu konfigurasi

Angka di tabel atas memakai **any-of-K**: pose selamat kalau *ada* satu dari K
kandidat yang bebas tabrakan. Itu **batas atas yang tidak realisable online** —
ia mengandaikan oracle yang tahu kandidat mana yang selamat. Versi yang benar-benar
bisa dijalankan memakai **canonical configuration** (§10). Keduanya diukur atas 12
penghalang yang sama:

| | any-of-K (K=256) | canonical (`manip`) | selisih |
|---|---|---|---|
| config, rugi 1 lengan | 16.37% | **18.98%** | +2.6 poin |
| config, rugi terkopling | 28.76% | **32.88%** | +4.1 poin |
| config, amplifikasi | 1.79× (null 1.84) | **1.76×** (null 1.81) | vonis sama |
| exist, rugi 1 lengan | 7.55% | **8.30%** | +0.75 poin |
| exist, rugi terkopling | 17.23% | **19.56%** | +2.3 poin |
| exist, amplifikasi | 2.29× (null 1.92) | **2.36×** (null 1.92) | vonis sama |

Jadi harga berkomitmen pada satu konfigurasi ≈ **0.75 poin** kapabilitas 1 lengan
dan **2.3 poin** ko-kelayakan pasangan. Kecil, dan **vonis kedua tingkat tidak
berubah**. Angka canonical yang dipakai untuk paper — itu yang bisa dijalankan.

### Sensitivitas terhadap policy canonical

| Policy | config: 1 lengan / terkopling / amp | exist: 1 lengan / terkopling / amp |
|---|---|---|
| `manip` Yoshikawa | 18.98% / 32.88% / 1.76× | 8.30% / 19.56% / 2.36× |
| `sigmin` | 19.01% / 32.92% / 1.76× | 8.19% / 19.53% / 2.39× |
| `limits` | 19.09% / 33.05% / 1.76× | 8.07% / 19.46% / 2.42× |
| `home` | 19.02% / 32.89% / 1.76× | 8.25% / 19.55% / 2.38× |
| `combo` | 19.04% / 32.96% / 1.76× | 8.16% / 19.39% / 2.38× |

Rentang total: config ±0.11 poin, exist ±0.23 poin, amplifikasi ±0.06×. **Hasilnya
invariant terhadap pilihan policy** — termasuk terhadap kritik bahwa Yoshikawa
(1985) buta joint limit: `limits` dan `combo` yang justru memperhitungkannya
memberi angka yang sama. Yoshikawa dipakai sebagai default karena ia baseline
standar yang sebanding dengan literatur, bukan karena ia yang terbaik.

Catatan satuan: `manip` di sini dihitung dari **baris translasi saja** (`J[:3]`),
jadi tidak terkena kritik klasik inkonsistensi dimensi pada bentuk 6-baris.

### Kekokohan

Kandidat per query dibatasi `K` demi vektorisasi. Dijalankan pada `K = 64`
(saturasi 17.4%) dan `K = 256` (saturasi 8.1%) atas 12 penghalang yang sama.
Angka di tabel adalah `K = 256` (lebih konvergen). Selisih amplifikasi
per-penghalang antara keduanya maks **0.03×** (konfigurasi) dan **0.15×**
(eksistensi); rata-ratanya 1.79 vs 1.79 dan 2.32 vs 2.29. Vonis kedua tingkat
tidak berubah, jadi pemotongan kandidat tidak menggerakkan kesimpulan.

Proksi polyline **meremehkan** volume sapuan lengan (siku menonjol keluar tali
busur), jadi semua angka rugi di atas adalah **batas bawah**.

---

## 8. Yang MASIH belum dimodelkan — X0/X1 di atas adalah batas atas

> ✅ **Poin 1 dan 2 sudah diukur 2026-08-12 — lihat §10. Keduanya TIDAK mengikat**
> (intra-gantry 0.00% pasangan hilang, inter-gantry 4.19% di level konfigurasi
> dengan pose independen). Jadi X0/X1 di §3–4 ternyata **bukan** batas atas yang
> longgar — hampir sama dengan angka sebenarnya. Sisa poin di bawah tetap berlaku.

Sengaja disebut supaya tidak lolos diam-diam:

1. ~~**Interferensi lengan–lengan tidak dicek.**~~ **Sudah diukur (§10).**
   Dugaan waktu itu — *"sebagian dari 88.6% itu pasti gugur karena tabrakan"* —
   **salah**: 0.00% pasangan hilang. Justru karena kedua lengan diametral 0.8 m
   dan menghadap keluar, mereka jarang bertemu.
2. **Interferensi gantry–gantry tidak dicek.** Rel berjarak 0.72 m, lengan
   mengayun ±0.4 m dari sumbunya masing-masing. Pada rotasi ~90° kedua gantry
   bisa menaruh lengan pada `y ≈ ∓0.04` — 8 cm terpisah. Nyata dan mengikat.
3. **Faktorisasi per gantry dipakai sebagai asumsi, bukan hasil.** Karena (2)
   belum dicek, "inter-gantry inert" belum diverifikasi ulang di sapuan ini.
4. Toleransi posisi 5 cm tanpa cek IK penuh; tabrakan lengan-dengan-lingkungan
   statis tidak dimodelkan.

~~Poin 1 dan 2 kemungkinan besar adalah kendala yang sebenarnya mengikat.~~
**Prediksi itu salah.** Sudah diukur di §10: interference tidak mengikat juga.
Baik jangkauan maupun geometri lengan **tidak** membatasi konkurensi 4 lengan.
Satu-satunya yang menggigit adalah **lingkungan dinamis** (§7, 17.2% pasangan
hilang) — dua orde lebih besar. Itu yang menentukan arah paper.

---

## 9. Cara mengulang

```bash
python3 -m reachability_gng.irm_sweep verify
python3 -m reachability_gng.irm_sweep cloud --n 2000000 --out /tmp/irm_cloud.npz
python3 -m reachability_gng.irm_sweep sweep --cloud /tmp/irm_cloud.npz \
    --approach 45 --out /tmp/irm_sweep.npz
python3 -m reachability_gng.irm_sweep analyze --sweep /tmp/irm_sweep.npz --tol 0.05
python3 -m reachability_gng.irm_sweep obstacle --cloud /tmp/irm_cloud.npz \
    --approach 45 --k-cand 256
```

`cloud` ~29 s, `sweep` ~10 s, `analyze` ~3 s, `obstacle` ~20 menit.

---

## 10. Arm–arm interference — gerbang go/no-go. **TIDAK mengikat.**

Diukur 2026-08-12 setelah keputusan pivot ke allocation & scheduling.
`python3 -m reachability_gng.irm_sweep interfere`

### Canonical redundancy resolution (koreksi metodologis)

Collision-freeness **bukan** properti gantry pose saja. Lengan redundant: banyak
`q` menjangkau `t` dari `g`, obstacle/partner bisa memblokir sebagian saja. Jadi
`G_a(t|O) = reach_mask & free_mask` **tidak sound** kecuali kita menetapkan
**satu konfigurasi kanonik** per `(target, gantry pose)`.

Di sini kanoniknya = **manipulability Yoshikawa tertinggi** di antara sampel yang
menjangkau. Setelah itu kapabilitas jadi fungsi well-defined atas `(t,g)`, dan
filter tabrakan jadi eksak **untuk policy itu**. Harganya konservatif, dan itu
harus dilaporkan sebagai pilihan model, bukan disembunyikan.

Lengan diwakili polyline base→siku→pergelangan→tool; jarak dihitung dengan
segment–segment eksak (Ericson, diverifikasi terhadap brute force sampai 2e-05 m).

### Hasil

| Pasangan | Level konfigurasi | Level eksistensi |
|---|---|---|
| **intra-gantry** (arm1+arm2, arm3+arm4) | **1.27%** dari `(i,j,pose)` yang sama-sama menjangkau, bertabrakan | **0.00% pasangan hilang** |
| **inter-gantry** (arm1+arm3, 398k sampel) | **4.19%** dari `(t_i,g1,t_j,g2)` bertabrakan | pose independen → jauh lebih kecil lagi |

Sensitivitas terhadap clearance (intra-gantry):

| Clearance | Konfigurasi bertabrakan | Pasangan hilang |
|---|---|---|
| 0.10 m | 1.3% | **0.00%** |
| 0.15 m | 2.6% | **0.00%** |
| 0.20 m | 5.1% | 0.62% |
| 0.30 m | 12.2% | 2.86% |

Jarak antar-lengan median **0.574 m**; minimum yang pernah terjadi 0.4 mm, tapi
p1 sudah 0.083 m. Tabrakan memang terjadi, tapi jarang — dan karena satu pasangan
target biasanya punya ratusan pose bersama, **tidak ada satu pun pasangan yang
hilang seluruhnya**.

Dampak ke X0/X1 praktis nol: konkurensi 4 lengan uniform **88.6% → 87.5%**,
derajat rata-rata 3.89 → 3.87.

### Kenapa begitu — dan ini konsekuensi langsung geser-π

Dua lengan se-gantry duduk **diametral, 0.8 m terpisah**, masing-masing menghadap
keluar. Justru geometri yang membuat mereka sulit melayani target berdekatan
(§3) adalah yang membuat mereka **jarang saling tabrak**. Kedua efek itu sisi
yang sama dari satu koin.

### Vonis gerbang

**Reach tidak mengikat (4 lengan tersedia 88%). Interference tidak mengikat
(0% pasangan hilang).** Dua-duanya sudah diukur, dua-duanya negatif.

Yang tersisa sebagai satu-satunya kendala yang benar-benar menggigit adalah
**lingkungan dinamis**: satu penghalang seukuran orang menghapus **17.2%
pasangan target** (§7) — dua orde lebih besar dari interference.

Jadi geometri statis sel ini **permisif**; yang langka adalah **waktu dan
lingkungan**. Itu justru membenarkan pivot ke allocation & scheduling, dan
menaikkan sumbu novelty #3 (kapabilitas bergantung lingkungan, diperbarui
online) jadi yang memikul paper — bukan #1 atau #2.

### Yang masih belum dimodelkan di sini

- **Struktur gantry** (platform + mount plate) tidak dimodelkan — hanya lengannya.
  Kedua pelat menyapu lingkaran radius 0.4 m di `y = ±0.36`, jadi irisannya di
  `y ∈ [−0.04, 0.04]`: tabrakan struktur **mungkin** terjadi dan itu batasan atas
  `(lin₁,rot₁,lin₂,rot₂)` yang belum dicek.
- Proksi polyline meremehkan volume sapuan (siku menonjol) → semua angka rugi
  adalah **batas bawah**.
- Kanonik = max manipulability; policy lain bisa memberi angka berbeda, walau
  dengan rugi sekecil ini kecil kemungkinannya berubah kualitatif.
- Inter-gantry hanya disampel, dan hanya `arm1+arm3` sebagai wakil.

---

## 11. Policy zona-eksklusi Meso vs tabrakan eksak — **ini yang membenarkan scheduling**

Mengikuti `GNG_HSR_Topological_Main` ([Meso-HSR/GNG.h:1223](../Meso-HSR/GNG.h)):
setiap node peta `m` yang berjarak `< r` dari node peta `n` dimatikan dan
edge-nya diputus. Dibaca sebagai aturan operasi: **hanya satu lengan boleh
bekerja di zona irisan.**

Operasionalisasinya gratis: *"t berada dalam radius `r` dari jangkauan lengan
lain"* identik dengan **mask pada toleransi `r`** — jadi seluruh sapuan `r`
keluar dari satu file sweep.

`python3 -m reachability_gng.irm_sweep policy`

| Policy | Pasangan co-feasible | Konkurensi 4 lengan | Derajat rata-rata |
|---|---|---|---|
| reach saja (batas atas) | 59.49% | **87.55%** | 3.88 |
| tabrakan eksak, canonical (§10) | 59.49% | **87.50%** | 3.87 |
| zona r=0.10, satu lengan kehilangan | 56.79% | 87.55% | 3.88 |
| zona r=0.20, satu lengan kehilangan | 52.98% | 84.69% | 3.85 |
| zona r=0.10, **kedua** lengan kehilangan | 51.91% | 80.00% | 3.80 |
| **zona r=0.20, kedua lengan kehilangan** | 40.45% | **61.84%** | 3.61 |
| zona r=0.30, kedua lengan kehilangan | 27.13% | **34.61%** | 3.30 |

`r = 0.20 m` adalah nilai `Dangerous Area` di kode sensei.

### Temuan utama: partisi statis membunuh handover, dan itu mahal

Kalau irisan **dihapus permanen dari kedua lengan** — partisi ruang statis,
aman tanpa koordinasi runtime apa pun — konkurensi 4 lengan jatuh dari
**87.5% ke 61.8%** pada `r = 0.20`, dan ke **34.6%** pada `r = 0.30`.

Dan ada biaya kedua yang lebih tajam: **handover jadi mustahil.** Handover
menuntut kedua lengan menjangkau titik yang *sama* — titik itu menurut definisi
ada di irisan. Menghapus irisan berarti menghapus 42.9% target yang punya pose
handover intra-gantry. Policy paling aman justru membunuh kapabilitas yang
paling dibutuhkan.

> Jadi irisan **tidak boleh dipartisi secara statis — ia harus dibagi dalam
> WAKTU.** Itu definisi resource ber-mutex, dan itulah yang membuat masalahnya
> jadi scheduling, bukan seleksi kelayakan.

**Selisih 61.8% → 87.5% (25 poin) adalah nilai yang dibeli oleh scheduling**,
plus handover yang tidak bisa dinilai dengan angka konkurensi saja. Ini argumen
kuantitatif inti untuk formulasi `XD [ST-MR-TA]`, dan ia tidak ada sebelum
mekanisme Meso dimasukkan.

### Kenapa pembacaan longgar tidak mengikat

Varian "satu lengan kehilangan irisan" (pembacaan harfiah signature
`(m, n, l, g)` — hanya peta `m` yang rugi) hampir tidak berpengaruh: 84.7% pada
`r = 0.20`. Sebabnya kuantor **ada pose**: satu pasangan target punya ratusan
pose bersama, jadi hampir selalu ada satu pose di mana lengan yang dirugikan
tetap bisa bekerja di luar zona. Efek mengikat baru muncul kalau eksklusi
diterapkan **dua arah**.

Itu juga menjelaskan kenapa tabrakan eksak (§10) nyaris tidak berbiaya: ia
memfilter *konfigurasi*, bukan *region*, dan kuantor ada-pose menyerap semuanya.

### Batasan

Aturan sensei berbasis **titik kerja node**, bukan badan lengan — lengan bisa
saja menyapu masuk zona untuk meraih target di luar zona. Jadi angka
zona-eksklusi di atas dan angka tabrakan eksak di §10 mengukur dua hal berbeda,
dan keduanya perlu dilaporkan: yang satu batas atas koordinasi sempurna, yang
lain batas bawah tanpa koordinasi sama sekali.

---

## 12. Lapis 2 dibangun — dan index topologis KALAH telak

`capability.py`. Index atas task space; tiap node membawa dua payload:
`mask G_a(t)` atas grid pose gantry, dan `canonical q`. Satu peta **per gantry**
(bukan per lengan) — pasangannya diturunkan dengan roll setengah putaran.

### Uji index: topologis vs grid, anggaran node sama

**Catatan keadilan.** Grid tidak bisa menghasilkan jumlah sel persis sesuai
permintaan (harus hasil kali dimensi), jadi realisasinya 960 / 3132 / 7872 vs
topo 1000 / 3000 / 8000 — meleset −4.0% / +4.4% / −1.6%, **arahnya tidak
konsisten** (di 1000 grid justru lebih sedikit, dan di sana topo menang tipis).
Skalanya juga tidak relevan: menaikkan sel 2.7× (3000→8000) hanya menggeser grid
1.9 poin, jadi selisih 4% bernilai ~0.1 poin — dibanding jurang 18 poin.

Grid bahkan **dirugikan** oleh caranya dibangun: ia membentang seluruh bbox
termasuk `z` yang kosong, sehingga **46.8% selnya berjarak >10 cm dari data
sampel mana pun** — terbuang percuma. Ia menang meski begitu.

Ground truth = mask dihitung langsung di titik query. Metrik = IoU atas himpunan
pose gantry. Distribusi target: `uniform` dan `surface` (objek di atas beberapa
bidang, mengelompok — seperti sel nyata).

| Distribusi | Sel | grid IoU | topo IoU | Menang |
|---|---|---|---|---|
| uniform | 1000 | 85.9% | 86.4% | topo |
| uniform | 3000 | 90.3% | 87.7% | grid |
| uniform | 8000 | 92.5% | 88.3% | grid |
| surface | 1000 | 80.9% | 71.7% | grid |
| **surface** | **3000** | **91.4%** | **72.9%** | **grid** |
| surface | 8000 | 93.3% | 73.1% | grid |

Topo **jenuh** — 71.7 → 72.9 → 73.1 saat node dinaikkan 8×. Bukan under-fitting:
melatih 6.5× lebih lama hanya sampai 75.7%, dan `seed_boundary` menaikkan mean
ke 77.1% tapi p5 jatuh ke 0.4% (node shell berada di luar region terjangkau).

### Dan selisih itu berdampak ke keputusan

| Sumber mask | Pasangan co-feasible | Keputusan salah | Handover |
|---|---|---|---|
| ground truth | 68.53% | — | 53.1% |
| grid index | 69.99% | **2.65%** | 54.6% |
| topo index | 78.60% | **11.18%** | 62.3% |

Index topologis melebihkan ko-kelayakan **10 poin** dan salah pada **11%**
keputusan pasangan — 4× error grid. Errornya **bertanda**: topo *optimis*, jadi
scheduler yang dibangun di atasnya akan menjadwalkan konkurensi yang gagal saat
dieksekusi.

### Kenapa — dan ini alasan yang bisa dipakai di paper

GNG menempatkan node menurut **kerapatan data**. Error mask ditentukan oleh
**seberapa cepat `G_a(t)` berubah di ruang**, dan gradien itu berasal dari
kinematika lengan, **bukan** dari di mana objek kebetulan berada. Kedua hal itu
tidak berkorelasi.

Akibatnya GNG menumpuk node di dalam gumpalan padat — tempat mask-nya justru
nyaris konstan — dan menelantarkan celah antar-gumpalan, tempat sebuah query
bisa mendarat jauh dari node mana pun. **Adaptivitas terhadap kerapatan adalah
adaptivitas yang salah untuk pekerjaan ini.** Untuk mengalahkan grid, index
harus beradaptasi terhadap *gradien mask* — dan itu bukan yang dilakukan GNG
maupun GCS.

### Yang TIDAK dibatalkan temuan ini

Ini hanya menyelesaikan soal **index**. Topologi tetap benar di tempat lain, dan
tidak tersentuh:

| Komponen | Representasi | Alasan |
|---|---|---|
| Peta lingkungan `O(t)` | **GNG (MS-BL)** | itu yang dihasilkan persepsi; sudah jalan di `env_gng.py` |
| Action map `xyz → q` | **GCS** | peran sensei di FRD-01, tidak diuji di sini |
| Operasi himpunan Meso | **graf ↔ graf** | operand-nya graf, hasilnya graf dengan edge terpotong |
| **Index capability** | **grid** | terukur, selisihnya besar |

Kalau index topologis tetap dipakai demi konsistensi, harganya **11% keputusan
salah dengan bias optimis**. Itu boleh diambil sebagai pilihan konsistensi —
tapi harus disebut konsistensi, bukan performa.

### Operasi Meso arm↔arm sudah jalan

`capability.py overlap`, gantry 1, index grid 3132 node:

| `r` | Node di zona | (node,pose) di zona | Edge terputus | Peran |
|---|---|---|---|---|
| 0.05 | 48.5% | 32.3% | **52.6%** | himpunan HANDOVER (irisan ketat) |
| 0.10 | 55.4% | 39.1% | 59.0% | zona bahaya (dilasi) |
| 0.20 | 65.4% | 52.3% | **67.7%** | zona bahaya (dilasi) |

Kolom "edge terputus" itu angka yang menjelaskan §11: menghapus zona pada
`r = 0.20` memutus **67.7% edge** — graf kerjanya hancur. Itu versi struktural
dari temuan bahwa partisi statis menjatuhkan konkurensi ke 61.8%.

---

## 13. Verifikasi silang — apa yang sudah dicek, dan bagaimana

Dijalankan di akhir sesi 2026-08-12, sebelum menyerahkan ke tahap implementasi.
Semua lolos **eksak** (bukan "dalam toleransi").

| # | Klaim yang diuji | Cara | Hasil |
|---|---|---|---|
| 1 | `mask(arm2) == roll(mask(arm1), π)` | 300 titik acak, roll vs hitung langsung | **identik, 0.0000% sel beda** |
| 2 | `mask(arm3) == mask(arm1)` dgn target `y+0.72` | idem | **identik** |
| 3 | `capability.reach()` == perhitungan langsung | 200 node, arm1 dan arm2 | **identik** |
| 4 | `irm_sweep` vs `capability` untuk target sama | 154 target grid | **identik, bit-for-bit** |
| 5 | segment–segment distance | 3000 pasang acak vs brute force + 6 kasus tepi | **2e-05 m**, semua kasus tepi lolos |
| 6 | penggabungan jahitan silinder (union-find) | 6 kasus sintetis, termasuk sentuhan diagonal-8 | semua lolos |
| 7 | filter approach tidak bergantung pose | 3 pose gantry berbeda | **2.5e-05°** |
| 8 | `T_a(g)` rigid vs pinocchio | 4 lengan × 3 uji | **1.1e-16** (sisa 2.65e-06 = literal `3.14159` URDF) |

Verifikasi #4 penting: dua modul menghitung mask lewat jalur kode berbeda dan
menghasilkan bit yang sama, jadi angka §1–§11 (`irm_sweep`) dan §12
(`capability`) berdiri di atas dasar yang sama.

### Kesalahan yang ditemukan audit ini dan sudah diperbaiki

1. **§8 poin 1** masih memuat dugaan lama *"sebagian dari 88.6% pasti gugur
   karena tabrakan"* — dugaan itu **salah** (§10: 0.00%). Sudah dicoret, tidak
   dihapus, supaya jejaknya terlihat.
2. **"jumlah node sama = memori sama"** terlalu tegas — grid meleset
   −4.0% / +4.4% / −1.6%. Sudah diberi catatan keadilan beserta besaran
   pengaruhnya (~0.1 poin, vs jurang 18 poin).
3. **Handover 42.9% dan 53.1% sempat ditulis sebagai rentang "42.9–53.1%"** di
   dokumen handoff. Itu **dua himpunan target berbeda**, bukan rentang. Sudah
   dipisah dan diberi label.

### Yang masih belum diverifikasi

- Tabrakan **struktur gantry–gantry** (pelat mount), bukan hanya lengannya.
- Proksi polyline vs mesh tabrakan sebenarnya — semua angka rugi adalah **batas bawah**.
- Aturan zona Meso berbasis **titik kerja node**, bukan badan lengan; lengan bisa
  menyapu masuk zona untuk meraih target di luar zona.
- Seluruh angka bersifat **kinematik**: tanpa dinamika, payload, dan waktu eksekusi.

---

## 14. Ukuran instance + torsi berbeban — dan pembalikan yang ditimbulkannya

### 14a. Coverage: berapa pose gantry yang sebenarnya dibutuhkan?

Menguji formulasi "pose sebagai stasiun" (set-cover + sequencing) sebelum scheduler
dibangun. Tiap pose menutupi tugas yang terjangkau salah satu dari dua lengan gantry itu;
greedy set cover; 25 ulangan per baris.

| N tugas | Sebaran | Cakupan 2 gantry | Pose g1 | Pose g2 | **Total** | 1 pose terbaik |
|---|---|---|---|---|---|---|
| 5 | surface | 100% | 1.44 | 0.00 | **1.44** | 90.4% |
| 20 | surface | 100% | 2.40 | 0.20 | **2.60** | 82.4% |
| 80 | surface | 100% | 3.48 | 0.64 | **4.12** | 75.7% |
| 20 | r ≤ 0.3 m | 100% | 1.04 | 0.20 | **1.24** | 98.6% |
| 80 | r ≤ 0.3 m | 100% | 1.40 | 0.32 | **1.72** | 94.3% |
| 20 | r ≤ 1.2 m | 100% | 2.80 | 0.76 | **3.56** | 73.6% |
| 80 | r ≤ 1.2 m | 100% | 4.28 | 1.44 | **5.72** | 61.6% |

**Satu pose menutupi 62–99% tugas. Total 1.0–5.7 pose, bahkan untuk 80 tugas.**

Konsekuensi: set-cover-nya **terlalu kecil untuk sulit**. 5–6 stasiun bisa dienumerasi
tuntas, jadi tidak ada optimality gap yang bisa dilaporkan. Formulasi "pose sebagai
stasiun" tidak menghasilkan masalah kombinatorial yang layak.

### 14b. Torsi gravitasi dengan payload

Semua pengukuran sebelumnya **kinematik murni**. Ini satu-satunya kendala fisik yang
belum disentuh. Gen3 Lite, batas efektuator `[10, 14, 10, 7, 7, 7]` N·m, rantai lengan
5.187 kg, 40k konfigurasi acak.

**Torsi gravitasi tidak bergantung pose gantry** — rel hanya bertranslasi dan rotasinya
hanya terhadap sumbu **vertikal**, jadi torsi gravitasi invarian terhadap `g`. Filter
torsi karenanya statis atas cloud, persis seperti filter approach.

| Payload | median | p95 | maks | Melampaui batas |
|---|---|---|---|---|
| 0.00 kg | 44.2% | 69.4% | 73.9% | **0.00%** |
| 0.25 kg | 48.3% | 78.9% | 85.2% | **0.00%** |
| **0.50 kg** (rating) | 52.8% | **89.0%** | **98.5%** | **0.00%** |
| 0.75 kg | 57.8% | 99.7% | 111.8% | 4.78% |
| 1.00 kg | 64.7% | 110.6% | 125.0% | 12.39% |

**Pada payload nominal, nol konfigurasi melampaui batas** — tapi marginnya tipis
(p95 89%, maks 98.5%). Dinamika (akselerasi menambah torsi di atas gravitasi statik),
gesekan, dan derating keselamatan bisa membuatnya menggigit di praktik. **Statik: tidak
mengikat. Dinamik: belum diukur.**

> ⚠️ Percobaan pertama memakai `J^T·(0,0,−g)` dan menghasilkan torsi yang **turun**
> saat payload naik — tanda terbalik. Ketahuan karena hasilnya mustahil secara fisik.
> Angka di atas dihitung dengan menambahkan massa titik ke model (`appendBodyToJoint`)
> lalu `computeGeneralizedGravity`, plus uji sanity lengan terentang (10.0 → 13.7 N·m).

### 14c. Rekapitulasi: lima kendala diukur, lima tidak mengikat

| # | Kendala | Mengikat? | Angka |
|---|---|---|---|
| 1 | Jangkauan (§3, §4) | ❌ | 4 lengan tersedia 87.5% |
| 2 | Interference lengan–lengan (§10) | ❌ | 0.00% pasangan hilang |
| 3 | Zona eksklusi, pembacaan harfiah (§11) | ❌ | 84.7% pada r=0.20 |
| 4 | Coverage / jumlah pose (§14a) | ❌ | 1–6 pose cukup |
| 5 | Torsi pada payload nominal (§14b) | ❌ | 0.00% melampaui |
| — | **Lingkungan dinamis (§7)** | ✅ | **19.6% pasangan hilang** |

**Sel ini kelewat longgar secara geometri dan mekanik untuk kelas tugas yang
diasumsikan.** Pengukuran kelayakan keenam tidak akan mengubah gambaran ini.

### 14d. Pembalikannya: kendalanya WAKTU, bukan kelayakan

Gabungkan §14a dengan parameter yang **masih ambigu 33×** (p1_state.md §7.3a):

| Skenario | Traverse rel | 5 perpindahan | vs 80 tugas × 5 s |
|---|---|---|---|
| `bridge.linear_speed` 31.4 mm/s | 63.7 s | **319 s** | mendominasi (400 s kerja) |
| `motor_config.speed` 1047 mm/s | 1.9 s | 10 s | dapat diabaikan |

Jadi apakah ada masalah keputusan di sel ini **sepenuhnya bergantung pada satu parameter
yang belum diukur**. Kelayakan sudah dijawab tuntas — waktu belum disentuh.

### 14e. Cara melaporkannya yang kokoh terhadap nilai apa pun

Jangan berharap gantry lambat. Yang harus dilaporkan adalah **titik-silangnya**:

> Penjadwalan sadar-kopling memberi keuntungan >X% hanya bila waktu traverse melampaui
> **T** detik; di bawah itu semua jadwal setara.

Itu panduan desain untuk siapa pun yang membangun sel semacam ini, dan ia tetap sahih
berapa pun kecepatan aslinya. Pengukuran stopwatch nanti tinggal menentukan **di mana
sel ini berada pada kurva**, bukan hidup-matinya paper.

Catatan: kecepatan konservatif mungkin punya alasan fisik — gantry mengangkut dua lengan
(~5.2 kg masing-masing) di ketinggian 2 m, rotasi dengan offset 0.4 m menimbulkan beban
sentrifugal, dan massanya bergerak di atas ruang kerja manusia. **Jangan mempercepat
gantry demi membuat paper menarik.**

---

## 15. Batas kecepatan gantry — mekanik terhitung, keselamatan normatif

Dihitung dari model massa/inersia URDF dengan RNEA pinocchio atas model 8-DOF
(gantry + lengan). **Yang di bawah ini masukan fisis untuk risk assessment, bukan
batas tersertifikasi.**

### 15a. Beban mekanik

| Besaran | Nilai |
|---|---|
| Massa bergerak gantry 1 (platform + rotasi + 2 pelat + 2 lengan + gripper) | **18.77 kg** |
| + 2 payload 0.5 kg | 19.77 kg |
| Inersia efektif linier | 18.8 kg |
| Inersia efektif rotasi (sumbu vertikal) | **2.84 kg·m²** |
| Gaya rel yang dibutuhkan | **18.8 N per m/s²** (3 m/s² → 56 N) |
| Torsi rotasi yang dibutuhkan | **2.98 N·m per 60 °/s²** (300 °/s² → 15 N·m) |

Bebannya kecil. Apakah motor sanggup **belum bisa dipastikan** — butuh kurva
torsi-kecepatan Oriental Motor yang tidak ada di repo.

### 15b. Aturan operasi: lipat lengan sebelum gantry bergerak

| Kondisi lengan | Utilisasi torsi statik | @ a = 10 m/s² |
|---|---|---|
| Terentang (terburuk) + payload 0.5 kg | **98.2%** | melampaui batas |
| **Terlipat (tuck)** | **31.5%** | **69.1%** |

Saat diam pun lengan terentang sudah memakai 98.2% batas efektuator hanya untuk
menahan bobotnya sendiri + payload. **Tanpa melipat, gantry praktis tidak boleh
berakselerasi sama sekali.** Dengan melipat, ≥10 m/s² masih aman.

> **Konsekuensi untuk scheduler — dan ini penting.** Biaya memindahkan gantry
> bukan hanya traverse, melainkan **lipat + traverse + rentang-ulang**, dan itu
> **menghentikan KEDUA lengan** pada gantry tersebut. Jadi setup cost-nya lebih
> besar dari yang diasumsikan, dan ia **terkopling** — persis mekanisme yang
> membuat penjadwalan di sel ini tidak terurai. Ini memperkuat sisi "memindahkan
> gantry itu mahal".

### 15c. Asimetri: linier aman, rotasi tidak

Ujung lengan terentang berjarak **1.4 m** dari sumbu rotasi (offset pelat 0.4 m +
jangkauan lengan hingga 1.0 m). Rotasi karena itu jauh lebih berbahaya daripada
angkanya terlihat:

| ω gantry | v ujung lengan @ 1.4 m |
|---|---|
| **10 °/s — setelan sekarang** | **244 mm/s** |
| 30 °/s | 733 mm/s |
| 60 °/s | 1.47 m/s |

Setelan `bridge.rotate_speed` sekarang menghasilkan **244 mm/s** di ujung lengan —
praktis tepat di figur *reduced speed* 250 mm/s (ISO 10218, mode manual/teach).
**Sudah di ambang; jangan dinaikkan tanpa analisis.**

Linier sebaliknya sangat longgar:

| v linier | E_kin (19.77 kg) | Jarak henti @ 2 m/s² |
|---|---|---|
| **31.4 mm/s — setelan sekarang** | **9.7 mJ** | 0.2 mm |
| 250 mm/s | 0.62 J | 16 mm |
| 500 mm/s | 2.47 J | 62 mm |

### 15d. Akibatnya untuk ambiguitas 33× waktu traverse

Rentang yang masuk akal **menyempit**, dan bukan ke ujung yang mana pun:

| Skenario | v linier | Traverse 2 m | Penilaian |
|---|---|---|---|
| Terkonfigurasi sekarang | 31.4 mm/s | 63.7 s | sangat konservatif; E_kin 9.7 mJ |
| Wajar setelah dinaikkan | ~250 mm/s | **~8 s** + akselerasi | masih konservatif |
| Rating motor | 1047 mm/s | 1.9 s | tidak perlu, dan meragukan untuk ruang berpenghuni |

Jadi rentang desain realistis **~8–64 s**, bukan 1.9–64 s. Sapuan sensitivitas
scheduler sebaiknya memakai rentang itu.

### 15e. Batas yang tidak bisa saya tetapkan

**Saya tidak dapat menetapkan batas keselamatan.** Dua hal yang harus dibawa ke
penilaian formal:

1. Dengan massa bergerak 19.77 kg, **power-and-force-limiting (PFL) praktis tidak
   tercapai** — massanya terlalu besar. Rute realistisnya *speed and separation
   monitoring* atau *safety-rated monitored stop*.
2. Figur 250 mm/s adalah *reduced speed* ISO 10218 untuk mode manual, **bukan batas
   universal**. Batas sebenarnya keluar dari ISO 10218-2 / TS 15066 sesuai mode
   kolaboratif yang dipilih, plus jarak pisah, waktu henti **terukur**, dan geometri
   ruang.

Untuk ruang sempit berpenghuni, yang biasanya diatur **bukan kecepatan** melainkan
**jarak pisah dan pemantauan** — di luar yang bisa dihitung dari model URDF.

**Belum dimodelkan:** dinamika lengan saat bergerak (angka torsi §14b/§15b statik +
inersia base, tanpa gerak sendi lengan sendiri), gesekan, derating termal motor,
kelenturan struktur rel/trailer, dan gaya cengkeram gripper (slip payload).
