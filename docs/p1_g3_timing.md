# P1 / G3 — Biaya waktu: traverse gantry + lipat/rentang lengan

> Sesi A, 2026-08-13. Tujuan tunggal: **mengubah biaya setup dari dugaan menjadi
> angka terukur**, lalu mengunci model biaya setup untuk scheduler (Lapis 3/4).
>
> §A ditulis **sebelum** data diambil (disiplin §8b). §B diisi sesudah.
> Kalau §B bertentangan dengan prediksi §A2, yang menang **§B**.

---

## A. Protokol — DIKUNCI SEBELUM AMBIL DATA

### A1. Temuan kode: ambiguitas 33× sebagian besar sudah runtuh

Ini hasil pembacaan jalur data, bukan pengukuran — tapi ia **fakta struktural**,
bukan intuisi geometris, jadi kelasnya berbeda dari lima dugaan yang meleset di §7.2.

**`motor_config.speed` bukan register kecepatan. Ia pembilang laju akselerasi.**

Jejaknya:

```
_configure_table_motors()                    dual_table_controller.py:291-312
  → configure_motor(m, acc=1000, speed=100000)      moving_table.py:28-42
      → writeParamAcc(time=acc, speed=speed)        oml_mrtu.py:415-432
          self.rateAcc = int(1000.0 * speed / time)   ← atribut Python lokal
          # TIDAK ADA modbusWriteReg di sini. Tidak ada yang dikirim ke driver.
```

`writeParamAcc` / `writeParamDec` / `writeParamCurrent` **tidak menyentuh bus sama
sekali** — ketiganya hanya menyetel atribut lokal. Nilai itu baru terkirim ketika
sebuah gerakan di-dispatch, sebagai field ke-5 dan ke-6 dari paket direct-data:

```python
directDriveData = [OpeDataNo, OpeType, position, speed,   # ← speed = argumen PER-PERINTAH
                   self.rateAcc, self.rateDec, self.OpeCurrent, Trigger, Memory]
modbusWriteRegWide(serverAddress, 0x0058, directDriveData)   oml_mrtu.py:386-398
```

Jadi:

| Parameter | Perannya yang sebenarnya |
|---|---|
| `motor_config.speed = 100000` | **bukan kecepatan.** Hanya pembilang `rateAcc = 1000·speed/time` |
| `motor_config.acceleration = 1000` | **bukan akselerasi.** Ia *waktu* ramp [ms], penyebutnya |
| `bridge.linear_speed` / `req.linear_speed` | **inilah kecepatan perintah** (field `speed`) |

Turunannya: `rateAcc = rateDec = 1000 × 100000 / 1000 = 100 000 pulse/s²`.

> **Konsekuensi: kandidat 1047 mm/s (1.9 s) tidak pernah menjadi kandidat kecepatan
> traverse.** Ia adalah 100 000 pulse/s² dibaca salah sebagai pulse/s. Rentang §15d
> "~8–64 s" karena itu bukan ketidaktahuan tentang setelan sekarang — ia rentang
> *desain* (8 s = kalau nanti dinaikkan ke ~250 mm/s). **Setelan sekarang punya satu
> nilai, bukan rentang**, dan A5 mengukurnya.

Satu keraguan yang tersisa dan tidak bisa diselesaikan dengan membaca: satuan
`rateAcc`. Komentar di kode saling bertentangan — `oml_mrtu.py:205` menyiratkan
1 000 000 = 1 kHz/s, sedangkan `:484` menyiratkan 1 000 000 = 1000 kHz/s dan
`:594` menyiratkan 30 000 = 30 kHz/s. **Dua dari tiga** konsisten dengan satuan
1 Hz/s. Diasumsikan 1 Hz/s; kalau salah, ia muncul sebagai `t₀` besar di fit A5.

### A2. Prediksi — ditulis untuk DIFALSIFIKASI, bukan untuk dikonfirmasi

```
PULSES_PER_MM     = 12000/(40π) = 95.4930      PULSES_PER_DEGREE = 9000/90 = 100
rateAcc = rateDec = 100 000 pulse/s²  →  1047 mm/s²  |  1000 °/s²
```

| Besaran | Prediksi @ setelan sekarang |
|---|---|
| v linier @ `speed=3000` | 31.4 mm/s |
| v rotasi @ `speed=1000` | 10.0 °/s |
| Waktu ramp linier | 0.030 s (jarak ramp 0.47 mm) — **tak teramati pada 10 Hz** |
| Traverse 2000 mm | 63.7 s |
| Rotasi 180° | 18.0 s |
| Linier ∥ rotasi | **konkuren**, bukan sekuensial (A4) |

Kalau ramp benar-benar 0.03 s, model trapezoidal runtuh jadi linier murni dan
`T = t₀ + Δ/v`. **Kalau fit memberi `t₀` besar (> 1 s), penyebabnya bukan mekanika
melainkan overhead Modbus / granularitas poll — dan itu justru angka yang paling
dibutuhkan scheduler.**

### A3. Dua waktu yang berbeda, dan scheduler memakai yang kedua

| Simbol | Definisi | Sumber |
|---|---|---|
| `T_phys` | gerak fisik pertama → terakhir, dari jejak enkoder | trace poll |
| `T_block` | `Background thread started` → `movement finished successfully` | stempel log |

`T_block` ⊃ `T_phys` + dispatch Modbus (3 motor × write) + deteksi selesai
(poll 0.1 s + 3 × `readPosition` di 115200 baud). **Scheduler memakai `T_block`** —
itulah lama gantry benar-benar tertahan. `T_phys` dipakai untuk menurunkan `v`.

### A4. Struktur biaya yang diuji, bukan diasumsikan

`go_to_table` (moving_table.py:103-107) men-dispatch motor1, motor2, motor3
berturut-turut lalu menunggu **ketiganya** — jadi kode menyiratkan konkuren:

```
T_traverse(Δlin, Δrot) = max( T_lin(Δlin), T_rot(Δrot) ) + overhead
```

Uji dengan satu gerakan gabungan Δlin & Δrot besar. **max → konkuren; ~jumlah →
sekuensial.** Ini mengubah bentuk fungsi biaya setup, jadi ia diukur, bukan
diasumsikan.

### A5. Rancangan pengukuran

Tangga tiga jarak, bukan satu ujung-ke-ujung. Alasannya: satu angka hanya memberi
rata-rata, sedangkan tangga memisahkan **`t₀` (overhead tetap)** dari **`v`** —
dan `t₀` yang menentukan apakah banyak perpindahan pendek mahal. Sekaligus ia
tangga keselamatan: gerakan 100 mm mengungkap arah, fault, dan skala sebelum
komit ke 2 m.

| # | Gerakan | Δ | speed (pulse/s) | Prediksi `T_phys` |
|---|---|---|---|---|
| L1 | linier | 100 mm | 3000 | 3.2 s |
| L2 | linier | 500 mm | 3000 | 15.9 s |
| L3 | linier | travel penuh tersedia | 3000 | ~63.7 s @ 2000 mm |
| L4 | linier | ulang Δ dari L2 | **6000** | 8.0 s ← **uji otoritas per-perintah** |
| R1 | rotasi | 10° | 1000 | 1.0 s |
| R2 | rotasi | 45° | 1000 | 4.5 s |
| R3 | rotasi | 180° | 1000 | 18.0 s |
| C1 | gabungan | Δlin L2 + Δrot R3 | 3000 / 1000 | max→18.0 s, sum→33.9 s |

**L4 adalah uji paling menentukan dari seluruh rangkaian.** Kalau waktu tepat
separuh L2, kecepatan per-perintah otoritatif → model bisa diskalakan ke setelan
mana pun, dan sapuan sensitivitas §14e punya dasar terukur, bukan diekstrapolasi.
Kalau waktunya tidak berubah, ada yang meng-clamp dan §A1 belum lengkap.

**Rotasi ditahan di 1000 pulse/s dan tidak dinaikkan** — §15c menunjukkan 10 °/s
sudah menghasilkan 244 mm/s di ujung lengan, praktis tepat di figur reduced-speed
250 mm/s. Linier dinaikkan ke 6000 (62.8 mm/s, E_kin 39 mJ) yang masih jauh di
bawah figur 250 mm/s dari §15c.

### A6. Instrumen — nol beban tambahan di bus

Poll loop `go_to_table` **sudah** mencatat posisi tiap motor tiap 0.1 s selama
belum sampai (moving_table.py:132), lewat bus yang memang sudah ia pakai.
Jadi jejak posisi didapat **gratis** dari log, tanpa subscriber tambahan.

Ini penting: `commPC` **tidak punya lock** (oml_mrtu.py:25-37), sementara
`publish_joint_states` @10 Hz dan poll loop @10 Hz memakai `client` yang sama dari
dua thread. Menambah pembaca lagi hanya memperburuk kontensi. Parsing log
menghindarinya sepenuhnya.

➜ Jalankan node dengan stdout ke berkas, kirim **satu** panggilan layanan, parse.

### A7. Kriteria sah — dikunci sekarang

Satu run **SAH** hanya bila keempatnya terpenuhi:

1. Log memuat `✅ All motors reached target position.` — bukan timeout, bukan stop.
2. Δ enkoder cocok dengan Δ yang diperintahkan dalam **1%** (penjaga skala
   `PULSES_PER_MM`; toleransi ±50 pulse milik controller terlalu longgar untuk ini).
3. Sampel `Could not read valid position` < 10% di dalam jendela terukur.
4. Jejak posisi monoton pada arah yang diperintahkan.

Model **TERKUNCI** hanya bila:

- L1–L3 memenuhi `T_block = t₀ + Δ/v` dengan **R² ≥ 0.99** dan `v` konsisten
  antar-jarak dalam **±5%**;
- kalau tidak linier → laporkan fit trapezoidal dan **sebut eksplisit** bahwa
  bentuk linier ditolak. Jangan paksakan.

Keluaran: `v_lin`, `v_rot`, `t₀_lin`, `t₀_rot`, vonis konkurensi, lalu formula
biaya setup di §C.

### A8. Fase 2 — lipat & rentang-ulang

Belum dirancang detail sampai Fase 1 selesai; butuh lengan hidup. Konfigurasi
"tuck" **wajib** memakai `ready_arm_joints = [0.0, 2.6, 2.6, 0.0, 0.0, 0.0]`
(reach_fusion.py:169) — konfigurasi yang sama yang dipakai analisis torsi §15b
(31.5%), supaya angka waktu dan angka torsi merujuk pose yang sama. Memakai tuck
lain akan membuat §15b dan §B tidak bisa disandingkan.

---

## B. Hasil terukur

Perangkat keras: **gantry_1 / table1, `/dev/ttyUSB0`**, 2026-08-13. Lengan
terpasang dan bertenaga, **belum terlipat** selama L1–L2 (jarak pendek saja;
L3/L4 dan seluruh uji rotasi ditahan sampai lengan terlipat — §15b/§15c).
Mulai dari enkoder 0.0 mm / 0.0°.

Instrumen: jejak posisi gratis dari poll loop `go_to_table` (§A6). Terbukti
perlu: laju poll terukur **7.4 Hz**, bukan 10 Hz — persis kontensi RS-485
tanpa lock yang diantisipasi. Menambah subscriber sendiri akan memperburuknya.

### B1. Sumbu linier @ `speed = 3000 pulse/s`

| Run | Δ perintah | `T_block` | `T_motion` | v (fit jejak) | R² | n |
|---|---|---|---|---|---|---|
| L1 | 100 mm (9549 pulse) | **3.438 s** | 3.275 s | **31.416 mm/s** | 1.00000 | 22 |
| L2 | 500 mm (47746 pulse) | **16.239 s** | 16.076 s | **31.416 mm/s** | 1.00000 | 111 |

Keempat kriteria §A7 terpenuhi pada kedua run: `✅ All motors reached`, Δ enkoder
cocok < 1%, nol galat baca, jejak monoton.

**v = 31.416 mm/s = 3000.0 pulse/s — persis nilai yang diperintahkan, empat angka
penting, pada dua jarak yang berbeda 5×.** R² = 1.00000 di keduanya: geraknya
linier murni, ramp akselerasi **tidak teramati** (konsisten dengan prediksi ramp
0.030 s / 0.47 mm di A2 — di bawah resolusi 7.4 Hz).

Overhead terurai:

```
dispatch (3 × startPosition over Modbus)  = 0.163 s   (L1 0.162, L2 0.163 — konstan)
teardown setelah "reached"                = 0.001 s
t₀ = T_block − Δ/v :  L1 0.255 s   L2 0.324 s
```

Selisih t₀ (0.069 s) **lebih kecil dari satu periode poll** (0.136 s) — itu memang
kuantisasi deteksi-selesai, bukan ketidakkonsistenan. Jadi:

> **`t₀_lin = 0.29 ± 0.07 s`**  (dispatch 0.163 + latensi deteksi ≈ ½–1 periode poll)

### B2. Model linier — sesuai kriteria kunci §A7

```
T_block(Δ) = 0.29 s + Δ_mm / 31.416 mm/s          @ speed = 3000 pulse/s
```

| Jarak | `T_block` |
|---|---|
| 100 mm | 3.44 s (terukur) |
| 500 mm | 16.24 s (terukur) |
| **2000 mm (rel penuh)** | **63.9 s** (ekstrapolasi dari v terukur) |

### B3. Vonis atas ambiguitas 33×

**Cabang lambat benar. `≈ 63.9 s` untuk rel 2 m pada setelan sekarang.**

Prediksi §7.3a "31.4 mm/s → 63.7 s" **tepat** — dan itu satu-satunya prediksi di
seluruh korpus P1 yang mengenai sasaran. Alasannya layak dicatat: ia satu-satunya
yang diturunkan dari **jalur data kode**, bukan dari intuisi geometris. Lima
dugaan yang meleset di §7.2 semuanya bertipe "seberapa mengikat kendala ini";
tak satu pun bertipe "nilai apa yang mengalir lewat variabel ini".

Yang **tidak** benar adalah pembingkaian "ambigu 33×": 1047 mm/s tidak pernah
jadi kandidat (§A1). Rentang sesungguhnya sudah sempit sejak awal, hanya belum
ada yang menelusuri `writeParamAcc` sampai ke dasarnya.

➜ Untuk §14d: **319 s untuk 5 perpindahan rel penuh, melawan ~400 s kerja pada 80
tugas.** Gerak gantry memang mendominasi. Sel ini berada di sisi "traverse mahal"
pada kurva titik-silang §14e.

---

### B3b. 🔴 BATAS GERAK LINIER FISIK — rel MENTOK jauh sebelum 2000 mm

**Kejadian, 2026-08-13.** L3 memerintahkan 600 → 1900 mm. Gantry **membentur ujung
rel** sebelum sampai; operator mematikan daya untuk menghentikannya.

```
posisi enkoder terakhir terbaca : motor1 158181 pulse = 1656.5 mm
                                  motor2 158170 pulse = 1656.4 mm
target yang diperintahkan       : 181435 pulse = 1900.0 mm  (TIDAK PERNAH TERCAPAI)
```

**Pembacaan enkoder read-only setelah daya kembali** (tanpa tulis, tanpa gerak):

| Motor | Posisi | Alarm |
|---|---|---|
| 1 (linier) | 158132 pulse = **1655.95 mm** | 0x00 tidak ada |
| 2 (linier) | 158250 pulse = **1657.19 mm** | 0x00 tidak ada |
| 3 (rotasi) | −52 pulse = −0.52° | 0x00 tidak ada |

Cocok dengan pembacaan terakhir sebelum daya mati (1656.5 / 1656.4 mm) dalam ±1 mm —
gantry berhenti di situ dan tidak bergeser.

> **Batas gerak linier maju ≈ 1656 mm dari titik nol enkoder, BUKAN 2000 mm.**
> Pakai **1600 mm sebagai batas operasional** (margin 56 mm), dan jangan
> memerintahkan apa pun di atas itu.

Dua catatan yang tidak boleh hilang:

- **Skew 1.24 mm antara kedua motor linier** (118 pulse). Sebelum benturan keduanya
  selisih 11 pulse. Jadi benturan meninggalkan ketidaksejajaran ~1.2 mm antara dua
  kereta rel. Perlu diperiksa mekanisnya; kalau menetap, ia bias sistematis pada
  setiap pose gantry.
- **Tidak ada alarm — tapi itu bukan bukti tidak pernah ada alarm.** Sebagian besar
  alarm AZ ter-reset oleh siklus daya, dan siklus daya persis yang terjadi. Jangan
  kutip "no alarm" sebagai bukti motor tidak stall.

**Batas perangkat lunak yang ada sekarang SALAH dan berbahaya — ini bug nyata:**

| Tempat | Nilai | Kenyataan |
|---|---|---|
| `bridge.max_mm` (dual_table_controller.py:101) | 2000.0 | ~344 mm melewati ujung rel |
| URDF `t*_linear_joint` | 0…2.0 m | idem |

Keduanya menjanjikan travel yang **tidak ada secara fisik**. Perintah absolut mana
pun mendekati 2000 mm akan menabrak end-stop, dan penjaga `bridge.min_mm/max_mm`
**tidak akan menolaknya**. Jalur layanan `move_dual_table` bahkan tidak melewati
penjaga itu sama sekali (§B6). Perlu dikoreksi sebelum scheduler mana pun
membangkitkan pose gantry — capability map memakai rentang linier ini.

**Konsekuensi untuk pengukuran:** semua angka §B1–B2 tetap **sah** — ketiganya
diambil di bawah 1656 mm. Yang berubah hanya ekstrapolasinya: "traverse rel penuh"
adalah **~1.65 m, bukan 2.0 m**.

| Jarak | `T_block` |
|---|---|
| **1650 mm (rel penuh sebenarnya)** | **52.8 s** |
| ~~2000 mm~~ | ~~63.9 s~~ — jarak ini tidak ada |

### B3c. L3 — traverse panjang, sebagian, tetap sah

Terpotong oleh pemutusan daya, tapi jejak sebelum terpotong panjang dan bersih:

| Run | Rentang jejak | v (fit) | R² | n |
|---|---|---|---|---|
| L3 (parsial) | **1055.6 mm** menerus | **31.416 mm/s** | 1.00000 | 237 |

Jadi v terkonfirmasi pada **tiga skala jarak — 100 mm, 500 mm, 1056 mm — identik
sampai 4 angka penting, R² = 1.00000 di ketiganya.** Tidak ada penurunan kecepatan
pada jarak jauh. Justru inilah yang ingin diuji L3, dan hasilnya didapat.

### B3d. Rotasi, konkurensi, dan otoritas kecepatan — semuanya terukur

Dijalankan setelah daya kembali, dari 1550 mm (mundur absolut dari ujung rel; gerakan
absolut itu sekaligus **menghapus skew 1.24 mm** — kedua motor linier mendarat di
148014 pulse yang sama persis).

| Run | Perintah | `T_block` | v (fit jejak) | R² |
|---|---|---|---|---|
| R1 | +10° @1000 | 1.255 s | **10.002 °/s** | 1.00000 |
| R2 | +45° @1000 | 4.767 s | **10.000 °/s** | 1.00000 |
| R3 | −180° @1000 | 18.195 s | **−10.000 °/s** | 1.00000 |
| **C1** | **−500 mm ∥ +125° serentak** | **16.200 s** | −31.417 mm/s **dan** +10.000 °/s | 1.00000 |
| L4 | −500 mm @**6000** | 8.215 s | **−62.834 mm/s** | 1.00000 |

**t₀_rot = 0.26 s** (R1 0.255, R2 0.267, R3 0.195 — sebar dalam satu periode poll,
sama seperti linier). Praktis identik dengan `t₀_lin`, dan memang seharusnya:
keduanya didominasi dispatch Modbus 0.16 s yang terukur konstan di **setiap** run.

**C1 — vonis konkurensi, dan ini menentukan bentuk fungsi biaya:**

```
prediksi konkuren  : max(16.20, 12.76) = 16.20 s
prediksi sekuensial: 16.20 + 12.76     = 28.96 s
TERUKUR                                = 16.200 s
```

> ✅ **`T_traverse(Δlin, Δrot) = max(T_lin, T_rot)` — KONKUREN, bukan jumlah.**
> Kedua sumbu berjalan pada kecepatan penuh serentak, R² = 1.00000 di ketiga motor.
> Dugaan §A4 dari pembacaan kode terkonfirmasi pengukuran.

**Konsekuensi penjadwalan yang langsung terpakai:** pada setelan sekarang rel penuh
= 52.8 s sedangkan 180° = 18.3 s. Karena konkuren, **reorientasi gantry praktis
gratis bila dibungkus bersama traverse yang lebih panjang dari 18.3 s.** Scheduler
tidak perlu menghindari perubahan rotasi — ia perlu menghindari perubahan *linier*.

**L4 — otoritas kecepatan per-perintah, terkonfirmasi langsung:**

6000 pulse/s menghasilkan 62.834 mm/s = **persis dua kali** 3000 pulse/s, dengan
`T_block` 8.215 s melawan prediksi 8.25 s. Jadi §A1 lengkap: kecepatan perintah
otoritatif, tidak ada yang meng-clamp, dan model **berskala linier ke setelan apa
pun** — sapuan sensitivitas §14e sekarang punya dasar terukur, bukan ekstrapolasi.

### B4. Lipat lengan — premis §15b ternyata SALAH pada perangkat keras nyata

Diukur pada `arm_1` (t1_a1), controller `arm_1_controller` (6-DOF, arm-only;
`gantry_1_with_arm_controller` tidak bisa aktif tanpa hardware sendi gantry —
lihat B6).

**Percobaan 1 — lipat lurus ke tuck `[0, 2.6, 2.6, 0, 0, 0]`, 10 s (0.24 rad/s):**

```
GAGAL. joint_2 mencapai 12.78 N·m = 91% dari batas 14 N·m,
hanya 0.41 rad masuk ke gerakan 2.39 rad. Dibatalkan oleh penjaga torsi.
```

Lengan dikembalikan ke pose menggantung semula: berhasil, galat akhir 0.0001 rad,
puncak torsi **4.24 N·m**, tanpa fault.

**Kenapa — dan ini membalik asumsi §15b.** Lengan ini **tergantung di langit-langit**.
Pose istirahat alaminya adalah **menggantung lurus ke bawah**, sejajar gravitasi,
sehingga lengan momennya ~0 — terukur: efektuator **≈ 0.00 N·m** di semua sendi.
"Melipat ke atas" berarti **mengangkat lengan melawan gravitasi**, melewati
konfigurasi horizontal yang justru **puncak torsi**. Kebalikan dari lengan
duduk-di-lantai, yang melipat *turun* dibantu gravitasi.

Kalibrasi model vs ukur (RNEA pinocchio, gravitasi dibalik untuk mount langit-langit):

| Pose | Model statik | Terukur |
|---|---|---|
| Pose abort, turun (dibantu gravitasi) | 4.66 N·m | **4.24 N·m** ✅ cocok |
| Pose abort, naik (melawan gravitasi) | 4.66 N·m | **12.78 N·m** ✗ 2.74× |

⚠️ **Baca tabel itu dengan tepat.** Kedua angka adalah **puncak sepanjang satu
gerakan**, bukan nilai sesaat pada satu pose yang sama. Yang setara: kedua gerakan
melewati **rentang joint_2 yang sama** (0.3413 ↔ 0.7542 rad), berlawanan arah.
Tapi kecepatannya berbeda (naik 0.24 rad/s, turun 0.073 rad/s), jadi arah dan
kecepatan **terkonfound** — selisih 8.5 N·m tidak boleh dinisbatkan ke arah saja.

Yang bisa disingkirkan: **percepatan**. Inersia efektif joint_2 = 0.386 kg·m²,
puncak percepatan quintic = 0.138 rad/s² → torsi inersial **0.053 N·m**. Tidak
relevan. Sisanya gravitasi + gesekan.

Apa yang diukur, persisnya: `feedback_.actuators(i).torque()` dari firmware Kinova
dalam N·m (`kortex_driver/src/hardware_interface.cpp:835`) → state interface
`effort` → `/joint_states`. Batas 14 N·m berasal dari `<limit effort="14">` pada
joint_2 di `gen3_lite.urdf` (joint_3 = 10 N·m).

**Yang BELUM bisa dibedakan:** apakah 12.78 itu lonjakan stiction saat mulai
bergerak, atau beban yang menanjak mengikuti gravitasi. Skrip hanya menyimpan
puncak, bukan deret waktu. Implikasinya berlawanan — kalau stiction, start lembut
mungkin lolos; kalau menanjak, ia memburuk lebih jauh di lintasan. Butuh satu run
ulang dengan deret waktu penuh.

Model menangkap statika dengan baik; **selisih 2.74× muncul hanya saat mendaki** —
gesekan/stiction gearbox. Diskalakan ke puncak sepanjang lintasan, **ketiga
lintasan kandidat melampaui batas 14 N·m**:

| Lintasan | puncak \|τ₂\| model | × 2.74 |
|---|---|---|
| lurus start→tuck | 7.23 N·m | ~19.8 ❌ |
| siku-dulu (j3 lalu j2) | 5.47 N·m | ~15.0 ❌ |
| lewat `arm_1_home` | 6.04 N·m | ~16.5 ❌ |

> 🔴 **Tuck `[0, 2.6, 2.6, …]` besar kemungkinan belum pernah dieksekusi di perangkat
> keras nyata.** `initial_positions.yaml` menyebut dirinya sendiri *"for
> trailer_workcell's ros2_control **fake system**"* — ia pose awal simulasi.
> `reach_fusion.py:169` memakainya sebagai `ready_arm_joints`, tapi jalur itu
> belum pernah diverifikasi di lengan nyata.

**Dan melipat ternyata TIDAK DIBUTUHKAN — bahkan sedikit merugikan.** Radius sapuan
(FK, jari-jari horizontal maksimum dari sumbu rotasi gantry, offset pelat 0.4 m):

| Pose lengan | radius dari sumbu gantry | v ujung @ 10 °/s |
|---|---|---|
| **menggantung (sekarang)** | **0.52 m** | **91 mm/s** ✅ |
| tuck terdokumentasi | 0.55 m | 96 mm/s |
| terentang penuh | 1.16 m | 203 mm/s |

**Menggantung sudah lebih rapat daripada tuck**, pada torsi ~0 dan tanpa gerakan
berisiko apa pun. Jadi prasyarat "lipat sebelum gantry berakselerasi" **sudah
terpenuhi secara gratis** oleh pose istirahat alami.

Catatan untuk §15c: jangkauan URDF memberi 1.16 m terentang, bukan 1.4 m — jadi
203 mm/s, bukan 244 mm/s. Kesimpulan §15c tidak berubah (tetap di orde figur
250 mm/s saat terentang), tapi angkanya sedikit lebih longgar.

**Konsekuensi untuk model biaya setup — ini yang penting:**

Biaya setup **bukan** `lipat + traverse + rentang`. Ia:

```
retract (pose kerja → menggantung)   ← DIBANTU gravitasi, murah
+ traverse
+ extend  (menggantung → pose kerja) ← MELAWAN gravitasi, TERBATAS TORSI
```

Asimetrinya nyata dan terukur (4.24 vs 12.78 N·m pada pose yang sama). Untuk
lengan langit-langit, **"merentang ke pose kerja" dibatasi torsi, bukan waktu** —
dan itu kendala yang sama sekali belum ada di korpus P1.

---

## C. Model biaya setup — TERKUNCI

### C1. TERKUNCI — gerak gantry, lengkap

```
v_lin(s)  = s / 95.4930   mm/s        s = linear_speed [pulse/s]   (uji @3000 dan @6000)
v_rot(s)  = s / 100.0     deg/s       s = rotate_speed [pulse/s]   (uji @1000)

T_lin(Δ)  = 0.29 s + Δ_mm  / v_lin
T_rot(Δ)  = 0.26 s + Δ_deg / v_rot

T_traverse(Δlin, Δrot) = max( T_lin, T_rot )        ← KONKUREN (C1, terukur)

berlaku    0 < Δlin ≤ ~1600 mm  (batas rel fisik, §B3b)
```

Pada setelan sekarang (3000 / 1000 pulse/s):

| Gerakan | `T_block` |
|---|---|
| rel penuh ~1650 mm | **52.8 s** |
| rotasi 180° | **18.3 s** |
| rel penuh **+** 180° serentak | **52.8 s** — rotasi gratis |

Memenuhi seluruh kriteria §A7 dengan selisih besar: **R² = 1.00000 pada sembilan
run**, v selalu sama persis dengan nilai yang diperintahkan, `t₀` konsisten dalam
satu periode poll, dan dispatch Modbus 0.16 s konstan di setiap run.

### C2. TERKUNCI — struktur biaya lengan (arah, bukan durasi)

```
T_setup = retract(kerja → menggantung)  +  T_lin  +  extend(menggantung → kerja)
             dibantu gravitasi, murah              MELAWAN gravitasi, TERBATAS TORSI
```

Bukan `lipat + traverse + rentang` seperti diduga §15b. Pose istirahat sudah
kompak dan bertorsi nol tanpa dilipat (§B4).

### C3. BELUM DIUKUR — jangan dikarang

| Besaran | Status |
|---|---|
| `t_retract` / `t_extend` (**durasi**) | **belum diukur.** Yang terukur baru torsinya (§B4) — dan torsi itulah yang mengikat, bukan waktunya |
| stiction-vs-gravitasi pada 12.78 N·m | belum bisa dibedakan; butuh run ulang dengan deret waktu penuh |
| skew rel 1.24 mm | terkoreksi oleh satu gerakan absolut, tapi penyebab mekanisnya belum diperiksa |

> Untuk sapuan sensitivitas §14e: rel penuh **52.8 s** pada setelan sekarang.
> Karena L4 membuktikan model berskala linier terhadap `linear_speed`, sapuan itu
> sekarang **terukur, bukan diekstrapolasi**: pada ~250 mm/s (§15c) rel penuh jadi
> **~6.9 s**. Rentang desain yang sah: **~7–53 s**.
>
> **Rotasi jangan ikut dinaikkan** — §15c menunjukkan 10 °/s sudah di ambang
> reduced-speed, dan §B4 menunjukkan radius sapuan saat lengan menggantung (0.52 m)
> memberi 91 mm/s, aman; tapi lengan yang terentang saat bekerja mencapai 1.16 m →
> 203 mm/s, sudah dekat figur 250 mm/s.

Dan biaya ini **menghentikan KEDUA lengan** pada gantry tersebut.
