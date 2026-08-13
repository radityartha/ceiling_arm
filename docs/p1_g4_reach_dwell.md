# P1 / G4 — Reach-and-dwell di perangkat keras: definisi sukses & bring-up

> Sesi B, 2026-08-13. Melanjutkan [p1_g3_timing.md](p1_g3_timing.md) (biaya setup
> TERKUNCI) dan [p1_state.md](p1_state.md) §8b/§8c nomor 2–5.
>
> **§A ditulis dan dikunci SEBELUM data diambil** — disiplin yang sama seperti G3.
> §B diisi sesudah. Kalau §B bertentangan dengan §A, yang menang **§B**.
>
> Yang dipakai dari `p1_plan.md`: **hanya** §2b–§2e, §6, §7. §1/§3/§4 dan angka §2
> **tidak dipakai**.

---

## A. Protokol — DIKUNCI SEBELUM AMBIL DATA

### A1. Definisi sukses tugas — TERKUNCI 2026-08-13, sebelum melihat hasil apa pun

Satu tugas **reach-and-dwell** dinyatakan **SUKSES** hanya bila **ketiganya** terpenuhi
serentak dan **kontinu**:

```
posisi   : ‖p_tool − p_cmd‖              <  5 mm
orientasi: ∠(a_tool, a_cmd)              <  5°      (sumbu approach, bukan roll penuh)
dwell    : 2.0 s KONTINU, disampel ≥10 Hz  (≥20 sampel)
```

**Satu sampel di luar toleransi me-RESET jendela dwell.** Bukan rata-rata, bukan
"90% sampel" — kontinu. Alasannya: yang mau dibuktikan adalah pose **ditahan**, dan
rata-rata tidak bisa membedakan "ditahan" dari "dilewati lalu kembali".

**Sukses N-lengan serentak** (ini yang dipakai untuk §8c nomor 3–5):

> Ada **satu** jendela selebar 2.0 s di mana **SEMUA** N lengan memenuhi toleransinya
> **bersamaan**.

Bukan "N lengan masing-masing sukses dalam episode ini" — itu bisa dipenuhi
bergantian, dan bergantian persis yang dilakukan prior work (§2c: Harada 2015
"either… or"). Jendela bersama adalah satu-satunya definisi yang membuat angka
konkurensi berarti.

**Sukses per-tugas dan per-episode dilaporkan TERPISAH**, sesuai §4 plan.

#### Kenapa angka-angka ini, dan bukan yang lain

| Ambang | Dasarnya |
|---|---|
| **5 mm** | Di atas seluruh sumber galat mekanis terukur: skew rel **1.24 mm** (§B3b G3), resolusi enkoder linier **0.0105 mm** (1/95.4930), toleransi controller ±50 pulse = **0.52 mm**, galat sendi kembali-ke-menggantung **0.0001 rad** (§B4 G3) ≈ 0.07 mm di ujung. Jadi kegagalan pada 5 mm bermakna **metode**, bukan gantry. Sekaligus cukup ketat untuk analog titik poles/las Qin et al. |
| **5°** | Toleransi sendi controller 0.03 rad = **1.7°** per sendi; 5° memberi margin akumulasi multi-sendi tanpa jadi longgar. Perhatikan: ini **pelacakan eksekusi**, berbeda dari kendala approach ≤45° di keputusan §5.4 p1_state (itu kendala **kelayakan**, bukan akurasi). |
| **2.0 s** | Cukup panjang untuk membuktikan pose statis (bukan fly-through) dan untuk merata-ratakan jitter poll; sekaligus memberi **durasi tugas tak-nol** yang dibutuhkan model biaya scheduler — durasi nol membuat makespan hanya soal setup. |

### A2. 🔴 TIGA toleransi, tiga arti berbeda — jangan dirancukan di naskah

Ini yang diperingatkan §8b p1_state. Ditulis eksplisit supaya tidak bisa tergelincir:

| Lapis | Angka | Artinya | Dipakai untuk |
|---|---|---|---|
| **L1 — peta kapabilitas** | **5 cm** | "ada solusi di sekitar sini" | membangkitkan **kandidat pose gantry** |
| **L2 — akurasi eksekusi** | **< 5 mm** | pose **perintah** → pose **tercapai** | definisi sukses tugas (A1) |
| **L3 — akurasi persepsi** | **3–5 cm** | objek **nyata** → pose **perintah** | dilaporkan, tidak masuk kriteria sukses |

L3 dari kalibrasi ekstrinsik RGBD 2026-07-30: kesepakatan lintas-kamera pada
centroid board **3.1 cm**, tumpang-tindih cloud (NN simetris) median **4.4 cm**,
fit bidang lantai dalam 3.5 cm dari z=0.

> ⚠️ **Batasan L2 yang WAJIB disebut di naskah.** L2 diukur dari TF
> `world → t{1,2}_a{1,2}_tool_frame`, yang dihitung `robot_state_publisher` sebagai
> **FK dari sendi TERUKUR**. Jadi L2 menangkap galat servo/pelacakan, tapi
> **TIDAK** menangkap galat kalibrasi kinematik maupun mounting. Akurasi absolut ke
> dunia nyata butuh pengukuran eksternal dan **tidak diklaim** di sini.

Kalau ketiganya tidak dipisah, reviewer membaca "menjangkau dalam 5 cm" dan itu
terdengar lemah — padahal 5 cm adalah L1, yang memang tidak pernah dimaksudkan
sebagai akurasi.

### A3. Instrumen — apa yang ADA dan apa yang TIDAK

Pembacaan kode 2026-08-13, `gantry_reach_executor.py`:

```
gantry_reach_executor.py:884-893   SUCCESS  ⟸  _wait_until_reached(...)
gantry_reach_executor.py:584-599   _wait_until_reached: max|q_goal − q_live| ≤ reach_tol
gantry_reach_executor.py:292       reach_tol default = 0.03
```

| Yang dibutuhkan A1 | Ada? |
|---|---|
| galat **task-space** di tool frame | ❌ tidak — kriterianya **joint-space** |
| galat orientasi | ❌ tidak |
| **dwell** | ❌ tidak ada sama sekali |
| jendela **bersama** antar-lengan | ❌ tidak ada |
| rekaman deret waktu galat per tugas | ❌ hanya CSV per-pick, tanpa galat tercapai |

Dan satuan `reach_tol = 0.03` **campur**: radian untuk sendi lengan, **meter** untuk
sumbu linier gantry. Jadi kriteria SUCCESS yang berlaku sekarang mengizinkan gantry
meleset **3 cm** dan tetap melaporkan sukses — 6× lebih longgar dari ambang L2.

> ➜ **Konsekuensi: A1 tidak bisa diukur oleh kode yang ada.** Instrumen baru
> (`reach_dwell_monitor`) dibangun lebih dulu, di §A4. Kriteria joint-space lama
> **tidak dibuang** — ia tetap berguna sebagai penjaga konvergensi controller — tapi
> ia **bukan** definisi sukses tugas dan tidak boleh dikutip sebagai itu.

### A4. `reach_dwell_monitor` — instrumen pengukur, terpisah dari yang diukur

Node pengamat murni. **Tidak menggerakkan apa pun**, hanya membaca TF dan menilai.
Dipisahkan dari executor dengan sengaja: yang menilai sukses tidak boleh yang juga
memilih kandidat, kalau tidak kriterianya bisa ikut disetel ke hasil.

```
langganan : /reach_dwell/target/<arm>   geometry_msgs/PoseStamped   (pose PERINTAH)
sampel    : TF world → <tool_frame>     @ rate (default 20 Hz)
terbitkan : /reach_dwell/status         std_msgs/String  (JSON, per-sampel)
CSV       : satu baris per sampel + satu baris ringkas per tugas
```

Beban bus: TF sudah disiarkan; monitor hanya membaca buffer. Nol perintah tambahan
ke Modbus maupun Kortex — pelajaran §A6 G3 (poll turun ke 7.4 Hz karena kontensi
RS-485 tanpa lock) tetap berlaku.

### A5. Prasyarat fisik — DIVERIFIKASI SENDIRI, bukan diterima

Sesi A menabrak ujung rel karena "rel bebas ~2 m" diterima tanpa diperiksa. Yang
wajib dicek **dengan pembacaan, bukan dengan pernyataan**, sebelum gerakan apa pun:

1. Posisi enkoder **aktual** ketiga motor tiap gantry (baca, jangan tulis).
2. Batas rel operasional **1600 mm** — bukan `bridge.max_mm = 2000`, bukan URDF 0…2.0 m.
   Keduanya **salah dan berbahaya** (§B3b G3). Diperbaiki di §A6 sebelum gerak.
3. Ruang bebas di sekitar sapuan lengan; lengan istirahat **MENGGANTUNG** (radius
   0.52 m), **jangan** diperintahkan ke tuck `[0, 2.6, 2.6, 0, 0, 0]` — 12.78/14 N·m,
   dibatalkan penjaga torsi (§B4 G3). Itu pose awal sistem **fake**.
4. Tangga jarak bertahap, naik bertahap, **berhenti pada batas terukur**.

### A6. Bug batas rel — diperbaiki sebelum data, bukan sesudah

| Tempat | Nilai sekarang | Kenyataan |
|---|---|---|
| `bridge.max_mm` (dual_table_controller.py) | 2000.0 | ~344 mm melewati ujung rel |
| URDF `t*_linear_joint` upper | 2.0 m | idem |
| jalur layanan `move_dual_table` | **tidak lewat penjaga sama sekali** | tidak ada perlindungan |

Ketiganya dikoreksi ke batas operasional **1600 mm** sebelum data diambil. Peta
kapabilitas memakai rentang linier ini, jadi scheduler mana pun akan membangkitkan
pose gantry di luar rel kalau dibiarkan.

### A7. Kriteria sah satu run — dikunci sekarang

Satu run **SAH** hanya bila keenamnya terpenuhi:

1. Rencana ditemukan dan eksekusi tidak dibatalkan.
2. Tidak ada torque fault; puncak |τ| tiap sendi dicatat (bukan hanya lolos/gagal).
3. TF `world → tool_frame` tersedia **sepanjang** jendela; nol lookup gagal.
4. Laju sampel efektif ≥10 Hz **terukur** (bukan diasumsikan dari parameter rate —
   §B G3 menunjukkan poll 10 Hz nyatanya 7.4 Hz).
5. Posisi enkoder gantry di dalam [0, 1600] mm sepanjang run.
6. Pose perintah tercatat **sebelum** gerakan, bukan direkonstruksi sesudah.

Run yang gagal salah satu dari 1–2 **tetap dilaporkan** sebagai kegagalan tugas.
Run yang gagal 3–6 adalah kegagalan **instrumen** — dibuang, dan sebabnya dicatat.

### A8. Urutan eksekusi, dan gerbang di antaranya

| # | Isi | Gerbang untuk lanjut |
|---|---|---|
| **0** | bring-up, verifikasi prasyarat A5, kalibrasi instrumen tanpa gerak | TF hidup, laju sampel terukur ≥10 Hz |
| **2** | satu lengan reach-and-dwell ke target terpersepsi | ≥8/10 tugas sukses per A1 |
| **3** | dua lengan se-gantry, jendela **bersama** | ada jendela bersama 2.0 s |
| **4** | + gerak gantry antar tugas | biaya setup terukur cocok dengan model C1 G3 (±10%) |
| **5** | dua gantry, empat lengan | dilaporkan apa adanya (§7 plan: paper berdiri di 2+3) |

Nomor 0 tidak butuh izin gerak. Nomor 2 ke atas **wajib minta izin sebelum
menggerakkan perangkat keras**, tiap kali naik tangga jarak.

### A9. Prediksi — ditulis untuk DIFALSIFIKASI

Ditulis sekarang supaya §7.2 p1_state ("ukur, jangan menduga") punya catatan yang
bisa dinilai. Lima dugaan sesi G2 meleset; satu dugaan G3 yang tepat adalah yang
diturunkan dari **jalur data kode**, bukan dari intuisi. Ditandai jenisnya:

| # | Prediksi | Jenis | Kalau meleset |
|---|---|---|---|
| P1 | Galat posisi tunak ≪ 5 mm (orde 0.1–1 mm), karena L2 adalah FK dari sendi terukur dan galat servo sendi terukur 0.0001 rad | jalur data | ambangnya yang salah, bukan sistemnya |
| P2 | Yang menggigit bukan akurasi melainkan **kelayakan rencana** — target terpersepsi bisa jatuh di L3 3–5 cm dari objek nyata, jadi lengan bisa "sukses" per A1 sambil meleset dari benda fisiknya | jalur data | L3 lebih baik dari yang diukur di kalibrasi |
| P3 | Dua lengan se-gantry akan **sukses**, karena interference terukur 0.00% (§10 G2) | intuisi geometris ⚠️ | justru hasil menarik — kopling lewat pose bersama, bukan tabrakan |
| P4 | Jendela bersama 2.0 s **tidak gratis**: butuh kedua lengan tiba lalu diam serentak, sedangkan MoveIt merencanakan tiap lengan terpisah | struktural | konkurensi lebih murah dari dugaan |

P3 ditandai ⚠️ justru karena bertipe "seberapa mengikat kendala ini" — persis tipe
yang meleset lima kali di G2, dan semuanya meleset ke arah **menduga lebih mengikat**.

---

## B. Hasil terukur

### B0. Instrumen dibangun dan DIVALIDASI — sebelum menyentuh perangkat keras

`reachability_gng/reach_dwell_monitor.py` (baru). Pengamat murni: membaca TF,
menilai, mencatat. Tidak mengirim perintah apa pun.

Divalidasi terhadap **TF sintetis dengan galat yang diketahui persis** —
`test/validate_reach_dwell_monitor.py`. Ini bukan formalitas: instrumen inilah yang
akan menghasilkan angka utama paper, jadi ia diuji terhadap kebenaran-dasar yang
tidak bisa ia pengaruhi (§7.1 "ground truth dulu" berlaku juga untuk alat ukurnya).

| Kasus | Diuji | Hasil |
|---|---|---|
| A | tepat di target | ✅ SUKSES, galat 0.0 mm, 41 sampel @ 20.3 Hz |
| B | meleset **6 mm** (ambang 5) | ✅ benar TIDAK sukses |
| C | di dalam toleransi, **satu** sampel menyimpang di t=1.0 s | ✅ jendela **RESET** — dwell selesai di 3.19 s, bukan 2.0 s |
| D | posisi sempurna, approach meleset **6°** | ✅ benar TIDAK sukses |
| E | dua lengan, masuk toleransi **berselang 1.5 s** | ✅ jendela bersama menyala **2.04 s setelah lengan TERAKHIR** masuk |

Kasus C adalah yang memisahkan "ditahan" dari "dilewati": kriteria rata-rata akan
lolos di situ. Kasus E memisahkan "serentak" dari "bergantian" — kalau jendela
di-anchor ke lengan pertama, ia menyala 1.5 s terlalu awal dan mengklaim konkurensi
yang tidak pernah terjadi.

> 🔴 **Validasi E menemukan bug nyata di rancangan pertama monitor.** Versi awal
> mengeluarkan lengan dari himpunan aktif begitu ia sukses — padahal lengan itu
> **masih menahan** posenya. Akibatnya konkurensi N-lengan praktis mustahil
> terdeteksi kecuali semua lengan kebetulan tiba dalam satu jendela yang sama.
> Persis angka yang dikejar §8c nomor 3–5. Ditemukan hanya karena diuji.

### B1. Bug batas rel — diperbaiki di empat tempat

| Tempat | Sebelum | Sesudah |
|---|---|---|
| `bridge.max_mm` (dual_table_controller.py) | 2000.0 | **1600.0** |
| jalur layanan `move_dual_table` | **tanpa pemeriksaan sama sekali** | `_check_travel_limits()` sebelum dispatch |
| `moving_table.urdf.xacro` `linear_joint` | upper 2.0 m | **1.6 m** |
| `workcell_full.urdf` (dipakai alat offline) | upper 2.0 m ×2 | **1.6 m** ×2 |

Penjaga layanan menyelesaikan target **absolut** lebih dulu, karena `go_to_table`
adalah gerakan **relatif** (`moving_table.py:100` menambahkan increment ke pembacaan
enkoder). Delta kecil yang tampak aman tetap bisa mendarat di luar ujung rel — persis
mode kegagalan 2026-08-13.

**Lubang yang SENGAJA dibiarkan terbuka, dan alasannya:** `jog` (operation_type
10–13) kontinu dan tidak punya target, jadi tidak bisa diperiksa jangkauannya. Ia
tetap akan menabrak end stop. Dibiarkan karena jog adalah alat manusia-dalam-loop
(`table_keyboard.py`) dan menutupnya butuh watchdog posisi; ia **tidak** ada di jalur
scheduler. Sekarang memberi peringatan keras bila dijalankan dalam 100 mm dari batas.

### B2. 🔴 TEMUAN BARU — peta kapabilitas memuat 8 pose gantry yang TIDAK ADA

Ini konsekuensi §B3b G3 yang belum pernah ditarik, dan ia menyentuh angka G2.

```
irm_sweep.py:263 (sebelum)   lin = np.arange(0.0, 2.0 + 1e-9, lin_step)
```

Dengan `lin_step = 0.05`, itu **41 pose linier**, di mana rel fisik hanya sampai
1.656 m. Jadi **8 dari 41 kolom (19.5%) setiap mask kapabilitas adalah pose yang
tidak bisa dicapai gantry.**

Diperbaiki: konstanta `RAIL_MAX_M = 1.60` dengan sitasi pengukuran, dipakai sebagai
default `make_grid`. Sapuan baru otomatis fisik.

> ⚠️ **Seluruh peta yang disapu SEBELUM 2026-08-13 memakai 0–2.0 m.** Angka cakupan
> dan ko-kelayakan yang diambil darinya **optimis** dan harus disapu ulang sebelum
> dikutip.

**Besar kerusakannya SUDAH DIUKUR, bukan ditakar** — dihitung langsung dari
`/tmp/cap_g1.npz` (41×72, dibangun 2026-08-12), dengan membandingkan 41 kolom
terhadap 33 kolom yang benar-benar ada:

| tol | target hilang total | penyusutan `\|G_a(t)\|` median | p90 |
|---|---|---|---|
| 0.05 m | 12 (**0.40%**) | 5.5% | 52.4% |
| 0.10 m | 5 (0.16%) | 7.3% | 50.5% |
| 0.20 m | **0** (0.00%) | 10.3% | 45.5% |

15.8% massa-pose berada di kolom hantu, tapi **tidak ada angka kelayakan G2 yang
runtuh**: "4 lengan tersedia 87.5%", cakupan, dan identitas geser-π semuanya
bertahan. Yang bergerak adalah **masukan scheduler**: ekor target yang dilayani dari
ujung jauh rel kehilangan ~**separuh** opsi pose gantry-nya (p90), dan 12 target
lenyap sama sekali pada tol 5 cm.

➜ **Koreksi, bukan penarikan.** Peta tetap wajib disapu ulang; angka utama G2 tidak.
Dan sekali lagi arah melesetnya sama seperti enam dugaan sebelumnya (§7.2):
**menduga kendala lebih mengikat daripada kenyataannya.** Yang terdampak paling langsung: "1–6 pose gantry cukup untuk 80 tugas"
> dan "1 pose menutupi 62–99%" (§2 p1_state) — keduanya diuntungkan oleh 8 kolom
> tambahan yang tidak ada. **Angka waktu G3 tidak terdampak** (semuanya diambil di
> bawah 1656 mm), begitu pula geser-π (`T_arm2 = roll(T_arm1)`, sifat rotasi).

### B3. Langkah 0 — prasyarat fisik, DIBACA bukan diterima

2026-08-13. **Nol perintah gerak dikirim.** Semua di bawah adalah pembacaan.

| Prasyarat | Hasil |
|---|---|
| `/dev/ttyUSB0`, `/dev/ttyUSB1` | ✅ keduanya ada |
| `enp112s0` | ✅ **192.168.2.100/24** — jebakan "naik tanpa IP" tidak aktif |
| 4 lengan (ICMP) | ✅ .10 .11 .12 .13 semua menjawab |
| 2× D455 | ✅ keduanya USB3 **5000 Mbps**; serial librealsense **cocok** dengan yang terkalibrasi |
| Proses basi | 1 ditemukan (`color_cloud`, 10 hari) — dibunuh. Nol `static_transform_publisher` basi |
| Init kedua gantry | ✅ Table 1 & 2 initialized, **nol** galat baca, **nol** alarm |

> ⚠️ **Serial USB ≠ serial kamera.** Deskriptor USB melaporkan 229123063427 /
> 308243062177, yang **tidak** cocok dengan serial terkalibrasi — itu serial **ASIC**.
> `rs-enumerate-devices` memberi 234222303079 / 241122302297, cocok persis. Nyaris
> jadi alarm palsu; diperiksa, bukan diasumsikan.

**Posisi gantry sesungguhnya — inilah yang tidak diperiksa Sesi A:**

| Sendi | Terbaca | Dalam pulse |
|---|---|---|
| `t1_linear` | **550.009 mm** | 52519 |
| `t1_rotation` | 0.000° | 0 |
| `t2_linear` | −0.010 mm | **−1** |
| `t2_rotation` | 0.190° | 19 |

Keempatnya **bilangan bulat pulse persis** — bukti bacaan enkoder sungguhan, bukan
nilai default yang di-cache. Stabil bit-identik sepanjang 20 sampel.

➜ **gantry_1 ada di 550 mm**, jadi tersisa **1050 mm maju** sampai batas operasional
1600 mm, dan 550 mm mundur. (Sesi A meninggalkannya di 1550 mm; sejak itu digeser.)

**Penjaga batas rel diuji — di luar perangkat keras, tempat bug tidak bisa menggerakkan
apa pun.** 11 kasus, semua benar, termasuk:

| Kasus | Vonis |
|---|---|
| **ABS 1900 mm — perintah persis yang menabrak rel di Sesi A** | 🚫 REJECT |
| RELATIF +1100 mm dari 550 → 1650 (lewat stop) | 🚫 REJECT |
| ABS 1600 / RELATIF +1000 → 1550 | ✅ ACCEPT |
| ABS 1601, ABS −1, RELATIF −600, rotasi 181° | 🚫 REJECT |

Kasus relatif itu yang penting: kode lama mengirimkannya tanpa keberatan sedikit pun.

**Dan penjaga itu diuji LANGSUNG DI PERANGKAT KERAS — dengan kegagalan yang dibuat
tidak berbahaya.** `bridge.max_mm` diturunkan sementara ke **600 mm**, lalu
diperintahkan ABS **700 mm**. Kalau penjaga bekerja → ditolak. Kalau penjaga rusak →
gantry bergerak 550→700 mm, jauh di dalam rel. Itu satu-satunya cara menguji penjaga
batas di mana kegagalannya bukan benturan.

```
response: success=False
message : 🚫 REJECTED table1: linear target 700.0mm is outside the physical rail
          [0, 600]mm (now at 550.0mm). The rail END STOP is at ~1656mm measured...
```

Bukti bahwa penolakan terjadi **di lapis yang benar**, bukan sekadar menangkap akibat:

| Pemeriksaan sesudah | Hasil |
|---|---|
| posisi enkoder | **bit-identik** 52519 pulse — nol gerakan |
| `Background thread started` di log | **0 kejadian** — thread gerak tidak pernah dibuat |
| tulisan Modbus ke driver | **tidak ada** — ditolak sebelum dispatch |
| `bridge.max_mm` sesudah | dikembalikan ke 1600.0 |

Penjaga membaca posisi **hidup** (`now at 550.0mm`), jadi ia menilai target absolut
sesungguhnya, bukan asumsi.

### B4. 🟡 Dugaan awal: kalibrasi RGBD memburuk — TERNYATA TIDAK (lihat §B4b)

Diperiksa tanpa board (board tidak bisa diasumsikan terpasang), dengan
**kesepakatan lintas-kamera**: deproyeksi kedua depth ke `world`, NN simetris.

| Metrik | 2026-07-30 | **Sekarang** |
|---|---|---|
| NN simetris, **lantai bersama** | 4.4 cm | **5.6 / 5.8 cm** (dua run) |
| bidang lantai `rgbd` | z₀ −0.035 m, tilt <2° | z₀ −0.067…−0.098 m, tilt 2.1–5.5° |
| bidang lantai `rgbd2` | z₀ −0.005 m, tilt <2.3° | z₀ −0.023 m, tilt 2.6° |

Referensi kalibrasi rusak dulu ~36 cm, jadi ini **jelas bukan** rezim rusak.

Yang bisa dan tidak bisa disimpulkan:

- ✅ Angka lantai-saja **berulang** (5.6 dan 5.8 cm) → bukan derau pengukuran.
- ✅ Sekarang **di atas** target penerimaan **<5 cm** milik memo kalibrasi sendiri.
- ⚠️ Fit bidang `rgbd` **berubah antar-frame berurutan** (−0.098/5.5° lalu −0.067/2.1°),
  jadi estimator bidang lantai saya sendiri berderau; **jangan** kutip satu angka darinya.
- ❌ **Belum bisa dinyatakan drift kalibrasi.** Estimator lantai saya berbeda dari
  yang menghasilkan angka 2026-07-30. Yang setara hanyalah baris NN lantai-bersama.

> ⚠️ **DUGAAN DI ATAS SALAH — DIKOREKSI, lihat §B4b. L3 TETAP 3–5 cm.** Proksi
> NN-lantai ternyata menyesatkan. Dibiarkan tertulis di sini karena §7.2 menghitung
> dugaan yang meleset, dan ini satu lagi ke arah yang sama: **menduga kendala lebih
> mengikat daripada kenyataannya**.

### B4b. Kalibrasi ulang DIJALANKAN, lalu DITOLAK oleh pengukuran

Board dipasang ulang di anchor tersimpan (dikonfirmasi pengguna). Lengan dilepas
sementara — awalnya `rgbd2` tidak bisa mengunci board karena **lengan menggantung
tepat menghalangi** garis pandangnya.

Solve baru berhasil, RMS reproyeksi **0.291 px** (rgbd) / **0.388 px** (rgbd2),
keduanya jauh di bawah ambang 1 px dan setara run 2026-07-30 (0.28/0.42).
Disambiguasi `--camera-hint-xyz` bekerja benar (rgbd → +Y, rgbd2 → −Y).

**Tapi nilai baru itu lebih BURUK pada kriteria yang sesungguhnya:**

| Ekstrinsik | NN lantai-saja, **adegan sama** |
|---|---|
| lama (default sekarang) | **6.5, 6.6 cm** |
| baru (solve segar) | 7.0, 7.0 cm |

➜ **Nilai lama dipertahankan. Solve baru dibuang.**

> 🔴 **RMS reproyeksi TIDAK bisa dipakai memilih kalibrasi.** Ia hanya mengukur
> konsistensi-diri terhadap pose board yang **kita ketikkan sendiri**. Solve baru
> menang di RMS dan kalah di kesepakatan lintas-kamera. Persis yang diperingatkan
> docstring `verify_extrinsics.py` sendiri.

**Dan perbandingan pertamanya sempat terkonfound — ini pelajaran metodologisnya.**
Pengukuran "lama" pertama (5.6–5.8 cm) diambil **dengan lengan terpasang**, yang
"baru" (7.0 cm) **tanpa lengan**. Mengukur ulang yang lama di adegan tanpa-lengan
memberi 6.5–6.6 cm — jadi **pelepasan lengan sendiri menggeser metrik ~0.9 cm**
(lantai yang tadinya terhalang jadi terlihat, pada sudut serempet yang depth-nya
berderau). Tanpa mengukur ulang, selisih 1.3 cm itu akan **salah dinisbatkan** ke
kalibrasi.

**Vonis sesungguhnya — sentroid board, tandingan langsung angka 3.1 cm:**

```
rgbd  : centroid -> world [0.9462, 0.0702, 0.6174]   (depth 1.804 m)
rgbd2 : centroid -> world [0.9340, 0.0611, 0.5932]   (depth 2.073 m)
kesepakatan lintas-kamera: 2.9 cm  -> GOOD
```

**2.9 cm, lebih baik dari baseline 3.1 cm.** Ekstrinsik tidak pernah memburuk.

Sekaligus memvalidasi anchor secara independen: sentroid terukur berada
≈(+0.105, −0.14) m dari origin tersimpan (0.849, 0.181, 0.593) — persis offset
setengah-board untuk grid 5×7 @ 35 mm.

> **L3 = 3–5 cm, TIDAK berubah.** NN-lantai adalah proksi yang lebih kasar,
> didominasi derau depth lantai bersudut serempet, bukan galat kalibrasi. Untuk
> menilai kalibrasi pakai **sentroid board** (target titik), bukan NN lantai.

### B5. Belum diisi — tugas §8c nomor 2–5

> Menunggu izin gerak. Diisi hanya dengan run yang memenuhi §A7.
