# P1 — status & handoff

> Ditulis 2026-08-12 akhir sesi. **Baca ini dulu, sebelum `p1_plan.md`.**
>
> ⚠️ `p1_plan.md` §1, §3, §4 masih menggambarkan formulasi **coordination-degree
> selection** yang **sudah ditinggalkan**. Jangan membangun dari sana. Angka §2
> juga mati. Yang masih sah dari plan: §2b (literatur), §6 (batas P1/P2/P3),
> §7 (risiko).
>
> 🟢 **PLAN B aktif (2026-08-13):** kelas tugas = **reach-and-dwell**, bukan
> pick-and-place. Grasp + handover ditunda, **tenggat keputusan 2026-11-13**. Lihat §5.5.
> Hardware bisa jalan sekarang tanpa menunggu grasp.
>
> 🚦 **Sebelum menulis kode scheduler, baca §7 (Disiplin kerja).** Satu hal di sana
> masih berlaku penuh: **ground truth dulu, heuristik belakangan**.
>
> ✅ **Ambiguitas traverse 33× SUDAH SELESAI (Sesi A, 2026-08-13).** Biaya setup
> terukur dan **TERKUNCI** — lihat [p1_g3_timing.md §C](p1_g3_timing.md). §7.3a di
> bawah dipertahankan sebagai catatan sejarah, **bukan** pekerjaan tersisa.
>
> ✅ **Sesi B, 2026-08-13** — [p1_g4_reach_dwell.md](p1_g4_reach_dwell.md):
> definisi sukses tugas **TERKUNCI**, instrumen pengukur dibangun + divalidasi,
> bug batas rel diperbaiki, peta kapabilitas disapu ulang ke rel fisik.
> **Tiga angka yang berubah dan memengaruhi dokumen ini: §3, §5.7, §8b.**

---

## 1. Formulasi sekarang

**Bukan** "berapa lengan yang layak bekerja bersama" (itu hampir selalu 4 — terukur).
**Melainkan** alokasi + penjadwalan:

> Diberi N tugas dan lingkungan terpersepsi, tentukan **urutan, penugasan lengan,
> dan lintasan pose gantry** untuk memaksimalkan throughput, di bawah kopling
> gantry-bersama dan kapabilitas yang berubah online.

Slot taksonominya: **`XD [ST-MR-TA]`**

> ⚠️ **`XD [ST-SR-TA]` sudah ditempati** — Qin et al., RCIM 2024, menyebutnya eksplisit
> ([p1_plan.md §2e](p1_plan.md)). Yang membedakan kita **bukan slotnya**, melainkan
> **asal-usul XD-nya**: pada mereka kopling = tabrakan antar lengan ber-base independen
> (hilangkan tabrakan → masalah terurai); pada kita kopling = **variabel pose bersama**
> (interference 0.00% tapi ko-kelayakan tetap 59.5% — jadi terbukti bukan tabrakan).
> Plus `MR` dari handover, yang mereka tidak punya.
- `XD` cross-schedule dependencies (Korsah 2013) — jadwal arm1 membatasi arm2 lewat `g` bersama
- `ST` satu tugas per lengan pada satu waktu
- `MR` **handover butuh dua lengan** → tugas campuran SR (pick) + MR (handover)
- `TA` time-extended, bukan instantaneous

Istilah pendukung: **shared positioning resource** (gantry), **sequence-dependent
setup times** (waktu traverse gantry).

Derajat koordinasi = **outcome yang dilaporkan**, bukan variabel keputusan.

---

## 2. Kenapa pivot — geometri statis tidak mengikat, lingkungan dan waktu yang mengikat

| Kendala | Mengikat? | Angka |
|---|---|---|
| Jangkauan | ❌ tidak | 4 lengan tersedia **87.5%** |
| Arm–arm collision (eksak, canonical) | ❌ tidak | **0.00%** pasangan hilang |
| Zona-eksklusi Meso `r=0.20`, dua arah | ✅ **ya** | 4 lengan **61.8%**, handover mustahil |
| Coverage / jumlah pose gantry | ❌ tidak | **1–6 pose** cukup untuk 80 tugas; 1 pose menutupi 62–99% |
| Torsi gravitasi @ payload nominal 0.5 kg | ❌ tidak | **0.00%** melampaui batas (tapi p95 89%, margin tipis) |
| Penghalang dinamis (1 orang) | ✅ **ya** | **19.6%** pasangan hilang |

> 🔴 **LIMA kendala diukur, lima tidak mengikat** ([p1_g2_results.md §14c](p1_g2_results.md)).
> Sel ini kelewat longgar secara geometri dan mekanik. **Kelayakan bukan kendalanya —
> WAKTU kendalanya**, dan waktu traverse gantry masih ambigu 33×. Apakah P1 punya
> masalah keputusan sama sekali bergantung pada satu parameter yang belum diukur.
> Laporkan sebagai **titik-silang** (§14e), jangan berharap gantry lambat.

Geometri statis **permisif**; yang langka adalah **waktu dan lingkungan**.

**Argumen kuantitatif inti paper:** irisan dua lengan tidak boleh dipartisi statis
(konkurensi 87.5% → 61.8%, dan handover ikut mati karena handover *menuntut*
irisan). Ia harus dibagi dalam **waktu** → itu definisi mutex resource → itu yang
membuat masalahnya scheduling. **Selisih 25 poin itu yang dibeli scheduling.**

---

## 3. Yang sudah dibangun

### `reachability_gng/irm_sweep.py` — instrumen pengukuran (offline, tanpa ROS)
`verify` · `cloud` · `sweep` · `analyze` · `interfere` · `policy` · `obstacle`

### `reachability_gng/capability.py` — **Lapis 2, oracle**
`build` · `validate` · `overlap`

```python
cm = CapabilityMap.load('/tmp/cap_g1_rail160.npz')   # ⚠ BUKAN cap_g1.npz
mask = cm.reach('arm2', xyz)      # (33, 72) bool: pose gantry yang menjangkau
# co-feasible = (mask_A & mask_B).any()
```

- Satu peta **per gantry**; pasangan diturunkan `np.roll(mask, 36, axis=rot)` — eksak
- Payload per node: mask multi-toleransi + canonical config index
- Index = **grid** (terukur mengalahkan topo; lihat §5)

> 🔴 **Grid linier sekarang 33×72, bukan 41×72 (Sesi B, 2026-08-13).** Rel fisik
> mentok ~1656 mm, jadi 8 dari 41 kolom peta lama adalah pose gantry yang **TIDAK
> ADA**. `irm_sweep.RAIL_MAX_M = 1.60` sekarang default.
>
> **PAKAI `/tmp/cap_g1_rail160.npz` dan `/tmp/cap_g2_rail160.npz`.** Yang lama
> (`cap_g1.npz`, `cap_g2.npz`) memuat kolom hantu — jangan dipakai, jangan dikutip.
>
> Besar kerusakannya **diukur, bukan ditakar**: target hilang total hanya
> **0.40% / 0.16% / 0.00%** (tol 0.05/0.10/0.20), penyusutan `|G_a(t)|` median
> 5.5–10.3% tapi **p90 ~50%**. Jadi **tidak ada angka kelayakan §2 yang runtuh** —
> ini koreksi, bukan penarikan. Yang bergerak adalah masukan scheduler: ekor target
> yang dilayani dari ujung jauh rel kehilangan ~separuh opsi pose gantry-nya.
> Peta baru terverifikasi **100.0000% identik** dengan yang lama pada 33 kolom
> bersama (7.44 juta sel, 3 toleransi + canon).

### `reachability_gng/reach_dwell_monitor.py` — **instrumen sukses tugas** (Sesi B)
Pengamat murni: baca TF, nilai, catat. Tidak memerintah apa pun. Menilai kriteria
§8b. Divalidasi terhadap TF sintetis bergalat-diketahui
(`test/validate_reach_dwell_monitor.py`, 5/5 lulus).

> ⚠️ **`gantry_reach_executor.py:884-893` BUKAN definisi sukses tugas.** Ia
> joint-space dengan satuan campur — `reach_tol = 0.03` berarti radian untuk sendi
> lengan tapi **meter** untuk sumbu linier gantry, jadi ia meloloskan meleset 3 cm.
> Tetap berguna sebagai penjaga konvergensi controller; jangan dikutip sebagai sukses.

### Fakta struktural yang dipakai algoritma
```
T_arm2(lin, rot) ≡ T_arm1(lin, rot + π)                    (eksak, 2e-16)
T_arm3(lin, rot) ≡ T_arm1(lin, rot) digeser y −0.72        (eksak)
```
Satu peta cukup untuk empat lengan. Cloud tool di frame base **bit-identik**.

`G_a(t)` **selalu terhubung sederhana** di silinder `(linear, rotation)` — 0 dari 616
pasangan punya >1 komponen. **Jangan klaim topologi C-space.**

---

## 4. Representasi — apa topologis, apa tidak

| Komponen | Representasi | Status |
|---|---|---|
| Peta lingkungan `O(t)` — statis **dan** dinamis | GNG → **MS-BL-GNG** | ✅ **DIPORT 2026-08-13 (Sesi C)** — `bl_gng.py`; dipakai `map_topo_static.py`, `topo_static_pub.py`, `env_gng.py` |
| Action map `xyz → q` | **GCS** | ✅ **DIPORT 2026-08-13 (Sesi C)** — `gcs.py`, invarian simpleks diperbaiki; 4 model di `/tmp/arm{n}_gcsx.npz` |
| Operasi himpunan Meso | graf ↔ graf | arm↔env jalan (`reach_fusion.py`); **arm↔arm baru di `capability.py overlap`** |
| Index capability | **grid** | terukur; topo salah 11% keputusan, bias optimis |

✅ **Sesi C selesai 2026-08-13** — [p1_g5_msbl_gcs.md](p1_g5_msbl_gcs.md).
Peta statis sekarang **invarian terhadap urutan sampel** (bit-identik; GNG
bergeser sampai 0.29 m hausdorff), kualitas peta **naik** di kelima adegan uji
(QE 0.93–0.96×), paritas IK **diuji dengan `move_group` nyata** di keempat lengan
(GCS ≥ GNG di semuanya). `gng.py` tidak tersentuh — tetap baseline ablation.
⚠️ Dua temuan yang harus dibaca sebelum menulis naskah: **§B1** (klaim "peta
berubah tiap run" itu SALAH — yang benar "tidak invarian terhadap urutan") dan
**§B4** (action map, GNG maupun GCS, **kalah dari seed nol** pada benchmark IK).

**Penugasan algoritma dikonfirmasi pengguna 2026-08-13 — jangan dibuka lagi:**
MS-BL-GNG untuk **kedua** layer lingkungan; GCS untuk **action map**; index
capability **tetap grid**. "Action map" ≠ "capability map" — yang pertama memetakan
titik tugas ke `q` representatif, yang kedua menyimpan mask pose gantry per node.

➜ Prompt sesi terpisah sudah disiapkan: [p1_prompt_gcs_msbl.md](p1_prompt_gcs_msbl.md).
Berisi path sumber terverifikasi, jebakan **tiga versi `nRobot.h` yang berbeda**,
dan kendala **10 file konsumen** yang menuntut `gcs.py` API-kompatibel dengan `GNG`.

> ✅ **Utang penamaan LUNAS 2026-08-13.** Naskah menyebut MS-BL-GNG dan GCS, dan
> sekarang itulah yang berjalan. Invarian simpleks dibuktikan lewat tes yang GAGAL
> pada implementasi belum-diperbaiki (91 pelanggaran per 60 add) dan LULUS setelah
> (0). Yang tersisa: peta statis nyata belum dibangun ulang dengan kamera — angka
> Sesi C memakai **proksi** distribusi dari peta 2026-08-02.

---

## 5. Keputusan yang sudah dikunci — jangan diulang

1. **Formulasi = allocation & scheduling**, XD [ST-MR-TA]
2. **Index capability = grid.** Topo IoU 72.9% vs grid 91.4% (distribusi realistis);
   jenuh terhadap jumlah node, latihan, dan boundary pinning. Sebabnya: GNG beradaptasi
   ke **kerapatan data**, sementara error mask ditentukan **gradien mask** — tidak
   berkorelasi. Boleh diganti ke topo demi konsistensi, tapi sebut **konsistensi**,
   bukan performa.
3. **Canonical redundancy resolution** wajib — tanpa itu filter tabrakan tidak bisa
   jadi mask. Policy (`manip`/`sigmin`/`limits`/`home`/`combo`) **tidak berpengaruh**
   (±0.23 poin), pakai `manip` sebagai default standar.
4. **Reach position-only + approach ≤45°** cukup; kendala orientasi hampir gratis
   (cakupan 23.11% → 22.64%).
5. **PLAN B — kelas tugas = reach-and-dwell, bukan pick-and-place** (diputuskan
   2026-08-13). Grasp dan handover **ditunda**, dikerjakan setelah kamera eye-in-hand
   siap dan gerak lengan terbukti. Alasannya:
   - Seluruh korpus pengukuran G2 memang sudah berbasis reach (posisi + approach),
     **tidak satu pun** bergantung cengkeraman — jadi Plan B sejajar, bukan penyempitan.
   - X2–X4 (2/3/4 lengan serentak) berubah dari nyaris mustahil jadi wajar: kecemasan
     lama "grasp 70% → per-episode 0.7⁴ ≈ 24%" hilang karena reach-and-dwell ≈ 100%.
   - Klaim "hardware nyata dengan persepsi dalam loop" **tetap utuh** — yang dinilai
     keputusan penjadwalannya, bukan manipulasinya.
   - Preseden literatur: tugas Qin et al. 2024 adalah titik poles/las, **bukan** grasp.

   ⏳ **TANGGAL KEPUTUSAN GRASP: 2026-11-13** (3 bulan). Kalau sampai tanggal itu satu
   lengan belum mengangkat objek ≥5 cm dan menahannya, paper **berkomitmen** pada
   formulasi non-grasping, dan framing + target venue disesuaikan saat itu juga.
   **Jangan biarkan "menyusul belakangan" jadi terbuka tanpa batas** — tanpa handover,
   slotnya turun dari `ST-MR-TA` ke `ST-SR-TA`, yaitu **persis slot Qin et al.**

6. **Biaya setup TERKUNCI** (Sesi A, [p1_g3_timing.md §C](p1_g3_timing.md)):
   ```
   v_lin(s) = s/95.4930 mm/s      v_rot(s) = s/100.0 deg/s     s = pulse/s perintah
   T_lin(Δ) = 0.29 s + Δ_mm/v_lin  T_rot(Δ) = 0.26 s + Δ_deg/v_rot
   T_traverse = max(T_lin, T_rot)   ← KONKUREN, terukur
   ```
   Sembilan run, R² = 1.00000 semua. Rel penuh **52.8 s**; 180° = 18.3 s dan **gratis
   bila dibungkus traverse >18.3 s**. Model berskala linier ke setelan mana pun.
   ➜ Scheduler perlu menghindari perubahan **linier**, bukan rotasi.

7. **Batas rel = 1600 mm operasional** (end stop terukur ~1656 mm, Sesi A).
   `bridge.max_mm`, URDF `t*_linear_joint`, dan `workcell_full.urdf` sudah dikoreksi;
   jalur layanan `move_dual_table` sekarang punya penjaga (`_check_travel_limits`)
   yang sebelumnya **tidak ada sama sekali** — diuji 11 kasus offline + 1 kasus di
   perangkat keras nyata (menolak sebelum dispatch, nol tulisan Modbus).
   ⚠️ `jog` (op 10–13) **tetap tidak terjaga** — kontinu, tanpa target. Bukan jalur
   scheduler; sekarang memperingatkan dalam 100 mm dari batas.

8. **Definisi sukses reach-and-dwell TERKUNCI** (Sesi B, sebelum data diambil):
   **posisi < 5 mm · orientasi < 5° (sumbu approach) · ditahan 2.0 s KONTINU**,
   disampel ≥10 Hz. Satu sampel di luar toleransi **me-reset** jendela.
   **Sukses N-lengan = SATU jendela bersama** di mana semua N memenuhi toleransi
   **serentak** — bukan "masing-masing sukses di suatu titik", karena itu bisa
   dipenuhi bergantian, dan bergantian persis yang dilakukan prior work.

---

## 6. Berikutnya: Lapis 3 + 4 — scheduler

Bahan sudah lengkap:
- oracle: `capability.py`
- resource mutex: zona irisan dari `overlap` (`r = 0.20`)
- MR task: handover = irisan ketat (`r = tol`). **42.9%** pada grid target
  154-titik (`z ∈ {1.05, 1.25}`); **53.1%** pada distribusi `surface`
  (`z ∈ [1.0, 1.4]`). Dua himpunan target berbeda — bukan rentang, jangan
  dikutip sebagai "42.9–53.1%".
- setup time: ✅ **TERKUNCI dan terukur** — `T_traverse = max(T_lin, T_rot)`, lihat
  §5.6. Bukan lagi lubang di model biaya.
- ⚠️ oracle memakai `cap_g{1,2}_rail160.npz` (33×72), **bukan** peta lama 41×72

Baseline (empat): fixed assignment · greedy/nearest · sequential · MIP/DP-optimal.
Dengan MR task dan mutex, MIP/DP **tidak lagi vacuous** — enumerasi lengkap tidak
tractable, jadi ada optimality gap yang bisa dilaporkan.

Yang belum dimodelkan dan harus disebut: **tabrakan struktur gantry–gantry**
(pelat mount menyapu lingkaran r=0.4 m di `y=±0.36`, beririsan di `y ∈ [−0.04, 0.04]`);
proksi polyline meremehkan volume sapuan → semua angka rugi adalah batas bawah.

---

## 7. Disiplin kerja — baca ini sebelum menulis kode scheduler

### 7.1 Ground truth dulu, heuristik belakangan. Ini urutan wajib, bukan saran.

**Bangun MIP/DP-optimal untuk instance kecil SEBELUM heuristik apa pun ditulis.**

Alasannya bukan kerapian. Output scheduler berupa **jadwal**, dan jadwal yang
salah tetap "kelihatan masuk akal" — tidak ada sinyal error otomatis seperti
exception atau test merah. Tanpa optimum sebagai pembanding, tidak ada yang bisa
membantah heuristiknya, termasuk penulisnya sendiri. Heuristik yang ditulis lebih
dulu akan jadi jangkar: begitu ia menghasilkan angka, semua orang mulai
mempercayainya.

Urutan yang benar:

1. generator instance kecil (2–6 tugas) yang bisa dienumerasi tuntas
2. solver exact → optimum, jadi ground truth
3. baru heuristik, dilaporkan sebagai **optimality gap** terhadap (2)
4. baru baseline pembanding: fixed assignment · greedy/nearest · sequential

### 7.2 Ukur, jangan menduga

Sepanjang sesi G2, prediksi asisten **salah empat kali berturut-turut**:
interference akan mengikat (ternyata 0.00%); policy canonical akan sensitif
(ternyata ±0.23 poin); zona-eksklusi pembacaan longgar akan mengikat (ternyata
tidak, hanya pembacaan dua-arah yang mengikat); index topologis akan menang pada
distribusi realistis (ternyata kalah 18 poin).

Keempatnya ketahuan **hanya karena diukur**, bukan karena dipikirkan lebih keras.
Semuanya juga meleset ke arah yang sama — menduga kendala lebih mengikat daripada
kenyataannya.

Konsekuensi praktis: kalau sebuah angka bisa diputuskan dengan pengukuran, ukur.
Jangan berdebat, dan jangan menulis kesimpulan berdasarkan intuisi geometris.

### 7.3 Dua jebakan yang paling mungkin berikutnya

**(a)** ~~Waktu traverse gantry belum pernah diukur — dan parameternya ambigu 33×.~~

> ✅ **SUDAH SELESAI 2026-08-13 (Sesi A). Seluruh sub-bagian (a) di bawah adalah
> CATATAN SEJARAH, bukan pekerjaan tersisa.** Jawabannya: cabang lambat benar,
> **31.416 mm/s**, rel penuh **52.8 s**. Model TERKUNCI di §5.6.
> Pembingkaian "ambigu 33×" itu sendiri **keliru** — 1047 mm/s tidak pernah jadi
> kandidat kecepatan; ia 100 000 pulse/s² yang salah dibaca sebagai pulse/s
> ([p1_g3_timing.md §A1](p1_g3_timing.md)).

Di formulasi lama ia tidak relevan. Di formulasi baru ia **inti biaya**: seluruh
objektif throughput bergantung padanya, dan `sequence-dependent setup time` adalah
separuh alasan masalah ini sulit.

Yang bisa diturunkan dari `dual_table_controller.py` (2026-08-12, **belum diukur
di hardware**):

```
PULSES_PER_MM  = 12000/(40·π) = 95.493      PULSES_PER_RAD = (9000/90)·(180/π) = 5729.6

bridge.linear_speed  = 3000   pulse/s ->   31.4 mm/s   -> rel 2.0 m  =  63.7 s
bridge.rotate_speed  = 1000   pulse/s ->   10.0 deg/s  -> 180 deg    =  18.0 s
motor_config.speed   = 100000 pulse/s -> 1047.2 mm/s   -> rel 2.0 m  =   1.9 s
motor_config.acceleration = 1000 (satuan driver Oriental Motor, belum dipastikan)
```

**Selisihnya 33×, dan itu mengubah seluruh bentuk masalah.** Kalau traverse 1.9 s,
gerak gantry murah dan scheduling pada dasarnya soal penugasan. Kalau 63.7 s, gerak
gantry **mendominasi segalanya** — satu traverse rel penuh lebih mahal dari banyak
operasi pick, dan objektifnya praktis menjadi "minimalkan perpindahan gantry",
dengan koordinasi = membatch tugas yang berbagi satu pose.

Penelusuran kode: `bridge.linear_speed` / `bridge.rotate_speed` yang mengalir ke
perintah gerak nyata (`req.linear_speed` → `move(linear_speed=…)`, baris 420-421,
487-488, 527+), sementara `motor_config.speed` hanya dikirim ke `configure_motor`
saat startup (baris 304) — kemungkinan register kecepatan operasi driver, dan belum
jelas apakah ia membatasi atau ditimpa per-perintah.

**Estimasi terbaik saat ini: 31.4 mm/s dan 10 deg/s.** Tapi ini nilai default
terkonfigurasi, bukan hasil pengukuran.

➜ **Selesaikan ini SEBELUM objektif scheduler dirancang**, dengan stopwatch di
hardware (satu perintah `move_dual_table` ujung-ke-ujung sudah cukup). **Jangan
mengarang konstanta.**

> 🔧 **Rentang sudah dipersempit oleh analisis mekanik ([p1_g2_results.md §15](p1_g2_results.md)):
> gunakan ~8–64 s, bukan 1.9–64 s.** Linier pada 31.4 mm/s sangat konservatif
> (E_kin 9.7 mJ) dan wajar dinaikkan ke ~250 mm/s → ~8 s. Rating motor 1047 mm/s
> (1.9 s) tidak perlu dan meragukan untuk ruang berpenghuni. **Rotasi sebaliknya
> sudah di ambang** — 10 °/s menghasilkan 244 mm/s di ujung lengan (lengan tuas
> 1.4 m dari sumbu), praktis tepat di figur reduced-speed 250 mm/s.
>
> ⚠️ **Dan setup cost lebih besar dari sekadar traverse:** lengan terentang sudah
> memakai 98.2% batas torsi hanya untuk menahan diri, jadi lengan **harus dilipat**
> sebelum gantry berakselerasi. Biaya pindah = **lipat + traverse + rentang-ulang**,
> dan itu menghentikan **kedua** lengan pada gantry itu. Modelkan begitu, jangan
> hanya traverse.

**(b) Tabrakan struktur gantry–gantry masih terbuka.**
Yang sudah dicek baru lengannya. Pelat mount menyapu lingkaran radius 0.4 m di
`y = ±0.36`, jadi beririsan di `y ∈ [−0.04, 0.04]` — dua gantry pada `x` yang
berdekatan dengan rotasi saling menghadap **bisa** bertabrakan secara struktur.
Ini batasan atas `(lin₁, rot₁, lin₂, rot₂)` yang tidak bergantung target sama
sekali, jadi ia memangkas ruang jadwal secara langsung. Kalau scheduler dibangun
tanpa ini, jadwalnya bisa memerintahkan konfigurasi yang merusak hardware.

---

## 8. Grasp — bukan lagi blocker hardware, tapi ber-tenggat

**Status:** satu lengan belum pernah mengangkat objek ≥5 cm dan menahannya. Kamera
eye-in-hand sedang disiapkan.

Sejak Plan B (§5.5), grasp **tidak lagi memblokir pekerjaan hardware** — reach-and-dwell
bisa jalan sekarang. Tapi ia ber-tenggat **2026-11-13**, karena handover (yang butuh
grasp) adalah satu-satunya hal yang menjaga slot tetap `ST-MR-TA`.

Catatan teknis eye-in-hand: infrastruktur charuco sudah ada (`charuco_common.py`,
`calibrate_extrinsics.py`, `generate_charuco_board.py`), tapi eye-in-hand butuh
kalibrasi **`AX = XB`** antara flange dan kamera — bukan kamera-ke-dunia seperti yang
sekarang. Deteksi board bisa dipakai ulang. Waspadai kelas kegagalan disambiguasi
flip 180° yang pernah kena di kalibrasi ekstrinsik RGBD.

---

## 8b. ✅ SUDAH DIKUNCI 2026-08-13 (Sesi B) — sebelum data apa pun diambil

**Definisi sukses tugas** (detail + pembenarannya:
[p1_g4_reach_dwell.md §A1](p1_g4_reach_dwell.md)):

```
posisi   : ‖p_tool − p_cmd‖         <  5 mm
orientasi: ∠(a_tool, a_cmd)         <  5°     (sumbu approach, roll bebas)
dwell    : 2.0 s KONTINU, ≥10 Hz    (satu sampel di luar toleransi = RESET)
```

Ambangnya dipilih dari sumber galat **terukur**, bukan selera: skew rel 1.24 mm,
resolusi enkoder 0.0105 mm, toleransi controller ±50 pulse = 0.52 mm, galat sendi
0.0001 rad. Jadi gagal pada 5 mm bermakna **metode**, bukan gantry.

**TIGA toleransi, tiga arti — pisahkan di naskah:**

| Lapis | Angka | Artinya | Dipakai untuk |
|---|---|---|---|
| **L1** peta kapabilitas | **5 cm** | "ada solusi di sekitar sini" | membangkitkan kandidat pose gantry |
| **L2** akurasi eksekusi | **< 5 mm** | pose **perintah** → pose **tercapai** | definisi sukses tugas |
| **L3** akurasi persepsi | **3–5 cm** | objek **nyata** → pose **perintah** | dilaporkan, **tidak** masuk kriteria sukses |

Kalau tidak dipisahkan, reviewer membaca "menjangkau dalam 5 cm" dan itu terdengar
lemah — padahal 5 cm adalah L1, yang memang tidak pernah dimaksudkan sebagai akurasi.

> ⚠️ **Batasan L2 yang WAJIB disebut:** diukur dari TF `world → t*_a*_tool_frame`,
> yaitu **FK dari sendi TERUKUR**. Menangkap galat servo, **tidak** menangkap galat
> kalibrasi kinematik/mounting. Akurasi absolut butuh pengukuran eksternal dan
> **tidak diklaim**.
>
> L3 diverifikasi ulang 2026-08-13: kesepakatan lintas-kamera pada sentroid board
> **2.9 cm — lebih baik dari 3.1 cm saat kalibrasi**. Solve segar dijalankan lalu
> **DITOLAK** (RMS reproyeksi lebih baik, kesepakatan lintas-kamera lebih buruk).
> **RMS reproyeksi tidak bisa dipakai memilih kalibrasi** — ia hanya konsistensi-diri
> terhadap pose board yang kita ketikkan sendiri.

---

## 8c. Urutan kerja hardware

| # | Isi | Status |
|---|---|---|
| **0** | bring-up + verifikasi prasyarat fisik + instrumen | ✅ **SELESAI** Sesi B |
| **1** | Ukur waktu traverse (+ lipat/rentang) | ✅ **SELESAI** Sesi A — model TERKUNCI (§5.6) |
| **2** | Satu lengan reach-and-dwell ke target terpersepsi | ⬜ **BERIKUTNYA** |
| **3** | Dua lengan se-gantry — kopling geser-π terlihat | ⬜ |
| **4** | Tambah gerak gantry antar tugas — setup cost nyata | ⬜ |
| **5** | Dua gantry, empat lengan | ⬜ |

**Yang sudah diverifikasi di langkah 0** (dibaca, bukan diterima —
[p1_g4 §B3](p1_g4_reach_dwell.md)): kedua port USB · `enp112s0` = 192.168.2.100/24 ·
4 lengan menjawab ICMP · 2× D455 USB3, serial cocok · kedua gantry init tanpa galat
maupun alarm · **posisi enkoder aktual: g1 = 550.009 mm, g2 ≈ 0** (keempatnya
bilangan bulat pulse persis → bacaan enkoder sungguhan).

> 🔴 **BLOCKER langkah 2 saat ini: lengan dilepas fisik** (2026-08-13, untuk
> membebaskan pandangan `rgbd2` ke board saat kalibrasi). Tanpa lengan tidak ada
> `t1_a1_tool_frame`, jadi L2 tidak bisa diukur.
>
> **Saat memasang ulang:** pemasangan bisa menggeser pose mounting. L2 berbasis FK
> **tidak akan melihat** galat itu, tapi klaim akurasi absolut mana pun mewarisinya.
> Yang jadi basi adalah **corner-reference 2026-07-25**
> (`corner_world_xyz = (-0.151, 0.181, 2.023)`), **bukan** ekstrinsik kamera.
>
> Board juga masih ada di dalam ruang kerja lengan — keluarkan sebelum langkah 2,
> kecuali sengaja dipakai sebagai target persepsi.

---

## 9. Dokumen

| File | Isi |
|---|---|
| **`p1_state.md`** | ini — status & handoff. **Baca pertama.** |
| `p1_g2_results.md` | pengukuran G2, §1–§12, dengan batasan tiap angka |
| `p1_g3_timing.md` | Sesi A — biaya setup gantry, **TERKUNCI** di §C |
| `p1_g4_reach_dwell.md` | Sesi B — definisi sukses (§A1), instrumen + validasi (§B0), fix batas rel (§B1), sapuan ulang (§B2), prasyarat fisik (§B3), kalibrasi (§B4b) |
| `p1_g5_msbl_gcs.md` | Sesi C — port MS-BL-GNG + GCS: kriteria terkunci (§A), hasil terukur (§B) |
| `p1_prompt_gcs_msbl.md` | prompt sesi port MS-BL-GNG + GCS — **sudah dieksekusi**, lihat `p1_g5_msbl_gcs.md` |
| `p1_plan.md` | ⚠️ §1/§3-lapisan/§4 stale. Sah: §2b–§2e, §3 utang teknis, §6, §7 |

### Catatan §7.2 — papan skor dugaan

Sampai 2026-08-13: **delapan dugaan meleset, semuanya ke arah yang sama** — menduga
kendala lebih mengikat daripada kenyataannya (interference, policy canonical,
zona-eksklusi, index topologis, dan di Sesi B: kalibrasi RGBD "memburuk" ternyata
tidak, dan kolom rel hantu ternyata hampir tidak menghapus target).

**Ke-8 (Sesi C):** "peta statis berubah tiap run karena GNG belajar online".
Diukur: **tidak** — `gng.py` ter-seed, jadi masukan identik memberi peta
bit-identik. Yang gagal adalah invariansi terhadap **urutan** sampel. Polanya
sama persis: masalahnya nyata, tapi **lebih sempit** dari dugaan. Dan seperti
dugaan yang tepat di Sesi A, jawabannya datang dari **membaca jalur data kode**
(`params.seed` → `default_rng`), bukan dari menakar.

Satu dugaan yang **tepat** (waktu traverse, Sesi A) adalah satu-satunya yang
diturunkan dari **jalur data kode**, bukan dari intuisi geometris. Itu pembeda yang
layak dipakai: telusuri variabelnya, jangan menakar keketatannya.
