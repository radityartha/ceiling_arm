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

> ✅ **Langkah §7.1 nomor 1 dan 2 SELESAI 2026-08-14 (Sesi G7)** —
> [p1_g7_sched.md](p1_g7_sched.md). Model penjadwalan terkunci (§A1), generator
> instance + solver exact ada di `reachability_gng/sched.py`, dan
> **exactness-nya dibuktikan lewat V0–V4** (`test/verify_sched_exact.py`),
> termasuk brute-force independen dan 8 instance patologis. Batas tractable
> terukur: **n ≤ 10 tugas, 4 lengan, |P| = 2376 penuh, 84 s, 500 MB.**
>
> 🔴 **Dua hasil G7 yang mengubah cara §2 boleh dikutip — baca sebelum menulis
> naskah:** (a) mutex `r = 0.20` **tidak menggerakkan makespan sama sekali**
> (0.000 s pada 105 pasang solve), jadi "selisih 25 poin itu yang dibeli
> scheduling" sah sebagai klaim **kelayakan**, **bukan** klaim waktu
> ([p1_g7 §B3](p1_g7_sched.md)); (b) **74–92% makespan adalah gerak gantry**,
> jadi masalahnya tur pose, bukan penugasan lengan ([p1_g7 §B5](p1_g7_sched.md)).
>
> ✅ **Langkah §7.1 nomor 3 dan 4 SELESAI 2026-08-14 (Sesi G8)** —
> [p1_g8_sched2.md](p1_g8_sched2.md). Heuristik `pose-tour` + tiga baseline +
> batas bawah ada di `reachability_gng/sched_heur.py`, kerangka evaluasi di
> `test/eval_sched_heur.py`. **Seluruh §7.1 (1–4) kini selesai.**
>
> 🔴 **Empat hasil G8 yang mengubah cara §2/§6 boleh dikutip:**
> (a) **ambang "cukup baik" TIDAK tercapai** — `pose-tour` mean gap **2.45%**
> (lolos ≤ 5%) tapi max **24.31%** (gagal ≤ 10%), pada 140 instance dan
> direplikasi pada 60 instance held-out. Ia persis optimal pada 51% instance dan
> 1 300× lebih cepat dari exact, tapi **ambangnya dilaporkan GAGAL**, bukan
> dinaikkan ([p1_g8 §B2](p1_g8_sched2.md));
> (b) **urutan tur menyumbang 0.00%** pada 40/40 instance `n` = 8…50 — yang
> menentukan adalah **pose mana** yang dipilih, bukan urutannya. Ini
> **membatasi** `p1_g7 §B5` ([p1_g8 §B6](p1_g8_sched2.md));
> (c) **"74–92% makespan adalah gerak gantry" adalah pernyataan `n ≤ 10`.** Di
> `n = 50` pangsanya **59.5%**. Kutip selalu dengan `n`-nya
> ([p1_g8 §B7](p1_g8_sched2.md));
> (d) ekor gap = **satu pose handover yang salah dipilih** (+9.55 s traverse),
> bukan tur yang salah diurutkan ([p1_g8 §B3](p1_g8_sched2.md)).
>
> ⚠️ `validate_schedule()` **eksponensial** dalam tugas per perhentian; ia
> menggantung di `n ≥ 20`. Baca [p1_g8 §B4](p1_g8_sched2.md) sebelum menulis
> evaluasi apa pun di atas `n = 10`.
>
> ⏳ **Sesi G9 (2026-08-14) — [p1_g9_sched3.md](p1_g9_sched3.md). Tabrakan
> struktur gantry–gantry DIMODELKAN, tapi GROUND TRUTH-nya BELUM TEGAK.**
>
> 🔴 **Gerbang W0–W4 §A3-K1 TIDAK LULUS**, jadi `sched_coll.solve_coupled`
> **tidak boleh disebut ground truth** dan §7.1 melarang menyentuh heuristik
> sadar-tabrakan. W2 (enumerator bersama brute force, satu-satunya uji yang
> menyerang himpunan waktu-mulai) **tidak dibangun**; W2b **gagal** (dengan
> tabrakan dimatikan, B&B memberi 46.7772 di mana `solve_exact` memberi
> 44.7772); W4 **tidak bisa dijalankan** karena `validate_schedule()` tidak
> punya tempat untuk waktu **tunggu** yang model baru tambahkan.
>
> ✅ **Yang tegak dan boleh dipakai:** predikat `BLOCK` (jejak URDF exact,
> reduksi 2-D terbukti; W0 1 184 kasus 0 mismatch, W0b 2.2e-16 vs
> `irm_sweep.base_pose`), lintasan traverse + sertifikat waktu kontinu
> (W1a–d lulus; durasi cocok `sched.traverse_time` sampai 3.6e-15 s),
> Lemma 1 (parkir aman `rot = 0`, margin 0.21 m), Lemma 3
> (`terkopel ≥ takterkopel`, batas bawah yang sah).
>
> 🔴 **Hasil yang mengubah cara §6 dan §7.3(b) boleh dikutip: pada 13 dari 40
> instance S1 (32.5%), jadwal yang G7/G8 sebut OPTIMUM benar-benar
> melanggar kendala tabrakan.** Jadi "semua angka rugi adalah batas bawah"
> bukan lagi kehati-hatian teoretis — ia punya dukungan kuantitatif.
> Pada 27 dari 40 sisanya `Δ = 0` **exact**.
>
> ⚠️ **Vonis "berapa mahal koordinasi" TIDAK DAPAT DITENTUKAN sesi ini**, dan
> subset yang bisa dibuktikan **bias ke bawah menurut konstruksi** (ia
> didefinisikan oleh "optimum takterkopelnya bebas tabrakan", yaitu oleh
> `Δ = 0`). **Dilarang mengutip mean `Δ` atas subset itu sebagai biaya
> koordinasi** — baca [p1_g9 §B6](p1_g9_sched3.md).
>
> ⛔ **TIDAK DIUKUR di G9**, dan tidak boleh diasumsikan selamat: apakah mutex
> `r = 0.20` masih 0.000 s pada model terkopel; apakah urutan tur masih
> menyumbang 0.00%; sapuan `c_clear`; probe adversarial S2.
>
> ➜ Prompt G10 siap pakai di [p1_g9_sched3.md §C](p1_g9_sched3.md). Urutannya
> sengaja terbalik dari G9: **oracle dulu** (gerbang sadar-tunggu + W2), solver
> belakangan.

> ✅ **Sesi G21 (2026-09-22, offline) — [p1_g21_sched_tfold.md](p1_g21_sched_tfold.md).
> Scheduler di bawah biaya setup TERUKUR.** Semua evaluasi G7–G15 memakai
> `t_fold = 0` (biaya pindah = traverse saja). G21 menurunkan `t_fold` dari
> g19 §B1.2 dan mengunci sebelum solve: **FISIK 50.80 s** (gerak retract 29.02 +
> eksekusi extend 12.50 + 9.28 — **nilai yang boleh dikutip**) dan **DINDING
> 126.80 s** (apa adanya; ~76 s-nya artefak perangkat lunak kita, **tidak**
> dikutip sebagai sifat sel). Dibebankan per perubahan pose **per gantry**
> (diperiksa di kode). Exactness `solve_exact` dibuktikan ulang di ketiga
> `t_fold` (V0–V4 + P8/P9, kontrol mutan tertangkap); C0 mereproduksi arsip g8
> 700/700.
>
> 🔴 **Empat hasil G21 yang mengubah cara §6 boleh dikutip:**
> (a) **optimum di FISIK dan DINDING IDENTIK pada 140/140** — pada biaya
> terukur masalahnya **minimum-jumlah-pindah**, traverse hanya pemecah seri.
> 61.7 % makespan adalah `t_fold`, traverse 32.8 %, dwell 5.5 %
> ([p1_g21 §B2](p1_g21_sched_tfold.md));
> (b) `pose-tour` mean gap **1.01 %** (110/140 persis optimal) tapi max
> **47.1 %** = satu pindah ekstra; **K1 tetap GAGAL**, ekornya 2× lebih buruk.
> Baseline miopik runtuh (`greedy` 78 %) (§B3);
> (c) urutan tur tetap **0.00 %** (40/40) — g8 §B6 bertahan; pangsa gerak
> gantry di `n = 50` **82.3 %**, bukan 59.5 % (§B4–B5);
> (d) mutex `r = 0.20` berbiaya **pertama kali**: +2.0 s pada 1/80 pasang
> (§B6). "0.000 s pada setiap pasang" = pernyataan `t_fold = 0`.
> Koreksi tertulis di g7 §B3/§B5, g8 §B2/§B6/§B7. Semua tetap batas bawah
> (tabrakan g9 di luar model). `t_fold` ∈ (0, 50.8) **tidak diukur**.
>
> ⚠️ Penyaring torsi `joint_2` (g20 §C, keputusan operator): distribusi
> offset terukur − RNEA atas 81 rencana g18–g20 di [p1_g21 §B8](p1_g21_sched_tfold.md)
> — maks **+7.15**, offset **naik dengan prediksi**; usulan `RNEA + 7.7 ≤ 14`.
> Kode penyaring **tidak** diubah.

> 🔴 **Sesi G22-HW (2026-09-22) — [p1_g22_hw.md](p1_g22_hw.md). Jadwal
> keluaran scheduler TIDAK DAPAT DIEKSEKUSI; makespan terukur TIDAK diukur.**
> Aturan instance dikunci sebelum solve (`gen_real` n = 6, mr = 0, rot = 0,
> p0 = rel terbaca, ≥ 1 pindah per gantry → 35/50), lalu saringan plan-only
> seluruh jadwal di stack nyata (torsi RNEA + 7.7, S18, sapuan rel S24):
> **0/35 lolos**. Per rencana 14/49 (29 %) PLANNED; di pose **sesudah pindah
> 0/13** (12 NO-PLAN di tepi jangkauan dengan approach vertikal). Sebabnya:
> optimum = min pindah (G21) → solver pindah **hanya** untuk tugas di pinggir
> peta L1, dan L1 (5 cm, approach ≤ 45°, tanpa torsi) ≠ L2. **Klaim yang boleh
> ditulis:** *oracle kapabilitas L1 menghasilkan jadwal optimum yang tidak
> dapat dieksekusi pada 35/35 instance; penolakan terkonsentrasi di perhentian
> yang membenarkan pindah gantry.* Nol gerak. Berikutnya: oracle L2-torsi
> (prompt G23, [p1_g22 §D](p1_g22_hw.md)).

> 🔴 **Sesi G23 (2026-09-22) — [p1_g23_oracle_hw.md](p1_g23_oracle_hw.md).
> Oracle L2-torsi dibangun dan divalidasi; jadwal TETAP tidak dapat dieksekusi
> (0/2); makespan terukur TIDAK diukur.** Oracle′ = L1 ∧ IK approach vertikal
> (< 2 mm, < 2°) ∧ torsi statis + offset + margin aman di **semua** solusi IK
> yang ditemukan. Margin dari rencana g18–g20 di konfigurasi akhir **terukur**:
> `m_2` **0.66**, `m_3` 0.82 (aturan terkunci pertama cacat: memakai cabang
> torsi-terendah → 5.91; diganti sebelum validasi, keputusan operator, g23 B0).
> Validasi pada 49 rencana G22: NO-PLAN **23/23** dan TORQUE-UNSAFE **11/12**
> ditolak, presisi 4/5, recall 4/14. Instance `gen_real` apa adanya → (i)
> **2/50**; saringan (iv) **0/2** — keduanya TORQUE-UNSAFE di cabang IK yang 8
> benih lewatkan. Sebabnya: **roll bebas ⇒ solusi IK adalah kurva 1-D**; ≥ 30 %
> tuple yang diterima punya titik tak aman di sana (64 benih). Nol gerak.
> Berikutnya: sapuan roll eksplisit (prompt G24, [p1_g23 §D](p1_g23_oracle_hw.md)).

> ⏳ **Sesi G24 (2026-09-22/23, OFFLINE — lengan mati, operator tidak di lokasi)
> — [p1_g24_roll_oracle.md](p1_g24_roll_oracle.md). Oracle‴ tervalidasi; 45/50
> instance (i)–(iii); saringan (iv) dan eksekusi = G24b.** Oracle″ (sapuan roll
> eksplisit, 72 roll) **gagal** gerbangnya (NC7 88/90, PC2 70/81), identik di
> grid 2.5° → sebabnya **kemiringan ≤ 2° perencana** + **singularitas
> pergelangan** (q5 ≈ 0: j4/j6 kontinum → hitung solusi tak jenuh), bukan grid.
> Oracle‴ (koreksi pasca-data, operator): + 8 kemiringan per solusi, jenuh atas
> amplop torsi → **semua kontrol lulus** (NC6 2/2, NC7 90/90, PC2′ 80/81); V:
> terima 2 PLANNED, 0 TORQUE/NO-PLAN (presisi 2/2, recall 2/14). **Tugas kini
> ditarik dari node layak-oracle‴** (keputusan operator; konsekuensi naskah di
> g24 A5): laju terima 33.7 %, ~35 % node peta layak, (i)–(iii) **45/50**, rerata
> 3.96 pindah. ⚠️ 19 % Newton-miring tidak konvergen dan tidak dipilah — sumber
> terima-palsu paling mungkin di (iv). Nol gerak. Berikutnya: G24b ([g24 §D](p1_g24_roll_oracle.md)).

> 🟢 **Sesi G24b (2026-09-23, sel NYATA) — [p1_g24_roll_oracle.md §E](p1_g24_roll_oracle.md).
> JADWAL SCHEDULER PERTAMA yang dijalankan UTUH di sel nyata: seed 1, 6/6 tugas
> sukses, kedua rel bergerak, makespan terukur 480.44 s.** (iv): seed 0 ditolak
> (t5 arm_1 RNEA j2 7.17 → 14.87, terima-palsu oracle‴), **seed 1 lolos 3/3**
> (g1 0.55→0.70, g2 0→**1.45**; 3+3 tugas; model FISIK 103.2 s). Tahap 2: gerak rel
> g2 **pertama di proyek**, 0→1.45→0, galat −0.58/+0.56 mm. Tahap 3 vs prediksi
> terkunci: ×2.91 P1-serial, ×2.37 P2-serial, ×1.36 P4-serial; suku terbesar =
> **tugas** (Σ 273 s = 57 %: rencana 12–30 s + eksekusi per tugas, model 2 s),
> lalu retract 52.7 s (> DINDING 48.8), rel ≈ T_cmd (debounce g19). Torsi puncak
> j2 9.62 < 12. D98–D101 4/4 tepat. Berikutnya: G25 offline — model biaya tugas terukur ([g24 §F](p1_g24_roll_oracle.md)).

> 🟡 **Sesi G25 (2026-09-23, OFFLINE) — [p1_g25_task_cost.md](p1_g25_task_cost.md).
> Model biaya TERUKUR P1′ memprediksi makespan G24b −4.8 % (P2 lama −57.9 %); optimum
> tetap minimum-jumlah-pindah (44/45).** Dekomposisi cap waktu (G19/G20 kalibrasi, G24b
> uji): **perencana MoveIt 0.04 s; "rencana 12–30 s" = saringan antar-lengan** (65–71 %
> tiap tugas, ∝ panjang lintasan, ≈ 35 % makespan G24b — perangkat lunak kita). Model
> dikunci sebelum prediksi (median, G19+G20 saja): `c_task` 43.65, `c_ret` 50.87, rel
> 0.9502·`T_cmd` + 4.74, lengan serial, retract dilewati pada keberangkatan pertama bila
> lengan REST. G24b: **457.39 vs 480.44**; per tugas |galat| rerata 7.9 s (−17…+57 %) —
> total dekat sebagian pembatalan, n = 6. `sched.py` diparameterisasi (default bit-identik:
> V0–V4, G21 E1 420/420, G24 100/100; G0′ brute force 192/192, 3 mutan tertangkap).
> 45 instance: jadwal lama tidak optimum **10/45**, regret mean 1.8 % / maks **19.9 %**;
> 5 besar semuanya **bias K2** (tugas di `p0` dulu — optimum P1′ meninggalkan `p0` tanpa
> retract). **Biaya tugas (53.5 % makespan) tidak menggeser satu pun optimum: 0/270 tugas
> layak di dua gantry** → alokasi terpaksa. Berikutnya: G26-HW replikasi, `p0` = 0/0
> ([g25 §D](p1_g25_task_cost.md)).

> 🟢 **Sesi G26-HW (2026-09-23, sel NYATA) — [p1_g26_hw.md](p1_g26_hw.md). Replikasi: 3 seed
> jadwal P1′ dijalankan utuh, 18/18 tugas SUCCESS; model P1′ (terkunci, tidak dikalibrasi ulang)
> meleset −11.9 / −0.4 / −5.6 % (gabungan −5.9 %), selalu di bawah terukur.** `p0` = rel terbaca
> 0/0; (i)–(iii) 46/50 (KG1 45/45 bit-identik); (iv) **3/14** lolos (seed 1, 2, 13). Makespan
> 557.69 / 585.59 / 550.59 s; P1 lama −39…−58 %. Bias di suku **non-tugas** (−23 s/seed: overhead
> `rail_to_g` 6.8–9.1 vs 4.74; retract dilewati tetap 2.5 s, model 0); suku tugas hampir tak bias
> dalam jumlah tetapi \|galat\| rerata **8.2 s** (lintasan pendek +15…+21, panjang −5…−15). Torsi
> j2 ≤ 9.66. Rel g1 ke **1.50 m** pertama kali (galat −0.85 mm). 🔴 **Pasca-data: lolos (iv) ⟺ tanpa
> tugas z = 1.40** (14/14; 11/11 penolakan TORQUE-UNSAFE di z = 1.40; 29/46 seed memuatnya) —
> terima-palsu oracle‴ terkonsentrasi di lapis teratas. Klaim "dapat dieksekusi" = subset lolos (iv).
> Semua G22–G26 = rel saja (R0 / S23). Berikutnya: G27 offline — diagnosis z = 1.40 ([g26 §D](p1_g26_hw.md)).
> 🔴 **Utang dikunci operator ([g26 §C1](p1_g26_hw.md)):** sesudah G27 → **ROTASI gantry** (belum pernah di jalur
> jadwal); lalu **peta lingkungan 3D** — octomap updater `move_group` gagal dimuat di setiap bring-up, jadi
> G22–G26 tidak punya penghindaran tabrakan lingkungan (pengaman = sel dikosongkan operator).

> 🟡 **Sesi G27 (2026-09-23, OFFLINE) — [p1_g27_z140_diag.md](p1_g27_z140_diag.md). Terima-palsu z = 1.40
> DIDIAGNOSIS: puncak torsi di TRANSIT dari REST, bukan di titik akhir.** Langkah 1 (instrumen, semua saringan
> g22–g26 + eksekusi): tuple diterima oracle‴ di z < 1.40 **145/145** aman (RNEA ≤ 5.49); di z = 1.40 **11/16**
> TORQUE, RNEA bimodal pada tuple yang **sama** (2.4–3.7 vs 6.4–8.1). Hipotesis + aturan adopsi dikunci sebelum
> uji. **T0:** maks gravitasi `joint_2` atas seluruh daerah tujuan perencana (2 mm, 2.83°, roll bebas) ≤ 5.83 <
> RNEA − m₂ pada **11/11** → bukan cabang (T2: 5× solusi, amplop +0.000), bukan miring (T1: −0.001), bukan model
> (2.7e-15). **T3:** proksi garis lurus REST → q_akhir tepat di C′ (RNEA − P maks +0.054); di z ≤ 1.32 **0/21 027**
> solusi menembus, di z = 1.40 **1735/1735** (+1.70 N·m di atas titik akhir). Oracle⁗ = oracle‴ ∧ PATH
> (`m₂ᵖ` 0.054) **≡ oracle‴ tanpa lapis z = 1.40** pada 27 659 tuple (53 → 0; lapis lain 100 % tetap; K0 1510/1510
> bit-identik). Prediksi (iv): **17/46** seed = persis seed tanpa z = 1.40; 14 seed baru untuk G28. P1″ LOSO
> \|galat\| 4.78 % (P1′ 5.69 %; `c_rovh″` 7.70, `c_skip` 2.50). ⚠️ Hipotesis dipilih pada set yang sama →
> validasi sejati = **V28** (61 tuple tak pernah direncanakan, prediksi terkunci) di G28 plan-only
> ([g27 §D](p1_g27_z140_diag.md)). Rute terlipat aman ada (5/16) — oracle⁗ membuang z = 1.40, tidak
> menyelamatkannya. Sesudah G28: ROTASI (g26 C1-2), lalu peta 3D (C1-3).

> 🟡 **Sesi G28-OFF (2026-09-23, offline) — [p1_g28_hw.md](p1_g28_hw.md).** §A dikunci (V28 3 sampel/tuple
> dari REST, lintasan disimpan; D133–D143). `v28_screen.py` (penyadap pass-through, RNEA per titik bit-identik
> dengan probe: K-RNEA 200/200) + `v28_score.py`; DRY lulus; smoke di stack **mock** pada 3 tuple **dev**:
> puncak TORQUE interior, lintasan ≈ garis lurus ruang-sendi, rencana aman z = 1.40 berakhir di cabang j2-rendah
> (statis 1.3–1.7 vs 5.3–5.6) → D144–D145 (berbasis-dev). V28 + (iv) menunggu operator: prompt [g28 §D](p1_g28_hw.md).

> ➜ Urutan kerja konkret + prompt sesi siap-pakai:
> [p1_next_steps.md](p1_next_steps.md) §1 Jalur A dan §3.
> 🔴 Perhatikan §0 di sana: **lengan sedang dilepas fisik**, jadi seluruh jalur
> hardware (§8c langkah 2–5) terblokir dan jalur offline adalah jalur utama.

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

~~Yang belum dimodelkan dan harus disebut: **tabrakan struktur gantry–gantry**
(pelat mount menyapu lingkaran r=0.4 m di `y=±0.36`, beririsan di `y ∈ [−0.04, 0.04]`).~~

> 🔻 **KOREKSI, 2026-08-14 (G9).** Kalimat di atas dipakai berulang sebagai
> **predikat tabrakan**, dan dalam peran itu ia **SALAH**. Lingkaran `r = 0.4`
> adalah **sapuan atas SELURUH rotasi** — ia menjawab "bisakah mereka bertemu
> sama sekali", bukan "apakah pose ini bertabrakan". Dipakai per-pose ia
> melarang sebagian besar ruang pose secara palsu.
>
> Predikat yang benar ada di `sched_coll.py` dan
> [p1_g9 §A2.2](p1_g9_sched3.md), diturunkan dari URDF apa adanya: jejak tiap
> gantry = **OBB batang `0.80 × 0.08` ∪ dua cakram pelat `r = 0.055`**, dan
> `BLOCK(p1, p2) ⟺ jarak_XY ≤ c_clear`. Ambang penting yang sudah terukur:
> pada `rot = ∓90°`, tabrakan terjadi untuk `|Δlin| ≤ 0.095 m` (**bukan**
> 0.11 — 0.11 adalah syarat **perlu**, dan memakainya sebagai syarat **cukup**
> adalah kesalahan pertama G9, [p1_g9 §B1](p1_g9_sched3.md)).
>
> Syarat perlu yang berguna dan sah: tabrakan menuntut **kedua** gantry
> berputar > 31.7° dari sejajar-rel. Gantry di `rot ≈ 0` atau `≈ 180°`
> **mustahil** ditabrak.

Tabrakan **lengan–lengan** antar gantry tetap di luar model, dan itu sekarang
lubang yang lebih besar: ujung lengan terentang 1.4 m dari sumbu rotasi, tiga
kali `R_MAX = 0.455 m` struktur yang G9 modelkan. **Semua angka rugi tetap
batas bawah.**

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

**(b)** ~~Tabrakan struktur gantry–gantry masih terbuka.~~

> ✅ **PREDIKATNYA SUDAH DIMODELKAN DAN DIVERIFIKASI 2026-08-14 (Sesi G9)** —
> `sched_coll.py`, [p1_g9 §A2.2 dan §B1](p1_g9_sched3.md). Yang **belum** tegak
> adalah solver terkopelnya, bukan predikatnya (§6).
>
> 🔻 **Kalimat asli di bawah memakai lingkaran sapuan `r = 0.4` sebagai
> predikat per-pose, dan itu SALAH** — sama dengan koreksi di §6. Lingkaran itu
> adalah gabungan atas semua rotasi.
>
> Yang terbukti benar dari paragraf asli: ia memang **batasan atas
> `(lin₁, rot₁, lin₂, rot₂)` yang tidak bergantung target sama sekali**, dan ia
> memang **memangkas ruang jadwal secara langsung**. Terukur: pada **13 dari 40**
> instance 2-gantry alami, jadwal optimum G7/G8 melanggarnya. Peringatan
> terakhirnya juga terbukti tepat — **jadwal G7/G8 memang bisa memerintahkan
> konfigurasi yang merusak hardware, dan pada sepertiga instance ia melakukannya.**

Yang **masih** terbuka setelah G9: tabrakan **lengan–lengan** antar gantry
selama gerak. `p1_g2 §10` mengukurnya pada konfigurasi *menjangkau* dan
menemukan tidak mengikat (0.00% pasangan hilang sampai clearance 0.15 m), tapi
itu proksi polyline dan bukan pada lengan yang sedang dibawa melintas. Lengan
terentang 1.4 m dari sumbu rotasi — **tiga kali** `R_MAX = 0.455 m` struktur
yang sudah dimodelkan.

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
| **2** | Satu lengan reach-and-dwell ke target terpersepsi | ✅ **LULUS 8/10** [g16 §B5.1](p1_g16_hw.md) — target **tetap** (`--target`), bukan terpersepsi; 3 dari 8 sukses di atas rating `joint_2` |
| **3** | Dua lengan se-gantry — ~~kopling geser-π terlihat~~ **jendela dwell BERSAMA** | ✅ **LULUS 9/10 CONCURRENT** [g18 §B2](p1_g18_hw.md) — ⚠️ lihat pertentangan di bawah |
| **4** | Tambah gerak gantry antar tugas — setup cost nyata | ✅ **LULUS 10/10 CONCURRENT** [g19 §B1](p1_g19_hw.md) — pindah 400 mm ≈ **148 s**, traverse hanya **21 s** (14 %); 5/10 di atas 12 N·m nominal |
| **5** | Dua gantry, empat lengan | ✅ **LULUS 10/10 CONCURRENT** [g20 §B2](p1_g20_hw.md) — SATU jendela 2.0 s untuk keempat lengan; rel **tetap** (0.550 / 0.000, nol gerak gantry); 🔴 rating `joint_2` **dilewati sekali** (14.27 N·m, arm_3) oleh rencana yang lolos penyaring; 5/10 di atas 12 |
| **6** | SATU jadwal scheduler end-to-end, gantry bergerak | ❌ **TIDAK DIJALANKAN** [g22 §B1](p1_g22_hw.md) — saringan plan-only jadwal lolos **0/35** (A2 terkunci: jadwal tidak dijalankan); nol gerak. Oracle L1 ≠ L2 ; **G23:** oracle L2-torsi → (i) 2/50, (iv) **0/2** [g23 §B3](p1_g23_oracle_hw.md) — cabang IK tak aman terlewat (roll bebas); nol gerak; **G24 (offline):** oracle‴ (roll + miring) → (i)–(iii) **45/50**, (iv) belum [g24 §B′2](p1_g24_roll_oracle.md); ✅ **G24b: DIJALANKAN** — seed 1, **6/6 sukses, makespan 480.44 s**, rel g1 0.55→0.70 + g2 0→1.45 [g24 §E5](p1_g24_roll_oracle.md) |

> 🔴 **Pertentangan (dicatat 2026-09-21, G19; g18 B0.10 no. 7).** Tabel ini
> dulu mendefinisikan langkah 3 sebagai *"kopling geser-π terlihat"*. Protokol
> yang **benar-benar dikunci dan dinilai** (g17 §A1–A2) mengukur **jendela dwell
> BERSAMA** dua lengan — besaran berbeda. Kopling geser-π **tidak pernah diukur**
> oleh g17 maupun g18; "langkah 3 LULUS" hanya berarti jendela bersama.
> Pelengkap terdekat yang terukur: g18 B2.3 (arm_1 menahan ≤ 0.17 mm selagi
> arm_2 terbang). Tidak diubah diam-diam; bila kopling geser-π dibutuhkan paper,
> ia butuh protokol sendiri.
>
> Langkah 2 juga menyimpang dari bunyinya: target **tetap**, persepsi dilewati
> (g16 A8 — L2 yang sama, persepsi dikeluarkan dari permukaan kegagalan).
>
> Langkah 5 (G20): rel **tidak** digerakkan (keputusan operator g20 A8-1), jadi
> langkah 5 = empat lengan + jendela bersama pada **satu** pasang rel; pindah
> gantry dengan empat lengan dan eksklusi antar-gantry di rel lain **belum
> diukur**. Hanya 4 kuartet target berbeda (saringan 3/3 meloloskan 4/10).
>
> Langkah 4: model yang diuji g3 §C2 (**retract + traverse + extend**, bukan
> "lipat + traverse + rentang"), dan traverse lewat bridge ros2_control terikat
> **durasi lintasan yang diperintah**, bukan `T_lin` — [g19 §A1](p1_g19_hw.md).

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
| `p1_g6_map.md` | Sesi G6 — peta statis dari kamera nyata; D2 lapangan lulus bit-identik |
| **`p1_g7_sched.md`** | **Sesi G7 — model penjadwalan (§A1) + solver EXACT + bukti V0–V4 (§B1). Baca §B3/§B5 sebelum mengutip §2.** Prompt G8 di §C |
| **`p1_next_steps.md`** | **rencana kerja setelah Sesi C** — 4 jalur berurut, keputusan menunggu + tenggat, prompt Sesi D |
| `p1_prompt_gcs_msbl.md` | prompt sesi port MS-BL-GNG + GCS — **sudah dieksekusi**, lihat `p1_g5_msbl_gcs.md` |
| `p1_g8_sched2.md` … `p1_g15_dense.md` | Sesi G8–G15 — heuristik + gap (G8), tabrakan gantry (G9–G10), lengan (G11–G15) |
| `p1_g16_hw.md` … `p1_g20_hw.md` | Sesi G16–G20 — §8c langkah 2–5 di perangkat keras nyata |
| **`p1_g28_hw.md`** | **Sesi G28 — §A terkunci (V28 + (iv) plan-only), bagian OFF: alat + DRY + smoke mock dev (§B0); prompt G28-ON (§D)** |
| **`p1_g27_z140_diag.md`** | **Sesi G27 — diagnosis z = 1.40: instrumen (§A0), hipotesis terkunci (§A), T0 11/11 transit bukan titik akhir (§B1), oracle⁗ ≡ tanpa z = 1.40 (§B2), P1″ LOSO (§B3); prompt G28 plan-only (§D)** |
| **`p1_g26_hw.md`** | **Sesi G26-HW — replikasi 3 seed P1′ di sel nyata: 18/18 tugas, galat −11.9/−0.4/−5.6 % (§B1–B2); (iv) 3/14, lolos ⟺ tanpa z = 1.40 (§B3); prompt G27 (§D)** |
| **`p1_g25_task_cost.md`** | **Sesi G25 — dekomposisi cap waktu (§A1, §B1), model P1′ terkunci (§A2), prediksi G24b −4.8 % (§B2), 45 instance: min-pindah 44/45, bias K2 ≤ 20 % (§B3); prompt G26 (§D)** |
| **`p1_g24_roll_oracle.md`** | **Sesi G24b (§E) — jadwal pertama di sel nyata: (iv) seed 1 lolos, tahap 2 rel g2 pertama, tahap 3 6/6 sukses, makespan 480.44 s vs P1–P4.** Sesi G24 (offline) — sapuan roll: oracle″ gagal (miring + singularitas pergelangan, §B1), oracle‴ lulus (§B′1), tugas dari node layak-oracle, 45/50 instance (§B′2); prompt G24b (§D)** |
| **`p1_g23_oracle_hw.md`** | **Sesi G23 — oracle L2-torsi: margin terukur (§B0), validasi vs 49 rencana G22 (§B1), (iv) 0/2 + roll bebas = kurva IK (§B3); prompt G24 (§D)** |
| **`p1_g22_hw.md`** | **Sesi G22 — jadwal scheduler di sel nyata: 0/35 lolos saringan plan-only; oracle L1 ≠ L2 (§B1); prompt G23 (§D)** |
| **`p1_g21_sched_tfold.md`** | **Sesi G21 — scheduler di bawah `t_fold` terukur (FISIK 50.80 s); koreksi g7 §B3/§B5, g8 §B2/§B6/§B7; offset torsi `joint_2` (§B8)** |
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

**Ke-9 (Sesi G6):** "penangkapan ulang di lapangan akan jauh lebih berantakan
daripada proksi; D2 lapangan adalah taruhan yang sesungguhnya." Diukur: **D2
lulus bit-identik** di awan nyata, dan proksi Sesi C ternyata memprediksi arah
dengan benar di keempat baris (`p1_g6_map §B5`). Sekali lagi dugaannya menaksir
kendala **lebih mengikat** dari kenyataannya — **sembilan dari sembilan, arah yang
sama.**

**Ke-10 dan ke-11 (Sesi G7):** "instance 6 tugas tidak akan muat anggaran 120 s"
— meleset, **10 tugas** muat tanpa penjarangan pose. Dan yang paling telak:
"mutex `r = 0.20` akan mengikat pada instance nyata" — meleset, **0.000 s pada
105 pasang solve**, bahkan ketika dwell dinaikkan sampai 92% garis waktu.
Sebabnya diukur, bukan ditakar: mutex memang berbiaya di **10.9%** pasangan
(himpunan tugas, pose), tapi **92%** di antaranya punya pose lain yang gratis,
dan optimum tidak pernah membangun perhentian sebesar yang dibutuhkan
([p1_g7 §B3](p1_g7_sched.md)). **Sebelas meleset, sepuluh ke arah yang sama.**
➜ Prior kerja yang sudah layak dipakai: **kendala yang belum diukur itu LONGGAR**,
sampai terbukti sebaliknya.

**Ke-12 sampai ke-16 (Sesi G8):** empat dari lima dugaan meleset
([p1_g8 §B9](p1_g8_sched2.md)), tapi **prior papan skor sendiri benar pada empat
dari lima**. Catatan penting yang lahir di sana: prior "longgar" berlaku untuk
**DUNIA**, **bukan** untuk **kode yang kita tulis sendiri** — heuristik kita
ternyata lebih **buruk** dari dugaan, bukan lebih baik.

**Ke-17 (Sesi G9):** dua dugaan dinilai, dua-duanya meleset, **ke arah yang
berbeda**, dan itu yang informatif ([p1_g9 §B8](p1_g9_sched3.md)):

- **D13** (tentang **kode sendiri**: "solver terkopel muat 120 s pada `n = 6`")
  — meleset, kodenya lebih lambat dan lebih sulit. Catatan G8 terkonfirmasi,
  sekarang **2 dari 2**.
- **D9** (tentang **dunia**: "`Δ = 0` pada ≥ 80% instance") — meleset ke
  **67.5%**, yaitu kendalanya **LEBIH MENGIKAT** dari dugaan.

🔴 **D9 adalah dugaan pertama dari 19 yang meleset melawan prior utama papan
skor.** Satu titik data tidak membatalkan pola 16-dari-18, tapi ia menandai
**batas** prior itu: "longgar" diturunkan seluruhnya dari kendala **kelayakan
statis** (interference, zona-eksklusi, co-feasibility, index topologis).
Tabrakan gantry–gantry adalah kendala **eksklusi ruang bersama** — kelas yang
berbeda, dan kelas itu baru saja memberi contoh tandingan pertamanya.

➜ Prior kerja yang sekarang lebih tepat, dua baris bukan satu:
> **Kendala KELAYAKAN yang belum diukur: tebak LONGGAR.**
> **Kendala EKSKLUSI RUANG BERSAMA: belum ada dasar untuk menebak — ukur.**
> **Dugaan tentang KODE SENDIRI: tebak lebih lambat, lebih rumit, lebih salah.**

**Papan skor (per G9): 17 meleset, 2 tepat.** ⚠️ Basi — lihat tally G10–G23 di akhir bagian ini.

Dugaan G7 yang **tepat** (tugas MR jauh lebih mahal, +7.0 s / +11.2 s) tetap
salah **mekanismenya** — mahal karena pose handover langka (p10 = 26 pose dari
2376), bukan karena dua lengan terkunci (itu paling banyak 2.0 s). Bagian yang
benar datang dari kolom yang dicetak `Instance.describe()`; bagian yang salah
tidak ditelusuri ke data sama sekali. Polanya konsisten dengan paragraf berikut.

Satu dugaan yang **tepat** (waktu traverse, Sesi A) adalah satu-satunya yang
diturunkan dari **jalur data kode**, bukan dari intuisi geometris. Itu pembeda yang
layak dipakai: telusuri variabelnya, jangan menakar keketatannya. Sesi G6
menambah satu contoh ke sisi yang sama: pertanyaan "kenapa MS-BL lambat" terjawab
dalam hitungan menit dengan **membaca `GNG_add` di akhir `MS_GNG_learning`**
(satu node per batch), bukan dengan menakar biaya batch learning.

**Tally G10–G27** (diperbarui 2026-09-23, G27; tiap baris dari §B dokumen
sesinya). D9 pindah sisi di G10 (17/2 → 16/3):

| sesi | ditambah (meleset / tepat) | papan skor |
|---|---|---|
| G10 | 3 / 3 | 19 / 6 |
| G11 | 3 / 1 | 22 / 7 |
| G12 | 1 / 3 | 23 / 10 |
| G13 | 2 / 2 | 25 / 12 |
| G14 | 3 / 3 | 28 / 15 |
| G15 | 2 / 2 | 30 / 17 |
| G16 | 2 / 1 | 32 / 18 |
| G17 | 0 / 0 (D51–D56 dinilai G18) | 32 / 18 |
| G18 | 2 / 4 | 34 / 22 |
| G19 | 4 / 3 | 38 / 25 |
| G20 | 2 / 5 | 40 / 30 |
| G21 | 1 / 7 | 41 / 37 |
| G22 | 0 / 0 (tidak ada dugaan dikunci sebelum saringan — kesalahan proses, g22 B2) | 41 / 37 |
| G23 | 7 / 4 (D79–D89; D90 tidak dinilai) | 48 / 41 |
| G24 | 4 / 8 (D91–D97, D102–D106) | 52 / 49 |
| G24b | 0 / 4 (D98–D101) | 52 / 53 |
| G25 | 4 / 2 (D108–D113; D107 terkontaminasi, tidak dihitung) | 56 / 55 |
| G26 | 3 / 6 (D114–D122) | 59 / 61 |
| **G27** | **2 / 7** (D123–D131; D132 dinilai G28) | **61 meleset / 68 tepat** |

➜ Yang G21 tambahkan ke pola: dugaan yang diturunkan dari **jalur data kode**
atau dari **mekanisme yang sudah diukur** tepat 7/7; satu-satunya meleset (D73)
adalah **besaran** yang ditaksir tanpa melihat lantainya. Dan D78 adalah dugaan
**kedua** yang tepat melawan prior "longgar" — keduanya (D9, D78) kendala
**eksklusi ruang bersama**, mengonfirmasi prior dua-baris di atas.

➜ Yang G23 tambahkan: keempat yang tepat adalah **penolakan** yang ditarik dari
mekanisme G22 terukur; ketujuh yang meleset menduga oracle **lebih menerima**
dari kenyataan, atau menduga **aturan saya sendiri** benar (margin A2, kontrol
NC3). Prior "kode sendiri: tebak lebih salah" terkonfirmasi lagi — dua kali
dalam satu sesi, keduanya pada aturan yang saya kunci.

➜ Yang G24 tambahkan: kedelapan yang tepat semuanya dari mekanisme **yang baru
diukur di sesi yang sama** (diagnosis miring/singularitas → D102–D105). Yang
meleset: tiga tentang **kontrol/aturan sendiri** (PC2 ambang, jenuh, IK), dan
D95 menduga ruang kerja aman **lebih sempit** (34.6 % node, bukan ≤ 25 %) — arah
"kendala lebih mengikat", berlawanan dengan G23.

➜ Yang G24b tambahkan: keempat tepat, semuanya dari mekanisme terukur (oracle‴
membuang kelas statis; overhead M3 + debounce rel). Tetapi **besaran** D101
(×2.37 di atas P2-serial) tidak diduga siapa pun: suku yang model anggap 2 s per
tugas ternyata 28–52 s. Papan skor kini **lebih banyak tepat daripada meleset**
untuk pertama kali.

➜ Yang G25 tambahkan: dua yang tepat (D108 saringan mendominasi, D111 regret kecil) dari
mekanisme yang **baru diukur di langkah 1**. D109/D113 menduga biaya tugas besar akan
mengubah keputusan — meleset karena **struktur instance** (0/270 tugas dua-gantry) tidak
diperiksa sebelum menduga: arah "kendala lebih mengikat", prior lama. D112 (kode sendiri
lebih salah) meleset untuk `sched.py`, tetapi skrip analisis memang crash sekali. Papan
skor kembali **lebih banyak meleset** (56/55).

➜ Yang G26 tambahkan: keenam yang tepat dari mekanisme terukur (jalur kode `return_rest`, bias alat
G22+, saringan ∝ lintasan, sebaran torsi). D115 meleset ke arah "kendala lebih **longgar** dari
kenyataan" — saringan (iv) menolak hampir semua seed dengan tugas z = 1.40; ini kendala
**kelayakan** yang lebih mengikat, berlawanan dengan prior lama (seperti D95 G24). D117/D122:
besaran/rentang dari n kecil. Papan skor kembali **lebih banyak tepat** (59/61).

➜ Yang G27 tambahkan: ketujuh yang tepat dari mekanisme yang **diukur di langkah 1 sesi yang sama** (RNEA bimodal
pada tuple sama, tilt_fail tidak terkonsentrasi, Δ_final naik dengan z) — pola G24 berulang. D126 meleset tentang
**kode sendiri** (miring gagal ternyata bukan divergensi Newton); D131 besaran dari n = 4 dengan derau per tugas 8 s.
