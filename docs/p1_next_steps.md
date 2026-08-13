# P1 — rencana kerja setelah Sesi C

> Ditulis 2026-08-13 di akhir Sesi C ([p1_g5_msbl_gcs.md](p1_g5_msbl_gcs.md)).
> **Baca [p1_state.md](p1_state.md) dulu.** Dokumen ini hanya urutan kerja —
> tidak ada keputusan baru di sini, dan tidak boleh dipakai untuk membuka ulang
> keputusan yang sudah terkunci di `p1_state.md §5`.
>
> Yang sah dari `p1_plan.md`: §2b–§2e, §3 utang teknis, §6, §7. Sisanya stale.

---

## 0. Keadaan fisik menentukan urutannya — baca ini sebelum memilih tugas

| | Status |
|---|---|
| **Lengan (4×)** | 🔴 **DILEPAS FISIK** 2026-08-13 (untuk membebaskan pandangan `rgbd2` saat kalibrasi) |
| **Kamera (2× D455)** | 🟢 terpasang, ekstrinsik terkalibrasi 2026-07-30 |
| **Gantry (2×)** | 🟢 jalan, batas rel 1600 mm terpasang di semua jalur |
| **Peta kapabilitas** | 🟢 `cap_g{1,2}_rail160.npz` (33×72) |
| **Biaya setup** | 🟢 TERKUNCI (§5.6) |

➜ **Konsekuensi:** seluruh `§8c` langkah 2–5 (reach-and-dwell) **terblokir** sampai
lengan dipasang ulang. Jadi jalur **offline** adalah jalur utama sekarang, bukan
pilihan kedua.

➜ **Dan satu kesempatan yang hilang kalau ditunda:** `map_topo_static` menginginkan
adegan **kosong tanpa lengan**. Sekarang lengan memang tidak ada. Ini jendela
terbaik untuk menangkap peta statis nyata, dan ia akan tertutup begitu lengan
dipasang lagi. Lihat Jalur C — **kerjakan lebih dulu meski kecil.**

---

## 1. Empat jalur, berurut menurut nilai per usaha

### 🥇 Jalur A — Scheduler: ground truth dulu (OFFLINE, sesi penuh, INI JALUR UTAMA)

Ini kontribusi paper. Formulasinya `XD [ST-MR-TA]`, dan **belum ada satu baris pun
kodenya**. Semua bahan model biaya sudah lengkap (`p1_state §6`).

**Urutan WAJIB dari §7.1 — jangan dibalik:**

1. **Generator instance kecil** (2–6 tugas) yang bisa dienumerasi tuntas.
2. **Solver exact** (MIP/DP) → optimum. **Ini ground truth-nya.**
3. **Baru** heuristik, dilaporkan sebagai **optimality gap** terhadap (2).
4. **Baru** baseline pembanding: fixed assignment · greedy/nearest · sequential.

Alasan urutan ini bukan kerapian: keluaran scheduler berupa **jadwal**, dan jadwal
yang salah tetap kelihatan masuk akal — tidak ada exception, tidak ada test merah.
Heuristik yang ditulis duluan jadi jangkar; begitu ia mengeluarkan angka, semua
orang mulai mempercayainya, termasuk penulisnya.

**Bahan yang sudah siap dipakai** (jangan bangun ulang):
- oracle kelayakan: `capability.py` + `cap_g{1,2}_rail160.npz`
- biaya setup: `T_traverse = max(T_lin, T_rot)`, `T_lin(Δ)=0.29+Δ_mm/v_lin`,
  `T_rot(Δ)=0.26+Δ_deg/v_rot` (§5.6, R²=1.00000)
- durasi tugas tak-nol: **dwell 2.0 s** (§5.8) — inilah yang membuat makespan
  bukan sekadar soal setup
- mutex sumber daya: zona irisan `overlap` (`r=0.20`)
- tugas MR (handover): irisan ketat — **42.9%** grid 154-titik, **53.1%**
  distribusi `surface`. ⚠️ Dua himpunan target BERBEDA — **jangan** dikutip
  sebagai rentang "42.9–53.1%".

**Yang belum dimodelkan dan wajib disebut di naskah:** tabrakan struktur
gantry–gantry (pelat mount menyapu r=0.4 m di `y=±0.36`, beririsan di
`y∈[−0.04,0.04]`). Proksi polyline **meremehkan** volume sapuan → semua angka rugi
adalah **batas bawah**, bukan nilai.

**Kunci kriteria sukses SEBELUM kode jalan** (disiplin G3/G4/G5). Draft yang perlu
diputuskan di awal sesi: ukuran instance yang masih tractable untuk exact, definisi
makespan, apakah handover dimodelkan sebagai satu tugas dua-lengan atau dua tugas
tergandeng, dan ambang optimality gap yang dianggap "cukup baik".

### ✅ Jalur C — Peta statis nyata — **SELESAI 2026-08-13 (Sesi G6)**

> **SELURUH sub-bagian di bawah adalah CATATAN SEJARAH, bukan pekerjaan tersisa.**
> Hasil: [p1_g6_map.md](p1_g6_map.md). **D2 lapangan LULUS bit-identik**
> (`max|ΔW|`: GNG 2,712 m → MS-BL 0,0). Awan mentah kedua penangkapan tersimpan
> di `/tmp/topo_cloud_{a,b}.npz` dan round-trip-nya **terverifikasi**, jadi
> jendela kamera sudah tidak diperlukan lagi untuk pertanyaan apa pun tentang
> adegan ini. Jalur B-4 (`n_comp`) ikut tertutup — jawabannya "tidak bisa
> dijawab untuk adegan nyata", bukan sebuah angka. Jalur B-3 (`grow=k`) juga
> terjawab: 14,1× lebih cepat pada k=16, D2 tetap bit-identik (§B6).

Kecil tapi ditaruh di atas Jalur B karena **jendelanya menutup** saat lengan dipasang.

Seluruh angka determinisme Sesi C memakai **proksi** distribusi dari peta
2026-08-02, bukan penangkapan ulang. Klaim D2 belum pernah diuji di lapangan.

1. Jalankan `map_topo_static` dengan kamera nyata (adegan kosong — lengan memang
   sedang tidak ada, jadi `self_filter` tidak menentukan sekarang).
2. Jalankan **dua kali**, simpan ke dua berkas.
3. Ukur: apakah dua penangkapan menghasilkan awan titik yang cukup mirip, dan
   berapa drift petanya (D3 di lapangan, bukan D3 sintetis).
4. Simpan **awan mentahnya** ke berkas. Sesi berikutnya bisa memutar ulang
   permutasi baris atas awan NYATA → **D2 di lapangan**, dan itu tidak akan
   pernah butuh kamera lagi.

⚠️ Poin 4 yang paling penting dan paling mudah terlewat: **simpan awannya**, bukan
cuma petanya. Tanpa awan mentah, D2 lapangan tidak bisa diukur belakangan.

### 🥉 Jalur B — Utang Sesi C (OFFLINE, kecil, bisa disisipkan)

Empat hal, urut menurut risiko menyesatkan:

1. **🔴 `eval.py ik` tidak punya split held-out.** Pose uji diambil dari dataset
   yang **sama** dengan yang melatih peta (`rng.choice` atas seluruh 80.000
   sampel). Docstring-nya menulis "held-out" — **itu salah**. Biasnya
   *menguntungkan* peta, jadi hasil negatif Sesi C tetap berlaku (bahkan lebih
   kuat), tapi **angka apa pun dari sini tidak layak terbit sebelum diperbaiki**.
   Perbaikan ~10 baris + jalankan ulang paritas 4 lengan.
2. **Metrik `mean manip` tidak informatif** — 1.0e-07 di arm1/3/4 (semua metode
   identik sampai 3 angka penting), wajar hanya di arm2 (0.196–0.232). Ada yang
   salah di `manip_at` atau di model tereduksi untuk grup tersebut. Didiagnosis,
   bukan ditambal.
3. ~~**Biaya offline MS-BL** — ukur `learn_batch(grow=k)` untuk `k>1`.~~
   ✅ **SELESAI (Sesi G6, p1_g6_map §B6).** Pada awan NYATA, setelan produksi:
   k=4 → 3,8×, k=8 → 7,4×, k=16 → **14,1×** (1406,7 s → 99,7 s), QE naik <2%,
   `n_nodes` tetap 1800, dan **D2 bit-identik di setiap k**. Jalur produksi
   tetap `grow=1`; angkanya ada, keputusannya keputusan naskah.
4. ~~**`n_comp` proxy 2→3**~~ ✅ **TERTUTUP (Sesi G6).** Di awan nyata GNG 5 →
   MS-BL 10, dan itu **tetap** tidak bisa disebut perbaikan maupun kemunduran —
   adegan nyata tidak punya ground truth jumlah komponen. Jawabannya adalah
   "tidak bisa dijawab", bukan sebuah angka. Jangan dibuka lagi.

### 4️⃣ Jalur D — Reach-and-dwell di hardware (TERBLOKIR sampai lengan dipasang)

`§8c` langkah 2→5. Saat lengan dipasang ulang, **wajib** lebih dulu:

- **Corner-reference 2026-07-25 jadi BASI** (`corner_world_xyz =
  (-0.151, 0.181, 2.023)`). Pemasangan menggeser pose mounting. Yang basi adalah
  corner-reference, **bukan** ekstrinsik kamera — jangan kalibrasi ulang kamera
  tanpa alasan.
- **L2 berbasis FK tidak akan melihat galat mounting.** Klaim akurasi absolut apa
  pun mewarisinya. Sudah tertulis di `p1_g4 §A2`, pastikan tetap di naskah.
- Keluarkan board kalibrasi dari ruang kerja lengan.
- Lengan istirahat **MENGGANTUNG**, jangan perintahkan ke tuck `[0,2.6,2.6,0,0,0]`
  — 12.78/14 N·m, dibatalkan penjaga torsi. Itu pose awal sistem **fake**.

---

## 2. Keputusan yang menunggu — dengan rekomendasi, bukan pertanyaan terbuka

| Keputusan | Rekomendasi | Kapan |
|---|---|---|
| **Klaim action map di naskah** | **Jangan klaim percepatan IK.** Bingkai ulang sebagai **indeks kelayakan / pembangkit kandidat**: `query_radius` menghasilkan kolam pose gantry+lengan yang layak untuk satu target. ⚠️ **Ranking-nya pakai biaya WAKTU SETUP `T_traverse` (§5.6), BUKAN energi** — `w_hold`/`w_manip` di `gantry_reach_executor` itu warisan paper konferensi Energy-Aware dan **di luar lingkup P1** (`p1_plan.md §6`); untuk P1 set keduanya 0. Laporkan ablation seeding sebagai hasil negatif satu tabel. | setelah Jalur B-1 |
| **GCS jadi default runtime?** | **Belum.** Keduanya sisi ablation. Kalau nanti dialihkan: cukup arahkan parameter `arm_models` ke `arm{n}_gcsx.npz` — tata-letak npz identik, **nol perubahan kode**. | keputusan naskah |
| **Metrik pengganti untuk klaim seeding** | Laju keberhasilan sudah **jenuh** (89–98%), tidak bisa membedakan apa pun. Kalau tetap mau klaim seeding, ukur **konsistensi konfigurasi antar tugas berurutan** (jarak sendi ditempuh / elbow-flip) — itu yang relevan untuk scheduler. | bersama Jalur A |
| **Grasp** | ⏳ **TENGGAT 2026-11-13** (≈3 bulan lagi). Kalau sampai tanggal itu belum ada lengan mengangkat objek ≥5 cm dan menahannya, paper **berkomitmen** ke formulasi non-grasping dan framing + venue disesuaikan **saat itu juga**. Tanpa handover, slotnya turun ke `ST-SR-TA` — persis slot Qin et al. | 2026-11-13 |

---

## 3. Prompt sesi berikutnya (Sesi D) — salin blok ini

```
Sesi D — scheduler Lapis 3+4: ground truth dulu, heuristik belakangan.

BACA DULU, berurutan:
1. docs/p1_state.md         — §5 (keputusan terkunci), §6 (bahan scheduler),
                              §7.1 (urutan wajib), §7.2 (ukur jangan menduga)
2. docs/p1_next_steps.md §1 — Jalur A
3. docs/p1_g5_msbl_gcs.md §A — contoh disiplin: kriteria ditulis dan DIKUNCI
                              sebelum kode jalan, instrumen divalidasi terhadap
                              ground truth sintetis sebelum dipakai

Dari p1_plan.md yang sah HANYA: §2b–§2e, §3 utang teknis, §6, §7.

TUGAS: bangun ground truth scheduler SEBELUM heuristik apa pun.
  1. generator instance 2–6 tugas yang bisa dienumerasi tuntas
  2. solver exact (MIP/DP) -> optimum
  3. BARU heuristik, dilaporkan sebagai optimality gap terhadap (2)
  4. BARU baseline: fixed assignment / greedy-nearest / sequential

JANGAN menulis heuristik lebih dulu "supaya ada angka". §7.1 melarangnya, dan
alasannya bukan kerapian: jadwal yang salah tetap kelihatan masuk akal.

PAKAI yang sudah ada, jangan bangun ulang:
  - oracle kelayakan : capability.py + /tmp/cap_g{1,2}_rail160.npz (33x72)
  - biaya setup      : T_traverse = max(T_lin, T_rot)  (p1_state §5.6, TERKUNCI)
  - durasi tugas     : dwell 2.0 s (§5.8)
  - mutex            : zona irisan overlap (r=0.20)
  - handover MR      : 42.9% (grid 154-titik) / 53.1% (surface) -- DUA himpunan
                       target berbeda, JANGAN dikutip sebagai rentang

KUNCI KRITERIA SUKSES SEBELUM MENJALANKAN APA PUN, dan tulis ke
docs/p1_g6_scheduler.md §A: ukuran instance maksimum yang masih tractable untuk
exact, definisi makespan, apakah handover = satu tugas dua-lengan atau dua tugas
tergandeng, ambang optimality gap yang dianggap cukup.

SEBUT sebagai batasan: tabrakan struktur gantry-gantry belum dimodelkan; proksi
polyline meremehkan volume sapuan, jadi semua angka rugi adalah BATAS BAWAH.

Tidak butuh perangkat keras. Lengan sedang dilepas fisik -- jangan rencanakan
apa pun yang membutuhkannya.
```

---

## 4. Pagar — jangan dibuka ulang di sesi mana pun

- **Index capability = grid.** Terukur (topo IoU 72.9% vs grid 91.4%), jenuh
  terhadap jumlah node/latihan/boundary pinning, dan sebabnya struktural. Mengganti
  GNG dengan GCS **tidak** memperbaikinya.
- **`gng.py` tetap tidak disentuh** — baseline ablation, 4 model bergantung padanya.
- **Formulasi = allocation & scheduling `XD [ST-MR-TA]`**, bukan pemilihan derajat
  koordinasi. Derajat koordinasi adalah **outcome yang dilaporkan**.
- **Kelas tugas = reach-and-dwell** (Plan B), bukan pick-and-place, sampai tenggat
  grasp 2026-11-13.
- **Batas rel operasional 1600 mm.** Bukan 2000.
- **Peta kapabilitas: pakai `_rail160`.** `cap_g{1,2}.npz` lama memuat 8 pose rel
  yang TIDAK ADA.
- **Energy-aware ADALAH PAPER TERPISAH** (`p1_plan.md §6` → `experiment_plan.md`).
  Fungsi biaya `J` di `gantry_reach_executor` memakai `hold` dan `manip` karena
  node itu dibangun untuk paper konferensi tersebut. **Jangan menarik energi ke
  dalam P1.** Biaya P1 = **waktu setup** `T_traverse` (§5.6), objektif = throughput
  / makespan. Kolom `hold`/`manip` tinggal tidak dipakai (`w_hold=w_manip=0`).
- **MS-BL-GNG dan GCS bukan kontribusi** — dikutip (Ardilla/Saputra/Kubota IJAT
  2023; Fritzke GCS). Perbaikan invarian simpleks juga bukan kontribusi: itu GCS
  Fritzke standar.
