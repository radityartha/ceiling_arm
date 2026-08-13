# Rencana P1 — Coordination-Degree Selection

> Dokumen rancangan Paper 1 (jurnal Q1). Disusun 2026-08-12.
> Companion: [experiment_plan.md](experiment_plan.md) (paper Energy-Aware, konferensi,
> terpisah) dan short paper "Topological Intelligence for Dual-Gantry Quad-Arm Ceiling
> Robot System in Trailer Living Lab" (Rochmanto, Takesue, Kubota — sudah di-submit).

---

## 1. Kesepakatan paper

- **Judul kerja:** *Coordination-Degree Selection for a Multi-Arm Ceiling Robot System
  under Shared-Gantry Coupling and Dynamic Perception*
- **Venue target:** IEEE Transactions on Systems, Man, and Cybernetics: Systems
  (IF 8.4 per Juni 2026, Q1, CiteScore ~18, 702 artikel/tahun, bulanan, open access).
  Scope resmi: *"systems engineering, issue formulation, analysis and modeling,
  **decision making**, and issue interpretation for any of the systems engineering
  lifecycle phases associated with the definition, development, and deployment of
  large systems."*
- **Cadangan:** Robotics and Autonomous Systems; IEEE T-Mech.
- **Ditolak sebagai target:** T-ASE (kontribusi terbaca incremental tanpa hasil
  algoritmik), RA-L (letter tidak dihitung program), T-RO (siklus review terlalu
  panjang).

### Pertanyaan paper

> Diberi sekumpulan tugas dan lingkungan yang terpersepsi saat ini — **berapa lengan
> yang dapat bekerja bersama, lengan yang mana, dan dengan mode apa?**

Derajat koordinasi adalah **variabel keputusan**, bukan asumsi. Pertanyaan ini hanya
muncul pada arsitektur base-bersama: pada sel dengan base tetap jawabannya selalu
"semuanya".

### Tiga kontribusi

> ⚠️ **Direvisi 2026-08-12 setelah penelusuran literatur (§2b).** Versi lama kontribusi 1
> ("model kapabilitas sebagai fungsi konfigurasi base") **tidak bisa diklaim** — itu
> Seraji 1995 dan IRM. Sumbu yang tersisa adalah **konkurensi**, **base majemuk**, dan
> **ketergantungan lingkungan online**.

1. **Konkurensi sebagai objektif** — derajat koordinasi (berapa lengan bekerja
   *serentak*) sebagai variabel keputusan, bukan panjang urutan perpindahan base yang
   diminimalkan. Prior work memilih tangan mana yang dipakai **bergantian**; di sini
   pertanyaannya berapa yang bisa bekerja **bersamaan**.
2. **Penugasan lintas base bergerak majemuk** — kelayakan terfaktorisasi per gantry,
   sehingga penugasan tugas ke gantry adalah tuas yang bekerja (~~1.2% → 11.4%~~ →
   **36.5% → 88.6%, gain 2.4×** setelah G2). Seluruh prior work base tunggal.
3. **Kapabilitas bergantung lingkungan, diperbarui online** + **deployment** 4 lengan
   di hardware nyata dengan persepsi dalam loop, analisis kegagalan dan pemulihan.
   Prior work: lingkungan statis, perencanaan offline.

### Yang BUKAN kontribusi (dikutip, bukan diklaim)

- **MS-BL-GNG** — Ardilla, Saputra, Kubota, *"Multi-Scale Batch-Learning Growing Neural
  Gas Efficiently for Dynamic Data Distributions"*, Int. J. Automation Technology,
  17(3), 206–216, 2023.
- **Topological twin / multi-layer GNG** — Obo, Matsuda, Takesue, Kubota,
  *"Topological Mapping based on Multi-Layer Growing Neural Gas for Topological Twin"*,
  IEEE (Xplore doc 11228342).
- **GCS** — Fritzke, Growing Cell Structures.
- **Inverse Reachability Map** — Vahrenkamp et al. 2013; Reuleaux (Makhal & Goins 2018).

---

## 2b. Penelusuran literatur — dilakukan 2026-08-12

Hasilnya **membunuh beberapa klaim** yang sempat masuk rancangan. Dicatat supaya tidak
dihidupkan ulang.

| Prior work | Apa yang sudah dikerjakan | Klaim yang mati |
|---|---|---|
| **Seraji, *Reachability Analysis for Base Placement in Mobile Manipulators*, J. Robotic Systems 12(1):29–43, 1995** | Base dengan **satu derajat mobilitas (tracked)**: himpunan posisi base layak = **region pada sumbu translasi, batasnya diturunkan analitik**. Diperluas ke struktur **gantry**. | ❌ "inversi pada manifold base terkendala rel" — sudah ada sejak 1995, dalam bentuk tertutup |
| ***Base Position Planning for Dual-Arm Mobile Manipulators Performing a Sequence of Pick-and-Place Tasks***, IEEE Xplore 7363551 | **Dua lengan pada satu base bergerak**; merumuskan **region posisi base** tempat objek dapat diambil; robot **memilih tangan kanan/kiri**; QP untuk IK; BB+SA untuk urutan posisi base | ❌ "dua lengan berbagi base bergerak + region posisi base + pemilihan lengan" |
| *Planning an Efficient and Robust Base Sequence for a Mobile Manipulator* (arXiv:2001.08042) | Mencari **area bersama** tempat satu base dapat menjangkau objek dari **beberapa tray sekaligus** | ❌ "region bersama yang melayani beberapa target" |
| MoMa-Pos (arXiv:2403.19940), B* (arXiv:2504.12719) | Penempatan base modern; pemilihan embodiment/lengan dari reachability | ❌ "pemilihan lengan dari reachability" |
| *Predictive Reachability for Embodiment Selection* (arXiv:2410.21059) | **Sudah dibaca penuh — lihat §2c.** "Embodiment" = **base vs lengan** pada satu mobile manipulator, bukan pemilihan antar-lengan | (tidak membunuh apa pun) |

**Yang bertahan sebagai sumbu novelty:**

1. **Simultanitas sebagai objektif.** Prior work meminimalkan *perpindahan base* /
   panjang urutan. Di sini yang dimaksimalkan adalah *berapa lengan bekerja serentak*.
2. **Base bergerak majemuk + penugasan lintas base.** Seluruh prior work base tunggal.
3. **Kapabilitas bergantung lingkungan, diperbarui online.** Prior work lingkungan
   statis, offline.

**Konsekuensi:** seluruh mesin reachability/inversi turun jadi **tooling standar yang
dikutip**, bukan kontribusi. Angka **1.2% → 11.4%** naik jadi hasil yang memikul paper.

> 🔴 **Dan angka itu kemudian mengempis (G2, 2026-08-12): 36.5% → 88.6%, gain 2.4×.**
> Jadi hasil yang dijadikan tumpuan justru yang paling terpukul. Pengganti yang
> tersedia: **kurva derajat koordinasi vs sebaran tugas, 37% → 90%**
> ([p1_g2_results.md §3](p1_g2_results.md)) — kebetulan lebih pas untuk sumbu #1.
> Penentu akhirnya interferensi lengan–lengan, belum diukur.

---

## 2c. Verifikasi dua paper penentu — 2026-08-12

Menindaklanjuti "TUGAS WAJIB" di §2b. **Hasil: sumbu novelty #1 tidak menyusut.**

### arXiv:2410.21059 — ✅ TERBACA PENUH

*Predictive Reachability for Embodiment Selection in Mobile Manipulation Behaviors*
(Feng, Horii, Nagai). PDF v1 discan seluruhnya (6.349 kata).

- `dual-arm` 0×, `two arms` 0×, `multi-robot` 0×, `both arm` 0× di seluruh teks.
- "Embodiment" (49×) = **base vs lengan** pada **satu** mobile manipulator:
  *"Selecting the embodiment between the base and arm can be determined based on
  reachability."* Bukan pemilihan di antara beberapa lengan.
- Eksperimen: RL berbasis citra **di simulasi**, base 2-DOF + **3 sendi lengan**,
  *"The robot's hand remains inactive throughout the experiments"* — reaching murni,
  tanpa grasp, tanpa hardware.

**Konsekuensi:** nol ancaman ke sumbu #1. Tapi baris §2b lama yang memakai paper ini
untuk membunuh "pemilihan lengan dari reachability" **salah baca** — sudah dikoreksi.
Klaim itu tetap mati, karena MoMa-Pos / B* / Harada 2015, bukan karena paper ini.

### IEEE 7363551 — ❌ FULL TEXT TIDAK DIPEROLEH

Harada, Tsuji, Kikuchi, Nagata, Onda, Kawai, *Humanoids 2015*, pp. 194–201,
DOI `10.1109/HUMANOIDS.2015.7363551`.

Closed access, **tidak ada salinan sah yang gratis**: Unpaywall `oa_status: closed`,
`has_repository_copy: false`; Semantic Scholar `openAccessPdf: CLOSED`; halaman
publikasi Harada sendiri di Osaka-u tidak memuat link PDF; IEEE Xplore 429, ACM DL 403.

Yang diperoleh: **abstrak verbatim** + **14 deskripsi independen** dari paper pensitasi.

> "the robot can **selectively use either the right or the left hand** to pick up an
> object and can **minimize the sequence size of the base position** needed to performed
> the given task."

"Either… or" = pemilihan tangan eksklusif per objek; objektifnya panjang urutan base.
Tidak satu pun dari 14 sitasi menyebut konkurensi. Xu & Harada (kelompok yang sama,
arXiv:2001.08042) menyebutnya *"preliminary numerical analysis on reducing the
unnecessary base movements"*.

**Penilaian:** sumbu #1 bertahan; sumbu #2 aman tanpa syarat (base mereka tunggal).
**Sisa risiko yang belum bisa dinihilkan:** badan paper mungkin membiarkan dua tangan
mengambil dua objek pada satu base position. Bahkan bila iya, itu tidak pernah menjadi
*objektif* mereka — jadi paling buruk ketajaman framing #1 berkurang, bukan klaimnya.

**Untuk menutup:** unduh PDF dari akses institusi (10 menit dari kampus), cek apakah ada
pengambilan dua objek serentak pada satu base position.

### Ancaman yang lebih relevan sekarang

Setelah reformulasi ke **allocation & scheduling XD [ST-MR-TA]** (§2 catatan merah),
ancaman ke "konkurensi sebagai objektif" bukan lagi dua paper base-placement ini,
melainkan literatur **MRTA + multi-arm cell scheduling**, tempat memaksimalkan jumlah
agen yang bekerja serentak adalah hal rutin. Sumbu #1 kini bertumpu pada **shared
positioning resource**-nya, bukan pada konkurensinya. **Penelusuran berikutnya.**

---

## 2d. Penelusuran MRTA / multi-arm cell scheduling — 2026-08-12

Menindaklanjuti "penelusuran berikutnya" di §2c. **Hasilnya membunuh sumbu novelty #1
dan memaksa #2 dan #3 dirumuskan ulang lebih sempit.** Ini penelusuran paling merugikan
sejauh ini — dicatat lengkap supaya tidak dihidupkan kembali.

### Empat literatur bertetangga yang belum pernah disebut

| Literatur | Strukturnya | Yang dibunuh |
|---|---|---|
| **Quay Crane Scheduling (QCSP)** — beberapa crane pada **satu rel bersama**, non-crossing + safety clearance, objektif makespan/throughput. Literatur besar dan matang | shared positioning resource, posisi tiap agen independen, kendala urutan + jarak aman | ❌ "shared rail sebagai kendala penjadwalan" bukan hal baru |
| **Multi-hoist scheduling** (jalur electroplating) — beberapa hoist pada **satu track**, collision-free, cyclic, minimalkan cycle time; MIP + branch-and-bound | idem, plus formulasi disjunctive untuk collision | ❌ idem, termasuk formulasi eksak-nya |
| **Dual-arm cluster tool scheduling** (semikonduktor) — **dua lengan terpasang berlawanan 180°** pada **satu robot berputar**, swap strategy, objektif cycle time. Ada pula varian *"two independent robot arms"* | **persis struktur geser-π kita** | ❌ "dua lengan pada satu base berputar dengan offset tetap" bukan hal baru |
| **Online task allocation multi-manipulator** (Robotics and Computer-Integrated Manufacturing, 2024) — workspace **beririsan**, *"shared sets"* untuk irisan, motion-envelope collision → *current feasible set*, alokasi **online**, tugas datang dari **sensor visual eksternal** selama eksekusi | online + persepsi + irisan workspace + himpunan bersama | ❌ "kapabilitas online dari persepsi" dan ❌ "zona irisan sebagai himpunan bersama" bukan hal baru |

> ⚠️ Full text RCIM 2024 tidak diperoleh (403). "Base tetap" **disimpulkan** dari domainnya
> (white body polishing / welding spot) — perlu dikonfirmasi. Kalau ternyata base-nya
> bergerak, sumbu #3 ikut jatuh.

### Akibatnya untuk tiga sumbu novelty §2b

1. **"Simultanitas/konkurensi sebagai objektif" — MATI sebagai novelty.**
   Memaksimalkan throughput dengan banyak agen serentak adalah rutin di QCSP, multi-hoist,
   dan cluster tool. §2c sudah menduga ini; sekarang terkonfirmasi. **Turunkan dari
   "kontribusi" jadi "pilihan objektif".** Jangan diklaim.
2. **"Base bergerak majemuk" — harus dirumuskan ulang.** QCSP sudah punya banyak agen di
   rel bersama; cluster tool sudah punya pasangan lengan offset-tetap. Yang tersisa bukan
   salah satunya.
3. **"Kapabilitas bergantung lingkungan, online" — melemah.** RCIM 2024 sudah melakukannya,
   lengkap dengan himpunan bersama untuk workspace beririsan. Yang tersisa: base mereka
   tidak bergerak, jadi kapabilitasnya tidak bergantung pose base.

### Yang benar-benar tersisa — satu kalimat, dan hanya ini

Di QCSP/hoist, kopling adalah **non-crossing**: kendala urutan geometris, sepele
dinyatakan. Di cluster tool, kopling adalah offset tetap tapi **reachability sepele**
(modul di posisi tetap, selalu terjangkau). Di RCIM 2024, ada persepsi online tapi
**base tidak bergerak**.

> **Di sini kopling bekerja LEWAT reachability.** Dua lengan berbagi satu pose,
> dan kelayakan sepasang tugas adalah **irisan dua himpunan turunan-IK di atas
> silinder `(linear, rotation)`** — yang ikut menyusut saat persepsi berubah.

Itu struktur yang tidak dimiliki keempat literatur di atas. Bukti terukur yang
mendukungnya sudah ada: geser-π eksak sehingga `G_a2 = roll(G_a1)`
([p1_g2_results.md §1](p1_g2_results.md)); zona irisan sekaligus mutex **dan**
himpunan handover (§11); penghalang dinamis teramplifikasi lewat kopling (§7).

### Peringatan kejujuran

Ini sekarang **novelty kombinasi**, dan novelty kombinasi rapuh di Q1. Tiap unsur ada
di suatu tempat; yang tidak ada adalah gabungannya. Supaya bertahan, paper **harus**
menunjukkan bahwa gabungan itu melahirkan **kesulitan struktural baru** — bukan sekadar
daftar fitur. Kandidat terkuatnya: irisan-turunan-IK yang bergantung lingkungan tidak
bisa direduksi jadi kendala urutan seperti non-crossing, sehingga teknik QCSP/hoist
tidak berlaku langsung. **Itu harus ditunjukkan, bukan diasumsikan.**

### Yang wajib dikerjakan berikutnya

1. Konfirmasi base tetap di RCIM 2024 (kalau bergerak, sumbu #3 jatuh).
2. Cari eksplisit: adakah QCSP/hoist dengan **reachability non-trivial** (bukan interval)?
3. Cari: cluster tool dengan **hub yang bergerak/translasi**?
4. Tunjukkan secara formal kenapa reduksi ke non-crossing gagal di kasus ini.

---

## 2e. Qin et al. 2024 — TERBACA PENUH. Paper terdekat yang ada.

Xinyu Qin, Zixuan Liao, Chao Liu, Zhenhua Xiong, *"Online task allocation and
scheduling in multi-manipulator system considering collision constraints and unknown
tasks"*, **Robotics and Computer-Integrated Manufacturing 90 (2024) 102808**, SJTU.
PDF di `docs/jurnal reff/`.

### Yang dikonfirmasi — lebih buruk dari dugaan

> **"This paper addresses the ST-SR-TA-XD class problems applicable in online
> scenarios."** (§2, verbatim)

**Slot `XD [ST-SR-TA]` sudah ditempati secara eksplisit, dengan menyebut taksonomi
Korsah dan Gerkey-Matarić.** Jadi "kami menggarap XD [ST-SR-TA]" **bukan** klaim yang
bisa dipakai. Yang mereka punya:

| Komponen | Mereka |
|---|---|
| Alokasi online | ✅ greedy atas *feasible set*, 1.3 ms untuk 800 tugas |
| Tugas tak dikenal saat runtime | ✅ dari sensor visual eksternal, Alg. 1 dipanggil ulang |
| Workspace beririsan → *shared sets* | ✅ `P(i)` independen, `P_s(i,j)` bersama |
| Deteksi tabrakan online | ✅ *motion envelope* + `D(E_p, E_q) ≥ d_safe`, silinder demi kecepatan |
| Penanganan deadlock | ✅ *interlocking* → mundur ke pose awal |
| Hardware nyata | ✅ 4× JAKA ZU12, 81 titik poles white body |

### Yang menyelamatkan kita — dan ini terukur, bukan asersi

**1. Base mereka TETAP, dan irisannya sengaja DIPERKECIL.**

> *"The white body parts are affixed to the workbench... The installation positions of
> the four manipulators have been optimized using accessibility and manipulability as
> indicators"* (ref [35], AIM 2022)
>
> *"in order to expand the overall workspace of a multi-manipulator system, the
> installation position of manipulators will relatively reduce its overlap space, so
> the tasks in these shared sets are relatively few."*

Mereka **merancang irisan agar kecil**. Kita **tidak bisa** — irisan dipaksa oleh
mekanismenya, 48–65% node ([p1_g2_results.md §12](p1_g2_results.md)).

**2. XD mereka berasal dari TABRAKAN. XD kita berasal dari VARIABEL BERSAMA.**

Ini diskriminator terpenting, dan kita punya angkanya:

| | Sumber kopling | Kalau tabrakan dihilangkan |
|---|---|---|
| Qin et al. | tabrakan antar lengan ber-base independen | masalah **terurai** jadi `k` jadwal independen |
| **Kita** | `arm1` dan `arm2` berbagi satu `(linear, rotation)` | **tetap tidak terurai** |

Terukur: interference **0.00%** pasangan hilang (§10), tapi ko-kelayakan pasangan tetap
hanya **59.5%**, bukan 100%. Jadi kopling di sistem kita terbukti **bukan** tabrakan.
Di sistem mereka, kopling **adalah** tabrakan. Itu perbedaan yang bisa dibuktikan, bukan
diklaim.

**3. "Online" mereka = tugas datang. "Online" kita = kapabilitas berubah.**
Lingkungan mereka statis (white body terpasang di workbench). Di sistem mereka sebuah
tugas **tidak pernah menjadi tidak layak**. Di sistem kita, satu orang lewat menghapus
**19.6%** pasangan target (§7).

**4. Mereka murni ST-SR — tidak ada handover, tidak ada MR task.**

**5. Base tidak bergerak → tidak ada setup time sama sekali.** Biaya mereka murni gerak
lengan antar tugas. Seluruh sumbu *sequence-dependent setup time* tidak tersentuh.

### Posisi yang bisa dipertahankan sekarang

> Qin et al. menyelesaikan `XD [ST-SR-TA]` di mana kebergantungan lintas-jadwal adalah
> **tabrakan antar lengan ber-base independen**, sengaja diperkecil lewat desain
> pemasangan, lingkungan statis, dan "online" berarti tugas baru berdatangan.
>
> Kita menggarap **`XD [ST-MR-TA]`** di mana kebergantungan itu adalah **shared
> positioning resource yang tidak bisa dirancang hilang**, kapabilitasnya berubah
> mengikuti persepsi, dan zona irisannya sekaligus bahaya **dan** kapabilitas handover.

### Nilai praktis: kita dapat baseline yang terbit dan sebanding

Metode mereka bisa dijalankan di sistem kita dengan **gantry dibekukan**. Itu baseline
"fixed-base multi-manipulator (Qin et al. 2024)" — jauh lebih kuat daripada
"greedy generik", dan langsung menunjukkan apa yang dibeli oleh mobilitas gantry.

### Pertanyaan riset yang benar-benar baru, dan tidak ada di mana pun

> Menggeser gantry melayani tugas berikutnya **satu** lengan, tapi bisa **merusak**
> kelayakan pasangannya — dan geseran itu sendiri berbiaya waktu.
> **Kapan layak menggeser gantry?**

Trade-off ini tidak ada di Qin (base tidak bergerak), tidak ada di QCSP (tiap crane
punya posisi sendiri), tidak ada di cluster tool (hub tidak bertranslasi). **Ini inti
scheduler kita.**

### Tetangga yang wajib dicek berikutnya (dari daftar pustaka mereka)

- [29] S. Zhang, F. Pecora, *Online and scalable motion coordination for multiple robot
  manipulators in shared workspaces*, **IEEE T-ASE 2023**
- [30] S. Zhang, F. Pecora, *Online sequential task assignment with execution
  uncertainties for multirobot manipulators*, **RA-L 6(4) 2021**
- [35] X. Qin et al., *Where to install the manipulator*, AIM 2022 — penempatan base
  offline, konfirmasi base mereka tetap

---

## 2. Bukti terukur (provisional)

Semua angka di bawah dihitung 2026-08-12 dari `/tmp/arm{1..4}_model.npz` (4 action map
terlatih, 3000 node, `[task|q]` 8-DOF) + FK pinocchio atas `workcell_full.urdf`.

> ⚠️ **Provisional.** Peta ini dibangun untuk *seeding IK* — satu `q` representatif per
> node. Karena itu `G_a(t)` tersampel jarang (median 6 pose pada tol 0.15 m, 14 pada
> 0.20 m). Sapuan khusus (§5, G2) yang mengunci angka final.
>
> 🔴 **SUDAH DIGANTI (2026-08-12).** G2 sudah dijalankan — lihat
> [p1_g2_results.md](p1_g2_results.md). Sapuan 2 juta sampel menemukan `G_a(t)` ~159×
> lebih besar dari peta 3000-node ini, jadi **semua baris konkurensi/ko-kelayakan di
> tabel bawah salah** (4 lengan sadar-kopling: 11.4% → **88.6%**; gain 9.5× → **2.4×**).
> Baris `5.10×` (E1) dan irisan workspace tidak terpengaruh. Jangan kutip angka
> konkurensi dari sini.
>
> 🔴 **Dan arm–arm interference sudah diukur juga — TIDAK mengikat** (intra-gantry
> 0.00% pasangan hilang, inter-gantry 4.19% level konfigurasi dengan pose
> independen; [p1_g2_results.md §10](p1_g2_results.md)). Jadi baik jangkauan
> maupun geometri lengan tidak membatasi konkurensi 4 lengan. Satu-satunya
> kendala yang menggigit adalah **lingkungan dinamis** (17.2% pasangan hilang).
> Konsekuensi: derajat koordinasi bukan variabel keputusan kelayakan — paper
> diformulasikan ulang sebagai **allocation & scheduling**, XD [ST-SR-TA]
> dengan shared positioning resource. Judul/struktur menyusul.

| Temuan | Angka | Implikasi |
|---|---|---|
| DOF gantry memperluas reachable workspace | **5.10×** (res 0.05 m) | sudah selesai di E1, jadi motivasi |
| Rentang `G_a(t)` di sepanjang rel | median **0.20–0.40 m** dari rel 2.0 m | satu target dilayani hanya dari 10–20% rel |
| Irisan workspace intra-gantry (swept, <5 cm) | 19.7% / 20.0% | |
| Irisan workspace inter-gantry | 11.1–12.1% | |
| Pasangan **se-gantry** co-feasible (tol 0.20) | **15.7%** | kopling intra-gantry mengikat keras |
| Pasangan **beda gantry** | **100%** | kopling inter-gantry tidak mengikat |
| 3 lengan (2 di g1, 1 di g2) | 16.0% | lengan ketiga praktis gratis |
| 4 lengan (2+2) | 2.5% ≈ 0.157 × 0.16 | **kelayakan terfaktorisasi per gantry** |
| Konflik inter-gantry (arm1 tetap, opsi arm3 terhalang) | 1.4 / 3.5 / 5.9 / 8.9% pada clearance 0.10 / 0.20 / 0.30 / 0.40 m; **0% pernah terhalang total** | inert secara global |
| Idem, target berjarak <0.4 m (rezim handover) | **35.8%** opsi terhalang, **57%** pasangan >25% terhalang (clearance 0.20 m) | mengikat hanya di medan-dekat |
| Penghalang dinamis seukuran orang | 4.5–15.4% kapabilitas hilang (rata-rata 7.5% atas grid 126 titik, maks 15.4%) | efek langsung sedang |
| **4 lengan serentak, penugasan TETAP** | **1.2%** | |
| **4 lengan serentak, penugasan SADAR-KOPLING** | **11.4%** | **9.5× — ini Figure 1** |
| Derajat koordinasi tercapai (penugasan terbaik) | 2 lengan 16.2%, **3 lengan 72.2%**, 4 lengan 11.4% | rata-rata **2.95 dari 4** |

### Konsekuensi untuk klaim

- ✅ **Derajat koordinasi memang variabel** (2/3/4 tergantung sebaran tugas).
- ✅ **Penugasan sadar-kopling adalah tuas yang bekerja** (1.2% → 11.4%).
- ❌ **JANGAN klaim graf kopling global 4 lengan.** Kelayakan terfaktorisasi bersih per
  gantry; inter-gantry inert kecuali di medan-dekat.
- ❌ **JANGAN klaim topologi C-space.** BMU hanya memakai dimensi task; `q` tidak pernah
  masuk metrik jarak maupun pembentukan edge. Keterbatasan ini diwarisi dari desain
  GCS sensei (FRD-01 `Node_GID` juga hanya memakai `inpN`).

---

## 3. Metode

| Lapis | Isi | Status | Lokasi |
|---|---|---|---|
| **0** | Persepsi → `O(t)`: GNG statis (dipersist) + GNG dinamis online; statis disubtraksi | ✅ | `map_topo_static.py`, `env_gng.py`, `topo_static_pub.py` |
| **1** | Kapabilitas sah per lengan: `carve` / `danger` / `cfree` lewat difusi `S = Σ γˡ Âˡ` | 🟡 | `reach_fusion.py:44`, `:447` |
| **2** | Inversi `G_a(t\|O)` — jangkauan **offline**, penyaringan tabrakan **online** | ❌ | dibangun |
| **3** | Kelayakan subset lengan; lengan se-gantry berbagi `g`; 15 subset → enumerasi | ❌ | **inti kontribusi** |
| **4** | Mode + penugasan; pose handover = `G_a(t\|O) ∩ G_b(t\|O) ≠ ∅` | ❌ | dibangun |
| **5** | Eksekusi: seed IK → MoveGroup → planning scene | ✅ | `gantry_reach_executor.py`, `seed_ik.py`, `gng_collision.py` |

### Definisi inti

Untuk subset lengan `A` dengan penugasan target:

> `A` layak ⟺ ada `(g₁, g₂)` sedemikian sehingga setiap `a ∈ A` punya targetnya di
> `G_a(·|O)` pada pose itu, **dan** tidak ada pasangan lengan di `A` yang bertabrakan.

Lengan yang berbagi gantry memakai `g` yang sama — itu yang membuat subset tidak bisa
diurai jadi keputusan per-lengan. **Derajat koordinasi = subset layak terbesar.**

### Loop online

```
persepsi berubah → O(t) diperbarui → G_a(·|O) menyusut/tumbuh
      → subset layak terbesar berubah → derajat koordinasi disesuaikan
```

### Yang wajib ditambahkan supaya jadi metode, bukan rekayasa

1. **Pernyataan sifat** — *soundness*: kalau dinyatakan layak pada `g`, semua lengan
   benar-benar menjangkau dalam toleransi τ. *Kelengkapan sampai ε*: kalau solusi ada
   dan target berjarak ≥ ε dari batas jangkauan, pasti ditemukan.
2. **Latensi replanning terukur** — milidetik dari perubahan persepsi ke keputusan baru.
   Ini yang membuktikan pemisahan offline/online berhasil.

### Utang teknis metode

- **MS-BL-GNG**: `gng.py` masih GNG Fritzke online (per-sampel, `age_max`, `lam`).
  Port dari `Meso-HSR/GNG.h:940` (`MS_GNG_learning`). Batch mean update →
  **deterministik**; peta statis sekarang berubah tiap run.
- **GCS**: port dari `FRD-01/nRobot.h:435-590`. **Catatan: implementasi sensei tidak
  memelihara invarian simpleks** — `GCS_add` tidak menyambungkan node baru ke tetangga
  bersama `h` dan `k` (blok itu di-comment out di `Meso-HSR/GNG.h:634`). Tanpa langkah
  itu jaring berdegradasi jadi graf biasa. Perbaikan ini **standar GCS Fritzke**, jadi
  bukan kontribusi — tapi wajib kalau nama "GCS" dipakai. Reviewer di society ini
  mengenal GCS.
- **Jangan sentuh `gng.py`** — 4 model terlatih + `test/test_gng.py` bergantung padanya.
  Tambahkan `bl_gng.py` dan `gcs.py` sebagai file baru; `gng.py` tetap berguna sebagai
  baseline ablation.

---

## 4. Eksperimen

| ID | Isi | Butuh robot | Output paper |
|----|-----|-------------|--------------|
| **X0** | Karakterisasi: derajat koordinasi vs sebaran tugas | tidak | kurva + tabel |
| **X1** | **Penugasan tetap vs sadar-kopling** (1.2% → 11.4%) | tidak (+konfirmasi hw) | **Figure 1** |
| X2 | 2 lengan bersamaan | ya | tabel sukses + waktu siklus |
| X3 | 3 lengan bersamaan | ya | idem |
| X4 | 4 lengan bersamaan | ya | idem |
| X5 | Handover lintas gantry | ya | sukses + presisi serah-terima |
| X6 | Penghalang dinamis masuk → derajat turun → sistem menyesuaikan | ya | timeline + video figure |
| X7 | Degradasi: satu lengan hilang, berapa cakupan tersisa | tidak | tabel dependability |

**Baseline (empat, jangan lebih):** fixed assignment · greedy/nearest · sequential
(satu lengan pada satu waktu) · MIP/DP-optimal sebagai batas atas.

**Metrik:**
- derajat koordinasi tercapai vs baseline
- throughput / waktu siklus
- **sukses per-tugas DAN per-episode, dilaporkan terpisah** — dengan grasp per-lengan
  70%, empat lengan per-episode ≈ 0.7⁴ ≈ 24%; definisikan sebelum mengambil data
- latensi replanning
- sukses handover + presisi
- analisis kegagalan + pemulihan sebagai klaim eksplisit

---

## 5. Gate

| Bulan | Isi | Gate |
|---|---|---|
| 1–2 | **Grasp satu lengan** (hardware) | objek naik ≥5 cm dan tetap tercengkeram |
| 1–2 | **G2 — sapuan `G_a(t)`** (paralel, tanpa robot) | X0 + X1 berangka pasti |
| 3 | Lapis 2–4 + pernyataan sifat | X1 dikonfirmasi, latensi terukur |
| 4 | X2, X3 | 2 dan 3 lengan bersamaan di hardware |
| 5 | X4, X5 | 4 lengan + handover |
| 6 | X6, X7 + sapuan baseline | semua klaim berangka |
| 7–8 | Analisis + penulisan | **SUBMIT bulan 8** |

### G2 — rancangan sapuan (bisa dimulai hari ini)

Balik urutan sampling `data_gen.py`:

1. Diskretkan pose gantry: `linear` ∈ [0, 2.0] tiap 5 cm (41) × `rotation` ∈ [−π, π]
   tiap 5° (72) ≈ 2900 pose.
2. **Jangan** hitung FK ulang per pose. Sendi gantry hanya memberi transformasi kaku
   pada base lengan → sampel 6 sendi lengan **sekali** (±200k), FK di frame base lengan
   → satu cloud `P_a`, satu KD-tree.
3. `t` terjangkau pada `g` ⟺ `‖P_a − T(g)⁻¹·t‖_min < tol`. **Transformasikan targetnya,
   bukan cloud-nya.** Satu query per pose → milidetik per target.
4. Konektivitas `G_a(t)` diuji di **2D** `(linear, rotation)`, graf tetangga-8.
   **Rotasi melingkar di ±π — topologinya silinder, bukan bidang.** Kalau diabaikan,
   satu komponen bisa terhitung dua.

Verifikasi dulu ke URDF bahwa pose base lengan memang fungsi kaku dari
`(linear, rotation)` saja — terutama letak sumbu rotasinya.

---

## 6. Batas P1

**Di dalam P1:** koordinasi rantai terbuka (bersamaan 2–4 lengan, handover); persepsi
dinamis → hindari obstacle → kapabilitas berubah; manusia sebagai **okupansi dinamis**.

**Di luar P1:**
- **P2** — co-lift rantai tertutup (4 lengan mengangkat bersama). Batasnya fisik:
  rantai terbuka vs tertutup.
- **P3** — interaksi manusia sesungguhnya (niat, komunikasi, serah-terima ke manusia).
  Belum ada infrastrukturnya sama sekali di repo.
- **RL / learning** — tidak ada ruang kontribusi tersisa; menambahkannya mengundang
  kritik "marginal". Dan adanya baseline DP-optimal membuat klaim "RL diperlukan"
  kontradiktif secara internal.
- **Energy-aware** — tetap paper konferensi terpisah ([experiment_plan.md](experiment_plan.md)).
- **Klaim topologi C-space.**

Busur disertasi: **koordinasi (P1) → kooperasi (P2) → interaksi (P3).**

---

## 7. Risiko

| Risiko | Mitigasi |
|---|---|
| **Grasp tidak terwujud** | Tidak ada. Blocker mutlak — kalau bulan 2 belum jalan, berhenti dan nilai ulang |
| 4 lengan terlalu berat di hardware | Paper berdiri di X2+X3; X4 dilaporkan apa adanya |
| Penghalang dinamis tidak cukup berdampak (7.5% rata-rata) | Ukur amplifikasinya lewat kopling di G2; kalau lemah, X6 turun jadi seksi, bukan hasil utama |
| Klaim GCS ditangkap reviewer se-society | GCS harus benar-benar GCS (simplex repair) atau namanya diturunkan |
| Angka provisional berubah setelah sapuan | Semua angka §2 ditandai provisional; G2 mengunci ulang sebelum penulisan |
| Reviewer: "inti algoritmanya sederhana" | Benar — enumerasi 15 subset + operasi himpunan. Yang dinilai adalah model + sistem + deployment. Pernyataan sifat dan latensi yang mengangkatnya dari rekayasa ke metode |
| Terbaca sebagai "kami bangun robot lalu mengukurnya" | Section 3 harus model eksplisit, bukan deskripsi sistem. Ini penentu terima/tolak di T-SMC:S |

---

## 8. Pertanyaan terbuka

1. ~~Apakah efek penghalang dinamis **teramplifikasi** lewat kopling?~~
   **SUDAH DIJAWAB 2026-08-12 — SEBAGIAN.** Wajib bandingkan ke baseline nol: karena
   ko-kelayakan adalah produk dua himpunan, pembuangan **independen** berlaju `s`
   sudah memberi `1−(1−s)² ≈ 2s` sendirian. Di tingkat **konfigurasi** tidak ada
   amplifikasi (1.79× vs baseline 1.84× — rugi kedua lengan justru berkorelasi).
   Di tingkat **eksistensi** ada: penghalang seukuran orang menghapus **7.6%** target
   dari satu lengan tapi **17.2% pasangan target** dari lengan se-gantry — 2.29× vs
   baseline 1.92×. Lihat [p1_g2_results.md §7](p1_g2_results.md).
2. ~~Apakah `G_a(t)` **terputus** di 2D `(linear, rotation)`?~~
   **SUDAH DIJAWAB 2026-08-12 — TIDAK.** `G_a(t)` selalu **terhubung sederhana**:
   0 dari 616 pasangan (lengan, target) punya >1 komponen di silinder
   `(linear, rotation)`, di semua toleransi 0.05–0.20 m, bahkan tanpa membuang
   komponen 1-sel. Yang 29–48% itu artefak memotong jahitan ±π — setiap kasus yang
   tampak terputus hilang begitu rotasi disambung. Lihat
   [p1_g2_results.md §6](p1_g2_results.md). **Konsekuensi: representasi topologis
   tidak _diperlukan_, hanya diwarisi.** Jalur klaim itu ditutup.
3. ~~Cek literatur irisan IRM untuk lengan majemuk pada satu base bergerak~~
   **SUDAH DILAKUKAN 2026-08-12 — lihat §2b. Hasilnya: sudah ada (Seraji 1995,
   IEEE 7363551), kontribusi 1 ditulis ulang.**
   ~~Sisa yang wajib: baca penuh IEEE 7363551 dan arXiv:2410.21059.~~
   **SUDAH DIKERJAKAN 2026-08-12 — lihat §2c.** arXiv:2410.21059 terbaca penuh, nol
   ancaman. IEEE 7363551 **full text tidak diperoleh** (closed access, tidak ada salinan
   sah gratis); abstrak verbatim + 14 sitasi konsisten "bergantian", jadi sumbu #1
   bertahan — tapi belum terkunci. Butuh akses institusi.
4. Paper konferensi Energy-Aware sudah di-submit atau belum? Menentukan urutan dan
   apakah ia jadi prior work resmi.
