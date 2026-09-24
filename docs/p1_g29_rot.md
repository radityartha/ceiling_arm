# P1 / G29 — ROTASI gantry, OFFLINE: penyaring berrotasi, amplop struktur, oracle‴ rot ≠ 0

> Sesi 2026-09-23 malam, **OFFLINE, nol perangkat keras, tidak ada `ros2 launch`** (stack mock tidak
> dipakai). Sumber prompt: [p1_g28_hw.md §E](p1_g28_hw.md) (= g26 C1-2 a, b, d; (c) = tabel untuk operator).
> Kode/hasil: `docs/results/p1_g29/`. **Tidak diubah:** `sched.py`, `sched_coll`, alat G22, probe, peta,
> `interarm_collision.py`. `oracle2.py`: hanya argumen opsional `rot` (default identik, K0).

---

## A0. Langkah 0 — INSTRUMEN (dilihat SEBELUM §A dikunci; tidak ada aturan dipilih)

[g29_a0.py](results/p1_g29/g29_a0.py) → [log](results/p1_g29/g29_a0.log), [json](results/p1_g29/g29_a0.json).
URDF = G23 `reach_dwell_live.urdf` (sha `f02e7c53…` = `/tmp/reach_dwell_live.urdf` G22–G28).

### A0.1 Tempat rotasi = 0 diasumsikan (grep `rotation_joint|rot0|ROT0|restrict_rot0|REF.rot`)

| tempat | apa | sifat |
|---|---|---|
| `scripts/interarm_collision.py` `CrossGantryChecker` | 16 pasangan struktur–struktur (SS) t1×t2 **dibuang**; 660 pasangan ber-lengan dihitung | asumsi "rel sejajar" — batal pada rot ≠ 0 |
| 🔴 `scripts/reach_dwell_probe.py:555–561` (S18 di `_plan_and_screen`) | `want` = rel + sendi lengan saja → **sendi rotasi tidak diteruskan** → `q_from` dari `neutral` → S18 menilai rot = 0 **walau `start_joints` berisi rotasi** | **diam** — baru ditemukan sesi ini |
| `g22_plan.sweep_screen` (S24) | menyapu `t{g}_linear_joint` saja; rotasi dari `state` (bila ada) | 1-D |
| `sched_screen.walk:34`, `v28_screen.py:60`, `v_sweep.py:13`, g20 `v_screen.py:13`, g19 `v12.py:15` | `t{g}_rotation_joint = 0.0` keras | alat plan-only |
| `run_g22.py:178`, `rail_to_g.py:88` (S23) | **menolak** bila \|rot\| > 0.5° | penjaga eksplisit ✅ |
| `make_instance.restrict_rot0` (g22) → `make_instance_g23:62`, `make_instance_g24:155`, `g26_make` (pembungkus) | sumbu pose dipotong ke rot = 0 | instance |
| `make_instance_g24.l1_rot0` / `ROT0` | oracle‴ malas hanya untuk tuple rot = 0 | cache |
| `oracle2.model` (`q_ref_full` = `neutral`), `p1_g23/oracle.solve` | rotasi dikunci 0 | oracle |
| `sched_coll` (G9) | rot dimodelkan; lintasan traverse A2.3 memakai **arah-pendek** (< 180°) | ⚠ lihat A0.3 |
| `sched.py` | pose (lin, rot) penuh 33 × 72; `T_rot` terkunci | sadar rotasi |

**`REF.rot`** (peta `cap_g{1,2}_rail160.npz`): 72 nilai, −180° … +175° langkah 5°; L1 (`masks`) tersedia untuk
**semua** rot. Node kandidat G24 (780): L1-benar di rot 0 **780/780**; tuple L1 per rot 26 604 (±90°) … 27 659 (0°, 180°).

### A0.2 Geometri (URDF, pinocchio)

- Sumbu rotasi gantry g: vertikal di `(lin, y_g, 2.01)`, y₁ = +0.36, y₂ = −0.36; batang di dunia sepanjang
  `(−cos rot, −sin rot)` (= G9 A2.1 `u(rot)`, tanda arah batang tak relevan, simetris).
- Jari-jari horizontal maks dari sumbu (rot 0, verteks hull / primitif): pelat **455.0 mm**, batang 402.0,
  platform 244.0 (tidak berputar), **tiap lengan REST 439.9 mm** (hull). ⚠ G3 §B4 menulis "menggantung 0.52 m"
  — pose/ukuran lain (bukan hull REST); dipakai ulang di (c), bukan di sini.
- Pita z: platform [2.0175, 2.0825], batang [2.0025, 2.0175], pelat [1.9525, 2.0025], **lengan REST
  [0.926, 1.9525]** — kedua gantry di pita yang sama; lengan menggantung 1.03 m di bawah pelat.
- **SS pinocchio-3D (tanpa platform) vs `sched_coll.pair_distance` (G9 2-D, W0/W0b lulus):** 3 605 sampel
  (3 000 acak, 600 dekat-kontak, 5 terstruktur): keduanya > 0 pada 3 304, maks |beda| **0.0022 mm**; keduanya
  kontak 301; **beda tanda 0**; platform tidak pernah kontak saat G9 bebas (0). Dua implementasi independen cocok.
- Rot 0/0, lengan REST, per kelas (LL lengan–lengan, LS lengan–struktur, SS): dx 0 → LL 605.6 / LS 575.1 /
  **SS 380.0 (platform–platform)**; dx 0.8 → 498.9 / 587.7 / 539.5. ⚠ docstring `CrossGantryChecker` "tetap
  429 mm" **tidak cocok**: SS terukur 380.0 (platform) dan tidak tetap (bergantung dx bila platform dibuang).
- Sampel P1 (rot ∓90°, dx 0): SS **−30.0 mm** (pelat–pelat); dx 0.095: −0.0 (pelat–sisi batang) = ambang
  Lemma 2 terkoreksi G9 (0.095).

### A0.3 Batas rotasi (bandingkan; B0b G28: yaml bisa lebih longgar dari URDF)

| lapisan | batas posisi | lain |
|---|---|---|
| URDF `t{g}_rotation_joint` | revolute **±π** (±180°) | velocity 1.0 rad/s, effort 10 |
| `joint_limits.yaml` | **tidak ada** `has_position_limits` → URDF | `max_velocity` **1.0 rad/s = 57.3 °/s = 5.7×** motor 10 °/s; accel t1 1.0 rad/s², **t2 accel mati** (tidak seragam) |
| bridge (`dual_table_controller`) | `bridge.min_deg/max_deg` **−180 / +180** | tolak target di luar |
| layanan `move_dual_table` `_check_travel_limits` | ±180 (sama parameter) | — |
| motor (G3) | encoder absolut; langkah ke **target absolut** → **tidak membungkus**: −170 → +170 = 340° lewat 0 | 10.0 °/s @1000, 1000 °/s², t₀ 0.26 s |
| fisik | ⛔ **tidak diukur**: henti mekanis, lilitan kabel, keselarasan encoder 0 ↔ sejajar rel | — |

⚠ **Pertentangan (Rule 7):** G9 A2.3 `sched_coll` memakai **arah-pendek** untuk traverse rotasi; motor/bridge
bergerak ke target encoder absolut dalam [−180, 180] → melintas 0, **tidak** lewat ±180. Untuk |Δrot| > 180° kedua
model beda lintasan. Yang dipakai sesi ini: **encoder (tanpa bungkus)** — itu yang dieksekusi perangkat keras.
`sched_coll` ditandai untuk dibereskan (tidak diubah).

Lain-lain dicatat, di luar cakupan: SRDF mematikan lengan-sendiri vs platform/batang-sendiri ("Never"); rotasi
tidak mengubah pasangan intra-gantry (kedua lengan ikut batang yang sama) → `InterArmChecker` tidak terpengaruh.

---

## A. Protokol — ditulis 2026-09-23 22:40 SESUDAH A0, SEBELUM alat (a)/(b)/(d) ditulis dan dijalankan

> 🔒 §B menang atas §A; pertentangan DITULIS. Aturan/ambang dikunci di sini, tidak dipilih sesudah melihat hasil.

### A1. 🔒 Penyaring berrotasi `g29_rot_screen.py` (modul BARU; `interarm_collision.py` tidak diubah)

- `RotCrossChecker(CrossGantryChecker)`: hull sama (konstruktor induk), pasangan ber-lengan **indeks & urutan
  identik** dengan induk, **ditambah** 16 pasangan SS di belakang. `check_split(q, only)` →
  `(d_arm, pair_arm, d_ss, pair_ss)`; `check` = min keduanya (kompatibel `screen_trajectory`). `only` (prefiks
  lengan) → SS tidak ikut (statis selama rencana lengan). SS dihitung **selalu** (pada rot 0 SS ≥ 380 mm ≫ margin
  → verdict tidak berubah; itu diuji N3, bukan diasumsikan).
- **Nama sendi ketat:** nama yang tidak ada di model → `KeyError` (induk `q_from` diam-diam membuang — itulah
  kelas galat A0.1 🔴). Rotasi WAJIB ada di `state` (tidak ada default 0).
- `sweep_rot(chk, g, frm=(lin, rot), to=(lin, rot), state, mode)`:
  - **`rect` (verdict terkunci):** grid penuh `[lin₀, lin₁] × [rot₀, rot₁]`, `n_lin = max(2, ⌈|Δlin|/0.010⌉ + 1)`
    (= S24), `n_rot = 1` bila Δrot = 0 selain `max(2, ⌈|Δrot|/1°⌉ + 1)`; rotasi **tanpa bungkus** (A0.3).
    Menutup **setiap** lintasan monoton per sumbu (bridge + debounce, sumbu konkuren, kecepatan beda) — tidak
    bergantung model lintasan.
  - `path` (dilaporkan, bukan verdict): lintasan G9 A2.3 (waktu mati 0.29 / 0.26 s, 31.416 mm/s, 10 °/s,
    konkuren), disampel ≤ 10 mm dan ≤ 1°.
  - keluaran: verdict CLEAR / MARGIN / COLLIDE (margin 0.05 m = S18/S24), `d_min`, pasangan, indeks,
    `d_arm_min`, `d_ss_min`, jumlah titik.
- Diskret: 1° pada r = 1.16 m = 20 mm antar-sampel di ujung lengan terentang (7.7 mm pada REST 0.44 m) — di
  bawah margin 50 mm, bukan bukti (= batas yang sama dengan S24 lama).

### A2. 🔒 Kontrol (a) — semua WAJIB lulus sebelum (b)/(d) dipakai

| id | jenis | isi | lulus bila |
|---|---|---|---|
| **N1** | negatif | 2 000 keadaan acak (rel U[0, 1.6], rot U[−π, π] **dan** 500 dengan rot = 0/0, 12 sendi lengan U[batas URDF]) | `d_arm`, nama pasangan baru **==** `CrossGantryChecker.check` lama, **bit-identik**, 2 000/2 000 |
| **N2** | negatif, arsip | 14 traverse HW G24b/G26 (`*_run.json`), keadaan terukur direkonstruksi dari `/tmp/g24b_js.csv`, `/tmp/g26_js.csv` (sampel terakhir ≤ `t_send`) | (i) `sweep_screen` lama mereproduksi `sweep_min_mm` arsip \|Δ\| ≤ 1.0 mm (validasi rekonstruksi); (ii) `sweep_rot(rect)` rot 0→0: `d_arm_min` **bit-identik** dengan lama, verdict sama, 14/14 |
| **N3** | negatif | keadaan N1 rot 0/0 + N2 | `d_ss ≥ 0.380 − 1e-6` m semua → verdict baru == lama |
| **P1** | positif, SS statis | rot₁ = −90°, rot₂ = +90°, dx 0 / 0.09 / 0.10 | `d_ss ≤ 0` pada 0 dan 0.09; `d_ss > 0` pada 0.10 (G9 Lemma 2 terkoreksi 0.095) |
| **P2** | positif, sapuan | g2 ditahan rot +90°; g1 di lin sama berputar a → b melewati −90°, a/b dipilih (dicari numerik, dicatat) sehingga **kedua ujung ≥ 50 mm** | `sweep_rot` = COLLIDE di tengah; kedua titik ujung sendiri CLEAR |
| **P3** | positif, lengan vs struktur gantry lain | g2 rot +60° lin 0.8; arm_1 (g1 rot 0, lin 0.8) di-IK (posisi, `_ik`) ke titik di pelat/batang g2 | kelas LS ≤ 0 dengan pasangan `t1_a1_* ↔ t2_{rotation_link, mount_plate_*}` |
| **P4** | positif, rotasi benar-benar terpasang | FK `t{g}_a{1,2}_base_link` di model penyaring pada 200 (lin, rot) acak vs `irm_sweep.base_pose` | maks \|selisih\| ≤ 1e-9 m; + nama rotasi salah-eja → `KeyError` |

### A3. 🔒 (b) Peta jarak gantry–gantry, lengan REST

- Grid: `dx = lin₁ − lin₂ ∈ [−1.60, 1.60]` langkah 0.01 (321), `rot₁, rot₂ ∈ REF.rot` (72 × 72) → 1.66 × 10⁶
  konfigurasi; lin₂ = 0 kecuali dx < 0 (lin₁ = 0, lin₂ = −dx) — jarak hanya bergantung dx (invarian translasi x;
  **diuji** pada 200 sampel dengan lin mutlak lain, bit-identik tidak dituntut, |Δ| ≤ 1e-9).
- Disimpan per konfigurasi `d_LL, d_LS, d_SS` (float32, npz). Hull untuk lengan (= penyaring); SS primitif (exact).
- **Amplop (margin 50 mm, semua kelas):** (i) `θ_one` = rot maks sehingga **satu** gantry di |rot| ≤ θ (atau
  |rot − 180°| ≤ θ) aman untuk rot gantry lain **apa pun** dan dx apa pun (perluasan Lemma 1 G9 dengan lengan);
  (ii) `θ_both(dx)` = θ maks simetris |rot₁|, |rot₂| ≤ θ aman pada dx itu (dan min atas dx); (iii) fraksi pasangan
  (rot₁, rot₂) grid yang tidak aman pada dx terburuk; (iv) fraksi konfigurasi yang **SS ≤ margin tetapi
  lengan > margin** (= di mana penyaring lama **buta**) dan yang SS ≤ 0 tetapi lengan ≥ 50 mm.
- **Verifikasi amplop pada 1°:** klaim (i) dan (ii)-min diperiksa ulang pada grid 1° × dx 0.01 **di dalam** himpunan
  yang diklaim aman — satu pelanggaran = klaim dikoreksi ke nilai 1°.
- Hull vs mesh: 50 konfigurasi dengan d_arm terdekat ke 50 mm di batas amplop dihitung ulang dengan mesh
  (pasangan ber-lengan) → selisih dilaporkan (g20 B0: hull bisa +0.4 mm di atas mesh).
- Lengan di tugas: **tidak** dipetakan (tidak ada peta tetap; S18 per rencana = penjaga). Dinyatakan, bukan diasumsikan.

### A4. 🔒 (d) oracle‴ dengan rot ≠ 0

- `oracle2`: `model(arm, rot=0.0)`, `gravity(..., rot=0.0)`, `solve(..., rot=0.0)`. rot ≠ 0 → sendi
  `t{g}_rotation_joint` di acuan dikunci pada `rot`; cache model per `(arm, rot)`. Default **tidak berubah**.
- **K0:** 72 tuple dari `g24_oracle3_cache.jsonl` (seed 29, 36 ok / 36 tidak-ok, ≥ 12 z = 1.40): `solve` dengan
  argumen default **dan** `rot=0.0` eksplisit → `n_sol, n_roll, rounds, saturated, taumax, tilt_fail, ok`
  **==** cache (bit-identik), 72/72. Gagal ⇒ berhenti.
- **K-FK:** posisi `base_link` lengan di model tereduksi pada rot ∈ {±30, ±60, ±90} vs `irm_sweep.base_pose` ≤ 1e-9 m.
- **K-EQ (ekuivariansi, positif untuk tanda/rangka):** gravitasi invarian yaw ⇒ solve(xyz, L, arm, rot = r) ≡
  solve(xyz′, L, arm, 0) dengan `xyz′ = c + Rz(−r)(xyz − c)`, `c = (L, y_g)`; untuk r kelipatan 30° anchor roll
  terpetakan ke anchor (roll bergeser r). 24 tuple (r ∈ {+90°, −60°}): lulus bila `ok` sama **24/24**, `n_roll`
  sama ≥ 22/24, median |Δtaumax| ≤ 1e-3 N·m. (Urutan penemuan berbeda → bit-identik **tidak** dituntut.)
- **Sampel nilai rotasi:** rot grid **R = {±30°, ±60°, ±90°}**. Node: 150 dari 510 node tidak-layak-oracle‴ di rot 0
  (acak seed 29, **berlapis |y|**: proporsional), + 50 dari 270 node layak (acak seed 29). Per (node, r, gantry,
  slot) L1-benar: 2 indeks rel dengan jarak horizontal base→node **terkecil** (`irm_sweep.base_pose`) →
  oracle‴ (tilt, envelope, = `make_instance_g24._job`) + PATH oracle⁗ (`g27_oracle4.path_verdict`, m^p g27 T3a)
  untuk tuple ok. **Batas bawah** (pemilihan rel heuristik; lebih banyak rel hanya bisa menambah).
- Dilaporkan: per r dan per lapis |y|/z — node tidak-layak-rot-0 yang jadi layak (oracle‴; oracle⁗), node layak-rot-0
  yang tetap layak; per z = 1.40 PATH.
- Ini **kelayakan kinematik+torsi**, bukan kelayakan jadwal: tabrakan (b) dan S18 per rencana tidak ikut.

### A5. 🔒 (c) kecepatan ujung pada 10 °/s — TABEL, tidak diputuskan

`v = ω · r_h`, ω = 10 °/s = 0.1745 rad/s. `r_h` = jarak horizontal dari sumbu gantry ke: `tool_frame` REST, titik
hull REST terjauh, `tool_frame` tuple-ok terjauh (cache rot 0 + sampel A4), jangkauan teoretis (0.40 + jangkauan
horizontal maks lengan). Figur 250 mm/s (G3 §15c) dicantumkan sebagai rujukan, **bukan** ambang yang dipilih.

### A6. 🔒 Aturan

Nol HW; tidak ada `ros2 launch`. `sched.py`, `sched_coll`, alat G22, probe, peta, `interarm_collision.py`
**tidak diubah**. Disk ≥ 2.5 GB bebas (peta ≤ 30 MB). Paralel ≤ 15 proses.

### A7. 🔒 Dugaan D146–D156 — DITULIS SEBELUM alat dan data

Prior tally: mekanisme terukur → tepat; kode sendiri → meleset; "kendala lebih longgar" → meleset.

| D | dugaan | dasar |
|---|---|---|
| D146 | (b) konfigurasi dengan **SS ≤ 0 tetapi d_arm ≥ 50 mm** (lama buta pada kontak): **0** | lengan REST (440 mm) tergantung tepat di bawah pelat (455); kontak SS hanya di zona dekat ujung batang |
| D147 | (b) konfigurasi dengan 0 < SS < 50 mm dan d_arm ≥ 50 mm: **> 0** (SS mengubah verdict di pita tipis) | pelat menonjol 15 mm radial dari hull lengan |
| D148 | (b) satu gantry di rot 0: min (semua kelas, rot lain apa pun, dx apa pun) **≥ 150 mm** (Lemma 1 + lengan) | pelat terjauh y = 0.36 − 0.455 = −0.095, lengan lawan ≈ −0.30 |
| D149 | (b) `θ_both` (min atas dx, grid 5°) ∈ **{25°, 30°}** | G9 N1: SS butuh keduanya > 31.7°; lengan sedikit lebih lebar |
| D150 | (b) fraksi pasangan (rot₁, rot₂) grid tidak aman pada dx terburuk: **10 %–40 %** | kasar |
| D151 | N1 + N2(ii) lulus **bit-identik pada jalan pertama** | pasangan ber-lengan = urutan induk |
| D152 | P1–P4 lulus **pada jalan pertama** | — |
| D153 | K0 72/72 bit-identik **pada jalan pertama** | default tak disentuh |
| D154 | K-EQ lulus (ok 24/24) | invarian yaw gravitasi + geser roll |
| D155 | node tidak-layak rot 0 dengan \|y\| ≤ 0.11 (0/174 layak di rot 0): **≥ 50 %** jadi layak-oracle‴ pada suatu r ∈ R; \|y\| = 0.60 (0/73): **≥ 30 %** | A0: kelayakan ditentukan jarak horizontal base→node; rotasi menggeser base ±0.4 m di y |
| D156 | tuple z = 1.40 yang lulus **PATH (oracle⁗)** pada r ≠ 0 dalam sampel: **0** | G27: oracle⁗ ≡ tanpa z = 1.40 (transit dari REST) |

(c) tidak diberi dugaan (tabel untuk operator).

---

## B. Hasil terukur

> §A dikunci **2026-09-23 22:37**, sha256 `15c911383d25…` ([sectionA_locked.md](results/p1_g29/sectionA_locked.md));
> judul §A menulis "22:40" — yang berlaku adalah cap di berkas terkunci. Sesi terputus sekali (2026-09-23 23:00 → 2026-09-24
> 06:00, proses latar mati; peta dibangun ulang dari awal). Semua alat baru di `docs/results/p1_g29/`.

### B1. 🔴 Temuan pasca-kunci: hull `CrossGantryChecker` BUKAN hull — jarak S18/S24 G22–G28 terbaca terlalu JAUH

Ditemukan saat N2 (rekonstruksi arsip gagal 4/14), dikejar sampai akar, lalu diperbaiki di modul baru saja.

**Mekanisme (terukur, [g29_gjk_probe.log](results/p1_g29/g29_gjk_probe.log), [g29_hull_diag.log](results/p1_g29/g29_hull_diag.log)):**
`coal` di mesin ini **dibangun tanpa qhull** (`Convex.convexHull` → *"Library built without qhull"*). Karena itu
`buildConvexRepresentation(False)` di `CrossGantryChecker` tidak pernah membuat hull: objek `Convex` menyimpan **semua
verteks mesh** (gripper_base 12 707 titik; hull sejati 926) dengan adjacency mesh yang **tidak konveks**. Support
GJK memakai hill-climbing pada graf itu → macet di maksimum lokal → jarak **terlalu besar**. Contoh: `t1_a2_gripper_base`
↔ `t2_a2_gripper_base` hull-lama **195.89 mm**, mesh 54.38, verteks-verteks 54.38, hull sejati (scipy → `coal.Convex`) **54.38**.

Gejala yang terlihat lebih dulu, semuanya akibat mekanisme yang sama:
- **bergantung riwayat:** sapuan yang sama 543.48 mm (segar) vs 546.01 (sesudah sapuan lain) — petunjuk support dari
  panggilan sebelumnya; per pasangan +129 / −119 mm antar-urutan pada 26 % pasangan-titik satu sapuan arsip;
- `GeometryData` segar per titik / 5 000 iterasi GJK **tidak** menolong (sama-sama salah, cocok satu sama lain);
- pada 50 konfigurasi peta dekat 50 mm: pasangan (mesh < 150 mm) hull-lama − mesh maks **+141.5 mm**, > 0.5 mm pada
  1 946/2 436; min per konfigurasi **+18.4 mm** (rerata +6.9). Hull tidak pernah membaca **lebih dekat** dari kenyataan
  (N1: −0.00 mm) — galatnya satu arah, ke arah **tidak aman**.

**Perbaikan (modul baru `g29_rot_screen.true_hull`, `interarm_collision.py` TIDAK diubah — keputusan operator):** hull
scipy `ConvexHull` dari titik yang sama, diserahkan ke `coal.Convex(points, triangles)`. Verifikasi
([g29_hull_fix_check.log](results/p1_g29/g29_hull_fix_check.log)): hull ≤ mesh pada **2 436/2 436** pasangan
(maks +0.001 mm; min −2.53, konservatif); min per konfigurasi −0.33 … +0.001 mm; hangat vs segar, 400 keadaan × 676
pasangan: **0.003 mm** (ketergantungan riwayat hilang); 1.64 ms/keadaan. Modul tetap memakai data segar per evaluasi
(3.1 ms) sebagai sabuk kedua.

**Dampak pada penyaring lama** (apa adanya: hull palsu, `GeometryData` dipakai ulang lintas rencana seperti di probe):
- N1, 2 000 keadaan acak: lama − benar **+77.9 / −0.00 mm**, > 0.1 mm pada 1 396/2 000.
- [g29_gjk_diag2.log](results/p1_g29/g29_gjk_diag2.log), 105 lintasan arm_1 REST → sasaran di jalur bersama (mirip rencana S18):
  lama − benar maks **+61.5 mm**; verdict benar CLEAR 99 / MARGIN 3 / COLLIDE 3; **1 terbalik: benar COLLIDE (−0.5 mm),
  lama CLEAR (50.3 mm)**. Pada sampel ini penyaring lama **meloloskan satu tabrakan**.
- Arsip traverse HW (N2): nilai tercatat = hitung lama (14/14 tereproduksi ≤ 0.04 mm); nilai benar lebih kecil:
  G26 s1 g1 **543.5 → 514.7**, G26 s2 g2 546.1 → 514.8, G24b g2 464.6 → 455.2, G26 s1 g2 459.6 → 452.5 mm. Semua 14 tetap
  CLEAR (≫ 50 mm). "X0 REST × REST 546.1 mm" (g20/g22) sebenarnya **514.8 mm**.
- `InterArmChecker` (se-gantry) memakai **mesh** (BVH, bukan `Convex`) → mekanisme ini tidak berlaku; diag n = 2 lintasan:
  0 galat (n kecil, dinyatakan). g20 B0 "hull ≤ mesh + 0.4 mm" diukur pada hull palsu di 3 konfigurasi — tidak berlaku.
- MoveIt (FCL C++) tidak memakai objek ini; tidak diperiksa.

Operator (2026-09-24): **laporkan saja**, tanpa patch ke `interarm_collision.py` / probe di sesi ini. → §C.

### B2. (a) Penyaring berrotasi — kontrol A2

Empat jalan, semuanya dicatat ([g29_controls*.log](results/p1_g29/)):

| jalan | isi | hasil |
|---|---|---|
| 1 | hull palsu, data hangat | **crash** di P4 — kunci `arm_1` vs `irm_sweep.ARMS` `arm1` (kode sendiri) |
| 2 | hull palsu, data hangat | N1 2000/2000, N3, P1, P3, P4 lulus; **P2 GAGAL** (ujung-a terbaca 45.6 mm → MARGIN; pemilih ujung dan sapuan membaca jarak berbeda — gejala B1); N2 bit-identik 14/14 tetapi rekonstruksi arsip **10/14** |
| 3 | hull palsu, data segar | semua lulus; N2 rekonstruksi 14/14 (arsip dihitung dengan checker baru per `rail_to_g`, jadi riwayat per event) |
| **4 (berlaku)** | **hull sejati, data segar** | **semua LULUS** (tabel) |

| id | hasil jalan 4 |
|---|---|
| P4 | FK `base_link` vs `irm_sweep.base_pose` maks **2.2e-16 m**; nama salah-eja → `KeyError`; keadaan tanpa rotasi → ditolak ✅ |
| N1 | baru vs induk-dengan-hull-sejati: **0/2 000** beda (bit-identik, nama pasangan sama) ✅. (Pertentangan: §A menulis "vs `CrossGantryChecker` lama"; lama apa adanya salah — B1 — jadi pembanding bit-identik = induk dengan hull sejati + data segar, `OldFresh`.) |
| N2 | 14/14: (i) hitung lama apa adanya mereproduksi arsip ≤ 1 mm 14/14; (ii) sapuan baru rot 0→0 bit-identik dengan `OldFresh`, verdict sama ✅ |
| N3 | SS min pada rot 0/0 **380.000 mm** (500 keadaan + 14 traverse) ✅ |
| P1 | rot ∓90°: dx 0 SS **−30.0** (pelat–pelat), dx 0.09 **−0.0** (pelat–batang), dx 0.10 **+5.0** ✅ |
| P2 | g2 +90°, g1 lin sama berputar **−130° → −63°**: ujung 53.7 / 50.3 mm CLEAR; sapuan **COLLIDE −59.6 mm** di −106° (lengan −59.6, SS −30.0) ✅ |
| P3 | arm_1 IK ke pelat kanan g2 (rot +60°): LS **−34.4 mm** `t1_a1_right_finger_dist ↔ t2_mount_plate_right` ✅ (sasaran pelat kiri: IK tidak konvergen, dicatat) |


### B3. (b) Peta jarak gantry–gantry, lengan REST ([g29_map.py](results/p1_g29/g29_map.py) → `g29_map.npz` 3.5 MB, [analisis](results/p1_g29/g29_map_analyse.log), [json](results/p1_g29/g29_map.json))

1.66 × 10⁶ konfigurasi (dx 321 × rot₁ 72 × rot₂ 72), **hull sejati** (B1), data segar. Versi pertama dengan hull palsu
disimpan di `pseudohull_run/` (tidak dipakai; bedanya ditulis di kolom kanan).

| besaran (margin 50 mm, semua kelas) | hull sejati | (hull palsu) |
|---|---|---|
| invarian translasi (200 sampel, lin mutlak lain) | maks \|Δ\| **1.5e-14 m** ✅ | 2.2e-7 m (gagal ambang 1e-9) |
| **θ_one** — satu gantry dalam ±θ dari 0°/180° ⇒ aman untuk rot lain APA PUN, dx apa pun | **20°** (grid 5°); tersertifikasi Lipschitz **10°** | 20° / 15° |
| jarak min bila satu gantry tepat di 0°/180° (Lemma 1 G9 + lengan) | **96.2 mm** | 96.2 |
| **θ_both** simetris \|rot₁\|, \|rot₂\| ≤ θ, min atas dx | **35°** (di dx 0.44); **verifikasi 1°: 1 618 161 konfigurasi, min 82.4 mm, 0 < 50 mm** ✅ | 35° |
| θ_both(\|dx\|) | 0.0: 40 · 0.1: 45 · 0.2: 55 · 0.3: 45 · 0.4: 40 · 0.5: **35** · 0.6: 40 · **≥ 0.8: 180 (bebas)** | sama |
| pasangan (rot₁, rot₂) tidak aman pada dx terburuk / konfigurasi tidak aman | **43.7 %** / 4.4 % | 43.7 / 4.3 |
| SS ≤ 0 tetapi lengan ≥ 50 mm (penyaring lama buta pada KONTAK) | **0** | 28 (artefak hull palsu) |
| 0 < SS < 50 mm dan lengan ≥ 50 mm (SS mengubah verdict) | **6 708** (mis. dx −0.6, rot −50/−50: SS 47.8, lengan 63.3) | 12 160 |
| kelas pengikat pada konfigurasi tidak aman | LL 34 548 · SS 38 380 · LS 0 | 28 172 / 43 896 / 0 |
| hull vs mesh (50 konfigurasi lengan ≈ 50 mm) | hull − mesh **+0.001 / −0.279 mm** ✅ | +18.4 mm |

- Simetri 180° terukur (θ_both sekitar 180° = sekitar 0°; contoh buta muncul berempat): rot + 180° menukar dua lengan REST
  identik → jejak sama.
- **Sertifikat Lipschitz** (pertentangan dengan A3, lihat B6): d berubah ≤ 0.455·(|Δrot₁| + |Δrot₂|) + |Δdx| (jari-jari
  maks titik gantry dari sumbu 455 mm, A0.2) → sel grid 5° × 0.01 aman kontinu bila jarak grid ≥ 50 + 44.7 mm.
  θ_one 1° **tidak** diverifikasi rapat (≈ 9 × 10⁶ konfigurasi); angka kontinu yang sah untuk θ_one = **10°**.
- Lengan di pose tugas **tidak dipetakan** (A3): amplop ini hanya untuk **lengan REST**. Lengan terentang menjangkau jauh
  lebih lebar — S18 per rencana (dengan rotasi diteruskan + hull sejati) tetap penjaga wajib.
- Aturan praktis yang didukung data (untuk operator/scheduler, bukan keputusan): (1) **|dx| ≥ 0.8 m → rotasi bebas**;
  (2) salah satu gantry ≤ 10° dari 0/180 → gantry lain bebas; (3) keduanya ≤ 35° dari 0° (atau keduanya dari 180°; campuran 0/180 tidak diuji) → aman di dx apa pun.

### B4. (d) oracle‴ dengan rot ≠ 0 ([g29_oracle_rot.py](results/p1_g29/g29_oracle_rot.py), [K](results/p1_g29/g29_oracle_k.log), [laporan](results/p1_g29/g29_oracle_report.log), cache `g29_oracle_rot_cache.jsonl` 5 696 tuple)

`oracle2.py`: `model(arm, rot=0.0)`, `gravity(…, rot=0.0)`, `solve(…, rot=0.0)`; rot ≠ 0 mengunci `t{g}_rotation_joint` di acuan.
Default tidak berubah (diff 20+/14−, hanya parameter + kunci cache model).

| kontrol | hasil |
|---|---|
| **K0** | jalan 1: 53/72 — **pembanding saya salah** (19 tuple `n_sol = 0` → taumax NaN, NaN ≠ NaN; salinan oracle2 PRA-G29 memberi 53/72 yang sama, 0 beda `ok`, 0 beda nilai hingga). Jalan 2 (pembanding NaN-sadar): **72/72 default, 72/72 `rot=0.0` eksplisit**, 12 di z = 1.40 ✅ |
| **K-FK** | `base_link` model tereduksi pada rot ∈ {±30, ±60, ±90} vs `irm_sweep.base_pose`: maks **2.2e-16 m** ✅ |
| **K-EQ** | 24 tuple (rot +90° / −60° vs sasaran diputar di rot 0): `ok` sama **24/24**, `n_roll` sama 24/24, median \|Δτ\| **0**, maks 1.0e-10 N·m ✅ — ekuivariansi yaw tepat, tanda/rangka rotasi benar |

**Nilai rotasi** — 150 node tidak-layak-oracle‴ di rot 0 (berlapis \|y\|) + 50 layak; R = {±30°, ±60°, ±90°}; 2 rel terdekat
per (node, r, gantry, slot) L1-benar → **batas bawah**:

| | node | layak oracle‴ pada suatu r ∈ R | oracle⁗ (PATH) |
|---|---|---|---|
| tidak-layak di rot 0 | 150 | **83 (55 %)** | **82** |
| layak di rot 0 | 50 | 28 (tetap layak di rot 0 — rotasi tidak wajib) | 28 |

Per r (tidak-layak rot 0, o‴/o⁗): +30° 48/48 · −30° 56/56 · +60° 42/42 · −60° 42/42 · +90° 46/45 · −90° 46/45.

Per lapis (tidak-layak rot 0: n → o‴ / o⁗):

| \|y\| | 0.035 | 0.106 | 0.176 | 0.247 | 0.318 | 0.388 | 0.459 | 0.529 | 0.600 |
|---|---|---|---|---|---|---|---|---|---|
| n → layak | 27 → **25**/25 | 24 → **18**/17 | 21 → 10 | 17 → 4 | 3 → 0 | 2 → 0 | 11 → 0 | 24 → 10 | 21 → **16** |

| z | 1.00 | 1.08 | 1.16 | 1.24 | 1.32 | 1.40 |
|---|---|---|---|---|---|---|
| n → o‴ / o⁗ | 16 → 16/16 | 26 → 19/19 | 26 → 21/21 | 26 → 13/13 | 31 → 13/13 | 25 → **1 / 0** |

- **Mekanisme (A0 terkonfirmasi):** kelayakan ditentukan jarak horizontal base→node; rotasi menggeser base ±0.4 m di y →
  jalur tengah (\|y\| ≤ 0.11, 0/174 layak di rot 0) dan tepi (\|y\| = 0.60) terbuka. Node dekat rel (\|y\| 0.32–0.46) yang
  tidak layak di rot 0 tetap tidak layak (0/16) — sebabnya bukan jarak y.
- **z = 1.40 tetap tertutup:** 2 tuple oracle‴-ok, **0 lulus PATH** (= G27: transit dari REST).
- Ekstrapolasi kasar (proporsional, bukan terukur): 510 × 83/150 ≈ 282 node tambahan → cakupan node kandidat ≈ **(270 + 282)/780 ≈ 71 %**
  vs **34.6 %** di rot 0. Ini **kelayakan kinematik+torsi saja**: amplop tabrakan B3 (lengan REST) dan S18 per rencana
  tidak ikut; jalur tengah justru tempat kedua gantry saling mendekat.
- Tuple layak terjauh dari sumbu dalam sampel: r_h 609 mm (B5).

### B5. (c) Kecepatan lengan pada 10 °/s — TABEL untuk operator ([g29_tipspeed.log](results/p1_g29/g29_tipspeed.log)); TIDAK diputuskan

ω = 10 °/s = 0.1745 rad/s, v = ω · r_h (r_h = jarak horizontal dari sumbu gantry):

| titik | r_h | v |
|---|---|---|
| `tool_frame`, REST | 407.5 mm | **71.1 mm/s** |
| titik hull lengan terjauh, REST | 439.9 mm | 76.8 mm/s |
| `tool_frame`, tuple layak-oracle‴ terjauh (cache rot 0, 1 510 tuple) | 630.0 mm | **110.0 mm/s** |
| `tool_frame`, tuple layak-oracle‴ terjauh (sampel G29 rot ≠ 0) | 609.1 mm | 106.3 mm/s |
| teoretis, z = 1.40 (0.40 + √(1.005² − 0.5525²)) | 1 239.5 mm | 216.3 mm/s |
| teoretis, terentang horizontal (0.40 + 1.005) | 1 405.0 mm | **245.2 mm/s** (= "~244" g26 C1) |

Rujukan (bukan ambang terpilih): figur 250 mm/s (G3 §15c). ⚠ G3 §B4 "menggantung 0.52 m → 91 mm/s" tidak cocok dengan
REST terukur di sini (407.5 / 439.9 mm) — pose "menggantung" G3 bukan REST ini; tidak dikejar.
Operator memutuskan: rotasi hanya dengan lengan REST (≤ 77 mm/s), atau juga dengan lengan di tugas (≤ 110 mm/s pada
tuple layak), atau batas kecepatan rotasi lain.

### B6. Pertentangan §B lawan §A

| # | Pertentangan |
|---|---|
| (1) | **A1 diubah pasca-kunci:** hull induk diganti hull sejati (B1) + `GeometryData` segar per evaluasi. Tanpa ini penyaring baru mewarisi galat satu arah hingga +78 mm. |
| (2) | **N1/N2(ii)** dikunci "== `CrossGantryChecker` lama bit-identik"; lama apa adanya salah → pembanding = induk dengan hull sejati + data segar (`OldFresh`). Perbandingan dengan lama apa adanya dilaporkan terpisah (B1). |
| (3) | N2(i) rekonstruksi arsip hanya lulus bila checker lama dibuat **baru per event** (riwayat, B1) — jalan 2 10/14, jalan 3–4 14/14. |
| (4) | A3 "θ_one diverifikasi 1°" **tidak** dijalankan (≈ 9 × 10⁶ konfigurasi); diganti sertifikat Lipschitz → θ_one kontinu **10°** (grid 20°). θ_both 1° dijalankan sesuai A3. |
| (5) | Invarian translasi ≤ 1e-9: gagal dengan hull palsu (2.2e-7), **lulus** dengan hull sejati (1.5e-14). |
| (6) | K0 jalan 1 gagal karena pembanding NaN (kode sendiri), bukan oracle; §A "gagal ⇒ berhenti" — diagnosis dilakukan sebelum melanjutkan, jalan 2 72/72. |
| (7) | Kontrol jalan 1 crash (kunci `arm_1` vs `arm1`), jalan 2 P2 gagal (gejala B1). |
| (8) | Judul §A "22:40" vs cap berkas terkunci 22:37 (berkas menang). |
| (9) | Sesi terputus (23:00 → 06:00); peta pertama mati di 20/321 dan dibangun ulang; tidak ada data parsial yang dipakai. |
| (10) | B1 (hull palsu) **tidak diduga** di §A — temuan pasca-data, tidak dihitung di tally. |

### B7. Papan skor D146–D156

| D | dugaan | hasil | nilai |
|---|---|---|---|
| D146 | SS ≤ 0 & lengan ≥ 50 mm: 0 | **0** (hull sejati; hull palsu memberi 28 artefak) | ✅ tepat |
| D147 | 0 < SS < 50 & lengan ≥ 50: > 0 | **6 708** | ✅ tepat |
| D148 | satu gantry di rot 0: min ≥ 150 mm | **96.2 mm** | ❌ meleset (lebih dekat) |
| D149 | θ_both ∈ {25°, 30°} | **35°** | ❌ meleset (kendala lebih LONGGAR dari dugaan) |
| D150 | pasangan tidak aman 10–40 % | **43.7 %** | ❌ meleset (tipis) |
| D151 | N1 + N2(ii) bit-identik jalan pertama | jalan 1 crash sebelum N1; jalan pertama yang menjalankan N1 (2): 0/2 000 beda, N2(ii) 14/14 | ✅ tepat (dengan catatan) |
| D152 | P1–P4 lulus jalan pertama | jalan 1 crash (P4, kode sendiri), jalan 2 P2 gagal | ❌ meleset |
| D153 | K0 72/72 jalan pertama | 53/72 (pembanding sendiri) | ❌ meleset |
| D154 | K-EQ lulus | 24/24, Δτ 0 | ✅ tepat |
| D155 | \|y\| ≤ 0.11 ≥ 50 %; \|y\| = 0.60 ≥ 30 % | **43/51 (84 %)**; **16/21 (76 %)** | ✅ tepat |
| D156 | z = 1.40 lulus PATH: 0 | **0** (2 ok‴) | ✅ tepat |

**G29: 5 meleset / 6 tepat.** Pola: tepat = mekanisme terukur di A0 (jarak y → kelayakan, transit z = 1.40, ekuivariansi);
meleset = **kode sendiri** (D152, D153 — lagi) dan **besaran geometri** tanpa hitung (D148–D150). D149 meleset ke arah
"kendala lebih longgar" — berlawanan dengan prior lama.

---

## C. Keadaan akhir (Rule 12)

- **Nol perangkat keras, nol `ros2 launch`** (stack mock tidak dipakai; `ros2 node list --no-daemon` tidak relevan). Proses
  latar sesi ini selesai semua; tidak ada crash dump. Disk ≈ 2.9 GB (sesi memakai ≈ 15 MB; penurunan 3.7 → 2.9 GB terjadi
  di luar sesi ini).
- **Diubah:** `docs/results/p1_g24/oracle2.py` — hanya argumen opsional `rot` (K0 72/72). **Baru:** `docs/results/p1_g29/`.
  **Tidak diubah:** `sched.py`, `sched_coll`, alat G22, probe, peta, `interarm_collision.py`, `joint_limits.yaml`.
- 🔴 **Untuk operator — dilaporkan, TIDAK dipatch (keputusan operator 2026-09-24):**
  1. `CrossGantryChecker` (S18/S24 antar-gantry, dipakai G20–G28 dan **akan dipakai V28 / G28-ON**) memakai hull palsu →
     jarak terlalu jauh hingga +78 mm; 1/105 lintasan uji: tabrakan nyata terbaca CLEAR (B1). Hasil G22–G26 di sel nyata
     tidak berubah verdict (traverse min benar ≥ 452 mm), tetapi **verdict S18 rencana tugas tidak dapat diaudit ulang**
     (lintasan tidak disimpan sebelum V28). Perbaikan siap: `g29_rot_screen.true_hull`.
  2. S18 di probe (`reach_dwell_probe.py:555`) tidak meneruskan sendi rotasi → menilai rot = 0 (A0.1). Wajib diperbaiki
     sebelum rencana lengan apa pun pada rot ≠ 0.
  3. `joint_limits.yaml` rotasi `max_velocity` 1.0 rad/s = 5.7× motor; t2 accel mati (A0.3).
  4. `sched_coll` arah-pendek vs motor tanpa bungkus (A0.3).
- **Tidak diukur:** amplop dengan lengan di pose tugas; θ_one pada 1°; campuran 0°/180°; batas rotasi fisik (henti, kabel,
  keselarasan encoder 0); `InterArmChecker` mesh hanya n = 2; nilai rotasi untuk **jadwal** (hanya kelayakan node);
  PATH oracle⁗ pakai m^p G27 (dikalibrasi rot 0; gravitasi invarian yaw, dinamika lintasan tidak diuji).
- Anggaran token Rule 6 (30k/sesi) **terlampaui jauh** — tugas multi-jam; dilaporkan, bukan disembunyikan.

---

## D. Prompt G30 (salin ke chat BARU) — HW tahap (e): ±10° lengan REST

**Rekomendasi: Opus, effort TINGGI** — gerak pertama sumbu rotasi di jalur ros2_control; dua penyaring yang dipakai diketahui
cacat (B1, A0.1) dan harus diperbaiki + diuji sebelum gerak. Kesalahan berikutnya kemungkinan diam lagi.

```
Sesi G30 -- ROTASI gantry di sel NYATA, tahap (e): +-10 deg, lengan REST. Repo ceiling_arm, branch
feat/rgbd-topo-deploy. Operator di lokasi.

BACA PENUH: CLAUDE.md; docs/p1_g29_rot.md (semua, terutama B1, B3, C); docs/p1_g28_hw.md B0b, C;
docs/p1_g22_hw.md A5 (tahap 0); docs/p1_g3_timing.md (rotasi); scripts/interarm_collision.py;
docs/results/p1_g29/g29_rot_screen.py; docs/results/p1_g18/rot_home.py; dual_table_controller.py (bridge).

GERBANG (semua, SEBELUM gerak apa pun; tanya operator bila belum):
 G1. joint_limits.yaml <= URDF (G28 B0b) sudah diperbaiki, DAN G28-ON selesai atau ditunda eksplisit operator.
 G2. Keputusan operator atas G29 C-1 (hull palsu CrossGantryChecker) dan C-2 (probe tanpa sendi rotasi):
     patch atau tidak. Tahap (e) sendiri hanya butuh sweep_rot (g29_rot_screen, hull sejati) -- tapi
     JANGAN jalankan rencana lengan pada rot != 0 sebelum C-2 diperbaiki.
 G3. Operator menjawab (c) kecepatan (g29 B5) dan batas fisik rotasi (henti, kabel, keselarasan encoder 0).

1. KUNCI §A (sebelum gerak): hipotesis, kriteria sukses, dugaan D157+ (prior tally G29: kode sendiri
   meleset, besaran geometri tanpa hitung meleset).
2. Alat BARU rot_to_g.py (salinan pola rail_to_g/rot_home): S12 lengan REST, S23' |rot| target <= 10 deg,
   sweep_rot rect dari keadaan TERUKUR (kedua rotasi dari /joint_states, nama ketat) -> tolak bila bukan CLEAR;
   trajektori cosinus pada gantry_{g}_with_arm_controller hanya sendi rotasi; DRY dulu.
3. Tahap 0 g22 A5 (LED, origin, sel kosong); bring-up enable_gantry_bridge:=true (SIG_DFL, cek SigIgn).
4. Per gantry, satu per satu: 0 -> +10 -> 0 -> -10 -> 0 deg. Ukur: rot terukur vs perintah, waktu (T_rot model
   0.26 + d/10), drift lengan, drift rel lain, torsi lengan, arming/debounce bridge; keselarasan visual encoder 0.
5. Laporan §B/§C, p1_state + tally, prompt G31 (jadwal dengan rotasi, amplop B3 + S18 berrotasi).
ATURAN: rel tidak digerakkan; rotasi <= 10 deg; satu gantry bergerak; B menang atas A dan DITULIS; matikan stack
(kill -INT launch, node list --no-daemon kosong, hapus crash dump milikmu); disk ~3 GB.
```
