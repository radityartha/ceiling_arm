<!-- locked 2026-09-23 22:37 sha256 15c911383d25c7a6d509021349ee8ad5b6690fd2072dfd690b510ca2098a8715 -->
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
