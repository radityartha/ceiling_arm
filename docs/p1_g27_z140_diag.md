# P1 / G27 — diagnosis terima-palsu oracle‴ di z = 1.40, lalu (sekunder) P1″ non-tugas

> Sesi 2026-09-23, **OFFLINE, nol perangkat keras, tidak ada `ros2 launch`.** Sumber prompt:
> [p1_g26_hw.md §D](p1_g26_hw.md). Kode/hasil: `docs/results/p1_g27/`.

---

## A0. Langkah 1 — INSTRUMEN (dilihat SEBELUM §A dikunci; tidak ada aturan dipilih)

[g27_table.py](results/p1_g27/g27_table.py) → [g27_table.json](results/p1_g27/g27_table.json),
[log](results/p1_g27/g27_table.log): **setiap** baris `task … @rail …: VERDICT` di saringan
g22/g23/g24/g26 (+ RNEA `joint_2` rencana dari baris `torsi RNEA+offset` sebelumnya) dan setiap
tugas yang **dieksekusi** (G24b, G26: RNEA hidup + τ terukur), digabung ke tuple oracle‴
`(node, g, p, slot)` dari `g24_oracle3_cache.jsonl` (semua baris ketemu di cache; tidak ada yang
dihitung ulang). Batas statis oracle‴ `joint_2` = 14 − 7.7 − m₂ = **5.641**; perencana menolak
bila RNEA > 6.30.

| himpunan | rencana | verdict | RNEA j2 rencana | RNEA − o_j2 (maks statis oracle‴) |
|---|---|---|---|---|
| tuple **diterima** oracle‴, z < 1.40 (saringan + eksekusi, G24b/G26) | **145** | PLANNED 121 / SUCCESS 24 / **TORQUE 0** | ≤ **5.49** | ≤ **+0.84** |
| tuple **diterima** oracle‴, **z = 1.40** (G26, "S16") | **16** (12 tuple) | PLANNED 5 / **TORQUE 11** | PLANNED **2.41–3.74**; TORQUE **6.43–8.14** | TORQUE **+0.81…+2.57** |
| (sama tuple, G24b) s0 t5 arm_1 @0.75 | 1 | TORQUE | 7.17 | +1.61 |

- **Tuple yang sama memberi kedua kelas**: s0 t5 arm_1 @0.75 PLANNED 2.41 / 2.54, TORQUE 7.18 (G26)
  dan 7.17 (G24b); s6 t4 arm_1 @1.30 2.45 / 7.36; s7 t4 arm_3 @1.45 3.74 / 6.75. RNEA **bimodal**
  (2.4–3.7 vs 6.4–8.1, celah 2.7 N·m). Beda kelas ada **di dalam sampel perencana** (titik akhir
  atau lintasan), bukan di tuple.
- **Start:** 9/11 TORQUE z = 1.40 direncanakan dari **REST** (s7 t4, s8 t4 dari akhir tugas
  sebelumnya) → start non-REST bukan penjelas utama.
- Tuple-ok z = 1.40 di cache: **53 / 6 154** (0.9 %); median o_j2 **5.61** (bawah batas 0.03);
  z < 1.40 median o_j2 4.98–5.27. `n_sol` median 120 (z < 1.40: 243–541).
- `tilt_fail / (8·n_sol)` pada tuple-ok: z 1.00 **0.230**, 1.08 0.045, 1.16 0.114, 1.24 0.193,
  1.32 0.214, **1.40 0.225** — **tidak** terkonsentrasi di z = 1.40.
- Set kalibrasi margin C′ (81 rencana dieksekusi, g18–g20) per z: n = 19 / 8 / 16 / 31 / **6 / 1**
  (z 1.00 … 1.32 / **1.40**); Δ_final j2 (RNEA − statis(q_akhir)) median −0.035 / −0.084 / −0.034 /
  −0.018 / **+0.306** / **+0.422**. m₂ = 0.659 ditetapkan di z = 1.24.

**TIDAK dapat dipilah dari data yang ada (ditulis sebelum diagnosis):**

1. Lintasan rencana saringan **tidak disimpan** — baik konfigurasi **titik akhir** maupun **titik
   jalan** tempat puncak RNEA terjadi tidak diketahui; log hanya memberi puncak skalar per sendi.
2. Bagian statis vs dinamis (v, a) puncak RNEA per rencana tidak terpisah.
3. Cabang IK / roll tempat perencana mendarat tidak diketahui.
4. **Tidak ada** eksekusi di z = 1.40 pada G24b/G26 (tak ada τ terukur); C′ punya 1.
5. Satu sampel per rencana; kelas stokastik (butir di atas) → per tuple, laju TORQUE sejati tidak
   diketahui (1–3 sampel).

---

## A. Protokol — ditulis 2026-09-23 SESUDAH A0, SEBELUM uji diagnosis apa pun (langkah 3)

> 🔒 Dikunci sebelum satu pun nilai T0–T3 dihitung. §B menang atas §A dan pertentangannya ditulis.
> **DILARANG** memilih/mengubah aturan oracle sesudah melihat hasilnya pada S16. Aturan koreksi per
> hipotesis dikunci **di sini**; hipotesis mana yang diadopsi diputuskan oleh uji mekanisme A2 (ambang
> terkunci), **bukan** oleh skor aturan pada S16. Skor pada S16 dilaporkan sebagai **dalam-sampel**.

### A1. 🔒 Hipotesis bersaing

Fakta pengikat (A0): RNEA rencana TORQUE 6.43–8.14 > o_j2 5.37–5.63 untuk tuple yang **sama**.
Titik akhir lintasan berparameter waktu diam (v = 0) → RNEA di titik akhir ≈ gravitasi statis.
Maka puncak RNEA > 6.3 berasal dari **(E)** titik akhir di luar himpunan yang oracle‴ enumerasi,
atau **(N)** titik **bukan-akhir** (lintasan / dinamika).

| # | Hipotesis | Kelas | Ramalan pembeda |
|---|---|---|---|
| **H1** | Newton-miring tak konvergen (g24 B′2): titik akhir miring ≤ 2° dengan j2 > 5.64 terlewat | E | kontinuasi miring yang kokoh menaikkan amplop j2 tuple S16 ke ≥ RNEA TORQUE |
| **H2** | Cabang IK terlewat (benih / grid roll): titik akhir tepat-vertikal dengan j2 > 5.64 tidak ditemukan | E | pencarian lebih rapat (benih ×4, roll 1°) menemukan solusi j2 ≥ 6.3 pada target tepat |
| **H3** | Margin m₂ (puncak lintasan − statis akhir) dikalibrasi di z < 1.40; di z = 1.40 **lintasan** dari REST melewati konfigurasi j2 jauh di atas titik akhir | N | maks Ω (A2-T0) < RNEA TORQUE; lintasan proksi dari REST menembus 6.3 jauh lebih sering di z = 1.40 |
| **H4a** | Toleransi posisi 2 mm perencana (oracle di posisi tepat) | E | hanya relaksasi posisi yang membawa maks Ω ke ≥ 6.3 |
| **H4b** | Model: RNEA probe (model penuh, sendi lain `neutral`) ≠ gravitasi oracle (model tereduksi, lengan lain REST) | — | selisih statis di konfigurasi yang sama > 0.01 N·m |

Hipotesis tidak saling eksklusif; A2 menilai masing-masing.

### A2. 🔒 Uji pembeda (offline, pinocchio atas URDF G23 sha `f02e7c53…`, tanpa ROS)

**Himpunan uji S12** = 12 tuple S16 (A0). **Kontrol lapis** = tuple-ok cache di z = 1.32 dan
z ≤ 1.24: 12 tuple per lapis z ∈ {1.00, 1.08, 1.16, 1.24, 1.32}, dipilih `default_rng(27)` dari
tuple-ok cache lapis itu yang **pernah** direncanakan PLANNED (A0) bila ≥ 12, dilengkapi acak
dari tuple-ok lapis itu.

- **T-model (H4b):** 50 konfigurasi acak (`default_rng(27)`) × 4 lengan: |τ₂ probe-model statis −
  τ₂ oracle| maks. H4b benar ⇔ > 0.01.
- **T0 (E vs N) — maks gravitasi j2 atas daerah tujuan perencana.** Ω(tuple) = {q dalam batas:
  ‖p(q) − p_t‖ ≤ 2 mm, ∠(z_alat, −z) ≤ **2.83°** (kerucut yang **memuat** kotak x/y 2°), roll bebas}.
  `G_Ω` = maks |τ₂| atas Ω, SLSQP (scipy) multi-start dari **setiap** solusi oracle‴ tuple itu
  (grid roll, tanpa miring) + 64 benih acak `default_rng(27)` diproyeksikan Newton 6-D (A1 g24,
  roll acak). Ditambah `G_0` (posisi tepat, tepat vertikal, roll bebas — daerah oracle‴ tanpa miring)
  dan `G_t` (posisi tepat, kerucut 2.83°).
  - Per rencana TORQUE `c`: **N-terbukti** ⇔ `G_Ω(tuple) < RNEA(c) − m₂` (m₂ = 0.659 = seluruh selisih puncak − statis-akhir
    teramati di C′, termasuk suku dinamis titik akhir); **E-mungkin** sebaliknya.
  - E-mungkin → atribusi: `G_0 ≥ RNEA − m₂` → **H2**; lain `G_t ≥ RNEA − m₂` → **H1**; lain → **H4a**.
- **T1 (H1 langsung):** per solusi oracle‴ tuple S12, 8 miring A′1 dengan **kontinuasi**
  (0.5° → 1° → 1.5° → 2°, Newton 50 iterasi tiap langkah dari langkah sebelumnya). Dilaporkan:
  miring gagal lama → kini konvergen, dan amplop j2 baru − lama.
- **T2 (H2 langsung):** `oracle2.solve` dengan `ROUNDS = (16,16,32,64,128,256)`, minimal 4 ronde,
  grid roll **1°**, tilt+envelope (A′). Dilaporkan: n_sol, amplop j2 baru − lama.
- **T3 (H3, proksi lintasan).** Proksi = **garis lurus ruang-sendi** REST → q_akhir, 101 titik,
  maks gravitasi statis per sendi `P_j`.
  - **T3a (validasi proksi pada C′, set pengembangan):** 81 rencana dieksekusi, q_akhir terukur:
    laporkan `RNEA₂ − P₂` (median, maks) dan `RNEA₂ − statis_akhir₂` untuk perbandingan.
    Proksi **LAYAK** ⇔ maks (RNEA₂ − P₂) ≤ maks Δ_final (0.659) (proksi tidak memperburuk margin).
  - **T3b:** untuk tiap tuple S12 + kontrol lapis: `P₂` atas **setiap** solusi oracle‴ (grid roll,
    tanpa miring). Laporkan per lapis z: fraksi solusi dengan `P₂ + 7.7 + m₂ᵖ > 14`, `m₂ᵖ` dari
    A3-H3. Dan per rencana S16: apakah `[min P₂, maks P₂]` tuple itu memuat RNEA rencana (PLANNED
    **dan** TORQUE).

**Aturan adopsi (terkunci):**

- **H3 DIADOPSI** ⇔ N-terbukti pada **≥ 6 / 11** rencana TORQUE S16 **dan** T3a LAYAK.
- **H1 DIADOPSI** ⇔ T0 atribusi H1 pada ≥ 3/11, **atau** T1 menaikkan amplop j2 > 0.30 pada ≥ 3/12 tuple.
- **H2 DIADOPSI** ⇔ T0 atribusi H2 pada ≥ 3/11, **atau** T2 menaikkan amplop j2 > 0.30 pada ≥ 3/12 tuple.
- **H4a DIADOPSI** ⇔ atribusi H4a ≥ 3/11. **H4b** ⇔ T-model > 0.01 (bug → diperbaiki dulu, semua diulang).
- Lebih dari satu diadopsi → aturannya **digabung** (konjungsi). Tidak satu pun → oracle **tidak**
  diubah; laporan "tidak ditentukan", G28 = kumpulkan lintasan (A6).

### A3. 🔒 Aturan koreksi per hipotesis (oracle⁗) — dikunci SEKARANG

Semua: oracle‴ apa adanya (grid 5°, benih, toleransi, margin A2′, tilt+envelope) **ditambah**:

- **H3 → PATH:** tuple ditolak kecuali **∀ solusi oracle‴ s (grid roll, tanpa miring)**:
  `∀ j: P_j(REST → s) + offset_j + m_jᵖ ≤ lim_j`, dengan
  `m_jᵖ = max(0, max_{c ∈ C′} (RNEA_j(c) − P_j(REST → q_akhir(c))))` (aturan A2′: maks, tanpa
  pembulatan). ∀-cabang, sama dengan TORQ′. Tidak dimodelkan: start non-REST (tugas ke-2 lengan
  yang sama) — tetap syarat PERLU.
- **H1 → MIRING-KONTINUASI:** amplop A′1 dihitung dengan kontinuasi T1; miring yang tetap gagal
  sesudah kontinuasi **dihitung** dan tuple ditolak bila ada solusi dengan gagal miring **dan**
  j2 statis ≥ batas − 0.5 (titik amplop yang terlewat di dekat batas).
- **H2 → BENIH/ROLL:** T2 (ROUNDS diperluas, roll 1°) menggantikan penemuan g24.
- **H4a → POSISI:** tiap solusi + 6 perpindahan target ±2 mm (±x, ±y, ±z), Newton 6-D 50 iterasi,
  masuk amplop.

**Kontrol yang MENGGERBANG oracle⁗:**

| # | Kontrol | Harus |
|---|---|---|
| **K0** | `oracle2.solve(tilt=True, envelope=True)` default pada **72** tuple cache (60 `default_rng(27)` + 12 S12) | **bit-identik** dengan `g24_oracle3_cache.jsonl` (n_sol, n_roll, rounds, saturated, taumax, tilt_fail, ok) |
| **K1** | oracle⁗ ⊆ oracle‴ (monoton) pada semua tuple yang dihitung | 0 pelanggaran |
| **K2** | oracle⁗ pada 145 rencana diterima-z<1.40 (A0) | menerima **≥ 50 %** tuple distinct-nya (kalau di bawah: aturan terlalu ketat — **dilaporkan**, tidak dilonggarkan) |
| **K3** | oracle⁗ pada C′ (81 dieksekusi aman) | dilaporkan (tuple C′ di rel/node peta bila ada) |

### A4. 🔒 Set VALIDASI — tidak dipakai untuk memilih; prediksi terkunci untuk G28 (plan-only)

**V28** = (a) **setiap** tuple-ok oracle‴ di z = 1.40 dalam cache yang **tidak** pernah direncanakan
(53 − 12 − tuple lain yang pernah direncanakan), dan (b) 20 tuple-ok oracle‴ di z = 1.32 yang tidak
pernah direncanakan, `default_rng(28)`. Per tuple, oracle⁗ memberi terima/tolak **sekarang**
(kalau oracle diubah). G28 merencanakan tiap tuple plan-only **3×** dari REST (rel di L, lengan lain
REST, `_plan_and_screen` apa adanya). Prediksi terkunci: oracle⁗ **tolak** ⇒ ≥ 1/3 TORQUE-UNSAFE;
oracle⁗ **terima** ⇒ 3/3 PLANNED. Presisi/recall oracle⁗ di V28 = hasil G28.

### A5. 🔒 Prediksi laju lolos (iv) — ditulis di §B bila oracle diubah

(a) 46 seed (i)–(iii) G26, jadwal P1′ apa adanya: seed yang **setiap** tuple (tugas, rel, lengan)
jadwalnya diterima oracle⁗ → prediksi lolos (iv). (b) tugas tetap, masker `reach` := oracle⁗,
P1′ di-solve ulang: (i)–(iii) dihitung ulang. Tarik-ulang tugas dari node layak-oracle⁗ (g24 A5)
= G28, bukan sesi ini (butuh oracle⁗ pada node baru; dicatat).

### A6. 🔒 Langkah 4 (sekunder) — P1″, dikunci SEBELUM fit

```
P1'' = P1' (g25_constants.json APA ADANYA) kecuali:
  traverse → c_rovh'' + r · T_cmd(Δ),  r = 0.95017 TETAP;  c_rovh'' = median_i (wall_i − r · T_cmd,i)
             atas traverse jadwal rail_to_g yang dieksekusi (G24b 2 + G26 12 = 14)
  retract dilewati (keberangkatan pertama, lengan REST) → c_skip = median wall retract-dilewati terukur
  c_task, c_ret TIDAK diubah.
Evaluasi: leave-one-seed-out atas 4 seed (G24b s1, G26 s1/s2/s13): konstanta dari 3 seed lain,
prediksi makespan seed ke-4; dilaporkan galat P1' vs P1''.
```

### A7. 🔒 Dugaan D123–D131 — DITULIS SEBELUM T0–T3

Prior (tally): dugaan dari **mekanisme terukur** tepat; tentang **kode/aturan sendiri** meleset;
"kendala lebih longgar" meleset di G26 (D115).

| # | Dugaan | Dasar |
|---|---|---|
| **D123** | T-model: selisih ≤ 1e-6 (H4b salah) | g24 B0 model tereduksi = penuh 1.8e-15; `neutral` untuk jari = acuan oracle |
| **D124** | T0: N-terbukti pada **≥ 8 / 11** rencana TORQUE S16 (H3 diadopsi) | RNEA bimodal pada tuple sama (A0); o_j2 sudah memuat miring sudut; tilt_fail tidak terkonsentrasi di 1.40; C′ Δ_final naik dengan z |
| **D125** | T2: amplop j2 naik ≤ 0.30 pada **≥ 11 / 12** tuple S12 (H2 salah) | g24 NC7 90/90, PC2′ 80/81; PC3 22/22 |
| **D126** | T1: kontinuasi memulihkan ≥ 50 % miring gagal; amplop naik ≤ 0.30 pada ≥ 11/12 (H1 salah) | torsi ≈ linear dalam miring kecil (A′1); sudut kotak sudah dihitung |
| **D127** | T3a LAYAK: maks (RNEA₂ − P₂) di C′ ≤ 0.659 | titik akhir termasuk dalam garis lurus → P₂ ≥ statis akhir |
| **D128** | T3b: fraksi solusi dengan proksi-lintasan tak aman di z = 1.40 **≥ 3×** fraksi di z ≤ 1.24 | lapis teratas = lengan harus naik melewati bahu mendatar dari REST menggantung |
| **D129** | oracle⁗ (bila H3): tuple-ok z = 1.40 tersisa **≤ 15 / 53** | median o_j2 z = 1.40 sudah 5.61 (A0) |
| **D130** | oracle⁗ K2: menerima **≥ 70 %** tuple distinct diterima-z<1.40 | lintasan di z < 1.40 tidak menembus (145/145 aman) |
| **D131** | P1″ LOSO: \|galat\| makespan rerata 4 seed **< 4 %** (P1′: 5.6 %) | bias −23 s/seed ada di suku yang dikalibrasi ulang (g26 B2) |

---

## B. Hasil terukur

> §A dikunci 2026-09-23 16:23, sha256 `49e1d10917e2…` ([sectionA_locked.md](results/p1_g27/sectionA_locked.md)).
> Kode: [g27_diag.py](results/p1_g27/g27_diag.py) (K0, T-model, T0–T3), [g27_oracle4.py](results/p1_g27/g27_oracle4.py)
> (oracle⁗ + K1–K3, V28, A5), [g27_p1pp.py](results/p1_g27/g27_p1pp.py) (A6). `oracle2.solve` mendapat
> 3 argumen opsional (`rounds_n`, `min_rounds`, `tilt_cont`); **default tidak berubah**.

### B0. Kontrol

| | Hasil |
|---|---|
| **K0** (72 tuple, A3) | **72/72 bit-identik** ([log](results/p1_g27/g27_k0.log)); diperluas: **1510/1510** tuple-ok cache dihitung ulang bit-identik ([log](results/p1_g27/g27_oracle4_cache.log)) |
| **T-model (H4b)** | maks \|τ probe-statis − τ oracle\| = **2.7e-15** N·m → H4b **salah** ([log](results/p1_g27/g27_tmodel.log)) |

### B1. Uji pembeda

**T0 — E vs N** ([log](results/p1_g27/g27_t0.log), [json](results/p1_g27/g27_t0.json)): **N-terbukti 11/11** rencana TORQUE.

| rencana TORQUE (S16) | RNEA₂ | RNEA₂ − m₂ | o_j2 | G_0 | G_t | **G_Ω** |
|---|---|---|---|---|---|---|
| rentang 11 rencana | 6.43–8.14 | 5.77–7.48 | 5.37–5.63 | 5.34–5.59 | 5.38–5.80 | **5.40–5.83** |
| paling ketat: s3 t4 arm_1 @0.65 | 6.43 | 5.77 | 5.62 | 5.58 | 5.63 | **5.65** (celah 0.12) |

Maksimum gravitasi `joint_2` atas **seluruh daerah tujuan perencana** (2 mm, kerucut 2.83° ⊇ kotak 2°,
roll bebas; 128–380 titik akhir layak per tuple) tidak pernah mencapai RNEA rencana − m₂. **Puncak torsi
yang ditolak bukan di titik akhir** — tidak ada cabang, kemiringan, atau toleransi posisi yang
menjelaskannya.

**T1 — H1** ([log](results/p1_g27/g27_t1.log)): amplop j2 berubah **−0.001** pada 12/12; kontinuasi hanya
memulihkan **208 / 3 090** miring gagal (6.7 %) — kegagalan miring **bukan** divergensi Newton (paling
mungkin batas sendi; tidak dipilah). H1 **tidak diadopsi**.

**T2 — H2** ([log](results/p1_g27/g27_t2.log)): benih 512 (vs 128), roll 1°, min 4 ronde → solusi **5×**,
amplop j2 berubah **+0.000** pada 12/12. H2 **tidak diadopsi**.

**T3a — proksi lintasan pada C′** ([log](results/p1_g27/g27_t3.log)): garis lurus ruang-sendi REST → q_akhir
terukur: RNEA₂ − P₂ median **−0.022**, maks **+0.054** (vs RNEA₂ − statis-akhir maks +0.659). Di C′ proksi
**lebih tepat** dari titik akhir → **LAYAK**; `m_jᵖ = [0.220, 0.054, 0.005, 0.009, 0.008, 0]`.

**T3b — per lapis z** (12 tuple-ok per lapis; batas P₂ > 14 − 7.7 − 0.054 = **6.246**):

| z | solusi | fraksi P₂ > 6.246 | P₂ median / maks | (P₂ − statis-akhir) median |
|---|---|---|---|---|
| 1.00 | 5 538 | **0.000** | 3.60 / 5.29 | 0.00 |
| 1.08 | 4 788 | 0.000 | 2.52 / 5.40 | 0.00 |
| 1.16 | 4 322 | 0.000 | 3.58 / 5.38 | 0.00 |
| 1.24 | 3 553 | 0.000 | 3.98 / 5.41 | 0.00 |
| 1.32 | 2 826 | 0.000 | 4.20 / 5.48 | 0.00 |
| **1.40** | 1 735 | **1.000** | **7.19 / 7.54** | **+1.70** |

Di z ≤ 1.32 puncak garis lurus **adalah** titik akhir; di z = 1.40 **setiap** garis lurus dari REST ke
**setiap** solusi melewati `joint_2` 6.38–7.54 N·m — 1.7 N·m di atas titik akhir. 7/11 RNEA TORQUE jatuh
di [min P₂, maks P₂ + m₂ᵖ]; s3 (6.43) dan s4 t1 (6.45) sedikit di bawah min, s8 (8.14, **start t3**, bukan
REST) di atas.

➜ **Aturan adopsi A2: H3 DIADOPSI** (N 11/11 ≥ 6, T3a LAYAK). H1, H2, H4a, H4b tidak.

**Mekanisme (klaim yang boleh ditulis):** *oracle‴ memodelkan torsi statis di konfigurasi AKHIR. Di lapis
z = 1.40 (0.65 m di bawah gantry) transit dari REST menggantung melewati konfigurasi dengan torsi gravitasi
`joint_2` 1.7 N·m di atas konfigurasi akhir; rute perencana acak — rendah pada 5/16 sampel (RNEA 2.4–3.7),
tinggi pada 11/16 (6.4–8.1). Di z ≤ 1.32 puncak lintasan = titik akhir (0/21 027 solusi).*

⚠️ Proksi garis lurus **bukan** lintasan perencana: 5 rencana PLANNED z = 1.40 (RNEA 2.4–3.7) berada **di
bawah** min P₂ 6.38 — perencana kadang menemukan rute terlipat yang aman. Aturan PATH ∀-solusi karena itu
**konservatif** (tolak-palsu) di z = 1.40, sesuai kebijakan (terima-palsu lebih mahal).

### B2. Oracle⁗ = oracle‴ ∧ PATH ([log](results/p1_g27/g27_oracle4.log), [json](results/p1_g27/g27_oracle4.json), cache [g27_oracle4_cache.jsonl](results/p1_g27/g27_oracle4_cache.jsonl))

| z | tuple cache | oracle‴ benar | **oracle⁗ benar** |
|---|---|---|---|
| 1.00 / 1.08 / 1.16 / 1.24 / 1.32 | 21 505 | 882 / 267 / 155 / 90 / 63 | **sama persis** |
| **1.40** | 6 154 | 53 | **0** |

Pada seluruh 27 659 tuple yang pernah dihitung, **oracle⁗ ≡ oracle‴ minus lapis z = 1.40**. Node layak
270 → **232** / 780.

| Kontrol | Hasil |
|---|---|
| K1 (oracle⁗ ⊆ oracle‴) | 0 pelanggaran (menurut konstruksi, dicek) ✅ |
| K2 (≥ 50 % tuple distinct diterima-z<1.40) | **53/53** ✅ |
| K3 (C′, 81 dieksekusi aman) | oracle‴ terima 29, oracle⁗ terima **29** (PATH menolak 0 dari 81) |
| S12 dalam-sampel (bukan uji) | oracle⁗ terima **0/12** |

**V28 (A4) — prediksi TERKUNCI untuk G28:** 41 tuple z = 1.40 tak-pernah-direncanakan → oracle⁗
**tolak 41/41** ⇒ tiap tuple ≥ 1/3 TORQUE-UNSAFE; 20 tuple z = 1.32 (`default_rng(28)`) → **terima 20/20**
⇒ tiap tuple 3/3 PLANNED. Daftar di `g27_oracle4.json` `V28`.

**A5 — laju lolos (iv) yang diprediksi:**
(a) jadwal P1′ G26 apa adanya: **17/46** seed setiap tuple-nya diterima oracle⁗ — **tepat** himpunan seed tanpa
tugas z = 1.40 (46/46 sepakat). (b) tugas tetap, `reach` := oracle⁗: (i) **18/50**, (i)–(iii) **17/50** —
himpunan yang sama, jadwal **identik** dengan G26 (makespan sama). Seed baru untuk G28:
**16, 17, 18, 20, 21, 22, 23, 26, 29, 33, 35, 36, 40, 42** (14; seed 1/2/13 sudah dijalankan).
Tarik-ulang tugas dari node layak-oracle⁗ (untuk memulihkan 50 seed) **tidak** dikerjakan — node baru
butuh oracle‴ + PATH (~6 jam lazy, g24 B′2).

### B3. P1″ (A6) — leave-one-seed-out ([log](results/p1_g27/g27_p1pp.log))

| seed ditahan | c_rovh″ (fit 3 seed lain) | c_skip | terukur | P1′ | **P1″** |
|---|---|---|---|---|---|
| G24b s1 | 7.700 (n 12) | 2.502 | 480.44 | −4.80 % | **−3.57 %** |
| G26 s1 | 7.734 (11) | 2.503 | 557.69 | −11.93 % | **−9.87 %** |
| G26 s2 | 7.734 (9) | 2.497 | 585.59 | −0.41 % | **+3.00 %** |
| G26 s13 | 7.541 (10) | 2.502 | 550.59 | −5.62 % | **−2.67 %** |
| \|galat\| rerata | | | | **5.69 %** | **4.78 %** |

Fit penuh: `c_rovh″` **7.700** s (n = 14; g25: 4.74), `c_skip` **2.502** s (n = 5). Bias non-tugas sebagian besar
hilang (3/4 seed membaik), tetapi s2 berbalik tanda — galat total P1′ s2 kecil karena **pembatalan** (g26 B1);
sisa galat = derau suku tugas (\|galat\| per tugas 8.2 s, g26 B2). Tetapan G24b T_cmd diambil dari input
model (pred − c_rovh)/r, bukan T_cmd terukur.

### B4. Papan skor — D123–D131

| # | Dugaan | Terukur | |
|---|---|---|---|
| D123 | T-model ≤ 1e-6 | 2.7e-15 | ✅ |
| D124 | N-terbukti ≥ 8/11 | **11/11** | ✅ |
| D125 | T2 amplop ≤ 0.30 pada ≥ 11/12 | 12/12 (+0.000) | ✅ |
| D126 | kontinuasi pulihkan ≥ 50 % miring gagal ∧ amplop ≤ 0.30 | pulih **6.7 %** (amplop ✓) | ❌ |
| D127 | T3a LAYAK | maks +0.054 | ✅ |
| D128 | fraksi z = 1.40 ≥ 3× z ≤ 1.24 | 1.000 vs 0.000 | ✅ |
| D129 | oracle⁗ z = 1.40 ≤ 15/53 | **0/53** | ✅ |
| D130 | K2 ≥ 70 % | 100 % | ✅ |
| D131 | P1″ LOSO < 4 % | 4.78 % | ❌ |

**G27: 2 meleset / 7 tepat → papan skor 61 meleset / 68 tepat.** Ketujuh yang tepat dari mekanisme yang
**diukur di langkah 1** (bimodal pada tuple sama, tilt_fail tak terkonsentrasi, Δ_final naik dengan z). D126
meleset tentang **kode sendiri** (miring gagal ternyata bukan Newton); D131 besaran dari n = 4 dengan derau
per tugas 8 s — seperti D117.

### B5. Pertentangan §B lawan §A — G27

| # | Pertentangan |
|---|---|
| (1) | T0 run 1: kendala posisi SLSQP dalam m² (2.5e-7) di bawah toleransi solver → titik akhir sedikit tak layak dan dibuang; 2 tuple G = −1 **salah dilabeli N** ([log run 1](results/p1_g27/g27_t0_run1_bug.log)). Diperbaiki (skala mm, titik awal layak ikut dihitung, G < 0 → TIDAK-DITENTUKAN), dijalankan ulang; kelas **tidak** berubah (11/11 N). Perbaikan solver, bukan aturan |
| (2) | K0 run 1: 21/72 "beda" = `NaN != NaN` pada tuple n_sol = 0; pembanding diperbaiki (NaN = NaN, lainnya eksak) → 72/72 |
| (3) | A2-T3b "memuat RNEA rencana" salah bentuk untuk PLANNED: proksi garis lurus bukan lintasan perencana (5 PLANNED di bawah min P₂) — B1 ⚠️ |
| (4) | A3-PATH "∀ solusi dari REST" tidak memodelkan start non-REST; s8 t4 (start t3) RNEA 8.14 > maks P₂ + m₂ᵖ |
| (5) | G_t < o_j2 sampai 0.05 pada 4 tuple (miring sudut oracle ± 0.5° toleransi dapat keluar kerucut 2.83°; atau optimum lokal) — margin kelas ≥ 0.12, kelas tidak berubah |
| (6) | A6 "T_cmd,i": G24b memakai T_cmd input model, G26 dari prediksi terkunci — keduanya input model, bukan T_cmd terukur |
| (7) | Aturan adopsi dijalankan pada S16 (set yang sama yang memilih hipotesis) — sesuai A2, tetapi karena itu **validasi sejati = V28 di G28**, bukan B2 |

**G27: TUJUH.**

---

## C. Keadaan akhir (Rule 12)

- **Nol perangkat keras**, tidak ada `ros2` diluncurkan. Disk 3.7 GB bebas di akhir sesi.
- **Diubah:** `results/p1_g24/oracle2.py` — 3 argumen opsional (default = perilaku lama; K0 1510/1510 +
  72/72 bit-identik). **Tidak diubah:** `sched.py`, peta, `make_instance*.py`, alat G22, probe,
  `g24_oracle3_cache.jsonl`, konstanta P1′.
- **Baru:** `results/p1_g27/` — tabel langkah 1, diagnosis T0–T3, oracle⁗ (cache 1510 tuple), P1″ LOSO.
- **Tidak dikerjakan / tidak diketahui:**
  - lintasan perencana nyata di z = 1.40 (proksi garis lurus saja; **G28 harus menyimpan lintasan**);
  - sebab miring gagal (T1: bukan Newton; batas sendi paling mungkin, tidak dipilah);
  - tarik-ulang tugas dari node layak-oracle⁗ (50 seed) — hanya 17 seed tersedia tanpa itu;
  - oracle⁗ **tidak** menyelamatkan z = 1.40: ia membuangnya. Rute terlipat aman ada (5/16) — perbaikan
    sisi perencana (kendala lintasan / titik antara terlipat) = pilihan terbuka, keputusan operator.
- 🔴 **Utang dikunci operator (g26 C1), urutan tidak berubah:** sesudah z = 1.40 → **ROTASI gantry**
  (C1-2: S18/S24 + sendi rotasi, tabrakan struktur rot ≠ 0, kecepatan ujung ~244 mm/s, oracle rot ≠ 0,
  HW bertahap); lalu **peta lingkungan 3D** (C1-3: keempat octomap updater gagal dimuat di setiap bring-up;
  G22–G26 = sel kosong).

## D. Prompt G28 (salin ke chat BARU saat operator di lokasi) — HW, PLAN-ONLY

**Kenapa ini berikutnya.** G27 menjelaskan terima-palsu z = 1.40 (puncak torsi di **transit**, bukan di
titik akhir; 11/11) dan memberi oracle⁗ (≡ oracle‴ tanpa lapis z = 1.40). Tetapi hipotesis dipilih pada
set yang sama; validasi sejati = **V28** (61 tuple tak pernah direncanakan, prediksi terkunci) + (iv) pada
**14 seed baru**. Keduanya plan-only (nol gerak lengan). Sesudah itu: ROTASI (g26 C1-2).

**D132 (dikunci di sini, SEBELUM G28; dinilai G28):** pada setiap rencana V28 z = 1.40 dengan
RNEA j2 > 6.30, indeks titik puncak RNEA j2 **<** titik terakhir **dan** gravitasi statis j2 di titik akhir **≤ 5.83**
(maks G_Ω T0). Dasar: T0 11/11, T3b.

**Rekomendasi: Opus, effort TINGGI** — pertama kali lintasan perencana disimpan dan dianalisis; kesalahan
paling mungkin diam (salah membaca titik puncak, atau "memvalidasi" dengan set yang dipakai memilih).

```
Sesi G28 -- validasi oracle'''' (G27) di stack NYATA, PLAN-ONLY: V28 + saringan (iv) 14 seed baru.
Repo ceiling_arm, branch feat/rgbd-topo-deploy. Operator di lokasi (bring-up butuh lengan hidup).

BACA PENUH: CLAUDE.md; docs/p1_g27_z140_diag.md (A0, A4, B1-B5, C); docs/p1_g26_hw.md A1-A2, B0, C1;
docs/p1_g22_hw.md A5 (tahap 0).

1. Tahap 0 g22 A5 (operator: LED, origin, sel kosong -- tanya ULANG; remount_check; bring-up
   enable_gantry_bridge:=true; 4x Actuator count; 7/7 controller). Rel terbaca -> R1 (g26 A1).
2. KUNCI sebelum data: alat v28_screen.py (baru; DRY dulu): per tuple V28 (g27_oracle4.json 'V28'),
   _plan_and_screen APA ADANYA dari start REST (rel di L, lengan lain REST, start_joints), 3 sampel,
   dan SIMPAN lintasan (joint_trajectory: posisi, v, a) + RNEA per titik. Dugaan D133+ dikunci.
   SUDAH dikunci G27 (nilai, jangan tulis ulang): V28 A4 (tolak => >=1/3 TORQUE; terima => 3/3 PLANNED)
   dan D132 (G27 D: puncak RNEA j2 > 6.30 di titik INTERIOR, statis akhir <= 5.83).
3. V28 plan-only (61 tuple x 3; ~80 menit). Laporkan presisi/recall oracle'''' vs perencana,
   dan posisi puncak RNEA di lintasan (interior vs akhir) per z.
4. (iv) sched_screen --plan docs/results/p1_g26/g26_candidates.json --repeats 3 pada seed
   16 17 18 20 21 22 23 26 29 33 35 36 40 42 (jadwal identik G26; p0 = rel terbaca). Prediksi G27 A5:
   semuanya tanpa z = 1.40 -> kelas penolakan TORQUE z=1.40 = 0; laju lolos dikunci sebelum saringan.
5. Tidak ada eksekusi jadwal (replikasi selesai G26) kecuali operator minta.
6. p1_state 6 + tally; prompt G29 = ROTASI gantry offline (g26 C1-2 a-d), lalu 3D map (C1-3).

ATURAN: B menang atas A dan DITULIS; nol gerak lengan (plan-only); launch python3 -c "...SIG_DFL...execvp" &,
cek SigIgn sebelum kill -INT; ros2 node list --no-daemon kosong di akhir; disk ~3 GB, hapus crash dump milikmu.
```
