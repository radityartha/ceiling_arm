# P1 / G26-HW — replikasi jadwal scheduler P1′ di sel NYATA, K seed, prediksi TERKUNCI

> Sesi 2026-09-23, operator di lokasi. Sumber prompt: [p1_g25_task_cost.md §D](p1_g25_task_cost.md).
> Kode/hasil: `docs/results/p1_g26/`. Alat G22 (`sched_screen`, `run_g22`, `rail_to_g`,
> `return_rest`, `reach_dwell_probe`) **tidak diubah**.

---

## A. Protokol — ditulis 2026-09-23 SEBELUM solve apa pun, SEBELUM bring-up, SEBELUM gerak

### A0. Keputusan operator (dijawab SEBELUM §A ini ditulis)

| # | Pertanyaan | Jawaban |
|---|---|---|
| 1 | Tahap 0 (ditanya ULANG): LED 4/4 tidak merah; origin g1 **dan** g2 = home fisik; rel + ruang antar-gantry bebas s.d. ~1.6 m; nol alat di sel | **semua benar** |
| 2 | Syarat (ii) g22 A2 ("≥ 1 pindah di SETIAP gantry") dinilai pada jadwal mana | **jadwal P1′** (yang dieksekusi); FISIK dihitung dan dilaporkan saja |
| 3 | K = jumlah seed yang dijalankan | **K = 3** (operator boleh berhenti lebih awal; dilaporkan) |

Pra-cek (nol gerak): nol proses ROS; enp112s0 192.168.2.100; `ros2_kortex` e712295;
disk 4.1 GB (S27 ≥ 1.5 ✅); `JOINT_TORQUE_OFFSET_NM[2]` = 7.7 (A8-1 g22).

### A1. 🔒 Instance — g24 A5 dengan `p0` = rel TERBACA

```
untuk seed = 0 … 49 (urut):
  nodes, rejects = g24_candidates.json[seed]      -- tarikan oracle‴ TIDAK bergantung p0
  inst = make_instance_g24.instance(seed, nodes, rejects, cache) APA ADANYA, kecuali
         R1 p0 = rel TERBACA di bring-up, dibulatkan ke grid 0.05 m (g24 E8 → harapan 0.00 / 0.00)
         (disuntik lewat restrict_rot0; make_instance_g24.py tidak diubah)
  x    = run_sched45.p1prime(inst, serial=True), konstanta g25_constants.json APA ADANYA
  (i)   solve_exact(x) layak
  (ii)  jadwal P1′ punya ≥ 1 pindah di SETIAP gantry            (A0-2)
  (iii) sched_coll.schedule_conflict(inst, P1′ stops) = None
  (iv)  sched_screen --plan g26_candidates.json --repeats 3 APA ADANYA
```

- **Kontrol KG1 (menggerbang):** dengan `p0` disuntik = 0.55 / 0.00 dan biaya FISIK, jalur
  G26 harus mereproduksi `g24_candidates.json` — makespan FISIK **dan** jadwal
  `(rail, tugas)` — **bit-identik** pada 45/45 seed lolos. Dengan biaya P1′ ia harus
  mereproduksi `g25_sched45.json` `P1'.opt` 45/45.
- **Kontrol KG2:** tiap optimum P1′ diputar ulang oleh `g0_gate.eval_schedule` (== `finish`).
- **Kunci json untuk alat:** alat G22 membaca jadwal dari `schedule_FISIK` dan `p0` dari
  `decomp.FISIK.<g>.legs`. Di `g26_candidates.json` kunci itu **berisi jadwal P1′** (dan
  `decomp` jadwal P1′) — nama lama dipertahankan agar alat tidak diubah. Jadwal FISIK sejati
  (p0 baru) disimpan di `schedule_FISIK_true`. Ini ditulis sebagai pertentangan di §B.
- Bila rel terbaca ≠ 0.00 / 0.00 sesudah pembulatan: candidates **dibuat ulang** dengan nilai
  terbaca sebelum (iv); hasil hitung awal dibuang dan dicatat.

### A2. 🔒 Pemilihan seed + urutan

(iv) dijalankan seed demi seed urut atas seed lolos (i)–(iii), sampai **K = 3** lolos atau
seed habis. Seluruh (iv) selesai **sebelum** gerak pertama (keadaan awal saringan =
keadaan awal tiap run: rel `p0`, keempat lengan REST — sama untuk semua seed). Seed dijalankan
dalam urutan lolos. Kurang dari 3 lolos → yang lolos dijalankan, dilaporkan. Nol lolos →
nol gerak.

### A3. 🔒 Prediksi — per seed, DIKUNCI sebelum gerak pertama sesi

Per event dari `g22_plan.events(row)` (urutan A3 g22 — sama dengan runner), konstanta
g25 **tidak dikalibrasi ulang**:

```
tugas    → c_task (43.646); tugas TERAKHIR run → c_task − t_end (43.475)
retract  → c_ret (50.874), ATAU 0 bila kedua lengan gantry itu masih REST
           (belum ada tugas di gantry itu sejak awal run: P1′ t_fold_first)
traverse → c_rovh (4.7395) + r (0.95017) · T_cmd(Δ)
makespan P1′-serial = Σ event (sela runner = 0)
```

Dilaporkan juga: P1′ paralel (`max_g`), dan P1 lama (FISIK, `T_lin`) untuk jadwal yang sama.

### A4. 🔒 Eksekusi — g24 E4–E5 apa adanya, per seed

1. (sekali, sebelum seed pertama) `return_rest` DRY lalu **`--move`** kedua gantry (izin operator).
2. `rail_to_g` DRY tiap traverse seed itu; `run_g22 --dry --plan g26_candidates.json
   --out-dir /tmp/g26_s<S> --archive docs/results/p1_g26 --prefix g26_s<S>_`.
3. **GERAK** `run_g22` sama tanpa `--dry` (izin operator). Penilai + perekam hidup (SigIgn dicek).
4. Retract penutup kedua gantry (`return_rest --move`, tidak dihitung) lalu **rel kembali ke
   `p0`** tiap gantry yang pindah: `rail_to_g --seed S 0.000` DRY → `--move` (izin operator;
   `p0` ada di `allowed_rails`). Keadaan awal seed berikut = rel `p0`, lengan REST.
5. Dekomposisi: `decomp_g24b.py` diadaptasi (prefix + daftar event) → per komponen.

Tidak ada tahap 2 terpisah (rel g2 sudah bergerak G24b; S25 sudah dipenuhi) — operator tetap
melihat carriage pada tiap traverse.

### A5. 🔒 Besaran yang DILAPORKAN — terlepas dari hasilnya

Per seed: jadwal (rel, tugas, lengan), pindah per gantry, jumlah retract yang dilewati;
per event terukur / prediksi / galat (s, %); makespan terukur vs P1′-serial, P1′-par, P1;
per tugas `start, P→M, R→X, exec, →succ` + torsi puncak `joint_2` + RNEA rencana hidup;
per traverse `t_traverse / T_cmd` + overhead alat; per retract dilewati: wall + rc.
Gabungan: galat makespan per seed, |galat| per tugas rerata atas seluruh tugas G26,
galat per jenis event. (iv): per seed hasil + kelas penolakan; per rencana PLANNED.

### A6. 🔒 Dugaan D114–D122 — DITULIS SEBELUM solve, (iv), dan gerak

Prior (tally): dugaan dari **mekanisme terukur** tepat; dari **kode sendiri** / "kendala
lebih mengikat" meleset.

| # | Dugaan | Dasar |
|---|---|---|
| **D114** | (i)–(iii) di bawah `p0` 0/0 + P1′: **≥ 45 / 50** | tarikan tugas sama; g1 dari 0.00 → tugas g1 (rel ~0.5–1.3 di G24) memaksa pindah lebih sering, bukan lebih jarang |
| **D115** | K = 3 seed lolos (iv) dalam **≤ 8** seed (i)–(iii) pertama | G24b: lolos ke-2; D99 95 % per rencana; terima-palsu oracle‴ (seed 0) ada |
| **D116** | tiap seed yang dijalankan: **6/6 SUCCESS**, nol HALTED, nol auto-stop S26 | G24b 6/6; saringan (iv) 3/3 sudah meloloskan rencananya |
| **D117** | galat makespan P1′-serial dalam **±10 %** pada **setiap** seed yang dijalankan | G24b −4.8 %; tetapi sebagian pembatalan (g25 B2) — taruhan |
| **D118** | terukur **>** prediksi pada **≥ 2 / 3** seed | bias semua komponen non-tugas di bawah (alat G22+ lebih lambat dari G19: overhead rel 6.7–8.7 vs 4.74, `start` 2.5 s, saringan +2.6) |
| **D119** | \|galat\| per tugas rerata atas semua tugas G26 **≥ 5 s** | satu konstanta tidak mengikuti panjang lintasan (saringan ∝ exec, g25 A1) |
| **D120** | tiap retract "keberangkatan pertama dengan lengan REST": `return_rest` mencetak "sudah di rest", rc 0, wall **≤ 4 s** (model 0) | jalur kode: `wait_joints` → cek < 0.5° → keluar; `start` retract G24b 2.37–2.68 s |
| **D121** | torsi puncak `joint_2` terukur **< 12 N·m** pada setiap tugas | G24b maks 9.62; saringan RNEA + 7.7 ≤ 12 konservatif ≥ 3.3 (g24 E5) |
| **D122** | `t_traverse / T_cmd` ∈ **[0.95, 0.98]** pada setiap traverse Δ ≥ 0.10 m | debounce g19; G24b 0.966 / 0.972 |

🔒 **DILARANG** mengubah konstanta P1′, aturan instance, K, atau dugaan sesudah hasil apa pun
terlihat. Bila model meleset, itu hasil, ditulis di §B.

---

## B. Hasil terukur

> §A dikunci 2026-09-23 14:06, sha256 `87de8bf2c01a…` ([sectionA_locked.md](results/p1_g26/sectionA_locked.md)).

### B0. SEBELUM gerak apa pun

**Offline (sesudah kunci):** [g26_make.py](results/p1_g26/g26_make.py) → [g26_candidates.json](results/p1_g26/g26_candidates.json), [log](results/p1_g26/g26_make.log).
KG1 **45/45 bit-identik** (FISIK makespan + jadwal g24, P1′ opt g25); assert `p0` tersuntik
ditambah sesudah run KG1 pertama (KG1 sendiri tidak dapat menangkap injeksi gagal — default juga
0.55). KG2 `eval_schedule == finish` 50/50. **(i) 50/50, (i)–(iii) 46/50** (24, 28, 37, 46: satu
gantry tanpa tugas); (ii) P1′ = (ii) FISIK-sejati pada 46/46.

**Tahap 0:**

| Gerbang | Hasil |
|---|---|
| `remount_check.py` | **GERBANG LULUS**; ICMP .10–.13 ✅ |
| bring-up (`/tmp/g26_t1.log`, launch PID 1131987, SigIgn `0x1001001` = SIGINT tidak diabaikan) | **4×** "Actuator count … '6'"; **7/7** controller active; **0** baris pola fault runner (termasuk `table1 NOT armed` — G24b punya 1); 7 spawner "process has died" (= G23/G24b) ✅ |
| `/joint_states` (40 pesan) | rel **0.000503 / 0.000534** → R1 **0.00 / 0.00** (= g24 E8, kandidat tidak dibuat ulang); rotasi 0 / 0.02° |
| lengan | maks \|q − REST\| 0.076 / 0.061 / 0.075 / 0.081° — keempatnya < 0.5°: `return_rest --move` A4-1 akan self-skip |
| validator penilai | **7/7 PASS**, penilai 0 sebelum / 0 sesudah ([log](results/p1_g26/g26_validator.log)) ✅ |

**Tahap 1 — (iv)** ([g26_screen.sh](results/p1_g26/g26_screen.sh), [log](results/p1_g26/g26_screen.log), [json](results/p1_g26/g26_screen.json), sha `76763b2a7d53…`):

| seed | hasil |
|---|---|
| 0 | PLANNED / PLANNED / **t5 arm_1 TORQUE-UNSAFE** (sampel 3; tugas yang sama ditolak G24b) |
| **1** | **LOLOS 3/3** |
| **2** | **LOLOS 3/3** |
| 3–12 | ditolak, **semuanya TORQUE-UNSAFE** di sampel 1 (6, 7: sampel 2) |
| **13** | **LOLOS 3/3** |

- **Seed jalan = 1, 2, 13** (lolos ke-1/2/3 pada seed (i)–(iii) ke-2, ke-3, ke-14). Per rencana
  PLANNED **104 / 115** (90 %).
- 🔴 **Kelas penolakan tunggal: 11/11 TORQUE-UNSAFE, dan 11/11 target di z = 1.40** (lapis grid
  tertinggi, paling dekat gantry). Tidak ada NO-PLAN / S18 / S24. Terima-palsu oracle‴ yang B′2
  prediksi (miring tak konvergen) — kini terkonsentrasi di satu lapis z. Tidak dipilah.

**Prediksi A3 DIKUNCI 15:02** ([g26_predict_locked.json](results/p1_g26/g26_predict_locked.json),
sha `93c08449306f…`, [log](results/p1_g26/g26_predict.log)):

| seed | jadwal (rel, tugas) | retract dilewati | **P1′-serial** | P1′-par | P1 lama serial |
|---|---|---|---|---|---|
| 1 | g1 0→0.55 [t0,t1 arm_1] →0.70 [t5]; g2 0 [t2,t3 arm_3] →1.45 [t4] | g1 | **491.16** | 263.09 | 233.71 |
| 2 | g1 0→1.10 [t2] →1.50 [t4]; g2 0→0.70 [t5 arm_3, t1 arm_4] →1.10 [t3] →1.25 [t0] | g1, g2 | **583.19** | 356.53 | 354.99 |
| 13 | g1 0→0.75 [t3 arm_1, t5 arm_2] →1.25 [t0]; g2 0→0.50 [t1 arm_4] →1.35 [t2, t4 arm_3] | g1, g2 | **519.66** | 262.55 | 299.12 |

Seed 1 = tugas G24b, tetapi `p0` 0/0 → jadwal berbeda (g1 berangkat dari 0 tanpa tugas).
⚠ Seed 2 membawa **g1 ke 1.50 m** — rel g1 belum pernah > 0.70 di proyek ini sejak origin
dikonfirmasi (end stop ~1.656, guard 1.600).

**Operator (sesudah §A, sebelum gerak):** "saya izinkan gerak, semua aman" — izin gerak untuk
seluruh langkah A4 sesi ini.

### B1. Eksekusi — per seed (log `results/p1_g26/g26_s<S>_*`, dekomposisi [g26_decomp.py](results/p1_g26/g26_decomp.py) → `g26_s<S>_components.json`)

Kontrol decomp: fungsi berparameter G26 pada arsip G24b == `g24b_components.json` (IDENTIK, semua medan).
Pra-gerak: `return_rest` DRY + `--move` kedua gantry **self-skip** (0.08°) ([log](results/p1_g26/g26_rest_init.log)).

#### Seed 1 — **6/6 SUCCESS, makespan 557.69 s** vs P1′-serial 491.16 → **−66.53 s (−11.93 %)**

| ev | jenis | prediksi | terukur | galat | rincian |
|---|---|---|---|---|---|
| 0 | retract g1 | 0.00 | 2.49 | −2.49 | **dilewati** ("sudah di rest"), rc 0 |
| 1 | traverse g1 0→0.55 | 33.77 | 36.70 | −2.92 | t_trav 28.91 / T_cmd 30.53; overhead 7.78 |
| 2 | t0 arm_1 | 43.65 | 56.98 | −13.34 | saringan **41.88**, exec 12.43; τ 5.98 |
| 3 | t1 arm_1 | 43.65 | 49.97 | −6.33 | saringan 34.09, exec 13.18; τ 4.63 |
| 4 | retract g1 | 50.87 | 51.91 | −1.03 | saringan 18.30, gerak 31.06; τ 5.43 |
| 5 | traverse g1 0.55→0.70 | 12.66 | 14.78 | −2.12 | t_trav 7.99 / 8.36; overhead 6.79 |
| 6 | t5 arm_1 | 43.65 | 58.72 | −15.08 | start 2.65, saringan **41.20**; τ 9.23 |
| 7 | t2 arm_3 | 43.65 | 47.35 | −3.71 | saringan 31.29; τ 8.96 |
| 8 | t3 arm_3 | 43.65 | 49.21 | −5.57 | saringan 33.30; τ 5.80 |
| 9 | retract g2 | 50.87 | 53.33 | −2.46 | saringan 19.74, gerak 31.05; τ 6.32 |
| 10 | traverse g2 0→1.45 | 81.28 | 86.99 | −5.71 | t_trav 77.91 / 80.53; overhead 9.08 |
| 11 | t4 arm_3 | 43.48 | 48.97 | −5.49 | saringan 31.91; τ 8.66 |

- **12/12 event di bawah terukur.** Tugas yang sama dengan G24b kini **lebih lambat**: t2 arm_3
  47.35 vs 27.73 (saringan 31.3 vs 19.4), t0 56.98 vs 52.20 — adegan berbeda (g1 di 0.55/0.70
  saat blok g2, bukan 0.70; rel g1 berangkat dari 0). Tidak dipilah.
- Pemulihan ([log](results/p1_g26/g26_s1_recovery.log)): retract penutup 0.042° / 0.046° (τ 4.01 / 6.60);
  rel g1 0.699 → **0.000586**, g2 1.449 → **0.000356** (S24 CLEAR 572.2 / 539.7 mm), drift ≤ 0.154°.

#### Seed 2 — **6/6 SUCCESS, makespan 585.59 s** vs P1′-serial 583.19 → **−2.40 s (−0.41 %)**

| ev | jenis | prediksi | terukur | galat | rincian |
|---|---|---|---|---|---|
| 0 | retract g1 | 0.00 | 2.50 | −2.50 | **dilewati** |
| 1 | traverse g1 0→1.10 | 62.81 | 67.51 | −4.70 | t_trav 59.58 / 61.08; overhead 7.93 |
| 2 | t2 arm_1 | 43.65 | 40.85 | +2.79 | saringan 29.28, exec 8.88; τ **9.67** |
| 3 | retract g1 | 50.87 | 54.37 | −3.49 | saringan 20.73; τ 8.09 |
| 4 | traverse g1 1.10→**1.50** | 25.86 | 28.49 | −2.63 | t_trav 20.85 / 22.23; akhir 1.499147 (−0.85 mm) — **g1 terjauh di proyek** |
| 5 | t4 arm_1 | 43.65 | 28.17 | **+15.48** | saringan 19.83, exec **5.58**; τ 4.65 |
| 6 | retract g2 | 0.00 | 2.50 | −2.50 | **dilewati** |
| 7 | traverse g2 0→0.70 | 41.69 | 45.27 | −3.58 | t_trav 37.64 / 38.87 |
| 8 | t5 arm_3 | 43.65 | 46.74 | −3.09 | saringan 30.48; τ 6.57 |
| 9 | t1 arm_4 | 43.65 | 45.96 | −2.31 | saringan 32.77; τ 8.06 |
| 10 | retract g2 | 50.87 | 53.07 | −2.19 | saringan 19.46 |
| 11 | traverse g2 0.70→1.10 | 25.86 | 28.53 | −2.68 | t_trav 20.86 / 22.24 |
| 12 | t3 arm_3 | 43.65 | 46.78 | −3.14 | saringan 31.62; τ 8.68 |
| 13 | retract g2 | 50.87 | 52.04 | −1.16 | |
| 14 | traverse g2 1.10→1.25 | 12.66 | 15.04 | −2.39 | t_trav 8.02 / 8.39 |
| 15 | t0 arm_3 | 43.48 | 27.16 | **+16.32** | saringan 19.16, exec **5.48**; τ 4.92 |

- Galat total kecil = **pembatalan**: dua tugas lintasan pendek (exec ~5.5 s, +15.5 / +16.3) menutup
  14 event lain yang semuanya di bawah (Σ −34.8 s). Pola ev5 G24b (+15.9) terulang.
- Pemulihan ([log](results/p1_g26/g26_s2_recovery.log)): retract 0.041° / 0.045° (τ 5.09 / 6.05);
  rel g1 1.499 → **0.000408** (t_trav 81.16), g2 1.249 → **0.000691**; drift ≤ 0.149°.

#### Seed 13 — **6/6 SUCCESS, makespan 550.59 s** vs P1′-serial 519.66 → **−30.93 s (−5.62 %)**

| ev | jenis | prediksi | terukur | galat | rincian |
|---|---|---|---|---|---|
| 0 | retract g1 | 0.00 | 2.51 | −2.51 | **dilewati** |
| 1 | traverse g1 0→0.75 | 44.33 | 47.95 | −3.62 | t_trav 39.89 / 41.64; overhead 8.07 |
| 2 | t3 arm_1 | 43.65 | 40.43 | +3.22 | saringan 27.44, exec 9.33; τ 8.06 |
| 3 | t5 arm_2 | 43.65 | 48.25 | −4.61 | saringan 33.00; τ 6.94 |
| 4 | retract g1 | 50.87 | 53.51 | −2.63 | saringan 19.90 |
| 5 | traverse g1 0.75→1.25 | 31.13 | 34.13 | −3.00 | t_trav 26.88 / 27.83 |
| 6 | t0 arm_1 | 43.65 | 54.23 | −10.59 | saringan 37.91; τ 8.59 |
| 7 | retract g2 | 0.00 | 2.50 | −2.50 | **dilewati** |
| 8 | traverse g2 0→0.50 | 31.13 | 33.93 | −2.80 | t_trav 26.59 / 27.74 |
| 9 | t1 arm_4 | 43.65 | 49.34 | −5.70 | saringan 34.15; τ 5.98 |
| 10 | retract g2 | 50.87 | 52.81 | −1.94 | saringan 19.19 |
| 11 | traverse g2 0.50→1.35 | 49.61 | 53.56 | −3.95 | t_trav 45.49 / 47.24 |
| 12 | t2 arm_3 | 43.65 | 22.99 | **+20.66** | saringan 15.12, exec **3.27**; τ 7.90 |
| 13 | t4 arm_3 | 43.48 | 53.80 | −10.33 | saringan 36.22, exec 13.93; τ 7.58 |

- Pemulihan akhir ([log](results/p1_g26/g26_s13_recovery.log)): retract 0.041° / 0.044° (τ 5.22 / 5.47);
  rel g1 1.250 → **0.000712**, g2 1.349 → **0.000681**; drift ≤ 0.149°.
- Semua traverse jadwal: galat akhir rel ≤ 0.97 mm; drift lengan ≤ 0.159°; nol JTC error.

### B2. Gabungan — 3 seed, 18 tugas, 42 event ([g26_s{1,2,13}_components.json](results/p1_g26/))

| | seed 1 | seed 2 | seed 13 | Σ |
|---|---|---|---|---|
| terukur | 557.69 | 585.59 | 550.59 | 1693.87 |
| **P1′-serial (dikunci)** | 491.16 | 583.19 | 519.66 | 1594.01 |
| galat | **−11.93 %** | **−0.41 %** | **−5.62 %** | **−5.90 %** |
| P1′-par (model, gantry paralel) | 263.09 | 356.53 | 262.55 | — |
| P1 lama serial (FISIK, T_lin, 2 s/slot) | 233.71 (−58 %) | 354.99 (−39 %) | 299.12 (−46 %) | — |

| per jenis | n | prediksi | terukur | galat Σ | per event |
|---|---|---|---|---|---|
| tugas | 18 | 785.11 | 815.93 | −30.81 | **\|galat\| rerata 8.21 s**, bertanda −1.71, rentang −15.08…+20.66 |
| traverse | 12 | 452.78 | 492.87 | **−40.09** | overhead alat 6.79–9.08 s vs `c_rovh` 4.74; `t/T_cmd` 0.938–0.975 vs `r` 0.950 |
| retract (penuh) | 7 | 356.12 | 371.03 | −14.91 | −1.0…−3.5 |
| retract dilewati | 5 | 0.00 | 12.50 | −12.50 | **2.49–2.51 s** tiap kali |
| sela runner | — | 0 | 1.46 | −1.46 | |

- **Model P1′ lulus secara kasar dan bias ke bawah secara sistematis.** Makespan dalam −0.4…−11.9 %
  (rerata gabungan −5.9 %). P1′ jauh lebih baik dari P1 lama (−39…−58 %).
- **Bias terletak di suku NON-tugas** (Σ −69.0 s atas 3 seed = −23 s/seed): overhead `rail_to_g`
  (saringan S24 + tunggu diam) > `c_rovh` kalibrasi `rail_to` G19 — **mekanisme g25 A1 (e), sudah
  diketahui**; retract dilewati tetap 2.5 s (proses `return_rest` start + baca keadaan) sementara
  model memberi 0.
- **Suku tugas hampir tak bias dalam jumlah, tetapi berisik per tugas:** tugas lintasan pendek
  (exec 3.3–5.6 s) +15…+21 s; lintasan panjang (exec 12–14 s) −5…−15 s. Saringan antar-lengan =
  **65–73 %** tiap tugas (g25: 65–71 %). Satu konstanta `c_task` tidak dapat mengikuti panjang
  lintasan (g25 A1, D119).
- **Torsi:** puncak `joint_2` seluruh 18 tugas **≤ 9.66 N·m** (< 12). Retract puncak 8.09.
- **Rel:** 12 traverse jadwal + 6 pemulihan, galat ≤ 0.97 mm; `t/T_cmd` 0.938–0.975 — tiga di
  bawah 0.95 (Δ 0.40, 0.40, 0.55 m).

### B3. Saringan (iv) — temuan PASCA-DATA (tidak diduga, tidak dihitung di tally)

Pada 14 seed yang disaring: **lolos ⟺ jadwal tidak memuat tugas z = 1.40** (14/14 — 3 lolos
tanpa z = 1.40; 11 ditolak, masing-masing dengan 1–2 tugas z = 1.40, ditolak TORQUE-UNSAFE tepat
di tugas itu). Rencana tugas z = 1.40 di saringan: PLANNED 5, TORQUE-UNSAFE 11. Pada 46 seed
(i)–(iii), **29 memuat ≥ 1 tugas z = 1.40** → paling banyak 17/46 dapat lolos (iv) dengan pola ini.
Tafsiran (belum diuji): terima-palsu oracle‴ (Newton-miring tak konvergen, g24 B′2) terkonsentrasi
di lapis grid teratas, yang paling dekat bahu dan menuntut `joint_2` terbesar. Kalimat naskah
"jadwal optimum dapat dieksekusi" berlaku untuk seed yang **lolos (iv)**: 3/14 disaring.

### B4. Papan skor — D114–D122

| # | Dugaan | Terukur | |
|---|---|---|---|
| D114 | (i)–(iii) ≥ 45/50 | **46/50** | ✅ |
| D115 | 3 lolos (iv) dalam ≤ 8 seed (i)–(iii) | lolos ke-3 = seed ke-**14** | ❌ |
| D116 | tiap seed 6/6, nol HALTED, nol auto-stop | **18/18** SUCCESS, 0 HALTED, 0 AUTO-STOP | ✅ |
| D117 | galat makespan ±10 % tiap seed | −11.93 / −0.41 / −5.62 % | ❌ (seed 1) |
| D118 | terukur > prediksi ≥ 2/3 seed | **3/3** | ✅ |
| D119 | \|galat\| per tugas rerata ≥ 5 s | **8.21 s** | ✅ |
| D120 | retract dilewati: "sudah di rest", rc 0, ≤ 4 s | **5/5**, 2.49–2.51 s | ✅ |
| D121 | τ `joint_2` < 12 tiap tugas | maks **9.66** | ✅ |
| D122 | `t/T_cmd` ∈ [0.95, 0.98] tiap traverse Δ ≥ 0.10 | 9/12; 0.938 / 0.938 / 0.947 | ❌ |

**G26: 3 meleset / 6 tepat → papan skor 59 meleset / 61 tepat.** Keenam yang tepat ditarik dari
mekanisme terukur (jalur kode `return_rest`, bias alat G22+, saringan ∝ lintasan, sebaran torsi
G24b). Yang meleset: D115 menduga saringan **lebih meloloskan** (kendala lebih longgar) — satu lapis
z menolak hampir semua; D117 besaran; D122 rentang dari n = 2 (G24b) terlalu sempit.

### B5. Pertentangan §B lawan §A / prompt — G26

| # | Pertentangan |
|---|---|
| (1) | Kunci json `schedule_FISIK` / `decomp.FISIK` di `g26_candidates.json` berisi jadwal **P1′** (A1, disengaja — alat tidak diubah); pembaca arsip harus membaca `schedule_key_note` |
| (2) | KG1 tidak dapat menangkap injeksi `p0` yang gagal diam-diam (default juga 0.55); assert `p0` ditambah **sesudah** run KG1 pertama, **sebelum** run `p0` 0/0 |
| (3) | Prompt: "jalankan seed lolos PERTAMA, lalu berikutnya"; A2 (dikunci) menyaring ketiganya **sebelum** gerak pertama. Keadaan awal sama untuk tiap seed (rel 0/0, REST) — urutan tidak memengaruhi saringan |
| (4) | A4 "izin operator per gerak"; operator memberi izin menyeluruh sekali ("saya izinkan gerak, semua aman") sesudah §A. Tiap gerak tetap diumumkan sebelum dikirim |

**G26: EMPAT.**

## C. Keadaan akhir (Rule 12)

- **Rel 0.000712 / 0.000681** (0/0), rotasi 0; keempat lengan REST (≤ 0.05° sesudah retract
  penutup, drift traverse ≤ 0.15°).
- **Stack dimatikan:** SIGINT penilai (1143137) + perekam (1143120), lalu launch 1131987 → keluar
  12 s (launch meng-eskalasi SIGTERM ke `ros2_control_node` / `move_group` / `rviz2`, = G24b);
  nol FAULT / Kortex exception / `INVALID_USER_SESSION`; `ros2 node list --no-daemon` kosong; nol
  proses sisa. Crash dump `move_group` sesi ini (205 MB, 15:50) **dihapus**; dump python 12:28
  (bukan milik sesi) dibiarkan. Disk **3.4 GB**.
- Arsip: log launch [g26_launch.log.gz](results/p1_g26/g26_launch.log.gz), ringkasan penilai
  `g26_monitor_summary.csv`, konsol per seed. `/tmp/g26_js.csv` (387 MB, perekam) dan
  `/tmp/g26_step_samples.csv` **tidak** disalin (ukuran), masih di `/tmp`.
- **Tidak diubah:** `sched.py`, peta, oracle, `make_instance*.py`, alat G22, probe, konstanta P1′.
  **Baru:** `docs/results/p1_g26/` — `g26_make.py`, `g26_predict.py`, `g26_decomp.py`,
  `g26_{screen,run,rail}.sh`, json/log.
- **Belum dikerjakan / tidak diukur:** model tugas bergantung lintasan; kalibrasi ulang `c_rovh`
  dari `rail_to_g` dan retract-dilewati 2.5 s (dilarang di sesi ini — konstanta terkunci);
  diagnosis oracle‴ di z = 1.40 (B3); rotasi gantry (R0: seluruh G22–G26 adalah masalah 1-D rel,
  S23 — penyaring S18/S24 tidak memuat sendi rotasi).

## D. Prompt G27 (salin ke chat BARU) — OFFLINE

**Kenapa ini berikutnya.** G26 menjawab replikasi: **4 seed, 24/24 tugas** (G24b + G26), makespan
P1′ dalam −0.4…−11.9 %. Yang kini membatasi klaim naskah bukan model biaya, tetapi **laju lolos
(iv)**: 3/14 seed, dan pola "lolos ⟺ tanpa tugas z = 1.40" (B3) berarti oracle‴ meloloskan satu
lapis grid yang sel tolak. Selama itu, "jadwal optimum dapat dieksekusi" hanya sah untuk subset
terseleksi. Bias model (non-tugas, −23 s/seed) mekanismenya sudah diketahui — sekunder.

**Rekomendasi: Opus, effort TINGGI** — kesalahan paling mungkin DIAM: menjelaskan z = 1.40 dari 16
rencana tanpa kontrol (post-hoc), dan "memperbaiki" oracle sampai cocok dengan data yang sama.

```
Sesi G27 -- diagnosis terima-palsu oracle''' di z = 1.40, lalu (sekunder) P1'' non-tugas. OFFLINE, nol perangkat keras.
Repo ceiling_arm, branch feat/rgbd-topo-deploy.

BACA PENUH: CLAUDE.md; docs/p1_state.md 6 + 7 + tally; docs/p1_g26_hw.md (B0, B2, B3, B4, C);
docs/p1_g24_roll_oracle.md A', B'1, B'2 (miring tak konvergen 19 %), E1; docs/p1_g23_oracle_hw.md B0 (margin).

LATAR: G26 (iv) 3/14 lolos; 11/11 penolakan TORQUE-UNSAFE, semuanya tugas z = 1.40; rencana z = 1.40
PLANNED 5 / ditolak 11; 29/46 seed memuat z = 1.40. Sumber: docs/results/p1_g26/g26_screen.json
(RNEA j2 per rencana di log), g24_oracle3_cache.jsonl (tuple oracle''' per node/rel/slot).

1. Instrumen dulu: untuk tiap rencana tugas di g22/g23/g24/g26 screen + eksekusi (RNEA j2 hidup,
   torsi terukur) tabelkan z, rel, lengan, verdict, RNEA j2 vs batas oracle''' (14 - 7.7 - m2), status
   oracle''' tuple (n_sol, saturated, tilt_fail). Tulis apa yang TIDAK dapat dipilah.
2. KUNCI sebelum diagnosis: hipotesis bersaing (H1 Newton-miring tak konvergen; H2 cabang IK terlewat;
   H3 margin m2 dikalibrasi di z < 1.40; H4 lain), uji pembeda per hipotesis, dan set VALIDASI yang
   tidak dipakai untuk memilih (mis. tuple z = 1.32 / 1.40 yang belum pernah direncanakan, dievaluasi
   dengan perencana plan-only BUKAN di sesi ini -> hanya prediksi terkunci untuk G28).
   Dugaan D123+ dikunci sebelum data langkah 3.
3. Diagnosis offline (IK/RNEA saja, tanpa ROS). Bila oracle diubah: default lama harus mereproduksi
   g24_oracle3_cache bit-identik (kontrol); laju lolos (iv) yang diprediksi untuk 46 seed ditulis.
4. Sekunder, bila waktu: P1'' = P1' + c_rovh dari rail_to_g (G24b+G26, n = 14) + retract-dilewati 2.5 s;
   dikunci sebelum fit, dievaluasi leave-one-seed-out atas 4 seed (G24b, G26 x3). c_task TIDAK diubah.
5. p1_state 6 + tally; prompt G28 (HW: saringan (iv) pada seed baru dengan oracle terkoreksi).

ATURAN: 7.1 ground truth dulu; 7.2 dugaan dikunci sebelum data; B menang atas A dan DITULIS.
DILARANG memilih aturan oracle sesudah melihat hasilnya pada 16 rencana z = 1.40 yang sama.
Nol perangkat keras; tidak ada ros2 launch. Disk ~3.4 GB.
```
