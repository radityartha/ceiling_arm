# P1 / G25 — model biaya tugas TERUKUR, lalu scheduler ulang (OFFLINE, nol perangkat keras)

> Sesi 2026-09-23. Nol gerak, tidak ada `ros2 launch`. Sumber: log arsip
> `docs/results/p1_g19`, `p1_g20`, `p1_g24` (G24b). Kode/hasil: `docs/results/p1_g25/`.
>
> Pemicu (g24 §E5/§F): G24b seed 1 = 6/6, makespan terukur **480.44 s** vs
> P2-serial 202.49 (×2.37); suku yang model beri 2 s/tugas terukur 28–52 s.

---

## A. Protokol — ditulis 2026-09-23 SEBELUM prediksi G24b, SEBELUM solve apa pun

### A0. Urutan kerja, dan satu penyimpangan dari prompt

Prompt: (1) dekomposisi **termasuk G24b** → (2) kunci → (3) kalibrasi. Dijalankan:
(1) dekomposisi **set kalibrasi saja** (G19, G20) → (2) kunci (bagian ini) → (1b)
dekomposisi G24b → (3). Alasannya: mengekstrak komponen G24b sebelum kunci akan
menambah kontaminasi di atas yang sudah ada. Log G24b hanya dibaca **strukturnya,
dengan semua angka dihapus** (`sed 's/[0-9.]+/N/g'`), untuk tahu apa yang dapat dipilah.

⚠️ **Kontaminasi yang sudah terjadi (seperti PC2′, g24 B′):** prompt mewajibkan membaca
g24 §E, jadi **total 480.44 s dan durasi per event** (tabel E5: tugas 52.20 / 52.44 /
49.59 / 27.73 / 41.49 / 49.79, retract 52.71 / 52.76, traverse 14.75 / 86.98) **sudah
terlihat** sebelum kunci ini. Yang **belum** terlihat: pemilahan tiap event G24b ke
komponen. Median kalibrasi (A2) juga sudah terlihat — itu memang tujuan langkah 1.
Konsekuensi: dugaan tentang **galat makespan G24b** tidak buta dan tidak dihitung di
papan skor (A6).

### A1. Temuan langkah 1 (set kalibrasi) — dasar bentuk model

[decomp_calib.py](results/p1_g25/decomp_calib.py) → [g25_calib_components.json](results/p1_g25/g25_calib_components.json).
Tiap angka = selisih dua cap waktu pada jam dinding yang sama.

| sumber cap waktu | dipakai untuk |
|---|---|
| `gXX_windows.txt` (bash `date +%s.%N` di runner, sekitar tiap panggilan alat) | awal/akhir proses probe; panggilan `return_rest` / `rail_to` (G19) |
| log launch, `move_group` | `P` Planning request · `M` Motion plan computed · `S` Starting execution · `C` Completed |
| log probe (cap ROS) | `R` baris `torsi RNEA+offset` · `X` baris `jarak antar-lengan minimum` |
| `gXX_monitor.log` | `>>> arm_k: SUCCESS` (penilai independen) |
| `g19_components.json` (perekam) | retract-ke-REST, traverse, sela (g19 B1.2, apa adanya) |

| komponen (per lengan dieksekusi) | G19 pertama-di-proses (n 11) | G19 berikutnya (11) | G20 pertama (10) | G20 berikutnya (30) |
|---|---|---|---|---|
| `t_plan` P→M (perencana MoveIt) | **0.05** | 0.05 | **0.04** | 0.04 |
| `t_rnea` M→R | 0.07 | 0.01 | 0.07 | 0.00 |
| `t_screen` R→X (saringan antar-lengan) | **27.55** (17.5–34.5) | 23.14 | **29.02** (17.4–37.7) | 21.29 |
| `t_go` X→S | 0.01 | 0.01 | 0.01 | 0.01 |
| `t_exec` S→C | 12.53 | 9.28 | 10.11 | 8.55 |
| `t_succ` C→SUCCESS | 1.82 | 1.81 | 1.85 | 1.87 |
| percobaan rencana | 1.09 | 1.00 | 1.00 | 1.03 |

Per proses probe: `t_start` (awal proses → P pertama) median 0.67 (G19) / 1.54 (G20);
`t_end` (event penilai terakhir → proses keluar) **0.17–0.19 s bila berakhir sukses**
(pada TORQUE-ABORT 24–53 s — probe menunggu sisa jendela; tidak dipakai).

**Jawaban atas "rencana 34 s belum dipilah perencana vs saringan" (prompt F):**
**perencana ≈ 0.04 s; > 99 % "rencana" adalah saringan antar-lengan `screen_interarm`.**
Saringan berkorelasi dengan panjang lintasan: `t_screen ≈ 1.9 + 2.35·t_exec`
(r = 0.91, lengan berikutnya) dan `7.7 + 1.88·t_exec` (r = 0.88, pertama; selisih
~6 s = bangun geometri pinocchio sekali per proses). Ini **mekanisme**, bukan dipakai
sebagai model (A2: `t_exec` tidak diketahui sebelum rencana ada).

Per pindah G19 (n 10, Δ 400 mm): panggilan `return_rest --move` **50.87** (48.8–53.4);
panggilan `rail_to --move` 26.00 (dua pencilan 70.8 = tunggu 45 s, g19 B3 (3)); traverse
perekam 21.11; `T_cmd(0.400)` = 22.22.

**Yang TIDAK dapat dipilah di set kalibrasi:** (a) tunggu persepsi vs init rclpy di
dalam `t_start`; (b) bangun geometri vs hitung saringan di `t_screen` lengan pertama —
hanya lewat selisih pertama − berikutnya (~6 s); (c) retract G20 — log `g20_rest*`
tanpa cap waktu, tidak dipakai; (d) retract **gantry 2** — tidak pernah diukur di set
kalibrasi (G19 hanya gantry 1); (e) alat `rail_to_g` (G22+, saringan sapuan S24) —
**tidak pernah jalan di set kalibrasi**; satu-satunya kalibrasi rel = `rail_to` G19;
(f) runner `run_g22` — tidak pernah jalan di set kalibrasi, sela antar-event tidak
terkalibrasi; (g) eksekusi dari target yang DITAHAN (tugas ke-2 lengan yang sama di
satu perhentian) — set kalibrasi selalu dari REST. G22/G23 (plan-only, in-process,
geometri dibangun sekali per proses) **tidak** dipakai untuk konstanta waktu — jalur
kodenya lain; G23 calib (81 rencana) = torsi saja.

### A2. 🔒 Model biaya — bentuk dan set kalibrasi DIKUNCI

**Set kalibrasi = G19 + G20 SAJA** (G22/G23 tidak punya eksekusi; alasan A1). **Set uji =
G24b** (10 event + makespan). Semua konstanta = **median** (konvensi g21 A1; tidak
diganti sesudah melihat galat).

```
c_task  = median_{21 lengan PERTAMA-di-proses G19+G20} [ t_start + t_armspan + t_exec + t_succ ]
          + median_{probe yang berakhir sukses} t_end
          (t_armspan = P pertama lengan itu -> S; memuat semua percobaan rencana + saringan)
          Alasan "pertama-di-proses": tiap tugas G24b = proses probe BARU, satu lengan,
          disaring vs 3 lengan lain -- jalur kode = lengan pertama G20 (dua bangun geometri).
c_ret   = median G19 panggilan return_rest --move (n 10)        -- 0 bila dilewati (kedua lengan sudah REST)
r       = median G19 traverse / T_cmd(0.400)                      -- rel = debounce bridge (g19)
c_rovh  = median G19 (panggilan rail_to − traverse)               -- overhead alat rel
T_cmd(Δ)= max(3, π·|Δ| / (2·0.9·v_lin))                          -- g22_plan.t_cmd, apa adanya
sela runner = 0                                                    -- TIDAK terkalibrasi (A1 f)
```

**Prediksi event G24b** (jadwal seed 1 terkunci g24 E1, urutan A3 g22):
tugas → `c_task` (tugas **terakhir** → `c_task − t_end`, makespan berakhir di `success`);
retract → `c_ret`; traverse → `c_rovh + r·T_cmd(Δ)`. **P1′-serial** = Σ event (eksekusi
A3 serial). Dibandingkan juga P1–P4 lama (g24 E2).

**Model scheduler P1′** (sama persis, dalam bentuk `sched.py`):
```
durasi perhentian U = |U| · c_task            -- lengan SERIAL: setiap alat terukur (probe dual/quad,
                                                 runner A3) menggerakkan satu lengan pada satu waktu
biaya pindah         = c_ret + c_rovh + r·T_cmd(Δ)
pindah PERTAMA dari p0 tanpa tugas di p0 = c_rovh + r·T_cmd(Δ)    -- retract dilewati (lengan masih REST, A3 g22)
makespan P1′         = max_g selesai_g         -- gantry paralel, seperti model sejak G7
```
Lengan/tugas MR: instance G24 `mr = 0`; `MR` = satu slot (dicatat, tidak diuji). Rotasi:
instance G24 `rot = 0`, `T_rot` tidak dipakai.

**Varian sekunder P1′-par (TIDAK dikutip):** perhentian = `slots · c_task` (bentuk tertutup
lama, lengan paralel dalam perhentian) — konkurensi lengan **tidak pernah diukur**; hanya
untuk melihat apakah kesimpulan bergantung pada asumsi serial.

### A3. 🔒 Perubahan `sched.py` — default lama bit-identik (KONTROL)

Tiga medan `Instance` baru, default = perilaku lama:
`serial_arms=False` (durasi = `(m + |SR|)·dwell`), `t_fold_first=None` (biaya pindah
pertama dari `p0`), `rail_cmd=0.0` (> 0 → sumbu linier = `rail_cmd · T_cmd(Δ)`).
`dwell` (= `c_task`) dan `t_fold` (= `c_ret + c_rovh`) sudah parameter.

**Kontrol C0′ (harus lulus sebelum hasil apa pun dibaca):**
1. `test/verify_sched_exact.py` lulus seperti sebelumnya (V0–V4 × 3 `t_fold`).
2. G21 E1 exact, 140 instance × `t_fold` {0, 50.8, 126.8}: `exact` dan dekomposisi per
   gantry (`moves`, `trav`, `dwell`, `finish`) **==** arsip `g21_e1_tf*.json` (float ==).
3. G24 45 instance (i)–(iii) × FISIK/DINDING: makespan dan jadwal `(pose, tasks)` **==**
   `g24_candidates.json`.

**Gerbang G0′ (jalur baru):** pemeriksa brute-force **independen** (tidak memanggil
`sched.py` kecuali untuk membangun instance) atas instance acak kecil: makespan
`solve_exact` == brute force untuk kombinasi {serial, par} × {t_fold_first ada/tidak} ×
{rail_cmd 0/>0}; plus evaluator jadwal independen == `finish` solver pada 45 instance.
Gagal → tidak ada hasil §B3 yang dibaca.

### A4. 🔒 Besaran yang DILAPORKAN — terlepas dari hasilnya

**G24b (uji):** per event: terukur, prediksi, galat (s, %); makespan P1′-serial vs 480.44 dan
P1–P4. Per tugas: `t_start, t_plan, t_rnea, t_screen, t_go, t_exec, t_succ, t_end`,
jumlah percobaan, torsi puncak (`probe.json tau_peak`) + RNEA j2. Per retract:
saringan / gerak / overhead. Per traverse: overhead `rail_to_g` vs `t_traverse`. Sela runner.

**45 instance G24 (P1′, dan P1′-par sekunder):**
1. jumlah instance di mana jadwal FISIK lama **tidak lagi optimum** di bawah P1′
   (biaya P1′ jadwal lama > optimum P1′ + 1e-6 — tahan terhadap seri);
2. **regret** jadwal lama di bawah P1′: mean / maks (%, s);
3. jumlah pindah total dan per gantry: lama vs P1′; pembagian tugas `n_1/n_2`;
4. **"optimum = minimum pindah?"**: `m_min` = minimum jumlah pindah total atas semua jadwal
   layak (DP `dwell = 0`, `t_fold = 1e4`, alokasi min-Σ); berapa optimum P1′ (dan FISIK)
   yang jumlah pindahnya = `m_min`;
5. pangsa makespan P1′: tugas / pindah (retract + overhead) / rel;
6. P1 (model lama) vs P1′ untuk jadwal seed 1 dan rerata 45.

### A5. Yang TIDAK dilakukan sesi ini
Tidak ada perubahan pada peta, oracle, `make_instance_g24.py`, alat G22 (`rail_to_g`,
`run_g22`, `sched_screen`, `return_rest`), `sched_heur.py` (heuristik tidak membaca medan
baru — dijaga assert, A3). Tidak ada instance baru (G26).

### A6. 🔒 Dugaan D107–D113 — DITULIS SEBELUM prediksi G24b dan sebelum solve

| # | Dugaan | status |
|---|---|---|
| D107 | P1′-serial G24b dalam **±10 %** dari 480.44, dan **di bawah** terukur | ⚠️ **terkontaminasi** (A0) — dinilai, **tidak dihitung** di tally |
| D108 | di G24b, pada **6/6** tugas: perencana P→M < 1 s **dan** saringan R→X ≥ 50 % wall event | buta (pemilahan G24b belum dilihat) |
| D109 | jadwal FISIK lama **tidak lagi optimum** di P1′ pada **≥ 23/45** instance | buta |
| D110 | rerata pindah total optimum P1′ **≤** rerata FISIK (3.96) | buta |
| D111 | regret jadwal lama di P1′: mean **≤ 5 %** dan maks **≤ 20 %** | buta |
| D112 | G0′ menemukan **≥ 1** ketidakcocokan di kode baru sebelum lulus (prior "kode sendiri: tebak lebih salah") | buta |
| D113 | pembagian tugas antar gantry lebih seimbang di P1′: mean `|n_1 − n_2|` **turun** | buta |

🔒 **DILARANG** mengubah bentuk model, set kalibrasi, atau statistik (median) sesudah
galat G24b terlihat. Bila model meleset, itu hasil, dan ditulis di §B.

---

## B. Hasil terukur

> §A dikunci 2026-09-23 11:40, sha256 `c080a44e6e17…` ([sectionA_locked.md](results/p1_g25/sectionA_locked.md)).
> Urutan sesudah kunci: `sched.py` diubah → C0′ (1)(3) + G0′ → dekomposisi G24b →
> kalibrasi + prediksi → C0′ (2) → 45 instance.

### B0. Kontrol dan gerbang — `sched.py` sebelum hasil dibaca

| gerbang | hasil |
|---|---|
| C0′ (1) `verify_sched_exact.py` (V1 + V0/V2/V3/V4 × 3 `t_fold`) | **PASS**, "EXACTNESS ESTABLISHED" |
| C0′ (2) G21 E1 140 × 3 `t_fold`, float == ([c0_g21.log](results/p1_g25/c0_g21.log)) | **420/420 bit-identik** (exact + `moves/trav/dwell/finish/n_stops`) |
| C0′ (3) G24 50 instance × FISIK/DINDING, makespan + jadwal json ([c0_g24.log](results/p1_g25/c0_g24.log)) | **100/100 bit-identik** |
| G0′ brute force independen, 8 kombinasi biaya × 12 instance × 2 ([g0_gate.log](results/p1_g25/g0_gate.log)) | **192/192** (solve == brute force, replay == `finish`) — percobaan pertama |
| kontrol daya G0′: 3 mutan ([g0_mutants.log](results/p1_g25/g0_mutants.log)) | abaikan `T0` 175/192 · abaikan `rail_cmd` 166/192 · abaikan serial 113/192 → **ketiganya TERTANGKAP** |

Perubahan `sched.py` (diff ~45 baris): `traverse_time/traverse_matrix(…, rail_cmd)`; `Instance`
+ `serial_arms`, `t_fold_first`, `rail_cmd`, `default_costs`; `_dur_table`/`stop_duration`
cabang serial; `GantryDP.T0` + `cost`; `_reconstruct` langkah pertama memakai `T0`;
`without_mutex` via `dataclasses.replace` (sebelumnya konstruktor posisional — akan
membuang medan baru diam-diam). `sched_heur.GantryView` dan `sched_coupled` **assert
`default_costs`** (fail loud: keduanya tidak membaca medan baru).

### B1. Dekomposisi G24b — [decomp_g24b.py](results/p1_g25/decomp_g24b.py) → [g24b_components.json](results/p1_g25/g24b_components.json)

Makespan dari cap waktu = **480.44** (= runner). Sela runner antar-event 0.017–0.020 s
(Σ 0.18 s).

**Tugas** (s; `P→M` perencana, `R→X` saringan antar-lengan; `geom1` = bangun geometri
pertama; bangun kedua + hitung saringan **tidak dapat dipilah**):

| ev | tugas | dari | start | P→M | M→R | geom1 | **R→X** | exec | →succ | end | wall | saringan/wall | τ puncak | RNEA j2 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | t0 arm_1 | REST | 2.49 | 0.069 | 0.094 | 3.17 | **35.03** | 12.53 | 1.80 | 0.18 | 52.20 | 67 % | 6.02 | 2.04 |
| 1 | t1 arm_1 | t0 ditahan | 0.59 | 0.043 | 0.073 | 3.15 | **36.29** | 13.47 | 1.79 | 0.18 | 52.44 | 69 % | **9.62** | 5.05 |
| 4 | t5 arm_1 | REST | 0.71 | 0.045 | 0.070 | 3.15 | **34.28** | 12.68 | 1.62 | 0.18 | 49.59 | 69 % | 9.25 | 3.69 |
| 5 | t2 arm_3 | REST | 0.57 | 0.026 | 0.071 | 3.16 | **19.44** | 5.68 | 1.77 | 0.18 | 27.73 | 70 % | 9.46 | 4.20 |
| 6 | t3 arm_3 | t2 ditahan | 0.58 | 0.044 | 0.072 | 3.19 | **29.37** | 9.38 | 1.85 | 0.19 | 41.49 | 71 % | 9.39 | 4.27 |
| 9 | t4 arm_3 | REST | 2.50 | 0.056 | 0.074 | 3.15 | **32.40** | 12.68 | 1.90 | — | 49.62 ᵃ | 65 % | 9.00 | 4.86 |

ᵃ sampai `success` (akhir makespan). Semua 1 percobaan rencana, 2 bangun geometri, SUCCESS.

- **Perencana MoveIt 0.03–0.07 s; saringan antar-lengan 19–36 s = 65–71 % tiap tugas.**
  "Rencana 12–30 s" di g24 E5 adalah **saringan**, bukan perencanaan.
- Saringan ∝ lintasan: R→X / exec = 2.6–3.4; di atas relasi kalibrasi pertama
  (`7.7 + 1.88·exec`) sebesar **+0.8…+4.0 s** (mean +2.6) — tidak dipilah (adegan G24b
  punya kedua gantry di pose berbeda; bangun kedua dan hitung tidak terpisah).
- Tugas dari target **ditahan** (ev1, ev6) **tidak** lebih pendek: exec 13.47 / 9.38 vs
  REST 12.53–12.68 / 5.68. Asumsi A1 (g) tidak terbantah di n = 2.
- `start` 2.5 s pada ev0 dan ev9 (tugas pertama run; tugas pertama sesudah traverse g2),
  0.57–0.71 sisanya. Tidak dipilah (tunggu persepsi vs init).
- **Torsi (tugas E8):** puncak `joint_2` per tugas 6.02–**9.62** N·m, semua < 12. Offset
  terukur − RNEA rencana hidup: +3.98 / +4.57 / +5.56 / **+5.26** / +5.12 / +4.14 — di dalam
  sebaran g21 B8 (median 4.49, maks 7.15). RNEA **rencana hidup** t5 = 3.69, bukan 5.21 dari
  rencana (iv) plan-only (g24 E5 membandingkan puncak dengan rencana (iv), yang tidak
  dieksekusi — rencana di-sampel ulang).

**Retract** (s): start / saringan / gerak / keluar — g1 **2.37 / 19.11 / 31.06 / 0.17** =
52.71; g2 **2.68 / 18.84 / 31.06 / 0.17** = 52.76. Gerak = 30 s perintah + 1.06.
Saringan se-gantry **dan** antar-gantry (2 bangun geometri).

**Traverse** (s): start / S24 / kirim / `t_traverse` / settle / keluar — g1 Δ 0.149 m:
1.37 / 3.47 / 0.06 / **8.005** / 1.01 / 0.16 = 14.75 (overhead 6.74; `T_cmd` 8.28);
g2 Δ 1.449 m: 1.38 / 3.74 / 0.06 / **78.25** / 1.77 / 0.18 = 86.98 (overhead 8.73;
`T_cmd` 80.53). `t_traverse/T_cmd` = 0.966 / 0.972.

### B2. Kalibrasi (A2 persis) → prediksi G24b — [calibrate.py](results/p1_g25/calibrate.py), [g25_predict_g24b.json](results/p1_g25/g25_predict_g24b.json)

| konstanta | nilai | n |
|---|---|---|
| `c_task` = 43.475 (awal → success) + `t_end` 0.171 | **43.65 s** | 21 / 11 |
| `c_ret` | **50.87 s** | 10 |
| `r` = traverse / `T_cmd` | **0.9502** | 10 |
| `c_rovh` | **4.74 s** | 10 |
| → sched: `dwell = 43.646`, `t_fold = 55.61`, `t_fold_first = 4.74`, `rail_cmd = 0.9502`, serial | | |

| ev | jenis | prediksi | terukur | galat |
|---|---|---|---|---|
| 0 | tugas g1 | 43.65 | 52.20 | −8.55 (−16.4 %) |
| 1 | tugas g1 | 43.65 | 52.44 | −8.79 (−16.8 %) |
| 2 | retract g1 | 50.87 | 52.71 | −1.84 (−3.5 %) |
| 3 | traverse g1 | 12.66 | 14.75 | −2.09 (−14.2 %) |
| 4 | tugas g1 | 43.65 | 49.59 | −5.94 (−12.0 %) |
| 5 | tugas g2 | 43.65 | 27.73 | **+15.91 (+57.4 %)** |
| 6 | tugas g2 | 43.65 | 41.49 | +2.16 (+5.2 %) |
| 7 | retract g2 | 50.87 | 52.76 | −1.88 (−3.6 %) |
| 8 | traverse g2 | 81.28 | 86.98 | −5.70 (−6.6 %) |
| 9 | tugas g2 | 43.48 | 49.62 | −6.14 (−12.4 %) |
| | **makespan P1′-serial** | **457.39** | **480.44** | **−23.05 (−4.80 %)** |

| per jenis | prediksi | terukur | galat |
|---|---|---|---|
| tugas (6) | 261.71 | 273.06 | −11.36 |
| retract (2) | 101.75 | 105.47 | −3.72 |
| traverse (2) | 93.94 | 101.73 | −7.79 (overhead `rail_to_g` 6.7/8.7 vs `c_rovh` 4.74; `r` 0.97 vs 0.95) |
| sela runner | 0 | 0.18 | −0.18 |

Lawan prediksi lama (serial, g24 E2): P1 165.11 (−65.6 %), P2 202.49 (−57.9 %), P3 317.11
(−34.0 %), P4 354.49 (−26.2 %). **P1′ memotong galat makespan dari −57.9 % (P2) ke −4.8 %.**

⚠️ **Galat per tugas besar walau total kecil:** |galat| rerata 7.9 s (18 %), rentang
−8.8…+15.9 — satu konstanta tidak dapat mengikuti panjang lintasan (saringan ∝ exec,
A1). Empat dari enam tugas diprediksi **terlalu rendah** (−5.9…−8.8 s: G24b lebih
lambat dari kalibrasi, sebagian oleh saringan +2.6 s dan `start` 2.5 s); ev5 (lintasan
pendek, +15.9) dan ev6 (+2.2) menutup sebagian. Total yang dekat adalah sebagian **kebetulan pembatalan**, n = 6.
Bias arah semua komponen non-tugas: **di bawah** (alat G22+ lebih lambat dari alat
G19 yang dipakai kalibrasi, A1 (d)(e)).

### B3. 45 instance G24 di bawah P1′ — [run_sched45.py](results/p1_g25/run_sched45.py) → [g25_sched45.json](results/p1_g25/g25_sched45.json), [log](results/p1_g25/g25_sched45.log)

Dibaca sesudah C0′ (1)(2)(3) dan G0′ lulus. Tiap optimum P1′ diputar ulang oleh evaluator
independen `g0_gate.eval_schedule` (== `finish` solver, 90/90). Konsistensi: jadwal seed 1
di P1′, Σ_g = **457.56** = prediksi event B2 457.39 + `t_end` 0.171 ✓.

| A4 | P1′ (utama) | P1′-par (sekunder, tidak dikutip) |
|---|---|---|
| 1. jadwal FISIK lama **tidak lagi optimum** | **10 / 45** | 11 / 45 |
| ↳ jadwal identik (alokasi + `(pose, tugas)`) | 18 / 45 | 21 / 45 |
| 2. regret jadwal lama: mean / median / maks | **1.83 % / 0.00 % / 19.86 %** (51.1 s) | 3.31 % / 0.00 % / 67.1 % (86.6 s) |
| 3. pindah total mean lama → P1′ | 3.956 → **3.978** (naik/sama/turun 1/44/0) | 3.956 → 3.978 |
| ↳ `|n_1 − n_2|` mean | 1.511 → **1.511** | 1.511 → 1.511 |
| 4. optimum dengan pindah = `m_min` | **P1′ 44/45**, FISIK 45/45 (`m_min` mean 3.956) | 44/45 |
| 5. pangsa gantry kritis: tugas / retract / overhead rel / rel | **53.5 / 25.3 / 3.5 / 17.7 %** | — |
| 6. makespan mean P1 → P1′ | 163.35 → **312.41 s** (×1.96) | — |
| seed 1: P1 → P1′ (paralel) · jadwal G24b tetap optimum? | 103.24 → 263.09 · **ya** (identik) | — |

**Kenapa jadwal berubah — dibaca per instance, bukan ditakar:**

- **5 instance regret besar (seed 0, 12, 17, 23, 38: 37.7–51.1 s, 12–20 %) = bias K2 G21.**
  Pada kelimanya jadwal lama mengerjakan tugas **di `p0` dulu**, lalu pindah (membayar
  retract penuh). Optimum P1′ **meninggalkan `p0` tanpa tugas** — lengan masih REST,
  retract dilewati (`t_fold_first` = 4.74 s) — dan menggabungkan tugas `p0` ke perhentian
  berikutnya. Jumlah pindah **sama**, retract **berkurang satu**. Contoh seed 38 g2: lama
  `0.00 [t0] → 0.05 [t1] → 1.45 [t4]`, baru `0.15 [t0, t1] → 1.45 [t4]`. Total atas 45:
  gantry yang mulai di `p0` **27 → 17** (dari 90), retract per instance **2.556 → 2.356**.
  Ini persis bias yang g21 A2 catat ("lengan siap di `p0` pada t = 0 — menguntungkan `p0`")
  dan tidak diperbaiki sampai sekarang: harganya terukur **≤ 51 s (20 %)** pada 5/45.
- **5 instance regret kecil (16, 20, 30, 34, 44: 0.2–0.5 s)** = pemecah seri rel berganti
  (`T_lin` → `r·T_cmd`, lantai 3 s).
- **Seed 18: 5 pindah vs `m_min` 4, regret 0** — optimum seri; solver memilih satu dari
  beberapa jadwal berbiaya sama (jadwal lama juga optimum P1′).
- **Biaya tugas (53.5 % makespan) tidak menggeser satu pun optimum**, dan ini struktural:
  **0/270 tugas layak di kedua gantry** (oracle‴ sempit; tiap tugas punya satu gantry),
  jadi alokasi tugas → gantry **terpaksa**, dan dengan lengan serial suku tugas =
  `n_g · c_task` = **konstanta per gantry**. Pada instance ini, suku terbesar model tidak
  dapat mengubah keputusan apa pun — ia hanya menaikkan harga. (Dengan tugas yang layak
  di dua gantry, penyeimbangan akan berbiaya ~44 s per tugas; tidak diukur di sini.)

**Kalimat yang boleh dikutip:** *di bawah biaya terukur (P1′), optimum tetap
minimum-jumlah-pindah (44/45, satu seri); yang berubah adalah **di mana** pindah terjadi —
perhentian di pose awal ditinggalkan bila lengan masih terlipat, karena retract (50.9 s)
adalah suku pindah yang mahal, bukan rel. Jadwal model lama rugi ≤ 20 % (mean 1.8 %).*
Batasan: n = 6, `mr = 0`, `rot = 0`, tugas satu-gantry, satu set kalibrasi (G19/G20).

### B4. Papan skor — D107–D113

| # | Dugaan | Terukur | |
|---|---|---|---|
| D107 | P1′-serial dalam ±10 % dan di bawah | −4.80 % (457.39 < 480.44) | ✅ ⚠️ terkontaminasi — **tidak dihitung** |
| D108 | 6/6: P→M < 1 s dan R→X ≥ 50 % wall | P→M ≤ 0.069 s; R→X 65–71 % | ✅ |
| D109 | jadwal lama tidak optimum ≥ 23/45 | **10/45** | ❌ |
| D110 | pindah mean P1′ ≤ 3.96 | **3.978** (satu seri, seed 18) | ❌ |
| D111 | regret mean ≤ 5 % dan maks ≤ 20 % | 1.83 % / **19.86 %** | ✅ (maks 0.14 poin di bawah bar) |
| D112 | G0′ menemukan ≥ 1 ketidakcocokan sebelum lulus | **192/192 percobaan pertama** (mutan tertangkap) | ❌ |
| D113 | `|n_1 − n_2|` turun | **1.511 → 1.511** — alokasi terpaksa (0/270 dua-gantry) | ❌ |

**G25: 4 meleset / 2 tepat** (D107 tidak dihitung) → papan skor **56 meleset / 55 tepat**.
Pola: D109, D113 menduga biaya tugas besar **akan** mengubah keputusan — meleset karena
struktur instance (alokasi terpaksa) yang tidak diperiksa sebelum menduga; arah "kendala
lebih mengikat dari kenyataan" (prior lama). D110 meleset oleh satu seri. D112: prior "kode
sendiri lebih salah" meleset untuk `sched.py` — tetapi skrip analisis saya **memang** crash
sekali (B5 (2)); bug-nya di alat ukur, bukan di solver yang diuji.

### B5. Pertentangan §B lawan §A — G25

| # | Pertentangan |
|---|---|
| (1) | Prompt: dekomposisi G24b **sebelum** kunci; dijalankan **sesudah** (A0, alasan kontaminasi) |
| (2) | A4.4: `m_min` dengan `dwell = 0` — `_dur_table` memberi `inf·0 = NaN` (run pertama crash); dipakai `dwell = 1e-6` (inf tetap inf; pembulatan pindah tidak terganggu, traverse Σ ≤ 318 s ≪ 5e3) |
| (3) | A5 menyebut assert di heuristik saja; ditambah juga di `sched_coupled` (membaca `t_fold` lewat `traverse_time`) |
| (4) | A2 "gantry paralel" untuk P1′ vs eksekusi G24b **serial** (A3 g22): keduanya dilaporkan (263.09 paralel, 457.56 serial) — keputusan runner, bukan model |

**G25: EMPAT.**

## C. Keadaan akhir (Rule 12)

- **Diubah:** `sched.py` (3 medan + `default_costs`; default bit-identik: V0–V4, G21 E1
  420/420, G24 100/100), `sched_heur.py` + `sched_coupled.py` (1 assert masing-masing).
- **Baru:** `docs/results/p1_g25/` — `decomp_calib.py`, `decomp_g24b.py`, `calibrate.py`,
  `c0_control.py`, `g0_gate.py`, `g0_mutants.py`, `run_sched45.py`, json/log, `sectionA_locked.md`.
- **Tidak diubah:** peta, oracle, `make_instance_g24.py`, alat G22 (`rail_to_g`, `run_g22`,
  `sched_screen`, `return_rest`), probe.
- **Belum dikerjakan / tidak diukur:** model tugas yang bergantung lintasan (saringan ∝ exec
  — butuh prediktor panjang lintasan sebelum rencana); retract gantry 2 dan `rail_to_g` di
  set kalibrasi (sekarang hanya G24b, n = 2); tugas yang layak di dua gantry (penyeimbangan);
  P1′ di instance G21 (n ≤ 10, 1–2 gantry) — hanya 45 instance G24; sensitivitas `c_task`.
- **Peluang terbesar yang terukur (tidak dikerjakan — alat G22 tidak diubah):** saringan
  antar-lengan = 65–71 % tiap tugas ≈ **35 % makespan G24b** (~187 s dari 480), dan > 99 %
  darinya adalah hitung jarak per titik lintasan + bangun geometri per proses. Itu
  perangkat lunak kita, bukan sel (seperti DINDING g21 A1a).

## D. Prompt G26 (salin ke chat BARU saat operator di lokasi) — hardware

**Rekomendasi: Opus, effort TINGGI** — perangkat keras + prediksi terkunci; kesalahan
paling mungkin diam: instance dibuat dengan `p0` salah (0.55 lama), dan dugaan yang ditulis
sesudah melihat run pertama.

```
Sesi G26-HW -- replikasi jadwal scheduler di sel NYATA, beberapa seed, prediksi P1' TERKUNCI.
Repo ceiling_arm, branch feat/rgbd-topo-deploy.

BACA PENUH: CLAUDE.md; docs/p1_state.md 6 + 7 + tally; docs/p1_g25_task_cost.md (A2, B1-B5, C);
docs/p1_g24_roll_oracle.md A5, D, E (E4 prosedur, E7 pertentangan, E8 keadaan akhir);
docs/p1_g22_hw.md A3, A5 (palang keselamatan).

LATAR: G24b seed 1 = 6/6, 480.44 s. G25: model terukur P1' (c_task 43.65, c_ret 50.87,
rel 0.9502*T_cmd + 4.74, lengan serial, retract dilewati pada keberangkatan pertama)
memprediksi G24b 457.39 (-4.8 %); galat per tugas rerata 7.9 s. Optimum P1' = min pindah
44/45, tetapi meninggalkan p0 tanpa tugas bila lengan REST (bias K2 lama, <= 51 s).
REL SEKARANG 0/0 (g24 E8: g1 0.000503, g2 0.000534) -- instance G22-G24 memakai p0 0.55/0.00.

0. Tahap 0 nol gerak = g24 E0 apa adanya (operator ditanya ULANG: LED, origin, rel bebas;
   remount_check; bring-up; /joint_states: rel TERBACA -> p0).
1. KUNCI SEBELUM SOLVE (A): instance = make_instance_g24 dengan p0 = rel TERBACA (0.00/0.00),
   aturan (i)-(iii) g24 A5 apa adanya; scheduler = sched.solve_exact dengan biaya P1'
   (docs/results/p1_g25/run_sched45.p1prime, konstanta g25_constants.json, TIDAK dikalibrasi ulang).
   Seed diambil berurutan; (iv) sched_screen; jalankan seed lolos PERTAMA, lalu berikutnya
   sampai K seed (K dikunci sebelum, usul 3) atau operator berhenti.
2. Prediksi per event + makespan P1'-serial DIKUNCI per seed SEBELUM gerak (calibrate.predict
   dengan urutan A3 seed itu). Dugaan D114+ SEBELUM run pertama: galat makespan, galat per tugas,
   apakah retract dilewati bila jadwal meninggalkan p0 tanpa tugas (run_g22 A3).
3. Eksekusi = g24 E4-E5 apa adanya (return_rest --move dulu, DRY, izin operator per gerak, S25).
   Sesudah tiap seed: decomp_g24b.py (diadaptasi prefix) -> galat per komponen.
4. Akhir: rel kembali ke 0/0 (atau nilai yang operator minta, DITULIS); stack mati (ros2-launch
   PID leak: SIGINT anak, cek node list --no-daemon); p1_state 6 + tally; prompt G27.

ATURAN: 7.1 ground truth dulu; 7.2 dugaan dikunci sebelum data; B menang atas A dan DITULIS.
Konstanta P1' TIDAK diubah sesudah melihat run. Gerak hanya dengan izin operator.
```
