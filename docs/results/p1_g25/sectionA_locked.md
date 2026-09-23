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
