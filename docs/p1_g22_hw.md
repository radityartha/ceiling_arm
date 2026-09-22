# P1 / G22-HW — SATU jadwal keluaran scheduler, end-to-end, di sel NYATA

Lanjutan [p1_g21_sched_tfold.md](p1_g21_sched_tfold.md) (model biaya pindah
terukur: `t_fold` FISIK **50.80 s** / DINDING **126.80 s**; optimum = minimum
jumlah pindah, §B2) dan [p1_g20_hw.md](p1_g20_hw.md) (langkah 5 LULUS 10/10
CONCURRENT, rel **tetap**). G20/G21 **tidak** disambung; dokumen ini dimulai baru.

Pertanyaan: jadwal yang keluar dari `solve_exact`, dijalankan di sel nyata
(empat lengan, dua gantry, gantry **bergerak**) — berapa makespan terukurnya
lawan prediksi model, dan komponen mana yang menjelaskan selisihnya?

Yang **diwarisi apa adanya dan tidak ditulis ulang**: g17 §A1 / g20 §A1
(sukses = penilai independen, 5 mm / 5° / 2.0 s, `pos_err_max_settled_mm`);
g16 §A1/§A3/§A5/§A6 (S1–S7), §A10 (Rule 6 dikecualikan); g17 §A3 (S8–S11);
g19 §A5 (S12–S17); g20 §A5 (S18–S23); g21 §A1 (tiga nilai `t_fold`, hanya
itu), §A2 (pembebanan per pindah per gantry). `sched.py` / `sched_heur.py` /
peta **tidak diubah** sesi ini.

---

## A. Protokol — ditulis 2026-09-22, SEBELUM satu pun gerak sesi ini

> 🔒 A1–A3 dikunci sebelum solve / saringan (A2 sebelum solve pertama, A3
> sebelum saringan (iv)); A4–A6 ditulis sebelum saringan (iv) selesai; A8
> dijawab operator sebelum bring-up. **Saringan (iv) meloloskan 0/35 → menurut
> A2 jadwal TIDAK dijalankan (§B1).** Nol gerak perangkat keras sesi ini.

### A1. Yang diuji — dan tiga ketidakcocokan model↔sel yang diketahui SEBELUM data

Model ([g7 §A1](p1_g7_sched.md), [g21 §A2](p1_g21_sched_tfold.md)), per gantry,
tanpa waktu tunggu:

```
finish_g = Σ_pindah [ T_traverse(Δ) + t_fold ]  +  Σ_perhentian slot × 2.0 s
makespan = max_g finish_g                          (gantry INDEPENDEN, paralel)
T_traverse = max(T_lin, T_rot);  rot = 0 di sesi ini → T_lin(Δ) = 0.29 + Δ_mm / 31.416
```

Dibaca dari kode/dokumen, **sebelum** data — ketiganya akan membuat terukur ≠
prediksi secara sistematis, dan karena itu masing-masing punya prediksi sendiri
(A4), bukan dijelaskan sesudahnya:

| # | Model | Sel sebagaimana dioperasikan | Arah |
|---|---|---|---|
| **M1** | dua gantry **paralel**, makespan = **max** | S11 + S18: tiap rencana disaring terhadap ketiga lengan lain pada konfigurasi **TERUKUR**; itu sah hanya bila lengan lain **diam**. Jadi hanya **satu** kelompok aktuator bergerak pada satu waktu (A3) → dua garis waktu gantry **berurutan**, makespan ≈ **Σ** | terukur ≫ max |
| **M2** | K2: lengan "siap" di `p0` pada t = 0 — perhentian pertama di `p0` tidak membayar extend; keberangkatan pertama membayar `t_fold` penuh (retract **dan** extend) | lengan mulai di **REST**: perhentian di `p0` membayar extend; keberangkatan pertama dari REST tidak punya gerak retract | tergantung jadwal — dihitung per jadwal (A4) |
| **M3** | gerak lengan **di dalam** perhentian = 0 (g7 §A4.4); tugas ke-2 lengan yang sama = +1 slot 2.0 s | tiap tugas berikutnya pada lengan yang sama = satu rencana + saringan + eksekusi (g19: ~25 s + ~11 s) | terukur > model |

Juga: `T_traverse` model = `T_lin`, sedangkan bridge terikat **`T_cmd`**
(g19 §A1, terukur ×1.62). Keduanya ditulis (A4).

MR tidak dipakai: tugas MR = **kedua** lengan satu gantry di **titik yang sama**
dalam satu jendela (g7 A2-K3). Dua ujung alat di satu titik = tabrakan (g20 X0-B:
−16.7 mm di titik bersama); saringan S18 akan menolaknya, jadi instance MR
tidak dapat dieksekusi di sel ini menurut konstruksi. `n_mr = 0`.

### A2. 🔒 Aturan pemilihan instance — DITULIS SEBELUM SOLVE PERTAMA

Tidak ada instance yang dipilih dengan tangan. Aturannya mekanis, urutannya
tetap, dan yang pertama lolos semua syarat **adalah** instancenya.

```
untuk seed = 0, 1, 2, … 49 (urut):
  inst = sched.gen_real(n_tasks=6, seed, n_mr=0, gantries=(1, 2),
                        maps=data/cap_g{1,2}_rail160.npz)          # tidak diubah
  R0  pose dibatasi ke rot = 0 (33 pose rel 0.00…1.60 m per gantry):
      poses/reach/zone/hand di-subset ke kolom rot = 0.  (S23: penyaring S18 tidak
      memuat sendi rotasi; dan pada rot = 0 BLOCK g9 MUSTAHIL — Lemma 1)
  R1  p0 = pose rel TERBACA saat bring-up, dibulatkan ke grid 0.05 m
      (g20 §C: g1 0.550910 → 0.55, g2 0.000000 → 0.00; dicek ulang tahap 0)
  syarat model (tanpa perangkat keras):
   (i)   tiap tugas layak di ≥ 1 gantry di bawah R0 (kalau tidak, solve_exact
         menolak → seed dilewati, dicatat)
   (ii)  jadwal exact di t_fold FISIK punya ≥ 1 pindah di SETIAP gantry
         (sesi ini ada untuk menggerakkan KEDUA rel; gantry 2 belum pernah)
   (iii) schedule_conflict (sched_coll, g9 BLOCK, c_clear = 0) = tidak ada
         — dicek, walau Lemma 1 menjaminnya pada rot = 0
  syarat perangkat keras, PLAN-ONLY, nol gerak (§A5 tahap 1):
   (iv)  saringan jadwal 3/3: seluruh jadwal FISIK disimulasikan dalam urutan
         eksekusi A3 — rel DITEMPATKAN, lengan lain DITEMPATKAN di keadaan
         saat itu (REST atau target terakhirnya), tiap rencana disaring torsi
         (A8-1) + S18 terhadap ketiga lengan lain; tiap traverse disaring S24.
         Tiga sampel independen, SEMUA rencana PLANNED di ketiganya (k-of-k).
  instance = seed pertama yang lolos (i)–(iv).
```

- Syarat (ii) dicek **hanya** di FISIK; jadwal yang dieksekusi = rekonstruksi
  FISIK. Kalau optimum DINDING beda **jadwal** (seri, g21 B2), prediksi DINDING
  dihitung pada **jadwal FISIK yang sama** (jumlah pindah sama → biaya sama, g21
  B2: selisih = 76.0 × pindah pada 140/140) — dan perbedaannya ditulis.
- Kalau tidak ada seed 0–49 yang lolos (ii): **cadangan yang dikunci
  sekarang** = syarat (ii) diganti "≥ 1 pindah total, dan kedua gantry punya
  ≥ 1 tugas", diulang dari seed 0. Tidak ada cadangan lain.
- Kalau tidak ada yang lolos (iv) sampai seed 49: sesi **tidak** menjalankan
  jadwal; dilaporkan. Saringan tidak dilonggarkan.
- `n = 6` (batas atas prompt): lebih sedikit tugas memperkecil peluang
  gantry bergerak sama sekali (optimum = min pindah).

### A3. 🔒 Urutan eksekusi — SERIAL per gantry, aturannya dikunci sebelum saringan

Satu runner, satu kelompok aktuator bergerak pada satu waktu (S11 diperluas ke
rel; alasan M1). Urutan:

```
awal     : keempat lengan REST (maks |q − REST| < 0.5°), rel di p0 (R1). t0 = perintah pertama dikirim.
BLOK g = 1, lalu g = 2 (gantry 1 dulu — tetap, bukan dipilih per jadwal):
  untuk tiap perhentian k gantry g, urutan rekonstruksi solve_exact:
    kalau pose_k ≠ pose saat ini:
      RETRACT : return_rest --arms <kedua lengan g> --move   — DILEWATI bila keduanya
                sudah < 0.5° dari REST (mis. keberangkatan pertama; dicatat 0 s)
      TRAVERSE: rail_to_g --gantry g <rel_k> --move          (S12 + S24, §A5)
    TUGAS   : per lengan, urut indeks tugas; lengan bergantian (slot-0 arm dulu:
              arm_1 / arm_3), tiap tugas = reach_dwell_probe --arms <lengan>
              --target=<xyz> --move (disaring torsi + S18 vs KETIGA lengan lain);
              lengan lalu MENAHAN di target itu sampai gerak berikutnya
  sesudah tugas terakhir blok g: lengan g TETAP MENAHAN (model: gantry diam)
akhir    : makespan terukur = t0 → event `success` penilai untuk tugas TERAKHIR.
           Retract penutup keempat lengan SESUDAHNYA, tidak dihitung.
```

Sukses tiap tugas = event `success` penilai (N = 1, A1 g17) untuk lengan itu
**sesudah** targetnya diterbitkan. Tugas yang HALTED (tidak ada rencana lolos
saringan dalam 3 percobaan) **tidak** diulang dan tidak diganti; runner lanjut
ke tugas berikutnya (tiap rencana tetap disaring hidup), dan hasilnya
**TIDAK LENGKAP k/6** — makespan lalu dilaporkan sebagai tidak terdefinisi,
komponennya tetap dilaporkan.

### A4. 🔒 Prediksi — dihitung dari jadwal yang DIKUNCI, ditulis di §B0 SEBELUM gerak

Per gantry dari jadwal FISIK: `m_g` pindah, `Σ T_lin`, `Σ T_cmd`
(`T_cmd = max(3, πΔ / (2·0.9·v_lin))`, rumus `rail_to` g19), `D_g = Σ slot × 2.0`.

| | rumus per gantry `F_g` | makespan model (paralel) | makespan serial (A3) |
|---|---|---|---|
| **P1** FISIK, `T_lin` (= angka model, yang dikutip naskah) | `Σ T_lin + 50.80·m_g + D_g` | `max_g F_g` | `Σ_g F_g` |
| **P2** FISIK, `T_cmd` | `Σ T_cmd + 50.80·m_g + D_g` | max | Σ |
| **P3** DINDING, `T_lin` | `Σ T_lin + 126.80·m_g + D_g` | max | Σ |
| **P4** DINDING, `T_cmd` | `Σ T_cmd + 126.80·m_g + D_g` | max | Σ |

Koreksi M2/M3 (sekunder, dilaporkan terpisah, **tidak** menggantikan P1–P4),
dari median g19 B1.2 apa adanya: perhentian di `p0` dari REST → `+` extend
(FISIK: exec 12.50 + 9.28 = 21.78 s untuk dua lengan; DINDING: 71.09 s);
keberangkatan dengan lengan sudah REST → `−` retract (FISIK 29.02; DINDING
48.81 + sela 4.62); tugas ke-2+ lengan yang sama di satu perhentian → `+` satu
rencana+eksekusi lengan (DINDING ~36 s, FISIK ~11 s). Nilai satu-lengan tidak
pernah diukur (g19 selalu dua) — ditulis sebagai asumsi, bukan angka.

### A5. 🔒 Palang keselamatan BARU + tahap

S1–S23 tetap. Tambahan:

| # | Aturan |
|---|---|
| **S24** | `rail_to_g` (baru, dari `rail_to` g19): gantry 1 **atau** 2; target **hanya** nilai rel jadwal terkunci gantry itu (+ p0-nya untuk pemulihan); ≤ 1.600 m; S12 untuk lengan gantry itu; **saringan sapuan**: rel gantry yang bergerak disampel ≤ 10 mm sepanjang lintasan, lengannya di REST terukur, keenam sendi lengan + rel gantry lain **terukur**, `CrossGantryChecker` margin 50 mm — MARGIN/COLLIDE/tak-bisa-menyaring = **MENOLAK**; rotasi kedua gantry dibaca 0 (S23); puncak setpoint 0.9·v_lin; auto-stop S17 (drift lengan > 0.5°, galat rel > 2 mm) |
| **S25** | Gerak rel gantry 2 **pertama di proyek ini**: operator **melihat** arah (+x world) dan carriage berhenti; origin 0.000000 dikonfirmasi fisik **sebelum** (S19) |
| **S26** | Runner berhenti (auto-stop) sebelum langkah berikut pada: rc ≠ 0 alat mana pun kecuali HALTED probe (A3); torsi terukur lengan mana pun > 14; fault / Kortex exception di log launch; node penilai / perekam / `ros2_control_node` mati; rel bergeser > 2 mm saat **tidak** diperintah; \|rotasi\| > 0.5° |
| **S27** | Disk: ≥ 1.5 GB bebas sebelum bring-up (perekam g20 = 95 MB, crash dump shutdown ~400 MB); crash dump milik sesi ini dihapus |

| Tahap | Isi | Izin |
|---|---|---|
| **0** | LED keempat lengan (operator, C1 g20); `remount_check` + 4× ICMP; bring-up `arm3_fake:=false arm4_fake:=false enable_gantry_bridge:=true`; 4× "Actuator count … '6'", 7/7 controller, nol fault; validator penilai 7/7 | operator |
| **1** | PLAN-ONLY, nol gerak: saringan (iv) per seed urut A2 → instance; `rail_to_g` DRY tiap traverse jadwal (S24); `return_rest` DRY. **Instance + prediksi §B0 + D79+ dikunci di sini** | — (nol gerak) |
| **2** | SATU PINDAH: gantry 2, lengan REST, rel 0.000 → rel pertama jadwal g2 → kembali 0.000 (S25). Lulus bila galat ≤ 2 mm, drift < 0.5°, nol fault. Tidak dihitung | operator |
| **3** | JADWAL PENUH, satu kali (A3). Retract penutup. | operator |

### A6. 🔒 Besaran yang DILAPORKAN — terlepas dari hasilnya

Instrumen: jam runner (tiap baris keluaran alat dicap saat tiba), perekam
`/joint_states` (`js_record.py`, 28 sendi), status penilai (`/reach_dwell/status`,
cap terima), log `rail_to_g` (JSON). Bukan stopwatch.

| Besaran | Definisi | Pembanding |
|---|---|---|
| **makespan terukur** | `t0` → event `success` tugas terakhir | P1–P4, max **dan** Σ (A4) |
| per gantry `W_g` | awal event pertama blok g → `success` terakhir blok g | `F_g` P1–P4 |
| `t_retract` | panggil `return_rest` → rc (dilewati = 0 + overhead panggil) | FISIK 29.02 / DINDING 48.81 (g19) |
| `t_traverse` | rel bergerak > 0.5 mm → rel berhenti (`rail_to_g`) | `T_lin` **dan** `T_cmd` |
| `t_extend` / `t_instop` | panggil probe → eksekusi selesai, untuk tugas pertama tiap lengan di perhentian / tugas berikutnya (M3); dipecah rencana+saringan / eksekusi dari log | FISIK: exec g19; DINDING 71.09 per dua lengan |
| `t_dwell` | eksekusi selesai → `success` | 2.0 s per slot (model) |
| `t_sela` | `W_g` − Σ di atas | 0 (model) |
| jumlah pindah | perubahan rel per gantry | `m_g` jadwal |
| torsi | puncak `joint_2` per tugas vs 14 / 12; terukur − RNEA per rencana (offset +7.7 dinilai pada rencana **baru**) | g21 B8 |
| rel | galat akhir, kopling (drift lengan saat traverse, termasuk lengan gantry LAIN yang menahan) | g19 B1.3 |
| jarak antar-gantry | min sapuan S24 per traverse; min S18 per rencana | 50 mm |

### A8. Keputusan OPERATOR — dicatat SEBELUM data (2026-09-22)

| # | Pertanyaan | Jawaban operator |
|---|---|---|
| **1** | Offset penyaring `joint_2`: RNEA + 6.6 ≤ 14 (g16–g20) atau RNEA + 7.7 ≤ 14 (g21 B8)? Batas 14 tetap | **+7.7 ≤ 14** — `JOINT_TORQUE_OFFSET_NM[2]` 6.6 → 7.7 di `reach_dwell_probe.py` (hanya `joint_2`; j1/j3 tetap 6.6, pergelangan 1.98). Berlaku untuk saringan (iv) **dan** hari-H |
| **2** | LED keempat lengan (g20 C1) | **tidak merah** (dicek operator sebelum bring-up) |
| **3** | Origin gantry 2 = home fisik (enkoder 0.000000); rel g2 bebas s.d. ~1.6 m; ruang antar-gantry bebas; nol alat operator | **semua benar** |
| **3b** | Mode eksekusi (M1) | **serial per gantry** (A3) |
| **3c** | Tahap 2: gerak rel g2 pertama keluar-kembali dulu, lengan REST | **ya** |
| **4** | Instance yang dipilih (A2) | ⏳ sesudah saringan (iv), tahap 1 |


---

## B. Hasil terukur

### B0. SEBELUM gerak apa pun — 2026-09-22

#### B0.1 Syarat model (i)–(iii) — offline, [make_instance.py](results/p1_g22/make_instance.py)

Data: [g22_candidates.json](results/p1_g22/g22_candidates.json). 50 seed, `n = 6`,
`n_mr = 0`, R0 (rot = 0, 33 pose per gantry), R1 (p0 = 0.55 / 0.00).

| | seed |
|---|---|
| (i) semua tugas layak di bawah R0 | **50 / 50** |
| (ii) ≥ 1 pindah di **setiap** gantry (FISIK) | **35 / 50** (sisanya: 0/0 pindah ×3, satu gantry saja ×12) |
| (iii) konflik BLOCK g9 | **0 / 50** (Lemma 1: rot = 0) |
| jadwal FISIK = jadwal DINDING | **50 / 50** (g21 B2 direplikasi pada R0) |
| makespan model FISIK, 35 lolos | 61.05 – 94.88 s; semua 1 pindah per gantry |

#### B0.2 Alat baru — nol gerak

- `reach_dwell_probe.py`: `JOINT_TORQUE_OFFSET_NM[2]` 6.6 → **7.7** (A8-1);
  `collect_concurrent` untuk **N = 1** kini kembali pada event `success` lengan itu
  (dulu menunggu penuh `--settle` 8 s karena penilai tidak pernah menerbitkan
  `concurrent` untuk satu lengan, g20 B1.2 — ~6 s sela palsu per tugas yang
  akan masuk makespan). Tidak dipakai di data sesi ini (tahap 3 tidak jalan).
- [g22_plan.py](results/p1_g22/g22_plan.py): urutan event A3 + saringan sapuan
  S24 — **satu** definisi untuk saringan dan runner.
- **S24 divalidasi** ([v_sweep.py](results/p1_g22/v_sweep.py) → [log](results/p1_g22/v_sweep.log)):
  X0 REST×REST g2 0→0.45 **CLEAR 546.1 mm** (= g20 X0-A); X1 kontrol negatif —
  arm_1 di-IK ke `t2_a1_arm_link` saat g2 menggantung di rel 0.25 — sapuan
  0→0.45 **COLLIDE −45.0 mm di tengah lintasan**, sementara X2 kedua ujung
  sendiri **CLEAR** 178.6 / 86.8 mm. Jadi S24 adalah sapuan, bukan cek ujung.
  🔴 Kesalahan saya selama menyusun kontrol: dua run pertama memberi angka
  bertentangan (−32 mm vs +119 mm untuk penempatan "sama") — `_ik` menulis ke
  `c.data`, jadi titik target dibaca **sesudah** IK sebelumnya merusak FK, dan
  run profil memakai IK arm_2 yang **tidak konvergen**. Penyaring tidak salah;
  skrip uji saya salah. Ditelusuri sebelum dipercaya.
- [rail_to_g.py](results/p1_g22/rail_to_g.py), [sched_screen.py](results/p1_g22/sched_screen.py),
  [run_g22.py](results/p1_g22/run_g22.py), [js_record.py](results/p1_g22/js_record.py).
  `rail_to_g` dan `run_g22` **belum pernah dijalankan hidup** (Rule 12).

#### B0.3 Tahap 0 — bring-up, nol gerak

| Gerbang | Hasil |
|---|---|
| `ros2_kortex` | `ceiling-arm-fixes` **e712295** ✅ |
| `remount_check.py` | **GERBANG LULUS**; ICMP .10–.13 ✅ |
| bring-up (`/tmp/g22_t1.log`, SIGINT tidak diabaikan) | **4×** "Actuator count … '6'"; **7/7** controller active; nol FAULT / Kortex exception. 3 spawner "process has died" = kelas spawner ganda g20 (controller tetap active) ✅ |
| `/joint_states` | rel g1 **0.550910**, g2 **0.000094** m (g20: 0.000000; 9 pulsa, di bawah grid) → R1 tetap 0.55 / 0.00; rotasi 0 / 0 ✅ |
| lengan | maks \|q − REST\| arm_1 0.655°, arm_2 0.465°, arm_3 **0.840°**, arm_4 0.573° — **tiga di atas 0.5°** (S12 / A3-awal). Tidak ada `return_rest` dijalankan (tahap 2–3 tidak terjadi) |
| validator penilai | **7/7 PASS** ([g22_validator.log](results/p1_g22/g22_validator.log)). 🔴 penghitung "sebelum/sesudah" mencetak 3/2 — **cocok dengan cmdline bash saya sendiri**, persis jebakan g19 B0.3 yang sudah tercatat; `ps` tanpa shell: **0** proses penilai sesudah; `remount_check` sebelum: "tidak ada reach_dwell_monitor basi" |

### B1. 🔒 Saringan (iv) — **0 / 35 seed lolos**. Jadwal TIDAK dijalankan (A2).

[sched_screen.py](results/p1_g22/sched_screen.py) di `move_group` stack nyata,
plan-only, lengan lain + rel **ditempatkan**; data [g22_screen.json](results/p1_g22/g22_screen.json),
[g22_screen.log](results/p1_g22/g22_screen.log). Tiap seed berhenti di penolakan
pertama (k-of-k: satu penolakan = seed gagal), jadi **tidak ada seed yang
mencapai sampel 2**, dan **tidak ada** yang mencapai blok gantry 2.

| per rencana tugas (49 dicoba) | PLANNED | TORQUE-UNSAFE | NO-PLAN |
|---|---|---|---|
| semua | **14** (29 %) | 12 | 23 |
| di `p0` (rel 0.55) | 14 / 36 | 11 | 11 |
| **sesudah pindah** (rel 0.75–1.40) | **0 / 13** | 1 | **12** |
| per lengan | arm_1 8 / 42, arm_2 6 / 7 | | |

- **NO-PLAN** = batas `allowed_planning_time` 15.0 s habis (23/23 tepat 15.0 s).
  Targetnya di tepi jangkauan dengan approach **vertikal** wajib (probe: ori
  tol 2°), mis. (1.929, −0.106, 1.08) di rel 1.40, (2.000, −0.106, 1.40) di rel
  1.10. Peta menerima tugas itu di **L1 = 5 cm, approach ≤ 45°, tanpa torsi**
  (p1_state §8b: L1 ≠ L2).
- **TORQUE-UNSAFE** diprediksi (RNEA + 7.7) **14.67 – 17.42** N·m; tiga
  (14.67 / 14.80 / 14.98) akan **lolos** dengan +6.6 — jadi keputusan A8-1
  menolak 3 dari 12, yang lain tegas.
- Retract / traverse tersimulasi: **4/4** retract dan **13/13** sapuan S24
  **CLEAR** (sapuan min 498.9 mm). Yang menolak **hanya** rencana lengan.

**Kenapa, menurut mekanisme yang sudah diukur (bukan diduga):** optimum
g21 = **minimum jumlah pindah**; solver pindah **hanya** ketika sebuah tugas
tidak terjangkau dari `p0`, dan itu berarti tugas di **pinggir** kapabilitas —
tepat kelas yang peta L1 terima dan perencana L2 + torsi tolak (g17 B1.8:
peta statis melebih-lebihkan kelayakan lintasan ~3×; g16 B6 / memori: ~30 %
ruang terjangkau arm_1 tidak aman-torsi). Di sini per rencana **29 %**, dan
**0 %** tepat di pose yang membuat jadwal ini jadwal (sesudah pindah).
Dengan 29 % per rencana, 18 rencana berturut-turut (6 tugas × 3/3) lolos hanya
kalau penolakan sangat berkorelasi; **kalau** independen (asumsi yang D65
tunjukkan salah arah) peluangnya ~0.29¹⁸ — bagaimanapun, 0/35 bukan nasib buruk.

➜ **Klaim yang boleh ditulis:** *jadwal optimum dari oracle kapabilitas L1
tidak dapat dieksekusi di sel ini pada 35/35 instance yang diuji; penolakan
terkonsentrasi tepat di perhentian yang membenarkan pindah gantry.* Makespan
terukur vs prediksi **tidak diukur** sesi ini.

### B2. Papan skor §7.2 — **tidak ada dugaan yang dinilai**

D79+ direncanakan dikunci **sesudah** instance terpilih dan **sebelum** gerak
(A5 tahap 1). Saringan (iv) sendiri adalah data, dan saya **tidak** mengunci
dugaan tentang hasil saringan sebelum menjalankannya — kesalahan proses saya
(prompt: "dugaan dikunci SEBELUM data"). Tidak ada dugaan yang ditulis sesudah
data untuk menutupinya. Papan skor tetap **41 meleset / 37 tepat**.

### B3. Pertentangan §B lawan §A — G22

G20 sembilan, G21 tujuh.

| # | Pertentangan |
|---|---|
| **(1)** | A2 menganggap syarat (iv) penyaring **akhir** yang meloloskan sebagian; ia meloloskan **0/35** — seluruh sesi berhenti di sana |
| **(2)** | A1/A4 menyiapkan M1–M3 dan P1–P4 untuk jadwal yang dieksekusi; tidak ada yang dieksekusi — **tak satu pun** prediksi A4 dinilai |
| **(3)** | A5 tahap 1 menempatkan penguncian dugaan **sesudah** saringan (iv); saringan itu data yang layak diduga, dan tidak diduga (B2) |
| **(4)** | A3/prompt: "saringan 3/3" — tidak ada seed yang mencapai sampel ke-2; yang terukur efektif **1/1 per seed** sampai penolakan pertama |
| **(5)** | A3-awal mengandaikan lengan di REST (< 0.5°) sesudah bring-up; terbaca 0.47–**0.84°** (3/4 di atas) |
| **(6)** | validator: penghitung proses cocok dengan cmdline bash saya sendiri — jebakan g19 B0.3 terulang (B0.3) |
| **(7)** | kontrol negatif S24: dua run pertama saya bertentangan karena skrip uji (FK dirusak IK; IK tak konvergen), bukan penyaring (B0.2) |

**G22: TUJUH.**

---

## C. Keadaan akhir, dan yang BELUM dikerjakan (Rule 12)

- **Nol gerak perangkat keras** sesi ini (tidak ada `return_rest`, `rail_to_g`,
  atau probe `--move`). Lengan ditinggal menggantung 0.47–0.84° dari REST
  seperti saat bring-up; rel **0.550910 / 0.000094**, rotasi 0.
- Stack dimatikan: `kill -INT` ke launch (SigIgn tanpa bit SIGINT, g20 B1.1
  terpakai), `ros2 node list --no-daemon` **kosong**, nol baris fault /
  Kortex exception / `INVALID_USER_SESSION` di log launch (bandingkan g20 C1),
  25 baris *deactivate*. Crash dump rviz2 sesi ini (170 MB) dihapus; dump
  python 2026-09-21 12:28 (lebih tua) dibiarkan. Disk **2.0 GB** bebas.
- Data: [results/p1_g22/](results/p1_g22/) — `g22_joint_states_all.csv.gz`
  (14 MB) hanya membuktikan tidak ada gerak.
- **Belum pernah dijalankan hidup:** `rail_to_g.py` (termasuk gerak rel gantry
  2 **pertama** — S25 masih berlaku apa adanya), `run_g22.py`, perbaikan probe
  N = 1. Diuji hanya: kompilasi, `sweep_screen` offline (V_SWEEP), dan jalur
  saringan (iv) yang memakai kode `_plan_and_screen` hari-H.
- **Tidak diukur:** makespan terukur vs prediksi (inti sesi), M1–M3, P1–P4,
  offset +7.7 pada rencana baru yang **dieksekusi**.
- **Diubah di kode bersama:** `reach_dwell_probe.py` (offset j2 7.7; N = 1
  kembali pada `success`). Belum di-commit.

## D. Prompt sesi berikutnya — G23 (salin ke chat BARU)

**Kenapa ini berikutnya.** G22 menunjukkan jadwal optimum dari oracle **L1**
(peta 5 cm, approach ≤ 45°, tanpa torsi) **tidak dapat dieksekusi**: 0/35, dan
0/13 rencana di pose sesudah pindah. Selama oracle scheduler ≠ kelayakan
eksekusi, "makespan terukur vs prediksi" tidak dapat diukur sama sekali. Yang
diperlukan adalah oracle **L2-torsi** (approach vertikal, IK konvergen, torsi
statis aman) sebagai masker `reach` — model penjadwalan **tidak** berubah.

**Rekomendasi: Opus 5, effort TINGGI** — oracle baru menentukan instance mana
yang akan menggerakkan rel gantry 2 untuk pertama kali; kesalahan di oracle
diam sampai perencana menolak atau perangkat keras menabrak. Bagian offline
(oracle + solve + saringan) bisa **sedang**.

```
Sesi G23 -- oracle kelayakan L2-torsi untuk scheduler, lalu (kalau lolos) SATU
jadwal end-to-end di sel NYATA. Repo ceiling_arm, branch feat/rgbd-topo-deploy.

BACA PENUH sebelum menulis kode atau menyentuh hardware:
1. CLAUDE.md (Working Rules; Rule 6 dikecualikan untuk sesi protokol P1, g16 A10).
2. docs/p1_g22_hw.md -- SELURUHNYA. A1 (M1-M3), A2-A6 (protokol yang TIDAK
   sempat dipakai: pakai ulang apa adanya), B1 (0/35; 0/13 sesudah pindah;
   NO-PLAN = batas 15 s di tepi jangkauan; TORQUE 14.67-17.42), C (alat yang
   belum pernah hidup).
3. docs/p1_g21_sched_tfold.md A1/B2; docs/p1_g20_hw.md A5/C1; docs/p1_g19_hw.md A1/B1.2.
4. docs/p1_state.md 5.6-5.8, 7, 8b (L1/L2/L3), 8c.
5. docs/results/p1_g22/{g22_plan,make_instance,sched_screen,rail_to_g,run_g22}.py.

=== TUGAS ===
1. Tulis docs/p1_g23_*.md A DULU, KUNCI sebelum hitung apa pun:
   a. Oracle L2-torsi per (node peta, pose rot=0, lengan): IK posisi +
      approach VERTIKAL (sama dengan probe: quat x=1), konvergen < 2 mm,
      dan RNEA statis joint_2 + 7.7 <= 14 dengan margin yang ditulis SEKARANG
      (statis melebih-lebihkan lintasan ~3x, g17 B1.8). Validasi: oracle vs
      hasil perencana nyata pada rencana G22 B1 (49 rencana, 14 PLANNED) --
      berapa yang oracle tolak/terima; kontrol negatif wajib.
   b. Instance: aturan G22 A2 APA ADANYA, masker reach diganti oracle (a).
      Dugaan D79+ dikunci SEBELUM saringan (iv) -- termasuk laju lolos (iv).
2. Kalau ada seed lolos (iv): protokol G22 A3-A6 + tahap 2 (rel g2 keluar-
   kembali, S25) + tahap 3, tiap gerak izin operator. rail_to_g/run_g22
   BELUM PERNAH hidup -- DRY dulu.
3. Perbarui p1_state.md 6 + 8c + tally 9.

=== ATURAN ===
- 7.1 ground truth dulu; 7.2 UKUR, dugaan dikunci sebelum data (G22 lupa ini
  untuk saringan -- jangan ulangi).
- DILARANG memilih instance / t_fold / margin oracle sesudah melihat hasil.
- Kalau B bertentangan dengan A, B menang dan DITULIS. G21 tujuh, G22 tujuh.
- Penghitung proses: JANGAN cocok dengan cmdline bash sendiri (g19 B0.3, g22 B0.3).
- Disk ~2 GB: crash dump shutdown ~170-400 MB, hapus milikmu.
```
