# P1 / G64 JOINT-PLAN — perencanaan gabungan gantry + lengan (PRIORITAS operator 2026-10-17)

> Prompt: [p1_g64_joint_plan_prompt.md](p1_g64_joint_plan_prompt.md). Alat: [results/p1_g64/](results/p1_g64/).
> Sesi 2026-10-17 siang: **LURING saja, NOL gerak HW, NOL mock** (tugas 0 + bahan desain tugas 1). §A belum dibuat.

## 0. Diagnosis luring

### 0.1 Pisah (i) perencanaan-terpisah vs (ii) fisika — [g64_diag.py](results/p1_g64/g64_diag.py)

Per tuple (target, lengan, rel, rot) = oracle‴ (`oracle2.solve`, alat-bawah, roll bebas, amplop tilt, semua cabang):

| gerbang | isi (penyaring SAMA dengan sistem, tidak dilonggarkan) |
|---|---|
| torsi | amplop gravitasi statis per cabang + OFFSET (j2 7.7) + MARGIN (j2 0.66) ≤ LIM (= G49 `_branch_ok`) |
| STATIC | torsi ∧ lengan tugas + struktur gantry sendiri: EnvChecker ≥ 50 mm ∧ bebas kotak scene MoveIt |
| E1 | torsi ∧ seluruh gantry sendiri, partner REST: env ≥ 50 mm ∧ bebas kotak (= G49 e1) |
| POSE_A | kedua lengan REST di stop: env ≥ 50 mm ∧ bebas kotak (= G40 pose (a); kotak = START-JUMP) |
| LINE | G41 `screen_line` (garis sendi lurus ke REST, partner REST: torsi, env, kotak, antar-lengan) |
| TRAV | sapuan persegi REST p0 → stop (G33 / pose_to_g), env ≥ 50 mm |
| CURRENT | POSE_A ∧ TRAV ∧ ∃ cabang E1 ∧ LINE (= predikat tuple G49/P1″ + traverse) |

Kelas target: **OK-NOW** (∃ tuple CURRENT), **SPLIT** = kelas (i) (∃ STATIC, 0 CURRENT), **PHYS-\*** = kelas (ii)
(0 STATIC: tanpa IK / tanpa cabang torsi-ok / semua cabang torsi-ok kena env).
Grid: TAS-RAK + DEMO-OFF rel 0.05 m / rot 5°; G42 0.10 / 10°; CAP (20 simpul acak REF, seed 64) 0.10 / 15°; rot di dalam
amplop HW (g1 −125…+90, g2 −35…+95); prasaring jarak horizontal basis–target ≤ 0.88 m.
**Batas jujur:** cek env ≤ 40 cabang torsi-ok per tuple (10 pertama urutan G41 + 30 tersebar), LINE ≤ 12 cabang (G49 tanpa batas)
→ STATIC dan CURRENT hanya bisa **kurang-hitung**; statis saja (dinamika transit = penyaring rencana MoveIt, bukan di sini);
partner dianggap REST untuk E1/LINE, gantry lain tidak disaring (sama dengan predikat penjadwal).

Hasil: **§0.1 B (menunggu rantai, `g64_diag_chain.log`)** — diisi sesudah selesai.

Awal TAS-RAK z 1.34 (590 tuple): IK 95, torsi-ok 27, STATIC 27, STATIC ∧ POSE_A 8, **CURRENT 1** — arm_3 (0.50, −20°) j2 statis
5.625 vs batas 5.64 (selisih 0.015 N·m; grid manual TAS-RAK tidak pernah mencoba stop ini; rencana MoveIt hampir pasti
TORQUE-UNSAFE karena dinamika). 7 tuple lain STATIC ∧ POSE_A gagal di **garis pulang lurus** (garis-env 7, garis-kotak 2);
19 STATIC sisanya gagal di **REST di stop** (mis. arm_4 (1.00, +90): REST −12 mm ke peta, j2 statis cabang terbaik 2.59).
TAS-RAK z 1.40: 0 IK di seluruh grid → fisika (jangkauan).

### 0.2 Lipat parsial — [g64_diag.py fold](results/p1_g64/g64_diag.py), [g64_transit.py](results/p1_g64/g64_transit.py)

Alat diangkat h di atas posisi alat REST (alat-bawah, cabang j2 terendah), gravitasi statis (RNEA = statika terukur, 9.25 vs 9.60):

| h (m) | 0 (REST) | 0.10 | 0.15 | 0.20 | 0.25 | 0.30 | 0.35 | 0.40 |
|---|---|---|---|---|---|---|---|---|
| j2 statis mentah (N·m) | 0.10 | 1.47 | 1.98 | 2.36 | 2.63 | 2.80 | 2.81 | tanpa IK |
| j2 + 7.7 + 0.66 (sistem) | — | 9.83 | 10.34 | 10.72 | 10.99 | 11.16 | 11.17 | — |
| titik terendah lengan (m, AABB hull) | 0.924 | 1.027 | 1.077 | 1.127 | 1.177 | 1.227 | 1.277 | — |

Cabang j2-terendah memutar j1 ≈ 107°. Kandidat transit **T1** (`g64_transit_T.json`, dipilih dari 500 cabang tersaring torsi, h
0.35, prioritas titik terendah tinggi + radius kecil, antar-lengan ≥ 50 mm saat keduanya T dan saat partner REST): q6 =
[−0.046, 1.023, 2.503, −1.573, 1.480, 1.617], j2 statis **4.04**, titik terendah **1.277 m** (REST 0.924), radius sapuan dari
sumbu rotasi **0.462 m** (REST 0.456), antar-lengan 242 mm (keduanya T) / 442 mm (partner REST). Puncak transit REST→T belum
diukur (memori fold: RNEA meremehkan naik ×2.74 pada lipat penuh; sisa ukur−RNEA 0b p95 7.08 → prediksi ≤ 4.04 + 7.7 ≈ 11.7).

### 0.3 Kalibrasi ulang offset torsi (0b) — [g64_torque_pairs.py](results/p1_g64/g64_torque_pairs.py) (data HW nyata)

200 pasangan (rencana yang dieksekusi: RNEA j2 rencana vs puncak terukur): g18–g20 81 (G21 `torque_offsets`), G57 87 (G61 C3),
log tugas tunggal 32 (G32hw…G36, G51, CAM-6DOF, CALIB-CHARUCO, ARM1/RGBD2-ZONE, G63/G63b, DEMO-ZONE-HW). Log mock/smoke dibuang;
g19/g20 dari log dibuang (log dua lengan, G21 sudah memetakan rencana dengan benar).

| sisa ukur − RNEA j2 (N·m) | n | p50 | p95 | maks |
|---|---|---|---|---|
| arm_1 | 59 | 4.66 | 7.09 | 7.30 |
| arm_2 | 59 | 4.87 | 6.44 | 7.03 |
| arm_3 | 59 | 5.54 | 7.16 | 8.56 |
| arm_4 | 23 | 5.53 | 7.77 | 8.01 |
| semua | 200 | 5.02 | 7.08 | 8.56 |

- Ukur > RNEA + 7.7: **3/200** (s3 G60 ev 10 13.22 vs 12.91; s8 R90sel ev 10 12.57 vs 12.49; g32hw R10 ev 08 12.30 vs 11.44 =
  lengan **menabrak rak** — puncak kontak, bukan model). Offset 7.7 ≈ p98 → **bukan terlalu ketat**; DEMO-ZONE-HW (sisa 4.49) ada di
  ekor bawah.
- Offset terkalibrasi per lengan (p95) tidak membuka apa-apa yang berarti: TAS-RAK RNEA rencana yang ditolak 6.56–9.01 → dengan
  arm_4 p95 7.77 tetap 14.33–16.78 > 14 (0/14 tetap 0); dengan p95 semua 7.08 hanya 6.56 → 13.64 lolos (1 percobaan). G42: lebihan
  statis vs batas 5.64 ≥ +0.36, median +2.16; offset 7.08 menggeser batas +0.62 → **2/256** tuple HANYA-torsi terbuka.
- Temuan penting: penolakan TAS-RAK punya RNEA rencana **7.5–9.0**, sedangkan cabang statis terbaik oracle di stop yang sama
  **4.93** (arm_4 0.70/+85). Penolakan torsi itu sifat **cabang/jalur yang dipilih MoveIt** (tujuan pose), bukan batas → tuas =
  perencanaan (tujuan sendi dari cabang pilihan + transit), bukan offset.

### 0.4 Arti `effort` (0c) — sumber primer Kinova

- Driver: `kortex_driver/src/hardware_interface.cpp:835` `arm_efforts_[i] = feedback_.actuators(i).torque();`.
- **Kinova Gen3 lite User Guide** (Clearpath TDS2, sha256 `6b8f3c48…`), Table 17 Actuators: sensor = "Position, current (motor),
  voltage, temperature (motor and CPU)"; Table 21: `torque` = **"Approximate equivalent torque derived from motor current in N·m"**.
  Datasheet Gen3 lite (OP-04_2020-03-R02): sensor "Position, current, voltage, temperature, accelerometer and gyroscope"; low-level
  "Position, velocity, current". → **TERVERIFIKASI: Gen3 lite tidak punya sensor torsi sendi; `effort` = arus motor × konstanta**,
  termasuk rugi gearbox/gesekan (cocok: sisa +5…+8 saat bergerak, tahan diam ≈ RNEA).
- User guide Table 28: batas lunak torsi sendi **9.0 N·m** sendi 1–6, hanya di kendali **high-level angular trajectory**; sistem kita
  low-level (ros2_control) → tidak berlaku (terukur 13.22 tanpa fault, konsisten).
- Rating 14 N·m j2 (URDF Kinova `gen3_lite_macro.xacro`): dasar angkanya **tidak terverifikasi** (user guide Gen3 lite tidak memuat
  torsi nominal/puncak aktuator). Lembar KA-75+/KA-58 (AS-ACT-KA58-KA75-SP-INT-EN 201804-1.2): 12.0 N·m nominal (kontinu sampai
  rangka 75 °C), 30.5 puncak (batas perangkat lunak; header menulis 37), gradien 13.8 N·m/A "tanpa rugi gearing", efisiensi gear maks
  76 %, backdrive 1.7–5.2 N·m, konstanta termal lilitan 22 s / rangka 39 min, "torque loads above nominal should always be brief".
  **Tetapi** lembar itu mendaftar sensor torsi + pengendali joystick/CAN (generasi Jaco), sedangkan Gen3 lite tidak punya sensor
  torsi → **KA-75+ = aktuator Gen3 lite TIDAK TERVERIFIKASI**; angka 12/30.5 hanya indikatif.
- Konsekuensi: saat mengangkat melawan gravitasi, `effort` (sisi motor) ≥ torsi keluaran + gesekan. Bila 14 = rating keluaran, batas
  "terukur ≤ 14" lebih ketat dari rating sebesar gesekan (~2–5 N·m). Dasar 14 tidak terverifikasi → **keputusan operator** (0.6).

### 0.5 Batas bergantung waktu (0d) — [g64_torque_time.py](results/p1_g64/g64_torque_time.py) (29 CSV HW, 104 seri j2)

- Puncak arsip maks **14.27** (g20 arm_3; 17.61 G16 tidak punya CSV arsip). Total waktu > 14 di seluruh arsip **0.01 s**; > 16: 0.
- > 12: terpanjang **0.38 s** kontinu (g18 arm_2), total per sesi ≤ 2.6 s. Semua puncak > 12 = transien transit.
- Rata-rata bergerak maks: 1 s **12.48**, 5 s **11.93**, 30 s **10.63**; RMS filter orde-1 τ 22 s (proksi I²t) maks **10.57**.
- Fault tercatat: **0** di semua sesi (termasuk 17.61 sesaat G16).
- Spesifikasi durasi/duty cycle/I²t Gen3 lite dari Kinova: **tidak ditemukan** (user guide hanya payload kontinu 500 g / 600 g
  mid-range). Satu-satunya angka waktu = KA-75+ (tidak terverifikasi untuk lite): τ lilitan 22 s.

### 0.6 Usulan batas torsi (BELUM diputuskan — aturan lama berlaku: penyaring RNEA + 7.7 ≤ 14, AUTO-STOP > 14 terukur)

Data mendukung: puncak ≤ 14 transien ≤ 0.4 s, rata-rata 5 s ≤ 12, RMS τ22 ≤ 11 — sudah dipenuhi semua jalan. Data **tidak**
mendukung batas puncak > 14 (hanya 0.01 s di atas 14 sepanjang arsip). Opsi ke operator (tombol 2026-10-17): (a) tetap + pengawas
durasi (rata-rata 5 s ≤ 12, > 12 kontinu ≤ 1 s), (b) longgarkan puncak ≤ 15 selama ≤ 0.5 s, (c) uji bertahap HW nanti.
**Keputusan operator 2026-10-17: (c) uji bertahap HW nanti** — fase terpisah SESUDAH G64: transien terkendali 14 → 15 → 16 terukur,
izin per gerak, berhenti jauh sebelum fault (LED merah = reset fisik). **Sampai itu aturan lama berlaku** (RNEA + 7.7 ≤ 14,
AUTO-STOP > 14 terukur, `--tau-max 14`); G64 tidak mengubah batas torsi.

## 1. Desain (bahan) — what-if opsi A: [g64_whatif.py](results/p1_g64/g64_whatif.py)

Opsi A = ganti REST oleh T1 (kedua lengan gantry yang bergerak) saat traverse dan sebagai awal/akhir lengan tugas; penyaring sama.
Uji awal TAS-RAK z 1.34: arm_4 (0.70, +85) **A_OK** (sekarang: garis pulang lurus kena lemari), arm_4 (1.00, +90) **A_OK**
(sekarang: REST di stop −12 mm), arm_3 (0.50, −20) sekarang OK tetapi **gagal lewat T1** (torsi garis T1→q) → opsi A harus "REST
**atau** T per tugas" (gabungan) agar nol regresi. Hitungan penuh menunggu §0.1.

### 1.1 Risiko opsi B — pelacakan bridge gantry (HW G57, [g64_bridge_track.py](results/p1_g64/g64_bridge_track.py))

Perintah = kosinus `pose_to_g` direkonstruksi (start, goal, t_send, T_cmd dari JSON run); terukur = `/joint_states` G57.
52 gerak rel + 53 gerak rotasi:

| sumbu | galat maks \|ukur − perintah\| | p95 (median antar-gerak) | lag terbaik (median, rentang) | sisa RMS di lag terbaik | galat akhir |
|---|---|---|---|---|---|
| rel | **71.1 mm** | 44.5 mm | **1.33 s** (0.55–1.70) | 8–11 mm | ≤ 0.6 mm |
| rotasi | **13.4°** (65° dalam 11.3 s) | 2.8° | **1.05 s** (0.6–4.8) | ≤ 1.9° | ≤ 0.5° |

Bridge = waktu mati ~1–1.7 s + kejar stop-start. Rencana whole-body yang disaring pada posisi rel/rotasi PERINTAH akan dieksekusi
dengan lengan sampai **71 mm / 13°** (≈ 0.2 m di radius 1 m) dari tempat penyaring meletakkannya — melebihi margin 50 mm
(antar-lengan, env). Opsi B butuh model lag + margin tambahan atau gantry diam saat lengan bergerak → kembali ke bertahap (A).
Gerak berurutan (A) tidak terkena: posisi akhir bridge tepat ≤ 0.6 mm / 0.5°, dan lengan diam (T atau REST) selama traverse.

## 0.7 Model torsi per-sampel (prioritas operator 2026-10-17 siang: "perbaiki perhitungan torque dulu, demo besok")

Data: 29 CSV HW → [g64_tsid_extract.py](results/p1_g64/g64_tsid_extract.py) → **513 segmen gerak, 19 sesi** (g22/g30/g33/
g34hw2/g36park/g51-bringup2: 0 segmen lengan — tidak dipakai, dicatat). τ = RNEA penuh di q/qd/qdd TERUKUR (SG 0.2 s, 100 Hz).
Tanda `effort` Kortex = **−** RNEA (rasio diam median −0.86).

Sisa j2 per sampel (ukur − RNEA, arah beban) — [g64_tsid_peak.py](results/p1_g64/g64_tsid_peak.py):

| fase | n | p1 | p50 | p99 | p99.9 | maks |
|---|---|---|---|---|---|---|
| ANGKAT (searah beban) | 225 718 | +1.18 | **+3.56** | +6.30 | +7.19 | +10.58 |
| TURUN (dibantu gravitasi) | 433 250 | −5.19 | **−2.66** | +0.07 | +0.81 | +5.76 |
| DIAM (j2 \|qd\| ≤ 0.005) | 432 584 | −4.11 | +0.13 | +5.13 | +5.96 | +7.74 |

- Gesekan gearbox searah gerak (angkat +3.6, turun −2.7) + stiksi/**windup servo saat tahan** (contoh g18: diam, RNEA 1.78 konstan,
  effort 0.1 → 6.6 dalam 2 s) + derau sampel ±1.4. Model rata-rata (Coulomb/viskos/efisiensi, Dahl — [g64_tsid_fit.py](results/p1_g64/g64_tsid_fit.py))
  galat puncak p95 3.6–4.1 N·m → ditolak.
- Prediktor amplop fase `maks_t(|τ| + Q[fase])`, tinggalkan-satu-sesi (513 segmen):

| aturan | kurang-prediksi | lebih rata-rata | lebih rata-rata (puncak ≥ 8, n 202) |
|---|---|---|---|
| **sekarang \|RNEA\| + 7.7** | **4** | 4.07 | 2.41 |
| fase q99 | 62 | 2.15 | 0.79 |
| fase q99.9 | 13 | 3.01 | 1.67 |
| fase q100 | 1 | 5.83 | 4.81 |

**Kesimpulan 0.7:** rumus sekarang (RNEA rencana + 7.7) **benar dan aman** untuk HW ini; sisa ±3 N·m = sifat sensor arus
(stiksi/windup/derau), bukan salah hitung. Model per-fase hanya mengetatkan ≤ 0.7 N·m dengan kurang-prediksi 3× lebih banyak →
**TIDAK dipasang**. Data torsi tambahan tidak menurunkan derau ini → tidak diperlukan.
**Sumber ketidaksesuaian mock/HW yang sebenarnya = cabang IK:** penjadwal menilai torsi pada cabang statis pilihannya (TAS-RAK 4.93)
sedangkan probe memberi MoveIt tujuan POSE dan MoveIt memilih cabang/jalur sendiri (RNEA 7.5–9.0 → TORQUE-UNSAFE; DEMO-ZONE-HW plan-only
j2 61 % vs eksekusi 97 %). Perbaikan = tujuan SENDI dari cabang penjadwal (sudah ada: `--goal-joints`, G41/G42, run_g54).

## 0.8 Konsistensi penjadwal ↔ probe ↔ HW untuk tujuan sendi (87 tugas HW G57, data G61 C3)

| ukuran | nilai |
|---|---|
| RNEA rencana probe − gravitasi statis garis sendi lurus (predikat penjadwal e3), j2 | p50 −0.04, p95 +0.006, **maks +0.05** N·m (MARGIN 0.66) |
| garis lurus vs jalur terukur (gravitasi) | ≤ 0.012 N·m |
| lolos penjadwal (garis + 7.7 + 0.66 ≤ 14) / lolos probe (rencana + 7.7 ≤ 14) | 87 / 87, 0 beda |
| ukur > prediksi penjadwal / > prediksi probe | **0** / 2 (maks +0.31) |

→ Dengan tujuan SENDI dari cabang penjadwal, prediksi torsi penjadwal = rencana mock = HW. Jalur demo yang sah = solver P1″ →
`run_g54` (`run_g32.task_cmd` menambah `--goal-joints`). `demo_runner.py` mengirim `--target` (tujuan POSE) → cabang bebas → selisih.
Aturan ini ditulis ke `p1_demo_sys_prompt.md` (demo besok).

## 1.2 Banding desain A / B / C (operator 2026-10-17 sore: "bandingkan semuanya, mana terbaik dan teraman")

Data baru (luring, statis, penyaring SISTEM: kotak **10 mm** G41b — `g64_whatif.py` kini `G64_BOX_MIN`, versi pertama memakai > 0;
lihat `p1_demo_sys.md` §1.1 untuk artefak yang sama di g64_diag):

| ukuran | nilai | sumber |
|---|---|---|
| TAS z 1.34 lewat T1 (40 tuple statis j2 terendah) | **40/40** layak (sekarang 0); arm_4 g2 rel 0.80–1.30, rot +50…+95 (di dalam amplop HW g2), j2 mentah 0.52–1.70; stop env 0.197–0.237 m, traverse 0.155 m | `g64_whatif_box10.jsonl` |
| z 1.40 lewat T1 | 36/40 layak → titik itu BUKAN kontrol negatif (benar diganti (1.0, 0, 0.60)) | sama |
| regresi "selalu T1" pada 48 tugas jadwal P1″ terpilih (g55 R90sel, 8 seed) | **44/48**; gagal 4 — semua arm_3 g2 garis T→q: s8 (1.35, −5) garis-kotak, s10 (0.85, −20) garis-kotak, s12 (1.40, −5) torsi-garis + garis-kotak → seed utuh 5/8 | `g64_whatif_reg.json` |
| garis lipat REST → T1 (statis) | j2 maks 3.82 → +7.7 = 11.52 (+0.66 = 12.18) ≤ 14; sendi terjauh j3 136° | — |
| titik terendah lengan | T1 1.277 m vs REST 0.924 m (memori stow-pose-high: operator ingin idle TINGGI) | §0.2 |
| bridge saat gantry + lengan bersamaan | lag 1.33 s, galat s.d. **71 mm / 13°** > margin 50 mm | §1.1 |

| | **A** T1 manual per run | **B** whole-body 8-DOF | **C** mode per gantry di penjadwal |
|---|---|---|---|
| TAS tercapai | ya (40/40 statis) | tidak diukur (OMPL acak) | ya (mode T1) |
| regresi jadwal lama | paksa T1: 44/48 (3 seed rusak); pilih REST manual: 0 tetapi manusia memilih | tidak diukur | **0 menurut konstruksi** (mode REST = sistem sekarang, predikat bit-identik) |
| siapa memilih | operator / flag → melanggar demo-sys-uses-p1-scheduler | perencana | **penjadwal** (biaya lipat masuk makespan) |
| sinkron gantry–lengan | lengan diam saat traverse; akhir bridge ≤ 0.6 mm / 0.5° | ✗ 71 mm / 13° melampaui margin → butuh model lag | = A |
| torsi | penyaring sama (RNEA + 7.7); lipat statis 11.52 | MoveIt buta torsi → pasca-saring + replan pada jalur 8-DOF acak | = A |
| antar-lengan / env | penyaring sama (S18, G33 terukur, MARGIN = TOLAK) | MoveIt buta antar-lengan (SRDF 112/121 dimatikan) | = A |
| kerja | runner: event lipat/buka + pose_to_g S12 terima T1 tersertifikasi | besar (model lag, eksekusi gabungan) | A + solver: EdgeMatrix T1, pose (a) T1, tuple T1 (garis T1→q), biaya lipat |

**Rekomendasi: C** — teraman sama dengan A (penyaring identik, gantry dan lengan tidak bergerak bersamaan), terbaik karena (i) nol
regresi (REST tetap pilihan), (ii) sistem yang memilih lipat atau tidak (syarat demo), (iii) membuka TAS. **B ditolak**: satu-satunya
opsi dengan galat terukur di atas margin keselamatan, dan MoveIt buta torsi + antar-lengan. Urutan kerja C: bagian runner A dulu
(dibutuhkan C) → mode di solver → mock 77 (TAS + regresi seed lama + kontrol negatif) → HW mulai lipat di tempat.
**Risiko bersama A/C (belum diukur):** dinamika lipat REST → T1 (j3 136°; memori fold: RNEA meremehkan angkat ×2.74 pada lipat
penuh → prediksi ≤ 11.7 dengan sisa p95 7.08, wajib saring rencana + ukur di HW di tempat dulu); waktu lipat/buka (biaya makespan)
belum terukur; traverse antar-gantry dengan T1 tidak disaring (sama dengan sekarang; radius T1 0.462 vs REST 0.456).
