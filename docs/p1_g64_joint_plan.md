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
mendukung batas puncak > 14 (hanya 0.01 s di atas 14 sepanjang arsip). Usulan ke operator: lihat tombol (sesi 2026-10-17).

## 1. Desain (bahan) — what-if opsi A: [g64_whatif.py](results/p1_g64/g64_whatif.py)

Opsi A = ganti REST oleh T1 (kedua lengan gantry yang bergerak) saat traverse dan sebagai awal/akhir lengan tugas; penyaring sama.
Uji awal TAS-RAK z 1.34: arm_4 (0.70, +85) **A_OK** (sekarang: garis pulang lurus kena lemari), arm_4 (1.00, +90) **A_OK**
(sekarang: REST di stop −12 mm), arm_3 (0.50, −20) sekarang OK tetapi **gagal lewat T1** (torsi garis T1→q) → opsi A harus "REST
**atau** T per tugas" (gabungan) agar nol regresi. Hitungan penuh menunggu §0.1.
