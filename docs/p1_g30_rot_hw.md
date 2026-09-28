# P1 / G30-HW — ROTASI gantry di sel NYATA, tahap (e): ±10°, lengan REST

> Sesi 2026-09-28, operator di lokasi. Sumber prompt: [p1_g29_rot.md §D](p1_g29_rot.md).
> Kode/hasil: `docs/results/p1_g30/`. **Tidak diubah:** `interarm_collision.py`, probe, `sched*`, alat G22,
> `joint_limits.yaml`, `dual_table_controller.py`.

---

## A0. Gerbang + instrumen (SEBELUM §A dikunci; nol gerak)

### A0.1 Gerbang (jawaban operator 2026-09-28, sebelum §A)

| # | Gerbang | Keadaan / jawaban |
|---|---|---|
| G1 | yaml ≤ URDF + G28-ON | yaml **tidak** diubah sejak `877a4fc`; G28-ON **belum** jalan. Operator: **tunda eksplisit** — (e) tidak merencanakan gerak lengan (JTC hanya sendi rotasi, lengan ditahan). Urutan G28 C tetap: yaml diperbaiki **sesudah** G28-ON, **sebelum** eksekusi rencana lengan berikutnya. |
| G2 | G29 C-1 (hull palsu) / C-2 (probe tanpa rotasi) | Operator: **tidak dipatch** sesi ini. `rot_to_g` memakai `g29_rot_screen` (hull sejati). **Tidak ada** rencana lengan pada rot ≠ 0 (alat tidak punya jalur itu). |
| G3a | kecepatan (g29 B5) | **10 °/s (bridge 1000 pulse/s), hanya lengan REST** (ujung ≤ 77 mm/s). Lengan-di-tugas ditunda ke G31. |
| G3b | batas fisik | Operator: **±10° bebas** henti mekanis / kabel di kedua gantry; **encoder 0 = batang sejajar rel**. |

Lain: disk **2.3 GB** bebas (prompt ~3). `ros2_kortex` di `e712295` (fix prefix ter-commit; memori "stash" basi).
Tidak ada proses ROS berjalan. URDF `/tmp/reach_dwell_live.urdf` sha `f02e7c53…` = G23.

### A0.2 Sapuan model ([g30_a0.py](results/p1_g30/g30_a0.py) → [log](results/p1_g30/g30_a0.log))

`sweep_rot(rect)`, `RotCrossChecker` hull sejati, rel 0/0, lengan REST, gantry lain rot 0: **8 kaki + ±10.5° keduanya
CLEAR**, min **380.0 mm** = SS `t1_platform_link_0 ↔ t2_platform_link_0` (platform tidak berputar → konstan);
lengan min 493.1 mm (±10°), 489.7 (±10.5°). `tool_frame` REST r_h **407.5 mm** → busur 10° = 71.1 mm.

### A0.3 Mekanisme bridge (baca kode, `_on_hw_command` + `moving_table.go_to_absolute`)

- Kirim ulang bila setpoint JTC bergeser ≥ `rot_tol_deg` **0.3°** (atau lin ≥ 1.0 mm) dari target terakhir **dan** tidak ada
  thread berjalan; target **absolut** (lin + rot bersama — rel diperintah ke setpoint tahan JTC).
- `go_to_absolute`: **lewati tanpa gerak** bila ketiga motor ≤ **50 pulsa** (rot 100 pulsa/° → **0.5°**; lin 0.5 mm); thread
  kembali saat ≤ 50 pulsa; motor tetap jalan ke target terakhir.
- ⇒ ujung kaki berhenti **kurang** (searah gerak): batas kasus-terburuk < 0.3 + 0.5 = **0.8°**. Sim kasar
  ([g30_bridge_sim.py](results/p1_g30/g30_bridge_sim.py) → [log](results/p1_g30/g30_bridge_sim.log), overhead 0.05–0.3 s,
  callback 10–20 ms): galat **0 … −0.21°**, t_rot **2.30–2.71 s**.
- Arsip linear (G24b/G26, 14 traverse): galat selalu kurang, maks 1.006 mm (batas 1.5); t_traverse ≈ 0.96·T_cmd;
  drift lengan 0.04–0.16°; torsi puncak 0.69–2.63 N·m.
- Skala 100 pulsa/° = konstanta kode (`9000 pulses = 90 degrees`); G3 mengukur 10.000 °/s **dengan encoder yang sama** —
  sudut fisik per pulsa **tidak pernah** diukur independen.

---

## A. Protokol — DIKUNCI sebelum `rot_to_g.py` ditulis, sebelum gerak

> 🔒 §B menang atas §A; pertentangan DITULIS.

### A1. 🔒 Alat `rot_to_g.py` (baru; pola `rail_to_g` + `rot_home`)

`python3 rot_to_g.py --gantry G GOAL_DEG [--move]`, DRY tanpa `--move`. Menolak (rc 1) bila:
- **S12** lengan gantry G maks |q − REST| ≥ 0.5° (lengan gantry lain dilaporkan);
- **S23′** |GOAL| > 10.0°; |rot terukur G| > 10.5°; **rot gantry lain** |·| > 0.5° (satu gantry bergerak);
- **S28** `sweep_rot(rect)` dari keadaan **terukur** (24 sendi lengan, kedua rel, **kedua rotasi** dari `/joint_states`, nama
  ketat — hilang = tolak) `(lin, rot) → (lin, GOAL)` bukan **CLEAR** (margin 50 mm), atau penyaring tidak bisa jalan.

Trajektori: cosinus, **hanya** `t{G}_rotation_joint`, `gantry_{G}_with_arm_controller`,
`T_cmd = max(3.0, π·D/(2·0.9·10 °/s))` (pola `t_cmd` g22), `STEPS = max(10, 2·T_cmd)`, kecepatan titik 0 (= rail_to_g).
Ukur (rekaman `/joint_states` dalam proses): akhir = encoder diam (< 0.01° selama 1.0 s) sesudah bergerak > 0.05°, batas 30 s.
Keluaran JSON: `err_deg`, `t_rot` (gerak pertama > 0.05° → perubahan terakhir), `T_cmd`, `T_rot_model = 0.26 + D/10`,
drift lengan (keempat), drift rel sendiri + rel lain (mm), drift rot lain (°), torsi puncak lengan (keempat, jendela gerak),
`sweep_min_mm` + pasangan, `jtc_error_code`. **rc 3** bila |err| > 1.0°, drift lengan > 0.5°, rel mana pun > 2 mm,
rot lain > 0.5°, atau torsi > 14 N·m.

### A2. 🔒 Urutan + palang

| tahap | isi | izin |
|---|---|---|
| 0 | operator: LED 4 lengan tidak merah, sel/ruang di bawah & antar-gantry kosong, origin; `remount_check` + ICMP .10–.13; bring-up `use_fake_hardware:=false enable_gantry_bridge:=true use_sim_time:=false` (SIG_DFL, cek SigIgn); 4× "Actuator count … '6'", 7/7 controller, nol fault; rel & rotasi terbaca; perekam `js_record` | operator |
| 1 | `rot_to_g` DRY: g1 +10, g2 +10 (nol gerak) | — |
| 2 | **g1**: 0 → +10 → 0 → −10 → 0, satu kaki per perintah, operator melihat tiap kaki | operator per gantry |
| 3 | **g2**: sama | operator |

Berhenti (tanya operator) bila: rc ≠ 0 kaki mana pun; fault / Kortex exception / `NOT armed` / `REJECTED` di log launch;
torsi > 14; `ros2_control_node` / perekam mati; rel bergerak > 2 mm; gerak arah salah (operator). Kaki berikut dibuat dari
keadaan **terukur** (bukan target nominal). Akhir tiap gantry: |rot| ≤ 1.0° (bila > 0.5°, S23 `rail_to_g` akan menolak —
dicatat, tidak dipaksa). Rel **tidak** digerakkan. Disk ≥ 1.5 GB sebelum bring-up.

### A3. 🔒 Kriteria sukses (e)

8/8 kaki: S28 CLEAR, JTC 0, |err| ≤ 1.0°, drift lengan < 0.5°, rel < 2 mm, rot lain < 0.5°, torsi < 14, nol fault;
operator melihat arah tiap kaki — **dicatat apa yang dilihat** (tanda + tidak diasumsikan).

### A4. 🔒 Dugaan D157–D166 (ditulis SEBELUM alat dan data)

Prior tally G29: kode sendiri meleset; besaran geometri tanpa hitung meleset; mekanisme terukur tepat.

| D | dugaan | dasar |
|---|---|---|
| D157 | `rot_to_g` DRY pertama **gagal** (≥ 1 galat kode sendiri sebelum DRY bersih) | prior: kode sendiri meleset berulang (G29 D152/D153) |
| D158 | S28 min 8 kaki **= 380.0 mm** (SS platform), lengan min ≥ 485 mm | A0.2 dihitung; keadaan terukur ≈ model |
| D159 | galat akhir **|err| ≤ 0.3°** dan **kurang/0 searah gerak** pada 8/8 | A0.3 kode + sim (≤ 0.21) + arsip linear (selalu kurang) |
| D160 | t_rot ∈ **[2.0, 3.2] s** 8/8 (≈ T_cmd 3.0), rasio t_rot / T_rot_model(1.26) ≥ 1.6 | bridge mengejar setpoint; linear 0.96·T_cmd |
| D161 | drift rel sendiri ≤ 0.1 mm, rel lain 0.000 mm | rel ditahan pada encoder saat aktivasi; deadband 0.5 mm |
| D162 | drift lengan (keempat) ≤ 0.2° 8/8 | arsip traverse 0.04–0.16° |
| D163 | torsi puncak lengan ≤ 3.0 N·m 8/8 | arsip traverse ≤ 2.63; rotasi 10°/s pendek |
| D164 | nol `NOT armed` / `REJECTED` / fault sepanjang sesi | G26 nol; target ±10 ≪ ±180 |
| D165 | rot terukur di encoder sesudah kaki kembali-ke-0: |rot| ≤ 0.3° (tidak menumpuk antar-kaki) | kaki dibuat dari keadaan terukur |
| D166 | (bila operator mengukur) tali-busur `tool_frame` 0 → 10° = **71 ± 10 mm** (skala 100 pulsa/° benar fisik) | A0.2 r_h 407.5 → 2·r·sin 5° = 71.0 |

---

## B. Hasil terukur

> §A dikunci **2026-09-28 10:15:44**, sha256 `ead94705247d…` ([sectionA_locked.md](results/p1_g30/sectionA_locked.md)).
> Gerak 10:30–10:40. Rekaman: [g30_joint_states.csv.gz](results/p1_g30/g30_joint_states.csv.gz) (perekam `js_record`),
> [g30_launch.log.gz](results/p1_g30/g30_launch.log.gz), per kaki `g30_g{1,2}_leg{1..4}.log` (+ `.launch` = irisan log),
> ringkasan [g30_summary.py](results/p1_g30/g30_summary.py) → [log](results/p1_g30/g30_summary.log), [json](results/p1_g30/g30_summary.json).

### B0. Sebelum gerak

| langkah | hasil |
|---|---|
| DRY palsu (`/joint_states` sintetis, domain 77, tanpa stack) | 7/7 sesuai: CLEAR g1 +10 / g2 −10 / g1 10→0; TOLAK target 11°, S12 (arm_1 1°), S23′ (rot lain 1°); g2 terima walau arm_1 1° (S12 hanya gantry sendiri) ([log](results/p1_g30/g30_dry_fake.log)) |
| smoke MOCK (`use_fake_hardware`, domain 77) — **bukan data** | `return_rest` mock → REST; g1 4 kaki `--move` rc 0, JSON lengkap ([log](results/p1_g30/g30_mock_smoke.log)). Stack mock dimatikan: node 0; crash dump `move_group` 195 MB (milik sesi) dihapus |
| `remount_check` + ICMP .10–.13, `/dev/ttyUSB0/1` | **GERBANG LULUS**; 4/4 ✅ |
| bring-up nyata (`/tmp/g30_t1.log`, launch PID **1325570**, SigIgn `0x1001000`) | **4×** "Actuator count … '6'"; **7/7** controller active; bridge **table1/table2 ARMED 0.7 mm / 0.0°**; octomap 4 updater gagal (= memori, lama); 10 "process has died" + 1 "Failed to configure" spawner gantry_2 (kelas spawner ganda g20; controller tetap active) |
| keadaan | rel **0.712 / 0.681 mm**; rot **+0.000 / +0.020°**; lengan maks \|q − REST\| arm_1 0.239, arm_2 0.335, arm_3 0.425, **arm_4 0.556°**; `/joint_states` 2 publisher (memori) |
| DRY nyata ([log](results/p1_g30/g30_dry_real.log)) | g1 +10 **CLEAR 380.0 mm** (SS platform), lengan 493.1; g2 **TOLAK S12** (arm_4 0.556°) |
| `return_rest` arm_3 + arm_4 (izin operator, sebelum g2) | DRY CLEAR (se-gantry 621.0, antar-gantry 575.1 — **penyaring lama, hull palsu**; rot 0/0, gerak < 0.6°); `--move` galat akhir 0.039°, **torsi puncak 4.466 N·m `t2_a1_joint_2`** (gerak 0.56°!); DRY g2 ulang CLEAR |

### B1. Delapan kaki — **8/8 lulus kriteria A3**

| kaki | mulai → target | akhir (encoder) | galat | t_rot | /T_rot | drift lengan | rel sendiri / lain | rot lain | torsi puncak | S28 min / lengan | kirim (lewati) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g1-1 | +0.00 → +10 | +9.84 | −0.16 | 2.564 | 2.03 | 0.083° | 0.000 / 0.000 mm | 0.000° | 1.18 `t1_a2_j3` | 380.0 / 493.1 | 7 (1) |
| g1-2 | +9.84 → 0 | +0.00 | 0.00 | 2.871 | 2.31 | 0.100 | 0 / 0 | 0 | 1.63 `t1_a1_j2` | 380.0 / 494.2 | 8 (1) |
| g1-3 | 0.00 → −10 | −9.80 | +0.20 | 2.769 | 2.20 | 0.086 | 0 / 0 | 0 | 1.26 `t1_a2_j3` | 380.0 / 493.1 | 8 (1) |
| g1-4 | −9.80 → 0 | +0.00 | 0.00 | 2.680 | 2.16 | 0.045 | 0 / 0 | 0 | 0.78 `t1_a2_j3` | 380.0 / 494.5 | 7 (1) |
| g2-1 | +0.02 → +10 | +9.92 | −0.08 | 2.835 | 2.25 | 0.103 | 0 / 0 | 0 | **2.56** `t2_a2_j2` | 380.0 / 493.1 | 8 (1) |
| g2-2 | +9.92 → 0 | +0.17 | +0.17 | 2.423 | 1.94 | 0.086 | 0 / 0 | 0 | 1.63 `t2_a1_j2` | 380.0 / 493.6 | 7 (1) |
| g2-3 | +0.17 → −10 | −9.76 | +0.24 | 2.772 | 2.17 | 0.077 | 0 / 0 | 0 | 1.65 `t2_a1_j2` | 380.0 / 493.1 | 7 (1) |
| g2-4 | −9.76 → 0 | **−0.29** | −0.29 | 2.799 | 2.26 | 0.086 | 0 / 0 | 0 | 1.88 `t2_a1_j2` | 380.0 / 494.7 | 7 (1) |

- JTC `error_code` 0 ×8; rc 0 ×8; nol `NOT armed` / `REJECTED` / fault / Kortex exception di log penuh; perekam hidup sepanjang.
- **Galat selalu kurang/0 searah gerak** (8/8), maks \|galat\| **0.29°** — mekanisme A0.3 (debounce 0.3° + deadband 0.5°):
  tiap kaki kirim pertama (0.3°) **dilewati** "Already at absolute target" (1/kaki, tepat seperti dibaca dari kode); kirim terakhir
  berhenti < 0.3° dari setpoint akhir. Kaki berikut dibuat dari keadaan terukur → galat tidak menumpuk.
- **Waktu:** t_rot 2.42–2.87 s = **0.81–0.96·T_cmd** (3.0 s, lantai `max(3.0, …)`), **1.94–2.31× T_rot model** (1.24–1.28 s).
  Sama dengan linear (bridge mengikuti durasi perintah, bukan motor). Sim A0.3 2.30–2.71 s (sedikit di bawah terukur).
- **Rel tidak bergerak** (0.000 mm keduanya, 8/8) walau tiap kirim bridge memerintah motor linear ke setpoint tahan (68 pulsa =
  encoder → tidak ada selisih).
- **Arah (operator, dilihat dari bawah):** g1 +10 → arm_1 bergerak **searah jarum jam** = URDF (sumbu `t{g}_rotation_joint` dunia
  **+z**; +rot = CCW dari atas = CW dari bawah) ✅.
- **Skala fisik (D166, operator, pita ukur):** tali-busur ujung gripper arm_1 0 → +9.84° ≈ **70 mm** vs 2·407.5·sin(4.92°) = 69.9 ✅
  — pertama kali sudut fisik per pulsa (100/°) diperiksa independen dari encoder (resolusi pita ± beberapa mm ≈ ± 0.5°).
- Keselarasan encoder 0 ↔ batang sejajar rel: operator menjawab "sejajar" di gerbang G3b (visual, tidak diukur angka).

### B2. Pertentangan §B lawan §A

| # | Pertentangan |
|---|---|
| (1) | A2 tahap 3 mengandaikan g2 langsung; S12 menolak (arm_4 0.556°) → `return_rest` arm_3/arm_4 disisipkan (izin operator). `return_rest` memakai `CrossGantryChecker` **lama** (hull palsu, G29 C-1, tidak dipatch) — rot 0/0 dan gerak < 0.6°, verdict tidak mungkin berubah; dinyatakan. Torsi puncak 4.47 N·m pada gerak 0.56° — tidak dikejar. |
| (2) | Smoke MOCK dan DRY sintetis tidak ada di A2 (ditambah sebelum HW untuk menguji jalur `--move`); bukan data. |
| (3) | PID launch yang disimpan pertama (1325558) **salah** (pgrep menangkap proses sementara); `kill -INT` ke PID itu "No such process", launch asli 1325570 ditemukan lalu di-SIGINT. Jebakan PID sama kelasnya dengan memori `ros2-launch-pid-leak`. |
| (4) | Disk 2.3 GB (A2: ≥ 1.5 ✅; prompt "~3"). |

### B3. Papan skor D157–D166

| D | dugaan | hasil | nilai |
|---|---|---|---|
| D157 | DRY pertama gagal (galat kode sendiri) | DRY sintetis pertama 7/7 bersih; smoke mock 4/4; HW 8/8 | ❌ meleset (kode sendiri **benar** jalan pertama) |
| D158 | S28 min = 380.0 mm, lengan ≥ 485 | 380.0 ×8; lengan 493.1–494.7 | ✅ tepat |
| D159 | \|err\| ≤ 0.3°, kurang/0 8/8 | maks 0.29°, kurang/0 8/8 | ✅ tepat (tipis) |
| D160 | t_rot ∈ [2.0, 3.2], rasio ≥ 1.6 | 2.42–2.87 s; 1.94–2.31 | ✅ tepat |
| D161 | rel sendiri ≤ 0.1 mm, lain 0.000 | 0.000 / 0.000 | ✅ tepat |
| D162 | drift lengan ≤ 0.2° | maks 0.103° | ✅ tepat |
| D163 | torsi ≤ 3.0 N·m | maks 2.56 | ✅ tepat |
| D164 | nol NOT armed / REJECTED / fault | 0 (spawner ganda = kelas lama, bukan fault) | ✅ tepat |
| D165 | kembali-ke-0 \|rot\| ≤ 0.3° | 0.00, 0.00, 0.17, 0.29 | ✅ tepat (tipis) |
| D166 | tali-busur 71 ± 10 mm | ≈ 70 mm | ✅ tepat |

**G30: 1 meleset / 9 tepat.** Semua yang tepat diturunkan dari **jalur kode yang dibaca** (A0.3: debounce/deadband → galat
kurang, lewati kirim pertama; lantai T_cmd → waktu) atau **dihitung** (A0.2 sapuan, r_h). Satu-satunya meleset adalah prior
"kode sendiri gagal jalan pertama" — kali ini alat disalin dari pola teruji (`rail_to_g`) dan diuji sintetis + mock sebelum HW.

---

## C. Keadaan akhir (Rule 12)

- **Stack dimatikan:** SIGINT ke launch 1325570 → keluar 12 s (launch meng-eskalasi SIGTERM/SIGKILL ke `ros2_control_node`,
  `move_group`, `rviz2` — pola lama); perekam SIGINT; `ros2 node list --no-daemon` **0**; nol proses sisa. Crash dump `rviz2`
  10:41 (166 MB) + `move_group` mock 10:23 (195 MB), keduanya milik sesi, **dihapus**; dump python 09-21 (bukan milik sesi) dibiarkan.
  Disk **2.2 GB**.
- **Keadaan sel akhir:** rel g1/g2 ≈ 0.7 mm (tidak digerakkan); rot g1 **0.00°**, g2 **−0.29°** (≤ 0.5 → S23 `rail_to_g` lolos);
  arm_3/arm_4 di REST (0.04°), arm_1/arm_2 seperti bring-up (0.24/0.34°).
- **Diubah:** tidak ada berkas lama. **Baru:** `docs/results/p1_g30/` (`rot_to_g.py`, A0, ringkasan, log). **Tidak diubah:**
  `interarm_collision.py`, probe, `sched*`, alat G22, `joint_limits.yaml`, `dual_table_controller.py`.
- **Gerbang yang masih terbuka (dibawa ke G31):** G28-ON belum jalan; `joint_limits.yaml` > URDF (perbaikan sesudah G28-ON,
  sebelum eksekusi rencana lengan); G29 C-1 (hull palsu, `return_rest` + `rail_to_g` + probe masih memakainya) dan C-2 (probe S18
  tanpa rotasi) **tidak dipatch** — wajib sebelum rencana lengan pada rot ≠ 0.
- **Tidak diukur:** rotasi > 10°; rotasi dengan lengan di pose tugas; rotasi serentak dengan rel; kedua gantry berputar bersama;
  keselarasan encoder 0 dengan angka (hanya visual); henti mekanis / kabel di luar ±10° (operator: bebas dalam ±10°).
- **Model waktu untuk scheduler:** via bridge, rotasi pendek = **T_cmd** (lantai 3.0 s), bukan `T_rot` motor. Untuk 180°,
  `T_cmd = π·180/(2·0.9·10) = 31.4 s` vs `T_rot` 18.26 s (1.72×) — belum diukur; linear punya pola yang sama (G24b/G26).
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi HW multi-langkah; dilaporkan.

---

## D. Prompt G31 (salin ke chat BARU)

**Rekomendasi: Opus, effort TINGGI** — patch penyaring bersama (C-1/C-2) yang dipakai semua sesi HW sebelumnya; galat di sini
diam dan satu arah (tidak aman). Sisi offline, tetapi keputusannya mengizinkan gerak lengan pada rot ≠ 0.

```
Sesi G31 -- JADWAL DENGAN ROTASI: penyaring rencana berrotasi + amplop B3, OFFLINE/PLAN-ONLY dulu. Repo ceiling_arm,
branch feat/rgbd-topo-deploy.

BACA PENUH: CLAUDE.md; docs/p1_g30_rot_hw.md (semua); docs/p1_g29_rot.md (B1, B3, B4, C); docs/p1_g28_hw.md (B0b, C, D);
scripts/interarm_collision.py; scripts/reach_dwell_probe.py (_plan_and_screen ~555); docs/results/p1_g29/g29_rot_screen.py;
docs/results/p1_g30/rot_to_g.py; sched.py (T_rot, sched_coll arah-pendek G29 A0.3).

GERBANG (tanya operator SEBELUM apa pun):
 G1. Urutan: G28-ON (V28 plan-only) dulu, atau G31 dulu? yaml <= URDF tetap sesudah G28-ON, sebelum eksekusi rencana lengan.
 G2. Izin patch C-1 (true_hull ke CrossGantryChecker, atau ganti pemakai ke RotCrossChecker) dan C-2 (probe S18 meneruskan
     KEDUA sendi rotasi, nama ketat) -- keduanya WAJIB sebelum rencana lengan pada rot != 0. Tanpa izin: berhenti di offline.
 G3. Model waktu rotasi di scheduler: T_rot motor (0.26 + d/10) atau T_cmd bridge (G30: 1.9-2.3x untuk 10 deg)?

1. KUNCI §A: kontrol negatif (patch tidak mengubah verdict/jarak pada rot 0 selain koreksi hull -- ukur dan tulis
   selisihnya), kontrol positif (probe S18 pada rot != 0 menangkap tabrakan yang rot=0 lewatkan), dugaan D167+.
   Prior tally G30: kode yang disalin dari pola teruji + diuji sintetis/mock -> tepat; mekanisme dari kode dibaca -> tepat.
2. Patch C-1/C-2 (bila diizinkan) + uji (G29 N1/N2/P2 ulang dengan pemakai baru).
3. Scheduler: pose (lin, rot) dengan amplop B3 lengan REST (|dx|>=0.8 bebas; satu gantry <=10 deg -> lain bebas;
   keduanya <=35 deg) sebagai kendala traverse; S18 berrotasi per rencana tugas. Plan-only pada seed (iv) G26/G28.
4. Laporan §B/§C, p1_state + tally, prompt G32 (HW: satu jadwal dengan rotasi, lengan REST saat berputar).
ATURAN: nol gerak kecuali operator mengizinkan eksplisit; rotasi hanya dengan lengan REST (G30 G3a); B menang atas A
dan DITULIS; mock/HW dimatikan bersih (PID launch ASLI, bukan $! / pgrep sementara); disk ~2 GB.
```
