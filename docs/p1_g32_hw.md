# P1 / G32-HW — EKSEKUSI seed 36 di sel NYATA: R0 lalu R10 (rotasi g2 −10°, lengan REST saat berputar)

> Sesi 2026-10-07, operator di lokasi. Sumber prompt: [p1_g32_rot_exec.md §D](p1_g32_rot_exec.md).
> Kode/hasil: `docs/results/p1_g32hw/`. Alat dipakai **apa adanya**: `docs/results/p1_g32/{pose_to_g,run_g32}.py`,
> `scripts/return_rest.py` (patch A2 G32, `c08c95f`), `docs/results/p1_g31/g31_screen.py`.

---

## A0. Gerbang (jawaban operator 2026-10-07, SEBELUM §A)

| # | Gerbang | Jawaban |
|---|---|---|
| H1 | tahap 0: LED 4/4 tidak merah; origin g1/g2 = home fisik; rel + ruang antar-gantry bebas s.d. ~1.6 m; g2 bebas henti/kabel di −10° pada rel 0.6 m | **Ya, semua aman** |
| H2 | pembanding R0 seed 36 sama-sesi | **Ya, R0 dulu lalu R10** |
| H3 | kaki uji `pose_to_g` g2 (0,0) → (0.60,−10) → (0,0), lengan REST, sebelum jadwal penuh | **Ya** |

Keadaan sebelum §A (baca saja): nol proses ROS; disk 57 GB; `enp112s0` 192.168.2.100/24 UP; `/dev/ttyUSB0/1` ada.

### A0.1 Mekanisme yang dibaca / dihitung (sebelum §A)

- **Arsip bridge G26** (18 traverse rel, `g26_launch.log.gz`): kirim **1.30–1.55 /s** (siklus ~0.75 s = Modbus, bukan debounce);
  550 mm → 38 kirim / 29.3 s; 1450 mm → 105 kirim / 78.7 s. ⇒ kirim ≈ 1.33·t_traverse.
- **Goal lin + rot bersama** (`_on_hw_command`): kirim ulang bila lin ≥ 1.0 mm **atau** rot ≥ 0.3° dari target terakhir. Pada
  g2 0 → (0.60, −10) rot sebanding lin: 1 mm ≙ 0.0167° ≪ 0.3° ⇒ **lin memicu semua kirim**, rot ikut sebagai penumpang.
  Ujung: kirim terakhir ≤ 1 mm sebelum setpoint akhir ⇒ galat rot ≤ ~0.017° (+ resolusi encoder 0.01°), kurang searah gerak.
  Kirim pertama: lin 1 mm = ~95 pulsa > 50 ⇒ **tidak** dilewati "Already at absolute target" (beda dengan G30 rot-saja: 1/kaki).
- Puncak laju setpoint rot = π/2·10°/33.33 s = 0.47 °/s ≪ motor 10 °/s; lin puncak 28.3 mm/s < motor 31.4 mm/s.
- **Event** (`g31_screen.events`, p0 0/0): R0 dan R10 masing-masing **14** (4 retract, 4 traverse, 6 tugas). R0 g2 stop 0 =
  (0, 0) → tugas t1 tanpa traverse; R0 traverse g2 (0→0.20, 0.20→1.45); R10 traverse g2 (0→(0.60,−10), →(1.45,0)).
- **P1′-serial** (G32-OFF B7 (2)): R0 **581.26 s**, R10 **530.39 s** (Δ −50.87 s, −8.75 %). Bias G26 terukur/model
  ×**1.004 – 1.135** (seed 2 … 1).

---

## A. Protokol — DIKUNCI sebelum bring-up, sebelum gerak

> 🔒 §B menang atas §A; pertentangan DITULIS.

### A1. 🔒 Urutan

| tahap | isi | berhenti bila |
|---|---|---|
| 0 | `remount_check.py` + ICMP .10–.13; bring-up `my_workcell.launch.py use_fake_hardware:=false enable_gantry_bridge:=true use_sim_time:=false` via `SIG_DFL…execvp` + `setsid nohup`, log `/tmp/g32hw_t1.log`; PID launch ASLI di-resolve ulang sesudah controller naik, SigIgn dicek; 4× "Actuator count … '6'", 7/7 controller, bridge table1/table2 ARMED; perekam `js_record.py` + monitor `reach_dwell_monitor` (csv `/tmp/g32hw_step`) | gerbang gagal / fault |
| 1 | plan-only **NYATA** `g31_screen --seeds 36 --variant R10` lalu `--variant R0`, `--repeats 3`, plan `g32_candidates.json`, lintasan disimpan | bukan LOLOS 3/3 → tanya operator |
| 2 | `run_g32 --dry` R0 dan R10 dari keadaan terukur | REFUSE / A3 gagal |
| 3 | (H3) `pose_to_g` DRY lalu `--move` g2 (0,0)→(0.60,−10), lalu (0.60,−10)→(0,0) (`--variant R10`; p0 sah); kaki ke-2 dibuat dari keadaan terukur; operator melihat tiap kaki | rc ≠ 0; fault / `REJECTED` / `NOT armed`; arah salah |
| 4 | `run_g32 --seed 36 --variant R0 --move` (setsid nohup) | auto-stop runner (S26) |
| 5 | pulang: `return_rest` g1, g2 bila perlu; `pose_to_g` g1, g2 → (0, 0) | rc ≠ 0 |
| 6 | `run_g32 --seed 36 --variant R10 --move` | auto-stop runner |
| 7 | pulang seperti 5; matikan stack bersih (SIGINT PID launch ASLI, `ros2 node list --no-daemon` = 0) | — |

Aturan: rotasi hanya dengan lengan REST (S12 `pose_to_g`); satu gantry bergerak per waktu (runner serial; tahap 3/5 satu
perintah per waktu); **tidak** `move_dual_table` selama ARMED; rc 134 runner saat keluar = benign (G32-OFF B7 (5)).

### A2. 🔒 Besaran yang dilaporkan (terlepas dari hasil)

- Tahap 3 per kaki: `err_mm`, `err_deg`, `t_lin`, `t_rot`, `t_traverse` vs `T_cmd`, drift lengan, rel/rot lain, torsi puncak,
  `sweep_min_mm`, jumlah kirim bridge (+ "Already at absolute target") di log launch.
- Tahap 4/6: makespan terukur vs P1′-serial (rasio), Δ(R10 − R0) terukur vs −50.87 s, sukses penilai x/6, per traverse
  galat lin/rot + t_traverse/T_cmd, torsi puncak per tugas, drift, halt/auto-stop.

### A3. 🔒 Dugaan D200–D212 (SEBELUM bring-up dan data)

Prior tally G32-OFF / G30: mekanisme kode dibaca → tepat; besaran perilaku baru tanpa hitung → meleset; alat disalin dari
pola teruji + diuji mock → jalan pertama.

| D | dugaan | dasar |
|---|---|---|
| D200 | bring-up: 4× Actuator '6', 7/7 controller, ARMED keduanya, nol fault runner | G26/G30 |
| D201 | plan-only NYATA R10 **LOLOS 3/3** (18 PLANNED), traverse min **380.0 mm** (SS platform) | mock G32-OFF 3/3, 380.0 konstan |
| D202 | plan-only NYATA R0 **LOLOS 3/3** | G31 R0 = jadwal rot 0 biasa, tugas sama |
| D203 | kaki uji 1 g2 (0,0)→(0.60,−10): `T_cmd` **33.33** (lin); `err_mm` ∈ **[−1.1, 0]**, `err_deg` ∈ **[0, +0.05]** (kurang searah gerak) | A0.1 lin memicu semua kirim |
| D204 | kaki uji 1: `t_lin` ∈ **[29.5, 33.0] s** (0.88–0.99·T_cmd); `t_rot / t_lin` ∈ **[0.85, 1.01]** (rot ikut tiap kirim, bukan 3 s) | arsip 0.96·T_cmd; rot penumpang |
| D205 | kaki uji 1: kirim bridge table2 **40 ± 8**, "Already at absolute target" **0** | 1.33/s × ~30 s; 95 pulsa > 50 |
| D206 | kaki uji 2 (0.60,−10)→(0,0): galat cermin D203 (`err_mm` ∈ [0, +1.1], `err_deg` ∈ [−0.05, 0]); \|rot\| akhir ≤ 0.05° | sama |
| D207 | kedua kaki: drift lengan ≤ **0.2°**, torsi puncak ≤ **3.0 N·m**, rel/rot g1 0.000 | G30 B1 / arsip |
| D208 | R0 6/6 sukses penilai, nol auto-stop | G26 18/18 |
| D209 | R10 6/6 sukses penilai, nol auto-stop — termasuk 3 tugas lengan pada g2 −10° | plan-only + mock |
| D210 | makespan / P1′-serial ∈ **[1.00, 1.14]** untuk R0 **dan** R10 | bias G26 |
| D211 | **Δ(R10 − R0) terukur ∈ [−80, −20] s** (tanda sama dengan model −50.87) | struktur event sama (14/14), bias sama-sesi sebagian batal |
| D212 | traverse R10 g2 berotasi (ev 7 dan ev 12): \|err_deg\| ≤ 0.05° keduanya | D203 |

---

## B. Hasil terukur

> §A dikunci **2026-10-07 10:24:36**, sha256 `3c0fd0bfb6e7…` ([sectionA_locked.md](results/p1_g32hw/sectionA_locked.md)).
> Log launch stack 1: [g32hw_launch1.log.gz](results/p1_g32hw/g32hw_launch1.log.gz); perekam: `results/p1_g32hw/g32hw_joint_states{1,2}.csv.gz` (52 + 6.6 MB, **lokal saja, tidak di-commit**).

### B0. Sebelum gerak

| langkah | hasil |
|---|---|
| `remount_check` + ICMP .10–.13 | **GERBANG LULUS**, 4/4 |
| bring-up (launch PID **1976439**, SigIgn `0x1001001`) | 4× Actuator '6'; **7/7** active; bridge table1 ARMED 0.7 mm/0.0°, **table2 0.7 mm/−0.9°**; 7 spawner "process has died" (baseline) |
| keadaan | rel 0.712 / 0.681 mm; rot 0.000 / **−0.940°**; lengan maks \|q − REST\| arm_1 0.466, arm_2 0.235, **arm_3 14.716, arm_4 10.644°** (j4/j5; G30 meninggalkan 0.04°). Operator: **diketahui / sengaja** |
| `return_rest` arm_3+arm_4 (izin operator) | CLEAR (se 622.7, antar 552.7); galat 0.050°, torsi puncak 4.466 `t2_a2_joint_2` |
| `return_rest` arm_1+arm_2 | **self-skip** (0.47° < 0.5; alat tanpa opsi paksa) → arm_1 tetap 0.466° |
| plan-only NYATA ([log](results/p1_g32hw/g32hw_screen.log)) | **R10 LOLOS 3/3**, **R0 LOLOS 3/3**, 36/36 PLANNED, traverse min 380.0 mm; lintasan disimpan |
| `run_g32 --dry` R0, R10 | 14 event masing-masing; A3 0.466°; S26 lolos (rot g2 −0.94 vs batas 1.0 — tipis) |

### B1. Kaki uji `pose_to_g` g2 (H3) — 2/2 rc 0

| kaki | mulai → target | galat | t_lin / t_rot / T_cmd | drift lengan | torsi | kirim (lewati) | fault |
|---|---|---|---|---|---|---|---|
| 1 | (0.7 mm, −0.94°) → (0.60, −10) | **−0.43 mm / +0.010°** | 31.823 / 30.816 / 33.30 | 0.2935° | 2.107 `t2_a2_j2` | 44 (**1**) | 0 |
| 2 | (599.6 mm, −9.99°) → (0, 0) | **+0.49 mm / 0.000°** | 31.833 / 31.122 / 33.31 | 0.195° | 1.535 `t2_a1_j2` | 44 (0) | 0 |

- Goal JTC lin + rot pertama di bridge nyata: kedua sumbu satu target absolut; rot ikut tiap kirim lin (t_rot/t_lin 0.968 / 0.978),
  galat rot ≤ 0.010° (= resolusi encoder). Mekanisme A0.1 tepat.
- Lewati kaki 1 = kirim pertama di posisi awal (0.7 mm/−0.9°): target ARMED tersimpan ≠ setpoint tahan JTC sesudahnya (≥ tol).
  Kaki 2: 0 lewati ⇒ artefak sekali sesudah ARMED, bukan per kaki. (G30 "1 lewati/kaki" mungkin kelas yang sama — tidak diperiksa.)
- Arah (operator, dari bawah, kepala di arm_4): −10° = **CCW**, arm_4 ke kiri (+y, arah g1) ≈ 69 mm = TF ✅.

### B2. R0 seed 36 — **6/6 SUKSES, makespan 647.47 s** = **1.114 ×** P1′-serial 581.26 ([console](results/p1_g32hw/g32hw_R0_console.log), [run.json](results/p1_g32hw/g32hw_R0_run.json))

| ev | isi | galat | t / T_cmd | torsi puncak (lengan mana pun) |
|---|---|---|---|---|
| 1 | g1 0 → 0.90 | −0.05 mm | 0.974 | 1.33 |
| 4 | g1 0.90 → 1.35 | −0.46 mm | 0.948 | 2.52 |
| 8 | g2 0 → 0.20 | −0.98 mm | 0.937 | 6.51 `t1_a1_j2` |
| 11 | g2 0.20 → 1.45 | −0.27 mm | 0.974 | 6.62 `t1_a1_j2` |

- Tugas: sukses @ 111.2 / 250.7 / 302.7 / 410.3 / 594.6 / 647.5 s; torsi puncak keseluruhan **10.90** `t2_a1_joint_2` (dalam tugas).
- Torsi traverse g2 6.5 N·m = **arm_1 diam di pose tugas t3** pada g1 (tidak ada retract g1 sesudah t3) — torsi tahan statis, bukan traverse.
- 14/14 rc 0; nol fault; runner rc 134 (benign).
- Pulang (tahap 5): `return_rest` g2 (galat 0.069°, torsi 5.06) + g1 (0.079°, 6.48); `pose_to_g` g1 1.35→0 (+0.68 mm, 72.6/74.97 s),
  g2 1.45→0 (+0.15 mm, 78.7/80.54 s); semua CLEAR, nol fault.

### B3. R10 seed 36 — 🔴 **BERHENTI di ev 8: arm_3 MENABRAK RAK** ([console](results/p1_g32hw/g32hw_R10_console.log), [ev08](results/p1_g32hw/g32hw_R10_ev08_task.log))

| ev | isi | hasil |
|---|---|---|
| 0–5 | g1: retract (skip), 0→0.90 (−0.99 mm), t4 arm_1 ✅ @113.1, retract, 0.90→1.35 (−0.13 mm), t3 arm_1 ✅ @249.9 | sama dengan R0 (±2 s) |
| 6 | retract g2 | skip (REST) |
| **7** | **g2 (0, 0) → (0.60, −10)** dalam jadwal | −0.36 mm / **+0.010°**, 31.92/33.33 s, drift 0.19°, torsi 4.02 |
| **8** | **t0 arm_3 → (0.7857, −0.4588, 1.00) @ g2 (0.60, −10)** | **TORQUE-ABORT 12.30 N.m `t2_a1_joint_2`** |

Kronologi (log launch + perekam):
- 432.49 eksekusi mulai (plan-only 3/3 PLANNED; antar-lengan CLEAR 492.7 mm; RNEA prediksi j2 11.44/14 = 82 %).
- ~436–437.7 torsi j2 naik 5 → 12.30 N·m; **437.70 `Kortex exception: WRONG_SERVOING_MODE`** (S26 menangkapnya sebagai fault baru).
- 446.65 probe "moveit MOVED" lalu TORQUE-ABORT (puncak > `--tau-max 12`); runner **AUTO-STOP sebelum ev 9**.
- **Operator: "arm 3 menabrak rak."** Sesudahnya: arm_3 bebas, LED tidak merah, tidak ada kerusakan terlihat, 3 lengan lain normal.
- ~497 `INVALID_USER_SESSION_ACCESS` membanjir (2449 baris); **`ros2_control_node` mati (exit −13)**.

**Penyebab (inferensi, belum diverifikasi geometri):** sel **tidak punya penyaring lingkungan sama sekali** — 4 octomap updater
`move_group` gagal dimuat tiap bring-up, tidak ada node kamera, probe tidak menambah CollisionObject (memori 🔴 sejak G26).
Rak tidak ada di model mana pun. Pada −10° pangkal arm_3 bergeser ~69 mm ke −y (menjauh dari g1); t0 (y −0.459) di R0 dikerjakan
dari rel 0.20 rot 0 **tanpa** kontak. Penyaring yang ada (S18 antar-lengan, S28 sapuan, torsi RNEA) semuanya lolos dengan benar
— mereka tidak bisa melihat rak. Batas torsi probe (12) dan Kortex adalah satu-satunya yang bereaksi.

**Tidak dapat dinilai:** makespan R10, Δ(R10 − R0), traverse ev 12 berotasi, tugas pada g2 −10° (0/3 sukses, 1 tabrakan).

### B4. Papan skor D200–D212

| D | dugaan | terukur | |
|---|---|---|---|
| D200 | bring-up bersih | 4×6, 7/7, ARMED (g2 −0.9°) | ✅ |
| D201 | R10 plan-only NYATA 3/3, 380.0 | 3/3, 18/18, 380.0 | ✅ |
| D202 | R0 3/3 | 3/3 | ✅ |
| D203 | kaki 1 T_cmd 33.33, err_mm [−1.1, 0], err_deg [0, +0.05] | 33.30, −0.43, +0.010 | ✅ |
| D204 | t_lin [29.5, 33.0], t_rot/t_lin [0.85, 1.01] | 31.823, 0.968 | ✅ |
| D205 | kirim 40 ± 8, lewati 0 | 44, **1** | ❌ (pemicu salah baca: target ARMED, bukan lin 1 mm) |
| D206 | kaki 2 cermin, \|rot\| akhir ≤ 0.05 | +0.49 mm, 0.000° | ✅ |
| D207 | drift ≤ 0.2°, torsi ≤ 3.0, g1 0 | **0.2935** / 0.195; 2.11 / 1.54; 0 | ❌ |
| D208 | R0 6/6 | 6/6 | ✅ |
| D209 | R10 6/6 termasuk 3 tugas di −10° | **2/6, tabrakan rak di tugas pertama pada −10°** | ❌ |
| D210 | makespan/P1′-serial ∈ [1.00, 1.14] | R0 **1.114**; R10 — | ✅ (R0 saja) |
| D211 | Δ(R10 − R0) ∈ [−80, −20] s | — | tidak dinilai |
| D212 | \|err_deg\| ≤ 0.05 di ev 7 dan ev 12 | ev 7 +0.010; ev 12 — | ✅ (ev 7 saja) |

**9 ✅ / 3 ❌ / 1 tidak dinilai.** Meleset D209 bukan galat dugaan besaran: tidak ada dugaan (dan tidak ada penyaring) yang memodelkan
lingkungan. Pola tetap: mekanisme kode dibaca → tepat (galat rot, waktu, kirim); pemicu yang tidak dibaca (target ARMED) → meleset.

### B5. Pertentangan §B lawan §A / prompt

| # | Pertentangan |
|---|---|
| (1) | `return_rest` arm_3/arm_4 disisipkan sebelum tahap 1 (lengan 14.7° dari REST; izin operator). arm_1 0.466° tidak bisa dipaksa (self-skip). |
| (2) | Tahap 6 (R10) **tidak selesai**: auto-stop ev 9 sesudah tabrakan rak di ev 8. Tahap 7 (pulang + matikan) **tidak dilakukan oleh PC**: `ros2_control_node` mati; stack 1 di-SIGINT bersih; **bring-up ulang ditolak pengklasifikasi izin** — keputusan ke operator. |
| (3) | 🔴 Plan-only NYATA "LOLOS 3/3" **bukan** bukti aman di sel ini: tidak ada geometri lingkungan di penyaring mana pun. Semua G22–G32 "CLEAR/LOLOS" = robot-vs-robot + torsi saja. |
| (4) | Memori G6 "awan titik tersimpan, tidak perlu kamera lagi" **salah**: `/tmp/topo_{static,cloud}_*.npz` sudah hilang. |

## C. Keadaan akhir (Rule 12)

- **Stack 1 dimatikan:** SIGINT launch 1976439 → keluar 12 s; perekam + monitor SIGINT; `ros2 node list --no-daemon` **0**.
  `ros2_control_node` sudah mati sendiri (exit −13) sesudah tabrakan.
- **Keadaan sel (terakhir terekam, ~91 s sebelum mati):** g1 rel **1.350 m**, rot 0.000°; g2 rel **0.600 m**, rot **−9.99°**;
  arm_1 di pose tugas t3 (jauh dari REST); arm_2 REST; **arm_3 di pose pasca-tabrakan** (j1 +69.8°, j4 +67.1° dari REST; operator:
  bebas dari rak, LED tidak merah); arm_4 REST. **Lengan tidak di REST, gantry tidak di home.**
- **Belum dipulangkan:** bring-up stack 2 ditolak izin → operator harus memutuskan (bring-up manual / izin / pulang manual Kinova).
  ⚠ Pulang otomatis pun **tanpa** penyaring lingkungan: `return_rest` arm_3 dari pose dekat rak tidak tahu rak ada.
- **Diubah:** tidak ada kode. **Baru:** `docs/p1_g32_hw.md`, `docs/results/p1_g32hw/` (skrip `g32hw_screen.sh`, `g32hw_run.sh`, log, run.json).
- **Tidak diukur:** makespan R10, Δ R10 − R0, ev 12, tugas pada rot ≠ 0 tanpa kontak.
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi HW multi-langkah; dilaporkan.

### C1. Pemulangan (sesudah §C awal, 2026-10-07; izin operator: bring-up oleh operator, PC memulangkan)

- Stack 2 dinyalakan **operator** (launch PID 1996488, SigIgn `0x1001001`, log [g32hw_launch2.log.gz](results/p1_g32hw/g32hw_launch2.log.gz)):
  4× '6', 7/7 active; table2 ARMED 599.6 mm/−10.0°; **table1 NOT armed** sekali (perintah awal 0.0 mm vs encoder 1349.9 mm = penjaga
  slam-to-zero), lalu **ARMED pada goal pertama `gantry_1_with_arm_controller`** (`return_rest` arm_1) tanpa gerak rel.
- Lintasan pulang arm_3 diperiksa dulu: pose sekarang 24.4° dari pose tabrakan; garis sendi ke REST lewat ≥ 19.5° (sendi maks)
  dari konfigurasi tabrakan (s = 0.24) — tidak nol, rak tidak dimodelkan → operator di e-stop.
- `return_rest` arm_3+arm_4: galat 0.033°, torsi 5.04 `t2_a1_j2`; arm_1+arm_2: 0.059°, 5.96 `t1_a1_j2`; `pose_to_g` g2 (0.60, −10) → (0, 0):
  +0.36 mm / 0.000°, 31.99 s; g1 1.35 → 0: +0.76 mm, 72.58 s. Semua rc 0, nol fault, tidak ada kontak dilaporkan.
- **Keadaan akhir: keempat lengan REST, g1 0.76 mm / 0°, g2 0.36 mm / 0°.** Stack 2 SIGINT → 12 s; `node list --no-daemon` 0;
  crash dump sesi (`move_group` 11:20, `rviz2` 11:40, `ros2` 10:52) dihapus. Disk 56 GB.

---

## D. Prompt G33 (salin ke chat BARU) — PETA 3D LINGKUNGAN dulu, NOL gerak sampai aktif

**Rekomendasi: Opus, effort TINGGI** — penyaring yang hilang di sini gagal **diam** (semua "CLEAR/LOLOS" G22–G32 buta lingkungan)
dan baru saja menghasilkan tabrakan nyata; verifikasi harus membuktikan rak *terlihat* oleh perencana, bukan sekadar plugin termuat.

```
Sesi G33 -- PETA 3D LINGKUNGAN: penyaring tabrakan lengan-vs-lingkungan AKTIF sebelum gerak lengan apa pun. Repo ceiling_arm,
branch feat/rgbd-topo-deploy. Pemicu: G32-HW arm_3 MENABRAK RAK (t0 di g2 (0.60,-10), 12.30 N.m, Kortex WRONG_SERVOING_MODE,
ros2_control_node mati). Operator memutuskan: peta 3D dulu, baru uji gerak lagi.

BACA PENUH: CLAUDE.md; docs/p1_g32_hw.md (B3, B5, C); docs/p1_g6_map.md; ros2_ws/src/reachability_gng/README_TOPO.md;
ros2_ws/src/workcell_moveit_config/config/sensors_3d.yaml + launch/my_workcell.launch.py (bagian move_group/sensors);
scripts/reach_dwell_probe.py (_plan_and_screen); log docs/results/p1_g32hw/g32hw_launch1.log.gz (grep octomap).

GERBANG (tanya operator SEBELUM apa pun):
 K0. Sel dipulangkan di akhir G32-HW (C1: 4 lengan REST, rel 0/0, rot 0/0). Ada yang berubah sejak itu?
 K1. Kedua D455 terhubung + rak dan benda tetap lain di sel TERLIHAT kamera (tidak tertutup)? Ruang kosong dari orang saat tangkap?
 K2. (opsional) ukuran pita rak -- HANYA pembanding untuk peta kamera, bukan penyaring.
KEPUTUSAN OPERATOR (2026-10-07): JALUR KAMERA = jalur utama dan permanen (octomap/peta dari 2x D455); objek kolisi dari ukuran
pita TIDAK dibangun sebagai penyaring. Urutan: perbaiki plugin octomap -> node kamera di bring-up -> peta statis beku -> octomap hidup.
FAKTA (operator 2026-10-07): 2x D455 TETAP di pojok atas sel; world->camera sudah dikalibrasi (memori rgbd-extrinsic-calibration,
<5 cm antar-kamera) dan dicek di RViz -> TF kamera statis; hanya TF ROBOT yang perlu sinkron waktu.
SELF-FILTER (kamera melihat lengan/gantry sendiri) -- rencana yang disetujui untuk dikunci di §A:
 - Peta STATIS dulu: tangkap sekali, keempat lengan REST, gantry home, robot DIAM; buang titik di dalam mesh link robot
   (4 lengan, 4 gripper, 2 platform) pada TF di cap waktu awan, padding >= 5 cm (= ketidakpastian antar-kamera); bekukan
   sisanya jadi objek kolisi statis (rak dll). Lengan tidak ada di peta beku -> tidak bisa "menabrak bayangan sendiri".
   Daerah tertutup lengan: tangkapan ke-2 dengan gantry di posisi lain (bukan tambal manual); daerah buta kamera DILAPORKAN.
 - Nama/TF ketat: link hilang/basi -> TOLAK awan (memori gng-collision: self-filter no-op diam saat TF hilang); jangan filter
   lengan yang dicopot (memori G6: lubang di lengan hantu).
 - ⚠ Kotak platform_link URDF (0.35x0.34 m) != pelat carriage nyata (memori kalibrasi) -> sisa pelat bisa jadi "penghalang";
   periksa sisa titik dekat platform, perbesar bentuk/padding platform bila perlu (DITULIS).
 - Kontrol self-filter: (N) titik peta dalam 5 cm dari mesh robot = 0, di REST dan >= 2 pose tugas, dan MoveIt tanpa
   "start state in collision"; (P) filter MATI -> titik lengan MUNCUL (bukti filter benar-benar membuang).
 - Octomap hidup (benda berpindah) dari kamera yang sama = tahap berikut sesudah peta statis lulus kontrol; arah jangka panjang.
POSE SIMPAN (operator 2026-10-07): REST menggantung lengan sampai z ~0.96 m -> ruang rumah yang sempit terasa penuh dan risiko
tabrakan dengan PENGHUNI/benda besar. Operator ingin pose default lengan TINGGI. Angka URDF (frame, arm_3; permukaan mesh ~5 cm lebih rendah):
   REST      terendah z 0.956  jorok horiz 0.094 m  tau_g j2 0.10
   candle    terendah z 0.949  jorok 0.070          j2 0.93
   home      terendah z 1.390  jorok 0.413          j2 2.78
   packaging terendah z 1.702  jorok 0.303          j2 4.09   <- kandidat paling ringkas ke plafon
 Evaluasi OFFLINE (sesudah peta 3D ada), kriteria dikunci di §A: (1) titik terendah MESH vs tinggi kepala; (2) sapuan saat
 traverse + rotasi vs gantry lain + peta (amplop G29 B3 dihitung ulang); (3) se-gantry/antar-lengan; (4) torsi tahan;
 (5) waktu simpan<->tugas (model P1). REST tertanam di return_rest.py DAN g22_plan.py (+ S12, A3 runner) -> satu sumber
 dulu, nama pose ketat. Ganti default HANYA sesudah evaluasi + plan-only; JANGAN di sesi yang sama dengan peta 3D bila waktu sempit.
1. Diagnosa (nol gerak): kenapa 4 octomap updater gagal ("According to the loaded plugin descriptions the class
   occupancy_map_monitor/PointCloudOctomapUpdater ..."): paket moveit_ros_perception terpasang? nama kelas? dari mana
   'default_sensor' / 'kinect_depthimage' termuat (bukan dari sensors_3d.yaml kita?).
2. KUNCI §A (dugaan D213+): kontrol POSITIF wajib = rencana G32 ev 8 (t0 arm_3 -> (0.7857,-0.4588,1.00) @ g2 (0.60,-10),
   lintasan tersimpan g32hw_screen_plans_R10.jsonl.gz) HARUS ditolak/menabrak-di-model setelah rak masuk; kontrol NEGATIF =
   R0 seed 36 (6/6 sukses nyata tanpa kontak) tetap PLANNED. Penyaring yang tidak menolak ev 8 = GAGAL, apa pun log-nya.
3. Pasang penyaring lingkungan di jalur yang dipakai HW: move_group planning scene (dilihat probe/MoveIt) DAN penyaring
   skrip (return_rest, pose_to_g sapuan, g31_screen) -- nama/objek ketat, hilang = TOLAK (pola nama ketat G32).
4. Plan-only di stack nyata/mock: ulang (iv) seed 36 R0 + R10; laporkan berapa rencana G22-G32 lama yang kini ditolak.
5. Laporan §B/§C, p1_state + tally, prompt G34 (HW ulang R10 hanya bila kontrol positif lulus) + rekomendasi model/effort.
ATURAN: NOL gerak lengan sampai K0 beres dan kontrol positif lulus; mock/HW dimatikan bersih (PID launch ASLI, node list
--no-daemon = 0); B menang atas A dan DITULIS; job > 10 menit via setsid nohup.
```
