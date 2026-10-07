# G32-HW §A dikunci 2026-10-07 10:24:36

sha256 `3c0fd0bfb6e7008144e8464093d59875c4a27bd835dbc2d3cee3b20d5265d07e` dari docs/p1_g32_hw.md (A0 + A)

```
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
```
