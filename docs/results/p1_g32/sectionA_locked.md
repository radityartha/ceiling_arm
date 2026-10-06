## A0. Gerbang (jawaban operator 2026-10-06, SEBELUM §A)

| # | Gerbang | Jawaban |
|---|---|---|
| G1 | patch `return_rest`: held += kedua rotasi, nama ketat, hilang = TOLAK | **IZIN** |
| G2 | rentang rotasi | **R10** (\|rot\| ≤ 10°, terverifikasi HW G30) |
| G3 | alat pose | **satu trajektori JTC lin + rot bersama** (bridge target absolut) — model `max(T_lin, T_rot)` |
| G4 | seed | **36** (G31 R10 −14 %) |
| — | stack plan-only (prompt: NYATA) | robot mati → **MOCK** (`use_fake_hardware:=true enable_gantry_bridge:=false`, `ROS_DOMAIN_ID=77`) — pertentangan dengan prompt, ditulis; plan-only NYATA pindah ke G32-HW |

### A0.1 Mekanisme yang dibaca (kode, sebelum §A)

- **Bridge** (`dual_table_controller._on_hw_command`): per gantry, kirim ulang bila setpoint JTC bergeser ≥ 1.0 mm **atau** ≥ 0.3°
  dari target terakhir **dan** tidak ada thread; satu `go_to_absolute(lin_mm, rot_deg)` untuk **ketiga** motor sekaligus;
  thread kembali hanya saat **ketiga** motor ≤ 50 pulsa. ⇒ JTC lin + rot dalam satu goal = satu urutan perintah absolut
  gabungan; sumbu lambat menentukan kapan thread kembali. Satu JTC sah untuk G3; tidak ada jalur kode baru di bridge.
- **`return_rest.py:150`** `held` = lengan gantry lain + rel lain, **tanpa rotasi**; `CrossGantryChecker.q_from` mulai dari
  `neutral` → rotasi dinilai 0 diam-diam. Penyaring se-gantry: kedua lengan pada platform yang sama → invarian terhadap rotasi
  gantry sendiri (tidak perlu rotasi). Model ketiga checker mengenal `t1/t2_linear_joint`, `t1/t2_rotation_joint` (diperiksa).
- **`run_g22.py`**: `s26` menolak \|rot\| > 0.5° (keras), traverse = `rail_to_g` (S23 menolak \|rot\| > 0.5°), event dari
  `g22_plan.events` (rel saja). `g31_screen.events` sudah sadar rotasi (traverse bila rel **atau** rot berubah).
- **G31 seed 36 R10** (`g31_candidates.json`): g1 (0.90, 0) → **(1.35, −10)**; g2 **(0.60, −10)** {0, 1, 5} → (1.45, 0) {2};
  makespan 311.478875. Rotasi g1 −10 berbarengan dengan rel 0.45 m (T_lin mendominasi) → dugaan **seri** (pemutus seri DP bebas).

---

## A. Protokol — DIKUNCI sebelum patch, alat, dan data

> 🔒 §B menang atas §A; pertentangan DITULIS.

### A1. 🔒 Jadwal ulang seed 36 R10 + pemutus seri rot 0 (`g32_sched.py`)

Instance = `g31_sched.build(row 36, R10)` apa adanya (P1′ + `rot_cmd 0.9235`, oracle‴ malas-eksak, cache G31 dipakai ulang).
Pemutus seri = biaya leksikografis: `sched.traverse_matrix` dibungkus **di skrip** (sched.py tidak diubah) menambah
`ε = 1e-6 s` untuk setiap pindah **ke** pose rot ≠ 0 (juga `T0`). Ketaksamaan segitiga tetap (tujuan berotasi selalu
membayar ε di kedua sisi). Makespan sejati dihitung ulang tanpa ε dari jadwal (traverse + durasi stop, per gantry).
Lulus bila: makespan sejati = R10 tanpa ε (|Δ| ≤ 1e-9) dan Σε < selisih biaya bukan-nol terkecil (dinyatakan).
Keluaran `g32_candidates.json` (format G31; varian `R10` = jadwal ber-ε, `R10_noeps`, `R0`).

### A2. 🔒 Patch `return_rest.py` (G1)

`held += ['t1_rotation_joint', 't2_rotation_joint']` (keduanya, selalu) → masuk ke keadaan antar-gantry; hilang dari
`/joint_states` → penolakan `missing` yang sudah ada (rc 1). Nama ketat: setiap nama di `names + [rail] + held` (antar)
dan `names + [rail] + partner` (se-gantry) harus `model.existJointName` di checker masing-masing — tidak → 🔴 TOLAK rc 1.
Tidak ada perubahan lain (interpolasi, 30 s, penjaga torsi).

### A3. 🔒 Alat `pose_to_g.py` (baru; gabungan `rail_to_g` + `rot_to_g`, G3)

`python3 pose_to_g.py --gantry G --seed S --plan P LIN ROT_DEG [--move]`, DRY tanpa `--move`. Menolak (rc 1) bila:
- **S12** lengan gantry G maks \|q − REST\| ≥ 0.5° (lengan gantry lain dilaporkan);
- **S13″** (LIN, ROT) bukan pose jadwal gantry G seed S (+ p0 (0, 0)); LIN ∉ [0, 1.600];
- **S23″** \|ROT\| > 10.0°; \|rot terukur G\| > 10.5°; \|rot gantry lain\| > 10.5° (R10; amplop B3 G29 "keduanya ≤ 35°, lengan
  REST" dipenuhi dengan sisa);
- **S28** `sweep_rot(RotCrossChecker, rect)` dari keadaan **terukur** (24 sendi lengan, kedua rel, kedua rotasi, nama ketat —
  hilang = tolak) `(lin, rot) → (LIN, ROT)` bukan **CLEAR** (margin 50 mm), atau penyaring tidak bisa jalan.

Trajektori: **satu** goal `[t{G}_linear_joint, t{G}_rotation_joint]`, `gantry_{G}_with_arm_controller`, kedua sumbu dari
**terukur** ke target jadwal, cosinus, `T_cmd = max(t_cmd_lin(Δlin), t_cmd_rot(Δrot))` (`g22_plan.t_cmd`, rumus `rot_to_g`;
sumbu tanpa gerak = 0), `STEPS = max(10, 2·T_cmd)`, kecepatan titik 0. Ukur: akhir = kedua encoder diam (lin < 0.05 mm,
rot < 0.01° selama 1.0 s) sesudah gerak (lin > 0.5 mm atau rot > 0.05°), batas 30 s. JSON: `err_mm`, `err_deg`,
`t_traverse` (gerak pertama sumbu mana pun → perubahan terakhir sumbu mana pun), `t_lin`, `t_rot`, `T_cmd`, `T_cmd_lin`,
`T_cmd_rot`, drift lengan (keempat), rel lain, rot lain, torsi puncak lengan (jendela gerak), `sweep_*`, `jtc_error_code`.
**rc 3** bila \|err_mm\| > 2.0, \|err_deg\| > 1.0, drift lengan > 0.5°, rel lain > 2 mm, rot lain > 0.5°, torsi > 14 N·m.

### A4. 🔒 Runner `run_g32.py` (salinan `run_g22`)

Perubahan saja: event = `g31_screen.events(schedule R10, p0)` (traverse bila rel **atau** rot berubah); traverse =
`pose_to_g --move`; retract = `return_rest` terpatch; `s26`: rel g = diharapkan ± 2 mm **dan** rot g = diharapkan ± 1.0°
(ganti "rot bukan 0"); diharapkan diperbarui dari target jadwal sesudah traverse rc 0. Lain (`FAULT_RE`, monitor/perekam/
`ros2_control_node` hidup, torsi > 14, basi, A3 awal keempat lengan REST, HALTED dicatat lanjut) apa adanya.
Rotasi hanya dengan lengan REST: dijamin S12 `pose_to_g` + urutan retract → traverse; satu gantry bergerak per waktu: runner serial.

### A5. 🔒 Plan-only MOCK + DRY + smoke

1. `g31_screen.py --seeds 36 --variant R10 --plan g32_candidates.json` k-of-k 3, lintasan disimpan. **Data plan-only (mock).**
2. `run_g32 --dry` di mock (perekam + monitor hidup) → daftar event + prasyarat.
3. `pose_to_g` DRY per traverse jadwal dari keadaan mock yang sesuai — lewat smoke (4).
4. **Smoke MOCK `run_g32` penuh `--move`** — `use_fake_hardware` (sendi palsu mengikuti JTC sempurna; bridge mati).
   **BUKAN data**: tidak dinilai, waktu/galat tidak dipakai; hanya bukti alat jalan ujung-ke-ujung.

### A6. 🔒 Kontrol

| id | jenis | isi | lulus bila |
|---|---|---|---|
| K-S0 | negatif | `g32_sched` tanpa ε vs `g31_candidates` R10 seed 36 | makespan + jadwal bit-identik |
| K-S1 | lulus | makespan sejati ber-ε vs tanpa ε | \|Δ\| ≤ 1e-9 |
| K-RR0 | negatif | `return_rest` DRY, `/joint_states` sintetis rot 0/0 (fake_js, domain 77, tanpa stack), lengan g2 +1.0°: patch vs `git show HEAD` | baris penyaring bit-identik |
| K-RRp | positif | sama, rot1 = −10°, rot2 = −10° (rel 0.9 / 0.6) | antar-gantry min **berbeda** dari pra-patch |
| K-RRm | positif | publisher tanpa `t1_rotation_joint` | TOLAK rc 1 |
| K-RRn | positif | nama sendi tak dikenal disuntikkan ke `held` (uji unit, monkeypatch) | TOLAK rc 1 |
| K-P | DRY sintetis | `pose_to_g` pada fake_js: (a) g2 (0,0)→(0.60,−10) CLEAR; (b) g1 (0.90,0)→(1.35,0) CLEAR; (c) lengan g2 +1° → S12; (d) target bukan jadwal → S13″; (e) ROT 15 → S23″; (f) tanpa rotasi di `/joint_states` → tolak | 6/6 sesuai |
| K-E | event | `run_g32` event list vs `g31_screen.events` (fungsi sama) | identik |

### A7. 🔒 Dugaan D189–D199 (SEBELUM patch, alat, data)

Prior tally G31: mekanisme kode dibaca → tepat; besaran perilaku baru tanpa hitung → meleset; kode sendiri → sering gagal jalan pertama.

| D | dugaan | dasar |
|---|---|---|
| D189 | jadwal ber-ε: g1 **tanpa rotasi** ((0.90, 0), (1.35, 0)); g2 tetap (0.60, −10) → (1.45, 0); makespan sejati **311.478875** | A0.1: rotasi g1 berbarengan rel 0.45 m → seri |
| D190 | K-S0 bit-identik | instance + cache sama |
| D191 | lazy: **1** iterasi (tuple rot ≠ 0 terpakai sudah di cache G31) | cache 23 500 tuple; tetangga terdekat diverifikasi |
| D192 | K-RR0 bit-identik | rot 0 = `neutral` |
| D193 | K-RRp antar-gantry berubah \|Δ\| ≥ **1 mm** | base bergeser 0.4·sin 10° ≈ 69 mm |
| D194 | K-RRm, K-RRn TOLAK | kode `missing` + `existJointName` |
| D195 | K-P 6/6 — tetapi **≥ 1 galat kode sendiri** sebelum DRY bersih (pola D157 G30) | prior |
| D196 | plan-only mock seed 36 R10 **LOLOS 3/3**, 18 PLANNED; traverse min ≥ **300 mm** | G31 seed 36 R35 (g1 −10 ⊃) LOLOS; R10-ε lebih sedikit rotasi |
| D197 | event: **14** (4 retract — 2 dilewati di REST awal, 4 traverse, 6 tugas); traverse berotasi **2** (g2 0 → −10, −10 → 0) | A1 + `events` |
| D198 | `pose_to_g` g2 (0,0)→(0.60,−10): T_cmd = T_cmd_lin = **33.33 s** (rot 3.0 s); P1′ traverse = 0.9235·… tak relevan, rel `rail_cmd` mendominasi | rumus |
| D199 | smoke mock penuh `--move` 6/6 tugas, rc 0 (sesudah perbaikan galat kode sendiri bila ada) | mock mengikuti JTC |

Aturan: NOL gerak fisik; mock `ROS_DOMAIN_ID=77`, dimatikan bersih (PID launch ASLI, `ros2 node list --no-daemon` = 0);
crash dump milik sesi dihapus; job > 10 menit via `setsid nohup`.

---

