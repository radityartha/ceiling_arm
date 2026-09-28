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
