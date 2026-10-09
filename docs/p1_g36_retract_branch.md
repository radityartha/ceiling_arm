# P1 / G36 — R10 ev 10/11: cabang IK yang tak bisa dipulangkan + B&B antar-lengan (OFFLINE + mock, nol gerak)

> Sesi G36, 2026-10-08. Lanjutan [p1_g35_env_speed.md](p1_g35_env_speed.md) §B7.1 (R10 ditolak plan-only NYATA: ev 10
> cabang +103.7° → retract ev 11 COLLIDE −14.1 mm + MoveIt NO-PLAN), §B7.4.3 (NO-PLAN belum didiagnosis).
> **§A ditulis dan DIKUNCI SEBELUM pengukuran apa pun** (salinan `docs/results/p1_g36/sectionA_locked.md` + sha256).
> §B diisi sesudah. §B menang atas §A; konflik DITULIS.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

- Kode: `return_rest.plan_retract` (lurus dua lengan → per lengan lurus / `plan_fn` MoveIt, goal sendi REST, `start_joints`
  = keadaan ditempatkan), `reach_dwell_probe._plan_and_screen` (MoveIt plan_only, 5 attempts, 15 s → tuck → torsi → S18 →
  lingkungan; NO-PLAN = action gagal / error_code ≠ SUCCESS / lintasan kosong, **alasan MoveIt tidak dicatat**),
  `move_to` (HW: `attempts` = `--plan-attempts` 3, RETRYABLE = NO-PLAN/TUCK/TORQUE-UNSAFE/INTERARM-COLLIDE, **ENV-* tidak
  diulang**), `g31_screen.walk` (plan-only: `_plan_and_screen` **sekali**, tanpa ulang), `run_g32` (tugas =
  `reach_dwell_probe --arms X --target ... --move`), `env_static_map_pub` (kotak 2 cm kolom, **tumbuh +0.05 m** L∞).
- Log: `g34_planonly_g35b_hw.log` ev 10/11 (di atas), `g34_planonly_g34n.log` ev 11 (sampel 1 MARGIN 19.6 → MoveIt
  PLANNED), `g35b_ev11_check.log` (7 rencana ev 10 arsip: 2 cabang j1 +100°, 5 cabang −96…−120° / +13°).
- **Belum dilihat:** log move_group launch HW G35b pada saat NO-PLAN, keadaan scene MoveIt pada konfigurasi itu.

### A1. 🔒 Diagnosis (A) — mock domain 77, nol gerak

Mock (`use_fake_hardware:=true enable_gantry_bridge:=false`, `ROS_DOMAIN_ID=77`), `env_static_map_pub` peta kanonik g34n
di scene. Konfigurasi = titik akhir rencana ev 10 arsip (`g34_planonly_plans_g35b_hw_R10.jsonl` s0 = +103.7°, kontrol
`..._g34n_R10.jsonl` s0 = +100.3°, dan satu cabang −° sebagai kontrol negatif). Per konfigurasi:
1. `/check_state_validity` (grup '') pada awal (titik akhir ev 10) dan REST-arm_3 dengan sisanya sama → valid + kontak.
2. Permintaan PERSIS `plan_fn` (`_plan_and_screen` goal sendi REST, `start_joints`, 15 s, 5 attempts) × n ≥ 5, dicatat
   error_code + baris move_group.
3. Variasi satu-faktor: (a) `env_static_map` dihapus dari scene, (b) waktu rencana 60 s. Lalu dinyatakan penyebabnya.
4. Log move_group HW G35b dibaca pada stempel NO-PLAN (bila tercatat).

### A2. 🔒 Perbaikan (B) — paling sederhana, tanpa melonggarkan MARGIN = TOLAK

Kandidat utama: **rencana tugas diterima hanya bila garis lurus pulang lengan itu dari titik akhirnya lolos
`plan_retract` (lurus, tanpa MoveIt)**, di `_plan_and_screen` (dipakai HW `move_to` dan plan-only). Penolakan baru =
verdict yang bisa diulang (rencana OMPL lain). Plan-only harus mengulang seperti HW (`attempts` 3), kalau tidak, laju
gagal plan-only tidak memprediksi HW. Penyaring yang ada tidak diubah.
Uji: plan-only mock **R10 seed 36 k ≥ 10** (laju gagal + CI 95 % Clopper–Pearson), **R0 k ≥ 3** (lolos seperti sebelumnya),
**P+ ev 8 G33** tetap COLLIDE (kontrol `g33_controls`), `--self-test` alat yang tersentuh.

### A3. 🔒 B&B antar-lengan (C) — hanya bila A + B selesai

`InterArmChecker` / `CrossGantryChecker.screen_trajectory`: badan diam dihitung sekali, pasangan (geometri, sampel)
dilewati bila batas bawah Lipschitz − slack > 0 DAN ≥ minimum berjalan (seperti G35 B2.1). Profil dulu (apakah fase
"antar-lengan" 24–38 s/tugas memang komputasi saring, atau muat/hal lain). **Regresi WAJIB** (lama beku vs baru, semua
arsip lintasan yang dipakai G35 A3 + retract): verdict identik 100 %, d_min bit-identik, pasangan + titik identik.
Gagal → kode lama dikembalikan, DITULIS.

### A4. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D239 | Konfigurasi awal +103.7° **VALID** di scene MoveIt (bukan "start state in collision"); REST juga VALID. |
| D240 | Permintaan `plan_fn` dari +103.7° gagal di mock **≥ 3/5**; alasan move_group = solver tidak menemukan jalur dalam waktu (bukan validasi jalur / start invalid). |
| D241 | Penyebab = peta: tanpa `env_static_map` → PLANNED ≥ 4/5; waktu 60 s → tetap gagal ≥ 3/5. Kontrol +100.3° PLANNED ≥ 4/5 (ulang G34). |
| D242 | Cabang buruk (arm_3 j1 > 0 di akhir ev 10) muncul di **20–45 %** rencana ev 10 mock (sebelum perbaikan). |
| D243 | Sesudah B: plan-only mock R10 k = 10 → **10/10 LOLOS**, semua retract ev 11 **lurus** (MoveIt tidak terpicu); penolakan baru terpicu ≥ 1× (bukti jalur hidup). |
| D244 | Sesudah B: R0 k = 3 → 3/3, penolakan baru terpicu **0–1×** di R0. P+ ev 8 tetap COLLIDE −12.04 mm. |
| D245 | Fase antar-lengan per tugas didominasi **komputasi saring** (bukan muat checker) — ≥ 70 % dari 24–38 s. |
| D246 | B&B antar-lengan eksak ≥ 5× pada lintasan tugas; regresi 100 % (verdict, pasangan, titik, d_min bit-identik). |

## B. Hasil terukur

> sha256 `sectionA_locked.md` = `89b95c31…c724a` (dikunci 2026-10-08 13:50, sebelum mock naik). Data + skrip:
> `docs/results/p1_g36/`. Mock launch 2242814 (domain 77, SIG_DFL, SigIgn `0x1001001`), `env_static_map_pub` g34n
> 136 558 voxel → 22 246 kotak, di scene **True**. Nol gerak lengan nyata sepanjang sesi.

### B1. Diagnosis (A1) — kenapa MoveIt NO-PLAN di HW

**Alasan HW (log move_group G35b, dibaca sesudah mengunci):** kedua permintaan retract arm_3 (dua urutan,
stempel …440.3 dan …470.3) = `Computed path is not valid. Invalid states at index locations: [55 58] out of 163` /
`[74 76 77 78] out of 161`, kontak `env_static_map` ↔ `t2_a1_right_finger_prox_link` / `_dist_link`, lalu
**"Motion plan was found but it seems to be invalid (possibly due to postprocessing)"**. Jadi OMPL MENEMUKAN jalur;
validasi pasca-proses (titik rapat) menolaknya di TENGAH jalur. Bukan waktu habis, bukan start-invalid.

**Mock, permintaan persis `plan_fn`** ([g36_diag.py](results/p1_g36/g36_diag.py), [log](results/p1_g36/g36_diag_base.log)):

| konfigurasi (akhir ev 10) | start | REST | rencana |
|---|---|---|---|
| G35b-HW s0, arm_3 j1 **+103.7°** | VALID | VALID | **SUCCESS 5/5** (0.3–3.5 s) |
| G34 g34n s0, **+100.3°** (kontrol) | VALID | VALID | SUCCESS 5/5 |
| G34b-HW s0, −119.5° (kontrol neg.) | VALID | VALID | SUCCESS 5/5 |

Kegagalan HW **tidak tereproduksi** di mock. Satu-satunya beda model yang ditemukan = **sendi jari gripper**: HW hanya
menerbitkan `*_right_finger_bottom_joint` (mimic dihitung MoveIt; tidak ada di perekam), mock menerbitkan semua = 0.
Sapuan jari t2_a1 di start ([log](results/p1_g36/g36_diag_finger.log), [log2](results/p1_g36/g36_diag_finger2.log),
[mimic](results/p1_g36/g36_diag_mimic.log)), dari +103.7°:

| `right_finger_bottom` | −0.09 | **0** | 0.05 | 0.10 | 0.15 | 0.20 | 0.25 | 0.30 | 0.60 | 0.96 | 0 / −0.05 + mimic URDF |
|---|---|---|---|---|---|---|---|---|---|---|---|
| start | VALID | VALID | **INVALID** | INVALID | INVALID | INVALID | INVALID | INVALID | INVALID | INVALID | VALID |
| SUCCESS | 5/5 | 5/5 | 5/5 | 1/5 | 3/5 | 1/5 | 2/5 | 5/5 | 0/5 | 0/5 | 10/10 |

- Titik akhir +103.7° duduk **di batas** kotak peta MoveIt (+5 cm L∞): jari +0.05 rad sudah membuat start "in collision".
  EnvChecker (Euklides, jari di netral) menilai titik itu CLEAR ≥ 50 mm — konsisten: L∞ sampai √3× lebih ketat di sudut.
- Mock gagal hanya di indeks **0–2** (start); HW di indeks **tengah**. Mekanisme HW persis tidak diketahui (nilai jari HW
  tidak terekam) — yang terbukti: **perilaku MoveIt dari titik ini rapuh dan bergantung detail model** (0 ↔ 0.05 rad jari).
- ⚠ **Temuan keselamatan (di luar lingkup, DITULIS):** dari start "in collision", adaptor `Fix Start State In Collision`
  menggeser start **0.07–1.12 rad** lalu MoveIt mengembalikan **SUCCESS** (j1 sampai +115.7° dari +103.7°) — segmen
  waypoint 0 → 1 adalah lompatan yang tidak disaring di antara titik (S18/lingkungan hanya menyaring waypoint).
- A1.3 (a) tanpa peta / (b) 60 s **tidak dijalankan**: dasar sudah SUCCESS 15/15, variasi tidak bisa membedakan apa pun.
  B menang atas A, ditulis.

### B2. Perbaikan (A2) — rencana tugas diterima hanya bila jalan pulang lurus CLEAR

`reach_dwell_probe._plan_and_screen(..., retract_check=False)`: bila True, sesudah semua penyaring lama,
`screen_retract()` = `return_rest.plan_retract(g, [arm], keadaan_di_titik_akhir, S18, lintas, env)` **tanpa MoveIt**
(garis lurus lengan itu ke REST; partner + gantry lain ditahan). Bukan CLEAR → `RETRACT-BLOCKED` (masuk `RETRYABLE`,
`move_to` minta sampel OMPL lain); tidak bisa disaring → `RETRACT-UNSCREENED` (menolak, tidak diulang). Dinyalakan di
jalur tugas HW (`dual_trial`, yang dipakai `run_g32` lewat `--arms`) dan `g31_screen` (plan-only kini mengulang 3× seperti
HW, `TASK_ATTEMPTS`). Pemanggil lain (`v28_screen`, `sched_screen`, `dual_arm_targets`, plan_fn `return_rest`) default
False = tidak berubah. Penyaring/margin tidak diubah.

- **Offline, 13 rencana ev 10 arsip** ([log](results/p1_g36/g36_retract_offline.log)): cabang **+100…+111°: 5/5 DITOLAK**
  (4 MARGIN, 1 COLLIDE = G35b-HW), cabang lain **8/8 CLEAR** — pemisahan bersih.
- **Plan-only mock** ([log](results/p1_g36/g36_planonly.log), [analisis](results/p1_g36/g36_analyze.log)), sampel
  independen (`--repeats 1` per panggilan):

| | LOLOS | gagal, CI95 Clopper–Pearson | RETRACT-BLOCKED | retract ev 11 | ev 10 diterima di +100° |
|---|---|---|---|---|---|
| **R10** | **10/10** | 0 / 10, **[0, 0.308]** | 8× (ev 10: 4, ev 13: 4) | **lurus 10/10** (MoveIt 0) | **0/10** (j1 −134…−91, +8) |
| **R0** | **3/3** | 0 / 3, [0, 0.708] | 1× (ev 13, t5 arm_4) | ev 3/7/10 lurus 3/3 | — |

  Percobaan tugas R10: PLANNED 60, RETRACT-BLOCKED 8, TORQUE-UNSAFE 2, NO-PLAN 1. Laju cabang buruk ev 10 = 4/14
  percobaan (29 %) mock, 5/13 arsip (38 %).
- **P+ ev 8 G33** (EnvChecker tidak diubah): **COLLIDE −12.04 mm** ([log](results/p1_g36/g36_pplus.log)).
- Biaya: plan-only per tugas median 40.9 s (G34n 30.8) — ulang sampel + saring garis pulang dengan S18 LAMA (B dijalankan
  sebelum C). Dengan C, saring garis pulang ≈ 1–2 s.

### B3. B&B antar-lengan (A3)

**Profil:** S18 = 121 pasangan mesh–mesh × ~1.5 ms = **175 ms/waypoint**, rata (tak ada pasangan dominan) → tugas
124–130 titik = **23.6–30.7 s**. Muat: S18 3.1 s, lintas 4.7 s (660 pasangan hull) per proses; lintas menyaring 0.18 s/tugas.
Fase "antar-lengan" HW 24–38 s/tugas = muat ~8 s + S18 ~24 s → komputasi saring ≈ 75 %.

**Implementasi** (`scripts/interarm_collision.py`): `InterArmChecker.screen_trajectory` = B&B eksak per PASANGAN
(batas bawah d₀ − perpindahan_a − perpindahan_b − 1 mm, perpindahan = |Δt| + r‖ΔR‖_F); loop lama tetap sebagai
`screen_trajectory_dense`. **`CrossGantryChecker` tetap padat** (B&B di Python lebih LAMBAT di sana: 0.18 → 0.4 s; hull
murah) — juga `RotCrossChecker` yang meng-override `check()`.

- ⚠ **Bug tertangkap sebelum dipakai (DITULIS):** radius dari `aabb_local` (cara G35) bernilai **inf** pada mesh BVH S18 —
  B&B versi pertama benar tetapi tidak pernah memangkas (24.7 s). Cek awal saya "AABB ≥ verteks" lolos karena
  `x > inf` selalu salah. Sekarang radius = |verteks| maks, inf pada geometri berpasangan → **raise**.
- Uji cepat 4 rencana arsip: S18 **24.2 → 1.3, 23.6 → 0.9, 25.1 → 1.6, 30.7 → 2.0 s (12–28×)**, identik 8/8.
- `--self-test` + uji baru C (arm_1 REST → kontak, dua lengan kontak → REST): terpangkas == padat, **LULUS**; `--cross`
  LULUS ([log](results/p1_g36/g36_selftest.log)). **Uji mutasi** (suku perpindahan dibuang): **CLEAR +192 mm vs
  COLLIDE** → self-test **GAGAL** ([log](results/p1_g36/g36_selftest_mutation.log)) — uji bisa gagal.

**Regresi WAJIB (A3)** — lama (`interarm_collision_old.py`, salinan beku sebelum port) vs baru, objek terpisah, 16 pekerja
([g36_regress.py](results/p1_g36/g36_regress.py), [ringkasan](results/p1_g36/g36_regress_summary.log)):

| sumber | kasus | titik | beda | lama → baru (s, saring saja) |
|---|---|---|---|---|
| plan-only G28 smoke / G31 / G32 / G32-HW / G33 / G34 (semua) / **G36** — tugas + retract dua lengan dari tiap titik akhir | 1 287 | 103 958 | **0** | 46 620 → 2 894 (16.1×) |
| V28 + V28-after (G28y) — sama | 732 | 67 188 | **0** | 22 983 → 1 395 (16.5×) |
| kontrol G33 (jendela HW terukur, 24 sendi lengan bergerak, S18 g1 + g2) | 46 | 8 512 | **0** | 3 404 → 127 (26.7×) |
| G34b HW retract lurus dari keadaan terukur | 5 | 300 | **0** | 159 → 20 (8.1×) |
| **jumlah** | **2 070** | **179 958** | **0** | 73 166 → 4 436 (**16.5×**) |

- **Verdict, pasangan, titik identik 2 070/2 070; d_min bit-identik** (perbandingan `==` float).
- Per kasus: tugas (lengan vs partner diam) median **20.9×** (5.6–269×); retract dua lengan bergerak median **12.2×** (3.3–232×).
- ⚠ **Semua kasus arsip S18 = CLEAR** (lengan se-gantry tidak pernah < 50 mm di arsip) — jalur MARGIN/COLLIDE B&B diuji hanya
  oleh self-test C (dua sapuan COLLIDE, identik) dan uji mutasi (CLEAR palsu tertangkap). Ditulis, bukan disembunyikan.
- Waktu diukur dengan 16 pekerja di 16 inti (plus jeda SIGSTOP selama parkir B6.6); rasio sah, angka absolut bukan.

### B4. Smoke MOCK `run_g32` R10 (bukan data) — jalur alat HW utuh

[g36_smoke.sh](results/p1_g36/g36_smoke.sh), [log](results/p1_g36/g36_smoke_R10.log), arsip `smoke/g36smoke_R10_*`:
kode G36 penuh (probe `move_to(retract_check=True)` + S18 B&B), monitor + perekam mock, lengan mock REST dulu.
**14/14 event rc 0, 6/6 tugas**, makespan mock 578.92 s (G34 smoke 884.67 s; mock + 16 pekerja regresi berbagi CPU —
tidak sebanding). Tiap tugas mencatat saring garis pulang (ev 2: se-gantry 537.8 / lintas 509.7 / lingkungan 88.6 mm,
~2 s). ev 11 retract g2 @ −10° **lurus dua lengan**. RETRACT-BLOCKED tidak terpicu di run ini. rc 134 runner saat keluar =
warisan benign (G24b).

### B5. Papan skor D239–D246

| # | Dugaan | Hasil |
|---|---|---|
| D239 | +103.7° VALID di scene MoveIt; REST VALID | ✓ di mock (jari 0, juga mimic URDF); HW: indeks invalid di tengah jalur → start tampaknya valid juga. Tetapi jari +0.05 rad → INVALID (batas tipis). |
| D240 | NO-PLAN ≥ 3/5 di mock; alasan = solver kehabisan waktu | ✗ **mock SUCCESS 5/5**; alasan HW = jalur ditemukan lalu **invalid sesudah pasca-proses** (kontak jari ↔ peta), bukan waktu habis |
| D241 | penyebab = peta (tanpa peta PLANNED, 60 s tetap gagal); kontrol +100.3° PLANNED ≥ 4/5 | — variasi (a)/(b) **tidak dijalankan** (dasar sudah sukses, B1); kontrol +100.3° ✓ 5/5. Kontak HW memang dengan `env_static_map` |
| D242 | cabang buruk 20–45 % rencana ev 10 | ✓ arsip 5/13 (38 %), mock G36 4/14 percobaan (29 %) |
| D243 | R10 k = 10 → 10/10, ev 11 lurus semua, penolakan baru ≥ 1× | ✓ **10/10** (CI95 gagal [0, 0.308]), lurus 10/10, RETRACT-BLOCKED 8× |
| D244 | R0 k = 3 → 3/3, penolakan baru 0–1×; P+ tetap COLLIDE −12.04 | ✓ 3/3, 1× (ev 13), P+ −12.04 mm |
| D245 | fase antar-lengan ≥ 70 % komputasi saring | ✓ ~75 % (S18 ~24 s dari 24–38 s; sisanya muat 2 checker ~8 s) |
| D246 | B&B ≥ 5× pada tugas; regresi 100 % | ✓ tugas median 20.9× (min 5.6×); **2 070/2 070** identik, d_min bit-identik (arsip hanya CLEAR, B3) |

**Tally G36: 6 tepat / 1 meleset (D240) / 1 tidak dinilai penuh (D241).**

### B6. Pertentangan / Rule 12

1. **B menang atas A (ditulis):** A1.3 (tanpa peta / 60 s) tidak dijalankan — tidak informatif setelah dasar mock sukses.
   Mekanisme persis kegagalan HW (indeks tengah) **tidak tereproduksi**; nilai sendi jari HW **tidak pernah direkam**
   (`js_record` + perekam hanya punya sendi lengan/rel). **Dibaca sesudahnya di HW (16:13, parkir B6.6):
   `*_right_finger_bottom_joint` = 0.0001 / 0.0 / 0.0 / −0.0002 — sama dengan mock** → jari BUKAN penyebabnya; beda
   mock↔HW tetap tak terjelaskan (sisa kandidat: keacakan OMPL + pasca-proses, sendi mimic). Perbaikan B tidak bergantung padanya: rencana yang ujungnya tidak
   bisa pulang lurus tidak diterima, jadi MoveIt-retract dari tepi rak tidak lagi dibutuhkan di jadwal ini.
2. **Celah keselamatan baru (tidak diperbaiki, di luar lingkup):** `Fix Start State In Collision` dapat menghasilkan
   SUCCESS dengan lompatan 0.07–1.12 rad antara waypoint 0 dan 1; penyaring hanya memeriksa waypoint. Berlaku untuk
   SEMUA rencana MoveIt yang start-nya "in collision" di mata MoveIt (mis. jari dekat kotak peta +5 cm). Kandidat G37:
   tolak rencana dengan |Δq| antar-waypoint > batas, atau matikan adaptor.
3. **Model jari:** `EnvChecker`/S18 memakai jari di netral (`CONFIG_JOINTS` tanpa sendi gripper). Gripper HW yang terbuka
   menonjol lebih jauh daripada model → CLEAR bisa optimistis di dekat rak. Belum diukur.
4. **Plan-only kini mengulang 3×** (`TASK_ATTEMPTS`) — sebelum G36 plan-only mengambil satu sampel sementara HW mengulang
   3× (`--plan-attempts`). Angka lolos plan-only G31–G35 lebih pesimistis daripada HW; G36+ sebanding dengan HW.
5. B dijalankan dengan S18 LAMA (C belum diport) — verdict tidak terpengaruh (C bit-identik), waktu plan-only ya.
6. **Parkir g1 (permintaan operator di tengah sesi, ditulis):** `pose_to_g` S13" kini juga mengizinkan
   `PARK = {1: (1.500, 0), 2: (0, 0)}`; saring offline 0 → 1.45/1.50 m lengan REST: S28 CLEAR 380 mm, lingkungan CLEAR
   86.6 mm (g2 di 0 atau 1.35) ([log](results/p1_g36/g36_park_screen.log)). p0 0/0 dan origin encoder **tidak** berubah.
   Cakupan peta di sekitar g1 x ≈ 1.5 tidak dicek terpisah.
   **Dijalankan di HW 16:13 (operator kembali: "kalau mau gerak gasken"; dipilih: parkir g1 SAJA; K0 sel = akhir G35b ya,
   K1 operator di e-stop ya).** Pekerja regresi dijeda (SIGSTOP) selama stack nyata hidup. `remount_check` LULUS;
   launch 2295472 (SIG_DFL, SigIgn `0x1001001`), 4× Actuator '6', 7/7 active, table1/table2 ARMED 0.5 / 0.4 mm;
   `env_static_map` di scene True; lengan ≤ 0.143° dari REST. `pose_to_g --gantry 1 ... 1.5 0`: DRY S28 CLEAR 380 mm,
   peta CLEAR 86.5 mm, T_cmd 83.3 s → `--move` **JTC 0, akhir 1.499241 m (−0.76 mm), 80.4 s, lengan bergeser maks
   0.067°, g2/rot tidak bergerak, torsi puncak 1.72 N·m** ([log](results/p1_g36/hw/g36park_g1.log)). Stack dimatikan
   (`node list --no-daemon` 0; crash dump shutdown baseline dihapus), regresi dilanjutkan (SIGCONT).
   **Keadaan sel sekarang: g1 PARKIR 1.499 m, g2 0.36 mm, rot 0/0, lengan REST.**
   **Keputusan operator sesudahnya (2026-10-08):** parkir 1.5 m = posisi istirahat TETAP sel. p0 jadwal = input P1
   (`make_instance`, kaki pertama dari p0), jadi: **G36b tetap p0 0/0** (g1 → 0 sekali di awal sesi, di luar makespan;
   sebanding dengan G24b–G35b); **blok eksperimen berikutnya (seed baru) dibuat dengan p0 g1 = 1.5** (instansi baru:
   make_instance → solve → oracle → plan-only mock k ≥ 3 → HW) — lebih realistis: sel mulai dari posisi istirahatnya.
7. Muat checker (S18 3.1 s + lintas 4.7 s per proses) kini dominan di fase antar-lengan (~8 s/tugas); cache geometri seperti
   G35 belum dikerjakan.

### B7. G36b — HW R0 + R10 seed 36 sama-sesi, kode G36 (2026-10-09)

> Prompt = §D. Gerbang (operator, sebelum apa pun): **K0 sel sama seperti akhir G36 — ya, g1 di PARKIR 1.5 m**;
> **K1 operator di e-stop selama semua gerak — ya**; K2 (jari) dibaca sesudah bring-up. Data: `docs/results/p1_g36/hw/`.
> **§B7.A DIKUNCI sebelum bring-up** (salinan `docs/results/p1_g36/g36b_A_locked.md` + sha256 di bawah); §B7.0+ diisi sesudah.

#### B7.A 🔒 Protokol + dugaan (SEBELUM bring-up)

Urutan = G35b B7 apa adanya, kode G36 (probe `retract_check`, S18 B&B, `pose_to_g` PARK): self-test env + interarm
(+ `--cross`) → remount_check → bring-up nyata (env_map:=true) → K2 jari → g1 PARK → 0 (DRY, `--move`; di luar makespan) →
plan-only NYATA k=3 R10 lalu R0 (`g36_planonly.sh`, sampel independen) → `g36b_run.sh` R0 DRY/`--move` → `g36b_home.sh`
R0hm R0 0 → R10 DRY/`--move` → `g36b_home.sh` R10hm R10 1.5 (PARKIR) → matikan. Gerbang plan-only gagal (< 3/3 salah satu
varian) → varian itu TIDAK dijalankan, operator ditanya. Fase per event = `p1_g35/hw_replay.phases()` pada log baru.

| # | Dugaan |
|---|---|
| D247 | Makespan R0 dalam **460–515 s** (G35b 629.67: antar-lengan −120…−130, retract −30, saring pulang +12). |
| D248 | Makespan R10 dalam **480–550 s**. |
| D249 | **\|Δ(R10 − R0)\| ≤ 30 s**; tanda TIDAK diduga (G34b +66 dikurangi hemat rotasi G35 ~71 → ≈ −5; ulang sampel R10 menambah). |
| D250 | Plan-only NYATA **R10 3/3, R0 3/3**; RETRACT-BLOCKED terpicu **≥ 1×** di 3 sampel R10 (mock 0.8/sampel). |
| D251 | Eksekusi: R0 6/6 dan R10 6/6, 14/14 rc 0 per varian, nol auto-stop; **semua retract tak-kosong lurus** (MoveIt `return_rest` tidak terpicu), termasuk ev 11 R10 g2 @ −10°. |
| D252 | Fase antar-lengan per tugas HW **≤ 12 s** (6/6 tugas R0; G35b 24.1–37.6 s). |
| D253 | Torsi puncak terukur semua event **≤ 12 N·m** (G35b 10.72). |
| D254 | K2: `*_right_finger_bottom_joint` \|q\| ≤ 0.01 rad keempat lengan (G36 B6.1: 0.0001 / 0 / 0 / −0.0002). |

sha256 `g36b_A_locked.md` = `3eb1773a…e5282968f` (dikunci 2026-10-09 13:58, sebelum bring-up).

#### B7.0 Sebelum gerak — 🔴 K0 GAGAL terukur, sel dipulihkan (izin operator)

`env_collision --self-test` PASS (0.8 s); `interarm_collision --self-test` LULUS (C terpangkas == padat) + `--self-test --cross`
LULUS (`--cross` sendirian = galat argparse, bukan uji). `remount_check` GERBANG LULUS (ICMP 4/4, nol basi). Bring-up nyata
launch log `/tmp/g36b_launch.log` (SIG_DFL + setsid nohup), 4× Actuator '6', **7/7** active; `env_static_map` di scene
False → HILANG → publish ulang → **True** (pola diharapkan), `--once` True. `js_record` + `reach_dwell_monitor`
(`/tmp/g36hw_step`) hidup. **K2 (D254):** `right_finger_bottom` = 8.4e-5 / 1.06e-4 / 6.1e-5 / 4.0e-5 rad.

**Keadaan terukur ≠ K0 (operator menjawab "sama, g1 PARK"):** arm_1 23.8° dari REST (j4 −103.3 vs −79.4), arm_2 3.2°,
**arm_3 104.2°** (j3 −10.8 vs +7.4, j4 +24.8 vs −79.4), arm_4 34.9° (j4/j5); g1 1499.2 mm **−1.48°**; g2 **−34.9 mm /
+2.91°** (akhir G36: g2 0.36 mm, rot 0/0, lengan REST). Bridge: table1 ARMED di posisi aktual; table2 "NOT armed" sekali
(sebelum controller g2 aktif). Semua launch sejak parkir G36 (2026-10-08 19:20–20:37) = **mock** (`TableFakeHardware`) +
kamera → perubahan **bukan dari perangkat lunak PC ini** (tangan / power-cycle / alat lain — penyebab tidak diketahui, DITULIS).
Operator memilih: pulihkan dengan alat repo, DRY + saring, tanya sebelum tiap `--move` (operator di e-stop):

| langkah | saring (DRY) | hasil HW |
|---|---|---|
| `return_rest` arm_1+2 | lurus; se-gantry 622.7, lintas 911.4, peta 117.7 mm | rc 0, galat 0.031°, torsi 4.31 N·m, rel tidak bergerak |
| restart `gantry_2_with_arm_controller` (deactivate/activate) | — | 7/7 active; `/table_hw_commands` diam = perintah rel = state |
| `return_rest` arm_3+4 | lurus; 548.8 / 887.5 / 117.8 mm | rc 0, galat 0.047°, torsi 5.63 N·m |
| `pose_to_g` g2 → (0, 0) | S28 859.1, peta 117.8 mm, T_cmd 3 s | table2 ARMED di −34.9 → 1× REJECTED (target < `min_mm` 0) → satu `go_to_absolute(0,0)` di akhir rampa; **0.000 mm / 0.000°**, 1.10 s |
| `pose_to_g` g1 (PARK) → (0, 0) | S28 380.0, peta 86.6 mm, T_cmd 83.3 s | **0.52 mm / 0.000°**, 81.1 s, lengan geser 0.067° |

Log: `hw/g36b_restore_*.log`. ⚠ **Temuan (DITULIS, tidak diubah):** restart controller ternyata tidak perlu (perintah sudah =
state); tetapi bila perintah JTC sendi rel basi ≠ state, goal **parsial** `return_rest` (lengan saja, `allow_partial_joints_goal`)
berpotensi membuat rampa rel dari state ke perintah basi → bridge ARM di tengah → rel bergerak bersama retract tanpa disaring
(dibaca dari kode JTC Humble, **tidak diuji**). Sel di p0 G36b sebelum langkah 2.

#### B7.1 Plan-only NYATA — R10 3/3, R0 3/3

`DOMAIN=0 g36_planonly.sh g36b_hw 3 3` ([log](results/p1_g36/g36b_planonly.log)), 14:17–14:24, **~1.2 min/sampel**
(mock G36 dengan S18 lama ~5 min). **R10 3/3, R0 3/3 LOLOS.** RETRACT-BLOCKED: **R10 2×** (sampel 1 ev 10: cabang
ditolak MARGIN 23.7 mm lalu **COLLIDE −14.9 mm** = mode gagal G35b; percobaan ke-3 PLANNED, ev 11 g2 @ −10° **lurus dua
lengan, lingkungan 63.4 mm**), **R0 1×** (sampel 3 ev 12). Retract MoveIt **0×**. Lingkungan min rencana tugas 62.7 mm.

#### B7.2 Eksekusi R0 + R10 (`run_g32` apa adanya, DRY dulu, operator di e-stop)

| | G35b R0 | **G36b R0** | **G36b R10** |
|---|---|---|---|
| penilai | 6/6, 14/14 rc 0 | **6/6, 14/14 rc 0**, nol auto-stop | **6/6, 14/14 rc 0**, nol auto-stop |
| sukses @ s | 112.6 / 238.4 / 293.9 / 411.4 / 594.9 / 629.7 | 93.4 / 216.1 / 248.0 / 337.6 / 483.9 / 519.4 | 88.2 / 189.9 / 260.4 / 286.7 / 319.6 / 452.8 |
| **makespan** | 629.67 s | **519.43 s** (−110.24) = 0.894 × P1′-serial 581.26 | **452.77 s** |
| torsi puncak | 10.72 | 11.02 `t2_a1_joint_2` | 10.93 `t2_a2_joint_2` |
| retract (tak-kosong) | 3 lurus | 3 lurus (ev 3/7/10), MoveIt 0 | 2 lurus (ev 3, **ev 11 g2 @ −10° dua lengan, 58.5 mm**), MoveIt 0 |
| ulang tugas | — | ev 5 MoveIt NO-PLAN → ulang (+~20.8 s); ev 13 RETRACT-BLOCKED (MARGIN) → ulang (+~14.6 s) | **0** |

**Δ(R10 − R0) = −66.67 s** — pertama kali terukur bersih di HW. Fase per jenis ([g36b_analyze.py](results/p1_g36/g36b_analyze.py),
[log](results/p1_g36/g36b_analyze.log)); runner serial → makespan = jumlah durasi event (R0 519.4, R10 452.6 ✓):

| jenis (s) | G35b R0 | G36b R0 | G36b R10 |
|---|---|---|---|
| traverse | 191.7 (S28 22.7, env 1.8, gerak 160.4) | 192.4 (22.1, 1.7, 160.3) | 200.7 (**30.0**, 3.7, 160.1) |
| tugas | 267.9 (rencana 9.1, **antar-lengan 181.3**, env 9.2, eks 55.8) | 192.2 (28.2, **64.8**, 9.3, **saring pulang 14.4**, 63.2) | 158.4 (10.2, 65.7, 9.2, 14.3, 46.7) |
| retract | 170.1 (muat 25.8, saring 39.6, gerak 93.2) | 134.8 (25.0, **6.1**, 93.2) | 93.5 (16.7, 4.3, 62.1) |

- **G36b R0 vs G35b R0 (−110.24 s):** antar-lengan tugas −116.5 (B&B: 24.1–37.6 → **10.3–11.6 s/tugas**, sisanya muat 2
  checker ~8 s), saring retract −33.5, saring garis pulang +14.4 (1.9–3.1 s/tugas), ulang tugas ev 5/13 ≈ +35.
- **Δ(R10 − R0) −66.67 s** = retract −41.3 (R10 hanya 2 retract tak-kosong vs 3: −31 s gerak; muat −8.3) + tugas −33.8
  (ulang R0 ≈ −35) + traverse +8.3 (sapuan berotasi S28 +7.9, env +2.0). **Tanpa ulang R0, Δ ≈ −31 s** (model P1′ −50.87).
  Satu sampel per varian: ulang OMPL/RETRACT-BLOCKED adalah keragaman ~±35 s per run.
- Fase lingkungan traverse berotasi R10: ev 7 1.30 s, ev 12 1.55 s (≤ 3 s, D234 G35b kini terjawab ✓).
- Pulang ([R0](results/p1_g36/hw/g36hw_home_R0.log), [R10](results/p1_g36/hw/g36hw_home_R10.log)): semua lurus (torsi ≤ 7.10),
  akhir R0 g1 0.96 / g2 0.78 mm; akhir R10 **g1 PARKIR 1.499859 m**, g2 0.67 mm, rot 0/0, lengan ≤ 0.037° dari REST.

#### B7.3 Papan skor D247–D254

| # | Dugaan | Hasil |
|---|---|---|
| D247 | R0 460–515 s | ✗ **519.43** (+4.4 di atas; dua ulang tugas ≈ +35 s tidak masuk prediksi) |
| D248 | R10 480–550 s | ✗ **452.77** (27 s DI BAWAH; nol ulang, 2 retract kosong) |
| D249 | \|Δ\| ≤ 30 s | ✗ **−66.67 s** (tanpa ulang R0 ≈ −31 — masih di luar) |
| D250 | plan-only 3/3 + 3/3, RETRACT-BLOCKED ≥ 1× di R10 | ✓ 3/3 + 3/3, R10 2× (+ R0 1×) |
| D251 | 6/6 + 6/6, 14/14 rc 0, nol auto-stop, semua retract lurus termasuk ev 11 R10 | ✓ (MoveIt `return_rest` tetap belum pernah dipakai lengan nyata) |
| D252 | antar-lengan ≤ 12 s/tugas | ✓ R0 10.31–11.57, R10 10.27–11.88 (12/12) |
| D253 | torsi ≤ 12 N·m | ✓ 11.02 |
| D254 | jari \|q\| ≤ 0.01 rad | ✓ ≤ 1.1e-4 awal; akhir ≤ 1.9e-3 |

**Tally G36b: 5 tepat / 3 meleset** (ketiganya makespan/Δ — keragaman ulang per run tidak dimodelkan).

#### B7.4 Pertentangan / Rule 12

1. **K0 dijawab "sama" tetapi terukur berbeda (B7.0)** — gerbang lisan tidak cukup; snapshot `/joint_states` gabungan
   (lengan vs REST, rel + rotasi) harus jadi bagian K0 di prompt berikut. Penyebab perubahan tidak diketahui.
2. Restart `gantry_2_with_arm_controller` dilakukan berdasarkan dugaan yang ternyata salah (perintah sudah = state); tanpa
   efek. Risiko goal parsial + perintah rel basi (B7.0) belum diuji — kandidat G37 (cek `/table_hw_commands` diam sebelum
   gerak, atau pose_to_g dulu).
3. Satu run per varian; Δ −66.67 s memuat ~35 s ulang di R0. Klaim "R10 lebih cepat" butuh k ≥ 3 run per varian.
4. Retract MoveIt (`return_rest` cabang `plan_fn`) **masih 0×** di HW — dengan `retract_check` jalur itu memang jarang.
5. Crash dump shutdown `ros2_control_node` (baseline) dan `ros2 topic hz` milik saya yang diputus `timeout` dihapus.
6. Perekam `hw/g36b_joint_states.csv.gz` (48 MB) **lokal saja, tidak di-commit** (seperti G32-HW … G35b).

## C. Keadaan akhir (Rule 12)

- **Gerak nyata: hanya parkir g1** 0.0005 → 1.499 m (B6.6, atas izin operator, lengan diam). Mock domain 77 (launch 2242814) naik 13:5x, dimatikan bersih 15:2x (helper SIGINT, launch SIGINT,
  `node list --no-daemon` 0, tanpa crash dump baru). Lengan mock digerakkan (smoke) — fake hardware.
- **Diubah:** `scripts/reach_dwell_probe.py` (`_plan_and_screen(retract_check)`, `screen_retract()`, `RETRYABLE` ke modul
  + `RETRACT-BLOCKED`, `move_to(retract_check)`, `dual_trial` menyalakan); `scripts/interarm_collision.py`
  (`InterArmChecker.screen_trajectory` B&B, `screen_trajectory_dense`, `_radii`, Cross tetap padat, self-test C);
  `docs/results/p1_g31/g31_screen.py` (tugas: 3 percobaan + retract_check); `docs/results/p1_g32/pose_to_g.py` (PARK).
- **Baru:** `docs/results/p1_g36/` (diag, offline, plan-only, analisis, smoke, regresi, `interarm_collision_old.py` beku,
  `g36b_run.sh`, `g36b_home.sh`).
- **Belum di HW:** retract_check, S18 B&B. **Sel berakhir dengan g1 di PARKIR 1.499 m** (G36b mulai dengan g1 → 0). Δ(R10 − R0) HW masih belum terukur.
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi multi-langkah dengan mock + regresi; dilaporkan.

## D. Prompt G36b (salin ke chat BARU)

> ✅ Dijalankan 2026-10-09 sebagai G36b → §B7, §C2.

**Rekomendasi: Opus, effort TINGGI** — aturan terima tugas baru (`retract_check`) dan S18 B&B dipakai lengan nyata untuk
pertama kali; galat di keduanya diam (CLEAR palsu / lengan tertinggal di tepi rak) sampai terjadi. Gerbang plan-only dan
regresi sudah ada, jadi ground truth tersedia — tetapi biaya galatnya fisik.

```
Sesi G36b -- HW R0 + R10 seed 36 SAMA-SESI (ukur Delta bersih) dengan kode G36. Repo ceiling_arm, branch feat/rgbd-deploy.
BACA PENUH: CLAUDE.md; docs/p1_g36_retract_branch.md (B1-B6, C); docs/p1_g35_env_speed.md (B7 urutan HW, B7.4);
docs/results/p1_g36/g36b_run.sh + g36b_home.sh; docs/results/p1_g32/run_g32.py.
FAKTA G36 (offline+mock, nol gerak): HW NO-PLAN G35b = "Computed path is not valid ... found but invalid (postprocessing)",
jari t2_a1 <-> env_static_map; mock dari konfigurasi yang sama SUCCESS 15/15; titik akhir +103.7 di batas kotak MoveIt
(jari +0.05 rad -> start INVALID). PERBAIKAN: tugas diterima hanya bila garis lurus pulang lengan itu dari titik akhirnya
CLEAR (return_rest.plan_retract tanpa MoveIt) -> RETRACT-BLOCKED -> sampel OMPL lain (move_to, --plan-attempts 3).
Plan-only mock R10 10/10 (CI95 gagal [0, 0.31]), ev 11 lurus 10/10, cabang +100 diterima 0/10; R0 3/3; P+ ev 8 COLLIDE
-12.04. S18 = branch-and-bound eksak (regresi lama-vs-baru: lihat B3), tugas ~24 s -> ~1-2 s. Smoke mock run_g32 R10 14/14.
PARKIR (operator 2026-10-08): g1 diparkir TETAP di 1.500 m (beban plafon); G36b SENGAJA tetap p0 0/0 (keterbandingan
dengan G35b; blok seed baru nanti p0 g1 = 1.5, B6.6); sel mulai dengan g1 di 1.499 m; pose_to_g
mengizinkan PARK; HW 2026-10-08 0 -> 1.499241 m sukses (galat -0.76 mm, 80.4 s).
PREDIKSI KASAR (kunci versi sendiri di §A): R0 ~460-515 s (G35b 629.67; antar-lengan -120..-130 s, retract -30 s,
saring pulang +12 s); R10 ~480-550 s (ulang sampel ev 10/13 menambah); Delta(R10-R0) tanda TIDAK pasti.
GERBANG (tanya operator SEBELUM apa pun): K0 sel sama seperti akhir G35b (lengan REST, g1/g2 ~0 ATAU g1 di PARK 1.5,
rot 0/0, rak + benda x~2.1 tidak dipindah)? K1 operator di e-stop selama gerak? K2 gripper: catat posisi jari (HW
hanya menerbitkan *_right_finger_bottom_joint; G36 B6.3) -- ros2 topic echo sekali, tulis nilainya.
0. KUNCI §A (dugaan D247+) SEBELUM bring-up.
1. env_collision --self-test, interarm_collision --self-test (+ --cross) LULUS; remount_check; bring-up nyata (SIG_DFL +
   setsid nohup, PID launch ASLI, SigIgn), 4x Actuator '6', 7/7, ARMED; env_static_map_pub kini DIJALANKAN LAUNCH
   (env_map:=true; log "di planning scene: True", bisa sesudah satu "HILANG -- publish ulang") + --once True;
   js_record + reach_dwell_monitor. Bila g1 di PARK: pose_to_g --gantry 1 --seed 36 --variant R0 0 0 (DRY lalu --move).
2. Plan-only NYATA k=3 R10 lalu R0 (DOMAIN=0 docs/results/p1_g36/g36_planonly.sh g36b_hw 3 3 -- sampel independen,
   tidak berhenti di gagal pertama) -> harus 3/3 + 3/3; catat RETRACT-BLOCKED.
3. g36b_run.sh R0 --dry lalu --move; g36b_home.sh R0hm R0 0; g36b_run.sh R10 --dry/--move; g36b_home.sh R10hm R10 1.5
   (akhir sesi = PARKIR g1). Matikan bersih (node list --no-daemon = 0).
4. Laporan §B (makespan vs prediksi, Delta, fase per event via p1_g35/hw_replay.phases, jumlah RETRACT-BLOCKED, tiap
   retract lurus/moveit), tally, commit; prompt G37 (celah Fix-Start-State B6.2, cache geometri S18/lintas) + model/effort.
ATURAN: tak ada gerak tanpa env_static_map di scene DAN EnvChecker; MARGIN = TOLAK; B menang atas A dan DITULIS;
job > 10 menit via setsid nohup; pkill -f membunuh shell sendiri (pilih PID via ps/awk).
```

## C2. Keadaan akhir G36b (Rule 12)

- **Gerak nyata:** pemulihan K0 (4 langkah, B7.0), R0 14 event + pulang, R10 14 event + pulang + PARKIR; semua rc 0, nol
  fault launch (0 baris `ros2_control_node` ERROR / Kortex exception), nol auto-stop, tidak ada kontak dilaporkan.
  **Akhir: keempat lengan REST (≤ 0.037°), g1 PARKIR 1.499859 m, g2 0.67 mm, rot 0/0.**
- **Stack:** monitor + perekam SIGINT; launch 2540658 (SigIgn `0x1001001`) SIGINT → keluar 8 s; `ros2 node list --no-daemon` **0**.
- **Kode tidak diubah** di G36b. **Baru:** `docs/results/p1_g36/` `g36b_A_locked.md`, `g36b_planonly.log` + keluaran
  `g36_planonly_*g36b_hw*`, `g36b_analyze.{py,log,json}`, `hw/g36hw_R0_*`, `hw/g36hw_R10_*`, `hw/g36hw_home_*`,
  `hw/g36b_*` (pemulihan, konsol, monitor, launch `.log.gz`).
- **Belum diukur:** keragaman makespan per varian (k = 1); MoveIt `return_rest` di HW; goal parsial dengan perintah rel basi.
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi HW multi-langkah; dilaporkan.

## E. Prompt G37 (salin ke chat BARU)

**Rekomendasi: Opus, effort TINGGI** — mengubah penyaring rencana (lompatan Fix-Start-State) dan memuat geometri dari cache;
galat di keduanya diam (rencana berbahaya diterima / geometri basi menilai CLEAR) sampai lengan bergerak. Regresi lama-vs-baru
tersedia sebagai ground truth, jadi offline + mock cukup; nol HW.

```
Sesi G37 -- OFFLINE + mock (domain 77), NOL gerak HW. Repo ceiling_arm, branch feat/rgbd-deploy.
BACA PENUH: CLAUDE.md; docs/p1_g36_retract_branch.md (B1, B3, B6.2, B7.0, B7.2, B7.4); scripts/reach_dwell_probe.py
(_plan_and_screen); scripts/interarm_collision.py; scripts/env_collision.py (cache geometri G35 = pola).
FAKTA G36b (HW 2026-10-09): R0 519.43 s, R10 452.77 s, Delta -66.67 s (R0 punya 2 ulang tugas ~35 s); antar-lengan
per tugas 10.3-11.9 s, ~8 s di antaranya MUAT InterArmChecker + CrossGantryChecker (S18 3.1 s + lintas 4.7 s per proses),
muat retract 8.3 s/retract; saring garis pulang 1.9-3.1 s/tugas. K0 dijawab "sama" tetapi lengan 1/3/4 jauh dari REST,
g2 -34.9 mm/+2.91 deg (B7.0) -> dipulihkan.
TUJUAN (urut, A dikunci dulu, dugaan D255+):
A. Celah Fix-Start-State (B6.2): rencana MoveIt dengan |dq| antar-waypoint > batas (usul: > 2x langkah rencana normal,
   ukur distribusinya dulu dari arsip plans jsonl G31-G36b) DITOLAK di _plan_and_screen sebagai verdict baru yang bisa
   diulang. Uji: konfigurasi G36 B1 jari +0.05 rad (start INVALID -> adaptor menggeser) harus DITOLAK; arsip rencana
   G31-G36b yang dieksekusi di HW harus 100 % tetap diterima (kalau tidak: batas salah, tulis).
B. Cache geometri S18 + lintas-gantry (pola env_collision G35: kunci = hash URDF+mesh+parameter, rusak -> bangun ulang).
   Regresi WAJIB: verdict/pasangan/titik/d_min bit-identik vs tanpa cache pada arsip G36 regresi (2 070 kasus).
   Ukur muat per proses (target <= 0.5 s).
C. K0 terukur: skrip kecil (atau opsi remount_check --ros) yang mencetak gabungan /joint_states >= 0.5 s: maks |q-REST|
   per lengan, rel + rotasi tiap gantry, jari; TOLAK bila tidak sesuai p0/PARK. Dipakai sebagai K0 di prompt HW berikut.
D. Laporan + prompt G37b (HW: k >= 2 run per varian R0/R10 seed 36, ATAU blok seed baru p0 g1 = 1.5 sesuai B6.6 --
   tanyakan operator mana) + rekomendasi model/effort.
ATURAN: B menang atas A dan DITULIS; MARGIN = TOLAK; job > 10 menit via setsid nohup; pkill -f membunuh shell sendiri.
```
