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

