# P1 / G31 — JADWAL DENGAN ROTASI: patch penyaring C-1/C-2, model waktu rotasi, scheduler (lin, rot), PLAN-ONLY MOCK

> Sesi 2026-10-06. **Nol gerak** (lengan, rel, rotasi). Robot tidak dinyalakan (G4 = mock). Sumber prompt:
> [p1_g30_rot_hw.md §D](p1_g30_rot_hw.md) dengan koreksi [p1_g28_yaml.md §D](p1_g28_yaml.md). Kode/hasil: `docs/results/p1_g31/`.

---

## A0. Gerbang + instrumen (SEBELUM §A dikunci)

### A0.1 Gerbang

| # | Gerbang | Jawaban operator |
|---|---|---|
| G1 | urutan G28-ON / yaml | **terjawab**: G28-ON + G28-YAML selesai (`joint_limits.yaml` = URDF 24/24) |
| G2 | patch C-1 / C-2 | **IZIN** (prompt). C-1 = `true_hull` **langsung** di `CrossGantryChecker` (satu tempat; `return_rest`, probe S18, `sched_screen`, `rail_to_g` ikut), **bukan** ganti pemakai ke `RotCrossChecker`, **tanpa** flag kompatibilitas. C-2 = probe S18 meneruskan **kedua** sendi rotasi, nama ketat, hilang = TOLAK |
| G3 | model waktu rotasi | **T_cmd bridge** (prompt): parameter `rot_cmd` kembar `rail_cmd` di `sched.traverse_time`, default 0 (model p1_state 5.6 tidak berubah) |
| G4 | stack plan-only | **MOCK** (`use_fake_hardware:=true enable_gantry_bridge:=false`) — dijawab 2026-10-06 |
| G5 | himpunan rotasi scheduler (ditanya sesi ini) | **\|rot\| ≤ 35°** per gantry (grid 5°, 15 nilai); ±10° dihitung offline sebagai pembanding |

Keadaan: disk **60 GB** bebas (prompt "1.5 GB" — ruang dibebaskan di luar sesi); nol proses ROS; URDF
`p1_g23/reach_dwell_live.urdf` = `/tmp/reach_dwell_live.urdf` sha `f02e7c532cdb`; arsip `/tmp/g24b_js.csv`, `/tmp/g26_js.csv` ada (N2).

### A0.2 Tempat yang disentuh (baca kode)

- **C-1** `scripts/interarm_collision.py:165` `CrossGantryChecker.__init__`: `buildConvexRepresentation(False)` → `g.geometry.convex`
  = semua verteks mesh + adjacency tak-konveks (coal tanpa qhull, G29 B1). Pemakai: probe `screen_interarm` (S18, objek **hangat**
  dipakai ulang lintas rencana), `return_rest.py:184`, `sched_screen.py:107`, `g22_plan.sweep_screen` (lewat `rail_to_g`, `run_g22`,
  `sched_screen`), subkelas `g29_rot_screen.RotCrossChecker` / `OldFresh` (memanggil `true_hull` **lagi** di atas hull induk),
  diag G29 (`g29_gjk_*`, `g29_hull_diag` — arsip; sesudah patch jalur "lama" mereka tidak lagi mereproduksi hull palsu).
- **C-2** `scripts/reach_dwell_probe.py:555` `want` = rel + sendi lengan lain → rotasi tidak diteruskan → `q_from` dari `neutral`
  → S18 menilai rot = 0. `CrossGantryChecker.q_from` membuang nama yang tidak dikenal **diam-diam**.
- 🔶 **Di luar C-2, ditemukan sesi ini:** `return_rest.py:150` `held` = sendi lengan gantry lain + rel lain, **tanpa rotasi** →
  penyaring retract `return_rest` juga menilai rot = 0. Tidak dipatch (izin hanya C-1/C-2); dilaporkan, gerbang G32.
- **G3** `sched.traverse_time` (`sched.py:90`): `dr = min(dr, 360 − dr)` (siklik) untuk T_rot motor; `rail_cmd` hanya sumbu linier.
  `sched_coll.axis_times` arah-pendek (G29 C-4). Tidak ada `assert` segitiga di kode — hanya docstring (`sched.py:45`).
- `sched_screen.walk` / `g22_plan.events`: traverse dideteksi dari **rel** saja; rotasi 0.0 keras → alat baru `g31_*` (yang lama tidak diubah).

### A0.3 Ukuran pekerjaan oracle ([g31_a0.py](results/p1_g31/g31_a0.py) → [log](results/p1_g31/g31_a0.log))

17 seed (iv) (G26 1/2/13 + G28 14), 96 node unik: tuple L1-benar rot ≠ 0, \|rot\| ≤ 35° = **34 579** (\|rot\| ≤ 10°: 9 680).
oracle‴ + PATH (`g29_oracle_rot._job`) **7.03 s/tuple** → penuh ≈ **4.5 jam** pada 15 proses. ⇒ oracle **malas-eksak** (A4).

---

## A. Protokol — DIKUNCI sebelum patch, sebelum data

> 🔒 §B menang atas §A; pertentangan DITULIS.

### A1. 🔒 Patch C-1 (`interarm_collision.py`)

`true_hull(g)` (salinan `g29_rot_screen.true_hull`, scipy `ConvexHull` → `coal.Convex(points, triangles)`) dipindah ke
`interarm_collision.py`; `CrossGantryChecker.__init__` memakai `true_hull(g.geometry.convex)` untuk setiap geometri ber-
`buildConvexRepresentation`. **Hanya hull**: `GeometryData` tetap dipakai ulang (hangat) — G29 B1: hangat vs segar dengan
hull sejati ≤ 0.003 mm; diukur ulang (K-C1a), tidak diubah. Docstring kelas: paragraf hull diperbarui; "konstan 429 mm"
dikoreksi ke 380 mm terukur. `g29_rot_screen.py` **tidak** diubah (hull-dari-hull diukur, K-C1d).

### A2. 🔒 Patch C-2 (`reach_dwell_probe.screen_interarm`)

`want += ['t1_rotation_joint', 't2_rotation_joint']` (keduanya, selalu, bila ada lengan lain). Nama ketat: setiap nama di
`want` dan di `joint_names` lintasan harus ada di model **setiap** checker yang dipakai (`model.existJointName`) — tidak → log
error + `return None` (pemanggil menolak, `INTERARM-UNSCREENED`). Hilang dari keadaan → penolakan yang sudah ada (`len(other) < len(want)`).

### A3. 🔒 Patch G3 (`sched.py`)

`traverse_time(dlin, drot, t_fold=0.0, rail_cmd=0.0, rot_cmd=0.0)`; `traverse_matrix(..., rot_cmd)`; `Instance.rot_cmd = 0.0`;
`default_costs` ikut `not rot_cmd`; `solve_gantry` meneruskan `inst.rot_cmd`. rot_cmd > 0:
`T_rot = rot_cmd · max(3, π·dr/(2·0.9·10 °/s))`, **dr = |Δrot| tanpa bungkus** (bridge = target encoder absolut, G29 C-4) —
**bukan** `min(dr, 360 − dr)`. rot_cmd = 0 → kode lama apa adanya (siklik).
- **Nilai:** `rot_cmd = 0.9235` = median G30 `t_rot / T_cmd` (8 kaki, T_cmd 3.0) — definisi sama dengan `r` G25 (median traverse / T_cmd).
- (a) **180° belum diukur** (T_cmd 31.4 s vs T_rot 18.26 s); G30 hanya 10° (lantai 3 s). Di himpunan R35, \|Δrot\| ≤ 70° → T_cmd ≤ 12.2 s,
  **ekstrapolasi** dari lantai.
- (b) **Pertentangan (Rule 7):** `sched_coll.axis_times` + rot_cmd = 0 memakai arah-pendek; rot_cmd > 0 memakai tanpa-bungkus (yang
  dieksekusi HW). Di R35 \|Δ\| ≤ 70° < 180° → keduanya **identik**; `sched_coll` tidak diubah, tetap ditandai.
- (c) Ketaksamaan segitiga dengan lantai 3 s: `max(3, k·d)` subaditif, max dua fungsi subaditif subaditif, `t_fold` per pindah ⇒ tetap.
  **Diuji numerik** (assert) atas SEMUA tripel pose R35 (495³) dengan biaya P1′ + rot_cmd.
- (d) **Konkuren:** `max(T_lin, T_rot)` mengandaikan rel + rotasi dalam **satu** perintah (bridge mendukung target absolut lin+rot
  bersama, G30 A0.3). `rail_to_g` lalu `rot_to_g` berurutan = jumlah + overhead kedua — keputusan eksekutor G32, dicatat.

### A4. 🔒 Scheduler (lin, rot) — instance G31

- Seed: 17 (iv) dari `p1_g26/g26_candidates.json` (node, p0 = 0.00/0.00, rot 0/0 — sama dengan sel).
- Pose per gantry: `REF.lin` (33) × **R35** = {−35 … +35, langkah 5°} (15) — G5. Aturan B3 (3) "keduanya ≤ 35°" ⇒ aman untuk
  **kombinasi apa pun** dan dx apa pun (lengan REST) ⇒ amplop terpenuhi **oleh konstruksi**, DP per gantry tidak perlu kopling.
  Aturan (1)/(2) (dx ≥ 0.8 / satu ≤ 10°) **tidak** dieksploitasi (pose > 35° tidak ditawarkan) — batasan, dinyatakan. Pembanding:
  **R10** (5 nilai) dan **R0** (= instance G26).
- reach rot 0 = cache oracle‴ G24 (sama G26). reach rot ≠ 0 = **oracle‴** (`g29_oracle_rot._job` `ok3`, predikat sama, K0 G29
  72/72); `ok4` (PATH) dicatat. **Malas-eksak:** mulai dari L1 (superset) untuk rot ≠ 0 → `solve_exact` → verifikasi tuple
  (node, g, pose, slot) rot ≠ 0 yang **dipakai** jadwal → yang gagal di-nolkan → ulang sampai semua tuple terpakai terverifikasi.
  Eksak karena oracle‴ ⊆ L1: optimum relaksasi yang seluruhnya layak = optimum sejati. `zone`/`hand` = L1 (sama G24/G26).
- Biaya: **P1′** (g25 konstanta, serial) + `rot_cmd = 0.9235`. (iii) `sched_coll.schedule_conflict` = None (seperti G26);
  (ii) "pindah ≥ 1 tiap gantry" dilaporkan, tidak menyaring plan-only.
- Keluaran: `g31_candidates.json` (format `g26_candidates` + `rot_deg` per stop, `rot_cmd`), makespan R0/R10/R35, pindah, rotasi terpakai.

### A5. 🔒 Plan-only (MOCK) — `g31_screen.py`

Salinan `sched_screen.walk` dengan: `placed` memuat **kedua rotasi** (p0 0/0); event = `g22_plan.events` disalin, traverse bila
(rel **atau** rot) berubah; **traverse** = `g29_rot_screen.sweep_rot(RotCrossChecker, rect)` (lengan REST — retract mendahului;
`assert` lengan placed = REST saat traverse: **rotasi hanya dengan lengan REST**, G30 G3a), SS ikut; **retract** = mesh se-gantry +
`CrossGantryChecker` (terpatch) dengan held berotasi; **tugas** = `_plan_and_screen` (terpatch C-2) dengan `start_joints` berotasi,
`--tau-max 12`, offset j2 7.7. k-of-k 3. **Lintasan disimpan** (jsonl: start, traj, verdict). Stack mock; KY1 runtime batas = URDF.
Urutan: R35 seed yang jadwalnya memakai rot ≠ 0 lebih dulu, lalu sisanya; kontrol R0 seed 16 (G28-YAML LOLOS) 3/3.

### A6. 🔒 Kontrol (wajib lulus sebelum langkah 3)

| id | jenis | isi | lulus bila |
|---|---|---|---|
| K-C1a | negatif | 2 000 keadaan N1 G29 (rng 29): terpatch (hangat) vs `OldFresh` (hull sejati, segar) | \|Δ\| ≤ 0.01 mm semua, verdict sama |
| K-C1b | negatif, arsip | 14 traverse N2: `sweep_screen` terpatch vs `OldFresh`; vs arsip | ≤ 0.01 mm, verdict sama; arsip − baru ≥ −0.01 mm |
| K-C1c | negatif, **V28 tersimpan G28** (183) | S18 antar-gantry (`only` = lengan, held = `start`) hull palsu (objek baru per lintasan) vs terpatch; per lintasan | selisih **satu arah** (lama − baru ≥ −0.01 mm) 183/183; verdict hanya boleh berubah ke arah ketat; ditulis per lintasan |
| K-C1d | negatif | `RotCrossChecker` (hull-dari-hull) bagian lengan vs terpatch, 2 000 keadaan | ≤ 0.01 mm |
| K-C1e | ulang G29 | `g29_rot_screen.py controls` dengan induk terpatch | N1 0/2000, N2 14/14, N3, P1–P4 LULUS |
| K-C1f | swa-uji | `interarm_collision.py --self-test --cross` | LULUS |
| K-C2p | positif | arm_1 IK ke pelat g2 pada g2 rot +60° (P3 G29), lintasan REST → IK; `screen_interarm` dengan `start` berotasi: sebelum C-2 (rotasi dibuang) vs sesudah | sebelum CLEAR/MARGIN lebih jauh, sesudah **COLLIDE**; `start` tanpa rotasi → `None`; nama salah → `None` |
| K-C2n | negatif | 183 lintasan V28 (start rot 0/0): `screen_interarm` sebelum vs sesudah C-2 (checker sama) | bit-identik 183/183 |
| K-RR | DRY | `return_rest` DRY pada `/joint_states` sintetis (domain 77, tanpa stack), REST + rot 0/0 | CLEAR, rc 0 |
| K-G3a | negatif | `traverse_time` rot_cmd = 0 vs salinan fungsi lama, 10⁵ acak | bit-identik |
| K-G3b | negatif | G25 45 instance (P1′, P1′-par) + G26 50 baris (p0 0/0): makespan + jadwal vs arsip | bit-identik |
| K-G3c | negatif | R0 instance 17 seed: rot_cmd 0.9235 vs 0 | bit-identik |
| K-G3d | segitiga | semua tripel 495³ R35 (g1, g2), biaya P1′ + rot_cmd | 0 pelanggaran > 1e-9 |
| K-G3e | daya | mutan "rot_cmd diabaikan" pada instance R35 | makespan berubah pada ≥ 1 seed **atau** dinyatakan tak teruji |

### A7. 🔒 Dugaan D175–D188 (ditulis SEBELUM patch dan data)

Prior tally: G30 — kode disalin dari pola teruji + diuji sintetis/mock → tepat; mekanisme dari kode dibaca → tepat.
G28-YAML — mekanisme terukur tepat 7/7; generalisasi dari n = 1 meleset. G29 — besaran geometri tanpa hitung meleset.

| D | dugaan | dasar |
|---|---|---|
| D175 | K-C1a maks \|Δ\| ≤ **0.003 mm** | G29 B1 hangat vs segar 0.003 |
| D176 | K-C1c satu arah 183/183; maks (lama − baru) **≥ 20 mm** | G29 N1 +77.9, 1 396/2 000 > 0.1 mm |
| D177 | K-C1c: verdict S18 berubah pada **0** dari 81 PLANNED V28 | lengan lain REST, antar-gantry jauh; traverse benar ≥ 452 mm |
| D178 | K-C1d maks ≤ 1e-6 mm (hull dari hull = hull) | himpunan konveks sama |
| D179 | K-C1e semua LULUS **jalan pertama** | patch = pindah fungsi teruji |
| D180 | K-C2p sebelum CLEAR, sesudah COLLIDE | G29 P3 LS −34.4 mm hanya ada pada rot +60° |
| D181 | K-C2n bit-identik 183/183 | rot 0 = `neutral` |
| D182 | K-G3a–c bit-identik; K-G3d 0 pelanggaran | default tak disentuh; subaditif |
| D183 | R35 makespan **< R0** pada **1 – 8** dari 17 seed (sisanya =) | optimum = min #pindah (G21); rotasi murah (lantai ≈ 2.8 s ≪ t_fold 55.6 s) — menolong hanya bila mengurangi pindah |
| D184 | R10 < R0 pada ≤ jumlah seed R35 < R0 (R10 ⊂ R35 ⇒ makespan R10 ≥ R35 — pasti; dugaan: **R10 menangkap < ½** perbaikan R35) | geser base 0.4·sin 10° = 69 mm kecil |
| D185 | malas-eksak: ≥ 1 seed butuh ≥ 2 iterasi; tiap seed ≤ 6 iterasi | A0: 2/8 tuple acak L1 lolos oracle‴ |
| D186 | plan-only R35: seed ber-rotasi LOLOS k-of-k 3 ≥ **80 %**; traverse rect semua CLEAR, min ≥ **82 mm** | B3 verifikasi 1° min 82.4 mm |
| D187 | kontrol R0 seed 16 mock LOLOS 3/3 | G28-YAML 14/14 nyata |
| D188 | K-RR CLEAR; antar-gantry min **< 575.1 mm** (G30 hull palsu) dan > 500 | hull sejati lebih dekat |

Aturan: NOL gerak; rotasi hanya dengan lengan REST (berlaku untuk rencana G32); mock dimatikan bersih (PID launch ASLI,
`ros2 node list --no-daemon` = 0); crash dump milik sesi dihapus.

---

## B. Hasil terukur

> §A dikunci **2026-10-06 11:42:24**, sha256 `e4fce725a2a8…` ([sectionA_locked.md](results/p1_g31/sectionA_locked.md)).
> Nilai SEBELUM patch ditangkap dari kode produksi apa adanya ([g31_pre.log](results/p1_g31/g31_pre.log), `g31_pre.json`), lalu patch dipasang.

### B0. Patch (diff)

| berkas | perubahan |
|---|---|
| `scripts/interarm_collision.py` | + `true_hull()` (salinan G29); `CrossGantryChecker` → `true_hull(g.geometry.convex)`; docstring hull + SS 380 mm. GeometryData tetap hangat |
| `scripts/reach_dwell_probe.py` | `screen_interarm`: `want += t1/t2_rotation_joint`; nama tak dikenal model checker mana pun → `None` (TOLAK) |
| `ros2_ws/.../sched.py` | `rot_cmd` (traverse_time, traverse_matrix, Instance, default_costs, solve_gantry); rot_cmd > 0 tanpa bungkus |

### B1. Kontrol A6 — semua LULUS ([g31_post.log](results/p1_g31/g31_post.log), [g31_g3_controls.log](results/p1_g31/g31_g3_controls.log), [g31_g29_controls.log](results/p1_g31/g31_g29_controls.log))

| id | hasil |
|---|---|
| K-C1a | terpatch (hangat) − `OldFresh`, 2 000 keadaan: maks **0.0014 mm**, > 0.001 mm pada 6, verdict beda 0 ✅ |
| K-C1b | 14 traverse: terpatch == `OldFresh` (bit, 14/14), verdict sama; arsip − baru ≥ 0 (maks G26 s1 g1 543.5 → 514.7, G26 s2 g2 546.1 → 514.8) ✅ |
| **K-C1c** | 183 lintasan V28 G28: lama − baru **min −0.000 / median +1.415 / maks +107.650 mm**; satu arah **183/183**; > 0.1 mm pada 115; **verdict berubah 0** (PLANNED 0/81); min baru PLANNED 323.1 mm, semua 317.9 ✅ |
| K-C1d | RotCrossChecker (hull-dari-hull, segar) − terpatch (hangat): 0.0014 mm ✅ (≤ 0.01). Terisolasi pasca-kunci (keduanya segar, [log](results/p1_g31/g31_c1d_iso.log)): **bit-identik 2000/2000** |
| K-C1e | `g29_rot_screen.py controls` ulang: P4, N1 0/2000, N3 380.000, P1, **P2 COLLIDE −59.6** (sama G29), P3 LS −34.4, N2 bit-identik 14/14 ✅. N2(i) rekonstruksi arsip 6/14 = efek C-1 (arsip dihitung hull palsu) |
| K-C1f | swa-uji `--self-test --cross` LULUS (REST×REST **514.8 mm** = koreksi G29 B1); `--self-test` se-gantry LULUS |
| **K-C2p** | arm_1 → pelat kanan g2 @ g2 rot +60°: **sebelum CLEAR 169.7 mm** (`t1_a1_right_finger_prox ↔ t2_platform`, dinilai rot 0) → **sesudah COLLIDE −34.4 mm** (`t1_a1_right_finger_dist ↔ t2_mount_plate_right`); start tanpa `t2_rotation_joint` → `None` ("21/22 sendi"); nama `t1_a1_jiont_6` → `None` ✅ |
| K-C2n | sesudah C-2 == resep pra-C-2 (checker sama, rot 0/0): **183/183** bit-identik ✅ |
| K-RR | `return_rest --arms arm_3 arm_4` DRY, `/joint_states` sintetis domain 77 (lengan +1.0°, rot2 −0.29°): CLEAR, se-gantry 615.2, **antar-gantry 558.7 mm** ([log](results/p1_g31/g31_return_rest_dry_fake.log)) ✅ |
| K-G3a | traverse_time rot_cmd = 0 vs fungsi lama: 4 × 10⁵, beda **0** ✅ |
| K-G3b | G25 **90/90** (P1′ + P1′-par, opt + jadwal) dan G26 **50/50** bit-identik ✅ |
| K-G3c | R0 17 seed rot_cmd 0.9235 vs 0: 17/17 bit-identik ✅ |
| K-G3d | 495 pose R35, 3 × 495³ tripel (t_fold, t_fold_first, 0): **0** pelanggaran; tanpa-bungkus == arah-pendek di R35 (maks 70°) ✅ |

### B2. Scheduler (lin, rot) — 17 seed ([g31_sched.py](results/p1_g31/g31_sched.py) → [log](results/p1_g31/g31_sched.log), [g31_candidates.json](results/p1_g31/g31_candidates.json))

P1′ + `rot_cmd = 0.9235`; R0 = G26 **17/17** (makespan bit-identik). oracle‴ malas-eksak: cache rot ≠ 0 **23 500** tuple
(dari 34 579 L1; 68 %, bukan "malas" seperti diharapkan — lihat B5 (1)); semua tuple rot ≠ 0 terpakai ok3 = ok4 (PATH) — tidak ada z = 1.40.

| seed | R0 | R10 | R35 | Δ R35 | pindah R0 → R35 | rot R35 dipakai (g1 / g2) |
|---|---|---|---|---|---|---|
| 1 | 263.09 | 257.82 | 257.82 | −2.0 % | 3 → 3 | +5, −25 / 0, −10 |
| 2 | 356.53 | 353.90 | 353.90 | −0.7 % | 5 → 5 | −25, −5 / −5, −25, +10 |
| 13 | 262.55 | 257.28 | 257.28 | −2.0 % | 4 → 4 | −5, −5 / −20, +5 |
| 16 | 300.92 | 300.92 | 300.92 | 0 | 4 → 4 | −25, −5 / +5, 0 |
| 17 | 216.27 | 213.64 | 213.64 | −1.2 % | 2 → 2 | −15 / −10 |
| 18 | 328.73 | 326.09 | 326.09 | −0.8 % | 5 → 4 | +5 / +5, −15, −20 |
| 20 | 265.19 | 265.19 | 265.19 | 0 | 3 → 3 | −5, −5 / 0 |
| 21 | 335.42 | 332.78 | 332.78 | −0.8 % | 5 → 5 | −15, 0 / −10, −15, +5 |
| 22 | 241.44 | 238.80 | 238.80 | −1.1 % | 3 → 3 | −15 / −15, −20 |
| 23 | 252.54 | 252.54 | 252.54 | 0 | 3 → 2 | 0, −10 / −20 |
| 26 | 293.00 | 293.00 | 293.00 | 0 | 3 → 3 | −5, −10 / 0 |
| 29 | 259.23 | 259.23 | 259.23 | 0 | 2 → 2 | 0, −5 / −15 |
| 33 | 260.45 | **259.92** | **257.28** | −1.2 % | 3 → 4 | −20, +10 / 0, +15 |
| 35 | 257.81 | 257.81 | 257.81 | 0 | 3 → 3 | 0, −10 / 0, 0 |
| **36** | 362.35 | 311.48 | 311.48 | **−14.0 %** | 4 → 4 | 0, −10 / −10, 0 |
| 40 | 265.19 | 265.19 | 265.19 | 0 | 4 → 4 | +5, −15 / +15, −5 |
| 42 | 328.73 | 326.09 | 326.09 | −0.8 % | 4 → 4 | −20 / +5, −25, +5 |

- **R35 < R0 pada 10/17** (Σ −80.45 s); **R10 < R0 pada 10/17 yang sama (Σ −77.81 s = 97 %)**; R35 < R10 hanya seed 33 (−2.64 s).
- Mekanisme dominan: kelipatan **2.64 s** = satu langkah rel 0.10 m dalam `r·T_cmd` — rotasi mengganti **jarak rel**, bukan jumlah pindah.
  Pengecualian **seed 36 (−50.9 s)**: g2 R0 = tugas di p0 → pindah (dengan retract `c_ret` 50.9 s) → pindah; R35 = pose (0.6, −10°)
  menampung tugas 0/1/5 sekaligus → pindah pertama dari p0 **tanpa tugas = tanpa retract** (`t_fold_first` 4.74 s).
- **Seri:** 7/17 seed tanpa perbaikan tetap memakai rot ≠ 0 (pemutus seri DP bebas) — G32 harus memilih rot 0 bila seri (B5 (6)).
- (iii) `schedule_conflict` None 17/17; (ii) pindah ≥ 1 tiap gantry 17/17. Rotasi maks dipakai **25°** (> ±10° terverifikasi HW G30).
- **K-G3e:** mutan "rot_cmd diabaikan" (T_rot motor) mengubah makespan **0/17** → `rot_cmd` **tidak teruji** oleh instance ini (sumbu rel
  mendominasi `max(T_lin, T_rot)`; Δrot terpakai ≤ 35° → T_rot ≤ 5.6 s). Dinyatakan, sesuai A6.

### B3. Plan-only MOCK — **17/17 seed LOLOS 3/3** ([g31_screen.log](results/p1_g31/g31_screen.log), [json](results/p1_g31/g31_screen.json), lintasan [g31_screen_plans.jsonl.gz](results/p1_g31/g31_screen_plans.jsonl.gz))

Stack mock domain 77: launch PID **1695712** (SIG_DFL…execvp via nohup, SigIgn `0x1001001`, SIGINT tidak diabaikan); **7/7**
controller; KY1 **6/6 = URDF**; 7 spawner mati + octomap = baseline ([launch log](results/p1_g31/g31_mock_launch.log.gz)).
Kontrol R0 seed 16 ([log](results/p1_g31/g31_r0_control.log)): **LOLOS 3/3**, 18/18 tugas.

| ukuran (17 seed × 3 sampel) | nilai |
|---|---|
| tugas | **306 PLANNED / 0 lain** (nol NO-PLAN / TORQUE / INTERARM / TUCK) |
| traverse (`sweep_rot` rect, SS ikut) | 177, semua CLEAR; **153 berotasi**; min **304.1 mm** (`t1_mount_plate_right ↔ t2_mount_plate_left`, g2 (1.00, −25°) → (1.50, +5°), g1 di rot ≠ 0) |
| retract antar-gantry (hull sejati, held berotasi) | 87, min **301.1 mm** |
| S18 probe (se-gantry mesh + antar-gantry hull, rotasi diteruskan) | 306, min **262.5 mm** |
| `assert` lengan REST saat traverse | 177/177 (rotasi hanya dengan lengan REST) |

Wall rencana 12–77 s (CPU berebut dengan pool oracle, bukan data).

### B4. Papan skor D175–D188 — **10 / 14 tepat**

| D | dugaan | terukur | |
|---|---|---|---|
| D175 | K-C1a ≤ 0.003 mm | 0.0014 | ✅ |
| D176 | satu arah 183/183, maks ≥ 20 mm | 183/183, +107.65 | ✅ |
| D177 | 0 verdict berubah (81 PLANNED) | 0 | ✅ |
| D178 | K-C1d ≤ 1e-6 mm | **0.0014** (kontrol terkunci mencampur hangat/segar; terisolasi 0, 2000/2000) | ❌ |
| D179 | K-C1e LULUS jalan pertama | ya | ✅ |
| D180 | K-C2p CLEAR → COLLIDE | 169.7 → −34.4 | ✅ |
| D181 | K-C2n 183/183 | 183/183 | ✅ |
| D182 | K-G3a–c bit-identik, K-G3d 0 | ya (K-G3 jalan 1 crash kode sendiri, sebelum data) | ✅ |
| D183 | R35 < R0 pada 1–8 / 17 | **10** | ❌ |
| D184 | R10 menangkap < ½ perbaikan R35 | **97 %** | ❌ |
| D185 | ≥ 1 seed ≥ 2 iterasi; tiap seed ≤ 6 | maks **44** | ❌ |
| D186 | ≥ 80 % LOLOS; traverse CLEAR min ≥ 82 mm | 17/17; 304.1 | ✅ |
| D187 | R0 seed 16 3/3 | 3/3 | ✅ |
| D188 | K-RR CLEAR, 500 < antar < 575.1 | 558.7 | ✅ |

Pola tally: semua yang tepat = **mekanisme kode yang dibaca / diukur sebelumnya** (patch, hull, debounce kontrol). Keempat meleset =
**besaran perilaku baru tanpa hitung** (D183–D185: berapa sering/berapa banyak rotasi menolong; geser base 0.4·sin 10° = 69 mm ternyata
cukup untuk 97 % perbaikan) dan **desain kontrol sendiri** (D178).

### B5. Pertentangan §B lawan §A

| # | Pertentangan |
|---|---|
| (1) | **Oracle malas dipercepat pasca-kunci**: jalan 1 (hanya tuple terpakai) 64 iterasi pada seed 1 R10 tanpa konvergen → dihentikan; jalan berlaku memverifikasi juga ≤ 14 tetangga terdekat per tuple terpakai (eksak tidak berubah: syarat berhenti sama). Akibatnya 23 500/34 579 tuple dihitung. |
| (2) | Proses mati: pre-capture jalan 1 (serial, ~3 jam) dihentikan → `Pool(15)`; scheduler jalan 2 + kontrol R0 jalan 1 **mati bersama sesi** (putus koneksi) → diulang dengan `setsid nohup`; cache oracle append-only dipakai ulang. Log mati disimpan `*_killed*`, `g31_sched_run1_slow.log`. |
| (3) | K-G3 jalan 1 crash (kode sendiri: modul dataclass tidak terdaftar di `sys.modules`); jalan 2 berlaku. |
| (4) | K-C1d terkunci membandingkan RotCross (segar) dengan terpatch (hangat) → mengukur hangat/segar, bukan hull-dari-hull; tambahan terisolasi pasca-kunci ([g31_c1d_iso.py](results/p1_g31/g31_c1d_iso.py)). |
| (5) | K-C1e N2(i) rekonstruksi arsip 6/14 (G29: 14/14) = efek C-1 yang diharapkan: arsip dihitung dengan hull palsu. Diag G29 (`g29_gjk_*`, `g29_hull_diag`) kini tidak lagi mereproduksi hull palsu (tanpa flag kompatibilitas, sesuai G2). |
| (6) | A5 "seed ber-rotasi dulu": **semua** 17 jadwal R35 memakai rot ≠ 0 (termasuk 7 seri tanpa perbaikan) → urutan = urutan seed. |
| (7) | K-RR: keadaan lengan +1.0° (G30 0.56°) → 558.7 vs 575.1 bukan perbandingan sama-keadaan; kasus kedua salah spesifikasi saya (lengan sama), tidak dihitung. |
| (8) | K-G3e mutan tak tertangkap (0/17) — A6 mengizinkan "dinyatakan tak teruji". |
| (9) | Disk 60 GB (prompt 1.5 GB). Mock di `ROS_DOMAIN_ID=77`. |

---

## C. Keadaan akhir (Rule 12)

- **Nol gerak**; robot tidak dinyalakan. Mock: SIGINT ke 1695712 → keluar 10 s; `ros2 node list --no-daemon` (domain 77) **0**; nol proses
  sisa. Crash dump `move_group` 20:06 (200 MB, milik sesi) **dihapus**. Disk akhir **57 GB**.
- **Diubah (belum di-commit):** `scripts/interarm_collision.py` (C-1), `scripts/reach_dwell_probe.py` (C-2),
  `ros2_ws/src/reachability_gng/reachability_gng/sched.py` (rot_cmd). **Baru:** `docs/results/p1_g31/`, dokumen ini.
  **Tidak diubah:** `g29_rot_screen.py`, `g22_plan.py`, `sched_screen.py`, `return_rest.py`, `run_g22.py`, `rail_to_g.py`, `rot_to_g.py`,
  `sched_coll.py`, `joint_limits.yaml`, `dual_table_controller.py`.
- 🔴 **Gerbang G32 (eksekusi jadwal berotasi), ditemukan/diukur sesi ini:**
  1. `return_rest.py:150` `held` **tanpa rotasi** → retract antar-gantry dinilai rot = 0; `run_g22` memanggilnya untuk setiap retract. Perlu izin patch (kembar C-2).
  2. `run_g22.py:178` dan `rail_to_g` (S23) **menolak** \|rot\| > 0.5°; `rot_to_g` hanya rotasi, ≤ 10°, gantry lain ≤ 0.5°. Tidak ada alat
     untuk pose (lin, rot) dalam satu perintah — model `max(T_lin, T_rot)` mengandaikannya (A3 (d)).
  3. Jadwal R35 memakai sampai **25°**; HW terverifikasi hanya **±10°** (G30). R10 menangkap 97 % perbaikan → usul G32 = R10.
  4. Seri: pilih rot 0 bila makespan sama (7/17 seed memakai rotasi tanpa manfaat).
- **Tidak diukur:** rot_cmd pada \|Δrot\| > 10° (ekstrapolasi lantai); aturan B3 (1)/(2) (> 35°); lengan di tugas saat berputar (dilarang);
  S18 pada lintasan **dieksekusi** (hanya plan-only mock); `sched_coll` arah-pendek tetap ditandai (identik di R35).
- Anggaran token Rule 6 (30k/sesi) **terlampaui jauh** — sesi multi-jam, dua kali putus koneksi; dilaporkan.

---

## D. Prompt G32 (salin ke chat BARU) — HW: SATU jadwal berotasi, lengan REST saat berputar

**Rekomendasi: Opus, effort TINGGI** — gerak lengan nyata pada rot ≠ 0 untuk pertama kalinya, dengan alat eksekusi baru
(pose lin+rot) dan patch `return_rest`; galat penyaring di sini diam dan satu arah.

```
Sesi G32 -- HW: eksekusi SATU jadwal dengan rotasi gantry (lengan REST saat berputar). Repo ceiling_arm,
branch feat/rgbd-topo-deploy. Operator di lokasi.

BACA PENUH: CLAUDE.md; docs/p1_g31_rot_sched.md (semua, terutama C); docs/p1_g30_rot_hw.md (A1-A3, B1);
docs/p1_g26_hw.md (B0, tahap 0); docs/results/p1_g22/run_g22.py; scripts/return_rest.py (~150);
docs/results/p1_g30/rot_to_g.py; docs/results/p1_g22/rail_to_g.py; docs/results/p1_g31/g31_screen.py, g31_sched.py.

GERBANG (tanya operator SEBELUM apa pun):
 G1. Izin patch return_rest: held += kedua rotasi, nama ketat, hilang = TOLAK (kembar C-2 G31).
 G2. Rentang rotasi: R10 (terverifikasi HW G30, 97 % perbaikan G31) atau R35 (butuh uji HW +-25 deg dulu)?
 G3. Alat pose: satu trajektori JTC (lin + rot bersama, bridge absolut) ATAU rail_to_g lalu rot_to_g berurutan
     (model max() vs jumlah + overhead kedua -- G31 A3 (d)).
 G4. Seed: usul 36 (R10 -14 %, satu pose berotasi per gantry) atau 1 (G24b/G26 pembanding langsung).
1. KUNCI §A: jadwal ulang seed terpilih dengan Rset G2 + pemutus seri rot 0; plan-only ulang di stack NYATA (g31_screen) 3/3;
   dugaan D189+ (prior tally G31: mekanisme dibaca -> tepat; besaran perilaku baru tanpa hitung -> meleset).
2. Alat run_g32 (salinan run_g22): traverse = alat pose G3 dengan sweep_rot rect dari keadaan TERUKUR; S12 lengan REST
   sebelum setiap rotasi; retract = return_rest terpatch; rekam /joint_states; DRY penuh dulu.
3. Tahap 0 g22 A5; bring-up enable_gantry_bridge:=true; eksekusi satu jadwal; ukur makespan vs P1' (rot_cmd 0.9235),
   galat rotasi per pose, drift, torsi.
4. Laporan §B/§C, p1_state + tally, prompt G33 + rekomendasi model/effort.
ATURAN: rotasi HANYA dengan lengan REST; satu gantry berputar pada satu waktu kecuali amplop B3 diizinkan operator;
B menang atas A dan DITULIS; stack dimatikan bersih (PID launch ASLI, node list --no-daemon = 0); job > 10 menit via setsid nohup.
```
