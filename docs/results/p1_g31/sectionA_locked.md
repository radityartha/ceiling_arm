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
