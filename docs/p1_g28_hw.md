# P1 / G28-HW — validasi oracle⁗ (G27) di stack NYATA, PLAN-ONLY: V28 + saringan (iv) 14 seed baru

> Sesi 2026-09-23. Bagian **offline** dikerjakan dulu (operator **tidak** di lokasi); **bagian ON 2026-09-28**
> (stack nyata, plan-only): §B0c–B3, §C-ON, prompt yaml §F. Sumber prompt: [p1_g27_z140_diag.md §D](p1_g27_z140_diag.md). Kode/hasil: `docs/results/p1_g28/`.
> Alat G22 (`sched_screen`, `g22_plan`), probe (`reach_dwell_probe._plan_and_screen`), oracle, peta:
> **tidak diubah**.

---

## A. Protokol — ditulis 2026-09-23 SEBELUM alat baru ditulis, SEBELUM data apa pun

### A0. Keputusan operator (dijawab SEBELUM §A ini ditulis)

| # | Pertanyaan | Jawaban |
|---|---|---|
| 1 | V28 + (iv) plan-only boleh di stack **mock** sekarang, atau tunggu stack **nyata**? | **Tetap NYATA.** Offline sekarang: §A, alat + DRY, kontrol, smoke test mock **hanya** pada tuple pengembangan (bukan V28) |

Pembagian sesi:

| Bagian | Kapan | Isi |
|---|---|---|
| **OFF** (sekarang) | tanpa operator | §A dikunci; `v28_screen.py` + `v28_score.py`; DRY; K-RNEA offline; smoke test **mock** pada tuple dev (A5) |
| **ON** (operator di lokasi) | G28-ON | tahap 0 g22 A5; R1; V28 (A1–A2); (iv) (A3); penilaian; p1_state; prompt G29 |

### A1. 🔒 Alat `v28_screen.py` — spesifikasi

- **Masukan:** `g27_oracle4.json['V28']` apa adanya (61 tuple, urutan berkas). Tidak ada tuple ditambah/dibuang.
- **Per tuple, 3 sampel, SELALU ketiganya** (tanpa berhenti di penolakan pertama — beda dengan
  `sched_screen`; tujuan = laju per rencana, bukan lolos/gagal jadwal).
- **Start (ditempatkan, nol gerak):** `t{g}_linear_joint` = rel tuple (`rail`), rel gantry lain = **R1**
  (rel terbaca bring-up, dibulatkan grid 0.05 m), kedua rotasi 0, **keempat lengan REST**
  (`g22_plan.REST`). Sama untuk ketiga sampel (tidak berantai).
- **Panggilan:** `_plan_and_screen(arm, pose(xyz), node, 0.002, 2.0, 0.15, 15.0, 12.0, others=3 lengan
  lain, start_joints)` — **argumen identik** `sched_screen.walk`, `pose()` identik (x = 1, w = 0).
- **Penyadap (pass-through, nilai balik tidak berubah):** `reach_dwell_probe._violates_tuck` dan
  `reach_dwell_probe.predict_peak_torque` dibungkus di namespace modul agar lintasan **setiap** rencana
  yang dikembalikan MoveIt (termasuk yang lalu ditolak TORQUE / INTERARM / TUCK) tersimpan.
  `_plan_and_screen` sendiri tidak disentuh.
- **Disimpan per sampel (JSONL, append, resume per `(indeks, sampel)`):** verdict, wall s,
  `joint_names`, per titik `positions / velocities / accelerations / time_from_start`, per titik
  RNEA penuh (6) dan gravitasi statis (6, v = a = 0), puncak probe tersadap, `k_peak2` = argmax pertama
  |RNEA j2|, `N` titik, |statis j2| di titik terakhir.
- **Model RNEA per titik = model probe persis:** `pin.neutral(m)` + 6 sendi lengan dari titik, `v`,
  `a` dari titik (nol bila kosong), URDF = `_urdf_path(node)` (cache `/tmp/reach_dwell_live.urdf`).

**Gerbang (gagal ⇒ berhenti, tidak ada data):**

| # | Gerbang | Harus |
|---|---|---|
| **KD1** | V28 dimuat | 61 tuple = 41 z = 1.40 (`ok4` false) + 20 z = 1.32 (`ok4` true); rel ∈ [0, 1.60] |
| **KD2** | sha256 `/tmp/reach_dwell_live.urdf` | `f02e7c53…` (= URDF G23/G27; cache basi ⇒ berhenti) |
| **KD3** | `JOINT_TORQUE_OFFSET_NM[2]` | 7.7 |
| **K-RNEA off** | 200 lintasan sintetis acak (4 lengan, dalam batas sendi, v/a acak) | maks_k \|RNEA per titik\| == `predict_peak_torque` **bit-identik** per sendi |
| **K-RNEA on** | setiap rencana ber-lintasan di V28 | maks_k \|RNEA per titik\| == puncak probe tersadap (≤ 1e-9); pelanggaran ⇒ berhenti |

### A2. 🔒 Penilaian V28 (`v28_score.py`)

**Dikunci G27 — dikutip, TIDAK ditulis ulang:** A4: oracle⁗ **tolak** ⇒ tuple ≥ 1/3 TORQUE-UNSAFE;
oracle⁗ **terima** ⇒ tuple 3/3 PLANNED. D132: pada setiap rencana V28 z = 1.40 dengan RNEA j2 > 6.30,
indeks puncak RNEA j2 < titik terakhir **dan** gravitasi statis j2 di titik akhir ≤ 5.83.

Definisi (dikunci di sini):

- **Kebenaran per tuple** (dari 3 sampel): **UNSAFE** ⇔ ≥ 1/3 TORQUE-UNSAFE; **SAFE** ⇔ 3/3 PLANNED;
  **LAIN** ⇔ selainnya (0 TORQUE tetapi ≥ 1 NO-PLAN / INTERARM / TUCK / UNSCREENED).
- **A4 ketat:** tuple tolak benar ⇔ UNSAFE; tuple terima benar ⇔ SAFE. LAIN = **salah** di kedua arah.
  Dilaporkan juga varian **torsi-saja** (terima benar ⇔ 0/3 TORQUE) — informatif, bukan penilaian A4.
- **Presisi / recall** (positif = oracle⁗ **terima**): presisi = terima ∩ SAFE / terima; recall =
  terima ∩ SAFE / SAFE. Plus: laju tolak-benar = tolak ∩ UNSAFE / tolak.
- **D132:** atas **setiap** rencana z = 1.40 ber-lintasan dengan RNEA j2 (dari RNEA per titik) > 6.30:
  `k_peak2 < N − 1` **dan** \|statis j2\|(titik N − 1) ≤ 5.83. Satu pelanggaran ⇒ D132 ❌.
- **Posisi puncak per z** (semua rencana ber-lintasan): `k_peak2 = 0` (start), interior, `= N − 1`
  (akhir); dan `RNEA j2 puncak − |statis j2 akhir|`.
- **Proksi G27 di rencana nyata:** `P2_own` = maks \|statis j2\| sepanjang garis lurus ruang-sendi
  REST → q_akhir **rencana itu** (101 titik, `g27_diag.path_max`); dilaporkan `RNEA j2 − P2_own`.

### A3. 🔒 Saringan (iv) — 14 seed baru

`sched_screen.py --plan docs/results/p1_g26/g26_candidates.json --repeats 3 --out
docs/results/p1_g28/g28_screen.json --seeds 16 17 18 20 21 22 23 26 29 33 35 36 40 42` **apa adanya**
(berhenti per seed di penolakan pertama, k-of-k). Diperiksa offline 2026-09-23: 14/14 `ok_ii`, 6 tugas
tiap seed (84 tugas), z ∈ {1.00 … 1.32}, **nol** tugas z = 1.40; `p0` di kandidat = 0.00 / 0.00.
Bila R1 ≠ 0.00 / 0.00: kandidat **dibuat ulang** (g26 A1) sebelum (iv), dicatat. Tidak ada eksekusi
jadwal (prompt 5) kecuali operator minta.

### A4. 🔒 Dugaan D133–D143 — DITULIS SEBELUM alat, DRY, smoke, dan data

Prior (tally 61 meleset / 68 tepat): dugaan dari **mekanisme terukur** tepat; tentang **kode sendiri**
meleset; "kendala lebih longgar" meleset (D115). Data acuan: S16 G26 (z = 1.40: TORQUE 11/16, RNEA j2
PLANNED 2.41–3.74 / TORQUE 6.43–8.14); 145/145 rencana diterima-z < 1.40 aman; C′ `RNEA₂ − P₂` median
−0.022, maks +0.054; `m₂` = 0.659; T3b z ≤ 1.32 puncak garis lurus = titik akhir (0/21 027 solusi);
waktu saringan G26 PLANNED 23.2 s, TORQUE 0.1 s per rencana.

| # | Dugaan | Dasar |
|---|---|---|
| **D133** | V28 z = 1.40: **≥ 38 / 41** tuple UNSAFE | laju S16 0.69 per rencana ⇒ P(≥ 1/3) ≈ 0.97, E ≈ 39.8 |
| **D134** | laju TORQUE per rencana di V28 z = 1.40 ∈ **[0.55, 0.85]** | S16 11/16 = 0.69; P2 V28 6.45–7.54 ≈ S12 |
| **D135** | V28 z = 1.32: **20 / 20** tuple 0/3 TORQUE (torsi-saja) | T3b 0/21 027; 145/145 z < 1.40 |
| **D136** | V28 z = 1.32: **≥ 19 / 20** tuple SAFE (A4 ketat) | idem; NO-PLAN / INTERARM tidak muncul pada tuple oracle‴ G24b/G26 |
| **D137** | RNEA j2 rencana z = 1.40 bimodal: **≤ 10 %** rencana ber-lintasan di (4.0, 6.0) | celah 3.74–6.43 pada tuple yang sama (A0 G27) |
| **D138** | tiap rencana TORQUE z = 1.40: \|statis j2\| di titik puncak ≥ RNEA j2 − 0.659 pada **≥ 90 %** | vel/acc scale 0.15; C′ Δ_final (memuat dinamika) ≤ 0.659 |
| **D139** | rencana PLANNED z = 1.32: RNEA j2 puncak − \|statis j2 akhir\| ≤ 0.659 pada **≥ 95 %** | T3b z ≤ 1.32 (P₂ − statis-akhir) median 0.00; C′ |
| **D140** | rencana TORQUE z = 1.40: RNEA j2 − P2_own ∈ **[−0.5, +0.7]** pada **≥ 70 %** | C′ RNEA₂ − P₂ median −0.022; 7/11 S16 di [min P₂, maks P₂ + m₂ᵖ] |
| **D141** | (iv): **≥ 12 / 14** seed LOLOS | tugas z < 1.40 diterima oracle‴: 145/145 aman; 3/3 seed G26 tanpa z = 1.40 lolos; risiko: start non-REST, INTERARM |
| **D142** | (iv): **nol** TORQUE-UNSAFE (z apa pun) di seluruh rencana | idem |
| **D143** | wall V28 (183 rencana) **≤ 60 menit** | ~60 × 23 s (z 1.32) + ~40 × 23 s + ~83 × 0.1 s (z 1.40) ≈ 38 menit + overhead |

### A5. 🔒 Smoke test MOCK (bagian OFF) — BUKAN data

Tujuan: membuktikan alat (penyadap, penyimpanan lintasan TORQUE, K-RNEA on, resume) di `move_group`
nyata dengan `use_fake_hardware:=true`, `enable_gantry_bridge:=false`, tanpa jaringan lengan.
Tuple = **pengembangan saja** (bukan V28), dari `g27_table.json`: (a) S12 `(2033, 1, 15, 0)` arm_1
(1.2857, 0.3882, 1.40) @0.75 (G26: PLANNED 2.41 / 2.54, TORQUE 7.18); (b) S12 `(83, 1, 11, 1)` arm_2
(0.0, 0.3176, 1.40) @0.55 (TORQUE 7.39); (c) z = 1.32 G22 seed 0 t2 arm_1 (1.0, 0.3882, 1.32) @0.55
(PLANNED 3.46). 3 sampel masing-masing. Hasil disimpan terpisah (`smoke_mock_*`), **tidak** dinilai, **tidak** dipakai untuk
mengubah dugaan A4 (sudah dikunci di atas). Launch: `python3 -c "…SIG_DFL…execvp" &`, SigIgn dicek,
`ros2 node list --no-daemon` kosong di akhir, crash dump milik sesi dihapus.

🔒 **DILARANG** mengubah A1–A4 sesudah DRY / smoke / data terlihat. §B menang atas §A; pertentangan ditulis.

---

## B. Hasil terukur

> §A dikunci 2026-09-23 18:22, sha256 `124fface5d4e…` ([sectionA_locked.md](results/p1_g28/sectionA_locked.md)).

### B0. Bagian OFF (2026-09-23, operator tidak di lokasi, nol perangkat keras)

**Alat:** [v28_screen.py](results/p1_g28/v28_screen.py), [v28_score.py](results/p1_g28/v28_score.py).

**DRY** ([v28_dry.log](results/p1_g28/v28_dry.log)): KD1 ✅ (61 = 41 z 1.40 tolak + 20 z 1.32 terima, rel ∈ [0, 1.6]);
KD2 ✅ (`/tmp/reach_dwell_live.urdf` sha `f02e7c532cdb`); KD3 ✅ (7.7); **K-RNEA off: 200 lintasan sintetis,
maks \|beda\| = 0.0 — bit-identik** dengan `predict_peak_torque` (termasuk v/a kosong dan sendi asing di
`joint_names`). 61 permintaan tercetak (start: rel tuple di gantry-nya, gantry lain R1, 4 lengan REST).

**Smoke MOCK (A5)** — `use_fake_hardware:=true`, `enable_gantry_bridge:=false`; launch PID 1171859 via
`SIG_DFL…execvp`, SigIgn `0x1001000` (SIGINT tidak diabaikan); move_group siap ≤ 4 s; spawner gripper/
`gantry_2_with_arm` mati = pola baseline G23–G26. Data [smoke_mock_plans.jsonl](results/p1_g28/smoke_mock_plans.jsonl),
[log](results/p1_g28/smoke_mock.log), [penilai](results/p1_g28/smoke_mock_score.json), launch
[smoke_mock_launch.log.gz](results/p1_g28/smoke_mock_launch.log.gz). **Tuple pengembangan — BUKAN V28, tidak dinilai.**

| tuple dev | sampel | verdict | RNEA j2 | k_peak2 / N−1 | \|statis j2\| akhir | RNEA2 − P2_own |
|---|---|---|---|---|---|---|
| (a) arm_1 (1.2857, 0.3882, 1.40) @0.75 | 3 | PLANNED ×3 | 2.40 / 2.57 / 2.40 | 74/128, 75/126, 70/117 | 1.43 / 1.71 / 1.31 | — |
| (b) arm_2 (0.0, 0.3176, 1.40) @0.55 | 3 | TORQUE ×3 (15.27 / 14.99 / 15.13 corr) | 7.57 / 7.29 / 7.43 | 84/127, 78/114, 84/126 | 5.39 / 5.57 / 5.32 | −0.002 … −0.003 |
| (c) arm_1 (1.0, 0.3882, 1.32) @0.55 | 3 | PLANNED ×3 | 3.74 / 4.88 / 3.19 | 127/127, 123/127, 126/126 | 3.78 / 4.91 / 3.22 | — |

Alat terbukti: lintasan rencana **TORQUE** tersimpan (penyadap bekerja), K-RNEA on 9/9 `krnea_err` = 0.0,
resume melewati 9/9 tanpa merencana ulang. ~85 KB/rencana → V28 ≈ 16 MB. Wall PLANNED 21–27 s, TORQUE 0.1 s
(= G26).

**Pengamatan dev (sesudah kunci; tidak mengubah A4):**

1. Rencana TORQUE: puncak **interior** (k/(N−1) ≈ 0.63), statis akhir 5.32–5.57 — bentuk D132.
2. `RNEA2 − P2_own` = −0.002 … −0.003: lintasan perencana ≈ **garis lurus ruang-sendi** REST → q_akhir-nya
   sendiri (proksi G27 tepat untuk cabang yang dipilih perencana).
3. 🟡 Rencana **PLANNED** z = 1.40 (tuple a) berakhir di cabang dengan \|statis j2\| akhir **1.3–1.7**, jauh dari
   rencana TORQUE (5.3–5.6). Bila (2)+(3) berlaku umum, kelas bimodal G27 = **pilihan cabang IK tujuan** oleh
   perencana (garis lurus ke cabang j2-rendah aman; ke cabang j2-tinggi menembus), **bukan** "rute terlipat"
   (g27 B1 ⚠️). Dari n = 1 tuple per kelas → dugaan, bukan temuan.

**Dugaan tambahan — dikunci 2026-09-23 SESUDAH smoke dev, SEBELUM V28 (dihitung terpisah, "berbasis-dev"):**

| # | Dugaan | Dasar |
|---|---|---|
| **D144** | V28 z = 1.40: setiap rencana PLANNED ber-lintasan \|statis j2\| akhir **< 4.0** dan setiap rencana TORQUE **≥ 4.5** (kelas dipisah cabang tujuan) | pengamatan dev (3); o_j2 z 1.40 median 5.61 |
| **D145** | V28 semua rencana ber-lintasan: \|RNEA2 − P2_own\| ≤ 0.30 pada **≥ 80 %** | pengamatan dev (2); C′ median −0.022 |

### B0b. Diagnosis dev: cabang aman z = 1.40 = DI LUAR BATAS SENDI URDF (2026-09-23, offline)

[dev_branch.py](results/p1_g28/dev_branch.py) → [log](results/p1_g28/dev_branch.log), [json](results/p1_g28/dev_branch.json).
Kriteria ditulis di docstring **sebelum** hitung (KENDALA / LUBANG / ADA). Tuple dev saja (smoke), bukan V28.

| rencana dev z = 1.40 | pos err | miring | batas URDF | \|statis j2\| akhir | jarak ke solusi oracle‴ | kelas |
|---|---|---|---|---|---|---|
| (a) PLANNED ×3 | 1.1–1.8 mm | 0.9–2.2° | ❌ **joint_5 = −2.562 / −2.609 / +2.571** (URDF ±2.53) | 1.31–1.71 | 2.2–3.2 rad (cabang lain) | **KENDALA** |
| (b) TORQUE ×3 | 1.4–1.9 mm | 0.9–2.1° | ✅ | 5.32–5.57 | 0.040 / 0.064 / 0.091 rad, statis sama | ADA (2× "LUBANG" = artefak ambang: grid roll 5° = 0.087 rad) |

Oracle‴ tuple (a): 180 solusi, statis j2 [5.33, 5.52], garis lurus P2 [6.44, 7.32] — **tidak ada** cabang j2-rendah
**dalam batas URDF**. Cabang aman yang dipakai perencana hanya ada karena joint_5 melewati ±2.53.

**Sebab:** `workcell_moveit_config/config/joint_limits.yaml` (sejak 2026-06, `877a4fc` terakhir) menimpa batas posisi
URDF dengan batas yang **lebih longgar** pada 6 sendi — MoveIt merencanakan dengan yaml:

| sendi | URDF (vendor) | yaml MoveIt |
|---|---|---|
| `t1_a1_joint_2`, `t2_a1_joint_2`, `t2_a2_joint_2` | ±2.61 | **±2.76** |
| `t1_a1_joint_5` | ±2.53 | **±2.70** |
| `t2_a1_joint_3`, `t2_a2_joint_3` | ±2.61 | **±2.85** |

(`arm_2` tidak ada; tidak seragam antar-lengan. Mock ros2_control j5 ±2.57 — berbeda lagi.)

**Riwayat eksekusi aman:** rekaman `/joint_states` G24b + G26 (`/tmp/g24b_js.csv`, `/tmp/g26_js.csv`): maks \|q\| tiap sendi
tiap lengan **di dalam** URDF (terdekat t2_a2_j6 2.509 / 2.60, t1_a1_j1 2.490 / 2.68); C′ (81 dieksekusi g18–g20) q_akhir
0 pelanggaran. Tidak ada gerak nyata yang pernah melewati batas vendor. Probe/saringan **tidak** memeriksa batas URDF.

**Akibat (dari n = 1 tuple; V28 akan mengukur):**
1. g27 B1 "rute terlipat aman ada (5/16)" **kemungkinan salah**: rencana PLANNED z = 1.40 = titik akhir **di luar batas
   sendi vendor**, bukan rute lain ke cabang sah. Oracle‴ benar memakai batas URDF.
2. Kelas SAFE di V28 z = 1.40 bisa berisi rencana ilegal → `v28_score` kini melaporkan pelanggaran batas URDF per
   (z, verdict) — **tambahan pasca-kunci, informatif**, tidak mengubah A4/D.
3. D144 ("PLANNED z 1.40 statis akhir < 4.0") kini punya penjelas: cabang j2-rendah = cabang j5 > 2.53.
4. 🔴 Sebelum **eksekusi** apa pun berikutnya: rencana yang lolos saringan dapat memerintah sendi melewati batas
   vendor. Perbaikan (yaml ≤ URDF) = keputusan operator (§C).

### B0c. Tahap 0 G28-ON (2026-09-28, sel NYATA, operator di lokasi)

| Gerbang | Hasil |
|---|---|
| operator (ditanya ULANG) | LED 4/4 normal; origin g1 + g2 = home; sel/ruang kosong. G30 di-commit dulu (`3084a0b`, izin operator) |
| `remount_check.py` + ICMP | **GERBANG LULUS**; .10–.13 ✅; nol stack sisa (hanya `ros2-daemon`); enp112s0 = 192.168.2.100 |
| bring-up `use_fake_hardware:=false enable_gantry_bridge:=true use_sim_time:=false` | launch PID **1334927** (dicari ulang via pgrep sesudah controller naik, PPID 1), SigIgn `0x1001000` (SIGINT tidak diabaikan); **4×** "Actuator count … '6'"; **7/7** controller active; 7 spawner "process has died" + KDL "not a chain" `gantry_*_with_arm` = baseline; nol fault ([launch log](results/p1_g28/g28on_launch.log.gz)) |
| `/joint_states` (40 pesan) | rel **0.000712 / 0.000681** → **R1 = 0.00 / 0.00** (kandidat (iv) **tidak** dibuat ulang); rot **0.000° / −0.290°** (= akhir G30, ≤ 0.5°) |
| DRY ON ([log](results/p1_g28/v28_dry_on.log)) | KD1 ✅ KD2 ✅ (`f02e7c532cdb`) KD3 ✅; **K-RNEA off 200/200 bit-identik** |
| disk | **2.1 GB** bebas di awal (prompt ~3; S27 ≥ 1.5 ✅) |

**Nol gerak** lengan / rel / rotasi sepanjang sesi (plan-only).

### B1. V28 — 61 tuple × 3 = **183 rencana** ([v28_screen.log](results/p1_g28/v28_screen.log), [v28_plans.jsonl.gz](results/p1_g28/v28_plans.jsonl.gz), [v28_score.log](results/p1_g28/v28_score.log), [json](results/p1_g28/v28_score.json))

Wall sesi **1971 s (32.9 menit)**, Σ wall rencana 32.8 menit; rc 0; sampel lengkap 183/183.
**K-RNEA on: 183/183 rencana ber-lintasan, maks `krnea_err` = 0** (bit-identik dengan puncak probe tersadap).
Kelas LAIN: **0** (nol NO-PLAN / INTERARM / TUCK / UNSCREENED).

| z | rencana | PLANNED | TORQUE-UNSAFE |
|---|---|---|---|
| 1.32 (oracle⁗ terima, 20 tuple) | 60 | **60** | 0 |
| 1.40 (oracle⁗ tolak, 41 tuple) | 123 | 21 | **102** (0.829) |

**A4 ketat:** tolak ⇒ UNSAFE **39/41** (salah: tuple **5**, **40** = SAFE 3/3, keduanya arm_1); terima ⇒ SAFE **20/20**.
Presisi terima **1.000**; recall SAFE 20/22 = **0.909**; tolak-benar 39/41. Torsi-saja terima ⇒ 0/3 TORQUE 20/20.
➜ **A4 tidak berlaku penuh** (2 tolak-palsu) — tetapi lihat batas URDF di bawah: keenam rencana tuple 5 dan 40 keluar batas vendor.

**Posisi puncak RNEA j2:** z 1.32 — interior 22 / akhir 38, `RNEA2 − statis_akhir` median −0.032, maks +0.001 (puncak = titik akhir,
= T3b G27); z 1.40 — **interior 123/123**, `k/(N−1)` median 0.70, `RNEA2 − statis_akhir` median **+1.228**, maks +2.156
(= transit G27 B1, kini 123/123 di stack nyata). `RNEA2 − P2_own` TORQUE z 1.40: median −0.002, [−0.007, −0.001] — lintasan
perencana = garis lurus ruang-sendi ke q_akhir-nya sendiri.

**Pelanggaran batas URDF vendor (informatif, pasca-kunci B0b; lintasan titik mana pun):**

| z / verdict | keluar URDF | sendi | per lengan |
|---|---|---|---|
| 1.32 PLANNED | **0 / 60** | — | — |
| 1.40 PLANNED | **20 / 21** | j5 (arm_1, 16), j3 (arm_3, 4) | arm_1 16/16, arm_2 **0/1**, arm_3 4/4 |
| 1.40 TORQUE-UNSAFE | 20 / 102 | j3 (arm_3 13, arm_4 7), ≤ 0.053 rad | — |

Kelas PLANNED z = 1.40 per lengan (sendi yaml > URDF di kurung): **arm_1 (j2, j5) 16/30**, arm_2 (tidak ada) **1/30**,
arm_3 (j2, j3) 4/39, arm_4 (j2, j3) 0/24. Kelas "aman" z = 1.40 **hampir seluruhnya = efek batas yaml yang lebih longgar**
(B0b terkonfirmasi 20/21; kelebihan 0.006–0.073 rad).

Tiga jenis rencana PLANNED z = 1.40:
1. **j2-rendah lewat j5 > 2.53** (arm_1, 16): statis akhir 0.86–2.10, RNEA 2.23–2.81 — pola tuple dev (a).
2. **j2-tinggi di bawah ambang** (arm_3: 18/s2, 30/s2, 31/s3): statis akhir **5.32–5.34**, RNEA **6.22–6.28** = 0.02–0.08 di
   bawah 6.30 (prediksi **13.92–13.98 N·m / rating 14 = 99.4–99.9 %**); j3 keluar URDF. Ini yang menjatuhkan D144.
3. 🟡 **j2-rendah DALAM URDF** (arm_2, tuple 4 sampel 1, rel 0.90): statis akhir **1.48**, RNEA 2.43, nol pelanggaran. Satu
   rencana; sampel 2–3 tuple yang sama TORQUE. Membantah generalisasi B0b ("tidak ada cabang j2-rendah dalam URDF", n = 1 arm_1)
   untuk arm_2 — cabang sah **ada** kadang-kadang; oracle⁗ (PATH ∀-solusi) menolaknya.

**G27 B1 "rute terlipat aman ada (5/16)"**: di V28, 20/21 rencana aman z = 1.40 memerlukan batas yaml > URDF; 1/21 sah.
Klaim itu **salah untuk 20/21**, benar untuk 1 (arm_2).

### B2. Saringan (iv) — **14/14 seed LOLOS** ([g28_screen.log](results/p1_g28/g28_screen.log), [json](results/p1_g28/g28_screen.json))

Seed 16 17 18 20 21 22 23 26 29 33 35 36 40 42 — **semuanya PLANNED/PLANNED/PLANNED**; **252/252** rencana tugas PLANNED, nol
TORQUE / NO-PLAN / INTERARM / TUCK; rc 0; wall ≈ 110 menit (11:27–13:17). Traverse/retract: semua CLEAR, minimum terbaca
antar-gantry **326.8 mm** (retract), traverse **386.0 mm**. Prediksi G27 A5 (seed tanpa z = 1.40 lolos) — **14/14**.
Gabung G26: seed tanpa z = 1.40 lolos **17/17** (1, 2, 13 + 14).
`g28_screen.json` tidak menyimpan lintasan → pelanggaran batas URDF di (iv) **tidak terukur** sesi ini (lihat §C).

### B3. Papan skor D132–D143 — **12 / 12 tepat** (+ D144–D145 berbasis-dev terpisah: 1 / 2)

| # | Dugaan | Hasil | |
|---|---|---|---|
| D132 | z 1.40 RNEA j2 > 6.30 ⇒ puncak interior ∧ statis akhir ≤ 5.83 | 102/102 | ✅ |
| D133 | ≥ 38/41 tuple z 1.40 UNSAFE | 39/41 | ✅ |
| D134 | laju TORQUE z 1.40 ∈ [0.55, 0.85] | 0.829 | ✅ |
| D135 | 20/20 tuple z 1.32 0/3 TORQUE | 20/20 | ✅ |
| D136 | ≥ 19/20 tuple z 1.32 SAFE | 20/20 | ✅ |
| D137 | ≤ 10 % rencana z 1.40 RNEA j2 ∈ (4, 6) | 0/123 | ✅ |
| D138 | ≥ 90 % TORQUE: statis@puncak ≥ RNEA − 0.659 | 102/102 | ✅ |
| D139 | ≥ 95 % PLANNED z 1.32: RNEA2 − statis akhir ≤ 0.659 | 60/60 | ✅ |
| D140 | ≥ 70 % TORQUE: RNEA2 − P2_own ∈ [−0.5, +0.7] | 102/102 (median −0.002) | ✅ |
| D141 | (iv) ≥ 12/14 lolos | 14/14 | ✅ |
| D142 | (iv) nol TORQUE | 0 | ✅ |
| D143 | V28 ≤ 60 menit | sesi 32.9 / Σ rencana 32.8 menit | ✅ |
| D144 (dev) | PLANNED z 1.40 statis akhir < 4.0 ∧ TORQUE ≥ 4.5 | 3 PLANNED arm_3 di 5.32–5.34 | ❌ |
| D145 (dev) | ≥ 80 % \|RNEA2 − P2_own\| ≤ 0.30 | 183/183 | ✅ |

D144 salah karena **ambang**, bukan karena cabang: cabang j2-tinggi tidak otomatis TORQUE — tiga rencana lolos dengan
margin 0.02–0.08 N·m. Bagian "cabang j2-rendah ⇒ aman" benar 18/18.

### B5. Pertentangan §B lawan §A — bagian OFF

| # | Pertentangan |
|---|---|
| (1) | D143 dinilai `v28_score` sebagai Σ wall **per rencana** (tanpa overhead start/ROS); wall sesi dicetak alat di akhir — G28-ON melaporkan keduanya, D143 dinilai pada wall **sesi** |
| (2) | D144–D145 ditulis sesudah melihat data smoke dev (tidak ada di A4) — dihitung terpisah dari tally utama, seperti D107 G25 |
| (3) | Penyadap `_violates_tuck` diteruskan dengan `*args` (tanda tangan asli punya `tol_deg=10.0` opsional) — `_plan_and_screen` memanggilnya dengan 2 argumen; nilai balik identik |
| (4) | `v28_score` mendapat laporan pelanggaran batas URDF **sesudah** §A dikunci (B0b) — informatif, di luar A2/A4 |

**Bagian ON (2026-09-28):**

| # | Pertentangan |
|---|---|
| (5) | Start V28 / (iv) = rotasi **0 / 0 ditempatkan** (A1), sel nyata g2 = −0.29° (sisa G30). Plan-only dengan start eksplisit → tidak memengaruhi perencanaan; dicatat |
| (6) | G29 C-1 ditemukan **sesudah** §A: `CrossGantryChecker` (INTERARM + CLEAR traverse/retract) memakai hull palsu, jarak terbaca ≤ +78 mm terlalu jauh. Nol penolakan INTERARM di V28/(iv); minimum terbaca 326.8 mm − 78 = 248.8 ≫ margin 50 → **verdict tidak berubah** walau kasus terburuk. Tidak dipatch (operator G29/G30: laporkan saja) |
| (7) | Prompt §D langkah 7 "Prompt G29" **basi**: G29 (offline) dan G30 (HW rotasi) dikerjakan **sebelum** G28-ON. Berikutnya = sesi yaml ≤ URDF (§F), lalu G31 ([g30 §D](p1_g30_rot_hw.md)); gerbang G1 G31 kini terjawab (G28-ON selesai) |
| (8) | `v28_plans.jsonl` di-gzip (15.3 → 5.8 MB, prompt langkah 6); `v28_score.py` membaca berkas tak-terkompresi → reproduksi: `gunzip -k v28_plans.jsonl.gz` dulu |
| (9) | Disk awal 2.1 GB (prompt ~3); cukup (S27 ≥ 1.5) |
| (10) | D143 dinilai pada wall **sesi** (B5 (1)): 1971 s = 32.9 menit; Σ per rencana 32.8 — keduanya ≤ 60 |
| (11) | "Kebenaran" A2 memakai verdict perencana dengan batas **yaml**; 2 tolak-palsu A4 (tuple 5, 40) = 6/6 rencana di luar batas vendor. Dengan yaml ≤ URDF keduanya **tidak lagi SAFE** (kelas baru tidak diukur: bisa UNSAFE atau LAIN) — informatif, A4 tetap dinilai apa adanya (39/41) |

---

## C. Keadaan akhir bagian OFF (Rule 12)

- **Nol perangkat keras.** Satu stack **mock** diluncurkan (smoke A5) dan dimatikan: SIGINT → keluar 12 s
  (launch meng-eskalasi SIGKILL ke `move_group` / `rviz2`); `ros2 node list --no-daemon` kosong; nol proses sisa.
  Crash dump `move_group` 18:27 (199 MB, milik sesi) **dihapus**; dump python 09-21 (bukan milik sesi) dibiarkan.
  Disk **3.6 GB**.
- **Tidak diubah:** probe, `sched_screen`, `g22_plan`, oracle, peta, kandidat G26. **Baru:** `docs/results/p1_g28/`.
- **Belum dikerjakan (butuh operator):** tahap 0, R1, V28, (iv), penilaian A2/A4 + D144–D145, p1_state tally,
  prompt G29.
- Commit: G27 `2b7813f`, G28-OFF `9cef07c`; B0b sesudahnya.
- 🔴 **Keputusan operator (2026-09-23, sesudah B0b):** V28 memakai perencana **apa adanya** (§A tetap); `joint_limits.yaml`
  diperbaiki (≤ URDF) **sesudah** G28-ON dan **sebelum eksekusi apa pun** berikutnya — gerbang wajib.
- 📣 **Permintaan operator (2026-09-23):** laporkan bila batas ≤ URDF membuat gerak **sering gagal**. Ukuran wajib:
  (1) G28-ON — fraksi rencana PLANNED (per z) yang keluar batas URDF di titik mana pun (`v28_score`, pelanggaran
  batas URDF) = perkiraan rencana yang **akan hilang**; (2) sesi perbaikan yaml — saringan sebelum/sesudah pada
  set yang sama (V28 z = 1.32 + seed (iv) G28): laju PLANNED / NO-PLAN / waktu rencana, dilaporkan ke operator
  secara eksplisit walau kecil. Bukti sejauh ini: eksekusi G24b/G26 + C′ = 0 pelanggaran (B0b).

### C-ON. Keadaan akhir bagian ON (2026-09-28, Rule 12)

- **Nol gerak.** Stack nyata dimatikan: SIGINT ke launch 1334927 → keluar **12 s**; `ros2 node list --no-daemon` **0**; nol proses
  ROS sisa. Crash dump `move_group` 13:18 (198 MB, milik sesi) **dihapus**; dump python 09-21 (`table_keyboard.py`, bukan milik
  sesi) dibiarkan. Disk akhir **1.7 GB**.
- **Keadaan sel:** tidak berubah dari akhir G30 (rel ≈ 0.7 mm, rot 0.00 / −0.29°, lengan REST/bring-up).
- **Tidak diubah:** probe, `sched_screen`, `g22_plan`, oracle, peta, kandidat G26, `joint_limits.yaml`, `interarm_collision.py`,
  §A. **Baru:** `v28_dry_on.log`, `v28_screen.log`, `v28_plans.jsonl.gz`, `v28_score.{log,json}`, `g28_screen.{log,json}`,
  `g28on_launch.log.gz`.
- **Tidak ada eksekusi jadwal** (operator tidak meminta; gerbang yaml ≤ URDF juga menutupnya).
- **Tidak diukur:** pelanggaran batas URDF pada rencana (iv) (saringan tidak menyimpan lintasan); kelas V28 **sesudah** yaml ≤ URDF.

📣 **LAPORAN KE OPERATOR (permintaan §C, ukuran (1)) — fraksi PLANNED yang keluar batas URDF = rencana yang akan hilang
sesudah yaml ≤ URDF:**

| z | PLANNED keluar URDF | Arti |
|---|---|---|
| **1.32** (dan lapis yang dipakai jadwal: oracle⁗ terima) | **0 / 60 (0 %)** | tidak ada rencana tugas yang akan hilang di set ini |
| **1.40** (oracle⁗ sudah menolak semuanya) | **20 / 21 (95 %)** | "aman" z = 1.40 ≈ artefak batas yaml; hilang tidak mengubah jadwal (oracle⁗ tidak memakai z = 1.40) |

Bukti tambahan: eksekusi G24b/G26 + C′ 0 pelanggaran (B0b). Ukuran (2) (sebelum/sesudah pada V28 z 1.32 + seed (iv)) = sesi §F.
**Perkiraan:** yaml ≤ URDF **tidak** membuat gerak sering gagal di lapis z ≤ 1.32; yang hilang = rencana z = 1.40 yang memang ilegal.

**Opsi untuk keputusan operator (bukan G29/G31):** D144 **salah** (ambang), tetapi rencana arm_2 tuple 4 menunjukkan cabang
j2-rendah **sah** kadang ada di z = 1.40. "Oracle per-cabang + kendala cabang tujuan di perencana" dapat menyelamatkan sebagian
z = 1.40 — hanya bila cabang sah itu ada (arm_2 1/30 rencana; arm_1 0 sah). Nilai kecil; tidak dikerjakan.

## F. Prompt G28-YAML (salin ke chat BARU) — `joint_limits.yaml` ≤ URDF, PLAN-ONLY, stack NYATA atau mock

**Rekomendasi: Opus, effort SEDANG** — perubahan kecil (6 angka yaml) dengan kebenaran dasar jelas (URDF vendor) dan
saringan sebelum/sesudah yang sudah ada; naikkan ke TINGGI bila laju NO-PLAN z ≤ 1.32 naik > 5 % (butuh diagnosis).

```
Sesi G28-YAML -- joint_limits.yaml <= URDF vendor (gerbang G28 B0b/C sebelum eksekusi rencana lengan APA PUN).
Repo ceiling_arm, branch feat/rgbd-topo-deploy. PLAN-ONLY.

BACA PENUH: CLAUDE.md; docs/p1_g28_hw.md B0b, B1, C, C-ON, F; workcell_moveit_config/config/joint_limits.yaml;
URDF vendor gen3_lite (batas posisi); docs/results/p1_g28/v28_screen.py, v28_score.py (bagian urdf_viol).

GERBANG: tanya operator (a) stack NYATA atau mock (plan-only; mock cukup bila ros2_control tidak dipakai perencana),
(b) izin ubah joint_limits.yaml (6 sendi B0b: t1_a1_j2/j5, t2_a1_j2/j3, t2_a2_j2/j3; periksa SEMUA 24 sendi lengan,
bukan hanya 6), (c) batas = URDF persis atau URDF - margin (mis. 0.01 rad).
1. INSTRUMEN: tabel 24 sendi URDF vs yaml vs mock ros2_control; kunci §A (dugaan D167+ prior tally: mekanisme terukur
   tepat, kode sendiri meleset) SEBELUM ubah.
2. SEBELUM (yaml lama, bila belum ada data di stack yang sama): V28 z 1.32 (20 tuple x 3) + (iv) seed G28 (14) -- laju
   PLANNED / NO-PLAN / waktu rencana. Data G28-ON (stack nyata) boleh dipakai sebagai SEBELUM bila stack sama.
3. Ubah yaml (izin b) -> rebuild (symlink) -> SESUDAH: set sama. Plus V28 z 1.40 (41 x 3) untuk kelas baru tuple 5/40.
   Kontrol: v28_score urdf_viol HARUS 0 pada semua rencana SESUDAH.
4. LAPORKAN KE OPERATOR (permintaan G28 C, ukuran (2)): laju PLANNED/NO-PLAN/waktu sebelum vs sesudah per z, walau kecil.
5. p1_state + tally; berikutnya G31 (g30 §D; G1 terjawab).
ATURAN: nol gerak; B menang atas A dan DITULIS; stack dimatikan bersih (PID launch ASLI via pgrep sesudah controller naik,
SigIgn, node list --no-daemon 0, crash dump milikmu dihapus); disk ~1.7 GB -- gzip jsonl > 10 MB.
```

## D. Prompt G28-ON (salin ke chat BARU saat operator di lokasi) — HW, PLAN-ONLY

**Rekomendasi: Opus, effort SEDANG** — alat, gerbang, dan penilai sudah dikunci dan diuji (DRY + smoke mock);
sisa sesi = operasi tahap 0 + menjalankan + membaca keluaran penilai. Naikkan ke TINGGI bila V28 memberi
kelas LAIN / K-RNEA on gagal (butuh diagnosis baru).

```
Sesi G28-ON -- V28 + saringan (iv) di stack NYATA, PLAN-ONLY. Repo ceiling_arm, branch
feat/rgbd-topo-deploy. Operator di lokasi. Bagian OFF SELESAI: docs/p1_g28_hw.md §A (dikunci 18:22,
sha 124fface5d4e), §B0 (DRY, smoke mock, D144-D145), §D.

BACA PENUH: CLAUDE.md; docs/p1_g28_hw.md (semua, terutama B0b); docs/p1_g27_z140_diag.md A4, B1-B2, D;
docs/p1_g22_hw.md A5 (tahap 0); docs/p1_g26_hw.md B0 (contoh tahap 0).

1. Tahap 0 g22 A5: tanya operator ULANG (LED 4/4, origin g1+g2 = home, sel kosong); remount_check.py;
   bring-up enable_gantry_bridge:=true via python3 -c "...SIG_DFL...execvp" &, cek SigIgn; 4x Actuator
   count; 7/7 controller. R1 = rel terbaca /joint_states, grid 0.05 (harapan 0.00/0.00; lain -> A3 buat ulang).
2. cd docs/results/p1_g28; python3 v28_screen.py --dry --r1 <R1>  (KD1-KD3 + K-RNEA off harus lulus)
3. python3 v28_screen.py --r1 <R1> 2>&1 | tee v28_screen.log   (~40-60 menit; resume aman bila putus)
4. python3 ../p1_g22/sched_screen.py --plan ../p1_g26/g26_candidates.json --repeats 3
   --out g28_screen.json --seeds 16 17 18 20 21 22 23 26 29 33 35 36 40 42 2>&1 | tee g28_screen.log
5. python3 v28_score.py | tee v28_score.log  -> isi §B1 (V28: A4 ketat, presisi/recall, D132, posisi
   puncak per z, pelanggaran batas URDF per (z, verdict) -- B0b: SAFE z=1.40 bisa ilegal), §B2 ((iv)), §B3 papan skor D132-D143 (+ D144-D145 terpisah), §B5, §C.
6. Tidak ada eksekusi jadwal kecuali operator minta. Matikan stack: kill -INT launch, node list --no-daemon
   kosong, hapus crash dump milikmu. gzip v28_plans.jsonl bila > 10 MB.
7. p1_state (blok G28 + tally). GERBANG sebelum eksekusi apa pun: joint_limits.yaml <= URDF (B0b,
   keputusan operator) -- sesi tersendiri sesudah G28-ON. LAPORKAN KE OPERATOR (permintaannya, §C): fraksi
   PLANNED yang keluar batas URDF per z = rencana yang akan hilang sesudah yaml <= URDF. Prompt G29 = ROTASI gantry offline (g26 C1-2 a-d), lalu 3D map (C1-3).
   Bila D144 benar: catat opsi "oracle per-cabang + kendala cabang tujuan di perencana" untuk
   menyelamatkan z = 1.40 (keputusan operator, bukan G29).

ATURAN: B menang atas A dan DITULIS; nol gerak lengan; §A G28 TIDAK diubah; disk ~3 GB.
```

## E. Prompt G29 (salin ke chat BARU) — ROTASI gantry, OFFLINE (boleh sebelum G28-ON)

**Kenapa ini berikutnya.** Urutan operator (g26 C1): sesudah z = 1.40 → **ROTASI gantry**, lalu peta 3D. Seluruh
G22–G28 = masalah rel 1-D (rotasi 0). Bagian (a), (b), (d) g26 C1-2 murni hitungan (pinocchio/URDF): lengan
**mati**, operator tidak perlu di lokasi. (c) = keputusan operator; (e) = HW, sesi lain. G29 **tidak bergantung**
pada hasil V28 (oracle⁗ membuang z = 1.40 apa pun hasilnya) — boleh dikerjakan sebelum G28-ON.

**Yang sudah diketahui (dicek 2026-09-23, jangan diulang, tapi verifikasi di A0):**
- `CrossGantryChecker` (scripts/interarm_collision.py) **membuang** pasangan struktur–struktur antar-gantry
  (platform, link rotasi, pelat mount) dengan asumsi "rel sejajar di y ±0.36 → hanya bertranslasi, jarak tetap
  429 mm". Asumsi itu **batal** pada rotasi ≠ 0 → S24 yang ada **buta** terhadap tabrakan struktur ber-rotasi.
- `g22_plan.sweep_screen` (S24) hanya menyapu `t{g}_linear_joint`; `sched_screen.walk`, `run_g22`, `v28_screen`
  menaruh `t{g}_rotation_joint = 0.0` keras. `make_instance_g24` memakai `restrict_rot0` (`REF.rot`, ROT0).
- URDF `rotation_joint` revolute ±π (moving_table.urdf.xacro:77–82); motor G3: 10.0 °/s @ speed 1000, 1000 °/s².
- Model tabrakan terkopel G9 (A2, `sched_coll`): gerbang ground truth W0–W4 **TIDAK LULUS** (g9 B4) → jangan
  dipakai sebagai kebenaran.
- `oracle2.model` mengunci semua sendi selain rel + 6 sendi lengan pada acuan G23 (rotasi 0).

**Rekomendasi: Opus, effort TINGGI** — geometri + penyaring baru; kesalahan paling mungkin **diam** (penyaring
yang salah tetap bilang CLEAR, seperti asumsi rel-sejajar di atas). Kontrol positif wajib.

```
Sesi G29 -- ROTASI gantry, OFFLINE (g26 C1-2 a, b, d). Nol HW; lengan mati. Repo ceiling_arm, branch
feat/rgbd-topo-deploy. Stack mock (use_fake_hardware:=true) boleh HANYA untuk menguji alat.

BACA PENUH: CLAUDE.md; docs/p1_g26_hw.md C1; docs/p1_g28_hw.md B0b, C, E; docs/p1_g9_sched3.md A2, B1-B4, B9;
docs/p1_g3_timing.md (rotasi); scripts/interarm_collision.py (CrossGantryChecker, docstring struktur);
docs/results/p1_g22/g22_plan.py; docs/results/p1_g24/oracle2.py, make_instance_g24.py; p1_state 5-6.

0. INSTRUMEN sebelum kunci: grep setiap tempat rotasi = 0 diasumsikan (rotation_joint, rot0, ROT0,
   restrict_rot0, "0.0" rotasi di start_joints) -> tabel. Geometri: sumbu rotasi, pusat, jari-jari platform
   + mount + lengan REST; jarak struktur antar-gantry vs (x1 - x2, rot1, rot2). Apa yang dicakup REF.rot.
   Batas rotasi di URDF / joint_limits.yaml / bridge / dual_table_controller (bandingkan; lihat B0b G28:
   yaml bisa lebih longgar dari URDF).
1. KUNCI §A (sebelum hitung): hipotesis, kontrol, aturan, dugaan D146+ (prior tally: mekanisme terukur tepat,
   kode sendiri meleset, "kendala lebih longgar" meleset).
2. (a) Penyaring S18/S24 BERROTASI -- modul BARU (alat G22 tidak diubah): sapuan (lin, rot) <= 10 mm / <= 1 deg,
   pasangan struktur-struktur antar-gantry IKUT dihitung bila rot != 0.
   Kontrol NEGATIF: rot = 0 -> jarak lengan-lengan identik dengan CrossGantryChecker lama pada sapuan arsip
   G22-G26. Kontrol POSITIF: tabrakan yang dibangun sengaja (dua gantry diputar saling menghadap di x sama)
   HARUS terdeteksi; juga lengan-vs-struktur gantry lain.
3. (b) Peta jarak struktur gantry-gantry atas grid (x1 - x2, rot1, rot2), lengan REST (dan lengan di tugas bila
   relevan): amplop rotasi AMAN (mis. |rot| <= theta*(dx)), hull vs mesh dicatat (g20 B0: hull bisa +0.4 mm).
4. (d) oracle''' dengan rotasi != 0: argumen rot opsional di oracle2 (DEFAULT TIDAK BERUBAH; K0 = rot 0
   bit-identik dengan g24_oracle3_cache pada >= 72 tuple). Sampel tuple per rot grid: berapa node/tugas
   baru jadi layak vs rot = 0 (nilai rotasi bagi scheduler), + PATH (oracle'''') bila z = 1.40 ikut.
5. (c) kecepatan ujung lengan pada 10 deg/s (REST dan pose terjauh) -> TABEL untuk operator; JANGAN putuskan.
6. Laporan §B (B menang atas A, DITULIS), §C Rule 12, p1_state 6 + tally, prompt G30 = HW tahap (e):
   +-10 deg lengan REST dulu. GERBANG sebelum gerak apa pun: joint_limits.yaml <= URDF (G28 B0b) + G28-ON
   selesai atau ditunda eksplisit oleh operator.

ATURAN: nol HW; sched.py / sched_coll / alat G22 / probe / peta TIDAK diubah (modul baru saja); oracle2 hanya
argumen opsional dengan default identik; disk ~3 GB; stack mock dimatikan, node list --no-daemon kosong,
crash dump milikmu dihapus.
```
