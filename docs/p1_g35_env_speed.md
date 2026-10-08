# P1 / G35 — OVERHEAD PENYARING LINGKUNGAN: ukur, percepat TANPA mengubah verdict (OFFLINE, nol gerak)

> Sesi G35, 2026-10-08. Lanjutan [p1_g34_rail_calib.md](p1_g34_rail_calib.md) §B7.4 (Δ R10 − R0 = +65.96 s vs model
> −50.87 s; traverse berotasi 55.3 / 81.1 s SEBELUM kirim), §B8.1 (G34b lupa mengunci dugaan).
> **§A ditulis dan DIKUNCI SEBELUM profil apa pun dijalankan.** §B diisi sesudah. §B menang atas §A; konflik DITULIS.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan, B8.1)

- Kode: `scripts/env_collision.py` (`EnvChecker.check` = 52 geometri × `coal.distance` ke octree 136 558 voxel per
  sampel; `screen_trajectory` = semua sampel, semua geometri, tanpa cache), `pose_to_g.py` (S28 `sweep_rot` lalu G33
  `EnvChecker().screen_trajectory(rect_points)`, EnvChecker dibangun per proses), `reach_dwell_probe.screen_env`
  (EnvChecker di-cache pada node, per proses), `return_rest.main` (EnvChecker per proses).
- **Satu log HW**: `hw/g34hw_R10_ev07_traverse.log` — stempel S28 → G33 = 846.61 → 891.28 (**44.7 s**, n 671 rect),
  start → S28 = 8.6 s. Dugaan D226 di bawah ditulis SESUDAH melihat angka ini (dibulatkan, bukan diuji ulang olehnya).
- Arsip lintasan: plan-only jsonl G33/G34/G34b (titik sendi lengan, 47–142 titik), V28 183 rencana (`v28_plans.jsonl.gz`),
  kontrol `g33_controls.py` (jendela keadaan TERUKUR G32-HW, `ec.check` per konfigurasi).

### A1. 🔒 Profil (offline, mesin yang sama dengan HW, tanpa ROS graph)

Per alat (`pose_to_g` traverse, `reach_dwell_probe` tugas, `return_rest` retract): waktu (a) impor + bangun EnvChecker
(URDF, 52 true_hull, octree), (b) per sampel `check`, (c) jumlah sampel dari log HW G34b. Rekonstruksi
pra-kirim = (a) + n × (b) + penyaring lain (S28 / antar-lengan / torsi / MoveIt diukur dari stempel log) dicocokkan
dengan B7.4 dan stempel log per event **±10 %**.

### A2. 🔒 Percepatan — hanya yang EKSAK

1. **Branch-and-bound Lipschitz** di `screen_trajectory`: d(geometri, sampel k) ≥ d(sampel j) − perpindahan maks
   titik geometri j→k (|Δt| + 2 sin(θ/2)·r_maks, r_maks dari verteks hull lokal). Pasangan (geometri, k) dilewati hanya
   bila batas bawah − slack > 0 DAN ≥ minimum sementara → minimum global, geometri, indeks = sama seperti kode lama.
   Badan diam (perpindahan 0) otomatis dihitung sekali.
2. Muat sekali / cache bila (a) dominan (mis. hull disimpan) — hasil geometri harus identik.
3. **Sampel sapuan TIDAK dijarangkan** (rect_points, titik MoveIt tetap). MARGIN = TOLAK tidak berubah.

### A3. 🔒 Regresi WAJIB (lama vs baru, berdampingan, mesin sama)

Semua lintasan arsip: plan-only jsonl G33 (R0/R10), G34 (reg3/g34/g34n/g34b_hw, R0/R10), V28 183, sapuan traverse
seed 36 R0/R10 (rect_points dari keadaan HW G34b), retract lurus G34b, + jendela kontrol `g33_controls` (P+, N−, R10
ev0–7) sebagai lintasan. Lulus bila **verdict identik 100 %**, |Δ d_min| ≤ 0.5 mm, geometri terburuk identik, dan
**(P+) ev 8 COLLIDE tetap, tolak ≥ 1.7 s sebelum kontak** (prefix). Gagal → kode lama dikembalikan, DITULIS.

### A4. 🔒 Dugaan (SEBELUM profil)

| # | Dugaan |
|---|---|
| D225 | Bangun EnvChecker 2–5 s per proses; hull (52 × scipy ConvexHull) > 50 % dari itu, octree < 1 s. |
| D226 | `check` per sampel (52 geometri) 50–80 ms, hampir sama antar alat (biaya = jumlah geometri, bukan jenis lintasan). |
| D227 | Rekonstruksi (a) + n×(b) + penyaring lain cocok dengan pra-kirim B7.4 per event ±10 %. |
| D228 | B&B eksak ≥ 5× pada sapuan traverse berotasi (671/946) dan ≥ 3× pada lintasan tugas (~128 titik). |
| D229 | Regresi A3 lulus 100 % (verdict, geometri, d_min bit-identik). |
| D230 | Sesudah percepatan: overhead lingkungan ≤ 10 s per traverse berotasi, ≤ 3 s per tugas; Δ(R10 − R0) prediksi **negatif**. |
| D231 | Sesudah percepatan, overhead terbesar yang tersisa BUKAN lingkungan (S28 / antar-lengan / start proses). |

## B. Hasil terukur

> Data + skrip: `docs/results/p1_g35/` — `profile_env.{py,log,json}`, `hw_replay.py` (+ `hw_replay_{old,new}.{json,log}`),
> `analyze.{py,log,json}`, `regress.py` (+ `regress_*.{json,log}`), `env_collision_old.py` (salinan beku G34, referensi),
> `sectionA_locked.md` (sha256 `8e399eef…`). Mesin = NUC yang sama dengan HW, tanpa ROS graph, nol gerak.

### B1. Profil (A1)

**Muat EnvChecker = 3.9–4.2 s per proses** (profil: impor 0.13, URDF model 0.003, `buildGeomFromUrdf` (mesh) **2.86**,
52 true_hull **1.42**, peta 0.007, octree **0.035**). Dibayar di SETIAP alat (tiap event = proses baru), sebelum sampel pertama.

**Per sampel** (52 geometri × `coal.distance` ke octree, offline): **30–62 ms**, bergantung KONFIGURASI, bukan alat —
semua lengan REST 29.8 ms (datar, 800 panggilan, tanpa drift), sapuan rel 32–40, tugas 38–62, sapuan rot 52–60.
Sumbernya jumlah: sapuan rot ev 7 / ev 12 = **671 / 957 sampel** (rect 10 mm × 1°) vs 21–127 rel-saja.

**Fase pra-kirim HW G34b** (stempel log, `analyze`/`hw_replay`):

| | R0 | R10 |
|---|---|---|
| traverse: start / S28 / **lingkungan** | 6.7 / 22.2 / **31.7** | 7.0 / 29.8 / **128.4** (ev 7 44.7, ev 12 69.5) |
| tugas: rencana / **antar-lengan** / **lingkungan** | 27.7 / **200.4** / **69.5** | 10.4 / **216.8** / **79.6** |
| retract: muat 3 penyaring / 3 saring | 38.5 / 48.6 | 25.9 / 33.2 |

Replay offline (keadaan HW persis dari perekam, input lingkungan sama → verdict + d_min sama dengan log HW di semua
traverse/retract) vs stempel HW, traverse: **−5, −9, −8, −12, −11, −10, −13, −12 %** — replay **selalu** lebih cepat,
HW ≈ **1.10 ± 0.03 ×** offline (stack ROS + move_group + controller berbagi CPU). Tugas: lintasan HW tidak diarsip →
proksi plan-only G34b-HW (k sama, n berbeda) −46…+20 %, **tidak dipakai** untuk uji ±10 %.

⚠ Artefak replay yang hampir menyesatkan (ditulis): run pertama muat 5.6–6.2 s — GC Python memindai jutaan objek States
(perekam 68 MB). `gc.freeze()` sesudah memuat → 4.0 s = profil. Log `hw_replay_old.log` tumpang-tindih dengan run yang
dihentikan; JSON = sumber.

### B2. Percepatan (A2) — keduanya eksak

1. **B&B Lipschitz** di `EnvChecker.screen_trajectory` (`check()` TIDAK diubah): pasangan (geometri, sampel) dilewati
   bila d_terakhir − (|Δt| + r·‖ΔR‖_F) − 1 mm > 0 dan ≥ minimum berjalan. Badan diam → dihitung sekali. Urutan
   iterasi (sampel, geometri) dan perbandingan `<` ketat sama → minimum, geometri, indeks sama persis.
   **Aksioma diuji pada coal sendiri** (`regress --source lipschitz`): 84 344 pasangan (berurutan + berjarak 10) dari
   lintasan nyata, |d_a − d_b| − perpindahan maks = **0.0** (tak satu pun melanggar).
2. **Cache geometri** (`~/.cache/ceiling_arm/env_geom_<sha>.pkl`, pickle GeometryModel ber-hull): kunci = byte URDF +
   byte SETIAP mesh yang dirujuk + sumber `true_hull` + versi pinocchio; mesh tak terselesaikan → tanpa cache; cache
   rusak → bangun ulang (bukan CLEAR). Cek 52 nama ketat tetap jalan sesudah muat. 4.1 → **0.17 s**.
3. Sampel TIDAK dijarangkan; MARGIN = TOLAK tidak berubah; tidak ada pemanggil yang diubah (`pose_to_g`,
   `reach_dwell_probe.screen_env`, `return_rest`, `g31_screen` mendapat keduanya lewat `EnvChecker()`).
4. `--self-test` + uji baru: sapuan 31 titik arm_1 melewati / menjauhi kotak, hasil terpangkas == `check()` per titik.
   **Uji mutasi**: pemangkasan dirusak (`lb > −1`) → self-test **FAIL** (+0.024 vs −0.019) — uji bisa gagal.

**Replay HW, lama → baru (muat + saring, offline):**

| | lama | baru | contoh |
|---|---|---|---|
| traverse rot ev 7 (671) / ev 12 (957) | 38.90 / 61.03 s | **1.31 / 1.49 s** | saring 34.9 → 1.13, 57.1 → 1.32 s |
| traverse rel (21–127) | 5.1–10.6 s | 0.26–0.66 s | |
| tugas (proksi, +1 s tunggu sendi) | 7.5–12.9 s | 1.36–1.67 s | |
| retract lurus (60) | 6.2–7.4 s | 0.32–0.46 s | |
| **jumlah per varian** | R0 116.75 / R10 188.88 s | **R0 11.64 / R10 13.34 s** | hemat **105.1 / 175.5 s** |

### B3. Regresi (A3)

Lama (`env_collision_old.py`) vs baru, objek terpisah, peta kanonik g34n, mesin sama ([ringkasan](results/p1_g35/regress_summary.json)):

| sumber | lintasan | beda | lama → baru (s, saring saja) |
|---|---|---|---|
| plan-only G34 (reg3/g34/g34n/g34b_hw × R0/R10) | 133 | 0 | 1 763 → 81 (21.8×) |
| plan-only G33, G32-HW, G32, G31 (screen + r0 control), smoke G28 | 410 | 0 | 4 019 → 187 (21.4×) |
| V28 + V28-after (G28y) | 366 | 0 | 4 252 → 210 (20.2×) |
| kontrol G33 sebagai lintasan: **P+** ev 8, **N−** R0 ev0–13 + R10 ev0–7 | 23 | 0 | 195 → 9.6 (20.3×) |
| G34b HW: sapuan traverse (rect) + retract lurus dari keadaan terukur | 13 | 0 | 134 → 4.7 (28.7×) |
| **jumlah** | **945** (106 829 sampel) | **0** | 10 363 → 493 (**21.0×**) |

- **Verdict identik 945/945** (888 CLEAR, **40 MARGIN, 17 COLLIDE** — jalur tolak ikut teruji), geometri + titik terburuk
  identik 945/945, **d_min bit-identik 945/945** (|Δ| maks = 0.0 m, gerbang 0.5 mm).
- **(P+) ev 8:** COLLIDE −12.04 mm keduanya; prefix pertama yang ditolak **−1.71 s sebelum kontak** keduanya (≥ 1.7 ✓).
  **(N−) R0 ev 13** MARGIN 32.14 mm keduanya.
- Waktu di tabel diukur dengan 10–18 pekerja paralel di 16 inti (lambat ~2×, lama dan baru sama-sama); rasio sah,
  angka absolut bukan (absolut = B2 replay).
- Bit-identik bukan kebetulan: pasangan yang dihitung memanggil `coal.distance` dengan input identik; pasangan yang
  dilewati terbukti (aksioma + slack 1 mm) tidak bisa menurunkan minimum. Cache geometri ikut teruji (baru = dari cache,
  lama = bangun segar).

### B4. Prediksi ulang makespan G34b (overhead baru)

Hemat offline × faktor HW 1.00–1.10 (B1), semua lain tetap (gerak, S28, antar-lengan, MoveIt):

| | G34b terukur | hemat | **prediksi** | × P1′-serial |
|---|---|---|---|---|
| R0 | 783.33 s | 105.1–115.6 | **667.7–678.2 s** | 1.149–1.167 × (581.26) |
| R10 | 849.29 s | 175.5–193.1 | **656.2–673.8 s** | 1.237–1.270 × (530.39) |
| Δ(R10 − R0) | +65.96 | | **−4.5 … −11.5 s** (model P1′ −50.87) | tanda = model |

- Sisa overhead lingkungan per event ≈ 0.2 s muat + 0.1–1.3 s saring (+1 s tunggu sendi di tugas) → **≈ 12–15 s per
  varian (~2 %)**, di dalam bias non-tugas P1′ (G26: −0.4…−11.9 %) → **tidak** ditambah suku ke P1′.
- Selisih Δ yang tersisa vs model (~40 s) BUKAN lingkungan: S28 sapuan rot 8.6 / 10.1 s vs 5.5 rel-saja (penyaring
  robot-vs-robot yang sama, belum dipercepat) dan **antar-lengan tugas 24.8–50.4 s per tugas** (P1′ memodelkannya
  ~34 s konstan, G25) — inilah overhead terbesar yang tersisa: **200 / 217 s per varian**.
- R0 prediksi 668–678 vs G32-HW R0 647.47 (sebelum EnvChecker): sisa +20–30 s = muat penyaring lain di retract
  (InterArm + CrossGantry ~9 s × 3) + saring lingkungan residual.

### B5. Papan skor D225–D231

| # | Dugaan | Hasil |
|---|---|---|
| D225 | muat 2–5 s; hull > 50 %; octree < 1 s | ✗ muat 3.9–4.2 ✓, octree 0.035 ✓, tetapi **mesh 69 % / hull 34 %** — klaim mekanisme salah |
| D226 | 50–80 ms/sampel, sama antar alat | ✗ **30–62 ms**, beda 2× menurut konfigurasi |
| D227 | rekonstruksi ±10 % per event | ✗ traverse 4/8 dalam ±10 %, 8/8 dalam −5…−13 % (HW sistematis ~10 % lebih lambat); tugas tak teruji (lintasan HW tidak diarsip) |
| D228 | B&B ≥ 5× rot, ≥ 3× tugas | ✓ rot **30–43×** (muat+saring; saring saja 31–43×), tugas 10–30× (arsip 21×, B3) |
| D229 | regresi 100 % | ✓ **945/945** verdict + geometri + titik, d_min bit-identik, P+ −1.71 s |
| D230 | ≤ 10 s / traverse rot, ≤ 3 s / tugas; Δ negatif | ✓ 1.3–1.5 s, 1.4–1.7 s; Δ −4.5…−11.5 s |
| D231 | sisa terbesar bukan lingkungan | ✓ antar-lengan tugas 200 / 217 s per varian |

### B6. Pertentangan / Rule 12

1. **A2.1 menyebut r_maks dari verteks hull**; dipakai **sudut AABB lokal** (mengandung geometri apa pun, termasuk Box
   struktur; lebih longgar = pemangkasan lebih sedikit, tetap sah). B menang, ditulis.
2. A1 "±10 %" gagal karena faktor beban HW sistematis, bukan karena model biaya salah; prediksi B4 memberi rentang
   1.00–1.10 alih-alih satu angka.
3. Tugas: hemat dihitung dari proksi plan-only (n lintasan HW tidak diketahui; log HW hanya memuat indeks titik minimum,
   mis. ev 8 titik 122 → n ≥ 123 vs proksi 53). Hemat per tugas ≈ 9 ± 2 s tidak sensitif terhadap n (muat 4 s dominan
   untuk n kecil).
4. Replay ev 12 R10 n 957 vs HW S28 n 946: keadaan diambil di CMD + 1.5 s, rel berbeda < 1 sampel lin (×11 rot).
   Verdict/d_min tetap sama dengan log HW (69.5 mm).
5. Pekerja regresi tunggal `plans` dan `v28` dihentikan sengaja (dipecah ke 10 + 8 pekerja paralel); log parsial
   `regress_{plans,v28}_killed_partial.log` (162 + 198 OK, 0 BEDA) disimpan, angka resmi = pekerja `head_*`, `g31_*`,
   `tail`, `v28_*` (semua lintasan diulang penuh, tanpa duplikat — `regress_sum.py`).
6. **Belum diuji di HW:** modul baru (B&B + cache) hanya offline/regresi. Pertama kali dipakai lengan nyata = G35b.

### B7. G35b — HW R0 + R10 seed 36 sama-sesi, penyaring dipercepat (2026-10-08)

> Prompt = §D. Gerbang (operator, sebelum apa pun): **K0 sel sama seperti akhir G34b — ya**; **K1 operator di e-stop — ya**;
> **K2 seed tambahan — tidak** (hanya 36). Data: `docs/results/p1_g35/hw/`. **§B7.A DIKUNCI sebelum bring-up**
> (salinan `docs/results/p1_g35/g35b_A_locked.md` + sha256 di bawah); §B7.1+ diisi sesudah.

#### B7.A 🔒 Protokol + dugaan (SEBELUM bring-up)

Urutan = G34b B7 apa adanya (alat tidak diubah; hanya `env_collision.py` G35): self-test → remount_check → bring-up nyata →
plan-only NYATA k=3 (R10, R0) → `run_g32` DRY → R0 → pulang → R10 → pulang → matikan. Fase per event =
`hw_replay.phases()` pada log baru (traverse: S28 → G33; tugas: antar-lengan → lingkungan, termasuk ~1 s tunggu sendi;
retract: `retract g` → `rencana pulang` mencakup S18 + lintas-gantry + lingkungan, dilaporkan terpisah bila log memisah).

| # | Dugaan |
|---|---|
| D232 | Makespan R0 dalam **667.7–678.2 s**, R10 dalam **656.2–673.8 s** (B4). |
| D233 | Δ(R10 − R0) **negatif** (B4: −4.5 … −11.5 s). |
| D234 | Fase lingkungan HW per traverse berotasi (ev 7, ev 12 R10) **≤ 3 s**; per traverse rel-saja ≤ 2 s. |
| D235 | Fase lingkungan HW per tugas **≤ 3 s** (12/12 tugas). |
| D236 | Verdict tiap event = G34b: R0 6/6, R10 6/6 sukses, 14/14 rc 0 per varian, nol auto-stop, semua saring lingkungan CLEAR, ev 8 R10 SUCCESS. |
| D237 | Plan-only NYATA R10 3/3, R0 3/3; semua retract tak-kosong lurus CLEAR (MoveIt tidak terpicu), juga di eksekusi. |
| D238 | Muat EnvChecker di HW dari cache ≤ 0.5 s per proses (tidak ada bangun ulang 4 s; cek di log self-test/alat). |

sha256 `g35b_A_locked.md` = `269b9192…aacda4f` (dikunci 2026-10-08 11:54, sebelum bring-up).

#### B7.0 Sebelum gerak

`env_collision.py --self-test` **PASS** (0.8 s total, dari cache). `remount_check` LULUS (ICMP 4/4, nol basi, 2× D455).
Bring-up nyata launch **2205253** (SIG_DFL + setsid nohup, SigIgn `0x1001001` → SIGINT tidak diabaikan): 4× Actuator '6',
**7/7** active (7 spawner "died" = baseline), table1/table2 **ARMED 0.1 / 0.8 mm**, rot 0/0. `env_static_map_pub` keep-alive
g34n 136 558 voxel → 22 246 kotak, di scene **True**; `--once` **True**. `js_record` + `reach_dwell_monitor` (csv
`/tmp/g35hw_step`) hidup. DRY R0: 14 event, lengan maks 0.113° dari REST, rel 0.14 / 0.76 mm, S26 lolos.

#### B7.1 Plan-only NYATA — 🔴 R10 DITOLAK, R0 3/3

`DOMAIN=0 g34_planonly.sh g35b_hw R10 R0` ([log](results/p1_g34/g34_planonly_g35b_hw.log); keluaran di `p1_g34/` karena skrip
dipakai apa adanya):

- **R0 3/3 LOLOS** (18/18 PLANNED), semua retract lurus CLEAR (0 MoveIt, 0 MARGIN/COLLIDE), lingkungan min **52.2 mm** (margin 50).
- **R10 ditolak di sampel 1** (k-of-k berhenti): ev 0–10 PLANNED/CLEAR, lalu **retract ev 11 g2 @ −10°**: garis lurus
  arm_3 **COLLIDE −14.1 mm** (titik 23/60, `t2_a1_left_finger_prox_link_0`), kedua urutan, **MoveIt NO-PLAN** →
  `RETRACT-arm_3:CLEAR/CLEAR/COLLIDE+NO-PLAN`. Nol gerak; penyaring menolak sesuai aturan.
- **Bukan galat G35** — garis retract yang sama dari titik akhir rencana ev 10 diulang offline dengan checker LAMA (beku G34)
  dan BARU ([skrip](results/p1_g35/g35b_ev11_check.py), [log](results/p1_g35/g35b_ev11_check.log)): **−14.09 mm, geometri +
  titik identik** di keduanya; 6 rencana ev 10 arsip lain juga identik lama = baru.
- **Mekanisme: cabang IK tugas ev 10** (t5 arm_3 → (0.9286, −0.4588, 1.0)). Rencana ini berakhir di arm_3 joint_1
  **+103.7°**; G34b-HW 3/3 berakhir di **−95.9 … −119.5°** (retract lurus CLEAR 58–63 mm); mock G34 g34n sampel 0 **+100.3°**
  → MARGIN 19.6 mm (dulu MoveIt PLANNED di mock = "ev 11 via MoveIt" G34). Jadi **2 dari 7** rencana ev 10 yang terarsip
  jatuh di cabang +100°, dan dari cabang itu garis lurus pulang lewat rak. Di eksekusi HW R10, cabang ini = `return_rest`
  menolak → AUTO-STOP dengan arm_3 tertinggal di pose tugas dekat rak (g2 −10°), pemulihan manual.
- Keputusan operator (ditanya sesudah plan-only): **R0 saja; R10 TIDAK dijalankan** (gerbang 3/3 gagal). Δ(R10 − R0) tidak terukur.

#### B7.2 Eksekusi R0 (`run_g32` apa adanya, DRY dulu)

| | G34b R0 (env lama) | **G35b R0 (env G35)** |
|---|---|---|
| sukses penilai | 6/6, 14/14 rc 0 | **6/6, 14/14 rc 0**, nol auto-stop, fault launch log 0 |
| sukses @ s | 139.7 / 312.3 / 378.9 / 525.4 / 737.5 / 783.3 | **112.6 / 238.4 / 293.9 / 411.4 / 594.9 / 629.7** |
| **makespan** | 783.33 s = 1.348 × P1′-serial | **629.67 s = 1.083 ×** P1′-serial 581.26 (−153.65 s, −19.6 %) |
| torsi puncak | 10.51 `t2_a1_joint_2` | 10.72 `t2_a2_joint_2` (ev 13; tugas 6.14–10.72, retract ≤ 5.52) |
| lingkungan min (rencana tugas) | 58.1 mm | **61.5 mm** (ev 13 arm_4); semua saring lingkungan CLEAR |

- Vs G32-HW R0 (sebelum EnvChecker, 647.47 s): **−17.8 s** — penyaring lingkungan sekarang lebih murah dari keragaman antar-run.
- Retract ev 3/7/10 **lurus** (88.6 / 209.3 / 109.5 mm); MoveIt `return_rest` tetap belum pernah dipakai lengan nyata.
- Pulang ([log](results/p1_g35/hw/g35hw_home_R0.log)): g2 lalu g1 lurus (torsi 5.52 / 5.38, galat 0.082 / 0.110°), rel g1
  **0.55 mm**, g2 **0.36 mm**, rot 0/0, drift lengan ≤ 0.193°.

**Fase per event** ([g35b_analyze.py](results/p1_g35/g35b_analyze.py) = `hw_replay.phases()` pada log G34b-R0 dan G35b-R0,
[log](results/p1_g35/g35b_analyze.log)) — jumlah per jenis, G34b → G35b (s):

| jenis | durasi | fase | lingkungan |
|---|---|---|---|
| traverse (4) | 221.8 → **191.7** | start 6.7 → 6.0, S28 22.2 → 22.7, gerak 160.5 → 160.4 | **31.7 → 1.8** (0.27–0.70 per traverse) |
| tugas (6) | 370.6 → **267.9** | rencana (termasuk muat EnvChecker) 27.7 → **9.1**, **antar-lengan 200.4 → 181.3**, eksekusi 60.7 → 55.8 | **69.5 → 9.2** (1.45–1.71 per tugas) |
| retract (3 + skip) | 190.9 → **170.1** | muat 3 penyaring 38.5 → **25.8**, saring (S18 + lintas + env) 48.6 → 39.6, gerak 93.2 → 93.2 | (di dalam "saring") |

Atribusi −153.65 s: lingkungan langsung (saring traverse −29.9, saring tugas −60.3, muat di rencana tugas −18.6, muat retract
−12.7, saring retract −9.0) ≈ **−130.5 s**; non-lingkungan **≈ −23 s** (antar-lengan −19.1 = rencana OMPL berbeda/beban CPU,
eksekusi −4.9). Gerak identik (±0.1 s) → selisih seluruhnya pra-kirim.

**Kenapa B4 meleset (memprediksi 105–116 s hemat):** hemat tugas di B4 dari **proksi plan-only** (n lebih kecil dari lintasan
HW — sudah ditandai B6.3); HW: hemat tugas 78.9 s (13.2 s/tugas) vs proksi ~6–11 s/tugas. Ditambah −19 s antar-lengan yang
memang acak. Faktor HW 1.10 (B1) tidak lagi relevan: overhead lingkungan HW yang tersisa **≈ 11 s per varian** (1.8 + 9.2).

Yang tersisa terbesar: **antar-lengan tugas 181.3 s (24.1–37.6 s/tugas)** + retract muat/saring 65 s (S18 + lintas-gantry) =
~246 s pra-kirim non-lingkungan per varian R0 — kandidat G36 (B&B yang sama: badan diam, Lipschitz).

#### B7.3 Papan skor D232–D238

| # | Dugaan | Hasil |
|---|---|---|
| D232 | R0 667.7–678.2, R10 656.2–673.8 s | ✗ **R0 629.67 s** — 38 s DI BAWAH rentang (hemat tugas diremehkan, proksi B6.3); R10 tak dijalankan |
| D233 | Δ(R10 − R0) negatif | — **tidak dinilai** (R10 tidak dijalankan) |
| D234 | lingkungan ≤ 3 s per traverse rot; ≤ 2 s rel-saja | — **tidak dinilai**: inti (traverse berotasi) tidak dijalankan; bagian rel-saja ✓ 0.27–0.70 s (4/4) |
| D235 | lingkungan ≤ 3 s per tugas | ✓ **1.45–1.71 s, 6/6** (R0; 6 tugas R10 tak dijalankan) |
| D236 | verdict tiap event = G34b | ✓ R0: 6/6, 14/14 rc 0, nol auto-stop, semua saring lingkungan CLEAR (R10 tak dijalankan) |
| D237 | plan-only R10 3/3, R0 3/3; semua retract lurus | ✗ **R10 ditolak di sampel 1** (ev 11, cabang IK ev 10); R0 3/3 dan eksekusi R0 lurus ✓ |
| D238 | muat dari cache ≤ 0.5 s, tanpa bangun ulang | ✓ cache `env_geom_e4d1…pkl` tidak ditulis ulang sepanjang sesi (mtime 11:16:15 tetap = tak ada miss); muat per proses turun ~3–4 s (rencana tugas −3.1 s/tugas, muat retract −4.2 s/retract) = 4.0 → 0.17 s B2. **Tidak langsung:** waktu muat tidak dicetak alat. |

**Tally G35b: 2 meleset / 3 tepat** (D233, D234 tidak dinilai).

#### B7.4 Pertentangan / Rule 12

1. **B menang atas A (ditulis):** protokol §B7.A meminta R0 + R10; R10 tidak dijalankan karena gerbang plan-only gagal
   (keputusan operator). Δ(R10 − R0) di HW **tetap belum terukur** bersih — pertanyaan G34b B7.4 belum terjawab.
2. Plan-only R10 G34b-HW 3/3 bukan bukti "R10 aman": dengan 2/7 rencana ev 10 di cabang +100°, peluang 3/3 ≈ (5/7)³ ≈ 0.36.
   G34b beruntung (HW R10 ev 11 lurus 67.8 mm). **Gerbang k=3 tidak mendeteksi peristiwa ~30 %** dengan andal.
3. Mengapa MoveIt NO-PLAN dari cabang +100° (mock G34 g34n: PLANNED dari konfigurasi serupa, MARGIN 19.6 mm vs COLLIDE −14.1)
   **belum didiagnosis** — kandidat: keadaan awal sudah bertabrakan di scene MoveIt (kotak peta +0.05 m) atau OMPL waktu habis.
4. Lingkungan min plan-only R0 52.2 mm vs margin 50 mm vs lintas-kamera ~5 cm: cadangan nyata ≈ 0–2 mm (seperti G34b B8.3).
5. Keluaran plan-only `g34_planonly_*g35b_hw*` tinggal di `p1_g34/` (skrip apa adanya), bukan `p1_g35/`.
6. Perekam `hw/g35hw_joint_states.csv.gz` (31.5 MB) **lokal saja, tidak di-commit** (seperti G32-HW / G34b).

## C. Keadaan akhir (Rule 12)

- **Gerak: nol.** Tidak ada bring-up, tidak ada proses ROS. Hanya Python offline.
- **Diubah:** `scripts/env_collision.py` — `EnvChecker.screen_trajectory` (B&B eksak), `RobotGeom.__init__` (cache
  geometri `~/.cache/ceiling_arm/`), `self_test` (+ sapuan terpangkas == `check()`). `check()`, `self_filter`,
  `point_distances`, ENV_MAP, ENV_MARGIN_M **tidak** diubah. Pemanggil tidak diubah.
- **Baru:** `docs/results/p1_g35/` (profil, replay HW, regresi, analisis, salinan beku `env_collision_old.py`).
- **Cache** dibuat di mesin ini (`env_geom_e4d1a2e9…pkl`, 4.3 MB). Hilang/rusak → dibangun ulang otomatis (~4 s sekali).
- **Belum diukur:** fase antar-lengan (S18 + lintas-gantry, 200 / 217 s per varian) dan S28 — di luar lingkup G35
  (penyaring lingkungan saja); mekanisme B&B yang sama berlaku untuk keduanya (badan diam, Lipschitz) → kandidat G36.
- **Belum diuji di HW:** modul baru. Prediksi B4 = dugaan untuk G35b, bukan hasil.
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi multi-langkah dengan regresi; dilaporkan.

## D. Prompt G35b (salin ke chat BARU)

> ✅ Dijalankan 2026-10-08 sebagai G35b → §B7 (R0 saja; R10 ditolak di plan-only), §C2.

**Rekomendasi: Opus, effort TINGGI** — penyaring lingkungan yang dipercepat dipakai lengan nyata untuk pertama kali; bila
B&B atau cache salah di HW, galatnya diam (CLEAR palsu) sampai lengan menabrak.

```
Sesi G35b -- HW R0 + R10 seed 36 SAMA-SESI dengan penyaring lingkungan dipercepat (G35). Repo ceiling_arm, branch
feat/rgbd-deploy. BACA PENUH: CLAUDE.md; docs/p1_g35_env_speed.md (B2, B4, C); docs/p1_g34_rail_calib.md (B7 urutan HW,
B7.4); docs/results/p1_g32/run_g32.py; docs/results/p1_g34/g34b_run.sh + g34b_home.sh.
FAKTA G35 (offline): EnvChecker.screen_trajectory = branch-and-bound Lipschitz EKSAK + cache geometri (~/.cache/ceiling_arm);
regresi lama-vs-baru pada SEMUA arsip: verdict/geometri/titik identik, d_min bit-identik, P+ ev 8 COLLIDE -12.04 mm ditolak
-1.71 s (sama), N- ev13 MARGIN 32.14 mm (sama). Replay HW: muat 4.0 -> 0.17 s, sapuan rot 671/957 sampel 38.9/61.0 ->
1.3/1.5 s; hemat offline R0 105 s, R10 176 s. PREDIKSI (HW = offline x 1.00-1.10): R0 667.7-678.2 s, R10 656.2-673.8 s,
Delta(R10-R0) -4.5 ... -11.5 s (model P1' -50.87). Sisa overhead terbesar = antar-lengan tugas 25-50 s/tugas (bukan G35).
GERBANG (tanya operator SEBELUM apa pun): K0 sel sama seperti akhir G34b (lengan REST, rel ~0/0, rot 0/0, rak + benda x~2.1
tidak dipindah, tidak ada benda baru)? K1 operator di e-stop selama gerak lengan? K2 seed tambahan? (g32_candidates.json
hanya punya seed 36 -> seed lain = persiapan offline g31 + plan-only mock 3/3 DULU; default: TIDAK di sesi ini.)
0. KUNCI §A (dugaan D232+: makespan R0/R10 dalam rentang B4, Delta negatif, fase lingkungan HW per traverse rot <= 3 s,
   per tugas <= 3 s, verdict tiap event = G34b) SEBELUM bring-up -- G34b lupa (B8.1).
1. python3 scripts/env_collision.py --self-test (PASS, juga menghangatkan cache); remount_check; bring-up nyata (SIG_DFL +
   setsid nohup, PID launch ASLI, SigIgn), 4x Actuator '6', 7/7, ARMED; env_static_map_pub keep-alive + --once True;
   js_record + reach_dwell_monitor.
2. Plan-only NYATA (DOMAIN=0 docs/results/p1_g34/g34_planonly.sh g35b_hw R10 R0, k=3) -> harus 3/3, verdict per lintasan
   dibandingkan dengan G34b-HW (lingkungan min sama +-0.5 mm untuk tugas yang sama tidak dijamin: OMPL acak).
3. run_g32 --seed 36 R0 (prefix g35hw_R0_, --out-dir/--archive docs/results/p1_g35/hw), pulang; R10 sama; pulang;
   matikan bersih (node list --no-daemon = 0).
4. Laporan p1_g35_env_speed.md §B7+: makespan vs prediksi B4 dan vs P1'-serial, Delta(R10-R0), fase per event (pakai
   docs/results/p1_g35/hw_replay.py phases() pada log baru), tiap retract lurus/moveit; p1_state + tally; commit;
   prompt G36 (antar-lengan B&B) + rekomendasi model/effort.
ATURAN: tak ada gerak tanpa env_static_map di scene DAN EnvChecker; MARGIN = TOLAK; B menang atas A dan DITULIS;
job > 10 menit via setsid nohup; pkill -f membunuh shell sendiri (pilih PID via ps/awk).
```

## C2. Keadaan akhir G35b (Rule 12)

- **Gerak:** lengan nyata R0 (14 event) + pulang; semua rc 0, nol fault, nol auto-stop, tidak ada kontak dilaporkan. **R10 tidak
  dijalankan** (plan-only ditolak, B7.1). Akhir: **keempat lengan REST** (≤ 0.110°), **g1 0.55 mm, g2 0.36 mm, rot 0/0**.
- **Stack:** helper (monitor, perekam, `env_static_map_pub`) SIGINT; launch 2205253 SIGINT → keluar 7 s;
  `ros2 node list --no-daemon` **0**. Crash dump `ros2_control_node` saat shutdown (baseline) dihapus. Disk 53 GB.
- **Kode tidak diubah** di G35b. **Baru:** `docs/results/p1_g35/` `g35b_A_locked.md`, `g35b_run.sh`, `g35b_home.sh`
  (salinan G34b, jalur saja), `g35b_analyze.{py,log,json}`, `g35b_ev11_check.{py,log}`, `hw/` (log event, run.json, launch
  `.log.gz`, monitor); `docs/results/p1_g34/g34_planonly_*g35b_hw*`.
- **Belum diukur di HW:** Δ(R10 − R0) bersih; traverse berotasi dengan penyaring baru; jalur MoveIt `return_rest`.
- Anggaran token Rule 6 (30k/sesi) **terlampaui** — sesi HW multi-langkah; dilaporkan.

## E. Prompt G36 (salin ke chat BARU)

**Rekomendasi: Opus, effort TINGGI** — mengubah aturan terima rencana tugas di dekat rak (galat = lengan tertinggal di pose
yang tak bisa dipulangkan, atau CLEAR palsu); bagian B&B antar-lengan menyentuh penyaring keselamatan lagi.

```
Sesi G36 -- OFFLINE + mock (nol gerak lengan nyata): R10 seed 36 tidak bisa dieksekusi andal (ev 10 cabang IK -> retract ev 11
ditolak), lalu antar-lengan = overhead terbesar. Repo ceiling_arm, branch feat/rgbd-deploy. BACA PENUH: CLAUDE.md;
docs/p1_g35_env_speed.md (B7.1, B7.2, B7.4); scripts/return_rest.py (plan_retract); scripts/reach_dwell_probe.py
(_plan_and_screen); docs/results/p1_g31/g31_screen.py; docs/results/p1_g35/g35b_ev11_check.py.
FAKTA G35b (HW 2026-10-08): R0 seed 36 6/6 629.67 s = 1.083x P1'-serial (G34b 783.33; lingkungan HW tersisa ~11 s/varian);
plan-only NYATA R10 DITOLAK sampel 1: ev 10 t5 arm_3 berakhir di joint_1 +103.7 deg -> retract lurus ev 11 COLLIDE -14.1 mm,
MoveIt NO-PLAN (checker lama = baru, bukan G35). 2/7 rencana ev 10 terarsip di cabang +100 deg (G34 mock g34n s0 +100.3 ->
MARGIN 19.6 -> MoveIt PLANNED di mock). Gerbang k=3 lolos ~36 % walau peluang cabang buruk ~30 %. Antar-lengan tugas
181 s/varian (24-38 s/tugas), retract muat+saring S18/lintas 65 s.
GERBANG: tidak ada gerak; mock domain 77 saja.
0. KUNCI §A (dugaan D239+) SEBELUM mengukur.
A. Diagnosis: kenapa MoveIt NO-PLAN dari cabang +100 deg (keadaan awal bertabrakan di scene MoveIt? kotak +0.05 m? waktu
   habis?) -- ulang di mock dari konfigurasi arsip g34_planonly_plans_g35b_hw_R10.jsonl.
B. Perbaikan paling sederhana yang membuat R10 ev 10/11 andal (contoh: probe menerima rencana tugas hanya bila retract dari
   titik akhirnya lolos plan_retract; atau batasi cabang IK) -- tanpa melonggarkan MARGIN = TOLAK. Uji: plan-only mock R10
   k >= 10 (estimasi laju gagal + CI), R0 k >= 3 tidak berubah, kontrol P+ ev 8 G33 tetap ditolak.
C. Bila A+B selesai: B&B Lipschitz antar-lengan (S18 + lintas-gantry, badan diam) dengan regresi WAJIB seperti G35 A3
   (verdict identik 100 %, d_min bit-identik, semua arsip). Gagal -> kembalikan, DITULIS.
D. Prompt G36b HW (R0 + R10 seed 36 sama-sesi, ukur Delta bersih) + rekomendasi model/effort.
ATURAN: nol gerak nyata; MARGIN = TOLAK; B menang atas A dan DITULIS; job > 10 menit via setsid nohup; pkill -f membunuh
shell sendiri (pilih PID via ps/awk).
```
