# P1 / G40 — penjadwal sadar-lingkungan: pose + traverse gantry vs peta, tugas = scene MoveIt, torsi = tabel G39 (OFFLINE + mock)

> Sesi 2026-10-10 (sesudah G38 + G39). NOL gerak HW; mock `ROS_DOMAIN_ID=77`. Instans = 20 set node G38 (3 tengah + 3
> dinding, R35, p0 g1 PARKIR 1.50). G38 B4–B5: 16/19 plan-only gagal TRAVERSE vs lingkungan (jari lengan REST masuk rak),
> 2–3 tugas dinding NO-PLAN; penjadwal buta pose/traverse gantry. **§A ditulis dan DIKUNCI SEBELUM pengukuran**
> (salinan `docs/results/p1_g40/g40_A_locked.md`, sha256 `216289d3…9a252dad`, dikunci 2026-10-10 07:26). §B menang atas
> §A dan DITULIS. MARGIN = TOLAK.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

- Log G38 plan-only (B4): pelanggar traverse = jari lengan REST gantry yang BERGERAK (contoh seed 0: g1 (1.20, +35) →
  (0.95, +30) MARGIN 28.3 mm `t1_a1_right_finger_dist_link`). NO-PLAN dinding: s1 t3 arm_3 @ (0.35, −35) 0.7 s; s16 t4
  arm_2 @ (1.60, −30) 0.4 s; s19 (k = 3, sampel 1) t4 arm_3 @ (0.85, −10) RETRACT-BLOCKED lalu NO-PLAN ×2.
- Penyaring layar (`g31_screen`): traverse = `EnvChecker.screen_trajectory` atas `rect_points` (grid penuh persegi
  [lin₀, lin₁] × [rot₀, rot₁], 10 mm × 1°), SEMUA 52 geometri, keadaan lain DITAHAN; tugas = rencana MoveIt + layar
  lingkungan SEMUA geometri ≥ 50 mm + retract_check garis lurus. Scene MoveIt = `env_static_map_pub.column_boxes`
  (22 246 kotak, bin 2 cm, kolom vertikal digabung, **tumbuh 5 cm per sisi** = inflasi L∞).
- `sched.solve_gantry`: DP h(R, p) = min_p′ [T(p, p′) + w(R, p′)], stop selalu U ≠ ∅ (tidak ada stop-titik-lewat).
- Tidak ada perhitungan lingkungan/penjadwalan baru sebelum mengunci (hanya hitung kotak + waktu muat EnvChecker 0.15 s).

### A1. 🔒 Model lingkungan penjadwal G40 (keempat predikat; geometri = semua `t{g}_` gantry yang dimaksud)

Keadaan = gantry g di (lin, rot), lengan g yang tidak bertugas di REST; geometri gantry lain TIDAK dimasukkan
(dekomposisi: gantry lain diam selama kejadian ini dan keadaan diamnya sudah lolos predikatnya sendiri ≥ 50 mm).

- **(a) Pose layak** ⟺ dengan KEDUA lengan REST: `EnvChecker.check(only=t{g}_)` ≥ 50 mm **DAN** bebas-kotak (lambung
  `true_hull` tiap geometri `t{g}_` tidak menembus satu pun kotak tumbuh `column_boxes`, jarak coal > 0). Dihitung untuk
  SEMUA 33 × 72 pose tiap gantry. Bebas-kotak ditambahkan di luar permintaan (a) karena keadaan awal MoveIt di setiap
  stop = lengan REST di pose itu; start-state yang menabrak scene ⇒ semua rencana di stop itu gagal.
  p0 kedua gantry harus layak; kalau tidak → DITULIS, instans tetap dijalankan dengan p0 apa adanya.
- **(b) Traverse layak** (tepi p → p′) ⟺ `EnvChecker.screen_trajectory([lin_g, rot_g], rect_points(p, p′), REST,
  only=t{g}_)` = CLEAR (≥ 50 mm). Kendala tepi di DP: T(p, p′) = ∞ bila tidak layak (termasuk tepi pertama dari p0).
  Diverifikasi LAZY (tepi tak dikenal = layak), di-cache simetris (persegi rect sama dua arah).
- **(c) Tuple tugas layak** (node, g, li, ri, s) ⟺ L1 ∧ **tabel G39 ok (aturan A2/A3 G39)** ∧ pose (a) layak ∧ oracle‴
  tepat `torq2` ok (dihitung bersama lingkungan; tabel ok ∧ tepat tidak = salah-terima di luar sampel → DITOLAK dan
  DILAPORKAN) ∧ **lingkungan-∃′**: ada solusi IK oracle (roll mana pun) dengan `EnvChecker.check(only=t{g}_)` ≥ 50 mm
  **DAN** bebas-kotak (seperti (a)). Lazy per tuple terpakai (+ tetangga `g31_sched.neighbours`), cache
  `g40_tuple_cache.jsonl`. Tuple dengan tabel ditolak TIDAK dihitung (aturan pakai A3: tabel menggantikan oracle).
- **(d)** struktur antar-gantry = `sched_coll.schedule_conflict` apa adanya (kriteria (iii)).

### A2. 🔒 Penjadwalan + klaim eksak

`g31_sched`/`g38_make.build` (P1′, rot_cmd 0.9235, R35, p0 g1 (1.50, 0°), g2 (0, 0°)), dengan (a) mengenolkan reach di
pose tak layak, (c) menggantikan predikat tuple, (b) sebagai T = ∞ di salinan `solve_gantry`/`solve_exact` (skrip G40,
`sched.py` tidak diubah). Loop lazy: selesaikan → tuple terpakai belum terverifikasi + tepi terpakai belum terverifikasi
→ hitung → ulang sampai keduanya kosong. **Klaim eksak** untuk model M40 = {stop U ≠ ∅, traverse LANGSUNG antar stop
berurutan (seperti runner), (a)–(c)}: relaksasi (tak dikenal = layak) ⊇ layak, dan optimum yang seluruh tuple + tepinya
terverifikasi layak ⇒ optimum sejati M40. **TIDAK eksak** terhadap: rute dengan titik-lewat (DP tidak punya stop kosong;
T = ∞ merusak ketaksamaan segitiga — jadwal bisa lebih buruk daripada yang memakai jalan memutar), salah-tolak tabel
(25.7 %), lintasan MoveIt (hanya titik akhir oracle yang dimodelkan), retract_check, lengan–lengan lintas gantry (celah 2).

### A3. 🔒 Pengukuran berurutan

1. **K**: (K1) predikat (b) mereproduksi SEMUA baris "traverse g vs lingkungan" di log G38 plan-only (+ s19k3): verdict
   sama; |Δ min| ≤ 1 mm bila geometri terburuk log milik `t{g}_`. (K2) torsi job G40 bit-identik dengan cache G38 pada
   ≥ 4 tuple. (K3, di mock) bebas-kotak vs MoveIt `/check_state_validity` pada ≥ 100 keadaan (pose REST + solusi tugas
   dekat dinding): salah-terima (kotak bebas, MoveIt tabrak `env_static_map`) harus 0.
2. **Pose** (a) semua 33 × 72 × 2 → `g40_pose_table.json`; lapor fraksi layak di R35 per gantry.
3. **Penjelasan NO-PLAN** (permintaan 2): predikat (c) pada tuple yang dipakai ketiga tugas NO-PLAN dinding G38 (s1 t3,
   s16 t4, s19 t4) dan pada semua tuple terpakai jadwal R35 G38 (114) + konfusi terhadap verdict tugas di log plan-only G38.
   "Dijelaskan" ⟺ (c) menolak tuple itu.
4. **Jadwal ulang** 20 seed (kriteria (i)–(iii) G38 A4) → `g40_candidates.json`.
5. **Plan-only mock** `g31_screen.py --variant G40 --repeats 3` pada semua seed lolos (i)–(iii); atribusi gagal = verdict
   terakhir; INTERARM-COLLIDE t1_*↔t2_* (atau verdict lintas di traverse) = **lintas-gantry**. Lolos ⟺ 3/3 PLANNED.

### A4. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D300 | Pose R35 (33 × 15) layak (a): g1 **75–95 %**, g2 **60–85 %**; rot 0 layak ≥ 95 % kedua gantry. |
| D301 | Predikat (c) menjelaskan **≥ 2 / 3** NO-PLAN dinding G38. |
| D302 | (c) menolak **5–30 %** dari 114 tuple terpakai jadwal R35 G38. |
| D303 | (i) terselesaikan **≥ 15 / 20** seed; lolos (i)–(iii) **≥ 12 / 20**. |
| D304 | Makespan median naik **≥ 5 %** dibanding G38 R35 (seed yang selesai di keduanya). |
| D305 | Plan-only mock 3/3 PLANNED **≥ 8** seed. |
| D306 | Gagal traverse vs lingkungan di plan-only **≤ 1** seed (predikat (b) eksak untuk geometri gantry yang bergerak). |
| D307 | Dari seed yang gagal plan-only, **≥ 30 %** gagal lintas-gantry (lengan atau traverse). |
| D308 | Salah-terima tabel G39 di luar sampel (tabel ok, tepat tidak) = **0** pada semua tuple yang dihitung G40. |

## B. Hasil terukur

### B0. K (A3.1) — [log K1/K2](results/p1_g40/g40_k.log), [log K3](results/p1_g40/g40_k3.log)

- **K1:** 47/47 baris "traverse vs lingkungan" G38 → verdict SAMA; 39 baris dengan geometri terburuk milik `t{g}_`
  |Δ min| ≤ 1 mm; 8 baris lain terburuk = gantry lain (min G40 ≥ log, verdict sama). **K2:** torsi bit-identik 4/4.
- **K3 — 🔺 kesalahan saya, DITULIS:** run pertama memakai `group_name='arm_1'`; `/check_state_validity` lalu hanya
  memeriksa link grup itu → tidak sah ([log](results/p1_g40/g40_k3_grp_arm1_INVALID.log), 63 "kotak tabrak, MoveIt
  bebas" palsu). Diulang dengan `group_name=''` (seluruh robot): **144/144 sepakat** — salah-terima **0**, salah-tolak
  **0**; MoveIt-tabrak d_box ≤ −0.34 mm, MoveIt-bebas d_box ≥ +0.85 mm. Predikat kotak = scene MoveIt.

### B1. Pose (a) — [log](results/p1_g40/g40_poses.log), 4 752 pose dalam 28 s

R35 layak: **g1 442/495 (89.3 %)**, **g2 379/495 (76.6 %)**; rot 0: g1 33/33, g2 31/33 (1.55, 1.60 = rak ujung).
g1 tak layak di rot +10…+35 (x 0.70–1.15) dan −30/−35 (x 1.50–1.60); g2 di −25…−35 (x 0.45–1.15) dan +5…+35
(x 1.05–1.60). p0 kedua gantry layak. Bebas-kotak menambah 23 (g1) / 12 (g2) pose tak layak di atas EnvChecker.

### B2. Penjelasan NO-PLAN dinding G38 (permintaan 2) — [log](results/p1_g40/g40_explain.log)

- 117 tuple unik terpakai jadwal R35 G38 (torsi tepat 117/117 ok): lolos tabel G39 **59**, pose (a) **69**, env2 87;
  **(c) menolak 80/117** — sebagian besar karena tabel (salah-tolak 58/117 = **50 %** di tuple tepi R35, jauh di atas
  25.7 % G39).
- **s16 t4 DIJELASKAN**: pose g1 (1.60, −30) tak layak (jari `t1_a2_left_finger_dist` REST −4.1 mm ke kotak); log
  move_group G38 mengonfirmasi: start-state tabrak `env_static_map`, Fix-Start-State geser 0.17 m, jalur tidak valid ×3.
- **s1 t3, s19 t4 TIDAK dijelaskan**: (c) menerima; semua solusi roll k = 0 (orientasi tujuan MoveIt) valid di MoveIt.
  s19 t4 = RETRACT-BLOCKED (cabang IK MoveIt), s1 t3 = belum diketahui (0.7 s). **1/3.**
- 🔴 **s1 t2 PLANNED di G38 dengan start-state TABRAK** (g1 (1.55, −30) REST tidak valid di MoveIt): Fix-Start-State
  melompat dan jalurnya kebetulan valid — celah G36 terulang; di HW lompatan itu = gerak tak terlihat penyaring.

### B3. Jadwal ulang, protokol terkunci (torsi = tabel G39) — [log](results/p1_g40/g40_run.log)

**(i)–(iii) 2/20** (seed 8, 19); 18/20 (i) GAGAL seketika "task k infeasible on every gantry" (15 tugas tengah, 3
dinding). Diagnosa ([log](results/p1_g40/g40_diag_infeasible.log)): untuk tugas gagal, tabel ∧ pose = **0** di semua
18; tuple tepat-ok hanya 2–19 per tugas dan pada 5 seed (4, 5, 6, 11, 12) tepat-ok ∧ pose = 0 tanpa tuple tak dikenal —
**rotasi yang aman torsi untuk tugas tengah adalah rotasi yang mendorong pasangan REST ke rak** (mekanisme G38 B5, kini
sebagai kendala keras).

### B4. Varian G40e (B menang atas A, DITULIS): torsi = oracle‴ tepat, tabel TIDAK dipakai — [log](results/p1_g40/g40e_run.log), [JSON](results/p1_g40/g40e_candidates.json)

Alasan: B2/B3 — aturan pakai A3 G39 ("tabel menggantikan oracle") membuang 50 % tuple tepi; oracle tepat = predikat G38.
(a)(b)(c-lingkungan) sama.
- **Penyimpangan kerja (bukan hasil):** run pertama memverifikasi tepi 1–2 per iterasi (seed 3: ≥ 68 iterasi); dihentikan
  ([log](results/p1_g40/g40e_run1_restarted.log)) dan diulang dengan `neighbour_edges` (tepi a–b′, a′–b dalam ±2 langkah
  grid ikut diverifikasi; eksak tetap — terminasi = tak ada tepi terpakai belum terverifikasi). Cache dipakai ulang;
  seed 0 terulang **identik** (254.64 s).
- **(i)–(iii) 6/20** (seed 0, 3, 8, 10, 18, 19); 14/20 (i) GAGAL seketika (tugas tanpa tuple tepat-ok ∧ pose layak).
  (ii), (iii) lolos semua yang (i)-nya lolos. Makespan vs G38 R35: **+15.6, +17.1, +12.3, 0.0, +21.2, 0.0 %** (median
  **+14 %**); seed 10 dan 19 = jadwal G38 sudah layak-lingkungan.
- Tepi diverifikasi 5 648: CLEAR 3 563, MARGIN 1 311, COLLIDE 774 (**37 % tepi tidak layak**); tuple dihitung 3 473
  (ok3 830, env2 di antara ok3 799). Iterasi lazy 1–42.
- **D308:** dari 3 473 tuple baru, tabel ok 618, salah-terima **0**, salah-tolak 212 (25.5 %) — [log](results/p1_g40/g40_d308.log).

### B5. Plan-only mock k = 3 — [G40](results/p1_g40/g40_planonly.json), [G40e](results/p1_g40/g40e_planonly.json)

Mock domain 77 (launch 2725056, SIG_DFL, SigIgn `0x1001001`, galat spawner = baseline G32/G34), `env_static_map` di scene
True. Seed 0–10 G40e berjalan bersamaan dengan run G40e (nice +10); 18/19 sesudahnya. Mati bersih 09:28 (launch keluar
SIGINT, `node list --no-daemon` 0; crash dump shutdown move_group baseline dihapus).

| varian | seed | sampel | gagal pada | kelas |
|---|---|---|---|---|
| G40 | 8 | 1 | traverse g2 (0.00, 0) → (0.10, +35) MARGIN 0.8 mm `t1_a1_arm` ↔ `t2_a1_arm` | **lintas-gantry** |
| G40 | 19 | PLANNED / 2 | t3 arm_3 RETRACT-BLOCKED → NO-PLAN | pulang lurus |
| G40e | 0 | PLANNED / 2 | t3 arm_3 RETRACT-BLOCKED ×3 (garis pulang COLLIDE peta) | pulang lurus |
| G40e | 3 | PLANNED, PLANNED / 3 | traverse g2 (0.00, 0) → (0.10, +35) COLLIDE −30.9 mm `t1_a1_forearm` ↔ `t2_a1_arm` | **lintas-gantry** |
| G40e | **8** | **PLANNED ×3** | — | **LOLOS** |
| G40e | 10 | PLANNED, PLANNED / 3 | t5 arm_3 (dinding) RETRACT-BLOCKED | pulang lurus |
| G40e | 18 | PLANNED / 2 | traverse g2 (0.10, +30) → (0.70, −20) COLLIDE −14.9 mm `t1_a2_forearm` ↔ `t2_a1_arm` | **lintas-gantry** |
| G40e | 19 | 1 | t3 arm_3 TORQUE-UNSAFE (j2 103 %, transit — G27) | torsi lintasan |

- **Traverse vs lingkungan: 0 gagal di 8 seed × semua sampel** (G38: 16/19). NO-PLAN start-state: 0.
- Lintas-gantry **3/7** gagal: selalu lengan gantry 1 yang TERJULUR (posisi akhir tugas terakhir g1 — runner tidak
  me-retract g1 sesudah tugas terakhirnya) vs lengan REST g2 yang berotasi ke tengah. Bergantung sampel (cabang IK MoveIt
  g1), maka k = 1 tidak cukup.
- Pulang lurus **3/7**: cabang IK pilihan MoveIt berakhir di tempat garis lurus ke REST memotong rak; oracle-∃ hanya
  menjamin ADA cabang baik, MoveIt (tujuan pose) memilih cabang sendiri.

### B6. Papan skor D300–D308

| # | Dugaan | Hasil |
|---|---|---|
| D300 | pose R35 layak g1 75–95 %, g2 60–85 %, rot 0 ≥ 95 % | ✗ g1 89.3 ✓, g2 76.6 ✓, tapi rot 0 g2 **93.9 %** |
| D301 | (c) menjelaskan ≥ 2/3 NO-PLAN | ✗ **1/3** (s16 t4, mekanisme terkonfirmasi log) |
| D302 | (c) menolak 5–30 % dari tuple G38 | ✗ **80/117 = 68 %** (tabel 58; lingkungan saja 30/117 = 26 %) |
| D303 | (i) ≥ 15/20; (i)–(iii) ≥ 12/20 | ✗ G40 **2/20** (G40e 6/20) |
| D304 | makespan median naik ≥ 5 % | ✓ G40 +13.3/+15.1 %; G40e median +14 % |
| D305 | plan-only 3/3 ≥ 8 seed | ✗ G40 0/2, G40e **1/6** |
| D306 | gagal traverse-lingkungan ≤ 1 seed | ✓ **0** |
| D307 | ≥ 30 % gagal lintas-gantry | ✓ 3/7 = 43 % |
| D308 | salah-terima tabel luar-sampel = 0 | ✓ 0 / 618 |

**Tally G40: 4 tepat / 5 meleset.**

## C. Keadaan akhir + langkah berikut (Rule 12)

- **Nol gerak HW.** Mock domain 77 naik 07:4x, mati bersih 09:28. Kode repo tidak diubah (`g40_make.py`, `g40_k3.py` baru;
  `sched.py`, `g31_sched.py`, `g31_screen.py`, `g38_make.py` apa adanya).
- **Kesimpulan:** kendala pose/traverse menutup celah 1 sepenuhnya (traverse-lingkungan 16/19 → 0/8) dengan harga
  makespan +14 %. Yang tersisa: (1) **lintas-gantry lengan** (celah 2) — kini penyebab utama, terukur 3/7; (2) **cabang IK
  MoveIt** (pulang lurus diblok rak 3/7, torsi transit 1/7) — oracle-∃ tidak mengendalikan cabang yang dipilih tujuan pose;
  (3) tugas tengah secara struktural langka: rotasi yang aman torsi = rotasi yang mendorong pasangan REST ke rak
  (14/20 seed tak terselesaikan bahkan dengan oracle tepat). Tabel G39 sebagai pengganti oracle **tidak cocok** untuk
  instans tepi (salah-tolak 50 % di tuple terpakai); tetap aman (salah-terima 0 / 618 luar sampel).
- 🔴 G38 s1 t2: rencana PLANNED dengan start-state tabrak (Fix-Start-State melompat) — celah G36 masih terbuka; G40 (a)
  mencegahnya untuk pose stop, tapi penyaring Fix-Start-State (G37) tetap prasyarat HW.
- **G41 (usul):** (a) tujuan tugas = **sendi** dari solusi oracle terpilih (cabang dengan garis pulang lurus CLEAR + torsi
  transit aman), bukan tujuan pose → menutup 4/7; (b) lengan–lengan lintas gantry di penjadwal: runner me-retract g1
  sesudah tugas terakhir ATAU kendala sapuan lengan REST/terjulur lawan pose gantry lain (sched_coll + lengan);
  (c) plan-only mock k ≥ 3 lagi. **Belum siap HW:** gerbang rotasi > 10° (HW baru ±10°, G30) dan origin g2 −35 mm
  (cek kamera) wajib sebelum gerak apa pun.
