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

