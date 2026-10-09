# P1 / G38-OFF — instans sulit: 3 tugas di TENGAH (zona bersama) + 3 dekat DINDING, rotasi R35 (OFFLINE, nol gerak)

> Sesi 2026-10-09 (sesudah G36b, sesi yang sama). Permintaan operator: titik sasaran instans G22–G36 "terlalu mudah"
> untuk menguji metode → sasaran di area tengah atau dekat dinding. Keputusan operator (ditanya): tengah = pita
> **|y| < 0.15 m** di antara kedua gantry; dinding = **0.10–0.20 m** dari peta D455; **3 tengah + 3 dinding** per instans;
> offline sesi ini. Setelah temuan A0 (tengah mustahil di rotasi 0): **pakai rotasi R35** (|rot| ≤ 35°).
> G37 (penyaring Fix-Start-State, cache geometri, K0 terukur) tetap prasyarat HW; G38 tidak menyentuh HW.
> **§A ditulis dan DIKUNCI SEBELUM pengukuran** (salinan `docs/results/p1_g38/g38_A_locked.md` + sha256). §B menang atas §A.

## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

Dari cache yang ADA (tanpa komputasi baru), node peta kapabilitas `cap_g{1,2}_rail160` (3 132 node, x 0–2.0, y ±0.6,
z 1.0–1.4; gantry di y = ±0.36):
- Target seed 36 (G32–G36b) semuanya di |y| 0.32–0.46 — di bawah gantry sendiri.
- **Rotasi 0:** 174 node |y| < 0.15 di cache oracle‴ G24 → **0 lolos**, slack terbaik −1.04 N·m, pengikat `joint_2`.
  Node aman dari KEDUA gantry: **0 / 780**. Sel tidak punya zona bersama di rotasi 0 — sebab instans lama "mudah".
- **Rotasi ≠ 0** (cache G29, rot ±30/±60/±90): tuple tengah 472/1 407 ok3; 43/51 node tengah ok di ≥ 1 tuple,
  **39/51 dari kedua gantry**; rot minimum yang membuat ok = 30°.
- Dinding (jarak ke voxel peta g34n 0.10–0.20 m): 180 node (L1); 40 di cache G24, 18 ok. Rintangan dekat kotak tugas:
  rak/dinding samping y ≈ −0.9…−0.6 (x 0.8–1.8) dan rak ujung x ≈ 2.0–2.2.
- Oracle‴ tidak memeriksa lingkungan sama sekali (hanya torsi ∀-cabang, ∀-roll, tilt).

### A1. 🔒 Kolam target

- **TENGAH** = node dengan |y| < 0.15 dan z ≤ 1.32 (oracle⁗ G27: lapis z = 1.40 dibuang).
- **DINDING** = node dengan jarak titik ke voxel peta g34n ∈ [0.10, 0.20) m, |y| ≥ 0.15, z ≤ 1.32 (saling lepas dengan TENGAH).

### A2. 🔒 Predikat tuple dan penarikan

Tuple = (node, g, rel li, rot ri ∈ R35, slot s). **Layak** ⟺ oracle‴ torsi ok (rot 0: cache G24 `ok`; rot ≠ 0: `ok3`
G29/G31 atau dihitung) **DAN lingkungan-∃**: ada ≥ 1 solusi IK oracle (roll mana pun) yang titik akhirnya, dengan lengan
lain REST, gantry lain di 0/0, berjarak ≥ 50 mm (`EnvChecker.check`, geometri lengan tugas saja) dari peta.
Penarikan per seed s ∈ {0…19}: `rng = default_rng(s)`; 3 dari TENGAH lalu 3 dari DINDING, seragam, tanpa duplikat.
Kandidat diterima ⟺ ada tuple layak di antara **40 tuple L1-benar pertama** urut (|rel − x_node|, −|rot|, g, s)
(batas biaya; penolakan = "tidak ditemukan dalam 40", dihitung dan dilaporkan). Tolak > 10 000 per seed → seed gagal.

### A3. 🔒 Penjadwalan

`g31_sched` apa adanya (P1′, rot_cmd 0.9235, lazy-exact), set pose REF.lin × **R35**, dengan dua perubahan di skrip G38
(bukan di `g31_sched`): (1) **p0 g1 = (1.50, 0°)** (PARKIR, B6.6 G36), g2 = (0, 0°); (2) verifikasi lazy mencakup tuple
rot 0 yang belum ada di cache G24 dan **lingkungan-∃** setiap tuple terpakai — gagal → nol, selesaikan ulang sampai
semua tuple terpakai terverifikasi (relaksasi L1 ⊇ layak → optimum terverifikasi = optimum sejati). Kontrol: varian
R10 dengan predikat yang sama pada **5 seed pertama yang tertarik** (biaya; diharapkan (i) GAGAL untuk tugas tengah).

### A4. 🔒 Kriteria (sama dengan G22–G31) + laporan

(i) terselesaikan; (ii) kedua gantry ≥ 1 pindah; (iii) `sched_coll.schedule_conflict` = None. Dilaporkan per seed: tolak
per kolam, gantry + rot yang melayani tiap tugas tengah, makespan, konflik. Gerbang HW (di luar lingkup): `pose_to_g`
S23″ menolak |rot| > 10°, rotasi HW baru diverifikasi ±10° (G30) — jadwal R35 butuh gerbang rotasi baru sebelum HW.

### A5. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D280 | ≥ 15 / 20 seed selesai ditarik (tidak ada seed melewati 10 000 tolak). |
| D281 | Laju tolak kandidat TENGAH **10–40 %**. |
| D282 | Laju tolak kandidat DINDING **40–75 %** (torsi 55 % gagal di cache + lingkungan). |
| D283 | (i) R35 terselesaikan pada **semua** seed yang tertarik; kontrol R10 (i) **gagal** pada 5/5 seed kontrol. |
| D284 | Setiap tugas tengah di jadwal optimum dilayani pada **\|rot\| ≥ 30°**. |
| D285 | (iii) konflik terjadi pada **≥ 30 %** seed (zona bersama). |
| D286 | Lolos (i)–(iii): **≥ 5 / 20** seed. |

sha256 `g38_A_locked.md` = `16254db0…5fb9a0` (dikunci 2026-10-09 15:10, sebelum komputasi baru).

### A6. 🔒 Addendum (G39b, 2026-10-10, SEBELUM plan-only) — plan-only MOCK seed yang lolos (i)–(iii)

Mock domain 77 (`use_fake_hardware:=true enable_gantry_bridge:=false`, env_map:=true), `g31_screen.py --variant R35
--repeats 1 --plan g38_candidates.json` untuk 19 seed, apa adanya (retract_check + 3 percobaan tugas, saring S18 / lintas /
torsi / lingkungan, urutan event g1 lalu g2 seperti runner). Atribusi gagal = verdict terakhir; INTERARM-COLLIDE dengan
pasangan t1_*↔t2_* = **lintas-gantry**. Satu sampel per seed (penyaringan, bukan laju).

| # | Dugaan |
|---|---|
| D295 | PLANNED ujung-ke-ujung **≤ 10 / 19** seed. |
| D296 | **≥ 30 %** seed yang gagal, gagal pada tabrakan lengan **lintas-gantry**. |
| D297 | **≥ 1** seed gagal pada ENV-* (tugas dinding). |
| D298 | Gagal akhir TORQUE-UNSAFE **≤ 2** seed. |

## B. Hasil terukur

### B0. Pemeriksaan + penyimpangan dari §A (DITULIS)

- **K** (`g38_make.py k`): field torsi `_job` G38 **bit-identik** dengan cache G24 (rot 0) dan G29 (rot ≠ 0), 6/6; FK model
  EnvChecker ke target ≤ 0.12 mm → model oracle dan model lingkungan sepakat.
- **B1 — urutan kandidat §A2 salah, diganti (B menang atas A).** §A2 mengurutkan 40 tuple per kandidat menurut
  (|rel − x|, −|rot|). Pangkal lengan duduk **±0.4 m di x dari titik rel** (pelat dudukan, `irm_sweep.base_pose`), dan
  rotasi θ menggeser pangkal ke tengah 0.4·sin θ (≈ 0.23 m di 35°). Urutan terkunci menghabiskan 40 tuple pada pangkal
  0.4 m dari target: **2 lolos dari 693 tuple**, seed 0 belum selesai sesudah 9 min ([log](results/p1_g38/g38_make_order1_killed.log)).
  Dihentikan; urutan diganti ke **jarak xy pangkal–target** (urutan `tuples_for` G29). Cache tuple (693) dipakai lagi:
  hasil per tuple tidak bergantung urutan. Predikat "∃ dalam 40" kini bermakna "∃ di 40 pangkal terdekat".
- **B2 — urutan kerja (bukan hasil):** lazy R35 seed 0 butuh ≥ 24 iterasi / ≥ 19 min (relaksasi L1 lemah di rot ≠ 0).
  `run` diubah: tarik semua seed → R35 semua seed → kontrol R10 (5 seed pertama) di akhir; hasil ditulis per seed.
  Perkiraan total ~10 jam (`setsid nohup`, cache bisa dilanjutkan).

- **B3 — run dihentikan untuk G39, lalu dirantai ulang (bukan hasil):** run ke-3 dihentikan 16:3x demi CPU build G39
  ([log](results/p1_g38/g38_make_run3_stopped_for_g39.log)); diulang otomatis 21:37 sesudah G39 selesai
  ([g39_chain.sh](results/p1_g39/g39_chain.sh)), oracle TEPAT (kode tidak diubah), cache dipakai lagi. Selesai 05:08 (7.5 jam).
- **B4 — kontrol R10 tidak masuk JSON:** `run` melompati `json.dump` saat (i) gagal → hasil R10 hanya di
  [log](results/p1_g38/g38_make.log). Dibaca dari log.

### B1. Penarikan + penjadwalan R35 (oracle tepat) — [log](results/p1_g38/g38_make.log), [JSON](results/p1_g38/g38_candidates.json)

- **20/20 seed tertarik.** Tolak TENGAH **20/80 = 25.0 %**, DINDING **1/61 = 1.6 %**, duplikat 0.
- **R35 (i) 20/20**; (ii) 19/20 (seed 15: g1 tanpa tugas, semua 6 ke g2, 0 pindah g1); (iii) konflik struktur gantry **0/20**.
  **Lolos (i)–(iii): 19/20.** Makespan P1′ median **328.7 s** (220.3–449.1); iterasi lazy median 34 (maks 65);
  ~12–40 min/seed. Pembagian tugas g1/g2: 3/3 ×10, 2/4 ×5, 1/5 ×3, 4/2 ×1, 0/6 ×1.
- **Kontrol R10 (5 seed pertama): 5/5 (i) GAGAL** — "task k is infeasible on every gantry" (tugas tengah).
- **Tugas tengah (60):** dilayani g1 32 / g2 28; \|rot\| 5° ×1, 15° ×1, 20° ×6, 25° ×10, **30° ×24, 35° ×18**
  (≥ 30°: 42/60). 65/86 pose berhenti memakai \|rot\| > 10°.
- Predikat lingkungan hidup: **37** tuple lolos torsi tetapi DITOLAK lingkungan (dari 3 781 ok3; cache 35 464).

### B2. 🔴 "0 konflik" bukan bukti aman — model konflik tidak melihat lengan

`sched_coll.schedule_conflict` (G9 A2) memeriksa **hanya struktur gantry** (batang rotasi 0.80 × 0.08 + pelat dudukan,
proyeksi XY) — **lengan tidak termasuk**. Di instans lama (tiap tugas di bawah gantry sendiri) itu tidak pernah penting;
di instans G38 lengan kedua gantry menjulur ke pita tengah yang SAMA. Jadi (iii) 0/20 hanya berarti batang tidak bertabrakan;
tabrakan lengan–lengan lintas gantry di zona tengah **tidak dimodelkan penjadwal** — hanya penyaring rencana
(`CrossGantryChecker` di probe/plan-only) yang melihatnya. Ini kelemahan metode yang memang ingin diungkap instans sulit;
diukur di plan-only (B5).

### B3. Papan skor D280–D286

| # | Dugaan | Hasil |
|---|---|---|
| D280 | ≥ 15/20 seed tertarik | ✓ 20/20 |
| D281 | tolak TENGAH 10–40 % | ✓ 25.0 % |
| D282 | tolak DINDING 40–75 % | ✗ **1.6 %** — predikat ∃-tuple di 40 pangkal terdekat hampir selalu menemukan satu tuple aman |
| D283 | (i) R35 semua; R10 gagal 5/5 | ✓ 20/20; 5/5 |
| D284 | tugas tengah semua \|rot\| ≥ 30° | ✗ **42/60** (5–25° ×18) — A0 hanya punya sampel G29 di ±30/60/90 |
| D285 | konflik (iii) ≥ 30 % seed | ✗ **0/20** — dan B2: model konflik tak melihat lengan |
| D286 | lolos (i)–(iii) ≥ 5/20 | ✓ 19/20 |

**Tally G38-OFF: 4 tepat / 3 meleset.**

### B4. Plan-only MOCK (A6) — 🔴 1/19, lalu 0/1 stabil

sha256 `g38_A6_locked.md` = `0b579618…514518c` (dikunci 2026-10-10 06:56). Mock domain 77 (launch 2711537, SIG_DFL, SigIgn
`0x1001001`), `env_static_map` di scene True; matikan bersih (`node list --no-daemon` 0; crash dump shutdown move_group
baseline dihapus). [log](results/p1_g38/g38_planonly.log), [JSON](results/p1_g38/g38_planonly.json).

| gagal pada | seed |
|---|---|
| **traverse vs lingkungan** (lengan REST, sapuan (lin, rot)) | **16**: ENV-COLLIDE −17.2…−7.9 mm ×7 (g2), ENV-MARGIN 2.0…48.7 mm ×9 (g1 7, g2 2) |
| tugas dinding NO-PLAN 3/3 (0.7 s, MoveIt menolak cepat) | 2 (seed 1, 16) |
| antar-lengan lintas gantry | **0** (tidak pernah tercapai) |
| torsi | 0 |
| **PLANNED** | **1** (seed 19) — ulang k = 3: sampel 1 **gagal** (t4 dinding: RETRACT-BLOCKED → NO-PLAN ×2) ([log](results/p1_g38/g38_planonly_s19k3.log)) |

Traverse yang gagal hampir semua menuju \|rot\| 20–35° (dari 0 atau antar-stop), geometri pelanggar = **jari lengan REST**
(t1_a1/a2 di g1, t2_a1/a2 di g2).

### B5. Mekanisme — rotasi untuk tengah mendorong pasangan lengan ke dinding

Batang rotasi berputar di tengahnya: rotasi θ membawa satu pelat dudukan **0.4·sin θ ke arah tengah** (≈ 0.23 m di 35°) dan
pelat lainnya **sejauh yang sama ke arah dinding**. Lengan REST menggantung sampai z ≈ 0.96 m, jadi lengan pasangan masuk ke
rak samping (y −0.6…−0.9, x 0.8–1.8; sisi g2) atau benda di y 0.75–0.9 (x 0.2–0.6; sisi g1). Penjadwal P1 **tidak** memeriksa
pose gantry (lin, rot) maupun sapuan traverse terhadap lingkungan — predikat lingkungan G38 hanya untuk titik akhir LENGAN TUGAS.
Kedua celah model penjadwal yang terungkap instans sulit:
1. **pose/traverse gantry vs lingkungan** (B4, 16/19) — tidak dimodelkan;
2. **lengan–lengan lintas gantry** (B2) — tidak dimodelkan (belum teruji: traverse gagal lebih dulu).
3. Tugas dinding: predikat lingkungan-∃ (Euklides 50 mm, lengan tugas saja) lebih longgar daripada scene MoveIt (kotak
   +5 cm L∞, semua lengan) → NO-PLAN (seed 1, 16, 19) — ketidakcocokan yang sama dengan G36 B1.

### B6. Papan skor D295–D298

| # | Dugaan | Hasil |
|---|---|---|
| D295 | PLANNED ≤ 10/19 | ✓ 1/19 (dan tidak stabil) |
| D296 | ≥ 30 % gagal lintas-gantry | ✗ 0 % — traverse gagal LEBIH DULU (B5); belum teruji |
| D297 | ≥ 1 gagal ENV-* | ✓ 16 traverse + 2–3 NO-PLAN dinding |
| D298 | gagal akhir TORQUE ≤ 2 | ✓ 0 |

**Tally A6: 3 tepat / 1 meleset. Total G38: 7 / 4.**

## C. Keadaan akhir + langkah berikut (Rule 12)

- **Nol gerak HW.** Mock domain 77 naik 06:57, mati bersih ~07:15. Kode tidak diubah (hanya `g38_make.py` baru).
- **Kesimpulan:** instans sulit TIDAK bisa dijalankan dengan penjadwal P1 sekarang — bukan karena torsi, tetapi karena
  penjadwal buta lingkungan untuk pose/traverse gantry. **Belum siap HW** (juga: gerbang rotasi > 10°, origin g2 −35 mm,
  G37).
- **G40 (usul):** tambahkan ke penjadwal (a) kelayakan pose gantry (lin, rot) dengan lengan REST vs peta (EnvChecker,
  ≥ 50 mm) dan (b) sapuan traverse sebagai biaya/kendala tepi (rect sweep seperti `pose_to_g`), (c) samakan predikat
  lingkungan tugas dengan scene MoveIt (kotak +5 cm L∞, semua lengan); jadwal ulang 20 seed (torsi dari tabel G39);
  plan-only mock k ≥ 3. Lengan–lengan lintas gantry lalu diukur.
