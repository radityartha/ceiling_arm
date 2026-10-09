# P1 / G39-OFF — tabel torsi di rangka pangkal lengan (semua rotasi, keempat lengan), OFFLINE

> Sesi 2026-10-09 (sesudah G38 dihentikan). Permintaan operator: hitung kemampuan untuk SEMUA rotasi gantry (sampai
> ±90° bila perlu) supaya jadwal baru tidak menghitung ulang terus. Hitung langsung = 3 143 576 tuple L1 (|rot| ≤ 90°,
> z ≤ 1.32) ≈ 36 hari di mesin ini → operator memilih **tabel pangkal-lengan grid 2 cm**; run G38 dihentikan (cache
> 5 456 tuple utuh, dilanjutkan sesudah G39). **§A DIKUNCI sebelum komputasi** (salinan
> `docs/results/p1_g39/g39_A_locked.md` + sha256). §B menang atas §A.

## A. Protokol — DIKUNCI

### A0. Dasar + yang sudah dilihat

- **G29 K-EQ** (p1_g29_rot.md): solve(xyz, L, arm, rot = r) ≡ solve(sasaran diputar, L, arm, rot = 0): `ok` 24/24,
  max |Δτ| 1.0e-10 N·m. Gravitasi invarian yaw; tool ke bawah dengan roll bebas juga invarian yaw.
- Pangkal lengan (`irm_sweep.base_pose`): p = (lin − y_p sin ψ, y_b + y_p cos ψ, 1.9525), ψ = π/2 + rot, y_p = ∓0.4,
  R = Rz(rot + yaw_off)·Rx(π); kedua lengan satu gantry berbeda tepat π.
- Posisi relatif rel = Rᵀ(sasaran − p) dari 400 000 tuple L1 sampel: |rel_xy| ≤ 0.72 m; rel_z hanya 6 nilai
  (z dunia 1.00…1.40 → 0.9525…0.5525). z = 1.40 dibuang (oracle⁗ G27) → 5 lapis.

### A1. 🔒 Tabel

Grid h = 0.02 m di (rel_x, rel_y), titik dengan |rel_xy| ≤ 0.76 m, 5 lapis rel_z = 1.9525 − z, z ∈ {1.00, 1.08, 1.16,
1.24, 1.32} → 22 545 titik. Tiap titik: oracle‴ PERSIS (`oracle2.solve(..., tilt=True, envelope=True)` +
`torq2(MARGIN)`) untuk **arm_1 di rel 0.80 m, rot 0**, sasaran dunia = p + R·rel. Disimpan per titik (ok, saturated,
n_sol, taumax) di jsonl yang bisa dilanjutkan.

### A2. 🔒 Aturan baca (konservatif)

Tuple (node, g, li, ri, s) → rel lewat base_pose lengan slot itu; lapis = rel_z tepat; **ok ⟺ keempat sudut sel grid
yang memuat rel_xy dihitung DAN ok**. Di luar cakram / sudut tak ada → tidak ok.

### A3. 🔒 Validasi + aturan pakai

Bandingkan aturan baca dengan SEMUA tuple tepat yang ada (G24 rot 0 `ok`, G29/G31 `ok3`, G38 `ok3`; z ≤ 1.32):
salah-terima (tabel ok, tepat tidak) dan salah-tolak. **Salah-terima = 0 → tabel boleh menggantikan oracle torsi**
(penjadwalan tanpa hitung ulang torsi; lingkungan tetap lazy per tuple terpakai). **Salah-terima > 0 → tabel hanya
penyaring awal** (menolak cepat; tuple yang dipakai jadwal tetap diverifikasi tepat). Lingkungan TIDAK ditabelkan
(bergantung posisi dunia).

### A4. 🔒 Dugaan (SEBELUM menghitung)

| # | Dugaan |
|---|---|
| D290 | Salah-terima **= 0** pada semua tuple tepat yang ada. |
| D291 | Salah-tolak **≤ 15 %** dari tuple tepat yang ok (harga aturan 4-sudut di tepi wilayah aman). |
| D292 | Pembangunan tabel **≤ 8 jam** (15 proses). |
| D293 | Membaca tabel untuk SEMUA 3.1 juta tuple L1 (\|rot\| ≤ 90°, z ≤ 1.32) **≤ 10 menit**. |
| D294 | Fraksi tuple L1 (\|rot\| ≤ 90°) yang ok menurut tabel **15–35 %**. |

sha256 `g39_A_locked.md` = `44b632f7…25fb9a0` (dikunci 2026-10-09 16:36, sebelum komputasi).

## B. Hasil terukur

### B0. Pemeriksaan sebelum membangun

- **K-EQ′** (`g39_table.py k`): 16 tuple tepat dari cache (kedua gantry, kedua slot, rot −35…+60°, 8 ok + 8 tidak) dihitung
  ulang sebagai arm_1 @ (0.80, rot 0) pada posisi relatif TEPAT-nya: **16/16 sama** → satu tabel melayani keempat lengan
  dan semua rotasi. ~5.5 s/titik/proses.
- Grid A1 menghasilkan **22 565** titik (perkiraan A0 22 545 dari cakram berbeda sedikit; tidak mengubah apa pun).
- Build 16:38 → **21:37, 4.98 jam** (`setsid nohup`, 1.2 titik/s), [log](results/p1_g39/g39_build.log); validasi dan G38 ulang
  dirantai otomatis ([g39_chain.sh](results/p1_g39/g39_chain.sh)) karena operator offline.

### B1. Validasi (A3) — [log](results/p1_g39/g39_validate.log)

- Tabel: 22 565 titik, **ok 667 (3.0 %)** — daerah aman torsi di bawah pangkal kecil.
- Terhadap SEMUA 55 089 tuple tepat yang ada (z ≤ 1.32; 7 410 ok): **salah-terima 0** (rot 0: 0 / 21 696; rot ≠ 0:
  0 / 33 393); salah-tolak **1 905 = 25.7 %** dari yang ok (rot 0: 435, rot ≠ 0: 1 470) — harga aturan 4-sudut di tepi.
- Baca SEMUA 3 143 576 tuple L1 (|rot| ≤ 90°, z ≤ 1.32): **2.2 s**; ok 133 986 (**4.3 %**).
- **Aturan pakai A3: salah-terima = 0 → tabel boleh menggantikan oracle torsi.** Harga: ~1/4 tuple aman dibuang
  (jadwal bisa sedikit lebih buruk dari optimum oracle tepat, tidak pernah lebih berbahaya — pada tuple yang ada).

| # | Dugaan | Hasil |
|---|---|---|
| D290 | salah-terima = 0 | ✓ 0 / 55 089 |
| D291 | salah-tolak ≤ 15 % | ✗ 25.7 % |
| D292 | build ≤ 8 jam | ✓ 4.98 jam |
| D293 | baca semua ≤ 10 min | ✓ 2.2 s |
| D294 | ok 15–35 % | ✗ 4.3 % |

**Tally G39: 3 tepat / 2 meleset.** ⚠ Salah-terima 0 hanya pada tuple yang PERNAH dihitung tepat (sampel yang
dipilih generator lama + G29/G31/G38, bukan seragam) — klaim di luar sampel itu belum terbukti.
