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
- ⏳ Build mulai 16:38 (`setsid nohup`, ~1.3 titik/s → ~5 jam), log `results/p1_g39/g39_build.log`. Sesudahnya:
  `python3 g39_table.py validate` (A3: salah-terima / salah-tolak, baca semua tuple ±90°), lalu G38 dijalankan ulang.
