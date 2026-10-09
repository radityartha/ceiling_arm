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
