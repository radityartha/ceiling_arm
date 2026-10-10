# P1 / G45 — K0 terukur sebagai prasyarat run (Fase 3 kampanye)

> Sesi 2026-10-10, kampanye Fase 3 (= G37 C). NOL gerak HW. Uji di mock domain 77 dan `fake_js` domain 78. **§A DIKUNCI
> SEBELUM mengukur** (salinan `docs/results/p1_g45/g45_A_locked.md` + sha256). §B menang atas §A dan DITULIS.

## A. Protokol — DIKUNCI

### A0. Pengungkapan

- G36b B7.0: K0 lisan "sama" padahal lengan 1/3/4 jauh dari REST dan g2 −34.9 mm / +2.91°. `run_g32` A3 awal = SATU
  snapshot `Watch` (dua penerbit `/joint_states` → gabungan bisa belum lengkap), lengan maks |q − REST| < 0.5°, rel ≤ 2 mm,
  rotasi ≤ 1.0° (1.0 persis lolos, `s26`); jari tidak dicek. HW menerbitkan hanya `*_right_finger_bottom_joint` (G36 B1,
  nilai HW ≈ 0.0001); mock menerbitkan semua sendi jari = 0. G36 B1: jari +0.05 rad sudah membuat start tabrak di MoveIt.
- `fake_js.py` (G32) tidak menerbitkan jari.

### A1. 🔒 Alat `scripts/k0_check.py`

- Berlangganan `/joint_states` selama `--seconds` (default 1.0, minimal 0.5 — kurang = `ap.error`), menggabung PER NAMA;
  per sendi: jumlah sampel, nilai median, rentang (maks − min).
- Diharapkan: 24 sendi lengan = REST; `t{g}_linear_joint` = `--rail g` (m); `t{g}_rotation_joint` = `--rot g` (deg, default
  0); 4 sendi `t{g}_a{k}_right_finger_bottom_joint` = 0.
- **TOLAK** bila salah satu: sendi diharapkan hilang atau < 3 sampel; |median − harapan| > batas; rentang > batas (bergerak).
  Batas: lengan **0.5°**, rel **2.0 mm**, rotasi **0.5°**, jari **0.02 rad**.
  (Rotasi diperketat dari 1.0 → 0.5: galat rotasi HW G30 ≤ 0.29°; MARGIN = TOLAK.)
- Keluaran: tabel per lengan (maks |q − REST| + sendi terburuk), rel, rotasi, jari, lalu satu baris `K0 LULUS|TOLAK: alasan`
  dan `--json` berkas; rc 0 lulus / 1 tolak / 2 argumen.
- **`run_g32`**: cek lengan A3 awal DIGANTI dengan subproses `k0_check.py --rail1 p0₁ --rail2 p0₂ --rot1 0 --rot2 0
  --seconds 1.0`; rc ≠ 0 → `finish('K0: …')` (berhenti sebelum event 0, juga di `--dry`). `s26` (rel 2 mm / rotasi 1.0° per
  event) TIDAK diubah.

### A2. 🔒 Uji

- **fake_js** (domain 78; salinan `docs/results/p1_g45/fake_js.py` = G32 + jari `--finger` default 0):
  (F0) semua cocok (REST, rel 1.50 / 0.00, rot 0/0) → LULUS; (F1) arm_3 +1° → TOLAK; (F2) rel g1 +3 mm → TOLAK;
  (F3) rot g2 +1° → TOLAK; (F4) jari +0.05 → TOLAK; (F5) sendi `t2_a2_joint_4` tidak diterbitkan → TOLAK;
  (F6) arm_1 +0.4° → LULUS (di dalam batas).
- **mock** (domain 77, nol gerak): (M0) harapan = keadaan mock (rel diukur dulu dengan `ros2 topic echo --once`, ditulis) →
  LULUS; (M1) harapan p0 g1 = 1.50 bila mock tidak di sana → TOLAK.
- **run_g32 --dry** dengan fake_js F0 → K0 LULUS, daftar event dicetak; dengan F1 → berhenti `K0:` sebelum event 0.
- **Gerbang 3:** semua kasus = harapan.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D370 | F0–F6 semua sesuai harapan (7/7). |
| D371 | M0 LULUS, M1 TOLAK. |
| D372 | run_g32 --dry: F0 lanjut ke daftar event, F1 berhenti `K0:` (2/2). |
| D373 | Gabungan 1.0 s melihat ≥ 3 sampel untuk semua 32 sendi di mock (kedua penerbit). |
