## A. Protokol — DIKUNCI

### A0. Yang sudah dilihat sebelum mengunci (pengungkapan)

- Jadwal G40e 6 seed (`g40e_candidates.json`): tugas ganda satu lengan per stop di seed 0 (g1 stop 1, g2 stop 0), 10
  (g1 stop 0), 19 (g2 stop 0, 2). `g31_screen.events`: retract hanya sebelum traverse; tugas berurutan satu lengan di stop
  yang sama mulai dari ujung tugas sebelumnya; semua event g1 lalu semua event g2 (serial; `run_g32` memakai daftar yang
  sama) — **tidak ada retract sesudah tugas terakhir g1**.
- Log plan-only G40/G40e (B5 G40). `_plan_and_screen` menerima tujuan sendi dict (`_joint_goal`, toleransi 0.2°).
- Pilz PTP (garis sendi lurus) dimuat default oleh `MoveItConfigsBuilder`, tetapi `joint_limits.yaml` 30/32 sendi
  `has_acceleration_limits: false` → Pilz tidak dapat dipakai tanpa mengubah konfigurasi; **tidak** dipakai. Tugas tetap
  OMPL (pipeline default) dengan tujuan sendi; garis lurus di predikat = PROKSI untuk lintasan OMPL (lihat A2).
- Tidak ada perhitungan baru sebelum mengunci (hanya baca kode + jadwal + log).

### A1. 🔒 Predikat tuple (c41) dan tujuan sendi

Tuple (node, g, li, ri, s) layak ⟺ L1 ∧ pose (a) G40 ∧ oracle‴ tepat `torq2` ok (∀ cabang, seperti G40e; tabel G39 TIDAK
dipakai) ∧ **∃ cabang** q ∈ `O2.solve(..., keep_solutions=True)['solutions']` (semua roll), dievaluasi berurutan menurut
max_i |q_i − REST_i| naik (seri = indeks), dan **cabang terpilih = yang PERTAMA lolos semua**:

- (e1) titik akhir = G40 env2: `EnvChecker.check(only=t{g}_)` ≥ 50 mm ∧ bebas-kotak (lambung `t{g}_` vs `column_boxes`).
- (e2) garis pulang lurus `return_rest._rest_line(q)` (60 titik kosinus; dibalik = transit REST → q), titik q ikut,
  lengan pasangan REST, gantry lain TIDAK dimasukkan: `EnvChecker.screen_trajectory` = CLEAR ∧ `InterArmChecker(gantry g)
  .screen_trajectory` = CLEAR (= dua dari tiga penyaring `screen_retract`/`plan_retract`) ∧ bebas-kotak di setiap titik.
- (e3) torsi sepanjang garis: ∀ titik, `O2.gravity(arm, q_pt, lin, rot)` + OFFSET + MARGIN ≤ LIM (aturan `torq2`).

Tujuan tugas = {`{prefix}joint_i`: q_i} cabang terpilih, disimpan di jadwal (`goal` per tugas). Yang tidak berubah: (a), (b)
traverse = G40 (cache tepi G40 disalin, tepi baru ditambahkan ke salinan G41), DP/klaim eksak = G40 A2 dengan (c) →
(c41). (c41) ⊆ (c40e) ⇒ seed yang (i)-nya gagal di G40e tetap gagal (tetap dijalankan, 20 seed).

### A2. 🔒 Perubahan runner/penyaring (plan-only)

- `g31_screen.events(..., final_retract)`: bila True, event `retract` gantry g ditambahkan SESUDAH tugas terakhir g
  (sebelum event gantry berikutnya). `g31_screen.walk`: tugas memakai `ev['goal']` (dict sendi) bila jadwal punya `goal`,
  selain itu pose seperti dulu. Default kedua perubahan = perilaku lama (run_g32/varian lama identik).
- **Celah yang diketahui (TIDAK eksak):** OMPL boleh tidak mengikuti garis lurus (transit torsi/env diperiksa ulang oleh
  penyaring di lintasan sebenarnya, retry 3); tugas ke-2+ satu lengan di stop sama mulai dari ujung tugas sebelumnya, bukan
  REST (titik akhirnya tetap terverifikasi (e1)–(e3)); pasangan lengan di predikat = REST, di runner bisa terjulur (tugas
  lengan lain di stop sama); lintas-gantry di luar (c41). Runner HW (`reach_dwell_probe` CLI `--target`) belum menerima
  tujuan sendi → G41 tidak dapat dieksekusi di HW apa adanya.

### A3. 🔒 Pengukuran berurutan

1. **M1 (permintaan 2, OFFLINE, sebelum jadwal ulang)**: pada setiap (seed, sampel) plan-only G40/G40e yang mencapai akhir
   event g1 (jsonl), keadaan akhir g1 = ujung rencana tugas terakhir g1; (M1a) `return_rest.plan_retract(1, [arm_1,
   arm_2], ..., plan_fn=None)` lurus = CLEAR?; (M1b) semua traverse g2 jadwal itu `sweep_rot(RotCrossChecker)` dengan g1
   REST di pose akhirnya. **Aturan keputusan:** M1 menutup lintas-gantry ⟺ untuk KETIGA gagal lintas-gantry (G40 s8,
   G40e s3, s18) M1a CLEAR ∧ M1b semua CLEAR. Tertutup ⇒ G41 memakai `--final-retract`; tidak ⇒ kendala lintas-gantry di
   penjadwal dirancang dan DITULIS di §B sebelum dijalankan.
2. **K**: (K1) (e2) implementasi G41 vs `plan_retract` lurus pada ≥ 30 keadaan ujung tugas tersimpan (jsonl G40/G40e):
   verdict + min se-gantry dan lingkungan identik (|Δ| ≤ 0.1 mm). (K2) (e3) di titik q = `O2.gravity` (bit-identik dengan
   `taumax` komponen bila q adalah cabang pengikat — cukup: torsi titik q sama dengan `gravity(q)` dipanggil langsung).
   (K3) (c41) pada keadaan ujung yang G40e tolak RETRACT-BLOCKED tidak dapat diulang (rencana ditolak tidak disimpan) —
   diganti: (c41) pada 4 tuple gagal non-lintas G40/G40e (s0 t3, s10 t5, s19 t3 ×2) harus menerima dengan cabang yang
   garisnya CLEAR, atau menolak (dilaporkan).
3. **Jadwal ulang** 20 seed (kriteria (i)–(iii) G38 A4) → `g41_candidates.json`; (iv′) offline untuk tiap seed lolos:
   traverse serial lintas-gantry seperti M1b (g2 vs g1 REST di pose akhir; g1 vs g2 REST di p0) — dilaporkan, tidak
   dipakai untuk optimasi ulang.
4. **Plan-only mock** `g31_screen.py --variant G41 --repeats 3 [--final-retract]` pada semua seed lolos (i)–(iii).
   Lolos ⟺ 3/3 PLANNED. Atribusi = verdict terakhir. Diagnostik: deviasi maks lintasan OMPL dari garis REST → q.

### A4. 🔒 Dugaan (SEBELUM mengukur)

| # | Dugaan |
|---|---|
| D310 | M1 menutup **3/3** gagal lintas-gantry (retract akhir lurus CLEAR dan traverse g2 CLEAR vs g1 REST). |
| D311 | Dari tuple yang G40e verifikasi layak (ok3 ∧ env2) dan dihitung ulang di G41, (c41) menerima **60–90 %**. |
| D312 | (i)–(iii) G41 **4–6 / 20** (≤ 6 pasti, ⊆ G40e). |
| D313 | Makespan median vs G40e (seed lolos keduanya) naik **≤ 5 %**. |
| D314 | Plan-only mock 3/3 PLANNED **≥ 3** seed. |
| D315 | RETRACT-BLOCKED di plan-only = **0** (titik akhir tugas deterministik dan terverifikasi). |
| D316 | TORQUE-UNSAFE transit di plan-only **≤ 1** kejadian. |
| D317 | Biaya (c41) per tuple median **≤ 10 s** CPU. |

