# Experiment Plan — Energy-Aware Arm & Base Selection (IEEE conference, sim-only)

> Dokumen rujukan desain eksperimen. Dikunci dulu di sini **sebelum** mengubah kode,
> supaya penambahan (`selection_mode`, seed `voxel`, script localization-error)
> menghasilkan kolom/metrik yang persis dipakai tabel paper. Angka posisi/trial di
> bawah adalah usulan — sesuaikan dengan waktu, tapi jangan turun di bawah minimum
> yang ditandai.

## 1. Kesepakatan paper

- **Judul (final, updated 2026-07-16):** *Energy-Aware Arm and Base Selection for a
  Dual-Gantry Quad-Arm Ceiling Robot Using a GNG Capability Map*
  (scope naik: 2 gantry / 4 lengan; lihat paper_draft.md).
- **Bintang / kontribusi utama:** fungsi biaya energi **J** yang memilih **lengan
  mana + konfigurasi 8-DOF mana** (termasuk perpindahan gantry yang mahal), di atas
  capability map GNG dengan **gantry sebagai DOF kelas-satu**.
- **Enabler (bukan kontribusi):** YOLOE (persepsi open-vocab) + Isaac Sim digital twin.
- **Data:** **simulasi penuh** (Isaac digital twin + offline FK/IK). Tidak ada
  eksperimen hardware.

### Klaim yang harus dibela data
1. Gantry sebagai DOF memperluas reachable workspace (→ E1). **DONE: ~4× @res 0.05**
   (rail-2.0 rerun; lihat experiment_results.md).
2. ~~GNG menghasilkan seed IK yang lebih baik dari no-seed/KDL-restart/voxel.~~
   **DIBATALKAN 2026-07-16 (user "E2-a"): hasil E2 NEGATIF** — GNG seed TIDAK memberi
   manfaat IK (kalah dari zero-seed di semua regime; E2/E2-b). GNG **bukan** klaim seed
   IK. Peran GNG = substrat SELEKSI arm+base (klaim 3) + representasi workspace (klaim 1).
   E2 disimpan sebagai negative result di experiment_results.md, **bukan tabel paper**.
3. Pemilihan energi-J mengalahkan nearest / fixed / random dalam travel/energi
   gantry+lengan (→ E3, **utama** — kini memilih dari **4 lengan** di 2 rail).
4. Pipeline utuh (persepsi→reachability→pick) berjalan di twin (→ E6, + E4/E5 pelengkap).

### Justifikasi sim-only (untuk Limitations)
Kontribusi bersifat **metodologis** (algoritma seleksi + representasi map); digital
twin dengan physics adalah platform evaluasi yang sah. Platform fisik (2 gantry, 4
Kinova Gen3 Lite, Livox Mid360) **nyata** dan digerakkan URDF yang sama → sertakan
**1 foto workcell asli** + kalimat "hardware validation is future work". Sim-to-real
gap diakui eksplisit, tidak disembunyikan.

### Zacharias
**Bukan** section/eksperimen terpisah. Turun jadi 1–2 kalimat Related Work yang
membedakan GNG (membawa `q`+manipulability+`hold` per node) dari voxel biner. ~~(b)
baseline seed ringan `voxel` di E2~~ **DIBATALKAN** bersama E2 (IK-seeding dropped).
Zacharias penuh (sphere-map reachability index) **tidak** diimplementasikan — ROI rendah,
arm-only, apple-to-oranges dengan base DOF.

## 2. Struktur paper (6 hlm IEEE) — draft final di paper_draft.md pakai I–VII
1. Introduction — ceiling **quad-arm** (2 gantry / 4 lengan) workcell; masalah: lengan mana + base di mana, hemat energi.
2. Related Work — reachability/capability maps, redundancy resolution, mobile-base placement, multi-arm coordination.
3. System — platform, sim, persepsi, planning stack.
4. Base-Aware GNG Capability Map — GNG `[task | q]`, gantry DOF, boundary-seeding, manipulability layer (**figure + tabel properti map = E0**).
5. **Energy-Aware Selection** — fungsi J, pooling, round-robin search (inti).
6. Experiments — **E3 (utama)** → E1 → E6 → E4/E5. (~~E2~~ dropped.)
7. Conclusion + Limitations (sim-only, hardware future work) + foto workcell asli.

## 3. Matriks eksperimen

| ID | Prioritas | Menjawab klaim | Output paper | Butuh kode baru? |
|----|-----------|----------------|--------------|------------------|
| E0 | Karakterisasi map | (menopang judul) | Figure map + tabel properti | tidak |
| ~~E2~~ | ~~Pendukung inti~~ | ~~2~~ | **DROPPED (negatif)** | — |
| E3 | **Bintang** | 3 | Tabel + boxplot utama | `selection_mode` (kecil) |
| E2 | Pendukung inti | 2 | Tabel utama | seed `voxel` (kecil) |
| E1 | Justifikasi map | 1 | Tabel + bar | tidak |
| E6 | Enabler/demo | 4 | Tabel sukses + breakdown gagal | tidak |
| E4 | Pelengkap | 4 | Tabel | script localization-error (kecil) |
| E5 | Pelengkap | 4 | Confusion matrix | tidak |

---

## E0 — Karakterisasi GNG map (untuk Section 3, membela judul)

- **Tujuan:** membuat map **terlihat** sebagai artefak — judul menyebut "Table-Aware GNG
  Capability Map", jadi map harus dikarakterisasi di satu tempat, bukan hanya implisit di
  E1–E3. Bukan eksperimen berat: 1 figure + 1 tabel dari artefak yang **sudah ada**.
- **Figure (RViz):** `ros2 launch reachability_gng view_gng.launch.py model_path:=/tmp/arm1_model.npz`
  → cloud node arm_1 (+arm_2), diwarnai **manipulability** (biru=rendah/tepi, merah=dexter),
  **boundary shell** terlihat. `color_by:=hits` untuk versi kepadatan reachability.
- **Tabel properti map** (angka dari `train.py`/`build_maps.sh`/`_stats.npz`, per arm):
  #node (shell pinned + interior), #FK sample, median node spacing (m), reachable volume (m³,
  dari E1), waktu training (s), `task_dim`, DOF vektor `q` (8 = 2 gantry + 6 arm).
- **Variabel bebas:** tidak ada (deskriptif). Opsional: kurva #node vs cakupan/edge-fidelity
  dari sweep `LAM` (mis. LAM 60/110/160/320) — 1 kurva kecil kalau ruang cukup.
- **Ulangan:** deterministik → 1×; kalau LAM sweep, 1× per nilai.
- **Log:** tangkap ringkasan stdout `train.py` (#node, waktu) + screenshot RViz.
- **→ Paper:** Fig. capability map di Section 3 (System & Map) + Table properti map.
  E1 (volume + boundary-seeding ablation) menempel ke tabel ini sebagai bukti kuantitatif.

---

## E1 — Perluasan workspace oleh gantry DOF

- **Tujuan:** buktikan `t1_linear`/`t1_rotation` sebagai DOF menambah reachable volume.
- **Tool:** `python3 -m reachability_gng.eval volume --datasets locked.npz active.npz --res 0.05`
  - `locked.npz` = dataset FK gantry dikunci di home; `active.npz` = gantry disampel 0–3.0 m & ±180°.
  - Generate dua dataset via `data_gen` (kunci vs sampel rail/rotasi di config).
- **Variabel bebas:** {gantry locked, gantry active}; resolusi voxel {0.03, 0.05, 0.08} (sensitivitas).
- **Kondisi tetap:** `--n` sama untuk kedua dataset (mis. 80000); arm_1.
- **Metrik:** reachable volume (m³), rasio gain (active/locked), bbox extent x/y/z.
- **Ablation boundary-seeding:** `BOUNDARY=0` vs `600` di `build_maps.sh` → **edge shortfall (m)**
  (target: ~0.26 m → 0.00 m). Figure kuat.
- **Ulangan:** deterministik (FK) → 1× per kondisi.
- **Output/log:** `eval volume` hanya *print* → **tangkap stdout ke file**
  (`... | tee /tmp/e1_volume.txt`); catat manual ke tabel.
- **→ Paper:** Table "Reachable-workspace gain" + bar chart; ablation shortfall di teks/figure.

## E2 — GNG-seeded IK benchmark — ❌ DROPPED (negative result, 2026-07-16)

**STATUS: run, negative, cut from the paper (user "E2-a").** Measured against live
`move_group` on arm_1 + arm_3 (N=500): GNG seed = 78%/75% success vs zero-seed 91%/92%
vs random-restart 99%, and GNG was slower than zero-seed. Stratified by z-shell and
node-distance: GNG loses in **every** regime, even for poses sitting ON a node. E2-b
(pipeline-faithful top-down grasp, target = a node, seed = that node's own q): still
tied/worse (54–77% vs 57–78%). Root cause: `task_dim=3`, node q matches position but has
an arbitrary wrist orientation, a worse KDL seed than neutral. **Conclusion: the GNG map
is NOT an IK accelerator; its value is arm+base selection (E3) + workspace rep (E0/E1).**
The `voxel` seed baseline (planned §5 code) is therefore also unnecessary — do not build
it. Full numbers + CSVs in experiment_results.md. Everything below is the ORIGINAL plan,
kept struck-through for provenance only.

- ~~**Tujuan:** GNG seed > no-seed, KDL random-restart, dan voxel-map ringan.~~
- ~~**Tool:** `python3 -m reachability_gng.eval ik --model /tmp/arm1_model.npz \
  --dataset /tmp/arm1_dataset.npz --config .../arm1_table1.yaml \
  --methods gng none random voxel --n 500 --csv /tmp/e2_ik.csv`
  (butuh `move_group` hidup: `gng_moveit.launch.py`.)~~
- **Variabel bebas:** metode seed = {`gng`, `none`, `random`(KDL restart), `voxel`(BARU)}.
- **Sub-analisis:** per **z-shell** {1.2–1.5, 1.5–1.8, 1.8–2.1 m}.
- **Kondisi tetap:** N=500 target pose acak dari dataset (default); group `gantry_1_with_arm_1`,
  ee `t1_a1_tool_frame`; `ik_timeout`, `restarts` sama antar run.
- **Metrik:** success rate (%), solve time mean & **median** (ms), manipulability solusi.
  Plus **memori: #node GNG vs #voxel** untuk cakupan volume sama (angka "Why GNG").
- **Uji signifikansi:** Wilcoxon signed-rank GNG vs tiap baseline (success/time per-pose).
- **Ulangan:** 1× (N=500 sudah cukup besar); seed RNG dicatat untuk `random`/`voxel`.
- **Kolom CSV (dari `eval.py`):** `method, success, time_ms, manip` (per pose).
- **Method `voxel` (BARU, di `seed_for()` eval.py):** voxelisasi dataset FK yang sama,
  simpan per-voxel occupancy + **q representatif** (medoid) + manip rata-rata; seed =
  q voxel terdekat ke task. Definisikan `--voxel-res` (mis. 0.05). Laporkan #voxel occupied.
- **→ Paper:** Table utama "IK seeding" (baris: gng/none/random/voxel; kolom: success,
  mean/median ms, manip, #cells). 1 paragraf "Why GNG".

## E3 — Energy-aware arm & base selection (UTAMA)

- **Tujuan:** J-ranking memilih lengan+base hemat travel/energi vs baseline.
- **Tool:** `gantry_reach_executor` (CSV per-pick sudah lengkap) via
  `gantry_pick.launch.py csv:=/tmp/e3.csv` **plan-only** (tak menggerakkan arm → cepat,
  lepas dari lambatnya physics; eksekusi fisik diserahkan ke E6).
- **Sumber posisi objek (efisien, tanpa relaunch Isaac):** publish grid pose target di
  `/target_object` (PoseStamped, frame `world`) via script driver kecil; executor memilih
  langsung dari `/target_object`. Menghindari memindah objek di `polish.py` + relaunch.
- **Grid posisi (REVISED 2026-07-16, scope 4-lengan):** dari **union** reachable hull
  keempat lengan (ceiling-capped), **simetris di y** supaya arm_3/arm_4 (sisi gantry_2,
  y<0) benar-benar ikut bersaing — grid arm_1-only lama (y∈[−0.33,1.08]) hampir tak
  pernah membuat arm_3/4 menang. Union terukur (4 arm, z≤2.05): x∈[−1.10, 3.10],
  y∈[−1.51, 1.51], z∈[0.95, 2.05]; pita meja z∈[1.0,1.3] terjangkau di x∈[−0.97, 2.87],
  y∈[−1.36, 1.36]. Usul grid meja: x∈[0.0, 2.8] (6), y∈[−1.2, 1.2] (5), z≈1.05–1.10 (1–2)
  → **≥60 posisi**, buang titik di luar union hull; sebar kiri/kanan setara.
- **Variabel bebas:** `selection_mode` ∈ {`energy`(usulan), `nearest`(jarak task-space,
  di antara 4 lengan), `fixed`, `random`(uniform atas 4 lengan, seed dicatat)} — **param
  BARU**. **Baseline `fixed` (REVISED): karena kini ada 4 pilihan tetap, jalankan keempat
  varian `fixed_arm ∈ {1,2,3,4}` dan laporkan BEST & WORST fixed arm** — klaim "J
  mengalahkan bahkan lengan-tetap terbaik" jauh lebih kuat dan menghindari tuduhan
  cherry-pick memilih satu lengan tetap. Plan-only → murah menjalankan keempatnya.
- **Weight study (sub-eksperimen):** di ~10 posisi, sweep `w_manip ∈ {0, 50, 300}` dan
  `w_dist ∈ {0, 1}` (mode `energy`), live via `-p` → tunjukkan pergeseran ranking.
- **Kondisi tetap & kontrol:** **reset gantry+arm ke home sebelum tiap trial** (karena
  `d_*` diukur dari state saat ini); orientasi grasp default top-down; `box_clearance` tetap.
- **Metrik (per pick, dari CSV):**
  - Base placement: `gantry_lin` (m), `gantry_rot` (rad).
  - Travel/energi: `d_gantry_lin`, `d_gantry_rot`, `d_arm`, `traj_energy`.
  - Pilihan: `arm`, `J`, `rank_J` vs `rank_dist` (apakah J memilih beda dari nearest).
  - Waktu: `ik_ms`, `plan_time_s`; `success` (plan ditemukan).
- **Ulangan:** selection deterministik → 1× untuk J/arm/travel; **3× per (posisi×mode)**
  untuk variance waktu plan (Isaac non-realtime + jitter).
- **Kolom CSV (dari `_init_csv`, sudah ada):**
  `t, obj, arm, attempt, rank_J, rank_dist, node, dist, ee_dist, d_gantry_lin,
  d_gantry_rot, d_arm, hold, manip, J, gantry_lin, gantry_rot, ik_ms, plan_time_s,
  plan_ms, traj_energy, success`. **Tambah 1 kolom:** `selection_mode` (agar A/B dalam satu CSV).
- **→ Paper:** Table utama (per mode: mean gantry travel, arm travel, plan time, success)
  + boxplot travel/energi per mode + contoh kasus "J memilih lengan lebih jauh tapi lebih murah".

## E6 — Pick end-to-end (Isaac digital twin)

- **Tujuan:** pipeline utuh berjalan; kuantifikasi sukses + kategorisasi gagal.
- **Setup:** T1 `./isaac_sim/launch_workcell.sh`; T2 `pick_stack.launch.py execute:=true
  box_clearance:=0.15 csv:=/tmp/e6.csv`; T3 `pick_cli`. YOLOE aktif (default).
- **Desain (REVISED 2026-07-16):** objek reachable (7 dari 8 di scene; **obj_2
  tomato_soup_can dikecualikan = unreachable-by-design**, gantikan potted_meat_can yang
  sudah dihapus). Objek: cracker_box, scissors, mustard_bottle, teddy_bear (IsaacLab,
  non-YCB), banana, mug, bowl. × beberapa penempatan. Usul **20 posisi**, **3 trial**.
- **Metrik:**
  - Pick success rate (%).
  - Time-to-pick (deteksi→plan→eksekusi selesai).
  - Arm terpilih & gantry placement (silang-cek dengan E3).
  - **Breakdown kegagalan** kategori: `no-detection` / `IK −31` / `plan-fail` /
    `execution-abort −3` (fail loud).
- **Ulangan:** 20×3 = 60 pick.
- **Log:** CSV executor (`success`, `arm`, `ik_ms`, `plan_time_s`) + catatan manual kategori gagal.
- **→ Paper:** Table sukses per objek + stacked bar breakdown kegagalan. HRI NL ("get me a
  box") sebagai 1 tabel kecil kualitatif (opsional).

## E4 — YOLOE deteksi & lokalisasi (pelengkap)

- **Tujuan:** persepsi open-vocab layak sebagai input (bukan input ideal).
- **Ground truth:** pose objek dari `polish.py` (snapshot koordinat 7 objek); Isaac
  ground-truth seg = oracle.
- **Variabel bebas:** `seg_source` {yoloe, isaac}; `conf` {0.1, 0.2, 0.3}; #kamera {1, 2};
  `retina_masks` {on, off}.
- **Metrik:** detection rate per objek; localization error ‖centroid − GT‖ (m, dipisah x/y/z);
  inference rate (Hz) + GPU util/mem; **track stability** (index-jump & object-drop rate
  sebelum/sesudah tracker); false-positive rate.
- **Ulangan:** **≥200 frame** per kondisi (scene statik).
- **Kode BARU (kecil):** script bandingkan `/detected_objects` vs GT `polish.py`
  → CSV `label, gt_x,gt_y,gt_z, det_x,det_y,det_z, err, err_x,err_y,err_z, detected(0/1)`.
- **→ Paper:** Table deteksi+lokalisasi (per objek/kondisi) + 1 baris track-stability.

## E5 — Klasifikasi reachable vs unreachable (pelengkap)

- **Tujuan:** map + persepsi menghasilkan keputusan grasp yang benar.
- **Ground truth biner (REVISED 2026-07-16):** potted_meat_can sudah DIHAPUS dari scene.
  Objek unreachable-by-design sekarang = **obj_2 (tomato_soup_can @ world (3.25, −0.66,
  ~1.08))** — DIVERIFIKASI 2026-07-16: node/sampel-FK terdekat ≥0.45 m untuk keempat
  lengan (di luar jangkauan x + terlalu rendah). Sisanya (cracker_box, scissors,
  mustard_bottle, teddy_bear, banana, mug, bowl) reachable. Verifikasi label "reachable"
  dengan IK/plan sukses (silang dari E3).
- **Metrik:** `% reachable by volume` per objek (`reachability_cloud` voxel mode);
  confusion matrix reachable/unreachable (objek × beberapa posisi); sensitivitas
  `reach_radius` {0.08, 0.12, 0.16}.
- **Ulangan:** tiap objek beberapa posisi dekat/di batas hull.
- **Log:** angka `% reachable` dari log `reachability_cloud` (print saat berubah) + catatan manual.
- **→ Paper:** Confusion matrix + kurva sensitivitas `reach_radius`.

---

## 4. Urutan pengambilan data (usulan 4–5 hari)

1. **Hari 1 (offline, tanpa Isaac):** E0 (figure+tabel map) + E1. **DONE 2026-07-16**
   (rail-2.0 rerun). ~~E2~~ dropped (negative). Bangun map dulu (`build_maps.sh`).
2. **Hari 2:** E3 plan-only (grid posisi × mode {energy,nearest,fixed×4,random} + weight study).
3. **Hari 3:** E4 + E5 (scene statik, batch frame).
4. **Hari 4–5:** E6 eksekusi fisik (paling rapuh/lambat) + ulangan cadangan.

## 5. Kode baru yang dibutuhkan (dikerjakan SETELAH dok ini disepakati)

| Untuk | Perubahan | Ukuran | File |
|-------|-----------|--------|------|
| E3 | Param `selection_mode ∈ {energy,nearest,fixed,random}` (+ `fixed_arm`) + kolom CSV `selection_mode` | ~30–40 baris | `gantry_reach_executor.py` |
| E3 | Script driver publish grid `/target_object` + reset-home antar trial | ~50 baris | `scripts/` atau node kecil |
| ~~E2~~ | ~~Seed method `voxel`~~ **DIBATALKAN** — E2 IK-seeding dropped (negative result), voxel baseline tak diperlukan | — | — |
| E4 | Script localization-error `/detected_objects` vs GT `polish.py` → CSV | ~40–60 baris | `scripts/` |

**Prinsip:** jangan sentuh bobot J default, `grasp_orientation`, atau GNG core. Semua
tambahan bersifat *additive* (mode/method baru), backward-compatible dengan pipeline live.
