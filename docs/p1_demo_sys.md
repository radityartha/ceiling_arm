# P1 / DEMO-SYS — demo = uji sistem P1 (snapshot → solver P1″ → run_g54)

> Prompt: [p1_demo_sys_now_prompt.md](p1_demo_sys_now_prompt.md) (2026-10-17 sore; berlaku, tas = kontrol POSITIF). Alat:
> [results/p1_demo_sys/](results/p1_demo_sys/). Sesi 2026-10-17 sore: LURING saja (0 gerak HW, 0 mock). §A belum dibuat (berikut D668).

## 0. Luring — solver P1″ ↔ run_g54 (tugas 0)

| | temuan |
|---|---|
| masukan solver | **indeks node** peta `cap_g{1,2}_rail160.npz` (3132 node, x 0–2, **y −0.6…0.6**, z 1.0–1.4, jarak ~7 cm). L1 = `masks` (IRM tol 0.05) → Feas49 (oracle‴ alat-bawah, roll bebas, amplop tilt; torsi per cabang +7.7 +0.66 ≤ 14; E1 env ≥ 50 mm + kotak ≥ 10 mm; garis pulang lurus G41b) + POSE_A + TRAV (EdgeMatrix) + stop (s42), malas |
| keluaran → runner | baris candidates `{p0, <varian>: {schedule: {g: [{rail_m, rot_deg, tasks, xyz, goal}]}}}`; run_g54 hanya baca `p0` + `schedule` (xyz + goal SENDI) — node tidak dipakai |
| titik sembarang | tas (0.978, −0.831) **di luar peta** (node terdekat 0.23 m → "node terdekat" tidak jujur). `capability.build_payload` = mask + canon untuk titik sembarang; K: 200 node acak/gantry dibangun ulang → masks + canon **bit-identik** (g1, g2); diulang 50 node di tiap load |
| tugas tak layak | solver: `ValueError('task i is infeasible on every gantry')` → SELURUH instance gagal. Alasan per penyaring = cache tuple (`n_sol`, `n_torque_ok`, `whys`) |
| orientasi | oracle‴ + probe `--approach 0` = alat-bawah; p_cmd = puncak objek + 0.25 z |

**Keputusan operator (tombol, 2026-10-17):** (1) titik = **node baru** (build_payload, penyaring sama, cache salinan per dir run,
dir terikat `nodes.json`); (2) **saring per tugas** dulu (solver, tugas tunggal; tolak = alasan solver + hitungan per penyaring), lalu
tugas diterima di-solve bersama; (3) **R90**.

Alat: [demo_solve.py](results/p1_demo_sys/demo_solve.py) (tasks.json → screen.json + demo_candidates.json),
[snapshot_tasks.py](results/p1_demo_sys/snapshot_tasks.py) (rekaman track_median → tasks.json; hadir ≥ 90 %, median, +0.25 z;
uji pada rekaman HW Z2 M1 10-17: 1 tugas, p_cmd (1.7176, 0.5387, 1.055)), [mock_sys.sh](results/p1_demo_sys/mock_sys.sh).

## 1. Luring — tugas sintetis (`tasks_synth.json`, p0 g1 1.40 / g2 0)

Hasil [run_synth/screen.json](results/p1_demo_sys/run_synth/screen.json) (hitungan = fakta F solver; "POSE_A tolak" = stop dengan
REST di stop gagal, tidak diverifikasi tuple-nya):

| tugas | putusan sistem | arm | L1 | POSE_A tolak | diverifikasi | tanpa IK | cabang torsi-ok | ok | penolak cabang |
|---|---|---|---|---|---|---|---|---|---|
| M1 (1.7258, 0.5383, 1.055) | **DITERIMA** 31.6 s | arm_1 | 326 | 98 | 14 | 0 | 14 | 14 | torsi 87 |
| | | arm_2 | 39 | 37 | 0 | — | — | 0 | — |
| TAS z 1.34 (kontrol POSITIF) | **DITOLAK** 799.6 s `task 0 is infeasible on every gantry` | arm_3 | 556 | 237 | 300 | 182 | 972 | **3** | torsi 48 525, garis-env 752, garis-kotak 217 |
| | | arm_4 | 460 | 253 | 207 | 133 | 683 | 0 | torsi 34 196, garis-env 624, garis-kotak 59 |
| z 1.40 (kontrol neg. lama) | **DITOLAK** 508.7 s, sama | arm_3 | 579 | 239 | 326 | 192 | 120 | 0 | torsi 57 395, garis-env 120 |
| | | arm_4 | 473 | 254 | 214 | 130 | 60 | 0 | torsi 39 698, garis-env 60 |
| z 0.60 (1.0, 0, 0.60) kontrol neg. BARU | **DITOLAK** 0.0 s, sama | semua | **0** | — | — | — | — | 0 | L1 kosong |

Jadwal bersama (tugas diterima = M1): **arm_1 @ g1 (1.40, 0°), makespan 29.04 s, LB = UB, konflik None, gantry tidak bergerak**
([demo_candidates.json](results/p1_demo_sys/run_synth/demo_candidates.json)). Mock 77 TIDAK dijalankan (keputusan operator di bawah).

**Mengapa 3 tuple TAS yang lolos Feas49 tetap tak terjadwal** — semuanya arm_3 g2 dengan rotasi di luar amplop HW g2 (−35…+95) dan
traverse dari p0 g2 (0, 0) (EdgeMatrix G46 E1, persegi rel × rot, env ≥ 50 mm) kena peta env:

| stop g2 | j2 cabang (statis) | traverse p0 → stop: d min |
|---|---|---|
| (0.45, −55°) | 5.25 | 25.1 mm < 50 |
| (0.50, −65°) | 5.29 | **−13.1 mm** (tembus) |
| (0.55, −75°) | 5.50 | **−17.4 mm** (tembus) |

Celah sistem dicatat (bukan dipakai): solver R90 tidak membaca amplop HW `rot_envelope.json` (runner yang menolak sebelum t0).

### 1.2 Kontrol negatif z 1.40 bukan "fisika"

Prompt sesi menyebut z 1.40 = 0 IK di seluruh grid (g64_diag `TASRAK_z140`). Di grid solver (R90, rel 0.05 / rot 5°, tanpa prasaring
0.88 m) **214/532 tuple punya IK**, 180 cabang torsi-ok; ditolak oleh torsi-cabang + garis-env = kelas penyaring yang SAMA dengan tas →
tidak membedakan "mustahil" dari "terlalu ketat". **Operator (tombol): ganti** → (1.0, 0, 0.60): L1 0 di keempat lengan, ditolak
seketika. z 1.40 tetap dicatat sebagai hasil.

## 2. Keputusan operator 2026-10-17 ~13:50 (tombol)

**DEMO-SYS DITUNDA → G64 dulu** (lipat T1 / traverse). Alasan data: tas (kontrol positif) ditolak sistem oleh penyaring yang tidak
dilonggarkan — 0 tuple layak dengan REST/garis lurus/traverse sekarang (kelas "perencanaan terpisah"). Bahan G64 dari sesi ini:
(a) 3 stop arm_3 g2 lolos tuple tetapi traverse dari p0 kena env (tabel 1.1) → obat = lengan terlipat (T1) saat traverse / traverse
bertahap; (b) amplop rotasi HW g2 −35 belum ada di solver; (c) `g64_diag` harus memakai BOX_MIN 10 mm (G41b) agar sama dengan
sistem (§1.1) — hitungan §0.1 G64 (STATIC/CURRENT) dengan BOX_MIN 0 terlalu optimis untuk penyaring kotak.
Adapter (demo_solve / snapshot_tasks) siap dipakai ulang sesudah G64. Tidak ada gerak HW, tidak ada mock di sesi ini.

### 1.1 🔴 Temuan: tuple "CURRENT" g64 tidak lolos predikat sistem

`g64_diag` (§0.1 G64: arm_3 @ g2 (0.50, −20°), j2 5.625) mengimpor `g41_make` LANGSUNG → `BOX_MIN = 0.0` (G41). Solver P1″ (rantai
g42 → `G41.set_b()`) memakai **BOX_MIN 10 mm** (G41b). Cabang yang sama (`q6` dari g64_diag.jsonl): garis pulang lurus jarak kotak
scene **0.03 mm** (BOX_MIN 0 → lolos) / **6.56 mm < 10** (BOX_MIN 10 mm → `garis-kotak`). Feas49 di solver: 479 IK, 10 cabang torsi-ok,
**10/10 `garis-kotak`**. → "1 tuple layak" G64 = artefak margin kotak; dengan predikat sistem tuple itu DITOLAK. Penyaring tidak
dilonggarkan.
