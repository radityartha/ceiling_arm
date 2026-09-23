## A. Protokol — ditulis 2026-09-23 SEBELUM solve apa pun, SEBELUM bring-up, SEBELUM gerak

### A0. Keputusan operator (dijawab SEBELUM §A ini ditulis)

| # | Pertanyaan | Jawaban |
|---|---|---|
| 1 | Tahap 0 (ditanya ULANG): LED 4/4 tidak merah; origin g1 **dan** g2 = home fisik; rel + ruang antar-gantry bebas s.d. ~1.6 m; nol alat di sel | **semua benar** |
| 2 | Syarat (ii) g22 A2 ("≥ 1 pindah di SETIAP gantry") dinilai pada jadwal mana | **jadwal P1′** (yang dieksekusi); FISIK dihitung dan dilaporkan saja |
| 3 | K = jumlah seed yang dijalankan | **K = 3** (operator boleh berhenti lebih awal; dilaporkan) |

Pra-cek (nol gerak): nol proses ROS; enp112s0 192.168.2.100; `ros2_kortex` e712295;
disk 4.1 GB (S27 ≥ 1.5 ✅); `JOINT_TORQUE_OFFSET_NM[2]` = 7.7 (A8-1 g22).

### A1. 🔒 Instance — g24 A5 dengan `p0` = rel TERBACA

```
untuk seed = 0 … 49 (urut):
  nodes, rejects = g24_candidates.json[seed]      -- tarikan oracle‴ TIDAK bergantung p0
  inst = make_instance_g24.instance(seed, nodes, rejects, cache) APA ADANYA, kecuali
         R1 p0 = rel TERBACA di bring-up, dibulatkan ke grid 0.05 m (g24 E8 → harapan 0.00 / 0.00)
         (disuntik lewat restrict_rot0; make_instance_g24.py tidak diubah)
  x    = run_sched45.p1prime(inst, serial=True), konstanta g25_constants.json APA ADANYA
  (i)   solve_exact(x) layak
  (ii)  jadwal P1′ punya ≥ 1 pindah di SETIAP gantry            (A0-2)
  (iii) sched_coll.schedule_conflict(inst, P1′ stops) = None
  (iv)  sched_screen --plan g26_candidates.json --repeats 3 APA ADANYA
```

- **Kontrol KG1 (menggerbang):** dengan `p0` disuntik = 0.55 / 0.00 dan biaya FISIK, jalur
  G26 harus mereproduksi `g24_candidates.json` — makespan FISIK **dan** jadwal
  `(rail, tugas)` — **bit-identik** pada 45/45 seed lolos. Dengan biaya P1′ ia harus
  mereproduksi `g25_sched45.json` `P1'.opt` 45/45.
- **Kontrol KG2:** tiap optimum P1′ diputar ulang oleh `g0_gate.eval_schedule` (== `finish`).
- **Kunci json untuk alat:** alat G22 membaca jadwal dari `schedule_FISIK` dan `p0` dari
  `decomp.FISIK.<g>.legs`. Di `g26_candidates.json` kunci itu **berisi jadwal P1′** (dan
  `decomp` jadwal P1′) — nama lama dipertahankan agar alat tidak diubah. Jadwal FISIK sejati
  (p0 baru) disimpan di `schedule_FISIK_true`. Ini ditulis sebagai pertentangan di §B.
- Bila rel terbaca ≠ 0.00 / 0.00 sesudah pembulatan: candidates **dibuat ulang** dengan nilai
  terbaca sebelum (iv); hasil hitung awal dibuang dan dicatat.

### A2. 🔒 Pemilihan seed + urutan

(iv) dijalankan seed demi seed urut atas seed lolos (i)–(iii), sampai **K = 3** lolos atau
seed habis. Seluruh (iv) selesai **sebelum** gerak pertama (keadaan awal saringan =
keadaan awal tiap run: rel `p0`, keempat lengan REST — sama untuk semua seed). Seed dijalankan
dalam urutan lolos. Kurang dari 3 lolos → yang lolos dijalankan, dilaporkan. Nol lolos →
nol gerak.

### A3. 🔒 Prediksi — per seed, DIKUNCI sebelum gerak pertama sesi

Per event dari `g22_plan.events(row)` (urutan A3 g22 — sama dengan runner), konstanta
g25 **tidak dikalibrasi ulang**:

```
tugas    → c_task (43.646); tugas TERAKHIR run → c_task − t_end (43.475)
retract  → c_ret (50.874), ATAU 0 bila kedua lengan gantry itu masih REST
           (belum ada tugas di gantry itu sejak awal run: P1′ t_fold_first)
traverse → c_rovh (4.7395) + r (0.95017) · T_cmd(Δ)
makespan P1′-serial = Σ event (sela runner = 0)
```

Dilaporkan juga: P1′ paralel (`max_g`), dan P1 lama (FISIK, `T_lin`) untuk jadwal yang sama.

### A4. 🔒 Eksekusi — g24 E4–E5 apa adanya, per seed

1. (sekali, sebelum seed pertama) `return_rest` DRY lalu **`--move`** kedua gantry (izin operator).
2. `rail_to_g` DRY tiap traverse seed itu; `run_g22 --dry --plan g26_candidates.json
   --out-dir /tmp/g26_s<S> --archive docs/results/p1_g26 --prefix g26_s<S>_`.
3. **GERAK** `run_g22` sama tanpa `--dry` (izin operator). Penilai + perekam hidup (SigIgn dicek).
4. Retract penutup kedua gantry (`return_rest --move`, tidak dihitung) lalu **rel kembali ke
   `p0`** tiap gantry yang pindah: `rail_to_g --seed S 0.000` DRY → `--move` (izin operator;
   `p0` ada di `allowed_rails`). Keadaan awal seed berikut = rel `p0`, lengan REST.
5. Dekomposisi: `decomp_g24b.py` diadaptasi (prefix + daftar event) → per komponen.

Tidak ada tahap 2 terpisah (rel g2 sudah bergerak G24b; S25 sudah dipenuhi) — operator tetap
melihat carriage pada tiap traverse.

### A5. 🔒 Besaran yang DILAPORKAN — terlepas dari hasilnya

Per seed: jadwal (rel, tugas, lengan), pindah per gantry, jumlah retract yang dilewati;
per event terukur / prediksi / galat (s, %); makespan terukur vs P1′-serial, P1′-par, P1;
per tugas `start, P→M, R→X, exec, →succ` + torsi puncak `joint_2` + RNEA rencana hidup;
per traverse `t_traverse / T_cmd` + overhead alat; per retract dilewati: wall + rc.
Gabungan: galat makespan per seed, |galat| per tugas rerata atas seluruh tugas G26,
galat per jenis event. (iv): per seed hasil + kelas penolakan; per rencana PLANNED.

### A6. 🔒 Dugaan D114–D122 — DITULIS SEBELUM solve, (iv), dan gerak

Prior (tally): dugaan dari **mekanisme terukur** tepat; dari **kode sendiri** / "kendala
lebih mengikat" meleset.

| # | Dugaan | Dasar |
|---|---|---|
| **D114** | (i)–(iii) di bawah `p0` 0/0 + P1′: **≥ 45 / 50** | tarikan tugas sama; g1 dari 0.00 → tugas g1 (rel ~0.5–1.3 di G24) memaksa pindah lebih sering, bukan lebih jarang |
| **D115** | K = 3 seed lolos (iv) dalam **≤ 8** seed (i)–(iii) pertama | G24b: lolos ke-2; D99 95 % per rencana; terima-palsu oracle‴ (seed 0) ada |
| **D116** | tiap seed yang dijalankan: **6/6 SUCCESS**, nol HALTED, nol auto-stop S26 | G24b 6/6; saringan (iv) 3/3 sudah meloloskan rencananya |
| **D117** | galat makespan P1′-serial dalam **±10 %** pada **setiap** seed yang dijalankan | G24b −4.8 %; tetapi sebagian pembatalan (g25 B2) — taruhan |
| **D118** | terukur **>** prediksi pada **≥ 2 / 3** seed | bias semua komponen non-tugas di bawah (alat G22+ lebih lambat dari G19: overhead rel 6.7–8.7 vs 4.74, `start` 2.5 s, saringan +2.6) |
| **D119** | \|galat\| per tugas rerata atas semua tugas G26 **≥ 5 s** | satu konstanta tidak mengikuti panjang lintasan (saringan ∝ exec, g25 A1) |
| **D120** | tiap retract "keberangkatan pertama dengan lengan REST": `return_rest` mencetak "sudah di rest", rc 0, wall **≤ 4 s** (model 0) | jalur kode: `wait_joints` → cek < 0.5° → keluar; `start` retract G24b 2.37–2.68 s |
| **D121** | torsi puncak `joint_2` terukur **< 12 N·m** pada setiap tugas | G24b maks 9.62; saringan RNEA + 7.7 ≤ 12 konservatif ≥ 3.3 (g24 E5) |
| **D122** | `t_traverse / T_cmd` ∈ **[0.95, 0.98]** pada setiap traverse Δ ≥ 0.10 m | debounce g19; G24b 0.966 / 0.972 |

🔒 **DILARANG** mengubah konstanta P1′, aturan instance, K, atau dugaan sesudah hasil apa pun
terlihat. Bila model meleset, itu hasil, ditulis di §B.

---

