# P1 / G19-HW — §8c langkah 4 di perangkat keras NYATA: gerak gantry ANTAR tugas, biaya setup NYATA

Lanjutan [p1_g18_hw.md](p1_g18_hw.md) (langkah 3 **LULUS 9/10 CONCURRENT**, §B2.1).
G18 **tidak** disambung; dokumen ini dimulai baru.

Yang **diwarisi apa adanya dan tidak ditulis ulang**: [g17 §A1](p1_g17_hw.md)
(sukses N-lengan = SATU jendela 2.0 s BERSAMA; vonis CONCURRENT / STAGGERED /
PARTIAL / NEITHER), g17 §A3 (S8–S11); [g16](p1_g16_hw.md) §A1, §A3, §A5, §A6
(S1–S7), §A10 (Rule 6 dikecualikan); [g18 §B2.1](p1_g18_hw.md) (DUA pembacaan:
A1 terkunci + torsi); g17 B3 / g18 B2.2 (HALTED = kegagalan kelayakan, **di dalam**
penyebut).

🔒 A1 tetap **5 mm / 5° / 2.0 s**, dinilai **penilai independen**
(`reach_dwell_monitor`), `pos_err_max_settled_mm`, bukan `pos_err_max`.

---

## A. Protokol — ditulis 2026-09-21, SEBELUM satu pun gerak sesi ini

> 🔒 **DIKUNCI 2026-09-21** setelah persetujuan operator atas §A8 (jawaban di
> sana), **sebelum** gerak pertama sesi ini. §A4 + tabel B0.2 ikut dikunci.

### A1. Model yang diuji — dan satu koreksi atas prompt sesi

[p1_g3_timing.md §C2](p1_g3_timing.md) (TERKUNCI) — **bukan** "lipat + traverse +
rentang-ulang":

```
T_setup = retract (kerja -> MENGGANTUNG)  +  T_traverse  +  extend (MENGGANTUNG -> kerja)
            dibantu gravitasi                                 MELAWAN gravitasi, terbatas torsi
T_traverse = max(T_lin, T_rot);  T_lin(Δ) = 0.29 s + Δ_mm / v_lin,  v_lin = 3000/95.4930 = 31.416 mm/s
```

"Lipat" di prompt = **retract ke pose menggantung** (REST, S1); tuck S2 tetap
dilarang. Dan biaya ini **menghentikan KEDUA lengan** gantry itu (§C1 penutup).

Hanya `T_traverse` yang punya model. `t_retract` / `t_extend` **belum pernah
diukur** (g3 §C3) → sesi ini pengukuran **pertama**, dilaporkan sebagai itu,
tidak "dibandingkan" dengan apa pun.

🔴 **Temuan kode sebelum data — traverse lewat bridge BUKAN `T_lin`.** Bridge
(`dual_table_controller._on_hw_command`) mengejar setpoint JTC dengan **rantai**
`go_to_absolute` stop-start (satu baru setelah yang lama selesai). Jadi durasi
traverse ditentukan **durasi lintasan yang KITA perintahkan** (`T_cmd`), dan
`T_lin` hanya batas bawah fisik. Terukur dari log g18 (rel 0 → 550 mm, `T_cmd`
30 s): **38 dispatch, 29.19 s** dispatch-pertama → selesai-terakhir, sementara
`T_lin(550) = 17.8 s`. Jalur `move_dual_table` (satu `go_to_absolute`, = model)
**tidak boleh** dipakai selama bridge ARMED: JTC masih memegang rel di posisi
lama, `topic_based_ros2_control` menerbitkan perintah ≠ state, dan bridge akan
**menyeret gantry kembali** (S15).

### A2. 🔒 Satu percobaan langkah 4 = satu PINDAH + tugas SESUDAHNYA

```
keadaan awal i : kedua lengan MENAHAN tugas i-1 di rel R(i-1)     (i = 1: tugas 0 di rel A)
1. RETRACT     : return_rest.py --arms arm_1 arm_2 --move          (30 s, sendi-lurus, TIDAK diubah)
2. TRAVERSE    : rel R(i-1) -> R(i) lewat bridge, profil kosinus    (HANYA setelah S12 lulus)
3. EXTEND+TUGAS: reach_dwell_probe --dual pasangan i di rel R(i)    (arm_1 lalu arm_2, S11)
4. penilai independen menilai tugas i -> vonis A1 N-lengan
```

| | Dikunci |
|---|---|
| **Rel** | A = **0.550 m** (g18, terbukti), B = **0.950 m** (usulan, §A8-1). Δ = **400 mm**, `T_lin(400) = 13.02 s` |
| **Arah** | percobaan ganjil A→B, genap B→A — 5 + 5, simetris |
| **`T_cmd`** | kosinus, puncak kecepatan setpoint = **0.9 · v_lin** → `T_cmd = πΔ / (2 · 0.9 · 31.416)` = **22.2 s** untuk 400 mm (gantry tidak pernah tertinggal dari JTC) |
| **Jumlah percobaan** | **10 pindah**, masing-masing dinilai pada tugas SESUDAH pindah. Tugas 0 (sebelum pindah pertama) dilaporkan, **tidak** dihitung |
| **LULUS** | **≥ 8 / 10 CONCURRENT** (analogi g16/g17 A2, sama dengan langkah 3) |
| **Vonis** | empat arah g17 A1 dari **penilai**; label probe dicatat terpisah (probe memasukkan TORQUE-ABORT dan HALTED ke "TIDAK VALID" — g18 pertentangan 6, **bukan** vonis) |
| **HALTED** (`TORQUE-UNSAFE` 3/3 dsb.) | kegagalan kelayakan → **di dalam** penyebut, tidak diganti |
| **TIDAK VALID (mesin, g16 A5)** | diulang; kalau pindahnya selesai, yang diulang tugasnya saja |
| **Dua pembacaan** | 1: A1 terkunci (formal). 2: torsi terukur vs 14 dan vs 12 N·m — keduanya disebut (g18 B2.1) |

### A3. 🔒 Besaran yang DIUKUR — biaya setup per komponen

Instrumen: perekam `/joint_states` (semua sendi `t1_*`, efort), log
`move_group` (mulai/selesai eksekusi), log `dual_table_controller` (dispatch
bridge), log penilai, cap waktu runner. Tiap angka dari **jam perekam/log**,
bukan stopwatch.

| Besaran | Definisi | Pembanding |
|---|---|---|
| `t_retract` | kirim `return_rest` → **kedua** lengan maks \|q − REST\| < 0.5° | durasi **perintah** 30 s (setelan kita) |
| `t_traverse` | enkoder rel berubah > 0.5 mm → \|rel − R(i)\| ≤ 0.52 mm dan tetap | **`T_lin(400)` = 13.02 s** (model) dan **`T_cmd`** = 22.2 s |
| `t_plan`, `t_exec` per lengan | log `move_group`: permintaan rencana → eksekusi mulai → selesai | — (pertama kali) |
| `t_extend` | Σ `t_plan` + `t_exec` kedua lengan | — (pertama kali) |
| `t_gap` | sela skrip (retract selesai → rel mulai; rel sampai → rencana arm_1) | — dilaporkan terpisah, tidak dilebur |
| **`T_setup`** | retract mulai → eksekusi arm_2 selesai (jam dinding) **dan** Σ komponen | g3 C2 |
| `t_to_concurrent` | eksekusi arm_2 selesai → penilai menyatakan CONCURRENT | — |
| kopling rel→lengan | maks \|Δq\| kedua lengan selama `t_traverse`; torsi puncak lengan selama traverse | g18: 0.018° (rel 550 mm, lengan REST) |
| galat rel | rel akhir − R(i); jumlah dispatch bridge | deadband 50 pulsa = 0.52 mm |

Dilaporkan per percobaan, per arah, dan median / p95 atas 10.

### A4. 🔒 Target — himpunan TERBUKTI di rel yang dipakai, saringan KONDISI BENAR

**Fakta geometri (FK, `/tmp/reach_dwell_live.urdf`):** `t1_linear_joint` =
translasi **+x murni** (rel 0 → 0.95: kedua base dan tool bergeser tepat
+0.950 m di x; y, z tetap). Jadi pasangan B2 digeser **+Δ di x** identik
secara geometri relatif lengan — torsi RNEA, jarak antar-lengan (terukur
**295.6 mm di kedua rel** pada konfigurasi uji yang sama), dan himpunan IK sama.
Yang berubah hanya lengan terhadap **dunia** (adegan MoveIt, gantry 2).
Himpunan aman-torsi `/tmp/torque_safe_arm_*.npy` **hilang** (reboot 2026-08-26),
jadi kolam farthest-point g17 tidak dapat dibangkitkan ulang tanpa menghitung
ulang; menggeser B2 lebih sederhana **dan** membuat rel satu-satunya variabel.

**Saringan baru** (`dual_arm_targets.py --pairs-file`, §B0): per sampel, arm_1
direncanakan dari REST di rel L; arm_2 direncanakan dari REST dengan arm_1
**DITEMPATKAN** di titik terakhir rencana arm_1 **sampel yang sama**; rel
ditempatkan di L. Lewat `start_state` MoveIt dan penyaring antar-lengan —
**nol gerak**. 3/3 (k-of-k), `--tau-max 12.0`.

| | Dikunci |
|---|---|
| `LA` | pasangan B2 (urutan asli) yang lolos 3/3 di rel **0.550** |
| `LB` | pasangan B2 **digeser +0.400 m di x** yang lolos 3/3 di rel **0.950** |
| Penugasan | tugas 0 = `LA[0]`; percobaan ganjil (→B) = `LB[0..4]`; genap (→A) = `LA[1..5]` |
| Kurang | kalau \|LA\| < 6 atau \|LB\| < 5: daftar **diputar** dari awal, urutan tetap. Tidak ada kandidat baru |
| Dikunci | tabel penugasan ditulis di §B **sebelum** pindah pertama. Yang gagal di hari-H **tidak** diganti |

### A5. 🔒 Palang keselamatan BARU untuk langkah 4

S1–S11 tetap. Tambahan:

| # | Aturan |
|---|---|
| **S12** | **LIPAT SEBELUM GANTRY.** Perintah rel hanya setelah `return_rest` rc = 0 **dan** runner membaca `/joint_states` (gabung per-nama ≥ 0.5 s): kedua lengan maks \|q − REST\| < 0.5°. Kalau tidak → STOP |
| **S13** | Rel hanya ke **0.550** atau **0.950**; `rail_to` menolak target lain. Puncak kecepatan setpoint ≤ 0.9 · v_lin. Batas operasional 1600 mm (S5) tidak disentuh |
| **S14** | **Origin:** sebelum gerak rel pertama, operator **konfirmasi fisik** carriage di posisi yang dibaca enkoder (sekarang **0.549737 m** = 549.7 mm dari home fisik yang di-preset g18 B0.7). End stop ~1656 mm dari home → 0.950 m menyisakan ~700 mm |
| **S15** | 🔴 Selama bridge ARMED: **tidak ada** `move_dual_table`, `table_keyboard.py`, atau `dual_table_controller` kedua. Rel hanya lewat JTC |
| **S16** | MoveIt **buta** gantry 1 ↔ gantry 2: **283** pasangan `t1_*`↔`t2_*` dimatikan di SRDF, dan lengan 3/4 palsu (di model semua sendi 0.0). Operator konfirmasi keadaan fisik gantry 2 + lengan 3/4 dan ruang bebas lengan gantry 1 sampai x ≈ **1.75 m** (base arm_1 1.35 m + jangkauan) |
| **S17** | Auto-stop sebelum langkah berikut: fault/Kortex exception di log launch; node penilai/perekam/`ros2_control_node` mati; `return_rest` rc ≠ 0 atau torsi > 13.5; torsi terukur > 14; **lengan bergeser > 0.5° selama traverse**; **galat rel akhir > 2 mm**; bridge `REJECTED`/`NOT armed`; INVALID mesin |

Gantry **tidak pernah** bergerak dengan lengan terentang di sesi ini (S12) —
kopling rel→lengan-terentang tetap **belum diukur**, dan tetap begitu.

### A6. Instrumen — PID ditemukan, bukan di-hardcode

Runner g18 (`batch.sh`) meng-hardcode PID dan `/tmp/g18_*`. Runner g19
(`docs/results/p1_g19/`) menemukan PID penilai / perekam / `ros2_control_node`
dari `/proc` saat mulai, menulis ke `/tmp/g19_*`, dan **mengarsip ke
`docs/results/p1_g19/` setiap percobaan** (`/tmp` hilang saat reboot; probe
menambah ke `/tmp/g17_step3.json` — di jalur `--dual` ia **menambah**, bukan
menimpa; tetap diarsip per percobaan).

Validator penilai sebelum data: **0 penilai sebelum, 5/5, 0 sesudah**. Penilai
dry-run / plan-check **terpisah** dari penilai hari-H.

### A7. 🔒 Papan skor §7.2 — dugaan D57–D63, DITULIS SEBELUM DATA

Skor masuk: **34 meleset, 22 tepat.** Prior kerja (p1_state §9): kendala
kelayakan yang belum diukur → longgar; kode sendiri → lebih lambat/rumit/salah;
g18: pemblokir mahal = **keadaan dunia**, bukan integrasi.

| # | Dugaan | Dasar |
|---|---|---|
| **D57** | `t_traverse` = `T_cmd` ± 1.0 s pada ≥ 9/10 pindah — bridge terikat setpoint; jadi `t_traverse / T_lin(400)` ≈ **1.7**, bukan ≈ 1 | g18: 29.19 s untuk `T_cmd` 30 s; puncak setpoint 0.9 · v_lin |
| **D58** | kopling rel→lengan (lengan di REST): maks \|Δq\| ≤ **0.1°** pada **10/10** pindah | g18: 0.018° |
| **D59** | langkah 4 **LULUS**: ≥ 8/10 CONCURRENT | geometri relatif lengan identik dengan langkah 3 (9/10); pindah tidak mengubah tugas |
| **D60** | traverse adalah komponen **TERKECIL** dari tiga (retract / traverse / extend) pada ≥ 9/10 percobaan; `t_extend` median ≥ 25 s | exec 8.5–13 s per lengan (g18 B2.3) + penyaring antar-lengan ~20 s per rencana (terukur §B0) |
| **D61** | galat rel akhir \|rel − R(i)\| ≤ **0.52 mm** pada 10/10; lima kedatangan di 0.550 berentang ≤ 1.0 mm | g18 −0.26 mm; deadband 50 pulsa |
| **D62** | saringan baru (arm_1 ditempatkan) **menolak pasangan 6** di rel 0.550 | g18 B2.2: 3/3 ditolak di perangkat keras dengan arm_1 di target |
| **D63** | ≥ 1 pemblokir **kode/integrasi sendiri** memakan > 15 menit sebelum pindah pertama | prior "kode sendiri lebih salah" (g8/g9) |

D59 dan D60 adalah yang menanggung klaim: D60 benar berarti prioritas scheduler
"hindari perubahan linier" (p1_state §5.6) **salah tempat** pada Δ ini — biaya
tetap lengan mendominasi.

### A8. 🔒 Keputusan OPERATOR — dicatat SEBELUM data (2026-09-21)

| # | Pertanyaan | Jawaban operator |
|---|---|---|
| **1** | Rel B | **0.950 m** |
| **2a** | S14: carriage fisik di ~550 mm dari home = bacaan enkoder 0.549737 | **ya** |
| **2b** | S16: keadaan fisik lengan 3/4 | **terpasang, menggantung, MENYALA** — model MoveIt tetap palsu (sendi 0.0) dan 283 pasangan t1↔t2 dimatikan; perlindungan = rel B menjauhkan gantry 1 dari x ±0.4 dan konfirmasi ruang operator |
| **2c** | S16: ruang bebas lengan gantry 1 sampai x ≈ 1.75 m | **bebas** |
| **3** | Penyaring torsi `joint_2` | **TETAP** (prediksi + 6.6 ≤ 14; `--tau-max 12.0` pengamat) |
| **4** | Stack g18 yang hidup | **dipakai terus** (launch log `/tmp/g18_t3.log`) |
| **5** | Daftar diputar (percobaan 10 = pasangan tugas 0) | **boleh** |

---|---|---|
| **1** | Rel B = 0.950 m? (alternatif 0.150 — lengan gantry 1 lalu tumpang-tindih x dengan lengan gantry 2 di ±0.4) + konfirmasi fisik S14 dan S16 | **0.950** |
| **2** | Penyaring torsi `joint_2` diperketat? Sekarang prediksi + 6.6 ≤ **14** (rating); g18 B2.4: satu rencana terukur 0.46 N·m **di atas** prediksi terkoreksi, margin 0.17 di rencana lain | *keputusan operator — tanpa usulan* |
| **3** | Stack g18 yang **masih hidup** (launch PID 871044, 7/7 active, lengan REST, rel 0.549737, nol fault baru) dipakai terus, atau bring-up baru? Bring-up baru = crash dump saat shutdown (disk 4.1 GB), risiko mid-boot A5 | **pakai terus**; saringan §A4 sudah dijalankan padanya plan-only |

---

## B. Hasil terukur

### B0. SEBELUM gerak apa pun — 2026-09-21

#### B0.1 Opsi "arm_1 DITEMPATKAN" — ditulis dan divalidasi, nol gerak

`reach_dwell_probe._plan_and_screen(..., start_joints=)` meneruskan penempatan ke
`start_state` MoveIt (diff) **dan** ke `screen_interarm(other_joints=)`;
`dual_arm_targets.py --pairs-file` memakainya (§A4). Bukan untuk jalur eksekusi.

Validasi dijalankan **plan-only** pada `move_group` stack g18 yang hidup, **bukan**
di stack palsu: meluncurkan stack palsu berarti mematikan stack nyata (crash
dump, bring-up ulang). `move_group` di stack itu memang memakai URDF mode palsu
(g18 B0.6), jadi adegannya sama. Nol perintah ke controller.

| # | Uji | Hasil |
|---|---|---|
| V0 | offline: arm_2 ke (L, 0.36, 1.40), arm_1 di REST vs **ditempatkan** di titik sama; L = 0.55 dan 0.95 | REST → **CLEAR 295.6 mm**; ditempatkan → **COLLIDE 0.0 mm**; identik di kedua rel. Penempatan tanpa sendi rel → **MENOLAK** (6/7). `/joint_states` tidak dibaca sama sekali (dijaga assert) ✅ |
| V1 | live: arm_1 ke B2#1 digeser (1.400, 0.318, 1.240), rel **ditempatkan** 0.95 | PLANNED; FK ujung rencana di rel 0.95 − target = **(−0.9, 0.5, −1.7) mm**. Kalau `start_state` diabaikan, selisihnya 400 mm ✅ |
| V2 | live: arm_2 ke B2#1, arm_1 di REST vs ditempatkan (IK **tidak konvergen**) | PLANNED keduanya, tetapi jarak min penyaring **589.3 → 489.7 mm**, pasangan terdekat berganti ke `t1_a1_right_finger_dist` — penempatan sampai ke penyaring hidup. Kontrol tidak bersih (IK gagal), dicatat apa adanya |
| V2' | live: arm_2 ke (0.55, 0.36, 1.40), arm_1 di REST vs ditempatkan (IK ok) | REST → PLANNED (98 %); ditempatkan → **TORQUE-UNSAFE** 15.21 N·m. Penyaring torsi berjalan **sebelum** penyaring antar-lengan, jadi ini membuktikan MoveIt memilih lintasan lain karena penempatan, **bukan** bahwa penyaring antar-lengan menangkapnya — itu dibuktikan V0 |

Terukur juga: satu rencana + saringan = **~20–25 s**, didominasi penyaring
antar-lengan (jarak pinocchio per titik). Masuk ke dasar D60.

#### B0.2 Saringan §A4 — dijalankan plan-only, nol gerak (13:24–14:03)

`dual_arm_targets.py --pairs-file results/p1_g18/b2_pairs.txt --lin {0.550,0.950}
--repeats 3 --tau-max 12.0` pada stack g18 yang hidup. Data:
[results/p1_g19/g19_screen_*.json/.log](results/p1_g19/).

| B2 # | rel 0.550 | rel 0.950 (+0.400 x) | g18 perangkat keras |
|---|---|---|---|
| 1 | ✅ 3/3 | ✅ 3/3 | CONCURRENT |
| 2 | ❌ a1 (sampel 1) | ✅ 3/3 | CONCURRENT |
| 3 | ✅ | ✅ | CONCURRENT |
| 4 | ❌ a2 (sampel 1) | ❌ a2 (sampel 1) | CONCURRENT |
| 5 | ✅ | ❌ a1 (sampel 3) | CONCURRENT |
| 6 | ❌ a1 (sampel 2; sampel 1 arm_2 **lolos** dengan arm_1 ditempatkan) | ❌ a1 (sampel 1) | PARTIAL (a2 HALTED) |
| 7 | ❌ a1 (sampel 1) | ❌ a1 (sampel 1) | CONCURRENT |
| 8 | ❌ a2 (sampel 3) | ❌ a2 (sampel 3) | CONCURRENT |
| 9 | ✅ | ✅ | CONCURRENT |
| 10 | ✅ | ✅ | CONCURRENT |
| **lolos** | **5/10** | **5/10** | 9/10 |

**Kesepuluh penolakan = `TORQUE-UNSAFE` `joint_2`.** Nol NO-PLAN, nol
INTERARM-COLLIDE. Prediksi terkoreksi (+6.6): enam **tipis** 14.03–14.64,
empat **tegas** 14.81–15.85.

🔴 **Saringan 3/3 meloloskan 5/10 pasangan yang 9/10-nya CONCURRENT di perangkat
keras.** Eksekusi = ≥ 1 dari 3 rencana lolos; saringan = 3 dari 3 (g17 B2.1
aritmetikanya). Pasangan 2 dan 5 membuktikan **variansi perencana murni**:
geometri relatif lengan identik di kedua rel, vonis berbeda. Juga tercatat:
pasangan 6 — alasan D62 dibuat — ditolak oleh **arm_1**, bukan oleh arm_2 dengan
arm_1 ditempatkan (arm_2 lolos di sampel 1). Dinilai di §B nanti apa adanya.

**Penugasan menurut §A4 (mekanis, `make_plan.py`)** — \|LA\| = 5 < 6 → diputar:

| i | rel | B2 # | arm_1 | arm_2 |
|---|---|---|---|---|
| 0 | 0.550 | 1 | 1.000, 0.318, 1.240 | 0.214, 0.247, 1.240 |
| 1 | 0.950 | 1 | 1.400, 0.318, 1.240 | 0.614, 0.247, 1.240 |
| 2 | 0.550 | 3 | 0.786, 0.388, 1.000 | 0.071, 0.176, 1.000 |
| 3 | 0.950 | 2 | 1.257, 0.035, 1.080 | 0.686, 0.529, 1.160 |
| 4 | 0.550 | 5 | 0.786, 0.529, 1.320 | 0.000, 0.388, 1.240 |
| 5 | 0.950 | 3 | 1.186, 0.388, 1.000 | 0.471, 0.176, 1.000 |
| 6 | 0.550 | 9 | 1.000, 0.247, 1.000 | 0.357, 0.318, 1.320 |
| 7 | 0.950 | 9 | 1.400, 0.247, 1.000 | 0.757, 0.318, 1.320 |
| 8 | 0.550 | 10 | 0.714, 0.318, 1.240 | 0.071, 0.247, 1.160 |
| 9 | 0.950 | 10 | 1.114, 0.318, 1.240 | 0.471, 0.247, 1.160 |
| 10 | 0.550 | 1 ↻ | 1.000, 0.318, 1.240 | 0.214, 0.247, 1.240 |

🔒 **DIKUNCI** bersama §A (§A8-5). Percobaan
10 mengulang pasangan tugas 0 (putaran). Kalau bentuk ini tidak dapat diterima,
§A4 diubah **sekarang**, sebelum data.

#### B0.3 Instrumen sebelum gerak

- Validator penilai: **0 penilai sebelum, 5/5 PASS, 0 sesudah** ([g19_validator.log](results/p1_g19/g19_validator.log)).
  Run pertama **dibuang**: penghitung "sebelum" saya cocok dengan `grep`-nya
  sendiri (terbaca 4). Validator juga mencetak *"monitor exited 1 mid-run"* —
  **menyesatkan**: kode keluar diperiksa **sesudah** validator sendiri mengirim
  SIGINT; kasus E (terakhir) dijawab penilai hidup. Dicatat, tidak diperbaiki.
- Perekam `/joint_states` pertama kehilangan CSV-nya: `rm -f … && setsid … &`
  melatarbelakangkan **seluruh rantai**, `rm` berpacu dengan perekam. Kesalahan
  saya; dimulai ulang sebelum gerak apa pun, diverifikasi baris masuk.

---

### B1. 🔒 LANGKAH 4 — sepuluh pindah di perangkat keras NYATA (14:17–14:47)

Izin operator: tugas 0 sendiri, percobaan 1 sendiri (+ konfirmasi fisik carriage
di ~95 cm sesudahnya), lalu 2–10 dengan auto-stop S17. **Auto-stop tidak pernah
terpicu.** Nol fault, nol Kortex exception, nol penolakan / `NOT armed` bridge,
nol red LED.

**Vonis dari PENILAI INDEPENDEN** (`reach_dwell_monitor`), tabel penugasan B0.2:

| i | rel | B2 # | vonis A1 N-lengan | arm_1 pos maks / **settled** | arm_2 pos maks / **settled** | ori maks | torsi puncak tugas | probe |
|---|---|---|---|---|---|---|---|---|
| 0 | 0.550 | 1 | ✅ CONCURRENT *(tidak dihitung)* | 3.86 / **1.80** | 2.81 / **1.17** | 1.38° | 10.97 `a2_j2` | CONCURRENT |
| 1 | →0.950 | 1 | ✅ CONCURRENT | 4.15 / **1.34** | 3.00 / **1.11** | 2.72° | 8.31 | CONCURRENT |
| 2 | →0.550 | 3 | ✅ CONCURRENT | 2.87 / **1.66** | 2.99 / **1.05** | 1.92° | 9.07 | CONCURRENT |
| 3 | →0.950 | 2 | ✅ CONCURRENT | 2.63 / **1.89** | 3.80 / **1.04** | 1.97° | **12.41** `a1_j2` | TORQUE-ABORT |
| 4 | →0.550 | 5 | ✅ CONCURRENT | 3.01 / **2.55** | 2.43 / **0.75** | 2.33° | **13.26** `a1_j2` | TORQUE-ABORT |
| 5 | →0.950 | 3 | ✅ CONCURRENT | 3.92 / **1.83** | 3.65 / **1.02** | 1.91° | 8.61 | CONCURRENT |
| 6 | →0.550 | 9 | ✅ CONCURRENT | 4.43 / **1.84** | 2.79 / **1.36** | 2.16° | **12.74** `a2_j2` | TORQUE-ABORT |
| 7 | →0.950 | 9 | ✅ CONCURRENT | 3.96 / **1.71** | 2.02 / **0.72** | 2.09° | **13.12** `a2_j2` | TORQUE-ABORT |
| 8 | →0.550 | 10 | ✅ CONCURRENT | 3.12 / **1.88** | 4.78 / **0.76** | 2.00° | **13.06** `a1_j2` | TORQUE-ABORT |
| 9 | →0.950 | 10 | ✅ CONCURRENT | 3.06 / **1.28** | 4.03 / **1.33** | 1.99° | 9.87 | CONCURRENT |
| 10 | →0.550 | 1 ↻ | ✅ CONCURRENT | 2.74 / **1.05** | 4.68 / **1.26** | 1.94° | 8.36 | CONCURRENT |

Semua jendela 41–42 sampel @ 20 Hz. **STAGGERED 0, PARTIAL 0, NEITHER 0,
HALTED 0**; nol penolakan penyaring antar-lengan.

#### B1.1 🔒 Dua pembacaan (g18 B2.1)

**Pembacaan 1 — A1 terkunci:** **10 / 10 CONCURRENT ≥ 8 → LANGKAH 4 LULUS.**

**Pembacaan 2 — torsi terukur:**

| ambang | percobaan melewatinya | kalau dikeluarkan |
|---|---|---|
| rating `joint_2` **14 N·m** | **0** | 10 / 10 |
| nominal KA-75+ **12 N·m** | **5** (3, 4, 6, 7, 8: 12.41 – **13.26**) | **5 / 10 → tidak lulus** |

Lebih buruk dari g18 (3 / 10 di atas 12). Margin terkecil ke rating: **0.74 N·m**
(percobaan 4 — pasangan yang di g18 terukur **5.53**: torsi = sifat lintasan).

#### B1.2 🔒 Biaya setup TERUKUR — pertama kali untuk retract dan extend

Dari perekam + log `move_group`; [g19_components.json](results/p1_g19/g19_components.json),
[g19_summary.txt](results/p1_g19/g19_summary.txt). Percobaan 1–10, Δ = 400 mm:

| komponen | median | min – maks | pembanding |
|---|---|---|---|
| **retract** (panggil → kedua lengan < 0.5° dari REST) | **48.8 s** | 46.6 – 51.2 | perintah 30 s |
| ↳ overhead `return_rest` sebelum lengan bergerak | 19.7 s | — ¹ | penyaring antar-lengan per titik + start |
| ↳ gerak | **29.0 s** | 28.6 – 29.2 | = perintah 30 s |
| **traverse** (rel mulai → berhenti) | **21.1 s** | 20.8 – 21.5 | **`T_lin` 13.0 s** (×1.62) · `T_cmd` 22.2 s |
| sela retract → rel | 4.6 s | 4.4 – 5.5 | skrip |
| **extend** (rencana arm_1 → eksekusi arm_2 selesai) | **71.1 s** | 53.8 – 91.4 | — |
| ↳ rencana + saringan arm_1 / arm_2 | 28.2 / 23.0 s | 17.6 – 38.6 | |
| ↳ eksekusi arm_1 / arm_2 | 12.5 / 9.3 s | 5.5 – 14.0 | |
| selesai → CONCURRENT | 1.8 s | 1.7 – 1.9 | dwell 2.0 s − masuk lebih dulu |
| **`T_setup` jam dinding** (percobaan 2–9) | **148 s** | 132 – 157 | ² |

¹ percobaan 7 terbaca 0.14 s: lengan masih bergeser dari tugas 6 saat panggilan —
detektor gerak saya tertipu; gerak 46.8 s di baris itu ikut tercemar.
² percobaan 1 dan 10 = 215 / 195 s: termasuk **tunggu 45 s** `rail_to` (B2.1).

**Yang terbaca, dan ini inti langkah 4:** dari ~148 s biaya setup satu pindah
400 mm, **traverse hanya 21 s (14 %)**. Gerak lengan murni (29 + 12.5 + 9.3 ≈ 51 s)
lebih besar, dan **perangkat lunak** (penyaringan rencana ~51 s + overhead retract
~20 s + sela ~7 s ≈ **78 s, > separuh**) terbesar. Kedua lengan gantry itu diam
sepanjang itu. Prioritas scheduler §5.6 *"hindari perubahan linier"* **salah
tempat** pada Δ ini: yang mahal adalah **pindah itu sendiri** (retract + extend),
hampir tanpa bergantung Δ.

#### B1.3 Traverse, kopling, galat rel

| | median | rentang |
|---|---|---|
| galat rel akhir | — | **−0.81 … +0.91 mm**; 8/10 dalam ±0.52 |
| lima kedatangan di 0.550 | — | 550.00 – 550.91 mm (rentang **0.91**) |
| torsi lengan selama rel bergerak | 1.73 N·m | 1.56 – 2.75 |
| drift lengan (maks − min) **selama** rel bergerak | **0.189°** | 0.184 – 0.195 |
| drift yang sama **20 s sesudah** rel berhenti | 0.107° | 0.036 – 0.164 |

Drift selama gerak **konsisten ~0.19°** (hampir selalu `t1_a2_joint_5`), di atas
riak diam median 0.11° — ada kopling kecil, tetapi besarnya **tidak dapat
dipisahkan** dari riak pergelangan dengan metrik ini (B2.1 (4)). Semua < 0.5°
(S17). Metrik `rail_to` (|q − q₀| sepanjang log) memberi 0.11 – **0.375°**
(percobaan 9); yang dihitung adalah metrik A3 (jendela gerak).

---

### B2. 🔒 D57–D63 DINILAI

| # | Dugaan | Terukur | Vonis |
|---|---|---|---|
| **D57** | `t_traverse` = `T_cmd` ± 1.0 s pada ≥ 9/10; rasio ke `T_lin` ≈ 1.7 | selisih −0.71 … −1.37 s → **4 / 10** dalam ±1.0; rasio **1.62** | ❌ **MELESET** ³ |
| **D58** | drift ≤ 0.1° pada 10/10 | **0 / 10** (0.184 – 0.195°) | ❌ **MELESET** |
| **D59** | ≥ 8/10 CONCURRENT | **10 / 10** | ✅ **TEPAT** |
| **D60** | traverse komponen terkecil pada ≥ 9/10; `t_extend` median ≥ 25 s | **10 / 10**; extend median **71.1 s** | ✅ **TEPAT** |
| **D61** | galat rel ≤ 0.52 mm pada 10/10; kedatangan 0.550 berentang ≤ 1.0 mm | **8 / 10** (−0.81, +0.91); rentang 0.91 ✓ | ❌ **MELESET** (konjungsi) |
| **D62** | saringan baru menolak pasangan 6 di 0.550 | ditolak — tetapi oleh **arm_1** (sampel 2); arm_2 dengan arm_1 ditempatkan **lolos** di sampel 1 | ✅ **TEPAT**, mekanisme **salah** ⁴ |
| **D63** | ≥ 1 pemblokir kode/integrasi sendiri > 15 menit sebelum pindah pertama | tiga kesalahan sendiri (penghitung `grep`, balapan `rm`, kontrol V2 IK) — masing-masing **< 5 menit** | ❌ **MELESET** |

³ Arah benar (bridge terikat setpoint, bukan `T_lin`), besaran salah: ekor
kosinus < 1 mm tidak pernah di-dispatch (debounce 1.0 mm), jadi rel berhenti
~1 s sebelum `T_cmd`.
⁴ Preseden G7: bagian yang tepat tidak boleh meminjam kredit mekanisme.

➜ Papan skor §7.2: **34 meleset / 22 tepat → 38 meleset / 25 tepat.**
Polanya: ketiga yang tepat tentang **hasil kasar** (lulus, urutan komponen,
satu pasangan ditolak); keempat yang meleset tentang **presisi instrumen dan
perangkat keras** (± 1 s, 0.1°, 0.52 mm) dan tentang **kode sendiri** — dan
D63 meleset ke arah yang **berlawanan** dengan prior "kode sendiri lebih
salah": integrasi sesi ini murah karena g18 sudah membayarnya.

---

### B3. Pertentangan §B lawan §A — G19

G16 enam, G17 enam, G18 tujuh.

| # | Pertentangan |
|---|---|
| **(1)** | prompt sesi: "pindah = LIPAT + TRAVERSE + RENTANG-ULANG"; g3 §C2 yang terkunci: retract ke **menggantung** + traverse + extend (A1, sebelum data) |
| **(2)** | A3 mendefinisikan `t_traverse` sampai \|rel − R\| ≤ 0.52 mm; bridge men-debounce **1.0 mm** (`bridge.pos_tol_mm`), jadi setpoint akhir < 1 mm tidak pernah dikirim → definisi **tak terpenuhi** di 2/10. Diganti "rel berhenti" (B1.2), **bukan** diam-diam |
| **(3)** | A3 menyebut `T_setup` jam dinding; `rail_to` menunggu ±0.52 mm hingga **45 s** → percobaan 1 dan 10 tercemar ~40 s. Alat **tidak** diubah di tengah seri; median dari 2–9 |
| **(4)** | A3 "kopling rel→lengan = maks \|Δq\| selama traverse" tidak memisahkan kopling dari riak pergelangan yang sudah ada; butuh garis dasar, ditambahkan post-hoc (B1.3) |
| **(5)** | A4 mengandaikan saringan 3/3 memilih himpunan "terbukti"; ia meloloskan **5/10** pasangan yang **9/10**-nya CONCURRENT di perangkat keras, dan pasangan 2/5 berganti vonis antar-rel dengan geometri **identik** → daftar harus diputar |
| **(6)** | prompt: probe **menimpa** `/tmp/g17_step3.json`; kode jalur `--dual`: **menambah**. Runner menghapusnya sebelum tiap percobaan |
| **(7)** | validator mencetak "monitor exited mid-run" untuk kode keluar **sesudah SIGINT miliknya sendiri** (B0.3) |

**G19: TUJUH.**

---

## C. Keadaan akhir, dan yang BELUM dikerjakan (Rule 12)

- Lengan dikembalikan ke **REST** setelah percobaan 10 (izin operator): galat
  akhir 0.097°, torsi puncak 4.97 N·m, antar-lengan min 530.9 mm, nol fault.
  Rel **0.550910 m**, rotasi 0. Stack g18 **masih hidup** (launch PID 871044);
  penilai dan perekam sesi ini dihentikan, nol proses sisa.
- **Tidak diukur, disengaja:** gantry bergerak dengan lengan **terentang**
  (S12 mencegahnya); rotasi gantry (`t1_rotation_joint` tetap 0); Δ selain
  400 mm — jadi **ketergantungan `T_setup` pada Δ belum diukur**, hanya satu titik.
- **Belum diperbaiki, disengaja:** debounce bridge 1.0 mm (galat rel sampai
  ~1 mm, B3 (2)); tunggu 45 s `rail_to`; overhead penyaringan ~20–30 s per
  rencana (biaya perangkat lunak terbesar di B1.2); label probe (g18
  pertentangan 6); pesan validator (B3 (7)); URDF palsu di RSP/`move_group`
  (g18 B0.6); SRDF 112/121 dan 283 t1↔t2 pasangan dimatikan.
- **Keputusan operator terbuka:** 5/10 percobaan **di atas 12 N·m nominal**
  (margin terkecil 0.74 ke rating 14) — penyaring `joint_2` tetap (A8-3),
  jadi pertanyaan g18 B2.4 kini lebih mendesak.

## D. Prompt sesi berikutnya — G20-HW (salin ke chat BARU)

**Rekomendasi: Opus 5, effort TINGGI.** Langkah 5 menghidupkan gantry 2 dan
lengan 3/4 yang **belum pernah** bergerak nyata di proyek ini, dan MoveIt buta
antar-gantry (283 pasangan t1↔t2 dimatikan). Kesalahannya diam sampai menabrak.

```
Sesi G20-HW -- p1_state.md 8c LANGKAH 5 di PERANGKAT KERAS NYATA:
DUA gantry, EMPAT lengan. Repo ceiling_arm, branch feat/rgbd-topo-deploy.

BACA PENUH sebelum menulis kode atau menyentuh hardware:
1. CLAUDE.md (Working Rules; Rule 6 dikecualikan untuk sesi protokol P1,
   g16 A10).
2. docs/p1_g19_hw.md -- SELURUHNYA. Langkah 4 LULUS 10/10 CONCURRENT.
   A2/A4 (struktur pindah + saringan --pairs-file), A5 (S12-S17), B1.1
   (5/10 di atas 12 N.m nominal, maks 13.26), B1.2 (pindah ~148 s,
   traverse 14 %), B2 (D57-D63), B3 (TUJUH pertentangan).
3. docs/p1_g18_hw.md B0.4-B0.8 (ros2_kortex, URDF palsu, origin gantry,
   kunci port) dan B2.4 (+6.6 BUKAN batas atas).
4. docs/p1_g17_hw.md A1-A3; docs/p1_g16_hw.md A1, A3, A5, A6 (S1-S7).
5. scripts/interarm_collision.py -- HANYA memeriksa lengan SE-gantry.

=== TUGAS ===
1. Protokol langkah 5 BELUM ADA. Tulis docs/p1_g20_hw.md A DULU dan KUNCI
   sebelum satu pun gerak: definisi sukses (A1 N-lengan g17 dengan N = 4:
   SATU jendela 2.0 s BERSAMA untuk keempat lengan), jumlah percobaan dan
   palang LULUS, pembagian vonis, papan skor D64+ (skor masuk: 38 meleset,
   25 tepat), keputusan operator (penyaring torsi joint_2; lihat g19 C).
2. PRASYARAT gantry 2 / arm_3 / arm_4 -- SEMUA belum pernah nyata:
   a. bring-up dengan arm3_fake:=false arm4_fake:=false (sesi terakhir
      memakai true). "Actuator count reported by robot is '6'" EMPAT kali.
   b. ORIGIN gantry 2: operator KONFIRMASI FISIK carriage = bacaan enkoder
      SEBELUM gerak gantry 2 apa pun (g18 B0.7 terjadi di gantry 1).
   c. return_rest.py hanya mengenal arm_1/arm_2 (PREFIX, CONTROLLER
      gantry_1). Perluas ke arm_3/arm_4 + gantry_2_with_arm_controller;
      validasi DRY RUN dulu.
   d. Penyaring ANTAR-GANTRY belum ada: interarm_collision.GANTRY_PAIRS
      hanya pasangan se-gantry; MoveIt mematikan 283 pasangan t1<->t2.
      Tulis + validasi (kontrol negatif yang HARUS menyala, spt V0 g19).
   e. Ruang kerja AMAN-TORSI arm_3/arm_4 belum dipetakan; target harus
      lolos saringan tingkat lintasan 3/3 dengan lengan lain DITEMPATKAN
      (dual_arm_targets.py --pairs-file sudah menempatkan satu lengan;
      empat lengan butuh perluasan). Kunci daftar SEBELUM data.
   f. Satu lengan baru bergerak sendiri dulu (arm_3), baru pasangan
      gantry 2, baru keempatnya -- tiap tahap izin operator.
3. Jalankan, nilai apa adanya, tulis di docs/p1_g20_hw.md, tautkan balik
   ke g19. JANGAN sambung/edit p1_g19_hw.md.
4. Perbarui p1_state.md 8c.

=== SEBELUM BRING-UP ===
- ros2_ws/src/ros2_kortex HARUS di branch ceiling-arm-fixes (e712295).
- Stack g18/g19 MUNGKIN masih hidup (launch PID 871044, arm3/4 PALSU).
  Langkah 5 butuh bring-up BARU (lengan 3/4 nyata): matikan dengan
  kill -INT ke PID ros2 launch ASLI, tunggu > 20 s, verifikasi nol sisa
  (ros2 node list --no-daemon) SEBELUM meluncurkan. Hapus crash dump
  /var/crash MILIKMU sendiri (~400 MB); disk ~3.9 GB.
- python3 scripts/remount_check.py -> GERBANG LULUS, keempat lengan ICMP.
- Validator penilai: 0 penilai sebelum, 5/5, 0 sesudah. Penghitung proses
  JANGAN cocok dengan grep/bash-mu sendiri (g19 B0.3).
- TIDAK ADA table_keyboard.py / dual_table_controller milik operator.

=== PERINTAH ===
cd ros2_ws && source install/setup.bash
ros2 launch workcell_moveit_config my_workcell.launch.py \
    use_fake_hardware:=false arm3_fake:=false arm4_fake:=false \
    enable_gantry_bridge:=true > /tmp/g20_t1.log 2>&1
# runner/alat g19: docs/results/p1_g19/ (batch.sh, rail_to.py, analyze.py,
#   make_plan.py) -- gantry_1 SAJA; rail_to hanya 0.550/0.950 (S13).
# penilai 4 lengan: -p arms:="['arm_1','arm_2','arm_3','arm_4']"
#   -p tool_frames:="['t1_a1_tool_frame','t1_a2_tool_frame',
#                     't2_a1_tool_frame','t2_a2_tool_frame']"

=== KESELAMATAN ===
- A6/S7: TIDAK ADA gerak tanpa persetujuan eksplisit operator di SETIAP
  langkah. Otorisasi g19 TIDAK berlanjut.
- LIPAT (REST menggantung) SEBELUM gantry mana pun bergerak (S12 g19).
- enable_gantry_bridge:=true WAJIB. Selama bridge ARMED: TIDAK ADA
  move_dual_table / table_keyboard (JTC menyeret gantry kembali, g19 S15).
- Bridge berhenti sampai ~1 mm dari target (debounce 1.0 mm) -- jangan
  menunggu 0.52 mm (tunggu 45 s rail_to, g19 B3 (2)-(3)).
- --tau-max 12.0 JANGAN dinaikkan; joint_2 disaring ke RATING 14.
  g19: 5/10 tugas terukur 12.41-13.26 N.m; margin terkecil 0.74.
- ros2_control TIDAK menegakkan effort; red LED = reset FISIK.
- Gerak PEMULIHAN lambat dan HARUS selesai (g16 B4.4).

=== JEBAKAN YANG SUDAH DIUKUR ===
- /joint_states DUA penerbit: gabung per-nama >= 0.5 s.
- Saringan 3/3 meloloskan 5/10 pasangan yang 9/10-nya CONCURRENT di
  perangkat keras (g19 B0.2); k-of-k membuang yang layak. Satu rencana +
  saringan ~20-30 s (penyaring antar-lengan pinocchio per titik).
- `cmd && setsid ... &` melatarbelakangkan SELURUH rantai (g19 B0.3).
- Probe dual MENAMBAH ke /tmp/g17_step3.json; hapus sebelum tiap run,
  arsipkan ke docs/results/p1_g20/ tiap run.
- Vonis A1 = PENILAI INDEPENDEN, bukan label probe.
- --approach 0 WAJIB. Tab browser ke 192.168.2.1x = SIGPIPE.

=== ATURAN ===
- 7.2: UKUR, JANGAN MENDUGA. Dugaan dikunci SEBELUM data.
- DILARANG menggeser A1 (5 mm / 5 deg / 2.0 s) atau mengganti target
  yang gagal. Pakai pos_err_max_settled_mm.
- Kalau B bertentangan dengan A, B menang dan pertentangannya DITULIS.
  G17 enam, G18 tujuh, G19 tujuh.
- Checkpoint (Rule 10) setelah tiap tahap; Rule 12.
```
