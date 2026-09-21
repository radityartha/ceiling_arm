# P1 / G20-HW — §8c langkah 5 di perangkat keras NYATA: DUA gantry, EMPAT lengan

Lanjutan [p1_g19_hw.md](p1_g19_hw.md) (langkah 4 **LULUS 10/10 CONCURRENT**, §B1).
G19 **tidak** disambung; dokumen ini dimulai baru.

Yang **diwarisi apa adanya dan tidak ditulis ulang**: [g17 §A1](p1_g17_hw.md)
(sukses N-lengan = SATU jendela 2.0 s BERSAMA; vonis CONCURRENT / STAGGERED /
PARTIAL / NEITHER), g17 §A3 (S8–S11); [g16](p1_g16_hw.md) §A1, §A3, §A5, §A6
(S1–S7), §A10 (Rule 6 dikecualikan); [g18 §B2.1](p1_g18_hw.md) (DUA pembacaan);
g17 B3 / g18 B2.2 (HALTED = kegagalan kelayakan, **di dalam** penyebut);
[g19 §A5](p1_g19_hw.md) S12–S17 sejauh berlaku (rel lihat §A5 di bawah).

🔒 A1 tetap **5 mm / 5° / 2.0 s**, dinilai **penilai independen**
(`reach_dwell_monitor`), `pos_err_max_settled_mm`, bukan `pos_err_max`.

---

## A. Protokol — ditulis 2026-09-21, SEBELUM satu pun gerak sesi ini

> 🔒 **DIKUNCI 2026-09-21** setelah jawaban operator atas §A8 dan tabel
> saringan §B0.3, **sebelum** tahap 0 (bring-up baru) dan sebelum gerak apa pun.

### A1. 🔒 Definisi sukses — g17 A1 dengan N = 4

> Sukses 4-lengan = ada **SATU** jendela 2.0 s di mana **KEEMPAT** lengan
> (`arm_1`–`arm_4`, dua gantry) memenuhi pos < 5 mm **dan** ori < 5° **BERSAMAAN**.

| Vonis | Artinya |
|---|---|
| **CONCURRENT** | jendela BERSAMA 2.0 s untuk keempatnya ada — **ini** hasilnya |
| **STAGGERED** | keempat lengan sukses sendiri-sendiri, tetapi **tidak pernah bersamaan** — BUKAN sukses |
| **PARTIAL** | 1–3 lengan sukses |
| **NEITHER** | tidak ada yang sukses |
| **HALTED** | satu lengan tidak dieksekusi karena tidak ada rencana lolos saringan dalam 3 percobaan rencana — kegagalan **kelayakan**, di dalam penyebut |
| **TIDAK VALID (mesin, g16 A5)** | diulang, di luar penyebut, sebab disebut |

Vonis = **penilai** (4 lengan, `n_arms ≥ 4`), bukan label probe (g18 pertentangan 6).

### A2. 🔒 Satu percobaan langkah 5

```
awal       : keempat lengan di REST (maks |q − REST| < 0.5°, /joint_states gabung per-nama ≥ 0.5 s)
1. TUGAS   : reach_dwell_probe --quad : arm_1 -> arm_2 -> arm_3 -> arm_4, BERURUTAN (S11)
             tiap rencana disaring torsi + terhadap KETIGA lengan lain (S18)
2. NILAI   : penilai independen 4 lengan -> vonis A1
3. RETRACT : return_rest --arms arm_3 arm_4 --move, LALU return_rest --arms arm_1 arm_2 --move
             (yang terakhir bergerak pulang lebih dulu; tiap panggilan disaring S18)
```

| | Dikunci |
|---|---|
| **Rel** | **TETAP** sepanjang sesi: gantry 1 = **0.550910** (terbaca), gantry 2 = **0.000000** (terbaca, dikonfirmasi fisik §A8). **Nol** perintah rel (§A8-1) |
| **Jumlah percobaan** | **10**, target dari §A4 |
| **LULUS** | **≥ 8 / 10 CONCURRENT** (analogi g16/g17/g18/g19 A2) |
| **Dua pembacaan** | 1: A1 terkunci. 2: torsi terukur vs **14** (rating) dan vs **12** (nominal) — keduanya disebut |
| **Tidak dihitung** | tahap 1–2 (§A6): arm_3 sendiri, pasangan gantry 2 — dilaporkan, bukan data A2 |

### A3. Besaran yang DILAPORKAN per percobaan

Vonis; `pos_err_max_settled_mm` + ori maks per lengan; torsi puncak per lengan
(perekam `/joint_states`); jarak minimum **se-gantry** dan **antar-gantry**
sepanjang tiap rencana yang dieksekusi (log probe); `t_plan` / `t_exec` per
lengan; seberapa lama `arm_1` menahan sebelum jendela bersama terbuka
(= waktu arm_2 + arm_3 + arm_4); selisih torsi terukur − RNEA `joint_2`
untuk **arm_3 / arm_4** (pertama kali, lawan g18 B2.4).

### A4. 🔒 Target — kuartet B2, gantry 2 = salinan translasi, saringan 3/3 EMPAT lengan

**Fakta geometri (FK + RNEA, `/tmp/reach_dwell_live.urdf`, §B0.2):** gantry 2 =
gantry 1 digeser **(Δrel, −0.72, 0)**, rotasi base identik. 200 konfigurasi
acak: galat pose 6.7e-16, selisih torsi gravitasi **0.00** N·m. Jadi pasangan B2
yang digeser adalah geometri relatif-lengan **identik** untuk arm_3/arm_4, dan
peta aman-torsi arm_1/arm_2 berlaku **di model** untuk arm_3/arm_4. Yang **tidak**
diwarisi: offset torsi perangkat keras arm_3/arm_4 (belum pernah diukur, D67).

| | Dikunci |
|---|---|
| Kandidat | kuartet *n* = B2 #*n* ([g18 b2_pairs](results/p1_g18/b2_pairs.txt)) di gantry 1 apa adanya (rel 0.550) **dan** B2 #*n* digeser (0.000 − 0.550, −0.72, 0) di gantry 2, *n* = 1…10, urutan asli |
| Saringan | `dual_arm_targets.py --pairs-file … --quad --lin 0.550 --lin2 0.000 --repeats 3 --tau-max 12.0`: per sampel arm_1 dari REST; arm_2 dengan arm_1 **DITEMPATKAN**; arm_3 dengan arm_1+arm_2 ditempatkan; arm_4 dengan ketiganya. Tiap rencana: torsi + S18. **3/3** (k-of-k), plan-only, nol gerak |
| `LQ` | kuartet yang lolos, urutan asli |
| Penugasan | percobaan *i* = `LQ[(i−1) mod |LQ|]`. \|LQ\| < 10 → **diputar**, tanpa kandidat baru |
| \|LQ\| = 0 | langkah 5 **tidak dijalankan**; dilaporkan. Tidak ada saringan yang dilonggarkan |
| Dikunci | tabel penugasan di §B0.3 **sebelum** tahap 1. Yang gagal di hari-H **tidak** diganti |

⚠️ Diketahui sebelum data (g19 B3 (5)): k-of-k membuang yang layak — 3/3 dua
lengan meloloskan 5/10 pasangan yang 9/10-nya CONCURRENT. Empat lengan
mengalikan itu (D65). Putaran dengan \|LQ\| kecil berarti **sedikit pose
berbeda**; itu dicatat, bukan disembunyikan.

### A5. 🔒 Palang keselamatan BARU untuk langkah 5

S1–S11 tetap; S12–S17 g19 tetap sejauh berlaku (tidak ada traverse → S12/S13/S17-rel
tidak terpicu). Tambahan:

| # | Aturan |
|---|---|
| **S18** | **Tiap** rencana lengan mana pun disaring terhadap **ketiga** lengan lain, pada konfigurasi **TERUKUR** (`/joint_states` gabung per-nama): se-gantry = pemeriksa mesh g17–g19; antar-gantry = `CrossGantryChecker` (hull cembung, 660 pasangan t1_*×t2_*, §B0.1). Margin **50 mm**. Tidak bisa menyaring = **MENOLAK** (S9). SRDF **bukan** cadangan: 283 pasangan t1↔t2 dimatikan (S10) |
| **S19** | **Origin gantry 2:** operator **konfirmasi fisik** carriage gantry 2 = bacaan enkoder **0.000000** SEBELUM gerak **lengan** gantry 2 apa pun — bukan hanya sebelum gerak rel: S18 memakai bacaan ini untuk menempatkan arm_3/arm_4 terhadap gantry 1 |
| **S20** | **Rel TIDAK diperintah sama sekali** sesi ini. Bridge tetap wajib ARMED-capable (`enable_gantry_bridge:=true`, g16), tetapi tidak ada goal yang menyebut `t*_linear` / `t*_rotation`. Tetap: nol `move_dual_table` / `table_keyboard.py` / `dual_table_controller` kedua (S15) |
| **S21** | **Bertahap, izin operator per tahap** (§A6). Lengan yang belum pernah bergerak nyata tidak ikut percobaan 4-lengan sebelum tahapnya sendiri lulus tanpa fault |
| **S22** | Auto-stop sebelum langkah berikut: fault / Kortex exception di log launch (keempat IP); node penilai / perekam / `ros2_control_node` mati; `return_rest` rc ≠ 0 atau torsi > 13.5; torsi terukur **lengan mana pun** > 14; `INTERARM-UNSCREENED`; TIDAK VALID mesin; rel bergeser > 2 mm dari bacaan awal (tidak diperintah → tidak boleh berubah) |
| **S23** | Rotasi kedua gantry tetap 0 (terbaca 0.000000). Penyaring S18 **tidak** memasukkan sendi rotasi — benar hanya selama ia 0; auto-stop bila \|rotasi\| > 0.5° |

### A6. 🔒 Urutan tahap — tiap tahap izin operator (S7, S21)

| Tahap | Isi | Lulus bila |
|---|---|---|
| **0** | matikan stack g18 (kill -INT PID `ros2 launch` asli, tunggu > 20 s, nol sisa `ros2 node list --no-daemon`); `remount_check.py` GERBANG LULUS + 4× ICMP; bring-up `arm3_fake:=false arm4_fake:=false enable_gantry_bridge:=true` | **4×** "Actuator count reported by robot is '6'"; 7/7 controller active; `/joint_states` memuat `t2_a*` **bukan** 0.0 palsu; nol fault |
| **1a** | konfirmasi fisik S19; `return_rest --arms arm_3` DRY RUN → izin → `--move` | rc 0, galat akhir < 0.5°, torsi < 13.5, nol fault — **gerak fisik pertama arm_3 di proyek ini** |
| **1b** | `reach_dwell_probe --arms arm_3 --target <LQ[0] arm_3>` → izin → `--move`; lalu `return_rest --arms arm_3` | penilai sukses (N = 1); nol fault |
| **2** | `return_rest --arms arm_4` (DRY → izin → move); `--arms arm_3 arm_4 --target <LQ[0] a3> --target2 <LQ[0] a4>` → izin → move; `return_rest --arms arm_3 arm_4` | CONCURRENT (N = 2) atau hasil apa pun **tanpa fault**; dilaporkan |
| **3** | percobaan 1 (4 lengan) sendiri, izin operator | selesai tanpa auto-stop |
| **4** | percobaan 2–10, auto-stop S22, izin operator sekali untuk seri | — |

Validator penilai (4 lengan) sebelum tahap 1a: **0 penilai sebelum, 5/5, 0
sesudah**; penghitung proses tidak cocok dengan `grep`/`bash` sendiri (g19 B0.3).

### A7. 🔒 Papan skor §7.2 — dugaan D64–D70, DITULIS SEBELUM DATA

Skor masuk: **38 meleset, 25 tepat.** Prior (p1_state §7.2): kelayakan belum
diukur → longgar; **eksklusi ruang bersama → belum ada dasar, ukur**; kode
sendiri → lebih lambat/rumit/salah (g19 D63 meleset ke arah berlawanan).

| # | Dugaan | Dasar |
|---|---|---|
| **D64** | langkah 5 **LULUS**: ≥ 8/10 CONCURRENT | g18 9/10, g19 10/10; yang langka adalah SAMPAI, bukan MENAHAN (g17 D51); saringan S18 hanya menambah penolakan, bukan gerak |
| **D65** | saringan 4-lengan 3/3 meloloskan **≤ 3 / 10** kuartet | g19: 5/10 pasangan per gantry; dua gantry ≈ 0.5 × 0.5 = 0.25 kalau independen |
| **D66** | penyaring antar-gantry menolak **0** rencana (saringan dan hari-H), dan jarak antar-gantry minimum sepanjang rencana yang dieksekusi **≥ 150 mm** | rel 0.550 / 0.000 memisahkan lengan di x; B2 menjaga tool gantry 1 di y ≥ −0.035 dan gantry 2 di y ≤ −0.19. **Dugaan di kelas yang prior-nya "belum ada dasar"** — dicatat begitu |
| **D67** | selisih torsi terukur − RNEA `joint_2` arm_3/arm_4 jatuh di rentang arm_1/arm_2 g18 (**+1.2…+7.1 N·m**) pada ≥ 80 % rencana yang dieksekusi | aktuator KA-75+ yang sama; g17 D55 (offset pindah ke arm_2) |
| **D68** | **STAGGERED = 0** meski arm_1 menahan ~3 kali lebih lama sebelum jendela terbuka | g18 B2.3: menahan ≤ 0.17 mm; dikunci kontrol posisi |
| **D69** | ≥ 1 pemblokir **integrasi** > 15 menit sebelum percobaan 4-lengan pertama | arm_3/arm_4 **belum pernah** nyata di stack ini; g16–g18 tiap bring-up baru memakan satu pemblokir (g19 D63 meleset karena stack dipakai ulang) |
| **D70** | origin gantry 2: operator mengonfirmasi carriage **di home fisik** (enkoder 0.000000 benar) | langkah 0 Sesi B: "g2 ≈ 0" dan gantry 2 tidak dikendarai sejak itu (dasar lemah — g18 B0.7 terjadi di gantry 1) |

D64 dan D66 yang menanggung klaim: D64 = langkah 5 berjalan; D66 = apakah
**eksklusi antar-gantry** mengikat pada konfigurasi ini (p1_state §7.2 D9).

### A8. Keputusan OPERATOR — dicatat SEBELUM data

| # | Pertanyaan | Jawaban operator |
|---|---|---|
| **1** | Rel TETAP (g1 0.550, g2 0.000), nol gerak gantry? | **tetap** |
| **2** | Penyaring torsi `joint_2` (prediksi + 6.6 ≤ 14; `--tau-max 12.0` pengamat) — tetap atau diperketat? g19: 5/10 terukur 12.41–13.26 | **tetap ≤ 14** |
| **3** | Keadaan fisik: carriage gantry 2 di home fisik = 0.000000 (S19); arm_3/arm_4 terpasang, menggantung, menyala; ruang antar-gantry bebas; tidak ada alat operator / tab browser | **keempatnya benar** (2026-09-21, sebelum bring-up) |
| **4** | Izin tahap 0: matikan stack g18 (setelah saringan), bring-up baru dengan lengan 3/4 NYATA — nol gerak lengan | **izin** |
| **5** | Daftar diputar bila \|LQ\| < 10 | **boleh** |

Konsekuensi jawaban 3: **D70 dinilai TEPAT** oleh konfirmasi operator —
sebelum data lain ada; dicatat di §B2.

---

## B. Hasil terukur

### B0. SEBELUM gerak apa pun — 2026-09-21

#### B0.1 Penyaring antar-gantry — ditulis dan divalidasi, nol gerak

`interarm_collision.CrossGantryChecker`: semua geometri `t1_*` × `t2_*` kecuali
struktur × struktur (rel paralel, konstan 429 mm) = **660 pasangan**, hull
cembung. Terukur: mesh 469 ms / titik untuk 121 pasangan (→ ~100 s per rencana
kalau 676 pasangan); hull **1.07 ms** untuk 676.

| # | Uji | Hasil |
|---|---|---|
| X0 | swa-uji `--self-test --cross`, rel 0.550/0.000 dan 0.550/0.550 | A: empat lengan REST **BEBAS** 546.1 / 575.1 mm; B: arm_1 + arm_3 ke (0.45, 0, 1.30) **TABRAKAN** −16.7 / −15.2 mm ✅ |
| X1 | hull vs mesh arm_1×arm_3, celah tool 20–300 mm | hull **lebih besar** dari mesh sampai **0.4 mm** pada 3/5 (13.3 vs 13.1, 59.6 vs 59.3, 143.8 vs 143.4); kontak −15.0 vs 0.0. Toleransi GJK diketatkan → **lebih buruk** (+3.8 mm). Default dipakai; 0.4 mm lawan margin 50 mm. "Hull selalu konservatif" **salah** secara numerik — ditulis, tidak diasumsikan |
| V1 | `screen_interarm` arm_3 → (0.45, 0, 1.30), arm_1 REST vs **ditempatkan** di titik sama | REST **CLEAR** 365.0 mm; ditempatkan **COLLIDE** −18.9 mm (`t1_a1_right_finger_dist` ↔ `t2_a1_left_finger_dist`) ✅ |
| V2 | arm_2 × arm_4 bertabrakan **satu sama lain** (−15.8 mm), arm_1 yang bergerak | filter `only`: arm_1 **CLEAR** 624.4 mm — lengan diam tidak disalahkan ✅ |
| V3 | regresi `--dual` = g19 V0, rel 0.55 dan 0.95 | REST **CLEAR 295.6 mm**, ditempatkan **COLLIDE 0.0 mm** — identik dengan g19 ✅ |
| V4 | penempatan tanpa `t2_linear_joint` | **MENOLAK** (19/20 sendi) ✅ |

Skrip: [results/p1_g20/v_screen.py](results/p1_g20/v_screen.py).

#### B0.2 Kode yang diubah (nol gerak)

- `reach_dwell_probe.py`: `screen_interarm` menerima **daftar** lengan lain
  (se-gantry mesh + antar-gantry hull); `--arms …` / `--quad` (tiap rencana
  disaring terhadap ketiga lengan lain); `--dual` perilakunya **tidak berubah**
  (V3). Keluaran `/tmp/g20_step5.json` (menambah, seperti g17).
- `return_rest.py`: arm_3/arm_4 + `gantry_2_with_arm_controller`; satu
  panggilan = satu gantry (dua gantry ditolak); saringan se-gantry **dan**
  antar-gantry; torsi puncak hanya lengan yang digerakkan.
  🔴 **Cacat lama ditemukan:** panggilan satu-lengan (`--arms arm_1`) dulu
  menyaring terhadap partner di pose **netral URDF** (semua sendi 0), bukan
  terukur. Diperbaiki. Ke-23 log `return_rest` yang terarsip (g18/g19)
  memakai **kedua** lengan, jadi tidak terdampak; panggilan satu-lengan
  sebelum itu (g17, jika ada) **tidak terarsip** dan tidak dapat dinilai.
  DRY RUN di stack g18 (arm3/4 palsu, sendi 0): `--arms arm_1 arm_3` →
  **ditolak** (dua gantry); `--arms arm_3 arm_4` → se-gantry CLEAR 622.7 mm,
  antar-gantry CLEAR 546.1 mm, **tidak ada yang dikirim**.
- `dual_arm_targets.py --quad --lin2`: §A4.
- FK/RNEA translasi gantry 2 = gantry 1: §A4 (galat 6.7e-16, torsi 0.00).

#### B0.3 Saringan §A4 — plan-only, nol gerak

`dual_arm_targets.py --pairs-file results/p1_g18/b2_pairs.txt --quad --lin 0.550
--lin2 0.000 --repeats 3 --tau-max 12.0` pada `move_group` stack g18 (arm3/4
palsu di controller; posisi ditempatkan lewat `start_state`, nol perintah).
Data: [g20_screen_quad.json/.log](results/p1_g20/).

| # | 3/3 | ditolak oleh | g19 B0.2 (dua lengan, rel 0.550) |
|---|---|---|---|
| 1 | ✅ | — | ✅ |
| 2 | ✅ | — | ❌ a1 |
| 3 | ✅ | — | ✅ |
| 4 | ❌ | arm_1 sampel 1 | ❌ a2 |
| 5 | ❌ | **arm_3** sampel 1 | ✅ |
| 6 | ❌ | arm_1 sampel 1 | ❌ a1 |
| 7 | ❌ | arm_1 sampel 1 | ❌ a1 |
| 8 | ❌ | arm_2 sampel 2 | ❌ a2 |
| 9 | ❌ | arm_2 sampel 2 | ✅ |
| 10 | ✅ | — | ✅ |
| **lolos** | **4/10** | | 5/10 |

**Keenam penolakan = `TORQUE-UNSAFE`.** Nol NO-PLAN, nol INTERARM-COLLIDE /
MARGIN di 60 rencana. Jarak minimum sepanjang rencana: **antar-gantry 143.1 mm**
(`t1_a2_left_finger_dist` ↔ `t2_a1_upper_wrist`), se-gantry 370.6 mm. (Log
mencetak minimum dari dua penyaring per rencana; 143.1 adalah minimum global.)
Pasangan 2, 5, 9 berganti vonis dibanding g19 — variansi perencana, sekali lagi
(g19 B3 (5)); pasangan 5 kali ini ditolak di **arm_3**, salinan translasi eksak
arm_1: perencana, bukan geometri.

**Penugasan menurut §A4 (mekanis, [make_plan.py](results/p1_g20/make_plan.py)
→ [g20_plan.txt](results/p1_g20/g20_plan.txt))** — \|LQ\| = 4 → diputar (A8-5):

| i | kuartet | arm_1 | arm_2 | arm_3 | arm_4 |
|---|---|---|---|---|---|
| 1, 5, 9 | 1 | 1.000, 0.318, 1.240 | 0.214, 0.247, 1.240 | 0.450, −0.402, 1.240 | −0.336, −0.473, 1.240 |
| 2, 6, 10 | 2 | 0.857, 0.035, 1.080 | 0.286, 0.529, 1.160 | 0.307, −0.685, 1.080 | −0.264, −0.191, 1.160 |
| 3, 7 | 3 | 0.786, 0.388, 1.000 | 0.071, 0.176, 1.000 | 0.236, −0.332, 1.000 | −0.479, −0.544, 1.000 |
| 4, 8 | 10 | 0.714, 0.318, 1.240 | 0.071, 0.247, 1.160 | 0.164, −0.402, 1.240 | −0.479, −0.473, 1.160 |

⚠️ Hanya **empat** kuartet berbeda untuk 10 percobaan (A4 menyatakannya sebelum
data). Tahap 1b/2 memakai **kuartet 1** (`LQ[0]`).

🔒 **DIKUNCI** bersama §A.

#### B0.4 Validator penilai — diperluas ke N = 4

`validate_reach_dwell_monitor.py` hanya menguji jendela bersama **dua** lengan
(E). Ditambah **F** (empat lengan masuk berselang 0.8 s → jendela 4-lengan
harus mulai dari lengan **terakhir**) dan **G** (tiga masuk, arm_4 6 mm di luar
→ **tidak boleh** ada jendela 4-lengan). Uji kode di stack g18: **0 penilai
sebelum, 7/7 PASS, 0 sesudah** (F: 2.10 s setelah lengan terakhir). Validasi
resmi diulang di stack baru sebelum tahap 1a.

---

### B1. Tahap 0–2 — bring-up baru, gerak fisik PERTAMA arm_3 dan arm_4 (tidak dihitung A2)

#### B1.1 Tahap 0 — mematikan stack g18: 🔴 `kill -INT` ke launch TIDAK BERBUAT APA-APA

`kill -INT 871044` (PID `ros2 launch` asli, diverifikasi dari `/proc`), tunggu
> 2 menit: **semua node tetap hidup**. `/proc/871044/status`: **`SigIgn` bit 2
(SIGINT) aktif** — launch dilatarbelakangkan tanpa job control, bash
mengabaikan SIGINT untuk job latar, dan yang diabaikan diwariskan lewat `exec`.
Prosedur g18/g19 ("kill -INT ke PID asli") **tidak pernah bisa bekerja** pada
launch yang dimulai seperti itu. Anak-anaknya **menangkap** SIGINT, jadi:
SIGINT ke kelima node langsung → Kortex *deactivate* + *shutdown* bersih
(keempat hardware), lalu move_group / rviz2 / ros2_control_node **segfault di
destruktor (−11)** — sesudah hardware lepas; `robot_state_publisher`
di-*respawn* launch (PID baru 927408) dan yatim → SIGINT. Launch → SIGTERM.
`ros2 node list --no-daemon` **kosong**. Crash dump sesi ini (move_group 371 MB
+ rviz2 396 MB) dihapus; yang 12:28 (lebih tua dari sesi) dibiarkan. Disk 3.7 GB.

Launch baru dimulai lewat pembungkus yang mengembalikan SIGINT ke `SIG_DFL`
sebelum `exec` — `SigIgn` launch baru = HUP/PIPE/XFSZ saja, SIGINT **tidak**.

| Gerbang | Hasil |
|---|---|
| `ros2_kortex` | `ceiling-arm-fixes` **e712295** ✅ |
| `remount_check.py` | **GERBANG LULUS**; ICMP .13 / .12 / .11 / .10 ✅ |
| bring-up `arm3_fake:=false arm4_fake:=false enable_gantry_bridge:=true` (`/tmp/g20_t1.log`) | **4×** `Actuator count reported by robot is '6'`; **7/7** controller active; nol FAULT / Kortex exception. `[ERROR]` = kelas yang sama dengan log g18 (spawner ganda, KDL "not a chain", octomap updater) ✅ |
| `/joint_states` | t2 **nyata**: arm_3 110.73°, arm_4 100.14° dari REST (menggantung, pose sendi lain; j2 ≈ 31°); arm_1/arm_2 ≤ 0.06°; rel 0.550910 / 0.000000, rotasi 0 ✅ |
| URDF RSP baru vs `/tmp/reach_dwell_live.urdf` | 45 sendi, placement dan inersia selisih **0.0** — cache sah. RSP masih URDF mode palsu (g18 B0.6) |
| validator penilai (4 lengan) | **0 sebelum, 7/7 PASS, 0 sesudah** ([g20_validator.log](results/p1_g20/g20_validator.log)) ✅ |
| perekam + penilai hari-H | `js_record.py` 28 sendi → `/tmp/g20_js.csv`; `reach_dwell_monitor` 4 lengan → `/tmp/g20_step5_*` |

#### B1.2 Tahap 1–2 — izin operator per tahap

| Tahap | Gerak | Hasil |
|---|---|---|
| **1a** | `return_rest --arms arm_3 --move` (110.7°, 30 s) — **gerak fisik pertama arm_3 di proyek ini** | galat akhir **0.046°**, torsi puncak **4.879** `t2_a1_joint_2`; lengan lain bergeser ≤ 0.011°; rel/rotasi tetap; nol fault ✅ |
| **1b** | arm_3 → (0.450, −0.402, 1.240), disaring vs ketiga lengan lain | **penilai: SUCCESS** — settled **0.872 mm**, ori 1.73°, 41 sampel @ 20 Hz. Antar-lengan min 463.1 mm. j2 terukur **8.704** vs RNEA 4.79 → **+3.91**. Pulang: 0.044°, 5.016 N·m ✅ |
| **2a** | `return_rest --arms arm_4 --move` (100.1°) — **gerak fisik pertama arm_4** | galat **0.035°**, torsi **4.466** `t2_a2_joint_2`, nol fault ✅ |
| **2b** | arm_3 + arm_4 → kuartet 1, berurutan | **penilai: 2-ARM CONCURRENT**. settled arm_3 **1.706** / arm_4 **1.408 mm**. Antar-gantry min **533.9 mm** (`t1_a2_right_finger_prox` ↔ `t2_a1_right_finger_dist`). j2 terukur arm_3 **9.18** (RNEA 4.96 → **+4.22**), arm_4 **10.67** (5.84 → **+4.83**) |
| **2c** | `return_rest --arms arm_3 arm_4 --move` | galat **0.045°**, torsi 5.451, se-gantry 583.2 / antar-gantry 534.0 mm CLEAR, nol fault ✅ |

🔴 **Kesalahan saya, sebelum 2b pertama:** target arm_4 x < 0 (`-0.336,…`)
dibaca argparse sebagai **opsi** → probe keluar rc = 2 **sebelum** menyentuh
apa pun ([g20_2b_argparse_fail.log](results/p1_g20/g20_2b_argparse_fail.log)).
Tidak ada gerak. Tetapi `batch.sh` mengoper `--target4 $a4` — arm_4 x < 0 di
**keempat** kuartet — **dan** runner **tidak berhenti** pada rc probe ≠ 0: seri
langkah 5 akan kehilangan 10/10 percobaan tanpa auto-stop. Diperbaiki sebelum
data: `--target=…` untuk semua target, dan rc ≠ 0 = auto-stop. 2b dijalankan
ulang dalam izin yang sama.

Pertentangan: probe melabeli 1b **STAGGERED** — penilai menerbitkan event
`concurrent` hanya untuk `len(active) > 1` (`reach_dwell_monitor.py:245`),
jadi N = 1 tidak pernah "CONCURRENT". Vonis tahap 1b = SUCCESS penilai.
Tidak memengaruhi A2 (N = 4).

---

### B2. 🔒 LANGKAH 5 — sepuluh percobaan EMPAT lengan, DUA gantry, perangkat keras NYATA

Izin operator: percobaan 1 sendiri; lalu seri 2–10 dengan auto-stop S22.
**Auto-stop terpicu sekali** (percobaan 8, §B2.2); pemulihan dan percobaan
9–10 masing-masing dengan izin operator. Rel **0.550910 / 0.000000** dan rotasi 0
sepanjang seri (dicek sebelum tiap percobaan). Nol fault, nol Kortex exception,
nol red LED (LED arm_3 dicek fisik oleh operator setelah percobaan 8).

**Vonis dari PENILAI INDEPENDEN** — event `>>> 4-ARM CONCURRENT` di
[g20_monitor.log](results/p1_g20/g20_monitor.log), satu per jendela percobaan;
[g20_scored.json](results/p1_g20/g20_scored.json) ([analyze.py](results/p1_g20/analyze.py)).

| i | kuartet | vonis A1 4-lengan | settled mm a1 / a2 / a3 / a4 | ori maks | j2 terukur N·m a1 / a2 / a3 / a4 | probe |
|---|---|---|---|---|---|---|
| 1 | 1 | ✅ CONCURRENT | 2.08 / 2.03 / 1.67 / 2.03 | 3.01° | 7.79 / 9.92 / 6.28 / 10.99 | CONCURRENT |
| 2 | 2 | ✅ CONCURRENT | 1.88 / 1.77 / 1.57 / 1.63 | 1.86° | **13.03** / 11.38 / **12.53** / 4.05 | TORQUE-ABORT |
| 3 | 3 | ✅ CONCURRENT | 1.58 / 1.53 / 1.72 / 2.00 | 1.76° | 3.76 / 7.63 / 4.38 / 7.99 | CONCURRENT |
| 4 | 10 | ✅ CONCURRENT | 2.14 / 1.41 / 1.46 / 2.55 | 1.78° | **12.87** / 10.10 / 3.62 / 4.67 | TORQUE-ABORT |
| 5 | 1 | ✅ CONCURRENT | 1.59 / 1.55 / 2.17 / 1.06 | 2.19° | 6.87 / 9.87 / 9.46 / 4.51 | CONCURRENT |
| 6 | 2 | ✅ CONCURRENT | 1.02 / 1.70 / 2.45 / 1.89 | 2.59° | 11.59 / 3.85 / **12.55** / **12.83** | TORQUE-ABORT |
| 7 | 3 | ✅ CONCURRENT | 1.53 / 1.48 / 1.90 / 1.27 | 2.65° | 3.57 / 9.48 / 9.16 / 10.03 | CONCURRENT |
| 8 | 10 | ✅ CONCURRENT | 0.90 / 1.70 / 0.12 / 1.98 | 2.71° | 3.92 / 10.35 / 🔴 **14.27** / 10.93 | TORQUE-ABORT |
| 9 | 1 | ✅ CONCURRENT | 1.35 / 0.91 / 1.76 / 1.54 | 2.09° | 8.57 / 9.94 / 6.18 / 4.58 | CONCURRENT |
| 10 | 2 | ✅ CONCURRENT | 1.71 / 1.72 / 2.08 / 1.12 | 1.95° | 11.54 / **12.60** / **13.08** / **12.46** | TORQUE-ABORT |

Semua jendela 41–42 sampel @ 20 Hz; settled maks **2.55 mm** (median 1.68),
`pos_err_max` maks 4.95 mm (artefak saat-masuk, g17 B1.3). **STAGGERED 0,
PARTIAL 0, NEITHER 0, HALTED 0.** Label probe "TORQUE-ABORT" = pengamat
`--tau-max 12.0`, **bukan** vonis (g18 pertentangan 6); penilai memberi
CONCURRENT pada kelimanya. Satu rencana ditolak penyaring torsi di hari-H
(percobaan 6, arm_3: prediksi 14.05 > 14) → percobaan-ulang rencana, yang kedua
lolos. **Nol** penolakan penyaring antar-lengan / antar-gantry, nol
`UNSCREENED`, di 40 rencana yang dieksekusi. Jarak minimum yang tercatat
sepanjang rencana yang dieksekusi **291.0 mm** (percobaan 1, se-gantry
arm_3 ↔ arm_4); antar-gantry ≥ itu (§B4 (8)).

arm_1 masuk toleransi → jendela 4-lengan terbuka: median **97.3 s** (44.9 –
131.7) — arm_1 **menahan** selama tiga lengan lain direncanakan dan terbang,
tanpa satu pun reset jendela yang mematikan vonis. Durasi tugas (probe mulai →
event penilai) median **140.2 s**.

#### B2.1 🔒 Dua pembacaan (g18 B2.1)

**Pembacaan 1 — A1 terkunci:** **10 / 10 CONCURRENT ≥ 8 → LANGKAH 5 LULUS.**

**Pembacaan 2 — torsi terukur `joint_2`:**

| ambang | percobaan melewatinya | kalau dikeluarkan |
|---|---|---|
| rating **14 N·m** | 🔴 **1** (percobaan 8: **14.27**, arm_3) | 9 / 10 — lulus |
| nominal KA-75+ **12 N·m** | **5** (2, 4, 6, 8, 10) | **5 / 10 → tidak lulus** |

Kelima yang di atas 12 = kuartet **2** (3/3 kali: 2, 6, 10) dan kuartet **10**
(2/2 kali: 4, 8) — **sifat kuartet**, bukan acak; kuartet 1 dan 3 tidak pernah
di atas 11 N·m.

#### B2.2 🔴 Percobaan 8 — rating 14 DILEWATI, oleh rencana yang DIIZINKAN penyaring

Puncak **14.269 N·m** `t2_a1_joint_2` saat eksekusi arm_3 (+82.6 s), satu
sampel > 14, 125 sampel > 12 selama 2.58 s. Rencana itu **lolos** penyaring:
RNEA 7.12 + 6.6 = **13.72 ≤ 14**. Offset nyata **+7.15** — di atas +7.06 terbesar
g18 B2.4, dan persis kelas yang diperingatkan di sana ("+6.6 BUKAN batas atas",
"rencana 13.9 bisa terukur ~14.4").

Dari 40 rencana yang dieksekusi, **4** terukur di atas prediksi terkoreksi:
(2, a1) +0.11, (6, a3) +0.09, (8, a3) **+0.55**, (10, a3) +0.15 — **tiga dari
empat di arm_3.** Offset `joint_2` arm_3/arm_4 (23 rencana termasuk tahap 1–2):
+1.79 … **+7.15**; arm_1/arm_2 (20): +1.80 … +6.71.

Auto-stop S22 berhenti **sebelum** retract, lengan ditinggal di target (torsi
tahan maks 6.20 N·m). Operator: LED arm_3 **tidak merah**; pemulihan diizinkan —
`return_rest` gantry 2 lalu gantry 1: galat 0.049° / 0.086°, puncak 7.17 / 8.66
N·m, nol fault. Operator lalu memilih **menjalankan** percobaan 9–10 (usulan
saya: berhenti; vonis formal sudah tetap). Percobaan 10 (kuartet 2) terukur
13.08 — di bawah 14, di atas 12.

---

### B3. 🔒 D64–D70 DINILAI

| # | Dugaan | Terukur | Vonis |
|---|---|---|---|
| **D64** | ≥ 8/10 CONCURRENT | **10 / 10** | ✅ **TEPAT** |
| **D65** | saringan 4-lengan meloloskan ≤ 3/10 | **4 / 10** | ❌ **MELESET** ⁵ |
| **D66** | penyaring antar-gantry menolak 0 rencana; jarak antar-gantry min ≥ 150 mm | **0** penolakan (60 rencana saringan + 40 hari-H); min tercatat ≥ **143.1 mm** di saringan, ≥ **291.0 mm** di rencana yang dieksekusi | ✅ **TEPAT** (hari-H); ⚠️ saringan menyentuh 143.1 < 150 — dugaan menyebut "yang dieksekusi" |
| **D67** | offset j2 arm_3/arm_4 di +1.2…+7.1 pada ≥ 80 % | **22 / 23** (96 %); satu +7.15 | ✅ **TEPAT** |
| **D68** | STAGGERED = 0 | **0** (arm_1 menahan median 97 s) | ✅ **TEPAT** |
| **D69** | ≥ 1 pemblokir integrasi > 15 menit sebelum percobaan 4-lengan pertama | dua pemblokir: launch mengabaikan SIGINT (**~4 menit**, B1.1), argparse target negatif (**~5 menit**, B1.2) | ❌ **MELESET** |
| **D70** | origin gantry 2 = home fisik | dikonfirmasi operator sebelum bring-up | ✅ **TEPAT** ⁶ |

⁵ Arah benar (4 lengan lebih ketat dari 2: 4 < 5), besaran salah: penolakan
**tidak independen** antar-gantry — kuartet 1, 3, 10 lolos di kedua gantry
bersama, sesuai geometri translasi eksak.
⁶ Dinilai dari konfirmasi operator, bukan instrumen; tidak ada pengukuran
independen origin gantry 2.

➜ Papan skor §7.2: **38 meleset / 25 tepat → 40 meleset / 30 tepat.** Kedua
yang meleset berpola sama dengan G19: **kode/integrasi sendiri lebih murah dari
dugaan** (D69, kedua kalinya berturut-turut — prior "kode sendiri lebih
lambat" kini meleset 2 sesi berturut-turut untuk *integrasi*), dan besaran
kombinatorik yang mengasumsikan independensi (D65). Yang **tepat** lagi-lagi
hasil kasar (lulus, nol STAGGERED) — dan D66: **eksklusi antar-gantry tidak
mengikat pada rel 0.550/0.000 dengan kuartet ini.** Itu **bukan** bukti bahwa
ia tidak mengikat di rel lain (p1_state §7.2 D9: kelas ini pernah memberi contoh
tandingan).

---

### B4. Pertentangan §B lawan §A — G20

G17 enam, G18 tujuh, G19 tujuh.

| # | Pertentangan |
|---|---|
| **(1)** | prompt / g18–g19: "matikan dengan **kill -INT** ke PID ros2 launch ASLI"; launch yang dilatarbelakangkan **mengabaikan SIGINT** (`SigIgn` bit 2) — perintah itu tidak berbuat apa-apa (B1.1) |
| **(2)** | prompt: "validator penilai 5/5"; validator hanya menguji jendela bersama **N = 2**, sedangkan A1 sesi ini N = 4 — diperluas ke **7** kasus (B0.4) |
| **(3)** | A5/S18 (draf) menyebut hull "hanya bisa lebih konservatif"; terukur hull **sampai 0.4 mm di atas** mesh (X1) — ditulis ulang sebelum dikunci |
| **(4)** | A6 tahap 1b "penilai sukses (N = 1)"; probe melabelinya **STAGGERED** karena penilai tidak pernah menerbitkan `concurrent` untuk satu lengan (B1.2) |
| **(5)** | A2 mengandaikan runner menjalankan kuartet yang dikunci; argparse membaca target x < 0 sebagai opsi dan runner tidak berhenti pada rc ≠ 0 — **10/10 percobaan** akan hilang diam-diam. Kesalahan saya, diperbaiki sebelum data (B1.2) |
| **(6)** | A8-2 / A4: penyaring "prediksi + 6.6 ≤ 14" diperlakukan sebagai batas aman; **4/40** rencana terukur di atas prediksi terkoreksi, satu di atas **14** (B2.2) |
| **(7)** | A2: seri 10 percobaan tanpa jeda; di-*interupsi* S22 di percobaan 8 — retract percobaan 8 = **pemulihan** berizin, bukan retract runner; 9–10 dijalankan sesudah keputusan operator |
| **(8)** | A3: "jarak minimum **antar-gantry** sepanjang tiap rencana"; log probe hanya mencetak **minimum dari kedua penyaring** per rencana — antar-gantry diketahui hanya sebagai **batas bawah** (≥ 291.0 mm) di rencana yang minimumnya se-gantry |
| **(9)** | g17 S8 "disaring pada konfigurasi TERUKUR"; `return_rest` satu-lengan menyaring terhadap partner di pose **netral URDF** sejak ia di-commit (B0.2) — diperbaiki; log terarsip tidak terdampak |

**G20: SEMBILAN.**

---

## C. Keadaan akhir, dan yang BELUM dikerjakan (Rule 12)

- Keempat lengan di **REST** setelah percobaan 10 (runner: 0.045° / 0.079°,
  puncak 7.70 / 8.29 N·m). Rel **0.550910 / 0.000000**, rotasi 0 — **tidak
  pernah diperintah** sesi ini. Nol fault.
- Stack g20 **masih hidup** (launch PID **932613**, SIGINT **tidak** diabaikan —
  `kill -INT 932613` bekerja). Penilai dan perekam sesi dihentikan, nol sisa.
- Data: [results/p1_g20/](results/p1_g20/). ⚠️ `g20_joint_states_all.csv.gz`
  **95 MB** (g19: 29 MB) — rekaman penuh 28 sendi sejak bring-up.
- **Tidak diukur, disengaja:** gerak gantry (S20) — jadi langkah 5 **tidak**
  mengukur pindah gantry dengan empat lengan, dan **eksklusi antar-gantry
  hanya pada satu pasang rel** (0.550 / 0.000); lengan terentang saat gantry
  bergerak; rotasi gantry.
- **Hanya 4 kuartet berbeda** untuk 10 percobaan (A4, diputar).
- **Belum diperbaiki, disengaja:** URDF palsu di RSP/`move_group` (g18 B0.6);
  SRDF 112/121 + 283 pasangan dimatikan; label probe (TORQUE-ABORT ≠ vonis,
  STAGGERED untuk N = 1); log probe yang tidak memisahkan jarak se-gantry dan
  antar-gantry (B4 (8)); penyaring S18 tanpa sendi rotasi (S23).
- 🔴 **Keputusan operator terbuka:** rating `joint_2` **14 dilewati sekali**
  (14.27, arm_3) oleh rencana yang lolos penyaring; 5/10 di atas 12. Offset
  +6.6 terbukti **bukan** batas atas di arm_3 (3/4 pelanggaran). Penyaring
  diperketat (mis. prediksi + 7.2 ≤ 14, atau + 6.6 ≤ 13) **atau** kuartet 2/10
  dikeluarkan — itu keputusan **sebelum** sesi berikutnya, bukan sesudah data.

Lanjutan dari [p1_g19_hw.md](p1_g19_hw.md) §D; g19 tidak diubah.
