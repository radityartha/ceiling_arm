# P1 / G56 — persiapan hari HW 2026-10-12 (OFFLINE + mock)

> Sesi 2026-10-11 malam (operator: "HW baru besok, selesaikan semua offline dulu"). NOL gerak HW. §A dikunci (salinan
> `docs/results/p1_g56/g56_A_locked.md` + sha256) sebelum uji mock. §B menang atas §A.

## A. Protokol — DIKUNCI

### A0. Pengungkapan + analisis P2 (dari kode, sebelum uji)

- **Bridge gantry** (`moving_table_pkg/dual_table_controller.py` `_on_hw_command`): setpoint JTC di-debounce (1 mm / 0.3°) dan
  diteruskan sebagai `go_to_absolute` ke motor AZ yang meramp SENDIRI; target baru hanya dikirim sesudah gerak sebelumnya
  SELESAI. `stop_event` hanya diset oleh layanan `move_dual_table` (op 0 STOP), BUKAN oleh cancel JTC.
  → **Cancel di tengah TRAVERSE di HW tidak menghentikan rel seketika**: kaki yang sudah terkirim diselesaikan, lalu bridge
  mengejar posisi tahan JTC (posisi saat cancel) — bisa MUNDUR sampai satu kaki. Semua titik itu ada di dalam persegi lin × rot
  traverse yang sudah disaring S28/G33, dan gantry lain DIAM (traverse eksklusif) → aman, tetapi "berhenti ≤ 0.5 s" (G54 B4,
  mock) TIDAK berlaku untuk traverse di HW. `move_dual_table` STOP tidak dipakai (memori: JTC yang menahan posisi lama menyeret
  gantry kembali).
- **Lengan Kortex**: cancel JTC → JTC menahan posisi terukur; driver mengalirkan posisi → berhenti cepat (deselerasi).
  Belum terukur di HW → uji stop HW memukul saat **retract lengan**, bukan traverse.
- Injeksi G54 berbasis URUTAN event bergantung linimasa mock (percobaan 1 tidak sah). Untuk HW: injeksi berbasis KONDISI.
- Penjaga G54 membaca SELURUH launch log 10 Hz; di HW log bisa besar → kini inkremental (biner, offset).
- s11 R90 butuh g2 −85; amplop HW g2 −35…+95. EdgeMatrix (env, 50 mm) g2 lin 0.15: 0 → −50/−70/−85/−90 CLEAR, min 301.5 /
  289.4 / 271.6 / 270.8 mm (dihitung sebelum §A).

### A1. 🔒 Perubahan

1. `run_g54 --inject-stop-on-motion G`: AUTO-STOP saat gantry G menjalankan RETRACT yang lengannya sudah bergerak > 2° dari
   awal event dan masih > 10° dari REST.
2. Pemeriksaan fault launch log inkremental.
3. Sweep v3 `results/p1_g56/g56_sweep.sh` + `g56_sweep_legs.json` (g2: (0.15, 0) → −50 → 0 → −70 → 0 → −90 → 0 → (0, 0)) +
   `g56_sweep_plan.json` (salinan G50 + stasiun negatif).

### A2. 🔒 Uji mock (domain 77, monitor + perekam, REST + PARK, K0)

1. **Injeksi gerak:** s7 R180 `--inject-stop-on-motion 2` → rc 3; `inject.moved_deg` > 2, `left_deg` > 10; posisi lengan g2
   konstan (perekam) dari t_stop + 1 s; tidak ada alat diluncurkan sesudah stop.
2. **Kontrol positif fault:** saat smoke berjalan, tambahkan satu baris `[ros2_control_node-1] [ERROR] [ros2_control_node]: G56
   uji` ke launch log mock → AUTO-STOP "launch log: 1 baris fault" ≤ 1 s; rc 3.
3. **Sweep v3:** 8/8 kaki rc 0 di mock (DRY tiap kaki dulu), G33/S28 CLEAR.

### A3. 🔒 Dugaan

| # | Dugaan |
|---|---|
| D480 | Injeksi gerak lulus pada percobaan pertama (sah: retract g2 sedang bergerak). |
| D481 | Kontrol positif fault: AUTO-STOP ≤ 1 s sesudah baris ditambahkan. |
| D482 | Sweep v3 8/8 rc 0, sapuan G33 min ≥ 200 mm. |

## B. Hasil terukur

§A sha256 `60b77801…` ([salinan](results/p1_g56/g56_A_locked.md)). Skrip [g56_tests.sh](results/p1_g56/g56_tests.sh) (dalam job
`results/p1_g55/g55_mock.sh`), mock seperti G55 B4.

### B1. Perubahan tambahan (tidak ada di §A)

- **Celah penjaga ditemukan:** `FAULT_RE` run_g32 (`\[ERROR\] \[ros2_control_node`) TIDAK pernah cocok dengan format launch nyata
  `[ros2_control_node-4] [ERROR] [<stamp>] [controller_manager]: …` → error ros2_control_node generik tidak terlihat oleh run_g32
  (dan run_g54 G54). run_g54 kini + `\[ros2_control_node-[0-9]+\] \[ERROR\]`. Log HW G51 (3 bring-up): 7 baris per bring-up, semua
  pesan spawner SAAT bring-up (masuk `base_faults`) → tidak ada pemicu palsu. **run_g32 TIDAK diubah** (alat serial; catat untuk
  dibersihkan).
- Baris uji A2.2 memakai format nyata di atas (bukan format di §A, yang tidak akan cocok dengan pola baru maupun lama).

### B2. Uji mock

1. **Injeksi gerak** (s7 `R90sel`, `--inject-stop-on-motion 2`) — [run](results/p1_g54/smoke/g56_inj_run.json), [perekam ±10 s](results/p1_g56/g56_inj_joint_states_window.csv.gz):
   AUTO-STOP pada percobaan pertama saat retract g2 bergerak 2.2° (105.8° dari REST, berakselerasi 0.04 → 0.62°/0.5 s); dari
   t_stop + 0.5 s perubahan 0.000° per 0.5 s sampai + 4 s; `post_stop_motion` {1: 0.0, 2: 0.0}; retract g1 yang sudah lolos gerbang
   "TIDAK diluncurkan"; rc 3; pulang + K0 LULUS → **D480 ✓**.
2. **Kontrol positif fault** — [log](results/p1_g56/g56_fault.log): baris ERROR ros2_control_node ditambahkan saat s10 berjalan →
   "AUTO-STOP KEDUA gantry: penjaga: launch log: 1 baris fault/exception baru" **0.01 s** kemudian, rc 3, pulang + K0 LULUS → **D481 ✓**.
3. **Sweep v3** — [log](results/p1_g56/g56_sweep_mock.log): **8/8 kaki** DRY lalu gerak, rc 0, G33/S28 CLEAR, t_traverse ≈ T_cmd
   (−90°: 15.55 vs 15.71 s). G33 melaporkan min **115.1 mm** = jari t1_a1 (g1 diam di PARK) vs peta, bukan sapuan g2; jarak g2 sendiri
   = EdgeMatrix 270.8 mm (A0). Dugaan "min ≥ 200 mm" salah rumus (min G33 mencakup tautan diam) → dinilai harfiah **D482 ✗**.
   Catatan: tidak ada jadwal terpilih G55 yang butuh g2 < −35; sweep v3 hanya untuk melengkapi amplop R90 bila operator mau.

### B3. Papan skor

| # | Dugaan | Hasil |
|---|---|---|
| D480 | injeksi gerak lulus percobaan pertama | ✓ |
| D481 | fault → AUTO-STOP ≤ 1 s | ✓ 0.01 s |
| D482 | sweep v3 8/8, G33 min ≥ 200 mm | ✗ harfiah (8/8 rc 0; min 115.1 = g1 diam) |

**Tally G56: 2/3.** Rencana hari HW: [p1_hw_day_2026-10-12.md](p1_hw_day_2026-10-12.md).
