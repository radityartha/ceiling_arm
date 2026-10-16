## DEMO-ZONE-LIVE

### A. Protokol — DIKUNCI (sebelum kamera dibuka)

**A0.** Keadaan: papan polos di jari arm_2/arm_3, gripper tertutup, semua REST, g1 1.40, g2 0, stack MATI. NOL gerak lengan/rel.
Tanda uji **C3 (1.8486, 0.6786)** dan **M1 (1.7497, 0.5493)** — dipilih karena bersih dari oklusi (bukan dari galat). Keduanya ADA di
fit P3 (20 obs) → ini uji **penempatan ulang + node hidup**, BUKAN uji-tahan tanda (LOMO P3 = analisis luring). Pengungkapan: data A
dari localizer fusi (rgbd tidak melihat zona → = rgbd2), B + uji ini dari localizer rgbd2 saja.

**A1. Urutan** (kamera saja, domain terpisah 79 bila kosong; harness incumbent, ekstrinsik incumbent + G34, default launch tidak diubah):
1. `ros2 node list --no-daemon` domain 79 kosong; `cam_up.sh seg_imgsz:=1280` + `loc_cam.sh rgbd2` + `zone_up.sh` (node, H 0.07),
   semua `setsid nohup`, PID dicatat. Lampu NYALA (operator).
2. **Kosong:** tanpa botol di zona, rekam 5 s `/demo_zone/tracks` + `/demo_zone/rejected`.
3. Per tanda (C3 lalu M1): operator meletakkan botol demo putih 7 cm tepat di tanda, menjauh → rekam **15 s** bersamaan
   `/loc_rgbd2/tracks` (mentah) dan `/demo_zone/tracks` (terkoreksi) + `/demo_zone/rejected` (`track_median.py`).
4. Matikan per PID (SIGINT, anak `object_localizer` dibunuh per PID; tanpa `pkill -f`); node 0 di domain 79.

**A2. 🔒 Gerbang (per tanda uji, KEDUA tanda wajib lulus; tidak dilonggarkan)**

| # | Lulus bila |
|---|---|
| L1 | satu track tunggal: tepat SATU track diterima di `/demo_zone/tracks`, tid sama, hadir di ≥ 90 % pesan 15 s |
| L2 | \|median xy terkoreksi − tanda\| ≤ **5 cm** |
| L3 | median z track MENTAH ∈ **[0.79, 0.84]** |

L1–L3 ✓ di C3 DAN M1 → koreksi zona DITERIMA untuk demo (pemakaian = keputusan operator; runner tetap belum di-remap). Lainnya → ✗,
ditulis, tidak dipakai. Berhenti: kamera/driver gagal, `inference failed` berulang, orang di zona saat rekam (ulang tanda itu).

**A3. 🔒 Dugaan**

| # | Dugaan |
|---|---|
| D649 | L1 ✓ di kedua tanda; botol wastafel (`cyan bottle`) muncul di `/demo_zone/rejected`, tidak pernah diterima. |
| D650 | L2 dengan galat ≤ **3 cm** di kedua tanda (residu fit A: C3 0.6, M1 2.2 + penempatan ulang 0.4–2.2). |
| D651 | z mentah ∈ [0.80, 0.83] di kedua tanda. |
| D652 | galat MENTAH (tanpa koreksi) 0.15–0.20 m di kedua tanda (A: M1 0.164, C3 0.188). |
| D653 | output node hidup = P3 luring dari median mentah ≤ 1 mm. |
| D654 | Kosong (A1.2): nol track diterima selama 5 s. |
