# G63b HW 2026-10-16 — laporan P0 arm_2 (cek PROSES)
Gerak 5/5 MOVED+SUCCESS (R0, A-01, A-02, A-03, R0); torsi puncak 12.00 N.m (A-01), 0 fault/REJECTED/GOAL-MISMATCH.
Tangkap 5/5 (A-01 sesudah deviasi G9: j6 dibebaskan pp <= 4.58e-3, operator "b saja"; 2 tangkap rc 2 sebelumnya).
Deteksi (kamera x muka direncanakan): R0#1 2/2, A-01 1/1, A-02 1/2 (rgbd B 46 deg/1.49 m < 50 % frame), A-03 1/1, R0#2 1/2 (rgbd B hilang)
 -> 6/8 = 75 % (+1 tak direncanakan: A-03 rgbd2 B). std sudut maks 0.18 px. Label salah 0 (overlay visual semua deteksi).
Jari arm_2 -0.0004..-0.0006 rad sepanjang blok (0.01 deg). j4 pp <= 2.8e-3; j6 pp 2.97e-3 hanya di A-01.
Solve P0: penjaga G10 TOLAK (rank 22 < 24) = sesuai dugaan.
G6' manual (solve ditolak -> PnP rgbd2 muka A, R0#1 vs R0#2, gerak FK dikurangkan; g6p_manual.py): tak-terjelaskan 5.31 mm, lateral 3.50 mm
 (> 2 mm) di 1.75 m (~1.2 px). Bukan gerbang resmi (G6' dinilai di solve), tapi tanda: geser dudukan / histeresis kardus / pengulangan FK.
Papan ChArUco lama masih di lantai kiri-bawah frame rgbd (penjaga menahan).
