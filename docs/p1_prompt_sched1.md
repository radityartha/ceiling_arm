# Prompt Sesi G7 — SCHED-1: ground truth penjadwalan

> Ditulis 2026-08-13 di akhir Sesi G6 ([p1_g6_map.md](p1_g6_map.md)).
> Rekomendasi: **Opus, effort tinggi.** Keluaran scheduler adalah **jadwal**, dan
> jadwal yang salah tetap kelihatan masuk akal — tidak ada exception, tidak ada
> test merah. Ini persis kategori "kesalahan tidak bersuara", jadi jangan
> turunkan effort. Tidak ada perangkat keras di sesi ini.

---

Sesi G7 — SCHED-1: solver exact sebagai GROUND TRUTH untuk penjadwalan.
Ini kontribusi paper, dan belum ada satu baris pun kodenya.

BACA DULU, berurutan:
1. docs/p1_state.md   — §1 (formulasi XD [ST-MR-TA]), §5 (keputusan terkunci,
                        TERUTAMA §5.6 biaya setup dan §5.8 dwell), §6 (bahan
                        model biaya), §7.1 (URUTAN WAJIB) dan §7.2 (papan skor:
                        SEMBILAN dugaan meleset, semuanya ke arah yang sama)
2. docs/p1_next_steps.md — §1 Jalur A. Jalur C sudah SELESAI, jangan dibuka.
3. docs/p1_g6_map.md  — §A saja, sebagai CONTOH disiplin mengunci kriteria
                        sebelum menjalankan apa pun. Isinya tidak relevan.

=== KEADAAN FISIK ===
Lengan 4× MASIH DILEPAS. Kamera terpasang. Gantry jalan.
➜ Sesi ini SEPENUHNYA OFFLINE. Tidak ada kamera, tidak ada gantry, tidak ada
  lengan, tidak ada move_group. Kalau Anda merasa butuh perangkat keras,
  Anda salah membaca tugasnya.

=== TUGAS ===
Urutan §7.1 WAJIB dan JANGAN DIBALIK. Sesi ini hanya langkah 1 dan 2:

  1. Generator instance kecil (2–6 tugas) yang bisa dienumerasi TUNTAS.
  2. Solver exact (MIP/DP) -> optimum. INI GROUND TRUTH-NYA.

  3 dan 4 (heuristik, baseline) BUKAN bagian sesi ini. Kalau langkah 2 selesai
  lebih cepat dari dugaan, JANGAN lanjut ke heuristik — perkuat langkah 2:
  perbesar cakupan enumerasi, tambah instance patologis, verifikasi silang
  optimum dengan brute-force.

Alasannya bukan kerapian: heuristik yang ditulis duluan jadi jangkar. Begitu ia
mengeluarkan angka, semua orang mulai mempercayainya, termasuk penulisnya.

=== BAHAN YANG SUDAH SIAP — JANGAN BANGUN ULANG ===
- oracle kelayakan: capability.py + cap_g{1,2}_rail160.npz (33×72, grid, TERKUNCI)
- biaya setup (§5.6, R²=1.00000):
    T_traverse = max(T_lin, T_rot)
    T_lin(Δ) = 0.29 + Δ_mm/v_lin
    T_rot(Δ) = 0.26 + Δ_deg/v_rot
- durasi tugas tak-nol: dwell 2.0 s (§5.8) — inilah yang membuat makespan bukan
  sekadar soal setup
- mutex sumber daya: zona irisan `overlap` (r=0.20)
- tugas MR (handover): irisan ketat 42.9% (grid 154 titik) dan 53.1% (distribusi
  `surface`). ⚠️ DUA HIMPUNAN TARGET BERBEDA — DILARANG dikutip sebagai rentang
  "42.9–53.1%".

=== KRITERIA SUKSES — KUNCI SEBELUM SATU BARIS KODE, ke docs/p1_g7_sched.md §A ===
Disiplin G3/G4/G5/G6. Yang WAJIB diputuskan dan dikunci di awal:

1. UKURAN INSTANCE yang masih tractable untuk exact. Sebutkan batasnya sebagai
   angka (n tugas, n lengan), bukan "kecil".
2. DEFINISI MAKESPAN. Persisnya apa yang diukur, dari kapan sampai kapan.
3. HANDOVER: satu tugas dua-lengan, atau dua tugas tergandeng? Pilih SATU,
   tulis alasannya. Ini mengubah seluruh struktur model.
4. BUKTI OPTIMALITAS. Bagaimana Anda tahu solver exact-nya benar-benar exact?
   Minimal: brute-force enumeration pada instance terkecil harus MEMBERI
   JAWABAN YANG SAMA. Solver tanpa pembanding independen bukan ground truth,
   ia cuma solver kedua.
5. INSTANCE PATOLOGIS yang jawabannya sudah diketahui TANPA solver (mis. semua
   tugas hanya terjangkau satu lengan -> makespan = jumlah; dua tugas identik
   di dua lengan disjoint -> makespan = paralel). Tetapkan SEKARANG, sebelum
   melihat keluaran solver.

=== YANG SUDAH TERKUNCI, JANGAN DIBUKA ===
- Index capability = grid (p1_state §5.2). Tidak disentuh, tidak diukur ulang.
- Biaya setup §5.6 dan dwell §5.8. Dipakai, tidak diturunkan ulang.
- Batas rel 1600 mm (rel berhenti ~1656 mm; URDF/bridge mengklaim 2000 mm — itu
  SALAH, jangan dipercaya).
- Peta statis / determinisme / MS-BL / GCS. Sesi G5 dan G6 sudah selesai.
- gng.py tidak disentuh.

=== YANG WAJIB DISEBUT DI NASKAH, JANGAN DILUPAKAN ===
Tabrakan struktur gantry–gantry BELUM dimodelkan (pelat mount menyapu r=0.4 m di
y=±0.36, beririsan di y∈[−0.04,0.04]). Proksi polyline MEREMEHKAN volume sapuan
-> semua angka rugi adalah BATAS BAWAH, bukan nilai. Tulis ini di §A, bukan
ditemukan lagi di akhir.

=== ATURAN ===
- §7.2: UKUR, JANGAN MENDUGA. SEMBILAN dugaan meleset, semuanya ke arah yang
  sama — menduga kendala lebih mengikat daripada kenyataannya. Dugaan yang
  TEPAT selalu datang dari menelusuri jalur data kode, bukan dari intuisi.
- Kalau §B bertentangan dengan §A, yang menang §B, dan pertentangannya ditulis
  eksplisit, bukan dihaluskan.
- Kalau solver tidak bisa dibuktikan exact, LAPORKAN dan berhenti. Jangan
  menyebut heuristik sebagai ground truth.
- Akhiri dengan prompt sesi berikutnya (G8 = SCHED-2, heuristik + optimality gap).
