P1 — ARM1-ZONE: galat +0.15 m di zona arm_1 itu lengan/gantry atau kamera? Lalu (bila lulus) DEMO-HW ulang. Repo ceiling_arm,
branch feat/rgbd-deploy. Model: Opus, effort HIGH (galat diam: dua sumber kesalahan memberi angka yang sama; track tampak stabil).
Tempel ulang prompt yang SAMA bila sesi putus; lanjut dari docs/p1_campaign_state.md ("Langkah berikut").

BACA DULU (penuh): CLAUDE.md; MEMORY.md (terutama demo-hw-rgbd2-range-error, camera-world-vs-urdf-offset); docs/p1_campaign_state.md;
docs/p1_demo_hw.md (§A, B1, B5, B6); docs/p1_calib_charuco.md (penuh); docs/results/p1_calib_charuco/arm_fk_check.py;
docs/results/p1_demo_hw/harness/*.sh; docs/results/p1_calib_charuco/harness/cam_rs.sh; docs/p1_demo_hw_prompt.md (aturan + langkah 3–6).

KEADAAN (akhir CALIB-CHARUCO, 2026-10-12 19:00): sel home — lengan REST, g1 PARKIR 1.4993 m / 0°, g2 0.0009 m / 0°; stack MATI. Ekstrinsik
dipakai = launch default + koreksi G34 lewat arg (`cam_rs.sh` / `p1_demo_hw/harness/cam_up.sh`); kandidat ChArUco DITOLAK.
Papan ChArUco mungkin masih di atas laci putih (tanda P1/P2 unting di permukaan 0.625 m).

FAKTA YANG DIPEGANG:
- rgbd2 melihat arm_1 di +0.15 m (x) dengan DUA set ekstrinsik yang berbeda 6–10 cm → kemungkinan BUKAN kamera: (a) arm_1/gantry-1 di 1.50
  ≠ URDF (G34 hanya memverifikasi rel ≤ 1.45), atau (b) `arm_fk_check` menelan titik bukan-lengan (tirai/jendela/kabel) di 2.7 m.
- DEMO-HW: botol putih 7 cm di meja kayu (puncak meja 0.77 m terukur) terlihat HANYA rgbd2 (~2.9 m), track (1.94, 0.70, 0.82); YOLOE perlu
  `seg_imgsz:=1280`; botol di luar sel (wastafel, (3.10, −0.33)) ikut ter-track → singkirkan/tutup.
- Penunjuk lengan + unting terbukti: TCP ≤ 2 mm dari perintah (TF `t1_aN_tool_frame` dari sendi terukur), operator menandai titik.
- 🔴 Bug `calibrate_extrinsics.py`: titik ORIGIN tercetak = corner id0 = (0.035, 0.035) di frame papan, tetapi `--board-xyz` dipakai sebagai
  sudut LUAR (0, 0) → geser 3.5 cm/sumbu. Docstring `generate_charuco_board.py`/`calibrate_extrinsics.py` menyatakan sebaliknya.
- Jangkauan tool-bawah ~0.75 m 3D dari basis. arm_1 basis (1.90, 0.36, 1.952) @ g1 1.50. P1 demo p_cmd (1.75, 0.55, 1.22) lolos mock 3/3;
  (1.94, 0.70, 1.07) di HW j2 99.6 %.
- `pose_to_g` S13″ hanya menerima stop jadwal/p0/PARK: g1 1.30 = g31 s33 R0, g2 0.45 = g31 s21 R0.
- table1 "NOT armed" sampai perintah gantry pertama (normal bila sesi lengan-saja). SIGINT ke wrapper `ros2 run` tidak mematikan anak →
  kill PID `lib/<pkg>/<node>`. Operator gagal membuka voice UI di Chrome → perintah teks via POST `/api/transcript` (operator setuju).

GERBANG (AskUserQuestion SEBELUM apa pun): K0 sel = akhir CALIB-CHARUCO? K1 operator di e-stop FISIK tiap gerak? K2 lampu nyala; botol
putih 7 cm siap; botol wastafel disingkirkan; papan ChArUco boleh diangkat? K3 tak ada task server / reach_fusion hidup (cek juga `ps`)?

TUGAS — dokumen fase docs/p1_arm1_zone.md: §A (kriteria + dugaan D570+) DIKUNCI sebelum gerak pertama (salinan
results/p1_arm1_zone/arm1_zone_A_locked.md + sha256), §B ditulis, commit per bagian, berkas keadaan diperbarui sesudah tiap langkah berarti.
0. LURING (nol gerak, sebelum §A dikunci): (i) perbaiki bug titik ORIGIN (pilih satu: `--board-xyz` = titik ORIGIN tercetak dan geser
   objek PnP −id0, ATAU docstring + cetakan; uji sintetis round-trip harus memberi galat 0); (ii) `arm_fk_check.py` opsi penyaring titik
   lengan saja (mis. hanya titik ≤ 3 cm dari geometri URDF lengan sesudah geser terbaik, laporkan jumlah titik dibuang) supaya (a)/(b)
   bisa dibedakan. Tulis di §A0.
1. Bring-up nyata (pola DEMO-HW B2; `octomap_updaters:=false`), K0 `--rail1 1.50 --rail2 0.00`, monitor + perekam (cek tumbuh). Kamera
   incumbent (`cam_rs.sh`), `arm_fk_check` lama + baru di g1 1.50 → apakah +0.15 m arm_1 bertahan dengan penyaring titik-lengan?
2. Uji zona arm_1 dengan arm_1 sebagai penunjuk (SETIAP gerak ditanya): plan-only lalu arm_1 → T = (1.75, 0.55, 1.22) tool-bawah; baca TCP;
   operator menandai unting di meja (0.77); kamera (rgbd2) mengukur gripper arm_1 yang melayang vs FK; return_rest. Botol putih diletakkan
   tepat di tanda → kamera + YOLOE (`seg_imgsz:=1280`) → track vs tanda (1.75, 0.55) = galat kamera di zona demo (acuan = world FK, bebas
   dari bentuk REST URDF). Opsional (operator): pita antar-basis arm_1–arm_2 (URDF 0.80 m) dan posisi carriage g1 terhadap stop ujung.
3. Keputusan (dikunci di §A sebelum langkah 2): track vs tanda ≤ 5 cm xy dan z ∈ [tinggi meja + 0.02, + tinggi botol] → DEMO-HW ulang di
   sesi ini (langkah 3–6 docs/p1_demo_hw_prompt.md, `--tau-max 14`, botol di tanda, 1–3 jalan, tiap jalan ditanya). > 5 cm → BERHENTI,
   laporkan apakah galat = lengan/gantry (FK vs tanda vs kamera) atau kamera; JANGAN menambah koreksi lokal tanpa keputusan operator.
4. Akhir: lengan REST, g1 PARKIR, K0, matikan bersih (node 0 di domain 0 dan 79, 0 static_transform_publisher), arsip /tmp, laporan
   + rekomendasi model/effort sesi berikut, "Langkah berikut" satu kalimat.

ATURAN KERAS: SETIAP gerak nyata ditanya dulu (satu langkah per persetujuan, termasuk rel dan pulang). Fault / LED merah / REJECTED /
NOT armed sesudah ARMED / torsi > 14 / benturan / kabel / gerak tak diminta → BERHENTI, amankan, tulis, tanya. MARGIN = TOLAK; penyaring
TIDAK dilonggarkan. Kriteria terkunci tidak diubah sesudah melihat data. Gantry hanya digerakkan bila operator setuju dan lewat
`pose_to_g` (stop jadwal).
