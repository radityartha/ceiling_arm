# Verifikasi kamera nyata sesudah fix freshness — 2026-10-14

**LULUS untuk perilaku kedaluwarsa track yang diuji; kalibrasi dan DEMO-HW belum diterima.** Fix produksi ada pada commit `c95e7d3`. Diagnosis awal tetap terdokumentasi pada commit `b8263d7`; hasil kampanye `8f93e14`, kunci §A, raw, model dan papan skor tidak diganti oleh uji ini.

## Lingkup dan konfigurasi

Operator telah mengonfirmasi sel aman, REST/PARK, lampu, e-stop, identitas kamera, serta botol yang sama 165×65 mm di atas D140 mm langsung pada FIT F5; operator kemudian meminta “lanjutkan”. Hanya kamera, relay, segmentasi, TF incumbent dan localizer terisolasi di domain79 dihidupkan. Controller/move_group/gantry bridge tetap mati; tidak ada gerak lengan/gantry, publikasi target gerak, fit atau penggunaan uji-tahan.

Harness `p1_demo_hw/harness/cam_up.sh seg_imgsz:=1280` dan `p1_cam6dof/harness/loc_cam.sh` memakai incumbent + koreksi G34. Argumen dan identitas PID/UID/command/startticks ada di `launch.json` dan `owned.json`. TTL **1.5 s**, `confirm_frames=3`, radius, rentang depth, minpixels dan gerbang tetap. Runtime workspace memakai source yang diperbaiki melalui symlink. Kamera RGBD2: serial234222303079, D455, firmware5.17.0.10, RealSense ROS/librealsense4.58.2/2.58.2 menurut log.

11 regresi luring sudah lulus pada checkpoint B62; kontrol kode asli membuktikan satu frame dapat menjadi track terkonfirmasi dan terus hidup hingga20 s, sementara fix mencegahnya. Uji kamera ini melengkapi bukti luring untuk jalur akuisisi nyata; tidak mengulang A0/G61/kalibrasi lama.

## Rekaman yang tidak cukup dan pengulangan terkontrol

Rekaman pertama `watch/` selesai sebelum RGBD2 dimatikan melalui `camera2_stop.json`. Karena tidak mencakup kejadian sesudah stop, **rekaman itu tidak dipakai untuk menyatakan expiry lulus**. Semua bukti tetap disimpan; kekurangan tersebut disampaikan kepada operator.

Hanya driver RGBD2 dihidupkan lagi menggunakan argv/params-file yang sama (PID954757, identitas di `camera2_restart_identity.json`), sementara relay/TF/localizer yang sama tetap hidup. `watch_freshness_live.py` merekam header depth/mask dan seluruh track selama30 s. Saat track RGBD2 pertama terkonfirmasi, alat memverifikasi identitas PID954757 lalu memberi SIGINT otomatis; tidak mengirim perintah robot. Rekaman sebelum/sesudah stop berada dalam satu `controlled/events.jsonl`.

## Hasil terkontrol

Waktu berikut adalah detik epoch; header akuisisi dan waktu penerimaan dicatat terpisah.

| Kejadian | Waktu |
|---|---:|
| Awal rekaman | 1791986716.853168 |
| Track RGBD2 pertama terkonfirmasi, ID72 | 1791986719.8072734 |
| SIGINT driver RGBD2 | 1791986719.8076468 |
| Header depth/mask RGBD2 terakhir | 1791986719.8661513 |
| Penerimaan track nonempty terakhir | 1791986721.3072417 |
| Penerimaan track empty berikutnya | 1791986721.8071203 |
| Akhir rekaman | 1791986746.8634253 |

Ada satu event sumber yang diterima sesudah SIGINT; SDK sedang menutup sensor. Karena itu umur dihitung dari **frame terakhir**, bukan dari sinyal stop. Track nonempty terakhir diterima **1.4411 s** sesudah header frame terakhir; empty berikutnya pada **1.9410 s**, sesuai TTL1.5 s + periode publikasi0.5 s. Setelah itu seluruh **51/51 pesan empty**, tanpa track muncul kembali. Total60 pesan track RGBD2,4 nonempty;44 depth dan5 mask. Angka/assertion ada di `controlled/result.json`. Tidak ada kode/parameter yang diubah sesudah melihat hasil ini.

Uji ini mendukung expiry cache pada jalur kamera nyata. Pembuktian satu frame tidak memenuhi tiga konfirmasi berasal dari regresi luring actual `_publish`, bukan jumlah pesan pada uji kamera. Track yang masih valid dapat dipublikasikan berulang selama TTL; **n pesan tetap bukan n pengukuran independen**. Watch mengamati topic, bukan instrumentasi callback internal; waktu penerimaan memiliki latensi dan penjadwalan. Ini bukan pengukuran presisi waktu keras atau gerbang C5 TF.

## Yang masih gagal dan tidak dijalankan

Track ID72 masih memberi z≈0.051–0.057 m, di bawah permukaan D0.140 m. Fix freshness tidak memperbaiki mask yang mencampur box, depth RGBD invalid, centroid, atau ekstrinsik. Tidak menilai ulang akurasi kalibrasi dari titik diagnosis tunggal ini. Mask/depth masih perlu diisolasi pada FIT; holdout F7/C3/C6 tidak digunakan untuk menyetel parameter.

Tidak ada TF kandidat, perubahan skala depth/default launch, kampanye baru, fit ulang, C2–C5 kandidat, K0 terukur baru atau DEMO-HW. Papan D590–D598 tetap **✗/✗/✗/—/—/✗/—/✓/✗**,1✓/5✗/3 tidak dijalankan; teks harfiah dan batas bukti ada di fase B58. “Hanya rgbd2 yang salah” **tidak didukung**: pipeline RGBD juga gagal gerbang lama; sumber galat fisik belum terisolasi.

## Penutupan dan arsip

SIGINT diberikan hanya pada root kamera952733 dan localizer952734/952736 setelah UID/command/startticks dicocokkan; restart kamera954757 sudah menerima SIGINT terkontrol. `shutdown_actions.json` mencatat aksi; `shutdown_verified.json` memverifikasi seluruh15 PID tercatat **absen**, ownedactive0/zombie0/staticTF0, domain0/79 masing-masing0 node dengan rc0. Tidak memakai pkill/pgrep-f atau menutup proses sesi lain. Tidak ada SIGKILL pada penutupan verifikasi ini. `seg_router` dan localizer menunjukkan pengecualian finalisasi `rcl_shutdown already called` sesudah SIGINT; log diperiksa, tidak diklaim semua exitrc0.

REST/PARK dikonfirmasi operator, bukan diukur ulang ketika controller mati. K0 terukur terakhir tetap penutupan kampanye CAM (`hw_20261014/k0_final.log`); tidak ada gerak baru sejak itu pada diagnosis/verifikasi ini. Gerbang fisik dan K0 wajib diverifikasi lagi sebelum bring-up HW selanjutnya.

Log/event/JSON disimpan di repo beserta SHA (`manifest.json`); tidak menimpa70 NPZ diagnosis awal dan tidak meng-commit raw besar. Anggaran4.000 token/tugas CLAUDE telah terlampaui pada rangkaian diagnosis/fix/verifikasi, diungkapkan. Model agen kuat, effort **high**, disarankan untuk isolasi mask/depth dan HW berikutnya; M6 tetap model utama bila FIT baru sah, M7 hanya lewat LOO FIT dan perubahan skala tersendiri dengan keputusan operator.

**Langkah berikut:** isolasi kontaminasi mask dan depth invalid memakai bukti FIT F5/D yang sudah ada, lalu siapkan dan kunci protokol pengukuran baru sebelum kampanye kalibrasi ulang; demo menunggu gerbang diterima dan izin operator.
