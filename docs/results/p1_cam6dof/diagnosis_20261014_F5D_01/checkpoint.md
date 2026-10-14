# Diagnosis pasif F5/D — checkpoint persiapan

Operator meminta diagnosis dan melaporkan botol sudah di lokasi. Konfirmasi gerbang/kesiapan sedang diminta; belum bring-up/capture/gerak. Domain0 kosong, inventaris proses tidak menemukan stack/controller/kamera/localizer/task server. Rencana kamera+localizer saja di domain79, incumbent+G34; controller tidak diperlukan.

F5=(0.6497,−0.4779), boxD140 langsung lantai, botol165×65. Raw kampanye lama utuh dan tidak diganti; diagnosis disimpan terpisah.

Gerbang operator diterima: “Semua benar dan aman; F5/D siap, saya sudah menjauh”. Domain79 preflight kosong. Camera+localizer terisolasi mulai, controller/move_group/taskserver tidak diluncurkan, NOL gerak. Decoder diuji pada32SC1 big-endian berpadded,32FC1 metrik dan proyeksi sinarutama: PASS tanpaROSnode. Raw disampling maksimal2Hz/kamera, timestamp/deltaRGB-depth-mask dan semua event/track direkam; bukan klaim sinkronisasi eksak.

Catatan proses: anggaran per-task4000token CLAUDE.md terlampaui pada diagnosis/perekam; batas ini diungkapkan, pekerjaan pengambilan/penutupan yang sudah diotorisasi tetap diselesaikan.
