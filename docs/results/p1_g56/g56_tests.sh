#!/usr/bin/env bash
# G56 A2 mock tests (domain 77; mock + monitor + recorder up, REST + PARK, K0). Fake HW, NOT data.
H=/home/user1/Documents/ceiling_arm/docs/results; cd $H/p1_g56
SP=/tmp/claude-1001/-home-user1-Documents-ceiling-arm/05bd37bd-31fc-4191-8a8f-e012117e87ee/scratchpad
export LAUNCH_LOG=/tmp/g56_mock_launch.log MON=$SP/g56_mon_samples.csv
SM=$H/p1_g54/g54_smoke.sh
echo "== A2.1 injeksi gerak $(date +%T)"
bash $SM g56_inj 7 $H/p1_g55/g55_selected.json R90sel --inject-stop-on-motion 2 > g56_inj.log 2>&1
grep -E "INJEKSI|gerak sesudah|RC run|K0 (LULUS|TOLAK)|TIDAK diluncurkan" g56_inj.log | cut -c1-200
echo "== A2.2 kontrol positif fault $(date +%T)"
bash $SM g56_fault 10 $H/p1_g55/g55_selected.json R90sel > g56_fault.log 2>&1 &
until grep -q "success@" g56_fault.log; do sleep 1; done
T=$(date +%s.%N); echo "[ros2_control_node-4] [ERROR] [$T] [controller_manager]: G56 uji kontrol positif penjaga" >> $LAUNCH_LOG
echo "baris fault ditambahkan $T"; wait
grep -E "AUTO-STOP KEDUA|RC run|K0 (LULUS|TOLAK)" g56_fault.log | cut -c1-200
grep -m1 "AUTO-STOP KEDUA" $H/p1_g54/smoke/g56_fault_runner.log | awk -v t=$T '{printf "jeda baris->AUTO-STOP %.2f s\n", $1-t}'
echo "== A2.3 sweep v3 $(date +%T)"
for k in 0 1 2 3 4 5 6 7; do bash g56_sweep.sh 2 $k $k --dry > /dev/null 2>&1 || { echo "DRY kaki $k gagal"; break; }
  bash g56_sweep.sh 2 $k $k 2>&1 | grep -E "== |G33|S28|akhir:|REFUSE|BERHENTI"; done > g56_sweep_mock.log
grep -cE "akhir:" g56_sweep_mock.log; grep -E "REFUSE|BERHENTI" g56_sweep_mock.log
echo "== SELESAI $(date +%T)"
