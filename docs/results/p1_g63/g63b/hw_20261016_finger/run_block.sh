#!/usr/bin/env bash
# Blok mode jari (izin operator satu blok): per pose DRY -> harus PLANNED -> gerak -> harus MOVED+SUCCESS, torsi <= 14 ->
# tangkap (rc 0). Stop otomatis: bukan PLANNED / bukan SUCCESS / REJECTED / GOAL-MISMATCH (eksekusi) / torsi > 14 / tangkap rc != 0 /
# R0 tanpa deteksi. Pemakaian: run_block.sh <arm> <board> <stage> <pose:role> ...
G=/home/user1/Documents/ceiling_arm/docs/results/p1_g63; B=$G/g63b; SF=$B/hw_20261016_finger
source $G/../p1_demo_hw/harness/env.sh; cd $G
arm=$1; board=$2; stage=$3; shift 3
fk=$([ $arm = arm_2 ] && echo t1_a2 || echo t2_a1)
for pr in "$@"; do
  pose=${pr%%:*}; role=${pr##*:}
  echo "=== $pose ($role) $(date +%T)"
  echo "- (jari blok $stage) [GERAK] $arm $pose" >> $B/hw_20261016/s0_operator.md
  d=$(timeout 400 python3 g63_move.py --session $SF --poses $B/poses_finger_v1.json --payload $SF/payload_finger.urdf --extra-env $G/s0c_extra_env.npz --arm $arm --pose $pose --dry 2>&1)
  echo "$d" | grep -E "torsi RNEA|antar-lengan minimum|lingkungan minimum" | sed 's/.*reach_dwell_probe\]: //' | cut -c1-150
  echo "$d" | grep -q "rencana: PLANNED" || { echo "STOP: DRY bukan PLANNED"; echo "$d" | tail -3; exit 3; }
  m=$(timeout 400 python3 g63_move.py --session $SF --poses $B/poses_finger_v1.json --payload $SF/payload_finger.urdf --extra-env $G/s0c_extra_env.npz --arm $arm --pose $pose 2>&1)
  echo "$m" | grep -E "MOVED|SUCCESS|rc " | tail -3
  echo "$m" | grep -qE "\] SUCCESS" || { echo "STOP: gerak bukan SUCCESS"; echo "$m" | grep -E "REJECT|MISMATCH|ERROR|FAIL" | tail -5; exit 4; }
  tq=$(echo "$m" | grep -oE "torsi puncak +[0-9.]+" | awk '{print $3}' | tail -1)
  awk -v t="$tq" 'BEGIN{exit !(t>14)}' && { echo "STOP: torsi $tq > 14"; exit 5; }
  [ "$pose" = REST ] && continue
  timeout 300 python3 $B/g63b_capture.py capture --arm finger --session $SF --pose-id $pose --board $board --role $role --stage $stage --mount $B/mount_finger.json --save-frames > $SF/cap_${pose}_$(date +%H%M%S).log 2>&1
  rc=$?; [ $rc -ne 0 ] && { echo "STOP: tangkap rc $rc"; exit 6; }
  n=$(tail -1 $SF/poses.jsonl | python3 -c "import json,sys;d=json.loads(sys.stdin.read());fk='$fk';print('jari %.4f'%d['joints'][fk+'_right_finger_bottom_joint'], ' '.join(f\"{c}:{len(bv['ids'])}\" for c,v in d['cams'].items() for b,bv in v['boards'].items()))")
  echo "  $n"
  if [ "$role" = R0 ] && ! echo "$n" | grep -qE ":12"; then echo "STOP: R0 tanpa deteksi"; exit 7; fi
done
echo "BLOK SELESAI $(date +%T)"
