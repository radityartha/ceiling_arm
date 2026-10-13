#!/usr/bin/env bash
# G61 A5.4 + A5.3 (mock 77, fake HW, NOT data). s7 baru smoke WITHOUT home; when gantry 1's first task has been running 5 s,
# SIGKILL move_group by PID ($1, resolved beforehand -- no pgrep/pkill -f). Then a DRY run must be refused before t0.
# bash g61_neg.sh <move_group pid>
MG=$1; D=/home/user1/Documents/ceiling_arm/docs/results; cd $D/p1_g61; source g61_env.sh
SEL=$D/p1_g55/g55_selected.json; RL=/tmp/claude-1001/g61_neg_s7B/runner.log
grep -q moveit_ros_move_group/move_group /proc/$MG/cmdline || { echo "PID $MG bukan move_group"; exit 2; }
( until grep -q "\[g1 2\] task" $RL 2>/dev/null; do sleep 0.2; done; sleep 5
  echo "$(date +%s.%N) SIGKILL move_group $MG" | tee -a g61_neg_kill.log; kill -KILL $MG ) &
NO_HOME=1 bash g61_smoke.sh neg_s7B 7 $SEL R90sel --goal-correct > g61_smoke_neg_s7B.log 2>&1
echo "== neg rc: $(grep '== RC' g61_smoke_neg_s7B.log)"
bash g61_smoke.sh pre_nomg 7 $SEL R90sel --goal-correct --dry > g61_smoke_pre_nomg.log 2>&1
echo "== pra-cek: $(grep -E 'AUTO-STOP|== RC' g61_smoke_pre_nomg.log | tr '\n' ' ')"
