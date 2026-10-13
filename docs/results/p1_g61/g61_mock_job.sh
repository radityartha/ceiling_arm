#!/usr/bin/env bash
# G61 A5.1 + A5.2 on the mock (domain 77; REST + PARK + K0 before each). Fake HW, NOT data. Resumable: a run whose log
# already says "== SELESAI" is skipped. Run 3 also pauses the recorder 8 s (A5.5, SIGSTOP/SIGCONT by PID $REC_PID).
D=/home/user1/Documents/ceiling_arm/docs/results; cd $D/p1_g61
SEL=$D/p1_g55/g55_selected.json; G52=$D/p1_g52/g52_candidates.json
RUNS="pos_s7B:7:$SEL:R90sel:--goal-correct pos_s12L:12:$G52:R90:--goal-correct inj_s12_nocorr:12:$G52:R90:--inject-rot-error=1:1:-0.48 inj_s12_corr:12:$G52:R90:--inject-rot-error=1:1:-0.48,--goal-correct"
for R in $RUNS; do
  IFS=: read TAG S P V X <<< "$R"
  L=g61_smoke_$TAG.log
  if [ -f $L ] && grep -q "== SELESAI" $L; then echo "== $TAG sudah selesai"; continue; fi
  echo "== $TAG (s$S $V ${X//,/ }) mulai $(date +%T)"
  if [ $TAG = inj_s12_nocorr ] && [ -n "$REC_PID" ]; then
    ( until grep -q "\[g1 2\] task" /tmp/claude-1001/g61_$TAG/runner.log 2>/dev/null; do sleep 1; done
      kill -STOP $REC_PID; echo "$(date +%s.%N) SIGSTOP perekam $REC_PID" >> g61_recpause.log; sleep 8
      kill -CONT $REC_PID; echo "$(date +%s.%N) SIGCONT perekam $REC_PID" >> g61_recpause.log ) &
  fi
  bash g61_smoke.sh $TAG $S $P $V ${X//,/ } > $L 2>&1
  echo "== $TAG: $(grep -E 'SELESAI: |AUTO-STOP: ' $L | head -1 | cut -c1-160) | $(grep -E 'K0 (LULUS|TOLAK)' $L | tail -1 | cut -c1-80) $(date +%T)"
  grep -q "K0 LULUS" <(grep -E 'K0 (LULUS|TOLAK)' $L | tail -1) || { echo "== BERHENTI: K0 sesudah $TAG tidak LULUS"; exit 3; }
done
echo "== SEMUA SELESAI $(date +%T)"
