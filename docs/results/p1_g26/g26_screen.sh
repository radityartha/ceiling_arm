#!/usr/bin/env bash
# G26 A2: (iv) sched_screen APA ADANYA, seed urut atas lolos (i)-(iii), sampai K=3 lolos. PLAN-ONLY.
ROOT=/home/user1/Documents/ceiling_arm
source $ROOT/ros2_ws/install/setup.bash
D=$ROOT/docs/results/p1_g26
SEEDS=$(python3 -c "import json;print(' '.join(str(r['seed']) for r in json.load(open('$D/g26_candidates.json')) if r.get('ok_ii')))")
K=3; n=0
for s in $SEEDS; do
  python3 -u $ROOT/docs/results/p1_g22/sched_screen.py --seeds $s --repeats 3 \
    --plan $D/g26_candidates.json --out $D/g26_screen.json 2>&1 | grep --line-buffered -v "UserWarning\|buildGeomFromUrdf"
  if python3 -c "import json,sys;r=[x for x in json.load(open('$D/g26_screen.json')) if x['seed']==$s];sys.exit(0 if r and r[-1]['ok'] else 1)"; then
    n=$((n+1)); echo "== LOLOS ke-$n: seed $s"; [ $n -ge $K ] && break
  fi
done
echo "== SELESAI: $n lolos"
