#!/usr/bin/env bash
# G61 A5 (fake HW, NOT data): run_g54 [--dry] then home by retrace (g47_home.sh) + K0. MOCK domain 77 only. = g54_smoke.sh
# with the G61 env + archive.   bash g61_smoke.sh <tag> <seed> <plan json> <variant> [extra run_g54 args...]
source $(dirname $0)/g61_env.sh
ROOT=/home/user1/Documents/ceiling_arm; H=$ROOT/docs/results/p1_g61; mkdir -p $H/smoke
TAG=$1; S=$2; PLAN=$3; V=$4; shift 4
F="UserWarning\|buildGeomFromUrdf\|self.geom = \|V = np.array"
python3 -u $ROOT/docs/results/p1_g54/run_g54.py --seed $S --plan $PLAN --variant $V --final-retract --rot-max 125 --goal-tol-deg 0.05 \
  --launch-log $LAUNCH_LOG --mon $MON --out-dir /tmp/claude-1001/g61_$TAG --archive $H/smoke --prefix ${TAG}_ "$@" 2>&1 \
  | grep --line-buffered -v "$F"
echo "== RC run_g54 ${PIPESTATUS[0]}"
case " $* " in *" --dry "*) echo "== DRY: tidak pulang"; exit 0;; esac
[ -n "$NO_HOME" ] && { echo "== NO_HOME: tidak pulang"; exit 0; }
bash $ROOT/docs/results/p1_g47/g47_home.sh $S $PLAN $V
echo "== SELESAI $(date +%T)"
