#!/usr/bin/env bash
# Build one GNG reachability/capability map PER ARM, looping over a config list.
#
# Each arm gets its own dataset + model:
#   <OUT_DIR>/<name>_dataset.npz   (FK samples)
#   <OUT_DIR>/<name>_model.npz     (+ _stats.npz)  <- loaded by visualize/seed_ik
#
# Run from the REPO ROOT (config 'urdf:' paths are repo-root-relative):
#   source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
#   ros2_ws/src/reachability_gng/build_maps.sh
#
# Override the dense recipe via env vars, e.g.  N=20000 LAM=120 build_maps.sh
set -euo pipefail

# --- arm list: "<name>:<config path, repo-root-relative>" ---------------------
ARMS=(
  "arm1:ros2_ws/src/reachability_gng/config/arm1_table1.yaml"
  "arm2:ros2_ws/src/reachability_gng/config/arm2_table1.yaml"
  "arm3:ros2_ws/src/reachability_gng/config/arm3_table2.yaml"
  "arm4:ros2_ws/src/reachability_gng/config/arm4_table2.yaml"
)

# --- dense recipe knobs (see README) -----------------------------------------
OUT_DIR="${OUT_DIR:-/tmp}"
N="${N:-80000}"
MAX_NODES="${MAX_NODES:-3000}"
LAM="${LAM:-60}"
EPOCHS="${EPOCHS:-2}"
TASK="${TASK:-pos}"
# BOUNDARY>0 pins that many fixed shell nodes on the true reachable surface so
# the node hull reaches the real workspace edge instead of settling ~half a cell
# inside it (set BOUNDARY=0 for the legacy centroid-only map).
BOUNDARY="${BOUNDARY:-600}"
BOUNDARY_TAU="${BOUNDARY_TAU:-0.4}"
# The gantry rail (ceiling) is at world z = 2.05 m and the arms hang below it, so
# any FK sample above this is physically impossible (the arm would go through the
# ceiling). Drop them so the capability map stays below the ceiling. Raise this to
# effectively disable the cut.
CEILING="${CEILING:-2.05}"

mkdir -p "$OUT_DIR"

# The 4 arms are fully independent (separate dataset + model), so build them in
# parallel. Each worker is pinned to nproc/JOBS BLAS threads to avoid the four
# processes oversubscribing the CPU (which would make the parallel run no faster
# than sequential). Override with JOBS=1 for the old one-at-a-time behaviour.
NPROC="$(nproc 2>/dev/null || echo 4)"
JOBS="${JOBS:-${#ARMS[@]}}"
THREADS="${THREADS:-$(( NPROC / JOBS > 0 ? NPROC / JOBS : 1 ))}"
export OMP_NUM_THREADS="$THREADS" OPENBLAS_NUM_THREADS="$THREADS" \
       MKL_NUM_THREADS="$THREADS" NUMEXPR_NUM_THREADS="$THREADS"

build_one() {
  local name="$1" cfg="$2"
  local dataset="$OUT_DIR/${name}_dataset.npz"
  local model="$OUT_DIR/${name}_model.npz"

  echo "=== [$name] data_gen ($N samples) -> $dataset ==="
  python3 -m reachability_gng.data_gen --config "$cfg" --out "$dataset" --n "$N"

  echo "=== [$name] train (max-nodes=$MAX_NODES lam=$LAM epochs=$EPOCHS boundary=$BOUNDARY) -> $model ==="
  python3 -m reachability_gng.train --dataset "$dataset" --out "$model" \
      --config "$cfg" \
      --task "$TASK" --max-nodes "$MAX_NODES" --lam "$LAM" --epochs "$EPOCHS" \
      --boundary-nodes "$BOUNDARY" --boundary-tau "$BOUNDARY_TAU" \
      --ceiling "$CEILING"
}

echo "=== building ${#ARMS[@]} arms, JOBS=$JOBS, THREADS=$THREADS/worker (nproc=$NPROC) ==="

pids=()
names=()
for entry in "${ARMS[@]}"; do
  name="${entry%%:*}"
  cfg="${entry#*:}"
  # cap concurrency at JOBS: block until a slot frees up
  while (( $(jobs -rp | wc -l) >= JOBS )); do wait -n; done
  build_one "$name" "$cfg" > "$OUT_DIR/${name}_build.log" 2>&1 &
  pids+=("$!")
  names+=("$name")
done

fail=0
for i in "${!pids[@]}"; do
  if wait "${pids[$i]}"; then
    echo "=== [${names[$i]}] OK (log: $OUT_DIR/${names[$i]}_build.log) ==="
  else
    echo "=== [${names[$i]}] FAILED -- last 20 log lines: ==="
    tail -n 20 "$OUT_DIR/${names[$i]}_build.log" || true
    fail=1
  fi
done

if (( fail )); then
  echo "=== ERROR: one or more arms failed to build ==="
  exit 1
fi
echo "=== done. Models in $OUT_DIR: ${ARMS[*]%%:*} ==="
