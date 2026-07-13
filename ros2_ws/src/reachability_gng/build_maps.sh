#!/usr/bin/env bash
# Build one GNG reachability/capability map PER ARM, looping over a config list.
#
# Adding arm_3 / arm_4 later is a ONE-LINE addition to the ARMS array below
# (plus a matching config/<name>.yaml). Each arm gets its own dataset + model:
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
  "arm3:ros2_ws/src/reachability_gng/config/arm3_table2.yaml"   # gantry_2, t2_a1
  "arm4:ros2_ws/src/reachability_gng/config/arm4_table2.yaml"   # gantry_2, t2_a2
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

mkdir -p "$OUT_DIR"

# The GNG fit dominates the ~2.7 min/arm build and is single-threaded Python, so
# extra BLAS threads barely help it -- but the 4 arm maps are INDEPENDENT, so
# building them CONCURRENTLY (one process per core) is a near-linear win with
# byte-identical output: ~11 min sequential -> ~3 min. Set PARALLEL=0 to force
# sequential (small/contended host); JOB_THREADS bounds each job's BLAS threads
# so N_arms x threads doesn't oversubscribe the host or starve a live move_group.
PARALLEL="${PARALLEL:-1}"
JOB_THREADS="${JOB_THREADS:-4}"
export OMP_NUM_THREADS="$JOB_THREADS" OPENBLAS_NUM_THREADS="$JOB_THREADS" \
       MKL_NUM_THREADS="$JOB_THREADS" NUMEXPR_NUM_THREADS="$JOB_THREADS"

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
      --boundary-nodes "$BOUNDARY" --boundary-tau "$BOUNDARY_TAU"
}

if [ "$PARALLEL" = "1" ]; then
  pids=(); names=(); fail=0
  for entry in "${ARMS[@]}"; do
    name="${entry%%:*}"
    build_one "$name" "${entry#*:}" > "$OUT_DIR/${name}_build.log" 2>&1 &
    pids+=("$!"); names+=("$name")
    echo "=== [$name] building in background (pid $!) -> $OUT_DIR/${name}_build.log ==="
  done
  for i in "${!pids[@]}"; do
    if wait "${pids[$i]}"; then echo "=== [${names[$i]}] OK ==="
    else echo "=== [${names[$i]}] FAILED (see $OUT_DIR/${names[$i]}_build.log) ==="; fail=1; fi
  done
  [ "$fail" = 0 ] || { echo "one or more arm builds failed"; exit 1; }
else
  for entry in "${ARMS[@]}"; do
    build_one "${entry%%:*}" "${entry#*:}"
  done
fi

echo "=== done. Models in $OUT_DIR: ${ARMS[*]%%:*} ==="
