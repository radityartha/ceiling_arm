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
  # "arm3:ros2_ws/src/reachability_gng/config/arm3_table2.yaml"   # future: one line
  # "arm4:ros2_ws/src/reachability_gng/config/arm4_table2.yaml"
)

# --- dense recipe knobs (see README) -----------------------------------------
OUT_DIR="${OUT_DIR:-/tmp}"
N="${N:-80000}"
MAX_NODES="${MAX_NODES:-3000}"
LAM="${LAM:-60}"
EPOCHS="${EPOCHS:-2}"
TASK="${TASK:-pos}"

mkdir -p "$OUT_DIR"

for entry in "${ARMS[@]}"; do
  name="${entry%%:*}"
  cfg="${entry#*:}"
  dataset="$OUT_DIR/${name}_dataset.npz"
  model="$OUT_DIR/${name}_model.npz"

  echo "=== [$name] data_gen ($N samples) -> $dataset ==="
  python3 -m reachability_gng.data_gen --config "$cfg" --out "$dataset" --n "$N"

  echo "=== [$name] train (max-nodes=$MAX_NODES lam=$LAM epochs=$EPOCHS) -> $model ==="
  python3 -m reachability_gng.train --dataset "$dataset" --out "$model" \
      --task "$TASK" --max-nodes "$MAX_NODES" --lam "$LAM" --epochs "$EPOCHS"
done

echo "=== done. Models in $OUT_DIR: ${ARMS[*]%%:*} ==="
