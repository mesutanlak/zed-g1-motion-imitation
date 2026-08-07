#!/usr/bin/env bash
set -euo pipefail

# Official Unitree 23-DOF physics motion-tracking pipeline.  It deliberately
# does not deploy to DDS or a physical robot.
PROJECT=${PROJECT:-/mnt/c/Users/Misafir/Desktop/ZED_G1_Projesi}
MJLAB=${MJLAB:-/home/misafir/unitree_rl_mjlab}
PYTHON=${PYTHON:-/home/misafir/miniconda3/envs/unitree_rl_mjlab/bin/python}
CSV=${1:?usage: prepare_and_train_g1_23dof.sh motion.csv [iterations] [envs]}
ITERATIONS=${2:-10}
ENVS=${3:-256}
NAME=${MOTION_NAME:-zed_g1_upper_body}

cd "$MJLAB"
"$PYTHON" scripts/csv_to_npz.py \
  --input-file "$CSV" \
  --output-name "$NAME" \
  --input-fps 30 \
  --output-fps 50 \
  --robot g1_23dof

MOTION="$MJLAB/src/assets/motions/g1_23dof/${NAME}.npz"
test -s "$MOTION"
export WANDB_MODE=disabled
"$PYTHON" scripts/train.py Unitree-G1-23Dof-Tracking-No-State-Estimation \
  --motion-file "$MOTION" \
  --env.scene.num-envs "$ENVS" \
  --agent.max-iterations "$ITERATIONS" \
  --agent.save-interval 5 \
  --agent.logger tensorboard \
  --agent.run-name zed_upper_body_smoke
