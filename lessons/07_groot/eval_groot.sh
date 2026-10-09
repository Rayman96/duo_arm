#!/usr/bin/env bash
# Closed-loop test of a fine-tuned GR00T checkpoint in Isaac Sim.
#   lessons/07_groot/eval_groot.sh [episodes=20] [checkpoint step=015000] [run=groot_pick_red]
#   EVAL_YAW=30 lessons/07_groot/eval_groot.sh   # cubes also rotated +-30 deg
# Extra args after the first three go to tools/serve_policy.py, e.g.
#   EVAL_NAME=my_test <script> 20 <step> <run> --n-action-steps 8
# Needs ~13 GB GPU for the policy server: don't run while GR00T is training.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EPISODES="${1:-20}"; STEP="${2:-015000}"; RUN="${3:-groot_pick_red}"; shift $(( $# < 3 ? $# : 3 ))  # remaining args go to tools/serve_policy.py
NAME="groot_$((10#$STEP / 1000))k"; [[ "${EVAL_YAW:-0}" != 0 ]] && NAME="${NAME}_yaw${EVAL_YAW}"
NAME="${EVAL_NAME:-$NAME}"  # set EVAL_NAME when passing custom server args, so standard results are kept
"$ROOT/tools/run_eval.sh" "$NAME" "$ROOT/outputs/train/$RUN/checkpoints/$STEP/pretrained_model" "$EPISODES" "$@"
