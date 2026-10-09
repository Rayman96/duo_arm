#!/usr/bin/env bash
# Closed-loop test of a trained ACT checkpoint in Isaac Sim (lesson 12: "验收 ACT").
#   lessons/06_act/eval_act.sh [episodes=20] [checkpoint step=040000] [run=act_pick_red]
#   EVAL_YAW=30 lessons/06_act/eval_act.sh      # cubes also rotated +-30 deg (never seen in the demos)
# Extra args after the first three go to tools/serve_policy.py, e.g.
#   EVAL_NAME=my_test <script> 20 <step> <run> --n-action-steps 8
# Results: outputs/eval/act_<step>[_yaw30].json + contact sheets.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EPISODES="${1:-20}"; STEP="${2:-040000}"; RUN="${3:-act_pick_red}"; shift $(( $# < 3 ? $# : 3 ))  # remaining args go to tools/serve_policy.py
NAME="act_$((10#$STEP / 1000))k"; [[ "${EVAL_YAW:-0}" != 0 ]] && NAME="${NAME}_yaw${EVAL_YAW}"
NAME="${EVAL_NAME:-$NAME}"  # set EVAL_NAME when passing custom server args, so standard results are kept
"$ROOT/tools/run_eval.sh" "$NAME" "$ROOT/outputs/train/$RUN/checkpoints/$STEP/pretrained_model" "$EPISODES" "$@"
