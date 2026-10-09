#!/usr/bin/env bash
# Evaluate a trained LeRobot checkpoint in Isaac Sim (left arm picks up the red cube).
#
#   tools/run_eval.sh <name> <checkpoint>/pretrained_model [episodes=20] [extra serve_policy args...]
#   EVAL_YAW=30 tools/run_eval.sh ...   # also rotate the cube by a random yaw in [-30, 30] deg
#
# Starts tools/serve_policy.py, runs tools/eval_in_sim.py in the Isaac Sim container, stops the server.
# Results: outputs/eval/<name>.json (+ contact sheets, eval_<name>.log, <name>_serve.log)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NAME="$1"; CKPT="$2"; EPISODES="${3:-20}"; shift $(( $# < 3 ? $# : 3 ))
PORT="${EVAL_PORT:-5600}"
SEED="${EVAL_SEED:-1000}"
YAW="${EVAL_YAW:-0}"
EVAL="$ROOT/outputs/eval"
mkdir -p "$EVAL"
LOG="$EVAL/${NAME}_serve.log"

"$ROOT/.venv-lerobot/bin/python" -u "$ROOT/tools/serve_policy.py" "$CKPT" --port "$PORT" "$@" > "$LOG" 2>&1 &
SERVER=$!
trap 'kill $SERVER 2>/dev/null || true' EXIT
until grep -q "listening" "$LOG"; do
  kill -0 $SERVER 2>/dev/null || { echo "policy server failed, see $LOG"; tail -5 "$LOG"; exit 1; }
  sleep 2
done
cd "$ROOT"
tools/run_isaac.sh tools/eval_in_sim.py "$NAME" "$EPISODES" "$SEED" "$PORT" "$YAW" > "$EVAL/eval_${NAME}.log" 2>&1 || true
grep -E "^\[EVAL\]" "$EVAL/eval_${NAME}.log" || { echo "eval failed, see outputs/eval/eval_${NAME}.log"; exit 1; }
