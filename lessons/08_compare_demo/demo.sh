#!/usr/bin/env bash
# Live demo: ACT and GR00T pick up the red cube in the sim window on this computer's screen.
#   lessons/08_compare_demo/demo.sh          start both policy servers + the sim window ("Policy demo" buttons)
#   SIM_POLICY_AUTOPLAY=1 lessons/08_compare_demo/demo.sh   auto-play: new cube -> ACT -> GR00T -> ...
#   lessons/08_compare_demo/demo.sh stop     close the sim window and stop the servers
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/outputs/eval"
ACT_CKPT="${ACT_CKPT:-$ROOT/outputs/train/act_pick_red/checkpoints/040000/pretrained_model}"
GROOT_CKPT="${GROOT_CKPT:-$ROOT/outputs/train/groot_pick_red/checkpoints/015000/pretrained_model}"
GUI="$ROOT/lessons/03_sim_teleop/run_gui.sh"

if [[ "${1:-}" == "stop" ]]; then
  "$GUI" stop
  for f in "$OUT"/demo_*.pid; do [[ -f "$f" ]] && kill "$(cat "$f")" 2>/dev/null; rm -f "$f"; done
  echo "policy servers stopped"; exit 0
fi

start_server() {  # start_server <name> <checkpoint> <port>
  mkdir -p "$OUT"
  local log="$OUT/demo_$1.log"
  if [[ -f "$OUT/demo_$1.pid" ]] && kill -0 "$(cat "$OUT/demo_$1.pid")" 2>/dev/null; then
    echo "$1 server already running"; return
  fi
  setsid "$ROOT/.venv-lerobot/bin/python" -u "$ROOT/tools/serve_policy.py" "$2" --port "$3" > "$log" 2>&1 < /dev/null &
  echo $! > "$OUT/demo_$1.pid"
  until grep -q "listening" "$log"; do
    kill -0 "$(cat "$OUT/demo_$1.pid")" 2>/dev/null || { echo "$1 server failed, see $log"; exit 1; }
    sleep 2
  done
  echo "$1 server ready on port $3"
}
start_server act "$ACT_CKPT" 5600
start_server groot "$GROOT_CKPT" 5601
"$GUI"
