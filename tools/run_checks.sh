#!/usr/bin/env bash
# Rebuild the digital twin and re-run its checks (lessons 01, 02, 04), ~6 min.
# Logs: outputs/<lesson output dir>/*.log
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
python3 lessons/01_urdf_to_usd/fix_urdf.py alohamini2

run() {  # run <log> <script> [args...]
  local log="$1"; shift
  mkdir -p "$(dirname "$log")"
  tools/run_isaac.sh "$@" > "$log" 2>&1
  grep -hE "^\[T0[0-9]" "$log" || { echo "FAILED: $* (see $log)"; exit 1; }
}
run outputs/usd/import.log lessons/01_urdf_to_usd/import_urdf.py
run outputs/twin_check/physics.log lessons/02_twin_check/check_physics.py
run outputs/twin_check/cameras_lift0.log lessons/02_twin_check/check_cameras.py 0.0
run outputs/twin_check/cameras_lift0.3.log lessons/02_twin_check/check_cameras.py 0.3
run outputs/grasp/grasp.log lessons/04_ik_grasp/grasp_ik.py
