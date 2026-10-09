#!/usr/bin/env bash
# Build the corrected AlohaMini2 digital twin (lesson 01) if it is not there yet.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ ! -f "$ROOT/outputs/usd/sim_alohamini2/alohamini2_sim/alohamini2_sim.usda" ]]; then
  echo "building AlohaMini2 USD (lesson 01, a few minutes)..."
  python3 "$ROOT/lessons/01_urdf_to_usd/fix_urdf.py" alohamini2
  (cd "$ROOT" && tools/run_isaac.sh lessons/01_urdf_to_usd/import_urdf.py > /dev/null)
fi
