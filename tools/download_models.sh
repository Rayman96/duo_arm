#!/usr/bin/env bash
# Download the trained policies into outputs/train/ so lessons 06-08 can evaluate and demo them
# without training first. The Hugging Face repo is private: log in with an account that has access
# (.venv-lerobot/bin/hf auth login).
#   tools/download_models.sh          ACT (10k + 40k, 0.4 GB) and GR00T (15k, 12 GB)
#   tools/download_models.sh act      ACT only
#   tools/download_models.sh groot    GR00T only
# Override the repo with MODELS_REPO=<user>/<repo>.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REPO="${MODELS_REPO:-ray9658/alohamini2-sim-pick-red}"
case "${1:-all}" in
  act)   INCLUDE=("act_pick_red/*") ;;
  groot) INCLUDE=("groot_pick_red/*") ;;
  all)   INCLUDE=("act_pick_red/*" "groot_pick_red/*") ;;
  *) echo "usage: $0 [act|groot|all]"; exit 1 ;;
esac
"$ROOT/.venv-lerobot/bin/hf" download "$REPO" --repo-type model --include "${INCLUDE[@]}" \
  --local-dir "$ROOT/outputs/train"
find "$ROOT/outputs/train" -path "*/checkpoints/*/pretrained_model" -type d | sort
