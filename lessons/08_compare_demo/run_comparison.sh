#!/usr/bin/env bash
# Compare trained policies: latency benchmark + closed-loop evals (standard and cube yaw +-30 deg)
# + summary table. Run with the GPU otherwise idle (no training running).
#
#   lessons/08_compare_demo/run_comparison.sh   # existing eval results are reused; FORCE=1 re-runs them
# Output: outputs/eval/comparison.md
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
HERE=lessons/08_compare_demo
cd "$ROOT"
# name | training run (outputs/train/<run>) | checkpoint step
MODELS=("act_40k act_pick_red 040000" "groot_15k groot_pick_red 015000")

bench=()
for m in "${MODELS[@]}"; do
  read -r name run step <<< "$m"
  bench+=("$name=outputs/train/$run/checkpoints/$step/pretrained_model")
done
.venv-lerobot/bin/python "$HERE/bench_latency.py" "${bench[@]}" 2>&1 | grep "^{"

for m in "${MODELS[@]}"; do
  read -r name run step <<< "$m"
  ckpt="outputs/train/$run/checkpoints/$step/pretrained_model"
  for yaw in 0 30; do
    eval_name="$name"; [[ $yaw != 0 ]] && eval_name="${name}_yaw${yaw}"
    if [[ -f "outputs/eval/$eval_name.json" && -z "${FORCE:-}" ]]; then
      echo "reuse $eval_name"; continue
    fi
    EVAL_YAW=$yaw tools/run_eval.sh "$eval_name" "$ckpt" 20 | tail -1
  done
done

args=()
for m in "${MODELS[@]}"; do read -r name run step <<< "$m"; args+=("$run:$name"); done
python3 "$HERE/summarize.py" "${args[@]}" | tee outputs/eval/comparison.md
