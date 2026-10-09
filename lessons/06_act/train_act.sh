#!/usr/bin/env bash
# Train ACT on the lesson-05 dataset.  ~40 min / 1.5 GB GPU for 40k steps on an RTX PRO 4500.
#   lessons/06_act/train_act.sh [steps=40000] [run name=act_pick_red] [extra lerobot-train args...]
# e.g. lessons/06_act/train_act.sh 20000 act_chunk50 --policy.chunk_size=50 --policy.n_action_steps=50
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
STEPS="${1:-40000}"; RUN="${2:-act_pick_red}"; shift $(( $# < 2 ? $# : 2 ))
mkdir -p outputs/train
rm -rf "outputs/train/$RUN"
# --dataset.video_backend=pyav: this server has no system FFmpeg for the default torchcodec decoder.
.venv-lerobot/bin/lerobot-train \
  --dataset.repo_id=local/alohamini2_sim_pick_red --dataset.root=outputs/datasets/alohamini2_sim_pick_red \
  --dataset.video_backend=pyav \
  --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --output_dir="outputs/train/$RUN" --steps="$STEPS" --batch_size=8 --save_freq=10000 --log_freq=500 \
  --wandb.enable=false --num_workers=4 "$@" 2>&1 | tee "outputs/train/$RUN.log" | grep --line-buffered -E "step:|End of training|Error"
