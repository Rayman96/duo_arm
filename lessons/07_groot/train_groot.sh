#!/usr/bin/env bash
# Fine-tune GR00T N1.7-3B on the lesson-05 dataset.  ~75 min / 19.7 GB GPU for 15k steps.
#   lessons/07_groot/train_groot.sh [steps=15000] [run name=groot_pick_red] [extra lerobot-train args...]
#
# Needs: Hugging Face login with access to nvidia/Cosmos-Reason2-2B (see tools/setup_env.sh).
# --policy.tune_diffusion_model=false: the default recipe trains 1.6B parameters and runs out of
# memory on a 32 GB GPU even alone; on a >= 40 GB GPU drop this flag and raise --batch_size (32-64).
# Relative actions (GR00T's convention) for the arm joints; grippers and base velocities stay absolute.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
STEPS="${1:-15000}"; RUN="${2:-groot_pick_red}"; shift $(( $# < 2 ? $# : 2 ))
mkdir -p outputs/train
rm -rf "outputs/train/$RUN"
PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True .venv-lerobot/bin/lerobot-train \
  --dataset.repo_id=local/alohamini2_sim_pick_red --dataset.root=outputs/datasets/alohamini2_sim_pick_red \
  --dataset.video_backend=pyav \
  --policy.type=groot --policy.device=cuda --policy.base_model_path=nvidia/GR00T-N1.7-3B \
  --policy.embodiment_tag=new_embodiment --policy.chunk_size=16 --policy.n_action_steps=16 \
  --policy.use_relative_actions=true --policy.relative_exclude_joints='["gripper","vel"]' \
  --policy.use_bf16=true --policy.tune_diffusion_model=false --policy.push_to_hub=false \
  --output_dir="outputs/train/$RUN" --steps="$STEPS" --batch_size=8 --save_freq=5000 --log_freq=250 \
  --use_policy_training_preset=true --wandb.enable=false --num_workers=4 "$@" 2>&1 \
  | tee "outputs/train/$RUN.log" | grep --line-buffered -E "step:|End of training|Error"
