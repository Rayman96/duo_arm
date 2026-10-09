#!/usr/bin/env bash
# One-time setup of the host side (training / inference). The Isaac Sim side needs only docker + the
# nvcr.io/nvidia/isaac-sim:6.1.0 image (tools/run_isaac.sh).
#
#   tools/setup_env.sh
# Creates .venv-lerobot/ (Python 3.12, PyTorch + CUDA, LeRobot from third_party/lerobot_alohamini
# with the [training,groot] extras) and clones the upstream repos into third_party/ if missing.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p third_party
for repo in AlohaMini lerobot_alohamini AM-ARM; do
  [[ -d "third_party/$repo" ]] || git clone --depth 1 "https://github.com/liyiteng/$repo.git" "third_party/$repo"
done
command -v uv >/dev/null || [[ -x ~/.local/bin/uv ]] || pip install --user uv
UV="$(command -v uv || echo ~/.local/bin/uv)"
[[ -d .venv-lerobot ]] || "$UV" venv -p 3.12 .venv-lerobot
"$UV" pip install -p .venv-lerobot/bin/python -e "third_party/lerobot_alohamini[training,groot]"
.venv-lerobot/bin/python -c "import torch, lerobot; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
echo "GR00T also needs: accept the license of nvidia/Cosmos-Reason2-2B on Hugging Face, then"
echo "  .venv-lerobot/bin/hf auth login"
