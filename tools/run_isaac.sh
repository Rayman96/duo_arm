#!/usr/bin/env bash
# Run an Isaac Sim 6.1 python script headless in its own container.
#   tools/run_isaac.sh lessons/01_urdf_to_usd/import_urdf.py [args...]   (paths relative to the repo root)
# Mounts the repo at /workspace and uses its own duo_arm_* cache volumes. Host networking lets
# scripts reach services on this machine (tools/serve_policy.py).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMG=nvcr.io/nvidia/isaac-sim:6.1.0
VOLS=(-v duo_arm_isaac_cache:/isaac-sim/.cache
      -v duo_arm_isaac_compute_cache:/isaac-sim/.nv/ComputeCache
      -v duo_arm_isaac_kit_cache:/isaac-sim/kit/cache
      -v duo_arm_isaac_logs:/isaac-sim/.nvidia-omniverse/logs)
if ! docker volume inspect duo_arm_isaac_cache >/dev/null 2>&1; then
  docker run --rm -u 0 --entrypoint bash "${VOLS[@]}" "$IMG" -lc \
    'chown -R 1234:1234 /isaac-sim/.cache /isaac-sim/.nv/ComputeCache /isaac-sim/kit/cache /isaac-sim/.nvidia-omniverse/logs'
fi
mkdir -p "$ROOT/outputs" && chmod a+rwx "$ROOT/outputs"
script="$1"; shift
# The container runs as uid 1234: hand what it wrote in outputs/ back to the host user afterwards
# (and keep it writable for the container on the next run).
trap 'docker run --rm -u 0 -v "$ROOT/outputs":/out --entrypoint bash "$IMG" -c "chown -R $(id -u):$(id -g) /out && chmod -R a+rwX /out"' EXIT
docker run --rm --gpus all --shm-size 2g --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e NVIDIA_DRIVER_CAPABILITIES=all \
  "${VOLS[@]}" -v "$ROOT":/workspace -w /workspace \
  --entrypoint /isaac-sim/python.sh "$IMG" "$script" "$@"
