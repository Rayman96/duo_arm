#!/usr/bin/env bash
# AlohaMini2 interactive simulation (Isaac Sim 6.1 in docker).
#
#   lessons/03_sim_teleop/run_gui.sh            open the window on this computer's desktop
#   lessons/03_sim_teleop/run_gui.sh stop       close it
#   docker logs -f duo-arm-sim  follow the log
#
# Run it from a terminal on the desktop session (it uses $DISPLAY, default :0).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
HERE="lessons/03_sim_teleop"
IMG=nvcr.io/nvidia/isaac-sim:6.1.0
NAME=duo-arm-sim
STATE="$ROOT/.run"
mkdir -p "$STATE" && chmod 700 "$STATE"
MODE="${1:-start}"

if [[ "$MODE" == "stop" ]]; then
  docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped $NAME" || echo "$NAME not running"
  exit 0
fi
if [[ "$MODE" != "start" ]]; then
  echo "usage: $0 [stop]"; exit 1
fi
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "already running ($0 stop to close it)"; exit 0
fi
docker rm -f "$NAME" >/dev/null 2>&1 || true  # previous exited run (kept for its log)

"$ROOT/tools/ensure_usd.sh"

VOLS=(-v duo_arm_isaac_cache:/isaac-sim/.cache
      -v duo_arm_isaac_compute_cache:/isaac-sim/.nv/ComputeCache
      -v duo_arm_isaac_kit_cache:/isaac-sim/kit/cache
      -v duo_arm_isaac_logs:/isaac-sim/.nvidia-omniverse/logs)
ENVS=(-e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e NVIDIA_DRIVER_CAPABILITIES=all -e SIM_AUTODEMO="${SIM_AUTODEMO:-}"
      -e SIM_POLICY_AUTOPLAY="${SIM_POLICY_AUTOPLAY:-}")

# The container runs as uid 1234, which cannot read the desktop's X cookie (mode 600).
# Copy it into a private directory and make only the copy readable to the container.
DISP="${DISPLAY:-:0}"
XAUTH=""
for f in "${XAUTHORITY:-}" "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
  [[ -n "$f" && -r "$f" ]] && { XAUTH="$f"; break; }
done
if [[ -z "$XAUTH" ]]; then
  echo "no X cookie found: run this from a terminal on the desktop, or run 'xhost +local:' first"; exit 1
fi
cp "$XAUTH" "$STATE/Xauthority" && chmod 644 "$STATE/Xauthority"
ENVS+=(-e DISPLAY="$DISP" -e XAUTHORITY=/tmp/.Xauthority)
VOLS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$STATE/Xauthority":/tmp/.Xauthority:ro)

docker run -d --name "$NAME" --no-healthcheck --gpus all --shm-size 2g --network host \
  "${ENVS[@]}" "${VOLS[@]}" -v "$ROOT":/workspace -w /workspace \
  --entrypoint /isaac-sim/python.sh "$IMG" "$HERE/sim_app.py" >/dev/null

echo "started '$NAME'. The Isaac Sim window opens on display $DISP in about a minute."
echo "log: docker logs -f $NAME"
