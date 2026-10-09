#!/usr/bin/env bash
# AlohaMini2 interactive simulation (Isaac Sim 6.1 in docker).
#
#   lessons/03_sim_teleop/run_gui.sh            window on this server's desktop (display :0 / NICE DCV console)
#   lessons/03_sim_teleop/run_gui.sh stream     headless + WebRTC livestream; connect the Isaac Sim WebRTC
#                               Streaming Client on your own computer to this server's IP
#   lessons/03_sim_teleop/run_gui.sh stop       close it
#   docker logs -f duo-arm-sim  follow the log
#
# Stream mode needs WebRTC ports 49100/tcp (signaling) and 47998/udp (media) free on this host.
# Override the advertised IP with SIM_PUBLIC_IP=...
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
HERE="lessons/03_sim_teleop"
IMG=nvcr.io/nvidia/isaac-sim:6.1.0
NAME=duo-arm-sim
STATE="$ROOT/.run"
mkdir -p "$STATE" && chmod 700 "$STATE"
MODE="${1:-desktop}"

if [[ "$MODE" == "stop" ]]; then
  docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped $NAME" || echo "$NAME not running"
  exit 0
fi
if [[ "$MODE" != "desktop" && "$MODE" != "stream" ]]; then
  echo "usage: $0 [desktop|stream|stop]"; exit 1
fi
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "already running ($0 stop to close it)"; exit 0
fi
docker rm -f "$NAME" >/dev/null 2>&1 || true  # previous exited run (kept for its log)

# The corrected digital twin; build it if the feasibility tests have not been run.
if [[ ! -f "$ROOT/outputs/usd/sim_alohamini2/alohamini2_sim/alohamini2_sim.usda" ]]; then
  echo "building AlohaMini2 USD..."
  python3 "$ROOT/lessons/01_urdf_to_usd/fix_urdf.py" alohamini2
  "$ROOT/tools/run_isaac.sh" lessons/01_urdf_to_usd/import_urdf.py
fi

VOLS=(-v duo_arm_isaac_cache:/isaac-sim/.cache
      -v duo_arm_isaac_compute_cache:/isaac-sim/.nv/ComputeCache
      -v duo_arm_isaac_kit_cache:/isaac-sim/kit/cache
      -v duo_arm_isaac_logs:/isaac-sim/.nvidia-omniverse/logs)
ENVS=(-e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e NVIDIA_DRIVER_CAPABILITIES=all -e SIM_AUTODEMO="${SIM_AUTODEMO:-}"
      -e SIM_POLICY_AUTOPLAY="${SIM_POLICY_AUTOPLAY:-}")

if [[ "$MODE" == "stream" ]]; then
  PUBLIC_IP="${SIM_PUBLIC_IP:-$(hostname -I | awk '{print $1}')}"
  ENVS+=(-e SIM_STREAM=1 -e SIM_PUBLIC_IP="$PUBLIC_IP")
else
  # The container runs as uid 1234, which cannot read the desktop's X cookie (mode 600).
  # Copy it into a private directory and make only the copy readable to the container.
  cp "${XAUTHORITY_SRC:-/run/user/$(id -u)/gdm/Xauthority}" "$STATE/Xauthority" && chmod 644 "$STATE/Xauthority"
  ENVS+=(-e DISPLAY="${SIM_DISPLAY:-:0}" -e XAUTHORITY=/tmp/.Xauthority)
  VOLS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$STATE/Xauthority":/tmp/.Xauthority:ro)
fi

docker run -d --name "$NAME" --no-healthcheck --gpus all --shm-size 2g --network host \
  "${ENVS[@]}" "${VOLS[@]}" -v "$ROOT":/workspace -w /workspace \
  --entrypoint /isaac-sim/python.sh "$IMG" "$HERE/sim_app.py" >/dev/null

if [[ "$MODE" == "stream" ]]; then
  echo "started '$NAME' (stream). In about a minute, connect the Isaac Sim WebRTC Streaming Client to: $PUBLIC_IP"
  echo "ports: 49100/tcp signaling, 47998/udp media"
else
  echo "started '$NAME'. The Isaac Sim window opens on display ${SIM_DISPLAY:-:0} in about a minute."
fi
echo "log: docker logs -f $NAME"
