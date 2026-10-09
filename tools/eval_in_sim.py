"""Closed-loop evaluation of a served policy in Isaac Sim: left arm picks up the red cube.

Start tools/serve_policy.py first, then (from the repo root):
  tools/run_isaac.sh tools/eval_in_sim.py <name> [episodes] [seed] [port] [yaw_deg]
Usually run through tools/run_eval.sh, which does both.

Per episode the red cube is placed at a random position (seed != the training demos' seed) and,
if yaw_deg > 0, rotated by a random yaw in [-yaw_deg, yaw_deg] (the demos only had yaw 0);
the policy acts at 30 Hz for up to 12 s, and success = cube > 5 cm above its start.
Writes outputs/eval/<name>.json and <name>_ep<k>.png contact sheets (first 4 episodes).
"""

import json
import os
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from alohamini_scene import (CUBE, TABLE_TOP, add_onboard_cameras, add_table_and_cubes,  # noqa: E402
                             new_stage, start_robot)
from policy_io import (FPS, IMG_H, IMG_W, POLICY_CAMERAS, RED_X, RED_Y, apply_features, connect,  # noqa: E402
                       recv_msg, send_msg, to_features)
from paths import EVAL_DIR as OUT  # noqa: E402

NAME = sys.argv[1] if len(sys.argv) > 1 else "policy"
N_EPISODES = int(sys.argv[2]) if len(sys.argv) > 2 else 20
SEED = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
PORT = int(sys.argv[4]) if len(sys.argv) > 4 else 5600
YAW_MAX = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
MAX_SECONDS = 12.0
os.makedirs(OUT, exist_ok=True)

stage = new_stage(1.0 / 120.0)
cubes = {name: RigidPrim(path) for name, path in add_table_and_cubes(stage).items()}
robot = start_robot(stage)
cams = add_onboard_cameras(stage)
annotators = {}
for name, sim_cam in POLICY_CAMERAS.items():
    ann = rep.AnnotatorRegistry.get_annotator("rgb")
    ann.attach([rep.create.render_product(cams[sim_cam], (IMG_W, IMG_H))])
    annotators[name] = ann
for _ in range(20):
    simulation_app.update()


def frame_step():
    SimulationManager.step(steps=2)
    simulation_app.update()


sock = connect(port=PORT)
rng = np.random.default_rng(SEED)
results, t_start = [], time.time()
for ep in range(N_EPISODES):
    x, y = rng.uniform(*RED_X), rng.uniform(*RED_Y)
    yaw = rng.uniform(-YAW_MAX, YAW_MAX) if YAW_MAX > 0 else 0.0  # no extra draw: same positions as yaw 0
    robot.teleport(np.zeros(robot.n))
    half = np.radians(yaw) / 2
    cubes["left"].set_world_poses(positions=[[x, y, TABLE_TOP + CUBE / 2]],
                                  orientations=[[np.cos(half), 0, 0, np.sin(half)]])
    cubes["left"].set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
    for _ in range(10):
        frame_step()
    z0 = float(cubes["left"].get_world_poses()[0].numpy()[0][2])
    send_msg(sock, {"cmd": "reset"})
    recv_msg(sock)
    target, max_lift, frames, t_success = robot.q(), 0.0, [], None
    for k in range(int(MAX_SECONDS * FPS)):
        images = {c: np.asarray(a.get_data())[..., :3].copy() for c, a in annotators.items()}
        send_msg(sock, {"cmd": "act"}, {"state": to_features(robot.q(), robot.idx), **images})
        _, reply = recv_msg(sock)
        target = apply_features(reply["action"], target, robot.idx)
        robot.set_targets(target)
        frame_step()
        lift = float(cubes["left"].get_world_poses()[0].numpy()[0][2]) - z0
        max_lift = max(max_lift, lift)
        if ep < 4 and k % 30 == 0:
            frames.append(images["forward"])
        if t_success is None and lift > 0.05:
            t_success = k / FPS
        if t_success is not None and k / FPS > t_success + 1.0:  # held for 1 s: done
            break
    success = t_success is not None and lift > 0.05
    results.append({"episode": ep, "cube_xy": [round(x, 4), round(y, 4)], "yaw_deg": round(yaw, 1), "success": success,
                    "final_lift_m": round(lift, 4), "max_lift_m": round(max_lift, 4), "time_to_lift_s": t_success})
    if frames:
        sheet = Image.new("RGB", (IMG_W * 4, IMG_H * ((len(frames) + 3) // 4)))
        for i, f in enumerate(frames):
            sheet.paste(Image.fromarray(f), ((i % 4) * IMG_W, (i // 4) * IMG_H))
        sheet.save(f"{OUT}/{NAME}_ep{ep}.png")
    n_ok = sum(r["success"] for r in results)
    print(f"[EVAL] {NAME} ep {ep}: cube ({x:.3f}, {y:.3f}, {yaw:+.0f} deg) success={success} lift {lift:+.3f} m "
          f"(max {max_lift:+.3f}) | {n_ok}/{ep + 1} | {(time.time() - t_start) / (ep + 1):.0f} s/ep", flush=True)

rate = sum(r["success"] for r in results) / len(results)
json.dump({"name": NAME, "seed": SEED, "yaw_max_deg": YAW_MAX, "episodes": len(results), "success_rate": rate,
           "results": results},
          open(f"{OUT}/{NAME}.json", "w"), indent=2)
print(f"[EVAL] {NAME}: success rate {rate:.0%} over {len(results)} episodes", flush=True)
sock.close()
simulation_app.close()
