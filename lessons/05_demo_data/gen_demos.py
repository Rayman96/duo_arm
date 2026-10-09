"""T07: Scripted demonstrations for imitation learning: left arm picks up the red cube.

Per episode: randomize the red cube's position, plan the left arm with IK, then run
approach -> descend -> close -> raise the lift, recording at 30 Hz:
  state  (18): AlohaMini feature order (left arm 7, right arm 7, x.vel, y.vel, theta.vel, lift mm)
  action (18): commanded targets in the same order
  images     : head camera ("forward", top-down) and left wrist camera, 320x240 RGB
Only episodes where the cube ends > 5 cm above its start are kept.

Writes outputs/demos/ep_XXXX.npz (+ summary.json).
Run: tools/run_isaac.sh lessons/05_demo_data/gen_demos.py [num_episodes] [seed] [out_dir]
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

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
from alohamini_ik import GRIPPER_CLOSED, GRIPPER_OPEN, ArmIK  # noqa: E402
from alohamini_scene import (CUBE, TABLE_TOP, add_onboard_cameras, add_table_and_cubes,  # noqa: E402
                             new_stage, start_robot)
from paths import DEMOS_DIR  # noqa: E402
from policy_io import FPS, IMG_H, IMG_W, POLICY_CAMERAS, RED_X, RED_Y, to_features  # noqa: E402

N_EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 50
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 0
OUT = sys.argv[3] if len(sys.argv) > 3 else str(DEMOS_DIR)
PHYSICS_DT = 1.0 / 120.0  # app.update() advances 1/60 s; + 2 manual steps = one 1/30 s frame
os.makedirs(OUT, exist_ok=True)

stage = new_stage(PHYSICS_DT)
cubes = {name: RigidPrim(path) for name, path in add_table_and_cubes(stage).items()}
robot = start_robot(stage)
cams = add_onboard_cameras(stage)
annotators = {}
for name, sim_cam in POLICY_CAMERAS.items():
    ann = rep.AnnotatorRegistry.get_annotator("rgb")
    ann.attach([rep.create.render_product(cams[sim_cam], (IMG_W, IMG_H))])
    annotators[name] = ann
ik = ArmIK(robot.art, "left")
for _ in range(20):  # warm up the renderer
    simulation_app.update()


def frame_step():
    """Advance 1/30 s and render once."""
    SimulationManager.step(steps=2)
    simulation_app.update()


def place_red(x, y):
    cubes["left"].set_world_poses(positions=[[x, y, TABLE_TOP + CUBE / 2]], orientations=[[1, 0, 0, 0]])
    cubes["left"].set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])


def segments(plan_pre, plan_grasp):
    """(goal, seconds) list for the scripted pick; goals are full joint target vectors."""
    def tgt(q, grip, lift=0.0):
        g = q.copy()
        g[robot.idx["left_gripper"]] = grip
        g[robot.lift] = lift
        return g
    home = np.zeros(robot.n)
    return [(tgt(home, GRIPPER_OPEN), 0.6), (tgt(plan_pre, GRIPPER_OPEN), 2.0),
            (tgt(plan_grasp, GRIPPER_OPEN), 1.3), (tgt(plan_grasp, GRIPPER_CLOSED), 0.8),
            (tgt(plan_grasp, GRIPPER_CLOSED), 0.4), (tgt(plan_grasp, GRIPPER_CLOSED, lift=0.10), 1.8),
            (tgt(plan_grasp, GRIPPER_CLOSED, lift=0.10), 0.6)]


# Solve once at the centre of the randomization box and seed every episode from it: better
# convergence at the edges and consistent arm postures across demos (easier to imitate).
z_grasp = TABLE_TOP + 0.028
center = [np.mean(RED_X), np.mean(RED_Y)]
seed_pre, _, _ = ik.solve(np.zeros(robot.n), [*center, z_grasp + 0.08])
seed_grasp, _, _ = ik.solve(seed_pre, [*center, z_grasp])
rng = np.random.default_rng(SEED)
kept, tried, t_start = 0, 0, time.time()
summary = []
while kept < N_EPISODES and tried < 2 * N_EPISODES:
    tried += 1
    x, y = rng.uniform(*RED_X), rng.uniform(*RED_Y)
    # plan (teleports the arm), then reset robot + cube
    q_pre, ok1, _ = ik.solve(seed_pre, [x, y, z_grasp + 0.08])
    q_grasp, ok2, _ = ik.solve(seed_grasp, [x, y, z_grasp])
    robot.teleport(np.zeros(robot.n))
    place_red(x, y)
    for _ in range(10):
        frame_step()
    if not (ok1 and ok2):
        print(f"[T07] try {tried}: IK failed for cube at ({x:.3f}, {y:.3f})", flush=True)
        continue
    z0 = float(cubes["left"].get_world_poses()[0].numpy()[0][2])
    states, actions, images = [], [], {c: [] for c in POLICY_CAMERAS}
    target = robot.q()
    for goal, seconds in segments(q_pre, q_grasp):
        start, steps = target.copy(), int(round(seconds * FPS))
        for k in range(steps):
            target = start + (goal - start) * (0.5 - 0.5 * np.cos(np.pi * (k + 1) / steps))
            # record the observation, then apply the action that follows it
            states.append(to_features(robot.q(), robot.idx))
            for c in POLICY_CAMERAS:
                images[c].append(np.asarray(annotators[c].get_data())[..., :3].copy())
            actions.append(to_features(target, robot.idx))
            robot.set_targets(target)
            frame_step()
    z1 = float(cubes["left"].get_world_poses()[0].numpy()[0][2])
    success = z1 - z0 > 0.05
    summary.append({"try": tried, "cube_xy": [round(x, 4), round(y, 4)], "lift_m": round(z1 - z0, 4),
                    "success": success, "frames": len(states)})
    if success:
        np.savez_compressed(f"{OUT}/ep_{kept:04d}.npz", state=np.array(states, np.float32),
                            action=np.array(actions, np.float32), cube_xy=np.array([x, y], np.float32),
                            **{f"image_{c}": np.array(v, np.uint8) for c, v in images.items()})
        kept += 1
    rate = (time.time() - t_start) / tried
    print(f"[T07] try {tried}: cube ({x:.3f}, {y:.3f}) lift {z1 - z0:+.3f} m success={success} "
          f"kept {kept}/{N_EPISODES} ({rate:.0f} s/episode)", flush=True)

json.dump({"kept": kept, "tried": tried, "episodes": summary}, open(f"{OUT}/summary.json", "w"), indent=2)
print(f"[T07] done: {kept} demos from {tried} tries", flush=True)
simulation_app.close()
