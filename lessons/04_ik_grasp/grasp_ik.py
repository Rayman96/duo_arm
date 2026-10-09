"""T06: Dynamic bimanual grasp: IK above each cube -> open -> descend -> close -> raise the lift.

Pass = the left and right cubes each end > 5 cm above their start.
Writes outputs/grasp/grasp.json and outputs/grasp/images/grid.png (4 frames of the sequence).
Run: tools/run_isaac.sh lessons/04_ik_grasp/grasp_ik.py
"""

import json
import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 640, "height": 480})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
from alohamini_ik import GRIPPER_CLOSED, GRIPPER_OPEN, plan_bimanual_pick  # noqa: E402
from alohamini_scene import add_table_and_cubes, add_view_camera, new_stage, reset_cubes, start_robot  # noqa: E402
from paths import GRASP_DIR as OUT  # noqa: E402

DT = 1 / 200
os.makedirs(f"{OUT}/images", exist_ok=True)

stage = new_stage(DT)
cubes = {name: RigidPrim(path) for name, path in add_table_and_cubes(stage).items()}
view = add_view_camera(stage, "/World/view", (1.0, -1.1, 1.05), (0, -0.35, 0.65), aspect=4 / 3)
robot = start_robot(stage)

plan, ok = plan_bimanual_pick(robot)  # kinematic: teleports the arms, so reset the cubes afterwards
print(f"[T06] IK converged: {ok}", flush=True)
reset_cubes(cubes)
SimulationManager.step(steps=50)
start_z = {s: float(cubes[s].get_world_poses()[0].numpy()[0][2]) for s in ("left", "right")}

ann = rep.AnnotatorRegistry.get_annotator("rgb")
ann.attach([rep.create.render_product(view, (640, 480))])
frames = []


def snap(label):
    for _ in range(4):
        simulation_app.update()
    rep.orchestrator.step(rt_subframes=4)
    frames.append((label, Image.fromarray(np.asarray(ann.get_data())[..., :3])))


def move(goal, seconds, hold=0.0):
    start = robot.art.get_dof_position_targets().numpy()[0].copy()
    steps = int(seconds / DT)
    for k in range(steps):
        robot.set_targets(start + (goal - start) * (0.5 - 0.5 * np.cos(np.pi * (k + 1) / steps)))
        SimulationManager.step(steps=1)
    SimulationManager.step(steps=int(hold / DT))


def with_gripper(q, g, lift=0.0):
    q = q.copy()
    q[robot.grippers] = g
    q[robot.lift] = lift
    return q


move(with_gripper(plan["pre"], GRIPPER_OPEN), 2.5); snap("1 approach")
move(with_gripper(plan["grasp"], GRIPPER_OPEN), 1.5); snap("2 descend")
move(with_gripper(plan["grasp"], GRIPPER_CLOSED), 1.0, hold=0.5); snap("3 close")
move(with_gripper(plan["grasp"], GRIPPER_CLOSED, lift=0.12), 2.5, hold=0.5); snap("4 lift +12 cm")

res = {}
for s, z0 in start_z.items():
    z = float(cubes[s].get_world_poses()[0].numpy()[0][2])
    res[s] = {"start_z": round(z0, 3), "end_z": round(z, 3), "lifted": z - z0 > 0.05}
res["gripper_q"] = dict(zip(("left", "right"), robot.q()[robot.grippers].round(3).tolist()))
print(f"[T06] {res}", flush=True)
json.dump(res, open(f"{OUT}/grasp.json", "w"), indent=2)
grid = Image.new("RGB", (1280, 960))
for k, (label, im) in enumerate(frames):
    grid.paste(im, ((k % 2) * 640, (k // 2) * 480))
    ImageDraw.Draw(grid).text(((k % 2) * 640 + 8, (k // 2) * 480 + 8), label, fill=(255, 0, 0))
grid.save(f"{OUT}/images/grid.png")
simulation_app.close()
