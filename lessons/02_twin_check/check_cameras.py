"""T03: Geometry check + the 5 onboard cameras, rendered headless with RTX.

  * Link world bounding boxes at the home pose for a given lift height (ground contact check),
    with poses read from physics (the USD transforms are not updated while simulating).
  * The 5 onboard cameras (see alohamini_scene.CAMERAS) plus a third-person view of the
    standard table scene; a 2x2 summary image (third person, front, chest, left wrist).

Writes outputs/twin_check/images/lift<L>_<camera>.png, summary_lift<L>.png and geometry_lift<L>.json.
Run: tools/run_isaac.sh lessons/02_twin_check/check_cameras.py [lift]
"""

import json
import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
from alohamini_scene import (CAM_H, CAM_W, ROBOT_PATH, add_onboard_cameras, add_table_and_cubes,  # noqa: E402
                             add_view_camera, new_stage, start_robot)
from paths import TWIN_CHECK_DIR as OUT  # noqa: E402

LIFT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
TAG = f"lift{LIFT:+.1f}"
os.makedirs(f"{OUT}/images", exist_ok=True)

stage = new_stage(1 / 200)
add_table_and_cubes(stage)
robot = start_robot(stage)
cam_paths = {name: (p, (CAM_W, CAM_H)) for name, p in add_onboard_cameras(stage).items()}
cam_paths["third_person"] = (add_view_camera(stage, "/World/cam_third", (1.5, -1.7, 1.35), (0, -0.35, 0.6)),
                             (1280, 720))
q = np.zeros(robot.n)
q[robot.lift] = LIFT
robot.teleport(q)
SimulationManager.step(steps=2)
simulation_app.update()

# ---- geometry: link bounding boxes (poses from physics) ---------------------------
# Links are nested in the USD hierarchy, so take each link's own (non-body) children in its
# local frame, then place the 8 corners with the physics link pose.
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
local = {}
for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    rng = Gf.Range3d()
    for child in prim.GetChildren():
        if not (child.HasAPI(UsdPhysics.RigidBodyAPI) or child.IsA(UsdGeom.Camera)):
            r = bbox.ComputeRelativeBound(child, prim).ComputeAlignedRange()
            if not r.IsEmpty():
                rng.UnionWith(r)
    if not rng.IsEmpty():
        local[str(prim.GetPath())] = rng
pos, quat = (a.numpy() for a in RigidPrim(list(local)).get_world_poses())  # quat is (w, x, y, z)
geo = {"lift": LIFT, "links": {}}
for (path, rng), p, qt in zip(local.items(), pos, quat):
    lo, hi = np.array(rng.GetMin()), np.array(rng.GetMax())
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    rot = np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(*qt.tolist())))).T  # Gf is row-vector
    wc = corners @ rot.T + p
    geo["links"][path.split("/")[-1]] = {"origin": p.round(4).tolist(), "min": wc.min(0).round(4).tolist(),
                                         "max": wc.max(0).round(4).tolist()}
mins = np.array([v["min"] for v in geo["links"].values()])
maxs = np.array([v["max"] for v in geo["links"].values()])
geo["robot_bbox"] = {"min": mins.min(0).round(4).tolist(), "max": maxs.max(0).round(4).tolist()}
geo["touching_or_below_ground"] = {k: v["min"][2] for k, v in geo["links"].items() if v["min"][2] < 2e-3}
with open(f"{OUT}/geometry_{TAG}.json", "w") as f:
    json.dump(geo, f, indent=2)
print(f"[T03] lift={LIFT} robot bbox {geo['robot_bbox']}")
print(f"[T03] links touching/below ground (min z < 2 mm): {geo['touching_or_below_ground']}")
for k in ("base_link", "vertical_link", "left_Fixed_Jaw", "right_Fixed_Jaw", "chest_camera", "front_camera"):
    print(f"[T03]   {k:16s} origin {geo['links'][k]['origin']} zmin {geo['links'][k]['min'][2]}")

# ---- render -------------------------------------------------------------------
annotators = {}
for name, (path, res) in cam_paths.items():
    ann = rep.AnnotatorRegistry.get_annotator("rgb")
    ann.attach([rep.create.render_product(path, res)])
    annotators[name] = ann
for _ in range(30):  # let RTX accumulate
    simulation_app.update()
rep.orchestrator.step(rt_subframes=8)
images = {}
for name, ann in annotators.items():
    images[name] = Image.fromarray(np.asarray(ann.get_data())[..., :3])
    images[name].save(f"{OUT}/images/{TAG}_{name}.png")
    print(f"[T03] saved {name} {images[name].size} mean={np.asarray(images[name]).mean():.1f}")
summary = Image.new("RGB", (1280, 960))
for k, name in enumerate(("third_person", "front", "chest", "wrist_left")):
    summary.paste(images[name].resize((640, 480)), ((k % 2) * 640, (k // 2) * 480))
summary.save(f"{OUT}/images/summary_{TAG}.png")
simulation_app.close()
