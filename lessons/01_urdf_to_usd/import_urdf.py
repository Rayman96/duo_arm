"""T01: URDF -> USD import with the Isaac Sim 6.1 importer, then inspect the result.

Imports (a) the raw upstream URDF and (b) the fixed URDF from fix_urdf.py, and
writes outputs/usd/import_<name>.json with joint/link/mass/collision statistics. Each output folder
is cleared first, so the robot USD always lands at the fixed path paths.ROBOT_USD
(the importer otherwise writes numbered copies next to an existing output).

Run: tools/run_isaac.sh lessons/01_urdf_to_usd/import_urdf.py
"""

import json
import shutil
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
from paths import ASSETS, THIRD_PARTY, USD_DIR  # noqa: E402

UPSTREAM = f"{THIRD_PARTY}/AlohaMini/AlohaMini2/urdf/alohamini2"

CASES = [
    {
        "name": "raw_alohamini2",
        "urdf": f"{UPSTREAM}/urdf/alohamini2.urdf",
        "ros_package_paths": [{"name": "alohamini2_urdf", "path": UPSTREAM}],
    },
    {
        "name": "sim_alohamini2",
        "urdf": f"{ASSETS}/alohamini2_sim/alohamini2_sim.urdf",
        "ros_package_paths": [],
    },
]


def inspect(usd_file: str) -> dict:
    stage = Usd.Stage.Open(usd_file)
    # Importer output keeps physics in a variant set; nothing is selected by default.
    default = stage.GetDefaultPrim()
    vset = default.GetVariantSets().GetVariantSet("Physics")
    variants = vset.GetVariantNames()
    vset.SetVariantSelection("physx")
    info = {"usd": usd_file, "physics_variants": variants, "joints": [], "links": {}, "articulation_roots": [], "collision_prims": 0,
            "mesh_points": 0, "zero_or_missing_mass": []}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            info["articulation_roots"].append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            info["collision_prims"] += 1
        if prim.GetTypeName() == "Mesh":
            pts = prim.GetAttribute("points").Get()
            info["mesh_points"] += len(pts) if pts else 0
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            m = UsdPhysics.MassAPI(prim).GetMassAttr().Get() if prim.HasAPI(UsdPhysics.MassAPI) else None
            info["links"][prim.GetName()] = m
            if not m:
                info["zero_or_missing_mass"].append(prim.GetName())
        if prim.IsA(UsdPhysics.Joint) and not prim.IsA(UsdPhysics.FixedJoint):
            entry = {"name": prim.GetName(), "type": prim.GetTypeName()}
            for attr in ("physics:lowerLimit", "physics:upperLimit", "physxJoint:maxJointVelocity"):
                a = prim.GetAttribute(attr)
                if a and a.HasValue():
                    entry[attr.split(":")[1]] = a.Get()
            for drive in ("angular", "linear"):
                d = UsdPhysics.DriveAPI.Get(prim, drive)
                if d:
                    entry["drive"] = {"type": drive, "stiffness": d.GetStiffnessAttr().Get(),
                                      "damping": d.GetDampingAttr().Get(), "maxForce": d.GetMaxForceAttr().Get()}
            info["joints"].append(entry)
    info["n_dof_joints"] = len(info["joints"])
    info["total_mass"] = round(sum(v for v in info["links"].values() if v), 3)
    return info


for case in CASES:
    cfg = URDFImporterConfig()
    cfg.urdf_path = case["urdf"]
    cfg.usd_path = f"{USD_DIR}/{case['name']}"
    shutil.rmtree(cfg.usd_path, ignore_errors=True)
    cfg.ros_package_paths = case["ros_package_paths"]
    cfg.fix_base = True  # 'root' link anchored; base moves through virtual x/y/yaw joints
    cfg.collision_type = "Convex Hull"
    cfg.joint_target_type = "position"
    t0 = time.time()
    try:
        usd = URDFImporter(cfg).import_urdf()
        dt = time.time() - t0
        info = inspect(usd)
        info["import_seconds"] = round(dt, 1)
        info["ok"] = True
    except Exception as exc:  # report, keep going
        info = {"ok": False, "error": repr(exc), "import_seconds": round(time.time() - t0, 1)}
    with open(f"{USD_DIR}/import_{case['name']}.json", "w") as f:
        json.dump(info, f, indent=2, default=str)
    print(f"[T01] {case['name']}: ok={info['ok']} t={info['import_seconds']}s "
          f"dof={info.get('n_dof_joints')} mass={info.get('total_mass')} "
          f"collisions={info.get('collision_prims')} usd={info.get('usd')}")

simulation_app.close()
