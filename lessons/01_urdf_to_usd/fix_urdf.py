"""Produce a simulation-ready AlohaMini2 URDF from the upstream CAD export.

Upstream: third_party/AlohaMini/AlohaMini2/urdf/<variant>/urdf/<variant>.urdf
Output:   assets/<variant>_sim/<variant>_sim.urdf  (+ fixes.json)

The upstream file is a raw SolidWorks export. Fixes applied (each one logged):
  1. Placeholder masses (10.095 kg on cameras/wheels, 1.27 kg on wrist cameras)
     replaced with estimates; inertia scaled by the same ratio so the radius of
     gyration from CAD is kept.
  2. effort="0" velocity="0" on every actuated joint replaced with values from
     the servo datasheets (STS3095 / STS3215 / STS3250) per lerobot_alohamini's
     arm profiles.
  3. package:// mesh URIs rewritten to relative paths.
  4. The virtual planar-base links (root, root_x_link, root_y_link) have zero
     mass, so the importer does not treat `root` as a body and --fix-base welds
     root_x_link to the world instead, deleting the x DOF. They get 1 kg: 0.01 kg
     (a 1:1400 ratio to the 14 kg body) made the PhysX articulation solver blow up
     the base (x ran off 4-7 m under a 0.5 m / 0.2 m/s command).
  5. Wheel links (Link2/3/4_dp) lose their collision geometry: they sit exactly
     on z=0, and with the base driven through virtual x/y/yaw joints the ground
     contact made the base drift several metres.
Joint position limits are left as exported (+-3.14) and flagged for measurement
on the real robot.

Usage: python3 lessons/01_urdf_to_usd/fix_urdf.py [alohamini2|alohamini2pro]
"""

import json
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "common"))
from paths import ASSETS, ROOT, THIRD_PARTY  # noqa: E402

# Stall torque (N*m) and no-load speed (rad/s) at 12 V. Speeds are datasheet
# estimates; torque is derated to ~80% of stall for a usable continuous value.
SERVOS = {
    "sts3215": {"stall_nm": 2.94, "vel": 4.7},  # 30 kg*cm, 0.222 s/60deg
    "sts3095": {"stall_nm": 9.31, "vel": 3.7},  # 95 kg*cm
    "sts3250": {"stall_nm": 4.90, "vel": 4.7},  # 50 kg*cm
}
DERATE = 0.8

# From lerobot_alohamini/src/lerobot/robots/alohamini/alohamini.py _ARM_PROFILES
ARM_PROFILE = {
    "alohamini2": {
        "shoulder_pan": "sts3095", "shoulder_lift": "sts3095", "elbow_flex": "sts3095",
        "wrist_flex": "sts3215", "wrist_yaw_joint": "sts3215", "wrist_roll": "sts3215",
        "gripper": "sts3215",
    },
    "alohamini2pro": {
        "shoulder_pan": "sts3250", "shoulder_lift": "sts3095", "elbow_flex": "sts3095",
        "wrist_flex": "sts3250", "wrist_yaw_joint": "sts3250", "wrist_roll": "sts3250",
        "gripper": "sts3250",
    },
}

# Placeholder masses in the CAD export -> estimated real masses (kg).
MASS_FIX = {
    "chest_camera": 0.05, "front_camera": 0.05, "back_camera": 0.05,
    "left_camera": 0.03, "right_camera": 0.03,
    "Link2_dp": 0.45, "Link3_dp": 0.45, "Link4_dp": 0.45,  # omni wheel + hub
}

VIRTUAL_LINKS = ("root", "root_x_link", "root_y_link")
NO_COLLISION_LINKS = ("Link2_dp", "Link3_dp", "Link4_dp")
VIRTUAL_MASS = 1.0

# Lift: STS3095 through a 131 mm/rev lead screw, ideal F = 2*pi*T/lead, ~50% efficiency.
LIFT_EFFORT_N = 0.5 * 2 * math.pi * SERVOS["sts3095"]["stall_nm"] / 0.131
LIFT_VEL = 0.10  # m/s, conservative
# Virtual planar base joints (root_x / root_y / root_z_rotation).
BASE_LIN = {"effort": 500.0, "velocity": 0.6}
BASE_ROT = {"effort": 200.0, "velocity": 2.0}


def main(variant: str) -> None:
    src_dir = os.path.join(THIRD_PARTY, "AlohaMini", "AlohaMini2", "urdf", variant)
    src = os.path.join(src_dir, "urdf", f"{variant}.urdf")
    out_dir = os.path.join(ASSETS, f"{variant}_sim")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"{variant}_sim.urdf")
    mesh_dir = os.path.relpath(os.path.join(src_dir, f"{variant}_meshes"), out_dir)

    tree = ET.parse(src)
    robot = tree.getroot()
    robot.set("name", f"{variant}_sim")
    log = {"source": os.path.relpath(src, ROOT), "mass": {}, "joints": {}, "meshes": 0, "todo": []}

    for link in robot.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if name in MASS_FIX and inertial is not None:
            m_el = inertial.find("mass")
            old = float(m_el.get("value"))
            new = MASS_FIX[name]
            m_el.set("value", f"{new:.6g}")
            inertia = inertial.find("inertia")
            for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                inertia.set(k, f"{float(inertia.get(k)) * new / old:.6e}")
            log["mass"][name] = {"old": round(old, 4), "new": new}
        if name in VIRTUAL_LINKS:
            if inertial is None:
                inertial = ET.SubElement(link, "inertial")
                ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
                ET.SubElement(inertial, "mass")
                ET.SubElement(inertial, "inertia")
            inertial.find("mass").set("value", str(VIRTUAL_MASS))
            inertial.find("inertia").attrib.update(ixx="1e-3", ixy="0", ixz="0", iyy="1e-3", iyz="0", izz="1e-3")
            log["mass"][name] = {"old": 0.0, "new": VIRTUAL_MASS}
        if name in NO_COLLISION_LINKS:
            for col in link.findall("collision"):
                link.remove(col)
            log.setdefault("collision_removed", []).append(name)
        for mesh in link.iter("mesh"):
            fn = mesh.get("filename")
            mesh.set("filename", f"{mesh_dir}/{os.path.basename(fn)}")
            log["meshes"] += 1

    profile = ARM_PROFILE[variant]
    for joint in robot.findall("joint"):
        name, jtype = joint.get("name"), joint.get("type")
        limit = joint.find("limit")
        if jtype == "fixed":
            continue
        if limit is None:  # continuous yaw joint
            limit = ET.SubElement(joint, "limit")
        if name.startswith(("left_", "right_")):
            servo = profile[name.split("_", 1)[1]]
            eff, vel = SERVOS[servo]["stall_nm"] * DERATE, SERVOS[servo]["vel"]
            tag = servo
        elif name == "vertical_move":
            eff, vel, tag = LIFT_EFFORT_N, LIFT_VEL, "sts3095+leadscrew"
        elif name in ("root_x_axis_joint", "root_y_axis_joint"):
            eff, vel, tag = BASE_LIN["effort"], BASE_LIN["velocity"], "virtual-base"
        elif name == "root_z_rotation_joint":
            eff, vel, tag = BASE_ROT["effort"], BASE_ROT["velocity"], "virtual-base"
        else:
            continue
        old = (limit.get("effort"), limit.get("velocity"))
        limit.set("effort", f"{eff:.4g}")
        limit.set("velocity", f"{vel:.4g}")
        log["joints"][name] = {"type": jtype, "servo": tag, "old_effort_vel": old,
                               "effort": round(eff, 3), "velocity": vel,
                               "lower": limit.get("lower"), "upper": limit.get("upper")}

    log["todo"] += [
        "Joint position limits are +-3.14 from CAD: measure real ranges (calibration files) and replace.",
        "Masses for cameras/wheels are estimates: weigh parts.",
        "Base is modelled as virtual x/y/yaw joints (no wheel contact): fine for manipulation, not for wheel dynamics.",
    ]
    tree.write(out, encoding="utf-8", xml_declaration=True)
    with open(os.path.join(out_dir, "fixes.json"), "w") as f:
        json.dump(log, f, indent=2, ensure_ascii=False)
    total = sum(float(l.find("inertial/mass").get("value")) for l in robot.findall("link")
                if l.find("inertial") is not None)
    print(f"wrote {os.path.relpath(out, ROOT)}  total mass {total:.2f} kg, "
          f"{len(log['mass'])} masses fixed, {len(log['joints'])} joints given effort/velocity")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "alohamini2")
