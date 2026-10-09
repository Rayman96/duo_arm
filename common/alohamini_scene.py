"""Shared AlohaMini2 scene setup for Isaac Sim 6.1 (import after SimulationApp has started).

Used by the lesson scripts (02, 03, 04, 05) and tools/eval_in_sim.py:
  robot USD path, joint names, drive gains, stage + robot loading, table / cubes,
  onboard cameras, and the body-velocity -> virtual-joint mapping of the holonomic base.
"""

import math

import isaacsim.core.experimental.utils.stage as stage_utils
import paths
import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

# Written by lessons/01_urdf_to_usd/import_urdf.py from the URDF that fix_urdf.py corrects.
ROBOT_USD = str(paths.ROBOT_USD)
ROBOT_PATH = "/World/robot"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw_joint", "wrist_roll", "gripper"]
BASE_JOINTS = ["root_x_axis_joint", "root_y_axis_joint", "root_z_rotation_joint"]
LIFT_JOINT = "vertical_move"

# PD gains emulating the bus servos' position loop (tensor-API units: N*m/rad, N/m) and max velocities.
# The base joints are virtual (x / y / yaw): these are position-hold gains; velocity driving uses
# kp = 0 with BASE_VEL_KD. Stiffer base gains combined with velocity limits made PhysX diverge.
GAINS = {  # joint-name suffix -> (stiffness, damping, max velocity)
    "shoulder_pan": (60.0, 3.0, 3.7), "shoulder_lift": (60.0, 3.0, 3.7), "elbow_flex": (60.0, 3.0, 3.7),
    "wrist_flex": (20.0, 1.0, 4.7), "wrist_yaw_joint": (20.0, 1.0, 4.7), "wrist_roll": (20.0, 1.0, 4.7),
    "gripper": (10.0, 0.5, 4.7),
    LIFT_JOINT: (20000.0, 2000.0, 0.10),
    "root_x_axis_joint": (2000.0, 400.0, 0.6), "root_y_axis_joint": (2000.0, 400.0, 0.6),
    "root_z_rotation_joint": (500.0, 100.0, 2.0),
}
BASE_VEL_KD = np.array([2000.0, 2000.0, 200.0])

# Table and cubes: the arms only reach ~0.3 m in front of the robot (it faces -y; left arm on +x).
TABLE_TOP, TABLE_EDGE_Y, CUBE = 0.55, -0.30, 0.04
CUBES = {"left": ((0.12, -0.40), (0.9, 0.15, 0.1)), "middle": ((0.0, -0.50), (0.15, 0.7, 0.2)),
         "right": ((-0.12, -0.40), (0.12, 0.3, 0.9))}

# Onboard cameras: optical axis = camera link +z (lens tip 19 mm out); USD cameras look down -Z,
# so rotate 180 deg about x, plus a roll that levels the horizon. 640x480, fx = fy = 320 (HFOV 90 deg).
CAMERAS = {"front": ("front_camera", 90.0), "chest": ("chest_camera", 90.0), "back": ("back_camera", 90.0),
           "wrist_left": ("left_camera", 0.0), "wrist_right": ("right_camera", 0.0)}
CAM_W, CAM_H, CAM_FX, H_APERTURE = 640, 480, 320.0, 20.955


def new_stage(dt, lights=True):
    """Fresh stage with physics (CPU, step dt), a ground plane and optional dome + sun lights."""
    stage_utils.create_new_stage()
    stage = stage_utils.get_current_stage()
    SimulationManager.setup_simulation(dt=dt, device="cpu")
    GroundPlane("/World/ground")
    if lights:
        UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(800)
        sun = UsdLux.DistantLight.Define(stage, "/World/sun")
        sun.CreateIntensityAttr(1500)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))
    return stage


def add_robot(stage, usd=ROBOT_USD):
    """Reference the robot with its PhysX variant; return the articulation root path."""
    stage_utils.add_reference_to_stage(usd_path=usd, path=ROBOT_PATH, variants=[("Physics", "physx")])
    root = Articulation.fetch_articulation_root_api_prim_paths(ROBOT_PATH)[0]
    # A settled articulation falls asleep and then ignores new targets from the tensor API: never sleep.
    PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(root)).CreateSleepThresholdAttr(0.0)
    return root


class Robot:
    """Articulation wrapper: DOF indices, default gains, state reads, base velocity commands."""

    def __init__(self, root_path):
        self.art = Articulation(root_path)

    def start(self):
        """Call once the timeline plays: apply default gains and velocity limits."""
        self.names = self.art.dof_names
        self.n = self.art.num_dofs
        self.idx = {nm: i for i, nm in enumerate(self.names)}
        self.arm = [i for i, nm in enumerate(self.names) if nm.startswith(("left_", "right_"))]
        self.base = [self.idx[j] for j in BASE_JOINTS]
        self.lift = self.idx[LIFT_JOINT]
        self.grippers = [self.idx["left_gripper"], self.idx["right_gripper"]]
        gains = [next(v for k, v in GAINS.items() if nm.endswith(k)) for nm in self.names]
        self.kp, self.kd, vmax = (np.array([g[k] for g in gains]) for k in range(3))
        self.set_gains()
        self.art.set_dof_max_velocities(vmax[None])
        return self

    def set_gains(self, kp=None, kd=None):
        self.art.set_dof_gains((self.kp if kp is None else kp)[None], (self.kd if kd is None else kd)[None])

    def set_base_velocity_mode(self, on):
        """Velocity mode drives the base with set_base_velocity(); off = position hold."""
        kp, kd = self.kp.copy(), self.kd.copy()
        if on:
            kp[self.base], kd[self.base] = 0.0, BASE_VEL_KD
        self.set_gains(kp, kd)

    def q(self):
        # .numpy() is a view of a buffer the tensor API reuses on the next read: copy it.
        return self.art.get_dof_positions().numpy()[0].copy()

    def qd(self):
        return self.art.get_dof_velocities().numpy()[0].copy()

    def set_targets(self, q):
        self.art.set_dof_position_targets(np.asarray(q)[None])

    def teleport(self, q):
        self.art.set_dof_positions(np.asarray(q)[None])
        self.art.set_dof_velocities(np.zeros((1, self.n)))
        self.set_targets(q)

    def set_base_velocity(self, vx, vy, wz, yaw):
        """Body-frame (vx forward, vy left, wz) -> world-frame velocity targets on the virtual joints.
        The robot's front is its -y axis: forward = -y_body, left = +x_body."""
        bx, by = vy, -vx
        v = np.zeros(self.n)
        v[self.base] = [bx * math.cos(yaw) - by * math.sin(yaw), bx * math.sin(yaw) + by * math.cos(yaw), wz]
        self.art.set_dof_velocity_targets(v[None])


def start_robot(stage, usd=ROBOT_USD):
    """add_robot + play the timeline + apply default gains; returns a started Robot."""
    robot = Robot(add_robot(stage, usd))
    omni.timeline.get_timeline_interface().play()
    omni.kit.app.get_app().update()
    return robot.start()


def find_link(stage, name):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if prim.GetName() == name and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
    raise KeyError(name)


def box(stage, path, center, size, color, mass=None, material=None):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(mass)
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, UsdShade.Tokens.weakerThanDescendants,
                                                               "physics")


def add_table_and_cubes(stage, cubes=CUBES):
    """Table in reach of both arms plus 4 cm, 30 g cubes with a grippy (mu = 1) material."""
    mat = UsdShade.Material.Define(stage, "/World/grip_material")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(1.0)
    pm.CreateRestitutionAttr(0.0)
    box(stage, "/World/table", (0.0, TABLE_EDGE_Y - 0.25, TABLE_TOP / 2), (0.9, 0.5, TABLE_TOP), (0.55, 0.42, 0.32))
    for name, ((cx, cy), color) in cubes.items():
        box(stage, f"/World/cube_{name}", (cx, cy, TABLE_TOP + CUBE / 2), (CUBE,) * 3, color, mass=0.03,
            material=mat)
    return {name: f"/World/cube_{name}" for name in cubes}


def reset_cubes(cube_prims, cubes=CUBES):
    """cube_prims: {name: RigidPrim}. Put every cube back at its start pose, at rest."""
    for name, ((cx, cy), _) in cubes.items():
        cube_prims[name].set_world_poses(positions=[[cx, cy, TABLE_TOP + CUBE / 2]], orientations=[[1, 0, 0, 0]])
        cube_prims[name].set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])


def add_onboard_cameras(stage):
    """Mount the 5 robot cameras on their URDF links; returns {name: camera prim path}."""
    paths = {}
    for name, (link, roll) in CAMERAS.items():
        cam = UsdGeom.Camera.Define(stage, find_link(stage, link).GetPath().AppendChild(f"cam_{name}"))
        cam.CreateHorizontalApertureAttr(H_APERTURE)
        cam.CreateVerticalApertureAttr(H_APERTURE * CAM_H / CAM_W)
        cam.CreateFocalLengthAttr(CAM_FX * H_APERTURE / CAM_W)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100))
        xf = UsdGeom.Xformable(cam)
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.019))
        rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), roll) * Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)
        xf.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
        paths[name] = str(cam.GetPath())
    return paths


def add_view_camera(stage, path, eye, target, focal=18.0, aspect=16 / 9):
    """Fixed third-person camera looking from eye to target."""
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(H_APERTURE)
    cam.CreateVerticalApertureAttr(H_APERTURE / aspect)
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    return path
