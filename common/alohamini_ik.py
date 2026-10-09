"""Kinematic IK for one AlohaMini2 arm (Isaac Sim 6.1, isaacsim.core.experimental).

Task (6-D): the grasp point (between the fingers) at a target position, the fingers pointing
straight down, and the closing direction (jaw +x) square to the world x or y axis so the
fingers close flat on an axis-aligned cube. Damped least squares on a finite-difference Jacobian built from
link poses: the tensor-API Jacobian returned a wrong row/column layout for this asset.

The solver teleports the articulation (set positions + one physics step per evaluation),
so call it before a motion, not while one is running, and restore the state afterwards.
"""

import numpy as np
from alohamini_scene import ARM_JOINTS, CUBES, TABLE_TOP
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager

IK_JOINTS = [j for j in ARM_JOINTS if j != "gripper"]
# Fixed_Jaw link frame (from the STL): the fixed finger runs along +z to the tip at z = 0.104 m and
# spans x in [-0.035, -0.013]; the moving finger closes toward it from +x.
# Half a 4 cm cube plus 1 cm clearance off the fixed finger's inner face, 2.4 cm short of the tip.
GRASP_POINT = np.array([0.017, 0.0, 0.080])
GRIPPER_OPEN, GRIPPER_CLOSED = -0.3, 0.8  # rad; negative opens, ~0.8 is fully closed


def quat_to_R(q):  # (w, x, y, z)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class ArmIK:
    def __init__(self, robot, side):
        self.robot = robot
        self.names = robot.dof_names
        self.n = robot.num_dofs
        self.cols = [self.names.index(f"{side}_{j}") for j in IK_JOINTS]
        path = [p for p in robot.link_paths[0] if p.endswith(f"{side}_Fixed_Jaw")][0]
        self.jaw = RigidPrim(path)

    def _fk(self, q):
        self.robot.set_dof_positions(q[None])
        self.robot.set_dof_velocities(np.zeros((1, self.n)))
        SimulationManager.step(steps=1)
        p, quat = (a.numpy()[0].copy() for a in self.jaw.get_world_poses())
        R = quat_to_R(quat)
        return p + R @ GRASP_POINT, R

    @staticmethod
    def _rot_err(R, R_des):
        return 0.5 * sum(np.cross(R[:, i], R_des[:, i]) for i in range(3))

    def solve(self, q0, target_p, iters=150, tol_p=2e-3, tol_r=2e-2):
        """Return (q, converged, position error m) starting from q0 (full DOF vector)."""
        target_p = np.asarray(target_p, float)
        q = q0.copy()
        # Desired frame: fingers (jaw +z) down; closing axis (jaw +x) snapped to the nearest world x/y axis.
        _, R = self._fk(q)
        x = R[:, 0].copy()
        x[2] = 0.0
        axis = int(np.argmax(np.abs(x[:2])))
        x_des = np.zeros(3)
        x_des[axis] = np.sign(x[axis]) or 1.0
        z_des = np.array([0.0, 0.0, -1.0])
        R_des = np.column_stack([x_des, np.cross(z_des, x_des), z_des])
        h = 1e-3
        for _ in range(iters):
            p, R = self._fk(q)
            e = np.concatenate([target_p - p, self._rot_err(R, R_des)])
            if np.linalg.norm(e[:3]) < tol_p and np.linalg.norm(e[3:]) < tol_r:
                return q, True, float(np.linalg.norm(e[:3]))
            J = np.zeros((6, len(self.cols)))
            for k, c in enumerate(self.cols):
                qh = q.copy()
                qh[c] += h
                ph, Rh = self._fk(qh)
                J[:3, k] = (ph - p) / h
                W = Rh @ R.T  # small rotation from R to Rh
                J[3:, k] = 0.5 * np.array([W[2, 1] - W[1, 2], W[0, 2] - W[2, 0], W[1, 0] - W[0, 1]]) / h
            dq = J.T @ np.linalg.solve(J @ J.T + 0.05 ** 2 * np.eye(6), e)
            q[self.cols] = np.clip(q[self.cols] + np.clip(dq, -0.2, 0.2), -np.pi, np.pi)
        p, _ = self._fk(q)
        return q, False, float(np.linalg.norm(target_p - p))


def plan_bimanual_pick(robot, on_step=None):
    """Plan both arms over the left / right cubes of the standard scene (base at the origin, lift 0).

    robot: alohamini_scene.Robot (started). Returns ({"pre": q, "grasp": q}, {side: converged}).
    The arms are teleported while solving and reset to zero afterwards. on_step() is called between
    solves (e.g. simulation_app.update to keep a GUI responsive).
    """
    plan = {"pre": np.zeros(robot.n), "grasp": np.zeros(robot.n)}
    ok = {}
    z_grasp = TABLE_TOP + 0.028  # grasp point 8 mm above the cube centre: fingertips clear the table
    for side in ("left", "right"):
        (cx, cy), _ = CUBES[side]
        ik = ArmIK(robot.art, side)
        # Each solve only changes this arm's joints, so the plans accumulate both arms.
        plan["pre"], ok_pre, _ = ik.solve(plan["pre"], [cx, cy, z_grasp + 0.08])
        seed = plan["grasp"].copy()
        seed[ik.cols] = plan["pre"][ik.cols]
        plan["grasp"], ok_grasp, _ = ik.solve(seed, [cx, cy, z_grasp])
        ok[side] = ok_pre and ok_grasp
        if on_step:
            on_step()
    robot.teleport(np.zeros(robot.n))
    return plan, ok
