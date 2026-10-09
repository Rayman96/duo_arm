"""T02: Physics sanity of the imported AlohaMini2 articulation (PhysX, headless).

  A. Torque margin: static holding torque over random arm poses vs servo effort limit.
     Measured as kp*(target-q) under a stiff, unclamped PD hold, because
     get_dof_gravity_compensation_forces() returned values in the wrong DOF order
     for this asset (156 N = total robot weight showed up on vertical_move).
  B. Gravity hold: hold home pose for 3 s, report steady-state error and drift.
  C. Tracking: 0.5 Hz sine on every arm joint, report RMS error / lag.
  D. Lift: drive vertical_move to both limits. Base: body-frame velocity commands
     (x.vel, y.vel, theta.vel, as in the real robot's LeRobot action space) mapped to
     velocity targets on the virtual x/y/yaw joints; report pose error vs integrated reference.
  E. Self-collision on: settle 3 s at home pose, report max joint velocity (explosion check).
  F. Throughput: physics steps per second for one robot.

Writes outputs/twin_check/physics.json and tracking.npz.
Run: tools/run_isaac.sh lessons/02_twin_check/check_physics.py [path/to/robot.usda]
"""

import json
import math
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
from alohamini_scene import GAINS, ROBOT_USD, new_stage, start_robot  # noqa: E402
from paths import TWIN_CHECK_DIR as OUT  # noqa: E402

DT = 1.0 / 200.0
usd = sys.argv[1] if len(sys.argv) > 1 else ROBOT_USD

robot = start_robot(new_stage(DT, lights=False), usd)
art, names, n, idx, arm = robot.art, robot.names, robot.n, robot.idx, robot.arm
max_eff = art.get_dof_max_efforts().numpy()[0]
masses = art.get_link_masses().numpy()[0]
report = {"usd": usd, "dof_names": names, "num_links": art.num_links, "num_shapes": art.num_shapes,
          "total_mass_kg": float(masses.sum()), "max_effort": dict(zip(names, max_eff.round(3).tolist())),
          "gains": GAINS, "link_masses": dict(zip(art.link_names, masses.round(4).tolist()))}
print(f"[T02] {n} dofs, {art.num_links} links, {art.num_shapes} collision shapes, mass {masses.sum():.2f} kg")
home = np.zeros(n)


def settle(q, seconds):
    robot.teleport(q)
    SimulationManager.step(steps=int(seconds / DT))


# ---- A. torque margin ------------------------------------------------------
rng = np.random.default_rng(0)
art.set_dof_max_efforts(np.full((1, n), 1e6))
kp_s, kd_s = robot.kp.copy(), robot.kd.copy()
kp_s[arm], kd_s[arm] = 5000.0, 50.0
robot.set_gains(kp_s, kd_s)
hold = []
for _ in range(100):
    q = home.copy()
    q[arm] = rng.uniform(-1.6, 1.6, len(arm))
    settle(q, 0.4)
    hold.append(np.abs(kp_s * (q - robot.q())))
hold = np.max(hold, axis=0)
art.set_dof_max_efforts(max_eff[None])
robot.set_gains()
report["A_torque_margin"] = {
    names[i]: {"max_static_torque": round(float(hold[i]), 3), "effort_limit": round(float(max_eff[i]), 3),
               "ratio": round(float(hold[i] / max_eff[i]), 3)} for i in arm}
worst = max(report["A_torque_margin"].items(), key=lambda kv: kv[1]["ratio"])
print(f"[T02-A] worst static/effort ratio: {worst[0]} {worst[1]}")

# ---- B. gravity hold -------------------------------------------------------
settle(home, 3.0)
q, v = robot.q(), robot.qd()
report["B_gravity_hold"] = {"max_abs_err_rad": float(np.abs(q - home)[arm].max()),
                            "per_joint_err": dict(zip([names[i] for i in arm], np.round((q - home)[arm], 4).tolist())),
                            "lift_err_m": float(q[robot.lift]), "max_abs_vel": float(np.abs(v).max())}
print(f"[T02-B] hold error max {report['B_gravity_hold']['max_abs_err_rad']:.4f} rad, "
      f"lift err {report['B_gravity_hold']['lift_err_m']*1000:.1f} mm")

# ---- C. sine tracking ------------------------------------------------------
settle(home, 0.5)
T, f, amp = 4.0, 0.5, 0.4
ts, tgt, meas = [], [], []
for k in range(int(T / DT)):
    t = k * DT
    q_t = home.copy()
    q_t[arm] = amp * np.sin(2 * np.pi * f * t)
    robot.set_targets(q_t)
    SimulationManager.step(steps=1)
    ts.append(t); tgt.append(q_t); meas.append(robot.q())
tgt, meas = np.array(tgt), np.array(meas)
skip = int(1.0 / DT)  # ignore first second
err = (meas - tgt)[skip:, arm]
lag_ms = []
for j in arm:  # lag by cross-correlation
    a, b = tgt[skip:, j], meas[skip:, j]
    c = [np.dot(a[: len(a) - s], b[s:]) for s in range(0, int(0.3 / DT))]
    lag_ms.append(int(np.argmax(c)) * DT * 1000)
report["C_tracking"] = {"freq_hz": f, "amp_rad": amp,
                        "rms_err_rad": dict(zip([names[i] for i in arm], np.round(np.sqrt((err ** 2).mean(0)), 4).tolist())),
                        "lag_ms": dict(zip([names[i] for i in arm], lag_ms))}
np.savez(f"{OUT}/tracking.npz", t=np.array(ts), target=tgt, measured=meas, names=np.array(names))
print(f"[T02-C] tracking RMS max {np.sqrt((err**2).mean(0)).max():.4f} rad, lag max {max(lag_ms):.0f} ms")

# ---- D. lift + base --------------------------------------------------------
lift_traces = {}
settle(home, 0.5)
for goal in (-0.3, 0.3, 0.0):
    q_t = home.copy(); q_t[robot.lift] = goal
    robot.set_targets(q_t)
    trace = []
    for _ in range(8):
        SimulationManager.step(steps=int(1.0 / DT))
        trace.append(round(float(robot.q()[robot.lift]), 4))
    lift_traces[str(goal)] = trace
report["D_lift_traces_per_s"] = lift_traces
for k, v in lift_traces.items():
    print(f"[T02-D] lift->{k}: {v}")


def base_velocity(vx, vy, wz, seconds=4.0):
    """Drive the base with a body-frame command; return max |pose - integrated reference| (x, y, yaw)."""
    settle(home, 0.2)
    robot.set_base_velocity_mode(True)
    ref, errs = np.zeros(3), []
    for _ in range(int(seconds / DT)):
        robot.set_base_velocity(vx, vy, wz, robot.q()[robot.base[2]])
        SimulationManager.step(steps=1)
        bx, by, c, s = vy, -vx, math.cos(ref[2]), math.sin(ref[2])  # same body->world map as the robot
        ref += DT * np.array([bx * c - by * s, bx * s + by * c, wz])
        errs.append(np.abs(robot.q()[robot.base] - ref))
    robot.set_base_velocity(0, 0, 0, 0)
    robot.set_base_velocity_mode(False)
    return np.array(errs).max(0)


base = {}
for label, cmd in {"fwd_0.2": (0.2, 0, 0), "strafe_0.2": (0, 0.2, 0), "spin_0.5": (0, 0, 0.5),
                   "arc": (0.2, 0, 0.5), "diag_spin": (0.15, -0.15, -0.8)}.items():
    e = base_velocity(*cmd)
    base[label] = {"cmd_vx_vy_wz": list(cmd), "max_err_x_y_yaw": e.round(4).tolist()}
    print(f"[T02-D] base {label:10s} cmd {cmd} max err x,y,yaw {e.round(4).tolist()}")
report["D_base_velocity"] = base

# ---- E. self-collision stability ------------------------------------------
art.set_enabled_self_collisions(True)
settle(home, 3.0)
q, v = robot.q(), robot.qd()
report["E_self_collision"] = {"max_abs_vel_after_3s": float(np.abs(v).max()),
                              "max_abs_drift_rad": float(np.abs(q - home)[arm].max())}
print(f"[T02-E] self-collision on: max vel {np.abs(v).max():.3f}, drift {np.abs(q-home)[arm].max():.3f} rad")
art.set_enabled_self_collisions(False)

# ---- F. throughput ---------------------------------------------------------
t0 = time.time()
SimulationManager.step(steps=2000)
sps = 2000 / (time.time() - t0)
report["F_throughput"] = {"steps_per_s": round(sps, 1), "realtime_factor": round(sps * DT, 2), "device": "cpu"}
print(f"[T02-F] {sps:.0f} steps/s = {sps*DT:.1f}x realtime")

with open(f"{OUT}/physics.json", "w") as fh:
    json.dump(report, fh, indent=2)
simulation_app.close()
