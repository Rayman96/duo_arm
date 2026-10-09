"""AlohaMini2 interactive simulation (Isaac Sim 6.1 GUI).

Scene: corrected AlohaMini2 digital twin at a table with three cubes; the onboard front,
chest and wrist cameras are shown in the bottom panel.

Controls (same keys as lerobot_alohamini's keyboard teleop):
  W / S   base forward / backward        Z / X   base strafe left / right
  A / D   base rotate left / right       U / J   lift up / down
  T / G   base speed up / down
"AlohaMini2 Control" panel: arm joint sliders, preset poses, bimanual pick / place
(planned with IK at start-up), a demo tour, and a policy demo: trained ACT / GR00T policies pick up
the red cube (needs tools/serve_policy.py servers; lessons/08_compare_demo/demo.sh starts them with the app).

Run (from the repo root):
  lessons/03_sim_teleop/run_gui.sh          window on this server's desktop (display :0)
  lessons/03_sim_teleop/run_gui.sh stream   headless + WebRTC livestream for the Isaac Sim WebRTC Streaming Client
"""

import math
import os
import sys
import time

from isaacsim import SimulationApp

STREAM = bool(os.environ.get("SIM_STREAM"))
# In stream mode Kit renders headless and the WebRTC client receives the whole UI (hide_ui False).
simulation_app = SimulationApp({"headless": STREAM, "width": 1920, "height": 1080, "window_width": 1920,
                                "window_height": 1080, "hide_ui": False})

import carb.input  # noqa: E402
import numpy as np  # noqa: E402
import omni.appwindow  # noqa: E402
import omni.timeline  # noqa: E402
import omni.ui as ui  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from omni.kit.viewport.utility import create_viewport_window  # noqa: E402

if STREAM:
    import carb.settings  # noqa: E402
    import omni.kit.app  # noqa: E402

    _settings = carb.settings.get_settings()
    _settings.set("/app/window/drawMouse", True)
    _stream = "/exts/omni.kit.livestream.app/primaryStream"
    _settings.set(f"{_stream}/streamType", "webrtc")
    _settings.set(f"{_stream}/signalPort", int(os.environ.get("SIM_SIGNAL_PORT", 49100)))
    _settings.set(f"{_stream}/streamPort", int(os.environ.get("SIM_STREAM_PORT", 47998)))
    _settings.set(f"{_stream}/allowDynamicResize", True)
    if os.environ.get("SIM_PUBLIC_IP"):  # address the WebRTC client connects to
        _settings.set(f"{_stream}/publicIp", os.environ["SIM_PUBLIC_IP"])
    _mgr = omni.kit.app.get_app().get_extension_manager()
    for _ext in ("omni.kit.livestream.app", "omni.kit.livestream.webrtc"):
        if not _mgr.set_extension_enabled_immediate(_ext, True):
            print(f"[AlohaMini2] livestream extension not available: {_ext}", flush=True)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "common"))
from alohamini_ik import GRIPPER_CLOSED, GRIPPER_OPEN, plan_bimanual_pick  # noqa: E402
from alohamini_scene import (ARM_JOINTS, CUBE, TABLE_TOP, add_onboard_cameras, add_table_and_cubes,  # noqa: E402
                             new_stage, reset_cubes, start_robot)
from policy_io import (FPS, IMG_H, IMG_W, POLICY_CAMERAS, RED_X, RED_Y, apply_features, connect,  # noqa: E402
                       recv_msg, send_msg, to_features)

PHYSICS_DT = 1.0 / 120.0
MAX_CATCHUP_STEPS = 6  # extra physics steps per frame to keep up with the wall clock
SLIDER_RANGE = {"gripper": (-1.0, 1.0)}  # rad; everything else +-pi
SPEED_LEVELS = [(0.1, 0.4), (0.2, 0.8), (0.3, 1.2)]  # (m/s, rad/s), like lerobot's speed levels
LIFT_SPEED = 0.08  # m/s target slew while U/J held
MAIN_EYE, MAIN_TARGET = [1.5, -1.7, 1.35], [0.0, -0.35, 0.6]

# Joint signs (measured on the twin): shoulder_lift + raises the arm, elbow_flex + raises the forearm,
# wrist_flex + tilts the gripper down, gripper - opens.
PRESETS = {
    "Home": {},
    "Tuck": {"shoulder_lift": -1.5},
    "Arms up": {"shoulder_lift": 1.0, "elbow_flex": 0.8},
}

# ---- scene + robot ----------------------------------------------------------------------------
stage = new_stage(PHYSICS_DT)
cube_prims = {name: RigidPrim(path) for name, path in add_table_and_cubes(stage).items()}
robot = start_robot(stage)
cam_paths = add_onboard_cameras(stage)  # mounted on robot links, so after the robot is loaded
timeline = omni.timeline.get_timeline_interface()
n, idx, il, base = robot.n, robot.idx, robot.lift, robot.base

print("[AlohaMini2] planning pick with IK...", flush=True)
plan, ik_ok = plan_bimanual_pick(robot, on_step=simulation_app.update)  # updates keep the window responsive
print(f"[AlohaMini2] IK converged: {ik_ok}", flush=True)
reset_cubes(cube_prims)

# ---- motion queue -----------------------------------------------------------------------------
# state["target"]: joint position targets for arms + lift (the base is driven separately).
# A queue segment is a dict: {"goal": full target vector, "T": seconds} or {"keys": set, "T": seconds}
# or {"call": fn}. Keyboard input always adds on top.
# Base: velocity mode while a base key is held, otherwise position hold at the pose where the keys
# were released (pure velocity mode with a zero command drifted 0.6 m in 14 h idle).
state = {"target": np.zeros(n), "keys": set(), "speed": 1, "queue": [], "seg": None, "seg_t": 0.0,
         "base_hold": None}


def hold_base(pose):
    robot.set_base_velocity_mode(pose is None)
    state["base_hold"] = pose


def arm_pose(pose, sides=("left", "right"), base_q=None):
    goal = (state["target"] if base_q is None else base_q).copy()
    for side in sides:
        for j in ARM_JOINTS:
            if j != "gripper":
                goal[idx[f"{side}_{j}"]] = pose.get(j, 0.0)
    return goal


def adjusted(goal, **kw):
    goal = goal.copy()
    if "gripper" in kw:
        goal[robot.grippers] = kw["gripper"]
    if "lift" in kw:
        goal[il] = kw["lift"]
    return goal


def enqueue(*segments, replace=True):
    if replace:
        state["queue"].clear()
        state["seg"] = None
    state["queue"].extend(segments)


def last_goal():
    """Target the queue ends at, so new segments can be built on top of it."""
    for seg in reversed(state["queue"]):
        if "goal" in seg:
            return seg["goal"]
    return state["target"]


def go_preset(name):
    enqueue({"goal": arm_pose(PRESETS[name]), "T": 1.5})


def set_grippers(value):
    enqueue({"goal": adjusted(state["target"], gripper=value), "T": 0.6})


def reset_base():
    q = robot.q()
    q[base] = 0.0
    robot.art.set_dof_positions(q[None])
    robot.art.set_dof_velocities(np.zeros((1, n)))
    state["base_hold"] = None  # re-captured at the new pose


def pick(replace=True):
    """Bimanual pick of the left and right cubes (the plan assumes the base at the origin)."""
    home = adjusted(np.zeros(n), gripper=GRIPPER_OPEN)
    pre = adjusted(plan["pre"], gripper=GRIPPER_OPEN)
    down = adjusted(plan["grasp"], gripper=GRIPPER_OPEN)
    closed = adjusted(plan["grasp"], gripper=GRIPPER_CLOSED)
    enqueue({"call": reset_base}, {"goal": home, "T": 1.0}, {"goal": pre, "T": 2.0}, {"goal": down, "T": 1.5},
            {"goal": closed, "T": 0.8}, {"goal": closed, "T": 0.6},
            {"goal": adjusted(closed, lift=0.12), "T": 2.0}, replace=replace)


def place(replace=True):
    closed = adjusted(plan["grasp"], gripper=GRIPPER_CLOSED)
    enqueue({"call": reset_base}, {"goal": adjusted(closed, lift=0.12), "T": 0.5}, {"goal": closed, "T": 2.0},
            {"goal": adjusted(closed, gripper=GRIPPER_OPEN), "T": 0.8},
            {"goal": adjusted(plan["pre"], gripper=GRIPPER_OPEN), "T": 1.2},
            {"goal": np.zeros(n), "T": 1.8}, replace=replace)


def wave(replace=True):
    up = arm_pose({"shoulder_lift": 1.0, "elbow_flex": 0.8}, sides=("right",),
                  base_q=state["target"] if replace else last_goal())
    segs = [{"goal": up, "T": 1.0}]
    for k in range(4):
        g = up.copy()
        g[idx["right_shoulder_pan"]] = 0.35 if k % 2 == 0 else -0.35
        segs.append({"goal": g, "T": 0.5})
    segs.append({"goal": arm_pose({}, sides=("right",), base_q=up), "T": 1.0})
    enqueue(*segs, replace=replace)


def tour():
    enqueue({"goal": arm_pose(PRESETS["Tuck"]), "T": 1.5},
            {"keys": {"s"}, "T": 1.5}, {"keys": {"a"}, "T": 2.0}, {"keys": {"d"}, "T": 2.0},
            {"keys": {"w"}, "T": 1.5}, {"keys": {"u"}, "T": 2.0}, {"keys": {"j"}, "T": 2.0},
            {"goal": np.zeros(n), "T": 1.5})
    pick(replace=False)
    enqueue({"keys": {"a"}, "T": 1.2}, {"keys": {"d"}, "T": 1.2}, replace=False)
    place(replace=False)
    wave(replace=False)


# ---- policy demo: trained policies (tools/serve_policy.py) pick up the red cube --------------------
# Same loop as tools/eval_in_sim.py: 30 Hz, observation = state + head + left-wrist camera.
POLICY_PORTS = {"ACT": int(os.environ.get("ACT_PORT", 5600)), "GR00T": int(os.environ.get("GROOT_PORT", 5601))}
demo = {"xy": (float(np.mean(RED_X)), float(np.mean(RED_Y))), "yaw": 0.0, "run": None, "history": [],
        "annotators": None, "rng": np.random.default_rng()}


def place_red():
    x, y = demo["xy"]
    half = math.radians(demo["yaw"]) / 2
    cube_prims["left"].set_world_poses(positions=[[x, y, TABLE_TOP + CUBE / 2]],
                                       orientations=[[math.cos(half), 0, 0, math.sin(half)]])
    cube_prims["left"].set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])


def stop_policy(result=None):
    run = demo["run"]
    if run is None:
        return
    run["sock"].close()
    demo["run"] = None
    if result:
        demo["history"] = (demo["history"] + [result])[-6:]
        print(f"[AlohaMini2] policy demo: {result} | cube {demo['xy'][0]:.3f}, {demo['xy'][1]:.3f}, "
              f"yaw {demo['yaw']:+.0f}", flush=True)
    state["target"] = robot.q()  # manual control continues from the current pose
    state["target"][base] = 0.0
    sync_sliders()


def new_position(rotate=False):
    stop_policy()
    demo["xy"] = (float(demo["rng"].uniform(*RED_X)), float(demo["rng"].uniform(*RED_Y)))
    demo["yaw"] = float(demo["rng"].uniform(-30, 30)) if rotate else 0.0
    enqueue()
    state["target"] = np.zeros(n)
    robot.teleport(np.zeros(n))
    place_red()
    sync_sliders()


def run_policy(name):
    stop_policy()
    if demo["annotators"] is None:  # render products at the policy's input resolution, created on first use
        import omni.replicator.core as rep

        demo["annotators"] = {}
        for cam, sim_cam in POLICY_CAMERAS.items():
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach([rep.create.render_product(cam_paths[sim_cam], (IMG_W, IMG_H))])
            demo["annotators"][cam] = ann
    try:
        sock = connect(port=POLICY_PORTS[name])
    except OSError:
        demo["history"] = (demo["history"] + [f"{name}: server not running (lessons/08_compare_demo/demo.sh)"])[-6:]
        return
    enqueue()
    state["target"] = np.zeros(n)
    robot.teleport(np.zeros(n))
    place_red()
    hold_base(np.zeros(3))
    demo["run"] = {"name": name, "sock": sock, "k": 0, "settle": 10, "target": np.zeros(n), "z0": None,
                   "t_success": None, "max_lift": 0.0}


def policy_frame():
    """One 1/30 s control step of the running policy; returns the status text."""
    run = demo["run"]
    if run["settle"] > 0:  # let the cube settle and the cameras render before the first observation
        run["settle"] -= 1
        if run["settle"] == 0:
            run["z0"] = float(cube_prims["left"].get_world_poses()[0].numpy()[0][2])
            send_msg(run["sock"], {"cmd": "reset"})
            recv_msg(run["sock"])
    else:
        images = {c: np.asarray(a.get_data())[..., :3].copy() for c, a in demo["annotators"].items()}
        send_msg(run["sock"], {"cmd": "act"}, {"state": to_features(robot.q(), idx), **images})
        _, reply = recv_msg(run["sock"])
        run["target"] = apply_features(reply["action"], run["target"], idx)
        robot.set_targets(run["target"])
        run["k"] += 1
    SimulationManager.step(steps=2)  # + the 1/60 s of simulation_app.update() = one 1/30 s frame
    if run["z0"] is None:
        return f"{run['name']}: starting..."
    t = run["k"] / FPS
    lift = float(cube_prims["left"].get_world_poses()[0].numpy()[0][2]) - run["z0"]
    run["max_lift"] = max(run["max_lift"], lift)
    if run["t_success"] is None and lift > 0.05:
        run["t_success"] = t
    if run["t_success"] is not None and t > run["t_success"] + 1.0:
        stop_policy(f"{run['name']}: success, lifted {100 * lift:.1f} cm in {run['t_success']:.1f} s")
    elif t > 12.0:
        stop_policy(f"{run['name']}: failed (max lift {100 * run['max_lift']:.1f} cm)")
    return f"{run['name']} running: t = {t:.1f} s, cube lift {100 * lift:+.1f} cm"


# ---- keyboard --------------------------------------------------------------------------------
KEYMAP = {getattr(carb.input.KeyboardInput, k.upper()): k for k in "wszxaduj"}  # base + lift keys


def on_key(event, *args):
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input in KEYMAP:
            state["keys"].add(KEYMAP[event.input])
        elif event.input == carb.input.KeyboardInput.T:
            state["speed"] = min(state["speed"] + 1, len(SPEED_LEVELS) - 1)
        elif event.input == carb.input.KeyboardInput.G:
            state["speed"] = max(state["speed"] - 1, 0)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE and event.input in KEYMAP:
        state["keys"].discard(KEYMAP[event.input])
    return True


_input = carb.input.acquire_input_interface()
_kb_sub = _input.subscribe_to_keyboard_events(omni.appwindow.get_default_app_window().get_keyboard(), on_key)

# ---- UI ---------------------------------------------------------------------------------------
sliders = {}
win = ui.Window("AlohaMini2 Control", width=430, height=900)
with win.frame:
    with ui.ScrollingFrame():
        with ui.VStack(spacing=6, height=0):
            ui.Label("Policy demo: trained models pick up the red cube", height=18)
            with ui.HStack(height=28, spacing=4):
                ui.Button("New position", clicked_fn=lambda: new_position(False))
                ui.Button("New position + rotation", clicked_fn=lambda: new_position(True))
            with ui.HStack(height=28, spacing=4):
                ui.Button("Run ACT", clicked_fn=lambda: run_policy("ACT"))
                ui.Button("Run GR00T", clicked_fn=lambda: run_policy("GR00T"))
                ui.Button("Stop", clicked_fn=lambda: stop_policy("stopped"))
            policy_label = ui.Label("", height=100, word_wrap=True)
            ui.Spacer(height=6)
            ui.Label("Base: W/S fwd/back  Z/X strafe  A/D turn  T/G speed", height=18)
            ui.Label("Lift: U up / J down   (click the 3D view first)", height=18)
            status_label = ui.Label("", height=36, word_wrap=True)
            ui.Label("Demos", height=18)
            with ui.HStack(height=28, spacing=4):
                ui.Button("Pick cubes", clicked_fn=pick)
                ui.Button("Place back", clicked_fn=place)
                ui.Button("Wave", clicked_fn=wave)
            with ui.HStack(height=28, spacing=4):
                ui.Button("Play demo tour", clicked_fn=tour)
                ui.Button("Reset scene", clicked_fn=lambda: state.update(reset=True))
            ui.Label("Poses (both arms)", height=18)
            with ui.HStack(height=28, spacing=4):
                for pname in PRESETS:
                    ui.Button(pname, clicked_fn=lambda p=pname: go_preset(p))
            with ui.HStack(height=28, spacing=4):
                ui.Button("Open grippers", clicked_fn=lambda: set_grippers(GRIPPER_OPEN))
                ui.Button("Close grippers", clicked_fn=lambda: set_grippers(GRIPPER_CLOSED))
            for arm in ("left", "right"):
                ui.Spacer(height=6)
                ui.Label(f"{arm.capitalize()} arm (rad)", height=18)
                for j in ARM_JOINTS:
                    lo, hi = SLIDER_RANGE.get(j, (-math.pi, math.pi))
                    with ui.HStack(height=22):
                        ui.Label(j.replace("_joint", ""), width=110)
                        s = ui.FloatSlider(min=lo, max=hi, step=0.01)
                        s.model.set_value(0.0)
                        sliders[f"{arm}_{j}"] = s


def on_slider(name):
    def _cb(model):
        if state.get("syncing"):
            return
        state["target"][idx[name]] = model.get_value_as_float()
        enqueue()  # manual input cancels a running motion
    return _cb


for name, s in sliders.items():
    s.model.add_value_changed_fn(on_slider(name))


def sync_sliders():
    state["syncing"] = True
    for name, s in sliders.items():
        s.model.set_value(float(state["target"][idx[name]]))
    state["syncing"] = False


# Onboard camera windows; docked side by side into the bottom panel by dock_layout().
cam_windows = []
for k, name in enumerate(("front", "chest", "wrist_left", "wrist_right")):
    try:
        w = create_viewport_window(f"cam: {name}", width=400, height=300,
                                   position_x=100 + k * 40, position_y=100 + k * 40, camera_path=cam_paths[name])
        w.viewport_api.resolution = (320, 240)  # small render targets keep the GUI responsive
        cam_windows.append(w)
    except Exception as exc:  # keep the app usable even if a viewport fails
        carb.log_warn(f"camera window {name} failed: {exc}")


def dock_layout():
    """Control panel -> right column (tab next to Stage); cameras -> bottom panel, side by side."""
    stage_win = ui.Workspace.get_window("Stage")
    if stage_win:
        win.dock_in(stage_win, ui.DockPosition.SAME)
    content = ui.Workspace.get_window("Content")
    if content and cam_windows:
        cam_windows[0].dock_in(content, ui.DockPosition.SAME)
        for k in range(1, len(cam_windows)):
            remaining = len(cam_windows) - k + 1
            cam_windows[k].dock_in(cam_windows[k - 1], ui.DockPosition.RIGHT, (remaining - 1) / remaining)
        cam_windows[0].focus()
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=MAIN_EYE, target=MAIN_TARGET)


def reset():
    stop_policy("stopped")
    state.update(target=np.zeros(n), queue=[], seg=None, base_hold=None)
    robot.teleport(np.zeros(n))
    reset_cubes(cube_prims)
    sync_sliders()


def advance_queue(dt):
    """Run the current queue segment for dt seconds; return the keys it holds."""
    while state["seg"] is None and state["queue"]:
        seg = state["queue"].pop(0)
        if "call" in seg:
            seg["call"]()
            continue
        state["seg"], state["seg_t"], state["seg_from"] = seg, 0.0, state["target"].copy()
    seg = state["seg"]
    if seg is None:
        return set()
    state["seg_t"] += dt
    if "goal" in seg:
        u = 0.5 - 0.5 * math.cos(math.pi * min(state["seg_t"] / seg["T"], 1.0))  # ease in/out
        state["target"] = state["seg_from"] + (seg["goal"] - state["seg_from"]) * u
    if state["seg_t"] >= seg["T"]:
        state["seg"] = None
    return seg.get("keys", set())


# ---- main loop ----------------------------------------------------------------------------------
print("[AlohaMini2] ready. Keyboard: W/S Z/X A/D U/J T/G; buttons and sliders in 'AlohaMini2 Control'.",
      flush=True)
if STREAM:
    print(f"[AlohaMini2] streaming: connect the Isaac Sim WebRTC Streaming Client to "
          f"{os.environ.get('SIM_PUBLIC_IP', '<server ip>')}", flush=True)
AUTODEMO = bool(os.environ.get("SIM_AUTODEMO"))
# Policy auto-play: new cube position -> ACT -> GR00T on the same cube -> next position (rotated every other).
AUTOPLAY = bool(os.environ.get("SIM_POLICY_AUTOPLAY"))
autoplay_step = 0
frame, last_wall, sim_rate = 0, time.time(), 1.0
while simulation_app.is_running():
    frame += 1
    if frame == 10:  # windows must exist for a few frames before they can be docked
        dock_layout()
    if frame == 30:
        win.focus()
        if AUTODEMO:
            tour()
    if state.pop("reset", False):
        reset()
    if AUTOPLAY and frame > 60 and demo["run"] is None:
        if autoplay_step % 2 == 0:
            new_position(rotate=autoplay_step % 4 == 2)
            run_policy("ACT")
        else:
            run_policy("GR00T")
        autoplay_step += 1
    if not timeline.is_playing():
        simulation_app.update()
        last_wall = time.time()
        continue
    if demo["run"] is not None:  # policy demo: fixed 30 Hz control, no keyboard / queue
        text = policy_frame()
        policy_label.text = "\n".join([text] + demo["history"][::-1])
        simulation_app.update()
        last_wall = time.time()
        continue
    if frame % 30 == 0:
        x, y = demo["xy"]
        policy_label.text = "\n".join([f"red cube at x {x:.3f} y {y:.3f} m, yaw {demo['yaw']:+.0f} deg"]
                                      + demo["history"][::-1])
    # Real-time pacing: app.update() advances one 1/60 s frame; add physics steps so simulated time
    # keeps up with the wall clock when rendering runs below 60 fps.
    now = time.time()
    wall_dt = min(now - last_wall, 0.25)
    last_wall = now
    extra = int(np.clip(round((wall_dt - 1.0 / 60.0) / PHYSICS_DT), 0, MAX_CATCHUP_STEPS))
    dt = 1.0 / 60.0 + extra * PHYSICS_DT  # simulated time this frame
    sim_rate = 0.9 * sim_rate + 0.1 * (dt / max(wall_dt, 1e-3))

    keys = state["keys"] | advance_queue(dt)
    if state["seg"] is not None or frame % 15 == 0:
        sync_sliders()
    state["target"][il] = float(np.clip(state["target"][il] + LIFT_SPEED * dt * (("u" in keys) - ("j" in keys)),
                                        -0.3, 0.3))
    lin, ang = SPEED_LEVELS[state["speed"]]
    vx = lin * (("w" in keys) - ("s" in keys))
    vy = lin * (("z" in keys) - ("x" in keys))
    wz = ang * (("a" in keys) - ("d" in keys))
    q = robot.q()
    yaw = float(q[base[2]])
    moving = bool(vx or vy or wz)
    if moving and state["base_hold"] is not None:
        hold_base(None)
    elif not moving and state["base_hold"] is None:
        hold_base(q[base])
    tgt = state["target"].copy()
    if state["base_hold"] is not None:
        tgt[base] = state["base_hold"]
    robot.set_targets(tgt)
    robot.set_base_velocity(vx, vy, wz, yaw)
    if extra:
        SimulationManager.step(steps=extra)
    status_label.text = (f"speed {lin:.1f} m/s {ang:.1f} rad/s | lift {q[il]*1000+300:.0f} mm | "
                         f"base x {q[base[0]]:+.2f} y {q[base[1]]:+.2f} yaw {math.degrees(yaw):+.0f} deg | "
                         f"sim speed {sim_rate:.2f}x realtime")
    if AUTODEMO and frame % 60 == 0:
        cz = {k: round(float(p.get_world_poses()[0].numpy()[0][2]), 3) for k, p in cube_prims.items()}
        print(f"[AlohaMini2] frame {frame} queue {len(state['queue'])} keys {sorted(keys)} lift {q[il]:+.3f} "
              f"base {q[base[0]]:+.2f},{q[base[1]]:+.2f},{math.degrees(yaw):+.0f} "
              f"grip {q[robot.grippers].round(2).tolist()} cubes z {cz} sim/wall {sim_rate:.2f}", flush=True)
    simulation_app.update()

simulation_app.close()
