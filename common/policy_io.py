"""Shared between Isaac Sim (data generation, closed-loop eval) and the LeRobot env (dataset, policy server).

numpy + stdlib only, so it imports in both Python environments.

Features follow the real AlohaMini2 LeRobot convention (AlohaMini._state_ft, am-follower-6dof),
18-D: arm_left_*.pos (7), arm_right_*.pos (7), x.vel, y.vel, theta.vel, lift_axis.height_mm.
Units are sim units: arm joints and grippers in rad (the real robot reports -100..100 / 0..100;
mapping needs each robot's calibration), base velocities in m/s and deg/s, lift in mm with
0 = bottom of travel (sim vertical_move -0.3 m).
"""

import json
import socket
import struct

import numpy as np

FPS = 30
IMG_W, IMG_H = 320, 240
# LeRobot camera feature name -> sim camera (alohamini_scene.CAMERAS); "forward" is the real robot's
# name for the head camera.
POLICY_CAMERAS = {"forward": "front", "wrist_left": "wrist_left"}
TASK = "pick up the red cube with the left arm"
# Red-cube randomization box (m). With the fingers pointing down the left arm reaches y >= -0.41 m:
# in the first 50-demo run 31 of the 35 IK failures were at y <= -0.40.
RED_X, RED_Y = (0.07, 0.17), (-0.40, -0.36)

_ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]
FEATURE_NAMES = ([f"arm_{s}_{j}.pos" for s in ("left", "right") for j in _ARM]
                 + ["x.vel", "y.vel", "theta.vel", "lift_axis.height_mm"])
_SIM_JOINT = {"wrist_yaw": "wrist_yaw_joint"}


def _arm_joints(idx):
    return [idx[f"{s}_{_SIM_JOINT.get(j, j)}"] for s in ("left", "right") for j in _ARM]


def to_features(q, idx, base_vel=(0.0, 0.0, 0.0)):
    """Sim joint vector (robot.q() or targets) -> 18-D feature vector."""
    f = np.zeros(len(FEATURE_NAMES), np.float32)
    f[:14] = q[_arm_joints(idx)]
    f[14:17] = base_vel
    f[17] = (q[idx["vertical_move"]] + 0.3) * 1000.0
    return f


def apply_features(f, q, idx):
    """Write an 18-D action into a copy of the sim target vector q (arms + lift; base handled separately)."""
    q = q.copy()
    q[_arm_joints(idx)] = f[:14]
    q[idx["vertical_move"]] = f[17] / 1000.0 - 0.3
    return q


# ---- minimal length-prefixed messages: JSON header + raw array bytes ----------------------
def send_msg(sock, meta, arrays=None):
    arrays = arrays or {}
    header = dict(meta, _arrays=[(k, str(a.dtype), list(a.shape)) for k, a in arrays.items()])
    hb = json.dumps(header).encode()
    payload = b"".join(np.ascontiguousarray(a).tobytes() for a in arrays.values())
    sock.sendall(struct.pack("!II", len(hb), len(payload)) + hb + payload)


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return bytes(buf)


def recv_msg(sock):
    hl, pl = struct.unpack("!II", _recv_exact(sock, 8))
    header = json.loads(_recv_exact(sock, hl))
    payload = _recv_exact(sock, pl)
    arrays, off = {}, 0
    for k, dtype, shape in header.pop("_arrays"):
        size = int(np.prod(shape)) * np.dtype(dtype).itemsize
        arrays[k] = np.frombuffer(payload[off:off + size], dtype=dtype).reshape(shape)
        off += size
    return header, arrays


def connect(host="127.0.0.1", port=5600):
    s = socket.create_connection((host, port))
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s
