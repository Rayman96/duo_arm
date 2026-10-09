"""Convert scripted sim demos (outputs/demos/ep_*.npz) into a LeRobot v3 dataset.

Run in the LeRobot env:
  .venv-lerobot/bin/python lessons/05_demo_data/make_dataset.py [--demos DIR] [--root DIR] [--repo-id ID]
"""

import argparse
import glob
import shutil
import sys
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
from paths import DATASET_DIR, DATASET_REPO_ID, DEMOS_DIR  # noqa: E402
from policy_io import FEATURE_NAMES, FPS, IMG_H, IMG_W, POLICY_CAMERAS, TASK  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--demos", default=str(DEMOS_DIR))
parser.add_argument("--root", default=str(DATASET_DIR))
parser.add_argument("--repo-id", default=DATASET_REPO_ID)
args = parser.parse_args()

hw = {name: float for name in FEATURE_NAMES}
cams = {cam: (IMG_H, IMG_W, 3) for cam in POLICY_CAMERAS}
features = {**hw_to_dataset_features(hw, "action"), **hw_to_dataset_features({**hw, **cams}, "observation")}

shutil.rmtree(args.root, ignore_errors=True)
ds = LeRobotDataset.create(repo_id=args.repo_id, fps=FPS, features=features, root=args.root,
                           robot_type="alohamini2_sim", use_videos=True, streaming_encoding=True)
files = sorted(glob.glob(f"{args.demos}/ep_*.npz"))
for i, f in enumerate(files):
    with np.load(f) as z:  # each z[key] access decompresses the whole array: read every array once
        d = {k: z[k] for k in z.files}
    for t in range(len(d["state"])):
        frame = {"observation.state": d["state"][t], "action": d["action"][t], "task": TASK}
        for cam in POLICY_CAMERAS:
            frame[f"observation.images.{cam}"] = d[f"image_{cam}"][t]
        ds.add_frame(frame)
    ds.save_episode()
    print(f"episode {i + 1}/{len(files)}: {len(d['state'])} frames", flush=True)
ds.finalize()
print(f"dataset: {args.root} ({len(files)} episodes)")
