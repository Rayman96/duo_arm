"""Every path the course code uses. Resolved from this file, so the same names work on the host
(~/Documents/duo_arm) and inside the Isaac Sim container (repo mounted at /workspace)."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY = ROOT / "third_party"
ASSETS = ROOT / "assets"  # corrected URDFs (lesson 01)
OUT = ROOT / "outputs"  # everything generated

USD_DIR = OUT / "usd"  # lesson 01: imported digital twins
ROBOT_USD = USD_DIR / "sim_alohamini2/alohamini2_sim/alohamini2_sim.usda"
TWIN_CHECK_DIR = OUT / "twin_check"  # lesson 02
GRASP_DIR = OUT / "grasp"  # lesson 04
DEMOS_DIR = OUT / "demos"  # lesson 05: scripted demos (npz)
DATASET_DIR = OUT / "datasets/alohamini2_sim_pick_red"  # lesson 05: LeRobot dataset
DATASET_REPO_ID = "local/alohamini2_sim_pick_red"
TRAIN_DIR = OUT / "train"  # lessons 06 / 07: training runs (checkpoints + logs)
EVAL_DIR = OUT / "eval"  # lessons 06-08: evaluations, latency, comparison
