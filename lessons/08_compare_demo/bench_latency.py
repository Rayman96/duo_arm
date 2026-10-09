"""Inference latency of trained checkpoints on real demo frames (run with the GPU otherwise idle).

  .venv-lerobot/bin/python lessons/08_compare_demo/bench_latency.py <name>=<checkpoint>/pretrained_model ...
Writes outputs/eval/latency_<name>.json: mean / p50 / max ms per action over 300 consecutive actions
(max = a step that predicts a new action chunk).
"""

import glob
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from serve_policy import PolicyRunner  # noqa: E402  (also puts common/ on sys.path)
from paths import DEMOS_DIR, EVAL_DIR  # noqa: E402
from policy_io import POLICY_CAMERAS  # noqa: E402

with np.load(sorted(glob.glob(str(DEMOS_DIR / "ep_*.npz")))[0]) as z:
    demo = {k: z[k] for k in z.files}

for arg in sys.argv[1:]:
    name, ckpt = arg.split("=", 1)
    runner = PolicyRunner(ckpt)
    runner.reset()
    times = []
    for t in range(320):
        k = t % len(demo["state"])
        images = {c: demo[f"image_{c}"][k] for c in POLICY_CAMERAS}
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        runner.act(demo["state"][k], images)
        torch.cuda.synchronize()
        if t >= 20:  # skip warm-up
            times.append(1000 * (time.perf_counter() - t0))
    res = {"name": name, "mean_ms": round(float(np.mean(times)), 2), "p50_ms": round(float(np.median(times)), 2),
           "max_ms": round(float(np.max(times)), 1), "gpu_mem_gb": round(torch.cuda.max_memory_allocated() / 1e9, 2)}
    json.dump(res, open(EVAL_DIR / f"latency_{name}.json", "w"), indent=2)
    print(res, flush=True)
    del runner
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
