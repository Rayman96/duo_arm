"""Comparison table for trained policies: evaluation results + training and serving logs (stdlib only).

  python3 lessons/08_compare_demo/summarize.py act_pick_red:act_40k groot_pick_red:groot_15k ...
      each argument is <training run>:<eval name prefix>; evals named <prefix> and <prefix>_yaw30 are used.
Prints a Markdown table.
"""

import json
import re
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "common"))
from paths import EVAL_DIR as EVAL, TRAIN_DIR  # noqa: E402


def eval_stats(name):
    f = EVAL / f"{name}.json"
    if not f.exists():
        return None
    d = json.load(open(f))
    ok = [r for r in d["results"] if r["success"]]
    lift = sum(r["final_lift_m"] for r in ok) / max(len(ok), 1)
    t = [r["time_to_lift_s"] for r in ok if r["time_to_lift_s"] is not None]
    return {"rate": f'{len(ok)}/{d["episodes"]} ({100 * d["success_rate"]:.0f}%)',
            "lift_cm": f"{100 * lift:.1f}", "t_lift": f"{sum(t) / max(len(t), 1):.1f}"}


def latency(name):
    """From bench_latency.py (idle GPU): mean / max ms per action and inference GPU memory."""
    f = EVAL / f"latency_{name}.json"
    if not f.exists():
        return "-", "-"
    d = json.load(open(f))
    return f'{d["mean_ms"]:.1f} / {d["max_ms"]:.0f}', f'{d["gpu_mem_gb"]:.1f} GB'


def train_stats(run):
    log = (TRAIN_DIR / f"{run}.log").read_text()
    params = re.search(r"num_learnable_params=\d+ \((\S+)\)", log)
    steps = re.findall(r"INFO (\S+ \S+) .*step:(\S+) .*loss:([\d.]+).*mem_gb:([\d.]+)", log)
    t0 = datetime.strptime(steps[0][0], "%Y-%m-%d %H:%M:%S")
    t1 = datetime.strptime(steps[-1][0], "%Y-%m-%d %H:%M:%S")
    return {"params": params.group(1) if params else "-", "steps": steps[-1][1], "loss": steps[-1][2],
            "mem": max(float(s[3]) for s in steps), "minutes": round((t1 - t0).total_seconds() / 60)}


rows = []
for arg in sys.argv[1:]:
    run, name = arg.split(":")
    tr, base, yaw = train_stats(run), eval_stats(name), eval_stats(f"{name}_yaw30")
    lat, inf_mem = latency(name)
    rows.append([name, tr["params"], tr["steps"], f'{tr["minutes"]} min', f'{tr["mem"]:.1f} GB', tr["loss"],
                 base["rate"] if base else "-", base["lift_cm"] if base else "-", base["t_lift"] if base else "-",
                 yaw["rate"] if yaw else "-", lat, inf_mem])
head = ["模型", "可训练参数", "训练步数", "训练时间", "训练显存", "最终 loss", "成功率（同分布）", "平均抬升 cm",
        "平均抓起用时 s", "成功率（方块旋转 ±30°）", "推理 ms/步（平均 / 最大）", "推理显存"]
print("| " + " | ".join(head) + " |")
print("|" + "---|" * len(head))
for r in rows:
    print("| " + " | ".join(str(c) for c in r) + " |")
