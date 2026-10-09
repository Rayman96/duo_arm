"""Serve a trained LeRobot policy (ACT, GR00T, ...) to the Isaac Sim evaluation loop over TCP.

Protocol (common/policy_io.py): the client sends {"cmd": "reset"} at the start of an episode and
{"cmd": "act"} with arrays state (18,) + one uint8 HxWx3 image per camera; the server replies
with {"ok": true} / the next action (18,).

Run in the LeRobot env:
  .venv-lerobot/bin/python tools/serve_policy.py <checkpoint>/pretrained_model [--port 5600]
      [--n-action-steps K]   # execute K actions of each predicted chunk before re-planning
"""

import argparse
import collections
import socket
import sys
import time
from pathlib import Path

import torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.utils import prepare_observation_for_inference

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from policy_io import POLICY_CAMERAS, TASK, recv_msg, send_msg  # noqa: E402


class PolicyRunner:
    """Loads a checkpoint and turns raw observations (numpy) into one action per call."""

    def __init__(self, checkpoint, n_action_steps=None, task=TASK):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.cfg = PreTrainedConfig.from_pretrained(checkpoint)
        if n_action_steps is not None:
            self.cfg.n_action_steps = n_action_steps
        self.policy = get_policy_class(self.cfg.type).from_pretrained(checkpoint, config=self.cfg)
        self.policy.to(self.device).eval()
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=self.cfg, pretrained_path=checkpoint,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}})
        self.task = task
        # Relative-action policies (GR00T with use_relative_actions) must post-process a whole predicted
        # chunk against the state it was predicted from: chunk -> postprocess -> queue. Others (ACT) use
        # select_action, which keeps its own action queue.
        self.chunked = bool(getattr(self.cfg, "use_relative_actions", False))
        self.queue = collections.deque()

    def reset(self):
        self.policy.reset()
        self.pre.reset()
        self.post.reset()
        self.queue.clear()

    @torch.inference_mode()
    def act(self, state, images):
        """state: (18,) float; images: {camera: HxWx3 uint8}. Returns (18,) float numpy."""
        obs = {"observation.state": state.astype("float32").copy()}
        for cam in POLICY_CAMERAS:
            obs[f"observation.images.{cam}"] = images[cam].copy()
        obs = prepare_observation_for_inference(obs, self.device, task=self.task, robot_type="alohamini2_sim")
        if not self.chunked:
            action = self.post(self.policy.select_action(self.pre(obs)))
        else:
            if not self.queue:
                chunk = self.post(self.policy.predict_action_chunk(self.pre(obs)))  # (1, chunk, D)
                self.queue.extend(chunk[0, : self.cfg.n_action_steps])
            action = self.queue.popleft()
        return action.reshape(-1).float().cpu().numpy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--n-action-steps", type=int, default=None)
    parser.add_argument("--task", default=TASK)
    args = parser.parse_args()

    runner = PolicyRunner(args.checkpoint, args.n_action_steps, args.task)
    print(f"[serve] {runner.cfg.type} from {args.checkpoint} on {runner.device} (chunked={runner.chunked}); "
          f"listening on :{args.port}", flush=True)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        n_calls, t_total = 0, 0.0
        try:
            while True:
                meta, arrays = recv_msg(conn)
                if meta["cmd"] == "reset":
                    runner.reset()
                    send_msg(conn, {"ok": True})
                    continue
                t0 = time.time()
                action = runner.act(arrays["state"], {cam: arrays[cam] for cam in POLICY_CAMERAS})
                send_msg(conn, {"ok": True}, {"action": action})
                n_calls += 1
                t_total += time.time() - t0
        except ConnectionError:
            print(f"[serve] client disconnected after {n_calls} actions "
                  f"({1000 * t_total / max(n_calls, 1):.1f} ms/action)", flush=True)
        finally:
            conn.close()


if __name__ == "__main__":
    main()
