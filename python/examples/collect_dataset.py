"""Collect a randomized dataset of nav episodes to disk.

    godot --headless demo/gps_denied_nav.tscn -- --port 5557   # terminal 1
    python examples/collect_dataset.py --port 5557 --episodes 20
"""
import argparse
import numpy as np
from skysim import SkySimEnv, NavTask, RecordRun, DomainRandomizer


def policy(obs):
    depth = obs["depth"]
    rows, cols = depth.shape
    band = depth[rows // 3: 2 * rows // 3]
    ahead = band.min()
    left, right = band[:, : cols // 2].min(), band[:, cols // 2:].min()
    pitch, yaw = -0.5, 0.0
    if ahead < 6.0:
        yaw = 0.6 if left < right else -0.6
        pitch = -0.2
    return np.array([0.0, pitch, yaw, 0.5], dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--out", default="runs")
    ap.add_argument("--max-steps", type=int, default=1500)
    args = ap.parse_args()

    base = SkySimEnv(port=args.port, task=NavTask(goal=(0, 2, -40)),
                     state_mode="gps_denied", include_depth=True, max_steps=args.max_steps)
    env = RecordRun(base, out_dir=args.out)
    dr = DomainRandomizer(seed=1234)

    for ep in range(args.episodes):
        obs, info = env.reset(seed=ep, options={"randomize": dr.sample()})
        done = False
        while not done:
            obs, r, term, trunc, info = env.step(policy(obs))
            done = term or trunc
        print(f"episode {ep:3d} recorded -> {env._run_dir}")
    env.close()
    print(f"\nDataset written under: {args.out}/")


if __name__ == "__main__":
    main()
