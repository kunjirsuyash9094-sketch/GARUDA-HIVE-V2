"""Run the SkySim benchmark suite against a policy and save a scorecard.

    godot --headless demo/gps_denied_nav.tscn -- --port 5557   # (or per-task scene)
    python examples/run_benchmark.py --port 5557 --out scorecard.json

Bring your own policy by editing `make_policy()`, or import your trained model.
"""
import argparse
import numpy as np
from skysim import run_suite, save_scorecard


class BaselinePolicy:
    """A trivial scripted baseline so the suite runs out of the box."""
    def reset(self): pass
    def __call__(self, obs):
        if isinstance(obs, dict):                 # gps_denied_nav (state+depth)
            depth = obs.get("depth")
            yaw = 0.0
            if depth is not None:
                rows, cols = depth.shape
                band = depth[rows//3:2*rows//3]
                if band.min() < 6.0:
                    yaw = 0.6 if band[:, :cols//2].min() < band[:, cols//2:].min() else -0.6
            return np.array([0, -0.4, yaw, 0.5], np.float32)
        alt = obs[1]                              # hover / waypoint (full state)
        return np.array([0, 0, 0, float(np.clip(0.33 + 0.15*(3-alt), 0, 1))], np.float32)


def make_policy():
    return BaselinePolicy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--out", default="scorecard.json")
    ap.add_argument("--tasks", nargs="*", default=None)
    args = ap.parse_args()
    results = run_suite(make_policy(), names=args.tasks, port=args.port, verbose=True)
    save_scorecard(results, args.out)
    print("scorecard saved ->", args.out)


if __name__ == "__main__":
    main()
