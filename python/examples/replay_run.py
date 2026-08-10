"""Replay a recorded run and report how tightly it reproduces.

    python examples/replay_run.py runs/run_XXced_seed0 --port 5557
"""
import argparse
from skysim import SkySimEnv, NavTask, replay_run, load_run


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--port", type=int, default=5557)
    args = ap.parse_args()

    m = load_run(args.run_dir)["manifest"]
    env = SkySimEnv(port=args.port, task=NavTask(),
                    state_mode=m["state_mode"], include_depth=m["include_depth"],
                    max_steps=m["n_steps"] + 5)
    result = replay_run(args.run_dir, env)
    print(result)
    print("REPRODUCED" if result["reproduced"] else "DIVERGED (expected cross-machine; "
          "needs reference-determinism build, roadmap Phase 5)")
    env.close()


if __name__ == "__main__":
    main()
