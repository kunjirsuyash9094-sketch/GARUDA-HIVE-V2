"""Report whether the running SkySim build reproduces trajectories.

    python examples/check_determinism.py --port 5557
"""
import argparse
from skysim import check_determinism


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--steps", type=int, default=500)
    args = ap.parse_args()
    r = check_determinism(port=args.port, steps=args.steps)
    print(r)
    if r["deterministic"]:
        print("DETERMINISTIC — safe for reproducible benchmarks.")
    else:
        print(f"NON-DETERMINISTIC (max divergence {r['max_divergence']:.2e}). "
              "For reproducible runs, build with -DDRONE_SIM_DETERMINISTIC=ON.")


if __name__ == "__main__":
    main()
