"""Smoke test: connect, run random actions, print a few observations.
No ML deps. Start the server first:
    godot --headless demo/agent_server.tscn -- --port 5557
"""
import argparse
from skysim import SkySimEnv, HoverTask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--steps", type=int, default=200)
    args = ap.parse_args()

    env = SkySimEnv(port=args.port, task=HoverTask())
    obs, info = env.reset(seed=0)
    print("reset ok, obs dim:", obs.shape)
    ret = 0.0
    for i in range(args.steps):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        ret += reward
        if i % 25 == 0:
            print(f"step {i:4d}  alt={obs[1]:6.2f}  reward={reward:7.3f}")
        if terminated or truncated:
            print("episode end:", "terminated" if terminated else "truncated")
            obs, info = env.reset()
    print("total return:", round(ret, 2))
    env.close()


if __name__ == "__main__":
    main()
