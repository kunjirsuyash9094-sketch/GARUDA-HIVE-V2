"""Train a hover policy with PPO (stable-baselines3).

    pip install "skysim[rl]"
    godot --headless demo/agent_server.tscn -- --port 5557
    python examples/train_rl.py --timesteps 200000
"""
import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--timesteps", type=int, default=200_000)
    ap.add_argument("--save", default="ppo_hover")
    args = ap.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_checker import check_env
    except ImportError:
        raise SystemExit("Install RL extras: pip install 'skysim[rl]'")

    from skysim import SkySimEnv, HoverTask

    env = SkySimEnv(port=args.port, task=HoverTask(target=(0.0, 3.0, 0.0)), max_steps=1500)
    check_env(env, warn=True)   # validates the Gym interface

    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=args.timesteps)
    model.save(args.save)
    print("saved:", args.save)
    env.close()


if __name__ == "__main__":
    main()
