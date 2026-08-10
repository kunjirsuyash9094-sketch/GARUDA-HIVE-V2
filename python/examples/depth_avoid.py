"""Reactive obstacle avoidance from the raycast depth grid (no ML).

Flies forward toward the goal direction while steering away from whichever side
has the nearest obstacle in the depth image. Demonstrates the perception obs.

    godot --headless demo/gps_denied_nav.tscn -- --port 5557    # terminal 1
    python examples/depth_avoid.py --port 5557                  # terminal 2
"""
import argparse
import numpy as np
from skysim import SkySimEnv, NavTask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--steps", type=int, default=3000)
    args = ap.parse_args()

    env = SkySimEnv(
        port=args.port, task=NavTask(goal=(0, 2, -40)),
        state_mode="gps_denied", include_depth=True, max_steps=args.steps,
    )
    obs, info = env.reset(seed=0)

    for i in range(args.steps):
        depth = obs["depth"]                      # (rows, cols), metres
        rows, cols = depth.shape
        band = depth[rows // 3: 2 * rows // 3]     # central horizontal band
        left = band[:, : cols // 2].min()
        right = band[:, cols // 2:].min()
        ahead = band.min()

        pitch = -0.5                               # nose down -> move forward
        yaw = 0.0
        if ahead < 6.0:                            # obstacle close: turn to opener side
            yaw = 0.6 if left < right else -0.6
            pitch = -0.2
        throttle = 0.5                             # hold altitude (approx)
        action = np.array([0.0, pitch, yaw, throttle], dtype=np.float32)

        obs, reward, terminated, truncated, info = env.step(action)
        if i % 200 == 0:
            print(f"step {i:4d}  nearest={ahead:5.1f}m  L={left:4.1f} R={right:4.1f}")
        if terminated:
            print("terminated at step", i, "collision" if info["collision"] else "(goal/oob)")
            break
        if truncated:
            print("truncated"); break
    env.close()


if __name__ == "__main__":
    main()
