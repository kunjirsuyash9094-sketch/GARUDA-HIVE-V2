"""A hand-written controller (no learning) that hovers the drone at a target
altitude using cascaded PID, driving the normalised action interface.

Shows that the SAME interface an RL policy uses also works for classical
control. Start the server first:
    godot --headless demo/agent_server.tscn -- --port 5557
"""
import argparse
import numpy as np
from skysim import SkySimEnv, HoverTask


class PID:
    def __init__(self, kp, ki, kd, out_lo=-1.0, out_hi=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.lo, self.hi = out_lo, out_hi
        self.i = 0.0
        self.prev = None

    def __call__(self, err, dt):
        self.i += err * dt
        d = 0.0 if self.prev is None else (err - self.prev) / max(dt, 1e-6)
        self.prev = err
        return float(np.clip(self.kp * err + self.ki * self.i + self.kd * d, self.lo, self.hi))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--alt", type=float, default=3.0)
    ap.add_argument("--steps", type=int, default=1600)
    args = ap.parse_args()

    env = SkySimEnv(port=args.port, task=HoverTask(target=(0.0, args.alt, 0.0)))
    obs, info = env.reset(seed=0)
    dt = env.dt

    alt_pid = PID(0.6, 0.2, 0.15)     # altitude error -> throttle offset around hover
    hover = 0.5                        # nominal hover throttle guess
    lvl = PID(2.0, 0.0, 0.2)           # keep level: attitude -> corrective tilt

    for i in range(args.steps):
        alt = obs[1]
        roll_ang, pitch_ang = obs[6], obs[7]
        thr = np.clip(hover + alt_pid(args.alt - alt, dt), 0.0, 1.0)
        # command opposite tilt to level out (small)
        roll_cmd = lvl(-roll_ang, dt)
        pitch_cmd = lvl(-pitch_ang, dt)
        action = np.array([roll_cmd, pitch_cmd, 0.0, thr], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        if i % 100 == 0:
            print(f"step {i:4d}  alt={alt:6.2f}  thr={thr:4.2f}  err={args.alt-alt:+5.2f}")
        if terminated:
            print("crashed / out of bounds at step", i); break
        if truncated:
            break
    print(f"final altitude: {obs[1]:.2f} m (target {args.alt} m)")
    env.close()


if __name__ == "__main__":
    main()
