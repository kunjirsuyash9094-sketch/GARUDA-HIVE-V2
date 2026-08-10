"""Deterministic replay: re-run a recorded run and measure state divergence.

Given the same seed + randomization + action sequence, a deterministic simulator
should reproduce the recorded trajectory. Bit-exact reproduction across machines
needs the reference-determinism build (roadmap Phase 5); on one machine this
should already be tight.
"""
from __future__ import annotations

import numpy as np
from .record import load_run


def _state_of(obs):
    return np.asarray(obs["state"] if isinstance(obs, dict) else obs, np.float32)


def replay_run(run_dir, env, atol: float = 1e-3) -> dict:
    data = load_run(run_dir)
    m = data["manifest"]
    options = None
    if m.get("dr_requested") is not None:
        options = {"randomize": m["dr_requested"]}

    obs, info = env.reset(seed=m.get("seed"), options=options)
    recorded = data["state"]                      # (n_obs, sdim)
    actions = data["actions"]                     # (n_steps, 4)

    traj = [_state_of(obs)]
    for a in actions:
        obs, r, term, trunc, info = env.step(a)
        traj.append(_state_of(obs))
        if term or trunc:
            break
    traj = np.stack(traj)

    n = min(len(traj), len(recorded))
    max_div = float(np.abs(traj[:n] - recorded[:n]).max()) if n else float("nan")
    return {
        "steps_compared": n,
        "max_divergence": max_div,
        "reproduced": bool(max_div < atol),
    }
