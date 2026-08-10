"""Determinism checking.

Reproducibility is a prerequisite for a trustworthy benchmark: the same seed +
same actions must give the same trajectory. This measures it directly by running
an identical action sequence twice and reporting the maximum state divergence.

Note: exact reproducibility depends on the *server* being deterministic. The
native Godot build compiled with -ffast-math is not guaranteed bit-identical
across machines/architectures (see roadmap: a reference-determinism build flag).
This harness is how you verify a given build/config actually reproduces.
"""
from __future__ import annotations

import numpy as np
from .env import SkySimEnv
from .tasks import HoverTask


def _rollout(env, actions, seed):
    obs, info = env.reset(seed=seed)
    states = [_vec(obs)]
    for a in actions:
        obs, r, term, trunc, info = env.step(a)
        states.append(_vec(obs))
        if term or trunc:
            break
    return np.stack(states)


def _vec(obs):
    return np.asarray(obs["state"] if isinstance(obs, dict) else obs, np.float32)


def check_determinism(port: int = 5557, host: str = "127.0.0.1",
                      seed: int = 0, steps: int = 500, atol: float = 1e-4) -> dict:
    """Run the same seeded action sequence twice; report max divergence."""
    rng = np.random.default_rng(seed)
    actions = [rng.uniform([-1, -1, -1, 0], [1, 1, 1, 1]).astype(np.float32)
               for _ in range(steps)]

    e1 = SkySimEnv(host=host, port=port, task=HoverTask(), max_steps=steps)
    t1 = _rollout(e1, actions, seed); e1.close()
    e2 = SkySimEnv(host=host, port=port, task=HoverTask(), max_steps=steps)
    t2 = _rollout(e2, actions, seed); e2.close()

    n = min(len(t1), len(t2))
    div = np.abs(t1[:n] - t2[:n])
    max_div = float(div.max())
    return {
        "seed": seed, "steps_compared": n,
        "max_divergence": max_div,
        "mean_divergence": float(div.mean()),
        "deterministic": bool(max_div < atol),
        "atol": atol,
    }
