"""Standard benchmark tasks and an evaluation runner.

A benchmark is a named, seeded scenario with a scoring function. Running the
suite against any policy produces a reproducible scorecard — the thing that lets
the community compare algorithms and cite results.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Callable, Optional
import json
import numpy as np

from .env import SkySimEnv
from .tasks import HoverTask, NavTask, WaypointTask


@dataclass
class Benchmark:
    name: str
    make_task: Callable
    env_kwargs: dict = field(default_factory=dict)
    seeds: tuple = (0, 1, 2, 3, 4)
    max_steps: int = 1500
    description: str = ""


# The standard suite. Deliberately small and stable — a benchmark's value is in
# not changing, so numbers stay comparable across versions.
SUITE = {
    "hover": Benchmark(
        "hover", lambda: HoverTask(target=(0, 3, 0)),
        env_kwargs=dict(state_mode="full"),
        description="Reach and hold 3 m altitude.",
    ),
    "waypoint": Benchmark(
        "waypoint", lambda: WaypointTask(waypoints=[(0, 3, -5), (5, 3, -5), (5, 3, 0)]),
        env_kwargs=dict(state_mode="full"),
        description="Fly a 3-waypoint square leg.",
    ),
    "gps_denied_nav": Benchmark(
        "gps_denied_nav", lambda: NavTask(goal=(0, 2, -40)),
        env_kwargs=dict(state_mode="gps_denied", include_depth=True),
        description="Reach a goal 40 m ahead through obstacles, vision-only.",
    ),
}


@dataclass
class EpisodeResult:
    seed: int
    steps: int
    total_reward: float
    terminated: bool
    truncated: bool
    collision: bool
    final_pos: list


def run_benchmark(policy, bench: Benchmark, port: int = 5557,
                  host: str = "127.0.0.1", verbose: bool = False) -> dict:
    """Run one benchmark across its seeds. `policy(obs) -> action`.
    policy.reset() is called at each episode start if it exists."""
    episodes = []
    for seed in bench.seeds:
        env = SkySimEnv(host=host, port=port, task=bench.make_task(),
                        max_steps=bench.max_steps, **bench.env_kwargs)
        obs, info = env.reset(seed=seed)
        if hasattr(policy, "reset"):
            policy.reset()
        ret, coll = 0.0, False
        term = trunc = False
        while not (term or trunc):
            action = policy(obs)
            obs, r, term, trunc, info = env.step(action)
            ret += r
            coll = coll or info.get("collision", False)
        episodes.append(EpisodeResult(seed, env._steps, ret, term, trunc, coll,
                                      [round(x, 2) for x in (info.get("gt_pos") or [0, 0, 0])]))
        env.close()
        if verbose:
            print(f"  seed {seed}: reward={ret:8.2f} steps={env._steps} "
                  f"{'COLLISION' if coll else 'clean'}")
    rewards = np.array([e.total_reward for e in episodes])
    return {
        "benchmark": bench.name,
        "description": bench.description,
        "n_episodes": len(episodes),
        "reward_mean": float(rewards.mean()),
        "reward_std": float(rewards.std()),
        "collision_rate": float(np.mean([e.collision for e in episodes])),
        "episodes": [asdict(e) for e in episodes],
    }


def run_suite(policy, names=None, port: int = 5557, verbose: bool = True) -> dict:
    names = names or list(SUITE)
    results = {}
    for n in names:
        if verbose:
            print(f"[{n}] {SUITE[n].description}")
        results[n] = run_benchmark(policy, SUITE[n], port=port, verbose=verbose)
        if verbose:
            r = results[n]
            print(f"  -> reward {r['reward_mean']:.2f} ± {r['reward_std']:.2f}, "
                  f"collisions {r['collision_rate']*100:.0f}%\n")
    return {"suite": results}


def save_scorecard(results: dict, path: str):
    with open(path, "w") as f:
        json.dump(results, f, indent=2)
