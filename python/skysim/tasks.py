"""Tasks define reward and termination on top of the raw simulator.

Keeping tasks separate from the sim keeps SkySim generic: the server just streams
state + ground truth, and the task decides what "good" means. Write your own by
subclassing Task.
"""
from __future__ import annotations
import numpy as np


class Task:
    """Base task. Override reward(); optionally reset()."""
    def reset(self, obs_msg: dict) -> None:
        pass

    def reward(self, obs_msg: dict) -> tuple[float, bool]:
        """Return (reward, terminated) from an obs message."""
        return 0.0, False


class HoverTask(Task):
    """Reward for holding a target position; terminate on crash/out-of-bounds."""
    def __init__(self, target=(0.0, 3.0, 0.0), vel_penalty: float = 0.05,
                 alive_bonus: float = 0.1, crash_alt: float = 0.18):
        self.target = np.asarray(target, dtype=np.float32)
        self.vel_penalty = vel_penalty
        self.alive_bonus = alive_bonus
        self.crash_alt = crash_alt

    def reward(self, obs_msg: dict) -> tuple[float, bool]:
        gt = obs_msg["gt"]
        pos = np.asarray(gt["pos"], dtype=np.float32)
        vel = np.asarray(gt["vel"], dtype=np.float32)
        dist = float(np.linalg.norm(pos - self.target))
        r = -dist - self.vel_penalty * float(np.linalg.norm(vel)) + self.alive_bonus
        crashed = bool(obs_msg.get("collision", False)) or pos[1] < self.crash_alt
        return r, crashed


class WaypointTask(Task):
    """Advance through a list of waypoints; reward proximity, bonus on reaching."""
    def __init__(self, waypoints, reach_radius: float = 0.6, reach_bonus: float = 10.0):
        self.waypoints = [np.asarray(w, dtype=np.float32) for w in waypoints]
        self.reach_radius = reach_radius
        self.reach_bonus = reach_bonus
        self._i = 0

    def reset(self, obs_msg: dict) -> None:
        self._i = 0

    def reward(self, obs_msg: dict) -> tuple[float, bool]:
        gt = obs_msg["gt"]
        pos = np.asarray(gt["pos"], dtype=np.float32)
        target = self.waypoints[self._i]
        dist = float(np.linalg.norm(pos - target))
        r = -dist + 0.1
        if dist < self.reach_radius:
            r += self.reach_bonus
            self._i += 1
        done = self._i >= len(self.waypoints) or bool(obs_msg.get("collision", False))
        return r, done


class NavTask(Task):
    """Reach a goal position through obstacles. Uses ground-truth position for
    reward (privileged), so the policy can be GPS-denied while training signal
    is dense. Rewards progress toward the goal; penalises collisions."""
    def __init__(self, goal=(0.0, 2.0, -40.0), reach_radius: float = 1.5,
                 reach_bonus: float = 100.0, collision_penalty: float = 50.0,
                 step_penalty: float = 0.01):
        self.goal = np.asarray(goal, dtype=np.float32)
        self.reach_radius = reach_radius
        self.reach_bonus = reach_bonus
        self.collision_penalty = collision_penalty
        self.step_penalty = step_penalty
        self._prev_dist = None

    def reset(self, obs_msg: dict) -> None:
        self._prev_dist = None

    def reward(self, obs_msg: dict):
        pos = np.asarray(obs_msg["gt"]["pos"], dtype=np.float32)
        dist = float(np.linalg.norm(pos - self.goal))
        progress = 0.0 if self._prev_dist is None else (self._prev_dist - dist)
        self._prev_dist = dist
        r = progress - self.step_penalty
        terminated = False
        if obs_msg.get("collision", False):
            r -= self.collision_penalty
            terminated = True
        if dist < self.reach_radius:
            r += self.reach_bonus
            terminated = True
        return r, terminated
