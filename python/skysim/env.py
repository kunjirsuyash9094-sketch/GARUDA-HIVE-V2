"""SkySim Gymnasium environment.

Connects to a running SkySim agent server (see docs/agent_protocol.md) over
WebSocket and exposes a standard Gymnasium interface.

Observation is a plain Box (state only) by default. Enable perception to switch
to a Dict space:

    env = SkySimEnv(include_depth=True, state_mode="gps_denied",
                    task=NavTask(goal=(0, 2, -40)))
    obs, info = env.reset(seed=0)   # obs = {"state": ..., "depth": ...}

Start the server first, e.g.:
    godot --headless demo/gps_denied_nav.tscn -- --port 5557
"""

from __future__ import annotations

import base64
import json
import math
from typing import Any, Optional

import numpy as np
import gymnasium as gym
from gymnasium import spaces

try:
    import websocket  # from the 'websocket-client' package
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "SkySimEnv needs the 'websocket-client' package: pip install websocket-client"
    ) from exc

from .tasks import Task, HoverTask

STATE_DIMS = {"full": 12, "gps_denied": 9}


class SkySimEnv(gym.Env):
    """Gymnasium env backed by a SkySim agent server.

    Action (Box, 4-d, normalised): [roll, pitch, yaw_rate, throttle].
    Observation:
      - state Box: full = pos,vel,euler,rates (12); gps_denied = vel,euler,rates (9).
      - +depth (rows,cols) and/or +rgb (h,w,3) -> Dict space {"state","depth","rgb"}.
    Reward / termination come from the pluggable `task`.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5557,
        task: Optional[Task] = None,
        max_steps: int = 2000,
        max_tilt_deg: float = 30.0,
        max_yaw_rate_deg: float = 120.0,
        state_mode: str = "full",
        include_depth: bool = False,
        include_rgb: bool = False,
        depth_size: tuple = (32, 24),   # (cols, rows)
        rgb_size: tuple = (128, 96),    # (w, h)
        max_range: float = 40.0,
        connect_timeout: float = 10.0,
    ) -> None:
        super().__init__()
        if state_mode not in STATE_DIMS:
            raise ValueError(f"state_mode must be one of {list(STATE_DIMS)}")
        self.url = f"ws://{host}:{port}"
        self.task: Task = task or HoverTask()
        self.max_steps = int(max_steps)
        self.max_tilt = math.radians(max_tilt_deg)
        self.max_yaw_rate = math.radians(max_yaw_rate_deg)
        self.state_mode = state_mode
        self.include_depth = include_depth
        self.include_rgb = include_rgb
        self.depth_cols, self.depth_rows = depth_size
        self.rgb_w, self.rgb_h = rgb_size
        self.max_range = max_range
        self.connect_timeout = connect_timeout

        self.action_space = spaces.Box(
            low=np.array([-1, -1, -1, 0], dtype=np.float32),
            high=np.array([1, 1, 1, 1], dtype=np.float32),
        )
        sdim = STATE_DIMS[state_mode]
        state_space = spaces.Box(-np.inf, np.inf, shape=(sdim,), dtype=np.float32)
        if include_depth or include_rgb:
            d = {"state": state_space}
            if include_depth:
                d["depth"] = spaces.Box(0.0, max_range,
                                        shape=(self.depth_rows, self.depth_cols),
                                        dtype=np.float32)
            if include_rgb:
                d["rgb"] = spaces.Box(0, 255, shape=(self.rgb_h, self.rgb_w, 3),
                                      dtype=np.uint8)
            self.observation_space = spaces.Dict(d)
        else:
            self.observation_space = state_space

        self._last_depth = np.full((self.depth_rows, self.depth_cols), max_range, np.float32)
        self._last_rgb = np.zeros((self.rgb_h, self.rgb_w, 3), np.uint8)

        self.dt = 1.0 / 400.0
        self._steps = 0
        self._ws = None
        self._connect()

    # -- connection -------------------------------------------------------
    def _connect(self) -> None:
        self._ws = websocket.create_connection(self.url, timeout=self.connect_timeout)
        hello = self._recv_any()
        if hello.get("type") != "hello":
            raise RuntimeError(f"expected hello, got {hello!r}")
        self.dt = float(hello.get("dt", self.dt))

    def _send(self, obj: dict) -> None:
        self._ws.send(json.dumps(obj))

    def _recv_any(self) -> dict:
        return json.loads(self._ws.recv())

    def _recv_obs(self) -> dict:
        while True:
            msg = self._recv_any()
            t = msg.get("type")
            if t == "obs":
                return msg
            if t == "error":
                raise RuntimeError(f"server error: {msg.get('message')}")

    # -- gym API ----------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        spawn = options.get("spawn") if options else None
        randomize = options.get("randomize") if options else None
        msg_out = {"type": "reset", "seed": seed, "spawn": spawn, "arm": True}
        if randomize is not None:
            msg_out["randomize"] = randomize
        self._send(msg_out)
        msg = self._recv_obs()
        self._steps = 0
        self._last_depth[:] = self.max_range
        self._last_rgb[:] = 0
        self.task.reset(msg)
        return self._obs(msg), self._info(msg)

    def step(self, action):
        a = np.asarray(action, dtype=np.float32).reshape(-1)
        self._send({
            "type": "action", "mode": "attitude",
            "roll": float(np.clip(a[0], -1, 1)) * self.max_tilt,
            "pitch": float(np.clip(a[1], -1, 1)) * self.max_tilt,
            "yaw_rate": float(np.clip(a[2], -1, 1)) * self.max_yaw_rate,
            "throttle": float(np.clip(a[3], 0, 1)),
        })
        msg = self._recv_obs()
        self._steps += 1
        obs = self._obs(msg)
        reward, terminated = self.task.reward(msg)
        if msg.get("out_of_bounds"):
            terminated = True
        truncated = self._steps >= self.max_steps
        return obs, float(reward), bool(terminated), bool(truncated), self._info(msg)

    def close(self) -> None:
        try:
            if self._ws is not None:
                self._send({"type": "close"})
                self._ws.close()
        except Exception:
            pass
        finally:
            self._ws = None

    # -- observation building --------------------------------------------
    def _state_vec(self, msg: dict) -> np.ndarray:
        gt = msg["gt"]
        if self.state_mode == "gps_denied":
            parts = list(gt["vel"]) + list(gt["euler"]) + list(gt["ang_vel"])
        else:
            parts = list(gt["pos"]) + list(gt["vel"]) + list(gt["euler"]) + list(gt["ang_vel"])
        return np.asarray(parts, dtype=np.float32)

    def _obs(self, msg: dict):
        if self.include_depth and msg.get("depth"):
            d = msg["depth"]
            self._last_depth = np.asarray(d["ranges"], dtype=np.float32).reshape(d["rows"], d["cols"])
        if self.include_rgb and msg.get("camera"):
            c = msg["camera"]
            raw = base64.b64decode(c["data"])
            self._last_rgb = np.frombuffer(raw, dtype=np.uint8).reshape(c["h"], c["w"], 3).copy()

        state = self._state_vec(msg)
        if not (self.include_depth or self.include_rgb):
            return state
        out = {"state": state}
        if self.include_depth:
            out["depth"] = self._last_depth.copy()
        if self.include_rgb:
            out["rgb"] = self._last_rgb.copy()
        return out

    @staticmethod
    def _info(msg: dict) -> dict:
        return {
            "t": msg.get("t"),
            "collision": bool(msg.get("collision", False)),
            "telemetry": msg.get("telemetry", {}),
            "gt_pos": msg.get("gt", {}).get("pos"),
            "dr": msg.get("dr"),
        }
