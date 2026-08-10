"""Reference MOCK agent server (pure Python) speaking the SkySim agent protocol,
including Phase-2 depth + RGB payloads. Executable spec of docs/agent_protocol.md.

    python tests/mock_server.py --port 5557
"""
import argparse
import asyncio
import base64
import json
import math

import numpy as np

try:
    import websockets
except ImportError:
    raise SystemExit("pip install websockets")

DT = 1.0 / 400.0
G = 9.81
HOVER_ACCEL = 2.0 * G
DEPTH_COLS, DEPTH_ROWS = 32, 24
RGB_W, RGB_H = 128, 96
MAX_RANGE = 40.0
CAM_EVERY = 13   # emit sensors every N steps (mimics camera_hz < physics_hz)


class FakeDrone:
    def __init__(self):
        self.reset([0, 2, 0])

    def reset(self, spawn):
        self.p = np.array(spawn, dtype=float)
        self.v = np.zeros(3)
        self.euler = np.zeros(3)
        self.ang = np.zeros(3)
        self.t = 0.0
        self.step_i = 0
        self.dr = None

    def apply_and_step(self, roll, pitch, yaw_rate, throttle):
        az = throttle * HOVER_ACCEL - G
        acc = np.array([G * math.sin(roll), az, G * math.sin(pitch)])
        self.v += acc * DT
        self.v *= 0.999
        self.p += self.v * DT
        if self.p[1] < 0.15:
            self.p[1] = 0.15
            self.v[:] = 0.0
        self.euler = np.array([roll * 0.5, pitch * 0.5, self.euler[2] + yaw_rate * DT])
        self.ang = np.array([0.0, 0.0, yaw_rate])
        self.t += DT
        self.step_i += 1

    def _depth(self):
        # synthetic: a wall dead ahead at ~7 m with an opening on the right half
        r = np.full((DEPTH_ROWS, DEPTH_COLS), MAX_RANGE, dtype=np.float32)
        r[:, : DEPTH_COLS // 2] = 7.0            # obstacle on the left
        classes = np.where(r < MAX_RANGE, 1, 0).astype(int)
        return {
            "cols": DEPTH_COLS, "rows": DEPTH_ROWS, "max_range": MAX_RANGE,
            "ranges": r.reshape(-1).tolist(), "classes": classes.reshape(-1).tolist(),
        }

    def _rgb(self):
        # synthetic gradient RGB, raw base64
        x = np.linspace(0, 255, RGB_W, dtype=np.uint8)
        img = np.zeros((RGB_H, RGB_W, 3), dtype=np.uint8)
        img[..., 0] = x[None, :]
        img[..., 1] = np.linspace(0, 255, RGB_H, dtype=np.uint8)[:, None]
        return {"w": RGB_W, "h": RGB_H, "encoding": "raw_rgb8",
                "data": base64.b64encode(img.tobytes()).decode()}

    def obs(self, with_sensors):
        oob = bool(abs(self.p[0]) > 200 or abs(self.p[2]) > 200 or self.p[1] > 200)
        o = {
            "type": "obs", "t": self.t, "step": self.step_i,
            "gt": {"pos": self.p.tolist(), "vel": self.v.tolist(),
                   "quat": [0, 0, 0, 1], "euler": self.euler.tolist(),
                   "ang_vel": self.ang.tolist()},
            "telemetry": {"altitude": float(self.p[1]), "power_draw": 120.0},
            "camera": self._rgb() if with_sensors else None,
            "depth": self._depth() if with_sensors else None,
            "collision": bool(self.p[1] <= 0.15), "out_of_bounds": oob,
            "dr": self.dr,
        }
        return o


async def handler(ws):
    drone = FakeDrone()
    await ws.send(json.dumps({"type": "hello", "protocol": 1, "physics_hz": 400,
                              "dt": DT, "control_modes": ["attitude", "motors"]}))
    async for raw in ws:
        msg = json.loads(raw)
        t = msg.get("type")
        if t == "reset":
            drone.reset(msg.get("spawn") or [0, 2, 0])
            drone.dr = msg.get("randomize")
        elif t == "action":
            drone.apply_and_step(float(msg.get("roll", 0)), float(msg.get("pitch", 0)),
                                 float(msg.get("yaw_rate", 0)), float(msg.get("throttle", 0)))
        elif t == "close":
            break
        else:
            await ws.send(json.dumps({"type": "error", "message": "unknown"}))
            continue
        await ws.send(json.dumps(drone.obs(drone.step_i % CAM_EVERY == 0)))


async def main(port):
    async with websockets.serve(handler, "127.0.0.1", port):
        print(f"mock server (with perception) on ws://127.0.0.1:{port}")
        await asyncio.Future()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5557)
    asyncio.run(main(ap.parse_args().port))
