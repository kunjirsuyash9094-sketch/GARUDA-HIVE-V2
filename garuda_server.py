import os
import sys
import time
import asyncio
import ctypes
from typing import Dict, List, Optional
from pathlib import Path
from pydantic import BaseModel

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

# -----------------------------------------------------------------------------
# C API ctypes Wrapper for garuda_physics.dll
# -----------------------------------------------------------------------------
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    ucrt_bin = Path(r"C:\msys64\ucrt64\bin")
    if ucrt_bin.exists():
        os.add_dll_directory(str(ucrt_bin))

dll_path = Path(__file__).parent / "garuda_physics.dll"
if not dll_path.exists():
    raise RuntimeError(f"Cannot find compiled DLL at {dll_path}")

garuda_lib = ctypes.CDLL(str(dll_path.resolve()))

class GarudaDroneTelemetryPOD(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("drone_id", ctypes.c_char * 32),
        ("tick", ctypes.c_uint64),
        ("time_s", ctypes.c_double),

        # Kinematics
        ("pos_x", ctypes.c_double),
        ("pos_y", ctypes.c_double),
        ("pos_z", ctypes.c_double),
        ("vel_x", ctypes.c_double),
        ("vel_y", ctypes.c_double),
        ("vel_z", ctypes.c_double),
        ("acc_x", ctypes.c_double),
        ("acc_y", ctypes.c_double),
        ("acc_z", ctypes.c_double),
        ("altitude", ctypes.c_double),
        ("ground_speed", ctypes.c_double),
        ("vertical_speed", ctypes.c_double),

        # Attitude
        ("quat_x", ctypes.c_double),
        ("quat_y", ctypes.c_double),
        ("quat_z", ctypes.c_double),
        ("quat_w", ctypes.c_double),
        ("roll_deg", ctypes.c_double),
        ("pitch_deg", ctypes.c_double),
        ("yaw_deg", ctypes.c_double),
        ("gyro_x", ctypes.c_double),
        ("gyro_y", ctypes.c_double),
        ("gyro_z", ctypes.c_double),

        # 8-Rotor Propulsion
        ("total_thrust", ctypes.c_double),
        ("twr", ctypes.c_double),
        ("thrust_margin", ctypes.c_double),
        ("motor_rpm", ctypes.c_double * 8),
        ("motor_thrust", ctypes.c_double * 8),
        ("motor_power", ctypes.c_double * 8),
        ("motor_temp", ctypes.c_double * 8),
        ("motor_health", ctypes.c_int32 * 8),

        # Electrical (6S LiPo)
        ("battery_v_term", ctypes.c_double),
        ("battery_v_ocv", ctypes.c_double),
        ("battery_current", ctypes.c_double),
        ("battery_soc", ctypes.c_double),
        ("battery_power", ctypes.c_double),
        ("battery_temp", ctypes.c_double),
        ("energy_consumed_j", ctypes.c_double),
        ("energy_remaining_j", ctypes.c_double),
        ("cell_v", ctypes.c_double * 6),

        # Modular Payload Subsystem
        ("payload_id", ctypes.c_char * 32),
        ("payload_name", ctypes.c_char * 64),
        ("payload_category", ctypes.c_char * 32),
        ("payload_type", ctypes.c_int32),
        ("payload_mass", ctypes.c_double),
        ("payload_attached", ctypes.c_int32),
        ("payload_power", ctypes.c_double),
        ("payload_state", ctypes.c_int32),
        ("payload_health", ctypes.c_int32),

        # Inspection Camera State
        ("camera_status", ctypes.c_int32),
        ("camera_health", ctypes.c_int32),
        ("camera_pitch", ctypes.c_double),
        ("camera_yaw", ctypes.c_double),
        ("camera_zoom", ctypes.c_double),

        # Extensible Sensor Suite (8 Sensors)
        ("sensor_status", ctypes.c_int32 * 8),
        ("sensor_health", ctypes.c_int32 * 8),
        ("lidar_distance", ctypes.c_double),
        ("prox_distance", ctypes.c_double),

        # Aerodynamics & Environment
        ("air_density", ctypes.c_double),
        ("ground_effect_factor", ctypes.c_double),
        ("vrs_active", ctypes.c_int32),
        ("vrs_severity", ctypes.c_double),

        # Status Flags & Unified Vehicle Health
        ("armed", ctypes.c_int32),
        ("in_ground_contact", ctypes.c_int32),
        ("low_voltage_warn", ctypes.c_int32),
        ("critical_cutoff", ctypes.c_int32),
        ("vehicle_health", ctypes.c_int32),
        ("vehicle_state", ctypes.c_int32),
    ]

# C Function Signatures
garuda_lib.garuda_world_create.argtypes = [ctypes.c_uint64, ctypes.c_double]
garuda_lib.garuda_world_create.restype = ctypes.c_void_p

garuda_lib.garuda_world_destroy.argtypes = [ctypes.c_void_p]
garuda_lib.garuda_world_destroy.restype = None

garuda_lib.garuda_world_reset.argtypes = [ctypes.c_void_p]
garuda_lib.garuda_world_reset.restype = None

garuda_lib.garuda_world_add_drone.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
garuda_lib.garuda_world_add_drone.restype = ctypes.c_int32

garuda_lib.garuda_world_drone_count.argtypes = [ctypes.c_void_p]
garuda_lib.garuda_world_drone_count.restype = ctypes.c_int32

garuda_lib.garuda_world_step.argtypes = [ctypes.c_void_p]
garuda_lib.garuda_world_step.restype = None

garuda_lib.garuda_world_step_n.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
garuda_lib.garuda_world_step_n.restype = None

garuda_lib.garuda_drone_arm.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
garuda_lib.garuda_drone_arm.restype = ctypes.c_int32

garuda_lib.garuda_drone_disarm.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
garuda_lib.garuda_drone_disarm.restype = ctypes.c_int32

garuda_lib.garuda_drone_set_attitude.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
garuda_lib.garuda_drone_set_attitude.restype = ctypes.c_int32

garuda_lib.garuda_drone_set_motors.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int32]
garuda_lib.garuda_drone_set_motors.restype = ctypes.c_int32

garuda_lib.garuda_drone_attach_payload.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32]
garuda_lib.garuda_drone_attach_payload.restype = ctypes.c_int32

garuda_lib.garuda_drone_detach_payload.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
garuda_lib.garuda_drone_detach_payload.restype = ctypes.c_int32

garuda_lib.garuda_drone_set_gimbal.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
garuda_lib.garuda_drone_set_gimbal.restype = ctypes.c_int32

garuda_lib.garuda_drone_set_sensor_enabled.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32, ctypes.c_int32]
garuda_lib.garuda_drone_set_sensor_enabled.restype = ctypes.c_int32

garuda_lib.garuda_drone_set_sensor_status.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32, ctypes.c_int32]
garuda_lib.garuda_drone_set_sensor_status.restype = ctypes.c_int32

garuda_lib.garuda_drone_get_sensor_status.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32]
garuda_lib.garuda_drone_get_sensor_status.restype = ctypes.c_int32

garuda_lib.garuda_drone_get_vehicle_health.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
garuda_lib.garuda_drone_get_vehicle_health.restype = ctypes.c_int32

garuda_lib.garuda_drone_inject_motor_failure.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32, ctypes.c_int32, ctypes.c_double]
garuda_lib.garuda_drone_inject_motor_failure.restype = ctypes.c_int32

garuda_lib.garuda_drone_inject_sensor_failure.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32]
garuda_lib.garuda_drone_inject_sensor_failure.restype = ctypes.c_int32

garuda_lib.garuda_drone_inject_battery_degradation.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double, ctypes.c_double]
garuda_lib.garuda_drone_inject_battery_degradation.restype = ctypes.c_int32

garuda_lib.garuda_drone_reset_failures.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
garuda_lib.garuda_drone_reset_failures.restype = ctypes.c_int32

garuda_lib.garuda_world_get_telemetry.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(GarudaDroneTelemetryPOD)]
garuda_lib.garuda_world_get_telemetry.restype = ctypes.c_int32

garuda_lib.garuda_world_get_state_hash.argtypes = [ctypes.c_void_p]
garuda_lib.garuda_world_get_state_hash.restype = ctypes.c_uint64

garuda_lib.garuda_world_get_tick.argtypes = [ctypes.c_void_p]
garuda_lib.garuda_world_get_tick.restype = ctypes.c_uint64

garuda_lib.garuda_world_get_time.argtypes = [ctypes.c_void_p]
garuda_lib.garuda_world_get_time.restype = ctypes.c_double

SENSOR_NAMES = ["IMU", "GNSS", "BAROMETER", "MAGNETOMETER", "LIDAR", "RGB_CAMERA", "THERMAL_CAMERA", "PROXIMITY"]
SENSOR_STATUS_NAMES = ["OFFLINE", "INITIALIZING", "NOMINAL", "DEGRADED", "FAULT"]
HEALTH_STATE_NAMES = ["NOMINAL", "DEGRADED", "WARNING", "CRITICAL", "FAULT", "OFFLINE"]
PAYLOAD_STATE_NAMES = ["AVAILABLE", "ATTACHING", "ATTACHED", "ACTIVE", "DETACHING", "DETACHED", "FAULT"]

# -----------------------------------------------------------------------------
# Simulation Runtime Manager
# -----------------------------------------------------------------------------
class SimulationRuntimeManager:
    def __init__(self, seed: int = 1000, dt: float = 0.0025):
        self.dt = dt
        self.seed = seed
        self.handle = garuda_lib.garuda_world_create(seed, dt)
        self.is_running = True
        self.speed_multiplier = 1.0
        self.drone_ids: List[str] = []

        # Initialize Canonical GARUDA-HL-01 Heavy-Lift Fleet
        self.add_drone("GARUDA-HL-01", 0.0, 0.28, 0.0)
        self.add_drone("GARUDA-HL-02", 4.0, 0.28, 0.0)
        self.add_drone("GARUDA-HL-03", 0.0, 0.28, 4.0)
        self.add_drone("GARUDA-HL-04", 4.0, 0.28, 4.0)

    def add_drone(self, drone_id: str, x: float, y: float, z: float):
        if drone_id not in self.drone_ids:
            garuda_lib.garuda_world_add_drone(self.handle, drone_id.encode('utf-8'), x, y, z)
            self.drone_ids.append(drone_id)

    def reset(self):
        garuda_lib.garuda_world_reset(self.handle)

    def step(self):
        garuda_lib.garuda_world_step(self.handle)

    def step_n(self, count: int):
        garuda_lib.garuda_world_step_n(self.handle, count)

    def arm(self, drone_id: str):
        garuda_lib.garuda_drone_arm(self.handle, drone_id.encode('utf-8'))

    def disarm(self, drone_id: str):
        garuda_lib.garuda_drone_disarm(self.handle, drone_id.encode('utf-8'))

    def set_control(self, drone_id: str, roll: float, pitch: float, yaw_rate: float, throttle: float):
        garuda_lib.garuda_drone_set_attitude(self.handle, drone_id.encode('utf-8'), roll, pitch, yaw_rate, throttle)

    def attach_payload(self, drone_id: str, payload_type: int) -> bool:
        ok = garuda_lib.garuda_drone_attach_payload(self.handle, drone_id.encode('utf-8'), payload_type)
        return bool(ok)

    def detach_payload(self, drone_id: str) -> bool:
        ok = garuda_lib.garuda_drone_detach_payload(self.handle, drone_id.encode('utf-8'))
        return bool(ok)

    def set_gimbal(self, drone_id: str, pitch_deg: float, yaw_deg: float, zoom: float):
        garuda_lib.garuda_drone_set_gimbal(self.handle, drone_id.encode('utf-8'), pitch_deg, yaw_deg, zoom)

    def set_sensor_status(self, drone_id: str, sensor_idx: int, status: int):
        garuda_lib.garuda_drone_set_sensor_status(self.handle, drone_id.encode('utf-8'), sensor_idx, status)

    def inject_failure(self, drone_id: str, motor_idx: int, failure_type: int):
        garuda_lib.garuda_drone_inject_motor_failure(self.handle, drone_id.encode('utf-8'), motor_idx, failure_type, 0.0)

    def reset_failures(self, drone_id: str):
        garuda_lib.garuda_drone_reset_failures(self.handle, drone_id.encode('utf-8'))

    def get_telemetry(self, drone_id: str) -> Optional[dict]:
        pod = GarudaDroneTelemetryPOD()
        ok = garuda_lib.garuda_world_get_telemetry(self.handle, drone_id.encode('utf-8'), ctypes.byref(pod))
        if not ok:
            return None

        sensors_list = []
        for i in range(8):
            st_code = pod.sensor_status[i]
            hl_code = pod.sensor_health[i]
            sensors_list.append({
                "index": i,
                "name": SENSOR_NAMES[i],
                "status_code": st_code,
                "status": SENSOR_STATUS_NAMES[st_code] if 0 <= st_code < len(SENSOR_STATUS_NAMES) else "UNKNOWN",
                "health_code": hl_code,
                "enabled": st_code != 0
            })

        v_health_code = pod.vehicle_health
        p_state_code = pod.payload_state

        return {
            "drone_id": pod.drone_id.decode('utf-8', errors='ignore'),
            "tick": pod.tick,
            "time_s": round(pod.time_s, 4),
            "position": {"x": round(pod.pos_x, 4), "y": round(pod.pos_y, 4), "z": round(pod.pos_z, 4)},
            "velocity": {"x": round(pod.vel_x, 4), "y": round(pod.vel_y, 4), "z": round(pod.vel_z, 4)},
            "acceleration": {"x": round(pod.acc_x, 4), "y": round(pod.acc_y, 4), "z": round(pod.acc_z, 4)},
            "altitude": round(pod.altitude, 4),
            "ground_speed": round(pod.ground_speed, 4),
            "vertical_speed": round(pod.vertical_speed, 4),
            "orientation": {"x": round(pod.quat_x, 4), "y": round(pod.quat_y, 4), "z": round(pod.quat_z, 4), "w": round(pod.quat_w, 4)},
            "rpy_deg": {"roll": round(pod.roll_deg, 2), "pitch": round(pod.pitch_deg, 2), "yaw": round(pod.yaw_deg, 2)},
            "angular_velocity": {"x": round(pod.gyro_x, 4), "y": round(pod.gyro_y, 4), "z": round(pod.gyro_z, 4)},
            "total_thrust": round(pod.total_thrust, 2),
            "twr": round(pod.twr, 2),
            "thrust_margin": round(pod.thrust_margin, 2),
            "motor_rpm": [round(pod.motor_rpm[i], 1) for i in range(8)],
            "motor_thrust": [round(pod.motor_thrust[i], 2) for i in range(8)],
            "motor_power": [round(pod.motor_power[i], 1) for i in range(8)],
            "motor_temp": [round(pod.motor_temp[i], 1) for i in range(8)],
            "motor_health": [pod.motor_health[i] for i in range(8)],
            "battery": {
                "voltage_terminal": round(pod.battery_v_term, 2),
                "voltage_ocv": round(pod.battery_v_ocv, 2),
                "current_amps": round(pod.battery_current, 2),
                "soc": round(pod.battery_soc, 4),
                "power_w": round(pod.battery_power, 1),
                "temp_c": round(pod.battery_temp, 1),
                "energy_consumed_j": round(pod.energy_consumed_j, 1),
                "energy_remaining_j": round(pod.energy_remaining_j, 1),
                "cells": [round(pod.cell_v[i], 2) for i in range(6)],
            },
            "payload": {
                "id": pod.payload_id.decode('utf-8', errors='ignore'),
                "name": pod.payload_name.decode('utf-8', errors='ignore'),
                "category": pod.payload_category.decode('utf-8', errors='ignore'),
                "type": pod.payload_type,
                "mass_kg": round(pod.payload_mass, 2),
                "attached": bool(pod.payload_attached),
                "power_w": round(pod.payload_power, 1),
                "state_code": p_state_code,
                "state": PAYLOAD_STATE_NAMES[p_state_code] if 0 <= p_state_code < len(PAYLOAD_STATE_NAMES) else "UNKNOWN",
                "health": pod.payload_health,
                "camera_status": pod.camera_status,
                "camera_health": pod.camera_health,
                "gimbal_pitch_deg": round(pod.camera_pitch, 1),
                "gimbal_yaw_deg": round(pod.camera_yaw, 1),
                "zoom_level": round(pod.camera_zoom, 1),
            },
            "camera": {
                "status": pod.camera_status,
                "health": pod.camera_health,
                "pitch_deg": round(pod.camera_pitch, 1),
                "yaw_deg": round(pod.camera_yaw, 1),
                "zoom_level": round(pod.camera_zoom, 1),
            },
            "sensors": sensors_list,
            "lidar_distance_m": round(pod.lidar_distance, 3),
            "proximity_distance_m": round(pod.prox_distance, 3),
            "environment": {
                "air_density": round(pod.air_density, 4),
                "ground_effect_factor": round(pod.ground_effect_factor, 3),
                "vrs_active": bool(pod.vrs_active),
                "vrs_severity": round(pod.vrs_severity, 2)
            },
            "health": {
                "overall_code": v_health_code,
                "overall": HEALTH_STATE_NAMES[v_health_code] if 0 <= v_health_code < len(HEALTH_STATE_NAMES) else "UNKNOWN"
            },
            "status": {
                "armed": bool(pod.armed),
                "in_ground_contact": bool(pod.in_ground_contact),
                "low_voltage_warn": bool(pod.low_voltage_warn),
                "critical_cutoff": bool(pod.critical_cutoff),
                "vehicle_health": v_health_code,
                "vehicle_state": pod.vehicle_state
            }
        }

    def get_world_snapshot(self) -> dict:
        drones_data = []
        for did in self.drone_ids:
            telem = self.get_telemetry(did)
            if telem:
                drones_data.append(telem)
        return {
            "tick": garuda_lib.garuda_world_get_tick(self.handle),
            "time_s": round(garuda_lib.garuda_world_get_time(self.handle), 4),
            "state_hash": hex(garuda_lib.garuda_world_get_state_hash(self.handle)),
            "physics_hz": 400.0,
            "running": self.is_running,
            "speed": self.speed_multiplier,
            "drones": drones_data
        }

sim = SimulationRuntimeManager()

# -----------------------------------------------------------------------------
# Background Physics Engine & Telemetry Broadcaster
# -----------------------------------------------------------------------------
connected_websockets: List[WebSocket] = []

async def physics_engine_loop():
    target_dt = 0.0025
    accumulator = 0.0
    last_time = time.perf_counter()

    while True:
        now = time.perf_counter()
        elapsed = now - last_time
        last_time = now

        if sim.is_running:
            accumulator += elapsed * sim.speed_multiplier
            if accumulator > 0.1:
                accumulator = 0.1
            while accumulator >= target_dt:
                sim.step()
                accumulator -= target_dt

        await asyncio.sleep(0.001)

async def telemetry_broadcast_loop():
    while True:
        if connected_websockets:
            snapshot = sim.get_world_snapshot()
            dead_sockets = []
            for ws in connected_websockets:
                try:
                    await ws.send_json(snapshot)
                except Exception:
                    dead_sockets.append(ws)
            for ds in dead_sockets:
                if ds in connected_websockets:
                    connected_websockets.remove(ds)
        await asyncio.sleep(1.0 / 60.0)

@asynccontextmanager
async def lifespan(app: FastAPI):
    t1 = asyncio.create_task(physics_engine_loop())
    t2 = asyncio.create_task(telemetry_broadcast_loop())
    yield
    t1.cancel()
    t2.cancel()

# -----------------------------------------------------------------------------
# FastAPI Application & REST Endpoints
# -----------------------------------------------------------------------------
app = FastAPI(title="GARUDA HIVE V2 Simulation Server", version="2.2.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class DroneControlPayload(BaseModel):
    roll: float = 0.0
    pitch: float = 0.0
    yaw_rate: float = 0.0
    throttle: float = 0.0

class PayloadAttachRequest(BaseModel):
    payload_type: int = 1 # 1=INSPECTION_CAMERA, 2=LIDAR, 3=THERMAL, 4=MAPPING, 5=CARGO, 6=NDT, 7=RESCUE

class GimbalControlRequest(BaseModel):
    pitch_deg: float = -15.0
    yaw_deg: float = 0.0
    zoom: float = 1.0

class SensorStatusRequest(BaseModel):
    sensor_index: int = 0
    status: int = 2 # 0=OFFLINE, 1=INITIALIZING, 2=NOMINAL, 3=DEGRADED, 4=FAULT

class FailurePayload(BaseModel):
    motor_index: int = 0
    failure_type: int = 2

@app.get("/api/simulation/state")
def get_simulation_state():
    return sim.get_world_snapshot()

@app.post("/api/simulation/start")
def start_simulation():
    sim.is_running = True
    return {"status": "running"}

@app.post("/api/simulation/pause")
def pause_simulation():
    sim.is_running = False
    return {"status": "paused"}

@app.post("/api/simulation/reset")
def reset_simulation():
    sim.reset()
    return {"status": "reset"}

@app.post("/api/simulation/step")
def step_simulation(count: int = 1):
    sim.step_n(count)
    return {"status": "stepped", "count": count}

@app.post("/api/simulation/speed")
def set_simulation_speed(multiplier: float):
    sim.speed_multiplier = max(0.1, min(10.0, multiplier))
    return {"speed_multiplier": sim.speed_multiplier}

@app.get("/api/drones")
def list_drones():
    return {"drone_ids": sim.drone_ids, "count": len(sim.drone_ids)}

@app.get("/api/drones/{drone_id}")
def get_drone(drone_id: str):
    telem = sim.get_telemetry(drone_id)
    if not telem:
        raise HTTPException(status_code=404, detail="Drone not found")
    return telem

@app.post("/api/drones/{drone_id}/arm")
def arm_drone(drone_id: str):
    sim.arm(drone_id)
    return {"drone_id": drone_id, "armed": True}

@app.post("/api/drones/{drone_id}/disarm")
def disarm_drone(drone_id: str):
    sim.disarm(drone_id)
    return {"drone_id": drone_id, "armed": False}

@app.post("/api/drones/{drone_id}/control")
def control_drone(drone_id: str, payload: DroneControlPayload):
    sim.set_control(drone_id, payload.roll, payload.pitch, payload.yaw_rate, payload.throttle)
    return {"status": "command_received"}

@app.get("/api/drones/{drone_id}/payload")
def get_drone_payload(drone_id: str):
    telem = sim.get_telemetry(drone_id)
    if not telem:
        raise HTTPException(status_code=404, detail="Drone not found")
    return telem.get("payload", {})

@app.post("/api/drones/{drone_id}/payload/attach")
def attach_payload_endpoint(drone_id: str, payload: PayloadAttachRequest):
    ok = sim.attach_payload(drone_id, payload.payload_type)
    if not ok:
        raise HTTPException(status_code=400, detail="Failed to attach payload: Configuration exceeds MTOW limit of 15.0 kg or invalid type")
    return {"status": "payload_attached", "type": payload.payload_type}

@app.post("/api/drones/{drone_id}/payload/detach")
def detach_payload_endpoint(drone_id: str):
    ok = sim.detach_payload(drone_id)
    return {"status": "payload_detached", "success": ok}

@app.get("/api/drones/{drone_id}/sensors")
def get_drone_sensors(drone_id: str):
    telem = sim.get_telemetry(drone_id)
    if not telem:
        raise HTTPException(status_code=404, detail="Drone not found")
    return {"sensors": telem.get("sensors", [])}

@app.post("/api/drones/{drone_id}/sensors/{sensor_idx}/status")
def set_sensor_status_endpoint(drone_id: str, sensor_idx: int, payload: SensorStatusRequest):
    sim.set_sensor_status(drone_id, sensor_idx, payload.status)
    return {"status": "sensor_status_updated", "sensor_index": sensor_idx, "new_status": payload.status}

@app.get("/api/drones/{drone_id}/camera")
def get_drone_camera(drone_id: str):
    telem = sim.get_telemetry(drone_id)
    if not telem:
        raise HTTPException(status_code=404, detail="Drone not found")
    return telem.get("camera", {})

@app.post("/api/drones/{drone_id}/camera/gimbal")
def set_gimbal_endpoint(drone_id: str, payload: GimbalControlRequest):
    sim.set_gimbal(drone_id, payload.pitch_deg, payload.yaw_deg, payload.zoom)
    return {"status": "gimbal_updated", "pitch_deg": payload.pitch_deg, "yaw_deg": payload.yaw_deg, "zoom": payload.zoom}

@app.get("/api/drones/{drone_id}/health")
def get_drone_health(drone_id: str):
    telem = sim.get_telemetry(drone_id)
    if not telem:
        raise HTTPException(status_code=404, detail="Drone not found")
    return telem.get("health", {})

@app.post("/api/drones/{drone_id}/failure")
def inject_drone_failure(drone_id: str, payload: FailurePayload):
    sim.inject_failure(drone_id, payload.motor_index, payload.failure_type)
    return {"status": "failure_injected", "motor": payload.motor_index, "type": payload.failure_type}

@app.post("/api/drones/{drone_id}/reset_failures")
def reset_drone_failures(drone_id: str):
    sim.reset_failures(drone_id)
    return {"status": "failures_cleared"}

# -----------------------------------------------------------------------------
# WebSocket Telemetry Stream & Real-Time Action Receiver
# -----------------------------------------------------------------------------
@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.append(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            if "action" in data:
                action = data["action"]
                did = data.get("drone_id", "GARUDA-HL-01")
                if action == "arm":
                    sim.arm(did)
                elif action == "disarm":
                    sim.disarm(did)
                elif action == "control":
                    sim.set_control(did, data.get("roll", 0.0), data.get("pitch", 0.0), data.get("yaw_rate", 0.0), data.get("throttle", 0.0))
                elif action == "takeoff":
                    sim.arm(did)
                    sim.set_control(did, 0.0, 0.0, 0.0, 0.45)
                elif action == "land":
                    sim.set_control(did, 0.0, 0.0, 0.0, 0.18)
                elif action == "attach_payload":
                    sim.attach_payload(did, data.get("payload_type", 1))
                elif action == "detach_payload":
                    sim.detach_payload(did)
                elif action == "set_gimbal":
                    sim.set_gimbal(did, data.get("pitch_deg", -15.0), data.get("yaw_deg", 0.0), data.get("zoom", 1.0))
                elif action == "set_sensor_status":
                    sim.set_sensor_status(did, data.get("sensor_index", 0), data.get("status", 2))
                elif action == "fail_motor":
                    sim.inject_failure(did, data.get("motor_index", 0), 2)
                elif action == "reset_failures":
                    sim.reset_failures(did)
    except WebSocketDisconnect:
        if websocket in connected_websockets:
            connected_websockets.remove(websocket)

# -----------------------------------------------------------------------------
# Web Client Frontend Static Serving (Mounted directly at /)
# -----------------------------------------------------------------------------
web_dir = Path(__file__).parent / "web"
if web_dir.exists():
    app.mount("/", StaticFiles(directory=str(web_dir), html=True), name="web")

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")
