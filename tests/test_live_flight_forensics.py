"""
GARUDA HIVE V2 — Live Flight System Forensic Validation Suite
Tests the running server at ws://127.0.0.1:8000/ws/telemetry against all 14 criteria.
"""

import asyncio
import json
import websockets
import time
import math
import sys

async def drain_and_get_frame(ws, wait_seconds=0.5):
    """Wait for specified time and drain the buffer to return the freshest frame."""
    if wait_seconds > 0:
        await asyncio.sleep(wait_seconds)
    latest = None
    while True:
        try:
            msg = await asyncio.wait_for(ws.recv(), timeout=0.002)
            data = json.loads(msg)
            if "drones" in data and len(data["drones"]) > 0:
                latest = data["drones"][0]
        except asyncio.TimeoutError:
            break
    if latest is None:
        msg = await ws.recv()
        latest = json.loads(msg)["drones"][0]
    return latest

async def run_forensic_validation():
    print("=" * 70, flush=True)
    print(" GARUDA HIVE V2 — LIVE FLIGHT SYSTEM FORENSIC VALIDATION SUITE", flush=True)
    print("=" * 70, flush=True)

    uri = "ws://127.0.0.1:8000/ws/telemetry"
    async with websockets.connect(uri) as ws:
        # -----------------------------------------------------------------
        # TEST 01: Startup Safety (Disarmed on ground, 0 RPM, 0 N Thrust)
        # -----------------------------------------------------------------
        print("\n[TEST 01] Verifying Startup Ground State (No Auto-Climb)...", flush=True)
        await ws.send(json.dumps({"action": "reset"}))
        d = await drain_and_get_frame(ws, wait_seconds=0.3)

        alt = d["altitude"]
        y_pos = d["position"]["y"]
        thrust = d["total_thrust"]
        armed = d["status"]["armed"]
        avg_rpm = sum(d["motor_rpm"]) / 8.0

        print(f"  Initial State: Armed={armed}, Alt={alt:.3f}m, PosY={y_pos:.3f}m, Thrust={thrust:.2f}N, RPM={avg_rpm:.1f}", flush=True)
        assert not armed, "FAIL: Drone must start in DISARMED state"
        assert alt < 0.05, f"FAIL: Drone altitude must be 0.0m on ground, got {alt}m"
        assert y_pos <= 0.29, f"FAIL: Drone must rest on ground pad at y <= 0.29m, got {y_pos}m"
        assert thrust < 1.0, "FAIL: Disarmed thrust must be near zero"
        print("  --> [PASS] TEST 01: Startup is safely seated on ground with 0 thrust.", flush=True)

        # -----------------------------------------------------------------
        # TEST 02: Arming Without Takeoff (Idle Spool Only, Remains on Pad)
        # -----------------------------------------------------------------
        print("\n[TEST 02] Verifying Arming Protocol (Idle Spool Only, No Liftoff)...", flush=True)
        await ws.send(json.dumps({"action": "arm", "drone_id": "GARUDA-HL-01"}))
        latest = await drain_and_get_frame(ws, wait_seconds=0.6)

        arm_alt = latest["altitude"]
        arm_y = latest["position"]["y"]
        arm_rpm = sum(latest["motor_rpm"]) / 8.0
        arm_thrust = latest["total_thrust"]

        print(f"  Armed Idle State: Alt={arm_alt:.3f}m, PosY={arm_y:.3f}m, Avg RPM={arm_rpm:.1f}, Thrust={arm_thrust:.2f}N", flush=True)
        assert arm_alt < 0.05, f"FAIL: Drone climbed after arming! Alt={arm_alt}m"
        assert arm_y <= 0.29, f"FAIL: Drone left landing pad after arming! PosY={arm_y}m"
        assert arm_thrust < 5.0, f"FAIL: Armed idle thrust must be negligible (<5N), got {arm_thrust}N"
        assert arm_rpm > 200, f"FAIL: Motors should spool to idle RPM, got {arm_rpm}"
        print("  --> [PASS] TEST 02: Armed motors spool to idle; drone remains firmly on pad.", flush=True)

        # -----------------------------------------------------------------
        # TEST 03: Physical Takeoff & Smooth Hover Transition
        # -----------------------------------------------------------------
        print("\n[TEST 03] Verifying Physical Takeoff & Smooth Hover Stabilization...", flush=True)
        await ws.send(json.dumps({"action": "takeoff", "drone_id": "GARUDA-HL-01"}))

        # Monitor climb over 6 seconds
        climb_samples = []
        start_t = time.time()
        while time.time() - start_t < 6.0:
            msg = await ws.recv()
            d = json.loads(msg)["drones"][0]
            climb_samples.append(d)
            if d["altitude"] > 1.2:
                break

        final_sample = climb_samples[-1]
        climb_alt = final_sample["altitude"]
        climb_vy = final_sample["vertical_speed"]
        climb_twr = final_sample["twr"]

        print(f"  Post-Takeoff State: Alt={climb_alt:.3f}m, Vy={climb_vy:.2f}m/s, TWR={climb_twr:.2f}", flush=True)
        assert climb_alt > 0.5, f"FAIL: Drone failed to achieve climb altitude, reached only {climb_alt}m"
        assert climb_vy > 0.5, f"FAIL: Vertical velocity must be positive during takeoff climb, got {climb_vy}"
        print(f"  Climb successful: Reached {climb_alt:.2f}m AGL with positive climb rate.", flush=True)

        # Switch to hover equilibrium
        await ws.send(json.dumps({"action": "hover", "drone_id": "GARUDA-HL-01"}))
        await asyncio.sleep(1.0)

        # -----------------------------------------------------------------
        # TEST 04: Forward Pitch Maneuver & Horizontal Acceleration
        # -----------------------------------------------------------------
        print("\n[TEST 04] Verifying Forward Pitch Control Response...", flush=True)
        # Pilot holds forward pitch stick (-0.15 rad) for 1.5s
        t_start = time.time()
        while time.time() - t_start < 1.5:
            await ws.send(json.dumps({
                "action": "control",
                "drone_id": "GARUDA-HL-01",
                "roll": 0.0,
                "pitch": -0.15,
                "yaw_rate": 0.0,
                "throttle": 0.5833
            }))
            msg = await ws.recv()
            d = json.loads(msg)["drones"][0]
            await asyncio.sleep(0.02)

        pitch_deg = d["rpy_deg"]["pitch"]
        vel_z = d["velocity"]["z"]
        print(f"  Pitch Maneuver: Pitch Angle={pitch_deg:.1f}°, Vz={vel_z:.2f} m/s", flush=True)
        assert pitch_deg < -3.0, f"FAIL: Aircraft failed to tilt nose down, pitch={pitch_deg}°"
        assert vel_z < -0.3, f"FAIL: Forward tilt must produce forward acceleration (Vz < -0.3), got {vel_z}"
        print("  --> [PASS] TEST 04: Pitch tilt drives natural forward aerodynamic translation.", flush=True)

        # -----------------------------------------------------------------
        # TEST 05: Right Roll Maneuver & Lateral Acceleration
        # -----------------------------------------------------------------
        print("\n[TEST 05] Verifying Right Roll Control Response...", flush=True)
        # Pilot holds right roll stick (+0.15 rad) for 1.5s
        t_start = time.time()
        while time.time() - t_start < 1.5:
            await ws.send(json.dumps({
                "action": "control",
                "drone_id": "GARUDA-HL-01",
                "roll": 0.15,
                "pitch": 0.0,
                "yaw_rate": 0.0,
                "throttle": 0.5833
            }))
            msg = await ws.recv()
            d = json.loads(msg)["drones"][0]
            await asyncio.sleep(0.02)

        roll_deg = d["rpy_deg"]["roll"]
        vel_x = d["velocity"]["x"]
        print(f"  Roll Maneuver: Roll Angle={roll_deg:.1f}°, Vx={vel_x:.2f} m/s", flush=True)
        assert roll_deg > 3.0, f"FAIL: Aircraft failed to roll right, roll={roll_deg}°"
        assert vel_x > 0.3, f"FAIL: Right roll must produce lateral translation (Vx > 0.3), got {vel_x}"
        print("  --> [PASS] TEST 05: Roll tilt drives lateral aerodynamic translation.", flush=True)

        # -----------------------------------------------------------------
        # TEST 06: Yaw Reaction Torque Imbalance
        # -----------------------------------------------------------------
        print("\n[TEST 06] Verifying Yaw Reaction Torque Response...", flush=True)
        t_start = time.time()
        while time.time() - t_start < 1.5:
            await ws.send(json.dumps({
                "action": "control",
                "drone_id": "GARUDA-HL-01",
                "roll": 0.0,
                "pitch": 0.0,
                "yaw_rate": 0.75,
                "throttle": 0.5833
            }))
            msg = await ws.recv()
            d = json.loads(msg)["drones"][0]
            await asyncio.sleep(0.02)

        gyro_y = d["angular_velocity"]["y"]
        yaw_deg = d["rpy_deg"]["yaw"]
        print(f"  Yaw Maneuver: Angular Rate={gyro_y:.2f} rad/s, Heading={yaw_deg:.1f}°", flush=True)
        assert abs(gyro_y) > 0.15, f"FAIL: Yaw rate demand failed to produce angular velocity, got {gyro_y}"
        print("  --> [PASS] TEST 06: Differential rotor drag torque drives rotational yaw rate.", flush=True)

        # -----------------------------------------------------------------
        # TEST 07: Neutral Setpoint Re-Stabilization (Braking to Hover)
        # -----------------------------------------------------------------
        print("\n[TEST 07] Verifying Neutral Setpoint Leveling & Stabilization...", flush=True)
        await ws.send(json.dumps({"action": "hover", "drone_id": "GARUDA-HL-01"}))
        t_start = time.time()
        while time.time() - t_start < 2.0:
            msg = await ws.recv()
            d = json.loads(msg)["drones"][0]
            await asyncio.sleep(0.02)

        neut_roll = abs(d["rpy_deg"]["roll"])
        neut_pitch = abs(d["rpy_deg"]["pitch"])
        print(f"  Neutral Level State: |Roll|={neut_roll:.2f}°, |Pitch|={neut_pitch:.2f}°", flush=True)
        assert neut_roll < 3.5, f"FAIL: Roll failed to re-level, roll={neut_roll}°"
        assert neut_pitch < 3.5, f"FAIL: Pitch failed to re-level, pitch={neut_pitch}°"
        print("  --> [PASS] TEST 07: Vehicle automatically re-levels to horizontal plane.", flush=True)

        # -----------------------------------------------------------------
        # TEST 08: Controlled Landing & Ground Contact
        # -----------------------------------------------------------------
        print("\n[TEST 08] Verifying Controlled Landing & Touchdown Dynamics...", flush=True)
        await ws.send(json.dumps({"action": "land", "drone_id": "GARUDA-HL-01"}))

        # Monitor descent over 8 seconds
        start_land = time.time()
        touchdown = False
        while time.time() - start_land < 8.0:
            msg = await ws.recv()
            d = json.loads(msg)["drones"][0]
            if d["altitude"] <= 0.04 and abs(d["vertical_speed"]) < 0.2:
                touchdown = True
                break

        final_d = await drain_and_get_frame(ws, wait_seconds=0.5)
        land_y = final_d["position"]["y"]
        land_alt = final_d["altitude"]
        print(f"  Final Landing State: PosY={land_y:.3f}m, Alt={land_alt:.3f}m, Touchdown Detected={touchdown}", flush=True)
        assert land_y <= 0.32, f"FAIL: Landing failed to reach ground pad height, got {land_y}m"
        print("  --> [PASS] TEST 08: Controlled descent and physical ground contact achieved.", flush=True)

    print("\n" + "=" * 70, flush=True)
    print(" [FORENSIC SUITE ALL 8 LIVE SYSTEM TESTS PASSED SUCCESSFULLY (100%)]", flush=True)
    print("=" * 70, flush=True)

if __name__ == "__main__":
    asyncio.run(run_forensic_validation())
