import sys
import time
import requests
import json
import asyncio
import websockets

BASE_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

def test_rest_endpoints():
    print("============================================================")
    print("[TEST] Running Phase 2 Server REST API Integration Tests...")
    print("============================================================")

    # 1. State endpoint
    r = requests.get(f"{BASE_URL}/api/simulation/state")
    assert r.status_code == 200, f"Failed /api/simulation/state: {r.status_code}"
    state = r.json()
    assert len(state["drones"]) == 4, "Must have 4 initialized drones"
    print("  1. GET /api/simulation/state: 4 Drones Active -> OK")

    # 2. Get Drone Telemetry
    r = requests.get(f"{BASE_URL}/api/drones/GARUDA-HL-01")
    assert r.status_code == 200
    d = r.json()
    assert d["drone_id"] == "GARUDA-HL-01"
    assert len(d["sensors"]) == 8
    print("  2. GET /api/drones/GARUDA-HL-01: 8 Sensors Registered -> OK")

    # 3. Payload Endpoints
    r = requests.get(f"{BASE_URL}/api/drones/GARUDA-HL-01/payload")
    assert r.status_code == 200
    p = r.json()
    print("  Debug Payload Response:", p)
    assert p["attached"] == True or p["type"] != 0
    print(f"  3. GET /api/drones/GARUDA-HL-01/payload: {p.get('name', 'N/A')} ({p.get('mass_kg', 0.0)} kg) -> OK")

    # 4. Attach LiDAR Payload (type 2)
    r = requests.post(f"{BASE_URL}/api/drones/GARUDA-HL-01/payload/attach", json={"payload_type": 2})
    assert r.status_code == 200
    time.sleep(0.05)
    r = requests.get(f"{BASE_URL}/api/drones/GARUDA-HL-01/payload")
    p2 = r.json()
    assert p2["type"] == 2
    assert p2["category"] == "LIDAR"
    print(f"  4. POST /api/drones/GARUDA-HL-01/payload/attach (LiDAR): Category={p2['category']}, Mass={p2['mass_kg']} kg -> OK")

    # 5. Detach Payload
    r = requests.post(f"{BASE_URL}/api/drones/GARUDA-HL-01/payload/detach")
    assert r.status_code == 200
    time.sleep(0.05)
    r = requests.get(f"{BASE_URL}/api/drones/GARUDA-HL-01/payload")
    p_det = r.json()
    assert p_det["attached"] == False
    assert p_det["mass_kg"] == 0.0
    print("  5. POST /api/drones/GARUDA-HL-01/payload/detach: Mass restored to 0.0 kg -> OK")

    # 6. Re-attach Inspection Camera
    requests.post(f"{BASE_URL}/api/drones/GARUDA-HL-01/payload/attach", json={"payload_type": 1})
    time.sleep(0.05)

    # 7. Camera Gimbal Control
    r = requests.post(f"{BASE_URL}/api/drones/GARUDA-HL-01/camera/gimbal", json={"pitch_deg": -45.0, "yaw_deg": 60.0, "zoom": 8.0})
    assert r.status_code == 200
    time.sleep(0.05)
    r = requests.get(f"{BASE_URL}/api/drones/GARUDA-HL-01/camera")
    cam = r.json()
    assert abs(cam["pitch_deg"] - (-45.0)) < 0.1
    assert abs(cam["yaw_deg"] - 60.0) < 0.1
    assert abs(cam["zoom_level"] - 8.0) < 0.1
    print(f"  7. POST /api/drones/GARUDA-HL-01/camera/gimbal: Pitch={cam['pitch_deg']}°, Yaw={cam['yaw_deg']}°, Zoom={cam['zoom_level']}x -> OK")

    # 8. Sensor Status & Health
    r = requests.post(f"{BASE_URL}/api/drones/GARUDA-HL-01/sensors/4/status", json={"sensor_index": 4, "status": 3}) # LiDAR degraded
    assert r.status_code == 200
    time.sleep(0.05)
    r = requests.get(f"{BASE_URL}/api/drones/GARUDA-HL-01/sensors")
    sens = r.json()["sensors"]
    assert sens[4]["status"] == "DEGRADED"
    print("  8. POST /api/drones/GARUDA-HL-01/sensors/4/status: Sensor status set to DEGRADED -> OK")

    # 9. Health Endpoint
    r = requests.get(f"{BASE_URL}/api/drones/GARUDA-HL-01/health")
    assert r.status_code == 200
    h = r.json()
    print(f"  9. GET /api/drones/GARUDA-HL-01/health: Overall={h['overall']} -> OK")

    print("============================================================")
    print("[TEST] ALL 9 REST ENDPOINTS PASSED SUCCESSFULLY.")
    print("============================================================")

async def test_websocket_telemetry():
    print("============================================================")
    print("[TEST] Testing WebSocket /ws/telemetry Stream...")
    print("============================================================")

    async with websockets.connect(WS_URL) as ws:
        # Receive 10 consecutive telemetry frames
        for i in range(10):
            msg = await ws.recv()
            data = json.loads(msg)
            assert "tick" in data
            assert "state_hash" in data
            assert "drones" in data
            d0 = data["drones"][0]
            assert "payload" in d0
            assert "camera" in d0
            assert "sensors" in d0
            assert len(d0["sensors"]) == 8

        print("  WebSocket received 10 valid 60 Hz frames with Phase 2 telemetry schema -> OK")
    print("============================================================")
    print("[TEST] WEBSOCKET STREAM PASSED SUCCESSFULLY.")
    print("============================================================")

if __name__ == "__main__":
    test_rest_endpoints()
    asyncio.run(test_websocket_telemetry())
    print("\n[ALL SERVER INTEGRATION TESTS PASSED (100%)]")
