#!/usr/bin/env python3
"""Smoke-test the ArduPilot JSON bridge without running ArduPilot.

Sends one JSON-interface servo packet (magic 18458, hover throttle) to the
sim's UDP port and prints the JSON state line that comes back. Run the Godot
scene with SITLManager.enabled_ardupilot = true first.

Usage: python ap_json_probe.py [host] [port] [count]
"""
import json
import socket
import struct
import sys
import time

host  = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
port  = int(sys.argv[2]) if len(sys.argv) > 2 else 9002
count = int(sys.argv[3]) if len(sys.argv) > 3 else 10

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

MAGIC = 18458
RATE  = 400
pwm   = [1500, 1500, 1500, 1500] + [1000] * 12  # hover-ish on a quad

for frame in range(1, count + 1):
    pkt = struct.pack("<HHI16H", MAGIC, RATE, frame, *pwm)
    sock.sendto(pkt, (host, port))
    try:
        data, addr = sock.recvfrom(4096)
    except socket.timeout:
        print(f"frame {frame}: TIMEOUT — is the Godot scene running with "
              f"enabled_ardupilot on port {port}?")
        sys.exit(1)
    line = data.decode(errors="replace").strip()
    state = json.loads(line)
    print(f"frame {frame}: t={state['timestamp']:.4f}s "
          f"pos={state['position']} att={state['attitude']}")
    time.sleep(0.0025)

print("OK — bridge is replying with lockstep JSON state.")
