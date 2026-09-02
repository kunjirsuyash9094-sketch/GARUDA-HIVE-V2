#!/usr/bin/env python3
"""
validate_built_asset.py
Post-Build Asset Integrity & Hierarchy Validator.
Verifies that the compiled GLTF/GLB models match the C++ authoritative specification manifest 100%.
"""

import os
import sys
import json
import pygltflib

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import get_manifest

def validate_master_glb():
    manifest = get_manifest()
    glb_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "00_MASTER", "GARUDA_HL_01_MASTER.glb")

    print("=================================================================")
    print(" GARUDA-HL-01 GLTF/GLB ASSET INTEGRITY VALIDATOR")
    print(f" Target Model: {glb_path}")
    print("=================================================================")

    if not os.path.exists(glb_path):
        print(f"[!] ERROR: Target GLB file does not exist at {glb_path}")
        return False

    gltf = pygltflib.GLTF2().load(glb_path)
    nodes_by_name = {n.name: n for n in gltf.nodes if n.name}

    tests = []
    
    # Check 1: Root & Subsystem Branches
    req_branches = ["AIRFRAME", "ARMS", "PROPULSION", "LANDING_GEAR", "PAYLOAD", "AVIONICS", "LIGHTS"]
    for b in req_branches:
        passed = b in nodes_by_name
        tests.append((f"Hierarchy Branch: {b}", passed, f"Node '{b}' in GLTF scene graph"))

    # Check 2: 8 Arms
    for i in range(1, 9):
        arm_id = f"ARM_{i:02d}"
        passed = arm_id in nodes_by_name
        tests.append((f"Arm Node: {arm_id}", passed, f"Arm node '{arm_id}' exists"))

    # Check 3: 8 Motors and Radial Positions
    for i in range(8):
        m_id = f"MOTOR_{i+1:02d}"
        m_cfg = manifest["motors"][i]
        exp_pos = [float(x) for x in m_cfg["position"]]
        node = nodes_by_name.get(m_id)
        if node and node.translation:
            meas_pos = [round(x, 4) for x in node.translation]
            pos_match = all(abs(m - e) < 1e-3 for m, e in zip(meas_pos, exp_pos))
            tests.append((f"Motor Transform: {m_id}", pos_match, f"Pos: {meas_pos} vs Exp: {exp_pos}"))
        else:
            tests.append((f"Motor Transform: {m_id}", False, f"Node '{m_id}' missing or untranslated"))

    # Check 4: 8 Rotors
    for i in range(1, 9):
        r_id = f"ROTOR_{i:02d}"
        passed = r_id in nodes_by_name
        tests.append((f"Rotor Node: {r_id}", passed, f"Rotor node '{r_id}' exists"))

    # Check 5: 3-Axis Gimbal Joint Chain
    gimbal_chain = ["PAYLOAD_MOUNT", "GIMBAL_YAW", "GIMBAL_PITCH", "GIMBAL_ROLL", "CAMERA_BODY"]
    for g_id in gimbal_chain:
        passed = g_id in nodes_by_name
        tests.append((f"Gimbal Joint Node: {g_id}", passed, f"Joint '{g_id}' in hierarchy"))

    # Check 6: Avionics & Antennas
    for av_id in ["ANTENNA_SYSTEM", "BEACON_BASE", "LIGHTS_BEACON"]:
        passed = av_id in nodes_by_name
        tests.append((f"Avionics Node: {av_id}", passed, f"Node '{av_id}' in hierarchy"))

    # Summary
    passed_count = sum(1 for _, p, _ in tests if p)
    total_count = len(tests)

    for name, pass_status, detail in tests:
        status_str = "[PASS]" if pass_status else "[FAIL]"
        print(f"{status_str} {name:<30} | {detail}")

    print("=================================================================")
    print(f" ASSET VALIDATION SUMMARY: {passed_count} / {total_count} PASSED")
    print("=================================================================")

    return passed_count == total_count

if __name__ == "__main__":
    success = validate_master_glb()
    if not success:
        sys.exit(1)
