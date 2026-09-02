#!/usr/bin/env python3
"""
run_full_pipeline.py
Executes the full modular UAV 3D asset generation and validation pipeline sequentially.
"""

import os
import subprocess
import sys

def main():
    pipeline_dir = os.path.dirname(os.path.abspath(__file__))
    steps = [
        ("Step 1: Central Airframe", "build_01_airframe.py"),
        ("Step 2: Master Arm Module", "build_02_arm_master.py"),
        ("Step 3: Master Motor Module", "build_03_motor_master.py"),
        ("Step 4: Master Rotor Module", "build_04_rotor_master.py"),
        ("Step 5: Landing Gear Module", "build_05_landing_gear.py"),
        ("Step 6: Payload & Gimbal Module", "build_06_payload_gimbal.py"),
        ("Step 7: Avionics & Navigation Module", "build_07_avionics.py"),
        ("Step 8: Master Mathematical Assembly", "assemble_garuda_master.py"),
        ("Step 9: Asset Integrity Validation", "validate_built_asset.py"),
    ]

    print("=================================================================")
    print(" GARUDA-HL-01 MANIFEST-DRIVEN 3D ASSET PRODUCTION PIPELINE")
    print("=================================================================")

    for name, script in steps:
        script_path = os.path.join(pipeline_dir, script)
        print(f"\n[*] Executing {name} ({script})...")
        res = subprocess.run([sys.executable, script_path], capture_output=True, text=True)
        if res.returncode != 0:
            print(f"[!] ERROR executing {script}:\n{res.stderr}\n{res.stdout}")
            sys.exit(1)
        else:
            print(res.stdout.strip())

    print("\n=================================================================")
    print(" [OK] FULL MODULAR PIPELINE EXECUTION COMPLETED SUCCESSFULLY")
    print("=================================================================")

if __name__ == "__main__":
    main()
