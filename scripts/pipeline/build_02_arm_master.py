#!/usr/bin/env python3
"""
build_02_arm_master.py
STEP 2: Master Arm Module Generator (Exact Blueprint Match)
Creates ARM_MASTER.glb containing:
- 0.550m High-modulus continuous carbon structural boom tube
- Heavy CNC machined aluminum root socket sleeve with dual M4 fastener clamp lugs
- Mid-span folding mechanism hinge collar & latch lever (X = 0.32m)
- Heavy CNC motor mounting platform & stator bolt circle flange (X = 0.55m)
- Cyan Luminescent Decal Stripe & Markings (Slot 3) matching blueprint reference
- Underside ESC heat sink enclosure with cooling fins & wire routing gland
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_oriented_cylinder, create_cylinder_mesh, create_box_mesh,
    combine_meshes, PipelineGLTFBuilder, get_manifest
)
from pygltflib import Node

def build_arm_master():
    manifest = get_manifest()
    arm_len = float(manifest["dimensions"]["arm_length_m"]) # 0.550m
    tube_rad = float(manifest["arms"][0]["tube_radius_m"])  # 0.015m

    builder = PipelineGLTFBuilder(root_name="ARM_MASTER")

    carbon_parts = []
    metal_parts = []
    glow_parts = []

    # 1. Seamless High-Modulus Carbon Boom Tube (From Root Socket X=0.10m to Motor Mount X=0.55m)
    p_start = [0.100, 0.0, 0.0]
    p_end = [arm_len, 0.0, 0.0]
    carbon_parts.append(create_oriented_cylinder(p_start, p_end, tube_rad, 24))

    # 2. Heavy Square CNC Root Socket Sleeve & Dual Clamp Collar (X = 0.10m to 0.17m)
    metal_parts.append(create_cylinder_mesh(radius=0.021, height=0.060, segments=24, center_x=0.135, center_y=0.0, center_z=0.0, axis='x'))
    metal_parts.append(create_box_mesh(0.055, 0.038, 0.038, 0.135, 0.0, 0.0))
    # Dual Clamp Lugs with M4 Fastener Flanges & Allen Bolts
    metal_parts.append(create_box_mesh(0.014, 0.016, 0.048, 0.115, 0.014, 0.0))
    metal_parts.append(create_box_mesh(0.014, 0.016, 0.048, 0.155, 0.014, 0.0))
    metal_parts.append(create_cylinder_mesh(radius=0.0035, height=0.050, segments=12, center_x=0.115, center_y=0.016, center_z=0.0, axis='z'))
    metal_parts.append(create_cylinder_mesh(radius=0.0035, height=0.050, segments=12, center_x=0.155, center_y=0.016, center_z=0.0, axis='z'))

    # 3. Mid-Span Folding Mechanism Hinge Collar (X = 0.32m)
    metal_parts.append(create_cylinder_mesh(radius=0.0195, height=0.032, segments=24, center_x=0.320, center_y=0.0, center_z=0.0, axis='x'))
    metal_parts.append(create_box_mesh(0.020, 0.024, 0.046, 0.320, 0.010, 0.0))
    # Quick-Release Latch Lever & Pin
    metal_parts.append(create_cylinder_mesh(radius=0.004, height=0.048, segments=12, center_x=0.320, center_y=0.016, center_z=0.0, axis='z'))
    metal_parts.append(create_box_mesh(0.026, 0.006, 0.008, 0.334, 0.022, 0.020))

    # 4. Cyan Luminescent Identification Decal Stripe & Tick Marks (X = 0.36m to 0.48m)
    glow_parts.append(create_box_mesh(0.120, 0.003, 0.008, 0.420, 0.016, 0.0))
    for tx in [0.38, 0.42, 0.46]:
        glow_parts.append(create_box_mesh(0.004, 0.004, 0.020, tx, 0.016, 0.0))

    # 5. Motor Mount Flange at Arm Tip (Centering on X = arm_len = 0.55m)
    metal_parts.append(create_cylinder_mesh(radius=0.040, height=0.008, segments=32, center_x=arm_len, center_y=0.004, center_z=0.0, axis='y'))
    metal_parts.append(create_cylinder_mesh(radius=0.024, height=0.006, segments=24, center_x=arm_len, center_y=0.010, center_z=0.0, axis='y'))
    for angle in [np.pi * 0.25, np.pi * 0.75, np.pi * 1.25, np.pi * 1.75]:
        sx = arm_len + 0.018 * np.cos(angle)
        sz = 0.018 * np.sin(angle)
        metal_parts.append(create_cylinder_mesh(radius=0.0025, height=0.008, segments=8, center_x=sx, center_y=0.012, center_z=sz, axis='y'))

    # Underside ESC Heat Sink Enclosure with Cooling Fins
    carbon_parts.append(create_box_mesh(0.068, 0.016, 0.036, arm_len - 0.035, -0.014, 0.0))
    metal_parts.append(create_box_mesh(0.050, 0.006, 0.030, arm_len - 0.035, -0.024, 0.0))
    metal_parts.append(create_cylinder_mesh(radius=0.006, height=0.012, segments=16, center_x=arm_len - 0.065, center_y=-0.012, center_z=0.0, axis='x'))

    # Assemble GLTF Nodes
    p_carb, n_carb, i_carb = combine_meshes(carbon_parts)
    m_carb_idx = builder.add_mesh(p_carb, n_carb, i_carb, 1, "Arm_CarbonTube")
    n_carb = Node(name="ARM_CARBON", mesh=m_carb_idx)
    n_carb_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_carb)
    builder.root_node.children.append(n_carb_idx)

    p_met, n_met, i_met = combine_meshes(metal_parts)
    m_met_idx = builder.add_mesh(p_met, n_met, i_met, 9, "Arm_MachinedCollars")
    n_met = Node(name="ARM_COLLARS", mesh=m_met_idx)
    n_met_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_met)
    builder.root_node.children.append(n_met_idx)

    p_glow, n_glow, i_glow = combine_meshes(glow_parts)
    m_glow_idx = builder.add_mesh(p_glow, n_glow, i_glow, 3, "Arm_CyanStripe")
    n_glow = Node(name="ARM_CYAN_STRIPE", mesh=m_glow_idx)
    n_glow_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_glow)
    builder.root_node.children.append(n_glow_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "02_ARMS", "ARM_MASTER.glb")
    builder.export(out_path)
    print(f"[OK] Arm Master compilation complete: {out_path}")

if __name__ == "__main__":
    build_arm_master()
