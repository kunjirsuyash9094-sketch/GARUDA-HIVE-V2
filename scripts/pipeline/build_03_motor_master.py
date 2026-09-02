#!/usr/bin/env python3
"""
build_03_motor_master.py
STEP 3: Master Motor Propulsion Module Generator (Reference-Matched)
Creates MOTOR_MASTER.glb containing:
- 6215 High-Torque Brushless Motor with CNC Stator Base & 4x M3 Screws
- 12-Slot CNC Anodized Red Aluminum Cooling Ventilation Ring (Slot 2)
- Rotor Bell with Crown Bevel & Hardened Steel Shaft Adapter
- Underside Navigation LED Lens (Slot 8)
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_cylinder_mesh, create_box_mesh, create_sphere_mesh,
    combine_meshes, PipelineGLTFBuilder, get_manifest
)
from pygltflib import Node

def build_motor_master():
    manifest = get_manifest()
    m_radius = float(manifest["motors"][0]["housing_radius_m"])  # 0.034m
    m_height = float(manifest["motors"][0]["housing_height_m"])  # 0.040m

    builder = PipelineGLTFBuilder(root_name="MOTOR_MASTER")

    metal_parts = []
    red_parts = []
    nav_parts = []

    # 1. CNC Machined Stator Base Plate & 4x M3 Screws (Slot 9: DARK_METAL)
    metal_parts.append(create_cylinder_mesh(radius=m_radius, height=0.008, segments=32, center_x=0.0, center_y=0.004, center_z=0.0, axis='y'))
    for angle in [np.pi * 0.25, np.pi * 0.75, np.pi * 1.25, np.pi * 1.75]:
        sx = 0.020 * np.cos(angle)
        sz = 0.020 * np.sin(angle)
        metal_parts.append(create_cylinder_mesh(radius=0.0022, height=0.006, segments=12, center_x=sx, center_y=0.008, center_z=sz, axis='y'))

    # 2. 12-Slot CNC Anodized Red Aluminum Cooling Ring (Slot 2: CNC_RED_ALUMINUM)
    red_parts.append(create_cylinder_mesh(radius=m_radius + 0.002, height=0.010, segments=32, center_x=0.0, center_y=0.018, center_z=0.0, axis='y'))
    for i in range(12):
        angle = (i / 12.0) * 2.0 * np.pi
        rx = (m_radius + 0.001) * np.cos(angle)
        rz = (m_radius + 0.001) * np.sin(angle)
        red_parts.append(create_box_mesh(0.004, 0.009, 0.004, rx, 0.018, rz))

    # 3. Dynamic Outer Rotor Bell & Crown Bevel (Slot 9: DARK_METAL)
    metal_parts.append(create_cylinder_mesh(radius=m_radius - 0.001, height=0.022, segments=32, center_x=0.0, center_y=0.034, center_z=0.0, axis='y'))
    # Top Bell Crown Bevel & Ventilation Cuts
    metal_parts.append(create_cylinder_mesh(radius=m_radius * 0.85, height=0.006, segments=24, center_x=0.0, center_y=0.046, center_z=0.0, axis='y'))
    for i in range(8):
        angle = (i / 8.0) * 2.0 * np.pi
        vx = (m_radius * 0.55) * np.cos(angle)
        vz = (m_radius * 0.55) * np.sin(angle)
        metal_parts.append(create_cylinder_mesh(radius=0.0035, height=0.008, segments=12, center_x=vx, center_y=0.048, center_z=vz, axis='y'))

    # 4. Hardened Steel Central Shaft & Propeller Collar
    metal_parts.append(create_cylinder_mesh(radius=0.005, height=0.018, segments=20, center_x=0.0, center_y=0.058, center_z=0.0, axis='y'))
    metal_parts.append(create_cylinder_mesh(radius=0.010, height=0.006, segments=20, center_x=0.0, center_y=0.052, center_z=0.0, axis='y'))

    # 5. Underside Navigation LED Lens (Slot 8: NAV_GREEN)
    nav_parts.append(create_sphere_mesh(radius=0.006, rings=12, sectors=16, center_x=0.0, center_y=-0.006, center_z=0.0))

    # Assemble GLTF Nodes
    p_met, n_met, i_met = combine_meshes(metal_parts)
    m_met_idx = builder.add_mesh(p_met, n_met, i_met, 9, "Motor_Housing")
    n_met = Node(name="MOTOR_CAN", mesh=m_met_idx)
    n_met_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_met)
    builder.root_node.children.append(n_met_idx)

    p_red, n_red, i_red = combine_meshes(red_parts)
    m_red_idx = builder.add_mesh(p_red, n_red, i_red, 2, "Motor_CNC_Red")
    n_red = Node(name="MOTOR_RING", mesh=m_red_idx)
    n_red_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_red)
    builder.root_node.children.append(n_red_idx)

    p_nav, n_nav, i_nav = combine_meshes(nav_parts)
    m_nav_idx = builder.add_mesh(p_nav, n_nav, i_nav, 8, "Motor_Nav_LED")
    n_nav = Node(name="NAV_LED", mesh=m_nav_idx)
    n_nav_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_nav)
    builder.root_node.children.append(n_nav_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "03_MOTORS", "MOTOR_MASTER.glb")
    builder.export(out_path)
    print(f"[OK] Motor Master compilation complete: {out_path}")

if __name__ == "__main__":
    build_motor_master()
