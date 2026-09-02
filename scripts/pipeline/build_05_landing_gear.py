#!/usr/bin/env python3
"""
build_05_landing_gear.py
STEP 5: Master Landing Gear & Ground Stand Module Generator (Unobstructed Camera FOV)
Creates LANDING_GEAR_MASTER.glb matching the reference specification:
- 360mm Ground Clearance to UAV Fuselage Datum (120mm clear space beneath camera)
- Wide Stance (0.560m Track Width, 0.480m Skid Length) with Splayed Carbon A-Frames
- Zero Camera Obstruction: Front hemisphere completely open for 360° pan and -90° to +30° tilt
- 4x Heavy CNC Machined Aluminum Clevis Joints & Quick-Release Pins
- Longitudinal Side Stiffener Braces & Aft Under-Belly Tie Rod
- Twin Carbon Skids with Heavy Rubber Ground Bumper End Caps
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_cylinder_mesh, create_box_mesh, create_sphere_mesh,
    create_oriented_cylinder, combine_meshes, PipelineGLTFBuilder, get_manifest
)
from pygltflib import Node

def build_landing_gear():
    manifest = get_manifest()
    lg_cfg = manifest["landing_gear"]
    skid_y = -float(lg_cfg["ground_clearance_m"]) # -0.360m
    skid_len = 0.480                                # 480mm skid length
    track_w = 0.560                                 # 560mm wide stance
    tube_rad = 0.012                                # 12mm carbon tube radius
    sx = track_w * 0.5                              # 0.280m

    builder = PipelineGLTFBuilder(root_name="LANDING_GEAR_MASTER")

    carbon_parts = []
    metal_parts = []

    # 1. Left and Right Longitudinal Ground Skid Tubes (Y = -0.360m, X = ±0.280m)
    for x_side in [-sx, sx]:
        # Main Skid Tube (Z = -0.240m to +0.240m)
        carbon_parts.append(create_oriented_cylinder([x_side, skid_y, -skid_len * 0.5], [x_side, skid_y, skid_len * 0.5], tube_rad, 24))
        # Rounded Front & Rear Bumper End Caps
        metal_parts.append(create_sphere_mesh(radius=tube_rad * 1.15, rings=12, sectors=16, center_x=x_side, center_y=skid_y, center_z=skid_len * 0.5))
        metal_parts.append(create_sphere_mesh(radius=tube_rad * 1.15, rings=12, sectors=16, center_x=x_side, center_y=skid_y, center_z=-skid_len * 0.5))

        # 2. Splayed Inverted A-Frame Struts (Wide Outward Angle)
        # Front Strut: From Chassis X = ±0.150m, Z = -0.060m -> Skid X = ±0.280m, Z = -0.180m
        top_f = [x_side * 0.54, -0.068, -0.060]
        bot_f = [x_side, skid_y + 0.010, -0.180]
        carbon_parts.append(create_oriented_cylinder(top_f, bot_f, 0.010, 24))
        metal_parts.append(create_box_mesh(0.036, 0.022, 0.036, top_f[0], top_f[1], top_f[2]))
        metal_parts.append(create_cylinder_mesh(radius=0.015, height=0.030, segments=20, center_x=bot_f[0], center_y=bot_f[1], center_z=bot_f[2], axis='z'))

        # Rear Strut: From Chassis X = ±0.150m, Z = +0.160m -> Skid X = ±0.280m, Z = +0.180m
        top_r = [x_side * 0.54, -0.068, 0.160]
        bot_r = [x_side, skid_y + 0.010, 0.180]
        carbon_parts.append(create_oriented_cylinder(top_r, bot_r, 0.010, 24))
        metal_parts.append(create_box_mesh(0.036, 0.022, 0.036, top_r[0], top_r[1], top_r[2]))
        metal_parts.append(create_cylinder_mesh(radius=0.015, height=0.030, segments=20, center_x=bot_r[0], center_y=bot_r[1], center_z=bot_r[2], axis='z'))

        # 3. Longitudinal Side Stiffener Tie-Rod (Connecting Front & Rear Struts on each side at Y = -0.280m)
        p_side_f = [x_side * 0.88, skid_y + 0.080, -0.150]
        p_side_r = [x_side * 0.88, skid_y + 0.080, 0.150]
        carbon_parts.append(create_oriented_cylinder(p_side_f, p_side_r, 0.007, 16))
        metal_parts.append(create_cylinder_mesh(radius=0.010, height=0.016, segments=16, center_x=p_side_f[0], center_y=p_side_f[1], center_z=p_side_f[2], axis='x'))
        metal_parts.append(create_cylinder_mesh(radius=0.010, height=0.016, segments=16, center_x=p_side_r[0], center_y=p_side_r[1], center_z=p_side_r[2], axis='x'))

    # 4. Aft Under-Belly Transverse Tie-Rod (Connecting Left & Right Rear Mounts at Z = +0.160m, BEHIND camera)
    carbon_parts.append(create_oriented_cylinder([-sx * 0.54, -0.076, 0.160], [sx * 0.54, -0.076, 0.160], 0.008, 16))

    # Assemble GLTF Nodes
    p_carb, n_carb, i_carb = combine_meshes(carbon_parts)
    m_carb_idx = builder.add_mesh(p_carb, n_carb, i_carb, 1, "Landing_Gear_CarbonStruts")
    n_carb = Node(name="LANDING_GEAR_CARBON", mesh=m_carb_idx)
    n_carb_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_carb)
    builder.root_node.children.append(n_carb_idx)

    p_met, n_met, i_met = combine_meshes(metal_parts)
    m_met_idx = builder.add_mesh(p_met, n_met, i_met, 9, "Landing_Gear_CNC_Joints")
    n_met = Node(name="LANDING_GEAR_JOINTS", mesh=m_met_idx)
    n_met_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_met)
    builder.root_node.children.append(n_met_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "05_LANDING_GEAR", "LANDING_GEAR_MASTER.glb")
    builder.export(out_path)
    print(f"[OK] Landing Gear Master compilation complete: {out_path}")

if __name__ == "__main__":
    build_landing_gear()
