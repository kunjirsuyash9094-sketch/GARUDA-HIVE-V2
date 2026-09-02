#!/usr/bin/env python3
"""
build_07_avionics.py
STEP 7: Master Avionics & Navigation System Module Generator (Reference-Matched)
Creates AVIONICS_MASTER.glb containing:
- 6x Multi-Band High-Gain Whip Antennas with CNC SMA Base Bushings
- Dual Multi-Constellation GNSS Receiver Domes
- High-Intensity Red Anti-Collision Strobe Warning Beacon (Slot 4)
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_cylinder_mesh, create_sphere_mesh, create_oriented_cylinder,
    combine_meshes, PipelineGLTFBuilder, get_manifest
)
from pygltflib import Node

def build_avionics():
    manifest = get_manifest()
    builder = PipelineGLTFBuilder(root_name="AVIONICS_MASTER")

    metal_parts = []
    beacon_base_parts = []
    beacon_lens_parts = []

    # 1. 6x Multi-Band Whip Antennas & Telemetry Masts matching Reference Top-View
    antenna_mounts = [
        (-0.06, 0.088, 0.12, 0.00, 0.10),
        (0.06, 0.088, 0.12, 0.00, 0.10),
        (-0.03, 0.088, 0.16, 0.00, 0.12),
        (0.03, 0.088, 0.16, 0.00, 0.12),
        (-0.07, 0.088, 0.18, -0.02, 0.09),
        (0.07, 0.088, 0.18, 0.02, 0.09)
    ]

    for ax, ay, az, dx, ah in antenna_mounts:
        # SMA Gold/Dark Machined Base Bushing
        metal_parts.append(create_cylinder_mesh(radius=0.006, height=0.014, segments=16, center_x=ax, center_y=ay + 0.007, center_z=az, axis='y'))
        # Flexible Whip Mast
        p1 = [ax, ay + 0.014, az]
        p2 = [ax + dx, ay + 0.014 + ah, az]
        metal_parts.append(create_oriented_cylinder(p1, p2, 0.002, 12))

    # Dual GNSS Receiver Domes
    metal_parts.append(create_cylinder_mesh(radius=0.024, height=0.012, segments=24, center_x=0.0, center_y=0.106, center_z=-0.040, axis='y'))
    metal_parts.append(create_sphere_mesh(radius=0.022, rings=12, sectors=20, center_x=0.0, center_y=0.110, center_z=-0.040))

    p_ant, n_ant, i_ant = combine_meshes(metal_parts)
    m_ant_idx = builder.add_mesh(p_ant, n_ant, i_ant, 9, "Avionics_Antennas")
    n_ant = Node(name="ANTENNA_SYSTEM", mesh=m_ant_idx)
    n_ant_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_ant)
    builder.root_node.children.append(n_ant_idx)

    # 2. Red Anti-Collision Pulsing Strobe Beacon
    beacon_base_parts.append(create_cylinder_mesh(radius=0.016, height=0.014, segments=20, center_x=0.0, center_y=0.108, center_z=0.040, axis='y'))
    p_bb, n_bb, i_bb = combine_meshes(beacon_base_parts)
    m_bb_idx = builder.add_mesh(p_bb, n_bb, i_bb, 9, "Beacon_Base_Housing")
    n_bbase = Node(name="BEACON_BASE", mesh=m_bb_idx)
    n_bbase_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bbase)
    builder.root_node.children.append(n_bbase_idx)

    # High-Transmission Red Optical Glass Dome (Slot 4: RED_STROBE)
    beacon_lens_parts.append(create_sphere_mesh(radius=0.012, rings=12, sectors=16, center_x=0.0, center_y=0.120, center_z=0.040))
    p_bd, n_bd, i_bd = combine_meshes(beacon_lens_parts)
    m_bd_idx = builder.add_mesh(p_bd, n_bd, i_bd, 4, "Beacon_Red_Lens")
    n_bdome = Node(name="LIGHTS_BEACON", mesh=m_bd_idx)
    n_bdome_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bdome)
    n_bbase.children.append(n_bdome_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "07_AVIONICS", "AVIONICS_MASTER.glb")
    builder.export(out_path)
    print(f"[OK] Avionics Master compilation complete: {out_path}")

if __name__ == "__main__":
    build_avionics()
