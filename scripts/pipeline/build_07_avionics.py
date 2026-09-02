#!/usr/bin/env python3
"""
build_07_avionics.py
STEP 7: Master Avionics & Navigation System Module Generator (Anchored Solid Deck)
Creates AVIONICS_MASTER.glb containing:
- Wide Solid CNC Machined Avionics Mounting Deck (anchored flush to fuselage hull)
- 6x SMA Antenna Bushings firmly seated on the deck with flexible high-gain whip masts
- Central Multi-Constellation GNSS Dome Receiver Pod
- Red Aviation Anti-Collision Strobe Beacon with CNC Flange Socket
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

def build_avionics():
    manifest = get_manifest()
    builder = PipelineGLTFBuilder(root_name="AVIONICS_MASTER")

    metal_parts = []
    beacon_base_parts = []
    beacon_lens_parts = []

    # 1. Wide Solid CNC Aluminum Mounting Deck Plate (anchored flush on aft fuselage deck)
    # Spans X = -0.090 to +0.090, Z = 0.110 to 0.230, Y = 0.065 to 0.080
    metal_parts.append(create_box_mesh(0.180, 0.015, 0.120, 0.0, 0.0725, 0.170))
    # Beveled Chamfer Step Rim
    metal_parts.append(create_box_mesh(0.160, 0.008, 0.100, 0.0, 0.084, 0.170))

    # 2. 6x SMA Telemetry Antenna Bushings & Masts (firmly seated on the deck)
    antenna_mounts = [
        # (X, Z, Mast Height, outward tilt dx)
        (-0.060, 0.135, 0.100, -0.005),
        (0.060, 0.135, 0.100, 0.005),
        (-0.065, 0.170, 0.120, -0.008),
        (0.065, 0.170, 0.120, 0.008),
        (-0.055, 0.205, 0.090, -0.005),
        (0.055, 0.205, 0.090, 0.005)
    ]

    deck_top_y = 0.088

    for ax, az, ah, dx in antenna_mounts:
        # Threaded SMA Hex Nut Collar (firmly embedded in deck)
        metal_parts.append(create_cylinder_mesh(radius=0.0075, height=0.012, segments=16, center_x=ax, center_y=deck_top_y + 0.006, center_z=az, axis='y'))
        # Knurled Retention Ring
        metal_parts.append(create_cylinder_mesh(radius=0.009, height=0.004, segments=16, center_x=ax, center_y=deck_top_y + 0.002, center_z=az, axis='y'))
        # High-Gain Flexible Whip Antenna Mast
        p1 = [ax, deck_top_y + 0.012, az]
        p2 = [ax + dx, deck_top_y + 0.012 + ah, az]
        metal_parts.append(create_oriented_cylinder(p1, p2, 0.0025, 12))
        # Top Rounded Tip
        metal_parts.append(create_sphere_mesh(radius=0.0035, rings=8, sectors=12, center_x=p2[0], center_y=p2[1], center_z=p2[2]))

    # 3. Central GNSS Dome Receiver Pod (seated in center of deck)
    metal_parts.append(create_cylinder_mesh(radius=0.024, height=0.012, segments=24, center_x=0.0, center_y=deck_top_y + 0.006, center_z=0.155, axis='y'))
    metal_parts.append(create_sphere_mesh(radius=0.022, rings=12, sectors=24, center_x=0.0, center_y=deck_top_y + 0.012, center_z=0.155))

    p_ant, n_ant, i_ant = combine_meshes(metal_parts)
    m_ant_idx = builder.add_mesh(p_ant, n_ant, i_ant, 9, "Avionics_Antennas_Deck")
    n_ant = Node(name="ANTENNA_SYSTEM", mesh=m_ant_idx)
    n_ant_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_ant)
    builder.root_node.children.append(n_ant_idx)

    # 4. Red Anti-Collision Pulsing Strobe Beacon
    beacon_base_parts.append(create_cylinder_mesh(radius=0.018, height=0.014, segments=24, center_x=0.0, center_y=0.098, center_z=0.030, axis='y'))
    beacon_base_parts.append(create_cylinder_mesh(radius=0.021, height=0.004, segments=24, center_x=0.0, center_y=0.093, center_z=0.030, axis='y'))

    p_bb, n_bb, i_bb = combine_meshes(beacon_base_parts)
    m_bb_idx = builder.add_mesh(p_bb, n_bb, i_bb, 9, "Beacon_Base_Housing")
    n_bbase = Node(name="BEACON_BASE", mesh=m_bb_idx)
    n_bbase_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bbase)
    builder.root_node.children.append(n_bbase_idx)

    # Strobe Translucent Dome (Slot 4: MAT_RED_STATUS)
    beacon_lens_parts.append(create_sphere_mesh(radius=0.013, rings=12, sectors=16, center_x=0.0, center_y=0.110, center_z=0.030))
    p_bd, n_bd, i_bd = combine_meshes(beacon_lens_parts)
    m_bd_idx = builder.add_mesh(p_bd, n_bd, i_bd, 4, "Beacon_Dome_Strobe")
    n_bdome = Node(name="LIGHTS_BEACON", mesh=m_bd_idx)
    n_bdome_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bdome)
    n_bbase.children.append(n_bdome_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "07_AVIONICS", "AVIONICS_MASTER.glb")
    builder.export(out_path)
    print(f"[OK] Avionics Master compilation complete: {out_path}")

if __name__ == "__main__":
    build_avionics()
