#!/usr/bin/env python3
"""
build_07_avionics.py
STEP 7: Master Avionics & Navigation System Module Generator (Direct Airframe Integration)
Creates AVIONICS_MASTER.glb containing:
- 6x Flush-Mounted Recessed SMA Sockets seated directly into the drone's curved carbon hull
- 6x Multi-Band Flexible High-Gain Whip Antenna Masts
- Low-Profile Circular GNSS Dome Receiver Pod (seated flush on hull centerline)
- Red Aviation Anti-Collision Strobe Beacon with Low-Profile Circular Bezel
NO square boxes or protruding blocks - 100% aerodynamic organic integration.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_cylinder_mesh, create_sphere_mesh,
    create_oriented_cylinder, combine_meshes, PipelineGLTFBuilder, get_manifest
)
from pygltflib import Node

def build_avionics():
    manifest = get_manifest()
    builder = PipelineGLTFBuilder(root_name="AVIONICS_MASTER")

    metal_parts = []
    beacon_base_parts = []
    beacon_lens_parts = []

    # 1. 6x Multi-Band Whip Antennas directly embedded into the drone's curved body
    # Coordinates exactly matching the upper hull contour:
    antenna_mounts = [
        # (X, Surface_Y, Z, Mast Height, outward tilt dx)
        (-0.045, 0.076, 0.080, 0.100, -0.004),
        ( 0.045, 0.076, 0.080, 0.100,  0.004),
        (-0.055, 0.066, 0.135, 0.120, -0.006),
        ( 0.055, 0.066, 0.135, 0.120,  0.006),
        (-0.040, 0.053, 0.185, 0.090, -0.004),
        ( 0.040, 0.053, 0.185, 0.090,  0.004)
    ]

    for ax, ay, az, ah, dx in antenna_mounts:
        # Sleek circular flush-mounted SMA grommet/bezel seated right into the carbon skin
        metal_parts.append(create_cylinder_mesh(radius=0.0070, height=0.008, segments=16, center_x=ax, center_y=ay + 0.004, center_z=az, axis='y'))
        metal_parts.append(create_cylinder_mesh(radius=0.0055, height=0.006, segments=16, center_x=ax, center_y=ay + 0.009, center_z=az, axis='y'))
        
        # High-Gain Flexible Whip Antenna Mast
        p1 = [ax, ay + 0.012, az]
        p2 = [ax + dx, ay + 0.012 + ah, az]
        metal_parts.append(create_oriented_cylinder(p1, p2, 0.0022, 12))
        metal_parts.append(create_sphere_mesh(radius=0.0032, rings=8, sectors=12, center_x=p2[0], center_y=p2[1], center_z=p2[2]))

    # 2. Low-Profile Circular GNSS Dome Receiver Pod (seated flush on hull centerline)
    gnss_y = 0.070
    gnss_z = 0.120
    metal_parts.append(create_cylinder_mesh(radius=0.022, height=0.006, segments=24, center_x=0.0, center_y=gnss_y + 0.003, center_z=gnss_z, axis='y'))
    metal_parts.append(create_sphere_mesh(radius=0.020, rings=12, sectors=24, center_x=0.0, center_y=gnss_y + 0.006, center_z=gnss_z))

    p_ant, n_ant, i_ant = combine_meshes(metal_parts)
    m_ant_idx = builder.add_mesh(p_ant, n_ant, i_ant, 9, "Avionics_Antennas_Direct")
    n_ant = Node(name="ANTENNA_SYSTEM", mesh=m_ant_idx)
    n_ant_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_ant)
    builder.root_node.children.append(n_ant_idx)

    # 3. Red Anti-Collision Pulsing Strobe Beacon (low-profile circular bezel flush on forward crest)
    strobe_y = 0.080
    strobe_z = 0.025
    beacon_base_parts.append(create_cylinder_mesh(radius=0.016, height=0.006, segments=24, center_x=0.0, center_y=strobe_y + 0.003, center_z=strobe_z, axis='y'))

    p_bb, n_bb, i_bb = combine_meshes(beacon_base_parts)
    m_bb_idx = builder.add_mesh(p_bb, n_bb, i_bb, 9, "Beacon_Base_Housing")
    n_bbase = Node(name="BEACON_BASE", mesh=m_bb_idx)
    n_bbase_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bbase)
    builder.root_node.children.append(n_bbase_idx)

    # Strobe Translucent Dome (Slot 4: MAT_RED_STATUS)
    beacon_lens_parts.append(create_sphere_mesh(radius=0.012, rings=12, sectors=16, center_x=0.0, center_y=strobe_y + 0.008, center_z=strobe_z))
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
