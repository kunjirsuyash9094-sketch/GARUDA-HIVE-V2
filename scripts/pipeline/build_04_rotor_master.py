#!/usr/bin/env python3
"""
build_04_rotor_master.py
STEP 4: Master Two-Blade Aerofoil Propeller Module Generator (Reference-Matched)
Creates ROTOR_MASTER.glb containing:
- Precision CNC Folding Hub with M6 central lock nut and dual pivot hinge lugs
- BLADE_A: Full physical 3D cambered aerofoil blade with spanwise twist (16° -> 7°),
  wide heavy-lift chord (36mm -> 22mm), and high-visibility tactical white tip stripes.
- BLADE_B: 180° Y-Opposed 3D cambered aerofoil blade.
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_cylinder_mesh, create_box_mesh, create_aerofoil_blade_mesh,
    combine_meshes, PipelineGLTFBuilder, get_manifest
)
from pygltflib import Node

def build_rotor_master():
    manifest = get_manifest()
    r_cfg = manifest["rotors"][0]
    prop_radius = float(r_cfg["radius_m"])          # 0.2032m
    twist_root = float(r_cfg["twist_root_deg"])      # 16.0 deg
    twist_tip = float(r_cfg["twist_tip_deg"])        # 7.0 deg
    hub_radius = float(r_cfg["hub_radius_m"])        # 0.025m

    builder = PipelineGLTFBuilder(root_name="ROTOR_MASTER")

    # 1. Precision CNC Machined Rotor Hub (Slot 9: Dark Metal)
    hub_metal = []
    # Central Shaft Mounting Collar & Hub Body
    hub_metal.append(create_cylinder_mesh(radius=0.016, height=0.012, segments=24, center_x=0.0, center_y=0.006, center_z=0.0, axis='y'))
    # M6 Top Anodized Lock Nut
    hub_metal.append(create_cylinder_mesh(radius=0.008, height=0.006, segments=6, center_x=0.0, center_y=0.015, center_z=0.0, axis='y'))
    # Dual Folding Blade Pivot Yoke Lugs at +/- X
    hub_metal.append(create_box_mesh(0.018, 0.010, 0.018, hub_radius, 0.006, 0.0))
    hub_metal.append(create_box_mesh(0.018, 0.010, 0.018, -hub_radius, 0.006, 0.0))
    # Pivot Pins & Fasteners
    hub_metal.append(create_cylinder_mesh(radius=0.003, height=0.014, segments=16, center_x=hub_radius, center_y=0.006, center_z=0.0, axis='y'))
    hub_metal.append(create_cylinder_mesh(radius=0.003, height=0.014, segments=16, center_x=-hub_radius, center_y=0.006, center_z=0.0, axis='y'))

    p_hub, n_hub, i_hub = combine_meshes(hub_metal)
    m_hub_idx = builder.add_mesh(p_hub, n_hub, i_hub, 9, "Rotor_Hub_Metal")
    n_hub = Node(name="ROTOR_HUB", mesh=m_hub_idx)
    n_hub_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_hub)
    builder.root_node.children.append(n_hub_idx)

    # 2. BLADE_A: Physical Cambered Aerofoil Blade (Root at hub_radius, Tip at prop_radius)
    # Carbon Structural Body (Slot 7: PROPELLER_CARBON)
    p_ba, n_ba, i_ba = create_aerofoil_blade_mesh(span=prop_radius, root_chord=0.036, tip_chord=0.022, max_thick=0.0055, twist_deg=twist_root, tip_white=False)
    m_ba_idx = builder.add_mesh(p_ba, n_ba, i_ba, 7, "Blade_Carbon_Body")

    # Tactical White Tip Stripe (Slot 6: TACTICAL_WHITE)
    p_ta, n_ta, i_ta = create_aerofoil_blade_mesh(span=prop_radius, root_chord=0.036, tip_chord=0.022, max_thick=0.0055, twist_deg=twist_root, tip_white=True)
    m_ta_idx = builder.add_mesh(p_ta, n_ta, i_ta, 6, "Blade_Tip_White")

    n_ba_body = Node(name="BLADE_A_BODY", mesh=m_ba_idx)
    n_ba_body_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_ba_body)

    n_ba_tip = Node(name="BLADE_A_TIP", mesh=m_ta_idx)
    n_ba_tip_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_ba_tip)

    n_blade_a = Node(name="BLADE_A", children=[n_ba_body_idx, n_ba_tip_idx])
    n_blade_a_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_blade_a)
    builder.root_node.children.append(n_blade_a_idx)

    # 3. BLADE_B: 180° Y-Opposed Aerofoil Blade
    n_bb_body = Node(name="BLADE_B_BODY", mesh=m_ba_idx)
    n_bb_body_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bb_body)

    n_bb_tip = Node(name="BLADE_B_TIP", mesh=m_ta_idx)
    n_bb_tip_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bb_tip)

    n_blade_b = Node(
        name="BLADE_B",
        rotation=[0.0, 1.0, 0.0, 0.0], # 180° Y Rotation
        children=[n_bb_body_idx, n_bb_tip_idx]
    )
    n_blade_b_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_blade_b)
    builder.root_node.children.append(n_blade_b_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "04_PROPELLERS", "ROTOR_MASTER.glb")
    builder.export(out_path)
    print(f"[OK] Rotor Master compilation complete: {out_path}")

if __name__ == "__main__":
    build_rotor_master()
