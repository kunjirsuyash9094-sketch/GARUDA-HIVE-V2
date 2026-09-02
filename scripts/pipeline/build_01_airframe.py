#!/usr/bin/env python3
"""
build_01_airframe.py
STEP 1: Central Airframe Module Generator (Exact Blueprint Match)
Creates GARUDA_BODY.glb matching the reference top-view:
- Broad, aggressive octagonal faceted stealth armored fuselage (0.46m W x 0.52m L)
- Tapered aerodynamic nose visor with dual recessed diagonal cyan slit lightguides (\ /) & lower bar (—)
- Raised central maintenance spine with stenciled GARUDA-HL-01 badge plate & 8 perimeter hex service bolts
- Red pulsing anti-collision aviation strobe beacon centered on upper deck
- Left & Right flank diamond mesh ventilation intake louvers (Slot 10: MAT_MESH_LOUVER)
- 8x Heavy CNC Machined Arm-Root Socket Bulkheads & Clamp Collars with dual M4 hex hardware
- Stepped rear avionics bay with 6x vertical SMA telemetry antenna masts and dual GNSS pods
- Lower Equipment Bay with 4x Landing Gear Clevis Blocks & Dual 15mm Payload Rails
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_oriented_cylinder, create_cylinder_mesh, create_box_mesh,
    create_sphere_mesh, combine_meshes, PipelineGLTFBuilder, get_manifest
)
from pygltflib import Node

def create_stealth_faceted_hull():
    """
    Builds a broad, massive, hard-surface faceted armored fuselage matching the blueprint reference.
    10 cross-section slices from Z = -0.270m to +0.270m with 16 radial perimeter vertices.
    """
    # 16-vertex perimeter profiles per Z-slice [X, Y]
    slices = [
        # Slice 0: Front Nose Visor Apex (Z = -0.270m)
        (-0.270, [
            [0.000, 0.015], [0.035, 0.012], [0.060, 0.005], [0.075, -0.012],
            [0.070, -0.028], [0.050, -0.042], [0.025, -0.048], [0.000, -0.050],
            [-0.025, -0.048], [-0.050, -0.042], [-0.070, -0.028], [-0.075, -0.012],
            [-0.060, 0.005], [-0.035, 0.012], [-0.015, 0.015], [0.000, 0.015]
        ]),
        # Slice 1: Sloped Nose Visor Chamfer (Z = -0.200m)
        (-0.200, [
            [0.000, 0.045], [0.060, 0.040], [0.110, 0.022], [0.135, -0.010],
            [0.125, -0.038], [0.090, -0.055], [0.045, -0.065], [0.000, -0.068],
            [-0.045, -0.065], [-0.090, -0.055], [-0.125, -0.038], [-0.135, -0.010],
            [-0.110, 0.022], [-0.060, 0.040], [-0.025, 0.045], [0.000, 0.045]
        ]),
        # Slice 2: Forward Arm Socket Bulkhead (Z = -0.130m)
        (-0.130, [
            [0.000, 0.068], [0.095, 0.062], [0.170, 0.035], [0.205, -0.008],
            [0.185, -0.045], [0.135, -0.065], [0.065, -0.075], [0.000, -0.078],
            [-0.065, -0.075], [-0.135, -0.065], [-0.185, -0.045], [-0.205, -0.008],
            [-0.170, 0.035], [-0.095, 0.062], [-0.040, 0.068], [0.000, 0.068]
        ]),
        # Slice 3: Forward Shoulder Crest (Z = -0.065m)
        (-0.065, [
            [0.000, 0.078], [0.120, 0.072], [0.200, 0.042], [0.228, -0.005],
            [0.205, -0.050], [0.150, -0.070], [0.075, -0.080], [0.000, -0.082],
            [-0.075, -0.080], [-0.150, -0.070], [-0.205, -0.050], [-0.228, -0.005],
            [-0.200, 0.042], [-0.120, 0.072], [-0.050, 0.078], [0.000, 0.078]
        ]),
        # Slice 4: Center Chassis Datum (Z = 0.000m)
        (0.000, [
            [0.000, 0.082], [0.125, 0.076], [0.210, 0.045], [0.235, -0.002],
            [0.210, -0.052], [0.155, -0.072], [0.078, -0.082], [0.000, -0.085],
            [-0.078, -0.082], [-0.155, -0.072], [-0.210, -0.052], [-0.235, -0.002],
            [-0.210, 0.045], [-0.125, 0.076], [-0.055, 0.082], [0.000, 0.082]
        ]),
        # Slice 5: Aft Shoulder Crest (Z = +0.065m)
        (0.065, [
            [0.000, 0.078], [0.120, 0.072], [0.200, 0.042], [0.228, -0.005],
            [0.205, -0.050], [0.150, -0.070], [0.075, -0.080], [0.000, -0.082],
            [-0.075, -0.080], [-0.150, -0.070], [-0.205, -0.050], [-0.228, -0.005],
            [-0.200, 0.042], [-0.120, 0.072], [-0.050, 0.078], [0.000, 0.078]
        ]),
        # Slice 6: Aft Arm Socket Bulkhead (Z = +0.130m)
        (0.130, [
            [0.000, 0.068], [0.095, 0.062], [0.170, 0.035], [0.205, -0.008],
            [0.185, -0.045], [0.135, -0.065], [0.065, -0.075], [0.000, -0.078],
            [-0.065, -0.075], [-0.135, -0.065], [-0.185, -0.045], [-0.205, -0.008],
            [-0.170, 0.035], [-0.095, 0.062], [-0.040, 0.068], [0.000, 0.068]
        ]),
        # Slice 7: Rear Deck Transition (Z = +0.190m)
        (0.190, [
            [0.000, 0.052], [0.075, 0.046], [0.135, 0.025], [0.155, -0.012],
            [0.135, -0.042], [0.095, -0.058], [0.048, -0.068], [0.000, -0.070],
            [-0.048, -0.068], [-0.095, -0.058], [-0.135, -0.042], [-0.155, -0.012],
            [-0.135, 0.025], [-0.075, 0.046], [-0.030, 0.052], [0.000, 0.052]
        ]),
        # Slice 8: Stepped Aft Avionics Bay (Z = +0.240m)
        (0.240, [
            [0.000, 0.032], [0.048, 0.028], [0.085, 0.015], [0.098, -0.018],
            [0.085, -0.038], [0.058, -0.048], [0.028, -0.055], [0.000, -0.058],
            [-0.028, -0.055], [-0.058, -0.048], [-0.085, -0.038], [-0.098, -0.018],
            [-0.085, 0.015], [-0.048, 0.028], [-0.020, 0.032], [0.000, 0.032]
        ]),
        # Slice 9: Tail Exhaust Apex (Z = +0.270m)
        (0.270, [
            [0.000, 0.015], [0.025, 0.012], [0.045, 0.005], [0.052, -0.015],
            [0.045, -0.030], [0.030, -0.038], [0.015, -0.042], [0.000, -0.045],
            [-0.015, -0.042], [-0.030, -0.038], [-0.045, -0.030], [-0.052, -0.015],
            [-0.045, 0.005], [-0.025, 0.012], [-0.010, 0.015], [0.000, 0.015]
        ])
    ]

    positions, normals, indices = [], [], []
    v_per_slice = 16

    for z_pos, profile in slices:
        for x, y in profile:
            positions.append([x, y, z_pos])
            n_vec = np.array([x * 1.5, y * 1.6, z_pos * 0.7], dtype=np.float32)
            n_len = np.linalg.norm(n_vec)
            if n_len > 1e-5: n_vec /= n_len
            normals.append(n_vec.tolist())

    num_slices = len(slices)
    for s in range(num_slices - 1):
        base_cur = s * v_per_slice
        base_next = (s + 1) * v_per_slice
        for i in range(v_per_slice):
            i_next = (i + 1) % v_per_slice
            p0 = base_cur + i
            p1 = base_cur + i_next
            p2 = base_next + i_next
            p3 = base_next + i
            indices.extend([p0, p2, p1, p0, p3, p2])

    # Sharp Nose Apex
    nose_apex = len(positions)
    positions.append([0.0, -0.015, -0.285])
    normals.append([0.0, 0.0, -1.0])
    for i in range(v_per_slice):
        i_next = (i + 1) % v_per_slice
        indices.extend([nose_apex, i, i_next])

    # Sharp Tail Apex
    tail_apex = len(positions)
    positions.append([0.0, -0.015, 0.285])
    normals.append([0.0, 0.0, 1.0])
    tail_base = (num_slices - 1) * v_per_slice
    for i in range(v_per_slice):
        i_next = (i + 1) % v_per_slice
        indices.extend([tail_apex, tail_base + i_next, tail_base + i])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def build_airframe():
    manifest = get_manifest()
    builder = PipelineGLTFBuilder(root_name="GARUDA_BODY")

    carbon_parts = []
    metal_parts = []
    mesh_louver_parts = []
    glow_parts = []
    beacon_parts = []

    # 1. Broad Faceted Stealth Armor Hull (Slot 0: MAT_CARBON_FIBER)
    p_hull, n_hull, i_hull = create_stealth_faceted_hull()
    carbon_parts.append((p_hull, n_hull, i_hull))

    # 2. Raised Central Maintenance Spine & Service Hatch with Beveled Seams
    carbon_parts.append(create_box_mesh(0.180, 0.014, 0.240, 0.0, 0.082, -0.010))
    carbon_parts.append(create_box_mesh(0.140, 0.008, 0.180, 0.0, 0.090, -0.010))
    # Stenciled Branding Nameplate Block (GARUDA-HL-01)
    metal_parts.append(create_box_mesh(0.110, 0.004, 0.040, 0.0, 0.096, -0.060))
    # Perimeter Service Hex Bolts (8x)
    for bx in [-0.075, 0.075]:
        for bz in [-0.090, -0.030, 0.030, 0.090]:
            metal_parts.append(create_cylinder_mesh(radius=0.0035, height=0.006, segments=8, center_x=bx, center_y=0.090, center_z=bz))

    # 3. Flank Diamond Mesh Ventilation Intake Louvers (Slot 10: MAT_MESH_LOUVER)
    # Triangular / diamond perforated mesh panels matching top reference
    mesh_louver_parts.append(create_box_mesh(0.055, 0.004, 0.100, 0.145, 0.068, 0.010))
    mesh_louver_parts.append(create_box_mesh(0.055, 0.004, 0.100, -0.145, 0.068, 0.010))
    metal_parts.append(create_box_mesh(0.062, 0.006, 0.108, 0.145, 0.066, 0.010))
    metal_parts.append(create_box_mesh(0.062, 0.006, 0.108, -0.145, 0.066, 0.010))

    # 4. 8x Heavy CNC Machined Arm-Root Socket Bulkheads & Clamp Collars
    for arm_cfg in manifest.get("arms", []):
        angle_deg = float(arm_cfg["radial_angle_deg"])
        ang_rad = np.deg2rad(angle_deg)
        sin_a, cos_a = np.sin(ang_rad), np.cos(ang_rad)

        # Heavy rectangular socket block emerging from the faceted armor perimeter
        p_socket_start = [0.120 * sin_a, 0.005, 0.120 * cos_a]
        p_socket_end = [0.185 * sin_a, 0.014, 0.185 * cos_a]
        metal_parts.append(create_oriented_cylinder(p_socket_start, p_socket_end, 0.030, 24))

        # Heavy Clamp Flange Ring with dual M4 bolt lugs
        p_flange_s = [0.170 * sin_a, 0.012, 0.170 * cos_a]
        p_flange_e = [0.180 * sin_a, 0.013, 0.180 * cos_a]
        metal_parts.append(create_oriented_cylinder(p_flange_s, p_flange_e, 0.033, 24))

        # Clamp Fastener Lugs with Hex Bolts
        perp_x, perp_z = -cos_a * 0.020, sin_a * 0.020
        metal_parts.append(create_box_mesh(0.012, 0.016, 0.012, 0.175 * sin_a + perp_x, 0.024, 0.175 * cos_a + perp_z))
        metal_parts.append(create_box_mesh(0.012, 0.016, 0.012, 0.175 * sin_a - perp_x, 0.024, 0.175 * cos_a - perp_z))

    # 5. Lower Equipment Bay & 4x Landing Gear Clevis Blocks & Dual 15mm Rails
    carbon_parts.append(create_box_mesh(0.190, 0.024, 0.280, 0.0, -0.068, 0.0))
    metal_parts.append(create_oriented_cylinder([-0.065, -0.082, -0.160], [-0.065, -0.082, 0.160], 0.0075, 16))
    metal_parts.append(create_oriented_cylinder([0.065, -0.082, -0.160], [0.065, -0.082, 0.160], 0.0075, 16))
    metal_parts.append(create_box_mesh(0.160, 0.012, 0.018, 0.0, -0.080, -0.130))
    metal_parts.append(create_box_mesh(0.160, 0.012, 0.018, 0.0, -0.080, 0.130))

    # 4 Heavy CNC Landing Gear Clevis Mount Blocks
    for gx in [-0.109, 0.109]:
        for gz in [-0.150, 0.150]:
            metal_parts.append(create_box_mesh(0.042, 0.024, 0.042, gx, -0.065, gz))
            metal_parts.append(create_cylinder_mesh(radius=0.016, height=0.020, segments=16, center_x=gx, center_y=-0.075, center_z=gz, axis='y'))

    # 6. Recessed Cyan Lightguide Channels matching Reference Blueprint
    # Front Visor Angled Chevrons (\ /)
    glow_parts.append(create_box_mesh(0.065, 0.008, 0.014, 0.060, 0.028, -0.205))
    glow_parts.append(create_box_mesh(0.065, 0.008, 0.014, -0.060, 0.028, -0.205))
    # Front Lower Visor Transverse Bar (—)
    glow_parts.append(create_box_mesh(0.090, 0.006, 0.010, 0.000, -0.012, -0.250))
    # Flank Shoulder Edge Lightguides
    glow_parts.append(create_box_mesh(0.008, 0.006, 0.200, 0.200, 0.042, 0.000))
    glow_parts.append(create_box_mesh(0.008, 0.006, 0.200, -0.200, 0.042, 0.000))

    # 7. Red Pulsing Anti-Collision Beacon (Mounted Centrally on Upper Deck at Z = +0.030m)
    metal_parts.append(create_cylinder_mesh(radius=0.016, height=0.014, segments=20, center_x=0.0, center_y=0.098, center_z=0.030, axis='y'))
    beacon_parts.append(create_sphere_mesh(radius=0.012, rings=12, sectors=16, center_x=0.0, center_y=0.110, center_z=0.030))

    # 8. Rear Deck 6x Telemetry Antenna Array & GNSS Pod
    for ax, az, ah in [(-0.06, 0.14, 0.10), (0.06, 0.14, 0.10), (-0.03, 0.18, 0.12), (0.03, 0.18, 0.12), (-0.08, 0.21, 0.09), (0.08, 0.21, 0.09)]:
        metal_parts.append(create_cylinder_mesh(radius=0.006, height=0.014, segments=12, center_x=ax, center_y=0.078, center_z=az, axis='y'))
        metal_parts.append(create_oriented_cylinder([ax, 0.085, az], [ax, 0.085 + ah, az], 0.0022, 12))

    # Dual GNSS Pod on Rear Center Deck
    metal_parts.append(create_cylinder_mesh(radius=0.024, height=0.012, segments=24, center_x=0.0, center_y=0.086, center_z=0.110, axis='y'))
    metal_parts.append(create_sphere_mesh(radius=0.022, rings=12, sectors=20, center_x=0.0, center_y=0.090, center_z=0.110))

    # Assemble GLTF Nodes
    p1, n1, i1 = combine_meshes(carbon_parts)
    m_carbon_idx = builder.add_mesh(p1, n1, i1, 0, "Airframe_Carbon")
    n_carbon = Node(name="BODY_MAIN", mesh=m_carbon_idx)
    n_carbon_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_carbon)
    builder.root_node.children.append(n_carbon_idx)

    p_met, n_met, i_met = combine_meshes(metal_parts)
    m_met_idx = builder.add_mesh(p_met, n_met, i_met, 9, "Airframe_MetalSockets")
    n_met = Node(name="AIRFRAME_SOCKETS", mesh=m_met_idx)
    n_met_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_met)
    builder.root_node.children.append(n_met_idx)

    p_louver, n_louver, i_louver = combine_meshes(mesh_louver_parts)
    m_louver_idx = builder.add_mesh(p_louver, n_louver, i_louver, 10, "Airframe_MeshLouvers")
    n_louver = Node(name="AIRFRAME_LOUVERS", mesh=m_louver_idx)
    n_louver_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_louver)
    builder.root_node.children.append(n_louver_idx)

    p2, n2, i2 = combine_meshes(glow_parts)
    m_glow_idx = builder.add_mesh(p2, n2, i2, 3, "Airframe_CyanGlow")
    n_glow = Node(name="AIRFRAME_DETAILS", mesh=m_glow_idx)
    n_glow_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_glow)
    builder.root_node.children.append(n_glow_idx)

    p_bc, n_bc, i_bc = combine_meshes(beacon_parts)
    m_bc_idx = builder.add_mesh(p_bc, n_bc, i_bc, 4, "Airframe_RedBeacon")
    n_bc = Node(name="LIGHTS_BEACON", mesh=m_bc_idx)
    n_bc_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bc)
    builder.root_node.children.append(n_bc_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "01_AIRFRAME", "GARUDA_BODY.glb")
    builder.export(out_path)
    print(f"[OK] Airframe compilation complete: {out_path}")

if __name__ == "__main__":
    build_airframe()
