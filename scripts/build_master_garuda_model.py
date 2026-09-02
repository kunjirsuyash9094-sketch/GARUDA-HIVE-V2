#!/usr/bin/env python3
"""
build_master_garuda_model.py
MASTER MECHANICAL 3D ASSET GENERATOR FOR GARUDA-HL-01
Senior UAV 3D & Simulation Asset Pipeline

Authoritative Engineering Specifications:
- Vehicle: GARUDA-HL-01 Heavy-Lift Recon / Utility UAV
- Configuration: 8-Rotor Octo-X (45° angular separation)
- Arm Length: 0.55m (Chassis mounting to motor shaft center)
- Motor-to-Motor Overall Span: 1.10m
- Rotor Center Angles:
    MOTOR_01 = 22.5°,  MOTOR_02 = 67.5°,  MOTOR_03 = 112.5°, MOTOR_04 = 157.5°,
    MOTOR_05 = 202.5°, MOTOR_06 = 247.5°, MOTOR_07 = 292.5°, MOTOR_08 = 337.5°
- 16 Aerodynamic Cambered Folding Propeller Blades with Spanwise Twist & White Tip Stripes
- Rotors Local Axis: Local +Y is rotor shaft axis
- Alternating CW/CCW Rotation:
    ROTOR_01=CW, ROTOR_02=CCW, ROTOR_03=CW, ROTOR_04=CCW,
    ROTOR_05=CW, ROTOR_06=CCW, ROTOR_07=CW, ROTOR_08=CCW
- Continuous Faceted Stealth Diamond Fuselage (Broad shoulders, sloped nose, tapered tail)
- 8x Structural Arm-Root Mounting Collars & Machined Flanges
- Modular 3-Axis Stabilized Military EO/IR Gimbal (Yaw -> Pitch -> Roll -> Camera Body)
- Heavy-Duty Inverted A-Frame Carbon Landing Gear with Ground Contact Skids
- 4x Multi-Band Whip Antennas & Top Red Anti-Collision Strobe Beacon
- Full PBR Stealth Black Carbon, Anodized CNC Red Aluminum, Cyan Lightguides, Optical Glass
"""

import os
import sys
import numpy as np
import pygltflib
from pygltflib import (
    GLTF2, Scene, Node, Mesh, Primitive, Attributes,
    Accessor, BufferView, Buffer, Material, PbrMetallicRoughness
)

# =============================================================================
# Precision Hard-Surface Mesh Geometry Library
# =============================================================================

def create_oriented_cylinder(p_start, p_end, radius, segments=32):
    """Creates a continuous seamless cylindrical tube between two 3D points with caps."""
    p_start = np.array(p_start, dtype=np.float32)
    p_end = np.array(p_end, dtype=np.float32)
    axis = p_end - p_start
    length = float(np.linalg.norm(axis))
    if length < 1e-6:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.uint32)

    dir_vec = axis / length

    arbitrary = np.array([0, 1, 0], dtype=np.float32)
    if abs(np.dot(dir_vec, arbitrary)) > 0.95:
        arbitrary = np.array([1, 0, 0], dtype=np.float32)

    u_vec = np.cross(dir_vec, arbitrary)
    u_vec /= np.linalg.norm(u_vec)
    v_vec = np.cross(dir_vec, u_vec)
    v_vec /= np.linalg.norm(v_vec)

    positions = []
    normals = []
    indices = []

    # Mantle
    for i in range(segments + 1):
        theta = (float(i) / segments) * 2.0 * np.pi
        c, s = np.cos(theta), np.sin(theta)
        radial_norm = c * u_vec + s * v_vec
        radial_offset = radius * radial_norm

        positions.append((p_start + radial_offset).tolist())
        normals.append(radial_norm.tolist())

        positions.append((p_end + radial_offset).tolist())
        normals.append(radial_norm.tolist())

    for i in range(segments):
        idx0 = i * 2
        idx1 = idx0 + 1
        idx2 = idx0 + 2
        idx3 = idx0 + 3
        indices.extend([idx0, idx2, idx1, idx1, idx2, idx3])

    # Start Cap
    cap_start_center = len(positions)
    positions.append(p_start.tolist())
    normals.append((-dir_vec).tolist())
    for i in range(segments):
        theta1 = (float(i) / segments) * 2.0 * np.pi
        theta2 = (float(i + 1) / segments) * 2.0 * np.pi
        v1 = p_start + radius * (np.cos(theta1) * u_vec + np.sin(theta1) * v_vec)
        v2 = p_start + radius * (np.cos(theta2) * u_vec + np.sin(theta2) * v_vec)
        idx_v1 = len(positions)
        positions.extend([v1.tolist(), v2.tolist()])
        normals.extend([(-dir_vec).tolist(), (-dir_vec).tolist()])
        indices.extend([cap_start_center, idx_v1 + 1, idx_v1])

    # End Cap
    cap_end_center = len(positions)
    positions.append(p_end.tolist())
    normals.append(dir_vec.tolist())
    for i in range(segments):
        theta1 = (float(i) / segments) * 2.0 * np.pi
        theta2 = (float(i + 1) / segments) * 2.0 * np.pi
        v1 = p_end + radius * (np.cos(theta1) * u_vec + np.sin(theta1) * v_vec)
        v2 = p_end + radius * (np.cos(theta2) * u_vec + np.sin(theta2) * v_vec)
        idx_v1 = len(positions)
        positions.extend([v1.tolist(), v2.tolist()])
        normals.extend([dir_vec.tolist(), dir_vec.tolist()])
        indices.extend([cap_end_center, idx_v1, idx_v1 + 1])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def create_cylinder_mesh(radius, height, segments=32, center_x=0.0, center_y=0.0, center_z=0.0, axis='y'):
    """Creates a cylinder along the specified axis."""
    hh = height * 0.5
    if axis == 'y':
        p1 = [center_x, center_y - hh, center_z]
        p2 = [center_x, center_y + hh, center_z]
    elif axis == 'x':
        p1 = [center_x - hh, center_y, center_z]
        p2 = [center_x + hh, center_y, center_z]
    elif axis == 'z':
        p1 = [center_x, center_y, center_z - hh]
        p2 = [center_x, center_y, center_z + hh]
    return create_oriented_cylinder(p1, p2, radius, segments)

def create_box_mesh(size_x, size_y, size_z, center_x=0.0, center_y=0.0, center_z=0.0):
    """Creates an indexed box mesh with 6 separate faces and crisp normals."""
    hx, hy, hz = size_x * 0.5, size_y * 0.5, size_z * 0.5
    cx, cy, cz = center_x, center_y, center_z

    positions = []
    normals = []
    indices = []

    faces = [
        ([(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)], (0, 0, 1)),
        ([(hx, -hy, -hz), (-hx, -hy, -hz), (-hx, hy, -hz), (hx, hy, -hz)], (0, 0, -1)),
        ([(-hx, hy, hz), (hx, hy, hz), (hx, hy, -hz), (-hx, hy, -hz)], (0, 1, 0)),
        ([(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)], (0, -1, 0)),
        ([(hx, -hy, hz), (hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz)], (1, 0, 0)),
        ([(-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz), (-hx, hy, -hz)], (-1, 0, 0)),
    ]

    for face_verts, n in faces:
        base_idx = len(positions)
        for v in face_verts:
            positions.append([v[0] + cx, v[1] + cy, v[2] + cz])
            normals.append(list(n))
        indices.extend([base_idx, base_idx + 1, base_idx + 2, base_idx, base_idx + 2, base_idx + 3])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def create_sphere_mesh(radius, rings=16, sectors=24, center_x=0.0, center_y=0.0, center_z=0.0):
    """Creates a UV sphere mesh."""
    positions = []
    normals = []
    indices = []
    cx, cy, cz = center_x, center_y, center_z

    for r in range(rings + 1):
        phi = np.pi * float(r) / float(rings)
        for s in range(sectors + 1):
            theta = 2.0 * np.pi * float(s) / float(sectors)
            x = np.sin(phi) * np.cos(theta)
            y = np.cos(phi)
            z = np.sin(phi) * np.sin(theta)
            positions.append([x * radius + cx, y * radius + cy, z * radius + cz])
            normals.append([x, y, z])

    for r in range(rings):
        for s in range(sectors):
            first = r * (sectors + 1) + s
            second = first + sectors + 1
            indices.extend([first, second, first + 1, second, second + 1, first + 1])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def create_aerofoil_blade_mesh(span, root_chord=0.034, tip_chord=0.020, max_thick=0.0055, twist_deg=12.0, tip_white=False):
    """
    Creates a physical aerodynamic cambered blade with thickness, taper, twist, and airfoil cross-section.
    tip_white: if True, builds the outer 18% white tip segment; if False, builds the main carbon blade body.
    """
    sections = 10
    positions = []
    normals = []
    indices = []

    # Span range for main blade vs tip stripe
    t_start = 0.82 if tip_white else 0.0
    t_end = 1.0 if tip_white else 0.82

    hub_offset = 0.022

    for s in range(sections + 1):
        frac = float(s) / sections
        t = t_start + frac * (t_end - t_start)

        x_span = hub_offset + t * span
        chord = root_chord * (1.0 - t * 0.45)
        thick = max_thick * (1.0 - t * 0.40)
        twist = np.deg2rad(twist_deg * (1.0 - t))

        # 6 Points per Airfoil Station (LE, Upper Camber 1, Upper Camber 2, TE, Lower Camber 2, Lower Camber 1)
        cos_tw, sin_tw = np.cos(twist), np.sin(twist)

        # Base 2D profile coordinates
        pts_2d = [
            (-chord * 0.35, 0.0),                  # 0: Leading Edge
            (-chord * 0.10, thick * 0.90),          # 1: Upper Max Camber
            (chord * 0.25, thick * 0.45),           # 2: Upper Aft
            (chord * 0.65, 0.0),                   # 3: Trailing Edge
            (chord * 0.25, -thick * 0.25),          # 4: Lower Aft
            (-chord * 0.10, -thick * 0.35),         # 5: Lower Fwd
        ]

        sec_base = len(positions)
        for pz, py in pts_2d:
            # Rotate by twist angle
            ry = py * cos_tw - pz * sin_tw
            rz = py * sin_tw + pz * cos_tw
            positions.append([x_span, ry, rz])
            # Approximate normal
            norm = np.array([0.0, py, pz], dtype=np.float32)
            n_len = np.linalg.norm(norm)
            if n_len > 1e-5: norm /= n_len
            else: norm = np.array([0, 1, 0], dtype=np.float32)
            normals.append(norm.tolist())

        if s > 0:
            p_prev = sec_base - 6
            p_curr = sec_base
            for k in range(6):
                k_next = (k + 1) % 6
                indices.extend([p_prev + k, p_curr + k, p_prev + k_next,
                                p_curr + k, p_curr + k_next, p_prev + k_next])

    # End Cap on Blade Tip if outermost segment
    if tip_white:
        tip_base = len(positions) - 6
        tip_center = len(positions)
        positions.append([hub_offset + span, 0.0, 0.0])
        normals.append([1.0, 0.0, 0.0])
        for k in range(6):
            k_next = (k + 1) % 6
            indices.extend([tip_center, tip_base + k, tip_base + k_next])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def create_stealth_diamond_fuselage_mesh():
    """
    Constructs the Faceted Aerodynamic Stealth Diamond Fuselage
    following Blueprint Views 01 (Front), 02 (Back), 03 (Left), 04 (Right), 05 (Top).
    """
    # Longitudinal Cross-Section Slices from Nose (-0.23m) to Tail (+0.23m)
    # 10 Perimeter Vertices per slice:
    # 0: Top-Center, 1: Top-Right, 2: Upper-Right Shoulder, 3: Mid-Right Chime, 4: Lower-Right Belly,
    # 5: Bottom-Center, 6: Lower-Left Belly, 7: Mid-Left Chime, 8: Upper-Left Shoulder, 9: Top-Left
    slices = [
        # Slice 0: Front Nose Intake Wedge (Z = -0.23)
        (-0.23, [
            [0.00, 0.025], [0.035, 0.020], [0.055, 0.005], [0.055, -0.015], [0.035, -0.030],
            [0.00, -0.035], [-0.035, -0.030], [-0.055, -0.015], [-0.055, 0.005], [-0.035, 0.020]
        ]),
        # Slice 1: Forward Canopy Sloped Hood (Z = -0.12)
        (-0.12, [
            [0.00, 0.075], [0.085, 0.065], [0.145, 0.025], [0.145, -0.025], [0.085, -0.055],
            [0.00, -0.060], [-0.085, -0.055], [-0.145, -0.025], [-0.145, 0.025], [-0.085, 0.065]
        ]),
        # Slice 2: Central Broad Shoulder Deck (Z = 0.00)
        (0.00, [
            [0.00, 0.085], [0.105, 0.075], [0.165, 0.030], [0.165, -0.030], [0.095, -0.060],
            [0.00, -0.065], [-0.095, -0.060], [-0.165, -0.030], [-0.165, 0.030], [-0.105, 0.075]
        ]),
        # Slice 3: Aft Avionics & Battery Hatch (Z = +0.12)
        (0.12, [
            [0.00, 0.070], [0.090, 0.060], [0.140, 0.022], [0.140, -0.025], [0.080, -0.050],
            [0.00, -0.055], [-0.080, -0.050], [-0.140, -0.025], [-0.140, 0.022], [-0.090, 0.060]
        ]),
        # Slice 4: Tapered Tail Exhaust Port (Z = +0.23)
        (0.23, [
            [0.00, 0.030], [0.045, 0.025], [0.075, 0.008], [0.075, -0.015], [0.045, -0.035],
            [0.00, -0.038], [-0.045, -0.035], [-0.075, -0.015], [-0.075, 0.008], [-0.045, 0.025]
        ])
    ]

    positions = []
    normals = []
    indices = []

    for z_pos, profile in slices:
        for x, y in profile:
            positions.append([x, y, z_pos])
            norm = np.array([x * 1.2, y * 1.6, z_pos * 0.4], dtype=np.float32)
            n_len = np.linalg.norm(norm)
            if n_len > 1e-5: norm /= n_len
            normals.append(norm.tolist())

    num_slices = len(slices)
    v_per_slice = 10

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

    # Front Nose Apex Cap
    nose_apex = len(positions)
    positions.append([0.0, 0.0, -0.245])
    normals.append([0.0, 0.0, -1.0])
    for i in range(v_per_slice):
        i_next = (i + 1) % v_per_slice
        indices.extend([nose_apex, i, i_next])

    # Rear Tail Apex Cap
    tail_apex = len(positions)
    positions.append([0.0, 0.0, 0.245])
    normals.append([0.0, 0.0, 1.0])
    tail_base = (num_slices - 1) * v_per_slice
    for i in range(v_per_slice):
        i_next = (i + 1) % v_per_slice
        indices.extend([tail_apex, tail_base + i_next, tail_base + i])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def combine_meshes(mesh_list):
    """Combines multiple (positions, normals, indices) tuples into a single indexed mesh."""
    if not mesh_list:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.uint32)

    total_pos = []
    total_norm = []
    total_idx = []
    cur_offset = 0

    for pos, norm, idx in mesh_list:
        if len(pos) == 0: continue
        total_pos.append(pos)
        total_norm.append(norm)
        total_idx.append(idx + cur_offset)
        cur_offset += len(pos)

    return np.vstack(total_pos), np.vstack(total_norm), np.concatenate(total_idx)

# =============================================================================
# Master Model Builder Class
# =============================================================================

class GarudaMasterModelBuilder:
    def __init__(self):
        self.gltf = GLTF2()
        self.gltf.scene = 0
        self.scene = Scene()
        self.gltf.scenes.append(self.scene)

        self.binary_blob = bytearray()
        self._setup_pbr_materials()

        # Root Node: GARUDA_HL_01 (Vehicle Reference Frame / Center of Mass at (0,0,0))
        self.root_node = Node(name="GARUDA_HL_01")
        self.gltf.nodes.append(self.root_node)
        self.scene.nodes.append(0)

        # Primary Architectural Branch Nodes
        self.node_airframe = self._create_sub_branch("AIRFRAME")
        self.node_arms = self._create_sub_branch("ARMS")
        self.node_propulsion = self._create_sub_branch("PROPULSION")
        self.node_landing_gear = self._create_sub_branch("LANDING_GEAR")
        self.node_payload = self._create_sub_branch("PAYLOAD")
        self.node_antennas = self._create_sub_branch("ANTENNAS")
        self.node_lights = self._create_sub_branch("LIGHTS")

    def _create_sub_branch(self, name):
        node = Node(name=name)
        idx = len(self.gltf.nodes)
        self.gltf.nodes.append(node)
        self.root_node.children.append(idx)
        return node

    def _setup_pbr_materials(self):
        """Creates reusable PBR materials for high-fidelity rendering."""
        # 0: MAT_STEALTH_CARBON (Radar Absorbent Matte Black)
        m0 = Material(
            name="MAT_STEALTH_CARBON",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.055, 0.060, 0.075, 1.0],
                metallicFactor=0.90,
                roughnessFactor=0.28
            )
        )
        # 1: MAT_CARBON_TUBE (Structural Carbon Fiber)
        m1 = Material(
            name="MAT_CARBON_TUBE",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.035, 0.040, 0.050, 1.0],
                metallicFactor=0.85,
                roughnessFactor=0.20
            )
        )
        # 2: MAT_CNC_RED_ALUMINUM (Anodized Motor Cooling Rings)
        m2 = Material(
            name="MAT_CNC_RED_ALUMINUM",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.95, 0.05, 0.05, 1.0],
                metallicFactor=0.98,
                roughnessFactor=0.15
            )
        )
        # 3: MAT_CYAN_LIGHTGUIDE (Emissive Cyan HUD Accents)
        m3 = Material(
            name="MAT_CYAN_LIGHTGUIDE",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.0, 0.95, 1.0, 1.0],
                metallicFactor=0.1,
                roughnessFactor=0.1
            ),
            emissiveFactor=[0.0, 0.95, 1.0]
        )
        # 4: MAT_RED_STROBE (Emissive Anti-Collision Strobe)
        m4 = Material(
            name="MAT_RED_STROBE",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[1.0, 0.05, 0.05, 1.0],
                metallicFactor=0.1,
                roughnessFactor=0.1
            ),
            emissiveFactor=[1.0, 0.05, 0.05]
        )
        # 5: MAT_OPTICAL_GLASS (Sensor Lenses)
        m5 = Material(
            name="MAT_OPTICAL_GLASS",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.02, 0.08, 0.16, 1.0],
                metallicFactor=0.98,
                roughnessFactor=0.02
            )
        )
        # 6: MAT_TACTICAL_WHITE (High-Visibility Propeller Stripes)
        m6 = Material(
            name="MAT_TACTICAL_WHITE",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.98, 0.98, 0.98, 1.0],
                metallicFactor=0.2,
                roughnessFactor=0.20
            )
        )
        # 7: MAT_PROPELLER_CARBON (Stealth Black Carbon Blade)
        m7 = Material(
            name="MAT_PROPELLER_CARBON",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.08, 0.09, 0.11, 1.0],
                metallicFactor=0.80,
                roughnessFactor=0.25
            )
        )
        # 8: MAT_NAV_GREEN (Starboard Navigation LED)
        m8 = Material(
            name="MAT_NAV_GREEN",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[0.0, 1.0, 0.35, 1.0],
                metallicFactor=0.1,
                roughnessFactor=0.1
            ),
            emissiveFactor=[0.0, 1.0, 0.35]
        )
        # 9: MAT_NAV_RED (Port Navigation LED)
        m9 = Material(
            name="MAT_NAV_RED",
            pbrMetallicRoughness=PbrMetallicRoughness(
                baseColorFactor=[1.0, 0.05, 0.05, 1.0],
                metallicFactor=0.1,
                roughnessFactor=0.1
            ),
            emissiveFactor=[1.0, 0.05, 0.05]
        )

        self.gltf.materials = [m0, m1, m2, m3, m4, m5, m6, m7, m8, m9]

    def _add_mesh_to_gltf(self, positions, normals, indices, material_idx, name):
        """Encodes geometry into binary buffer and creates a GLTF Mesh."""
        pos_bytes = positions.tobytes()
        norm_bytes = normals.tobytes()
        idx_bytes = indices.tobytes()

        def pad(b):
            r = len(b) % 4
            return b + (b'\x00' * (4 - r) if r != 0 else b'')

        pos_offset = len(self.binary_blob)
        self.binary_blob.extend(pad(pos_bytes))

        norm_offset = len(self.binary_blob)
        self.binary_blob.extend(pad(norm_bytes))

        idx_offset = len(self.binary_blob)
        self.binary_blob.extend(pad(idx_bytes))

        bv_pos_idx = len(self.gltf.bufferViews)
        self.gltf.bufferViews.append(BufferView(
            buffer=0, byteOffset=pos_offset, byteLength=len(pos_bytes), target=34962
        ))

        bv_norm_idx = len(self.gltf.bufferViews)
        self.gltf.bufferViews.append(BufferView(
            buffer=0, byteOffset=norm_offset, byteLength=len(norm_bytes), target=34962
        ))

        bv_idx_idx = len(self.gltf.bufferViews)
        self.gltf.bufferViews.append(BufferView(
            buffer=0, byteOffset=idx_offset, byteLength=len(idx_bytes), target=34963
        ))

        min_pos = positions.min(axis=0).tolist() if len(positions) > 0 else [0, 0, 0]
        max_pos = positions.max(axis=0).tolist() if len(positions) > 0 else [0, 0, 0]

        acc_pos_idx = len(self.gltf.accessors)
        self.gltf.accessors.append(Accessor(
            bufferView=bv_pos_idx, byteOffset=0, componentType=5126, count=len(positions),
            type="VEC3", min=min_pos, max=max_pos
        ))

        acc_norm_idx = len(self.gltf.accessors)
        self.gltf.accessors.append(Accessor(
            bufferView=bv_norm_idx, byteOffset=0, componentType=5126, count=len(normals),
            type="VEC3"
        ))

        acc_idx_idx = len(self.gltf.accessors)
        self.gltf.accessors.append(Accessor(
            bufferView=bv_idx_idx, byteOffset=0, componentType=5125, count=len(indices),
            type="SCALAR"
        ))

        prim = Primitive(
            attributes=Attributes(POSITION=acc_pos_idx, NORMAL=acc_norm_idx),
            indices=acc_idx_idx,
            material=material_idx
        )

        mesh_idx = len(self.gltf.meshes)
        self.gltf.meshes.append(Mesh(name=name, primitives=[prim]))
        return mesh_idx

    # =========================================================================
    # 1. Central Airframe (Faceted Diamond Fuselage + Avionics Deck)
    # =========================================================================
    def build_airframe(self):
        carbon_parts = []
        glow_parts = []

        # 1. BODY_CORE & Faceted Stealth Diamond Hull
        p_hull, n_hull, i_hull = create_stealth_diamond_fuselage_mesh()
        carbon_parts.append((p_hull, n_hull, i_hull))

        # 2. BODY_UPPER: Elevated Central Avionics Cooling Deck
        carbon_parts.append(create_box_mesh(0.18, 0.022, 0.22, 0.0, 0.092, -0.01))
        # Chamfered Upper Lid
        carbon_parts.append(create_box_mesh(0.14, 0.012, 0.18, 0.0, 0.106, -0.01))

        # 3. BODY_LOWER: Underside Equipment Bay & Payload Rail
        carbon_parts.append(create_box_mesh(0.16, 0.020, 0.24, 0.0, -0.065, 0.0))
        carbon_parts.append(create_oriented_cylinder([-0.07, -0.075, -0.12], [-0.07, -0.075, 0.12], 0.006, 16))
        carbon_parts.append(create_oriented_cylinder([0.07, -0.075, -0.12], [0.07, -0.075, 0.12], 0.006, 16))

        # 4. Cyan Lightguides on Nose & Shoulders
        glow_parts.append(create_box_mesh(0.06, 0.006, 0.012, 0.05, 0.035, -0.18))
        glow_parts.append(create_box_mesh(0.06, 0.006, 0.012, -0.05, 0.035, -0.18))
        glow_parts.append(create_box_mesh(0.008, 0.006, 0.16, 0.16, 0.045, 0.0))
        glow_parts.append(create_box_mesh(0.008, 0.006, 0.16, -0.16, 0.045, 0.0))

        # Attach Airframe Meshes
        p1, n1, i1 = combine_meshes(carbon_parts)
        m_carbon_idx = self._add_mesh_to_gltf(p1, n1, i1, 0, "Airframe_Carbon")
        n_body = Node(name="BODY_CORE", mesh=m_carbon_idx)
        n_body_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_body)
        self.node_airframe.children.append(n_body_idx)

        p2, n2, i2 = combine_meshes(glow_parts)
        m_glow_idx = self._add_mesh_to_gltf(p2, n2, i2, 3, "Airframe_CyanGlow")
        n_glow = Node(name="LIGHTS_CYAN_GUIDES", mesh=m_glow_idx)
        n_glow_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_glow)
        self.node_lights.children.append(n_glow_idx)

    # =========================================================================
    # 2. 8-Arm Carbon Frame with Structural Arm Roots & Collars
    # =========================================================================
    def build_arms(self):
        """
        Builds 8 independent arm nodes (ARM_01 through ARM_08) with:
        - Reinforced fuselage socket root
        - Machined clamp collar
        - 0.55m seamless carbon fiber structural tube
        - Motor mounting flange & ESC housing
        """
        arm_angles = [
            np.deg2rad(22.5),  np.deg2rad(67.5),  np.deg2rad(112.5), np.deg2rad(157.5),
            np.deg2rad(202.5), np.deg2rad(247.5), np.deg2rad(292.5), np.deg2rad(337.5)
        ]
        arm_len = 0.55 # Exact Authoritative Arm Length

        for i, ang in enumerate(arm_angles):
            arm_num = i + 1
            mx = float(np.sin(ang) * arm_len)
            mz = float(np.cos(ang) * arm_len)
            my = 0.015

            carbon_parts = []

            # 1. Structural Arm Root Socket (emerging from central fuselage)
            p_socket = [mx * 0.18, my, mz * 0.18]
            carbon_parts.append(create_box_mesh(0.045, 0.036, 0.045, p_socket[0], p_socket[1], p_socket[2]))

            # 2. Machined Aluminum Clamp Collar
            p_collar = [mx * 0.26, my, mz * 0.26]
            carbon_parts.append(create_cylinder_mesh(0.020, 0.030, 24, p_collar[0], p_collar[1], p_collar[2]))

            # 3. Seamless Continuous Carbon Boom Arm Tube (radius 15mm)
            p_start = [mx * 0.22, my, mz * 0.22]
            p_end = [mx, my, mz]
            carbon_parts.append(create_oriented_cylinder(p_start, p_end, 0.015, 24))

            # 4. Motor Mount Flange & ESC Housing at Arm Tip
            carbon_parts.append(create_cylinder_mesh(0.038, 0.016, 24, mx, my - 0.008, mz))
            carbon_parts.append(create_box_mesh(0.030, 0.014, 0.045, mx * 0.92, my - 0.015, mz * 0.92))

            p, n, idx = combine_meshes(carbon_parts)
            m_arm_idx = self._add_mesh_to_gltf(p, n, idx, 1, f"Arm_Mesh_{arm_num:02d}")
            n_arm = Node(name=f"ARM_{arm_num:02d}", mesh=m_arm_idx)
            n_arm_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_arm)
            self.node_arms.children.append(n_arm_idx)

    # =========================================================================
    # 3. Propulsion System (8 Separate Motors & 16 Aerodynamic Propeller Blades)
    # =========================================================================
    def build_propulsion(self):
        """
        Builds MOTOR_01 through MOTOR_08:
        - Brushless Motor Can (Stator + Rotor + CNC Red Ring + Shaft)
        - ROTOR_01 through ROTOR_08 node with origin EXACTLY at motor shaft center
        - Local +Y rotation axis
        - 2 physical aerofoil blades per rotor with camber, spanwise twist, and white tip stripes
        """
        arm_angles = [
            np.deg2rad(22.5),  np.deg2rad(67.5),  np.deg2rad(112.5), np.deg2rad(157.5),
            np.deg2rad(202.5), np.deg2rad(247.5), np.deg2rad(292.5), np.deg2rad(337.5)
        ]
        arm_len = 0.55
        blade_span = 0.205 # 205mm radius -> 410mm (16.2 inch) diameter -> zero overlap

        for i, ang in enumerate(arm_angles):
            num = i + 1
            mx = float(np.sin(ang) * arm_len)
            mz = float(np.cos(ang) * arm_len)
            my = 0.015

            # --- A. MOTOR HOUSING (Stator + Bell + Red CNC Anodized Ring) ---
            motor_carbon = []
            motor_red = []
            motor_nav = []

            # Stator Base (Local Y = 0.01)
            motor_carbon.append(create_cylinder_mesh(0.034, 0.020, 24, 0.0, 0.010, 0.0))
            # Rotor Bell (Local Y = 0.032)
            motor_carbon.append(create_cylinder_mesh(0.032, 0.026, 24, 0.0, 0.032, 0.0))
            # Signature CNC Anodized Red Cooling Ring
            motor_red.append(create_cylinder_mesh(0.034, 0.007, 24, 0.0, 0.030, 0.0))
            # Motor Shaft Stub
            motor_carbon.append(create_cylinder_mesh(0.008, 0.022, 20, 0.0, 0.050, 0.0))

            # Navigation LED under motor pod (Right=Green, Left=Red)
            is_right = mx > 0.0
            motor_nav.append(create_sphere_mesh(0.008, 12, 16, 0.0, -0.020, 0.0))

            p_mc, n_mc, i_mc = combine_meshes(motor_carbon)
            m_mc_idx = self._add_mesh_to_gltf(p_mc, n_mc, i_mc, 1, f"Motor_Can_{num:02d}")
            n_mcan = Node(name=f"MOTOR_CAN_{num:02d}", mesh=m_mc_idx)
            n_mcan_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_mcan)

            p_mr, n_mr, i_mr = combine_meshes(motor_red)
            m_mr_idx = self._add_mesh_to_gltf(p_mr, n_mr, i_mr, 2, f"Motor_RedRing_{num:02d}")
            n_mred = Node(name=f"MOTOR_RING_{num:02d}", mesh=m_mr_idx)
            n_mred_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_mred)

            p_nav, n_nav, i_nav = combine_meshes(motor_nav)
            mat_nav_idx = 8 if is_right else 9
            m_nav_idx = self._add_mesh_to_gltf(p_nav, n_nav, i_nav, mat_nav_idx, f"Nav_LED_{num:02d}")
            n_nav_node = Node(name=f"NAV_LED_{num:02d}", mesh=m_nav_idx)
            n_nav_node_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_nav_node)

            # --- B. ROTOR & 2 PHYSICAL AEROFOIL BLADES (BLADE_A & BLADE_B) ---
            # Rotor Hub (at local 0,0,0 of ROTOR node)
            hub_carbon = []
            hub_carbon.append(create_cylinder_mesh(0.018, 0.012, 20, 0.0, 0.0, 0.0))
            hub_carbon.append(create_cylinder_mesh(0.005, 0.016, 16, 0.0, 0.008, 0.0))
            p_hub, n_hub, i_hub = combine_meshes(hub_carbon)
            m_hub_idx = self._add_mesh_to_gltf(p_hub, n_hub, i_hub, 7, f"Rotor_Hub_{num:02d}")
            n_hub = Node(name=f"ROTOR_HUB_{num:02d}", mesh=m_hub_idx)
            n_hub_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_hub)

            # BLADE_A (Extends in +X with aerofoil camber)
            p_ba, n_ba, i_ba = create_aerofoil_blade_mesh(blade_span, tip_white=False)
            m_ba_idx = self._add_mesh_to_gltf(p_ba, n_ba, i_ba, 7, f"Blade_Carbon_Mesh_{num:02d}")

            p_ta, n_ta, i_ta = create_aerofoil_blade_mesh(blade_span, tip_white=True)
            m_ta_idx = self._add_mesh_to_gltf(p_ta, n_ta, i_ta, 6, f"Blade_Tip_Mesh_{num:02d}")

            n_ba_body = Node(name="BLADE_A_BODY", mesh=m_ba_idx)
            n_ba_body_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_ba_body)

            n_ba_tip = Node(name="BLADE_A_TIP", mesh=m_ta_idx)
            n_ba_tip_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_ba_tip)

            n_blade_a = Node(name="BLADE_A", children=[n_ba_body_idx, n_ba_tip_idx])
            n_blade_a_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_blade_a)

            # BLADE_B (Extends in -X with separate node instances sharing the mesh)
            n_bb_body = Node(name="BLADE_B_BODY", mesh=m_ba_idx)
            n_bb_body_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_bb_body)

            n_bb_tip = Node(name="BLADE_B_TIP", mesh=m_ta_idx)
            n_bb_tip_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_bb_tip)

            n_blade_b = Node(
                name="BLADE_B",
                rotation=[0.0, 1.0, 0.0, 0.0], # 180° Y rotation quaternion [x, y, z, w]
                children=[n_bb_body_idx, n_bb_tip_idx]
            )
            n_blade_b_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_blade_b)

            # ROTOR node: origin at motor shaft center (0, 0.058, 0) relative to motor
            # Documented Local Rotation Axis: Local +Y
            n_rotor = Node(
                name=f"ROTOR_{num:02d}",
                translation=[0.0, 0.058, 0.0],
                children=[n_hub_idx, n_blade_a_idx, n_blade_b_idx]
            )
            n_rotor_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_rotor)

            # MOTOR node: positioned at (mx, my, mz) in world space
            n_motor = Node(
                name=f"MOTOR_{num:02d}",
                translation=[mx, my, mz],
                children=[n_mcan_idx, n_mred_idx, n_nav_node_idx, n_rotor_idx]
            )
            n_motor_idx = len(self.gltf.nodes)
            self.gltf.nodes.append(n_motor)
            self.node_propulsion.children.append(n_motor_idx)

    # =========================================================================
    # 4. Heavy-Duty Symmetrical Landing Gear
    # =========================================================================
    def build_landing_gear(self):
        """
        Builds symmetrical heavy-duty inverted A-frame tubular carbon landing gear:
        - Upper chassis mounting bracket
        - Angled tubular carbon strut
        - Secondary cross-brace support
        - Lower ground contact skid tube with curved footings & rubber caps
        """
        carbon_parts = []
        skid_y = -0.36
        skid_len = 0.54
        skid_radius = 0.013

        for sx in [-0.22, 0.22]:
            # Longitudinal Ground Skid Tube
            p_skid_start = [sx, skid_y, -skid_len * 0.5]
            p_skid_end = [sx, skid_y, skid_len * 0.5]
            carbon_parts.append(create_oriented_cylinder(p_skid_start, p_skid_end, skid_radius, 24))
            carbon_parts.append(create_sphere_mesh(skid_radius * 1.25, 14, 20, sx, skid_y, skid_len * 0.5))
            carbon_parts.append(create_sphere_mesh(skid_radius * 1.25, 14, 20, sx, skid_y, -skid_len * 0.5))

            # Dual Angled A-Frame Struts
            for sz in [-0.14, 0.14]:
                p_bot = [sx, skid_y, sz]
                p_top = [sx * 0.58, -0.055, sz]
                carbon_parts.append(create_oriented_cylinder(p_bot, p_top, 0.011, 24))
                carbon_parts.append(create_box_mesh(0.035, 0.022, 0.035, p_top[0], p_top[1], p_top[2]))

            # Cross-Braces
            carbon_parts.append(create_oriented_cylinder([-0.22, skid_y + 0.03, -0.14], [0.22, skid_y + 0.03, -0.14], 0.007, 16))
            carbon_parts.append(create_oriented_cylinder([-0.22, skid_y + 0.03, 0.14], [0.22, skid_y + 0.03, 0.14], 0.007, 16))

        p, n, idx = combine_meshes(carbon_parts)
        m_gear_idx = self._add_mesh_to_gltf(p, n, idx, 1, "LandingGear_Mesh")
        n_gear = Node(name="LANDING_GEAR_MAIN", mesh=m_gear_idx)
        n_gear_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_gear)
        self.node_landing_gear.children.append(n_gear_idx)

    # =========================================================================
    # 5. Payload & Modular 3-Axis Stabilized Gimbal
    # =========================================================================
    def build_payload_and_gimbal(self):
        """
        Builds the 3-axis stabilized military EO/IR gimbal turret hierarchy:
        PAYLOAD_MOUNT -> GIMBAL_YAW -> GIMBAL_PITCH -> GIMBAL_ROLL -> CAMERA_BODY
        """
        # 1. PAYLOAD_MOUNT (Vibration Isolation Plate & Silicone Dampers)
        mount_carbon = []
        mount_carbon.append(create_box_mesh(0.10, 0.010, 0.10, 0.0, -0.065, -0.12))
        for dx in [-0.038, 0.038]:
            for dz in [-0.038, 0.038]:
                mount_carbon.append(create_sphere_mesh(0.009, 12, 16, dx, -0.076, -0.12 + dz))

        p_pm, n_pm, i_pm = combine_meshes(mount_carbon)
        m_pm_idx = self._add_mesh_to_gltf(p_pm, n_pm, i_pm, 0, "Payload_Mount_Mesh")
        n_pm = Node(name="PAYLOAD_MOUNT", mesh=m_pm_idx)
        n_pm_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_pm)
        self.node_payload.children.append(n_pm_idx)

        # 2. GIMBAL_YAW (Yaw collar at -0.088m)
        yaw_parts = []
        yaw_parts.append(create_cylinder_mesh(0.028, 0.018, 20, 0.0, 0.0, 0.0))
        p_yaw, n_yaw, i_yaw = combine_meshes(yaw_parts)
        m_yaw_idx = self._add_mesh_to_gltf(p_yaw, n_yaw, i_yaw, 0, "Gimbal_Yaw_Mesh")
        n_gyaw = Node(name="GIMBAL_YAW", translation=[0.0, -0.088, -0.12], mesh=m_yaw_idx)
        n_gyaw_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_gyaw)
        self.node_payload.children.append(n_gyaw_idx)

        # 3. GIMBAL_PITCH (U-Yoke Arm)
        pitch_parts = []
        pitch_parts.append(create_box_mesh(0.08, 0.015, 0.022, 0.0, -0.017, 0.0))
        pitch_parts.append(create_box_mesh(0.015, 0.06, 0.022, 0.040, -0.047, 0.0))
        pitch_parts.append(create_box_mesh(0.015, 0.06, 0.022, -0.040, -0.047, 0.0))
        p_pitch, n_pitch, i_pitch = combine_meshes(pitch_parts)
        m_pitch_idx = self._add_mesh_to_gltf(p_pitch, n_pitch, i_pitch, 0, "Gimbal_Pitch_Mesh")
        n_gpitch = Node(name="GIMBAL_PITCH", mesh=m_pitch_idx)
        n_gpitch_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_gpitch)
        n_gyaw.children.append(n_gpitch_idx)

        # 4. GIMBAL_ROLL & CAMERA_BODY (Turret Sphere + Quad Optical Lenses)
        cam_carbon = []
        cam_glass = []

        # Spherical Turret Housing
        cam_carbon.append(create_box_mesh(0.068, 0.068, 0.068, 0.0, -0.057, 0.0))

        # Quad Apertures on Front Face
        cam_carbon.append(create_cylinder_mesh(0.016, 0.014, 20, 0.016, -0.044, -0.035, axis='z'))
        cam_glass.append(create_cylinder_mesh(0.014, 0.004, 20, 0.016, -0.044, -0.041, axis='z'))

        cam_carbon.append(create_cylinder_mesh(0.012, 0.014, 20, -0.016, -0.044, -0.035, axis='z'))
        cam_glass.append(create_cylinder_mesh(0.010, 0.004, 20, -0.016, -0.044, -0.041, axis='z'))

        cam_carbon.append(create_cylinder_mesh(0.009, 0.012, 16, -0.016, -0.070, -0.035, axis='z'))
        cam_glass.append(create_cylinder_mesh(0.007, 0.004, 16, -0.016, -0.070, -0.040, axis='z'))

        cam_carbon.append(create_cylinder_mesh(0.009, 0.012, 16, 0.016, -0.070, -0.035, axis='z'))
        cam_glass.append(create_cylinder_mesh(0.007, 0.004, 16, 0.016, -0.070, -0.040, axis='z'))

        p_cc, n_cc, i_cc = combine_meshes(cam_carbon)
        m_cc_idx = self._add_mesh_to_gltf(p_cc, n_cc, i_cc, 0, "Camera_Body_Mesh")
        n_cbody = Node(name="CAMERA_BODY", mesh=m_cc_idx)
        n_cbody_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_cbody)

        p_cg, n_cg, i_cg = combine_meshes(cam_glass)
        m_cg_idx = self._add_mesh_to_gltf(p_cg, n_cg, i_cg, 5, "Camera_Lenses_Mesh")
        n_cg = Node(name="CAMERA_LENSES", mesh=m_cg_idx)
        n_cg_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_cg)
        n_cbody.children.append(n_cg_idx)

        n_groll = Node(name="GIMBAL_ROLL", children=[n_cbody_idx])
        n_groll_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_groll)
        n_gpitch.children.append(n_groll_idx)

    # =========================================================================
    # 6. Antennas & Status Beacon
    # =========================================================================
    def build_antennas(self):
        """Builds 4 multi-band whip antennas and top red anti-collision strobe beacon."""
        ant_parts = []
        antennas = [
            (0.06, 0.12, -0.08), (-0.06, 0.12, -0.08),
            (0.065, 0.12, 0.08), (-0.065, 0.12, 0.08)
        ]
        for ax, ay, az in antennas:
            ant_parts.append(create_cylinder_mesh(0.008, 0.02, 16, ax, ay, az))
            ant_parts.append(create_cylinder_mesh(0.003, 0.12, 16, ax, ay + 0.06, az))

        p_ant, n_ant, i_ant = combine_meshes(ant_parts)
        m_ant_idx = self._add_mesh_to_gltf(p_ant, n_ant, i_ant, 0, "Antennas_Mesh")
        n_ant = Node(name="ANTENNA_SYSTEM", mesh=m_ant_idx)
        n_ant_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_ant)
        self.node_antennas.children.append(n_ant_idx)

        # Red Anti-Collision Strobe
        beacon_base = []
        beacon_base.append(create_cylinder_mesh(0.016, 0.015, 20, 0.0, 0.115, 0.0))
        p_bb, n_bb, i_bb = combine_meshes(beacon_base)
        m_bb_idx = self._add_mesh_to_gltf(p_bb, n_bb, i_bb, 0, "Beacon_Base_Mesh")
        n_bbase = Node(name="BEACON_BASE", mesh=m_bb_idx)
        n_bbase_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_bbase)

        beacon_dome = []
        beacon_dome.append(create_sphere_mesh(0.014, 14, 20, 0.0, 0.128, 0.0))
        p_bd, n_bd, i_bd = combine_meshes(beacon_dome)
        m_bd_idx = self._add_mesh_to_gltf(p_bd, n_bd, i_bd, 4, "Beacon_Strobe_Mesh")
        n_bdome = Node(name="LIGHTS_BEACON", mesh=m_bd_idx)
        n_bdome_idx = len(self.gltf.nodes)
        self.gltf.nodes.append(n_bdome)

        n_bbase.children.append(n_bdome_idx)
        self.node_lights.children.append(n_bbase_idx)

    def export_glb(self, output_path):
        """Packages all buffers and exports GLB binary file."""
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        self.gltf.buffers.append(Buffer(byteLength=len(self.binary_blob)))
        self.gltf.set_binary_blob(bytes(self.binary_blob))
        self.gltf.save_binary(output_path)
        print(f"[+] Master Mechanical GLB Asset exported to: {output_path} ({len(self.binary_blob)} bytes)")

def main():
    out_dir = os.path.join(os.path.dirname(__file__), "..", "models")
    out_glb = os.path.join(out_dir, "GARUDA_HL_01.glb")
    # Also overwrite the active garuda_hl01.glb
    out_active_glb = os.path.join(out_dir, "garuda_hl01.glb")

    builder = GarudaMasterModelBuilder()
    builder.build_airframe()
    builder.build_arms()
    builder.build_propulsion()
    builder.build_landing_gear()
    builder.build_payload_and_gimbal()
    builder.build_antennas()

    builder.export_glb(out_glb)
    builder.export_glb(out_active_glb)

if __name__ == "__main__":
    main()
