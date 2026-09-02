#!/usr/bin/env python3
"""
geo_utils.py
Precision 3D Hard-Surface & Aerodynamic Geometry Utilities for GARUDA-HL-01 Pipeline
"""

import os
import numpy as np
import pygltflib
from pygltflib import (
    GLTF2, Scene, Node, Mesh, Primitive, Attributes,
    Accessor, BufferView, Buffer, Material, PbrMetallicRoughness
)

def create_oriented_cylinder(p_start, p_end, radius, segments=32):
    """Creates a continuous cylindrical tube between two points with end-caps."""
    segments = int(segments)
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

    # Mantle vertices
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

    # Start Cap (at p_start, normal = -dir_vec)
    cap_start_center = len(positions)
    positions.append(p_start.tolist())
    normals.append((-dir_vec).tolist())

    cap_start_rim_base = len(positions)
    for i in range(segments + 1):
        theta = (float(i) / segments) * 2.0 * np.pi
        c, s = np.cos(theta), np.sin(theta)
        v = p_start + radius * (c * u_vec + s * v_vec)
        positions.append(v.tolist())
        normals.append((-dir_vec).tolist())

    for i in range(segments):
        indices.extend([cap_start_center, cap_start_rim_base + i + 1, cap_start_rim_base + i])

    # End Cap (at p_end, normal = +dir_vec)
    cap_end_center = len(positions)
    positions.append(p_end.tolist())
    normals.append(dir_vec.tolist())

    cap_end_rim_base = len(positions)
    for i in range(segments + 1):
        theta = (float(i) / segments) * 2.0 * np.pi
        c, s = np.cos(theta), np.sin(theta)
        v = p_end + radius * (c * u_vec + s * v_vec)
        positions.append(v.tolist())
        normals.append(dir_vec.tolist())

    for i in range(segments):
        indices.extend([cap_end_center, cap_end_rim_base + i, cap_end_rim_base + i + 1])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def create_cylinder_mesh(radius, height, segments=32, center_x=0.0, center_y=0.0, center_z=0.0, axis='y'):
    """Creates cylinder along specified axis."""
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
    """Creates an indexed box mesh with crisp flat face normals."""
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
        ([(-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz), (-hx, -hy, -hz)], (-1, 0, 0)),
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
    """Creates a physical cambered aerodynamic blade mesh with thickness, twist, and taper."""
    sections = 10
    positions = []
    normals = []
    indices = []

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

        cos_tw, sin_tw = np.cos(twist), np.sin(twist)

        pts_2d = [
            (-chord * 0.35, 0.0),
            (-chord * 0.10, thick * 0.90),
            (chord * 0.25, thick * 0.45),
            (chord * 0.65, 0.0),
            (chord * 0.25, -thick * 0.25),
            (-chord * 0.10, -thick * 0.35),
        ]

        sec_base = len(positions)
        for pz, py in pts_2d:
            ry = py * cos_tw - pz * sin_tw
            rz = py * sin_tw + pz * cos_tw
            positions.append([x_span, ry, rz])
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

    if tip_white:
        tip_base = len(positions) - 6
        tip_center = len(positions)
        positions.append([hub_offset + span, 0.0, 0.0])
        normals.append([1.0, 0.0, 0.0])
        for k in range(6):
            k_next = (k + 1) % 6
            indices.extend([tip_center, tip_base + k, tip_base + k_next])

    return np.array(positions, dtype=np.float32), np.array(normals, dtype=np.float32), np.array(indices, dtype=np.uint32)

def combine_meshes(mesh_list):
    """Combines a list of (positions, normals, indices) tuples into one indexed mesh."""
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

class PipelineGLTFBuilder:
    """Manages GLTF2 scene construction with standard PBR material slots and binary buffer encoding."""
    def __init__(self, root_name="ROOT"):
        self.gltf = GLTF2()
        self.gltf.scene = 0
        self.scene = Scene()
        self.gltf.scenes.append(self.scene)

        self.binary_blob = bytearray()
        self._setup_materials()

        self.root_node = Node(name=root_name)
        self.gltf.nodes.append(self.root_node)
        self.scene.nodes.append(0)

    def _setup_materials(self):
        # Material library matching industrial UAV spec with calibrated PBR dielectric/metal responses
        m0 = Material(name="MAT_STEALTH_CARBON", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.055, 0.058, 0.065, 1.0], metallicFactor=0.08, roughnessFactor=0.42))
        m1 = Material(name="MAT_CARBON_TUBE", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.045, 0.048, 0.052, 1.0], metallicFactor=0.05, roughnessFactor=0.35))
        m2 = Material(name="MAT_CNC_RED_ALUMINUM", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.78, 0.04, 0.06, 1.0], metallicFactor=0.92, roughnessFactor=0.22))
        m3 = Material(name="MAT_CYAN_STATUS", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.0, 0.90, 1.0, 1.0], metallicFactor=0.1, roughnessFactor=0.1), emissiveFactor=[0.0, 2.2, 2.6])
        m4 = Material(name="MAT_RED_STATUS", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[1.0, 0.04, 0.04, 1.0], metallicFactor=0.1, roughnessFactor=0.1), emissiveFactor=[2.8, 0.1, 0.1])
        m5 = Material(name="MAT_OPTICAL_GLASS", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.02, 0.04, 0.06, 1.0], metallicFactor=0.12, roughnessFactor=0.04))
        m6 = Material(name="MAT_TACTICAL_WHITE", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.90, 0.90, 0.92, 1.0], metallicFactor=0.0, roughnessFactor=0.25))
        m7 = Material(name="MAT_PROPELLER_CARBON", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.040, 0.042, 0.045, 1.0], metallicFactor=0.06, roughnessFactor=0.38))
        m8 = Material(name="MAT_NAV_GREEN", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.0, 1.0, 0.35, 1.0], metallicFactor=0.1, roughnessFactor=0.1), emissiveFactor=[0.0, 1.8, 0.5])
        m9 = Material(name="MAT_DARK_METAL", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.12, 0.13, 0.14, 1.0], metallicFactor=0.88, roughnessFactor=0.28))
        m10 = Material(name="MAT_MESH_LOUVER", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.025, 0.025, 0.028, 1.0], metallicFactor=0.35, roughnessFactor=0.70))
        m11 = Material(name="MAT_LENS_EMERALD_GREEN", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.02, 0.40, 0.25, 1.0], metallicFactor=0.15, roughnessFactor=0.03))
        m12 = Material(name="MAT_LENS_GERMANIUM_PURPLE", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.35, 0.05, 0.35, 1.0], metallicFactor=0.20, roughnessFactor=0.04))
        m13 = Material(name="MAT_LENS_SAPPHIRE_BLUE", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.05, 0.20, 0.45, 1.0], metallicFactor=0.18, roughnessFactor=0.03))
        m14 = Material(name="MAT_SILICONE_DAMPER", pbrMetallicRoughness=PbrMetallicRoughness(
            baseColorFactor=[0.55, 0.58, 0.62, 1.0], metallicFactor=0.02, roughnessFactor=0.60))

        self.gltf.materials = [m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14]

    def add_mesh(self, positions, normals, indices, material_idx, name):
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
        self.gltf.bufferViews.append(BufferView(buffer=0, byteOffset=pos_offset, byteLength=len(pos_bytes), target=34962))

        bv_norm_idx = len(self.gltf.bufferViews)
        self.gltf.bufferViews.append(BufferView(buffer=0, byteOffset=norm_offset, byteLength=len(norm_bytes), target=34962))

        bv_idx_idx = len(self.gltf.bufferViews)
        self.gltf.bufferViews.append(BufferView(buffer=0, byteOffset=idx_offset, byteLength=len(idx_bytes), target=34963))

        min_pos = positions.min(axis=0).tolist() if len(positions) > 0 else [0, 0, 0]
        max_pos = positions.max(axis=0).tolist() if len(positions) > 0 else [0, 0, 0]

        acc_pos_idx = len(self.gltf.accessors)
        self.gltf.accessors.append(Accessor(bufferView=bv_pos_idx, byteOffset=0, componentType=5126, count=len(positions), type="VEC3", min=min_pos, max=max_pos))

        acc_norm_idx = len(self.gltf.accessors)
        self.gltf.accessors.append(Accessor(bufferView=bv_norm_idx, byteOffset=0, componentType=5126, count=len(normals), type="VEC3"))

        acc_idx_idx = len(self.gltf.accessors)
        self.gltf.accessors.append(Accessor(bufferView=bv_idx_idx, byteOffset=0, componentType=5125, count=len(indices), type="SCALAR"))

        prim = Primitive(attributes=Attributes(POSITION=acc_pos_idx, NORMAL=acc_norm_idx), indices=acc_idx_idx, material=material_idx)
        mesh_idx = len(self.gltf.meshes)
        self.gltf.meshes.append(Mesh(name=name, primitives=[prim]))
        return mesh_idx

    def export(self, output_path):
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        self.gltf.buffers.append(Buffer(byteLength=len(self.binary_blob)))
        self.gltf.set_binary_blob(bytes(self.binary_blob))
        self.gltf.save_binary(output_path)
        print(f"[+] Exported: {output_path} ({len(self.binary_blob)} bytes)")

import json

_CACHED_MANIFEST = None

def get_manifest():
    """Loads and validates the authoritative C++ generated model specification manifest."""
    global _CACHED_MANIFEST
    if _CACHED_MANIFEST is not None:
        return _CACHED_MANIFEST

    manifest_path = os.path.join(os.path.dirname(__file__), "..", "..", "build", "generated", "GARUDA_HL_01_MODEL_SPEC.json")
    if not os.path.exists(manifest_path):
        raise RuntimeError(f"[!] Critical Error: Model manifest not found at {manifest_path}. Run generate_model_manifest first!")

    with open(manifest_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Basic Schema & Authority Validation
    if data.get("schema_version") != "2.0.0" or data.get("vehicle_identity", {}).get("model_name") != "GARUDA-HL-01":
        raise ValueError(f"[!] Invalid manifest schema or identity in {manifest_path}")

    _CACHED_MANIFEST = data
    return _CACHED_MANIFEST

