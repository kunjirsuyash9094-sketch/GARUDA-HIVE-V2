#!/usr/bin/env python3
"""
build_06_payload_gimbal.py
STEP 6: Precision 3-Axis Gimbal & Quad Sensor Turret Generator
Creates GIMBAL_MASTER.glb matching the reference engineering blueprint:
- 120mm x 120mm x 23mm Vibration Isolated Quick-Release Mount Plate
- 4x Corner Silicone Dampers with CNC Anodized Red Compression Bands
- Yaw Axis (Pan) 360° Continuous Direct-Drive Pancake Motor
- Rigid Symmetrical CNC Machined U-Yoke Arm with Precision Pitch Trunnions
- Quad Sensor Turret (156mm W x 203mm H x 163mm D envelope) facing strictly -Z (Forward):
  1. Top-Right: 4K Daylight Camera (Emerald Green AR Lens)
  2. Top-Left: FLIR LWIR Thermal Camera (Deep Purple Germanium Lens)
  3. Bottom-Left: SWIR / NIR Camera (Sapphire Blue Lens)
  4. Bottom-Right: Laser Rangefinder LRF (2x2 Quad Optical Array)
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

def build_gimbal():
    manifest = get_manifest()
    builder = PipelineGLTFBuilder(root_name="GIMBAL_MASTER")

    # -------------------------------------------------------------------------
    # 1. PAYLOAD MOUNT & VIBRATION ISOLATOR (120mm x 120mm x 23mm)
    # -------------------------------------------------------------------------
    mount_metal = []
    mount_damper = []
    mount_red = []

    # Top Interface Plate (attaches to UAV bottom rails)
    mount_metal.append(create_box_mesh(0.120, 0.005, 0.120, 0.0, 0.010, 0.0))
    # Bottom Decoupled Yaw Base Plate
    mount_metal.append(create_box_mesh(0.110, 0.005, 0.110, 0.0, -0.012, 0.0))

    # 4 Corner Silicone Vibration Dampers with Red Rings
    for dx in [-0.045, 0.045]:
        for dz in [-0.045, 0.045]:
            mount_metal.append(create_cylinder_mesh(radius=0.007, height=0.004, segments=16, center_x=dx, center_y=0.006, center_z=dz, axis='y'))
            mount_metal.append(create_cylinder_mesh(radius=0.007, height=0.004, segments=16, center_x=dx, center_y=-0.008, center_z=dz, axis='y'))
            # Silicone Damper Ball (Slot 14: MAT_SILICONE_DAMPER)
            mount_damper.append(create_sphere_mesh(radius=0.009, rings=12, sectors=16, center_x=dx, center_y=-0.001, center_z=dz))
            # CNC Red Compression Band (Slot 2: MAT_CNC_RED_ALUMINUM)
            mount_red.append(create_cylinder_mesh(radius=0.0098, height=0.003, segments=16, center_x=dx, center_y=-0.001, center_z=dz, axis='y'))

    p_mm, n_mm, i_mm = combine_meshes(mount_metal)
    m_mm_idx = builder.add_mesh(p_mm, n_mm, i_mm, 9, "Mount_Metal_Plates")

    p_md, n_md, i_md = combine_meshes(mount_damper)
    m_md_idx = builder.add_mesh(p_md, n_md, i_md, 14, "Mount_Silicone_Dampers")

    p_mr, n_mr, i_mr = combine_meshes(mount_red)
    m_mr_idx = builder.add_mesh(p_mr, n_mr, i_mr, 2, "Mount_Red_Rings")

    n_mount_m = Node(name="MOUNT_METAL", mesh=m_mm_idx)
    n_mount_m_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_mount_m)

    n_mount_d = Node(name="MOUNT_DAMPERS", mesh=m_md_idx)
    n_mount_d_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_mount_d)

    n_mount_r = Node(name="MOUNT_RINGS", mesh=m_mr_idx)
    n_mount_r_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_mount_r)

    n_payload_mount = Node(
        name="PAYLOAD_MOUNT",
        children=[n_mount_m_idx, n_mount_d_idx, n_mount_r_idx]
    )
    n_pm_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_payload_mount)
    builder.root_node.children.append(n_pm_idx)

    # -------------------------------------------------------------------------
    # 2. YAW AXIS DIRECT-DRIVE MOTOR (Pan 360°)
    # -------------------------------------------------------------------------
    yaw_parts = []
    # Pancake Direct-Drive Motor Housing
    yaw_parts.append(create_cylinder_mesh(radius=0.036, height=0.014, segments=32, center_x=0.0, center_y=-0.007, center_z=0.0, axis='y'))
    yaw_parts.append(create_cylinder_mesh(radius=0.024, height=0.006, segments=24, center_x=0.0, center_y=-0.016, center_z=0.0, axis='y'))

    p_yaw, n_yaw, i_yaw = combine_meshes(yaw_parts)
    m_yaw_idx = builder.add_mesh(p_yaw, n_yaw, i_yaw, 9, "Gimbal_Yaw_Motor")
    n_yaw_node = Node(name="GIMBAL_YAW", translation=[0.0, -0.020, 0.0], mesh=m_yaw_idx)
    n_yaw_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_yaw_node)
    n_payload_mount.children.append(n_yaw_idx)

    # -------------------------------------------------------------------------
    # 3. PITCH AXIS RIGID U-YOKE ARM (-90° to +30°)
    # -------------------------------------------------------------------------
    pitch_yoke_parts = []
    # Horizontal Top Cross-Beam connecting to Yaw rotor
    pitch_yoke_parts.append(create_box_mesh(0.144, 0.012, 0.034, 0.0, -0.006, 0.0))
    # Right Side Drop Arm (X = +0.072m)
    pitch_yoke_parts.append(create_box_mesh(0.014, 0.070, 0.030, 0.072, -0.042, 0.0))
    # Right Pitch Motor Housing & Trunnion Hub
    pitch_yoke_parts.append(create_cylinder_mesh(radius=0.024, height=0.016, segments=32, center_x=0.080, center_y=-0.076, center_z=0.0, axis='x'))
    pitch_yoke_parts.append(create_cylinder_mesh(radius=0.015, height=0.006, segments=24, center_x=0.089, center_y=-0.076, center_z=0.0, axis='x'))
    # Left Side Drop Arm (X = -0.072m)
    pitch_yoke_parts.append(create_box_mesh(0.014, 0.070, 0.030, -0.072, -0.042, 0.0))
    # Left Idler Trunnion Hub
    pitch_yoke_parts.append(create_cylinder_mesh(radius=0.020, height=0.012, segments=24, center_x=-0.078, center_y=-0.076, center_z=0.0, axis='x'))

    p_pitch, n_pitch, i_pitch = combine_meshes(pitch_yoke_parts)
    m_pitch_idx = builder.add_mesh(p_pitch, n_pitch, i_pitch, 9, "Gimbal_Pitch_Yoke")
    n_pitch_node = Node(name="GIMBAL_PITCH", translation=[0.0, -0.014, 0.0], mesh=m_pitch_idx)
    n_pitch_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_pitch_node)
    n_yaw_node.children.append(n_pitch_idx)

    # -------------------------------------------------------------------------
    # 4. ROLL AXIS JOINT (±45° Stabilization)
    # -------------------------------------------------------------------------
    n_roll_node = Node(name="GIMBAL_ROLL", translation=[0.0, -0.076, 0.0])
    n_roll_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_roll_node)
    n_pitch_node.children.append(n_roll_idx)

    # -------------------------------------------------------------------------
    # 5. QUAD SENSOR TURRET (Facing strictly -Z Forward)
    # -------------------------------------------------------------------------
    body_parts = []
    body_metal = []
    lens_emerald = []
    lens_germanium = []
    lens_sapphire = []
    lens_lrf = []

    # Main Turret Housing (106mm W x 112mm H x 116mm D)
    body_parts.append(create_box_mesh(0.106, 0.112, 0.116, 0.0, 0.0, 0.0))
    # Front Beveled Armor Faceplate (Z = -0.058m)
    body_parts.append(create_box_mesh(0.098, 0.104, 0.008, 0.0, 0.0, -0.058))

    # Left & Right Trunnion Axle Mounts (connecting flush to Yoke Trunnions)
    body_metal.append(create_cylinder_mesh(radius=0.018, height=0.012, segments=24, center_x=0.058, center_y=0.0, center_z=0.0, axis='x'))
    body_metal.append(create_cylinder_mesh(radius=0.018, height=0.012, segments=24, center_x=-0.058, center_y=0.0, center_z=0.0, axis='x'))

    # 8x Faceplate Perimeter Hex Fastener Screws
    for fx in [-0.045, 0.045]:
        for fy in [-0.046, -0.015, 0.015, 0.046]:
            body_metal.append(create_cylinder_mesh(radius=0.002, height=0.004, segments=8, center_x=fx, center_y=fy, center_z=-0.063, axis='z'))

    # Top Heat Sink Cooling Fins
    for rz in [-0.030, -0.010, 0.010, 0.030]:
        body_parts.append(create_box_mesh(0.076, 0.004, 0.005, 0.0, 0.058, rz))

    # -------------------------------------------------------------------------
    # Quad Multi-Spectral Optical Sensors on Front Faceplate (Z = -0.062m)
    # -------------------------------------------------------------------------
    # 1. Top-Right: 4K Daylight Optical Zoom Camera (f = 6.4 - 192mm)
    body_metal.append(create_cylinder_mesh(radius=0.021, height=0.010, segments=32, center_x=0.026, center_y=0.026, center_z=-0.065, axis='z'))
    body_metal.append(create_cylinder_mesh(radius=0.018, height=0.003, segments=24, center_x=0.026, center_y=0.026, center_z=-0.070, axis='z'))
    # Emerald Green Anti-Reflective Optical Glass Lens (Slot 11)
    lens_emerald.append(create_cylinder_mesh(radius=0.0165, height=0.002, segments=32, center_x=0.026, center_y=0.026, center_z=-0.071, axis='z'))

    # 2. Top-Left: FLIR LWIR Thermal Camera (640x512 Germanium)
    body_metal.append(create_cylinder_mesh(radius=0.019, height=0.009, segments=32, center_x=-0.026, center_y=0.026, center_z=-0.065, axis='z'))
    for ta in range(6):
        ang = (ta / 6.0) * 2.0 * np.pi
        body_metal.append(create_cylinder_mesh(radius=0.0015, height=0.003, segments=8, center_x=-0.026 + 0.015 * np.cos(ang), center_y=0.026 + 0.015 * np.sin(ang), center_z=-0.069, axis='z'))
    # Deep Purple Germanium Thermal Lens (Slot 12)
    lens_germanium.append(create_cylinder_mesh(radius=0.0135, height=0.002, segments=32, center_x=-0.026, center_y=0.026, center_z=-0.070, axis='z'))

    # 3. Bottom-Left: SWIR / NIR Camera (900 - 1700nm InGaAs Sensor)
    body_metal.append(create_cylinder_mesh(radius=0.023, height=0.010, segments=32, center_x=-0.024, center_y=-0.026, center_z=-0.065, axis='z'))
    body_metal.append(create_cylinder_mesh(radius=0.019, height=0.003, segments=24, center_x=-0.024, center_y=-0.026, center_z=-0.070, axis='z'))
    # Sapphire Blue Optical Glass Lens (Slot 13)
    lens_sapphire.append(create_cylinder_mesh(radius=0.0175, height=0.002, segments=32, center_x=-0.024, center_y=-0.026, center_z=-0.071, axis='z'))

    # 4. Bottom-Right: Laser Rangefinder (LRF, Class 1, 6000m) with 2x2 Array
    body_metal.append(create_box_mesh(0.040, 0.040, 0.006, 0.026, -0.026, -0.064))
    for lx in [0.017, 0.035]:
        for ly in [-0.035, -0.017]:
            body_metal.append(create_cylinder_mesh(radius=0.0065, height=0.005, segments=20, center_x=lx, center_y=ly, center_z=-0.067, axis='z'))
            lens_lrf.append(create_cylinder_mesh(radius=0.005, height=0.002, segments=16, center_x=lx, center_y=ly, center_z=-0.069, axis='z'))

    # Assemble Turret Node Hierarchy
    p_cb, n_cb, i_cb = combine_meshes(body_parts)
    m_cb_idx = builder.add_mesh(p_cb, n_cb, i_cb, 0, "Camera_Armor_Body")

    p_cbm, n_cbm, i_cbm = combine_meshes(body_metal)
    m_cbm_idx = builder.add_mesh(p_cbm, n_cbm, i_cbm, 9, "Camera_Bezel_Hardware")

    p_le, n_le, i_le = combine_meshes(lens_emerald)
    m_le_idx = builder.add_mesh(p_le, n_le, i_le, 11, "Lens_Daylight_Emerald")

    p_lg, n_lg, i_lg = combine_meshes(lens_germanium)
    m_lg_idx = builder.add_mesh(p_lg, n_lg, i_lg, 12, "Lens_Thermal_Germanium")

    p_ls, n_ls, i_ls = combine_meshes(lens_sapphire)
    m_ls_idx = builder.add_mesh(p_ls, n_ls, i_ls, 13, "Lens_SWIR_Sapphire")

    p_lr, n_lr, i_lr = combine_meshes(lens_lrf)
    m_lr_idx = builder.add_mesh(p_lr, n_lr, i_lr, 5, "Lens_LRF_Array")

    n_cb_inst = Node(name="CAMERA_BODY_MESH", mesh=m_cb_idx)
    n_cb_inst_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_cb_inst)

    n_cbm_inst = Node(name="CAMERA_BEZELS", mesh=m_cbm_idx)
    n_cbm_inst_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_cbm_inst)

    n_le_inst = Node(name="LENS_4K_DAYLIGHT", mesh=m_le_idx)
    n_le_inst_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_le_inst)

    n_lg_inst = Node(name="LENS_FLIR_THERMAL", mesh=m_lg_idx)
    n_lg_inst_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_lg_inst)

    n_ls_inst = Node(name="LENS_SWIR_NIR", mesh=m_ls_idx)
    n_ls_inst_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_ls_inst)

    n_lr_inst = Node(name="LENS_LRF_ARRAY", mesh=m_lr_idx)
    n_lr_inst_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_lr_inst)

    n_camera_body = Node(
        name="CAMERA_BODY",
        children=[n_cb_inst_idx, n_cbm_inst_idx, n_le_inst_idx, n_lg_inst_idx, n_ls_inst_idx, n_lr_inst_idx]
    )
    n_cbody_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_camera_body)
    n_roll_node.children.append(n_cbody_idx)

    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "06_PAYLOAD", "GIMBAL_MASTER.glb")
    builder.export(out_path)
    print(f"[OK] Gimbal Master compilation complete: {out_path}")

if __name__ == "__main__":
    build_gimbal()
