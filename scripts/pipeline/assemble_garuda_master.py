#!/usr/bin/env python3
"""
assemble_garuda_master.py
STEP 8: Master Manifest-Driven UAV Assembly Pipeline (Reference-Matched)
Assembles the complete GARUDA-HL-01 industrial heavy-lift octocopter from
authoritative C++ manifest data and precision component modules:
- 1x Central Airframe (Stealth Faceted Composite Hull, Diamond Mesh Louvers, Sockets, Clevis Blocks, Rails)
- 8x Instanced Carbon Boom Arms (0.550m span at 22.5° + i*45°) with Cyan Accent Stripes
- 8x Instanced 6215 Heavy-Lift Motors (CNC Anodized Red Cooling Rings & Stator Mounts)
- 8x Instanced 2-Blade Folding Rotors (16 physical aerofoil cambered blades, 16" dia)
- 1x Landing Gear System (Wide-Stance Splayed A-Frames, 0.36m ground clearance, zero camera obstruction)
- 1x Modular Payload & 3-Axis Gimbal (Isolated Mount -> Yaw -> Pitch U-Yoke -> Roll -> Quad-Lens Camera)
- 1x Avionics & Antennas (6x Telemetry Antennas, GNSS, Red Strobe Beacon)
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from geo_utils import (
    create_oriented_cylinder, create_cylinder_mesh, create_box_mesh,
    create_sphere_mesh, create_aerofoil_blade_mesh,
    combine_meshes, PipelineGLTFBuilder, get_manifest
)
from build_01_airframe import create_stealth_faceted_hull
from pygltflib import Node

def assemble_master_asset():
    print("[*] Assembling GARUDA-HL-01 Master Modular Asset from C++ Manifest...")
    manifest = get_manifest()
    builder = PipelineGLTFBuilder(root_name="GARUDA_HL_01")

    # Primary Functional Branches
    node_airframe = Node(name="AIRFRAME")
    node_airframe_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(node_airframe)
    builder.root_node.children.append(node_airframe_idx)

    node_arms = Node(name="ARMS")
    node_arms_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(node_arms)
    builder.root_node.children.append(node_arms_idx)

    node_propulsion = Node(name="PROPULSION")
    node_propulsion_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(node_propulsion)
    builder.root_node.children.append(node_propulsion_idx)

    node_landing_gear = Node(name="LANDING_GEAR")
    node_landing_gear_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(node_landing_gear)
    builder.root_node.children.append(node_landing_gear_idx)

    node_payload = Node(name="PAYLOAD")
    node_payload_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(node_payload)
    builder.root_node.children.append(node_payload_idx)

    node_avionics = Node(name="AVIONICS")
    node_avionics_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(node_avionics)
    builder.root_node.children.append(node_avionics_idx)

    node_lights = Node(name="LIGHTS")
    node_lights_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(node_lights)
    builder.root_node.children.append(node_lights_idx)

    # -------------------------------------------------------------------------
    # 1. Central Airframe (Matching Top-View Reference)
    # -------------------------------------------------------------------------
    carbon_body_parts = []
    metal_body_parts = []
    mesh_louver_parts = []
    glow_parts = []

    # Faceted Aerodynamic Composite Hull
    p_hull, n_hull, i_hull = create_stealth_faceted_hull()
    carbon_body_parts.append((p_hull, n_hull, i_hull))
    carbon_body_parts.append(create_box_mesh(0.180, 0.014, 0.240, 0.0, 0.082, -0.010))
    carbon_body_parts.append(create_box_mesh(0.140, 0.008, 0.180, 0.0, 0.090, -0.010))
    metal_body_parts.append(create_box_mesh(0.110, 0.004, 0.040, 0.0, 0.096, -0.060))
    carbon_body_parts.append(create_box_mesh(0.190, 0.024, 0.280, 0.0, -0.068, 0.0))

    for bx in [-0.075, 0.075]:
        for bz in [-0.090, -0.030, 0.030, 0.090]:
            metal_body_parts.append(create_cylinder_mesh(radius=0.0035, height=0.006, segments=8, center_x=bx, center_y=0.090, center_z=bz))

    # Flank Diamond Mesh Louvers (Slot 10: MAT_MESH_LOUVER)
    mesh_louver_parts.append(create_box_mesh(0.055, 0.004, 0.100, 0.145, 0.068, 0.010))
    mesh_louver_parts.append(create_box_mesh(0.055, 0.004, 0.100, -0.145, 0.068, 0.010))
    metal_body_parts.append(create_box_mesh(0.062, 0.006, 0.108, 0.145, 0.066, 0.010))
    metal_body_parts.append(create_box_mesh(0.062, 0.006, 0.108, -0.145, 0.066, 0.010))

    # 8x CNC Machined Arm Root Sockets & Clamp Collars
    for arm_cfg in manifest.get("arms", []):
        ang_rad = np.deg2rad(float(arm_cfg["radial_angle_deg"]))
        sin_a, cos_a = np.sin(ang_rad), np.cos(ang_rad)

        p_s = [0.120 * sin_a, 0.005, 0.120 * cos_a]
        p_e = [0.185 * sin_a, 0.014, 0.185 * cos_a]
        metal_body_parts.append(create_oriented_cylinder(p_s, p_e, 0.030, 24))

        p_fs = [0.170 * sin_a, 0.012, 0.170 * cos_a]
        p_fe = [0.180 * sin_a, 0.013, 0.180 * cos_a]
        metal_body_parts.append(create_oriented_cylinder(p_fs, p_fe, 0.033, 24))

        perp_x, perp_z = -cos_a * 0.020, sin_a * 0.020
        metal_body_parts.append(create_box_mesh(0.012, 0.016, 0.012, 0.175 * sin_a + perp_x, 0.024, 0.175 * cos_a + perp_z))
        metal_body_parts.append(create_box_mesh(0.012, 0.016, 0.012, 0.175 * sin_a - perp_x, 0.024, 0.175 * cos_a - perp_z))

    # Dual 15mm Quick-Release Payload Rails
    metal_body_parts.append(create_oriented_cylinder([-0.065, -0.082, -0.160], [-0.065, -0.082, 0.160], 0.0075, 16))
    metal_body_parts.append(create_oriented_cylinder([0.065, -0.082, -0.160], [0.065, -0.082, 0.160], 0.0075, 16))
    metal_body_parts.append(create_box_mesh(0.160, 0.012, 0.018, 0.0, -0.080, -0.130))
    metal_body_parts.append(create_box_mesh(0.160, 0.012, 0.018, 0.0, -0.080, 0.130))

    # 4 Landing Gear Clevis Blocks
    for gx in [-0.150, 0.150]:
        for gz in [-0.060, 0.160]:
            metal_body_parts.append(create_box_mesh(0.036, 0.022, 0.036, gx, -0.068, gz))

    # Cyan Accent Lightguides (Nose chevrons \ / + lower bar — + shoulder lines)
    glow_parts.append(create_box_mesh(0.065, 0.008, 0.014, 0.060, 0.028, -0.205))
    glow_parts.append(create_box_mesh(0.065, 0.008, 0.014, -0.060, 0.028, -0.205))
    glow_parts.append(create_box_mesh(0.090, 0.006, 0.010, 0.000, -0.012, -0.250))
    glow_parts.append(create_box_mesh(0.008, 0.006, 0.200, 0.200, 0.042, 0.000))
    glow_parts.append(create_box_mesh(0.008, 0.006, 0.200, -0.200, 0.042, 0.000))

    p1, n1, i1 = combine_meshes(carbon_body_parts)
    m_body_idx = builder.add_mesh(p1, n1, i1, 0, "Airframe_Carbon")
    n_body = Node(name="BODY_MAIN", mesh=m_body_idx)
    n_body_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_body)
    node_airframe.children.append(n_body_idx)

    p_bm, n_bm, i_bm = combine_meshes(metal_body_parts)
    m_bm_idx = builder.add_mesh(p_bm, n_bm, i_bm, 9, "Airframe_MetalSockets")
    n_bm = Node(name="AIRFRAME_SOCKETS", mesh=m_bm_idx)
    n_bm_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bm)
    node_airframe.children.append(n_bm_idx)

    p_louv, n_louv, i_louv = combine_meshes(mesh_louver_parts)
    m_louv_idx = builder.add_mesh(p_louv, n_louv, i_louv, 10, "Airframe_Louvers")
    n_louv = Node(name="AIRFRAME_LOUVERS", mesh=m_louv_idx)
    n_louv_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_louv)
    node_airframe.children.append(n_louv_idx)

    p2, n2, i2 = combine_meshes(glow_parts)
    m_glow_idx = builder.add_mesh(p2, n2, i2, 3, "Airframe_CyanGlow")
    n_glow = Node(name="AIRFRAME_CYAN_LIGHTS", mesh=m_glow_idx)
    n_glow_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_glow)
    node_lights.children.append(n_glow_idx)

    # -------------------------------------------------------------------------
    # 2. Master Arm Mesh (Along +X)
    # -------------------------------------------------------------------------
    arm_len = float(manifest["dimensions"]["arm_length_m"])  # 0.550m
    tube_rad = float(manifest["arms"][0]["tube_radius_m"])   # 0.015m

    arm_parts = []
    arm_metal = []
    arm_glow = []

    arm_parts.append(create_oriented_cylinder([0.100, 0.0, 0.0], [arm_len, 0.0, 0.0], tube_rad, 24))
    arm_metal.append(create_cylinder_mesh(radius=0.021, height=0.060, segments=24, center_x=0.135, center_y=0.0, center_z=0.0, axis='x'))
    arm_metal.append(create_box_mesh(0.055, 0.038, 0.038, 0.135, 0.0, 0.0))
    arm_metal.append(create_box_mesh(0.014, 0.016, 0.048, 0.115, 0.014, 0.0))
    arm_metal.append(create_box_mesh(0.014, 0.016, 0.048, 0.155, 0.014, 0.0))
    arm_metal.append(create_cylinder_mesh(radius=0.0195, height=0.032, segments=24, center_x=0.320, center_y=0.0, center_z=0.0, axis='x'))
    arm_metal.append(create_box_mesh(0.020, 0.024, 0.046, 0.320, 0.010, 0.0))
    arm_glow.append(create_box_mesh(0.120, 0.003, 0.008, 0.420, 0.016, 0.0))
    arm_metal.append(create_cylinder_mesh(radius=0.040, height=0.008, segments=32, center_x=arm_len, center_y=0.004, center_z=0.0, axis='y'))
    arm_metal.append(create_cylinder_mesh(radius=0.024, height=0.006, segments=24, center_x=arm_len, center_y=0.010, center_z=0.0, axis='y'))
    arm_parts.append(create_box_mesh(0.068, 0.016, 0.036, arm_len - 0.035, -0.014, 0.0))

    p_arm, n_arm, i_arm = combine_meshes(arm_parts)
    m_arm_idx = builder.add_mesh(p_arm, n_arm, i_arm, 1, "Arm_CarbonTube_Mesh")

    p_am, n_am, i_am = combine_meshes(arm_metal)
    m_am_idx = builder.add_mesh(p_am, n_am, i_am, 9, "Arm_Collars_Mesh")

    p_ag, n_ag, i_ag = combine_meshes(arm_glow)
    m_ag_idx = builder.add_mesh(p_ag, n_ag, i_ag, 3, "Arm_CyanStripe_Mesh")

    # -------------------------------------------------------------------------
    # 3. Master Motor & Rotor Meshes
    # -------------------------------------------------------------------------
    m_radius = float(manifest["motors"][0]["housing_radius_m"]) # 0.034m
    motor_can_parts = []
    motor_red_parts = []
    motor_nav_parts = []

    motor_can_parts.append(create_cylinder_mesh(radius=m_radius, height=0.016, segments=32, center_x=0.0, center_y=0.008, center_z=0.0, axis='y'))
    motor_can_parts.append(create_cylinder_mesh(radius=m_radius - 0.001, height=0.022, segments=32, center_x=0.0, center_y=0.035, center_z=0.0, axis='y'))
    motor_can_parts.append(create_cylinder_mesh(radius=0.004, height=0.016, segments=16, center_x=0.0, center_y=0.056, center_z=0.0, axis='y'))

    motor_red_parts.append(create_cylinder_mesh(radius=m_radius + 0.002, height=0.010, segments=32, center_x=0.0, center_y=0.018, center_z=0.0, axis='y'))
    for i in range(12):
        angle = (i / 12.0) * 2.0 * np.pi
        motor_red_parts.append(create_box_mesh(0.004, 0.009, 0.004, (m_radius + 0.001) * np.cos(angle), 0.018, (m_radius + 0.001) * np.sin(angle)))

    motor_nav_parts.append(create_sphere_mesh(radius=0.006, rings=12, sectors=16, center_x=0.0, center_y=-0.008, center_z=0.0))

    p_mc, n_mc, i_mc = combine_meshes(motor_can_parts)
    m_mcan_idx = builder.add_mesh(p_mc, n_mc, i_mc, 9, "Motor_Housing_Mesh")

    p_mr, n_mr, i_mr = combine_meshes(motor_red_parts)
    m_mred_idx = builder.add_mesh(p_mr, n_mr, i_mr, 2, "Motor_CNC_Red_Mesh")

    p_mn, n_mn, i_mn = combine_meshes(motor_nav_parts)
    m_mnav_idx = builder.add_mesh(p_mn, n_mn, i_mn, 8, "Motor_Nav_Mesh")

    prop_radius = float(manifest["dimensions"]["propeller_radius_m"]) # 0.2032m
    hub_radius = float(manifest["rotors"][0]["hub_radius_m"])         # 0.025m
    hub_parts = []
    hub_parts.append(create_cylinder_mesh(radius=0.016, height=0.012, segments=24, center_x=0.0, center_y=0.006, center_z=0.0, axis='y'))
    hub_parts.append(create_cylinder_mesh(radius=0.008, height=0.006, segments=6, center_x=0.0, center_y=0.015, center_z=0.0, axis='y'))
    hub_parts.append(create_box_mesh(0.018, 0.010, 0.018, hub_radius, 0.006, 0.0))
    hub_parts.append(create_box_mesh(0.018, 0.010, 0.018, -hub_radius, 0.006, 0.0))

    p_hub, n_hub, i_hub = combine_meshes(hub_parts)
    m_hub_idx = builder.add_mesh(p_hub, n_hub, i_hub, 9, "Rotor_Hub_Mesh")

    p_ba, n_ba, i_ba = create_aerofoil_blade_mesh(span=prop_radius, root_chord=0.036, tip_chord=0.022, max_thick=0.0055, twist_deg=16.0, tip_white=False)
    m_ba_idx = builder.add_mesh(p_ba, n_ba, i_ba, 7, "Blade_Carbon_Mesh")

    p_ta, n_ta, i_ta = create_aerofoil_blade_mesh(span=prop_radius, root_chord=0.036, tip_chord=0.022, max_thick=0.0055, twist_deg=16.0, tip_white=True)
    m_ta_idx = builder.add_mesh(p_ta, n_ta, i_ta, 6, "Blade_Tip_White_Mesh")

    # -------------------------------------------------------------------------
    # 4. 8-Rotor Octo-X Instantiation (Driven by Manifest)
    # -------------------------------------------------------------------------
    for i, arm_cfg in enumerate(manifest.get("arms", [])):
        arm_id = arm_cfg["id"]
        motor_cfg = manifest["motors"][i]
        rotor_cfg = manifest["rotors"][i]

        angle_deg = float(arm_cfg["radial_angle_deg"])
        motor_pos = [float(x) for x in motor_cfg["position"]]
        rotor_offset = [float(x) for x in rotor_cfg["local_offset"]]

        # Arm Node Components
        rot_y_rad = np.deg2rad(-angle_deg + 90.0) * 0.5
        n_arm_c = Node(name=f"{arm_id}_TUBE", mesh=m_arm_idx)
        n_arm_c_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_arm_c)

        n_arm_m = Node(name=f"{arm_id}_COLLARS", mesh=m_am_idx)
        n_arm_m_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_arm_m)

        n_arm_g = Node(name=f"{arm_id}_GLOW", mesh=m_ag_idx)
        n_arm_g_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_arm_g)

        n_arm = Node(
            name=arm_id,
            rotation=[0.0, float(np.sin(rot_y_rad)), 0.0, float(np.cos(rot_y_rad))],
            children=[n_arm_c_idx, n_arm_m_idx, n_arm_g_idx]
        )
        n_arm_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_arm)
        node_arms.children.append(n_arm_idx)

        # Motor Node (Positioned at exact motor_pos)
        n_mcan = Node(name=f"MOTOR_CAN_{i+1:02d}", mesh=m_mcan_idx)
        n_mcan_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_mcan)

        n_mred = Node(name=f"MOTOR_RING_{i+1:02d}", mesh=m_mred_idx)
        n_mred_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_mred)

        n_mnav = Node(name=f"NAV_LED_{i+1:02d}", mesh=m_mnav_idx)
        n_mnav_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_mnav)

        # Rotor Node (Child of Motor at rotor_offset)
        n_hub_inst = Node(name=f"ROTOR_HUB_{i+1:02d}", mesh=m_hub_idx)
        n_hub_inst_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_hub_inst)

        n_ba_b = Node(name=f"BLADE_A_BODY_{i+1:02d}", mesh=m_ba_idx)
        n_ba_b_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_ba_b)

        n_ba_t = Node(name=f"BLADE_A_TIP_{i+1:02d}", mesh=m_ta_idx)
        n_ba_t_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_ba_t)

        n_blade_a = Node(name=f"BLADE_A_{i+1:02d}", children=[n_ba_b_idx, n_ba_t_idx])
        n_blade_a_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_blade_a)

        n_bb_b = Node(name=f"BLADE_B_BODY_{i+1:02d}", mesh=m_ba_idx)
        n_bb_b_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_bb_b)

        n_bb_t = Node(name=f"BLADE_B_TIP_{i+1:02d}", mesh=m_ta_idx)
        n_bb_t_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_bb_t)

        n_blade_b = Node(
            name=f"BLADE_B_{i+1:02d}",
            rotation=[0.0, 1.0, 0.0, 0.0],
            children=[n_bb_b_idx, n_bb_t_idx]
        )
        n_blade_b_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_blade_b)

        n_rotor = Node(
            name=f"ROTOR_{i+1:02d}",
            translation=rotor_offset,
            children=[n_hub_inst_idx, n_blade_a_idx, n_blade_b_idx]
        )
        n_rotor_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_rotor)

        n_motor = Node(
            name=f"MOTOR_{i+1:02d}",
            translation=motor_pos,
            children=[n_mcan_idx, n_mred_idx, n_mnav_idx, n_rotor_idx]
        )
        n_motor_idx = len(builder.gltf.nodes)
        builder.gltf.nodes.append(n_motor)
        node_propulsion.children.append(n_motor_idx)

    # -------------------------------------------------------------------------
    # 5. Landing Gear Assembly (Wide Splayed Stance, Unobstructed Camera FOV)
    # -------------------------------------------------------------------------
    lg_cfg = manifest["landing_gear"]
    skid_y = -float(lg_cfg["ground_clearance_m"]) # -0.360m
    skid_len = 0.480                                # 0.480m skid length
    track_w = 0.560                                 # 0.560m wide stance
    sx_val = track_w * 0.5                          # 0.280m

    gear_carbon = []
    gear_metal = []
    for sx in [-sx_val, sx_val]:
        gear_carbon.append(create_oriented_cylinder([sx, skid_y, -skid_len * 0.5], [sx, skid_y, skid_len * 0.5], 0.012, 24))
        gear_metal.append(create_sphere_mesh(radius=0.014, rings=12, sectors=16, center_x=sx, center_y=skid_y, center_z=skid_len * 0.5))
        gear_metal.append(create_sphere_mesh(radius=0.014, rings=12, sectors=16, center_x=sx, center_y=skid_y, center_z=-skid_len * 0.5))
        
        # Splayed A-Frames (Forward Z = -0.060m, Aft Z = +0.160m)
        top_f = [sx * 0.54, -0.068, -0.060]
        bot_f = [sx, skid_y + 0.010, -0.180]
        gear_carbon.append(create_oriented_cylinder(top_f, bot_f, 0.010, 24))
        gear_metal.append(create_box_mesh(0.036, 0.022, 0.036, top_f[0], top_f[1], top_f[2]))
        gear_metal.append(create_cylinder_mesh(radius=0.015, height=0.030, segments=20, center_x=bot_f[0], center_y=bot_f[1], center_z=bot_f[2], axis='z'))

        top_r = [sx * 0.54, -0.068, 0.160]
        bot_r = [sx, skid_y + 0.010, 0.180]
        gear_carbon.append(create_oriented_cylinder(top_r, bot_r, 0.010, 24))
        gear_metal.append(create_box_mesh(0.036, 0.022, 0.036, top_r[0], top_r[1], top_r[2]))
        gear_metal.append(create_cylinder_mesh(radius=0.015, height=0.030, segments=20, center_x=bot_r[0], center_y=bot_r[1], center_z=bot_r[2], axis='z'))

        # Longitudinal Side Stiffener
        p_sf = [sx * 0.88, skid_y + 0.080, -0.150]
        p_sr = [sx * 0.88, skid_y + 0.080, 0.150]
        gear_carbon.append(create_oriented_cylinder(p_sf, p_sr, 0.007, 16))

    # Aft under-belly tie rod behind camera
    gear_carbon.append(create_oriented_cylinder([-sx_val * 0.54, -0.076, 0.160], [sx_val * 0.54, -0.076, 0.160], 0.008, 16))

    p_gc, n_gc, i_gc = combine_meshes(gear_carbon)
    m_gc_idx = builder.add_mesh(p_gc, n_gc, i_gc, 1, "Landing_Gear_Carbon")
    n_gear_main = Node(name="LANDING_GEAR_MAIN", mesh=m_gc_idx)
    n_gear_main_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_gear_main)
    node_landing_gear.children.append(n_gear_main_idx)

    # -------------------------------------------------------------------------
    # 6. Payload & 3-Axis Gimbal System (Precision Isolated Mount + Quad Sensor Turret)
    # -------------------------------------------------------------------------
    g_mount_pos = [float(x) for x in manifest["payload_mount"]["mount_location"]] # [0, -0.088, -0.12]

    # Mount Base with 4x Silicone Dampers & Red Compression Bands
    p_pm_met, n_pm_met, i_pm_met = combine_meshes([
        create_box_mesh(0.120, 0.005, 0.120, 0.0, 0.010, 0.0),
        create_box_mesh(0.110, 0.005, 0.110, 0.0, -0.012, 0.0)
    ])
    m_pm_met_idx = builder.add_mesh(p_pm_met, n_pm_met, i_pm_met, 9, "Payload_Mount_Plates")

    p_damp, n_damp, i_damp = combine_meshes([
        create_sphere_mesh(radius=0.009, rings=12, sectors=16, center_x=-0.045, center_y=-0.001, center_z=-0.045),
        create_sphere_mesh(radius=0.009, rings=12, sectors=16, center_x=0.045, center_y=-0.001, center_z=-0.045),
        create_sphere_mesh(radius=0.009, rings=12, sectors=16, center_x=-0.045, center_y=-0.001, center_z=0.045),
        create_sphere_mesh(radius=0.009, rings=12, sectors=16, center_x=0.045, center_y=-0.001, center_z=0.045)
    ])
    m_damp_idx = builder.add_mesh(p_damp, n_damp, i_damp, 14, "Payload_Dampers")

    n_pm = Node(name="PAYLOAD_MOUNT", translation=g_mount_pos, mesh=m_pm_met_idx)
    n_pm_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_pm)
    node_payload.children.append(n_pm_idx)

    # Yaw Motor (Pan 360°)
    p_yaw, n_yaw, i_yaw = combine_meshes([
        create_cylinder_mesh(radius=0.036, height=0.014, segments=32, center_x=0.0, center_y=-0.007, center_z=0.0, axis='y'),
        create_cylinder_mesh(radius=0.024, height=0.006, segments=24, center_x=0.0, center_y=-0.016, center_z=0.0, axis='y')
    ])
    m_yaw_idx = builder.add_mesh(p_yaw, n_yaw, i_yaw, 9, "Gimbal_Yaw_Mesh")
    n_gyaw = Node(name="GIMBAL_YAW", translation=[0.0, -0.020, 0.0], mesh=m_yaw_idx)
    n_gyaw_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_gyaw)
    n_pm.children.append(n_gyaw_idx)

    # Pitch Rigid U-Yoke Arm (-90° to +30°)
    p_pitch, n_pitch, i_pitch = combine_meshes([
        create_box_mesh(0.144, 0.012, 0.034, 0.0, -0.006, 0.0),
        create_box_mesh(0.014, 0.070, 0.030, 0.072, -0.042, 0.0),
        create_cylinder_mesh(radius=0.024, height=0.016, segments=32, center_x=0.080, center_y=-0.076, center_z=0.0, axis='x'),
        create_box_mesh(0.014, 0.070, 0.030, -0.072, -0.042, 0.0),
        create_cylinder_mesh(radius=0.020, height=0.012, segments=24, center_x=-0.078, center_y=-0.076, center_z=0.0, axis='x')
    ])
    m_pitch_idx = builder.add_mesh(p_pitch, n_pitch, i_pitch, 9, "Gimbal_Pitch_Yoke")
    n_gpitch = Node(name="GIMBAL_PITCH", translation=[0.0, -0.014, 0.0], mesh=m_pitch_idx)
    n_gpitch_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_gpitch)
    n_gyaw.children.append(n_gpitch_idx)

    # Roll Joint (±45°)
    n_groll = Node(name="GIMBAL_ROLL", translation=[0.0, -0.076, 0.0])
    n_groll_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_groll)
    n_gpitch.children.append(n_groll_idx)

    # Quad Sensor Turret Body (Facing strictly -Z Forward)
    p_cc, n_cc, i_cc = combine_meshes([
        create_box_mesh(0.106, 0.112, 0.116, 0.0, 0.0, 0.0),
        create_box_mesh(0.098, 0.104, 0.008, 0.0, 0.0, -0.058),
        create_cylinder_mesh(radius=0.018, height=0.012, segments=24, center_x=0.058, center_y=0.0, center_z=0.0, axis='x'),
        create_cylinder_mesh(radius=0.018, height=0.012, segments=24, center_x=-0.058, center_y=0.0, center_z=0.0, axis='x')
    ])
    m_cc_idx = builder.add_mesh(p_cc, n_cc, i_cc, 0, "Camera_Body_Mesh")
    n_cbody = Node(name="CAMERA_BODY", mesh=m_cc_idx)
    n_cbody_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_cbody)
    n_groll.children.append(n_cbody_idx)

    # Quad Sensor Lenses (1: 4K Emerald, 2: Thermal Purple, 3: SWIR Sapphire, 4: LRF Array)
    p_le, n_le, i_le = combine_meshes([create_cylinder_mesh(radius=0.0165, height=0.002, segments=32, center_x=0.026, center_y=0.026, center_z=-0.071, axis='z')])
    m_le_idx = builder.add_mesh(p_le, n_le, i_le, 11, "Lens_Daylight_Emerald")
    n_le = Node(name="LENS_4K_DAYLIGHT", mesh=m_le_idx)
    n_le_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_le)
    n_cbody.children.append(n_le_idx)

    p_lg, n_lg, i_lg = combine_meshes([create_cylinder_mesh(radius=0.0135, height=0.002, segments=32, center_x=-0.026, center_y=0.026, center_z=-0.070, axis='z')])
    m_lg_idx = builder.add_mesh(p_lg, n_lg, i_lg, 12, "Lens_Thermal_Germanium")
    n_lg = Node(name="LENS_FLIR_THERMAL", mesh=m_lg_idx)
    n_lg_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_lg)
    n_cbody.children.append(n_lg_idx)

    p_ls, n_ls, i_ls = combine_meshes([create_cylinder_mesh(radius=0.0175, height=0.002, segments=32, center_x=-0.024, center_y=-0.026, center_z=-0.071, axis='z')])
    m_ls_idx = builder.add_mesh(p_ls, n_ls, i_ls, 13, "Lens_SWIR_Sapphire")
    n_ls = Node(name="LENS_SWIR_NIR", mesh=m_ls_idx)
    n_ls_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_ls)
    n_cbody.children.append(n_ls_idx)

    p_lr, n_lr, i_lr = combine_meshes([
        create_cylinder_mesh(radius=0.005, height=0.002, segments=16, center_x=0.017, center_y=-0.035, center_z=-0.069, axis='z'),
        create_cylinder_mesh(radius=0.005, height=0.002, segments=16, center_x=0.035, center_y=-0.035, center_z=-0.069, axis='z'),
        create_cylinder_mesh(radius=0.005, height=0.002, segments=16, center_x=0.017, center_y=-0.017, center_z=-0.069, axis='z'),
        create_cylinder_mesh(radius=0.005, height=0.002, segments=16, center_x=0.035, center_y=-0.017, center_z=-0.069, axis='z')
    ])
    m_lr_idx = builder.add_mesh(p_lr, n_lr, i_lr, 5, "Lens_LRF_Array")
    n_lr = Node(name="LENS_LRF_ARRAY", mesh=m_lr_idx)
    n_lr_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_lr)
    n_cbody.children.append(n_lr_idx)

    # -------------------------------------------------------------------------
    # 7. Avionics & Strobe Lighting (Direct Airframe Body Integration - NO Boxes)
    # -------------------------------------------------------------------------
    av_parts = []

    # 6x Multi-Band Whip Antennas directly embedded into the drone's curved body
    antenna_mounts = [
        (-0.045, 0.076, 0.080, 0.100, -0.004),
        ( 0.045, 0.076, 0.080, 0.100,  0.004),
        (-0.055, 0.066, 0.135, 0.120, -0.006),
        ( 0.055, 0.066, 0.135, 0.120,  0.006),
        (-0.040, 0.053, 0.185, 0.090, -0.004),
        ( 0.040, 0.053, 0.185, 0.090,  0.004)
    ]

    for ax, ay, az, ah, dx in antenna_mounts:
        # Sleek circular flush-mounted SMA grommet/bezel seated right into the carbon skin
        av_parts.append(create_cylinder_mesh(radius=0.0070, height=0.008, segments=16, center_x=ax, center_y=ay + 0.004, center_z=az, axis='y'))
        av_parts.append(create_cylinder_mesh(radius=0.0055, height=0.006, segments=16, center_x=ax, center_y=ay + 0.009, center_z=az, axis='y'))
        
        # High-Gain Flexible Whip Antenna Mast
        p1 = [ax, ay + 0.012, az]
        p2 = [ax + dx, ay + 0.012 + ah, az]
        av_parts.append(create_oriented_cylinder(p1, p2, 0.0022, 12))
        av_parts.append(create_sphere_mesh(radius=0.0032, rings=8, sectors=12, center_x=p2[0], center_y=p2[1], center_z=p2[2]))

    # Low-Profile Circular GNSS Dome Receiver Pod (seated flush on hull centerline)
    gnss_y = 0.070
    gnss_z = 0.120
    av_parts.append(create_cylinder_mesh(radius=0.022, height=0.006, segments=24, center_x=0.0, center_y=gnss_y + 0.003, center_z=gnss_z, axis='y'))
    av_parts.append(create_sphere_mesh(radius=0.020, rings=12, sectors=24, center_x=0.0, center_y=gnss_y + 0.006, center_z=gnss_z))

    p_av, n_av, i_av = combine_meshes(av_parts)
    m_av_idx = builder.add_mesh(p_av, n_av, i_av, 9, "Avionics_Mesh")
    n_av = Node(name="ANTENNA_SYSTEM", mesh=m_av_idx)
    n_av_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_av)
    node_avionics.children.append(n_av_idx)

    strobe_y = 0.080
    strobe_z = 0.025
    p_bb, n_bb, i_bb = combine_meshes([create_cylinder_mesh(radius=0.016, height=0.006, segments=24, center_x=0.0, center_y=strobe_y + 0.003, center_z=strobe_z, axis='y')])
    m_bb_idx = builder.add_mesh(p_bb, n_bb, i_bb, 9, "Beacon_Base_Mesh")
    n_bbase = Node(name="BEACON_BASE", mesh=m_bb_idx)
    n_bbase_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bbase)
    node_avionics.children.append(n_bbase_idx)

    p_bd, n_bd, i_bd = combine_meshes([create_sphere_mesh(radius=0.012, rings=12, sectors=16, center_x=0.0, center_y=strobe_y + 0.008, center_z=strobe_z)])
    m_bd_idx = builder.add_mesh(p_bd, n_bd, i_bd, 4, "Beacon_Strobe_Mesh")
    n_bdome = Node(name="LIGHTS_BEACON", mesh=m_bd_idx)
    n_bdome_idx = len(builder.gltf.nodes)
    builder.gltf.nodes.append(n_bdome)
    n_bbase.children.append(n_bdome_idx)

    # -------------------------------------------------------------------------
    # 8. Export All Deliverables
    # -------------------------------------------------------------------------
    out_master = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "00_MASTER", "GARUDA_HL_01_MASTER.glb")
    out_root1 = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01.glb")
    out_root2 = os.path.join(os.path.dirname(__file__), "..", "..", "models", "garuda_hl01.glb")
    out_godot = os.path.join(os.path.dirname(__file__), "..", "..", "models", "GARUDA_HL_01", "11_GODOT", "GARUDA_HL_01.glb")

    builder.export(out_master)
    builder.export(out_root1)
    builder.export(out_root2)
    builder.export(out_godot)
    print("[OK] Successfully assembled and exported all master deliverables.")

if __name__ == "__main__":
    assemble_master_asset()
