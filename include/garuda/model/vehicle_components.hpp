#pragma once
#include "garuda/model/component_types.hpp"
#include "garuda/model/transform_3d.hpp"
#include <string>
#include <vector>
#include <array>

namespace garuda::model {

/**
 * @brief Base class for all authoritative C++ UAV mechanical components.
 */
struct GarudaComponentBase {
    std::string id{};
    std::string name{};
    ComponentCategory category{ComponentCategory::AIRFRAME};
    std::string parent_id{};
    Transform3D local_transform{Transform3D::identity()};
    VisualNodeBinding visual_binding{};
    double mass_kg{0.0};
};

/**
 * @brief Central Faceted Aerodynamic Fuselage & Structural Chassis.
 */
struct GarudaAirframe : public GarudaComponentBase {
    double length_m{0.460};
    double width_m{0.340};
    double height_m{0.160};
    Vec3d  center_of_mass_offset{0.0, 0.0, 0.0};
    bool   has_avionics_bay{true};
    bool   has_payload_rail{true};

    GarudaAirframe() {
        id = "AIRFRAME";
        name = "Faceted Aerodynamic Fuselage";
        category = ComponentCategory::AIRFRAME;
        parent_id = "ROOT";
        visual_binding.gltf_node_name = "BODY_MAIN";
        visual_binding.gltf_mesh_name = "BODY_MAIN";
        visual_binding.glb_relative_path = "01_AIRFRAME/GARUDA_BODY.glb";
        visual_binding.default_material = MaterialCategory::STEALTH_CARBON;
        mass_kg = 2.40;
    }
};

/**
 * @brief High-Modulus Carbon Fiber Structural Boom Arm.
 */
struct GarudaArm : public GarudaComponentBase {
    uint8_t arm_index{1};       // 1 to 8
    double  radial_angle_deg{22.5};
    double  length_m{0.550};    // 0.550m from chassis socket center to motor shaft center
    double  tube_radius_m{0.015};
    Vec3d   root_attachment_point{0.0, 0.0, 0.0};
    Vec3d   tip_motor_mount_point{0.0, 0.0, 0.0};

    GarudaArm() = default;
    GarudaArm(uint8_t idx, double angle_deg, double length, const Transform3D& tf)
        : arm_index(idx), radial_angle_deg(angle_deg), length_m(length) {
        id = "ARM_" + (idx < 10 ? std::string("0") : "") + std::to_string(idx);
        name = "Carbon Boom Arm " + std::to_string(idx);
        category = ComponentCategory::ARM;
        parent_id = "AIRFRAME";
        local_transform = tf;
        visual_binding.gltf_node_name = id;
        visual_binding.gltf_mesh_name = id;
        visual_binding.glb_relative_path = "02_ARMS/ARM_MASTER.glb";
        visual_binding.default_material = MaterialCategory::CARBON_TUBE;
        mass_kg = 0.32;
    }
};

/**
 * @brief Industrial 6215 Heavy-Lift High-Torque Brushless Motor.
 */
struct GarudaMotor : public GarudaComponentBase {
    uint8_t motor_index{1};     // 1 to 8
    double  kv_rating{380.0};   // RPM / V
    double  housing_radius_m{0.034};
    double  housing_height_m{0.040};
    double  shaft_radius_m{0.005};
    Vec3d   shaft_center_offset{0.0, 0.058, 0.0}; // Offset from motor base to shaft top

    GarudaMotor() = default;
    GarudaMotor(uint8_t idx, const std::string& parent_arm_id, const Transform3D& tf)
        : motor_index(idx) {
        id = "MOTOR_" + (idx < 10 ? std::string("0") : "") + std::to_string(idx);
        name = "6215 Brushless Motor " + std::to_string(idx);
        category = ComponentCategory::MOTOR;
        parent_id = parent_arm_id;
        local_transform = tf;
        visual_binding.gltf_node_name = id;
        visual_binding.gltf_mesh_name = id;
        visual_binding.glb_relative_path = "03_MOTORS/MOTOR_MASTER.glb";
        visual_binding.default_material = MaterialCategory::DARK_METAL;
        mass_kg = 0.38;
    }
};

/**
 * @brief Real 3D Folding Aerofoil Rotor (Hub + 2 Physical Cambered Blades).
 */
struct GarudaRotor : public GarudaComponentBase {
    uint8_t        rotor_index{1};      // 1 to 8
    uint8_t        motor_index{1};      // 1 to 8
    RotorDirection direction{RotorDirection::CW};
    Vec3d          shaft_axis{0.0, 1.0, 0.0}; // Local rotation axis (+Y)
    double         radius_m{0.2032};    // 16-inch prop (radius = 0.2032m)
    uint8_t        blade_count{2};      // 2 physical cambered blades
    double         twist_root_deg{16.0};
    double         twist_tip_deg{7.0};
    double         hub_radius_m{0.025};

    GarudaRotor() = default;
    GarudaRotor(uint8_t r_idx, uint8_t m_idx, RotorDirection dir, const std::string& parent_motor_id, const Transform3D& tf)
        : rotor_index(r_idx), motor_index(m_idx), direction(dir) {
        id = "ROTOR_" + (r_idx < 10 ? std::string("0") : "") + std::to_string(r_idx);
        name = "Aerofoil Rotor " + std::to_string(r_idx);
        category = ComponentCategory::ROTOR;
        parent_id = parent_motor_id;
        local_transform = tf;
        visual_binding.gltf_node_name = id;
        visual_binding.gltf_mesh_name = id;
        visual_binding.glb_relative_path = "04_PROPELLERS/ROTOR_MASTER.glb";
        visual_binding.default_material = MaterialCategory::PROPELLER_CARBON;
        mass_kg = 0.085;
    }
};

/**
 * @brief Inverted A-Frame Carbon Struts and Longitudinal Ground Skids.
 */
struct GarudaLandingGear : public GarudaComponentBase {
    double strut_height_m{0.360};
    double skid_length_m{0.620};
    double track_width_m{0.420};
    double ground_clearance_m{0.360};

    GarudaLandingGear() {
        id = "LANDING_GEAR";
        name = "Inverted A-Frame Landing Assembly";
        category = ComponentCategory::LANDING_GEAR;
        parent_id = "AIRFRAME";
        visual_binding.gltf_node_name = "LANDING_GEAR_MAIN";
        visual_binding.gltf_mesh_name = "LANDING_GEAR_MAIN";
        visual_binding.glb_relative_path = "05_LANDING_GEAR/LANDING_GEAR_MASTER.glb";
        visual_binding.default_material = MaterialCategory::CARBON_TUBE;
        mass_kg = 0.75;
    }
};

/**
 * @brief Standardized Modular Payload Attachment Interface.
 */
struct GarudaPayloadMount : public GarudaComponentBase {
    Vec3d  mount_location{0.0, -0.088, -0.120};
    double max_payload_mass_kg{6.50};
    std::string interface_type{"QUICK_RELEASE_RAIL_V2"};

    GarudaPayloadMount() {
        id = "PAYLOAD_MOUNT";
        name = "Modular Quick-Release Payload Rail";
        category = ComponentCategory::PAYLOAD_MOUNT;
        parent_id = "AIRFRAME";
        local_transform = Transform3D::from_translation(mount_location);
        visual_binding.gltf_node_name = "PAYLOAD_MOUNT";
        visual_binding.gltf_mesh_name = "PAYLOAD_MOUNT";
        visual_binding.glb_relative_path = "06_PAYLOAD/GIMBAL_MASTER.glb";
        visual_binding.default_material = MaterialCategory::DARK_METAL;
        mass_kg = 0.18;
    }
};

/**
 * @brief Modular 3-Axis Articulated Sensor Turret Joint.
 */
struct GarudaGimbalJoint : public GarudaComponentBase {
    std::string joint_type{"YAW"}; // "YAW", "PITCH", "ROLL"
    Vec3d       rotation_axis{0.0, 1.0, 0.0};
    double      min_angle_deg{-180.0};
    double      max_angle_deg{180.0};
    double      default_angle_deg{0.0};

    GarudaGimbalJoint() = default;
    GarudaGimbalJoint(const std::string& j_id, const std::string& p_id, const std::string& type,
                      const Vec3d& axis, double min_deg, double max_deg, const Transform3D& tf)
        : joint_type(type), rotation_axis(axis), min_angle_deg(min_deg), max_angle_deg(max_deg) {
        id = j_id;
        name = "Gimbal " + type + " Joint";
        category = ComponentCategory::GIMBAL_JOINT;
        parent_id = p_id;
        local_transform = tf;
        visual_binding.gltf_node_name = j_id;
        visual_binding.gltf_mesh_name = j_id;
        visual_binding.glb_relative_path = "06_PAYLOAD/GIMBAL_MASTER.glb";
        visual_binding.default_material = MaterialCategory::DARK_METAL;
        mass_kg = 0.12;
    }
};

/**
 * @brief Multi-Band Avionics, GNSS Mast, and Anti-Collision Strobe Beacon.
 */
struct GarudaAvionics : public GarudaComponentBase {
    uint8_t antenna_count{4};
    uint8_t gnss_receiver_count{2};
    double  strobe_flash_rate_hz{1.0};

    GarudaAvionics() {
        id = "AVIONICS";
        name = "Avionics Mast & Strobe Subsystem";
        category = ComponentCategory::AVIONICS;
        parent_id = "AIRFRAME";
        visual_binding.gltf_node_name = "ANTENNA_SYSTEM";
        visual_binding.gltf_mesh_name = "ANTENNA_SYSTEM";
        visual_binding.glb_relative_path = "07_AVIONICS/AVIONICS_MASTER.glb";
        visual_binding.default_material = MaterialCategory::DARK_METAL;
        mass_kg = 0.22;
    }
};

} // namespace garuda::model
