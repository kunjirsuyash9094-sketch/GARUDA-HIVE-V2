#include "garuda/model/vehicle_specification.hpp"
#include "garuda/model/model_validator.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;
using namespace garuda::model;

int main(int argc, char** argv) {
    std::cout << "=================================================================\n";
    std::cout << " GARUDA-HL-01 C++ AUTHORITATIVE MODEL MANIFEST GENERATOR\n";
    std::cout << "=================================================================\n";

    // 1. Instantiate Canonical Specification
    GarudaVehicleSpecification spec = GarudaVehicleSpecification::create_canonical();

    // 2. Validate Specification Before Manifest Export
    ValidationReport report = ModelValidator::validate(spec);
    report.print(std::cout);

    if (!report.overall_success) {
        std::cerr << "[!] CRITICAL ERROR: Model specification validation failed. Aborting manifest generation.\n";
        return 1;
    }

    // 3. Resolve Target Output Path
    std::string out_dir = "build/generated";
    if (argc > 1) {
        out_dir = argv[1];
    }
    fs::create_directories(out_dir);
    std::string out_file = (fs::path(out_dir) / "GARUDA_HL_01_MODEL_SPEC.json").string();

    std::ofstream ofs(out_file);
    if (!ofs.is_open()) {
        std::cerr << "[!] Failed to open output manifest file: " << out_file << "\n";
        return 1;
    }

    // 4. Deterministic JSON Serialization
    ofs << "{\n";
    ofs << "  \"_generator_notice\": \"AUTOMATICALLY GENERATED FROM C++20 AUTHORITATIVE SPECIFICATION (garuda::model::GarudaVehicleSpecification). DO NOT EDIT DIRECTLY.\",\n";
    ofs << "  \"schema_version\": \"" << spec.schema_version << "\",\n";
    ofs << "  \"spec_version\": " << spec.spec_version << ",\n";
    ofs << "  \"spec_hash\": \"" << spec.compute_specification_hash() << "\",\n";

    // Vehicle Identity
    ofs << "  \"vehicle_identity\": {\n";
    ofs << "    \"model_name\": \"" << spec.model_name << "\",\n";
    ofs << "    \"platform_class\": \"" << spec.platform_class << "\"\n";
    ofs << "  },\n";

    // Coordinate System
    ofs << "  \"coordinate_system\": {\n";
    ofs << "    \"forward_axis\": \"" << spec.forward_axis << "\",\n";
    ofs << "    \"right_axis\": \"" << spec.right_axis << "\",\n";
    ofs << "    \"up_axis\": \"" << spec.up_axis << "\",\n";
    ofs << "    \"rotor_shaft_axis\": \"" << spec.rotor_shaft_axis << "\",\n";
    ofs << "    \"convention\": \"Right-Handed (+X Right, +Y Up, -Z Forward)\"\n";
    ofs << "  },\n";

    // Authoritative Dimensions
    ofs << "  \"dimensions\": {\n";
    ofs << "    \"motor_span_m\": " << std::fixed << std::setprecision(4) << spec.motor_span_m << ",\n";
    ofs << "    \"arm_length_m\": " << spec.arm_length_m << ",\n";
    ofs << "    \"initial_rotor_angle_deg\": " << spec.initial_rotor_angle_deg << ",\n";
    ofs << "    \"angular_spacing_deg\": " << spec.angular_spacing_deg << ",\n";
    ofs << "    \"rotor_count\": " << static_cast<int>(spec.rotor_count) << ",\n";
    ofs << "    \"blade_count_per_rotor\": " << static_cast<int>(spec.blade_count_per_rotor) << ",\n";
    ofs << "    \"total_blade_count\": " << static_cast<int>(spec.total_blade_count) << ",\n";
    ofs << "    \"propeller_radius_m\": " << spec.propeller_radius_m << ",\n";
    ofs << "    \"ground_clearance_m\": " << spec.ground_clearance_m << "\n";
    ofs << "  },\n";

    // Airframe
    ofs << "  \"airframe\": {\n";
    ofs << "    \"id\": \"" << spec.airframe.id << "\",\n";
    ofs << "    \"name\": \"" << spec.airframe.name << "\",\n";
    ofs << "    \"length_m\": " << spec.airframe.length_m << ",\n";
    ofs << "    \"width_m\": " << spec.airframe.width_m << ",\n";
    ofs << "    \"height_m\": " << spec.airframe.height_m << ",\n";
    ofs << "    \"mass_kg\": " << spec.airframe.mass_kg << ",\n";
    ofs << "    \"visual_binding\": {\n";
    ofs << "      \"gltf_node_name\": \"" << spec.airframe.visual_binding.gltf_node_name << "\",\n";
    ofs << "      \"glb_relative_path\": \"" << spec.airframe.visual_binding.glb_relative_path << "\",\n";
    ofs << "      \"default_material\": \"" << to_string(spec.airframe.visual_binding.default_material) << "\"\n";
    ofs << "    }\n";
    ofs << "  },\n";

    // Arms Array
    ofs << "  \"arms\": [\n";
    for (size_t i = 0; i < spec.arms.size(); ++i) {
        const auto& a = spec.arms[i];
        ofs << "    {\n";
        ofs << "      \"id\": \"" << a.id << "\",\n";
        ofs << "      \"arm_index\": " << static_cast<int>(a.arm_index) << ",\n";
        ofs << "      \"radial_angle_deg\": " << a.radial_angle_deg << ",\n";
        ofs << "      \"length_m\": " << a.length_m << ",\n";
        ofs << "      \"tube_radius_m\": " << a.tube_radius_m << ",\n";
        ofs << "      \"root_attachment_point\": [" << a.root_attachment_point.x << ", " << a.root_attachment_point.y << ", " << a.root_attachment_point.z << "],\n";
        ofs << "      \"tip_motor_mount_point\": [" << a.tip_motor_mount_point.x << ", " << a.tip_motor_mount_point.y << ", " << a.tip_motor_mount_point.z << "],\n";
        ofs << "      \"visual_binding\": {\n";
        ofs << "        \"gltf_node_name\": \"" << a.visual_binding.gltf_node_name << "\",\n";
        ofs << "        \"glb_relative_path\": \"" << a.visual_binding.glb_relative_path << "\",\n";
        ofs << "        \"default_material\": \"" << to_string(a.visual_binding.default_material) << "\"\n";
        ofs << "      }\n";
        ofs << "    }" << (i + 1 < spec.arms.size() ? ",\n" : "\n");
    }
    ofs << "  ],\n";

    // Motors Array
    ofs << "  \"motors\": [\n";
    for (size_t i = 0; i < spec.motors.size(); ++i) {
        const auto& m = spec.motors[i];
        ofs << "    {\n";
        ofs << "      \"id\": \"" << m.id << "\",\n";
        ofs << "      \"motor_index\": " << static_cast<int>(m.motor_index) << ",\n";
        ofs << "      \"parent_id\": \"" << m.parent_id << "\",\n";
        ofs << "      \"kv_rating\": " << m.kv_rating << ",\n";
        ofs << "      \"position\": [" << m.local_transform.translation.x << ", " << m.local_transform.translation.y << ", " << m.local_transform.translation.z << "],\n";
        ofs << "      \"housing_radius_m\": " << m.housing_radius_m << ",\n";
        ofs << "      \"housing_height_m\": " << m.housing_height_m << ",\n";
        ofs << "      \"shaft_center_offset\": [" << m.shaft_center_offset.x << ", " << m.shaft_center_offset.y << ", " << m.shaft_center_offset.z << "],\n";
        ofs << "      \"visual_binding\": {\n";
        ofs << "        \"gltf_node_name\": \"" << m.visual_binding.gltf_node_name << "\",\n";
        ofs << "        \"glb_relative_path\": \"" << m.visual_binding.glb_relative_path << "\",\n";
        ofs << "        \"default_material\": \"" << to_string(m.visual_binding.default_material) << "\"\n";
        ofs << "      }\n";
        ofs << "    }" << (i + 1 < spec.motors.size() ? ",\n" : "\n");
    }
    ofs << "  ],\n";

    // Rotors Array
    ofs << "  \"rotors\": [\n";
    for (size_t i = 0; i < spec.rotors.size(); ++i) {
        const auto& r = spec.rotors[i];
        ofs << "    {\n";
        ofs << "      \"id\": \"" << r.id << "\",\n";
        ofs << "      \"rotor_index\": " << static_cast<int>(r.rotor_index) << ",\n";
        ofs << "      \"motor_index\": " << static_cast<int>(r.motor_index) << ",\n";
        ofs << "      \"parent_id\": \"" << r.parent_id << "\",\n";
        ofs << "      \"direction\": \"" << to_string(r.direction) << "\",\n";
        ofs << "      \"spin_multiplier\": " << (r.direction == RotorDirection::CW ? -1 : 1) << ",\n";
        ofs << "      \"shaft_axis\": [" << r.shaft_axis.x << ", " << r.shaft_axis.y << ", " << r.shaft_axis.z << "],\n";
        ofs << "      \"radius_m\": " << r.radius_m << ",\n";
        ofs << "      \"blade_count\": " << static_cast<int>(r.blade_count) << ",\n";
        ofs << "      \"twist_root_deg\": " << r.twist_root_deg << ",\n";
        ofs << "      \"twist_tip_deg\": " << r.twist_tip_deg << ",\n";
        ofs << "      \"hub_radius_m\": " << r.hub_radius_m << ",\n";
        ofs << "      \"local_offset\": [" << r.local_transform.translation.x << ", " << r.local_transform.translation.y << ", " << r.local_transform.translation.z << "],\n";
        ofs << "      \"visual_binding\": {\n";
        ofs << "        \"gltf_node_name\": \"" << r.visual_binding.gltf_node_name << "\",\n";
        ofs << "        \"glb_relative_path\": \"" << r.visual_binding.glb_relative_path << "\",\n";
        ofs << "        \"default_material\": \"" << to_string(r.visual_binding.default_material) << "\"\n";
        ofs << "      }\n";
        ofs << "    }" << (i + 1 < spec.rotors.size() ? ",\n" : "\n");
    }
    ofs << "  ],\n";

    // Landing Gear
    ofs << "  \"landing_gear\": {\n";
    ofs << "    \"id\": \"" << spec.landing_gear.id << "\",\n";
    ofs << "    \"strut_height_m\": " << spec.landing_gear.strut_height_m << ",\n";
    ofs << "    \"skid_length_m\": " << spec.landing_gear.skid_length_m << ",\n";
    ofs << "    \"track_width_m\": " << spec.landing_gear.track_width_m << ",\n";
    ofs << "    \"ground_clearance_m\": " << spec.landing_gear.ground_clearance_m << ",\n";
    ofs << "    \"visual_binding\": {\n";
    ofs << "      \"gltf_node_name\": \"" << spec.landing_gear.visual_binding.gltf_node_name << "\",\n";
    ofs << "      \"glb_relative_path\": \"" << spec.landing_gear.visual_binding.glb_relative_path << "\"\n";
    ofs << "    }\n";
    ofs << "  },\n";

    // Payload Mount & Gimbal
    ofs << "  \"payload_mount\": {\n";
    ofs << "    \"id\": \"" << spec.payload_mount.id << "\",\n";
    ofs << "    \"mount_location\": [" << spec.payload_mount.mount_location.x << ", " << spec.payload_mount.mount_location.y << ", " << spec.payload_mount.mount_location.z << "],\n";
    ofs << "    \"max_payload_mass_kg\": " << spec.payload_mount.max_payload_mass_kg << ",\n";
    ofs << "    \"interface_type\": \"" << spec.payload_mount.interface_type << "\"\n";
    ofs << "  },\n";

    ofs << "  \"gimbal\": [\n";
    for (size_t i = 0; i < spec.gimbal_joints.size(); ++i) {
        const auto& g = spec.gimbal_joints[i];
        ofs << "    {\n";
        ofs << "      \"id\": \"" << g.id << "\",\n";
        ofs << "      \"parent_id\": \"" << g.parent_id << "\",\n";
        ofs << "      \"joint_type\": \"" << g.joint_type << "\",\n";
        ofs << "      \"rotation_axis\": [" << g.rotation_axis.x << ", " << g.rotation_axis.y << ", " << g.rotation_axis.z << "],\n";
        ofs << "      \"min_angle_deg\": " << g.min_angle_deg << ",\n";
        ofs << "      \"max_angle_deg\": " << g.max_angle_deg << ",\n";
        ofs << "      \"local_offset\": [" << g.local_transform.translation.x << ", " << g.local_transform.translation.y << ", " << g.local_transform.translation.z << "],\n";
        ofs << "      \"visual_binding\": {\n";
        ofs << "        \"gltf_node_name\": \"" << g.visual_binding.gltf_node_name << "\"\n";
        ofs << "      }\n";
        ofs << "    }" << (i + 1 < spec.gimbal_joints.size() ? ",\n" : "\n");
    }
    ofs << "  ],\n";

    // Avionics
    ofs << "  \"avionics\": {\n";
    ofs << "    \"id\": \"" << spec.avionics.id << "\",\n";
    ofs << "    \"antenna_count\": " << static_cast<int>(spec.avionics.antenna_count) << ",\n";
    ofs << "    \"gnss_receiver_count\": " << static_cast<int>(spec.avionics.gnss_receiver_count) << ",\n";
    ofs << "    \"strobe_flash_rate_hz\": " << spec.avionics.strobe_flash_rate_hz << "\n";
    ofs << "  }\n";

    ofs << "}\n";
    ofs.close();

    std::cout << "[+] Authoritative Model Specification exported successfully to:\n    " << out_file << "\n";
    return 0;
}
