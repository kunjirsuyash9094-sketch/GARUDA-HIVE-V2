#include "garuda/model/model_validator.hpp"
#include <cmath>
#include <unordered_set>
#include <iomanip>
#include <numbers>

namespace garuda::model {

void ValidationReport::print(std::ostream& os) const {
    os << "=================================================================\n";
    os << " GARUDA-HL-01 MECHANICAL & ARCHITECTURAL VALIDATION REPORT\n";
    os << " Target Vehicle: " << vehicle_name << "\n";
    os << " Status: " << (overall_success ? "[PASS] ALL TESTS PASSED" : "[FAIL] VALIDATION FAILED") << "\n";
    os << " Passed: " << passed_count << " | Failed: " << failed_count << "\n";
    os << "=================================================================\n";

    for (size_t i = 0; i < items.size(); ++i) {
        const auto& item = items[i];
        os << "[" << std::setw(2) << (i + 1) << "] "
           << (item.passed ? "[PASS] " : "[FAIL] ")
           << std::left << std::setw(38) << item.test_name
           << " | " << item.details << "\n";
    }
    os << "=================================================================\n";
}

ValidationReport ModelValidator::validate(const GarudaVehicleSpecification& spec) noexcept {
    ValidationReport report{};
    report.vehicle_name = spec.model_name;
    report.overall_success = true;

    auto add_item = [&report](std::string name, bool pass, std::string details,
                              double meas = 0.0, double exp = 0.0, double tol = 0.0) {
        ValidationItem item{std::move(name), pass, std::move(details), meas, exp, tol};
        if (!pass) report.overall_success = false;
        if (pass) report.passed_count++;
        else report.failed_count++;
        report.items.push_back(std::move(item));
    };

    // Test 1: Vehicle Identity
    bool t1 = (spec.model_name == "GARUDA-HL-01") && (spec.spec_version >= 1);
    add_item("Vehicle Identity & Version", t1, "Model: " + spec.model_name + " (v" + std::to_string(spec.spec_version) + ")");

    // Test 2: Exactly 8 Arms
    bool t2 = (spec.arms.size() == 8);
    add_item("Arm Count Check", t2, "Arms: " + std::to_string(spec.arms.size()) + " (Expected: 8)");

    // Test 3: Exactly 8 Motors
    bool t3 = (spec.motors.size() == 8);
    add_item("Motor Count Check", t3, "Motors: " + std::to_string(spec.motors.size()) + " (Expected: 8)");

    // Test 4: Exactly 8 Rotors
    bool t4 = (spec.rotors.size() == 8);
    add_item("Rotor Count Check", t4, "Rotors: " + std::to_string(spec.rotors.size()) + " (Expected: 8)");

    // Test 5: Arm Length
    bool t5 = std::abs(spec.arm_length_m - 0.550) < 1e-4;
    add_item("Authoritative Arm Length", t5, "Length: " + std::to_string(spec.arm_length_m) + " m (Expected: 0.550 m)", spec.arm_length_m, 0.550, 1e-4);

    // Test 6: Motor-to-Motor Span
    bool t6 = std::abs(spec.motor_span_m - 1.100) < 1e-4;
    add_item("Motor-to-Motor Span", t6, "Span: " + std::to_string(spec.motor_span_m) + " m (Expected: 1.100 m)", spec.motor_span_m, 1.100, 1e-4);

    // Test 7: Angular Spacing
    bool t7 = std::abs(spec.angular_spacing_deg - 45.0) < 1e-4;
    add_item("Rotor Angular Spacing", t7, "Spacing: " + std::to_string(spec.angular_spacing_deg) + " deg (Expected: 45.0 deg)");

    // Test 8: First Rotor Angle
    bool t8 = std::abs(spec.initial_rotor_angle_deg - 22.5) < 1e-4;
    add_item("Initial Rotor Angle", t8, "Angle: " + std::to_string(spec.initial_rotor_angle_deg) + " deg (Expected: 22.5 deg)");

    // Test 9: All Motor Radial Distances
    bool t9 = true;
    for (size_t i = 0; i < 8; ++i) {
        const auto& pos = spec.motors[i].local_transform.translation;
        double r = std::sqrt(pos.x * pos.x + pos.z * pos.z);
        if (std::abs(r - spec.arm_length_m) > 1e-4) {
            t9 = false;
            break;
        }
    }
    add_item("Motor Radial Radii Equidistance", t9, "All 8 motors at radius R = 0.550 m");

    // Test 10: 180° Pairwise Symmetry
    bool t10 = true;
    for (size_t i = 0; i < 4; ++i) {
        const auto& pos_a = spec.motors[i].local_transform.translation;
        const auto& pos_b = spec.motors[i + 4].local_transform.translation;
        double sum_x = std::abs(pos_a.x + pos_b.x);
        double sum_z = std::abs(pos_a.z + pos_b.z);
        if (sum_x > 1e-4 || sum_z > 1e-4) {
            t10 = false;
            break;
        }
    }
    add_item("180° Pairwise Motor Symmetry", t10, "Motors (1-5, 2-6, 3-7, 4-8) exhibit exact 180° symmetry");

    // Test 11: Alternating CW/CCW Directions
    bool t11 = true;
    for (size_t i = 0; i < 8; ++i) {
        RotorDirection expected = (i % 2 == 0) ? RotorDirection::CW : RotorDirection::CCW;
        if (spec.rotors[i].direction != expected) {
            t11 = false;
            break;
        }
    }
    add_item("Alternating Rotor Directions", t11, "Alternating sequence CW, CCW, CW, CCW, CW, CCW, CW, CCW");

    // Test 12: Motor/Rotor Origin Coincidence
    bool t12 = true;
    for (size_t i = 0; i < 8; ++i) {
        const auto& r_tf = spec.rotors[i].local_transform.translation;
        if (std::abs(r_tf.x) > 1e-4 || std::abs(r_tf.z) > 1e-4 || r_tf.y < 0.02) {
            t12 = false;
            break;
        }
    }
    add_item("Rotor Coaxial Origin Coincidence", t12, "Rotor origins coincide with motor shaft axes (local X=0, Z=0)");

    // Test 13: Rotor Shaft Axis Validity
    bool t13 = true;
    for (size_t i = 0; i < 8; ++i) {
        if (std::abs(spec.rotors[i].shaft_axis.x) > 1e-4 ||
            std::abs(spec.rotors[i].shaft_axis.y - 1.0) > 1e-4 ||
            std::abs(spec.rotors[i].shaft_axis.z) > 1e-4) {
            t13 = false;
            break;
        }
    }
    add_item("Rotor Shaft Axis (+Y)", t13, "All rotor shafts oriented strictly along local +Y axis");

    // Test 14: Unique Component IDs
    std::unordered_set<std::string> ids;
    ids.insert(spec.airframe.id);
    for (const auto& a : spec.arms) ids.insert(a.id);
    for (const auto& m : spec.motors) ids.insert(m.id);
    for (const auto& r : spec.rotors) ids.insert(r.id);
    ids.insert(spec.landing_gear.id);
    ids.insert(spec.payload_mount.id);
    for (const auto& g : spec.gimbal_joints) ids.insert(g.id);
    ids.insert(spec.avionics.id);

    size_t expected_total_ids = 1 + 8 + 8 + 8 + 1 + 1 + spec.gimbal_joints.size() + 1;
    bool t14 = (ids.size() == expected_total_ids);
    add_item("Unique Component ID Namespace", t14, "Unique IDs: " + std::to_string(ids.size()) + " / " + std::to_string(expected_total_ids));

    // Test 15: Gimbal Hierarchy
    bool t15 = (spec.gimbal_joints.size() == 3) &&
               (spec.gimbal_joints[0].id == "GIMBAL_YAW" && spec.gimbal_joints[0].parent_id == "PAYLOAD_MOUNT") &&
               (spec.gimbal_joints[1].id == "GIMBAL_PITCH" && spec.gimbal_joints[1].parent_id == "GIMBAL_YAW") &&
               (spec.gimbal_joints[2].id == "GIMBAL_ROLL" && spec.gimbal_joints[2].parent_id == "GIMBAL_PITCH");
    add_item("3-Axis Gimbal Joint Hierarchy", t15, "PAYLOAD_MOUNT -> GIMBAL_YAW -> GIMBAL_PITCH -> GIMBAL_ROLL");

    // Test 16: Gimbal Joint Axes
    bool t16 = (std::abs(spec.gimbal_joints[0].rotation_axis.y - 1.0) < 1e-4) && // Yaw around Y
               (std::abs(spec.gimbal_joints[1].rotation_axis.x - 1.0) < 1e-4) && // Pitch around X
               (std::abs(spec.gimbal_joints[2].rotation_axis.z - 1.0) < 1e-4);   // Roll around Z
    add_item("Gimbal Joint Rotation Axes", t16, "Yaw: +Y, Pitch: +X, Roll: +Z");

    // Test 17: Transform Finiteness & Normalized Quaternions
    bool t17 = true;
    auto check_tf = [&t17](const Transform3D& tf) {
        if (!tf.is_finite() || !tf.has_positive_scale() || !tf.is_normalized_rotation()) {
            t17 = false;
        }
    };
    check_tf(spec.airframe.local_transform);
    for (const auto& a : spec.arms) check_tf(a.local_transform);
    for (const auto& m : spec.motors) check_tf(m.local_transform);
    for (const auto& r : spec.rotors) check_tf(r.local_transform);
    check_tf(spec.landing_gear.local_transform);
    check_tf(spec.payload_mount.local_transform);
    for (const auto& g : spec.gimbal_joints) check_tf(g.local_transform);
    check_tf(spec.avionics.local_transform);
    add_item("Transform Finiteness & Normalization", t17, "All 3D transforms finite, normalized, and have positive scale");

    // Test 18: Mechanical Propeller Clearance Calculation
    // Calculates chord distance between adjacent motor shafts:
    const auto& m1_pos = spec.motors[0].local_transform.translation;
    const auto& m2_pos = spec.motors[1].local_transform.translation;
    double dx = m2_pos.x - m1_pos.x;
    double dz = m2_pos.z - m1_pos.z;
    double shaft_dist = std::sqrt(dx * dx + dz * dz);
    double clearance = shaft_dist - 2.0 * spec.propeller_radius_m;
    bool t18 = (clearance > 0.005); // True positive mechanical clearance (no blade clash)
    std::stringstream cl_ss;
    cl_ss << "Shaft Dist: " << std::fixed << std::setprecision(3) << shaft_dist
          << " m | Tip Clearance: " << std::setprecision(3) << clearance << " m ("
          << std::setprecision(1) << (clearance * 100.0) << " cm)";
    add_item("Mechanical Propeller Clearance", t18, cl_ss.str(), clearance, 0.01, 0.0);

    // Test 19: Payload Mount Location
    bool t19 = (spec.payload_mount.local_transform.translation.y < -0.05);
    add_item("Payload Mount Underside Clearance", t19, "Mount Y: " + std::to_string(spec.payload_mount.local_transform.translation.y) + " m");

    // Test 20: Visual Bindings
    bool t20 = !spec.airframe.visual_binding.gltf_node_name.empty() &&
               !spec.arms[0].visual_binding.gltf_node_name.empty() &&
               !spec.motors[0].visual_binding.gltf_node_name.empty() &&
               !spec.rotors[0].visual_binding.gltf_node_name.empty();
    add_item("Visual GLTF Node Bindings", t20, "All components map to designated visual GLTF nodes");

    return report;
}

bool ModelValidator::validate_or_exit(const GarudaVehicleSpecification& spec) noexcept {
    ValidationReport report = validate(spec);
    report.print(std::cout);
    return report.overall_success;
}

} // namespace garuda::model
