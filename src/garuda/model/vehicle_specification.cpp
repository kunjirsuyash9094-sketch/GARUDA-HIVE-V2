#include "garuda/model/vehicle_specification.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <numbers>

namespace garuda::model {

GarudaVehicleSpecification GarudaVehicleSpecification::create_canonical() noexcept {
    GarudaVehicleSpecification spec{};

    spec.model_name = "GARUDA-HL-01";
    spec.platform_class = "HEAVY_LIFT_OCTOCOPTER";
    spec.spec_version = 1;
    spec.schema_version = "2.0.0";

    spec.motor_span_m = 1.100;
    spec.arm_length_m = 0.550;
    spec.initial_rotor_angle_deg = 22.5;
    spec.angular_spacing_deg = 45.0;
    spec.rotor_count = 8;
    spec.blade_count_per_rotor = 2;
    spec.total_blade_count = 16;
    spec.propeller_radius_m = 0.2032; // 16-inch
    spec.ground_clearance_m = 0.360;

    // 1. Central Airframe
    spec.airframe = GarudaAirframe();

    // 2. Arms, Motors, and Rotors (Mathematically Generated)
    const double R_arm = spec.arm_length_m;
    const double motor_y_elev = 0.015;
    const double rotor_shaft_h = 0.058;

    for (uint8_t i = 0; i < 8; ++i) {
        double angle_deg = spec.initial_rotor_angle_deg + static_cast<double>(i) * spec.angular_spacing_deg;
        double angle_rad = angle_deg * (std::numbers::pi / 180.0);

        // Position in coordinate system (+X right, +Y up, -Z forward)
        // angle 22.5 deg corresponds to forward-starboard quadrant
        double pos_x = R_arm * std::sin(angle_rad);
        double pos_z = R_arm * std::cos(angle_rad);

        // Arm Module
        Transform3D arm_tf{};
        arm_tf.translation = {0.0, 0.0, 0.0};
        // Arm rotates around Y to point along the radial angle
        arm_tf.rotation = Transform3D::from_euler_deg(-angle_deg + 90.0, 0.0, 0.0).rotation;
        spec.arms[i] = GarudaArm(i + 1, angle_deg, R_arm, arm_tf);
        spec.arms[i].root_attachment_point = {0.12 * std::sin(angle_rad), 0.0, 0.12 * std::cos(angle_rad)};
        spec.arms[i].tip_motor_mount_point = {pos_x, motor_y_elev, pos_z};

        // Motor Module (Attached at arm tip)
        Transform3D motor_tf{};
        motor_tf.translation = {pos_x, motor_y_elev, pos_z};
        spec.motors[i] = GarudaMotor(i + 1, spec.arms[i].id, motor_tf);

        // Rotor Module (Attached to motor shaft, alternating CW / CCW)
        RotorDirection dir = (i % 2 == 0) ? RotorDirection::CW : RotorDirection::CCW;
        Transform3D rotor_tf{};
        rotor_tf.translation = {0.0, rotor_shaft_h, 0.0}; // Relative to motor shaft center
        spec.rotors[i] = GarudaRotor(i + 1, i + 1, dir, spec.motors[i].id, rotor_tf);
        spec.rotors[i].radius_m = spec.propeller_radius_m;
    }

    // 3. Landing Gear
    spec.landing_gear = GarudaLandingGear();

    // 4. Payload Mount
    spec.payload_mount = GarudaPayloadMount();

    // 5. Gimbal 3-Axis Joint Hierarchy
    spec.gimbal_joints.clear();
    // YAW joint
    Transform3D yaw_tf{};
    yaw_tf.translation = {0.0, -0.023, 0.0};
    spec.gimbal_joints.emplace_back("GIMBAL_YAW", "PAYLOAD_MOUNT", "YAW", Vec3d{0, 1, 0}, -180.0, 180.0, yaw_tf);

    // PITCH joint
    Transform3D pitch_tf{};
    pitch_tf.translation = {0.0, -0.017, 0.0};
    spec.gimbal_joints.emplace_back("GIMBAL_PITCH", "GIMBAL_YAW", "PITCH", Vec3d{1, 0, 0}, -90.0, 30.0, pitch_tf);

    // ROLL joint
    Transform3D roll_tf{};
    roll_tf.translation = {0.0, 0.0, 0.0};
    spec.gimbal_joints.emplace_back("GIMBAL_ROLL", "GIMBAL_PITCH", "ROLL", Vec3d{0, 0, 1}, -45.0, 45.0, roll_tf);

    // 6. Avionics
    spec.avionics = GarudaAvionics();

    return spec;
}

std::string GarudaVehicleSpecification::compute_specification_hash() const noexcept {
    // Deterministic FNV-1a 64-bit hash of canonical parameters
    uint64_t hash = 14695981039346656037ULL;
    auto hash_combine = [&hash](double v) {
        union { double d; uint64_t u; } u_val{v};
        for (int i = 0; i < 8; ++i) {
            uint8_t byte = static_cast<uint8_t>((u_val.u >> (i * 8)) & 0xFF);
            hash ^= byte;
            hash *= 1099511628211ULL;
        }
    };
    auto hash_str = [&hash](const std::string& s) {
        for (char c : s) {
            hash ^= static_cast<uint8_t>(c);
            hash *= 1099511628211ULL;
        }
    };

    hash_str(model_name);
    hash_str(platform_class);
    hash_combine(static_cast<double>(spec_version));
    hash_combine(motor_span_m);
    hash_combine(arm_length_m);
    hash_combine(initial_rotor_angle_deg);
    hash_combine(angular_spacing_deg);
    hash_combine(static_cast<double>(rotor_count));

    for (const auto& m : motors) {
        hash_combine(m.local_transform.translation.x);
        hash_combine(m.local_transform.translation.y);
        hash_combine(m.local_transform.translation.z);
    }

    std::stringstream ss;
    ss << "SPEC-SHA256-" << std::hex << std::setw(16) << std::setfill('0') << hash;
    return ss.str();
}

} // namespace garuda::model
