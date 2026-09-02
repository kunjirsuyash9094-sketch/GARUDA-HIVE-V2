#pragma once
#include "garuda/model/component_types.hpp"
#include "garuda/model/transform_3d.hpp"
#include "garuda/model/vehicle_components.hpp"
#include <string>
#include <vector>
#include <array>
#include <memory>

namespace garuda::model {

/**
 * @brief Authoritative, immutable Single Source of Truth for GARUDA-HL-01 UAV Mechanical Specification.
 */
struct GarudaVehicleSpecification {
    // 1. Identity & Versioning
    std::string model_name{"GARUDA-HL-01"};
    std::string platform_class{"HEAVY_LIFT_OCTOCOPTER"};
    uint32_t    spec_version{1};
    std::string schema_version{"2.0.0"};

    // 2. Coordinate Conventions
    std::string forward_axis{"-Z"};
    std::string right_axis{"+X"};
    std::string up_axis{"+Y"};
    std::string rotor_shaft_axis{"+Y"};

    // 3. Authoritative Dimensions
    double  motor_span_m{1.100};
    double  arm_length_m{0.550};
    double  initial_rotor_angle_deg{22.5};
    double  angular_spacing_deg{45.0};
    uint8_t rotor_count{8};
    uint8_t blade_count_per_rotor{2};
    uint8_t total_blade_count{16};
    double  propeller_radius_m{0.2032}; // 16-inch diameter
    double  ground_clearance_m{0.360};

    // 4. Subsystem Components
    GarudaAirframe                  airframe{};
    std::array<GarudaArm, 8>        arms{};
    std::array<GarudaMotor, 8>      motors{};
    std::array<GarudaRotor, 8>      rotors{};
    GarudaLandingGear               landing_gear{};
    GarudaPayloadMount              payload_mount{};
    std::vector<GarudaGimbalJoint>  gimbal_joints{};
    GarudaAvionics                  avionics{};

    // 5. Factory Method
    [[nodiscard]] static GarudaVehicleSpecification create_canonical() noexcept;

    // 6. Metadata & Hash computation
    [[nodiscard]] std::string compute_specification_hash() const noexcept;
};

} // namespace garuda::model
