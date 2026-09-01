#pragma once
#include "core/math_types.hpp"
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <string_view>

namespace garuda {

using dronesim::Vec3d;

/**
 * @brief Canonical payload types supported by GARUDA-HL-01 modular payload bay.
 */
enum class PayloadType : int {
    NONE = 0,
    INSPECTION_CAMERA = 1,
    LIDAR_MODULE = 2,
    THERMAL_MODULE = 3,
    MAPPING_MODULE = 4,
    EMERGENCY_SUPPLY = 5,
    MAINTENANCE_MODULE = 6,
    RESCUE_MODULE = 7
};

/**
 * @brief Authoritative payload lifecycle state machine.
 */
enum class PayloadState : int {
    AVAILABLE = 0,   // Unattached, in catalogue/depot
    ATTACHING = 1,   // Mechanical/electrical latching in progress
    ATTACHED = 2,    // Physically attached and locked to payload bay
    ACTIVE = 3,      // Powered on, transmitting telemetry/data
    DETACHING = 4,   // Unlocking latching mechanism
    DETACHED = 5,    // Fully released from airframe
    FAULT = 6        // Hardware or communication fault
};

/**
 * @brief Complete metadata descriptor for a modular payload unit.
 */
struct PayloadDescriptor {
    std::string  id{"PAYLOAD-NONE"};
    std::string  name{"No Payload"};
    std::string  category{"NONE"};
    PayloadType  type{PayloadType::NONE};
    double       mass_kg{0.0};
    Vec3d        dimensions_m{0.0, 0.0, 0.0};
    Vec3d        com_offset_m{0.0, -0.12, 0.0}; // Offset relative to vehicle nominal CoM (m)
    Vec3d        inertia_diag_kgm2{0.0, 0.0, 0.0}; // Intrinsic moment of inertia about payload's own CoM (kg*m^2)
    double       power_w{0.0};
    bool         enabled{false};
    int          health{0};                      // 0=NOMINAL, 1=DEGRADED, 2=FAULT
    PayloadState state{PayloadState::AVAILABLE};
    std::string  capabilities{"None"};
};

/**
 * @brief Centralized payload catalogue providing verified hardware definitions.
 */
class PayloadCatalogue {
public:
    [[nodiscard]] static PayloadDescriptor get_descriptor(PayloadType type) noexcept {
        PayloadDescriptor desc{};
        desc.type = type;

        switch (type) {
            case PayloadType::INSPECTION_CAMERA:
                desc.id = "PAYLOAD-CAM-4K";
                desc.name = "GARUDA 4K Optical Inspection Gimbal";
                desc.category = "OPTICAL";
                desc.mass_kg = 1.50;
                desc.dimensions_m = {0.18, 0.22, 0.18};
                desc.com_offset_m = {0.0, -0.12, 0.0};
                desc.inertia_diag_kgm2 = {0.0035, 0.0035, 0.0040};
                desc.power_w = 18.0;
                desc.enabled = true;
                desc.health = 0;
                desc.state = PayloadState::ATTACHED;
                desc.capabilities = "4K RGB 60fps, 30x Optical Zoom, 2-Axis Stabilized Gimbal";
                break;

            case PayloadType::LIDAR_MODULE:
                desc.id = "PAYLOAD-LIDAR-360";
                desc.name = "Multi-Beam 3D Infrastructure LiDAR";
                desc.category = "LIDAR";
                desc.mass_kg = 2.20;
                desc.dimensions_m = {0.16, 0.20, 0.16};
                desc.com_offset_m = {0.0, -0.10, 0.0};
                desc.inertia_diag_kgm2 = {0.0055, 0.0060, 0.0055};
                desc.power_w = 32.0;
                desc.enabled = true;
                desc.health = 0;
                desc.state = PayloadState::ATTACHED;
                desc.capabilities = "360-deg 16-Beam LiDAR, 200m Range, 300k pts/s";
                break;

            case PayloadType::THERMAL_MODULE:
                desc.id = "PAYLOAD-IR-DUAL";
                desc.name = "Radiometric Dual Optical-Thermal Pod";
                desc.category = "THERMAL";
                desc.mass_kg = 1.10;
                desc.dimensions_m = {0.14, 0.18, 0.14};
                desc.com_offset_m = {0.0, -0.12, 0.0};
                desc.inertia_diag_kgm2 = {0.0022, 0.0022, 0.0028};
                desc.power_w = 14.0;
                desc.enabled = true;
                desc.health = 0;
                desc.state = PayloadState::ATTACHED;
                desc.capabilities = "640x512 Radiometric Thermal IR + 4K Visible";
                break;

            case PayloadType::MAPPING_MODULE:
                desc.id = "PAYLOAD-MAP-RGB";
                desc.name = "High-Resolution Photogrammetry Mapping Module";
                desc.category = "SURVEY";
                desc.mass_kg = 1.60;
                desc.dimensions_m = {0.16, 0.16, 0.15};
                desc.com_offset_m = {0.0, -0.11, 0.0};
                desc.inertia_diag_kgm2 = {0.0038, 0.0038, 0.0042};
                desc.power_w = 20.0;
                desc.enabled = true;
                desc.health = 0;
                desc.state = PayloadState::ATTACHED;
                desc.capabilities = "61MP Full-Frame RGB, Global Mechanical Shutter, RTK Sync";
                break;

            case PayloadType::EMERGENCY_SUPPLY:
                desc.id = "PAYLOAD-CARGO-BAY";
                desc.name = "Quick-Release Heavy Cargo Pod";
                desc.category = "CARGO";
                desc.mass_kg = 3.50;
                desc.dimensions_m = {0.35, 0.25, 0.25};
                desc.com_offset_m = {0.0, -0.14, 0.0};
                desc.inertia_diag_kgm2 = {0.0180, 0.0150, 0.0220};
                desc.power_w = 0.0;
                desc.enabled = true;
                desc.health = 0;
                desc.state = PayloadState::ATTACHED;
                desc.capabilities = "Autonomous Servo Drop Mechanism, 5kg Rating";
                break;

            case PayloadType::MAINTENANCE_MODULE:
                desc.id = "PAYLOAD-NDT-ACTUATOR";
                desc.name = "Ultrasonic NDT Surface Probe & Actuator";
                desc.category = "MAINTENANCE";
                desc.mass_kg = 2.80;
                desc.dimensions_m = {0.25, 0.30, 0.20};
                desc.com_offset_m = {0.02, -0.15, 0.0};
                desc.inertia_diag_kgm2 = {0.0120, 0.0110, 0.0140};
                desc.power_w = 25.0;
                desc.enabled = true;
                desc.health = 0;
                desc.state = PayloadState::ATTACHED;
                desc.capabilities = "Contact Thickness Gauge, Surface Crack Probe, Actuator Arm";
                break;

            case PayloadType::RESCUE_MODULE:
                desc.id = "PAYLOAD-RESCUE-POD";
                desc.name = "Search & Rescue High-Intensity Illumination & Winch";
                desc.category = "SEARCH_RESCUE";
                desc.mass_kg = 4.20;
                desc.dimensions_m = {0.28, 0.26, 0.22};
                desc.com_offset_m = {0.0, -0.16, 0.0};
                desc.inertia_diag_kgm2 = {0.0240, 0.0210, 0.0280};
                desc.power_w = 45.0;
                desc.enabled = true;
                desc.health = 0;
                desc.state = PayloadState::ATTACHED;
                desc.capabilities = "12,000 Lumen Searchlight, 15m Automated Tether Winch";
                break;

            case PayloadType::NONE:
            default:
                desc.id = "PAYLOAD-NONE";
                desc.name = "No Payload";
                desc.category = "NONE";
                desc.mass_kg = 0.0;
                desc.power_w = 0.0;
                desc.enabled = false;
                desc.state = PayloadState::DETACHED;
                desc.capabilities = "None";
                break;
        }

        return desc;
    }

    [[nodiscard]] static std::vector<PayloadDescriptor> all_available_payloads() noexcept {
        return {
            get_descriptor(PayloadType::INSPECTION_CAMERA),
            get_descriptor(PayloadType::LIDAR_MODULE),
            get_descriptor(PayloadType::THERMAL_MODULE),
            get_descriptor(PayloadType::MAPPING_MODULE),
            get_descriptor(PayloadType::EMERGENCY_SUPPLY),
            get_descriptor(PayloadType::MAINTENANCE_MODULE),
            get_descriptor(PayloadType::RESCUE_MODULE)
        };
    }

    [[nodiscard]] static std::string_view state_to_string(PayloadState s) noexcept {
        switch (s) {
            case PayloadState::AVAILABLE: return "AVAILABLE";
            case PayloadState::ATTACHING: return "ATTACHING";
            case PayloadState::ATTACHED:  return "ATTACHED";
            case PayloadState::ACTIVE:    return "ACTIVE";
            case PayloadState::DETACHING: return "DETACHING";
            case PayloadState::DETACHED:  return "DETACHED";
            case PayloadState::FAULT:     return "FAULT";
            default:                      return "UNKNOWN";
        }
    }
};

} // namespace garuda
