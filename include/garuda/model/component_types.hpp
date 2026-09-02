#pragma once
#include <string>
#include <cstdint>

namespace garuda::model {

/**
 * @brief Categorization of physical UAV mechanical subsystems.
 */
enum class ComponentCategory : uint8_t {
    AIRFRAME = 0,
    ARM,
    MOTOR,
    ROTOR,
    LANDING_GEAR,
    PAYLOAD_MOUNT,
    GIMBAL_JOINT,
    CAMERA_PAYLOAD,
    AVIONICS,
    LIGHT
};

[[nodiscard]] constexpr const char* to_string(ComponentCategory cat) noexcept {
    switch (cat) {
        case ComponentCategory::AIRFRAME:       return "AIRFRAME";
        case ComponentCategory::ARM:            return "ARM";
        case ComponentCategory::MOTOR:          return "MOTOR";
        case ComponentCategory::ROTOR:          return "ROTOR";
        case ComponentCategory::LANDING_GEAR:   return "LANDING_GEAR";
        case ComponentCategory::PAYLOAD_MOUNT:  return "PAYLOAD_MOUNT";
        case ComponentCategory::GIMBAL_JOINT:   return "GIMBAL_JOINT";
        case ComponentCategory::CAMERA_PAYLOAD: return "CAMERA_PAYLOAD";
        case ComponentCategory::AVIONICS:       return "AVIONICS";
        case ComponentCategory::LIGHT:          return "LIGHT";
        default:                                return "UNKNOWN";
    }
}

/**
 * @brief Physical rotor rotation direction.
 */
enum class RotorDirection : int8_t {
    CW  = -1, // Clockwise when viewed from above (+Y shaft axis)
    CCW =  1  // Counter-Clockwise when viewed from above (+Y shaft axis)
};

[[nodiscard]] constexpr const char* to_string(RotorDirection dir) noexcept {
    return (dir == RotorDirection::CW) ? "CW" : "CCW";
}

/**
 * @brief Reusable PBR material classification for industrial UAV rendering.
 */
enum class MaterialCategory : uint8_t {
    STEALTH_CARBON = 0,
    CARBON_TUBE,
    CNC_RED_ALUMINUM,
    CYAN_LIGHTGUIDE,
    RED_STROBE,
    OPTICAL_GLASS,
    TACTICAL_WHITE,
    PROPELLER_CARBON,
    NAV_GREEN,
    NAV_RED,
    DARK_METAL
};

[[nodiscard]] constexpr const char* to_string(MaterialCategory mat) noexcept {
    switch (mat) {
        case MaterialCategory::STEALTH_CARBON:   return "MAT_STEALTH_CARBON";
        case MaterialCategory::CARBON_TUBE:      return "MAT_CARBON_TUBE";
        case MaterialCategory::CNC_RED_ALUMINUM: return "MAT_CNC_RED_ALUMINUM";
        case MaterialCategory::CYAN_LIGHTGUIDE:  return "MAT_CYAN_STATUS";
        case MaterialCategory::RED_STROBE:       return "MAT_RED_STATUS";
        case MaterialCategory::OPTICAL_GLASS:    return "MAT_OPTICAL_GLASS";
        case MaterialCategory::TACTICAL_WHITE:   return "MAT_TACTICAL_WHITE";
        case MaterialCategory::PROPELLER_CARBON: return "MAT_PROPELLER_CARBON";
        case MaterialCategory::NAV_GREEN:        return "MAT_NAV_GREEN";
        case MaterialCategory::NAV_RED:          return "MAT_NAV_RED";
        case MaterialCategory::DARK_METAL:       return "MAT_DARK_METAL";
        default:                                 return "MAT_UNKNOWN";
    }
}

/**
 * @brief Binding connecting an authoritative C++ mechanical component to a Godot GLTF visual node.
 */
struct VisualNodeBinding {
    std::string gltf_node_name{};
    std::string gltf_mesh_name{};
    std::string glb_relative_path{};
    MaterialCategory default_material{MaterialCategory::STEALTH_CARBON};
};

} // namespace garuda::model
