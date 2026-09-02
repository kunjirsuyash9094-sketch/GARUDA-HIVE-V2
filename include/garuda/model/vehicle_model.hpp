#pragma once
#include "garuda/model/vehicle_specification.hpp"
#include <unordered_map>
#include <optional>
#include <string_view>

namespace garuda::model {

/**
 * @brief Authoritative C++20 Runtime Model Manager for GARUDA-HL-01.
 * 
 * Provides queried access to components, evaluated world-space transforms,
 * and direct kinematic state evaluation.
 */
class GarudaVehicleModel {
public:
    explicit GarudaVehicleModel(GarudaVehicleSpecification spec = GarudaVehicleSpecification::create_canonical()) noexcept;

    [[nodiscard]] const GarudaVehicleSpecification& specification() const noexcept { return _spec; }
    
    [[nodiscard]] const GarudaAirframe& airframe() const noexcept { return _spec.airframe; }
    [[nodiscard]] const GarudaArm& arm(size_t index) const;
    [[nodiscard]] const GarudaMotor& motor(size_t index) const;
    [[nodiscard]] const GarudaRotor& rotor(size_t index) const;
    [[nodiscard]] const GarudaLandingGear& landing_gear() const noexcept { return _spec.landing_gear; }
    [[nodiscard]] const GarudaPayloadMount& payload_mount() const noexcept { return _spec.payload_mount; }
    [[nodiscard]] const std::vector<GarudaGimbalJoint>& gimbal_joints() const noexcept { return _spec.gimbal_joints; }
    [[nodiscard]] const GarudaAvionics& avionics() const noexcept { return _spec.avionics; }

    [[nodiscard]] std::optional<Transform3D> get_world_transform(std::string_view component_id) const noexcept;
    [[nodiscard]] std::vector<std::string> get_all_component_ids() const noexcept;
    [[nodiscard]] size_t total_component_count() const noexcept;

private:
    GarudaVehicleSpecification _spec;
    std::unordered_map<std::string, Transform3D> _world_transforms;
    void _recompute_world_transforms() noexcept;
};

} // namespace garuda::model
