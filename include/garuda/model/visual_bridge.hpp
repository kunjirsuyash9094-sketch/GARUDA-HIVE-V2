#pragma once
#include "garuda/model/vehicle_specification.hpp"
#include <array>
#include <vector>

namespace garuda::model {

/**
 * @brief Visual state evaluated by the C++ VisualBridge for Godot 4 rendering.
 */
struct VehicleVisualState {
    // 1. Vehicle Pose
    Vec3d world_position{0.0, 0.0, 0.0};
    Quat  world_orientation{0.0, 0.0, 0.0, 1.0};

    // 2. 8-Rotor Visual Joint Angles & Velocities
    std::array<double, 8> rotor_angles_rad{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 8> rotor_angular_velocities_rad_s{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<int8_t, 8> rotor_directions{-1, 1, -1, 1, -1, 1, -1, 1}; // Visual spin direction around +Y

    // 3. 3-Axis Gimbal Articulation (Radians)
    double gimbal_yaw_rad{0.0};
    double gimbal_pitch_rad{0.0};
    double gimbal_roll_rad{0.0};

    // 4. Status Lighting
    bool   strobe_active{false};
    double light_intensity{1.0};
};

/**
 * @brief Authoritative C++20 Visualization Bridge.
 * 
 * Maps C++ simulation telemetry deterministically into Godot node transforms.
 * Does NOT compute flight physics or aerodynamics.
 */
class VisualBridge {
public:
    explicit VisualBridge(const GarudaVehicleSpecification& spec = GarudaVehicleSpecification::create_canonical()) noexcept;

    void reset() noexcept;

    /**
     * @brief Updates visual state from raw telemetry inputs over time dt.
     */
    void update(
        const Vec3d& pos_world,
        const Quat&  orient_world,
        const double motor_rpms[8],
        double       gimbal_yaw_deg,
        double       gimbal_pitch_deg,
        double       gimbal_roll_deg,
        bool         is_armed,
        double       dt
    ) noexcept;

    [[nodiscard]] const VehicleVisualState& state() const noexcept { return _visual_state; }

private:
    GarudaVehicleSpecification _spec;
    VehicleVisualState         _visual_state{};
    double                     _strobe_timer{0.0};
};

} // namespace garuda::model
