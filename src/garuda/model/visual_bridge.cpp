#include "garuda/model/visual_bridge.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>

namespace garuda::model {

VisualBridge::VisualBridge(const GarudaVehicleSpecification& spec) noexcept
    : _spec(spec) {
    reset();
}

void VisualBridge::reset() noexcept {
    _visual_state.world_position = {0.0, 0.0, 0.0};
    _visual_state.world_orientation = {0.0, 0.0, 0.0, 1.0};
    _visual_state.rotor_angles_rad.fill(0.0);
    _visual_state.rotor_angular_velocities_rad_s.fill(0.0);

    for (size_t i = 0; i < 8; ++i) {
        _visual_state.rotor_directions[i] = static_cast<int8_t>(_spec.rotors[i].direction);
    }

    _visual_state.gimbal_yaw_rad = 0.0;
    _visual_state.gimbal_pitch_rad = 0.0;
    _visual_state.gimbal_roll_rad = 0.0;
    _visual_state.strobe_active = false;
    _visual_state.light_intensity = 1.0;
    _strobe_timer = 0.0;
}

void VisualBridge::update(
    const Vec3d& pos_world,
    const Quat&  orient_world,
    const double motor_rpms[8],
    double       gimbal_yaw_deg,
    double       gimbal_pitch_deg,
    double       gimbal_roll_deg,
    bool         is_armed,
    double       dt
) noexcept {
    // 1. Vehicle Pose
    _visual_state.world_position = pos_world;
    _visual_state.world_orientation = orient_world;

    // 2. 8-Rotor Angular Integrations
    constexpr double rpm_to_rad_s = (2.0 * std::numbers::pi) / 60.0;

    for (size_t i = 0; i < 8; ++i) {
        double rpm = (motor_rpms != nullptr) ? std::max(0.0, motor_rpms[i]) : 0.0;
        int8_t dir = _visual_state.rotor_directions[i];

        // Angular velocity in rad/s (signed with respect to +Y shaft axis)
        double rad_s = rpm * rpm_to_rad_s * static_cast<double>(dir);
        _visual_state.rotor_angular_velocities_rad_s[i] = rad_s;

        if (dt > 0.0 && rpm > 0.0) {
            _visual_state.rotor_angles_rad[i] += rad_s * dt;
            // Wrap angle to [-PI, PI] to prevent floating point precision degradation
            if (_visual_state.rotor_angles_rad[i] > std::numbers::pi) {
                _visual_state.rotor_angles_rad[i] -= 2.0 * std::numbers::pi;
            } else if (_visual_state.rotor_angles_rad[i] < -std::numbers::pi) {
                _visual_state.rotor_angles_rad[i] += 2.0 * std::numbers::pi;
            }
        }
    }

    // 3. Gimbal Articulation (Clamped within C++ specification joint limits)
    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    if (_spec.gimbal_joints.size() >= 3) {
        double clamped_yaw = std::clamp(gimbal_yaw_deg, _spec.gimbal_joints[0].min_angle_deg, _spec.gimbal_joints[0].max_angle_deg);
        double clamped_pitch = std::clamp(gimbal_pitch_deg, _spec.gimbal_joints[1].min_angle_deg, _spec.gimbal_joints[1].max_angle_deg);
        double clamped_roll = std::clamp(gimbal_roll_deg, _spec.gimbal_joints[2].min_angle_deg, _spec.gimbal_joints[2].max_angle_deg);

        _visual_state.gimbal_yaw_rad = clamped_yaw * deg_to_rad;
        _visual_state.gimbal_pitch_rad = clamped_pitch * deg_to_rad;
        _visual_state.gimbal_roll_rad = clamped_roll * deg_to_rad;
    }

    // 4. Strobe & Status Lighting (1.0 Hz flash rate)
    if (dt > 0.0) {
        _strobe_timer += dt;
        if (_strobe_timer >= 1.0) {
            _strobe_timer -= 1.0;
        }
        _visual_state.strobe_active = (_strobe_timer < 0.10); // 100ms flash pulse
        _visual_state.light_intensity = is_armed ? 1.0 : 0.40;
    }
}

} // namespace garuda::model
