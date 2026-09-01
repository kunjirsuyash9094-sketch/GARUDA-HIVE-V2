#pragma once
#include "core/math_types.hpp"
#include "garuda/config/quadrotor_config.hpp"

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

/**
 * @brief Authoritative 6-DOF Rigid-Body State.
 */
struct RigidBody6DOFState {
    Vec3d position{0.0, 0.0, 0.0};         // Position in world frame (m)
    Vec3d velocity{0.0, 0.0, 0.0};         // Linear velocity in world frame (m/s)
    Vec3d acceleration{0.0, 0.0, 0.0};     // Linear acceleration in world frame (m/s^2)
    Quat  orientation{0.0, 0.0, 0.0, 1.0}; // Attitude quaternion (Hamilton, body-to-world)
    Vec3d angular_velocity{0.0, 0.0, 0.0}; // Angular velocity in body frame (rad/s)
    Vec3d angular_acceleration{0.0, 0.0, 0.0}; // Angular acceleration in body frame (rad/s^2)
};

/**
 * @brief Authoritative 6-DOF Rigid-Body Integrator.
 * 
 * Semi-implicit Euler translation + Euler's rotational equations with
 * quaternion kinematics and strict normalization.
 * Supports dynamic mass and inertia coupling for modular payloads.
 */
class RigidBodyIntegrator {
public:
    explicit RigidBodyIntegrator(const QuadrotorConfig& config) noexcept
        : _cfg(config)
        , _effective_mass(config.dry_mass_kg)
        , _effective_inertia(config.inertia_diag_kgm2)
        , _com_offset{0.0, 0.0, 0.0} {}

    void reset(const Vec3d& spawn_position = {0.0, 0.0, 0.0},
               const Quat&  spawn_orientation = Quat::identity()) noexcept {
        _state.position = spawn_position;
        _state.velocity = {0.0, 0.0, 0.0};
        _state.acceleration = {0.0, 0.0, 0.0};
        _state.orientation = spawn_orientation.normalized();
        _state.angular_velocity = {0.0, 0.0, 0.0};
        _state.angular_acceleration = {0.0, 0.0, 0.0};
        _effective_mass = _cfg.dry_mass_kg;
        _effective_inertia = _cfg.inertia_diag_kgm2;
        _com_offset = {0.0, 0.0, 0.0};
    }

    void set_mass_properties(double mass_kg, const Vec3d& inertia_diag, const Vec3d& com_offset = {0.0, 0.0, 0.0}) noexcept {
        _effective_mass = std::max(0.1, mass_kg);
        _effective_inertia = Vec3d{ std::max(1e-4, inertia_diag.x), std::max(1e-4, inertia_diag.y), std::max(1e-4, inertia_diag.z) };
        _com_offset = com_offset;
    }

    /**
     * @brief Integrates rigid body motion over dt seconds.
     * 
     * @param total_force_world Total net external force in world frame (N)
     * @param total_torque_body Total net external torque in body frame (N*m)
     * @param dt Fixed physics integration timestep (s)
     */
    void step(const Vec3d& total_force_world, const Vec3d& total_torque_body, double dt) noexcept;

    [[nodiscard]] const RigidBody6DOFState& state() const noexcept { return _state; }
    [[nodiscard]] RigidBody6DOFState& mutable_state() noexcept { return _state; }
    [[nodiscard]] double effective_mass() const noexcept { return _effective_mass; }
    [[nodiscard]] const Vec3d& effective_inertia() const noexcept { return _effective_inertia; }
    [[nodiscard]] const Vec3d& com_offset() const noexcept { return _com_offset; }

private:
    QuadrotorConfig    _cfg;
    RigidBody6DOFState _state{};
    double             _effective_mass{8.50};
    Vec3d              _effective_inertia{0.185, 0.185, 0.320};
    Vec3d              _com_offset{0.0, 0.0, 0.0};
};

} // namespace garuda
