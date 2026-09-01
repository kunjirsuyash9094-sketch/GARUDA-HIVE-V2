#pragma once
#include "core/math_types.hpp"
#include "garuda/config/quadrotor_config.hpp"

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

/**
 * @brief Contact result holding normal, friction, and restoring wrenches.
 */
struct ContactResult {
    bool   in_contact{false};
    double penetration_depth_m{0.0};
    Vec3d  force_world{0.0, 0.0, 0.0};
    Vec3d  torque_world{0.0, 0.0, 0.0};
};

class GroundContactModel {
public:
    explicit GroundContactModel(const QuadrotorConfig& config) noexcept
        : _cfg(config) {}

    void set_config(const QuadrotorConfig& config) noexcept {
        _cfg = config;
    }

    /**
     * @brief Computes deterministic ground reaction forces and torques.
     * 
     * @param pos_world Position of the vehicle CoM in world frame
     * @param vel_world Velocity of the vehicle CoM in world frame
     * @param omega_bf  Angular velocity in body frame
     * @param orient    Orientation quaternion
     * @param ground_y  Elevation of the terrain/ground plane
     * @param dt        Physics integration timestep
     */
    [[nodiscard]] ContactResult evaluate(
        const Vec3d& pos_world,
        const Vec3d& vel_world,
        const Vec3d& omega_bf,
        const Quat&  orient,
        double ground_y,
        double dt
    ) const noexcept;

    /**
     * @brief Enforces hard kinematic non-penetration constraint for resting contact.
     */
    void resolve_penetration(
        Vec3d& pos_world,
        Vec3d& vel_world,
        Vec3d& omega_bf,
        double ground_y
    ) const noexcept;

private:
    QuadrotorConfig _cfg;
};

} // namespace garuda
