#include "garuda/physics/ground_contact.hpp"
#include <algorithm>
#include <cmath>

namespace garuda {

ContactResult GroundContactModel::evaluate(
    const Vec3d& pos_world,
    const Vec3d& vel_world,
    const Vec3d& omega_bf,
    const Quat&  orient,
    double ground_y,
    double /*dt*/
) const noexcept {
    ContactResult res{};
    const double contact_surface_y = ground_y + _cfg.ground_contact_radius_m;
    const double penetration = contact_surface_y - pos_world.y;

    if (penetration < -0.005) {
        return res; // No contact
    }

    res.in_contact = true;
    res.penetration_depth_m = std::max(0.0, penetration);

    // Normal force using non-linear spring-damper
    // F_N = max(0, k * delta - d * v_y)
    double f_spring = _cfg.ground_spring_k * penetration;
    double f_damper = -_cfg.ground_damper_d * vel_world.y;
    double f_normal = std::max(0.0, f_spring + f_damper);

    // Tangential friction
    Vec3d v_tangent{vel_world.x, 0.0, vel_world.z};
    double v_tan_mag = v_tangent.norm();
    Vec3d f_friction{0.0, 0.0, 0.0};

    if (v_tan_mag > 1e-5 && f_normal > 0.0) {
        double max_friction = _cfg.ground_friction_coeff * f_normal;
        double dynamic_friction = std::min(max_friction, _cfg.ground_damper_d * 0.5 * v_tan_mag);
        f_friction = -v_tangent * (dynamic_friction / v_tan_mag);
    }

    res.force_world = Vec3d{f_friction.x, f_normal, f_friction.z};

    // Rotational restitution & contact damping
    Vec3d omega_world = orient.rotate(omega_bf);
    double rot_damper = 0.05 * f_normal;
    res.torque_world = -omega_world * rot_damper;

    return res;
}

void GroundContactModel::resolve_penetration(
    Vec3d& pos_world,
    Vec3d& vel_world,
    Vec3d& omega_bf,
    double ground_y
) const noexcept {
    const double contact_surface_y = ground_y + _cfg.ground_contact_radius_m;
    if (pos_world.y < contact_surface_y) {
        pos_world.y = contact_surface_y;
        if (vel_world.y < 0.0) {
            // Apply restitution for small rebound or absorb
            if (std::abs(vel_world.y) > 0.5) {
                vel_world.y = -vel_world.y * _cfg.ground_restitution;
            } else {
                vel_world.y = 0.0; // Resting contact
            }
        }
        // Lateral friction deceleration on ground clamp
        vel_world.x *= 0.90;
        vel_world.z *= 0.90;
        omega_bf *= 0.85;
    }
}

} // namespace garuda
