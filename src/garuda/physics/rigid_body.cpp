#include "garuda/physics/rigid_body.hpp"
#include <cmath>

namespace garuda {

void RigidBodyIntegrator::step(const Vec3d& total_force_world, const Vec3d& total_torque_body, double dt) noexcept {
    if (dt <= 0.0) return;

    const double mass = _effective_mass;
    const Vec3d& I = _effective_inertia;

    // 1. Translational Integration (World Frame, Semi-Implicit Euler)
    _state.acceleration = total_force_world / mass;
    _state.velocity += _state.acceleration * dt;
    _state.position += _state.velocity * dt;

    // 2. Rotational Dynamics (Body Frame Euler Equations)
    // I * w_dot + w x (I * w) = tau
    Vec3d w = _state.angular_velocity;
    Vec3d Iw{ I.x * w.x, I.y * w.y, I.z * w.z };
    Vec3d gyro_torque = w.cross(Iw);

    _state.angular_acceleration = Vec3d{
        (total_torque_body.x - gyro_torque.x) / I.x,
        (total_torque_body.y - gyro_torque.y) / I.y,
        (total_torque_body.z - gyro_torque.z) / I.z
    };

    _state.angular_velocity += _state.angular_acceleration * dt;

    // 3. Quaternion Kinematics Integration (Body Rates -> Quaternion Derivative)
    // q_dot = 0.5 * q * [w, 0]
    Quat wq{ _state.angular_velocity.x, _state.angular_velocity.y, _state.angular_velocity.z, 0.0 };
    Quat qd = _state.orientation * wq;

    _state.orientation.x += 0.5 * qd.x * dt;
    _state.orientation.y += 0.5 * qd.y * dt;
    _state.orientation.z += 0.5 * qd.z * dt;
    _state.orientation.w += 0.5 * qd.w * dt;

    // Strict normalization to eliminate floating-point drift
    _state.orientation = _state.orientation.normalized();
}

} // namespace garuda
