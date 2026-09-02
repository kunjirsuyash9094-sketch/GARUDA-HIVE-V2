#pragma once
#include "core/math_types.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace garuda::model {

using dronesim::Vec3d;
using dronesim::Quat;

/**
 * @brief Precision 3D Affine Transform for UAV component kinematics and hierarchy.
 * 
 * Coordinate System (Authoritative GARUDA C++):
 * - +X : Starboard / Right
 * - +Y : Up / Vertical normal
 * - -Z : Forward / Nose heading
 */
struct Transform3D {
    Vec3d translation{0.0, 0.0, 0.0};
    Quat  rotation{0.0, 0.0, 0.0, 1.0}; // [x, y, z, w]
    Vec3d scale{1.0, 1.0, 1.0};

    [[nodiscard]] static constexpr Transform3D identity() noexcept {
        return Transform3D{};
    }

    [[nodiscard]] static Transform3D from_translation(const Vec3d& t) noexcept {
        Transform3D tf{};
        tf.translation = t;
        return tf;
    }

    [[nodiscard]] static Transform3D from_euler_deg(double yaw_deg, double pitch_deg, double roll_deg) noexcept {
        Transform3D tf{};
        // Convert Euler degrees to Quat (ZYX convention)
        double y_rad = yaw_deg * (dronesim::PI / 180.0) * 0.5;
        double p_rad = pitch_deg * (dronesim::PI / 180.0) * 0.5;
        double r_rad = roll_deg * (dronesim::PI / 180.0) * 0.5;

        double cy = std::cos(y_rad), sy = std::sin(y_rad);
        double cp = std::cos(p_rad), sp = std::sin(p_rad);
        double cr = std::cos(r_rad), sr = std::sin(r_rad);

        tf.rotation.w = cr * cp * cy + sr * sp * sy;
        tf.rotation.x = sr * cp * cy - cr * sp * sy;
        tf.rotation.y = cr * sp * cy + sr * cp * sy;
        tf.rotation.z = cr * cp * sy - sr * sp * cy;
        return tf;
    }

    [[nodiscard]] Vec3d transform_point(const Vec3d& p) const noexcept {
        Vec3d scaled = {p.x * scale.x, p.y * scale.y, p.z * scale.z};
        return translation + rotation.rotate(scaled);
    }

    [[nodiscard]] Vec3d transform_vector(const Vec3d& v) const noexcept {
        return rotation.rotate(v);
    }

    [[nodiscard]] Transform3D operator*(const Transform3D& child) const noexcept {
        Transform3D res{};
        // Scale composition
        res.scale = {scale.x * child.scale.x, scale.y * child.scale.y, scale.z * child.scale.z};
        // Rotation composition (Hamilton quaternion multiplication: q1 * q2)
        const auto& q1 = rotation;
        const auto& q2 = child.rotation;
        res.rotation.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        res.rotation.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        res.rotation.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        res.rotation.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
        // Translation composition: t_parent + R_parent * (scale_parent * t_child)
        Vec3d scaled_child_t = {child.translation.x * scale.x, child.translation.y * scale.y, child.translation.z * scale.z};
        res.translation = translation + rotation.rotate(scaled_child_t);
        return res;
    }

    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(translation.x) && std::isfinite(translation.y) && std::isfinite(translation.z) &&
               std::isfinite(rotation.x) && std::isfinite(rotation.y) && std::isfinite(rotation.z) && std::isfinite(rotation.w) &&
               std::isfinite(scale.x) && std::isfinite(scale.y) && std::isfinite(scale.z);
    }

    [[nodiscard]] bool has_positive_scale() const noexcept {
        return (scale.x > 1e-6) && (scale.y > 1e-6) && (scale.z > 1e-6);
    }

    [[nodiscard]] bool is_normalized_rotation(double eps = 1e-4) const noexcept {
        double len2 = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z + rotation.w * rotation.w;
        return std::abs(len2 - 1.0) < eps;
    }
};

} // namespace garuda::model
