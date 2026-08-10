#pragma once
#include "core/math_types.hpp"

// ---------------------------------------------------------------------------
// Frame conventions used throughout the simulator
//
//   Godot world:  X = east, Y = up, -Z = north   (right-handed, Y-up)
//   Godot body:   X = right, Y = up, -Z = forward
//   NED world:    X = north, Y = east, Z = down  (aerospace standard)
//   FRD body:     X = forward, Y = right, Z = down
//
// The permutation P mapping Godot axes to NED/FRD axes is identical for the
// world and body frames:
//     ned.x = -godot.z,  ned.y = godot.x,  ned.z = -godot.y
// P is a proper rotation (det = +1), so quaternions transform by permuting
// the vector part with P and leaving w unchanged.
// ---------------------------------------------------------------------------
namespace dronesim::frames {

// Godot world vector -> NED  (also Godot body vector -> FRD)
[[nodiscard]] inline Vec3d godot_to_ned(const Vec3d& v) noexcept {
    return { -v.z, v.x, -v.y };
}

// NED vector -> Godot world  (also FRD body vector -> Godot body)
[[nodiscard]] inline Vec3d ned_to_godot(const Vec3d& v) noexcept {
    return { v.y, -v.z, -v.x };
}

// Godot body-to-world quaternion -> NED body(FRD)-to-world(NED) quaternion
[[nodiscard]] inline Quat godot_to_ned(const Quat& q) noexcept {
    return { -q.z, q.x, -q.y, q.w };
}

[[nodiscard]] inline Quat ned_to_godot(const Quat& q) noexcept {
    return { q.y, -q.z, -q.x, q.w };
}

} // namespace dronesim::frames
