#include "garuda/model/vehicle_model.hpp"
#include <stdexcept>

namespace garuda::model {

GarudaVehicleModel::GarudaVehicleModel(GarudaVehicleSpecification spec) noexcept
    : _spec(std::move(spec)) {
    _recompute_world_transforms();
}

const GarudaArm& GarudaVehicleModel::arm(size_t index) const {
    if (index < 1 || index > 8) {
        throw std::out_of_range("GarudaVehicleModel::arm index out of range (1..8)");
    }
    return _spec.arms[index - 1];
}

const GarudaMotor& GarudaVehicleModel::motor(size_t index) const {
    if (index < 1 || index > 8) {
        throw std::out_of_range("GarudaVehicleModel::motor index out of range (1..8)");
    }
    return _spec.motors[index - 1];
}

const GarudaRotor& GarudaVehicleModel::rotor(size_t index) const {
    if (index < 1 || index > 8) {
        throw std::out_of_range("GarudaVehicleModel::rotor index out of range (1..8)");
    }
    return _spec.rotors[index - 1];
}

std::optional<Transform3D> GarudaVehicleModel::get_world_transform(std::string_view component_id) const noexcept {
    auto it = _world_transforms.find(std::string(component_id));
    if (it != _world_transforms.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<std::string> GarudaVehicleModel::get_all_component_ids() const noexcept {
    std::vector<std::string> ids;
    ids.reserve(_world_transforms.size());
    for (const auto& [id, _] : _world_transforms) {
        ids.push_back(id);
    }
    return ids;
}

size_t GarudaVehicleModel::total_component_count() const noexcept {
    return _world_transforms.size();
}

void GarudaVehicleModel::_recompute_world_transforms() noexcept {
    _world_transforms.clear();

    // 1. Root & Airframe
    Transform3D root_tf = Transform3D::identity();
    Transform3D airframe_tf = root_tf * _spec.airframe.local_transform;
    _world_transforms[_spec.airframe.id] = airframe_tf;

    // 2. Arms, Motors, and Rotors
    for (size_t i = 0; i < 8; ++i) {
        Transform3D arm_world = airframe_tf * _spec.arms[i].local_transform;
        _world_transforms[_spec.arms[i].id] = arm_world;

        // Motors are positioned at their absolute local offset relative to airframe
        Transform3D motor_world = airframe_tf * _spec.motors[i].local_transform;
        _world_transforms[_spec.motors[i].id] = motor_world;

        // Rotor is child of motor
        Transform3D rotor_world = motor_world * _spec.rotors[i].local_transform;
        _world_transforms[_spec.rotors[i].id] = rotor_world;
    }

    // 3. Landing Gear
    _world_transforms[_spec.landing_gear.id] = airframe_tf * _spec.landing_gear.local_transform;

    // 4. Payload Mount
    Transform3D payload_world = airframe_tf * _spec.payload_mount.local_transform;
    _world_transforms[_spec.payload_mount.id] = payload_world;

    // 5. Gimbal Chain: PAYLOAD_MOUNT -> YAW -> PITCH -> ROLL
    Transform3D curr_gimbal_tf = payload_world;
    for (const auto& joint : _spec.gimbal_joints) {
        curr_gimbal_tf = curr_gimbal_tf * joint.local_transform;
        _world_transforms[joint.id] = curr_gimbal_tf;
    }

    // 6. Avionics
    _world_transforms[_spec.avionics.id] = airframe_tf * _spec.avionics.local_transform;
}

} // namespace garuda::model
