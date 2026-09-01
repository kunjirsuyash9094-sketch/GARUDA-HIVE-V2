#pragma once
#include "garuda/config/quadrotor_config.hpp"
#include "garuda/physics/rigid_body.hpp"
#include "garuda/physics/motor_system.hpp"
#include "garuda/physics/propulsion_system.hpp"
#include "garuda/physics/battery_model.hpp"
#include "garuda/physics/ground_contact.hpp"
#include "garuda/physics/environment.hpp"
#include "garuda/sensors/sensor_suite.hpp"
#include "garuda/control/flight_controller.hpp"
#include "garuda/payload/payload_system.hpp"
#include "garuda/core/health_system.hpp"
#include "garuda/core/telemetry_snapshot.hpp"
#include <string>
#include <vector>
#include <memory>
#include <cstdint>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

class DroneInstance {
public:
    explicit DroneInstance(
        std::string drone_id,
        QuadrotorConfig config = {},
        uint64_t seed = 1000,
        Vec3d spawn_pos = {0.0, 0.28, 0.0}
    ) noexcept;

    void reset(Vec3d spawn_pos = {0.0, 0.28, 0.0}, Quat spawn_orient = Quat::identity()) noexcept;

    // -------------------------------------------------------------------------
    // Control Commands
    // -------------------------------------------------------------------------
    void arm() noexcept;
    void disarm() noexcept;
    void set_attitude_setpoint(double roll_rad, double pitch_rad, double yaw_rate_rads, double throttle_norm) noexcept;
    void set_direct_motor_throttles(const std::vector<double>& throttles) noexcept;

    // -------------------------------------------------------------------------
    // Payload & Inspection Camera Subsystems
    // -------------------------------------------------------------------------
    bool attach_payload(PayloadType type) noexcept;
    bool detach_payload() noexcept;
    void set_camera_gimbal(double pitch_deg, double yaw_deg) noexcept;
    void set_camera_zoom(double zoom) noexcept;
    [[nodiscard]] const PayloadSystem& payload_system() const noexcept { return _payload; }
    [[nodiscard]] PayloadSystem& mutable_payload_system() noexcept { return _payload; }
    [[nodiscard]] const InspectionCamera& inspection_camera() const noexcept { return _payload.camera(); }

    // -------------------------------------------------------------------------
    // Sensor Suite & Health Foundation
    // -------------------------------------------------------------------------
    void set_sensor_enabled(size_t index, bool enabled) noexcept;
    void set_sensor_status(size_t index, SensorStatus status) noexcept;
    [[nodiscard]] const DeterministicSensorSuite& sensor_suite() const noexcept { return _sensors; }
    [[nodiscard]] DeterministicSensorSuite& mutable_sensor_suite() noexcept { return _sensors; }
    [[nodiscard]] const HealthSystem& health_system() const noexcept { return _health; }
    [[nodiscard]] const SubsystemHealthReport& health_report() const noexcept { return _last_health_report; }

    // -------------------------------------------------------------------------
    // Failure Injection (Deterministic Simulation Failures)
    // -------------------------------------------------------------------------
    void inject_motor_failure(size_t motor_index, MotorHealthState state, double degradation = 0.5) noexcept;
    void inject_sensor_failure(SensorMode mode) noexcept;
    void inject_battery_degradation(double capacity_factor, double resistance_multiplier) noexcept;
    void reset_all_failures() noexcept;

    // -------------------------------------------------------------------------
    // Physics Stepping Loop (400 Hz Authoritative Tick)
    // -------------------------------------------------------------------------
    void step(EnvironmentSystem& env, uint64_t tick, double dt) noexcept;

    // -------------------------------------------------------------------------
    // Telemetry & State Inspection
    // -------------------------------------------------------------------------
    [[nodiscard]] const std::string& id() const noexcept { return _id; }
    [[nodiscard]] const QuadrotorConfig& config() const noexcept { return _cfg; }
    [[nodiscard]] const RigidBody6DOFState& physics_state() const noexcept { return _integrator.state(); }
    [[nodiscard]] RigidBody6DOFState& mutable_physics_state() noexcept { return _integrator.mutable_state(); }
    [[nodiscard]] const BatteryState& battery_state() const noexcept { return _battery.state(); }
    [[nodiscard]] const DroneTelemetrySnapshot& telemetry() const noexcept { return _telemetry; }
    [[nodiscard]] bool is_armed() const noexcept { return _armed; }
    [[nodiscard]] bool is_in_contact() const noexcept { return _in_contact; }

private:
    std::string              _id;
    QuadrotorConfig          _cfg;
    uint64_t                 _seed{1000};
    bool                     _armed{false};
    bool                     _direct_motor_mode{false};
    std::vector<double>      _direct_throttles;
    FlightControlSetpoints   _setpoints{};
    bool                     _in_contact{true};

    // Subsystems
    RigidBodyIntegrator      _integrator;
    MotorSystem              _motors;
    PropulsionSystem         _propulsion;
    BatteryModel             _battery;
    GroundContactModel       _ground_contact;
    DeterministicSensorSuite _sensors;
    FlightControllerSystem   _controller;
    PayloadSystem            _payload;
    HealthSystem             _health;

    // Telemetry & Diagnostic Cache
    DroneTelemetrySnapshot   _telemetry{};
    SubsystemHealthReport    _last_health_report{};
    SyntheticIMUReading      _last_imu{};
    SyntheticBaroReading     _last_baro{};
    std::optional<SyntheticGPSReading> _last_gps{};
    SyntheticMagReading      _last_mag{};
    SyntheticLidarReading    _last_lidar{};
    SyntheticProximityReading _last_prox{};
};

} // namespace garuda
