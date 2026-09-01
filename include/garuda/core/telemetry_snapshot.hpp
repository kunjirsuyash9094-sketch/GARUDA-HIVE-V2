#pragma once
#include "core/math_types.hpp"
#include "garuda/payload/payload_catalogue.hpp"
#include "garuda/payload/inspection_camera.hpp"
#include "garuda/sensors/sensor_types.hpp"
#include "garuda/core/health_system.hpp"
#include <string>
#include <vector>
#include <array>
#include <cstdint>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

enum class VehicleLifecycleState : int {
    DISARMED = 0,
    ARMED = 1,
    TAKEOFF = 2,
    FLIGHT = 3,
    LANDING = 4,
    LANDED = 5,
    FAULT = 6
};

/**
 * @brief Authoritative telemetry snapshot for GARUDA-HL-01 at a discrete simulation tick.
 */
struct DroneTelemetrySnapshot {
    std::string drone_id{"GARUDA-HL-01"};
    uint64_t    simulation_tick{0};
    double      simulation_time_s{0.0};

    // Kinematic State (World Frame)
    Vec3d       position_world{0.0, 0.0, 0.0};
    Vec3d       velocity_world{0.0, 0.0, 0.0};
    Vec3d       acceleration_world{0.0, 0.0, 0.0};
    double      altitude_m{0.0};
    double      ground_speed_ms{0.0};
    double      vertical_speed_ms{0.0};

    // Attitude & Rotational State (Body/FRD Frame)
    Quat        orientation{0.0, 0.0, 0.0, 1.0};
    Vec3d       euler_rpy_deg{0.0, 0.0, 0.0}; // [Roll, Pitch, Yaw] in degrees
    Vec3d       angular_velocity_rads{0.0, 0.0, 0.0}; // [wx, wy, wz] in rad/s

    // Propulsion & Actuation (8-Rotor Heavy-Lift)
    double      total_thrust_n{0.0};
    double      thrust_to_weight_ratio{0.0};
    double      available_thrust_margin_n{0.0};
    std::vector<double> motor_rpm;
    std::vector<double> motor_thrust_n;
    std::vector<double> motor_power_w;
    std::vector<double> motor_temp_c;
    std::vector<int>    motor_health; // 0=Normal, 1=Degraded, 2=Failed

    // Electrical Energy & Battery System (6S LiPo)
    double      battery_voltage_terminal{25.2};
    double      battery_voltage_ocv{25.2};
    double      battery_current_amps{0.0};
    double      battery_soc{1.0}; // [0.0 - 1.0]
    double      battery_power_w{0.0};
    double      battery_temp_c{25.0};
    double      energy_consumed_joules{0.0};
    double      energy_remaining_joules{0.0};
    std::array<double, 6> cell_voltages{4.2, 4.2, 4.2, 4.2, 4.2, 4.2};

    // Modular Payload System
    std::string payload_id{"PAYLOAD-CAM-4K"};
    std::string payload_name{"GARUDA 4K Optical Inspection Gimbal"};
    std::string payload_category{"OPTICAL"};
    int         payload_type{1}; // 1 = INSPECTION_CAMERA
    double      payload_mass_kg{1.50};
    bool        payload_attached{true};
    double      payload_power_w{18.0};
    int         payload_state{2}; // 2 = ATTACHED
    int         payload_health{0}; // 0 = NOMINAL
    Vec3d       effective_com_offset_m{0.0, -0.018, 0.0};

    // Inspection Camera Subsystem
    int         camera_status{2}; // 2 = STREAMING
    int         camera_health{0}; // 0 = NOMINAL
    double      camera_pitch_deg{-15.0};
    double      camera_yaw_deg{0.0};
    double      camera_zoom{1.0};

    // Extensible Sensor Suite Foundation (8 Sensors)
    // Indices: 0=IMU, 1=GNSS, 2=BARO, 3=MAG, 4=LIDAR, 5=RGB, 6=THERMAL, 7=PROXIMITY
    std::array<int, 8> sensor_status{2, 2, 2, 2, 2, 2, 2, 2}; // 2 = NOMINAL
    std::array<int, 8> sensor_health{0, 0, 0, 0, 0, 0, 0, 0}; // 0 = NOMINAL
    double      lidar_distance_m{0.0};
    double      proximity_distance_m{0.0};

    // Aerodynamics & Environment
    double      air_density_kgm3{1.225};
    double      ground_effect_factor{1.0};
    bool        vrs_active{false};
    double      vrs_severity{0.0};
    Vec3d       wind_world{0.0, 0.0, 0.0};

    // Unified Vehicle Health & Lifecycle State
    bool        armed{false};
    bool        in_ground_contact{false};
    bool        low_voltage_warning{false};
    bool        critical_battery_cutoff{false};
    int         sensor_health_mode{0}; // 0=Ideal, 1=Simulated, 2=Degraded, 3=Failed
    VehicleHealthState vehicle_health{VehicleHealthState::NOMINAL};
    std::string health_diagnostics{"All systems nominal"};
    VehicleLifecycleState vehicle_state{VehicleLifecycleState::DISARMED};
    std::string flight_mode{"ATTITUDE_PID"};
};

/**
 * @brief World-level aggregate snapshot for multi-drone broadcasting.
 */
struct WorldTelemetrySnapshot {
    uint64_t                         tick{0};
    double                           timestamp_s{0.0};
    double                           physics_hz{400.0};
    double                           realtime_factor{1.0};
    std::vector<DroneTelemetrySnapshot> drones;
};

} // namespace garuda
