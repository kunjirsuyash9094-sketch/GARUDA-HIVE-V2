#pragma once
#include "garuda/physics/motor_system.hpp"
#include "garuda/physics/battery_model.hpp"
#include "garuda/payload/payload_system.hpp"
#include "garuda/sensors/sensor_suite.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <algorithm>

namespace garuda {

/**
 * @brief Unified vehicle health states.
 */
enum class VehicleHealthState : int {
    NOMINAL = 0,    // All subsystems operating within normal parameters
    DEGRADED = 1,   // Minor subsystem performance reduction (e.g. sensor degraded, motor 10% degraded)
    WARNING = 2,    // Subsystem alert requiring attention (e.g. low battery, single motor loss)
    CRITICAL = 3,   // Severe emergency (e.g. battery cutoff impending, multiple motor failures)
    FAULT = 4,      // System level failure preventing safe flight
    OFFLINE = 5     // Vehicle disarmed/unpowered
};

/**
 * @brief Subsystem health breakdown.
 */
struct SubsystemHealthReport {
    int motor_health_level{0};       // 0=NOMINAL, 1=DEGRADED, 2=CRITICAL/FAULT
    int battery_health_level{0};     // 0=NOMINAL, 1=WARNING, 2=CRITICAL
    int sensor_health_level{0};      // 0=NOMINAL, 1=DEGRADED, 2=FAULT
    int payload_health_level{0};     // 0=NOMINAL, 1=DEGRADED, 2=FAULT
    int comms_health_level{0};       // 0=NOMINAL, 1=DEGRADED, 2=FAULT
    VehicleHealthState overall{VehicleHealthState::NOMINAL};
    std::string diagnostics{"All systems nominal"};
};

class HealthSystem {
public:
    HealthSystem() noexcept = default;

    [[nodiscard]] SubsystemHealthReport evaluate(
        const MotorSystem& motors,
        const BatteryModel& battery,
        const DeterministicSensorSuite& sensors,
        const PayloadSystem& payload,
        bool armed
    ) const noexcept {
        SubsystemHealthReport report{};

        if (!armed) {
            report.overall = VehicleHealthState::OFFLINE;
            report.diagnostics = "Vehicle disarmed / standby";
            return report;
        }

        // 1. Motor Health Evaluation (8 Motors)
        size_t failed_motors = 0;
        size_t degraded_motors = 0;
        for (const auto& m : motors.motors()) {
            if (m.health == MotorHealthState::FAILED) failed_motors++;
            else if (m.health == MotorHealthState::DEGRADED) degraded_motors++;
        }

        if (failed_motors >= 2) {
            report.motor_health_level = 2; // FAULT
        } else if (failed_motors == 1 || degraded_motors >= 2) {
            report.motor_health_level = 1; // DEGRADED / WARNING
        } else if (degraded_motors == 1) {
            report.motor_health_level = 1;
        } else {
            report.motor_health_level = 0;
        }

        // 2. Battery Health Evaluation
        const auto& bstate = battery.state();
        if (bstate.critical_cutoff || bstate.soc <= 0.0) {
            report.battery_health_level = 2; // CRITICAL
        } else if (bstate.low_voltage_warning || bstate.soc < 0.20) {
            report.battery_health_level = 1; // WARNING
        } else {
            report.battery_health_level = 0;
        }

        // 3. Sensor Suite Health Evaluation
        size_t faulty_sensors = 0;
        size_t degraded_sensors = 0;
        for (const auto& s : sensors.descriptors()) {
            if (s.status == SensorStatus::FAULT) faulty_sensors++;
            else if (s.status == SensorStatus::DEGRADED) degraded_sensors++;
        }

        if (faulty_sensors >= 2 || (sensors.descriptors()[0].status == SensorStatus::FAULT)) {
            report.sensor_health_level = 2; // IMU fault is critical
        } else if (faulty_sensors > 0 || degraded_sensors > 0) {
            report.sensor_health_level = 1;
        } else {
            report.sensor_health_level = 0;
        }

        // 4. Payload Health Evaluation
        if (payload.state() == PayloadState::FAULT || payload.current().health == 2) {
            report.payload_health_level = 2;
        } else if (payload.current().health == 1) {
            report.payload_health_level = 1;
        } else {
            report.payload_health_level = 0;
        }

        // 5. Overall Health Aggregation
        if (report.battery_health_level == 2 || report.motor_health_level == 2 || faulty_sensors >= 2) {
            report.overall = VehicleHealthState::CRITICAL;
            report.diagnostics = "CRITICAL: Multiple subsystem failures detected";
        } else if (failed_motors == 1) {
            report.overall = VehicleHealthState::WARNING;
            report.diagnostics = "WARNING: Single rotor failure — Octo-X redundancy active";
        } else if (report.battery_health_level == 1) {
            report.overall = VehicleHealthState::WARNING;
            report.diagnostics = "WARNING: Low battery voltage / SoC threshold";
        } else if (report.motor_health_level == 1 || report.sensor_health_level == 1 || report.payload_health_level == 1) {
            report.overall = VehicleHealthState::DEGRADED;
            report.diagnostics = "DEGRADED: Subsystem degradation active";
        } else {
            report.overall = VehicleHealthState::NOMINAL;
            report.diagnostics = "All systems nominal (400 Hz authoritative C++ kernel)";
        }

        return report;
    }

    [[nodiscard]] static std::string_view health_state_to_string(VehicleHealthState s) noexcept {
        switch (s) {
            case VehicleHealthState::NOMINAL:  return "NOMINAL";
            case VehicleHealthState::DEGRADED: return "DEGRADED";
            case VehicleHealthState::WARNING:  return "WARNING";
            case VehicleHealthState::CRITICAL: return "CRITICAL";
            case VehicleHealthState::FAULT:    return "FAULT";
            case VehicleHealthState::OFFLINE:  return "OFFLINE";
            default:                           return "UNKNOWN";
        }
    }
};

} // namespace garuda
