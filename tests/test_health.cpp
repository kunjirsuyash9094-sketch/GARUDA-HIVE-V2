#include "garuda/core/health_system.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>

using namespace garuda;

int main() {
    std::cout << "============================================================\n";
    std::cout << "[TEST] Running test_health (Phase 2 Drone Health System)\n";
    std::cout << "============================================================\n";

    QuadrotorConfig cfg{};
    MotorSystem motors(cfg);
    BatteryModel battery(cfg);
    DeterministicSensorSuite sensors(cfg, 1000);
    PayloadSystem payload;
    HealthSystem health;

    // 1. Disarmed State -> OFFLINE
    auto rep_offline = health.evaluate(motors, battery, sensors, payload, false);
    assert(rep_offline.overall == VehicleHealthState::OFFLINE);
    std::cout << "  1. Disarmed State -> Overall Health: OFFLINE -> OK\n";

    // 2. Armed Nominal State -> NOMINAL
    auto rep_nominal = health.evaluate(motors, battery, sensors, payload, true);
    assert(rep_nominal.overall == VehicleHealthState::NOMINAL);
    assert(rep_nominal.motor_health_level == 0);
    assert(rep_nominal.battery_health_level == 0);
    assert(rep_nominal.sensor_health_level == 0);
    assert(rep_nominal.payload_health_level == 0);
    std::cout << "  2. Armed All Systems Nominal -> Overall Health: NOMINAL -> OK\n";

    // 3. Subsystem Degradation -> DEGRADED
    sensors.set_sensor_status(4, SensorStatus::DEGRADED); // LiDAR degraded
    auto rep_degraded = health.evaluate(motors, battery, sensors, payload, true);
    assert(rep_degraded.overall == VehicleHealthState::DEGRADED);
    assert(rep_degraded.sensor_health_level == 1);
    sensors.set_sensor_status(4, SensorStatus::NOMINAL);
    std::cout << "  3. Single Sensor Degradation -> Overall Health: DEGRADED -> OK\n";

    // 4. Single Motor Failure -> WARNING (Octo-X 8-Rotor Redundancy Mode)
    motors.set_motor_health(2, MotorHealthState::FAILED);
    auto rep_warning = health.evaluate(motors, battery, sensors, payload, true);
    assert(rep_warning.overall == VehicleHealthState::WARNING);
    assert(rep_warning.motor_health_level == 1);
    std::cout << "  4. Single Rotor Failure -> Overall Health: WARNING (Octo-X Redundant) -> OK\n";

    // 5. Dual Motor Failure -> CRITICAL
    motors.set_motor_health(3, MotorHealthState::FAILED);
    auto rep_critical = health.evaluate(motors, battery, sensors, payload, true);
    assert(rep_critical.overall == VehicleHealthState::CRITICAL);
    assert(rep_critical.motor_health_level == 2);
    std::cout << "  5. Dual Rotor Failure -> Overall Health: CRITICAL -> OK\n";

    // 6. Battery Critical Cutoff -> CRITICAL
    motors.set_motor_health(2, MotorHealthState::NORMAL);
    motors.set_motor_health(3, MotorHealthState::NORMAL);
    // Discharge battery to empty
    for (int i = 0; i < 500; ++i) battery.step(2000.0, 1.0);
    auto rep_bat_crit = health.evaluate(motors, battery, sensors, payload, true);
    assert(rep_bat_crit.overall == VehicleHealthState::CRITICAL);
    assert(rep_bat_crit.battery_health_level == 2);
    std::cout << "  6. Battery Depletion / Critical Cutoff -> Overall Health: CRITICAL -> OK\n";

    std::cout << "============================================================\n";
    std::cout << "[TEST] test_health: ALL CHECKS PASSED (100%).\n";
    std::cout << "============================================================\n";
    return 0;
}
