#include "garuda/sensors/sensor_suite.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "============================================================\n";
    std::cout << "[TEST] Running test_sensor_suite (Phase 2 Sensor Foundation)\n";
    std::cout << "============================================================\n";

    QuadrotorConfig cfg{};
    DeterministicSensorSuite sensors(cfg, 2026);

    // 1. Verify 8-Sensor Suite Registration
    const auto& descs = sensors.descriptors();
    assert(descs.size() == 8 && "Must register exactly 8 sensor modules");
    std::cout << "  1. Sensor Suite Registration (8 Sensors):\n";
    for (size_t i = 0; i < descs.size(); ++i) {
        std::cout << "     [" << i << "] " << std::left << std::setw(18) << descs[i].sensor_id
                  << " | " << std::setw(34) << descs[i].name
                  << " | Rate: " << std::setw(5) << descs[i].update_rate_hz << " Hz"
                  << " | Status: " << sensor_status_to_string(descs[i].status) << "\n";
        assert(descs[i].enabled == true);
        assert(descs[i].status == SensorStatus::NOMINAL);
        assert(descs[i].health == 0);
    }

    // 2. Physical Measurement Sanity
    dronesim::AtmosphericState atm{};
    atm.pressure = 101325.0;
    atm.density = 1.225;
    atm.temperature = 288.15;

    Vec3d true_accel{0.0, 0.0, 0.0};
    Vec3d true_omega{0.0, 0.0, 0.0};
    Quat  orient = Quat::identity();

    auto imu = sensors.measure_imu(true_accel, true_omega, orient, 9.80665, 0.0025, 0.0);
    std::cout << "  2. Physical Measurement Tests:\n";
    std::cout << "     - IMU Specific Force Body: (" << imu.accel_body_ms2.x << ", " << imu.accel_body_ms2.y << ", " << imu.accel_body_ms2.z << ") m/s^2\n";
    assert(std::abs(imu.accel_body_ms2.y - 9.80665) < 0.20 && "IMU at rest on level ground must measure +1g specific force upwards");

    auto baro = sensors.measure_baro(atm, 0.0, 0.0025, 0.0);
    std::cout << "     - Barometer Altitude: " << baro.altitude_m << " m (Pressure: " << baro.pressure_pa << " Pa)\n";
    assert(std::abs(baro.pressure_pa - 101325.0) < 50.0 && "Baro pressure must be near ISA sea level");

    auto lidar = sensors.measure_lidar(15.50, 0.0025, 0.0);
    std::cout << "     - LiDAR Distance: " << lidar.distance_m << " m (Ground truth: 15.50 m)\n";
    assert(std::abs(lidar.distance_m - 15.50) < 0.15 && "LiDAR measurement must be accurate within noise bounds");

    auto prox = sensors.measure_proximity(0.28, 0.0025, 0.0);
    std::cout << "     - Proximity Distance: " << prox.distance_down_m << " m, Obstacle: " << prox.obstacle_detected << "\n";
    assert(prox.obstacle_detected == true && "Proximity sensor must flag obstacle when within ground threshold");

    // 3. Sensor Enabling & Disabling
    sensors.set_sensor_enabled(4, false); // Disable LiDAR
    assert(sensors.descriptors()[4].enabled == false);
    assert(sensors.descriptors()[4].status == SensorStatus::OFFLINE);
    auto lidar_off = sensors.measure_lidar(15.50, 0.0025, 0.0);
    assert(lidar_off.signal_quality == 0.0 && "Disabled sensor must report dead signal");
    sensors.set_sensor_enabled(4, true);
    assert(sensors.descriptors()[4].enabled == true);
    std::cout << "  3. Per-Sensor Enable/Disable Toggling -> OK\n";

    // 4. Degradation and Fault Injection
    sensors.set_sensor_status(0, SensorStatus::DEGRADED); // IMU degraded
    assert(sensors.descriptors()[0].health == 1);
    sensors.set_sensor_status(1, SensorStatus::FAULT);    // GPS fault
    assert(sensors.descriptors()[1].health == 2);
    auto gps_fault = sensors.measure_gps({0, 10, 0}, {0, 0, 0}, 1.0, 1.0);
    assert(!gps_fault.has_value() && "Faulty GNSS must produce no position fix");
    std::cout << "  4. Sensor Fault & Degradation Injection -> OK\n";

    // 5. Determinism Verification
    DeterministicSensorSuite s1(cfg, 4242);
    DeterministicSensorSuite s2(cfg, 4242);

    for (int i = 0; i < 50; ++i) {
        auto imu1 = s1.measure_imu(true_accel, true_omega, orient, 9.80665, 0.0025, i * 0.0025);
        auto imu2 = s2.measure_imu(true_accel, true_omega, orient, 9.80665, 0.0025, i * 0.0025);
        assert(imu1.accel_body_ms2.x == imu2.accel_body_ms2.x);
        assert(imu1.accel_body_ms2.y == imu2.accel_body_ms2.y);
        assert(imu1.accel_body_ms2.z == imu2.accel_body_ms2.z);
    }
    std::cout << "  5. Bit-Identical Determinism across Seeds Verified -> OK\n";

    std::cout << "============================================================\n";
    std::cout << "[TEST] test_sensor_suite: ALL CHECKS PASSED (100%).\n";
    std::cout << "============================================================\n";
    return 0;
}
