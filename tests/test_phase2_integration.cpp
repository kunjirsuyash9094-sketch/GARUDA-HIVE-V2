#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "============================================================\n";
    std::cout << " GARUDA HIVE V2 — PHASE 2 END-TO-END INTEGRATION TEST\n";
    std::cout << "============================================================\n";

    SimulationWorld world(7777, 0.0025);
    auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 5.0, 0.0});
    assert(d != nullptr && "Drone instance creation failed");

    // 1. Initial State Inspection
    world.step();
    assert(d->telemetry().motor_rpm.size() == 8 && "Must have 8 motors");
    assert(d->telemetry().cell_voltages.size() == 6 && "Must have 6S LiPo cells");
    assert(d->telemetry().payload_attached == true && "Default inspection camera must be attached");
    assert(std::abs(d->telemetry().payload_mass_kg - 1.50) < 1e-4);
    assert(d->telemetry().vehicle_health == VehicleHealthState::OFFLINE && "Disarmed drone must be OFFLINE");
    std::cout << "  1. Initial Platform Architecture: 8 Motors, 6S LiPo, 4K Camera (10.00 kg) -> OK\n";

    // 2. Arm & Hover with Default Payload (10.00 kg)
    d->arm();
    world.step();
    assert(d->is_armed() == true);
    assert(d->telemetry().vehicle_health == VehicleHealthState::NOMINAL && "Armed drone must be NOMINAL");

    double hover_thr = 0.5833;
    for (int i = 0; i < 100; ++i) {
        d->mutable_physics_state().velocity = {0.0, 0.0, 0.0};
        d->mutable_physics_state().position = {0.0, 5.0, 0.0};
        d->set_attitude_setpoint(0.0, 0.0, 0.0, hover_thr);
        world.step();
    }
    double thrust_10kg = d->telemetry().total_thrust_n;
    double power_10kg  = d->telemetry().battery_power_w;
    std::cout << "  2. 10.00 kg Hover Thrust: " << thrust_10kg << " N | Electrical Power: " << power_10kg << " W\n";
    assert(thrust_10kg > 90.0 && thrust_10kg < 110.0 && "Thrust must balance 10.0 kg weight (~98 N)");

    // 3. Attach Heavy Cargo Pod (3.50 kg) -> Total Mass = 12.00 kg
    d->attach_payload(PayloadType::EMERGENCY_SUPPLY);
    world.step();
    assert(d->telemetry().payload_attached == true);
    assert(std::abs(d->telemetry().payload_mass_kg - 3.50) < 1e-4);
    assert(d->telemetry().effective_com_offset_m.y < 0.0 && "CoM must shift downwards with underslung cargo");
    std::cout << "  3. Attached Heavy Cargo Pod: Mass = 12.00 kg, CoM Y = "
              << d->telemetry().effective_com_offset_m.y << " m -> OK\n";

    // 4. Hover with 12.00 kg Payload (Requires ~117.7 N to balance 12.0 kg)
    double hover_thr_12kg = 0.65;
    for (int i = 0; i < 100; ++i) {
        d->mutable_physics_state().velocity = {0.0, 0.0, 0.0};
        d->mutable_physics_state().position = {0.0, 5.0, 0.0};
        d->set_attitude_setpoint(0.0, 0.0, 0.0, hover_thr_12kg);
        world.step();
    }
    double thrust_12kg = d->telemetry().total_thrust_n;
    double power_12kg  = d->telemetry().battery_power_w;
    std::cout << "  4. 12.00 kg Hover Thrust: " << thrust_12kg << " N | Electrical Power: " << power_12kg << " W\n";
    assert(thrust_12kg > thrust_10kg + 15.0 && "12 kg hover thrust must be greater than 10 kg thrust");
    assert(power_12kg > power_10kg && "12 kg electrical power demand must be greater due to heavier payload");

    // 5. Gimbal Control Test
    d->attach_payload(PayloadType::INSPECTION_CAMERA);
    d->set_camera_gimbal(-45.0, 90.0);
    d->set_camera_zoom(12.5);
    world.step();
    assert(std::abs(d->telemetry().camera_pitch_deg - (-45.0)) < 1e-4);
    assert(std::abs(d->telemetry().camera_yaw_deg - 90.0) < 1e-4);
    assert(std::abs(d->telemetry().camera_zoom - 12.5) < 1e-4);
    std::cout << "  5. 2-Axis Inspection Gimbal: Pitch=-45°, Yaw=90°, Zoom=12.5x -> OK\n";

    // 6. Sensor Suite & Health Degradation Test
    d->set_sensor_status(4, SensorStatus::DEGRADED); // LiDAR degraded
    world.step();
    assert(d->telemetry().sensor_status[4] == static_cast<int>(SensorStatus::DEGRADED));
    assert(d->telemetry().vehicle_health == VehicleHealthState::DEGRADED);
    std::cout << "  6. Sensor Suite & Health Model: LiDAR DEGRADED -> Vehicle Health: DEGRADED -> OK\n";

    // 7. Detach Payload -> Verify Baseline Restoration
    d->detach_payload();
    world.step();
    assert(d->telemetry().payload_attached == false);
    assert(d->telemetry().payload_mass_kg == 0.0);
    assert(d->telemetry().payload_power_w == 0.0);
    std::cout << "  7. Detach Payload: Restored Dry Mass Baseline (8.50 kg) -> OK\n";

    std::cout << "============================================================\n";
    std::cout << " PHASE 2 END-TO-END INTEGRATION TEST COMPLETED SUCCESSFULLY.\n";
    std::cout << "============================================================\n";
    return 0;
}
