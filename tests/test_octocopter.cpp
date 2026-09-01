#include "garuda/core/simulation_world.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_octocopter...\n";

    SimulationWorld world(2026, 0.0025);
    QuadrotorConfig cfg{};
    auto* d = world.add_drone("GARUDA-HL-01", cfg, {0.0, 5.0, 0.0});
    assert(d != nullptr && "Failed to add octocopter instance");

    // 1. Verify 8-Motor Registration & Geometry
    const auto& motors = d->telemetry().motor_rpm;
    assert(motors.size() == 8 && "Octocopter must register exactly 8 motors");
    std::cout << "  8 Motors Registered: OK\n";

    // 2. Arm and Verify Idle Spin
    d->arm();
    std::cout << "  Armed flag: " << d->is_armed() << std::endl;
    for (int i = 0; i < 10; ++i) {
        try {
            world.step();
            std::cout << "  Tick " << (i+1) << " | Motor 1 RPM: " << d->telemetry().motor_rpm[0] << " | Thrust: " << d->telemetry().total_thrust_n << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "  EXCEPTION ON TICK " << (i+1) << ": " << e.what() << std::endl;
            throw;
        }
    }
    for (size_t i = 0; i < 8; ++i) {
        std::cout << "  Final Motor " << (i+1) << " RPM: " << d->telemetry().motor_rpm[i] << std::endl;
    }
    for (size_t i = 0; i < 8; ++i) {
        assert(d->telemetry().motor_rpm[i] > 10.0 && "All 8 motors must spin up to idle RPM upon arming");
    }
    std::cout << "  8-Motor Idle Spin-up: OK" << std::endl;

    // 3. Hover Equilibrium Test
    // Nominal mass: 10.0 kg (8.5 kg dry + 1.5 kg camera) -> Weight = 10.0 * 9.80665 = 98.0665 N
    // 8x 15" rotors with 380 KV on 6S LiPo (25.2V) produce ~280 N max thrust.
    // Calibrate steady-state hover throttle for 98.07 N:
    double hover_thr = 0.5833;
    for (int i = 0; i < 100; ++i) { // 0.25s (16.7 tau, full ESC steady-state)
        d->set_attitude_setpoint(0.0, 0.0, 0.0, hover_thr);
        world.step();
    }

    double thrust = d->telemetry().total_thrust_n;
    double weight = (cfg.dry_mass_kg + d->telemetry().payload_mass_kg) * 9.80665;
    double thrust_err = std::abs(thrust - weight);
    std::cout << "  Hover Thrust at thr=" << hover_thr << ": " << thrust << " N (Weight: " << weight << " N)" << std::endl;
    std::cout << "  Hover Thrust Error: " << thrust_err << " N" << std::endl;
    assert(thrust_err < 10.0 && "8-motor hover thrust must balance 10.0 kg vehicle weight");

    // 4. Individual Motor Failure Test (Motor 3 Failure)
    d->inject_motor_failure(2, MotorHealthState::FAILED);
    for (int i = 0; i < 100; ++i) { // 0.25s
        d->set_attitude_setpoint(0.0, 0.0, 0.0, hover_thr);
        world.step();
    }
    std::cout << "  Motor 3 Failed RPM: " << d->telemetry().motor_rpm[2] << " (Expected: 0.0)\n";
    assert(d->telemetry().motor_rpm[2] < 1.0 && "Failed motor 3 must spin down to 0 RPM");
    assert(d->telemetry().motor_rpm[0] > 1000.0 && "Healthy motors must maintain spin");

    std::cout << "[TEST] test_octocopter: ALL CHECKS PASSED.\n";
    return 0;
}
