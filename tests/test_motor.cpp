#include "garuda/physics/motor_system.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_motor...\n";
    QuadrotorConfig cfg;
    MotorSystem motors(cfg);

    // 1. Initial State
    assert(motors.size() == 8);
    for (const auto& m : motors.motors()) {
        assert(m.rpm == 0.0);
        assert(m.health == MotorHealthState::NORMAL);
    }

    // 2. First-Order Step Response (tau = cfg.esc_time_constant_s)
    double supply_v = 25.2;
    motors.set_commands({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, supply_v);

    double dt = 0.0025;
    double target_omega = motors.motors()[0].omega_cmd;
    std::cout << "  Commanded Max Omega: " << target_omega << " rad/s\n";

    // Step by exactly 1 tau (cfg.esc_time_constant_s / dt ticks)
    int tau_ticks = static_cast<int>(std::round(cfg.esc_time_constant_s / dt));
    for (int i = 0; i < tau_ticks; ++i) {
        motors.step_dynamics(dt);
    }

    double omega_at_tau = motors.motors()[0].omega;
    double expected_ratio = 1.0 - std::exp(-1.0); // ~0.6321
    double actual_ratio = omega_at_tau / target_omega;
    std::cout << "  Omega at 1 tau (" << tau_ticks << " ticks): " << omega_at_tau << " (Ratio: " << actual_ratio << ", Expected: ~0.632)\n";
    assert(std::abs(actual_ratio - expected_ratio) < 0.02 && "Motor step response must match first-order exponential lag");

    // 3. Failure State Injection
    motors.set_motor_health(0, MotorHealthState::FAILED);
    motors.set_commands({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, supply_v);
    assert(motors.motors()[0].omega_cmd == 0.0 && "Failed motor commanded omega must be zero");

    for (int i = 0; i < 100; ++i) motors.step_dynamics(dt); // 0.25s = 16.7 tau
    std::cout << "  Failed Motor RPM after 0.25s: " << motors.motors()[0].rpm << "\n";
    assert(motors.motors()[0].rpm < 0.1 && "Failed motor must spin down to 0 RPM");
    assert(motors.motors()[1].rpm > 5000.0 && "Healthy motors must spin at full speed");

    std::cout << "[TEST] test_motor: ALL CHECKS PASSED.\n";
    return 0;
}
