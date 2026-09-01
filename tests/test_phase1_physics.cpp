#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_phase1_physics...\n";

    // 1. Free Fall Test (Initial acceleration at v=0 must equal -9.80665 m/s^2)
    {
        SimulationWorld world(101, 0.0025);
        auto* d = world.add_drone("TEST-01", {}, {0.0, 50.0, 0.0});
        d->disarm();

        world.step();
        double a = d->physics_state().acceleration.y;
        std::cout << "  Initial Free Fall Acceleration: " << a << " m/s^2 (Expected: -9.80665)\n";
        assert(std::abs(a - (-9.80665)) < 1e-4 && "Free fall initial acceleration must equal -9.80665 m/s^2");
    }

    // 2. Hover Equilibrium Test (8-Rotor Heavy Lift with 1.50kg Inspection Camera = 10.00kg Total Mass)
    {
        SimulationWorld world(7777, 0.0025);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 5.0, 0.0});
        d->arm();

        double weight = (d->config().dry_mass_kg + 1.50) * 9.80665; // 98.0665 N
        double hover_thr = 0.5833; // Calibrated 8-rotor hover throttle for 10.0kg at 25.2V
        // Hold altitude fixed while ESC spins up to hover RPM (0.25s)
        for (int i = 0; i < 100; ++i) {
            d->mutable_physics_state().velocity = {0.0, 0.0, 0.0};
            d->mutable_physics_state().position = {0.0, 5.0, 0.0};
            d->set_attitude_setpoint(0.0, 0.0, 0.0, hover_thr);
            world.step();
        }
        double thrust = d->telemetry().total_thrust_n;
        std::cout << "  Hover Thrust at thr=" << hover_thr << ": " << thrust << " N (Weight: " << weight << " N)" << std::endl;
        double thr_err = std::abs(thrust - weight);
        std::cout << "  Error: " << thr_err << " N" << std::endl;
        assert(thr_err < 5.0 && "Hover thrust must balance weight within operating margin");
    }

    // 3. Aerodynamic Drag Test
    {
        SimulationWorld world(103, 0.0025);
        auto* d = world.add_drone("TEST-01", {}, {0.0, 50.0, 0.0});
        d->disarm();
        d->mutable_physics_state().velocity = { 20.0, 0.0, 0.0 }; // Fast horizontal velocity

        world.step();
        double vx1 = d->physics_state().velocity.x;
        assert(vx1 < 20.0 && "Airframe drag must decelerate horizontal speed");
    }

    std::cout << "[TEST] test_phase1_physics: ALL CHECKS PASSED.\n";
    return 0;
}
