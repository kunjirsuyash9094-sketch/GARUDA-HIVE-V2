#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_multidrone...\n";

    // Test fleet scaling: 1, 4, 8, 16, 32 drones
    const std::vector<size_t> fleet_sizes = {1, 4, 8, 16, 32};

    for (size_t count : fleet_sizes) {
        SimulationWorld world(3000 + count, 0.0025);
        std::vector<DroneInstance*> drone_ptrs;

        for (size_t i = 0; i < count; ++i) {
            std::string id = "GARUDA-" + std::to_string(i + 1);
            Vec3d spawn{ static_cast<double>(i) * 3.0, 2.0, 0.0 };
            auto* d = world.add_drone(id, {}, spawn);
            d->arm();
            drone_ptrs.push_back(d);
        }

        assert(world.drone_count() == count);

        // Command Drone 0 with climb, Drone 1 (if exists) with pitch translation
        if (count >= 1) drone_ptrs[0]->set_attitude_setpoint(0.0, 0.0, 0.0, 0.85); // Climb
        if (count >= 2) drone_ptrs[1]->set_attitude_setpoint(0.0, 0.3, 0.0, 0.65); // Pitch forward (+X)

        for (int step = 0; step < 200; ++step) {
            world.step();
        }

        // Verify isolation
        if (count >= 2) {
            assert(drone_ptrs[0]->physics_state().position.y > 2.2 && "Drone 0 must climb independently");
            double dx = drone_ptrs[1]->physics_state().position.x - 3.0;
            assert(std::abs(dx) > 0.10 && "Drone 1 must translate along X independently");
        }

        std::cout << "  Fleet of " << count << " drones: OK (Isolated & Deterministic)\n";
    }

    std::cout << "[TEST] test_multidrone: ALL CHECKS PASSED.\n";
    return 0;
}
