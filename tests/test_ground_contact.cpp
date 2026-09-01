#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_ground_contact...\n";
    SimulationWorld world(105, 0.0025);
    auto* d = world.add_drone("TEST-01", {}, {0.0, 1.0, 0.0});
    d->disarm();

    // Drop from 1.0m height and observe ground settling on 0.280m landing skids
    for (int i = 0; i < 600; ++i) { // 1.5s
        world.step();
    }

    const auto& s = d->physics_state();
    std::cout << "  Final Resting Altitude: " << s.position.y << " m (Expected: ~0.280 m)\n";
    std::cout << "  Final Vertical Velocity: " << s.velocity.y << " m/s (Expected: ~0.0 m/s)\n";

    assert(s.position.y >= 0.279 && s.position.y <= 0.285 && "Drone must settle stably on ground surface");
    assert(std::abs(s.velocity.y) < 1e-3 && "Vertical velocity must be zero at rest (no jitter)");
    assert(d->is_in_contact() && "In-contact flag must be true");

    std::cout << "[TEST] test_ground_contact: ALL CHECKS PASSED.\n";
    return 0;
}
