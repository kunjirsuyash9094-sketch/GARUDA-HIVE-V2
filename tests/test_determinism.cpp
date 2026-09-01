#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <cassert>
#include <vector>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_determinism...\n";

    auto run_sim = [](uint64_t seed) -> uint64_t {
        SimulationWorld world(seed, 0.0025);
        auto* d1 = world.add_drone("GARUDA-01", {}, {0.0, 2.0, 0.0});
        auto* d2 = world.add_drone("GARUDA-02", {}, {5.0, 2.0, 0.0});
        d1->arm();
        d2->arm();

        for (int i = 0; i < 400; ++i) {
            double thr1 = 0.50 + 0.30 * std::sin(i * 0.05);
            double roll2 = 0.20 * std::cos(i * 0.03);
            d1->set_attitude_setpoint(0.0, 0.0, 0.0, thr1);
            d2->set_attitude_setpoint(roll2, 0.0, 0.0, 0.584);
            world.step();
        }
        return world.compute_world_state_hash();
    };

    uint64_t baseline_hash = run_sim(424242);
    std::cout << "  Baseline Run Hash: 0x" << std::hex << baseline_hash << std::dec << "\n";

    // Run 10 consecutive identical simulations
    for (int run = 1; run <= 10; ++run) {
        uint64_t run_hash = run_sim(424242);
        std::cout << "  Run #" << run << " Hash: 0x" << std::hex << run_hash << std::dec << " -> "
                  << (run_hash == baseline_hash ? "MATCH" : "MISMATCH") << "\n";
        assert(run_hash == baseline_hash && "Determinism violation: identical initial conditions must produce identical state hashes");
    }

    std::cout << "[TEST] test_determinism: ALL 10 RUNS 100% BIT-IDENTICAL.\n";
    return 0;
}
