#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>

using namespace garuda;

int main() {
    std::cout << "============================================================\n";
    std::cout << " GARUDA HIVE V2 — PHYSICS BENCHMARK & PROFILER\n";
    std::cout << "============================================================\n";

    const std::vector<size_t> fleet_counts = {1, 4, 8, 16, 32};
    const std::vector<double> frequencies  = {100.0, 200.0, 400.0, 500.0};
    const size_t ticks_to_benchmark = 2000; // 2000 ticks per run

    std::cout << std::left
              << std::setw(10) << "Drones"
              << std::setw(12) << "Freq (Hz)"
              << std::setw(14) << "Dt (s)"
              << std::setw(18) << "Tick Time (us)"
              << std::setw(18) << "Real-Time Factor"
              << std::setw(14) << "Max Drones @ 400Hz"
              << "\n";
    std::cout << "------------------------------------------------------------------------------------\n";

    for (size_t drones : fleet_counts) {
        for (double freq : frequencies) {
            double dt = 1.0 / freq;
            SimulationWorld world(777, dt);

            for (size_t i = 0; i < drones; ++i) {
                std::string id = "DRONE-" + std::to_string(i + 1);
                auto* d = world.add_drone(id, {}, {static_cast<double>(i) * 2.0, 2.0, 0.0});
                d->arm();
                d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.584);
            }

            // Warmup
            for (int w = 0; w < 100; ++w) world.step();

            auto t_start = std::chrono::high_resolution_clock::now();
            for (size_t t = 0; t < ticks_to_benchmark; ++t) {
                world.step();
            }
            auto t_end = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double, std::micro> elapsed_us = t_end - t_start;
            double avg_tick_us = elapsed_us.count() / static_cast<double>(ticks_to_benchmark);
            double sim_time_s = ticks_to_benchmark * dt;
            double wall_time_s = elapsed_us.count() / 1e6;
            double rtf = sim_time_s / wall_time_s;

            // Compute theoretical max drones at 400 Hz (target budget: 2500 us per tick)
            double tick_per_drone_us = avg_tick_us / static_cast<double>(drones);
            double max_drones_400hz = 2500.0 / std::max(tick_per_drone_us, 0.1);

            std::cout << std::left
                      << std::setw(10) << drones
                      << std::setw(12) << std::fixed << std::setprecision(0) << freq
                      << std::setw(14) << std::fixed << std::setprecision(4) << dt
                      << std::setw(18) << std::fixed << std::setprecision(2) << avg_tick_us
                      << std::setw(18) << std::fixed << std::setprecision(1) << rtf << "x"
                      << std::setw(14) << std::fixed << std::setprecision(0) << max_drones_400hz
                      << "\n";
        }
    }

    std::cout << "============================================================\n";
    std::cout << " BENCHMARK COMPLETE.\n";
    std::cout << "============================================================\n";
    return 0;
}
