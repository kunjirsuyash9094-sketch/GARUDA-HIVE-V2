#include "garuda/physics/battery_model.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "[TEST] Running test_battery...\n";
    QuadrotorConfig cfg;
    BatteryModel battery(cfg);

    // 1. Initial State Check (6S LiPo: 25.20 V max)
    assert(battery.state().soc == 1.0);
    assert(std::abs(battery.terminal_voltage() - 25.20) < 1e-3);
    assert(battery.state().ledger.energy_consumed_joules == 0.0);

    // 2. Voltage Sag Under Load
    // Drawing 200W electrical load on 6S pack (I ~ 7.94 A)
    double v_loaded = battery.step(200.0, 0.1);
    std::cout << "  Loaded Voltage at 200W: " << v_loaded << " V (Unloaded: 25.20 V)\n";
    std::cout << "  Current draw at 200W: " << battery.state().current_amps << " A\n";
    assert(v_loaded < 25.20 && "Terminal voltage must sag under electrical load");
    assert(battery.state().current_amps > 5.0 && "Current draw must be positive and non-zero");

    // 3. Coulomb Counting & Energy Conservation Ledger
    double initial_energy = battery.state().ledger.initial_energy_joules;
    for (int i = 0; i < 100; ++i) {
        battery.step(250.0, 0.1); // 10 seconds total
    }

    const auto& s = battery.state();
    std::cout << "  SoC after 10s: " << (s.soc * 100.0) << "%\n";
    std::cout << "  Energy Consumed: " << s.ledger.energy_consumed_joules << " J\n";
    std::cout << "  Energy Remaining: " << s.ledger.energy_remaining_joules << " J\n";

    assert(s.soc < 1.0 && "SoC must decrease");
    assert(s.ledger.energy_consumed_joules > 2000.0 && "Energy consumed must account for power x dt");
    double energy_sum = s.ledger.energy_consumed_joules + s.ledger.energy_remaining_joules;
    assert(std::abs(energy_sum - initial_energy) < 1e-4 && "Energy ledger must be perfectly conserved");

    // 4. Peukert Effect Test (High C-rate derates effective capacity)
    BatteryModel b_nom(cfg);
    BatteryModel b_high(cfg);

    // 1C discharge (16A, ~380W) vs 2.5C discharge (40A, ~950W)
    b_nom.step(380.0, 0.1);
    b_high.step(950.0, 0.1);

    std::cout << "  1C Effective Capacity: " << b_nom.state().effective_capacity_ah
              << " Ah | 2.5C Effective Capacity: " << b_high.state().effective_capacity_ah << " Ah\n";
    assert(b_high.state().effective_capacity_ah < b_nom.state().effective_capacity_ah && "Peukert effect must derate effective capacity under high discharge rates");

    std::cout << "[TEST] test_battery: ALL CHECKS PASSED.\n";
    return 0;
}
