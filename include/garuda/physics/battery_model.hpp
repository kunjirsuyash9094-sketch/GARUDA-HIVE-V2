#pragma once
#include "garuda/config/quadrotor_config.hpp"
#include <vector>
#include <array>

namespace garuda {

/**
 * @brief Energy ledger tracking all power consumers deterministically.
 */
struct EnergyLedger {
    double motor_power_w{0.0};
    double avionics_power_w{0.0};
    double total_power_w{0.0};
    double energy_consumed_joules{0.0};
    double energy_remaining_joules{0.0};
    double initial_energy_joules{0.0};
};

/**
 * @brief Complete battery telemetry state.
 */
struct BatteryState {
    double soc{1.0};                      // State of charge [0.0 - 1.0]
    double voltage_terminal{25.2};        // Loaded terminal voltage (V)
    double voltage_ocv{25.2};             // Open circuit voltage (V)
    double current_amps{0.0};             // Total discharge current (A)
    double temperature_c{25.0};           // Battery pack temperature (deg C)
    double capacity_used_ah{0.0};         // Cumulative capacity discharged (Ah)
    double effective_capacity_ah{16.0};   // Peukert-adjusted capacity (Ah)
    bool   low_voltage_warning{false};    // Cell voltage < 3.50V
    bool   critical_cutoff{false};        // Cell voltage <= 3.27V
    std::array<double, 6> cell_voltages{4.20, 4.20, 4.20, 4.20, 4.20, 4.20}; // Individual cell voltages (V)
    EnergyLedger ledger{};
};

/**
 * @brief Simplified Equivalent-Circuit Battery Model for 6S2P Industrial Pack.
 */
class BatteryModel {
public:
    explicit BatteryModel(const QuadrotorConfig& config) noexcept;

    void reset() noexcept;

    /**
     * @brief Steps the battery simulation by dt seconds.
     * 
     * @param electrical_power_demand_w Total electrical power demanded (motors + avionics + payload)
     * @param dt Physics integration timestep
     * @return Loaded terminal voltage available to ESCs
     */
    double step(double electrical_power_demand_w, double dt) noexcept;

    [[nodiscard]] const BatteryState& state() const noexcept { return _state; }
    [[nodiscard]] double terminal_voltage() const noexcept { return _state.voltage_terminal; }
    [[nodiscard]] double soc() const noexcept { return _state.soc; }
    [[nodiscard]] bool is_depleted() const noexcept { return _state.critical_cutoff; }

    void inject_battery_degradation(double capacity_factor, double resistance_multiplier) noexcept;

private:
    [[nodiscard]] double _calculate_cell_ocv(double soc) const noexcept;

    const QuadrotorConfig& _cfg;
    BatteryState           _state{};
    double                 _nominal_capacity_ah{16.0};
    double                 _internal_resistance_ohm{0.012};
    double                 _capacity_degradation{1.0};
    double                 _resistance_multiplier{1.0};
};

} // namespace garuda
