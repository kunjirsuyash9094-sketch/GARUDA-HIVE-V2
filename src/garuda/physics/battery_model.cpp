#include "garuda/physics/battery_model.hpp"
#include <cmath>
#include <algorithm>

namespace garuda {

BatteryModel::BatteryModel(const QuadrotorConfig& config) noexcept
    : _cfg(config) {
    reset();
}

void BatteryModel::reset() noexcept {
    _capacity_degradation = 1.0;
    _resistance_multiplier = 1.0;
    _state.soc = 1.0;
    _state.voltage_ocv = _cfg.max_voltage();
    _state.voltage_terminal = _cfg.max_voltage();
    _state.current_amps = 0.0;
    _state.temperature_c = _cfg.motor_temp_ambient_c;
    _state.capacity_used_ah = 0.0;
    _state.effective_capacity_ah = _cfg.battery_capacity_ah;
    _state.low_voltage_warning = false;
    _state.critical_cutoff = false;

    for (auto& cv : _state.cell_voltages) {
        cv = _cfg.battery_v_max_cell;
    }

    _state.ledger.initial_energy_joules = _cfg.nominal_energy_joules();
    _state.ledger.energy_remaining_joules = _cfg.nominal_energy_joules();
    _state.ledger.energy_consumed_joules = 0.0;
    _state.ledger.total_power_w = 0.0;
    _state.ledger.motor_power_w = 0.0;
    _state.ledger.avionics_power_w = 0.0;
}

double BatteryModel::_calculate_cell_ocv(double soc) const noexcept {
    soc = std::clamp(soc, 0.0, 1.0);
    // Smooth 5th-order empirical LiPo OCV-SoC characteristic curve
    // OCV(0.0) = 3.27V, OCV(0.5) = 3.82V, OCV(1.0) = 4.20V
    double s = soc;
    double s2 = s * s;
    double s3 = s2 * s;
    double s5 = s3 * s2;
    return 3.27 + 0.55 * s + 0.38 * s2 - 0.20 * s3 + 0.20 * s5;
}

double BatteryModel::step(double motor_power_w, double dt) noexcept {
    if (dt <= 0.0) return _state.voltage_terminal;

    const int n_cells = _cfg.battery_cells_series;
    const double total_electrical_power = motor_power_w + _cfg.avionics_power_draw_w;

    // 1. Calculate Open Circuit Voltage from current SoC
    double cell_ocv = _calculate_cell_ocv(_state.soc);
    double pack_ocv = cell_ocv * n_cells;
    _state.voltage_ocv = pack_ocv;

    // 2. Equivalent Circuit Load Solution:
    // P = V_term * I = (V_ocv - I * R_int) * I  =>  R_int * I^2 - V_ocv * I + P = 0
    double r_internal = _cfg.battery_internal_r_ohm * _resistance_multiplier;
    double current = 0.0;

    if (total_electrical_power > 1e-4 && r_internal > 1e-6) {
        double discr = pack_ocv * pack_ocv - 4.0 * r_internal * total_electrical_power;
        if (discr >= 0.0) {
            // Standard physical branch
            current = (pack_ocv - std::sqrt(discr)) / (2.0 * r_internal);
        } else {
            // Power demand exceeds maximum power transfer point (V_ocv^2 / 4*R_int)
            current = pack_ocv / (2.0 * r_internal);
        }
    } else if (pack_ocv > 1e-3) {
        current = total_electrical_power / pack_ocv;
    }

    current = std::max(0.0, current);
    _state.current_amps = current;

    // 3. Terminal Voltage with IR drop
    double v_term = pack_ocv - current * r_internal;
    v_term = std::max(0.0, v_term);
    _state.voltage_terminal = v_term;

    // Update individual cell voltages
    double current_cell_v = (n_cells > 0) ? (v_term / n_cells) : 0.0;
    for (auto& cv : _state.cell_voltages) {
        cv = current_cell_v;
    }

    // 4. Peukert Capacity Derating
    // C_eff = C_nom * (I_nom / I)^(k - 1)
    double nominal_c_rate_amps = _cfg.battery_capacity_ah; // 1C current
    double peukert_factor = 1.0;
    if (current > nominal_c_rate_amps) {
        double current_ratio = current / nominal_c_rate_amps;
        peukert_factor = std::pow(current_ratio, _cfg.battery_peukert_exponent - 1.0);
    }
    double effective_capacity = (_cfg.battery_capacity_ah * _capacity_degradation) / peukert_factor;
    _state.effective_capacity_ah = effective_capacity;

    // 5. Coulomb Counting Integration
    double amp_hours_used = (current * dt) / 3600.0;
    _state.capacity_used_ah += amp_hours_used;

    double d_soc = (amp_hours_used / std::max(effective_capacity, 0.1)) / _cfg.battery_discharge_efficiency;
    _state.soc = std::clamp(_state.soc - d_soc, 0.0, 1.0);

    // 6. Thermal Evolution: P_heat = I^2 * R_int
    double p_heat = current * current * r_internal;
    double temp_diff = _state.temperature_c - _cfg.motor_temp_ambient_c;
    double heat_loss = temp_diff / 4.5; // Thermal resistance pack-to-air (C/W)
    double d_temp = ((p_heat - heat_loss) / 180.0) * dt; // Heat capacity ~180 J/C
    _state.temperature_c += d_temp;

    // 7. Low-Voltage & Cutoff Warnings
    _state.low_voltage_warning = (current_cell_v < 3.50);
    _state.critical_cutoff = (_state.soc <= 0.0 || current_cell_v <= _cfg.battery_v_min_cell);

    // 8. Energy Accounting Ledger
    double actual_power_draw = v_term * current;
    double energy_step_joules = actual_power_draw * dt;

    _state.ledger.motor_power_w = motor_power_w;
    _state.ledger.avionics_power_w = _cfg.avionics_power_draw_w;
    _state.ledger.total_power_w = actual_power_draw;
    _state.ledger.energy_consumed_joules += energy_step_joules;
    _state.ledger.energy_remaining_joules = std::max(0.0, _state.ledger.initial_energy_joules - _state.ledger.energy_consumed_joules);

    return _state.voltage_terminal;
}

void BatteryModel::inject_battery_degradation(double capacity_factor, double resistance_multiplier) noexcept {
    _capacity_degradation = std::clamp(capacity_factor, 0.1, 1.0);
    _resistance_multiplier = std::max(1.0, resistance_multiplier);
}

} // namespace garuda
