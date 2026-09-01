#pragma once
#include "core/math_types.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <numbers>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

enum class MotorHealthState {
    NORMAL = 0,
    DEGRADED = 1,
    FAILED = 2
};

struct MotorDynamicState {
    double omega{0.0};              // Current shaft angular velocity (rad/s)
    double omega_cmd{0.0};          // Commanded angular velocity (rad/s)
    double rpm{0.0};                // Current RPM
    double thrust{0.0};             // Individual thrust (N)
    double torque{0.0};             // Individual torque (N*m)
    double reaction_torque{0.0};    // Aerodynamic reaction torque (N*m)
    double current{0.0};            // Electrical current draw (A)
    double power_electrical{0.0};   // Electrical power consumed (W)
    double temperature_c{25.0};     // Coil temperature (°C)
    MotorHealthState health{MotorHealthState::NORMAL};
    double degradation_factor{1.0}; // Multiplier when degraded
    int spin_dir{1};                // +1 = CCW, -1 = CW
    Vec3d position_bf{0.0, 0.0, 0.0}; // Position in body frame (m)
};

class MotorSystem {
public:
    explicit MotorSystem(const QuadrotorConfig& config) noexcept
        : _cfg(config) {
        _motors.resize(_cfg.rotor_count);
        init_positions();
        reset();
    }

    void init_positions() noexcept {
        // Octo-X heavy-lift arm layout: psi_i = 22.5 deg + i * 45 deg
        // position_bf = { arm * cos(angle), 0.0, -arm * sin(angle) }
        for (int i = 0; i < _cfg.rotor_count; ++i) {
            double angle_rad = (22.5 + static_cast<double>(i) * 45.0) * (std::numbers::pi / 180.0);
            _motors[i].position_bf = { _cfg.arm_length_m * std::cos(angle_rad), 0.0, -_cfg.arm_length_m * std::sin(angle_rad) };
            _motors[i].spin_dir = (i % 2 == 0) ? 1 : -1; // Alternating CCW / CW
        }
    }

    void reset() noexcept {
        for (size_t i = 0; i < _motors.size(); ++i) {
            _motors[i].omega = 0.0;
            _motors[i].omega_cmd = 0.0;
            _motors[i].rpm = 0.0;
            _motors[i].thrust = 0.0;
            _motors[i].torque = 0.0;
            _motors[i].reaction_torque = 0.0;
            _motors[i].current = 0.0;
            _motors[i].power_electrical = 0.0;
            _motors[i].temperature_c = _cfg.motor_temp_ambient_c;
            _motors[i].health = MotorHealthState::NORMAL;
            _motors[i].degradation_factor = 1.0;
        }
    }

    void set_commands(const std::vector<double>& throttle_inputs, double supply_voltage) noexcept {
        const double kv_rad_s_v = _cfg.motor_kv * (2.0 * std::numbers::pi / 60.0);
        const double max_rad_s = kv_rad_s_v * supply_voltage;

        for (size_t i = 0; i < _motors.size(); ++i) {
            double u = (i < throttle_inputs.size()) ? throttle_inputs[i] : 0.0;
            u = std::clamp(u, 0.0, 1.0);

            if (_motors[i].health == MotorHealthState::FAILED) {
                _motors[i].omega_cmd = 0.0;
            } else if (_motors[i].health == MotorHealthState::DEGRADED) {
                _motors[i].omega_cmd = u * max_rad_s * _motors[i].degradation_factor;
            } else {
                _motors[i].omega_cmd = u * max_rad_s;
            }
        }
    }

    void step_dynamics(double dt) noexcept {
        if (dt <= 0.0) return;

        // Exact 1st-order ESC lag integration: alpha = 1 - exp(-dt / tau)
        const double alpha = 1.0 - std::exp(-dt / std::max(1e-5, _cfg.esc_time_constant_s));

        for (auto& m : _motors) {
            if (m.health == MotorHealthState::FAILED) {
                m.omega_cmd = 0.0;
            }

            m.omega += alpha * (m.omega_cmd - m.omega);
            m.rpm = m.omega * (60.0 / (2.0 * std::numbers::pi));

            // Thermal Model: Electrical heating + ambient convective cooling
            double heat_gen = 0.15 * m.power_electrical * dt; // ~85% nominal electrical efficiency
            double heat_diss = (1.0 / std::max(0.1, _cfg.motor_thermal_resistance_cw)) * (m.temperature_c - _cfg.motor_temp_ambient_c) * dt;
            double delta_t = (heat_gen - heat_diss) / std::max(1.0, _cfg.motor_thermal_capacity_jc);
            m.temperature_c += delta_t;
        }
    }

    void set_motor_health(size_t index, MotorHealthState state, double degradation = 0.5) noexcept {
        if (index < _motors.size()) {
            _motors[index].health = state;
            _motors[index].degradation_factor = std::clamp(degradation, 0.0, 1.0);
            if (state == MotorHealthState::FAILED) {
                _motors[index].omega_cmd = 0.0;
            }
        }
    }

    [[nodiscard]] double total_electrical_power() const noexcept {
        double sum = _cfg.avionics_power_draw_w;
        for (const auto& m : _motors) {
            sum += m.power_electrical;
        }
        return sum;
    }

    [[nodiscard]] size_t size() const noexcept { return _motors.size(); }
    [[nodiscard]] size_t motor_count() const noexcept { return _motors.size(); }
    [[nodiscard]] const std::vector<MotorDynamicState>& motors() const noexcept { return _motors; }
    [[nodiscard]] std::vector<MotorDynamicState>& mutable_motors() noexcept { return _motors; }

private:
    QuadrotorConfig _cfg;
    std::vector<MotorDynamicState> _motors;
};

} // namespace garuda
