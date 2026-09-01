#pragma once
#include "core/math_types.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

struct ScalarPID {
    double kp{0.0}, ki{0.0}, kd{0.0};
    double integral_limit{50.0};
    double _integral{0.0};
    double _prev_error{0.0};
    bool   _first{true};

    void reset() noexcept {
        _integral = 0.0;
        _prev_error = 0.0;
        _first = true;
    }

    [[nodiscard]] double update(double error, double dt) noexcept {
        if (dt <= 0.0) return 0.0;
        _integral += error * dt;
        _integral = std::clamp(_integral, -integral_limit, integral_limit);

        double derivative = 0.0;
        if (!_first) {
            derivative = (error - _prev_error) / dt;
        }
        _prev_error = error;
        _first = false;

        return kp * error + ki * _integral + kd * derivative;
    }
};

struct FlightControlSetpoints {
    double roll_rad{0.0};       // Target roll angle (rad)
    double pitch_rad{0.0};      // Target pitch angle (rad)
    double yaw_rate_rads{0.0};  // Target yaw angular rate (rad/s)
    double thrust_norm{0.0};    // Target collective throttle [0.0 - 1.0]
};

struct FlightControlOutput {
    double tau_roll_dem{0.0};   // Normalized roll torque demand
    double tau_pitch_dem{0.0};  // Normalized pitch torque demand
    double tau_yaw_dem{0.0};    // Normalized yaw torque demand
    double thrust_norm{0.0};    // Normalized collective thrust
    std::vector<double> motor_throttles; // Per-motor throttle allocations [0.0 - 1.0]
};

class FlightControllerSystem {
public:
    explicit FlightControllerSystem(const QuadrotorConfig& config) noexcept
        : _cfg(config) {
        init_controllers();
    }

    void init_controllers() noexcept {
        _pid_roll.kp = _cfg.rate_roll_p;
        _pid_roll.ki = _cfg.rate_roll_i;
        _pid_roll.kd = _cfg.rate_roll_d;
        _pid_roll.integral_limit = _cfg.pid_integral_limit;

        _pid_pitch.kp = _cfg.rate_pitch_p;
        _pid_pitch.ki = _cfg.rate_pitch_i;
        _pid_pitch.kd = _cfg.rate_pitch_d;
        _pid_pitch.integral_limit = _cfg.pid_integral_limit;

        _pid_yaw.kp = _cfg.rate_yaw_p;
        _pid_yaw.ki = _cfg.rate_yaw_i;
        _pid_yaw.kd = _cfg.rate_yaw_d;
        _pid_yaw.integral_limit = _cfg.pid_integral_limit;

        // Authoritative 8-Motor Octo-X Mixer Matrix (8 rows x 4 cols: [thr, roll, pitch, yaw])
        // Rotors placed at psi_i = 22.5 deg + i * 45 deg
        // position_bf = { arm * cos(angle_rad), 0.0, -arm * sin(angle_rad) }
        _mix_matrix.clear();
        _mix_matrix.resize(_cfg.rotor_count);

        for (int i = 0; i < _cfg.rotor_count; ++i) {
            double angle_rad = (22.5 + static_cast<double>(i) * 45.0) * (std::numbers::pi / 180.0);
            double k_roll  =  std::sin(angle_rad);
            double k_pitch =  std::cos(angle_rad);
            double k_yaw   = (i % 2 == 0) ? -1.0 : +1.0;

            _mix_matrix[i] = { 1.0, k_roll, k_pitch, k_yaw };
        }
    }

    void reset() noexcept {
        _pid_roll.reset();
        _pid_pitch.reset();
        _pid_yaw.reset();
    }

    [[nodiscard]] FlightControlOutput update(
        const FlightControlSetpoints& sp,
        const Quat& current_orientation,
        const Vec3d& current_omega_bf,
        bool armed,
        double dt
    ) noexcept {
        FlightControlOutput out{};
        out.motor_throttles.assign(_cfg.rotor_count, 0.0);

        if (!armed) {
            reset();
            return out;
        }

        // 1. Attitude Representation & Error (Euler RPY extraction in Y-up frame)
        double qx = current_orientation.x;
        double qy = current_orientation.y;
        double qz = current_orientation.z;
        double qw = current_orientation.w;

        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll_cur = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (qw * qz - qx * qy);
        double pitch_cur = (std::abs(sinp) >= 1.0) ? std::copysign(std::numbers::pi / 2.0, sinp) : std::asin(sinp);

        // 2. Outer Attitude Loop (P-Controller -> Rate Setpoints)
        double roll_sp_clamped = std::clamp(sp.roll_rad, -_cfg.max_tilt_angle_rad, _cfg.max_tilt_angle_rad);
        double pitch_sp_clamped = std::clamp(sp.pitch_rad, -_cfg.max_tilt_angle_rad, _cfg.max_tilt_angle_rad);

        double roll_err = roll_sp_clamped - roll_cur;
        double pitch_err = pitch_sp_clamped - pitch_cur;

        double roll_rate_sp = std::clamp(_cfg.att_roll_p * roll_err, -_cfg.max_roll_pitch_rate_rad_s, _cfg.max_roll_pitch_rate_rad_s);
        double pitch_rate_sp = std::clamp(_cfg.att_pitch_p * pitch_err, -_cfg.max_roll_pitch_rate_rad_s, _cfg.max_roll_pitch_rate_rad_s);
        double yaw_rate_sp = std::clamp(sp.yaw_rate_rads, -_cfg.max_yaw_rate_rad_s, _cfg.max_yaw_rate_rad_s);

        // 3. Inner Angular Rate Loop (PID-Controller -> Torque Demands)
        double rate_roll_err = roll_rate_sp - current_omega_bf.x;
        double rate_pitch_err = pitch_rate_sp - current_omega_bf.z; // Pitch around z in body frame
        double rate_yaw_err = yaw_rate_sp - current_omega_bf.y;   // Yaw around y

        out.tau_roll_dem = _pid_roll.update(rate_roll_err, dt);
        out.tau_pitch_dem = _pid_pitch.update(rate_pitch_err, dt);
        out.tau_yaw_dem = _pid_yaw.update(rate_yaw_err, dt);
        out.thrust_norm = std::clamp(sp.thrust_norm, 0.0, 1.0);

        // 4. 8-Rotor Mixer Matrix Allocation
        for (size_t i = 0; i < _mix_matrix.size() && i < out.motor_throttles.size(); ++i) {
            double alloc = _mix_matrix[i][0] * out.thrust_norm
                         + _mix_matrix[i][1] * out.tau_roll_dem
                         + _mix_matrix[i][2] * out.tau_pitch_dem
                         + _mix_matrix[i][3] * out.tau_yaw_dem;

            // Apply idle spin floor when armed
            if (armed && out.thrust_norm > 0.01) {
                alloc = std::max(alloc, _cfg.motor_idle_throttle);
            }
            out.motor_throttles[i] = std::clamp(alloc, 0.0, 1.0);
        }

        return out;
    }

private:
    const QuadrotorConfig& _cfg;
    ScalarPID _pid_roll{};
    ScalarPID _pid_pitch{};
    ScalarPID _pid_yaw{};
    std::vector<std::array<double, 4>> _mix_matrix;
};

} // namespace garuda
