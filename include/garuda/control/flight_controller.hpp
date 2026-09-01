#pragma once
#include "core/math_types.hpp"
#include "core/frames.hpp"
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
    double roll_rad{0.0};       // Target roll angle in FRD frame (rad)
    double pitch_rad{0.0};      // Target pitch angle in FRD frame (rad)
    double yaw_rate_rads{0.0};  // Target yaw angular rate in FRD frame (rad/s)
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
    }

    void reset() noexcept {
        _pid_roll.reset();
        _pid_pitch.reset();
        _pid_yaw.reset();
    }

    [[nodiscard]] FlightControlOutput update(
        const FlightControlSetpoints& sp,
        const Quat& current_orientation_godot,
        const Vec3d& current_omega_bf_godot,
        bool armed,
        double dt
    ) noexcept {
        FlightControlOutput out{};
        out.motor_throttles.assign(_cfg.rotor_count, 0.0);

        if (!armed) {
            reset();
            return out;
        }

        // 1. Convert to Standard Aerospace FRD Body Frame
        Quat q_frd = dronesim::frames::godot_to_ned(current_orientation_godot);
        Vec3d w_frd = dronesim::frames::godot_to_ned(current_omega_bf_godot);
        Vec3d rpy = q_frd.to_euler_rpy(); // [roll, pitch, yaw] in radians

        // 2. Outer Attitude Loop (P-Controller -> Angular Rate Setpoints)
        double roll_sp_clamped = std::clamp(sp.roll_rad, -_cfg.max_tilt_angle_rad, _cfg.max_tilt_angle_rad);
        double pitch_sp_clamped = std::clamp(sp.pitch_rad, -_cfg.max_tilt_angle_rad, _cfg.max_tilt_angle_rad);

        double roll_err = roll_sp_clamped - rpy.x;
        double pitch_err = pitch_sp_clamped - rpy.y;

        double roll_rate_sp = std::clamp(_cfg.att_roll_p * roll_err, -_cfg.max_roll_pitch_rate_rad_s, _cfg.max_roll_pitch_rate_rad_s);
        double pitch_rate_sp = std::clamp(_cfg.att_pitch_p * pitch_err, -_cfg.max_roll_pitch_rate_rad_s, _cfg.max_roll_pitch_rate_rad_s);
        double yaw_rate_sp = std::clamp(sp.yaw_rate_rads, -_cfg.max_yaw_rate_rad_s, _cfg.max_yaw_rate_rad_s);

        // 3. Inner Angular Rate Loop (PID-Controller -> Normalized Moment Demands)
        double rate_roll_err = roll_rate_sp - w_frd.x;
        double rate_pitch_err = pitch_rate_sp - w_frd.y;
        double rate_yaw_err = yaw_rate_sp - w_frd.z;

        out.tau_roll_dem = std::clamp(_pid_roll.update(rate_roll_err, dt), -0.25, 0.25);
        out.tau_pitch_dem = std::clamp(_pid_pitch.update(rate_pitch_err, dt), -0.25, 0.25);
        out.tau_yaw_dem = std::clamp(_pid_yaw.update(rate_yaw_err, dt), -0.20, 0.20);
        out.thrust_norm = std::clamp(sp.thrust_norm, 0.0, 1.0);

        // 4. 8-Rotor Octo-X Mixer Allocation in FRD Body Frame
        for (int i = 0; i < _cfg.rotor_count; ++i) {
            double angle_rad = (22.5 + static_cast<double>(i) * 45.0) * (std::numbers::pi / 180.0);
            double k_roll  = -std::cos(angle_rad);
            double k_pitch =  std::sin(angle_rad);
            double k_yaw   = (i % 2 == 0) ? 1.0 : -1.0;

            double alloc = out.thrust_norm
                         + k_roll * out.tau_roll_dem
                         + k_pitch * out.tau_pitch_dem
                         + k_yaw * out.tau_yaw_dem;

            if (armed) {
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
};

} // namespace garuda
