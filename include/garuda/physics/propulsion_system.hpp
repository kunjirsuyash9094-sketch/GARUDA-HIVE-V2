#pragma once
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"
#include "aero/aero_effects.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include "garuda/physics/motor_system.hpp"
#include <vector>
#include <cmath>
#include <numbers>
#include <algorithm>

namespace garuda {

struct PropulsionOutput {
    Wrench wrench_body{};             // Total aerodynamic wrench (thrust + torque) in body frame
    double total_thrust_n{0.0};       // Total scalar thrust magnitude (N)
    double total_power_draw_w{0.0};   // Total mechanical power required (W)
    double ground_effect_factor{1.0}; // Multiplier from ground proximity
    bool   vrs_active{false};         // True if in Vortex Ring State
    double vrs_severity{0.0};         // VRS degradation severity [0.0 - 1.0]
};

class PropulsionSystem {
public:
    explicit PropulsionSystem(const QuadrotorConfig& config) noexcept
        : _cfg(config)
        , _ground_effect(config.rotor_radius_m) {
        _prev_inflow.resize(config.rotor_count, 0.0);
        _last_thrust.resize(config.rotor_count, 0.0);
    }

    void reset() noexcept {
        std::fill(_prev_inflow.begin(), _prev_inflow.end(), 0.0);
        std::fill(_last_thrust.begin(), _last_thrust.end(), 0.0);
    }

    [[nodiscard]] PropulsionOutput solve(
        MotorSystem& motor_sys,
        const Vec3d& body_vel_world,
        const Vec3d& body_omega_bf,
        const Quat&  body_orient,
        const dronesim::AtmosphericState& atm,
        const Vec3d& wind_world,
        double       ground_elevation_m,
        double       alt_world_m,
        double       dt
    ) noexcept {
        PropulsionOutput out{};
        auto& motors = motor_sys.mutable_motors();
        const size_t N = motors.size();
        if (_prev_inflow.size() != N) {
            _prev_inflow.resize(N, 0.0);
            _last_thrust.resize(N, 0.0);
        }

        const double R       = _cfg.rotor_radius_m;
        const double hub     = _cfg.hub_radius_m;
        const double rho     = atm.density;
        const int    n_annuli = _cfg.bet_annuli_count;
        const double dr      = (R - hub) / static_cast<double>(n_annuli);
        const double A_disk  = std::numbers::pi * R * R;
        const Vec3d  rotor_up_w = body_orient.rotate({0.0, 1.0, 0.0});

        Wrench aggregate_wrench{};
        double sum_thrust = 0.0;
        double sum_power = 0.0;

        for (size_t i = 0; i < N; ++i) {
            auto& m = motors[i];
            const double omega = m.omega;

            if (omega < 1.0) {
                m.thrust = 0.0;
                m.reaction_torque = 0.0;
                m.power_electrical = 0.0;
                _prev_inflow[i] = 0.0;
                _last_thrust[i] = 0.0;
                continue;
            }

            // Rotor hub linear velocity in world frame: V_hub = V_body + R * (omega_bf x pos_bf)
            Vec3d hub_pos_bf = m.position_bf;
            Vec3d hub_vel_w  = body_vel_world + body_orient.rotate(body_omega_bf.cross(hub_pos_bf));

            // Axial inflow component perpendicular to rotor disk
            double v_axial = -(hub_vel_w - wind_world).dot(rotor_up_w);

            // Iterative Momentum Inflow Solve (Rankine-Froude with damping)
            double vi = _prev_inflow[i];
            for (int iter = 0; iter < 3; ++iter) {
                double t_est = std::max(_last_thrust[i], 0.0);
                double vi_new = t_est / (2.0 * rho * A_disk * std::max(std::abs(v_axial + vi), 0.5));
                vi = vi + 0.5 * (vi_new - vi); // Damped relaxation
            }
            _prev_inflow[i] = vi;

            // Integrate 24 annular blade elements
            double rotor_thrust = 0.0;
            double rotor_torque = 0.0;
            double rotor_power  = 0.0;

            for (int j = 0; j < n_annuli; ++j) {
                const double r  = hub + (j + 0.5) * dr;
                const double Vt = omega * r;
                const double Va = v_axial + vi;
                const double V2 = Vt*Vt + Va*Va;
                if (V2 < 1e-6) continue;

                // Geometric twist linear interpolation
                const double t = (r - hub) / (R - hub);
                const double theta_rad = (_cfg.twist_root_deg + t * (_cfg.twist_tip_deg - _cfg.twist_root_deg)) * (std::numbers::pi / 180.0);

                // Effective AoA
                const double phi = std::atan2(Va, Vt);
                const double alpha = theta_rad - phi;

                // Aerodynamic coefficients
                const double Cl = _cfg.cl_alpha * alpha;
                const double Cd = _cfg.cd0 + _cfg.cd2 * Cl * Cl;

                // 2D elemental dynamic pressure and forces
                const double q_dyn = 0.5 * rho * V2;
                const double dL = q_dyn * _cfg.rotor_chord_m * Cl * dr * _cfg.blade_count;
                const double dD = q_dyn * _cfg.rotor_chord_m * Cd * dr * _cfg.blade_count;

                const double cos_phi = std::cos(phi);
                const double sin_phi = std::sin(phi);

                const double dT = dL * cos_phi - dD * sin_phi;
                const double dQ = (dL * sin_phi + dD * cos_phi) * r;

                rotor_thrust += dT;
                rotor_torque += dQ;
                rotor_power  += dQ * omega;
            }

            rotor_thrust = std::max(0.0, rotor_thrust);
            _last_thrust[i] = rotor_thrust;

            m.thrust = rotor_thrust;
            m.reaction_torque = rotor_torque;
            m.power_electrical = rotor_power / 0.85 + 2.0; // 85% mechanical/electrical efficiency + ESC idle

            sum_thrust += rotor_thrust;
            sum_power  += m.power_electrical;

            // Body-frame thrust vector (points in +y body axis)
            Vec3d f_rotor_bf{0.0, rotor_thrust, 0.0};

            // Moment arm torque: r x F
            Vec3d tau_arm = hub_pos_bf.cross(f_rotor_bf);

            // Reaction torque: opposes spin direction
            Vec3d tau_reaction = Vec3d{0.0, 1.0, 0.0} * (-static_cast<double>(m.spin_dir) * rotor_torque);

            // Gyroscopic precession torque: tau_gyro = omega_body x (0, I_rotor * Omega * spin_dir, 0)
            double gyro_factor = _cfg.motor_inertia_kgm2 * omega * static_cast<double>(m.spin_dir);
            Vec3d tau_gyro = body_omega_bf.cross({0.0, gyro_factor, 0.0});

            aggregate_wrench.force += f_rotor_bf;
            aggregate_wrench.torque += (tau_arm + tau_reaction + tau_gyro);
        }

        // Apply Ground Effect
        double h_agl = std::max(0.0, alt_world_m - ground_elevation_m);
        double ge_factor = _ground_effect.effective_multiplier(h_agl);
        if (_cfg.enable_ground_effect) {
            aggregate_wrench.force.y *= ge_factor;
            sum_thrust *= ge_factor;
        }

        // Apply Vortex Ring State
        double descent_rate  = -body_vel_world.y;
        double lateral_speed = std::sqrt(body_vel_world.x * body_vel_world.x + body_vel_world.z * body_vel_world.z);
        double vc_hover = std::sqrt(std::max(sum_thrust, 0.1) / (2.0 * rho * A_disk * static_cast<double>(N)));
        auto vrs = _vrs_model.evaluate(vc_hover, descent_rate, lateral_speed, dt);

        if (_cfg.enable_vrs && vrs.active) {
            aggregate_wrench.force.y *= vrs.thrust_factor;
            sum_thrust *= vrs.thrust_factor;
        }

        out.wrench_body = aggregate_wrench;
        out.total_thrust_n = sum_thrust;
        out.total_power_draw_w = sum_power;
        out.ground_effect_factor = ge_factor;
        out.vrs_active = vrs.active;
        out.vrs_severity = vrs.severity;

        return out;
    }

private:
    QuadrotorConfig             _cfg;
    dronesim::GroundEffectModel _ground_effect;
    dronesim::VortexRingStateModel _vrs_model;
    std::vector<double>         _prev_inflow;
    std::vector<double>         _last_thrust;
};

} // namespace garuda
