#pragma once
#include <string>
#include <array>
#include <cmath>
#include <numbers>
#include "core/math_types.hpp"

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

/**
 * @brief Centralized, immutable physical vehicle configuration for GARUDA Heavy-Lift Octocopter.
 * 
 * Canonical Vehicle: GARUDA-HL-01
 * Configuration: 8-Rotor Heavy-Lift Infrastructure Inspection Platform
 * Scale: SI Units (m, kg, s, N, N*m, rad, rad/s, J, V, A)
 */
struct QuadrotorConfig {
    // -------------------------------------------------------------------------
    // Structural & Mass Properties (GARUDA-HL-01 Canonical Heavy-Lift)
    // -------------------------------------------------------------------------
    std::string model_name{"GARUDA-HL-01"};
    double dry_mass_kg{8.50};                // Base vehicle dry mass without payload (kg)
    double mass_kg{10.00};                   // Nominal operating mass with default inspection camera (kg)
    double max_takeoff_mass_kg{15.00};       // Maximum Takeoff Weight (MTOW) (kg)
    double payload_capacity_kg{6.50};        // Maximum structural payload capacity (kg)
    
    // Diagonal principal inertia for heavy-lift 1100mm octocopter (kg*m^2)
    Vec3d  inertia_diag_kgm2{0.185, 0.185, 0.320}; 
    double arm_length_m{0.550};              // Distance from CoM to rotor center (0.55m -> 1100mm diameter)
    double ground_contact_radius_m{0.280};   // Landing skid height clearance (m)

    // -------------------------------------------------------------------------
    // Rotor & Aerodynamic Properties (Blade Element Theory)
    // -------------------------------------------------------------------------
    int    rotor_count{8};                   // 8 Rotors in Octo-X radial layout
    double rotor_radius_m{0.1905};           // 15-inch propeller tip radius (15 * 0.0254 / 2 = 0.1905m)
    double rotor_chord_m{0.028};             // Mean aerodynamic chord (m)
    int    blade_count{2};                   // Number of blades per rotor
    double twist_root_deg{16.0};             // Blade geometric twist at hub (deg)
    double twist_tip_deg{7.0};               // Blade geometric twist at tip (deg)
    double hub_radius_m{0.025};              // Cutout radius at hub (m)
    double cl_alpha{5.85};                   // 2D lift curve slope per radian
    double cd0{0.014};                       // Profile drag coefficient at zero lift
    double cd2{0.075};                       // Induced drag factor (CDi = cd2 * CL^2)
    int    bet_annuli_count{24};             // Number of radial annuli for numerical BET integration

    // -------------------------------------------------------------------------
    // Motor & ESC Dynamics (Industrial High-Torque Brushless)
    // -------------------------------------------------------------------------
    double motor_kv{380.0};                  // Heavy-lift motor velocity constant (RPM/V)
    double motor_resistance_ohm{0.045};      // Low phase resistance for high continuous current (Ohms)
    double motor_inertia_kgm2{6.5e-5};       // Rotor bell + 15" carbon prop rotational inertia (kg*m^2)
    double esc_time_constant_s{0.015};       // First-order ESC lag tau (s)
    double motor_idle_throttle{0.04};        // Idle spin throttle when armed [0.0 - 0.1]
    double motor_temp_ambient_c{25.0};       // Ambient temperature (deg C)
    double motor_thermal_resistance_cw{1.2}; // Thermal resistance to ambient (C/W)
    double motor_thermal_capacity_jc{120.0}; // Thermal capacity (J/C)

    // -------------------------------------------------------------------------
    // Battery & Energy Ledger Properties (6S2P Industrial LiPo Pack)
    // -------------------------------------------------------------------------
    int    battery_cells_series{6};          // 6S LiPo configuration (22.2V nominal, 25.2V max)
    double battery_capacity_ah{16.0};        // 16,000 mAh (16 Ah) high-capacity pack
    double battery_v_max_cell{4.20};         // Max fully-charged cell voltage (V) -> 25.2V pack
    double battery_v_nominal_cell{3.70};     // Nominal cell voltage (V) -> 22.2V pack
    double battery_v_min_cell{3.27};         // Cutoff cell voltage at 0% SoC (V) -> 19.62V pack
    double battery_internal_r_ohm{0.012};    // Low internal resistance pack (Ohms)
    double battery_peukert_exponent{1.04};   // Peukert capacity loss factor
    double battery_discharge_efficiency{0.97}; // Coulombic efficiency
    double avionics_power_draw_w{12.0};      // Avionics, GNSS, companion computer baseline draw (W)

    // -------------------------------------------------------------------------
    // Aerodynamic Airframe Drag & Interference
    // -------------------------------------------------------------------------
    double body_drag_linear_kg_s{0.06};      // Skin friction drag factor (kg/s)
    double body_drag_quad_kg_m{0.45};        // Quadratic form drag factor for octocopter frame (kg/m)
    bool   enable_ground_effect{true};       // Enable Cheeseman-Bennett In-Ground-Effect
    bool   enable_vrs{true};                 // Enable Leishman Vortex Ring State model

    // -------------------------------------------------------------------------
    // Flight Controller Limits & Gains (Tuned for Heavy-Lift Octocopter)
    // -------------------------------------------------------------------------
    double max_tilt_angle_rad{0.4363};       // Max pitch/roll angle (25 deg = 0.4363 rad)
    double max_yaw_rate_rad_s{1.5708};       // Max yaw angular rate (90 deg/s = 1.5708 rad/s)
    double max_roll_pitch_rate_rad_s{3.1416};// Max body roll/pitch rate (180 deg/s)
    double att_roll_p{6.5};                  // Outer attitude loop P gain (1/s)
    double att_pitch_p{6.5};
    double rate_roll_p{0.25}, rate_roll_i{0.08}, rate_roll_d{0.006}; // Inner rate PID
    double rate_pitch_p{0.25}, rate_pitch_i{0.08}, rate_pitch_d{0.006};
    double rate_yaw_p{0.35}, rate_yaw_i{0.12}, rate_yaw_d{0.000};
    double pid_integral_limit{40.0};         // Anti-windup clamping threshold

    // -------------------------------------------------------------------------
    // Ground Interaction & Contact Physics (Heavy-Lift Landing Gear)
    // -------------------------------------------------------------------------
    double ground_spring_k{12000.0};         // Ground normal spring stiffness (N/m)
    double ground_damper_d{850.0};           // Ground normal damping (N*s/m)
    double ground_friction_coeff{0.70};      // Coulomb friction coefficient
    double ground_restitution{0.10};         // Coefficient of restitution (damped landing)

    // -------------------------------------------------------------------------
    // Sensor Suite Noise & Bias Configuration (Industrial Sensors)
    // -------------------------------------------------------------------------
    double accel_bias_dc{0.02};              // Initial accelerometer DC bias (m/s^2)
    double gyro_bias_dc{0.005};              // Initial gyroscope DC bias (rad/s)
    double gyro_bias_drift_rate{1e-6};       // Gyro bias random walk drift rate
    double accel_noise_density{0.005};       // Accel white noise density (m/s^2/sqrt(Hz))
    double gyro_noise_density{0.001};        // Gyro white noise density (rad/s/sqrt(Hz))
    double baro_filter_tau_s{0.05};          // Barometer acoustic lag filter time constant (s)
    double baro_noise_pa{1.2};               // Barometer pressure noise (Pa)
    double gps_update_rate_hz{10.0};         // GPS update rate limiter (Hz)
    double gps_pos_noise_h_m{0.25};          // GPS horizontal position noise 1-sigma (m)
    double gps_pos_noise_v_m{0.50};          // GPS vertical position noise 1-sigma (m)
    double gps_vel_noise_m_s{0.05};          // GPS velocity noise 1-sigma (m/s)
    double mag_noise_rad{0.008};             // Magnetometer heading noise (rad)

    // Helper functions
    [[nodiscard]] constexpr double max_voltage() const noexcept {
        return battery_cells_series * battery_v_max_cell;
    }
    [[nodiscard]] constexpr double nominal_voltage() const noexcept {
        return battery_cells_series * battery_v_nominal_cell;
    }
    [[nodiscard]] constexpr double cutoff_voltage() const noexcept {
        return battery_cells_series * battery_v_min_cell;
    }
    [[nodiscard]] constexpr double nominal_energy_joules() const noexcept {
        return battery_capacity_ah * nominal_voltage() * 3600.0;
    }
};

} // namespace garuda
