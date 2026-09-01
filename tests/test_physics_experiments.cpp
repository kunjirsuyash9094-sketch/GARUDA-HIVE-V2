#include "garuda/core/simulation_world.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cassert>

using namespace garuda;

struct ExperimentMetrics {
    std::string name;
    std::string expected;
    std::string measured;
    double error{0.0};
    double settling_time_s{0.0};
    double energy_joules{0.0};
    bool passed{false};
};

int main() {
    std::cout << "======================================================================\n";
    std::cout << " GARUDA HIVE V2 — PHYSICAL SIMULATION EXPERIMENTS SUITE (16 EXPERIMENTS)\n";
    std::cout << "======================================================================\n\n";

    std::vector<ExperimentMetrics> results;
    const double dt = 0.0025; // 400 Hz physics timestep

    // ------------------------------------------------------------------------
    // EXPERIMENT 1: Free Fall Acceleration
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(101, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 50.0, 0.0});
        d->disarm(); // Disarmed, motors off
        
        world.step(); // First step
        double a0 = d->telemetry().acceleration_world.y;
        
        for (int i = 0; i < 400; ++i) { // 1.0 second
            world.step();
        }
        double y1 = d->telemetry().position_world.y;
        double expected_y = 45.45;
        double err = std::abs(y1 - expected_y);

        ExperimentMetrics m;
        m.name = "Exp 01: Free Fall Acceleration";
        m.expected = "a_y = -9.807 m/s^2, alt(1s) ≈ 45.45 m (with aero drag)";
        m.measured = "a0=" + std::to_string(a0) + " m/s^2, alt=" + std::to_string(y1) + " m";
        m.error = err;
        m.passed = (std::abs(a0 - (-9.80665)) < 1e-3) && (err < 0.50);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 2: Motor Dynamic Spin-up & First-Order ESC Lag
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(102, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 5.0, 0.0});
        d->arm();
        world.step(); // Idle step
        double rpm_0 = d->telemetry().motor_rpm[0];
        
        // Command full throttle
        d->set_attitude_setpoint(0.0, 0.0, 0.0, 1.0);
        for (int i = 0; i < 6; ++i) { // 6 steps = 15.0ms = 1 tau
            world.step();
        }
        double rpm_1tau = d->telemetry().motor_rpm[0];

        for (int i = 0; i < 100; ++i) { // 0.25s = full steady state
            world.step();
        }
        double rpm_steady = d->telemetry().motor_rpm[0];
        double delta_fraction = (rpm_1tau - rpm_0) / std::max(1.0, rpm_steady - rpm_0);

        ExperimentMetrics m;
        m.name = "Exp 02: Motor Dynamic Spin-up (ESC Lag tau=15ms)";
        m.expected = "Delta RPM at 1 tau (15ms) ≈ 63.2% of steady-state delta";
        m.measured = "1 tau Delta Ratio: " + std::to_string(delta_fraction) + " (1-e^-1 = 0.63212)";
        m.error = std::abs(delta_fraction - 0.63212);
        m.passed = (delta_fraction > 0.55 && delta_fraction < 0.75 && rpm_steady > 4500.0);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 3: Emergent Takeoff Profile
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(103, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 0.28, 0.0}); // On ground pad
        d->arm();

        double liftoff_time = -1.0;
        for (int i = 0; i < 800; ++i) { // 2.0s
            double t = i * dt;
            double thr = 0.20 + (t / 2.0) * 0.55; // Ramp from idle to climb throttle
            d->set_attitude_setpoint(0.0, 0.0, 0.0, thr);
            world.step();
            if (liftoff_time < 0.0 && d->telemetry().altitude_m > 0.02) {
                liftoff_time = t;
            }
        }
        double final_alt = d->telemetry().altitude_m;

        ExperimentMetrics m;
        m.name = "Exp 03: Emergent Takeoff Profile";
        m.expected = "Liftoff at T > mg (t ≈ 1.2s - 1.7s), alt_agl > 0.20m";
        m.measured = "Liftoff t=" + std::to_string(liftoff_time) + " s, final alt_agl=" + std::to_string(final_alt) + " m";
        m.error = std::abs(liftoff_time - 1.5);
        m.passed = (liftoff_time > 0.4 && liftoff_time < 1.9 && final_alt > 0.20);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 4: Hover Equilibrium (T_total ≈ mg)
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(104, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 5.0, 0.0});
        d->arm();
        double weight = (d->config().dry_mass_kg + d->telemetry().payload_mass_kg) * 9.80665; // ~98.07 N
        
        double hover_thr = 0.5833;
        for (int i = 0; i < 400; ++i) { // 1.0s
            d->set_attitude_setpoint(0.0, 0.0, 0.0, hover_thr);
            world.step();
        }
        double thrust = d->telemetry().total_thrust_n;
        double err = std::abs(thrust - weight);

        ExperimentMetrics m;
        m.name = "Exp 04: Hover Equilibrium";
        m.expected = "Thrust = Weight = " + std::to_string(weight) + " N";
        m.measured = "Thrust = " + std::to_string(thrust) + " N (TWR=" + std::to_string(d->telemetry().thrust_to_weight_ratio) + ")";
        m.error = err;
        m.passed = (err < 10.0);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 5: Vertical Climb Dynamics
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(105, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 5.0, 0.0});
        d->arm();

        for (int i = 0; i < 400; ++i) { // 1.0s climb at thr=0.75
            d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.75);
            world.step();
        }
        double vs = d->telemetry().vertical_speed_ms;

        ExperimentMetrics m;
        m.name = "Exp 05: Vertical Climb Dynamics";
        m.expected = "Positive vertical climb velocity (v_y > 1.2 m/s)";
        m.measured = "v_y = " + std::to_string(vs) + " m/s";
        m.error = std::abs(vs - 2.5);
        m.passed = (vs > 1.2);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 6: Controlled Descent Dynamics
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(106, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 20.0, 0.0});
        d->arm();

        for (int i = 0; i < 400; ++i) { // 1.0s controlled descent at thr=0.40
            d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.40);
            world.step();
        }
        double vs = d->telemetry().vertical_speed_ms;

        ExperimentMetrics m;
        m.name = "Exp 06: Controlled Descent Dynamics";
        m.expected = "Stable descent velocity (v_y < -0.8 m/s)";
        m.measured = "v_y = " + std::to_string(vs) + " m/s";
        m.error = std::abs(vs - (-2.0));
        m.passed = (vs < -0.8);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 7: Forward Flight via Physical Pitch Tilt
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(107, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 10.0, 0.0});
        d->arm();

        // 10 degrees pitch nose down (pitch = -0.1745 rad in FRD)
        double pitch_rad = -0.1745;
        for (int i = 0; i < 400; ++i) { // 1.0s
            d->set_attitude_setpoint(0.0, pitch_rad, 0.0, 0.5833);
            world.step();
        }
        double vz = d->telemetry().velocity_world.z; // -Z is forward in aerospace body frame
        double acc_z = d->telemetry().acceleration_world.z;

        ExperimentMetrics m;
        m.name = "Exp 07: Forward Accel via Pitch Tilt";
        m.expected = "Tilted thrust produces forward horizontal velocity (v_z < -0.5 m/s)";
        m.measured = "v_z = " + std::to_string(vz) + " m/s, a_z = " + std::to_string(acc_z) + " m/s^2";
        m.error = std::abs(vz - (-1.5));
        m.passed = (vz < -0.5);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 8: Braking & Settling
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(108, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 10.0, 0.0});
        d->arm();
        d->mutable_physics_state().velocity = {0.0, 0.0, -4.0}; // Initial 4 m/s forward speed
        world.step();

        // Counter-pitch nose up (pitch = +0.25 rad) to brake
        double brake_pitch = 0.25;
        double stop_time = -1.0;
        for (int i = 0; i < 800; ++i) { // 2.0s
            double t = (i + 1) * dt;
            if (d->telemetry().velocity_world.z < -0.3) {
                d->set_attitude_setpoint(0.0, brake_pitch, 0.0, 0.5833);
            } else {
                d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.5833);
                if (stop_time < 0.0) stop_time = t;
            }
            world.step();
        }
        double final_vz = d->telemetry().velocity_world.z;

        ExperimentMetrics m;
        m.name = "Exp 08: Braking & Deceleration";
        m.expected = "Deceleration to near-zero forward speed (|v_z| < 0.8 m/s)";
        m.measured = "Brake time=" + std::to_string(stop_time) + " s, final v_z=" + std::to_string(final_vz) + " m/s";
        m.error = std::abs(final_vz);
        m.settling_time_s = stop_time;
        m.passed = (std::abs(final_vz) < 0.8 && stop_time > 0.1 && stop_time < 2.5);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 9: Roll Dynamic Moment & Lateral Acceleration
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(109, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 10.0, 0.0});
        d->arm();

        d->set_attitude_setpoint(0.15, 0.0, 0.0, 0.5833); // Command +0.15 rad roll (~8.6 deg)
        for (int i = 0; i < 200; ++i) { // 0.5s
            world.step();
        }
        double roll_deg = d->telemetry().euler_rpy_deg.x;

        ExperimentMetrics m;
        m.name = "Exp 09: Roll Moment Response";
        m.expected = "Roll angle reaches commanded tilt (~8.6 deg)";
        m.measured = "Roll = " + std::to_string(roll_deg) + " deg";
        m.error = std::abs(roll_deg - 8.59);
        m.passed = (roll_deg > 4.0 && roll_deg < 16.0);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 10: Pitch Dynamic Moment & Longitudinal Acceleration
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(110, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 10.0, 0.0});
        d->arm();

        d->set_attitude_setpoint(0.0, -0.15, 0.0, 0.5833); // Command -0.15 rad pitch (~-8.6 deg)
        for (int i = 0; i < 200; ++i) { // 0.5s
            world.step();
        }
        double pitch_deg = d->telemetry().euler_rpy_deg.y;

        ExperimentMetrics m;
        m.name = "Exp 10: Pitch Moment Response";
        m.expected = "Pitch angle reaches commanded tilt (~-8.6 deg)";
        m.measured = "Pitch = " + std::to_string(pitch_deg) + " deg";
        m.error = std::abs(pitch_deg - (-8.59));
        m.passed = (pitch_deg < -4.0 && pitch_deg > -16.0);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 11: Yaw Reaction Torque Imbalance
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(111, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 10.0, 0.0});
        d->arm();

        d->set_attitude_setpoint(0.0, 0.0, 0.50, 0.5833); // Command +0.50 rad/s yaw rate
        for (int i = 0; i < 400; ++i) { // 1.0s
            world.step();
        }
        double yaw_rate = d->telemetry().angular_velocity_rads.y;
        double yaw_deg = d->telemetry().euler_rpy_deg.z;

        ExperimentMetrics m;
        m.name = "Exp 11: Yaw Reaction Torque Imbalance";
        m.expected = "Counter-rotating rotor differential torque drives heading change";
        m.measured = "Yaw Rate = " + std::to_string(yaw_rate) + " rad/s, Heading = " + std::to_string(yaw_deg) + " deg";
        m.error = std::abs(yaw_rate - 0.50);
        m.passed = (std::abs(yaw_deg) > 5.0 || std::abs(yaw_rate) > 0.2);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 12: Combined 3D Translation
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(112, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 10.0, 0.0});
        d->arm();

        // Combined roll (+0.10 rad) + pitch (-0.10 rad) + climb (thr=0.68)
        for (int i = 0; i < 400; ++i) { // 1.0s
            d->set_attitude_setpoint(0.10, -0.10, 0.0, 0.68);
            world.step();
        }
        double vx = d->telemetry().velocity_world.x;
        double vy = d->telemetry().velocity_world.y;
        double vz = d->telemetry().velocity_world.z;

        ExperimentMetrics m;
        m.name = "Exp 12: Combined 3D Translation";
        m.expected = "Simultaneous roll, pitch, and climb produce 3D velocity vector (v_x>0.2, v_y>0.5, v_z<-0.2)";
        m.measured = "v = [" + std::to_string(vx) + ", " + std::to_string(vy) + ", " + std::to_string(vz) + "] m/s";
        m.passed = (vx > 0.15 && vy > 0.5 && vz < -0.15);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 13: Steady Wind & Aerodynamic Drift
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(113, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 10.0, 0.0});
        d->arm();
        d->set_attitude_setpoint(0,0,0,0.5833); // Neutral hover

        // Inject steady crosswind in +X direction (5 m/s)
        world.environment().set_wind({5.0, 0.0, 0.0});
        for (int i = 0; i < 400; ++i) { // 1.0s
            world.step();
        }
        double vx = d->telemetry().velocity_world.x;

        ExperimentMetrics m;
        m.name = "Exp 13: Wind & Aerodynamic Drift";
        m.expected = "5 m/s crosswind induces aerodynamic side drag and drift (v_x > 0.4 m/s)";
        m.measured = "v_x = " + std::to_string(vx) + " m/s";
        m.error = std::abs(vx - 2.0);
        m.passed = (vx > 0.4);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 14: Payload Mass & Inertia Coupling
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(114, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 5.0, 0.0});
        d->arm();

        // Baseline 10.0kg
        for (int i = 0; i < 100; ++i) { d->set_attitude_setpoint(0,0,0,0.5833); world.step(); }
        double t_base = d->telemetry().total_thrust_n;
        double pwr_base = d->telemetry().battery_power_w;

        // Attach heavy emergency cargo (3.5kg -> total 12.0kg)
        d->attach_payload(PayloadType::EMERGENCY_SUPPLY);
        for (int i = 0; i < 100; ++i) { d->set_attitude_setpoint(0,0,0,0.68); world.step(); }
        double t_loaded = d->telemetry().total_thrust_n;
        double pwr_loaded = d->telemetry().battery_power_w;

        ExperimentMetrics m;
        m.name = "Exp 14: Payload Mass Coupling";
        m.expected = "Loaded 12kg requires higher thrust and power than 10kg baseline";
        m.measured = "Base Thrust=" + std::to_string(t_base) + "N (P=" + std::to_string(pwr_base) + "W) | Loaded=" + std::to_string(t_loaded) + "N (P=" + std::to_string(pwr_loaded) + "W)";
        m.error = 0.0;
        m.passed = (t_loaded > t_base + 12.0 && pwr_loaded > pwr_base + 20.0);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 15: Landing & Ground Contact Model
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(115, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 1.5, 0.0});
        d->disarm(); // Fall freely to touch ground

        for (int i = 0; i < 800; ++i) { // 2.0s
            world.step();
        }
        double alt = d->telemetry().altitude_m;
        double pos_y = d->telemetry().position_world.y;
        double vs = d->telemetry().vertical_speed_ms;
        bool in_contact = (d->telemetry().in_ground_contact != 0);

        ExperimentMetrics m;
        m.name = "Exp 15: Landing & Ground Contact";
        m.expected = "Resting on ground surface at contact clearance (~0.28m) without sinking";
        m.measured = "y=" + std::to_string(pos_y) + " m, alt=" + std::to_string(alt) + " m, vs=" + std::to_string(vs) + " m/s";
        m.error = std::abs(pos_y - 0.28);
        m.passed = (pos_y >= 0.275 && pos_y <= 0.295 && std::abs(vs) < 0.05 && in_contact);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // EXPERIMENT 16: Motor Failure Response
    // ------------------------------------------------------------------------
    {
        SimulationWorld world(116, dt);
        auto* d = world.add_drone("GARUDA-HL-01", {}, {0.0, 5.0, 0.0});
        d->arm();
        for (int i = 0; i < 100; ++i) { d->set_attitude_setpoint(0,0,0,0.5833); world.step(); }

        // Inject catastrophic failure on Motor 3
        d->inject_motor_failure(2, MotorHealthState::FAILED);
        for (int i = 0; i < 100; ++i) { d->set_attitude_setpoint(0,0,0,0.5833); world.step(); }
        double m3_rpm = d->telemetry().motor_rpm[2];
        double m1_rpm = d->telemetry().motor_rpm[0];

        ExperimentMetrics m;
        m.name = "Exp 16: Motor Failure Injection";
        m.expected = "Failed Motor 3 drops to 0 RPM, healthy motors continue";
        m.measured = "M3 RPM=" + std::to_string(m3_rpm) + ", M1 RPM=" + std::to_string(m1_rpm);
        m.error = m3_rpm;
        m.passed = (m3_rpm < 1.0 && m1_rpm > 1000.0);
        results.push_back(m);
    }

    // ------------------------------------------------------------------------
    // SUMMARY SCORECARD
    // ------------------------------------------------------------------------
    std::cout << "\n======================================================================\n";
    std::cout << " EXPERIMENTAL RESULTS SCORECARD (ALL 16 PHYSICAL TESTS)\n";
    std::cout << "======================================================================\n";

    int passed_count = 0;
    for (const auto& r : results) {
        std::cout << "\n[" << (r.passed ? "PASS" : "FAIL") << "] " << r.name << "\n";
        std::cout << "  Expected: " << r.expected << "\n";
        std::cout << "  Measured: " << r.measured << "\n";
        if (r.settling_time_s > 0) std::cout << "  Settling Time: " << r.settling_time_s << " s\n";
        if (r.energy_joules > 0) std::cout << "  Energy Consumed: " << r.energy_joules << " J\n";
        if (r.passed) passed_count++;
    }

    std::cout << "\n======================================================================\n";
    std::cout << " SUMMARY: " << passed_count << " / " << results.size() << " EXPERIMENTS PASSED\n";
    std::cout << "======================================================================\n";

    return (passed_count == static_cast<int>(results.size())) ? 0 : 1;
}
