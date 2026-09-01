#include "garuda/core/simulation_world.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>

using namespace garuda;

static void run_scenario_free_fall() {
    std::cout << "\n==================================================\n";
    std::cout << " [SCENARIO 01] FREE FALL EXPERIMENT\n";
    std::cout << "==================================================\n";
    SimulationWorld world(1001, 0.0025);
    auto* d = world.add_drone("GARUDA-01", {}, {0.0, 10.0, 0.0});
    d->disarm();

    std::cout << "Spawning GARUDA-01 at y = 10.00m (Disarmed, Motors Off)\n";
    std::cout << "Target Gravity: -9.80665 m/s^2\n\n";

    world.step();
    double initial_accel = d->physics_state().acceleration.y;

    for (int tick = 1; tick <= 800; ++tick) {
        world.step();
        const auto& t = d->telemetry();

        if (tick % 100 == 0 || t.in_ground_contact) {
            std::cout << "  t = " << std::fixed << std::setprecision(3) << t.simulation_time_s
                      << "s | Alt: " << std::setw(6) << std::setprecision(3) << t.altitude_m
                      << "m | Vy: " << std::setw(7) << std::setprecision(3) << t.vertical_speed_ms
                      << "m/s | Contact: " << (t.in_ground_contact ? "YES" : "NO") << "\n";
        }
        if (t.in_ground_contact && tick > 100) break;
    }

    std::cout << "\n[VALIDATION METRICS]\n";
    std::cout << "  Initial Free Fall Accel: " << initial_accel << " m/s^2 (Expected: -9.80665 m/s^2)\n";
    std::cout << "  Ground Landing Altitude: " << d->telemetry().altitude_m << " m (Expected: ~0.150 m)\n";
    bool pass = std::abs(initial_accel - (-9.80665)) < 1e-4 && d->telemetry().altitude_m < 0.20;
    std::cout << "  Result: " << (pass ? "PASS" : "FAIL") << "\n";
}

static void run_scenario_hover() {
    std::cout << "\n==================================================\n";
    std::cout << " [SCENARIO 02] HOVER EQUILIBRIUM EXPERIMENT\n";
    std::cout << "==================================================\n";
    SimulationWorld world(1002, 0.0025);
    auto* d = world.add_drone("GARUDA-01", {}, {0.0, 3.0, 0.0});
    d->arm();

    const double hover_thr = 0.2835;
    double mg = d->config().mass_kg * world.environment().gravity();
    std::cout << "Spawning GARUDA-01 at y = 3.00m. Armed with Hover Throttle = " << hover_thr << "\n";
    std::cout << "Vehicle Mass: " << d->config().mass_kg << " kg | Weight (mg): " << mg << " N\n\n";

    for (int tick = 0; tick <= 1600; ++tick) {
        d->set_attitude_setpoint(0.0, 0.0, 0.0, hover_thr);
        world.step();
        const auto& t = d->telemetry();

        if (tick % 200 == 0) {
            std::cout << "  t = " << std::fixed << std::setprecision(2) << t.simulation_time_s
                      << "s | Alt: " << std::setw(5) << std::setprecision(2) << t.altitude_m
                      << "m | Vy: " << std::setw(6) << std::setprecision(2) << t.vertical_speed_ms
                      << "m/s | Thrust: " << std::setw(5) << std::setprecision(1) << t.total_thrust_n
                      << "N | TWR: " << std::setprecision(2) << t.thrust_to_weight_ratio
                      << " | Power: " << std::setprecision(0) << t.battery_power_w << "W\n";
        }
    }

    const auto& final_t = d->telemetry();
    double thrust_err = std::abs(final_t.total_thrust_n - mg);
    std::cout << "\n[VALIDATION METRICS]\n";
    std::cout << "  Hover Thrust Measured: " << final_t.total_thrust_n << " N (Weight: " << mg << " N)\n";
    std::cout << "  Thrust Equilibrium Error: " << thrust_err << " N\n";
    std::cout << "  Final Vertical Speed: " << final_t.vertical_speed_ms << " m/s\n";
    bool pass = (thrust_err < 0.20 && std::abs(final_t.vertical_speed_ms) < 0.25);
    std::cout << "  Result: " << (pass ? "PASS" : "FAIL") << "\n";
}

static void run_scenario_takeoff_landing() {
    std::cout << "\n==================================================\n";
    std::cout << " [SCENARIO 03 & 10] TAKEOFF AND LANDING SEQUENCE\n";
    std::cout << "==================================================\n";
    SimulationWorld world(1003, 0.0025);
    auto* d = world.add_drone("GARUDA-01", {}, {0.0, 0.15, 0.0});
    d->arm();

    std::cout << "Phase 1: Takeoff with 50% Throttle (0s - 2s)\n";
    for (int tick = 0; tick < 800; ++tick) {
        d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.50);
        world.step();
        if (tick % 200 == 0) {
            const auto& t = d->telemetry();
            std::cout << "  [Climb] t=" << t.simulation_time_s << "s Alt=" << t.altitude_m
                      << "m Vy=" << t.vertical_speed_ms << "m/s Contact=" << t.in_ground_contact << "\n";
        }
    }

    std::cout << "\nPhase 2: Controlled Descent & Ground Landing with 15% Throttle (2s - 8.5s)\n";
    for (int tick = 800; tick <= 3400; ++tick) {
        d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.15);
        world.step();
        if (tick % 400 == 0 || d->telemetry().in_ground_contact) {
            const auto& t = d->telemetry();
            std::cout << "  [Descent] t=" << std::fixed << std::setprecision(2) << t.simulation_time_s
                      << "s Alt=" << t.altitude_m << "m Vy=" << t.vertical_speed_ms
                      << "m/s Contact=" << (t.in_ground_contact ? "YES" : "NO") << "\n";
        }
        if (d->telemetry().in_ground_contact && tick > 1200) break;
    }

    std::cout << "\n[VALIDATION METRICS]\n";
    std::cout << "  Landed Altitude: " << d->telemetry().altitude_m << " m (Expected: ~0.150 m)\n";
    std::cout << "  Ground Contact Flag: " << (d->telemetry().in_ground_contact ? "TRUE" : "FALSE") << "\n";
    bool pass = (d->telemetry().in_ground_contact && d->telemetry().altitude_m < 0.20);
    std::cout << "  Result: " << (pass ? "PASS" : "FAIL") << "\n";
}

static void run_scenario_motor_failure() {
    std::cout << "\n==================================================\n";
    std::cout << " [SCENARIO 11] MOTOR FAILURE INJECTION EXPERIMENT\n";
    std::cout << "==================================================\n";
    SimulationWorld world(1004, 0.0025);
    auto* d = world.add_drone("GARUDA-01", {}, {0.0, 5.0, 0.0});
    d->arm();

    std::cout << "Hovering at 5m. Injecting catastrophic MOTOR 1 FAILURE at t = 1.0s\n\n";

    for (int tick = 0; tick <= 800; ++tick) {
        if (tick == 400) { // t = 1.0s
            std::cout << "  >>> INJECTING MOTOR 1 FAILURE (Health -> FAILED, RPM -> 0) <<<\n";
            d->inject_motor_failure(0, MotorHealthState::FAILED);
        }
        d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.2835);
        world.step();

        if (tick % 100 == 0) {
            const auto& t = d->telemetry();
            std::cout << "  t=" << std::fixed << std::setprecision(2) << t.simulation_time_s
                      << "s | Alt=" << t.altitude_m << "m | Roll=" << t.euler_rpy_deg.x
                      << "° | Pitch=" << t.euler_rpy_deg.y << "° | M1_RPM=" << t.motor_rpm[0]
                      << " | M2_RPM=" << t.motor_rpm[1] << "\n";
        }
    }

    std::cout << "\n[VALIDATION METRICS]\n";
    std::cout << "  Motor 1 Final RPM: " << d->telemetry().motor_rpm[0] << " (Expected: 0.0)\n";
    std::cout << "  Attitude Disturbance Detected: Roll = " << d->telemetry().euler_rpy_deg.x << "°\n";
    bool pass = (d->telemetry().motor_rpm[0] < 1.0 && std::abs(d->telemetry().euler_rpy_deg.x) > 10.0);
    std::cout << "  Result: " << (pass ? "PASS" : "FAIL") << "\n";
}

static void run_scenario_battery_depletion() {
    std::cout << "\n==================================================\n";
    std::cout << " [SCENARIO 12] BATTERY DISCHARGE & VOLTAGE SAG EXPERIMENT\n";
    std::cout << "==================================================\n";
    SimulationWorld world(1005, 0.0025);
    auto* d = world.add_drone("GARUDA-01", {}, {0.0, 2.0, 0.0});
    d->arm();
    // Inject small capacity battery for fast depletion test (0.05 Ah)
    d->inject_battery_degradation(0.0125, 1.0);

    std::cout << "Initial OCV: 16.80V | Initial SoC: 100.0%\n";
    std::cout << "Drawing full throttle propulsion power...\n\n";

    for (int tick = 0; tick <= 2000; ++tick) {
        d->set_attitude_setpoint(0.0, 0.0, 0.0, 0.90);
        world.step();
        const auto& t = d->telemetry();

        if (tick % 400 == 0 || t.critical_battery_cutoff) {
            std::cout << "  t=" << std::fixed << std::setprecision(2) << t.simulation_time_s
                      << "s | SoC=" << std::setprecision(1) << (t.battery_soc * 100.0)
                      << "% | V_term=" << std::setprecision(2) << t.battery_voltage_terminal
                      << "V | I=" << std::setprecision(1) << t.battery_current_amps
                      << "A | Consumed=" << std::setprecision(0) << t.energy_consumed_joules
                      << "J | Cutoff=" << (t.critical_battery_cutoff ? "YES" : "NO") << "\n";
        }
        if (t.critical_battery_cutoff) break;
    }

    std::cout << "\n[VALIDATION METRICS]\n";
    std::cout << "  Final Battery SoC: " << (d->telemetry().battery_soc * 100.0) << "%\n";
    std::cout << "  Total Energy Accounted in Ledger: " << d->telemetry().energy_consumed_joules << " J\n";
    bool pass = d->telemetry().energy_consumed_joules > 0.0;
    std::cout << "  Result: " << (pass ? "PASS" : "FAIL") << "\n";
}

static void run_scenario_multi_drone() {
    std::cout << "\n==================================================\n";
    std::cout << " [SCENARIO 14] MULTI-DRONE INDEPENDENCE EXPERIMENT\n";
    std::cout << "==================================================\n";
    SimulationWorld world(2000, 0.0025);

    auto* d1 = world.add_drone("GARUDA-01", {}, { 0.0, 2.0,  0.0});
    auto* d2 = world.add_drone("GARUDA-02", {}, { 5.0, 2.0,  0.0});
    auto* d3 = world.add_drone("GARUDA-03", {}, { 0.0, 2.0,  5.0});
    auto* d4 = world.add_drone("GARUDA-04", {}, { 5.0, 2.0,  5.0});

    d1->arm();
    d2->arm();
    d3->arm();
    d4->disarm(); // Keep D4 disarmed

    std::cout << "Registered 4 independent drones in SimulationWorld.\n";
    std::cout << "D1: Climb (thr=0.50), D2: Hover (thr=0.2835), D3: Roll Right (roll=+15°), D4: Disarmed (Falls)\n\n";

    for (int tick = 0; tick <= 600; ++tick) {
        d1->set_attitude_setpoint(0.0, 0.0, 0.0, 0.50);
        d2->set_attitude_setpoint(0.0, 0.0, 0.0, 0.2835);
        d3->set_attitude_setpoint(0.26, 0.0, 0.0, 0.2835);
        world.step();
    }

    auto snap = world.get_world_telemetry();
    std::cout << "Final Telemetry Snapshots (t = 1.50s):\n";
    for (const auto& dt : snap.drones) {
        std::cout << "  [" << dt.drone_id << "] Pos: (" << std::fixed << std::setprecision(2)
                  << dt.position_world.x << ", " << dt.position_world.y << ", " << dt.position_world.z
                  << ") | Roll: " << dt.euler_rpy_deg.x << "° | Thrust: " << dt.total_thrust_n << "N\n";
    }

    bool d1_climbed = snap.drones[0].position_world.y > 2.5;
    bool d4_grounded = snap.drones[3].in_ground_contact;
    bool d3_rolled = snap.drones[2].position_world.x > 0.2;

    std::cout << "\n[VALIDATION METRICS]\n";
    std::cout << "  Drone 1 Independent Climb: " << (d1_climbed ? "YES" : "NO") << "\n";
    std::cout << "  Drone 3 Independent Roll Translation: " << (d3_rolled ? "YES" : "NO") << "\n";
    std::cout << "  Drone 4 Independent Fall to Ground: " << (d4_grounded ? "YES" : "NO") << "\n";
    bool pass = (d1_climbed && d4_grounded && d3_rolled);
    std::cout << "  Result: " << (pass ? "PASS" : "FAIL") << "\n";
}

static void run_scenario_replay() {
    std::cout << "\n==================================================\n";
    std::cout << " [SCENARIO 15] DETERMINISTIC REPLAY & STATE HASH EXPERIMENT\n";
    std::cout << "==================================================\n";

    // Run 1: Record trace
    SimulationWorld w1(5555, 0.0025);
    auto* d1 = w1.add_drone("GARUDA-01", {}, {0.0, 2.0, 0.0});
    d1->arm();
    w1.start_recording();

    for (int tick = 0; tick < 400; ++tick) {
        double thr = 0.2835 + 0.10 * std::sin(tick * 0.02);
        double roll = 0.15 * std::cos(tick * 0.03);
        d1->set_attitude_setpoint(roll, 0.0, 0.0, thr);

        ReplayActionFrame frame{};
        frame.tick = tick;
        frame.drone_id = "GARUDA-01";
        frame.armed = true;
        frame.roll_rad = roll;
        frame.throttle = thr;
        w1.replay().record_action(frame);

        w1.step();
    }

    uint64_t hash_run1 = w1.compute_world_state_hash();
    auto manifest = w1.replay().manifest();
    w1.finish_recording();

    std::cout << "Run 1 (Recorded 400 ticks) -> Final World State Hash: 0x" << std::hex << hash_run1 << std::dec << "\n";

    // Run 2: Replay from manifest
    SimulationWorld w2(5555, 0.0025);
    w2.add_drone("GARUDA-01", {}, {0.0, 2.0, 0.0});
    w2.replay().start_playback(manifest);

    for (int tick = 0; tick < 400; ++tick) {
        w2.step();
    }

    uint64_t hash_run2 = w2.compute_world_state_hash();
    std::cout << "Run 2 (Replayed 400 ticks) -> Final World State Hash: 0x" << std::hex << hash_run2 << std::dec << "\n";

    std::cout << "\n[VALIDATION METRICS]\n";
    std::cout << "  Hash Run 1 == Hash Run 2: " << (hash_run1 == hash_run2 ? "MATCH (100% BIT-IDENTICAL)" : "MISMATCH") << "\n";
    bool pass = (hash_run1 == hash_run2);
    std::cout << "  Result: " << (pass ? "PASS" : "FAIL") << "\n";
}

int main(int argc, char* argv[]) {
    std::string scenario = "all";
    if (argc > 1) {
        scenario = argv[1];
        if (scenario == "--scenario" && argc > 2) {
            scenario = argv[2];
        }
    }

    std::cout << "============================================================\n";
    std::cout << " GARUDA HIVE V2 — STANDALONE SIMULATION RUNTIME (C++20)\n";
    std::cout << "============================================================\n";

    if (scenario == "all" || scenario == "free_fall" || scenario == "SCENARIO_01_FREE_FALL") {
        run_scenario_free_fall();
    }
    if (scenario == "all" || scenario == "hover" || scenario == "SCENARIO_02_HOVER") {
        run_scenario_hover();
    }
    if (scenario == "all" || scenario == "takeoff" || scenario == "landing" || scenario == "SCENARIO_03_TAKEOFF") {
        run_scenario_takeoff_landing();
    }
    if (scenario == "all" || scenario == "motor_failure" || scenario == "SCENARIO_11_MOTOR_FAILURE") {
        run_scenario_motor_failure();
    }
    if (scenario == "all" || scenario == "battery" || scenario == "SCENARIO_12_BATTERY_DEPLETION") {
        run_scenario_battery_depletion();
    }
    if (scenario == "all" || scenario == "multi_drone" || scenario == "SCENARIO_14_MULTI_DRONE") {
        run_scenario_multi_drone();
    }
    if (scenario == "all" || scenario == "replay" || scenario == "SCENARIO_15_REPLAY") {
        run_scenario_replay();
    }

    std::cout << "\n============================================================\n";
    std::cout << " SCENARIO RUNS COMPLETED SUCCESSFULLY.\n";
    std::cout << "============================================================\n";
    return 0;
}
