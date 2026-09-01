#include "garuda/core/drone_instance.hpp"
#include <cmath>
#include <algorithm>

namespace garuda {

DroneInstance::DroneInstance(
    std::string drone_id,
    QuadrotorConfig config,
    uint64_t seed,
    Vec3d spawn_pos
) noexcept
    : _id(std::move(drone_id))
    , _cfg(std::move(config))
    , _seed(seed)
    , _integrator(_cfg)
    , _motors(_cfg)
    , _propulsion(_cfg)
    , _battery(_cfg)
    , _ground_contact(_cfg)
    , _sensors(_cfg, seed)
    , _controller(_cfg)
    , _payload()
    , _health() {
    reset(spawn_pos);
}

void DroneInstance::reset(Vec3d spawn_pos, Quat spawn_orient) noexcept {
    _armed = false;
    _direct_motor_mode = false;
    _direct_throttles.assign(_cfg.rotor_count, 0.0);
    _setpoints = {};
    _in_contact = true;

    _integrator.reset(spawn_pos, spawn_orient);
    _motors.reset();
    _propulsion.reset();
    _battery.reset();
    _sensors.reset(_seed);
    _controller.reset();
    _payload.attach_payload(PayloadType::INSPECTION_CAMERA);

    _telemetry = DroneTelemetrySnapshot{};
    _telemetry.drone_id = _id;
    _telemetry.position_world = spawn_pos;
    _telemetry.orientation = spawn_orient;
    _telemetry.battery_voltage_terminal = _cfg.max_voltage();
    _telemetry.battery_voltage_ocv = _cfg.max_voltage();
    _telemetry.motor_rpm.assign(_cfg.rotor_count, 0.0);
    _telemetry.motor_thrust_n.assign(_cfg.rotor_count, 0.0);
    _telemetry.motor_power_w.assign(_cfg.rotor_count, 0.0);
    _telemetry.motor_temp_c.assign(_cfg.rotor_count, _cfg.motor_temp_ambient_c);
    _telemetry.motor_health.assign(_cfg.rotor_count, 0);
    _telemetry.cell_voltages.fill(_cfg.battery_v_max_cell);
    _telemetry.vehicle_state = VehicleLifecycleState::DISARMED;
    _telemetry.vehicle_health = VehicleHealthState::OFFLINE;
    _telemetry.sensor_status.fill(2); // NOMINAL
    _telemetry.sensor_health.fill(0); // NOMINAL
}

void DroneInstance::arm() noexcept {
    if (!_armed) {
        _armed = true;
        _direct_motor_mode = false;
        _controller.reset();
    }
}

void DroneInstance::disarm() noexcept {
    _armed = false;
    _direct_motor_mode = false;
}

void DroneInstance::set_attitude_setpoint(double roll_rad, double pitch_rad, double yaw_rate_rads, double throttle_norm) noexcept {
    _setpoints.roll_rad = roll_rad;
    _setpoints.pitch_rad = pitch_rad;
    _setpoints.yaw_rate_rads = yaw_rate_rads;
    _setpoints.thrust_norm = throttle_norm;
    _direct_motor_mode = false;
}

void DroneInstance::set_direct_motor_throttles(const std::vector<double>& throttles) noexcept {
    _direct_throttles = throttles;
    _direct_throttles.resize(_cfg.rotor_count, 0.0);
    _direct_motor_mode = true;
}

bool DroneInstance::attach_payload(PayloadType type) noexcept {
    return _payload.attach_payload(type);
}

bool DroneInstance::detach_payload() noexcept {
    return _payload.detach_payload();
}

void DroneInstance::set_camera_gimbal(double pitch_deg, double yaw_deg) noexcept {
    _payload.set_gimbal(pitch_deg, yaw_deg);
}

void DroneInstance::set_camera_zoom(double zoom) noexcept {
    _payload.set_zoom(zoom);
}

void DroneInstance::set_sensor_enabled(size_t index, bool enabled) noexcept {
    _sensors.set_sensor_enabled(index, enabled);
}

void DroneInstance::set_sensor_status(size_t index, SensorStatus status) noexcept {
    _sensors.set_sensor_status(index, status);
}

void DroneInstance::inject_motor_failure(size_t motor_index, MotorHealthState state, double degradation) noexcept {
    _motors.set_motor_health(motor_index, state, degradation);
}

void DroneInstance::inject_sensor_failure(SensorMode mode) noexcept {
    _sensors.set_mode(mode);
}

void DroneInstance::inject_battery_degradation(double capacity_factor, double resistance_multiplier) noexcept {
    _battery.inject_battery_degradation(capacity_factor, resistance_multiplier);
}

void DroneInstance::reset_all_failures() noexcept {
    for (size_t i = 0; i < _motors.motor_count(); ++i) {
        _motors.set_motor_health(i, MotorHealthState::NORMAL, 1.0);
    }
    _sensors.set_mode(SensorMode::SIMULATED);
    _battery.inject_battery_degradation(1.0, 1.0);
    if (_payload.current().type != PayloadType::NONE) {
        _payload.mutable_camera().set_health(0);
        _payload.mutable_camera().set_status(CameraStatus::STREAMING);
    }
}

void DroneInstance::step(EnvironmentSystem& env, uint64_t tick, double dt) noexcept {
    if (dt <= 0.0) return;

    const auto& s = _integrator.state();
    const double alt = s.position.y;
    const double sim_time = tick * dt;

    // 1. Authoritative Dynamic Physical Payload Coupling
    const double effective_mass_kg = _payload.effective_total_mass_kg(_cfg.dry_mass_kg);
    const Vec3d effective_inertia = _payload.effective_inertia_diag(_cfg.dry_mass_kg, _cfg.inertia_diag_kgm2);
    const Vec3d effective_com = _payload.effective_com(_cfg.dry_mass_kg);
    const double payload_power_w = _payload.payload_power_w();

    // Update physical parameters in rigid body integrator
    _integrator.set_mass_properties(effective_mass_kg, effective_inertia, effective_com);

    // 2. Atmosphere & Wind
    dronesim::AtmosphericState atm = env.at_altitude(alt);
    Vec3d wind_w = env.sample_wind(alt, dt);

    // 3. Flight Control Allocation
    std::vector<double> motor_cmds(_cfg.rotor_count, 0.0);
    if (_battery.is_depleted() || !_armed) {
        // Disarmed or battery cutoff -> motors off
        motor_cmds.assign(_cfg.rotor_count, 0.0);
    } else if (_direct_motor_mode) {
        motor_cmds = _direct_throttles;
    } else {
        // Active Cascade PID with 8-Rotor Mixer
        auto fc_out = _controller.update(_setpoints, s.orientation, s.angular_velocity, _armed, dt);
        motor_cmds = fc_out.motor_throttles;
    }

    // 4. Battery Supply & Voltage Sag Step (Motors + Avionics + Payload Draw)
    double electrical_power_demand = _motors.total_electrical_power() + payload_power_w;
    double supply_voltage = _battery.step(electrical_power_demand, dt);

    // 5. Motor Dynamics Step (8 Motors)
    _motors.set_commands(motor_cmds, supply_voltage);
    _motors.step_dynamics(dt);

    // 6. Rotor Blade Element Aerodynamics & Wrench Solve
    auto aero_out = _propulsion.solve(
        _motors,
        s.velocity,
        s.angular_velocity,
        s.orientation,
        atm,
        wind_w,
        env.ground_height(),
        alt,
        dt
    );

    // 7. World Forces Assembly (Propulsion + Dynamic Mass Gravity + Airframe Drag)
    Vec3d total_force_world = s.orientation.rotate(aero_out.wrench_body.force);
    Vec3d total_torque_body = aero_out.wrench_body.torque;

    // Dynamic Gravity Force: F_g = [0, -m_effective * g, 0]
    total_force_world += Vec3d{0.0, -effective_mass_kg * env.gravity(), 0.0};

    // Quadratic & Linear Airframe Drag
    Vec3d v_rel = s.velocity - wind_w;
    double v_rel_norm2 = v_rel.norm2();
    if (v_rel_norm2 > 1e-6) {
        double v_rel_norm = std::sqrt(v_rel_norm2);
        double drag_scalar = _cfg.body_drag_linear_kg_s + _cfg.body_drag_quad_kg_m * atm.density * v_rel_norm;
        total_force_world -= v_rel * drag_scalar;
    }

    // 8. Ground Interaction & Contact Forces
    auto contact = _ground_contact.evaluate(
        s.position,
        s.velocity,
        s.angular_velocity,
        s.orientation,
        env.ground_height(),
        dt
    );

    if (contact.in_contact) {
        total_force_world += contact.force_world;
        total_torque_body += s.orientation.conjugate().rotate(contact.torque_world);
        _in_contact = true;
    } else {
        _in_contact = false;
    }

    // 9. Rigid Body 6-DOF Spatial Integration (using effective mass & inertia)
    _integrator.step(total_force_world, total_torque_body, dt);

    // 10. Resting Ground Contact Penetration Resolution
    _ground_contact.resolve_penetration(
        _integrator.mutable_state().position,
        _integrator.mutable_state().velocity,
        _integrator.mutable_state().angular_velocity,
        env.ground_height()
    );

    _in_contact = contact.in_contact || (_integrator.state().position.y <= env.ground_height() + _cfg.ground_contact_radius_m + 0.005);

    // 11. Synthetic Sensors Update (All 8 Sensors)
    const auto& updated_s = _integrator.state();
    double h_agl = std::max(0.0, updated_s.position.y - env.ground_height());
    _last_imu   = _sensors.measure_imu(updated_s.acceleration, updated_s.angular_velocity, updated_s.orientation, env.gravity(), dt, sim_time);
    _last_baro  = _sensors.measure_baro(atm, updated_s.position.y, dt, sim_time);
    _last_gps   = _sensors.measure_gps(updated_s.position, updated_s.velocity, dt, sim_time);
    _last_mag   = _sensors.measure_mag(updated_s.orientation, dt, sim_time);
    _last_lidar = _sensors.measure_lidar(h_agl, dt, sim_time);
    _last_prox  = _sensors.measure_proximity(h_agl, dt, sim_time);

    // 12. Subsystem Health Evaluation
    _last_health_report = _health.evaluate(_motors, _battery, _sensors, _payload, _armed);

    // 13. Lifecycle State Machine Resolution
    VehicleLifecycleState v_state = VehicleLifecycleState::DISARMED;
    bool any_motor_failed = false;
    for (const auto& m : _motors.motors()) {
        if (m.health == MotorHealthState::FAILED) any_motor_failed = true;
    }

    if (any_motor_failed || _battery.is_depleted()) {
        v_state = VehicleLifecycleState::FAULT;
    } else if (!_armed) {
        v_state = _in_contact ? VehicleLifecycleState::LANDED : VehicleLifecycleState::DISARMED;
    } else { // Armed
        if (_in_contact) {
            v_state = (_setpoints.thrust_norm > 0.10) ? VehicleLifecycleState::TAKEOFF : VehicleLifecycleState::ARMED;
        } else {
            v_state = (_setpoints.thrust_norm < 0.22 && updated_s.velocity.y < -0.5) ? VehicleLifecycleState::LANDING : VehicleLifecycleState::FLIGHT;
        }
    }

    // 14. Compile Authoritative Telemetry Snapshot
    Quat q_frd{ -updated_s.orientation.z, updated_s.orientation.x, -updated_s.orientation.y, updated_s.orientation.w };
    Vec3d rpy = q_frd.to_euler_rpy();
    double twr = aero_out.total_thrust_n / std::max(0.1, effective_mass_kg * env.gravity());
    double weight_n = effective_mass_kg * env.gravity();
    double max_possible_thrust = aero_out.total_thrust_n * (1.0 / std::max(0.01, _setpoints.thrust_norm));
    double thrust_margin = std::max(0.0, max_possible_thrust - weight_n);

    _telemetry.drone_id = _id;
    _telemetry.simulation_tick = tick;
    _telemetry.simulation_time_s = sim_time;

    _telemetry.position_world = updated_s.position;
    _telemetry.velocity_world = updated_s.velocity;
    _telemetry.acceleration_world = updated_s.acceleration;
    _telemetry.altitude_m = updated_s.position.y;
    _telemetry.ground_speed_ms = std::sqrt(updated_s.velocity.x * updated_s.velocity.x + updated_s.velocity.z * updated_s.velocity.z);
    _telemetry.vertical_speed_ms = updated_s.velocity.y;

    _telemetry.orientation = updated_s.orientation;
    _telemetry.euler_rpy_deg = Vec3d{ rpy.x * (180.0 / std::numbers::pi),
                                      rpy.y * (180.0 / std::numbers::pi),
                                      rpy.z * (180.0 / std::numbers::pi) };
    _telemetry.angular_velocity_rads = updated_s.angular_velocity;

    _telemetry.total_thrust_n = aero_out.total_thrust_n;
    _telemetry.thrust_to_weight_ratio = twr;
    _telemetry.available_thrust_margin_n = thrust_margin;

    _telemetry.motor_rpm.resize(_motors.motor_count());
    _telemetry.motor_thrust_n.resize(_motors.motor_count());
    _telemetry.motor_power_w.resize(_motors.motor_count());
    _telemetry.motor_temp_c.resize(_motors.motor_count());
    _telemetry.motor_health.resize(_motors.motor_count());

    for (size_t i = 0; i < _motors.motor_count(); ++i) {
        _telemetry.motor_rpm[i] = _motors.motors()[i].rpm;
        _telemetry.motor_thrust_n[i] = _motors.motors()[i].thrust;
        _telemetry.motor_power_w[i] = _motors.motors()[i].power_electrical;
        _telemetry.motor_temp_c[i] = _motors.motors()[i].temperature_c;
        _telemetry.motor_health[i] = static_cast<int>(_motors.motors()[i].health);
    }

    const auto& bstate = _battery.state();
    _telemetry.battery_voltage_terminal = bstate.voltage_terminal;
    _telemetry.battery_voltage_ocv = bstate.voltage_ocv;
    _telemetry.battery_current_amps = bstate.current_amps;
    _telemetry.battery_soc = bstate.soc;
    _telemetry.battery_power_w = bstate.ledger.total_power_w;
    _telemetry.battery_temp_c = bstate.temperature_c;
    _telemetry.energy_consumed_joules = bstate.ledger.energy_consumed_joules;
    _telemetry.energy_remaining_joules = bstate.ledger.energy_remaining_joules;
    _telemetry.cell_voltages = bstate.cell_voltages;

    // Modular Payload Telemetry
    const auto& pdesc = _payload.current();
    _telemetry.payload_id = pdesc.id;
    _telemetry.payload_name = pdesc.name;
    _telemetry.payload_category = pdesc.category;
    _telemetry.payload_type = static_cast<int>(pdesc.type);
    _telemetry.payload_mass_kg = _payload.has_payload() ? pdesc.mass_kg : 0.0;
    _telemetry.payload_attached = _payload.has_payload();
    _telemetry.payload_power_w = _payload.payload_power_w();
    _telemetry.payload_state = static_cast<int>(pdesc.state);
    _telemetry.payload_health = pdesc.health;
    _telemetry.effective_com_offset_m = effective_com;

    // Inspection Camera Telemetry
    const auto& cam = _payload.camera();
    _telemetry.camera_status = static_cast<int>(cam.status());
    _telemetry.camera_health = cam.health();
    _telemetry.camera_pitch_deg = cam.pitch_deg();
    _telemetry.camera_yaw_deg = cam.yaw_deg();
    _telemetry.camera_zoom = cam.zoom_level();

    // Sensor Suite Telemetry (8 Sensors)
    for (size_t i = 0; i < DeterministicSensorSuite::NUM_SENSORS; ++i) {
        _telemetry.sensor_status[i] = static_cast<int>(_sensors.descriptors()[i].status);
        _telemetry.sensor_health[i] = _sensors.descriptors()[i].health;
    }
    _telemetry.lidar_distance_m = _last_lidar.distance_m;
    _telemetry.proximity_distance_m = _last_prox.distance_down_m;

    // Environment & Aerodynamics
    _telemetry.air_density_kgm3 = atm.density;
    _telemetry.ground_effect_factor = aero_out.ground_effect_factor;
    _telemetry.vrs_active = aero_out.vrs_active;
    _telemetry.vrs_severity = aero_out.vrs_severity;
    _telemetry.wind_world = wind_w;

    // Status & Health
    _telemetry.armed = _armed;
    _telemetry.in_ground_contact = _in_contact;
    _telemetry.low_voltage_warning = bstate.low_voltage_warning;
    _telemetry.critical_battery_cutoff = bstate.critical_cutoff;
    _telemetry.sensor_health_mode = static_cast<int>(_sensors.mode());
    _telemetry.vehicle_health = _last_health_report.overall;
    _telemetry.health_diagnostics = _last_health_report.diagnostics;
    _telemetry.vehicle_state = v_state;
    _telemetry.flight_mode = _direct_motor_mode ? "DIRECT_THROTTLE" : (_armed ? "ATTITUDE_PID" : "DISARMED");
}

} // namespace garuda
