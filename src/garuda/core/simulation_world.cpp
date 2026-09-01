#include "garuda/core/simulation_world.hpp"
#include <algorithm>

namespace garuda {

SimulationWorld::SimulationWorld(
    uint64_t base_seed,
    double   fixed_timestep_s,
    EnvironmentState env_state
) noexcept
    : _base_seed(base_seed)
    , _clock(fixed_timestep_s)
    , _env(env_state) {
    reset();
}

void SimulationWorld::reset() noexcept {
    _clock.reset();
    _events.clear();
    _event_seq = 0;
    for (auto& drone : _drones) {
        drone->reset();
    }
}

DroneInstance* SimulationWorld::add_drone(
    const std::string& drone_id,
    const QuadrotorConfig& config,
    Vec3d spawn_pos
) {
    // Prevent duplicate drone IDs
    for (const auto& d : _drones) {
        if (d->id() == drone_id) {
            return d.get();
        }
    }

    uint64_t drone_seed = _base_seed + static_cast<uint64_t>(_drones.size()) * 100ULL;
    auto instance = std::make_unique<DroneInstance>(drone_id, config, drone_seed, spawn_pos);
    DroneInstance* ptr = instance.get();
    _drones.push_back(std::move(instance));

    log_event(drone_id, SimulationEventType::SIMULATION_STARTED, "Drone registered into world");
    return ptr;
}

DroneInstance* SimulationWorld::get_drone(const std::string& drone_id) noexcept {
    for (auto& d : _drones) {
        if (d->id() == drone_id) return d.get();
    }
    return nullptr;
}

const DroneInstance* SimulationWorld::get_drone(const std::string& drone_id) const noexcept {
    for (const auto& d : _drones) {
        if (d->id() == drone_id) return d.get();
    }
    return nullptr;
}

void SimulationWorld::log_event(
    const std::string& drone_id,
    SimulationEventType type,
    const std::string& payload
) noexcept {
    SimulationEvent ev{};
    ev.event_id = ++_event_seq;
    ev.tick = _clock.tick();
    ev.simulation_time_s = _clock.time_s();
    ev.drone_id = drone_id;
    ev.event_type = type;
    ev.payload = payload;
    _events.push_back(ev);
}

void SimulationWorld::step() noexcept {
    const uint64_t current_tick = _clock.tick();
    const double dt = _clock.dt();

    // 1. Replay Playback Injection (if active)
    if (_replay.is_playing()) {
        auto actions = _replay.get_actions_for_tick(current_tick);
        for (const auto& a : actions) {
            if (auto* d = get_drone(a.drone_id)) {
                if (a.armed) d->arm(); else d->disarm();
                if (a.direct_mode) {
                    d->set_direct_motor_throttles(a.direct_throttles);
                } else {
                    d->set_attitude_setpoint(a.roll_rad, a.pitch_rad, a.yaw_rate_rads, a.throttle);
                }
            }
        }
    }

    // 2. Strict Deterministic Iteration over all Drones
    for (auto& drone : _drones) {
        bool prev_contact = drone->is_in_contact();
        bool prev_low_v = drone->battery_state().low_voltage_warning;

        drone->step(_env, current_tick, dt);

        // Detect State Events
        if (prev_contact && !drone->is_in_contact()) {
            log_event(drone->id(), SimulationEventType::TAKEOFF, "Lifted off terrain surface");
        } else if (!prev_contact && drone->is_in_contact()) {
            log_event(drone->id(), SimulationEventType::GROUND_CONTACT, "Made physical ground contact");
        }

        if (!prev_low_v && drone->battery_state().low_voltage_warning) {
            log_event(drone->id(), SimulationEventType::BATTERY_LOW, "Pack voltage entered low voltage warning threshold");
        }

        if (drone->battery_state().critical_cutoff) {
            log_event(drone->id(), SimulationEventType::BATTERY_CRITICAL, "Battery critical cutoff triggered");
        }
    }

    // 3. Advance Authoritative Simulation Clock
    _clock.advance_tick();
}

void SimulationWorld::step_n(size_t n_ticks) noexcept {
    for (size_t i = 0; i < n_ticks; ++i) {
        step();
    }
}

void SimulationWorld::start_recording() noexcept {
    _replay.start_recording(_base_seed, _clock.dt());
}

void SimulationWorld::finish_recording() noexcept {
    uint64_t final_hash = compute_world_state_hash();
    _replay.finish_recording(final_hash);
}

uint64_t SimulationWorld::compute_world_state_hash() const noexcept {
    StateHasher hasher;
    for (const auto& drone : _drones) {
        hasher.update_bytes(drone->id().data(), drone->id().size());
        hasher.update_snapshot(drone->telemetry());
    }
    return hasher.digest();
}

WorldTelemetrySnapshot SimulationWorld::get_world_telemetry() const noexcept {
    WorldTelemetrySnapshot snap{};
    snap.tick = _clock.tick();
    snap.timestamp_s = _clock.time_s();
    snap.physics_hz = _clock.frequency_hz();
    snap.realtime_factor = _clock.time_multiplier();

    snap.drones.reserve(_drones.size());
    for (const auto& drone : _drones) {
        snap.drones.push_back(drone->telemetry());
    }
    return snap;
}

} // namespace garuda
