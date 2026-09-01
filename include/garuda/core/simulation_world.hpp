#pragma once
#include "core/math_types.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include "garuda/physics/environment.hpp"
#include "garuda/core/simulation_clock.hpp"
#include "garuda/core/drone_instance.hpp"
#include "garuda/core/simulation_event.hpp"
#include "garuda/core/telemetry_snapshot.hpp"
#include "garuda/replay/replay_system.hpp"
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <cstdint>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

class SimulationWorld {
public:
    explicit SimulationWorld(
        uint64_t base_seed = 1000,
        double   fixed_timestep_s = 0.0025,
        EnvironmentState env_state = {}
    ) noexcept;

    void reset() noexcept;

    // -------------------------------------------------------------------------
    // Fleet / Drone Registry Management
    // -------------------------------------------------------------------------
    DroneInstance* add_drone(
        const std::string& drone_id,
        const QuadrotorConfig& config = {},
        Vec3d spawn_pos = Vec3d{0.0, 0.15, 0.0}
    );

    [[nodiscard]] DroneInstance* get_drone(const std::string& drone_id) noexcept;
    [[nodiscard]] const DroneInstance* get_drone(const std::string& drone_id) const noexcept;
    [[nodiscard]] size_t drone_count() const noexcept { return _drones.size(); }
    [[nodiscard]] const std::vector<std::unique_ptr<DroneInstance>>& drones() const noexcept { return _drones; }

    // -------------------------------------------------------------------------
    // Authoritative Simulation Stepping
    // -------------------------------------------------------------------------
    void step() noexcept;
    void step_n(size_t n_ticks) noexcept;

    // -------------------------------------------------------------------------
    // Event System
    // -------------------------------------------------------------------------
    void log_event(const std::string& drone_id, SimulationEventType type, const std::string& payload = "") noexcept;
    [[nodiscard]] const std::vector<SimulationEvent>& events() const noexcept { return _events; }
    void clear_events() noexcept { _events.clear(); }

    // -------------------------------------------------------------------------
    // Replay & Checksum Infrastructure
    // -------------------------------------------------------------------------
    void start_recording() noexcept;
    void finish_recording() noexcept;
    [[nodiscard]] uint64_t compute_world_state_hash() const noexcept;

    // -------------------------------------------------------------------------
    // Subsystem Accessors
    // -------------------------------------------------------------------------
    [[nodiscard]] SimulationClock& clock() noexcept { return _clock; }
    [[nodiscard]] const SimulationClock& clock() const noexcept { return _clock; }
    [[nodiscard]] EnvironmentSystem& environment() noexcept { return _env; }
    [[nodiscard]] const EnvironmentSystem& environment() const noexcept { return _env; }
    [[nodiscard]] ReplaySystem& replay() noexcept { return _replay; }
    [[nodiscard]] const ReplaySystem& replay() const noexcept { return _replay; }

    // -------------------------------------------------------------------------
    // Aggregate Telemetry
    // -------------------------------------------------------------------------
    [[nodiscard]] WorldTelemetrySnapshot get_world_telemetry() const noexcept;

private:
    uint64_t                                _base_seed{1000};
    SimulationClock                         _clock;
    EnvironmentSystem                       _env;
    ReplaySystem                            _replay;
    std::vector<std::unique_ptr<DroneInstance>> _drones;
    std::vector<SimulationEvent>            _events;
    uint64_t                                _event_seq{0};
};

} // namespace garuda
