#pragma once
#include <cstdint>

namespace garuda {

enum class ClockRunMode {
    PAUSED,
    STEPPING,
    REALTIME,
    ACCELERATED_2X,
    ACCELERATED_5X,
    ACCELERATED_10X,
    UNBOUNDED_FAST
};

class SimulationClock {
public:
    explicit SimulationClock(double dt = 0.0025) noexcept
        : _dt(dt)
        , _physics_hz(dt > 0.0 ? 1.0 / dt : 400.0) {}

    void set_timestep(double dt) noexcept {
        if (dt > 0.0) {
            _dt = dt;
            _physics_hz = 1.0 / dt;
        }
    }

    void reset() noexcept {
        _tick = 0;
        _sim_time_s = 0.0;
        _run_mode = ClockRunMode::PAUSED;
    }

    void advance_tick() noexcept {
        ++_tick;
        _sim_time_s = _tick * _dt;
    }

    void set_mode(ClockRunMode mode) noexcept { _run_mode = mode; }
    [[nodiscard]] ClockRunMode mode() const noexcept { return _run_mode; }

    [[nodiscard]] uint64_t tick() const noexcept { return _tick; }
    [[nodiscard]] double time_s() const noexcept { return _sim_time_s; }
    [[nodiscard]] double dt() const noexcept { return _dt; }
    [[nodiscard]] double frequency_hz() const noexcept { return _physics_hz; }
    [[nodiscard]] bool is_running() const noexcept { return _run_mode != ClockRunMode::PAUSED; }

    [[nodiscard]] double time_multiplier() const noexcept {
        switch (_run_mode) {
            case ClockRunMode::PAUSED:          return 0.0;
            case ClockRunMode::STEPPING:        return 1.0;
            case ClockRunMode::REALTIME:        return 1.0;
            case ClockRunMode::ACCELERATED_2X:  return 2.0;
            case ClockRunMode::ACCELERATED_5X:  return 5.0;
            case ClockRunMode::ACCELERATED_10X: return 10.0;
            case ClockRunMode::UNBOUNDED_FAST:  return 100.0;
            default:                            return 1.0;
        }
    }

private:
    double       _dt{0.0025};         // Fixed timestep in seconds (400 Hz)
    double       _physics_hz{400.0};  // Nominal frequency (Hz)
    uint64_t     _tick{0};            // Monotonic simulation tick counter
    double       _sim_time_s{0.0};    // Accumulated simulation time (s)
    ClockRunMode _run_mode{ClockRunMode::PAUSED};
};

} // namespace garuda
