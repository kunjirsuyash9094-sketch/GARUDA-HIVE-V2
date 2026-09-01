#pragma once
#include "core/math_types.hpp"
#include "garuda/core/telemetry_snapshot.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <cstring>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

/**
 * @brief 64-bit FNV-1a Deterministic State Hasher.
 */
class StateHasher {
public:
    StateHasher() noexcept { reset(); }

    void reset() noexcept {
        _hash = 0xCBF29CE484222325ULL; // FNV offset basis
    }

    void update_bytes(const void* data, size_t size) noexcept {
        const auto* bytes = static_cast<const uint8_t*>(data);
        for (size_t i = 0; i < size; ++i) {
            _hash ^= bytes[i];
            _hash *= 0x100000001B3ULL; // FNV prime
        }
    }

    void update_double(double val) noexcept {
        // Canonicalize -0.0 to +0.0 to prevent floating-point sign differences
        if (val == 0.0) val = 0.0;
        uint64_t bits = 0;
        std::memcpy(&bits, &val, sizeof(double));
        update_bytes(&bits, sizeof(uint64_t));
    }

    void update_vec3(const Vec3d& v) noexcept {
        update_double(v.x);
        update_double(v.y);
        update_double(v.z);
    }

    void update_quat(const Quat& q) noexcept {
        update_double(q.x);
        update_double(q.y);
        update_double(q.z);
        update_double(q.w);
    }

    void update_snapshot(const DroneTelemetrySnapshot& telem) noexcept {
        update_vec3(telem.position_world);
        update_vec3(telem.velocity_world);
        update_quat(telem.orientation);
        update_vec3(telem.angular_velocity_rads);
        update_double(telem.total_thrust_n);
        update_double(telem.battery_soc);
        update_double(telem.battery_voltage_terminal);
        for (double rpm : telem.motor_rpm) update_double(rpm);
    }

    [[nodiscard]] uint64_t digest() const noexcept { return _hash; }
    [[nodiscard]] std::string hex_digest() const {
        std::ostringstream ss;
        ss << std::hex << std::setw(16) << std::setfill('0') << _hash;
        return ss.str();
    }

private:
    uint64_t _hash{0xCBF29CE484222325ULL};
};

struct ReplayActionFrame {
    uint64_t    tick{0};
    std::string drone_id{};
    bool        armed{true};
    bool        direct_mode{false};
    double      roll_rad{0.0};
    double      pitch_rad{0.0};
    double      yaw_rate_rads{0.0};
    double      throttle{0.0};
    std::vector<double> direct_throttles{};
};

struct ReplayManifest {
    uint64_t                 seed{1000};
    double                   timestep_dt{0.0025};
    size_t                   total_ticks{0};
    std::vector<std::string> drone_ids{};
    std::vector<ReplayActionFrame> frames{};
    uint64_t                 final_state_hash{0};
};

class ReplaySystem {
public:
    ReplaySystem() = default;

    void start_recording(uint64_t seed, double dt) noexcept {
        _manifest = ReplayManifest{};
        _manifest.seed = seed;
        _manifest.timestep_dt = dt;
        _is_recording = true;
        _is_playing = false;
        _play_index = 0;
    }

    void record_action(const ReplayActionFrame& frame) {
        if (_is_recording) {
            _manifest.frames.push_back(frame);
            _manifest.total_ticks = std::max(_manifest.total_ticks, frame.tick);
        }
    }

    void finish_recording(uint64_t final_hash) noexcept {
        _is_recording = false;
        _manifest.final_state_hash = final_hash;
    }

    void start_playback(ReplayManifest manifest) noexcept {
        _manifest = std::move(manifest);
        _is_recording = false;
        _is_playing = true;
        _play_index = 0;
    }

    [[nodiscard]] std::vector<ReplayActionFrame> get_actions_for_tick(uint64_t tick) const {
        std::vector<ReplayActionFrame> matched;
        for (const auto& f : _manifest.frames) {
            if (f.tick == tick) {
                matched.push_back(f);
            }
        }
        return matched;
    }

    [[nodiscard]] const ReplayManifest& manifest() const noexcept { return _manifest; }
    [[nodiscard]] bool is_recording() const noexcept { return _is_recording; }
    [[nodiscard]] bool is_playing() const noexcept { return _is_playing; }

private:
    ReplayManifest _manifest{};
    bool           _is_recording{false};
    bool           _is_playing{false};
    size_t         _play_index{0};
};

} // namespace garuda
