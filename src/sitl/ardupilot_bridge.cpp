// ===========================================================================
// ArduPilot JSON external-physics bridge
//
// Reference: ardupilot/libraries/SITL/examples/JSON/readme.md
//
// ArduPilot (SIM_JSON backend) sends a binary servo packet to our UDP port
// (default 9002) and waits for a newline-delimited JSON state reply. The
// exchange is lockstep: AP's simulation clock is OUR timestamp, and AP will
// not advance until it hears back — so we reply exactly once per received
// packet, from the physics tick.
// ===========================================================================
#include "sitl/firmware_bridge.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>

using namespace dronesim;
using namespace dronesim::sitl;

// ---------------------------------------------------------------------------
bool ArduPilotBridge::tick(const SimState& s, ActuatorOutput& out) noexcept {
    if (!_transport.is_open()) return false;

    // Drain all pending datagrams; keep the newest valid servo packet.
    // (Replying to the latest keeps us in sync if AP ran ahead.)
    ServoPacket latest{};
    bool got_packet = false;
    for (;;) {
        int n = _transport.recv(_recv_buf, sizeof(_recv_buf));
        if (n <= 0) break;
        if (n < static_cast<int>(sizeof(ServoPacket))) continue;

        ServoPacket p{};
        std::memcpy(&p, _recv_buf, sizeof(p));
        if (p.magic != JSON_MAGIC_16) continue;

        // frame_count resets => ArduPilot was restarted
        if (_got_first && p.frame_count < _last_frame_count)
            _got_first = false;

        latest = p;
        got_packet = true;
    }

    if (!got_packet) return false;

    _last_frame_count = latest.frame_count;
    _got_first        = true;
    _last_rx          = std::chrono::steady_clock::now();

    // ---- Actuator output: PWM µs -> normalised [0,1] ----------------------
    out.n_channels = 16;
    out.source = ActuatorOutput::Source::ArduPilot;
    for (int i = 0; i < 16; ++i) {
        out.pwm_us[i]   = latest.pwm[i];
        out.channels[i] = pwm_to_norm(latest.pwm[i], 1000, 2000);
    }

    // ---- Lockstep reply: JSON state ---------------------------------------
    _send_json_state(s);
    return true;
}

// ---------------------------------------------------------------------------
// JSON reply. Required fields only (readme: timestamp, imu, position,
// attitude, velocity). Leading '\n' guards against any partial line AP may
// have buffered; trailing '\n' terminates the message.
// ---------------------------------------------------------------------------
void ArduPilotBridge::_send_json_state(const SimState& s) noexcept {
    const Vec3d rpy = s.orientation.to_euler_rpy();

    const int len = std::snprintf(
        _json_buf, sizeof(_json_buf),
        "\n{"
        "\"timestamp\":%.6f,"
        "\"imu\":{"
            "\"gyro\":[%.7f,%.7f,%.7f],"
            "\"accel_body\":[%.7f,%.7f,%.7f]"
        "},"
        "\"position\":[%.7f,%.7f,%.7f],"
        "\"attitude\":[%.7f,%.7f,%.7f],"
        "\"velocity\":[%.7f,%.7f,%.7f],"
        "\"airspeed\":%.5f"
        "}\n",
        s.sim_time_s,
        s.gyro_rads.x,  s.gyro_rads.y,  s.gyro_rads.z,
        s.accel_ms2.x,  s.accel_ms2.y,  s.accel_ms2.z,
        s.position_ned_m.x, s.position_ned_m.y, s.position_ned_m.z,
        rpy.x, rpy.y, rpy.z,
        s.vel_ned_ms.x, s.vel_ned_ms.y, s.vel_ned_ms.z,
        s.true_airspeed_ms);

    if (len > 0 && len < static_cast<int>(sizeof(_json_buf)))
        _transport.send(reinterpret_cast<const uint8_t*>(_json_buf),
                        static_cast<size_t>(len));
}
