#pragma once
#include "sitl/socket_transport.hpp"
#include "sitl/sitl_state.hpp"
#include "sitl/mavlink/mavlink_types.hpp"
#include "sitl/msp/msp_types.hpp"
#include <memory>
#include <string>
#include <chrono>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
// FirmwareBridge — abstract interface every firmware adapter implements
// ---------------------------------------------------------------------------
class FirmwareBridge {
public:
    virtual ~FirmwareBridge() = default;

    // Called once at sim startup
    virtual bool connect() noexcept = 0;

    // Called every physics tick (from Godot _integrate_forces thread):
    //   1. send SimState → firmware
    //   2. non-blocking recv firmware → parse actuator output
    //   3. populate `out` only if a new frame was received
    // Returns true if a new actuator frame was received this tick
    virtual bool tick(const SimState& state, ActuatorOutput& out) noexcept = 0;

    virtual void disconnect() noexcept = 0;

    [[nodiscard]] virtual bool is_connected() const noexcept = 0;
    [[nodiscard]] virtual const char* firmware_name() const noexcept = 0;
    [[nodiscard]] virtual uint16_t port() const noexcept = 0;
};

// ===========================================================================
// ArduPilotBridge — ArduPilot JSON external-physics backend
//
// Protocol (libraries/SITL/examples/JSON in the ArduPilot tree):
//   - The SIM is a UDP *server* on port 9002 (+10 per extra instance).
//   - ArduPilot sends a binary servo packet:
//       uint16 magic (18458), uint16 frame_rate, uint32 frame_count,
//       uint16 pwm[16] (µs)
//   - The sim replies to the sender with one newline-delimited JSON line:
//       {"timestamp":s, "imu":{"gyro":[rad/s x3],"accel_body":[m/s² x3]},
//        "position":[NED m], "attitude":[rad rpy], "velocity":[NED m/s]}
//   - This is lockstep: ArduPilot advances only after our reply, using our
//     timestamp as simulation time.
//
// ArduPilot SITL command (from ardupilot repo, e.g. inside WSL2):
//   sim_vehicle.py -v ArduCopter -f JSON:<ip-of-sim-host> --console --map
// ===========================================================================
class ArduPilotBridge : public FirmwareBridge {
public:
    explicit ArduPilotBridge(SocketTransport::Config cfg) noexcept
        : _transport(std::move(cfg)) {}

    bool connect() noexcept override { return _transport.open(); }
    void disconnect() noexcept override { _transport.close(); }
    [[nodiscard]] bool is_connected() const noexcept override {
        // "Connected" = heard from ArduPilot within the last 2 s
        return _transport.is_open() && _rx_recent(2.0);
    }
    [[nodiscard]] const char* firmware_name() const noexcept override { return "ArduPilot"; }
    [[nodiscard]] uint16_t port() const noexcept override { return _transport.config().local_port; }

    bool tick(const SimState& s, ActuatorOutput& out) noexcept override;

private:
#pragma pack(push, 1)
    struct ServoPacket {                  // ArduPilot -> sim
        uint16_t magic{};                 // 18458 (16ch) / 29569 (32ch)
        uint16_t frame_rate{};            // Hz requested by AP (SIM_RATE_HZ)
        uint32_t frame_count{};
        uint16_t pwm[16]{};               // µs
    };
#pragma pack(pop)
    static constexpr uint16_t JSON_MAGIC_16 = 18458;

    void _send_json_state(const SimState& s) noexcept;
    [[nodiscard]] bool _rx_recent(double window_s) const noexcept {
        if (_last_rx.time_since_epoch().count() == 0) return false;
        auto dt = std::chrono::steady_clock::now() - _last_rx;
        return std::chrono::duration<double>(dt).count() < window_s;
    }

    SocketTransport _transport;
    uint32_t  _last_frame_count{0};
    bool      _got_first{false};
    std::chrono::steady_clock::time_point _last_rx{};

    uint8_t _recv_buf[1024]{};
    char    _json_buf[1024]{};
};

// ===========================================================================
// PX4Bridge — PX4 Simulator MAVLink API
//
// Protocol (docs.px4.io "Simulator MAVLink API"):
//   - The SIM is a TCP *server* on port 4560; PX4 SITL connects as client.
//   - Sim → PX4: HIL_SENSOR every physics tick + HIL_GPS (~10 Hz)
//                + HEARTBEAT (1 Hz). time_usec = sim time (drives lockstep).
//   - PX4 → Sim: HIL_ACTUATOR_CONTROLS, 16 floats; motors are [0,1].
//
// PX4 SITL command (from PX4-Autopilot repo, e.g. inside WSL2):
//   make px4_sitl none_iris
//   (PX4 connects to TCP 4560 on the simulator host; set PX4_SIM_HOSTNAME
//    to the Windows host IP when running under WSL2.)
// ===========================================================================
class PX4Bridge : public FirmwareBridge {
public:
    explicit PX4Bridge(SocketTransport::Config cfg) noexcept
        : _transport(std::move(cfg)) {}

    bool connect() noexcept override { return _transport.open(); }
    void disconnect() noexcept override { _transport.close(); }
    [[nodiscard]] bool is_connected() const noexcept override { return _transport.is_connected(); }
    [[nodiscard]] const char* firmware_name() const noexcept override { return "PX4"; }
    [[nodiscard]] uint16_t port() const noexcept override { return _transport.config().local_port; }

    bool tick(const SimState& s, ActuatorOutput& out) noexcept override;

private:
    SocketTransport _transport;
    mavlink::Parser _parser;
    uint8_t         _seq{};
    uint64_t        _tick{};

    static constexpr int GPS_DIVIDER = 25;   // ~10 Hz at 250 Hz physics
    static constexpr int HB_DIVIDER  = 250;  // ~1 Hz at 250 Hz physics

    void _send_hil_sensor(const SimState& s) noexcept;
    void _send_hil_gps(const SimState& s)    noexcept;
    void _send_heartbeat()                   noexcept;

    [[nodiscard]] bool _parse_hil_actuator_controls(
        const mavlink::Frame& f, ActuatorOutput& out) const noexcept;

    uint8_t _recv_buf[4096]{};
};

// ===========================================================================
// BetaflightBridge
//
// Protocol: MSP v2 over TCP (Betaflight SITL listens on port 5760)
// Betaflight polls the sim with MSP requests; sim responds.
// Betaflight → Sim: MSP_SET_MOTOR (8 × uint16 PWM µs)
// Sim → BF:         MSP_RAW_IMU, MSP_ATTITUDE, MSP_ALTITUDE responses
//
// Betaflight SITL command:
//   ./obj/main/betaflight_SITL.elf
// ===========================================================================
class BetaflightBridge : public FirmwareBridge {
public:
    explicit BetaflightBridge(SocketTransport::Config cfg) noexcept
        : _transport(std::move(cfg)) {}

    bool connect() noexcept override {
        bool ok = _transport.open();
        if (!ok && _transport.config().mode == TransportMode::TCP)
            return true; // non-blocking connect pending
        return ok;
    }
    void disconnect() noexcept override { _transport.close(); }
    [[nodiscard]] bool is_connected() const noexcept override { return _transport.is_connected(); }
    [[nodiscard]] const char* firmware_name() const noexcept override { return "Betaflight"; }
    [[nodiscard]] uint16_t port() const noexcept override { return _transport.config().remote_port; }

    bool tick(const SimState& s, ActuatorOutput& out) noexcept override;

private:
    SocketTransport _transport;
    msp::Parser     _parser;
    SimState        _last_state{};

    bool _handle_request(const msp::Frame& req, const SimState& s) noexcept;

    uint8_t _recv_buf[4096]{};
};

} // namespace dronesim::sitl
