#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "sitl/firmware_bridge.hpp"
#include "sitl/sitl_state_adapter.hpp"
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"
#include "sensors/sensor_suite.hpp"

#include <memory>
#include <optional>

namespace dronesim {

// ---------------------------------------------------------------------------
// SITLManager — Godot Node (not physics node)
//
// Add as a child of the DroneBody (auto-detected) or anywhere in the scene
// (then set DroneBody.sitl_manager_path). Exposes per-firmware enable/port
// properties in the inspector. Each physics tick DroneBody calls
// physics_tick(...) and receives ActuatorOutput to feed into RotorArray.
//
// Inspector properties:
//   enabled_ardupilot   (bool, default false)
//   enabled_px4         (bool, default false)
//   enabled_betaflight  (bool, default false)
//   port_ardupilot      (int, default 9002 — sim is UDP server, AP JSON)
//   port_px4            (int, default 4560 — sim is TCP server, MAVLink HIL)
//   port_betaflight     (int, default 5760 — sim is TCP client, MSP)
//   gps_origin_lat/lon/alt
//
// Active firmware priority (first with a fresh frame wins per tick):
//   Betaflight > PX4 > ArduPilot
// Override with set_active_firmware("ardupilot"|"px4"|"betaflight"|"auto")
// ---------------------------------------------------------------------------
class SITLManager : public godot::Node {
    GDCLASS(SITLManager, godot::Node)

public:
    SITLManager();
    ~SITLManager() override;

    void _ready()   override;
    void _process(double delta) override;

    // Called from DroneBody::_integrate_forces every physics tick.
    // sim_time_s = accumulated physics time (drives firmware lockstep).
    // Returns true if a new actuator frame was received from any firmware.
    bool physics_tick(
        double                sim_time_s,
        const RigidBodyState& body,
        const Vec3d&          wind_world,
        const Atmosphere&     atm,
        const IMUReading&     imu,
        const BaroReading&    baro,
        const std::optional<GPSReading>& gps,
        sitl::ActuatorOutput& out
    ) noexcept;

    // ---- GDScript API ------------------------------------------------------
    void set_active_firmware(godot::String name); // "auto","ardupilot","px4","betaflight"
    godot::String get_active_firmware() const;

    godot::Dictionary get_sitl_status() const; // per-firmware connected / port / last_rx_ms

    // Restart a specific bridge (useful when changing ports at runtime)
    void reconnect_ardupilot();
    void reconnect_px4();
    void reconnect_betaflight();

    // ---- Property setters/getters -----------------------------------------
    void set_enabled_ardupilot(bool v);
    bool get_enabled_ardupilot() const;
    void set_enabled_px4(bool v);
    bool get_enabled_px4() const;
    void set_enabled_betaflight(bool v);
    bool get_enabled_betaflight() const;

    void   set_port_ardupilot(int p);
    int    get_port_ardupilot() const;
    void   set_port_px4(int p);
    int    get_port_px4() const;
    void   set_port_betaflight(int p);
    int    get_port_betaflight() const;

    void   set_gps_origin_lat(double v);
    double get_gps_origin_lat() const;
    void   set_gps_origin_lon(double v);
    double get_gps_origin_lon() const;
    void   set_gps_origin_alt(double v);
    double get_gps_origin_alt() const;

    static void _bind_methods();

private:
    void _build_bridges();
    [[nodiscard]] sitl::SocketTransport::Config _make_udp_server_config(
        uint16_t local_port) const noexcept;
    [[nodiscard]] sitl::SocketTransport::Config _make_tcp_server_config(
        uint16_t local_port) const noexcept;
    [[nodiscard]] sitl::SocketTransport::Config _make_tcp_client_config(
        uint16_t remote_port) const noexcept;

    // Bridges
    std::unique_ptr<sitl::ArduPilotBridge>  _ap;
    std::unique_ptr<sitl::PX4Bridge>        _px4;
    std::unique_ptr<sitl::BetaflightBridge> _bf;

    sitl::SITLStateAdapter _adapter;

    // Inspector config
    bool   _en_ap{false}, _en_px4{false}, _en_bf{false};
    int    _port_ap{9002}, _port_px4{4560}, _port_bf{5760};
    sitl::SITLStateAdapter::OriginConfig _origin{};

    // Priority override: -1 = auto
    int    _forced_firmware{-1}; // 0=AP,1=PX4,2=BF

    // Per-bridge last-rx timestamps (ms since epoch) for status display
    uint64_t _last_rx_ap{}, _last_rx_px4{}, _last_rx_bf{};

    bool _built{false};
};

} // namespace dronesim
