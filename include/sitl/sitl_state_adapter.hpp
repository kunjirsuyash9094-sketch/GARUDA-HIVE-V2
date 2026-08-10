#pragma once
#include "sitl/sitl_state.hpp"
#include "core/math_types.hpp"
#include "core/frames.hpp"
#include "aero/atmosphere.hpp"
#include "sensors/sensor_suite.hpp"
#include <optional>
#include <cmath>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
// SITLStateAdapter
//
// Translates the Godot/physics-engine body state (from DroneBody) into a
// SimState ready to be sent to firmware bridges. All firmware-facing
// quantities use aerospace conventions: NED world frame, FRD body frame
// (see core/frames.hpp for the Godot mapping).
//
// Also converts world-frame positions into absolute lat/lon/alt around a
// configurable GPS origin (flat-earth approximation, valid ~100 km).
// ---------------------------------------------------------------------------
class SITLStateAdapter {
public:
    struct OriginConfig {
        double lat_deg{-35.363261};   // ArduPilot default (Canberra RAAF)
        double lon_deg{149.165230};
        double alt_msl_m{584.0};
    };

    explicit SITLStateAdapter(OriginConfig origin = {}) noexcept
        : _origin(origin) {}

    // -----------------------------------------------------------------------
    // Primary conversion — call every physics tick.
    // sim_time_s: accumulated physics time (NOT wall clock — firmware
    //             lockstep is driven by these timestamps)
    // imu:        noisy IMU reading in Godot body axes
    // -----------------------------------------------------------------------
    [[nodiscard]] SimState build(
        double                sim_time_s,
        const RigidBodyState& body,
        const Vec3d&          wind_world,
        const Atmosphere&     atm,
        const IMUReading&     imu,
        const BaroReading&    baro,
        const std::optional<GPSReading>& gps_opt
    ) noexcept {
        using namespace frames;
        SimState s;

        s.sim_time_s   = sim_time_s;
        s.timestamp_us = static_cast<uint64_t>(sim_time_s * 1e6);

        // ---- IMU: Godot body axes -> FRD ---------------------------------
        s.accel_ms2 = godot_to_ned(imu.accel_body);
        s.gyro_rads = godot_to_ned(imu.gyro_body);

        // ---- Atmosphere ---------------------------------------------------
        const double alt_world = body.position.y; // Godot Y-up
        const AtmosphericState atm_s = atm.at_altitude(alt_world);
        s.pressure_hpa   = atm_s.pressure / 100.0;
        s.temperature_c  = atm_s.temperature - 273.15;
        s.pressure_alt_m = baro.altitude_m;

        // ---- Position: Godot world -> NED -> lat/lon ---------------------
        const Vec3d pos_ned = godot_to_ned(body.position);
        const double R_earth = 6371000.0;
        const double lat_rad = _origin.lat_deg * DEG2RAD;

        s.lat_deg   = _origin.lat_deg + (pos_ned.x / R_earth) * RAD2DEG;
        s.lon_deg   = _origin.lon_deg + (pos_ned.y / (R_earth * std::cos(lat_rad))) * RAD2DEG;
        s.alt_msl_m = _origin.alt_msl_m - pos_ned.z;
        s.position_ned_m = pos_ned;

        // ---- Velocity ------------------------------------------------------
        s.vel_ned_ms = godot_to_ned(body.velocity);

        // ---- GPS fix quality ----------------------------------------------
        s.fix_type = gps_opt ? gps_opt->fix_type : 3;
        s.sats  = 12;
        s.eph_m = 0.5;
        s.epv_m = 1.0;

        // ---- Attitude: FRD-body-to-NED quaternion + FRD body rates -------
        s.orientation      = godot_to_ned(body.orientation);
        s.angular_vel_rads = godot_to_ned(body.angular_velocity);

        // ---- Airspeed ------------------------------------------------------
        s.true_airspeed_ms = (body.velocity - wind_world).norm();

        // ---- Magnetometer: earth field (NED) rotated into FRD body -------
        const Vec3d mag_ned{21.0, 0.0, 42.0}; // µT, mid-latitude N + down
        s.mag_ut = s.orientation.conjugate().rotate(mag_ned);

        return s;
    }

    void set_origin(OriginConfig o) noexcept { _origin = o; }
    const OriginConfig& origin() const noexcept { return _origin; }

private:
    OriginConfig _origin;
};

} // namespace dronesim::sitl
