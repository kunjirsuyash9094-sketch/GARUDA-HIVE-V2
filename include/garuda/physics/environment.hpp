#pragma once
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

/**
 * @brief Atmospheric and Environmental State for GARUDA HIVE V2.
 */
struct EnvironmentState {
    double gravity_m_s2{9.80665};
    double ground_elevation_m{0.0};
    Vec3d  wind_steady_world{0.0, 0.0, 0.0};
    double turbulence_intensity{0.0};
    double air_density_sea_level{1.225};
    double temperature_sea_level_k{288.15};
    double pressure_sea_level_pa{101325.0};
};

class EnvironmentSystem {
public:
    explicit EnvironmentSystem(EnvironmentState state = {}) noexcept
        : _state(state) {
        _atm.set_wind_global(_state.wind_steady_world);
        _atm.set_turbulence({ _state.turbulence_intensity, 200.0 });
    }

    void set_state(EnvironmentState state) noexcept {
        _state = state;
        _atm.set_wind_global(_state.wind_steady_world);
        _atm.set_turbulence({ _state.turbulence_intensity, 200.0 });
    }

    void set_wind(Vec3d wind) noexcept {
        _state.wind_steady_world = wind;
        _atm.set_wind_global(wind);
    }

    void set_turbulence_intensity(double intensity) noexcept {
        _state.turbulence_intensity = intensity;
        _atm.set_turbulence({ intensity, 200.0 });
    }

    void set_ground_elevation(double h) noexcept {
        _state.ground_elevation_m = h;
    }

    [[nodiscard]] const EnvironmentState& state() const noexcept { return _state; }
    [[nodiscard]] const dronesim::Atmosphere& atmosphere_model() const noexcept { return _atm; }

    [[nodiscard]] dronesim::AtmosphericState at_altitude(double alt_m) const noexcept {
        return _atm.at_altitude(alt_m);
    }

    [[nodiscard]] Vec3d sample_wind(double alt_m, double dt) noexcept {
        return _atm.sample_wind(alt_m, dt);
    }

    [[nodiscard]] double ground_height() const noexcept {
        return _state.ground_elevation_m;
    }

    [[nodiscard]] double gravity() const noexcept {
        return _state.gravity_m_s2;
    }

private:
    EnvironmentState      _state;
    dronesim::Atmosphere  _atm;
};

} // namespace garuda
