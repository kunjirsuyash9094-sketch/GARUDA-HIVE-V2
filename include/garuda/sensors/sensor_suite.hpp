#pragma once
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"
#include "garuda/config/quadrotor_config.hpp"
#include "garuda/sensors/sensor_types.hpp"
#include <cstdint>
#include <cmath>
#include <optional>
#include <numbers>
#include <array>
#include <vector>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;
using dronesim::Wrench;

enum class SensorMode {
    IDEAL,       // Perfect ground truth (zero noise, zero bias, zero latency)
    SIMULATED,   // Physical noise, bias drift, quantization, update rate limits
    DEGRADED,    // 5x noise and severe bias
    FAILED       // No fix / dead output
};

/**
 * @brief Deterministic 64-bit PRNG (XorShift64Star + Box-Muller).
 */
class DeterministicGaussianNoise {
public:
    explicit DeterministicGaussianNoise(uint64_t seed = 0xCAFEBABEDEADBEEFULL) noexcept
        : _state(seed != 0 ? seed : 0x123456789ABCDEF0ULL) {}

    void set_seed(uint64_t seed) noexcept {
        _state = (seed != 0 ? seed : 0x123456789ABCDEF0ULL);
        _has_spare = false;
    }

    [[nodiscard]] double sample(double sigma = 1.0) noexcept {
        if (_has_spare) {
            _has_spare = false;
            return _spare * sigma;
        }

        double u, v, s;
        do {
            u = 2.0 * _uniform() - 1.0;
            v = 2.0 * _uniform() - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);

        double m = std::sqrt(-2.0 * std::log(s) / s);
        _spare = v * m;
        _has_spare = true;
        return u * m * sigma;
    }

private:
    uint64_t _state;
    double   _spare{0.0};
    bool     _has_spare{false};

    [[nodiscard]] double _uniform() noexcept {
        _state ^= _state >> 12;
        _state ^= _state << 25;
        _state ^= _state >> 27;
        return static_cast<double>((_state * 0x2545F4914F6CDD1DULL) >> 11) / (1ULL << 53);
    }
};

struct SyntheticIMUReading {
    Vec3d accel_body_ms2{}; // Specific force in body frame (includes gravity)
    Vec3d gyro_body_rads{}; // Angular rate in body frame
};

struct SyntheticBaroReading {
    double pressure_pa{101325.0};
    double altitude_m{0.0};
    double temperature_k{288.15};
};

struct SyntheticGPSReading {
    Vec3d  position_ned_m{};
    Vec3d  velocity_ned_ms{};
    int    fix_type{3}; // 0=None, 2=2D, 3=3D
    int    satellites{12};
    double hdop{1.0};
    double vdop{1.5};
};

struct SyntheticMagReading {
    Vec3d mag_body_ut{}; // Magnetic field in body frame (micro-Tesla)
};

struct SyntheticLidarReading {
    double distance_m{0.0};
    int    points_valid{16};
    double signal_quality{1.0};
};

struct SyntheticProximityReading {
    double distance_down_m{0.0};
    double distance_forward_m{10.0};
    bool   obstacle_detected{false};
};

/**
 * @brief Deterministic, extensible sensor suite for GARUDA-HL-01.
 */
class DeterministicSensorSuite {
public:
    static constexpr size_t NUM_SENSORS = 8;

    explicit DeterministicSensorSuite(const QuadrotorConfig& config, uint64_t base_seed = 1000) noexcept
        : _cfg(config)
        , _rng_acc(base_seed + 1)
        , _rng_gyro(base_seed + 2)
        , _rng_baro(base_seed + 3)
        , _rng_gps(base_seed + 4)
        , _rng_mag(base_seed + 5)
        , _rng_lidar(base_seed + 6)
        , _rng_prox(base_seed + 7) {
        init_descriptors();
        reset(base_seed);
    }

    void init_descriptors() noexcept {
        _descriptors[0] = { "SENSOR-IMU-01",     "Triple-Redundant Industrial IMU",   SensorType::IMU,            true, 0, SensorStatus::NOMINAL, 400.0, 0.0, 0 };
        _descriptors[1] = { "SENSOR-GNSS-01",    "Multi-Constellation RTK GNSS",      SensorType::GNSS,           true, 0, SensorStatus::NOMINAL, 10.0,  0.0, 0 };
        _descriptors[2] = { "SENSOR-BARO-01",    "Digital Barometric Altimeter",     SensorType::BAROMETER,      true, 0, SensorStatus::NOMINAL, 50.0,  0.0, 0 };
        _descriptors[3] = { "SENSOR-MAG-01",     "3-Axis Digital Magnetometer",       SensorType::MAGNETOMETER,   true, 0, SensorStatus::NOMINAL, 50.0,  0.0, 0 };
        _descriptors[4] = { "SENSOR-LIDAR-01",   "Downward Laser Altimeter LiDAR",   SensorType::LIDAR,          true, 0, SensorStatus::NOMINAL, 50.0,  0.0, 0 };
        _descriptors[5] = { "SENSOR-RGB-01",     "Primary Navigation RGB Optical",   SensorType::RGB_CAMERA,     true, 0, SensorStatus::NOMINAL, 30.0,  0.0, 0 };
        _descriptors[6] = { "SENSOR-THERMAL-01", "Core Thermal IR Microbolometer",   SensorType::THERMAL_CAMERA, true, 0, SensorStatus::NOMINAL, 30.0,  0.0, 0 };
        _descriptors[7] = { "SENSOR-PROX-01",    "Ventral Ultrasound Proximity",     SensorType::PROXIMITY,      true, 0, SensorStatus::NOMINAL, 20.0,  0.0, 0 };
    }

    void reset(uint64_t seed) noexcept {
        _rng_acc.set_seed(seed + 1);
        _rng_gyro.set_seed(seed + 2);
        _rng_baro.set_seed(seed + 3);
        _rng_gps.set_seed(seed + 4);
        _rng_mag.set_seed(seed + 5);
        _rng_lidar.set_seed(seed + 6);
        _rng_prox.set_seed(seed + 7);

        _accel_bias = { _cfg.accel_bias_dc, _cfg.accel_bias_dc * 0.9, _cfg.accel_bias_dc * 1.1 };
        _gyro_bias  = { _cfg.gyro_bias_dc,  _cfg.gyro_bias_dc * 0.8,  _cfg.gyro_bias_dc * 1.05 };
        _baro_filtered_p = 101325.0;
        _gps_timer = 0.0;
        _lidar_timer = 0.0;
        _prox_timer = 0.0;
        _mode = SensorMode::SIMULATED;

        for (auto& desc : _descriptors) {
            desc.enabled = true;
            desc.health = 0;
            desc.status = SensorStatus::NOMINAL;
            desc.sample_count = 0;
            desc.last_update_time_s = 0.0;
        }
    }

    void set_mode(SensorMode mode) noexcept {
        _mode = mode;
        for (auto& desc : _descriptors) {
            if (mode == SensorMode::FAILED) {
                desc.health = 2;
                desc.status = SensorStatus::FAULT;
            } else if (mode == SensorMode::DEGRADED) {
                desc.health = 1;
                desc.status = SensorStatus::DEGRADED;
            } else {
                desc.health = 0;
                desc.status = SensorStatus::NOMINAL;
            }
        }
    }

    [[nodiscard]] SensorMode mode() const noexcept { return _mode; }

    void set_sensor_enabled(size_t index, bool enabled) noexcept {
        if (index < NUM_SENSORS) {
            _descriptors[index].enabled = enabled;
            _descriptors[index].status = enabled ? SensorStatus::NOMINAL : SensorStatus::OFFLINE;
        }
    }

    void set_sensor_status(size_t index, SensorStatus status) noexcept {
        if (index < NUM_SENSORS) {
            _descriptors[index].status = status;
            _descriptors[index].health = (status == SensorStatus::FAULT) ? 2 : ((status == SensorStatus::DEGRADED) ? 1 : 0);
        }
    }

    [[nodiscard]] const std::array<SensorDescriptor, NUM_SENSORS>& descriptors() const noexcept {
        return _descriptors;
    }

    [[nodiscard]] const SensorDescriptor& descriptor(size_t index) const noexcept {
        return _descriptors[std::min(index, NUM_SENSORS - 1)];
    }

    // -------------------------------------------------------------------------
    // Synthetic Measurement Generators (Deterministic & Physical)
    // -------------------------------------------------------------------------
    [[nodiscard]] SyntheticIMUReading measure_imu(
        const Vec3d& true_accel_world,
        const Vec3d& true_omega_bf,
        const Quat&  orient,
        double       gravity_m_s2,
        double       dt,
        double       sim_time_s = 0.0
    ) noexcept {
        auto& d = _descriptors[0];
        d.last_update_time_s = sim_time_s;
        d.sample_count++;

        SyntheticIMUReading r{};
        if (!d.enabled || d.status == SensorStatus::FAULT || _mode == SensorMode::FAILED) {
            return r;
        }

        // Specific force: f_world = a_world - g_world (Godot Y-up: g_world = [0, -g, 0])
        Vec3d g_world{0.0, -gravity_m_s2, 0.0};
        Vec3d specific_force_world = true_accel_world - g_world;
        Vec3d specific_force_body = orient.conjugate().rotate(specific_force_world);

        if (_mode == SensorMode::IDEAL) {
            r.accel_body_ms2 = specific_force_body;
            r.gyro_body_rads = true_omega_bf;
            return r;
        }

        double noise_mult = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 5.0 : 1.0;

        // Gyro bias random walk drift
        double drift_scale = std::sqrt(std::max(dt, 1e-4)) * _cfg.gyro_bias_drift_rate * noise_mult;
        _gyro_bias.x += _rng_gyro.sample(drift_scale);
        _gyro_bias.y += _rng_gyro.sample(drift_scale);
        _gyro_bias.z += _rng_gyro.sample(drift_scale);

        // Accelerometer measurement
        double acc_sigma = _cfg.accel_noise_density / std::sqrt(std::max(dt, 1e-4)) * noise_mult;
        r.accel_body_ms2 = specific_force_body + _accel_bias + Vec3d{
            _rng_acc.sample(acc_sigma),
            _rng_acc.sample(acc_sigma),
            _rng_acc.sample(acc_sigma)
        };

        // Gyroscope measurement
        double gyro_sigma = _cfg.gyro_noise_density / std::sqrt(std::max(dt, 1e-4)) * noise_mult;
        r.gyro_body_rads = true_omega_bf + _gyro_bias + Vec3d{
            _rng_gyro.sample(gyro_sigma),
            _rng_gyro.sample(gyro_sigma),
            _rng_gyro.sample(gyro_sigma)
        };

        return r;
    }

    [[nodiscard]] SyntheticBaroReading measure_baro(
        const dronesim::AtmosphericState& atm,
        double true_alt_m,
        double dt,
        double sim_time_s = 0.0
    ) noexcept {
        auto& d = _descriptors[2];
        d.last_update_time_s = sim_time_s;
        d.sample_count++;

        SyntheticBaroReading r{};
        if (!d.enabled || d.status == SensorStatus::FAULT || _mode == SensorMode::FAILED) {
            return r;
        }

        if (_mode == SensorMode::IDEAL) {
            r.pressure_pa = atm.pressure;
            r.altitude_m = true_alt_m;
            r.temperature_k = atm.temperature;
            return r;
        }

        double noise_mult = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 5.0 : 1.0;

        // 1st-order acoustic lag filter on pressure
        double decay = std::exp(-dt / std::max(1e-4, _cfg.baro_filter_tau_s));
        _baro_filtered_p = _baro_filtered_p * decay + atm.pressure * (1.0 - decay);

        double p_meas = _baro_filtered_p + _rng_baro.sample(_cfg.baro_noise_pa * noise_mult);
        r.pressure_pa = p_meas;
        r.temperature_k = atm.temperature;

        // ISA inversion for barometric altitude
        const double T0 = 288.15, L = 0.0065, P0 = 101325.0, R_gas = 287.058, g = 9.80665;
        r.altitude_m = (T0 / L) * (1.0 - std::pow(std::max(p_meas, 1000.0) / P0, (R_gas * L) / g));

        return r;
    }

    [[nodiscard]] std::optional<SyntheticGPSReading> measure_gps(
        const Vec3d& true_pos_world,
        const Vec3d& true_vel_world,
        double dt,
        double sim_time_s = 0.0
    ) noexcept {
        auto& d = _descriptors[1];
        if (!d.enabled || d.status == SensorStatus::FAULT || _mode == SensorMode::FAILED) {
            return std::nullopt;
        }

        _gps_timer += dt;
        double gps_period = 1.0 / std::max(0.1, _cfg.gps_update_rate_hz);
        if (_gps_timer < gps_period) {
            return std::nullopt;
        }
        _gps_timer -= gps_period;
        d.last_update_time_s = sim_time_s;
        d.sample_count++;

        SyntheticGPSReading r{};
        // Map Godot world (Y-up, -Z north, X east) to NED
        Vec3d true_pos_ned{ -true_pos_world.z, true_pos_world.x, -true_pos_world.y };
        Vec3d true_vel_ned{ -true_vel_world.z, true_vel_world.x, -true_vel_world.y };

        if (_mode == SensorMode::IDEAL) {
            r.position_ned_m = true_pos_ned;
            r.velocity_ned_ms = true_vel_ned;
            r.fix_type = 3;
            r.satellites = 14;
            r.hdop = 0.8;
            r.vdop = 1.2;
            return r;
        }

        double noise_mult = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 5.0 : 1.0;
        r.position_ned_m = true_pos_ned + Vec3d{
            _rng_gps.sample(_cfg.gps_pos_noise_h_m * noise_mult),
            _rng_gps.sample(_cfg.gps_pos_noise_h_m * noise_mult),
            _rng_gps.sample(_cfg.gps_pos_noise_v_m * noise_mult)
        };
        r.velocity_ned_ms = true_vel_ned + Vec3d{
            _rng_gps.sample(_cfg.gps_vel_noise_m_s * noise_mult),
            _rng_gps.sample(_cfg.gps_vel_noise_m_s * noise_mult),
            _rng_gps.sample(_cfg.gps_vel_noise_m_s * noise_mult)
        };
        r.fix_type = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 2 : 3;
        r.satellites = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 6 : 14;
        r.hdop = 1.0 * noise_mult;
        r.vdop = 1.5 * noise_mult;

        return r;
    }

    [[nodiscard]] SyntheticMagReading measure_mag(const Quat& orient, double /*dt*/, double sim_time_s = 0.0) noexcept {
        auto& d = _descriptors[3];
        d.last_update_time_s = sim_time_s;
        d.sample_count++;

        SyntheticMagReading r{};
        if (!d.enabled || d.status == SensorStatus::FAULT || _mode == SensorMode::FAILED) return r;

        // Earth magnetic field vector
        Vec3d b_ned{ 21.0, 0.0, 42.0 };
        Quat q_frd{ -orient.z, orient.x, -orient.y, orient.w };
        Vec3d b_body = q_frd.conjugate().rotate(b_ned);

        if (_mode == SensorMode::IDEAL) {
            r.mag_body_ut = b_body;
            return r;
        }

        double noise_mult = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 5.0 : 1.0;
        r.mag_body_ut = b_body + Vec3d{
            _rng_mag.sample(0.2 * noise_mult),
            _rng_mag.sample(0.2 * noise_mult),
            _rng_mag.sample(0.2 * noise_mult)
        };
        return r;
    }

    [[nodiscard]] SyntheticLidarReading measure_lidar(double true_altitude_agl_m, double dt, double sim_time_s = 0.0) noexcept {
        auto& d = _descriptors[4];
        d.last_update_time_s = sim_time_s;
        d.sample_count++;

        SyntheticLidarReading r{};
        if (!d.enabled || d.status == SensorStatus::FAULT || _mode == SensorMode::FAILED) {
            r.signal_quality = 0.0;
            return r;
        }

        double noise_mult = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 5.0 : 1.0;
        double noise = (_mode == SensorMode::IDEAL) ? 0.0 : _rng_lidar.sample(0.015 * noise_mult);
        r.distance_m = std::max(0.0, true_altitude_agl_m + noise);
        r.points_valid = (d.status == SensorStatus::DEGRADED) ? 8 : 16;
        r.signal_quality = (d.status == SensorStatus::DEGRADED) ? 0.6 : 1.0;
        return r;
    }

    [[nodiscard]] SyntheticProximityReading measure_proximity(double true_altitude_agl_m, double /*dt*/, double sim_time_s = 0.0) noexcept {
        auto& d = _descriptors[7];
        d.last_update_time_s = sim_time_s;
        d.sample_count++;

        SyntheticProximityReading r{};
        if (!d.enabled || d.status == SensorStatus::FAULT || _mode == SensorMode::FAILED) {
            return r;
        }

        double noise_mult = (d.status == SensorStatus::DEGRADED || _mode == SensorMode::DEGRADED) ? 5.0 : 1.0;
        double noise = (_mode == SensorMode::IDEAL) ? 0.0 : _rng_prox.sample(0.02 * noise_mult);
        r.distance_down_m = std::max(0.0, true_altitude_agl_m + noise);
        r.distance_forward_m = 10.0;
        r.obstacle_detected = (r.distance_down_m < 0.35);
        return r;
    }

private:
    QuadrotorConfig                             _cfg;
    SensorMode                                  _mode{SensorMode::SIMULATED};
    std::array<SensorDescriptor, NUM_SENSORS>  _descriptors{};

    DeterministicGaussianNoise  _rng_acc;
    DeterministicGaussianNoise  _rng_gyro;
    DeterministicGaussianNoise  _rng_baro;
    DeterministicGaussianNoise  _rng_gps;
    DeterministicGaussianNoise  _rng_mag;
    DeterministicGaussianNoise  _rng_lidar;
    DeterministicGaussianNoise  _rng_prox;

    Vec3d  _accel_bias{0.01, 0.01, 0.01};
    Vec3d  _gyro_bias{0.001, 0.001, 0.001};
    double _baro_filtered_p{101325.0};
    double _gps_timer{0.0};
    double _lidar_timer{0.0};
    double _prox_timer{0.0};
};

} // namespace garuda
