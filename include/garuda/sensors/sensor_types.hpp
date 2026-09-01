#pragma once
#include "core/math_types.hpp"
#include <string>
#include <string_view>
#include <cstdint>

namespace garuda {

using dronesim::Vec3d;
using dronesim::Quat;

enum class SensorType : int {
    IMU = 0,
    GNSS = 1,
    BAROMETER = 2,
    MAGNETOMETER = 3,
    LIDAR = 4,
    RGB_CAMERA = 5,
    THERMAL_CAMERA = 6,
    PROXIMITY = 7
};

enum class SensorStatus : int {
    OFFLINE = 0,
    INITIALIZING = 1,
    NOMINAL = 2,
    DEGRADED = 3,
    FAULT = 4
};

struct SensorDescriptor {
    std::string   sensor_id{"SENSOR-NONE"};
    std::string   name{"Unknown Sensor"};
    SensorType    type{SensorType::IMU};
    bool          enabled{true};
    int           health{0};            // 0=NOMINAL, 1=DEGRADED, 2=FAULT
    SensorStatus  status{SensorStatus::NOMINAL};
    double        update_rate_hz{400.0};
    double        last_update_time_s{0.0};
    uint64_t      sample_count{0};
};

[[nodiscard]] inline std::string_view sensor_type_to_string(SensorType t) noexcept {
    switch (t) {
        case SensorType::IMU: return "IMU";
        case SensorType::GNSS: return "GNSS";
        case SensorType::BAROMETER: return "BAROMETER";
        case SensorType::MAGNETOMETER: return "MAGNETOMETER";
        case SensorType::LIDAR: return "LIDAR";
        case SensorType::RGB_CAMERA: return "RGB_CAMERA";
        case SensorType::THERMAL_CAMERA: return "THERMAL_CAMERA";
        case SensorType::PROXIMITY: return "PROXIMITY";
        default: return "UNKNOWN";
    }
}

[[nodiscard]] inline std::string_view sensor_status_to_string(SensorStatus s) noexcept {
    switch (s) {
        case SensorStatus::OFFLINE: return "OFFLINE";
        case SensorStatus::INITIALIZING: return "INITIALIZING";
        case SensorStatus::NOMINAL: return "NOMINAL";
        case SensorStatus::DEGRADED: return "DEGRADED";
        case SensorStatus::FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

} // namespace garuda
