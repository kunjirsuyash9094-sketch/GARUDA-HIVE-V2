#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace garuda {

enum class CameraStatus : int {
    OFFLINE = 0,
    STANDBY = 1,
    STREAMING = 2,
    RECORDING = 3,
    FAULT = 4
};

enum class CameraResolution : int {
    RES_4K_60FPS = 0,
    RES_1080P_60FPS = 1,
    RES_720P_120FPS = 2,
    RES_THERMAL_RAW = 3
};

struct InspectionCameraState {
    CameraStatus     status{CameraStatus::STREAMING};
    CameraResolution resolution{CameraResolution::RES_4K_60FPS};
    double           pitch_deg{-15.0};      // -90° (nadir) to +20° (horizon up)
    double           yaw_deg{0.0};          // -180° to +180°
    double           zoom_level{1.0};       // 1.0x to 30.0x optical zoom
    double           power_draw_w{18.0};    // Electrical power consumed (W)
    int              health{0};             // 0=NOMINAL, 1=DEGRADED, 2=FAULT
    bool             stabilization_active{true};
};

class InspectionCamera {
public:
    // Physical & Software Limits
    static constexpr double MIN_PITCH_DEG = -90.0;
    static constexpr double MAX_PITCH_DEG =  20.0;
    static constexpr double MIN_YAW_DEG   = -180.0;
    static constexpr double MAX_YAW_DEG   =  180.0;
    static constexpr double MIN_ZOOM      =   1.0;
    static constexpr double MAX_ZOOM      =  30.0;

    InspectionCamera() noexcept {
        reset();
    }

    void reset() noexcept {
        _state = InspectionCameraState{};
        _state.status = CameraStatus::STREAMING;
        _state.resolution = CameraResolution::RES_4K_60FPS;
        _state.pitch_deg = -15.0;
        _state.yaw_deg = 0.0;
        _state.zoom_level = 1.0;
        _state.power_draw_w = 18.0;
        _state.health = 0;
        _state.stabilization_active = true;
    }

    bool set_gimbal(double pitch_deg, double yaw_deg) noexcept {
        _state.pitch_deg = std::clamp(pitch_deg, MIN_PITCH_DEG, MAX_PITCH_DEG);
        _state.yaw_deg = std::clamp(yaw_deg, MIN_YAW_DEG, MAX_YAW_DEG);
        return true;
    }

    bool set_zoom(double zoom) noexcept {
        _state.zoom_level = std::clamp(zoom, MIN_ZOOM, MAX_ZOOM);
        return true;
    }

    void set_status(CameraStatus status) noexcept {
        _state.status = status;
        switch (status) {
            case CameraStatus::OFFLINE:
                _state.power_draw_w = 0.0;
                break;
            case CameraStatus::STANDBY:
                _state.power_draw_w = 4.0;
                break;
            case CameraStatus::STREAMING:
                _state.power_draw_w = 18.0;
                break;
            case CameraStatus::RECORDING:
                _state.power_draw_w = 22.0;
                break;
            case CameraStatus::FAULT:
                _state.power_draw_w = 2.0;
                _state.health = 2;
                break;
        }
    }

    void set_health(int health) noexcept {
        _state.health = std::clamp(health, 0, 2);
        if (_state.health == 2) {
            _state.status = CameraStatus::FAULT;
        }
    }

    [[nodiscard]] const InspectionCameraState& state() const noexcept { return _state; }
    [[nodiscard]] double pitch_deg() const noexcept { return _state.pitch_deg; }
    [[nodiscard]] double yaw_deg() const noexcept { return _state.yaw_deg; }
    [[nodiscard]] double zoom_level() const noexcept { return _state.zoom_level; }
    [[nodiscard]] double power_draw_w() const noexcept { return _state.power_draw_w; }
    [[nodiscard]] CameraStatus status() const noexcept { return _state.status; }
    [[nodiscard]] int health() const noexcept { return _state.health; }

private:
    InspectionCameraState _state{};
};

} // namespace garuda
