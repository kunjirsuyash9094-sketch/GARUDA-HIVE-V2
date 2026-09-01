#include "garuda/payload/inspection_camera.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>

using namespace garuda;

int main() {
    std::cout << "============================================================\n";
    std::cout << "[TEST] Running test_inspection_camera (Phase 2 Inspection Camera Subsystem)\n";
    std::cout << "============================================================\n";

    InspectionCamera cam;

    // 1. Initial Default State
    assert(cam.status() == CameraStatus::STREAMING);
    assert(std::abs(cam.pitch_deg() - (-15.0)) < 1e-4);
    assert(std::abs(cam.yaw_deg() - 0.0) < 1e-4);
    assert(std::abs(cam.zoom_level() - 1.0) < 1e-4);
    assert(std::abs(cam.power_draw_w() - 18.0) < 1e-4);
    std::cout << "  1. Initial Camera State: Pitch=-15°, Yaw=0°, Zoom=1.0x, Power=18W -> OK\n";

    // 2. Pitch Limits & Clamping (-90° nadir to +20° horizon up)
    cam.set_gimbal(-45.0, 0.0);
    assert(std::abs(cam.pitch_deg() - (-45.0)) < 1e-4);

    cam.set_gimbal(-90.0, 0.0); // Nadir boundary
    assert(std::abs(cam.pitch_deg() - (-90.0)) < 1e-4);

    cam.set_gimbal(+20.0, 0.0); // Max up boundary
    assert(std::abs(cam.pitch_deg() - (+20.0)) < 1e-4);

    cam.set_gimbal(-135.0, 0.0); // Out of bounds down
    assert(std::abs(cam.pitch_deg() - (-90.0)) < 1e-4 && "Pitch below -90° must be clamped to -90°");

    cam.set_gimbal(+60.0, 0.0); // Out of bounds up
    assert(std::abs(cam.pitch_deg() - (+20.0)) < 1e-4 && "Pitch above +20° must be clamped to +20°");
    std::cout << "  2. Gimbal Pitch Limits Clamping [-90°, +20°] -> OK\n";

    // 3. Yaw Limits & Clamping (-180° to +180°)
    cam.set_gimbal(0.0, 90.0);
    assert(std::abs(cam.yaw_deg() - 90.0) < 1e-4);

    cam.set_gimbal(0.0, 220.0);
    assert(std::abs(cam.yaw_deg() - 180.0) < 1e-4 && "Yaw above +180° must be clamped to +180°");

    cam.set_gimbal(0.0, -250.0);
    assert(std::abs(cam.yaw_deg() - (-180.0)) < 1e-4 && "Yaw below -180° must be clamped to -180°");
    std::cout << "  3. Gimbal Yaw Limits Clamping [-180°, +180°] -> OK\n";

    // 4. Optical Zoom Limits & Clamping (1.0x to 30.0x)
    cam.set_zoom(15.0);
    assert(std::abs(cam.zoom_level() - 15.0) < 1e-4);

    cam.set_zoom(0.2);
    assert(std::abs(cam.zoom_level() - 1.0) < 1e-4 && "Zoom below 1.0x must clamp to 1.0x");

    cam.set_zoom(50.0);
    assert(std::abs(cam.zoom_level() - 30.0) < 1e-4 && "Zoom above 30.0x must clamp to 30.0x");
    std::cout << "  4. Optical Zoom Limits Clamping [1.0x, 30.0x] -> OK\n";

    // 5. Operating Statuses & Power Ledger Impacts
    cam.set_status(CameraStatus::STANDBY);
    assert(std::abs(cam.power_draw_w() - 4.0) < 1e-4);

    cam.set_status(CameraStatus::RECORDING);
    assert(std::abs(cam.power_draw_w() - 22.0) < 1e-4);

    cam.set_status(CameraStatus::OFFLINE);
    assert(std::abs(cam.power_draw_w() - 0.0) < 1e-4);

    cam.set_health(2); // Fault
    assert(cam.status() == CameraStatus::FAULT);
    assert(cam.health() == 2);
    std::cout << "  5. Camera Status & Power Consumption Ledger -> OK\n";

    std::cout << "============================================================\n";
    std::cout << "[TEST] test_inspection_camera: ALL CHECKS PASSED (100%).\n";
    std::cout << "============================================================\n";
    return 0;
}
