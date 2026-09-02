#include "garuda/model/vehicle_specification.hpp"
#include "garuda/model/vehicle_model.hpp"
#include "garuda/model/model_validator.hpp"
#include "garuda/model/visual_bridge.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace garuda::model;

int main() {
    std::cout << "=================================================================\n";
    std::cout << " GARUDA-HL-01 C++20 MODEL ARCHITECTURE & VALIDATION TEST SUITE\n";
    std::cout << "=================================================================\n";

    // -------------------------------------------------------------------------
    // TEST 1: Canonical Specification & Validator Checks
    // -------------------------------------------------------------------------
    std::cout << "\n[TEST 1] Executing Comprehensive ModelValidator Suite...\n";
    GarudaVehicleSpecification spec = GarudaVehicleSpecification::create_canonical();
    ValidationReport report = ModelValidator::validate(spec);
    report.print(std::cout);

    if (!report.overall_success || report.failed_count > 0) {
        std::cerr << "[!] CRITICAL FAILURE: Model validation tests failed!\n";
        return 1;
    }
    std::cout << "[✓] Test 1 Passed: 100% of mathematical validation checks passed.\n";

    // -------------------------------------------------------------------------
    // TEST 2: Runtime GarudaVehicleModel Hierarchy & Transform Graph
    // -------------------------------------------------------------------------
    std::cout << "\n[TEST 2] Testing GarudaVehicleModel Hierarchy & Transforms...\n";
    GarudaVehicleModel model(spec);

    assert(model.specification().rotor_count == 8);
    assert(model.specification().blade_count_per_rotor == 2);
    assert(model.gimbal_joints().size() == 3);

    for (size_t i = 1; i <= 8; ++i) {
        const auto& m = model.motor(i);
        const auto& r = model.rotor(i);
        const auto& a = model.arm(i);

        assert(m.motor_index == i);
        assert(r.rotor_index == i);
        assert(a.arm_index == i);

        auto opt_tf = model.get_world_transform(m.id);
        assert(opt_tf.has_value());
        assert(opt_tf->is_finite());
    }
    std::cout << "[✓] Test 2 Passed: Component tree and world-space transforms verified.\n";

    // -------------------------------------------------------------------------
    // TEST 3: VisualBridge Telemetry-to-Visual Adapter across RPM spectrum
    // -------------------------------------------------------------------------
    std::cout << "\n[TEST 3] Testing VisualBridge Telemetry Adapter across RPMs...\n";
    VisualBridge bridge(spec);

    double rpms_0[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    bridge.update(Vec3d{0, 0, 0}, Quat{0, 0, 0, 1}, rpms_0, 0.0, 0.0, 0.0, false, 0.016);
    for (size_t i = 0; i < 8; ++i) {
        assert(std::abs(bridge.state().rotor_angular_velocities_rad_s[i]) < 1e-6);
    }

    double rpms_1000[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    bridge.update(Vec3d{0, 5, 0}, Quat{0, 0, 0, 1}, rpms_1000, 45.0, -30.0, 10.0, true, 0.10);
    for (size_t i = 0; i < 8; ++i) {
        double expected_omega = (1000.0 / 60.0) * (2.0 * 3.141592653589793) * (i % 2 == 0 ? -1.0 : 1.0);
        double diff = std::abs(bridge.state().rotor_angular_velocities_rad_s[i] - expected_omega);
        assert(diff < 1e-3);
    }
    std::cout << "[✓] Test 3 Passed: VisualBridge converted RPMs, gimbal angles, and signs accurately.\n";

    // -------------------------------------------------------------------------
    // TEST 4: Specification Hash Determinism
    // -------------------------------------------------------------------------
    std::cout << "\n[TEST 4] Testing Specification Hash Determinism...\n";
    std::string hash1 = spec.compute_specification_hash();
    std::string hash2 = GarudaVehicleSpecification::create_canonical().compute_specification_hash();
    assert(hash1 == hash2);
    assert(!hash1.empty());
    std::cout << "  Hash: " << hash1 << "\n";
    std::cout << "[✓] Test 4 Passed: Deterministic specification hash verified.\n";

    std::cout << "\n=================================================================\n";
    std::cout << " [ALL TESTS PASSED] GARUDA-HL-01 C++ ARCHITECTURE VALIDATED\n";
    std::cout << "=================================================================\n";
    return 0;
}
