#pragma once
#include "garuda/model/vehicle_specification.hpp"
#include <string>
#include <vector>
#include <iostream>

namespace garuda::model {

struct ValidationItem {
    std::string test_name{};
    bool        passed{false};
    std::string details{};
    double      measured_value{0.0};
    double      expected_value{0.0};
    double      tolerance{0.0};
};

struct ValidationReport {
    std::string                vehicle_name{};
    bool                       overall_success{false};
    std::vector<ValidationItem> items{};
    size_t                     passed_count{0};
    size_t                     failed_count{0};

    void print(std::ostream& os = std::cout) const;
};

class ModelValidator {
public:
    [[nodiscard]] static ValidationReport validate(const GarudaVehicleSpecification& spec) noexcept;
    static bool validate_or_exit(const GarudaVehicleSpecification& spec) noexcept;
};

} // namespace garuda::model
