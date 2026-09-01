#pragma once
#include <string>
#include <cstdint>

namespace garuda {

enum class SimulationEventType {
    SIMULATION_STARTED,
    SIMULATION_PAUSED,
    SIMULATION_RESET,
    DRONE_ARMED,
    DRONE_DISARMED,
    TAKEOFF,
    LANDING,
    GROUND_CONTACT,
    GROUND_SEPARATION,
    MOTOR_FAILURE,
    MOTOR_RESTORED,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    SENSOR_FAILURE,
    SENSOR_RESTORED,
    COMMUNICATION_LOSS
};

struct SimulationEvent {
    uint64_t            event_id{0};
    uint64_t            tick{0};
    double              simulation_time_s{0.0};
    std::string         drone_id{};
    SimulationEventType event_type{SimulationEventType::SIMULATION_STARTED};
    std::string         payload{};

    [[nodiscard]] static std::string event_type_name(SimulationEventType type) {
        switch (type) {
            case SimulationEventType::SIMULATION_STARTED: return "SIMULATION_STARTED";
            case SimulationEventType::SIMULATION_PAUSED:  return "SIMULATION_PAUSED";
            case SimulationEventType::SIMULATION_RESET:   return "SIMULATION_RESET";
            case SimulationEventType::DRONE_ARMED:        return "DRONE_ARMED";
            case SimulationEventType::DRONE_DISARMED:     return "DRONE_DISARMED";
            case SimulationEventType::TAKEOFF:            return "TAKEOFF";
            case SimulationEventType::LANDING:            return "LANDING";
            case SimulationEventType::GROUND_CONTACT:     return "GROUND_CONTACT";
            case SimulationEventType::GROUND_SEPARATION:  return "GROUND_SEPARATION";
            case SimulationEventType::MOTOR_FAILURE:      return "MOTOR_FAILURE";
            case SimulationEventType::MOTOR_RESTORED:     return "MOTOR_RESTORED";
            case SimulationEventType::BATTERY_LOW:        return "BATTERY_LOW";
            case SimulationEventType::BATTERY_CRITICAL:   return "BATTERY_CRITICAL";
            case SimulationEventType::SENSOR_FAILURE:     return "SENSOR_FAILURE";
            case SimulationEventType::SENSOR_RESTORED:    return "SENSOR_RESTORED";
            case SimulationEventType::COMMUNICATION_LOSS: return "COMMUNICATION_LOSS";
            default:                                      return "UNKNOWN_EVENT";
        }
    }
};

} // namespace garuda
