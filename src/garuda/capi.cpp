#define GARUDA_EXPORTS
#include "garuda/capi.h"
#include "garuda/core/simulation_world.hpp"
#include <cstring>
#include <vector>

using namespace garuda;

static SimulationWorld* get_world(void* handle) {
    return reinterpret_cast<SimulationWorld*>(handle);
}

void* garuda_world_create(uint64_t seed, double dt) {
    return new SimulationWorld(seed, dt);
}

void garuda_world_destroy(void* world_handle) {
    if (world_handle) {
        delete get_world(world_handle);
    }
}

void garuda_world_reset(void* world_handle) {
    if (auto* w = get_world(world_handle)) {
        w->reset();
    }
}

int32_t garuda_world_add_drone(void* world_handle, const char* drone_id, double spawn_x, double spawn_y, double spawn_z) {
    if (auto* w = get_world(world_handle)) {
        auto* d = w->add_drone(drone_id, {}, {spawn_x, spawn_y, spawn_z});
        return d != nullptr ? 1 : 0;
    }
    return 0;
}

int32_t garuda_world_drone_count(void* world_handle) {
    if (auto* w = get_world(world_handle)) {
        return static_cast<int32_t>(w->drone_count());
    }
    return 0;
}

void garuda_world_step(void* world_handle) {
    if (auto* w = get_world(world_handle)) {
        w->step();
    }
}

void garuda_world_step_n(void* world_handle, uint32_t step_count) {
    if (auto* w = get_world(world_handle)) {
        w->step_n(step_count);
    }
}

int32_t garuda_drone_arm(void* world_handle, const char* drone_id) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->arm();
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_disarm(void* world_handle, const char* drone_id) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->disarm();
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_set_attitude(void* world_handle, const char* drone_id, double roll_rad, double pitch_rad, double yaw_rate_rads, double throttle) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->set_attitude_setpoint(roll_rad, pitch_rad, yaw_rate_rads, throttle);
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_set_motors(void* world_handle, const char* drone_id, const double* throttles, int32_t count) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            std::vector<double> th(throttles, throttles + count);
            d->set_direct_motor_throttles(th);
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_attach_payload(void* world_handle, const char* drone_id, int32_t payload_type) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            bool ok = d->attach_payload(static_cast<PayloadType>(payload_type));
            return ok ? 1 : 0;
        }
    }
    return 0;
}

int32_t garuda_drone_detach_payload(void* world_handle, const char* drone_id) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            bool ok = d->detach_payload();
            return ok ? 1 : 0;
        }
    }
    return 0;
}

int32_t garuda_drone_set_gimbal(void* world_handle, const char* drone_id, double pitch_deg, double yaw_deg, double zoom) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->set_camera_gimbal(pitch_deg, yaw_deg);
            d->set_camera_zoom(zoom);
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_set_sensor_enabled(void* world_handle, const char* drone_id, int32_t sensor_idx, int32_t enabled) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->set_sensor_enabled(static_cast<size_t>(sensor_idx), enabled != 0);
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_set_sensor_status(void* world_handle, const char* drone_id, int32_t sensor_idx, int32_t status) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->set_sensor_status(static_cast<size_t>(sensor_idx), static_cast<SensorStatus>(status));
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_get_sensor_status(void* world_handle, const char* drone_id, int32_t sensor_idx) {
    if (auto* w = get_world(world_handle)) {
        if (const auto* d = w->get_drone(drone_id)) {
            if (sensor_idx >= 0 && static_cast<size_t>(sensor_idx) < DeterministicSensorSuite::NUM_SENSORS) {
                return static_cast<int32_t>(d->sensor_suite().descriptors()[sensor_idx].status);
            }
        }
    }
    return -1;
}

int32_t garuda_drone_get_vehicle_health(void* world_handle, const char* drone_id) {
    if (auto* w = get_world(world_handle)) {
        if (const auto* d = w->get_drone(drone_id)) {
            return static_cast<int32_t>(d->health_report().overall);
        }
    }
    return -1;
}

int32_t garuda_drone_inject_motor_failure(void* world_handle, const char* drone_id, int32_t motor_index, int32_t failure_type, double degradation) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->inject_motor_failure(static_cast<size_t>(motor_index), static_cast<MotorHealthState>(failure_type), degradation);
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_inject_sensor_failure(void* world_handle, const char* drone_id, int32_t sensor_mode) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->inject_sensor_failure(static_cast<SensorMode>(sensor_mode));
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_inject_battery_degradation(void* world_handle, const char* drone_id, double capacity_factor, double resistance_mult) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->inject_battery_degradation(capacity_factor, resistance_mult);
            return 1;
        }
    }
    return 0;
}

int32_t garuda_drone_reset_failures(void* world_handle, const char* drone_id) {
    if (auto* w = get_world(world_handle)) {
        if (auto* d = w->get_drone(drone_id)) {
            d->reset_all_failures();
            return 1;
        }
    }
    return 0;
}

int32_t garuda_world_get_telemetry(void* world_handle, const char* drone_id, GarudaDroneTelemetryPOD* out_pod) {
    if (!out_pod) return 0;
    if (auto* w = get_world(world_handle)) {
        if (const auto* d = w->get_drone(drone_id)) {
            const auto& t = d->telemetry();
            std::memset(out_pod, 0, sizeof(GarudaDroneTelemetryPOD));

            std::strncpy(out_pod->drone_id, t.drone_id.c_str(), sizeof(out_pod->drone_id) - 1);
            out_pod->tick = t.simulation_tick;
            out_pod->time_s = t.simulation_time_s;

            out_pod->pos_x = t.position_world.x;
            out_pod->pos_y = t.position_world.y;
            out_pod->pos_z = t.position_world.z;
            out_pod->vel_x = t.velocity_world.x;
            out_pod->vel_y = t.velocity_world.y;
            out_pod->vel_z = t.velocity_world.z;
            out_pod->acc_x = t.acceleration_world.x;
            out_pod->acc_y = t.acceleration_world.y;
            out_pod->acc_z = t.acceleration_world.z;
            out_pod->altitude = t.altitude_m;
            out_pod->ground_speed = t.ground_speed_ms;
            out_pod->vertical_speed = t.vertical_speed_ms;

            out_pod->quat_x = t.orientation.x;
            out_pod->quat_y = t.orientation.y;
            out_pod->quat_z = t.orientation.z;
            out_pod->quat_w = t.orientation.w;
            out_pod->roll_deg = t.euler_rpy_deg.x;
            out_pod->pitch_deg = t.euler_rpy_deg.y;
            out_pod->yaw_deg = t.euler_rpy_deg.z;
            out_pod->gyro_x = t.angular_velocity_rads.x;
            out_pod->gyro_y = t.angular_velocity_rads.y;
            out_pod->gyro_z = t.angular_velocity_rads.z;

            out_pod->total_thrust = t.total_thrust_n;
            out_pod->twr = t.thrust_to_weight_ratio;
            out_pod->thrust_margin = t.available_thrust_margin_n;

            for (size_t i = 0; i < 8 && i < t.motor_rpm.size(); ++i) {
                out_pod->motor_rpm[i] = t.motor_rpm[i];
                out_pod->motor_thrust[i] = t.motor_thrust_n[i];
                out_pod->motor_power[i] = t.motor_power_w[i];
                out_pod->motor_temp[i] = t.motor_temp_c[i];
                out_pod->motor_health[i] = t.motor_health[i];
            }

            out_pod->battery_v_term = t.battery_voltage_terminal;
            out_pod->battery_v_ocv = t.battery_voltage_ocv;
            out_pod->battery_current = t.battery_current_amps;
            out_pod->battery_soc = t.battery_soc;
            out_pod->battery_power = t.battery_power_w;
            out_pod->battery_temp = t.battery_temp_c;
            out_pod->energy_consumed_j = t.energy_consumed_joules;
            out_pod->energy_remaining_j = t.energy_remaining_joules;

            for (size_t i = 0; i < 6 && i < t.cell_voltages.size(); ++i) {
                out_pod->cell_v[i] = t.cell_voltages[i];
            }

            std::strncpy(out_pod->payload_id, t.payload_id.c_str(), sizeof(out_pod->payload_id) - 1);
            std::strncpy(out_pod->payload_name, t.payload_name.c_str(), sizeof(out_pod->payload_name) - 1);
            std::strncpy(out_pod->payload_category, t.payload_category.c_str(), sizeof(out_pod->payload_category) - 1);
            out_pod->payload_type = t.payload_type;
            out_pod->payload_mass = t.payload_mass_kg;
            out_pod->payload_attached = t.payload_attached ? 1 : 0;
            out_pod->payload_power = t.payload_power_w;
            out_pod->payload_state = t.payload_state;
            out_pod->payload_health = t.payload_health;

            out_pod->camera_status = t.camera_status;
            out_pod->camera_health = t.camera_health;
            out_pod->camera_pitch = t.camera_pitch_deg;
            out_pod->camera_yaw = t.camera_yaw_deg;
            out_pod->camera_zoom = t.camera_zoom;

            for (size_t i = 0; i < 8 && i < t.sensor_status.size(); ++i) {
                out_pod->sensor_status[i] = t.sensor_status[i];
                out_pod->sensor_health[i] = t.sensor_health[i];
            }
            out_pod->lidar_distance = t.lidar_distance_m;
            out_pod->prox_distance = t.proximity_distance_m;

            out_pod->air_density = t.air_density_kgm3;
            out_pod->ground_effect_factor = t.ground_effect_factor;
            out_pod->vrs_active = t.vrs_active ? 1 : 0;
            out_pod->vrs_severity = t.vrs_severity;

            out_pod->armed = t.armed ? 1 : 0;
            out_pod->in_ground_contact = t.in_ground_contact ? 1 : 0;
            out_pod->low_voltage_warn = t.low_voltage_warning ? 1 : 0;
            out_pod->critical_cutoff = t.critical_battery_cutoff ? 1 : 0;
            out_pod->vehicle_health = static_cast<int32_t>(t.vehicle_health);
            out_pod->vehicle_state = static_cast<int32_t>(t.vehicle_state);

            return 1;
        }
    }
    return 0;
}

uint64_t garuda_world_get_state_hash(void* world_handle) {
    if (auto* w = get_world(world_handle)) {
        return w->compute_world_state_hash();
    }
    return 0;
}

uint64_t garuda_world_get_tick(void* world_handle) {
    if (auto* w = get_world(world_handle)) {
        return w->clock().tick();
    }
    return 0;
}

double garuda_world_get_time(void* world_handle) {
    if (auto* w = get_world(world_handle)) {
        return w->clock().time_s();
    }
    return 0.0;
}
