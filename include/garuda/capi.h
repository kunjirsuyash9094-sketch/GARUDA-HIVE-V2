#pragma once
#include <stdint.h>

#ifdef _WIN32
  #ifdef GARUDA_EXPORTS
    #define GARUDA_API __declspec(dllexport)
  #else
    #define GARUDA_API __declspec(dllimport)
  #endif
#else
  #define GARUDA_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plain Old Data (POD) struct representing full Phase 2 drone telemetry across C-ABI / FFI.
 */
#pragma pack(push, 8)
typedef struct {
    char        drone_id[32];
    uint64_t    tick;
    double      time_s;

    // Kinematics
    double      pos_x, pos_y, pos_z;
    double      vel_x, vel_y, vel_z;
    double      acc_x, acc_y, acc_z;
    double      altitude;
    double      ground_speed;
    double      vertical_speed;

    // Attitude
    double      quat_x, quat_y, quat_z, quat_w;
    double      roll_deg, pitch_deg, yaw_deg;
    double      gyro_x, gyro_y, gyro_z;

    // 8-Rotor Propulsion
    double      total_thrust;
    double      twr;
    double      thrust_margin;
    double      motor_rpm[8];
    double      motor_thrust[8];
    double      motor_power[8];
    double      motor_temp[8];
    int32_t     motor_health[8];

    // Electrical (6S LiPo)
    double      battery_v_term;
    double      battery_v_ocv;
    double      battery_current;
    double      battery_soc;
    double      battery_power;
    double      battery_temp;
    double      energy_consumed_j;
    double      energy_remaining_j;
    double      cell_v[6];

    // Modular Payload & Inspection Camera Subsystems
    char        payload_id[32];
    char        payload_name[64];
    char        payload_category[32];
    int32_t     payload_type;
    double      payload_mass;
    int32_t     payload_attached;
    double      payload_power;
    int32_t     payload_state;
    int32_t     payload_health;

    // Inspection Camera State
    int32_t     camera_status;
    int32_t     camera_health;
    double      camera_pitch;
    double      camera_yaw;
    double      camera_zoom;

    // Extensible Sensor Suite (8 Sensors: IMU, GNSS, BARO, MAG, LIDAR, RGB, THERMAL, PROX)
    int32_t     sensor_status[8];
    int32_t     sensor_health[8];
    double      lidar_distance;
    double      prox_distance;

    // Aerodynamics & Environment
    double      air_density;
    double      ground_effect_factor;
    int32_t     vrs_active;
    double      vrs_severity;

    // Status Flags & Unified Vehicle Health
    int32_t     armed;
    int32_t     in_ground_contact;
    int32_t     low_voltage_warn;
    int32_t     critical_cutoff;
    int32_t     vehicle_health; // 0=NOMINAL, 1=DEGRADED, 2=WARNING, 3=CRITICAL, 4=FAULT, 5=OFFLINE
    int32_t     vehicle_state;  // 0=DISARMED, 1=ARMED, 2=TAKEOFF, 3=FLIGHT, 4=LANDING, 5=LANDED, 6=FAULT
} GarudaDroneTelemetryPOD;
#pragma pack(pop)

// ----------------------------------------------------------------------------
// World Lifecycle & Simulation Stepping
// ----------------------------------------------------------------------------
GARUDA_API void*    garuda_world_create(uint64_t seed, double dt);
GARUDA_API void     garuda_world_destroy(void* world_handle);
GARUDA_API void     garuda_world_reset(void* world_handle);
GARUDA_API int32_t  garuda_world_add_drone(void* world_handle, const char* drone_id, double spawn_x, double spawn_y, double spawn_z);
GARUDA_API int32_t  garuda_world_drone_count(void* world_handle);
GARUDA_API void     garuda_world_step(void* world_handle);
GARUDA_API void     garuda_world_step_n(void* world_handle, uint32_t step_count);

// ----------------------------------------------------------------------------
// Drone Actuation & Control Setpoints
// ----------------------------------------------------------------------------
GARUDA_API int32_t  garuda_drone_arm(void* world_handle, const char* drone_id);
GARUDA_API int32_t  garuda_drone_disarm(void* world_handle, const char* drone_id);
GARUDA_API int32_t  garuda_drone_set_attitude(void* world_handle, const char* drone_id, double roll_rad, double pitch_rad, double yaw_rate_rads, double throttle);
GARUDA_API int32_t  garuda_drone_set_motors(void* world_handle, const char* drone_id, const double* throttles, int32_t count);

// ----------------------------------------------------------------------------
// Modular Payload & Inspection Camera Commands
// ----------------------------------------------------------------------------
GARUDA_API int32_t  garuda_drone_attach_payload(void* world_handle, const char* drone_id, int32_t payload_type);
GARUDA_API int32_t  garuda_drone_detach_payload(void* world_handle, const char* drone_id);
GARUDA_API int32_t  garuda_drone_set_gimbal(void* world_handle, const char* drone_id, double pitch_deg, double yaw_deg, double zoom);

// ----------------------------------------------------------------------------
// Sensor Suite & Subsystem Commands
// ----------------------------------------------------------------------------
GARUDA_API int32_t  garuda_drone_set_sensor_enabled(void* world_handle, const char* drone_id, int32_t sensor_idx, int32_t enabled);
GARUDA_API int32_t  garuda_drone_set_sensor_status(void* world_handle, const char* drone_id, int32_t sensor_idx, int32_t status);
GARUDA_API int32_t  garuda_drone_get_sensor_status(void* world_handle, const char* drone_id, int32_t sensor_idx);
GARUDA_API int32_t  garuda_drone_get_vehicle_health(void* world_handle, const char* drone_id);

// ----------------------------------------------------------------------------
// Failure Injection & Developer Tools
// ----------------------------------------------------------------------------
GARUDA_API int32_t  garuda_drone_inject_motor_failure(void* world_handle, const char* drone_id, int32_t motor_index, int32_t failure_type, double degradation);
GARUDA_API int32_t  garuda_drone_inject_sensor_failure(void* world_handle, const char* drone_id, int32_t sensor_mode);
GARUDA_API int32_t  garuda_drone_inject_battery_degradation(void* world_handle, const char* drone_id, double capacity_factor, double resistance_mult);
GARUDA_API int32_t  garuda_drone_reset_failures(void* world_handle, const char* drone_id);

// ----------------------------------------------------------------------------
// Telemetry & State Queries
// ----------------------------------------------------------------------------
GARUDA_API int32_t  garuda_world_get_telemetry(void* world_handle, const char* drone_id, GarudaDroneTelemetryPOD* out_pod);
GARUDA_API uint64_t garuda_world_get_state_hash(void* world_handle);
GARUDA_API uint64_t garuda_world_get_tick(void* world_handle);
GARUDA_API double   garuda_world_get_time(void* world_handle);

#ifdef __cplusplus
}
#endif
