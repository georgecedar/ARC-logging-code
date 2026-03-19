#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>

struct __attribute__((packed)) DataPoint {
    uint32_t timestamp_ms;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    uint8_t d5;
};

#define CSV_HEADER "timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,quat_w,quat_x,quat_y,quat_z,pressure_pa,temperature_c,altitude_m,d5"

enum FlightState {
    STATE_BOOT,
    STATE_IDLE,
    STATE_ARMED,
    STATE_FLIGHT,
    STATE_LANDED,
    STATE_DOWNLOAD
};

inline const char* stateName(FlightState s) {
    switch (s) {
        case STATE_BOOT:     return "BOOT";
        case STATE_IDLE:     return "IDLE";
        case STATE_ARMED:    return "ARMED";
        case STATE_FLIGHT:   return "FLIGHT";
        case STATE_LANDED:   return "LANDED";
        case STATE_DOWNLOAD: return "DOWNLOAD";
        default:             return "UNKNOWN";
    }
}

#endif
