#pragma once
#include <stdint.h>
#include <stddef.h>

#pragma pack(push, 1)

struct DroneCommand {
    uint8_t  sync_byte;      // 0xA5
    int16_t  throttle;       // 1000 - 2000 us
    int16_t  roll_target;    // -30 to +30 deg
    int16_t  pitch_target;   // -30 to +30 deg
    int16_t  yaw_rate;       // -150 to +150 deg/s
    uint8_t  armed;          // 0 = Disarmed, 1 = Armed
    uint8_t  emergency_kill; // 1 = Immediate Cutoff
    uint8_t  checksum;
};

struct DroneTelemetry {
    uint8_t  sync_byte;      // 0x5A
    int16_t  current_roll;   // deg * 10
    int16_t  current_pitch;  // deg * 10
    uint16_t battery_mv;     // mV
    uint16_t current_ma;     // mA
    uint16_t front_dist_mm;  // ToF front
    uint16_t down_dist_mm;   // ToF down
    uint8_t  status_flags;
    uint8_t  checksum;
};

#pragma pack(pop)

inline uint8_t calculate_checksum(const uint8_t* data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) {
        c ^= data[i];
    }
    return c;
}