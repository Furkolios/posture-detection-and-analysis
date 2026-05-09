#ifndef POSTURE_TYPES_H
#define POSTURE_TYPES_H

#include <stdint.h>

/* Classification result of the posture pipeline. */
typedef enum {
    POSTURE_GOOD         = 0,
    POSTURE_MILD_SLOUCH  = 1,
    POSTURE_FULL_SLOUCH  = 2
} PostureState;

#define POSTURE_STATE_COUNT 3

/* Raw IMU sample. Accel in m/s^2, gyro in degrees/second.
 * Matches what an MPU-6050 driver will produce in Phase 2. */
typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} ImuRaw;

/* Orientation quaternion (w, x, y, z). Used by Phase 2 Madgwick. */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} ImuQuaternion;

/* Wire format for telemetry. Packed little-endian, 7 bytes total.
 * Layout (offset : size : meaning):
 *   0 : 1 : sync byte (0xA5)
 *   1 : 4 : angle in degrees, IEEE-754 float
 *   5 : 1 : posture state (PostureState cast to uint8)
 *   6 : 1 : XOR checksum of bytes [0..5]
 *
 * This format is LOCKED for all phases. Do not edit without coordinating
 * with the BLE bridge code and the host-side visualizer. */
#if defined(_MSC_VER)
#  pragma pack(push, 1)
#endif
typedef struct __attribute__((packed)) {
    uint8_t sync;
    float   angle_deg;
    uint8_t state;
    uint8_t checksum;
} TelemetryPacket;
#if defined(_MSC_VER)
#  pragma pack(pop)
#endif

#define TELEMETRY_PACKET_SIZE 7

#endif /* POSTURE_TYPES_H */
