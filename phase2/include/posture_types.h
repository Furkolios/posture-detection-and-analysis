#ifndef POSTURE_TYPES_H
#define POSTURE_TYPES_H

#include <stdint.h>

/* Classification result of the posture pipeline.
 *
 * Values 0..2 are FROZEN from Phase 1. Values 3..5 were added in
 * Phase 2 and must keep their numeric assignments so Phase 1 telemetry
 * decoders still understand at least the first three states. */
typedef enum {
    POSTURE_GOOD          = 0,
    POSTURE_MILD_SLOUCH   = 1,
    POSTURE_FULL_SLOUCH   = 2,
    POSTURE_LEAN_FORWARD  = 3,
    POSTURE_LEAN_BACK     = 4,
    POSTURE_LATERAL_TILT  = 5
} PostureState;

#define POSTURE_STATE_COUNT 6

/* Raw IMU sample. Accel in m/s^2, gyro in degrees/second.
 * Matches what the MPU-6050 driver produces. */
typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} ImuRaw;

/* Orientation quaternion (w, x, y, z), unit norm by convention. */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} ImuQuaternion;

/* --- Telemetry V1 (Phase 1) --- FROZEN ---------------------------------- *
 * Wire format, 7 bytes total:
 *   0 : 1 : sync byte (0xA5)
 *   1 : 4 : angle in degrees, IEEE-754 float, little-endian
 *   5 : 1 : posture state (PostureState cast to uint8)
 *   6 : 1 : XOR checksum of bytes [0..5]
 *
 * Do NOT edit. Phase 2 emits V2 by default; V1 stays available. */
typedef struct __attribute__((packed)) {
    uint8_t sync;
    float   angle_deg;
    uint8_t state;
    uint8_t checksum;
} TelemetryPacket;

#define TELEMETRY_PACKET_SIZE  7

/* --- Telemetry V2 (Phase 2) --------------------------------------------- *
 * Wire format, 11 bytes total. Same sync/state/checksum mechanics as V1,
 * payload grown to carry pitch and roll independently:
 *    0 : 1 : sync byte (0xA5)
 *    1 : 4 : pitch in degrees, IEEE-754 float, little-endian
 *    5 : 4 : roll  in degrees, IEEE-754 float, little-endian
 *    9 : 1 : posture state (PostureState cast to uint8)
 *   10 : 1 : XOR checksum of bytes [0..9] */
typedef struct __attribute__((packed)) {
    uint8_t sync;
    float   pitch_deg;
    float   roll_deg;
    uint8_t state;
    uint8_t checksum;
} TelemetryPacketV2;

#define TELEMETRY_PACKET_V2_SIZE  11

#endif /* POSTURE_TYPES_H */
