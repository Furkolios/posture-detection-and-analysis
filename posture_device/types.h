/*
 * Shared types used across all sketch tabs. Lives in a header (rather than
 * in posture_device.ino directly) so that Energia's auto-generated function
 * prototypes — which get injected at the top of the main .ino before any
 * user-level typedef can run — already have these types in scope.
 */

#ifndef POSTURE_TYPES_H
#define POSTURE_TYPES_H

#include <stdint.h>

typedef enum {
    POSTURE_GOOD         = 0,
    POSTURE_MILD_SLOUCH  = 1,
    POSTURE_FULL_SLOUCH  = 2,
    POSTURE_LEAN_FORWARD = 3,
    POSTURE_LEAN_BACK    = 4,
    POSTURE_LATERAL_TILT = 5
} PostureState;

typedef struct {
    float ax, ay, az;    // m/s^2
    float gx, gy, gz;    // rad/s
} IMUSample;

typedef struct {
    float q0, q1, q2, q3;
} MadgwickState;

#endif /* POSTURE_TYPES_H */
