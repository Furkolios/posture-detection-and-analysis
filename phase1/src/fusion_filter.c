#include <math.h>
#include "fusion_filter.h"

/* Complementary filter weight. Higher alpha = more trust in gyro
 * integration (good for short timescales), lower = more trust in
 * accelerometer (good against drift). 0.98 is the textbook starting
 * point for a 100 Hz loop. */
#define FUSION_ALPHA           0.98f

/* Conversion factor between radians and degrees. */
#define RAD_TO_DEG             (180.0f / 3.14159265358979323846f)

/* --- Module state -------------------------------------------------------- *
 * Single static angle accumulator. Module-private; nothing else may
 * touch it. */
static float s_angle_deg = 0.0f;

/* Compute pitch angle (in degrees) from an accelerometer vector.
 * For a sensor whose Z axis points up at rest, pitch around the
 * sensor's X axis comes from atan2(-ax, az). */
static float accel_pitch_deg(const ImuRaw *s) {
    return atan2f(-s->ax, s->az) * RAD_TO_DEG;
}

void fusion_filter_reset(void) {
    s_angle_deg = 0.0f;
}

float fusion_filter_update(const ImuRaw *lower,
                           const ImuRaw *upper,
                           float dt_seconds) {
    /* Accelerometer-derived relative pitch: difference of the two
     * sensors' tilt-from-vertical, which is what we ultimately want. */
    float accel_relative = accel_pitch_deg(upper) - accel_pitch_deg(lower);

    /* Gyro-derived increment: integrate the difference in pitch-axis
     * rotation rates. We use gx as the pitch-axis rate (consistent with
     * the dummy generator's axis convention; a real driver may need
     * remapping). */
    float gyro_rate = upper->gx - lower->gx;          /* deg / s */
    float gyro_increment = gyro_rate * dt_seconds;    /* deg */

    /* Standard complementary blend. */
    s_angle_deg = FUSION_ALPHA * (s_angle_deg + gyro_increment)
                + (1.0f - FUSION_ALPHA) * accel_relative;

    return s_angle_deg;
}
