#ifndef FUSION_FILTER_H
#define FUSION_FILTER_H

#include "posture_types.h"

/* Reset both internal orientation quaternions to identity {1,0,0,0}.
 * Call once before the first update, and any time you re-zero. */
void fusion_filter_reset(void);

/* Run one Madgwick AHRS step on each IMU and return the relative
 * pitch angle (upper relative to lower) in degrees.
 *   lower      : sample from the hip-mounted IMU
 *   upper      : sample from the upper-back-mounted IMU
 *   dt_seconds : elapsed time since the previous call, in seconds */
float fusion_filter_update(const ImuRaw *lower,
                           const ImuRaw *upper,
                           float dt_seconds);

/* Retrieve the most recently computed relative pitch and roll angles
 * (in degrees). Both pointers must be non-NULL. Values reflect the
 * state after the last call to fusion_filter_update(); calling this
 * before any update yields zeros. */
void fusion_filter_get_angles(float *out_pitch_deg, float *out_roll_deg);

#endif /* FUSION_FILTER_H */
