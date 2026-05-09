#ifndef FUSION_FILTER_H
#define FUSION_FILTER_H

#include "posture_types.h"

/* Reset the filter's internal angle state to zero. Call once before
 * the first update, and any time you re-zero (e.g. after calibration). */
void fusion_filter_reset(void);

/* Run one step of the complementary filter on a paired IMU sample and
 * return the relative pitch angle (upper minus lower) in degrees.
 *   lower      : sample from the hip-mounted IMU
 *   upper      : sample from the upper-back-mounted IMU
 *   dt_seconds : elapsed time since the previous call, in seconds
 * Returns the fused relative pitch angle in degrees. */
float fusion_filter_update(const ImuRaw *lower,
                           const ImuRaw *upper,
                           float dt_seconds);

#endif /* FUSION_FILTER_H */
