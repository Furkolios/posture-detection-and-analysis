#ifndef IMU_SOURCE_H
#define IMU_SOURCE_H

#include <stdbool.h>
#include "posture_types.h"

/* Abstract IMU source. Phase 1 is implemented by dummy_generator.c.
 * Phase 2 will be implemented by an mpu6050 driver behind the same
 * interface so main.c does not need to change. */

/* Initialise the underlying IMU source (RNG seed for dummy, I2C/wakeup
 * for the real driver). Must be called once at startup. */
void imu_source_init(void);

/* Read one paired sample from both sensors.
 *   out_lower : sample from the IMU mounted on the hip (reference)
 *   out_upper : sample from the IMU mounted on the upper back
 * Returns true on success, false on read error. */
bool imu_source_read(ImuRaw *out_lower, ImuRaw *out_upper);

#endif /* IMU_SOURCE_H */
