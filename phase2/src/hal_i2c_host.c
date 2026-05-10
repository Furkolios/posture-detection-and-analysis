#include <string.h>
#include <math.h>
#include "hal_i2c.h"
#include "dummy_generator.h"   /* for the DummyScenario enum + getter */

/* Host stub for the I2C bus. Synthesises plausible MPU-6050 register
 * values so imu_source_mpu6050.c can be exercised end-to-end on a
 * laptop without real hardware.
 *
 * For the host firmware build (which links dummy_generator.c), we
 * read the active scenario via dummy_get_target_angle() so the same
 * scenario-cycling logic in main.c keeps working.
 *
 * For test_phase2 (which does NOT link dummy_generator.c — it links
 * imu_source_mpu6050.c instead, and that conflicts with dummy's
 * imu_source impl), we expose a weak fallback below so the link still
 * succeeds and the test can set the simulated upper-IMU pitch
 * directly via hal_i2c_host_set_pitch().
 *
 * Only the ACCEL_XOUT_H register block (0x3B..0x48, 14 bytes) and the
 * WHO_AM_I register (0x75) are faked. */

#define MPU_REG_ACCEL_XOUT_H   0x3B
#define MPU_REG_WHO_AM_I       0x75
#define MPU_WHO_AM_I_VALUE     0x68

#define ACCEL_LSB_PER_G        16384.0f   /* +/- 2g full-scale */
#define GYRO_LSB_PER_DPS         131.0f   /* +/- 250 dps full-scale */
#define G_MS2                      9.81f

/* Scenario knob. By default we read from dummy_generator if present,
 * but the test harness can override via hal_i2c_host_set_pitch(). */
static float s_override_pitch    = 0.0f;
static int   s_override_active   = 0;

void hal_i2c_host_set_pitch(float pitch_deg);
void hal_i2c_host_set_pitch(float pitch_deg) {
    s_override_pitch  = pitch_deg;
    s_override_active = 1;
}

/* Weak default for dummy_get_target_angle — used only if the dummy
 * generator's strong definition isn't linked in (e.g. Phase 2 tests).
 * The weak attribute means the linker prefers the strong symbol when
 * present.  */
__attribute__((weak)) float dummy_get_target_angle(void) {
    return 0.0f;
}

static float current_upper_pitch_deg(void) {
    if (s_override_active) {
        return s_override_pitch;
    }
    return dummy_get_target_angle();
}

static void synth_accel_gyro(float pitch_deg, float roll_deg,
                             int16_t *ax, int16_t *ay, int16_t *az,
                             int16_t *gx, int16_t *gy, int16_t *gz) {
    float p = pitch_deg * (3.14159265358979323846f / 180.0f);
    float r = roll_deg  * (3.14159265358979323846f / 180.0f);

    float ax_g = -sinf(p);
    float ay_g =  sinf(r) * cosf(p);
    float az_g =  cosf(r) * cosf(p);

    *ax = (int16_t)(ax_g * ACCEL_LSB_PER_G);
    *ay = (int16_t)(ay_g * ACCEL_LSB_PER_G);
    *az = (int16_t)(az_g * ACCEL_LSB_PER_G);
    *gx = 0; *gy = 0; *gz = 0;
}

static void pack_be(int16_t v, uint8_t *hi, uint8_t *lo) {
    uint16_t u = (uint16_t)v;
    *hi = (uint8_t)((u >> 8) & 0xFFu);
    *lo = (uint8_t)( u       & 0xFFu);
}

void hal_i2c_init(void) {
    s_override_active = 0;
    s_override_pitch  = 0.0f;
}

bool hal_i2c_read_bytes(uint8_t slave_addr_7bit,
                        uint8_t reg,
                        uint8_t *buf,
                        size_t n) {
    if (buf == NULL || n == 0u) {
        return false;
    }

    if (reg == MPU_REG_WHO_AM_I && n == 1u) {
        buf[0] = MPU_WHO_AM_I_VALUE;
        return true;
    }

    if (reg == MPU_REG_ACCEL_XOUT_H && n == 14u) {
        float pitch, roll = 0.0f;
        if (slave_addr_7bit == 0x68) {
            pitch = 0.0f;                       /* lower IMU level */
        } else {
            pitch = current_upper_pitch_deg();  /* upper IMU tilted */
        }

        int16_t ax, ay, az, gx, gy, gz;
        synth_accel_gyro(pitch, roll, &ax, &ay, &az, &gx, &gy, &gz);

        pack_be(ax,    &buf[0],  &buf[1]);
        pack_be(ay,    &buf[2],  &buf[3]);
        pack_be(az,    &buf[4],  &buf[5]);
        pack_be(0,     &buf[6],  &buf[7]);   /* temperature */
        pack_be(gx,    &buf[8],  &buf[9]);
        pack_be(gy,    &buf[10], &buf[11]);
        pack_be(gz,    &buf[12], &buf[13]);
        return true;
    }

    /* Any other register: zero-fill, succeed. */
    memset(buf, 0, n);
    return true;
}

bool hal_i2c_write_bytes(uint8_t slave_addr_7bit,
                         uint8_t reg,
                         const uint8_t *buf,
                         size_t n) {
    (void)slave_addr_7bit;
    (void)reg;
    (void)buf;
    (void)n;
    return true;
}
